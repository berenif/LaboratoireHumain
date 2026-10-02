//! Contact-gated get-up motor intent. Physics bodies are read-only here.
//! Initialization fixtures are constructed before the first integration.
use crate::{Profile, Simulation, measurement, rotation, vector};
use lh_contracts::Motion;
use lh_model::{Model, Pose as BodyPose, Segment};
use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};

pub const FIXTURES: &str = include_str!("../data/recovery-v1.json");
const RATE: f32 = 1.0;
const READY_S: f32 = 0.20;
const STABLE_S: f32 = 1.0;

fn blend_targets(current: &mut [[f32; 3]], desired: &[[f32; 3]], maximum_delta: f32) {
    let maximum = current
        .iter()
        .zip(desired)
        .flat_map(|(a, b)| a.iter().zip(b).map(|(a, b)| (a - b).abs()))
        .fold(0.0_f32, f32::max);
    let blend = if maximum > maximum_delta {
        maximum_delta / maximum
    } else {
        1.0
    };
    for (a, b) in current.iter_mut().zip(desired) {
        for k in 0..3 {
            a[k] += (b[k] - a[k]) * blend;
        }
    }
}

#[derive(Deserialize)]
struct FixtureSet {
    cases: Vec<Fixture>,
}
#[derive(Deserialize)]
struct Fixture {
    id: String,
    heading: f32,
    poses: Vec<BodyPose>,
}

pub(crate) fn validate_seed(model: &Model, poses: &[BodyPose]) -> Result<(), String> {
    if poses.len() != model.segments.len() {
        return Err("Recovery seed topology".into());
    }
    for (i, (segment, pose)) in model.segments.iter().zip(poses).enumerate() {
        let q = rotation(pose.rotation);
        if pose.id != segment.id
            || !pose.position.finite()
            || !pose.linear_velocity.finite()
            || !pose.angular_velocity.finite()
            || !q.is_finite()
            || (q.length_squared() - 1.0).abs() > 1e-4
            || vector(pose.linear_velocity).length_squared() != 0.0
            || vector(pose.angular_velocity).length_squared() != 0.0
        {
            return Err(format!("Invalid recovery seed {}", segment.id));
        }
        if let Some(joint) = &segment.joint_profile {
            let parent = model
                .segments
                .iter()
                .position(|s| Some(&s.id) == segment.parent.as_ref())
                .ok_or("Missing seed parent")?;
            if parent >= i {
                return Err("Recovery seed order".into());
            }
            let a = vector(poses[parent].position)
                + rotation(poses[parent].rotation) * vector(joint.parent_frame.anchor);
            let b = vector(pose.position) + q * vector(joint.child_frame.anchor);
            if a.distance(b) > 0.0001 {
                return Err(format!("Disconnected recovery seed {}", segment.id));
            }
        }
    }
    Ok(())
}

fn index(s: &Simulation, id: &str) -> usize {
    s.model
        .segments
        .iter()
        .position(|part| part.id == id)
        .expect("canonical recovery segment")
}
fn body<'a>(s: &'a Simulation, id: &str) -> &'a RigidBody {
    &s.world.bodies[s.bodies[index(s, id)]]
}
fn up(s: &Simulation, id: &str) -> f32 {
    (*body(s, id).rotation() * Vector::Y).y
}
fn load(s: &Simulation, id: &str) -> f32 {
    let i = index(s, id);
    s.contacts
        .iter()
        .filter(|c| c.segment as usize == i && c.persistence_s + 1e-6 >= 0.05)
        .map(|c| c.normal_load_n)
        .sum()
}
fn foot(s: &Simulation, side: &str) -> bool {
    load(s, &format!("{side}Foot")) + load(s, &format!("{side}Forefoot")) >= 3.0
}
pub(crate) fn has_persistent_support(s: &Simulation) -> bool {
    s.contacts
        .iter()
        .any(|c| c.normal_load_n >= 3.0 && c.persistence_s + 1e-6 >= 0.05)
}

pub(crate) fn standing_reference(s: &Simulation) -> Vector {
    let feet = ["leftFoot", "rightFoot"].map(|id| body(s, id).translation());
    let initial_feet =
        ["leftFoot", "rightFoot"].map(|id| vector(s.model.segments[index(s, id)].initial.position));
    let initial_com = s.model.segments.iter().fold(Vector::ZERO, |sum, part| {
        sum + (vector(part.initial.position)
            + rotation(part.initial.rotation) * vector(part.golden_mass.center_of_mass))
            * part.mass_kg
    }) / s.total_mass();
    (feet[0] + feet[1]) * 0.5
        + s.heading * (initial_com - (initial_feet[0] + initial_feet[1]) * 0.5)
}

#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum Phase {
    Roll,
    Brace,
    Kneel,
    PlantLead,
    PlantTrailing,
    Stand,
}
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Status {
    pub phase: Phase,
    pub phase_time_s: f32,
    pub qualified_time_s: f32,
    pub stable_time_s: f32,
    pub loaded_feet: [bool; 2],
}
pub(crate) struct Controller {
    pub status: Status,
    targets: Vec<[f32; 3]>,
    hands: [Vector; 2],
    lead: usize,
    stand_feet: Option<[Vector; 2]>,
    stand_root: Option<Pose>,
}
impl Controller {
    pub fn new(s: &Simulation) -> Result<Self, String> {
        if s.model.segments.len() != 25 {
            return Err("Recovery requires the complete anatomy".into());
        }
        let feet = [foot(s, "left"), foot(s, "right")];
        let height = body(s, "pelvis").translation().y - s.support_height();
        let phase = if feet.iter().all(|x| *x) && up(s, "torso") > 0.65 && height > 0.5 {
            Phase::Stand
        } else if up(s, "torso") > 0.65 && height > 0.30 {
            Phase::PlantLead
        } else if (*body(s, "torso").rotation() * Vector::Z).y < -0.45 {
            Phase::Brace
        } else {
            Phase::Roll
        };
        let mut targets = s.rest_targets.clone();
        for (i, part) in s.model.segments.iter().enumerate() {
            if let Some(p) = &part.joint_profile {
                let parent = s
                    .world
                    .impulse_joints
                    .get(s.joints[i].ok_or("Lost recovery joint")?)
                    .ok_or("Lost recovery joint")?
                    .body1();
                targets[i] = measurement::coordinates(
                    *s.world.bodies[parent].rotation(),
                    *s.world.bodies[s.bodies[i]].rotation(),
                    p,
                );
            }
        }
        let hands = ["left", "right"].map(|side| {
            let shoulder = body(s, &format!("{side}UpperArm")).translation();
            let sign = if side == "left" { -1.0 } else { 1.0 };
            let p = shoulder + s.heading * Vector::new(sign * 0.12, 0.0, 0.18);
            Vector::new(
                p.x,
                s.surface_height(p.x, p.z, shoulder.y + 0.1) + 0.035,
                p.z,
            )
        });
        Ok(Self {
            status: Status {
                phase,
                phase_time_s: 0.0,
                qualified_time_s: 0.0,
                stable_time_s: 0.0,
                loaded_feet: feet,
            },
            targets,
            hands,
            lead: usize::from(feet[1] && !feet[0]),
            stand_feet: None,
            stand_root: None,
        })
    }
    pub fn standing(&self) -> bool {
        self.status.phase == Phase::Stand
    }
    pub fn root_torque(&self, s: &Simulation, part: &Segment) -> Vector {
        if !self.standing() || part.role != "thigh" {
            return Vector::ZERO;
        }
        let side = usize::from(part.id.starts_with("right"));
        let loaded = [foot(s, "left"), foot(s, "right")];
        if !loaded[side] {
            return Vector::ZERO;
        }
        let pelvis = body(s, "pelvis");
        let mut orientation = s.heading.inverse() * *pelvis.rotation();
        if orientation.w < 0.0 {
            orientation = -orientation;
        }
        let error = orientation.to_scaled_axis();
        let omega = s.heading.inverse() * pelvis.angvel();
        // Equal and opposite hip actuation restores the measured root. Only
        // loaded legs participate; the existing native motors enforce caps.
        s.heading * (error * 1000.0 + omega * 140.0)
            / loaded.into_iter().filter(|loaded| *loaded).count() as f32
    }
    pub fn weight(&self, part: &Segment, coordinate: char) -> f32 {
        if self.standing() {
            return match part.role.as_str() {
                "thigh" => {
                    if coordinate == 'x' {
                        1.0
                    } else {
                        0.22
                    }
                }
                "shin" => 0.60,
                "ankle" | "hindfoot" | "forefoot" => 1.0,
                "lumbar" | "ribcage" => {
                    if coordinate == 'x' {
                        0.30
                    } else {
                        0.12
                    }
                }
                "upper-arm" | "forearm" | "shoulder-girdle" | "hand" | "forearm-twist" => 0.06,
                _ => 0.05,
            };
        }
        match part.role.as_str() {
            "thigh" => {
                if coordinate == 'x' {
                    1.0
                } else {
                    0.18
                }
            }
            "shin" => 1.0,
            "ankle" => 0.65,
            "hindfoot" => 0.35,
            "forefoot" => 0.15,
            "lumbar" | "ribcage" => {
                if coordinate == 'x' {
                    0.7
                } else {
                    0.25
                }
            }
            "upper-arm" | "forearm" | "shoulder-girdle" | "hand" | "forearm-twist" => {
                if self.standing() {
                    0.08
                } else if coordinate == 'x' {
                    0.8
                } else {
                    0.3
                }
            }
            _ => 0.05,
        }
    }
    fn enter(&mut self, phase: Phase) {
        self.status.phase = phase;
        self.status.phase_time_s = 0.0;
        self.status.qualified_time_s = 0.0;
    }
    pub fn update(&mut self, s: &Simulation) -> Result<Vec<[f32; 3]>, String> {
        let dt = s.profile.dt_s;
        self.status.phase_time_s += dt;
        let feet = [foot(s, "left"), foot(s, "right")];
        self.status.loaded_feet = feet;
        let shins = [load(s, "leftShin") >= 3.0, load(s, "rightShin") >= 3.0];
        let height = body(s, "pelvis").translation().y - s.support_height();
        let torso_up = up(s, "torso");
        let qualified = match self.status.phase {
            Phase::Roll => {
                (*body(s, "torso").rotation() * Vector::Z).y < -0.6 && has_persistent_support(s)
            }
            Phase::Brace => shins.iter().all(|x| *x) && height > 0.28 && torso_up > 0.40,
            Phase::Kneel => shins.iter().all(|x| *x) && height > 0.38 && torso_up > 0.75,
            Phase::PlantLead => feet[self.lead],
            Phase::PlantTrailing => feet.iter().all(|x| *x) && torso_up > 0.65,
            Phase::Stand => false,
        };
        self.status.qualified_time_s = if qualified {
            self.status.qualified_time_s + dt
        } else {
            0.0
        };
        if self.status.qualified_time_s + 1e-6 >= READY_S {
            let next = match self.status.phase {
                Phase::Roll => Phase::Brace,
                Phase::Brace => Phase::Kneel,
                Phase::Kneel => Phase::PlantLead,
                Phase::PlantLead => Phase::PlantTrailing,
                _ => Phase::Stand,
            };
            self.enter(next);
        }
        let mut desired = s.rest_targets.clone();
        if self.standing() {
            let planted = *self.stand_feet.get_or_insert_with(|| {
                ["leftFoot", "rightFoot"].map(|id| {
                    let p = body(s, id).translation();
                    Vector::new(p.x, s.surface_height(p.x, p.z, p.y + 0.1) + 0.045, p.z)
                })
            });
            let canonical_midpoint =
                (vector(s.model.segments[index(s, "leftFoot")].initial.position)
                    + vector(s.model.segments[index(s, "rightFoot")].initial.position))
                    * 0.5;
            let mut goal = (planted[0] + planted[1]) * 0.5
                + s.heading * (vector(s.model.segments[0].initial.position) - canonical_midpoint);
            let measured = body(s, "pelvis").translation();
            let mut center_error = goal - measured;
            center_error.y = 0.0;
            let root = self.stand_root.get_or_insert_with(|| {
                Pose::from_parts(
                    Vector::new(
                        measured.x,
                        measured.y.max(s.support_height() + 0.40),
                        measured.z,
                    ),
                    s.heading,
                )
            });
            // Keep knee reserve until measured mass has moved over the newly
            // loaded footprint. The virtual root belongs only to leg IK.
            if center_error.length() > 0.06 || torso_up < 0.80 {
                goal.y = root.translation.y;
            }
            let mut travel = goal - root.translation;
            travel.y = 0.0;
            root.translation += travel.clamp_length_max(0.25 * dt);
            root.translation.y += (goal.y - root.translation.y).clamp(-0.15 * dt, 0.15 * dt);
            root.rotation = *body(s, "pelvis").rotation();
            let local_root = s.heading.inverse() * root.rotation;
            let pitch = 2.0 * local_root.x.atan2(local_root.w);
            for (side, id) in ["leftFoot", "rightFoot"].into_iter().enumerate() {
                let mut leg =
                    crate::stepping::leg_targets_with_root(s, index(s, id), planted[side], *root)?;
                // The ankle has less dorsiflexion than a deep crouch may need.
                // Preserve the world sole angle by extending the knee instead
                // of silently pitching the foot after an independent clamp.
                leg[1].1[0] = leg[0].1[0] + leg[2].1[0] - pitch;
                for (i, angles) in leg {
                    desired[i] = angles;
                }
            }
        }
        for (i, part) in s.model.segments.iter().enumerate() {
            let side = usize::from(part.id.starts_with("right"));
            let lead = side == self.lead;
            let x = match (self.status.phase, part.role.as_str()) {
                (Phase::Roll, "thigh") => Some(1.2),
                (Phase::Roll, "shin") => Some(2.1),
                (Phase::Roll, "ankle") => Some(0.2),
                (Phase::Brace, "thigh") => Some(1.45),
                (Phase::Brace, "shin") => Some(2.1),
                (Phase::Brace, "ankle") => Some(-0.55),
                (Phase::Kneel, "thigh") => Some(0.08),
                (Phase::Kneel, "shin") => Some(1.85),
                (Phase::Kneel, "ankle") => Some(-0.65),
                (Phase::PlantLead | Phase::PlantTrailing, "thigh") => {
                    Some(if lead || self.status.phase == Phase::PlantTrailing {
                        1.55
                    } else {
                        0.08
                    })
                }
                (Phase::PlantLead | Phase::PlantTrailing, "shin") => {
                    Some(if lead || self.status.phase == Phase::PlantTrailing {
                        1.65
                    } else {
                        1.85
                    })
                }
                (Phase::PlantLead | Phase::PlantTrailing, "ankle") => {
                    Some(if lead || self.status.phase == Phase::PlantTrailing {
                        0.10
                    } else {
                        -0.65
                    })
                }
                _ => None,
            };
            if let Some(x) = x {
                desired[i] = [x, 0.0, 0.0];
            }
            if self.status.phase == Phase::Roll
                && matches!(part.role.as_str(), "lumbar" | "ribcage")
            {
                desired[i] = [0.3, 0.0, if self.lead == 0 { 0.25 } else { -0.25 }];
            }
        }
        if !self.standing() {
            for (side, id) in ["leftHand", "rightHand"].into_iter().enumerate() {
                for (i, angles) in
                    crate::reaching::arm_targets(s, index(s, id), Vector::ZERO, self.hands[side])?
                {
                    desired[i] = angles;
                }
            }
        }
        for (i, part) in s.model.segments.iter().enumerate() {
            if let Some(p) = &part.joint_profile {
                for (k, c) in ['x', 'y', 'z'].into_iter().enumerate() {
                    if let Some(axis) = p.axes.iter().find(|a| a.coordinate == c) {
                        desired[i][k] = desired[i][k].clamp(axis.min_radians, axis.max_radians);
                        self.targets[i][k] =
                            self.targets[i][k].clamp(axis.min_radians, axis.max_radians);
                    } else {
                        desired[i][k] = 0.0;
                        self.targets[i][k] = 0.0;
                    }
                }
            }
        }
        // One common progress parameter preserves the coordinated hip/knee/
        // ankle path. Independent rate clamps pitch the sole during a rise.
        blend_targets(&mut self.targets, &desired, RATE * dt);
        Ok(self.targets.clone())
    }
}

impl Simulation {
    pub fn recovery_status(&self) -> Option<&Status> {
        self.recovery.as_ref().map(|r| &r.status)
    }
    pub(crate) fn observe_recovery(&mut self) {
        if self.recovery.as_ref().is_some_and(|r| r.standing())
            && (body(self, "pelvis").translation().y - self.support_height() < 0.56
                || up(self, "torso") < 1.25_f32.cos())
        {
            // A failed rise hands back to the existing passive fall observer.
            // The next attempt must earn settled, persistent support again.
            self.motion = Motion::Falling;
            self.recovery = None;
            self.motion_observation = crate::motion::Observation::default();
            return;
        }
        let stable = measurement::standing_support(self, 0).is_ok()
            && body(self, "pelvis").translation().y - self.support_height() > 0.93
            && ["left", "right"].iter().all(|side| foot(self, side))
            && self.bodies.iter().all(|h| {
                self.world.bodies[*h].linvel().length() <= 0.22
                    && self.world.bodies[*h].angvel().length() <= 0.65
            });
        if let Some(controller) = self.recovery.as_mut() {
            controller.status.stable_time_s = if stable {
                controller.status.stable_time_s + self.profile.dt_s
            } else {
                0.0
            };
            if controller.status.stable_time_s + 1e-6 >= STABLE_S {
                self.motion = Motion::Upright;
                self.counters.recoveries += 1;
                self.recovery = None;
                self.rest_com = self.center_of_mass();
                self.motion_observation = crate::motion::Observation::default();
                if self.profile.corrective_steps_enabled {
                    self.stepping = crate::stepping::StepController::new(self).ok();
                }
            }
        }
    }
}

pub fn fixture_trials(
    profile: &Profile,
    selected: Option<&str>,
) -> Result<Vec<serde_json::Value>, String> {
    if !profile.recovery_enabled {
        return Err("Recovery diagnostic requires recovery_enabled".into());
    }
    let fixtures: FixtureSet = serde_json::from_str(FIXTURES).map_err(|e| e.to_string())?;
    let mut reports = Vec::new();
    for fixture in fixtures
        .cases
        .into_iter()
        .filter(|f| selected.is_none_or(|id| f.id == id))
    {
        let mut s = Simulation::build(
            fixture.heading,
            profile.clone(),
            true,
            None,
            Some(&fixture.poses),
        )?;
        let initial = s.snapshot();
        let original = s.bodies.clone();
        let mut failure = None;
        let mut trace = Vec::new();
        let mut handoff_substeps = 0_u32;
        for _ in 0..1500 {
            if let Err(e) = s.advance_tick_observed(|state, substep| {
                if state.counters.recoveries > 0 {
                    measurement::standing_support(state, substep)?;
                    let speed = state
                        .bodies
                        .iter()
                        .map(|h| {
                            let b = &state.world.bodies[*h];
                            (b.linvel().length() / 0.22).max(b.angvel().length() / 0.65)
                        })
                        .fold(0.0_f32, f32::max);
                    if state.motion != Motion::Upright || speed > 1.0 {
                        return Err(measurement::Failure::new(
                            state.tick,
                            substep,
                            "recovery-handoff",
                            "standing control lost the recovered pose".into(),
                            speed.max(if state.motion != Motion::Upright {
                                2.0
                            } else {
                                0.0
                            }),
                            1.0,
                        ));
                    }
                    handoff_substeps += 1;
                }
                Ok(())
            }) {
                failure = Some(e);
                break;
            }
            if s.tick % 15 == 0 || handoff_substeps == 1 {
                trace.push(
                    serde_json::json!({"snapshot":s.snapshot(),"recovery":s.recovery_status(), "actuation":diagnostic_actuation(&s)}),
                );
            }
            if handoff_substeps >= 240 {
                break;
            }
        }
        let ownership =
            s.bodies == original && s.bodies.iter().all(|h| s.world.bodies[*h].is_dynamic());
        let passed = failure.is_none()
            && ownership
            && s.counters.recoveries == 1
            && s.motion == Motion::Upright
            && handoff_substeps >= 240;
        reports.push(serde_json::json!({"id":fixture.id,"heading":fixture.heading,"passed":passed,
            "initial":initial,"final":s.snapshot(),"status":s.recovery_status(),"failure":failure,"dynamicOwnership":ownership,
            "handoffStandingSubsteps":handoff_substeps,"trace":trace}));
    }
    if reports.is_empty() {
        return Err("Unknown recovery fixture".into());
    }
    Ok(reports)
}

fn diagnostic_actuation(s: &Simulation) -> Vec<serde_json::Value> {
    s.model.segments.iter().enumerate().filter_map(|(i, part)| {
        let profile = part.joint_profile.as_ref()?;
        let joint = s.world.impulse_joints.get(s.joints[i]?)?;
        let parent = &s.world.bodies[joint.body1()];
        let child = &s.world.bodies[s.bodies[i]];
        let measured = measurement::coordinates(*parent.rotation(), *child.rotation(), profile);
        let motors = profile.axes.iter().map(|axis| {
            let k = match axis.coordinate { 'x' => 0, 'y' => 1, _ => 2 };
            let m = &joint.data.motors[k + 3];
                serde_json::json!({"coordinate":axis.coordinate,"measured":measured[k],"target":m.target_pos,"lastSolverImpulseNms":m.impulse,"ceilingNm":m.max_force})
        }).collect::<Vec<_>>();
        Some(serde_json::json!({"id":part.id,"motors":motors}))
    }).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn recovery_requires_real_support_and_remains_opt_in() {
        assert!(!Profile::default().recovery_enabled);
        let set: FixtureSet = serde_json::from_str(FIXTURES).unwrap();
        let fixture = set
            .cases
            .iter()
            .find(|f| f.id == "landed-crouch-left")
            .unwrap();
        let profile = Profile {
            recovery_enabled: true,
            ..Profile::default()
        };
        let mut s = Simulation::build(0.0, profile, false, None, Some(&fixture.poses)).unwrap();
        let original = s.bodies.clone();
        for _ in 0..15 {
            s.advance_tick().unwrap();
            assert!(s.recovery.is_none());
            assert_eq!(s.counters.recoveries, 0);
            assert_eq!(s.bodies, original);
            assert!(s.bodies.iter().all(|h| s.world.bodies[*h].is_dynamic()));
        }
    }
    #[test]
    fn recovery_caps_and_input_lockout_survive_an_interrupting_impact() {
        let set: FixtureSet = serde_json::from_str(FIXTURES).unwrap();
        let fixture = set
            .cases
            .iter()
            .find(|f| f.id == "landed-crouch-left")
            .unwrap();
        let profile = Profile {
            recovery_enabled: true,
            ..Profile::default()
        };
        let mut s = Simulation::build(0.0, profile, true, None, Some(&fixture.poses)).unwrap();
        for _ in 0..10 {
            s.advance_tick().unwrap();
        }
        assert_eq!(s.motion, Motion::Recovering);
        let mut ceiling_sum = 0.0;
        for (i, handle) in s.joints.iter().enumerate() {
            if let (Some(handle), Some(p)) = (handle, &s.model.segments[i].joint_profile) {
                let joint = s.world.impulse_joints.get(*handle).unwrap();
                for axis in &p.axes {
                    let motor = joint
                        .data
                        .motor(crate::axis(axis.coordinate).unwrap())
                        .unwrap();
                    assert!(motor.max_force > 0.0 && motor.max_force <= axis.max_motor_torque_nm);
                    assert!(motor.target_vel.is_finite());
                    ceiling_sum += motor.max_force;
                }
            }
        }
        assert!(ceiling_sum <= s.profile.aggregate_motor_cap_nm + 1e-3);
        assert!(
            s.begin_grab(0, 1, lh_model::Vec3::default(), lh_model::Vec3::default())
                .is_err()
        );
        let poses = serde_json::to_value(s.snapshot().segments).unwrap();
        let handles = s.bodies.clone();
        s.observe_impact();
        assert_eq!(s.motion, Motion::Falling);
        assert!(s.recovery.is_none());
        assert!(s.grab.is_none());
        assert_eq!(s.counters.recoveries, 0);
        assert_eq!(s.bodies, handles);
        assert_eq!(serde_json::to_value(s.snapshot().segments).unwrap(), poses);
    }
    #[test]
    fn failed_rise_returns_to_passive_falling_without_changing_a_pose() {
        let set: FixtureSet = serde_json::from_str(FIXTURES).unwrap();
        let fixture = set
            .cases
            .iter()
            .find(|f| f.id == "landed-prone-left")
            .unwrap();
        let mut s =
            Simulation::build(0.0, Profile::default(), true, None, Some(&fixture.poses)).unwrap();
        let mut controller = Controller::new(&s).unwrap();
        controller.status.phase = Phase::Stand;
        s.recovery = Some(controller);
        s.motion = Motion::Recovering;
        let before = serde_json::to_value(s.snapshot().segments).unwrap();
        s.observe_recovery();
        assert_eq!(s.motion, Motion::Falling);
        assert!(s.recovery.is_none());
        assert_eq!(s.counters.recoveries, 0);
        assert_eq!(serde_json::to_value(s.snapshot().segments).unwrap(), before);
    }
    #[test]
    fn bounded_rise_preserves_the_sagittal_sole_orientation_during_interpolation() {
        let mut q = [[0.34, 0.0, 0.0], [0.66, 0.0, 0.0], [0.32, 0.0, 0.0]];
        let target = [[0.167, 0.0, 0.0], [0.342, 0.0, 0.0], [0.175, 0.0, 0.0]];
        for _ in 0..120 {
            let before = q;
            blend_targets(&mut q, &target, 1.0 / 240.0);
            assert!((q[1][0] - q[0][0] - q[2][0]).abs() < 1e-6);
            for (a, b) in q.iter().zip(before) {
                assert!((a[0] - b[0]).abs() <= 1.0 / 240.0 + 1e-6);
            }
        }
        assert_eq!(q, target);
    }
    #[test]
    fn canonical_seed_validation_rejects_reordered_disconnected_and_moving_bodies() {
        let model = Model::canonical().unwrap();
        let mut set: FixtureSet = serde_json::from_str(FIXTURES).unwrap();
        for fixture in &set.cases {
            validate_seed(&model, &fixture.poses).unwrap();
        }
        let poses = &mut set.cases[0].poses;
        poses.swap(0, 1);
        assert!(validate_seed(&model, poses).is_err());
        poses.swap(0, 1);
        poses[0].position.x += 0.1;
        assert!(validate_seed(&model, poses).is_err());
        poses[0].position.x -= 0.1;
        poses[0].linear_velocity.x = 1.0;
        assert!(validate_seed(&model, poses).is_err());
    }
    #[test]
    fn recovery_planning_preserves_every_dynamic_pose_and_caps_coordinates() {
        let fixtures: FixtureSet = serde_json::from_str(FIXTURES).unwrap();
        let fixture = fixtures
            .cases
            .iter()
            .find(|f| f.id == "landed-crouch-left")
            .unwrap();
        let s =
            Simulation::build(0.0, Profile::default(), true, None, Some(&fixture.poses)).unwrap();
        let before = serde_json::to_value(s.snapshot()).unwrap();
        let mut controller = Controller::new(&s).unwrap();
        let targets = controller.update(&s).unwrap();
        assert_eq!(before, serde_json::to_value(s.snapshot()).unwrap());
        for (i, p) in s
            .model
            .segments
            .iter()
            .enumerate()
            .filter_map(|(i, s)| s.joint_profile.as_ref().map(|p| (i, p)))
        {
            for axis in &p.axes {
                let k = match axis.coordinate {
                    'x' => 0,
                    'y' => 1,
                    _ => 2,
                };
                assert!((axis.min_radians..=axis.max_radians).contains(&targets[i][k]));
            }
        }
    }
}
