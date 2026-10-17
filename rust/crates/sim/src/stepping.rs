//! Contact-gated corrective steps request anatomical motors only.
use crate::{Simulation, measurement, vector};
use rapier3d::prelude::*;
use serde::Serialize;

#[derive(Clone, Default, Serialize)]
pub(crate) struct Status {
    pub phase: &'static str,
    pub side: Option<usize>,
    pub elapsed_s: f32,
    pub loads_n: [f32; 2],
    pub sole_clearance_m: [f32; 2],
    pub retained_capture_inside: bool,
    pub capture: [f32; 2],
    pub reference: [f32; 2],
    pub ready_s: f32,
    pub airborne_s: f32,
    pub verified_liftoff: bool,
    pub touchdown_s: f32,
    pub aborted: u32,
}

#[derive(Clone)]
struct Swing {
    side: usize,
    start: Vector,
    target: Vector,
    elapsed: f32,
    ready_s: f32,
    swing_s: Option<f32>,
    airborne_s: f32,
    verified_liftoff: bool,
    landed_s: f32,
    demand: Vector,
    transfer_bias: Vector,
    qualified_reference: Option<Vector>,
    redirected: bool,
    flat_started: bool,
    unload_s: f32,
    following: bool,
    follow_start: Pose,
}
#[derive(Clone)]
pub(crate) struct StepController {
    feet: [usize; 2],
    planted: [Vector; 2],
    initial_midpoint: Vector,
    initial_pelvis_height: f32,
    pelvis_com_offset: Vector,
    swing: Option<Swing>,
    next_side: usize,
    following_side: Option<usize>,
    repositioned: bool,
    cooldown_s: f32,
    press_start: Option<(u64, Vector)>,
    last_step_demand: Vector,
    settle_from: Vec<[f32; 3]>,
    settle_s: f32,
    settle_reference: Vector,
    pub status: Status,
}
#[cfg(all(test, feature = "experimental-solver-step-observation"))]
impl StepController {
    /// Exhaustive read-only projection for native preview parity. Public Status
    /// alone omits the stance anchors and timers used after recovery handoff.
    pub(crate) fn parity_memory(&self) -> serde_json::Value {
        let Self {
            feet,
            planted,
            initial_midpoint,
            initial_pelvis_height,
            pelvis_com_offset,
            swing,
            next_side,
            following_side,
            repositioned,
            cooldown_s,
            press_start,
            last_step_demand,
            settle_from,
            settle_s,
            settle_reference,
            status,
        } = self;
        let vector = |v: Vector| v.to_array().map(f32::to_bits);
        let pose = |p: Pose| {
            serde_json::json!({
            "translationBits":vector(p.translation),
            "rotationBits":p.rotation.to_array().map(f32::to_bits)})
        };
        let swing = swing.as_ref().map(|s| {
            let Swing {
                side,
                start,
                target,
                elapsed,
                ready_s,
                swing_s,
                airborne_s,
                verified_liftoff,
                landed_s,
                demand,
                transfer_bias,
                qualified_reference,
                redirected,
                flat_started,
                unload_s,
                following,
                follow_start,
            } = s;
            serde_json::json!({"side":side,"startBits":vector(*start),"targetBits":vector(*target),
                "elapsedBits":elapsed.to_bits(),"readyBits":ready_s.to_bits(),
                "swingBits":swing_s.map(f32::to_bits),"airborneBits":airborne_s.to_bits(),
                "verifiedLiftoff":verified_liftoff,"landedBits":landed_s.to_bits(),
                "demandBits":vector(*demand),"transferBiasBits":vector(*transfer_bias),
                "qualifiedReferenceBits":qualified_reference.map(vector),"redirected":redirected,
                "flatStarted":flat_started,"unloadBits":unload_s.to_bits(),"following":following,
                "followStart":pose(*follow_start)})
        });
        let Status {
            phase,
            side,
            elapsed_s,
            loads_n,
            sole_clearance_m,
            retained_capture_inside,
            capture,
            reference,
            ready_s,
            airborne_s,
            verified_liftoff,
            touchdown_s,
            aborted,
        } = status;
        serde_json::json!({"feet":feet,"plantedBits":planted.map(vector),
            "initialMidpointBits":vector(*initial_midpoint),"initialPelvisHeightBits":initial_pelvis_height.to_bits(),
            "pelvisComOffsetBits":vector(*pelvis_com_offset),"swing":swing,"nextSide":next_side,
            "followingSide":following_side,"repositioned":repositioned,"cooldownBits":cooldown_s.to_bits(),
            "pressStart":press_start.map(|(clock,v)|(clock,vector(v))),"lastStepDemandBits":vector(*last_step_demand),
            "settleFromBits":settle_from.iter().map(|q|q.map(f32::to_bits)).collect::<Vec<_>>(),
            "settleBits":settle_s.to_bits(),"settleReferenceBits":vector(*settle_reference),
            "status":{"phase":phase,"side":side,"elapsedBits":elapsed_s.to_bits(),
                "loadsBits":loads_n.map(f32::to_bits),"soleClearanceBits":sole_clearance_m.map(f32::to_bits),
                "retainedCaptureInside":retained_capture_inside,"captureBits":capture.map(f32::to_bits),
                "referenceBits":reference.map(f32::to_bits),"readyBits":ready_s.to_bits(),
                "airborneBits":airborne_s.to_bits(),"verifiedLiftoff":verified_liftoff,
                "touchdownBits":touchdown_s.to_bits(),"aborted":aborted}})
    }
}

#[cfg(all(test, feature = "experimental-solver-step-observation"))]
mod parity_tests {
    use super::*;
    fn fixture() -> StepController {
        StepController {
            feet: [18, 23],
            planted: [Vector::ZERO; 2],
            initial_midpoint: Vector::ZERO,
            initial_pelvis_height: 0.,
            pelvis_com_offset: Vector::ZERO,
            swing: Some(Swing {
                side: 0,
                start: Vector::ZERO,
                target: Vector::ZERO,
                elapsed: 0.,
                ready_s: 0.,
                swing_s: Some(0.),
                airborne_s: 0.,
                verified_liftoff: false,
                landed_s: 0.,
                demand: Vector::ZERO,
                transfer_bias: Vector::ZERO,
                qualified_reference: Some(Vector::ZERO),
                redirected: false,
                flat_started: false,
                unload_s: 0.,
                following: false,
                follow_start: Pose::from_parts(Vector::ZERO, Rotation::IDENTITY),
            }),
            next_side: 0,
            following_side: Some(0),
            repositioned: false,
            cooldown_s: 0.,
            press_start: Some((0, Vector::ZERO)),
            last_step_demand: Vector::ZERO,
            settle_from: vec![[0.; 3]],
            settle_s: 0.,
            settle_reference: Vector::ZERO,
            status: Status::default(),
        }
    }
    #[test]
    fn pure_step_private_stance_and_swing_memory_all_change_identity() {
        let original = fixture();
        let before = original.parity_memory();
        macro_rules! changed {
            ($c:ident,$edit:expr) => {{
                let mut $c = original.clone();
                $edit;
                assert_ne!(before, $c.parity_memory(), stringify!($edit));
            }};
        }
        changed!(c, c.feet[0] += 1);
        changed!(c, c.planted[0].x = 1.);
        changed!(c, c.initial_midpoint.y = 1.);
        changed!(c, c.initial_pelvis_height = 1.);
        changed!(c, c.pelvis_com_offset.z = 1.);
        changed!(c, c.swing = None);
        changed!(c, c.next_side = 1);
        changed!(c, c.following_side = None);
        changed!(c, c.repositioned = true);
        changed!(c, c.cooldown_s = 1.);
        changed!(c, c.press_start = None);
        changed!(c, c.press_start.as_mut().unwrap().0 = 1);
        changed!(c, c.press_start.as_mut().unwrap().1.x = 1.);
        changed!(c, c.last_step_demand.x = 1.);
        changed!(c, c.settle_from[0][0] = 1.);
        changed!(c, c.settle_s = 1.);
        changed!(c, c.settle_reference.x = 1.);
        changed!(c, c.swing.as_mut().unwrap().side = 1);
        changed!(c, c.swing.as_mut().unwrap().start.x = 1.);
        changed!(c, c.swing.as_mut().unwrap().target.x = 1.);
        changed!(c, c.swing.as_mut().unwrap().elapsed = 1.);
        changed!(c, c.swing.as_mut().unwrap().ready_s = 1.);
        changed!(c, c.swing.as_mut().unwrap().swing_s = None);
        changed!(c, c.swing.as_mut().unwrap().airborne_s = 1.);
        changed!(c, c.swing.as_mut().unwrap().verified_liftoff = true);
        changed!(c, c.swing.as_mut().unwrap().landed_s = 1.);
        changed!(c, c.swing.as_mut().unwrap().demand.x = 1.);
        changed!(c, c.swing.as_mut().unwrap().transfer_bias.x = 1.);
        changed!(c, c.swing.as_mut().unwrap().qualified_reference = None);
        changed!(c, c.swing.as_mut().unwrap().redirected = true);
        changed!(c, c.swing.as_mut().unwrap().flat_started = true);
        changed!(c, c.swing.as_mut().unwrap().unload_s = 1.);
        changed!(c, c.swing.as_mut().unwrap().following = true);
        changed!(c, c.swing.as_mut().unwrap().follow_start.translation.x = 1.);
        changed!(
            c,
            c.swing.as_mut().unwrap().follow_start.rotation = Rotation::from_rotation_x(0.1)
        );
        changed!(c, c.status.phase = "changed");
        changed!(c, c.status.side = Some(1));
        changed!(c, c.status.elapsed_s = 1.);
        changed!(c, c.status.loads_n[0] = 1.);
        changed!(c, c.status.sole_clearance_m[0] = 1.);
        changed!(c, c.status.retained_capture_inside = true);
        changed!(c, c.status.capture[0] = 1.);
        changed!(c, c.status.reference[0] = 1.);
        changed!(c, c.status.ready_s = 1.);
        changed!(c, c.status.airborne_s = 1.);
        changed!(c, c.status.verified_liftoff = true);
        changed!(c, c.status.touchdown_s = 1.);
        changed!(c, c.status.aborted = 1);
    }
    #[test]
    fn pure_step_projection_preserves_signed_zero_and_nan_payloads() {
        let original = fixture();
        let before = original.parity_memory();
        for bits in [(-0.0_f32).to_bits(), 0x7fc00001, 0x7fc00002] {
            let mut changed = original.clone();
            changed.settle_s = f32::from_bits(bits);
            assert_eq!(changed.parity_memory()["settleBits"], bits);
            assert_ne!(before, changed.parity_memory());
        }
    }
}

pub(crate) struct Intent {
    pub reference: Vector,
    pub targets: Vec<(usize, [f32; 3])>,
    pub velocities: Vec<(usize, [f32; 3])>,
    pub landed: bool,
}
impl StepController {
    pub fn owns_stance(&self) -> bool {
        self.status.phase != "unsupported" && (self.repositioned || self.swing.is_some())
    }
    pub fn preparing_loaded_sole(&self) -> Option<usize> {
        self.swing
            .as_ref()
            .filter(|swing| (swing.flat_started || swing.following) && swing.swing_s.is_none())
            .map(|swing| 1 - swing.side)
    }
    /// Read only, after motor installation and before integration. Both FK
    /// endpoints use that same measured pelvis, including installed corrections.
    pub(crate) fn diagnose_swing(
        &self,
        s: &Simulation,
    ) -> Result<Option<serde_json::Value>, String> {
        let Some(swing) = self.swing.as_ref().filter(|swing| swing.swing_s.is_some()) else {
            return Ok(None);
        };
        let clock = (swing.swing_s.expect("admitted swing") - s.profile.dt_s).max(0.0);
        let progress = (clock / 0.45).min(1.0);
        let smooth = progress * progress * (3.0 - 2.0 * progress);
        let mut goal = swing.start.lerp(swing.target, smooth);
        goal.y += 0.04 * (std::f32::consts::PI * progress).sin();
        let intended = leg_targets(s, self.feet[swing.side], goal)?;
        let terminal = leg_targets(s, self.feet[swing.side], swing.target)?;
        let chain: Vec<_> = intended.iter().map(|(i, _)| *i).collect();
        let root = *s.world.bodies[s.bodies[0]].position();
        let endpoint = |targets: &[(usize, [f32; 3])]| {
            let angles: Vec<_> = targets.iter().map(|(_, angles)| *angles).collect();
            crate::reaching::endpoint(s, &chain, root, &angles, Vector::ZERO)
        };
        let installed: Vec<_> = chain.iter().map(|i| (*i, s.targets[*i])).collect();
        let intended_endpoint = endpoint(&intended)?;
        let installed_endpoint = endpoint(&installed)?;
        let terminal_endpoint = endpoint(&terminal)?;
        let joint = |i: usize| {
            s.model.segments[i]
                .joint_profile
                .as_ref()
                .ok_or("Missing diagnostic leg joint")
        };
        let upper = vector(joint(chain[0])?.child_frame.anchor)
            .distance(vector(joint(chain[1])?.parent_frame.anchor));
        let lower = vector(joint(chain[1])?.child_frame.anchor)
            .distance(vector(joint(chain[2])?.parent_frame.anchor));
        let hip = root.transform_point(vector(joint(chain[0])?.parent_frame.anchor));
        let foot_offset = s.heading
            * (vector(joint(chain[3])?.child_frame.anchor)
                - vector(joint(chain[3])?.parent_frame.anchor));
        let reach_excess =
            |goal: Vector| (goal + foot_offset).distance(hip) - (upper + lower - 0.0001);
        let mut axes = Vec::new();
        for (i, angles) in &intended {
            let profile = joint(*i)?;
            let native = s
                .world
                .impulse_joints
                .get(s.joints[*i].ok_or("Missing diagnostic handle")?)
                .ok_or("Missing diagnostic native joint")?;
            let measured = measurement::coordinates(
                *s.world.bodies[native.body1()].rotation(),
                *s.world.bodies[native.body2()].rotation(),
                profile,
            );
            for axis in &profile.axes {
                let k = ['x', 'y', 'z']
                    .iter()
                    .position(|c| *c == axis.coordinate)
                    .ok_or("Unknown diagnostic coordinate")?;
                let motor = &native.data.motors[3 + k];
                axes.push(serde_json::json!({
                    "segment":i,"id":s.model.segments[*i].id,"coordinate":axis.coordinate,
                    "intendedRad":angles[k],"installedRad":s.targets[*i][k],"measuredRad":measured[k],
                    "minRad":axis.min_radians,"maxRad":axis.max_radians,
                    "velocityRadS":motor.target_vel,"capNm":motor.max_force,
                }));
            }
        }
        Ok(Some(serde_json::json!({
            "side":swing.side,"clockS":clock,"progress":progress,
            "start":crate::vec3(swing.start),"target":crate::vec3(swing.target),
            "requested":crate::vec3(goal),"intendedEndpoint":crate::vec3(intended_endpoint),
            "installedEndpoint":crate::vec3(installed_endpoint),
            "terminalEndpoint":crate::vec3(terminal_endpoint),
            "ikResidualM":intended_endpoint.distance(goal),
            "installedResidualM":installed_endpoint.distance(goal),
            "terminalResidualM":terminal_endpoint.distance(swing.target),
            "reachExcessM":reach_excess(goal),"terminalReachExcessM":reach_excess(swing.target),
            "footBefore":crate::vec3(s.world.bodies[s.bodies[self.feet[swing.side]]].translation()),
            "pelvisBefore":s.snapshot().segments[0],"stepBefore":self.status,"axesBefore":axes,
        })))
    }
    pub fn root_torque(&self, s: &Simulation, part: &lh_model::Segment) -> Vector {
        if part.role != "thigh"
            || (!self.repositioned
                && self.retained_side().is_none()
                && self
                    .swing
                    .as_ref()
                    .is_none_or(|swing| !lateral_hand_transfer(s, swing.demand)))
        {
            return Vector::ZERO;
        }
        let side = usize::from(part.id.starts_with("right"));
        let loads: [f32; 2] = std::array::from_fn(|side| {
            s.contacts
                .iter()
                .filter(|contact| {
                    let part = &s.model.segments[contact.segment as usize];
                    matches!(part.role.as_str(), "hindfoot" | "forefoot")
                        && usize::from(part.id.starts_with("right")) == side
                        && contact.persistence_s + 1e-6 >= 0.05
                })
                .map(|contact| contact.normal_load_n)
                .sum()
        });
        let total = loads.iter().sum::<f32>();
        if loads[side] < 3.0 || total < 3.0 {
            return Vector::ZERO;
        }
        let pelvis = &s.world.bodies[s.bodies[0]];
        let mut orientation = s.heading.inverse() * *pelvis.rotation();
        if orientation.w < 0.0 {
            orientation = -orientation;
        }
        let mut error = orientation.to_scaled_axis();
        let mut omega = s.heading.inverse() * pelvis.angvel();
        if self
            .swing
            .as_ref()
            .is_some_and(|swing| swing.flat_started && swing.swing_s.is_none())
        {
            error.z = 0.0;
            omega.z = 0.0;
        }
        // Child hip actuation gives the measured pelvis its equal-and-opposite
        // restoring torque. The ordinary anatomical motor budget still applies.
        s.heading * (error * 1000.0 + omega * 140.0) * (loads[side] / total)
    }
    pub fn retained_side(&self) -> Option<usize> {
        self.swing
            .as_ref()
            .filter(|swing| swing.swing_s.is_some())
            .map(|swing| 1 - swing.side)
    }
    pub fn support_reserve(&self) -> Option<(usize, f32)> {
        if self.status.phase == "unsupported" {
            return None;
        }
        if let Some(side) = self.retained_side() {
            return Some((side, 1.0));
        }
        if self.repositioned && self.settle_s < 0.6 {
            let progress = self.settle_s / 0.6;
            let blend = progress * progress * (3.0 - 2.0 * progress);
            // Landing records the previously retained side as the next mover.
            // Fade its swing allocation while the measured stance is blended.
            Some((self.next_side, 1.0 - blend))
        } else {
            None
        }
    }
    pub fn transfer_load_share(&self, s: &Simulation) -> Option<([f32; 2], f32)> {
        let swing = self.swing.as_ref()?;
        if !self.repositioned || self.status.phase != "transfer" || swing.swing_s.is_some() {
            return None;
        }
        let loads: [f32; 2] = std::array::from_fn(|side| {
            s.contacts
                .iter()
                .filter(|contact| {
                    let segment = &s.model.segments[contact.segment as usize];
                    matches!(segment.role.as_str(), "hindfoot" | "forefoot")
                        && usize::from(segment.id.starts_with("right")) == side
                        && contact.persistence_s + 1e-6 >= 0.05
                })
                .map(|contact| contact.normal_load_n)
                .sum()
        });
        let total = loads.iter().sum::<f32>();
        (total >= 3.0).then(|| {
            (
                loads.map(|load| load / total),
                (swing.elapsed / 0.6).min(1.0),
            )
        })
    }
    pub fn new(s: &Simulation) -> Result<Self, String> {
        let find = |id: &str| {
            s.model
                .segments
                .iter()
                .position(|p| p.id == id)
                .ok_or("Missing foot")
        };
        let feet = [find("leftFoot")?, find("rightFoot")?];
        let planted = feet.map(|i| s.world.bodies[s.bodies[i]].translation());
        Ok(Self {
            feet,
            planted,
            initial_midpoint: (planted[0] + planted[1]) * 0.5,
            initial_pelvis_height: s.world.bodies[s.bodies[0]].translation().y,
            pelvis_com_offset: s.rest_com - s.world.bodies[s.bodies[0]].translation(),
            swing: None,
            next_side: 0,
            following_side: None,
            repositioned: false,
            cooldown_s: 0.0,
            press_start: None,
            last_step_demand: Vector::ZERO,
            settle_from: s.rest_targets.clone(),
            settle_s: 0.6,
            settle_reference: s.rest_com,
            status: Status {
                phase: "idle",
                ..Status::default()
            },
        })
    }
    pub(crate) fn planted_feet(&self) -> [Vector; 2] {
        self.planted
    }
    /// The ordinary stance controller receives measured support and coordinates.
    /// Recovery earns no completion while this 0.6 second blend is in progress.
    pub(crate) fn recovered_stance(s: &Simulation, planted: [Vector; 2]) -> Result<Self, String> {
        let mut result = Self::new(s)?;
        result.planted = planted;
        result.repositioned = true;
        result.settle_s = 0.0;
        result.settle_reference = s.center_of_mass();
        result.pelvis_com_offset = s.center_of_mass() - s.world.bodies[s.bodies[0]].translation();
        for (i, angles) in measured_leg_targets(s)? {
            result.settle_from[i] = angles;
        }
        Ok(result)
    }
    pub fn update(&mut self, s: &Simulation) -> Result<Intent, String> {
        let dt = s.profile.dt_s;
        self.cooldown_s = (self.cooldown_s - dt).max(0.0);
        let mut load = [0.0; 2];
        let mut clearance = [0.20_f32; 2];
        let mut support: [Vec<[f32; 2]>; 2] = Default::default();
        for (i, segment) in s
            .model
            .segments
            .iter()
            .enumerate()
            .filter(|(_, p)| matches!(p.role.as_str(), "hindfoot" | "forefoot"))
        {
            let force = s.support_normal_impulse(s.colliders[i])? / dt;
            let side = usize::from(segment.id.starts_with("right"));
            load[side] += force;
            let foot = &s.world.colliders[s.colliders[i]];
            let mut distance = 0.20_f32;
            for handle in s.environment_colliders() {
                let surface = &s.world.colliders[handle];
                if !surface.is_enabled() {
                    continue;
                }
                if let Some(contact) = rapier3d::parry::query::contact(
                    surface.position(),
                    surface.shape(),
                    foot.position(),
                    foot.shape(),
                    0.20,
                )
                .map_err(|e| format!("Step sole clearance: {e:?}"))?
                    && contact.normal1.y >= 0.65
                {
                    distance = distance.min(contact.dist);
                }
            }
            clearance[side] = clearance[side].min(distance);
            if force < 3.0 {
                continue;
            }
            let pose = s.world.bodies[s.bodies[i]].position();
            for v in segment.geometry.vertices.chunks_exact(3) {
                let world = pose.transform_point(Vector::new(v[0], v[1], v[2]));
                if world.y - s.surface_height(world.x, world.z, world.y + 0.02)
                    > s.profile.contact_skin_m + 0.012
                {
                    continue;
                }
                let point = s.heading.inverse() * world;
                support[side].push([point.x, point.z]);
            }
        }
        if self.swing.is_none() && !self.repositioned {
            self.planted = self.feet.map(|i| s.world.bodies[s.bodies[i]].translation());
            self.initial_pelvis_height = s.world.bodies[s.bodies[0]].translation().y;
        }
        let mut reference =
            s.rest_com + (self.planted[0] + self.planted[1]) * 0.5 - self.initial_midpoint;
        let mut targets = Vec::new();
        let mut velocities = Vec::new();
        let mut landed = false;
        self.status.loads_n = load;
        self.status.sole_clearance_m = clearance;
        if load.iter().all(|n| *n < 3.0) {
            self.swing = None;
            self.status.phase = "unsupported";
            self.status.side = None;
            return Ok(Intent {
                reference: s.rest_com,
                targets,
                velocities,
                landed,
            });
        }
        let com = s.center_of_mass();
        let velocity: Vector = s
            .bodies
            .iter()
            .map(|h| {
                let b = &s.world.bodies[*h];
                b.linvel() * b.mass()
            })
            .sum::<Vector>()
            / s.total_mass();
        let capture = s.heading.inverse()
            * (com + velocity * ((com.y - s.support_height()).max(0.2) / 9.81).sqrt());
        if self.repositioned {
            self.settle_s = (self.settle_s + dt).min(0.6);
            let blend_at = |time: f32| {
                let progress = (time / 0.6).min(1.0);
                progress * progress * (3.0 - 2.0 * progress)
            };
            let blend = blend_at(self.settle_s);
            let next_blend = blend_at(self.settle_s + dt);
            let next_reference = self.settle_reference.lerp(reference, next_blend);
            reference = self.settle_reference.lerp(reference, blend);
            let height = self.initial_pelvis_height + (self.planted[0].y + self.planted[1].y) * 0.5
                - self.initial_midpoint.y;
            let stance_root = |reference| {
                let mut root = Pose::from_parts(reference - self.pelvis_com_offset, s.heading);
                let yaw = (s.heading.inverse() * *s.world.bodies[s.bodies[0]].rotation())
                    .to_scaled_axis()
                    .y;
                root.rotation *= Rotation::from_rotation_y(yaw);
                root.translation.y = height;
                reachable_stance_root(s, self.feet, self.planted, root)
            };
            let root = stance_root(reference)?;
            let next_root = stance_root(next_reference)?;
            for side in 0..2 {
                let intent = leg_targets_with_root(s, self.feet[side], self.planted[side], root)?;
                let next = if next_blend == blend {
                    intent.clone()
                } else {
                    leg_targets_with_root(s, self.feet[side], self.planted[side], next_root)?
                };
                for ((i, desired), (_, next_desired)) in intent.into_iter().zip(next) {
                    let profile = s.model.segments[i]
                        .joint_profile
                        .as_ref()
                        .ok_or("Missing stance profile")?;
                    let interpolate = |desired: [f32; 3], blend: f32| {
                        std::array::from_fn(|k| {
                            profile
                                .axes
                                .iter()
                                .find(|axis| axis.coordinate == ['x', 'y', 'z'][k])
                                .map_or(0.0, |axis| {
                                    (self.settle_from[i][k]
                                        + (desired[k] - self.settle_from[i][k]) * blend)
                                        .clamp(axis.min_radians, axis.max_radians)
                                })
                        })
                    };
                    let current: [f32; 3] = interpolate(desired, blend);
                    let future: [f32; 3] = interpolate(next_desired, next_blend);
                    velocities.push((i, std::array::from_fn(|k| (future[k] - current[k]) / dt)));
                    targets.push((i, current));
                }
            }
        }
        let mut demand = Vector::ZERO;
        let mut dragged_side = None;
        if let Some(grab) = &s.grab {
            if self
                .press_start
                .is_none_or(|(press, _)| press != grab.press)
            {
                self.press_start = Some((grab.press, grab.control_target()));
                self.last_step_demand = Vector::ZERO;
            }
            demand = grab.control_target() - self.press_start.expect("press initialized").1;
            demand.y = 0.0;
            if s.model.segments[grab.segment].role == "hindfoot" {
                dragged_side = Some(usize::from(
                    s.model.segments[grab.segment].id.starts_with("right"),
                ));
            }
        } else {
            self.press_start = None;
        }
        if self.repositioned
            && s.grab.is_none()
            && self
                .swing
                .as_ref()
                .is_some_and(|swing| !swing.following && swing.swing_s.is_none())
            && load.iter().all(|n| *n >= 3.0)
            && inside_support(
                support[0].iter().chain(&support[1]).copied().collect(),
                [capture.x, capture.z],
                0.003,
            )
        {
            self.swing = None;
            self.cooldown_s = 0.15;
        }
        if self.swing.is_none() && self.cooldown_s == 0.0 {
            let following_side = self.following_side;
            let points = support[0].iter().chain(&support[1]).copied().collect();
            let outside = !inside_support(points, [capture.x, capture.z], 0.015);
            let requested = s.grab.is_some()
                && (demand - self.last_step_demand).length()
                    >= if dragged_side.is_some() { 0.075 } else { 0.16 };
            if (outside || requested && self.settle_s >= 0.6 || following_side.is_some())
                && load.iter().all(|n| *n >= 3.0)
            {
                let midpoint = s.heading.inverse() * ((self.planted[0] + self.planted[1]) * 0.5);
                let direction = s.heading.inverse() * demand;
                let side = if let Some(side) = following_side {
                    side
                } else if let Some(side) = dragged_side {
                    side
                } else if direction.x > 0.05 || capture.x - midpoint.x > 0.05 {
                    1
                } else if direction.x < -0.05 || capture.x - midpoint.x < -0.05 {
                    0
                } else {
                    self.next_side
                };
                let start = self.planted[side];
                let mut target = s.heading * capture;
                target += velocity * 0.12;
                if requested && demand.length() > 0.0 {
                    target = start + demand.normalize() * (0.07 + demand.length() * 0.45).min(0.28);
                }
                target.y = start.y;
                let mut travel = target - start;
                travel.y = 0.0;
                if travel.length() > 0.28 {
                    travel *= 0.28 / travel.length();
                }
                target = start + travel;
                // Keep a physical stance width; a corrective step cannot route
                // through the other leg to make its target appear reachable.
                let mut local = s.heading.inverse() * target;
                let other = s.heading.inverse() * self.planted[1 - side];
                if side == 0 {
                    local.x = local.x.min(other.x - 0.14);
                } else {
                    local.x = local.x.max(other.x + 0.14);
                }
                if following_side.is_some() {
                    local.x =
                        (s.heading.inverse() * start).x + if side == 0 { -0.07 } else { 0.07 };
                    local.z = (s.heading.inverse() * start).z;
                }
                target = s.heading * local;
                if s.environment.is_some() {
                    let foot = &s.model.segments[self.feet[side]];
                    let sole_offset = -foot
                        .geometry
                        .vertices
                        .chunks_exact(3)
                        .map(|v| v[1])
                        .fold(f32::INFINITY, f32::min);
                    target.y = s.surface_height(target.x, target.z, start.y + 0.6) + sole_offset;
                }
                self.swing = Some(Swing {
                    side,
                    start,
                    target,
                    elapsed: 0.0,
                    ready_s: 0.0,
                    swing_s: None,
                    airborne_s: 0.0,
                    verified_liftoff: false,
                    landed_s: 0.0,
                    demand,
                    transfer_bias: Vector::ZERO,
                    qualified_reference: None,
                    redirected: false,
                    flat_started: false,
                    unload_s: 0.0,
                    following: following_side.is_some(),
                    follow_start: Pose::from_parts(
                        s.world.bodies[s.bodies[0]].translation(),
                        *s.world.bodies[s.bodies[0]].rotation(),
                    ),
                });
            }
        }
        if let Some(swing) = &mut self.swing {
            // A pointer reversal can change the intended mover only before
            // measured admission. The committed swing keeps its own clock.
            if swing.swing_s.is_none()
                && let Some(grab) = &s.grab
                && grab.reverses(swing.demand)
            {
                let direction = s.heading.inverse() * grab.target_velocity();
                let side = usize::from(direction.x > 0.0);
                if direction.x.abs() > direction.z.abs() && side != swing.side {
                    swing.side = side;
                    swing.start = s.world.bodies[s.bodies[self.feet[side]]].translation();
                    let mut travel = grab.target_velocity();
                    travel.y = 0.0;
                    travel = travel.normalize() * 0.07;
                    swing.target = swing.start + travel;
                    swing.elapsed = 0.0;
                    swing.ready_s = 0.0;
                    swing.transfer_bias = Vector::ZERO;
                    swing.qualified_reference = None;
                    swing.demand = travel;
                    swing.redirected = true;
                    swing.flat_started = false;
                    swing.unload_s = 0.0;
                }
            }
            swing.elapsed += dt;
            let retained = 1 - swing.side;
            let transfer = (swing.elapsed / 0.6).min(1.0);
            let retained_reference = s.rest_com + self.planted[retained] - self.initial_midpoint;
            if swing.swing_s.is_none() {
                let mut error = retained_reference - s.heading * capture;
                error.y = 0.0;
                swing.transfer_bias += error * (2.0 * dt);
                if swing.transfer_bias.length() > 0.10 {
                    swing.transfer_bias *= 0.10 / swing.transfer_bias.length();
                }
            }
            let desired = s.heading.inverse() * (retained_reference + swing.transfer_bias);
            if let Some(point) = project_support(&support[retained], [desired.x, desired.z], 0.003)
            {
                swing.qualified_reference =
                    Some(s.heading * Vector::new(point[0], desired.y, point[1]));
            }
            // Losing a usable support polygon cannot turn measured drift into
            // a new COM goal. Readiness remains false until support is measured.
            let qualified_reference = swing.qualified_reference.unwrap_or(retained_reference);
            // Anti-windup keeps intent inside the measured retained sole rather
            // than driving the COM past its edge while waiting for unloading.
            swing.transfer_bias = qualified_reference - retained_reference;
            reference = reference.lerp(qualified_reference, transfer);
            if let Some(time) = swing.swing_s
                && load[retained] >= 3.0
                && !swing.following
            {
                let mut local = s.heading.inverse() * reference;
                let sole = s.heading.inverse()
                    * s.world.bodies[s.bodies[self.feet[retained]]].translation();
                let progress = (time / 0.2).min(1.0);
                let blend = progress * progress * (3.0 - 2.0 * progress);
                let centered = sole.x + if retained == 0 { -0.015 } else { 0.015 };
                local.x += (centered - local.x) * blend;
                reference = s.heading * local;
            }
            let retained_capture_inside =
                inside_support(support[retained].clone(), [capture.x, capture.z], 0.003);
            if swing.swing_s.is_none() {
                let sole = s.heading.inverse()
                    * s.world.bodies[s.bodies[self.feet[retained]]].translation();
                if swing.redirected
                    && (capture.x - sole.x).abs() < 0.03
                    && Vector::new(velocity.x, 0.0, velocity.z).length() < 0.12
                    && load[retained] >= 0.52 * s.total_mass() * 9.81
                    && load[swing.side] <= 0.25 * s.total_mass() * 9.81
                {
                    swing.flat_started = true;
                }
                if !self.repositioned && lateral_hand_transfer(s, swing.demand) {
                    let error = s.heading.inverse() * (reference - com);
                    let sway = if swing.redirected {
                        (2.0 * error.x).clamp(-0.15, 0.15)
                    } else {
                        (0.5 * error.x).clamp(-0.05, 0.05)
                    };
                    for (i, part) in s
                        .model
                        .segments
                        .iter()
                        .enumerate()
                        .filter(|(_, part)| part.role == "thigh")
                    {
                        let profile = part.joint_profile.as_ref().ok_or("Missing transfer hip")?;
                        let axis = profile
                            .axes
                            .iter()
                            .find(|axis| axis.coordinate == 'z')
                            .ok_or("Missing transfer hip roll")?;
                        let mut angles = s.rest_targets[i];
                        angles[2] = (angles[2] + sway).clamp(axis.min_radians, axis.max_radians);
                        targets.push((i, angles));
                    }
                }
                if swing.redirected || swing.following {
                    // This small motor-driven toe-off prepares unloading. It
                    // never substitutes for the persistent load/capture proof.
                    if retained_capture_inside
                        && (swing.following || load[retained] >= 0.52 * s.total_mass() * 9.81)
                        && (!swing.redirected
                            || s.grab
                                .as_ref()
                                .is_none_or(|grab| grab.target_velocity().length() <= 0.05))
                    {
                        swing.unload_s = (swing.unload_s + dt).min(0.10);
                    } else {
                        swing.unload_s = 0.0;
                    }
                    let goal = |side: usize, time: f32| {
                        let progress = (time / 0.10).min(1.0);
                        let mut goal = self.planted[side];
                        if side == swing.side {
                            goal.y += 0.010 * progress * progress * (3.0 - 2.0 * progress);
                        }
                        goal
                    };
                    let next_unload = if swing.unload_s > 0.0 {
                        swing.unload_s + dt
                    } else {
                        0.0
                    };
                    let current =
                        leg_targets(s, self.feet[swing.side], goal(swing.side, swing.unload_s))?;
                    let next =
                        leg_targets(s, self.feet[swing.side], goal(swing.side, next_unload))?;
                    for ((i, angles), (_, future)) in current.into_iter().zip(next) {
                        targets.push((i, angles));
                        velocities.push((i, std::array::from_fn(|k| (future[k] - angles[k]) / dt)));
                    }
                }
                let weight = s.total_mass() * 9.81;
                let persistent_load: f32 = s
                    .contacts
                    .iter()
                    .filter(|contact| {
                        let segment = &s.model.segments[contact.segment as usize];
                        matches!(segment.role.as_str(), "hindfoot" | "forefoot")
                            && usize::from(segment.id.starts_with("right")) == retained
                            && contact.persistence_s + 1e-6 >= 0.05
                    })
                    .map(|contact| contact.normal_load_n)
                    .sum();
                if retained_capture_inside
                    && persistent_load >= 0.52 * weight
                    && load[swing.side] <= 0.25 * weight
                    && (!swing.redirected
                        || s.grab
                            .as_ref()
                            .is_none_or(|grab| grab.target_velocity().length() <= 0.05))
                {
                    swing.ready_s += dt;
                } else {
                    swing.ready_s = 0.0;
                }
                if swing.ready_s + 1e-6 >= 0.10 {
                    let measured = s.world.bodies[s.bodies[self.feet[swing.side]]].translation();
                    swing.target += measured - swing.start;
                    swing.start = measured;
                    if s.grab
                        .as_ref()
                        .is_some_and(|grab| grab.target_velocity().dot(swing.demand) < -0.01)
                    {
                        swing.target = s.heading * capture + velocity * 0.12;
                        swing.target.y = measured.y;
                        let mut local = s.heading.inverse() * swing.target;
                        let other = s.heading.inverse()
                            * s.world.bodies[s.bodies[self.feet[retained]]].translation();
                        local.x = if swing.side == 0 {
                            local.x.min(other.x - 0.14)
                        } else {
                            local.x.max(other.x + 0.14)
                        };
                        swing.target = s.heading * local;
                        swing.demand = demand;
                    }
                    swing.swing_s = Some(0.0);
                    if swing.redirected || swing.following {
                        let foot = &s.model.segments[self.feet[swing.side]];
                        let sole_offset = -foot
                            .geometry
                            .vertices
                            .chunks_exact(3)
                            .map(|v| v[1])
                            .fold(f32::INFINITY, f32::min);
                        swing.target.y =
                            s.surface_height(swing.target.x, swing.target.z, swing.start.y + 0.6)
                                + sole_offset;
                    }
                }
            }
            if swing.following {
                // Continue a reversed demand on the already loaded support
                // side. Only the retained leg uses the planned pelvis motion;
                // the mover stays anchored through measured-pelvis IK above.
                let height = self.initial_pelvis_height
                    + (self.planted[0].y + self.planted[1].y) * 0.5
                    - self.initial_midpoint.y;
                let yaw = (s.heading.inverse() * *s.world.bodies[s.bodies[0]].rotation())
                    .to_scaled_axis()
                    .y;
                let mut end = Pose::from_parts(
                    retained_reference - self.pelvis_com_offset,
                    s.heading * Rotation::from_rotation_y(yaw),
                );
                end.translation +=
                    s.heading * Vector::X * if retained == 0 { -0.015 } else { 0.015 };
                end.translation.y = height;
                let root_at = |time: f32| {
                    let time = (time / 0.8).min(1.0);
                    let blend = time * time * (3.0 - 2.0 * time);
                    let root = Pose::from_parts(
                        swing.follow_start.translation.lerp(end.translation, blend),
                        swing.follow_start.rotation.slerp(end.rotation, blend),
                    );
                    reachable_stance_root(s, self.feet, self.planted, root)
                };
                let root = root_at(swing.elapsed)?;
                let next_root = root_at(swing.elapsed + dt)?;
                reference = root.translation + self.pelvis_com_offset;
                let current =
                    leg_targets_with_root(s, self.feet[retained], self.planted[retained], root)?;
                let next = leg_targets_with_root(
                    s,
                    self.feet[retained],
                    self.planted[retained],
                    next_root,
                )?;
                for ((i, angles), (_, future)) in current.into_iter().zip(next) {
                    targets.push((i, angles));
                    velocities.push((i, std::array::from_fn(|k| (future[k] - angles[k]) / dt)));
                }
            }
            let progress = swing.swing_s.map_or(0.0, |time| (time / 0.45).min(1.0));
            if let Some(time) = &mut swing.swing_s {
                *time += dt;
                let goal = |progress: f32| {
                    let smooth = progress * progress * (3.0 - 2.0 * progress);
                    let mut goal = swing.start.lerp(swing.target, smooth);
                    goal.y += 0.04 * (std::f32::consts::PI * progress).sin();
                    goal
                };
                let current = leg_targets(s, self.feet[swing.side], goal(progress))?;
                let next = leg_targets(s, self.feet[swing.side], goal((*time / 0.45).min(1.0)))?;
                // The motor's velocity reference follows the same anatomical
                // trajectory as its position reference. Both samples use the
                // measured pelvis pose; no rigid body is advanced or rewritten.
                velocities.extend(current.iter().zip(&next).map(|((i, from), (_, to))| {
                    (*i, std::array::from_fn(|k| (to[k] - from[k]) / dt))
                }));
                targets.extend(current);
            }
            let actual = s.world.bodies[s.bodies[self.feet[swing.side]]].translation();
            let displacement = actual - swing.start;
            let horizontal_travel = Vector::new(displacement.x, 0.0, displacement.z).length();
            if swing.swing_s.is_some()
                && lifted(
                    displacement.y,
                    load[swing.side],
                    clearance[swing.side],
                    s.profile.contact_skin_m,
                )
            {
                swing.airborne_s += dt;
                swing.verified_liftoff |= swing.airborne_s >= 0.05;
            } else {
                swing.airborne_s = 0.0;
            }
            if progress == 1.0
                && swing.verified_liftoff
                && load[retained] >= 3.0
                && load[swing.side] >= 3.0
                && horizontal_travel >= 0.04
                && Vector::new(actual.x - swing.target.x, 0.0, actual.z - swing.target.z).length()
                    <= 0.09
            {
                swing.landed_s += dt;
            } else {
                swing.landed_s = 0.0;
            }
            if swing.landed_s > 0.0 {
                let feet = self.feet.map(|i| s.world.bodies[s.bodies[i]].translation());
                let grounded = s.rest_com + (feet[0] + feet[1]) * 0.5 - self.initial_midpoint;
                let progress = (swing.landed_s / 0.10).min(1.0);
                let blend = progress * progress * (3.0 - 2.0 * progress);
                reference = reference.lerp(grounded, blend);
            }
            self.status = Status {
                phase: if swing.swing_s.is_some() {
                    "swing"
                } else {
                    "transfer"
                },
                side: Some(swing.side),
                elapsed_s: swing.elapsed,
                loads_n: load,
                sole_clearance_m: clearance,
                retained_capture_inside,
                capture: [capture.x, capture.z],
                reference: {
                    let point = s.heading.inverse() * reference;
                    [point.x, point.z]
                },
                ready_s: swing.ready_s,
                airborne_s: swing.airborne_s,
                verified_liftoff: swing.verified_liftoff,
                touchdown_s: swing.landed_s,
                aborted: self.status.aborted,
            };
            if swing.landed_s >= 0.10 {
                // Both soles belong to the new measured stance: the retained
                // sole may also have moved under the external disturbance.
                self.planted = self.feet.map(|i| s.world.bodies[s.bodies[i]].translation());
                let mut local = self.planted.map(|point| s.heading.inverse() * point);
                if local[1].x - local[0].x < 0.16 {
                    let center = (local[0].x + local[1].x) * 0.5;
                    local[0].x = center - 0.08;
                    local[1].x = center + 0.08;
                    self.planted = local.map(|point| s.heading * point);
                }
                self.next_side = retained;
                if swing.following {
                    self.following_side = None;
                } else if swing.redirected {
                    self.following_side = Some(swing.side);
                }
                self.repositioned = true;
                self.last_step_demand = swing.demand;
                self.settle_reference = reference;
                self.settle_s = 0.0;
                for (i, angles) in measured_leg_targets(s)? {
                    self.settle_from[i] = angles;
                }
                self.swing = None;
                self.cooldown_s = 0.15;
                landed = true;
                self.status.phase = "landed";
            } else if swing.swing_s.unwrap_or(swing.elapsed)
                > if swing.redirected && swing.swing_s.is_none() {
                    5.0
                } else {
                    2.0
                }
            {
                // A late admitted swing owns a new bounded phase clock; time
                // spent proving unloaded support cannot consume its trajectory.
                // An uncompleted request cannot increment the step counter.
                self.swing = None;
                self.cooldown_s = 0.15;
                self.status.phase = "aborted";
                self.status.aborted += 1;
            }
        } else {
            self.status.phase = "idle";
            self.status.side = None;
        }
        self.status.capture = [capture.x, capture.z];
        let local_reference = s.heading.inverse() * reference;
        self.status.reference = [local_reference.x, local_reference.z];
        Ok(Intent {
            reference,
            targets,
            velocities,
            landed,
        })
    }
}

fn lateral_hand_transfer(s: &Simulation, demand: Vector) -> bool {
    let direction = s.heading.inverse() * demand;
    direction.x.abs() > direction.z.abs()
        && s.grab
            .as_ref()
            .is_some_and(|grab| s.model.segments[grab.segment].role != "hindfoot")
}

/// Solver compliance can leave a measured joint slightly outside its range.
/// A stance motor request must still honor the hard anatomical target bounds.
fn measured_leg_targets(s: &Simulation) -> Result<Vec<(usize, [f32; 3])>, String> {
    s.model
        .segments
        .iter()
        .enumerate()
        .filter(|(_, segment)| {
            matches!(
                segment.role.as_str(),
                "thigh" | "shin" | "ankle" | "hindfoot"
            )
        })
        .map(|(i, segment)| {
            let profile = segment
                .joint_profile
                .as_ref()
                .ok_or("Missing stance profile")?;
            let joint = s
                .world
                .impulse_joints
                .get(s.joints[i].ok_or("Missing stance joint")?)
                .ok_or("Lost stance joint")?;
            let measured = measurement::coordinates(
                *s.world.bodies[joint.body1()].rotation(),
                *s.world.bodies[joint.body2()].rotation(),
                profile,
            );
            let angles = std::array::from_fn(|k| {
                profile
                    .axes
                    .iter()
                    .find(|axis| axis.coordinate == ['x', 'y', 'z'][k])
                    .map_or(0.0, |axis| {
                        measured[k].clamp(axis.min_radians, axis.max_radians)
                    })
            });
            Ok((i, angles))
        })
        .collect()
}

/// Lower a requested upright stance enough for both measured soles to remain
/// within the actual two-link reach. This is an IK intent, never a body write.
pub(crate) fn reachable_stance_root(
    s: &Simulation,
    feet: [usize; 2],
    goals: [Vector; 2],
    root: Pose,
) -> Result<Pose, String> {
    reachable_stance_root_at_pitch(s, feet, goals, root, None)
}

pub(crate) fn reachable_stance_root_at_pitch(
    s: &Simulation,
    feet: [usize; 2],
    goals: [Vector; 2],
    mut root: Pose,
    pitches: Option<[f32; 2]>,
) -> Result<Pose, String> {
    for (side, (foot, goal)) in feet.into_iter().zip(goals).enumerate() {
        let parent = |i: usize| {
            s.model
                .segments
                .iter()
                .position(|p| Some(&p.id) == s.model.segments[i].parent.as_ref())
                .ok_or("Missing stance parent")
        };
        let ankle = parent(foot)?;
        let shin = parent(ankle)?;
        let thigh = parent(shin)?;
        let joint = |i: usize| {
            s.model.segments[i]
                .joint_profile
                .as_ref()
                .ok_or("Missing stance joint")
        };
        let upper = vector(joint(thigh)?.child_frame.anchor)
            .distance(vector(joint(shin)?.parent_frame.anchor));
        let lower = vector(joint(shin)?.child_frame.anchor)
            .distance(vector(joint(ankle)?.parent_frame.anchor));
        let hip_offset = root.rotation * vector(joint(thigh)?.parent_frame.anchor);
        let sole = pitches.map_or(s.heading, |pitch| {
            s.heading * Rotation::from_rotation_x(pitch[side])
        });
        let pivot = goal
            + sole
                * (vector(joint(foot)?.child_frame.anchor)
                    - vector(joint(foot)?.parent_frame.anchor));
        let horizontal = Vector::new(
            pivot.x - root.translation.x - hip_offset.x,
            0.0,
            pivot.z - root.translation.z - hip_offset.z,
        )
        .length_squared();
        let reach = upper + lower - 0.0001;
        if horizontal >= reach * reach {
            return Err("Requested stance has no vertical leg reach".into());
        }
        let maximum_y = pivot.y + (reach * reach - horizontal).sqrt() - hip_offset.y;
        root.translation.y = root.translation.y.min(maximum_y);
    }
    Ok(root)
}

fn lifted(raise: f32, load: f32, clearance: f32, skin: f32) -> bool {
    raise >= 0.02 && load < 3.0 && clearance > 2.0 * skin + 0.002
}

// Monotone-chain hull of the exact loaded sole vertices in heading coordinates.
// A bounding rectangle would incorrectly admit points outside a tapered sole.
fn support_hull(mut points: Vec<[f32; 2]>) -> Vec<[f32; 2]> {
    points.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    points.dedup();
    if points.len() < 3 {
        return Vec::new();
    }
    let cross = |a: [f32; 2], b: [f32; 2], c: [f32; 2]| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    let mut hull = Vec::new();
    for p in &points {
        while hull.len() >= 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], *p) <= 0.0 {
            hull.pop();
        }
        hull.push(*p);
    }
    let lower = hull.len();
    for p in points.iter().rev().skip(1) {
        while hull.len() > lower && cross(hull[hull.len() - 2], hull[hull.len() - 1], *p) <= 0.0 {
            hull.pop();
        }
        hull.push(*p);
    }
    hull.pop();
    hull
}
pub(crate) fn inside_support(points: Vec<[f32; 2]>, point: [f32; 2], margin: f32) -> bool {
    inside_hull(&support_hull(points), point, margin)
}
fn inside_hull(hull: &[[f32; 2]], point: [f32; 2], margin: f32) -> bool {
    if hull.len() < 3 {
        return false;
    }
    let cross = |a: [f32; 2], b: [f32; 2], c: [f32; 2]| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    hull.iter().zip(hull.iter().cycle().skip(1)).all(|(a, b)| {
        let length = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
        length > 0.0 && cross(*a, *b, point) >= margin * length
    })
}
pub(crate) fn project_support(
    points: &[[f32; 2]],
    desired: [f32; 2],
    margin: f32,
) -> Option<[f32; 2]> {
    let hull = support_hull(points.to_vec());
    if hull.is_empty() {
        return None;
    }
    if inside_hull(&hull, desired, margin) {
        return Some(desired);
    }
    let center = hull
        .iter()
        .fold([0.0; 2], |sum, point| {
            [sum[0] + point[0], sum[1] + point[1]]
        })
        .map(|value| value / hull.len() as f32);
    if !inside_hull(&hull, center, margin) {
        return None;
    }
    let mut low = 0.0;
    let mut high = 1.0;
    for _ in 0..16 {
        let fraction = (low + high) * 0.5;
        let candidate = std::array::from_fn(|k| center[k] + (desired[k] - center[k]) * fraction);
        if inside_hull(&hull, candidate, margin) {
            low = fraction;
        } else {
            high = fraction;
        }
    }
    Some(std::array::from_fn(|k| {
        center[k] + (desired[k] - center[k]) * low
    }))
}

/// Exact two-link leg geometry in a shared sagittal plane, including ankle and
/// hindfoot offsets. All candidate rotations stay outside the live world.
fn leg_targets(
    s: &Simulation,
    foot: usize,
    goal: Vector,
) -> Result<Vec<(usize, [f32; 3])>, String> {
    leg_targets_at(s, foot, goal, None)
}
fn leg_targets_at(
    s: &Simulation,
    foot: usize,
    goal: Vector,
    pelvis_height: Option<f32>,
) -> Result<Vec<(usize, [f32; 3])>, String> {
    leg_targets_in_frame(s, foot, goal, pelvis_height, None, None)
}

pub(crate) fn leg_targets_with_root(
    s: &Simulation,
    foot: usize,
    goal: Vector,
    pelvis: Pose,
) -> Result<Vec<(usize, [f32; 3])>, String> {
    leg_targets_in_frame(s, foot, goal, None, Some(pelvis), None)
}

pub(crate) fn leg_targets_with_root_and_pitch(
    s: &Simulation,
    foot: usize,
    goal: Vector,
    pelvis: Pose,
    pitch: f32,
) -> Result<Vec<(usize, [f32; 3])>, String> {
    leg_targets_in_frame(s, foot, goal, None, Some(pelvis), Some(pitch))
}

fn leg_targets_in_frame(
    s: &Simulation,
    foot: usize,
    goal: Vector,
    pelvis_height: Option<f32>,
    root: Option<Pose>,
    sole_pitch: Option<f32>,
) -> Result<Vec<(usize, [f32; 3])>, String> {
    let parent_index = |index: usize| -> Result<usize, String> {
        let id = s.model.segments[index]
            .parent
            .as_ref()
            .ok_or("Missing leg parent")?;
        s.model
            .segments
            .iter()
            .position(|p| &p.id == id)
            .ok_or("Unknown leg parent".into())
    };
    let ankle = parent_index(foot)?;
    let shin = parent_index(ankle)?;
    let thigh = parent_index(shin)?;
    let pelvis = parent_index(thigh)?;
    let joint = |i: usize| {
        s.model.segments[i]
            .joint_profile
            .as_ref()
            .ok_or("Missing leg joint")
    };
    let mut pelvis_pose = root.unwrap_or(*s.world.bodies[s.bodies[pelvis]].position());
    if let Some(height) = pelvis_height {
        pelvis_pose.translation.y = height;
        // A planted stance requests an upright pelvis through the anatomical
        // hip motors. Following its measured tilt would erase that restoring
        // joint error after landing. This pose exists only in the IK calculation.
        pelvis_pose.rotation = s.heading;
    }
    let hip = pelvis_pose.transform_point(vector(joint(thigh)?.parent_frame.anchor));
    let upper =
        vector(joint(thigh)?.child_frame.anchor).distance(vector(joint(shin)?.parent_frame.anchor));
    let lower =
        vector(joint(shin)?.child_frame.anchor).distance(vector(joint(ankle)?.parent_frame.anchor));
    let sole = sole_pitch.map_or(s.heading, |pitch| {
        s.heading * Rotation::from_rotation_x(pitch)
    });
    let pivot = goal
        + sole
            * (vector(joint(foot)?.child_frame.anchor) - vector(joint(foot)?.parent_frame.anchor));
    let local = s.heading.inverse() * (pivot - hip);
    if !local.is_finite() || upper <= 0.0 || lower <= 0.0 {
        return Err("Invalid leg geometry".into());
    }
    let roll = local.x.atan2(-local.y);
    let plane = Rotation::from_rotation_z(-roll) * local;
    let distance = plane
        .length()
        .clamp((upper - lower).abs() + 0.0001, upper + lower - 0.0001);
    let pitch = (-plane.z).atan2(-plane.y);
    let proximal = ((upper * upper + distance * distance - lower * lower)
        / (2.0 * upper * distance))
        .clamp(-1.0, 1.0)
        .acos();
    let distal = ((lower * lower + distance * distance - upper * upper) / (2.0 * lower * distance))
        .clamp(-1.0, 1.0)
        .acos();
    let plane_rotation = s.heading * Rotation::from_rotation_z(roll);
    let orientations = [
        plane_rotation * Rotation::from_rotation_x(pitch - proximal),
        plane_rotation * Rotation::from_rotation_x(pitch + distal),
        sole_pitch.map_or(plane_rotation, |pitch| {
            plane_rotation * Rotation::from_rotation_x(pitch)
        }),
        sole,
    ];
    let mut parent = pelvis_pose.rotation;
    let mut result = Vec::new();
    for (i, child) in [thigh, shin, ankle, foot].into_iter().zip(orientations) {
        let profile = joint(i)?;
        let mut angles = measurement::coordinates(parent, child, profile);
        for (k, coordinate) in ['x', 'y', 'z'].into_iter().enumerate() {
            angles[k] = profile
                .axes
                .iter()
                .find(|a| a.coordinate == coordinate)
                .map_or(0.0, |a| angles[k].clamp(a.min_radians, a.max_radians));
        }
        result.push((i, angles));
        parent = child;
    }
    // Anatomical angle clamping changes the solved foot pose. Refine that
    // constrained result through real joint frames instead of assuming the
    // unclamped two-link endpoint was retained.
    let chain = [thigh, shin, ankle, foot];
    let mut angles: [[f32; 3]; 4] = std::array::from_fn(|k| result[k].1);
    if crate::reaching::endpoint(s, &chain, pelvis_pose, &angles, Vector::ZERO)?.distance(goal)
        > 1e-5
    {
        refine_leg_targets(s, &chain, pelvis_pose, &mut angles, goal)?;
        for (target, angles) in result.iter_mut().zip(angles) {
            target.1 = angles;
        }
    }
    Ok(result)
}

fn refine_leg_targets(
    s: &Simulation,
    chain: &[usize; 4],
    root: Pose,
    angles: &mut [[f32; 3]; 4],
    goal: Vector,
) -> Result<(), String> {
    use rapier3d::na;
    // Keep the clamped ankle/sole coordinates. Hip pitch/roll and knee flexion
    // supply the three positional degrees of freedom of the two-link leg.
    let coordinates = [(0, 0), (0, 2), (1, 0)];
    for _ in 0..s.profile.reaching_iterations {
        let position = crate::reaching::endpoint(s, chain, root, angles, Vector::ZERO)?;
        let error = goal - position;
        if error.length() <= 1e-5 {
            break;
        }
        let current = *angles;
        let mut jacobian = na::Matrix3::<f32>::zeros();
        for (column, (joint, coordinate)) in coordinates.into_iter().enumerate() {
            let mut probe = current;
            probe[joint][coordinate] += 0.001;
            let derivative = (crate::reaching::endpoint(s, chain, root, &probe, Vector::ZERO)?
                - position)
                / 0.001;
            jacobian.set_column(
                column,
                &na::Vector3::new(derivative.x, derivative.y, derivative.z),
            );
        }
        let Some(inverse) = jacobian.try_inverse() else {
            break;
        };
        let delta = inverse * na::Vector3::new(error.x, error.y, error.z);
        if !delta.iter().all(|value| value.is_finite()) {
            break;
        }
        let delta = delta.map(|value| {
            value.clamp(
                -s.profile.reaching_coordinate_increment_rad,
                s.profile.reaching_coordinate_increment_rad,
            )
        });
        let mut admitted = false;
        for attempt in 0..6 {
            let fraction = 0.5_f32.powi(attempt);
            let mut candidate = current;
            for (column, (joint, coordinate)) in coordinates.into_iter().enumerate() {
                let axis = s.model.segments[chain[joint]]
                    .joint_profile
                    .as_ref()
                    .ok_or("Missing refinement joint")?
                    .axes
                    .iter()
                    .find(|axis| axis.coordinate == ['x', 'y', 'z'][coordinate])
                    .ok_or("Missing refinement coordinate")?;
                candidate[joint][coordinate] = (current[joint][coordinate]
                    + delta[column] * fraction)
                    .clamp(axis.min_radians, axis.max_radians);
            }
            let endpoint = crate::reaching::endpoint(s, chain, root, &candidate, Vector::ZERO)?;
            if endpoint.distance_squared(goal) < error.length_squared() {
                *angles = candidate;
                admitted = true;
                break;
            }
        }
        if !admitted {
            break;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn swing_diagnostic_separates_reach_installation_and_motion_without_body_writes() {
        for heading in [0.0, std::f32::consts::PI / 3.0] {
            let mut s = Simulation::new(heading, Profile::default(), true).expect("fixture");
            for _ in 0..24 {
                s.advance_tick().expect("loaded stance");
            }
            let mut controller = StepController::new(&s).expect("feet");
            assert!(controller.diagnose_swing(&s).expect("idle").is_none());
            let mut swing = request(&controller, &s);
            swing.target = swing.start + s.heading * Vector::Z * 0.10;
            swing.swing_s = Some(0.20);
            controller.swing = Some(swing);
            let intent = controller.update(&s).expect("swing");
            for (i, angles) in &intent.targets {
                s.targets[*i] = *angles;
            }
            let before = serde_json::to_vec(&s.snapshot()).expect("before");
            let diagnostic = controller
                .diagnose_swing(&s)
                .expect("diagnostic")
                .expect("sample");
            assert!((diagnostic["clockS"].as_f64().expect("clock") - 0.20).abs() < 1e-6);
            assert!(diagnostic["ikResidualM"].as_f64().expect("IK residual") < 1e-4);
            assert_eq!(
                diagnostic["installedEndpoint"],
                diagnostic["intendedEndpoint"]
            );
            assert_ne!(diagnostic["footBefore"], diagnostic["requested"]);
            assert_eq!(before, serde_json::to_vec(&s.snapshot()).expect("after"));
            let ankle = intent.targets[2].0;
            s.targets[ankle][0] += 0.03;
            let corrected = controller
                .diagnose_swing(&s)
                .expect("installed correction")
                .expect("sample");
            assert_eq!(
                corrected["intendedEndpoint"],
                diagnostic["intendedEndpoint"]
            );
            assert_ne!(
                corrected["installedEndpoint"],
                diagnostic["installedEndpoint"]
            );
            let swing = controller.swing.as_mut().expect("swing");
            swing.target = swing.start + s.heading * Vector::Z * 2.0;
            swing.swing_s = Some(0.45 + s.profile.dt_s);
            let before = serde_json::to_vec(&s.snapshot()).expect("before unreachable");
            let unreachable = controller
                .diagnose_swing(&s)
                .expect("unreachable")
                .expect("sample");
            assert_eq!(unreachable["progress"], 1.0);
            assert!(unreachable["reachExcessM"].as_f64().expect("excess") > 1.0);
            assert!(unreachable["ikResidualM"].as_f64().expect("residual") > 1.0);
            for axis in unreachable["axesBefore"].as_array().expect("axes") {
                let target = axis["intendedRad"].as_f64().expect("target");
                assert!(target >= axis["minRad"].as_f64().expect("minimum"));
                assert!(target <= axis["maxRad"].as_f64().expect("maximum"));
            }
            assert_eq!(
                before,
                serde_json::to_vec(&s.snapshot()).expect("after unreachable")
            );
            controller.swing.as_mut().expect("swing").swing_s = None;
            assert!(controller.diagnose_swing(&s).expect("transfer").is_none());
        }
    }
    #[test]
    fn transfer_intent_cannot_wind_up_outside_measured_tapered_support() {
        let polygon = [[-0.06, -0.15], [0.06, -0.15], [0.05, 0.15], [-0.05, 0.15]];
        for i in 0..512 {
            let angle = i as f32 * std::f32::consts::TAU / 512.0;
            let point = project_support(&polygon, [10.0 * angle.cos(), 10.0 * angle.sin()], 0.003)
                .expect("measured support");
            for (a, b) in polygon.iter().zip(polygon.iter().cycle().skip(1)) {
                let dx = b[0] - a[0];
                let dz = b[1] - a[1];
                let signed_distance =
                    (dx * (point[1] - a[1]) - dz * (point[0] - a[0])) / (dx * dx + dz * dz).sqrt();
                assert!(
                    signed_distance >= 0.003 - 1e-6,
                    "{point:?}: {signed_distance}"
                );
            }
        }
        assert!(project_support(&[], [0.0; 2], 0.003).is_none());
        assert!(project_support(&[[0.0; 2], [0.1, 0.0]], [0.0; 2], 0.003).is_none());
    }
    use crate::Profile;
    #[test]
    fn liftoff_rejects_a_raised_center_with_a_toe_still_on_the_floor() {
        assert!(!lifted(0.05, 0.0, 0.0, 0.004));
        assert!(!lifted(0.05, 0.0, 0.008, 0.004));
        assert!(!lifted(0.05, 3.0, 0.03, 0.004));
        assert!(lifted(0.05, 0.0, 0.03, 0.004));
    }
    #[test]
    fn tapered_support_rejects_the_empty_corners_of_its_bounding_rectangle() {
        let diamond = vec![[0.0, -1.0], [1.0, 0.0], [0.0, 1.0], [-1.0, 0.0]];
        assert!(inside_support(diamond.clone(), [0.0, 0.0], 0.6));
        assert!(!inside_support(diamond.clone(), [0.9, 0.9], 0.0));
        assert!(!inside_support(diamond, [0.5, 0.5], 0.01));
        assert!(!inside_support(
            vec![[0.0, 0.0], [0.5, 0.0], [1.0, 0.0]],
            [0.5, 0.0],
            0.0
        ));
        assert!(!inside_support(
            vec![[0.0, 0.0], [1.0, 0.0]],
            [0.5, 0.0],
            0.0
        ));
    }
    fn request(controller: &StepController, s: &Simulation) -> Swing {
        let actual = s.world.bodies[s.bodies[controller.feet[0]]].translation();
        Swing {
            side: 0,
            start: actual - Vector::X * 0.07,
            target: actual,
            elapsed: 1.0,
            ready_s: 0.05,
            swing_s: Some(0.45),
            airborne_s: 0.0,
            verified_liftoff: false,
            landed_s: 0.0,
            demand: Vector::X,
            transfer_bias: Vector::ZERO,
            qualified_reference: None,
            redirected: false,
            flat_started: false,
            unload_s: 0.0,
            following: false,
            follow_start: Pose::from_parts(
                s.world.bodies[s.bodies[0]].translation(),
                *s.world.bodies[s.bodies[0]].rotation(),
            ),
        }
    }
    #[test]
    fn touchdown_targets_bound_solver_overshoot_without_modifying_the_pose() {
        for heading in [0.0, std::f32::consts::PI / 3.0] {
            for upper in [false, true] {
                let mut s = Simulation::new(heading, Profile::default(), true).expect("fixture");
                for _ in 0..24 {
                    s.advance_tick().expect("loaded stance");
                }
                let legs: Vec<_> = s
                    .model
                    .segments
                    .iter()
                    .enumerate()
                    .filter(|(_, segment)| {
                        matches!(
                            segment.role.as_str(),
                            "thigh" | "shin" | "ankle" | "hindfoot"
                        )
                    })
                    .map(|(i, _)| i)
                    .collect();
                // Exercise real quaternion-derived coordinates at the solver's
                // allowed overshoot, including axes locked by the anatomy.
                for i in &legs {
                    let profile = s.model.segments[*i]
                        .joint_profile
                        .as_ref()
                        .expect("profile");
                    let joint = s
                        .world
                        .impulse_joints
                        .get(s.joints[*i].expect("joint"))
                        .expect("joint");
                    let coordinates: [f32; 3] = std::array::from_fn(|k| {
                        profile
                            .axes
                            .iter()
                            .find(|axis| axis.coordinate == ['x', 'y', 'z'][k])
                            .map_or(0.03, |axis| {
                                if upper {
                                    axis.max_radians + 0.03
                                } else {
                                    axis.min_radians - 0.03
                                }
                            })
                    });
                    let relative = Rotation::from_xyzw(
                        (coordinates[0] * 0.5).tan(),
                        (coordinates[1] * 0.5).tan(),
                        (coordinates[2] * 0.5).tan(),
                        1.0,
                    )
                    .normalize();
                    let child = joint.body2();
                    let rotation = *s.world.bodies[joint.body1()].rotation()
                        * crate::rotation(profile.parent_frame.rotation)
                        * relative
                        * crate::rotation(profile.child_frame.rotation).inverse();
                    s.world.bodies[child].set_rotation(rotation, true);
                }
                let before = serde_json::to_vec(&s.snapshot()).expect("before");
                let targets = measured_leg_targets(&s).expect("bounded targets");
                assert_eq!(targets.len(), 8);
                for (i, angles) in &targets {
                    let profile = s.model.segments[*i]
                        .joint_profile
                        .as_ref()
                        .expect("profile");
                    let joint = s
                        .world
                        .impulse_joints
                        .get(s.joints[*i].expect("joint"))
                        .expect("joint");
                    let measured = measurement::coordinates(
                        *s.world.bodies[joint.body1()].rotation(),
                        *s.world.bodies[joint.body2()].rotation(),
                        profile,
                    );
                    for (k, coordinate) in ['x', 'y', 'z'].into_iter().enumerate() {
                        if let Some(axis) = profile
                            .axes
                            .iter()
                            .find(|axis| axis.coordinate == coordinate)
                        {
                            let bound = if upper {
                                axis.max_radians
                            } else {
                                axis.min_radians
                            };
                            assert!((measured[k] - bound).abs() > 0.029);
                            assert_eq!(angles[k], bound);
                        } else {
                            assert!(measured[k].abs() > 0.029);
                            assert_eq!(angles[k], 0.0);
                        }
                    }
                }
                let mut controller = StepController::new(&s).expect("feet");
                let mut swing = request(&controller, &s);
                swing.verified_liftoff = true;
                swing.landed_s = 0.099;
                controller.swing = Some(swing);
                assert!(controller.update(&s).expect("landing intent").landed);
                for (i, angles) in targets {
                    assert_eq!(controller.settle_from[i], angles);
                }
                assert_eq!(before, serde_json::to_vec(&s.snapshot()).expect("after"));
            }
        }
    }
    #[test]
    fn an_aborted_transfer_preserves_the_existing_stance_policy() {
        for heading in [0.0, std::f32::consts::PI / 3.0] {
            let mut s = Simulation::new(heading, Profile::default(), true).expect("fixture");
            for _ in 0..24 {
                s.advance_tick().expect("loaded stance");
            }
            let mut controller = StepController::new(&s).expect("feet");
            let mut swing = request(&controller, &s);
            swing.elapsed = 2.01;
            swing.swing_s = None;
            swing.ready_s = 0.0;
            controller.swing = Some(swing);
            let planted = controller.planted;
            let before = serde_json::to_vec(&s.snapshot()).expect("before");
            let intent = controller.update(&s).expect("abort");
            assert!(!intent.landed);
            assert_eq!(s.counters.steps, 0);
            assert_eq!(controller.status.phase, "aborted");
            assert_eq!(controller.status.aborted, 1);
            assert!(controller.swing.is_none());
            assert!(!controller.repositioned);
            assert_eq!(controller.planted, planted);
            assert_eq!(controller.last_step_demand, Vector::ZERO);
            assert!(intent.targets.is_empty());
            assert!(
                controller
                    .update(&s)
                    .expect("existing policy")
                    .targets
                    .is_empty()
            );
            assert_eq!(before, serde_json::to_vec(&s.snapshot()).expect("after"));
        }
    }
    #[test]
    fn a_late_admitted_swing_gets_its_own_bounded_clock_without_counting_a_slide() {
        let mut s = Simulation::new(0.0, Profile::default(), true).expect("fixture");
        s.advance_tick().expect("initial contact");
        let mut controller = StepController::new(&s).expect("feet");
        let mut swing = request(&controller, &s);
        swing.elapsed = 1.99;
        swing.swing_s = Some(0.01);
        controller.swing = Some(swing);
        let before = serde_json::to_vec(&s.snapshot().segments).expect("before");
        for _ in 0..150 {
            assert!(!controller.update(&s).expect("intent").landed);
            assert_eq!(controller.status.phase, "swing");
            assert!(!controller.status.verified_liftoff);
        }
        assert!(controller.status.elapsed_s > 2.0);
        for _ in 0..400 {
            assert!(!controller.update(&s).expect("bounded intent").landed);
            if controller.status.phase == "aborted" {
                break;
            }
        }
        assert_eq!(controller.status.phase, "aborted");
        assert_eq!(controller.status.aborted, 1);
        assert_eq!(
            before,
            serde_json::to_vec(&s.snapshot().segments).expect("after")
        );
    }
    #[test]
    fn a_loaded_slide_cannot_count_as_a_step_and_floor_loss_cancels_the_plan() {
        let mut s = Simulation::new(0.0, Profile::default(), true).expect("fixture");
        s.advance_tick().expect("initial contact");
        let mut controller = StepController::new(&s).expect("feet");
        controller.swing = Some(request(&controller, &s));
        let before = serde_json::to_vec(&s.snapshot().segments).expect("before");
        for _ in 0..30 {
            assert!(!controller.update(&s).expect("intent").landed);
            assert!(!controller.status.verified_liftoff);
            assert_eq!(controller.status.touchdown_s, 0.0);
        }
        assert_eq!(
            before,
            serde_json::to_vec(&s.snapshot().segments).expect("after")
        );
        s.set_floor_enabled(false).expect("floor off");
        let intent = controller.update(&s).expect("unsupported");
        assert!(intent.targets.is_empty());
        assert!(controller.swing.is_none());
        assert_eq!(controller.status.phase, "unsupported");
    }
    #[test]
    fn swing_velocity_tracks_the_virtual_foot_path_and_clears_on_support_loss() {
        for heading in [0.0, std::f32::consts::PI / 3.0] {
            for time in [0.0_f32, 0.20, 0.45] {
                let mut s = Simulation::new(heading, Profile::default(), true).expect("fixture");
                for _ in 0..24 {
                    s.advance_tick().expect("loaded stance");
                }
                let mut controller = StepController::new(&s).expect("feet");
                let mut swing = request(&controller, &s);
                swing.start = controller.planted[0];
                swing.target = swing.start + s.heading * Vector::Z * 0.10;
                swing.swing_s = Some(time);
                let start = swing.start;
                controller.swing = Some(swing);
                let before = serde_json::to_vec(&s.snapshot().segments).expect("before");
                let mut copy = controller.clone();
                let intent = copy.update(&s).expect("swing intent");
                assert!(!intent.landed);
                assert_eq!(intent.targets.len(), 4);
                assert_eq!(intent.velocities.len(), 4);
                let chain: Vec<_> = intent.targets.iter().map(|(i, _)| *i).collect();
                let predicted: Vec<_> = intent
                    .targets
                    .iter()
                    .zip(&intent.velocities)
                    .map(|((i, angles), (j, velocity))| {
                        assert_eq!(i, j);
                        assert!(velocity.iter().all(|v| v.is_finite()));
                        if time == 0.45 {
                            assert_eq!(*velocity, [0.0; 3]);
                        }
                        std::array::from_fn(|k| angles[k] + velocity[k] * s.profile.dt_s)
                    })
                    .collect();
                let root = *s.world.bodies[s.bodies[0]].position();
                let endpoint =
                    crate::reaching::endpoint(&s, &chain, root, &predicted, Vector::ZERO)
                        .expect("velocity-predicted FK");
                let progress = ((time + s.profile.dt_s) / 0.45).min(1.0);
                let mut expected = start
                    + s.heading * Vector::Z * (0.10 * progress * progress * (3.0 - 2.0 * progress));
                expected.y += 0.04 * (std::f32::consts::PI * progress).sin();
                assert!(endpoint.distance(expected) < 1e-4);
                s.stepping = Some(controller);
                s.configure_motors().expect("native motor installation");
                for (i, velocity) in intent.velocities {
                    if s.model.segments[i].role == "thigh" {
                        continue; // Its existing pelvis torque adds a separate feedforward rate.
                    }
                    let joint = s
                        .world
                        .impulse_joints
                        .get(s.joints[i].expect("joint"))
                        .expect("joint");
                    let profile = s.model.segments[i].joint_profile.as_ref().expect("profile");
                    for axis in &profile.axes {
                        let k = ['x', 'y', 'z']
                            .iter()
                            .position(|c| *c == axis.coordinate)
                            .expect("coordinate");
                        let motor = joint
                            .data
                            .motor(crate::axis(axis.coordinate).expect("axis"))
                            .expect("motor");
                        assert!((motor.target_vel - velocity[k]).abs() < 1e-6);
                        assert!(
                            motor.max_force > 0.0 && motor.max_force <= axis.max_motor_torque_nm
                        );
                    }
                }
                assert_eq!(s.counters.steps, 0);
                s.set_floor_enabled(false).expect("floor off");
                s.configure_motors().expect("unsupported motors");
                for i in chain {
                    let joint = s
                        .world
                        .impulse_joints
                        .get(s.joints[i].expect("joint"))
                        .expect("joint");
                    for axis in &s.model.segments[i]
                        .joint_profile
                        .as_ref()
                        .expect("profile")
                        .axes
                    {
                        assert_eq!(
                            joint
                                .data
                                .motor(crate::axis(axis.coordinate).expect("axis"))
                                .expect("motor")
                                .target_vel,
                            0.0
                        );
                    }
                }
                assert_eq!(
                    before,
                    serde_json::to_vec(&s.snapshot().segments).expect("after")
                );
            }
        }
    }
    #[test]
    fn transfer_reserve_requires_landing_persistent_load_and_uncommitted_swing() {
        let mut s = Simulation::new(0.0, Profile::default(), true).expect("fixture");
        for _ in 0..24 {
            s.advance_tick().expect("loaded stance");
        }
        let before = serde_json::to_vec(&s.snapshot().segments).expect("before");
        let mut controller = StepController::new(&s).expect("feet");
        let mut swing = request(&controller, &s);
        swing.swing_s = None;
        swing.elapsed = 0.6;
        controller.swing = Some(swing);
        controller.update(&s).expect("first transfer");
        assert!(controller.transfer_load_share(&s).is_none());
        controller.repositioned = true;
        // An explicitly queued continuation persists after pointer release.
        controller.swing.as_mut().expect("transfer").following = true;
        controller.cooldown_s = 1.0;
        controller.update(&s).expect("later transfer");
        let (shares, strength) = controller
            .transfer_load_share(&s)
            .expect("measured reserve");
        assert_eq!(strength, 1.0);
        assert!(shares.iter().all(|share| *share > 0.0 && *share < 1.0));
        assert!((shares.iter().sum::<f32>() - 1.0).abs() < 1e-6);
        let contacts = s.contacts.clone();
        for contact in &mut s.contacts {
            if s.model.segments[contact.segment as usize]
                .id
                .starts_with("right")
            {
                contact.persistence_s = 0.049;
            }
        }
        assert_eq!(
            controller
                .transfer_load_share(&s)
                .expect("persistent left support")
                .0,
            [1.0, 0.0]
        );
        for contact in &mut s.contacts {
            contact.normal_load_n = 0.0;
        }
        assert!(controller.transfer_load_share(&s).is_none());
        s.contacts = contacts;
        controller.swing.as_mut().expect("transfer").swing_s = Some(0.1);
        assert!(controller.transfer_load_share(&s).is_none());
        assert_eq!(controller.support_reserve(), Some((1, 1.0)));
        s.set_floor_enabled(false).expect("floor off");
        let unsupported = controller.update(&s).expect("unsupported");
        assert!(unsupported.targets.is_empty() && unsupported.velocities.is_empty());
        assert!(controller.transfer_load_share(&s).is_none());
        assert!(controller.support_reserve().is_none());
        assert_eq!(s.counters.steps, 0);
        assert_eq!(
            before,
            serde_json::to_vec(&s.snapshot().segments).expect("after")
        );
    }
    #[test]
    fn release_cancels_only_a_safe_uncommitted_transfer_without_counting_a_step() {
        for (outside, following, admitted, keep) in [
            (false, false, false, false),
            (true, false, false, true),
            (false, true, false, true),
            (false, false, true, true),
        ] {
            let mut s = Simulation::new(0.0, Profile::default(), true).expect("fixture");
            for _ in 0..24 {
                s.advance_tick().expect("loaded stance");
            }
            if outside {
                // Keep the measured sole contacts but move the actual capture
                // point beyond their hull through physical body velocities.
                for handle in &s.bodies {
                    s.world.bodies[*handle].set_linvel(Vector::X, true);
                }
            }
            let mut controller = StepController::new(&s).expect("feet");
            controller.repositioned = true;
            controller.cooldown_s = 1.0;
            let mut swing = request(&controller, &s);
            swing.swing_s = admitted.then_some(0.1);
            swing.elapsed = 0.2;
            swing.following = following;
            controller.swing = Some(swing);
            let before = serde_json::to_vec(&s.snapshot().segments).expect("before");
            let intent = controller.update(&s).expect("released transfer");
            assert_eq!(controller.swing.is_some(), keep);
            assert!(!intent.landed);
            assert_eq!(s.counters.steps, 0);
            assert_eq!(
                before,
                serde_json::to_vec(&s.snapshot().segments).expect("after")
            );
        }
    }
    #[test]
    fn stance_velocity_tracks_the_complete_blend_and_clears_without_support() {
        for heading in [0.0, std::f32::consts::PI / 3.0] {
            for time in [0.0, 0.2, 0.6] {
                let mut s = Simulation::new(heading, Profile::default(), true).expect("fixture");
                for _ in 0..24 {
                    s.advance_tick().expect("loaded stance");
                }
                let mut controller = StepController::new(&s).expect("feet");
                controller.repositioned = true;
                controller.cooldown_s = 1.0;
                controller.settle_s = time;
                controller.settle_reference = s.center_of_mass() + s.heading * Vector::X * 0.03;
                for (i, angles) in measured_leg_targets(&s).expect("measured stance") {
                    controller.settle_from[i] = angles;
                }
                let before = serde_json::to_vec(&s.snapshot().segments).expect("before");
                let mut probe = controller.clone();
                let current = probe.update(&s).expect("current blend");
                let future = probe.update(&s).expect("next blend");
                assert_eq!(current.targets.len(), 8);
                assert_eq!(current.velocities.len(), 8);
                for (((i, angles), (j, velocity)), (next_i, next_angles)) in current
                    .targets
                    .iter()
                    .zip(&current.velocities)
                    .zip(&future.targets)
                {
                    assert_eq!(i, j);
                    assert_eq!(i, next_i);
                    let profile = s.model.segments[*i]
                        .joint_profile
                        .as_ref()
                        .expect("profile");
                    for k in 0..3 {
                        assert!(velocity[k].is_finite());
                        assert!(
                            (angles[k] + velocity[k] * s.profile.dt_s - next_angles[k]).abs()
                                < 1e-6
                        );
                        if let Some(axis) = profile
                            .axes
                            .iter()
                            .find(|axis| axis.coordinate == ['x', 'y', 'z'][k])
                        {
                            assert!(angles[k] >= axis.min_radians && angles[k] <= axis.max_radians);
                        } else {
                            assert_eq!(angles[k], 0.0);
                            assert_eq!(velocity[k], 0.0);
                        }
                        if time == 0.6 {
                            assert_eq!(velocity[k], 0.0);
                        }
                    }
                }
                s.stepping = Some(controller);
                s.configure_motors().expect("native blend motors");
                for (i, velocity) in &current.velocities {
                    if !matches!(s.model.segments[*i].role.as_str(), "shin" | "ankle") {
                        continue;
                    }
                    let joint = s
                        .world
                        .impulse_joints
                        .get(s.joints[*i].expect("joint"))
                        .expect("joint");
                    let motor = joint
                        .data
                        .motor(crate::axis('x').expect("axis"))
                        .expect("motor");
                    assert!((motor.target_vel - velocity[0]).abs() < 1e-6);
                }
                s.set_floor_enabled(false).expect("floor off");
                s.configure_motors().expect("unsupported motors");
                assert_eq!(
                    s.stepping.as_ref().expect("controller").support_reserve(),
                    None
                );
                for (i, _) in &current.velocities {
                    let joint = s
                        .world
                        .impulse_joints
                        .get(s.joints[*i].expect("joint"))
                        .expect("joint");
                    for axis in &s.model.segments[*i]
                        .joint_profile
                        .as_ref()
                        .expect("profile")
                        .axes
                    {
                        assert_eq!(
                            joint
                                .data
                                .motor(crate::axis(axis.coordinate).expect("axis"))
                                .expect("motor")
                                .target_vel,
                            0.0
                        );
                    }
                }
                assert_eq!(s.counters.steps, 0);
                assert_eq!(
                    before,
                    serde_json::to_vec(&s.snapshot().segments).expect("after")
                );
            }
        }
    }
    #[test]
    fn verified_landing_keeps_native_caps_bounded_and_new_swing_takes_priority() {
        fn motors(s: &Simulation) -> Vec<[f32; 3]> {
            let scale = s
                .profile
                .motor_scale
                .min(s.profile.aggregate_motor_cap_nm / s.motor_ceiling_sum);
            let mut result = Vec::new();
            let mut total = 0.0;
            for (i, handle) in s.joints.iter().enumerate() {
                let (Some(handle), Some(profile)) = (handle, &s.model.segments[i].joint_profile)
                else {
                    continue;
                };
                let joint = s.world.impulse_joints.get(*handle).expect("joint");
                for axis in &profile.axes {
                    let motor = joint
                        .data
                        .motor(crate::axis(axis.coordinate).expect("axis"))
                        .expect("motor");
                    assert!(motor.max_force > 0.0 && motor.max_force <= axis.max_motor_torque_nm);
                    assert!(
                        (motor.stiffness
                            - axis.passive_stiffness_nm_per_rad
                                * s.profile.posture_stiffness_multiplier
                                * scale)
                            .abs()
                            < 1e-3
                    );
                    assert!(
                        (motor.damping
                            - axis.damping_nms_per_rad
                                * s.profile.posture_damping_multiplier
                                * scale)
                            .abs()
                            < 1e-3
                    );
                    total += motor.max_force;
                    result.push([motor.max_force, motor.stiffness, motor.damping]);
                }
            }
            assert!(total <= s.profile.aggregate_motor_cap_nm + 1e-3);
            result
        }
        let mut s = Simulation::new(0.0, Profile::default(), true).expect("fixture");
        for _ in 0..24 {
            s.advance_tick().expect("loaded stance");
        }
        s.configure_motors().expect("ordinary allocation");
        let ordinary = motors(&s);
        let before = serde_json::to_vec(&s.snapshot().segments).expect("before");
        let mut controller = StepController::new(&s).expect("feet");
        let mut swing = request(&controller, &s);
        swing.verified_liftoff = true;
        swing.landed_s = 0.099;
        controller.swing = Some(swing);
        s.stepping = Some(controller);
        s.configure_motors().expect("verified landing");
        assert_eq!(s.counters.steps, 1);
        assert_eq!(
            s.stepping.as_ref().expect("controller").support_reserve(),
            Some((1, 1.0))
        );
        let landing = motors(&s);
        assert_ne!(landing, ordinary);
        for time in [0.15, 0.3, 0.45, 0.6] {
            let controller = s.stepping.as_mut().expect("controller");
            controller.cooldown_s = 1.0;
            controller.settle_s = time - s.profile.dt_s;
            s.configure_motors().expect("settling allocation");
            let current = motors(&s);
            for (current, ordinary) in current.iter().zip(&ordinary) {
                // Active-stance caps follow measured native PD demand; the
                // reserve API still fades and gains/hard ceilings stay fixed.
                assert_eq!(current[1..], ordinary[1..]);
            }
            if time == 0.3 {
                assert_eq!(
                    s.stepping.as_ref().expect("controller").support_reserve(),
                    Some((1, 0.5))
                );
            } else if time == 0.6 {
                assert_eq!(
                    s.stepping.as_ref().expect("controller").support_reserve(),
                    None
                );
            }
        }
        let mut controller = s.stepping.take().expect("controller");
        controller.settle_s = 0.1 - s.profile.dt_s;
        let mut transfer = request(&controller, &s);
        transfer.side = 1;
        transfer.start = controller.planted[1];
        transfer.target = controller.planted[1];
        transfer.swing_s = None;
        transfer.following = true;
        transfer.elapsed = 0.6;
        controller.swing = Some(transfer);
        s.stepping = Some(controller);
        s.configure_motors().expect("opposite transfer");
        motors(&s);
        let controller = s.stepping.as_ref().expect("controller");
        assert_eq!(controller.status.phase, "transfer");
        assert_eq!(
            controller
                .transfer_load_share(&s)
                .expect("loaded transfer")
                .1,
            1.0
        );
        assert_eq!(controller.support_reserve().expect("fading reserve").0, 1);
        s.stepping
            .as_mut()
            .expect("controller")
            .swing
            .as_mut()
            .expect("transfer")
            .swing_s = Some(0.1);
        s.configure_motors().expect("opposite swing");
        assert_eq!(
            s.stepping.as_ref().expect("controller").support_reserve(),
            Some((0, 1.0))
        );
        assert_ne!(motors(&s), landing);
        s.set_floor_enabled(false).expect("floor off");
        s.configure_motors().expect("unsupported allocation");
        assert_eq!(
            s.stepping.as_ref().expect("controller").support_reserve(),
            None
        );
        assert_eq!(motors(&s), ordinary);
        assert_eq!(s.counters.steps, 1);
        assert_eq!(
            before,
            serde_json::to_vec(&s.snapshot().segments).expect("after")
        );
        assert_eq!(
            StepController::new(&s)
                .expect("fresh controller")
                .support_reserve(),
            None
        );
    }
    #[test]
    fn swing_reserves_preserve_native_gains_and_each_hard_effort_ceiling() {
        let mut s = Simulation::new(0.0, Profile::default(), true).expect("fixture");
        s.advance_tick().expect("initial contact");
        let mut controller = StepController::new(&s).expect("feet");
        let mut swing = request(&controller, &s);
        swing.swing_s = Some(0.1);
        controller.swing = Some(swing);
        s.stepping = Some(controller);
        let before = serde_json::to_vec(&s.snapshot().segments).expect("before");
        s.configure_motors().expect("configure");
        let base_scale = s
            .profile
            .motor_scale
            .min(s.profile.aggregate_motor_cap_nm / s.motor_ceiling_sum);
        let mut sum = 0.0;
        for (i, handle) in s.joints.iter().enumerate() {
            let (Some(handle), Some(profile)) = (handle, &s.model.segments[i].joint_profile) else {
                continue;
            };
            let joint = s.world.impulse_joints.get(*handle).expect("joint");
            for a in &profile.axes {
                let motor = joint
                    .data
                    .motor(crate::axis(a.coordinate).expect("axis"))
                    .expect("motor");
                assert!(motor.max_force > 0.0 && motor.max_force <= a.max_motor_torque_nm);
                assert!(
                    (motor.stiffness
                        - a.passive_stiffness_nm_per_rad
                            * s.profile.posture_stiffness_multiplier
                            * base_scale)
                        .abs()
                        < 1e-3
                );
                assert!(
                    (motor.damping
                        - a.damping_nms_per_rad
                            * s.profile.posture_damping_multiplier
                            * base_scale)
                        .abs()
                        < 1e-3
                );
                sum += motor.max_force;
            }
        }
        assert!(sum <= s.profile.aggregate_motor_cap_nm + 1e-3, "sum {sum}");
        assert_eq!(
            before,
            serde_json::to_vec(&s.snapshot().segments).expect("after")
        );
    }
    #[test]
    fn leg_inverse_matches_canonical_geometry_at_each_heading_without_pose_writes() {
        for heading in [0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0] {
            let s = Simulation::new(heading, Profile::default(), true).expect("fixture");
            let controller = StepController::new(&s).expect("feet");
            let before = serde_json::to_vec(&s.snapshot()).expect("before");
            for side in 0..2 {
                for (i, angles) in
                    leg_targets(&s, controller.feet[side], controller.planted[side]).expect("IK")
                {
                    for (k, value) in angles.into_iter().enumerate() {
                        assert!(
                            (value - s.rest_targets[i][k]).abs() < 2e-5,
                            "canonical angle {} axis {k}: {value}",
                            s.model.segments[i].id
                        );
                    }
                }
            }
            assert_eq!(before, serde_json::to_vec(&s.snapshot()).expect("after"));
        }
    }

    #[test]
    fn leg_inverse_reaches_shifted_stance_and_swing_goals_in_forward_kinematics() {
        for heading in [0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0] {
            let s = Simulation::new(heading, Profile::default(), true).expect("fixture");
            let controller = StepController::new(&s).expect("feet");
            let before = serde_json::to_vec(&s.snapshot()).expect("before");
            for offset in [
                Vector::new(0.05, -0.03, 0.0),
                Vector::new(-0.05, -0.03, 0.03),
                Vector::new(0.0, -0.03, -0.03),
            ] {
                let root = Pose::from_parts(
                    s.world.bodies[s.bodies[0]].translation() + s.heading * offset,
                    s.heading,
                );
                for side in 0..2 {
                    for travel in [Vector::ZERO, Vector::new(0.0, 0.03, 0.04)] {
                        let goal = controller.planted[side] + s.heading * travel;
                        let mut pose = root;
                        for (i, angles) in
                            leg_targets_with_root(&s, controller.feet[side], goal, root)
                                .expect("IK")
                        {
                            let profile =
                                s.model.segments[i].joint_profile.as_ref().expect("joint");
                            for (coordinate, value) in ['x', 'y', 'z'].into_iter().zip(angles) {
                                if let Some(axis) = profile
                                    .axes
                                    .iter()
                                    .find(|axis| axis.coordinate == coordinate)
                                {
                                    assert!(value >= axis.min_radians && value <= axis.max_radians);
                                } else {
                                    assert_eq!(value, 0.0);
                                }
                            }
                            let relative = Rotation::from_xyzw(
                                (angles[0] * 0.5).tan(),
                                (angles[1] * 0.5).tan(),
                                (angles[2] * 0.5).tan(),
                                1.0,
                            )
                            .normalize();
                            let rotation = pose.rotation
                                * crate::rotation(profile.parent_frame.rotation)
                                * relative
                                * crate::rotation(profile.child_frame.rotation).inverse();
                            pose = Pose::from_parts(
                                pose.transform_point(vector(profile.parent_frame.anchor))
                                    - rotation * vector(profile.child_frame.anchor),
                                rotation,
                            );
                        }
                        let error = pose.translation.distance(goal);
                        assert!(
                            error < 1e-4,
                            "heading {heading}, side {side}, offset {offset:?}, travel {travel:?}: FK error {error}"
                        );
                    }
                }
            }
            assert_eq!(before, serde_json::to_vec(&s.snapshot()).expect("after"));
        }
    }
}
