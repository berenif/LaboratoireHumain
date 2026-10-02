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
}
#[derive(Clone)]
pub(crate) struct StepController {
    feet: [usize; 2],
    planted: [Vector; 2],
    initial_midpoint: Vector,
    initial_pelvis_height: f32,
    swing: Option<Swing>,
    next_side: usize,
    repositioned: bool,
    cooldown_s: f32,
    press_start: Option<(u64, Vector)>,
    last_step_demand: Vector,
    pub status: Status,
}
pub(crate) struct Intent {
    pub reference: Vector,
    pub targets: Vec<(usize, [f32; 3])>,
    pub landed: bool,
}
impl StepController {
    pub fn retained_side(&self) -> Option<usize> {
        self.swing
            .as_ref()
            .filter(|swing| swing.swing_s.is_some())
            .map(|swing| 1 - swing.side)
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
            swing: None,
            next_side: 0,
            repositioned: false,
            cooldown_s: 0.0,
            press_start: None,
            last_step_demand: Vector::ZERO,
            status: Status {
                phase: "idle",
                ..Status::default()
            },
        })
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
        }
        let mut reference =
            s.rest_com + (self.planted[0] + self.planted[1]) * 0.5 - self.initial_midpoint;
        let mut targets = Vec::new();
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
            for side in 0..2 {
                targets.extend(leg_targets_at(
                    s,
                    self.feet[side],
                    self.planted[side],
                    Some(self.initial_pelvis_height),
                )?);
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
        if self.swing.is_none() && self.cooldown_s == 0.0 {
            let points = support[0].iter().chain(&support[1]).copied().collect();
            let outside = !inside_support(points, [capture.x, capture.z], 0.015);
            let changed_direction = self.last_step_demand.length_squared() == 0.0
                || demand.dot(self.last_step_demand) < 0.0;
            let requested = changed_direction
                && (demand - self.last_step_demand).length()
                    >= if dragged_side.is_some() { 0.075 } else { 0.16 };
            if (outside || requested) && load.iter().all(|n| *n >= 3.0) {
                let midpoint = s.heading.inverse() * ((self.planted[0] + self.planted[1]) * 0.5);
                let direction = s.heading.inverse() * demand;
                let side = if let Some(side) = dragged_side {
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
                    target = start + demand.normalize() * 0.07;
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
                });
            }
        }
        if let Some(swing) = &mut self.swing {
            swing.elapsed += dt;
            let retained = 1 - swing.side;
            let transfer = (swing.elapsed / 0.6).min(1.0);
            let retained_reference = s.rest_com + self.planted[retained] - self.initial_midpoint;
            reference = reference.lerp(retained_reference, transfer);
            if swing.swing_s.is_none() {
                let mut error = retained_reference - s.heading * capture;
                error.y = 0.0;
                swing.transfer_bias += error * (2.0 * dt);
                if swing.transfer_bias.length() > 0.10 {
                    swing.transfer_bias *= 0.10 / swing.transfer_bias.length();
                }
            }
            reference += swing.transfer_bias;
            let retained_capture_inside =
                inside_support(support[retained].clone(), [capture.x, capture.z], 0.003);
            if swing.swing_s.is_none() {
                let weight = s.total_mass() * 9.81;
                if retained_capture_inside
                    && load[retained] >= 0.65 * weight
                    && load[swing.side] <= 0.35 * weight
                {
                    swing.ready_s += dt;
                } else {
                    swing.ready_s = 0.0;
                }
                if swing.ready_s >= 0.05 {
                    swing.swing_s = Some(0.0);
                }
            }
            let progress = swing.swing_s.map_or(0.0, |time| (time / 0.45).min(1.0));
            if let Some(time) = &mut swing.swing_s {
                *time += dt;
                let smooth = progress * progress * (3.0 - 2.0 * progress);
                let mut goal = swing.start.lerp(swing.target, smooth);
                goal.y += 0.04 * (std::f32::consts::PI * progress).sin();
                targets.extend(leg_targets(s, self.feet[swing.side], goal)?);
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
                && actual.distance(swing.target) < 0.10
            {
                swing.landed_s += dt;
            } else {
                swing.landed_s = 0.0;
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
                self.planted[swing.side] = actual;
                self.next_side = retained;
                self.repositioned = true;
                self.last_step_demand = swing.demand;
                self.swing = None;
                self.cooldown_s = 0.15;
                landed = true;
                self.status.phase = "landed";
            } else if swing.elapsed > 2.0 {
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
            landed,
        })
    }
}

fn lifted(raise: f32, load: f32, clearance: f32, skin: f32) -> bool {
    raise >= 0.02 && load < 3.0 && clearance > 2.0 * skin + 0.002
}

// Monotone-chain hull of the exact loaded sole vertices in heading coordinates.
// A bounding rectangle would incorrectly admit points outside a tapered sole.
fn inside_support(mut points: Vec<[f32; 2]>, point: [f32; 2], margin: f32) -> bool {
    points.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    points.dedup();
    if points.len() < 3 {
        return false;
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
    if hull.len() < 3 {
        return false;
    }
    hull.iter().zip(hull.iter().cycle().skip(1)).all(|(a, b)| {
        let length = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
        length > 0.0 && cross(*a, *b, point) >= margin * length
    })
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
    leg_targets_in_frame(s, foot, goal, pelvis_height, None)
}

pub(crate) fn leg_targets_with_root(
    s: &Simulation,
    foot: usize,
    goal: Vector,
    pelvis: Pose,
) -> Result<Vec<(usize, [f32; 3])>, String> {
    leg_targets_in_frame(s, foot, goal, None, Some(pelvis))
}

fn leg_targets_in_frame(
    s: &Simulation,
    foot: usize,
    goal: Vector,
    pelvis_height: Option<f32>,
    root: Option<Pose>,
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
    let pivot = goal
        + s.heading
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
        plane_rotation,
        s.heading,
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
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
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
        }
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
}
