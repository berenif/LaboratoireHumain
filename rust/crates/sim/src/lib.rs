//! Dynamic f32 simulation with read-only substep inspection and no DOM/GPU dependencies.
#[cfg(not(target_arch = "wasm32"))]
pub mod articulation;
pub mod calibration;
pub mod environment;
pub mod grab;
pub mod measurement;
mod motion;
mod passive;
pub mod profiling;
mod quiet;
mod reaching;
pub mod recovery;
pub mod runtime;
pub mod scenarios;
mod stepping;
pub mod striker;
use lh_contracts::{Contact, Counters, Diagnostics, Motion, SUBSTEP_S, Snapshot, Stamp};
use lh_model::{Model, Quat, Vec3, integrate_mass};
use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub fn vector(v: Vec3) -> Vector {
    Vector::new(v.x, v.y, v.z)
}
pub fn rotation(q: Quat) -> Rotation {
    Rotation::from_xyzw(q.x, q.y, q.z, q.w)
}
pub fn vec3(v: Vector) -> Vec3 {
    Vec3 {
        x: v.x,
        y: v.y,
        z: v.z,
    }
}
pub fn quat(q: Rotation) -> Quat {
    Quat {
        x: q.x,
        y: q.y,
        z: q.z,
        w: q.w,
    }
}
pub fn axis(c: char) -> Result<JointAxis, String> {
    match c {
        'x' => Ok(JointAxis::AngX),
        'y' => Ok(JointAxis::AngY),
        'z' => Ok(JointAxis::AngZ),
        _ => Err("Invalid axis".into()),
    }
}
pub fn anatomical_joint(p: &lh_model::Joint) -> Result<GenericJoint, String> {
    let mut locked = JointAxesMask::LIN_AXES;
    for (c, mask) in [
        ('x', JointAxesMask::ANG_X),
        ('y', JointAxesMask::ANG_Y),
        ('z', JointAxesMask::ANG_Z),
    ] {
        if !p.axes.iter().any(|a| a.coordinate == c) {
            locked |= mask;
        }
    }
    let mut joint = GenericJointBuilder::new(locked)
        .local_frame1(Pose::from_parts(
            vector(p.parent_frame.anchor),
            rotation(p.parent_frame.rotation),
        ))
        .local_frame2(Pose::from_parts(
            vector(p.child_frame.anchor),
            rotation(p.child_frame.rotation),
        ))
        .contacts_enabled(false)
        .build();
    for a in &p.axes {
        joint.set_limits(axis(a.coordinate)?, [a.min_radians, a.max_radians]);
    }
    Ok(joint)
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum ContactFriction {
    #[default]
    Simplified,
    Coulomb,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Profile {
    pub id: String,
    pub dt_s: f32,
    pub tick_substeps: u32,
    #[serde(default)]
    pub contact_friction_model: ContactFriction,
    pub solver_iterations: usize,
    pub internal_pgs_iterations: usize,
    #[serde(default = "default_fall_iterations")]
    pub fall_internal_pgs_iterations: usize,
    pub additional_body_iterations: usize,
    pub allowed_linear_error_m: f32,
    pub prediction_distance_m: f32,
    pub motor_scale: f32,
    pub aggregate_motor_cap_nm: f32,
    pub angular_damping: f32,
    pub contact_skin_m: f32,
    pub max_ccd_substeps: usize,
    /// Rapier's default 400 m/s clamp changes momentum without a physical force.
    /// Disable it explicitly; acceptance still checks the actual integrated speed.
    pub disable_solver_speed_cap: bool,
    pub posture_stiffness_multiplier: f32,
    pub posture_damping_multiplier: f32,
    pub supported_tilt_gain: f32,
    pub supported_com_gain_rad_per_m: f32,
    pub supported_com_velocity_gain_s_rad_per_m: f32,
    #[serde(default)]
    pub supported_angular_velocity_gain_s: f32,
    pub grab_limits: grab::Limits,
    pub reaching_iterations: u32,
    pub reaching_coordinate_increment_rad: f32,
    pub reaching_damping_m2: f32,
    #[serde(default)]
    pub corrective_steps_enabled: bool,
    #[serde(default)]
    pub recovery_enabled: bool,
}
impl Default for Profile {
    fn default() -> Self {
        Self {
            id: "rust-physics-v24-com-restoration".into(),
            dt_s: SUBSTEP_S,
            tick_substeps: 4,
            contact_friction_model: ContactFriction::Coulomb,
            solver_iterations: 20,
            internal_pgs_iterations: 20,
            fall_internal_pgs_iterations: default_fall_iterations(),
            additional_body_iterations: 5,
            allowed_linear_error_m: 0.001,
            prediction_distance_m: 0.002,
            motor_scale: 1.0,
            aggregate_motor_cap_nm: 760.0,
            angular_damping: 0.52,
            contact_skin_m: 0.004,
            max_ccd_substeps: 4,
            disable_solver_speed_cap: true,
            posture_stiffness_multiplier: 20.0,
            posture_damping_multiplier: 40.0,
            supported_tilt_gain: 2.0,
            supported_com_gain_rad_per_m: 5.0,
            supported_com_velocity_gain_s_rad_per_m: 4.0,
            supported_angular_velocity_gain_s: 2.0,
            grab_limits: grab::Limits::default(),
            reaching_iterations: 6,
            reaching_coordinate_increment_rad: 0.15,
            reaching_damping_m2: 0.01,
            corrective_steps_enabled: false,
            recovery_enabled: false,
        }
    }
}
impl Profile {
    pub fn validate(&self) -> Result<(), String> {
        if self.dt_s != SUBSTEP_S
            || self.tick_substeps != 4
            || self.solver_iterations == 0
            || self.internal_pgs_iterations == 0
            || self.fall_internal_pgs_iterations == 0
            || self.max_ccd_substeps == 0
            || self.reaching_iterations > 12
            || (self.reaching_iterations > 0 && self.reaching_damping_m2 <= 0.0)
            || ![
                self.allowed_linear_error_m,
                self.prediction_distance_m,
                self.motor_scale,
                self.aggregate_motor_cap_nm,
                self.angular_damping,
                self.contact_skin_m,
                self.posture_stiffness_multiplier,
                self.posture_damping_multiplier,
                self.supported_tilt_gain,
                self.supported_com_gain_rad_per_m,
                self.supported_com_velocity_gain_s_rad_per_m,
                self.supported_angular_velocity_gain_s,
                self.reaching_coordinate_increment_rad,
                self.reaching_damping_m2,
            ]
            .iter()
            .all(|v| v.is_finite() && *v >= 0.0)
            || self.motor_scale > 1.0
            || self.aggregate_motor_cap_nm == 0.0
            || !self.grab_limits.valid()
        {
            return Err("Invalid physics profile".into());
        }
        Ok(())
    }
}
fn default_fall_iterations() -> usize {
    32
}
pub fn configure(world: &mut PhysicsWorld, p: &Profile) {
    world.integration_parameters.friction_model = match p.contact_friction_model {
        ContactFriction::Simplified => FrictionModel::Simplified,
        ContactFriction::Coulomb => FrictionModel::Coulomb,
    };
    world.integration_parameters.dt = p.dt_s;
    world.integration_parameters.num_solver_iterations = p.solver_iterations;
    world.integration_parameters.num_internal_pgs_iterations = p.internal_pgs_iterations;
    world.integration_parameters.normalized_allowed_linear_error = p.allowed_linear_error_m;
    world.integration_parameters.normalized_prediction_distance = p.prediction_distance_m;
    world.integration_parameters.max_ccd_substeps = p.max_ccd_substeps;
    world.integration_parameters.normalized_max_linear_velocity = if p.disable_solver_speed_cap {
        f32::MAX
    } else {
        400.0
    };
}
struct Filter {
    exclusions: BTreeSet<(u128, u128)>,
}
fn handle_key(h: ColliderHandle) -> u128 {
    let (i, g) = h.into_raw_parts();
    ((g as u128) << 32) | i as u128
}
impl PhysicsHooks for Filter {
    fn filter_contact_pair(&self, c: &PairFilterContext) -> Option<SolverFlags> {
        let (a, b) = (handle_key(c.collider1), handle_key(c.collider2));
        if self.exclusions.contains(&(a.min(b), a.max(b))) {
            None
        } else {
            Some(SolverFlags::COMPUTE_IMPULSES)
        }
    }
}
pub struct Simulation {
    pub(crate) world: PhysicsWorld,
    pub(crate) model: Model,
    pub(crate) bodies: Vec<RigidBodyHandle>,
    pub(crate) colliders: Vec<ColliderHandle>,
    pub(crate) joints: Vec<Option<ImpulseJointHandle>>,
    pub(crate) floor: ColliderHandle,
    filter: Filter,
    pub profile: Profile,
    pub tick: u64,
    pub generation: u64,
    pub motion: Motion,
    pub counters: Counters,
    targets: Vec<[f32; 3]>,
    rest_targets: Vec<[f32; 3]>,
    rest_com: Vector,
    heading: Rotation,
    grab: Option<grab::Grab>,
    last_press: u64,
    applied_sequence: u64,
    contacts: Vec<Contact>,
    persistence: Vec<f32>,
    pub diagnostics: Diagnostics,
    pub first_failure: Option<measurement::Failure>,
    motor_ceiling_sum: f32,
    substeps_completed: u64,
    stepping: Option<stepping::StepController>,
    motion_observation: motion::Observation,
    environment: Option<environment::Environment>,
    striker: Option<striker::Striker>,
    recovery: Option<recovery::Controller>,
}
impl Simulation {
    pub fn new(heading: f32, profile: Profile, floor_enabled: bool) -> Result<Self, String> {
        Self::build(heading, profile, floor_enabled, None, None)
    }
    pub fn new_protocol(
        heading: f32,
        profile: Profile,
        floor_enabled: bool,
    ) -> Result<Self, String> {
        let mut s = Self::new(heading, profile, floor_enabled)?;
        s.striker = Some(striker::Striker::new(&mut s.world)?);
        Ok(s)
    }
    pub fn request_strike(&mut self) -> Result<bool, String> {
        if self.first_failure.is_some() {
            return Ok(false);
        }
        let torso = self
            .model
            .segments
            .iter()
            .position(|s| s.id == "torso")
            .ok_or("Missing torso")?;
        let target = self.world.bodies[self.bodies[torso]].translation();
        self.striker
            .as_mut()
            .ok_or("Not a protocol trial")?
            .request(&mut self.world, &self.colliders, target)
    }
    pub fn new_playground(
        heading: f32,
        profile: Profile,
        floor_enabled: bool,
        settings: lh_contracts::PlaygroundSettings,
    ) -> Result<Self, String> {
        Self::build(heading, profile, floor_enabled, Some(settings), None)
    }
    fn build(
        heading: f32,
        profile: Profile,
        floor_enabled: bool,
        settings: Option<lh_contracts::PlaygroundSettings>,
        seed: Option<&[lh_model::Pose]>,
    ) -> Result<Self, String> {
        profile.validate()?;
        if !heading.is_finite() || profile.dt_s != SUBSTEP_S || profile.tick_substeps != 4 {
            return Err("Invalid initialization/profile timing".into());
        }
        let model = Model::canonical()?;
        if let Some(poses) = seed {
            recovery::validate_seed(&model, poses)?;
        }
        let mut world = PhysicsWorld::new();
        configure(&mut world, &profile);
        let floor = world.insert_collider(
            ColliderBuilder::cuboid(
                lh_model::FLAT_FLOOR_HALF_EXTENTS[0],
                lh_model::FLAT_FLOOR_HALF_EXTENTS[1],
                lh_model::FLAT_FLOOR_HALF_EXTENTS[2],
            )
            .translation(Vector::from_array(lh_model::FLAT_FLOOR_CENTER))
            .friction(4.0)
            .enabled(floor_enabled),
            None,
        );
        let heading_rotation = Rotation::from_rotation_y(heading);
        let environment = settings
            .map(|config| environment::Environment::new(&mut world, &model, config))
            .transpose()?;
        let spawn_offset = environment
            .as_ref()
            .map(|e| e.spawn_offset(&world, &model))
            .transpose()?
            .unwrap_or(Vector::ZERO);
        let mut bodies = Vec::new();
        let mut colliders = Vec::new();
        let mut joints = Vec::new();
        let mut targets = Vec::new();
        for (index, s) in model.segments.iter().enumerate() {
            let mass = integrate_mass(&s.geometry, s.mass_kg)?;
            let mass = MassProperties::with_principal_inertia_frame(
                vector(mass.center),
                s.mass_kg,
                vector(mass.principal),
                rotation(mass.frame),
            );
            let points: Vec<_> = s
                .geometry
                .vertices
                .chunks_exact(3)
                .map(|p| Vector::new(p[0], p[1], p[2]))
                .collect();
            let triangles: Vec<_> = s
                .geometry
                .indices
                .chunks_exact(3)
                .map(|i| [i[0], i[1], i[2]])
                .collect();
            let collider = ColliderBuilder::convex_mesh(points, &triangles)
                .ok_or_else(|| format!("Invalid convex {}", s.id))?
                .mass_properties(mass)
                .friction(if matches!(s.role.as_str(), "hindfoot" | "forefoot") {
                    4.0
                } else if matches!(
                    s.role.as_str(),
                    "hand" | "forearm" | "forearm-twist" | "shin" | "ankle"
                ) {
                    1.2
                } else {
                    0.45
                })
                .friction_combine_rule(CoefficientCombineRule::Min)
                .restitution(0.02)
                .contact_skin(profile.contact_skin_m)
                .active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS);
            let initial_pose = seed.map_or_else(
                || {
                    Pose::from_parts(
                        heading_rotation * vector(s.initial.position) + spawn_offset,
                        heading_rotation * rotation(s.initial.rotation),
                    )
                },
                |poses| {
                    Pose::from_parts(
                        vector(poses[index].position),
                        rotation(poses[index].rotation),
                    )
                },
            );
            let body = RigidBodyBuilder::dynamic()
                .pose(initial_pose)
                .linear_damping(0.0)
                .angular_damping(profile.angular_damping)
                .can_sleep(false)
                .ccd_enabled(true)
                .soft_ccd_prediction(
                    if matches!(s.role.as_str(), "ankle" | "hindfoot" | "forefoot") {
                        0.0
                    } else {
                        0.12
                    },
                )
                .additional_solver_iterations(profile.additional_body_iterations);
            let (b, c) = world.insert(body, collider);
            world.bodies[b].recompute_mass_properties_from_colliders(&world.colliders);
            bodies.push(b);
            colliders.push(c);
        }
        let mut exclusions = BTreeSet::new();
        for (i, s) in model.segments.iter().enumerate() {
            for id in &s.collision_exclusions {
                let j = model
                    .segments
                    .iter()
                    .position(|s| &s.id == id)
                    .ok_or("Missing exclusion")?;
                let (a, b) = (handle_key(colliders[i]), handle_key(colliders[j]));
                exclusions.insert((a.min(b), a.max(b)));
            }
            if let (Some(parent), Some(p)) = (&s.parent, &s.joint_profile) {
                let parent = model
                    .segments
                    .iter()
                    .position(|s| &s.id == parent)
                    .ok_or("Missing parent")?;
                let joint = anatomical_joint(p)?;
                let q = if seed.is_some() {
                    measurement::coordinates(
                        heading_rotation * rotation(model.segments[parent].initial.rotation),
                        heading_rotation * rotation(s.initial.rotation),
                        p,
                    )
                } else {
                    measurement::coordinates(
                        *world.bodies[bodies[parent]].rotation(),
                        *world.bodies[bodies[i]].rotation(),
                        p,
                    )
                };
                targets.push(q);
                joints.push(Some(world.insert_impulse_joint(
                    bodies[parent],
                    bodies[i],
                    joint,
                )));
            } else {
                joints.push(None);
                targets.push([0.0; 3]);
            }
        }
        let motor_ceiling_sum = model
            .segments
            .iter()
            .filter_map(|s| s.joint_profile.as_ref())
            .flat_map(|j| &j.axes)
            .map(|a| a.max_motor_torque_nm)
            .sum();
        let mut result = Self {
            world,
            model,
            bodies,
            colliders,
            joints,
            floor,
            filter: Filter { exclusions },
            profile,
            tick: 0,
            generation: 1,
            motion: Motion::Upright,
            counters: Counters::default(),
            rest_targets: targets.clone(),
            targets,
            rest_com: Vector::ZERO,
            heading: heading_rotation,
            grab: None,
            last_press: 0,
            applied_sequence: 0,
            contacts: Vec::new(),
            persistence: vec![0.0; 25],
            diagnostics: Diagnostics::default(),
            first_failure: None,
            motor_ceiling_sum,
            substeps_completed: 0,
            stepping: None,
            motion_observation: motion::Observation::default(),
            environment,
            striker: None,
            recovery: None,
        };
        result.rest_com = result.center_of_mass();
        if seed.is_some() {
            result.motion = Motion::Fallen;
        }
        if result.profile.corrective_steps_enabled {
            result.stepping = Some(stepping::StepController::new(&result)?);
        }
        result.configure_motors()?;
        Ok(result)
    }
    fn configure_motors(&mut self) -> Result<(), String> {
        if self.profile.recovery_enabled
            && self.motion == Motion::Fallen
            && self.recovery.is_none()
            && recovery::has_persistent_support(self)
        {
            self.recovery = Some(recovery::Controller::new(self)?);
            self.motion = Motion::Recovering;
        }
        let falling = matches!(self.motion, Motion::Falling | Motion::Fallen);
        let recovering = self.motion == Motion::Recovering;
        if falling {
            self.stepping = None;
        }
        // This intent changes bounded native motor targets only. Unsupported
        // bodies receive no balance correction, and physics owns every pose.
        self.targets.clone_from(&self.rest_targets);
        if let Some(mut controller) = self.recovery.take() {
            self.targets = controller.update(self)?;
            self.recovery = Some(controller);
        }
        let mut reference = self.rest_com;
        let mut step_targets = Vec::new();
        if let Some(mut controller) = self.stepping.take() {
            let intent = controller.update(self)?;
            reference = intent.reference;
            step_targets = intent.targets;
            if intent.landed {
                self.counters.steps += 1;
            }
            self.stepping = Some(controller);
        }
        if let Some(grab) = &self.grab {
            reaching::arm_targets(self, grab.segment, grab.local_anchor, grab.control_target())?
                .into_iter()
                .for_each(|(index, angles)| self.targets[index] = angles);
        }
        for (index, angles) in step_targets {
            self.targets[index] = angles;
        }
        let mut supported = [false; 2];
        {
            for (i, segment) in self
                .model
                .segments
                .iter()
                .enumerate()
                .filter(|(_, s)| matches!(s.role.as_str(), "hindfoot" | "forefoot"))
            {
                let load = self.support_normal_impulse(self.colliders[i])? / self.profile.dt_s;
                if load >= 3.0 {
                    supported[usize::from(segment.id.starts_with("right"))] = true;
                }
            }
        }
        if recovering && self.recovery.as_ref().is_some_and(|r| r.standing()) {
            reference = recovery::standing_reference(self);
        }
        if !falling
            && (!recovering || self.recovery.as_ref().is_some_and(|r| r.standing()))
            && supported.iter().any(|s| *s)
        {
            let pelvis = self.world.bodies.get(self.bodies[0]).ok_or("Lost pelvis")?;
            let mut q = self.heading.inverse() * (*pelvis.rotation());
            if q.w < 0.0 {
                q = -q;
            }
            let tilt = 2.0 * q.x.atan2(q.w);
            let roll = 2.0 * q.z.atan2(q.w);
            let angular_velocity = self.heading.inverse() * pelvis.angvel();
            let displacement = self.heading.inverse() * (self.center_of_mass() - reference);
            let mut velocity = Vector::ZERO;
            for h in &self.bodies {
                let b = self.world.bodies.get(*h).ok_or("Lost body")?;
                velocity += b.linvel() * b.mass();
            }
            velocity = self.heading.inverse() * velocity / self.total_mass();
            let correction = self.profile.supported_tilt_gain * tilt
                + self.profile.supported_com_gain_rad_per_m * displacement.z
                + self.profile.supported_com_velocity_gain_s_rad_per_m * velocity.z
                + self.profile.supported_angular_velocity_gain_s * angular_velocity.x;
            let lateral = self.profile.supported_tilt_gain * roll
                - self.profile.supported_com_gain_rad_per_m * displacement.x
                - self.profile.supported_com_velocity_gain_s_rad_per_m * velocity.x
                + self.profile.supported_angular_velocity_gain_s * angular_velocity.z;
            for (i, segment) in self
                .model
                .segments
                .iter()
                .enumerate()
                .filter(|(_, s)| matches!(s.role.as_str(), "ankle" | "hindfoot"))
            {
                if !supported[usize::from(segment.id.starts_with("right"))] {
                    continue;
                }
                let joint_profile = segment.joint_profile.as_ref().ok_or("Missing ankle")?;
                let parent = self
                    .world
                    .impulse_joints
                    .get(self.joints[i].ok_or("Missing ankle joint")?)
                    .ok_or("Lost ankle joint")?
                    .body1();
                for (index, basis, angle) in [(0, Vector::X, correction), (2, Vector::Z, lateral)] {
                    if (segment.role == "ankle") != (index == 0) {
                        continue;
                    }
                    let axis = joint_profile
                        .axes
                        .iter()
                        .find(|a| a.coordinate == if index == 0 { 'x' } else { 'z' })
                        .ok_or("Missing supported limb coordinate")?;
                    let world_axis = *self
                        .world
                        .bodies
                        .get(parent)
                        .ok_or("Lost supported limb parent")?
                        .rotation()
                        * rotation(joint_profile.parent_frame.rotation)
                        * basis;
                    let direction = world_axis.dot(self.heading * basis);
                    let angle = if recovering {
                        angle.clamp(-0.12, 0.12)
                    } else {
                        angle
                    };
                    self.targets[i][index] = (self.targets[i][index] + angle * direction)
                        .clamp(axis.min_radians, axis.max_radians);
                }
            }
        }
        // A swing reserves more of the existing aggregate ceiling for the loaded
        // leg. Every axis retains a nonzero reserve; a zero pre-solve PD request
        // cannot remove the implicit motor response needed to oppose gravity.
        let retained = self
            .stepping
            .as_ref()
            .and_then(|controller| controller.retained_side());
        let mut weights = [1.0; 25];
        if let Some(side) = retained {
            for (i, segment) in self.model.segments.iter().enumerate() {
                weights[i] = if matches!(
                    segment.role.as_str(),
                    "thigh" | "shin" | "ankle" | "hindfoot" | "forefoot"
                ) {
                    if usize::from(segment.id.starts_with("right")) == side {
                        1.0
                    } else {
                        0.55
                    }
                } else {
                    0.12
                };
            }
        }
        let weighted_ceiling: f32 = if retained.is_some() || recovering {
            self.model
                .segments
                .iter()
                .enumerate()
                .filter_map(|(i, s)| {
                    s.joint_profile.as_ref().map(|p| {
                        p.axes
                            .iter()
                            .map(|a| {
                                a.max_motor_torque_nm
                                    * weights[i]
                                    * self
                                        .recovery
                                        .as_ref()
                                        .map_or(1.0, |r| r.weight(s, a.coordinate))
                            })
                            .sum::<f32>()
                    })
                })
                .sum()
        } else {
            self.motor_ceiling_sum
        };
        let effort_scale = self
            .profile
            .motor_scale
            .min(self.profile.aggregate_motor_cap_nm / weighted_ceiling);
        // Native force-based motors own each anatomical axis. Quiet-standing
        // stiffness and damping stay unchanged when swing reserves are enabled.
        let scale = self
            .profile
            .motor_scale
            .min(self.profile.aggregate_motor_cap_nm / self.motor_ceiling_sum);
        let mut requested_effort = 0.0_f32;
        for (i, h) in self.joints.iter().enumerate() {
            if let (Some(h), Some(p)) = (h, &self.model.segments[i].joint_profile) {
                let parent_handle = self
                    .world
                    .impulse_joints
                    .get(*h)
                    .ok_or("Lost joint")?
                    .body1();
                let parent = &self.world.bodies[parent_handle];
                let child = &self.world.bodies[self.bodies[i]];
                let coordinates =
                    measurement::coordinates(*parent.rotation(), *child.rotation(), p);
                let parent_frame = *parent.rotation() * rotation(p.parent_frame.rotation);
                let relative =
                    parent_frame.inverse() * (*child.rotation() * rotation(p.child_frame.rotation));
                let omega = parent_frame.inverse() * (child.angvel() - parent.angvel());
                let recovery_torque = self.recovery.as_ref().map_or(Vector::ZERO, |r| {
                    parent_frame.inverse() * r.root_torque(self, &self.model.segments[i])
                });
                let joint = self
                    .world
                    .impulse_joints
                    .get_mut(*h, true)
                    .ok_or("Lost joint")?;
                for a in &p.axes {
                    let index = match a.coordinate {
                        'x' => 0,
                        'y' => 1,
                        _ => 2,
                    };
                    let v = relative.xyz();
                    let denominator = relative.w * relative.w + v[index] * v[index];
                    if !denominator.is_finite() || denominator < 1e-12 {
                        return Err("Singular motor coordinate".into());
                    }
                    let e = match index {
                        0 => Vector::X,
                        1 => Vector::Y,
                        _ => Vector::Z,
                    };
                    let jac =
                        (relative.w * relative.w * e + relative.w * v.cross(e) + v[index] * v)
                            / denominator;
                    let (target, stiffness, damping, max_force) = if falling {
                        let p =
                            passive::feedback(coordinates[index], a, p.limit_soft_zone_fraction);
                        (p.target, p.stiffness, p.damping, p.ceiling)
                    } else {
                        (
                            self.targets[i][index],
                            a.passive_stiffness_nm_per_rad
                                * self.profile.posture_stiffness_multiplier,
                            a.damping_nms_per_rad * self.profile.posture_damping_multiplier,
                            a.max_motor_torque_nm,
                        )
                    };
                    let error = (target - coordinates[index] + std::f32::consts::PI)
                        .rem_euclid(std::f32::consts::TAU)
                        - std::f32::consts::PI;
                    // Passive tissue gains retain their canonical values;
                    // the shared torque budget still caps every native motor.
                    let gain_scale = if falling { 1.0 } else { scale };
                    let feedforward = recovery_torque.dot(e);
                    let target_velocity = if damping * gain_scale > 0.0 {
                        feedforward / (damping * gain_scale)
                    } else {
                        0.0
                    };
                    let request =
                        (stiffness * error - damping * jac.dot(omega)) * gain_scale + feedforward;
                    let ceiling = max_force
                        * effort_scale
                        * weights[i]
                        * self
                            .recovery
                            .as_ref()
                            .map_or(1.0, |r| r.weight(&self.model.segments[i], a.coordinate));
                    requested_effort += request.clamp(-ceiling, ceiling).abs();
                    joint
                        .data
                        .set_motor_model(axis(a.coordinate)?, MotorModel::ForceBased)
                        .set_motor(
                            axis(a.coordinate)?,
                            target,
                            target_velocity,
                            stiffness * gain_scale,
                            damping * gain_scale,
                        )
                        .set_motor_max_force(axis(a.coordinate)?, ceiling);
                }
            }
        }
        self.diagnostics.requested_motor_effort_nm = requested_effort;
        Ok(())
    }
    pub(crate) fn environment_colliders(&self) -> impl Iterator<Item = ColliderHandle> + '_ {
        std::iter::once(self.floor)
            .chain(
                self.environment
                    .iter()
                    .flat_map(|e| e.colliders.iter().copied()),
            )
            .chain(self.striker.iter().flat_map(|s| {
                s.room_colliders
                    .iter()
                    .copied()
                    .chain(std::iter::once(s.collider))
            }))
    }
    pub(crate) fn support_normal_impulse(&self, collider: ColliderHandle) -> Result<f32, String> {
        let mut impulse = 0.0;
        for pair in self.world.narrow_phase.contact_pairs_with(collider) {
            let other = if pair.collider1 == collider {
                pair.collider2
            } else {
                pair.collider1
            };
            if (other == self.floor
                || self.environment.as_ref().is_some_and(|e| e.contains(other))
                || self
                    .striker
                    .as_ref()
                    .is_some_and(|s| s.room_colliders.contains(&other)))
                && self.world.colliders[other].is_enabled()
            {
                impulse += measurement::support_normal_impulse(pair, collider)?;
            }
        }
        Ok(impulse)
    }
    pub(crate) fn surface_height(&self, x: f32, z: f32, ceiling: f32) -> f32 {
        self.environment
            .as_ref()
            .map_or(0.0, |e| e.height_at(&self.world, x, z, ceiling))
    }
    pub(crate) fn support_height(&self) -> f32 {
        let mut height = 0.0;
        let mut count = 0;
        for (i, _) in self
            .model
            .segments
            .iter()
            .enumerate()
            .filter(|(_, p)| p.role == "hindfoot")
        {
            let foot = self.world.bodies[self.bodies[i]].translation();
            height += self.surface_height(foot.x, foot.z, foot.y + 0.18);
            count += 1;
        }
        if count == 0 {
            0.0
        } else {
            height / count as f32
        }
    }
    pub fn set_floor_enabled(&mut self, enabled: bool) -> Result<(), String> {
        self.world
            .colliders
            .get_mut(self.floor)
            .ok_or("Lost floor")?
            .set_enabled(enabled);
        if !enabled {
            self.contacts.clear();
            self.persistence.fill(0.0);
        }
        Ok(())
    }
    pub fn begin_grab(
        &mut self,
        segment: usize,
        press: u64,
        local_anchor: Vec3,
        target: Vec3,
    ) -> Result<(), String> {
        if !matches!(self.motion, Motion::Upright | Motion::Reacting) {
            return Err("Grab unavailable during a fall or halted trial".into());
        }
        if press <= self.last_press {
            return Err("Fresh grab press required".into());
        }
        if self.bodies.get(segment).is_none() {
            return Err("Unknown grab segment".into());
        }
        self.grab = Some(grab::Grab::new(
            segment,
            press,
            vector(local_anchor),
            vector(target),
        )?);
        self.last_press = press;
        Ok(())
    }
    pub fn move_grab(&mut self, press: u64, target: Vec3) -> Result<(), String> {
        if !target.finite() {
            return Err("Non-finite grab target".into());
        }
        let grab = self.grab.as_mut().ok_or("No active grab")?;
        if grab.press != press {
            return Err("Stale grab press".into());
        }
        grab.raw_target = vector(target);
        Ok(())
    }
    pub fn cancel_grab(&mut self) {
        self.grab = None;
    }
    /// Narrow dispatcher for the physical-grab command family. The caller owns
    /// the Timeline and applies its boundary before these four substeps.
    pub fn apply_grab_command(
        &mut self,
        command: &lh_contracts::Command,
    ) -> Result<(), lh_contracts::Reject> {
        use lh_contracts::{Reject, SCHEMA_VERSION};
        let stamp = command.stamp;
        if stamp.schema != SCHEMA_VERSION {
            return Err(Reject::Schema);
        }
        if stamp.generation != self.generation {
            return Err(Reject::Generation);
        }
        if stamp.tick != self.tick {
            return Err(Reject::Late);
        }
        if stamp.sequence <= self.applied_sequence {
            return Err(Reject::OutOfOrder);
        }
        self.apply_grab_action(&command.action)?;
        self.applied_sequence = stamp.sequence;
        Ok(())
    }
    /// Timeline has already checked receipt ordering. A queued lower sequence
    /// may legitimately have a later scheduled tick than another queued command.
    fn apply_grab_action(
        &mut self,
        action: &lh_contracts::Action,
    ) -> Result<(), lh_contracts::Reject> {
        use lh_contracts::{Action, Reject};
        match action {
            Action::GrabBegin {
                press,
                segment,
                local_anchor,
                target,
            } => self.begin_grab(*segment as usize, *press, *local_anchor, *target),
            Action::GrabMove { press, target } => self.move_grab(*press, *target),
            Action::GrabEnd { press } => {
                if self.grab.as_ref().is_some_and(|g| g.press == *press) {
                    self.cancel_grab();
                    Ok(())
                } else {
                    Err("Stale grab end".into())
                }
            }
            Action::CancelGrab => {
                self.cancel_grab();
                Ok(())
            }
            _ => Err("Not a physical grab command".into()),
        }
        .map_err(|_| Reject::Invalid)
    }
    pub fn center_of_mass(&self) -> Vector {
        let mut sum = Vector::ZERO;
        let mut mass = 0.0;
        for h in &self.bodies {
            let b = &self.world.bodies[*h];
            sum += b.center_of_mass() * b.mass();
            mass += b.mass();
        }
        sum / mass
    }
    pub fn total_mass(&self) -> f32 {
        self.bodies
            .iter()
            .map(|h| self.world.bodies[*h].mass())
            .sum()
    }
    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            stamp: Stamp::new(self.generation, self.substeps_completed + 1, self.tick),
            simulation_time_s: self.substeps_completed as f64 / 240.0,
            contact_time_s: self.tick as f64 / 60.0,
            discontinuity: self.tick == 0 || self.first_failure.is_some(),
            segments: self
                .bodies
                .iter()
                .enumerate()
                .map(|(i, h)| {
                    let b = &self.world.bodies[*h];
                    lh_model::Pose {
                        id: self.model.segments[i].id.clone(),
                        position: vec3(b.translation()),
                        rotation: quat(*b.rotation()),
                        linear_velocity: vec3(b.linvel()),
                        angular_velocity: vec3(b.angvel()),
                    }
                })
                .collect(),
            environment: self.environment.as_ref().map(|e| e.snapshot(&self.world)),
            striker: self.striker.as_ref().map(|s| s.snapshot(&self.world)),
            motion: self.motion,
            contacts: self.contacts.clone(),
            counters: self.counters.clone(),
            diagnostics: self.diagnostics.clone(),
            acknowledged_sequence: self.applied_sequence,
        }
    }
    pub fn advance_tick(&mut self) -> Result<(), measurement::Failure> {
        self.advance_tick_observed(|_, _| Ok(()))
    }
    pub fn advance_tick_observed(
        &mut self,
        observer: impl FnMut(&Self, u32) -> Result<(), measurement::Failure>,
    ) -> Result<(), measurement::Failure> {
        self.advance_tick_traced(observer, |_, _| {})
    }
    pub fn advance_tick_traced(
        &mut self,
        mut observer: impl FnMut(&Self, u32) -> Result<(), measurement::Failure>,
        mut trace: impl FnMut(&Self, profiling::Event),
    ) -> Result<(), measurement::Failure> {
        if let Some(f) = &self.first_failure {
            return Err(f.clone());
        }
        let mut impulses = [0.0_f32; 25];
        for substep in 0..4 {
            let event = |s: &Self, phase, begin| profiling::Event {
                generation: s.generation,
                tick: s.tick,
                substep,
                phase,
                begin,
            };
            trace(self, event(self, profiling::Phase::Controller, true));
            if let Some(grab) = &mut self.grab {
                let Some(body) = self.world.bodies.get_mut(self.bodies[grab.segment]) else {
                    let failure = measurement::Failure::new(
                        self.tick,
                        substep,
                        "grab-ownership",
                        "lost body".into(),
                        1.0,
                        0.0,
                    );
                    self.motion = Motion::Halted;
                    self.first_failure = Some(failure.clone());
                    self.grab = None;
                    trace(self, event(self, profiling::Phase::Controller, false));
                    return Err(failure);
                };
                if let Err(error) = grab.apply(body, self.profile.dt_s, &self.profile.grab_limits) {
                    let failure = measurement::Failure::new(
                        self.tick,
                        substep,
                        "grab-input",
                        error,
                        1.0,
                        0.0,
                    );
                    self.motion = Motion::Halted;
                    self.first_failure = Some(failure.clone());
                    self.grab = None;
                    trace(self, event(self, profiling::Phase::Controller, false));
                    return Err(failure);
                }
            }
            if let Err(error) = self.configure_motors() {
                let f = measurement::Failure::new(self.tick, substep, "ownership", error, 0.0, 0.0);
                self.first_failure = Some(f.clone());
                self.motion = Motion::Halted;
                self.cancel_grab();
                trace(self, event(self, profiling::Phase::Controller, false));
                return Err(f);
            }
            trace(self, event(self, profiling::Phase::Controller, false));
            trace(self, event(self, profiling::Phase::Physics, true));
            if let Some(striker) = &mut self.striker
                && let Err(error) =
                    striker.before_step(&mut self.world, &self.colliders, self.profile.dt_s)
            {
                let failure = measurement::Failure::new(
                    self.tick,
                    substep,
                    "striker-clearance",
                    error,
                    1.0,
                    0.0,
                );
                self.motion = Motion::Halted;
                self.first_failure = Some(failure.clone());
                self.cancel_grab();
                trace(self, event(self, profiling::Phase::Physics, false));
                return Err(failure);
            }
            if let Some(environment) = &self.environment {
                environment.update(
                    &mut self.world,
                    (self.substeps_completed + 1) as f32 * self.profile.dt_s,
                );
            }
            // The retained recovery path resolves passive limb/contact loads
            // with 32 internal passes. Keep quiet-standing's budget unchanged.
            self.world
                .integration_parameters
                .num_internal_pgs_iterations = if matches!(
                self.motion,
                Motion::Falling | Motion::Fallen | Motion::Recovering
            ) {
                self.profile
                    .fall_internal_pgs_iterations
                    .max(self.profile.internal_pgs_iterations)
            } else {
                self.profile.internal_pgs_iterations
            };
            self.world.step_with_events(&self.filter, &());
            self.substeps_completed += 1;
            trace(self, event(self, profiling::Phase::Physics, false));
            trace(self, event(self, profiling::Phase::Contacts, true));
            if let Some(striker) = &mut self.striker {
                match striker.after_step(&mut self.world, &self.colliders) {
                    Ok(true) => {
                        self.counters.strikes = striker.snapshot(&self.world).impact_id;
                        self.observe_impact();
                    }
                    Ok(false) => {}
                    Err(error) => {
                        let failure = measurement::Failure::new(
                            self.tick,
                            substep,
                            "striker-contact",
                            error,
                            1.0,
                            0.0,
                        );
                        self.motion = Motion::Halted;
                        self.first_failure = Some(failure.clone());
                        self.cancel_grab();
                        trace(self, event(self, profiling::Phase::Contacts, false));
                        return Err(failure);
                    }
                }
            }
            let mut loads_n = [0.0; 25];
            for (i, c) in self.colliders.iter().enumerate() {
                match self.support_normal_impulse(*c) {
                    Ok(impulse) => {
                        impulses[i] += impulse;
                        loads_n[i] = impulse / self.profile.dt_s;
                    }
                    Err(error) => {
                        let failure = measurement::Failure::new(
                            self.tick,
                            substep,
                            "contact-observation",
                            format!("{}: {error}", self.model.segments[i].id),
                            1.0,
                            0.0,
                        );
                        self.motion = Motion::Halted;
                        self.first_failure = Some(failure.clone());
                        self.cancel_grab();
                        trace(self, event(self, profiling::Phase::Contacts, false));
                        return Err(failure);
                    }
                }
            }
            trace(self, event(self, profiling::Phase::Contacts, false));
            trace(self, event(self, profiling::Phase::Integrity, true));
            let inspection = measurement::inspect(self, substep);
            trace(self, event(self, profiling::Phase::Integrity, false));
            if let Err(f) = inspection {
                self.motion = Motion::Halted;
                self.first_failure = Some(f.clone());
                self.cancel_grab();
                return Err(f);
            }
            trace(self, event(self, profiling::Phase::Observer, true));
            self.observe_motion(&loads_n);
            let observation = observer(self, substep);
            trace(self, event(self, profiling::Phase::Observer, false));
            if let Err(f) = observation {
                self.motion = Motion::Halted;
                self.first_failure = Some(f.clone());
                self.cancel_grab();
                return Err(f);
            }
        }
        self.contacts.clear();
        for (i, impulse) in impulses.iter().enumerate().take(self.bodies.len()) {
            let load = impulse / (4.0 * self.profile.dt_s);
            if load >= 3.0 {
                self.persistence[i] += 1.0 / 60.0;
                self.contacts.push(Contact {
                    segment: i as u8,
                    normal_load_n: load,
                    persistence_s: self.persistence[i],
                });
            } else {
                self.persistence[i] = 0.0;
            }
        }
        if matches!(self.motion, Motion::Upright | Motion::Reacting) {
            self.counters.upright_ticks += 1;
        }
        self.tick += 1;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn contact_profile_is_explicit_and_legacy_json_preserves_simplified_default() {
        let mut json = serde_json::to_value(Profile::default()).expect("profile");
        json.as_object_mut()
            .unwrap()
            .remove("contact_friction_model");
        let legacy: Profile = serde_json::from_value(json.clone()).expect("legacy profile");
        assert_eq!(legacy.contact_friction_model, ContactFriction::Simplified);
        let mut world = PhysicsWorld::new();
        configure(&mut world, &legacy);
        assert_eq!(
            world.integration_parameters.friction_model,
            FrictionModel::Simplified
        );
        json["contact_friction_model"] = serde_json::json!("coulomb");
        let coulomb: Profile = serde_json::from_value(json.clone()).expect("explicit profile");
        configure(&mut world, &coulomb);
        assert_eq!(
            world.integration_parameters.friction_model,
            FrictionModel::Coulomb
        );
        json["contact_fricton_model"] = serde_json::json!("simplified");
        assert!(
            serde_json::from_value::<Profile>(json).is_err(),
            "typos cannot silently tune the wrong profile"
        );
    }
    #[test]
    fn mass_accounting_and_ballistic_com_are_independent_of_heading() {
        for heading in [0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0] {
            let mut s = Simulation::new(heading, Profile::default(), false).expect("fixture");
            assert!(
                (s.total_mass() - 72.2).abs() < 1e-4,
                "mass counted exactly once"
            );
            let origin = s.center_of_mass();
            for tick in 1..=30 {
                s.advance_tick().expect("ballistic integrity");
                let time = tick as f32 / 60.0;
                let com = s.center_of_mass();
                let expected = origin.y - 0.5 * 9.81 * time * time;
                // Semi-implicit fixed integration has a deterministic O(dt) position
                // offset; velocity and horizontal COM must still be ballistic.
                assert!((com.y - expected).abs() < 9.81 * time * SUBSTEP_S / 2.0 + 0.0001);
                assert!((com.x - origin.x).abs() < 0.0001 && (com.z - origin.z).abs() < 0.0001);
                let momentum: Vector = s
                    .bodies
                    .iter()
                    .map(|h| s.world.bodies[*h].linvel() * s.world.bodies[*h].mass())
                    .sum();
                assert!((momentum.y / s.total_mass() + 9.81 * time).abs() < 0.0002);
                assert!(s.snapshot().contacts.is_empty());
            }
        }
    }
    #[test]
    fn rejects_invalid_profile_before_world_creation() {
        let p = Profile {
            aggregate_motor_cap_nm: f32::NAN,
            ..Profile::default()
        };
        assert!(Simulation::new(0.0, p, true).is_err());
        let p = Profile {
            reaching_damping_m2: 0.0,
            ..Profile::default()
        };
        assert!(Simulation::new(0.0, p, true).is_err());
    }
    #[test]
    fn lost_grab_body_halts_and_cancels_before_any_integration() {
        let mut s = Simulation::new(0.0, Profile::default(), true).expect("fixture");
        s.begin_grab(14, 41, Vec3::default(), Vec3::default())
            .expect("press");
        s.world
            .remove_body(s.bodies[14])
            .expect("deliberate ownership violation");
        let failure = s.advance_tick().expect_err("lost body halts");
        assert_eq!(failure.check, "grab-ownership");
        assert_eq!(s.motion, Motion::Halted);
        assert!(s.grab.is_none());
        assert_eq!(s.substeps_completed, 0);
        assert_eq!(
            s.advance_tick().expect_err("halt persists").check,
            failure.check
        );
    }
    #[test]
    fn equivalent_quaternion_signs_produce_the_same_supported_intent() {
        for heading in [0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0] {
            let mut s = Simulation::new(heading, Profile::default(), true).expect("fixture");
            s.advance_tick().expect("first grounded tick");
            s.configure_motors().expect("supported intent");
            let targets = s.targets.clone();
            let h = s.bodies[0];
            let q = *s.world.bodies[h].rotation();
            s.world.bodies[h].set_rotation(-q, true);
            s.configure_motors().expect("same physical orientation");
            for (a, b) in targets.iter().zip(&s.targets) {
                for axis in 0..3 {
                    assert!((a[axis] - b[axis]).abs() < 1e-6);
                }
            }
        }
    }
    #[test]
    fn halted_substep_observation_has_the_integrated_time_and_new_sequence() {
        let mut s = Simulation::new(0.0, Profile::default(), false).expect("fixture");
        let initial = s.snapshot();
        s.advance_tick_observed(|s, substep| {
            Err(measurement::Failure::new(
                s.tick,
                substep,
                "deliberate-observer-stop",
                "fixture".into(),
                1.0,
                0.0,
            ))
        })
        .expect_err("halt");
        let halted = s.snapshot();
        assert_eq!(halted.simulation_time_s, 1.0 / 240.0);
        assert!(lh_contracts::accepts_observation(
            initial.stamp,
            halted.stamp
        ));
        assert_eq!(halted.motion, Motion::Halted);
        assert!(halted.discontinuity);
        s.advance_tick()
            .expect_err("no further integration while halted");
        assert_eq!(s.snapshot().simulation_time_s, halted.simulation_time_s);
    }
}
