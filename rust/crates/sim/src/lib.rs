//! Dynamic f32 simulation with read-only substep inspection and no DOM/GPU dependencies.
pub mod actuators;
#[cfg(not(target_arch = "wasm32"))]
pub mod articulation;
pub mod calibration;
pub mod cycles;
pub mod environment;
pub mod grab;
pub mod getup;
pub mod legacy;
pub mod measurement;
mod motion;
mod passive;
pub mod profiling;
mod quiet;
mod reaching;
pub mod recovery;
pub mod recovery_contract;
pub mod runtime;
pub mod scenarios;
mod stepping;
pub mod striker;
use lh_contracts::{Contact, Counters, Diagnostics, Motion, SUBSTEP_S, Snapshot, Stamp};
use lh_model::{Model, Quat, Vec3};
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
    joint.enforce_motor_vector_limit = true;
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
/// Explicit numerical semantics for an experimental GetUp qualification.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum GetUpSolverBudget {
    #[serde(rename = "consistent-recovery-phase-v1")]
    ConsistentRecoveryPhaseV1,
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum RootRotationNormalization {
    #[serde(rename = "unit-free-root-v1")]
    UnitFreeRootV1,
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum GetUpRepeatedRecovery {
    #[serde(rename = "completed-cycle-confirmed-fall-v1")]
    CompletedCycleConfirmedFallV1,
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
    /// Omitted profiles retain the historical teacher/real phase selection.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub getup_solver_budget: Option<GetUpSolverBudget>,
    /// Physical quaternion integration policy; disabled in historical profiles.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub root_rotation_normalization: Option<RootRotationNormalization>,
    /// Opt-in lifecycle for a genuinely new fall after a completed recovery.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub getup_repeated_recovery: Option<GetUpRepeatedRecovery>,
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
    #[serde(default, skip_serializing_if = "is_false")]
    pub recovery_first_phase_only: bool,
    /// Experimental pretrained policy; remains off until multi-case qualification.
    #[serde(default)]
    pub getup_policy_enabled: bool,
}
fn is_false(value: &bool) -> bool { !*value }
impl Default for Profile {
    fn default() -> Self {
        Self {
            id: "getup-reference-5ad52fbe".into(),
            dt_s: SUBSTEP_S,
            tick_substeps: 20,
            contact_friction_model: ContactFriction::Coulomb,
            solver_iterations: 20,
            internal_pgs_iterations: 20,
            fall_internal_pgs_iterations: default_fall_iterations(),
            getup_solver_budget: None,
            root_rotation_normalization: None,
            getup_repeated_recovery: None,
            additional_body_iterations: 5,
            allowed_linear_error_m: 0.001,
            prediction_distance_m: 0.002,
            motor_scale: 1.0,
            aggregate_motor_cap_nm: 920.0,
            angular_damping: 0.0,
            contact_skin_m: 0.0,
            max_ccd_substeps: 4,
            disable_solver_speed_cap: true,
            posture_stiffness_multiplier: 1.0,
            posture_damping_multiplier: 1.0,
            supported_tilt_gain: 2.0,
            supported_com_gain_rad_per_m: 5.0,
            supported_com_velocity_gain_s_rad_per_m: 5.0,
            supported_angular_velocity_gain_s: 2.0,
            grab_limits: grab::Limits::default(),
            reaching_iterations: 6,
            reaching_coordinate_increment_rad: 0.15,
            reaching_damping_m2: 0.01,
            corrective_steps_enabled: false,
            recovery_enabled: false,
            recovery_first_phase_only: false,
            getup_policy_enabled: false,
        }
    }
}
impl Profile {
    fn internal_pgs_for_phase(&self, recovery: bool) -> usize {
        if recovery {
            self.fall_internal_pgs_iterations.max(self.internal_pgs_iterations)
        } else {
            self.internal_pgs_iterations
        }
    }
    pub fn validate(&self) -> Result<(), String> {
        #[cfg(not(feature = "getup-repeat-cycle-experiment"))]
        if self.getup_repeated_recovery.is_some() {
            return Err("Repeated GetUp recovery requires getup-repeat-cycle-experiment".into());
        }
        if self.getup_repeated_recovery.is_some()
            && (!self.getup_policy_enabled || cfg!(feature = "getup-terminal-base-ready-experiment"))
        {
            return Err("Repeated GetUp recovery requires the source/terminal-balance controller without early or bumpless transfer".into());
        }
        #[cfg(not(feature = "unit-root-rotation-experiment"))]
        if self.root_rotation_normalization.is_some() {
            return Err("Unit root rotation requires unit-root-rotation-experiment".into());
        }
        if self.recovery_enabled || self.recovery_first_phase_only || self.corrective_steps_enabled {
            return Err("Legacy body controllers are unavailable for the GetUp reference body".into());
        }
        if self.dt_s != SUBSTEP_S
            || self.tick_substeps != lh_contracts::SUBSTEPS
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
#[derive(Clone, Serialize, Deserialize)]
struct Filter {
    exclusions: BTreeSet<(u128, u128)>,
}
fn handle_key(h: ColliderHandle) -> u128 {
    let (i, g) = h.into_raw_parts();
    ((g as u128) << 32) | i as u128
}
impl PhysicsHooks for Filter {
    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        let source = |h: ColliderHandle| (1..=25).contains(&context.colliders[h].user_data);
        if source(context.collider1) && source(context.collider2) {
            *context.friction = 0.0;
        }
    }
    #[cfg(all(test, feature = "experimental-solver-step-observation"))]
    fn before_solver_step(&self, dt: Real, bodies: &RigidBodySet) {
        recovery::planner::contact_observation::before_solve(dt, bodies);
    }
    #[cfg(all(test, feature = "experimental-solver-step-observation"))]
    fn after_solver_step(&self, dt: Real, bodies: &RigidBodySet, phase: &NarrowPhase) {
        recovery::planner::contact_observation::after_solve(dt, bodies, phase);
    }

    #[cfg(all(test, feature = "experimental-solver-step-observation"))]
    fn after_solver_positions(&self, bodies: &RigidBodySet) {
        recovery::planner::contact_observation::after_positions(bodies);
    }

    fn filter_contact_pair(&self, c: &PairFilterContext) -> Option<SolverFlags> {
        let (a, b) = (handle_key(c.collider1), handle_key(c.collider2));
        if self.exclusions.contains(&(a.min(b), a.max(b))) {
            None
        } else {
            Some(SolverFlags::COMPUTE_IMPULSES)
        }
    }
}
#[derive(Serialize, Deserialize)]
pub struct Simulation {
    /// Native experiment only. Held residual; PD is still recomputed each substep.
    #[cfg(feature = "stationary-balance-experiment")]
    #[serde(skip)]
    balance_residual: Option<[f32; 21]>,
    #[cfg(feature = "stationary-balance-experiment")]
    #[serde(skip)]
    balance_motion_profile: Option<getup::balance::MotionProfile>,
    #[cfg(feature = "stationary-balance-experiment")]
    #[serde(skip)]
    balance_source_command: Option<getup::balance::SourceCommand>,
    #[cfg(feature = "stationary-balance-experiment")]
    #[serde(skip)]
    balance_torso_force_n: [f32; 2],
    #[cfg(all(test, feature = "experimental-solver-step-observation"))]
    #[serde(skip)]
    coordination: Option<recovery::planner::whole_body_controller::Controller>,
    #[serde(skip)]
    pub(crate) world: PhysicsWorld,
    #[serde(skip, default = "checkpoint_model")]
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
    rest_com_roundoff: Vector,
    heading: Rotation,
    #[serde(skip)]
    grab: Option<grab::Grab>,
    last_press: u64,
    applied_sequence: u64,
    contacts: Vec<Contact>,
    persistence: Vec<f32>,
    integrated_support_substeps: [u64; 25],
    integrated_support_loads_n: [f32; 25],
    pub diagnostics: Diagnostics,
    /// Ephemeral test-only receipt written by the actual actuator application.
    /// Never controller state, a checkpoint field, or a source of commands.
    #[cfg(feature = "getup-terminal-tail-observer-experiment")]
    #[serde(skip)]
    last_motor_application: Option<getup::MotorApplication>,
    pub first_failure: Option<measurement::Failure>,
    motor_ceiling_sum: f32,
    substeps_completed: u64,
    #[serde(skip)]
    stepping: Option<stepping::StepController>,
    motion_observation: motion::Observation,
    #[serde(skip)]
    environment: Option<environment::Environment>,
    striker: Option<striker::Striker>,
    #[serde(skip)]
    recovery: Option<recovery::Controller>,
    getup_controller: Option<getup::controller::Controller>,
    #[serde(default)]
    getup_fixture: Option<getup::fixtures::State>,
}
fn checkpoint_model() -> Model {
    Model::canonical().expect("pinned GetUp model")
}
impl Simulation {
    /// Restore policy only. Saved quaternion/velocity/contact/carry bits are
    /// never projected or otherwise edited by construction/checkpoint plumbing.
    fn restore_unit_root_rotation_policy(&mut self) -> Result<(), String> {
        let enabled = self.profile.root_rotation_normalization.is_some();
        #[cfg(not(feature = "unit-root-rotation-experiment"))]
        {
            if enabled { return Err("Unit root rotation requires unit-root-rotation-experiment".into()); }
            Ok(())
        }
        #[cfg(feature = "unit-root-rotation-experiment")]
        {
            let root = *self.bodies.first().ok_or("Missing GetUp root")?;
            let link = *self.world.multibody_joints.rigid_body_link(root).ok_or("Missing root articulation")?;
            if link.id != 0 { return Err("GetUp root is not an articulation root".into()); }
            self.world.multibody_joints.get_multibody_mut(link.multibody)
                .ok_or("Missing root articulation")?
                .set_unit_root_rotation_enabled(enabled).map_err(str::to_string)
        }
    }

    fn validate_unit_root_rotation_policy(&self) -> Result<(), String> {
        let expected = self.profile.root_rotation_normalization.is_some();
        #[cfg(not(feature = "unit-root-rotation-experiment"))]
        {
            if expected { return Err("Unit root rotation requires unit-root-rotation-experiment".into()); }
            Ok(())
        }
        #[cfg(feature = "unit-root-rotation-experiment")]
        {
            let root = *self.bodies.first().ok_or("Missing GetUp root")?;
            let link = *self.world.multibody_joints.rigid_body_link(root).ok_or("Missing root articulation")?;
            if link.id != 0 { return Err("GetUp root is not an articulation root".into()); }
            let mb = self.world.multibody_joints.get_multibody(link.multibody).ok_or("Missing root articulation")?;
            if mb.unit_root_rotation_enabled() != expected {
                return Err("Unit root rotation profile/runtime policy mismatch".into());
            }
            mb.validate_unit_root_rotation().map_err(str::to_string)?;
            if self.world.multibody_joints.multibodies().any(|other|
                other.root().rigid_body_handle() != root && other.unit_root_rotation_enabled())
            {
                return Err("Unit root rotation enabled on an unrelated articulation".into());
            }
            Ok(())
        }
    }
    fn selected_internal_pgs_iterations(&self) -> usize {
        let active_getup = self.profile.getup_solver_budget
            == Some(GetUpSolverBudget::ConsistentRecoveryPhaseV1)
            && self.getup_controller.as_ref().is_some_and(|c| c.source_recovery_active());
        self.profile.internal_pgs_for_phase(
            matches!(self.motion, Motion::Falling | Motion::Fallen)
                || self.motion == Motion::Recovering
                    && (self.recovery.as_ref().is_some_and(|r| !r.handed_off()) || active_getup),
        )
    }
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
        self.request_strike_direction(false)
    }
    pub fn request_strike_from_behind(&mut self) -> Result<bool, String> {
        self.request_strike_direction(true)
    }
    fn request_strike_direction(&mut self, from_behind: bool) -> Result<bool, String> {
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
        let forward = *self.world.bodies[self.bodies[torso]].rotation() * Vector::Z;
        self.striker
            .as_mut()
            .ok_or("Not a protocol trial")?
            .request_direction(&mut self.world, &self.colliders, target, from_behind.then_some(forward))
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
        if !heading.is_finite() || profile.dt_s != SUBSTEP_S || profile.tick_substeps != lh_contracts::SUBSTEPS {
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
            .friction(1.0)
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
            let collider = getup::collider(s, index)?;
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
                .ccd_enabled(false)
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
            joints.push(None);
            targets.push([0.0; 3]);
        }
        let motor_ceiling_sum = model
            .segments
            .iter()
            .filter_map(|s| s.joint_profile.as_ref())
            .flat_map(|j| &j.axes)
            .map(|a| a.max_motor_torque_nm)
            .sum();
        let mut result = Self {
            #[cfg(feature = "stationary-balance-experiment")]
            balance_residual: None,
            #[cfg(feature = "stationary-balance-experiment")]
            balance_motion_profile: None,
            #[cfg(feature = "stationary-balance-experiment")]
            balance_source_command: None,
            #[cfg(feature = "stationary-balance-experiment")]
            balance_torso_force_n: [0.0; 2],
            #[cfg(all(test, feature = "experimental-solver-step-observation"))]
            coordination: None,
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
            rest_com_roundoff: Vector::ZERO,
            heading: heading_rotation,
            grab: None,
            last_press: 0,
            applied_sequence: 0,
            contacts: Vec::new(),
            persistence: vec![0.0; 25],
            integrated_support_substeps: [0; 25],
            integrated_support_loads_n: [0.0; 25],
            diagnostics: Diagnostics::default(),
            #[cfg(feature = "getup-terminal-tail-observer-experiment")]
            last_motor_application: None,
            first_failure: None,
            motor_ceiling_sum,
            substeps_completed: 0,
            stepping: None,
            motion_observation: motion::Observation::default(),
            environment,
            striker: None,
            recovery: None,
            getup_controller: None,
            getup_fixture: None,
        };
        getup::assemble(&mut result, spawn_offset.y)?;
        if result.profile.root_rotation_normalization.is_some() {
            result.restore_unit_root_rotation_policy()?;
        }
        result.capture_rest_com();
        if result.profile.getup_policy_enabled {
            result.getup_controller = Some(getup::controller::Controller::new(&result.model));
        }
        if seed.is_some() { return Err("Legacy anatomical seed is not a GetUp initial state".into()); }
        result.configure_motors()?;
        Ok(result)
    }
    fn configure_motors(&mut self) -> Result<(), String> {
        getup::configure(self)
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
            self.integrated_support_substeps.fill(0);
            self.integrated_support_loads_n.fill(0.0);
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
    /// the Timeline and applies its boundary before the twenty reference substeps.
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
    fn capture_rest_com(&mut self) {
        self.rest_com = self.center_of_mass();
        self.rest_com_roundoff = self.relative_center_of_mass(self.rest_com);
    }
    fn relative_center_of_mass(&self, reference: Vector) -> Vector {
        // Absolute f32 mass sums lose small COM changes at translated spawns.
        // Subtract the reference before accumulating, and round only once.
        let mut sum = [0.0_f64; 3];
        let mut mass = 0.0_f64;
        for handle in &self.bodies {
            let body = &self.world.bodies[*handle];
            let weight = f64::from(body.mass());
            let center = body.center_of_mass();
            for (axis, total) in sum.iter_mut().enumerate() {
                *total += (f64::from(center[axis]) - f64::from(reference[axis])) * weight;
            }
            mass += weight;
        }
        Vector::new(
            (sum[0] / mass) as f32,
            (sum[1] / mass) as f32,
            (sum[2] / mass) as f32,
        )
    }
    fn com_displacement_from(&self, reference: Vector, reference_roundoff: Vector) -> Vector {
        // Captured COM targets include their sub-ULP residual. Independent
        // world-space stepping and recovery targets have no such residual.
        // The observer never resets this reference at the settling boundary.
        self.relative_center_of_mass(reference) - reference_roundoff
    }
    pub fn total_mass(&self) -> f32 {
        self.bodies
            .iter()
            .map(|h| self.world.bodies[*h].mass())
            .sum()
    }
    pub fn snapshot(&self) -> Snapshot {
        let mut diagnostics = self.diagnostics.clone();
        diagnostics.recovery =
            self.recovery_status()
                .map(|status| lh_contracts::RecoveryObservation {
                    phase: format!("{:?}", status.phase),
                    arm_transfer: self.recovery.as_ref().and_then(|r| r.arm_transfer_stage()),
                    roll_coordination: self.recovery.as_ref().and_then(|r| r.roll_coordination()),
                    first_phase: self.recovery.as_ref().and_then(|r| r.first_phase_observation()),
                    elapsed_s: status.elapsed_s,
                    phase_time_s: status.phase_time_s,
                    qualified_time_s: status.qualified_time_s,
                    stable_time_s: status.stable_time_s,
                    stalled_s: status.stalled_s,
                    unsupported_s: status.unsupported_s,
                    retries: status.retries,
                    last_retry: status.last_retry.map(str::to_owned),
                    progress: status.progress,
                });
        diagnostics.terrain = self.environment.as_ref().map(|_| {
            let mut loaded_feet = [false; 2];
            let mut persistent_support_load_n = 0.0;
            for contact in &self.contacts {
                if contact.persistence_s + 1e-6 < 0.05 {
                    continue;
                }
                persistent_support_load_n += contact.normal_load_n;
                let segment = &self.model.segments[contact.segment as usize];
                if matches!(segment.role.as_str(), "hindfoot" | "forefoot")
                    && contact.normal_load_n >= 3.0
                {
                    loaded_feet[usize::from(segment.id.starts_with("right"))] = true;
                }
            }
            lh_contracts::TerrainObservation {
                support_height_m: self.support_height(),
                center_of_mass: vec3(self.center_of_mass()),
                loaded_feet,
                persistent_support_load_n,
            }
        });
        Snapshot {
            stamp: Stamp::new(self.generation, self.substeps_completed + 1, self.tick),
            simulation_time_s: self.substeps_completed as f64 / 800.0,
            contact_time_s: self.tick as f64 / f64::from(lh_contracts::TICK_HZ),
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
            diagnostics,
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
        self.diagnostics.delivered_motor_impulse_nm_s = None;
        self.diagnostics.delivered_motors = None;
        // Check ownership before force reset or any other body indexing. A
        // missing handle is a halted trial, never a panic or a skipped link.
        if let Some((index, _)) = self
            .bodies
            .iter()
            .enumerate()
            .find(|(_, handle)| self.world.bodies.get(**handle).is_none())
        {
            let grabbed = self.grab.as_ref().is_some_and(|grab| grab.segment == index);
            let failure = measurement::Failure::new(
                self.tick,
                0,
                if grabbed {
                    "grab-ownership"
                } else {
                    "ownership"
                },
                self.model.segments[index].id.clone(),
                1.0,
                0.0,
            );
            self.motion = Motion::Halted;
            self.first_failure = Some(failure.clone());
            self.cancel_grab();
            return Err(failure);
        }
        for substep in 0..lh_contracts::SUBSTEPS {
            if (cfg!(feature = "unit-root-rotation-experiment") || self.profile.root_rotation_normalization.is_some())
                && let Err(error) = self.validate_unit_root_rotation_policy()
            {
                let failure = measurement::Failure::new(self.tick, substep,
                    "unit-root-rotation", error, 1.0, 0.0);
                self.motion = Motion::Halted;
                self.first_failure = Some(failure.clone());
                self.cancel_grab();
                return Err(failure);
            }
            for handle in &self.bodies {
                self.world.bodies[*handle].reset_forces(false);
                self.world.bodies[*handle].reset_torques(false);
            }
            let event = |s: &Self, phase, begin| profiling::Event {
                generation: s.generation,
                tick: s.tick,
                substep,
                phase,
                begin,
            };
            trace(self, event(self, profiling::Phase::Controller, true));
            if let Some(grab) = &self.grab {
                let Some(body) = self.world.bodies.get(self.bodies[grab.segment]) else {
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
                let radius =
                    body.position().transform_point(grab.local_anchor) - body.center_of_mass();
                let velocity = getup::current_twists(self).map(|twists| {
                    let (linear, angular) = twists[grab.segment];
                    linear + angular.cross(radius)
                });
                let result = velocity.and_then(|velocity| {
                    let grab = self.grab.as_mut().expect("grab checked above");
                    grab.apply(
                        &mut self.world.bodies[self.bodies[grab.segment]],
                        velocity,
                        self.profile.dt_s,
                        &self.profile.grab_limits,
                    )
                });
                if let Err(error) = result {
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
                let check = if self.getup_controller.is_some() { "getup-policy" } else { "ownership" };
                let f = measurement::Failure::new(self.tick, substep, check, error, 0.0, 0.0);
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
            self.world.integration_parameters.num_internal_pgs_iterations =
                self.selected_internal_pgs_iterations();
            #[cfg(test)]
            if let Err(failure) = recovery::offline_trajectory_before_step(self, substep) {
                self.first_failure = Some(failure.clone());
                self.motion = Motion::Halted;
                return Err(failure);
            }
            #[cfg(all(test, feature = "experimental-solver-step-observation"))]
            recovery::planner::contact_observation::begin_step(self.substeps_completed + 1);
            #[cfg(all(test, feature = "numeric-parity-worker"))]
            if let Err(reason) = recovery::planner::numeric_parity_worker::before_integration(self) {
                let failure = measurement::Failure::new(self.tick, substep, "numeric-parity-admission", reason, 1., 0.);
                self.first_failure = Some(failure.clone());
                self.motion = Motion::Halted;
                return Err(failure);
            }
            #[cfg(all(test, feature = "coordination-worker"))]
            if let Err(reason) = recovery::planner::transition_worker::before_integration(self) {
                let failure = measurement::Failure::new(self.tick, substep, "coordination-admission", reason, 1., 0.);
                self.first_failure = Some(failure.clone());
                self.motion = Motion::Halted;
                return Err(failure);
            }
            self.world.step_with_events(&self.filter, &());
            #[cfg(all(test, feature = "coordination-worker"))]
            if let Err(reason) = recovery::planner::transition_worker::after_integration(self) {
                let failure = measurement::Failure::new(self.tick, substep, "coordination-accounting", reason, 1., 0.);
                self.first_failure = Some(failure.clone());
                self.motion = Motion::Halted;
                return Err(failure);
            }
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
            for (i, load) in loads_n.iter().enumerate() {
                self.integrated_support_loads_n[i] = *load;
                self.integrated_support_substeps[i] = if *load
                    >= recovery_contract::contract()
                        .recovery_acceptance
                        .minimum_load_n
                {
                    self.integrated_support_substeps[i] + 1
                } else {
                    0
                };
            }
            self.observe_motion(&loads_n);
            getup::fixtures::observe(self, &loads_n);
            #[cfg(feature = "stationary-balance-experiment")]
            let balance_active = self.balance_residual.is_some();
            #[cfg(not(feature = "stationary-balance-experiment"))]
            let balance_active = false;
            if !balance_active && let Some(mut controller) = self.getup_controller.take() {
                controller.observe(self, &loads_n);
                if let Some(observation) = self.diagnostics.getup.as_mut() { observation.policy = Some(controller.status.clone()); }
                self.getup_controller = Some(controller);
            }
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
            let load = impulse / (lh_contracts::SUBSTEPS as f32 * self.profile.dt_s);
            if load >= 3.0 {
                self.persistence[i] += 1.0 / lh_contracts::TICK_HZ as f32;
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
    fn com_feedback_preserves_the_initial_reference_at_translated_spawns() {
        for heading in [0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0] {
            let mut s = Simulation::new_playground(
                heading,
                Profile::default(),
                true,
                lh_contracts::PlaygroundSettings::default(),
            )
            .expect("translated flat fixture");
            let reference = s.rest_com;
            let reference_roundoff = s.rest_com_roundoff;
            assert_eq!(
                s.com_displacement_from(reference, reference_roundoff),
                Vector::ZERO
            );
            let displacement = Vector::new(0.001, 0.002, -0.003);
            for handle in &s.bodies {
                let body = &mut s.world.bodies[*handle];
                body.set_translation(body.translation() + displacement, false);
            }
            assert!(
                (s.com_displacement_from(reference, reference_roundoff) - displacement).length()
                    < 0.0000003
            );
            // A separately supplied absolute target must retain its literal
            // world position rather than inheriting the captured COM residual.
            let absolute_target = Vector::new(0.25, 0.75, 3.0);
            let actual = s.com_displacement_from(absolute_target, Vector::ZERO);
            let mut centroid = [0.0_f64; 3];
            let mut mass = 0.0_f64;
            for handle in &s.bodies {
                let body = &s.world.bodies[*handle];
                let weight = f64::from(body.mass());
                for (axis, coordinate) in centroid.iter_mut().enumerate() {
                    *coordinate += f64::from(body.center_of_mass()[axis]) * weight;
                }
                mass += weight;
            }
            for axis in 0..3 {
                let expected = (centroid[axis] / mass - f64::from(absolute_target[axis])) as f32;
                assert!((actual[axis] - expected).abs() < 0.0000001);
            }
            assert_eq!(
                s.rest_com, reference,
                "actual displacement must not rebase the target"
            );
            s.capture_rest_com();
            assert_eq!(
                s.com_displacement_from(s.rest_com, s.rest_com_roundoff),
                Vector::ZERO
            );
        }
    }
    #[test]
    fn mass_accounting_and_ballistic_com_are_independent_of_heading() {
        for heading in [0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0] {
            let mut s = Simulation::new(heading, Profile::default(), false).expect("fixture");
            assert!(
                (s.total_mass() - s.model.total_mass_kg).abs() < 1e-4,
                "mass counted exactly once"
            );
            assert!(
                (s.model.total_mass_kg - 40.844_02).abs() < 1e-4,
                "pinned GetUp mass"
            );
            let origin = s.center_of_mass();
            for tick in 1_u32..=30 {
                s.advance_tick().expect("ballistic integrity");
                assert_eq!(
                    s.substeps_completed,
                    u64::from(tick * lh_contracts::SUBSTEPS)
                );
                let time = tick as f32 / lh_contracts::TICK_HZ as f32;
                let com = s.center_of_mass();
                let expected = origin.y - 0.5 * 9.81 * time * time;
                // Semi-implicit fixed integration has a deterministic O(dt) position
                // offset; velocity and horizontal COM must still be ballistic.
                assert!((com.y - expected).abs() < 9.81 * time * SUBSTEP_S / 2.0 + 0.0001);
                assert!((com.x - origin.x).abs() < 0.0001 && (com.z - origin.z).abs() < 0.0001);
                let twists = getup::current_twists(&s).expect("integrated generalized velocity");
                let momentum: Vector = s
                    .bodies
                    .iter()
                    .zip(twists)
                    .map(|(h, (velocity, _))| velocity * s.world.bodies[*h].mass())
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
    fn hand_pull_changes_integrated_body_and_release_removes_external_wrench() {
        let mut baseline = Simulation::new(0.0, Profile::default(), true).expect("baseline");
        let mut pulled = Simulation::new(0.0, Profile::default(), true).expect("pulled");
        let hand = pulled
            .model
            .segments
            .iter()
            .position(|p| p.id == "right_hand")
            .unwrap();
        let torso = pulled
            .model
            .segments
            .iter()
            .position(|p| p.id == "torso")
            .unwrap();
        let origin = pulled.world.bodies[pulled.bodies[hand]].translation();
        pulled
            .begin_grab(hand, 1, Vec3::default(), vec3(origin))
            .unwrap();
        pulled.move_grab(1, vec3(origin + Vector::X * 0.2)).unwrap();
        for _ in 0..4 {
            baseline.advance_tick().expect("baseline integrity");
            pulled.advance_tick().expect("pull integrity");
        }
        let difference = |segment: usize| {
            pulled.world.bodies[pulled.bodies[segment]]
                .translation()
                .distance(baseline.world.bodies[baseline.bodies[segment]].translation())
        };
        let hand_difference = difference(hand);
        let torso_difference = difference(torso);
        assert!(
            hand_difference > 0.0001,
            "hand must physically move: {hand_difference}"
        );
        assert!(
            torso_difference > 0.000001,
            "pull must propagate: {torso_difference}"
        );
        pulled.cancel_grab();
        assert!(pulled.move_grab(1, vec3(origin)).is_err());
        for _ in 0..4 {
            pulled.advance_tick().expect("released integrity");
            assert!(pulled.grab.is_none());
            assert_eq!(
                pulled.world.bodies[pulled.bodies[hand]].user_force(),
                Vector::ZERO
            );
        }
        let next = pulled.world.bodies[pulled.bodies[hand]].translation();
        pulled
            .begin_grab(hand, 2, Vec3::default(), vec3(next))
            .expect("fresh press after release");
        assert!(
            pulled.move_grab(1, vec3(origin)).is_err(),
            "old press cannot move new grab"
        );
        pulled.cancel_grab();
        println!(
            "GetUp pull after 0.1 s: hand={hand_difference} m, torso={torso_difference} m; 0.1 s release with zero hand force"
        );
    }
    #[test]
    fn lost_grab_body_halts_and_cancels_before_any_integration() {
        let mut s = Simulation::new(0.0, Profile::default(), true).expect("fixture");
        let hand = s
            .model
            .segments
            .iter()
            .position(|part| part.id == "right_hand")
            .unwrap();
        s.begin_grab(hand, 41, Vec3::default(), Vec3::default())
            .expect("press");
        s.world
            .remove_body(s.bodies[hand])
            .expect("deliberate ownership violation");
        let failure = s.advance_tick().expect_err("lost body halts");
        assert_eq!(failure.check, "grab-ownership");
        assert_eq!(s.motion, Motion::Halted);
        assert!(s.grab.is_none());
        assert_eq!(s.substeps_completed, 0);
        assert!(s.diagnostics.delivered_motor_impulse_nm_s.is_none());
        assert!(s.diagnostics.delivered_motors.is_none());
        assert_eq!(
            s.advance_tick().expect_err("halt persists").check,
            failure.check
        );
    }
    #[test]
    fn lost_ungrabbed_body_halts_before_force_reset_or_integration() {
        let mut s = Simulation::new(0.0, Profile::default(), true).unwrap();
        let shin = s
            .model
            .segments
            .iter()
            .position(|part| part.id == "left_shin")
            .unwrap();
        s.world.remove_body(s.bodies[shin]).unwrap();
        let failure = s.advance_tick().expect_err("lost body must halt safely");
        assert_eq!(failure.check, "ownership");
        assert_eq!(failure.offending, "left_shin");
        assert_eq!(s.motion, Motion::Halted);
        assert_eq!(s.substeps_completed, 0);
        assert!(s.diagnostics.delivered_motor_impulse_nm_s.is_none());
        assert!(s.diagnostics.delivered_motors.is_none());
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
        assert_eq!(
            halted.simulation_time_s,
            1.0 / f64::from(lh_contracts::TICK_HZ * lh_contracts::SUBSTEPS)
        );
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
#[cfg(all(test, feature = "coordination-worker", feature = "numeric-parity-worker"))]
compile_error!("Native workers require separate executables and permits");
#[cfg(all(test, feature = "coordination-worker", not(feature = "coordination-transfer-worker"), not(feature = "numeric-parity-worker")))]
fn main() { recovery::planner::transition_worker::main(); }
#[cfg(all(test, feature = "coordination-transfer-worker", not(feature = "numeric-parity-worker")))]
fn main() { recovery::planner::transition_worker::transfer_main(); }
#[cfg(all(test, feature = "numeric-parity-worker", not(feature = "coordination-worker")))]
fn main() { recovery::planner::numeric_parity_worker::main(); }
