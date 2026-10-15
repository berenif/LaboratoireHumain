//! Read-only analytic 46-coordinate task assembly for one frozen contact family.
//!
//! This is a kinematic proposal under the OBSERVED floating-base twist, not
//! inverse dynamics or a prediction of ForceBased motor/contact delivery. A
//! native preview and measured support remain mandatory before acceptance.
use super::coordinated_runtime;
use super::support_state::{BODY_COUNT, ContactMode, MaterialPatch};
use super::whole_body_tasks::{AXES, HardTask, Problem, Task};
use crate::recovery::trajectory_program::{Command, Coordinates, validate_command};
use crate::{Simulation, measurement};
use lh_model::Model;
use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};

/// Explicit measured values in canonical segment order. Replay needs neither a
/// PhysicsWorld nor a constructed Simulation. Do not normalize rotations or
/// recompute a cached world CoM while adapting a live native state.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub(crate) struct ObservedBody {
    pub position_m: [f32; 3],
    pub rotation_xyzw: [f32; 4],
    pub linear_velocity_mps: [f32; 3],
    pub angular_velocity_radps: [f32; 3],
    pub world_center_of_mass_m: [f32; 3],
    pub mass_kg: f32,
}

impl ObservedBody {
    pub(crate) fn rotation(&self) -> Rotation {
        Rotation::from_array(self.rotation_xyzw)
    }

    pub(crate) fn pose(&self) -> Pose {
        Pose::from_parts(Vector::from_array(self.position_m), self.rotation())
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub(crate) struct ObservedState {
    pub native_substep: u64,
    pub bodies: Vec<ObservedBody>,
}

impl ObservedState {
    /// Pure field copies; no pose writes, geometry query, or native step.
    pub(crate) fn from_simulation(s: &Simulation) -> Result<Self, String> {
        if s.bodies.len() != BODY_COUNT
            || s.joints.len() != BODY_COUNT
            || s.profile.dt_s != lh_contracts::SUBSTEP_S
            || s.profile.tick_substeps != 4
        {
            return Err(
                "Geometric observation requires 25 bodies and the original 240 Hz clock".into(),
            );
        }
        let bodies = s
            .bodies
            .iter()
            .map(|handle| {
                let body = s
                    .world
                    .bodies
                    .get(*handle)
                    .ok_or("Missing observed task body")?;
                Ok(ObservedBody {
                    position_m: body.translation().to_array(),
                    rotation_xyzw: body.rotation().to_array(),
                    linear_velocity_mps: body.linvel().to_array(),
                    angular_velocity_radps: body.angvel().to_array(),
                    world_center_of_mass_m: body.center_of_mass().to_array(),
                    mass_kg: body.mass(),
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        let result = Self {
            native_substep: s.substeps_completed,
            bodies,
        };
        result.validate()?;
        Ok(result)
    }

    pub(crate) fn validate(&self) -> Result<(), String> {
        if self.bodies.len() != BODY_COUNT {
            return Err("Explicit geometric state requires exactly 25 measured bodies".into());
        }
        for body in &self.bodies {
            if body
                .position_m
                .iter()
                .chain(&body.rotation_xyzw)
                .chain(&body.linear_velocity_mps)
                .chain(&body.angular_velocity_radps)
                .chain(&body.world_center_of_mass_m)
                .any(|v| !v.is_finite())
                || !body.mass_kg.is_finite()
                || body.mass_kg <= 0.0
            {
                return Err("Nonfinite or massless observed task body".into());
            }
        }
        if !self.total_mass().is_finite() || !self.center_of_mass().is_finite() {
            return Err("Nonfinite aggregated geometric observation".into());
        }
        Ok(())
    }

    /// Keep the exact f32 iterator sum used by Simulation::total_mass.
    pub(crate) fn total_mass(&self) -> f32 {
        self.bodies.iter().map(|body| body.mass_kg).sum()
    }

    /// Keep the exact f32 Vector sum, scalar sum, order, and division used by
    /// Simulation::center_of_mass. An f64/reassociated sum changes the frozen QP.
    pub(crate) fn center_of_mass(&self) -> Vector {
        let mut sum = Vector::ZERO;
        let mut mass = 0.0;
        for body in &self.bodies {
            sum += Vector::from_array(body.world_center_of_mass_m) * body.mass_kg;
            mass += body.mass_kg;
        }
        sum / mass
    }
}

#[derive(Clone, Debug)]
pub(crate) struct PointGoal {
    pub segment: usize,
    pub local_point_m: [f64; 3],
    pub world_velocity_mps: [f64; 3],
    pub weight: f64,
}

#[derive(Clone, Debug)]
pub(crate) struct OrientationGoal {
    pub segment: usize,
    pub world_angular_velocity_radps: [f64; 3],
    pub weight: f64,
}

#[derive(Clone, Debug)]
pub(crate) struct TaskGoals {
    pub nominal: Command,
    /// The caller derives this bounded velocity from its frozen CoM/capture
    /// reference. There are no hidden gains or support-centroid substitutions.
    pub com_velocity_mps: [f64; 3],
    pub com_weight: f64,
    pub point_tasks: Vec<PointGoal>,
    pub orientation_tasks: Vec<OrientationGoal>,
    pub posture_weight: f64,
    pub regularization: f64,
    /// Declared numerical/kinematic band, not a relaxed physical success gate.
    pub retained_velocity_tolerance_mps: f64,
}

#[derive(Clone, Debug)]
struct Axis {
    segment: usize,
    pivot_m: [f64; 3],
    angular: [f64; 3],
}

#[derive(Clone, Debug)]
struct Geometry {
    axes: Vec<Axis>,
    parents: [Option<usize>; BODY_COUNT],
    root_com_m: [f64; 3],
    root_linear_mps: [f64; 3],
    root_angular_radps: [f64; 3],
}

/// Body-local points and observed rotations are used directly. No pose is
/// changed and no finite-difference/FK inspection is executed by this function.
pub(crate) fn assemble(
    s: &Simulation,
    retained: &[MaterialPatch],
    goals: &TaskGoals,
) -> Result<Problem, String> {
    let observed = ObservedState::from_simulation(s)?;
    assemble_observed(&s.model, &observed, retained, goals)
}

/// Same algebra as the live adapter, operating only on archived body values.
/// Local point transforms are f64 -> f32 -> native Pose -> f64 exactly as before.
pub(crate) fn assemble_observed(
    model: &Model,
    observed: &ObservedState,
    retained: &[MaterialPatch],
    goals: &TaskGoals,
) -> Result<Problem, String> {
    assemble_with_contact_requirement(model, observed, retained, goals, true)
}

/// Experimental command guidance from current contacts, with flight allowed.
/// No contact persistence or physical admission is asserted by this function.
#[cfg(feature = "coordination-transfer-worker")]
pub(crate) fn assemble_exploratory_observed(model: &Model, observed: &ObservedState,
    current: &[MaterialPatch], goals: &TaskGoals) -> Result<Problem, String> {
    let mut problem = assemble_with_contact_requirement(model, observed, current, goals, false)?;
    for guide in std::mem::take(&mut problem.hard) {
        problem.soft.push(Task {row:guide.row, target:0.5*(guide.lower+guide.upper), weight:1.});
    }
    Ok(problem)
}

fn assemble_with_contact_requirement(model: &Model, observed: &ObservedState,
    retained: &[MaterialPatch], goals: &TaskGoals, require_contact: bool) -> Result<Problem, String> {
    let layout = coordinated_runtime::axes(model)?;
    observed.validate()?;
    validate_command(model, &goals.nominal)?;
    validate_goals(goals)?;
    if retained.len() > 64 || (require_contact && retained.is_empty()) {
        return Err(
            "Geometric tasks require the complete native model, retained contacts and 240 Hz clock"
                .into(),
        );
    }
    let geometry = measured_geometry(model, observed, &layout)?;
    let mut problem = Problem {
        lower: [-1.0; AXES],
        upper: [1.0; AXES],
        hard: Vec::new(),
        soft: Vec::new(),
        regularization: goals.regularization,
    };
    let control_dt_s = f64::from(lh_contracts::SUBSTEP_S) * 4.0;
    for (index, patch) in retained.iter().enumerate() {
        if patch.mode == ContactMode::Free {
            continue;
        }
        if patch.segment >= BODY_COUNT
            || !patch.has_current_support()
            || patch.last_observed_substep != observed.native_substep
            || !finite(patch.local_point_m)
            || !finite(patch.world_anchor_m)
            || retained[..index].iter().any(|p| p.id == patch.id)
        {
            return Err(
                "Retained task lacks a current material contact or unique patch identity".into(),
            );
        }
        let point = point_world_observed(observed, patch.segment, patch.local_point_m)?;
        let columns = geometry.point_columns(patch.segment, point)?;
        let root_velocity = geometry.root_velocity_at(point);
        // A first-order endpoint condition retains the ORIGINAL world anchor.
        // It never refreshes the anchor to an already-slipped manifold point.
        let desired = std::array::from_fn(|k| (patch.world_anchor_m[k] - point[k]) / control_dt_s);
        append_retained(
            &mut problem,
            patch.id,
            &columns,
            root_velocity,
            desired,
            goals.retained_velocity_tolerance_mps,
        )?;
    }
    if require_contact && problem.hard.is_empty() {
        return Err("No retained material task in the proposed support family".into());
    }
    let mass = f64::from(observed.total_mass());
    if !mass.is_finite() || mass <= 0.0 {
        return Err("Invalid mass in CoM task".into());
    }
    let mut com_columns = [[0.0; 3]; AXES];
    for (i, body) in observed.bodies.iter().enumerate() {
        let fraction = f64::from(body.mass_kg) / mass;
        let columns = geometry.point_columns(i, body.world_center_of_mass_m.map(f64::from))?;
        for (total, column) in com_columns.iter_mut().zip(columns) {
            for k in 0..3 {
                total[k] += column[k] * fraction;
            }
        }
    }
    let com = observed.center_of_mass().to_array().map(f64::from);
    append_soft_vector(
        &mut problem,
        &com_columns,
        geometry.root_velocity_at(com),
        goals.com_velocity_mps,
        goals.com_weight,
    )?;
    for goal in &goals.point_tasks {
        let point = point_world_observed(observed, goal.segment, goal.local_point_m)?;
        let columns = geometry.point_columns(goal.segment, point)?;
        append_soft_vector(
            &mut problem,
            &columns,
            geometry.root_velocity_at(point),
            goal.world_velocity_mps,
            goal.weight,
        )?;
    }
    for goal in &goals.orientation_tasks {
        let mut columns = [[0.0; 3]; AXES];
        for (column, axis) in columns.iter_mut().zip(&geometry.axes) {
            if geometry.descendant(goal.segment, axis.segment)? {
                *column = axis.angular;
            }
        }
        append_soft_vector(
            &mut problem,
            &columns,
            geometry.root_angular_radps,
            goal.world_angular_velocity_radps,
            goal.weight,
        )?;
    }
    // Low-priority servo reference, integrated over the same four-step clock.
    // All 46 coordinates remain decision variables; no arm/trunk is frozen.
    for (axis, entry) in layout.iter().enumerate() {
        let mut row = [0.0; AXES];
        row[axis] = 1.0;
        problem.soft.push(Task {
            row,
            target: f64::from(goals.nominal.velocities[entry.segment][entry.coordinate]),
            weight: goals.posture_weight,
        });
    }
    Ok(problem)
}

pub(crate) fn point_world(
    s: &Simulation,
    segment: usize,
    local_point_m: [f64; 3],
) -> Result<[f64; 3], String> {
    point_world_observed(&ObservedState::from_simulation(s)?, segment, local_point_m)
}

pub(crate) fn point_world_observed(
    observed: &ObservedState,
    segment: usize,
    local_point_m: [f64; 3],
) -> Result<[f64; 3], String> {
    if segment >= observed.bodies.len() || !finite(local_point_m) {
        return Err("Invalid anatomical material point".into());
    }
    let local = Vector::new(
        local_point_m[0] as f32,
        local_point_m[1] as f32,
        local_point_m[2] as f32,
    );
    let point = observed.bodies[segment].pose().transform_point(local);
    if !local.is_finite() || !point.is_finite() {
        return Err("Nonfinite transformed material point".into());
    }
    Ok(point.to_array().map(f64::from))
}

fn validate_goals(goals: &TaskGoals) -> Result<(), String> {
    if !finite(goals.com_velocity_mps)
        || !goals.com_weight.is_finite()
        || goals.com_weight < 0.0
        || !goals.posture_weight.is_finite()
        || goals.posture_weight < 0.0
        || !goals.regularization.is_finite()
        || goals.regularization <= 0.0
        || !goals.retained_velocity_tolerance_mps.is_finite()
        || goals.retained_velocity_tolerance_mps < 0.0
        || goals.point_tasks.len() > 24
        || goals.orientation_tasks.len() > 24
    {
        return Err("Invalid declared geometric task parameters".into());
    }
    for goal in &goals.point_tasks {
        if goal.segment >= BODY_COUNT
            || !finite(goal.local_point_m)
            || !finite(goal.world_velocity_mps)
            || !goal.weight.is_finite()
            || goal.weight < 0.0
        {
            return Err("Invalid point acquisition task".into());
        }
    }
    for goal in &goals.orientation_tasks {
        if goal.segment >= BODY_COUNT
            || !finite(goal.world_angular_velocity_radps)
            || !goal.weight.is_finite()
            || goal.weight < 0.0
        {
            return Err("Invalid orientation acquisition task".into());
        }
    }
    Ok(())
}

fn measured_geometry(
    model: &Model,
    observed: &ObservedState,
    layout: &[coordinated_runtime::Axis],
) -> Result<Geometry, String> {
    let root = &observed.bodies[0];
    let mut geometry = Geometry {
        axes: Vec::with_capacity(AXES),
        parents: [None; BODY_COUNT],
        root_com_m: root.world_center_of_mass_m.map(f64::from),
        root_linear_mps: root.linear_velocity_mps.map(f64::from),
        root_angular_radps: root.angular_velocity_radps.map(f64::from),
    };
    let mut measured: Coordinates = [[0.0; 3]; BODY_COUNT];
    for (i, part) in model.segments.iter().enumerate().skip(1) {
        let parent = part
            .parent
            .as_ref()
            .and_then(|id| model.segments.iter().position(|p| &p.id == id))
            .ok_or("Missing anatomical parent for geometric task")?;
        geometry.parents[i] = Some(parent);
        let profile = part
            .joint_profile
            .as_ref()
            .ok_or("Missing native joint chart")?;
        let parent_body = &observed.bodies[parent];
        let body = &observed.bodies[i];
        measured[i] = measurement::coordinates(parent_body.rotation(), body.rotation(), profile);
    }
    for axis in layout {
        let profile = model.segments[axis.segment]
            .joint_profile
            .as_ref()
            .ok_or("Missing chart profile")?;
        let parent = geometry.parents[axis.segment].ok_or("Root cannot be a command axis")?;
        let body = &observed.bodies[parent];
        let pivot = body
            .pose()
            .transform_point(crate::vector(profile.parent_frame.anchor));
        let frame = body.rotation() * crate::rotation(profile.parent_frame.rotation);
        let angular = frame * chart_axis(measured[axis.segment], axis.coordinate)?;
        if !pivot.is_finite() || !angular.is_finite() {
            return Err("Nonfinite native chart generator".into());
        }
        geometry.axes.push(Axis {
            segment: axis.segment,
            pivot_m: pivot.to_array().map(f64::from),
            angular: angular.to_array().map(f64::from),
        });
    }
    if geometry.axes.len() != AXES
        || !finite(geometry.root_com_m)
        || !finite(geometry.root_linear_mps)
        || !finite(geometry.root_angular_radps)
    {
        return Err("Invalid complete geometric chart".into());
    }
    Ok(geometry)
}

/// Spatial generator for q=normalize([tan(qx/2),tan(qy/2),tan(qz/2),1]).
/// This is the native coupled chart, not three independently rotated Euler axes.
fn chart_axis(q: [f32; 3], coordinate: usize) -> Result<Vector, String> {
    if coordinate >= 3 || q.iter().any(|v| !v.is_finite()) {
        return Err("Invalid native chart coordinate".into());
    }
    let t = Vector::new((q[0] * 0.5).tan(), (q[1] * 0.5).tan(), (q[2] * 0.5).tan());
    let e = [Vector::X, Vector::Y, Vector::Z][coordinate];
    let result =
        (e + t.cross(e)) * ((1.0 + t[coordinate] * t[coordinate]) / (1.0 + t.length_squared()));
    if !result.is_finite() {
        return Err("Nonfinite native chart derivative".into());
    }
    Ok(result)
}

impl Geometry {
    fn descendant(&self, mut segment: usize, ancestor: usize) -> Result<bool, String> {
        if segment >= BODY_COUNT || ancestor >= BODY_COUNT {
            return Err("Invalid ancestry request".into());
        }
        for _ in 0..BODY_COUNT {
            if segment == ancestor {
                return Ok(true);
            }
            let Some(parent) = self.parents[segment] else {
                return Ok(false);
            };
            if parent >= BODY_COUNT {
                return Err("Invalid anatomical parent index".into());
            }
            segment = parent;
        }
        Err("Cyclic anatomical ancestry".into())
    }

    fn point_columns(&self, segment: usize, point: [f64; 3]) -> Result<[[f64; 3]; AXES], String> {
        let mut columns = [[0.0; 3]; AXES];
        for (column, axis) in columns.iter_mut().zip(&self.axes) {
            if self.descendant(segment, axis.segment)? {
                *column = cross(axis.angular, sub(point, axis.pivot_m));
                if !finite(*column) {
                    return Err("Nonfinite anatomical point Jacobian".into());
                }
            }
        }
        Ok(columns)
    }

    fn root_velocity_at(&self, point: [f64; 3]) -> [f64; 3] {
        let angular = cross(self.root_angular_radps, sub(point, self.root_com_m));
        std::array::from_fn(|k| self.root_linear_mps[k] + angular[k])
    }
}

fn append_retained(
    problem: &mut Problem,
    patch: u64,
    columns: &[[f64; 3]; AXES],
    root_velocity: [f64; 3],
    desired: [f64; 3],
    tolerance: f64,
) -> Result<(), String> {
    if !finite(root_velocity) || !finite(desired) {
        return Err("Nonfinite retained support velocity".into());
    }
    for k in 0..3 {
        let target = desired[k] - root_velocity[k];
        let lower = target - tolerance;
        let upper = target + tolerance;
        if !target.is_finite() || !lower.is_finite() || !upper.is_finite() {
            return Err("Nonfinite retained task boundary".into());
        }
        problem.hard.push(HardTask {
            name: format!("material-patch-{patch}-world-{k}"),
            row: std::array::from_fn(|axis| columns[axis][k]),
            lower,
            upper,
        });
    }
    Ok(())
}

fn append_soft_vector(
    problem: &mut Problem,
    columns: &[[f64; 3]; AXES],
    root_velocity: [f64; 3],
    desired: [f64; 3],
    weight: f64,
) -> Result<(), String> {
    if !finite(root_velocity) || !finite(desired) {
        return Err("Nonfinite geometric objective".into());
    }
    for k in 0..3 {
        let target = desired[k] - root_velocity[k];
        if !target.is_finite() {
            return Err("Nonfinite relative geometric objective".into());
        }
        problem.soft.push(Task {
            row: std::array::from_fn(|axis| columns[axis][k]),
            target,
            weight,
        });
    }
    Ok(())
}

fn finite(v: [f64; 3]) -> bool {
    v.iter().all(|v| v.is_finite())
}
fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|k| a[k] - b[k])
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn observed_state() -> ObservedState {
        ObservedState {
            native_substep: 12,
            bodies: vec![
                ObservedBody {
                    position_m: [0.0; 3],
                    rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    linear_velocity_mps: [0.0; 3],
                    angular_velocity_radps: [0.0; 3],
                    world_center_of_mass_m: [1.0, 0.0, 0.0],
                    mass_kg: 1.0,
                };
                BODY_COUNT
            ],
        }
    }

    #[test]
    fn archived_aggregation_keeps_native_f32_order_without_a_world() {
        let mut observed = observed_state();
        observed.bodies[0].mass_kg = 16_777_216.0;
        observed.bodies[0].world_center_of_mass_m = [0.0; 3];
        observed.validate().unwrap();
        // The native f32 sequence loses each following +1. Replacing it by a
        // more accurate f64 sum would alter the already frozen problem.
        assert_eq!(observed.total_mass().to_bits(), 16_777_216.0_f32.to_bits());
        let native = 24.0_f32 / 16_777_216.0_f32;
        let reassociated = (24.0_f64 / (16_777_216.0_f64 + 24.0)) as f32;
        assert_eq!(observed.center_of_mass().x.to_bits(), native.to_bits());
        assert_ne!(native.to_bits(), reassociated.to_bits());
    }

    #[test]
    fn explicit_observation_roundtrips_without_normalizing_body_rotation() {
        let mut observed = observed_state();
        observed.bodies[3].rotation_xyzw = [0.0, 0.0, 0.0, f32::from_bits(1.0_f32.to_bits() - 1)];
        let bytes = serde_json::to_vec(&observed).unwrap();
        let decoded: ObservedState = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(
            decoded.bodies[3].rotation().to_array().map(f32::to_bits),
            observed.bodies[3].rotation_xyzw.map(f32::to_bits)
        );
        assert_eq!(decoded.native_substep, 12);
        assert_eq!(
            decoded.total_mass().to_bits(),
            observed.total_mass().to_bits()
        );
        observed.bodies.pop();
        assert!(observed.validate().is_err());
    }

    fn geometry() -> Geometry {
        let mut parents = [None; BODY_COUNT];
        parents[1] = Some(0);
        parents[2] = Some(1);
        let mut axes = vec![
            Axis {
                segment: 24,
                pivot_m: [0.0; 3],
                angular: [0.0; 3]
            };
            AXES
        ];
        axes[0] = Axis {
            segment: 1,
            pivot_m: [0.0; 3],
            angular: [0.0, 0.0, 1.0],
        };
        Geometry {
            axes,
            parents,
            root_com_m: [0.0; 3],
            root_linear_mps: [0.2, 0.0, 0.0],
            root_angular_radps: [0.0, 0.0, 0.1],
        }
    }

    #[test]
    fn analytic_columns_keep_root_motion_as_observed_state_without_integration() {
        let g = geometry();
        let columns = g.point_columns(2, [1.0, 0.0, 0.0]).unwrap();
        assert_eq!(columns[0], [0.0, 1.0, 0.0]);
        assert_eq!(columns[45], [0.0; 3]);
        assert_eq!(g.root_velocity_at([1.0, 0.0, 0.0]), [0.2, 0.1, 0.0]);
        assert_eq!(
            g.point_columns(0, [1.0, 0.0, 0.0]).unwrap(),
            [[0.0; 3]; AXES]
        );
    }

    #[test]
    fn retained_translation_is_a_hard_constraint_not_a_posture_weight_without_integration() {
        let mut problem = Problem {
            lower: [-1.0; AXES],
            upper: [1.0; AXES],
            hard: Vec::new(),
            soft: Vec::new(),
            regularization: 0.01,
        };
        let mut columns = [[0.0; 3]; AXES];
        columns[0] = [1.0, 0.0, 0.0];
        append_retained(&mut problem, 7, &columns, [0.2, 0.0, 0.0], [0.0; 3], 0.0).unwrap();
        let mut row = [0.0; AXES];
        row[0] = 1.0;
        problem.soft.push(Task {
            row,
            target: 1.0,
            weight: 1000.0,
        });
        let solution = problem.solve().unwrap();
        assert!((solution.rates[0] + 0.2).abs() < 1e-8);
        assert_eq!(problem.hard.len(), 3);
    }

    #[test]
    fn uncontrollable_root_translation_refuses_retained_task_without_integration() {
        let mut problem = Problem {
            lower: [-1.0; AXES],
            upper: [1.0; AXES],
            hard: Vec::new(),
            soft: Vec::new(),
            regularization: 0.01,
        };
        append_retained(
            &mut problem,
            1,
            &[[0.0; 3]; AXES],
            [0.1, 0.0, 0.0],
            [0.0; 3],
            0.0,
        )
        .unwrap();
        assert!(problem.solve().is_err());
    }

    #[test]
    fn native_chart_at_neutral_is_cartesian_without_integration() {
        assert_eq!(chart_axis([0.0; 3], 0).unwrap(), Vector::X);
        assert_eq!(chart_axis([0.0; 3], 1).unwrap(), Vector::Y);
        assert_eq!(chart_axis([0.0; 3], 2).unwrap(), Vector::Z);
        assert!(chart_axis([f32::NAN, 0.0, 0.0], 0).is_err());
    }
}
