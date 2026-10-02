//! Experimental SIMD solving with the admitted scalar 3D row assembly.
//! Scalar assembly preserves coordinate Jacobians, row order, per-axis motor
//! models/caps and warm-start seeds. Only body-disjoint row solves are widened.
use super::joint_constraint_builder::JointConstraintBuilder;
use super::joint_velocity_constraint::{JointConstraint, WritebackId};
use crate::dynamics::solver::joint_num_constraints;
use crate::dynamics::solver::solver_body::SolverBodies;
use crate::dynamics::{
    ImpulseJoint, IntegrationParameters, JointGraphEdge, JointIndex, RigidBodySet,
};
use crate::math::{ANG_DIM, Real, SIMD_WIDTH, SPATIAL_DIM, SimdReal, Vector};
use crate::na::SimdValue;
use crate::utils::ScalarType;

const MAX_ROWS: usize = SPATIAL_DIM + ANG_DIM;

pub(super) struct MotorJointGroup {
    builders: [JointConstraintBuilder; SIMD_WIDTH],
    rows: usize,
    live_lanes: usize,
}
impl MotorJointGroup {
    pub fn new(
        joints: &[&ImpulseJoint; SIMD_WIDTH],
        bodies: &RigidBodySet,
        ids: [JointIndex; SIMD_WIDTH],
    ) -> Self {
        let rows = joint_num_constraints(joints[0]);
        assert!(rows <= MAX_ROWS);
        Self {
            builders: core::array::from_fn(|lane| {
                JointConstraintBuilder::from_joint(joints[lane], bodies, ids[lane], 0)
            }),
            rows,
            // Short groups repeat lane zero in their padding. Assemble it
            // once; splat_row already fills all of those identical lanes.
            live_lanes: ids
                .iter()
                .enumerate()
                .skip(1)
                .find_map(|(lane, id)| (*id == ids[0]).then_some(lane))
                .unwrap_or(SIMD_WIDTH),
        }
    }
    pub fn refresh_warmstart_seeds(&mut self, joints: &[JointGraphEdge]) {
        for builder in &mut self.builders {
            builder.refresh_warmstart_seeds(joints);
        }
    }
    pub fn update(
        &mut self,
        params: &IntegrationParameters,
        substep: usize,
        warmstart: Option<Real>,
        bodies: &SolverBodies,
        out: &mut [JointConstraint<SimdReal, SIMD_WIDTH>],
    ) {
        // Preserve the solved impulses before lane 0 overwrites the output.
        let mut previous = [SimdReal::splat(0.0); MAX_ROWS];
        if warmstart.is_some() && substep > 0 {
            for (value, row) in previous[..self.rows].iter_mut().zip(out.iter()) {
                *value = row.impulse;
            }
        }
        let mut scalar = [empty_row(); MAX_ROWS];
        for (lane, builder) in self.builders[..self.live_lanes].iter().enumerate() {
            if warmstart.is_some() && substep > 0 {
                for (row, value) in scalar[..self.rows].iter_mut().zip(previous.iter()) {
                    row.impulse = value.extract(lane);
                }
            }
            builder.update(params, substep, warmstart, bodies, &mut scalar);
            for (wide, row) in out[..self.rows].iter_mut().zip(scalar.iter()) {
                if lane == 0 {
                    *wide = splat_row(row);
                } else {
                    replace_lane(wide, lane, row);
                }
            }
        }
    }
}

fn empty_row() -> JointConstraint<Real, 1> {
    JointConstraint {
        joint_id: [0],
        solver_vel1: [0],
        solver_vel2: [0],
        impulse: 0.0,
        impulse_bounds: [0.0; 2],
        lin_jac: Default::default(),
        ang_jac1: Default::default(),
        ang_jac2: Default::default(),
        ii_ang_jac1: Default::default(),
        ii_ang_jac2: Default::default(),
        inv_lhs: 0.0,
        rhs: 0.0,
        rhs_wo_bias: 0.0,
        cfm_gain: 0.0,
        cfm_coeff: 0.0,
        im1: Default::default(),
        im2: Default::default(),
        writeback_id: WritebackId::Dof(0),
    }
}
fn splat_vector(value: Vector) -> <SimdReal as ScalarType>::Vector {
    let scalar: crate::na::Vector3<Real> = value.into();
    let lanes = [scalar; SIMD_WIDTH];
    lanes.into()
}
fn splat_row(row: &JointConstraint<Real, 1>) -> JointConstraint<SimdReal, SIMD_WIDTH> {
    JointConstraint {
        joint_id: [row.joint_id[0]; SIMD_WIDTH],
        solver_vel1: [row.solver_vel1[0]; SIMD_WIDTH],
        solver_vel2: [row.solver_vel2[0]; SIMD_WIDTH],
        impulse: SimdReal::splat(row.impulse),
        impulse_bounds: row.impulse_bounds.map(SimdReal::splat),
        lin_jac: splat_vector(row.lin_jac),
        ang_jac1: splat_vector(row.ang_jac1),
        ang_jac2: splat_vector(row.ang_jac2),
        ii_ang_jac1: splat_vector(row.ii_ang_jac1),
        ii_ang_jac2: splat_vector(row.ii_ang_jac2),
        inv_lhs: SimdReal::splat(row.inv_lhs),
        rhs: SimdReal::splat(row.rhs),
        rhs_wo_bias: SimdReal::splat(row.rhs_wo_bias),
        cfm_gain: SimdReal::splat(row.cfm_gain),
        cfm_coeff: SimdReal::splat(row.cfm_coeff),
        im1: splat_vector(row.im1),
        im2: splat_vector(row.im2),
        writeback_id: row.writeback_id,
    }
}
fn replace_lane(
    wide: &mut JointConstraint<SimdReal, SIMD_WIDTH>,
    lane: usize,
    row: &JointConstraint<Real, 1>,
) {
    debug_assert_eq!(wide.writeback_id, row.writeback_id);
    wide.joint_id[lane] = row.joint_id[0];
    wide.solver_vel1[lane] = row.solver_vel1[0];
    wide.solver_vel2[lane] = row.solver_vel2[0];
    wide.impulse.replace(lane, row.impulse);
    for i in 0..2 {
        wide.impulse_bounds[i].replace(lane, row.impulse_bounds[i]);
    }
    wide.lin_jac.replace(lane, row.lin_jac.into());
    wide.ang_jac1.replace(lane, row.ang_jac1.into());
    wide.ang_jac2.replace(lane, row.ang_jac2.into());
    wide.ii_ang_jac1.replace(lane, row.ii_ang_jac1.into());
    wide.ii_ang_jac2.replace(lane, row.ii_ang_jac2.into());
    wide.inv_lhs.replace(lane, row.inv_lhs);
    wide.rhs.replace(lane, row.rhs);
    wide.rhs_wo_bias.replace(lane, row.rhs_wo_bias);
    wide.cfm_gain.replace(lane, row.cfm_gain);
    wide.cfm_coeff.replace(lane, row.cfm_coeff);
    wide.im1.replace(lane, row.im1.into());
    wide.im2.replace(lane, row.im2.into());
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::solver::solver_body::SolverVel;
    use crate::dynamics::{GenericJoint, JointAxesMask, JointAxis, MotorModel};

    #[test]
    fn wide_motor_signature_preserves_axis_and_model_groups_and_rejects_coupling() {
        let mut joint = GenericJoint::new(JointAxesMask::LIN_AXES);
        joint.set_motor_position(JointAxis::AngX, 0.2, 20.0, 40.0);
        assert!(joint.supports_simd_constraints());
        let signature = joint.simd_row_signature();
        let mut other = joint;
        other.set_motor_position(JointAxis::AngY, 0.3, 20.0, 40.0);
        assert_ne!(signature, other.simd_row_signature());
        other = joint;
        other.set_motor_model(JointAxis::AngX, MotorModel::ForceBased);
        assert_ne!(signature, other.simd_row_signature());
        other = joint;
        other.set_motor_model(JointAxis::AngZ, MotorModel::ForceBased);
        assert_eq!(signature, other.simd_row_signature());
        other = joint;
        other.coupled_axes = JointAxesMask::ANG_X | JointAxesMask::ANG_Y;
        assert!(!other.supports_simd_constraints());
        other = joint;
        other.locked_axes.remove(JointAxesMask::LIN_X);
        other.set_motor_position(JointAxis::LinX, 0.1, 10.0, 20.0);
        assert!(!other.supports_simd_constraints());
    }

    fn row(lane: usize, kind: usize) -> JointConstraint<Real, 1> {
        let mut result = empty_row();
        let sign = if lane % 2 == 0 { 1.0 } else { -1.0 };
        result.joint_id = [lane];
        result.solver_vel1 = [2 * lane as u32];
        result.solver_vel2 = [2 * lane as u32 + 1];
        result.im1 = if lane == 0 {
            Vector::ZERO
        } else {
            Vector::splat(0.8)
        };
        result.im2 = Vector::splat(0.6);
        result.lin_jac = if kind == 0 {
            Vector::ZERO
        } else {
            Vector::new(0.8, -0.2, 0.1)
        };
        result.ang_jac1 = Vector::new(0.3, 0.9, -0.1);
        result.ang_jac2 = Vector::new(0.2, 0.7, 0.3);
        result.ii_ang_jac1 = if lane == 0 {
            Vector::ZERO
        } else {
            result.ang_jac1 * 0.5
        };
        result.ii_ang_jac2 = result.ang_jac2 * 0.4;
        result.inv_lhs = 0.15;
        result.rhs = sign * (0.7 + lane as Real * 0.08);
        result.rhs_wo_bias = result.rhs * 0.5;
        result.impulse = sign * 0.02;
        match kind {
            0 => {
                result.writeback_id = WritebackId::Motor(4);
                result.impulse_bounds = [-0.12, 0.12];
                result.cfm_gain = 0.1 + lane as Real * 0.01;
            }
            1 => {
                result.writeback_id = WritebackId::Dof(0);
                result.impulse_bounds = [-Real::INFINITY, Real::INFINITY];
            }
            _ => {
                result.writeback_id = WritebackId::Limit(5);
                result.impulse_bounds = if sign > 0.0 {
                    [0.0, Real::INFINITY]
                } else {
                    [-Real::INFINITY, 0.0]
                };
            }
        }
        result
    }
    fn wide_velocity(values: &[SolverVel<Real>; SIMD_WIDTH]) -> SolverVel<SimdReal> {
        let mut result: SolverVel<SimdReal> = SolverVel::zero();
        result.linear = splat_vector(values[0].linear);
        result.angular = splat_vector(values[0].angular);
        for (lane, value) in values.iter().enumerate().skip(1) {
            result.linear.replace(lane, value.linear.into());
            result.angular.replace(lane, value.angular.into());
        }
        result
    }
    fn assert_velocity(actual: &SolverVel<SimdReal>, expected: &[SolverVel<Real>; SIMD_WIDTH]) {
        for (lane, value) in expected.iter().enumerate() {
            for axis in 0..3 {
                assert_eq!(
                    actual.linear.extract(lane)[axis].to_bits(),
                    value.linear[axis].to_bits()
                );
                assert_eq!(
                    actual.angular.extract(lane)[axis].to_bits(),
                    value.angular[axis].to_bits()
                );
            }
        }
    }
    #[test]
    fn packed_motor_lock_and_limit_solves_preserve_exact_scalar_velocities_and_caps() {
        let mut scalar: [[JointConstraint<Real, 1>; 3]; SIMD_WIDTH] =
            core::array::from_fn(|lane| core::array::from_fn(|kind| row(lane, kind)));
        let mut wide: [JointConstraint<SimdReal, SIMD_WIDTH>; 3] = core::array::from_fn(|kind| {
            let mut result = splat_row(&scalar[0][kind]);
            for (lane, rows) in scalar.iter().enumerate().skip(1) {
                replace_lane(&mut result, lane, &rows[kind]);
            }
            result
        });
        let mut first: [SolverVel<Real>; SIMD_WIDTH] = core::array::from_fn(|lane| {
            let mut value = SolverVel::zero();
            if lane > 0 {
                value.linear = Vector::new(0.1, 0.2, -0.1);
                value.angular = Vector::new(-0.2, 0.3, 0.1);
            }
            value
        });
        let mut second: [SolverVel<Real>; SIMD_WIDTH] = core::array::from_fn(|lane| {
            let mut value = SolverVel::zero();
            value.linear = Vector::new(-0.1, lane as Real * 0.1, 0.2);
            value.angular = Vector::new(0.2, -0.1, lane as Real * 0.05);
            value
        });
        let mut wide_first = wide_velocity(&first);
        let mut wide_second = wide_velocity(&second);
        for kind in 0..3 {
            for lane in 0..SIMD_WIDTH {
                scalar[lane][kind].warmstart_generic(&mut first[lane], &mut second[lane]);
            }
            wide[kind].warmstart_generic(&mut wide_first, &mut wide_second);
        }
        assert_velocity(&wide_first, &first);
        assert_velocity(&wide_second, &second);
        for pass in 0..20 {
            for kind in 0..3 {
                if pass == 10 {
                    wide[kind].remove_bias_from_rhs();
                    for rows in &mut scalar {
                        rows[kind].remove_bias_from_rhs();
                    }
                }
                for lane in 0..SIMD_WIDTH {
                    scalar[lane][kind].solve_generic(&mut first[lane], &mut second[lane]);
                }
                wide[kind].solve_generic(&mut wide_first, &mut wide_second);
                for (lane, rows) in scalar.iter().enumerate() {
                    assert_eq!(
                        wide[kind].impulse.extract(lane).to_bits(),
                        rows[kind].impulse.to_bits()
                    );
                    assert!(rows[0].impulse.abs() <= 0.12);
                }
                assert_velocity(&wide_first, &first);
                assert_velocity(&wide_second, &second);
            }
        }
    }
}
