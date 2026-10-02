//! One solve pass of a substep: joint colors, worker-0 joint overflow, contact
//! colors, then the worker-0 contact overflow (all completion-gated stages).

use crate::dynamics::IntegrationParameters;
use crate::dynamics::solver::JointConstraintsSet;
use crate::math::Real;

use super::{GroupLayout, SharedCtx, stage_batch};

/// Solve one scalar joint's rows with its two dynamic velocities held locally.
/// Row order, warmstart order, bias removal and all arithmetic stay identical.
/// World, aliased or mixed-body rows retain the ordinary accessor semantics.
#[cfg(feature = "experimental-joint-velocity-cache")]
fn solve_cached_joint_rows(
    rows: &mut [crate::dynamics::solver::joint_constraint::JointConstraint<Real, 1>],
    bodies: &mut crate::dynamics::solver::solver_body::SolverBodies,
    wo_bias: bool,
    warmstart: bool,
) {
    let Some(first) = rows.first() else { return };
    let a = first.solver_vel1[0];
    let b = first.solver_vel2[0];
    if a == b
        || a as usize >= bodies.len()
        || b as usize >= bodies.len()
        || rows
            .iter()
            .any(|row| row.solver_vel1 != [a] || row.solver_vel2 != [b])
    {
        for row in rows {
            if wo_bias {
                row.remove_bias_from_rhs();
            }
            if warmstart {
                row.warmstart(bodies);
            }
            row.solve(bodies);
        }
        return;
    }
    let mut va = bodies.get_vel(a);
    let mut vb = bodies.get_vel(b);
    for row in rows {
        if wo_bias {
            row.remove_bias_from_rhs();
        }
        if warmstart {
            row.warmstart_generic(&mut va, &mut vb);
        }
        row.solve_generic(&mut va, &mut vb);
    }
    bodies.set_vel(a, va);
    bodies.set_vel(b, vb);
}

/// One solve pass: joints (colored, then overflow + generic on worker 0) then
/// contact colors (parallel). Returns the caller's updated stage ordinal.
pub(super) unsafe fn solve_pass(
    ctx: &SharedCtx,
    group: &GroupLayout,
    worker_id: usize,
    mut stage: usize,
    wo_bias: bool,
    warmstart_joints: bool,
    params: &IntegrationParameters,
    solved_dt: Real,
) -> usize {
    let sync = ctx.sync;
    // Optionally, friction can be solved only in the unbiased pass
    // ("no friction when applying bias"), unless there is no unbiased pass.
    let solve_friction = wo_bias
        || ctx.base_params.friction_in_bias_pass
        || ctx.base_params.num_internal_stabilization_iterations == 0;

    // Helpers solving one scalar joint (all its rows), one SIMD joint chunk, or
    // one contact chunk.
    let solve_joint = |joint_id: usize| {
        // SAFETY: constraints of a same color (or claimed by worker 0's exclusive
        // overflow stage) touch bodies no other concurrent constraint touches.
        let vs = unsafe { &mut *ctx.velocity_solver };
        let joints = unsafe { &mut *ctx.joint_constraints };
        #[cfg(feature = "experimental-joint-velocity-cache")]
        solve_cached_joint_rows(
            &mut joints.velocity_constraints[ctx.joint_rows[joint_id].clone()],
            &mut vs.solver_bodies,
            wo_bias,
            warmstart_joints,
        );
        #[cfg(not(feature = "experimental-joint-velocity-cache"))]
        for row in ctx.joint_rows[joint_id].clone() {
            let c = &mut joints.velocity_constraints[row];
            if wo_bias {
                c.remove_bias_from_rhs();
            }
            if warmstart_joints {
                c.warmstart(&mut vs.solver_bodies);
            }
            c.solve(&mut vs.solver_bodies);
        }
    };
    let solve_joint_chunk = |chunk_id: usize| {
        // SAFETY: same argument as `solve_joint`; the lanes of a chunk share a color, so are
        // pairwise body-disjoint (padding lanes replicate lane 0 and recompute its exact
        // values, so their duplicated scatter is value-identical).
        let vs = unsafe { &mut *ctx.velocity_solver };
        let joints = unsafe { &mut *ctx.joint_constraints };
        for row in ctx.joint_chunk_rows[chunk_id].clone() {
            let c = &mut joints.simd_velocity_constraints[row];
            if wo_bias {
                c.remove_bias_from_rhs();
            }
            if warmstart_joints {
                c.warmstart(&mut vs.solver_bodies);
            }
            c.solve(&mut vs.solver_bodies);
        }
    };
    let solve_chunk = |chunk_id: usize| {
        // SAFETY: same argument as `solve_joint`.
        let solver_bodies = unsafe { &mut (*ctx.velocity_solver).solver_bodies };
        #[cfg(feature = "dim3")]
        if ctx.use_twist {
            let c = unsafe { &mut *ctx.twist_constraints.add(chunk_id) };
            if wo_bias {
                // Positions were integrated after the biased pass: refresh the unbiased rhs
                // from the current poses (reusing pre-integration separations
                // destabilizes stack rocking).
                let builder = unsafe { &*ctx.twist_builders.add(chunk_id) };
                builder.refresh_rhs_wo_bias(params, solved_dt, solver_bodies, c);
            }
            c.solve(solver_bodies, true, solve_friction);
        }
        if !ctx.use_twist {
            let c = unsafe { &mut *ctx.coulomb_constraints.add(chunk_id) };
            if wo_bias {
                let builder = unsafe { &*ctx.coulomb_builders.add(chunk_id) };
                builder.refresh_rhs_wo_bias(params, solved_dt, solver_bodies, c);
            }
            c.solve(solver_bodies, true, solve_friction);
        }
    };

    // ALL joints (colored, overflow, generic) solve BEFORE any contact in every pass: the last
    // constraint solved on a body wins its velocity residual and contacts must win, else a joint
    // re-imposed after a heavier body's contacts lets it push through (heavy cube on a spring-hung
    // ball). Generic constraints exist only single-group; vec lengths are fixed once laid out.
    let has_generic_joints = ctx.groups.len() == 1
        && !unsafe { &*ctx.joint_constraints }
            .generic_velocity_constraints
            .is_empty();
    let has_generic_contacts = ctx.groups.len() == 1
        && !unsafe { &*ctx.contact_constraints }
            .generic_velocity_constraints
            .is_empty();

    // Joint color stages (ascending color id; parallel within a color:
    // constraints of a color touch pairwise-disjoint bodies). Only this group's
    // slices of the layout tables participate.
    for (_, joint_range) in &ctx.joint_color_ranges[group.joint_colors.clone()] {
        let virt = 0..joint_range.end - joint_range.start;
        let mut done = 0;
        while let Some(claimed) = sync.claim(
            stage,
            &virt,
            stage_batch(virt.end, sync.num_workers),
            worker_id,
        ) {
            let claimed_len = claimed.len();
            for idx in claimed {
                solve_joint_chunk(joint_range.start + idx);
                // Without SIMD, the joint color ranges hold scalar builders.
            }
            done += claimed_len;
        }
        // One completion flush per worker per stage: per-batch RMWs on the
        // shared counter measurably throttle scalar builds (4x the chunks).
        sync.complete(stage, done, virt.end);
        stage = sync.sync(stage, virt.end);
    }

    // Overflow + generic joints, solved exclusively by worker 0 (they write body
    // velocities, so no other worker may access solver bodies concurrently: they
    // wait at the barrier). Skipped entirely when empty (the jointless case).
    if !group.joint_overflow.is_empty() || has_generic_joints {
        if worker_id == 0 {
            for joint_id in group.joint_overflow.clone() {
                solve_joint(joint_id);
            }

            if has_generic_joints {
                let joints = unsafe { &mut *ctx.joint_constraints };
                let vs = unsafe { &mut *ctx.velocity_solver };
                let JointConstraintsSet {
                    generic_jacobians,
                    generic_velocity_constraints,
                    ..
                } = joints;
                for c in generic_velocity_constraints.iter_mut() {
                    if wo_bias {
                        c.remove_bias_from_rhs();
                    }
                    c.solve(
                        generic_jacobians,
                        &mut vs.solver_bodies,
                        &mut vs.generic_solver_vels,
                    );
                }
            }
            sync.complete(stage, 1, 1);
        }
        stage = sync.sync(stage, 1);
    }

    // Contact color stages (ascending color id).
    for (_, chunk_range) in &ctx.color_ranges[group.colors.clone()] {
        let virt = 0..chunk_range.end - chunk_range.start;
        let mut done = 0;
        while let Some(claimed) = sync.claim(
            stage,
            &virt,
            stage_batch(virt.end, sync.num_workers),
            worker_id,
        ) {
            let claimed_len = claimed.len();
            for idx in claimed {
                solve_chunk(chunk_range.start + idx);
            }
            done += claimed_len;
        }
        // One completion flush per worker per stage (see the joint stages above).
        sync.complete(stage, done, virt.end);
        stage = sync.sync(stage, virt.end);
    }

    // Overflow + generic contacts, solved exclusively by worker 0. Unlike the
    // joint overflow stage this one is unconditional: it doubles as the pass'
    // trailing barrier so every worker leaves solve_pass in lockstep.
    if worker_id == 0 {
        for chunk_id in group.overflow.clone() {
            solve_chunk(chunk_id);
        }

        if has_generic_contacts {
            let contacts = unsafe { &mut *ctx.contact_constraints };
            let vs = unsafe { &mut *ctx.velocity_solver };
            let jac = &contacts.generic_jacobians;
            for c in contacts.generic_velocity_constraints.iter_mut() {
                if wo_bias {
                    c.remove_cfm_and_bias_from_rhs();
                }
                c.solve(
                    jac,
                    &mut vs.solver_bodies,
                    &mut vs.generic_solver_vels,
                    true,
                    solve_friction,
                );
            }
        }
        sync.complete(stage, 1, 1);
    }
    sync.sync(stage, 1)
}

#[cfg(all(test, feature = "dim3", feature = "experimental-joint-velocity-cache"))]
mod velocity_cache_tests {
    use super::solve_cached_joint_rows;
    use crate::alloc_prelude::Vec;
    use crate::dynamics::solver::joint_constraint::{JointConstraint, WritebackId};
    use crate::dynamics::solver::solver_body::SolverBodies;
    use crate::math::{Real, Vector};

    fn check_exact(ids: [u32; 2], mixed: bool) {
        let mut original = SolverBodies::default();
        original.resize(3);
        for (i, v) in original.vels.iter_mut().enumerate() {
            v.linear = Vector::new(i as Real * 0.17, -0.23, 0.08);
            v.angular = Vector::new(0.31, i as Real * -0.12, -0.07);
        }
        let mut cached = SolverBodies::default();
        cached.resize(3);
        cached.vels.clone_from(&original.vels);
        let mut rows: Vec<_> = (0..9)
            .map(|i| JointConstraint {
                solver_vel1: [ids[0]],
                solver_vel2: [ids[1]],
                joint_id: [0],
                impulse: (i as Real - 4.0) * 0.001,
                impulse_bounds: if i % 3 == 0 {
                    [-0.012, 0.012]
                } else {
                    [-1.0, 1.0]
                },
                lin_jac: Vector::new(0.1, 0.2, -0.3),
                ang_jac1: Vector::new(0.2, i as Real * 0.01, -0.12),
                ang_jac2: Vector::new(-0.08, 0.11, i as Real * 0.02),
                ii_ang_jac1: Vector::new(0.02, 0.01, -0.012),
                ii_ang_jac2: Vector::new(-0.008, 0.011, 0.02),
                inv_lhs: 0.13,
                rhs: (i as Real - 5.0) * 0.07,
                rhs_wo_bias: (i as Real - 3.0) * 0.03,
                cfm_gain: 0.04,
                cfm_coeff: 0.09,
                im1: Vector::splat(0.4),
                im2: Vector::splat(0.7),
                writeback_id: if i % 3 == 0 {
                    WritebackId::Motor(3)
                } else if i % 3 == 1 {
                    WritebackId::Limit(4)
                } else {
                    WritebackId::Dof(0)
                },
            })
            .collect();
        if mixed {
            rows[4].solver_vel2 = [2];
        }
        let mut cached_rows = rows.clone();
        for pass in 0..40 {
            let wo_bias = pass % 2 == 1;
            let warmstart = pass % 3 == 0;
            for row in &mut rows {
                if wo_bias {
                    row.remove_bias_from_rhs();
                }
                if warmstart {
                    row.warmstart(&mut original);
                }
                row.solve(&mut original);
            }
            solve_cached_joint_rows(&mut cached_rows, &mut cached, wo_bias, warmstart);
            for (a, b) in rows.iter().zip(&cached_rows) {
                assert_eq!(a.impulse.to_bits(), b.impulse.to_bits());
                assert_eq!(a.rhs.to_bits(), b.rhs.to_bits());
                assert!(b.impulse >= b.impulse_bounds[0] && b.impulse <= b.impulse_bounds[1]);
            }
            for (a, b) in original.vels.iter().zip(&cached.vels) {
                for (a, b) in a
                    .linear
                    .to_array()
                    .into_iter()
                    .chain(a.angular.to_array())
                    .zip(b.linear.to_array().into_iter().chain(b.angular.to_array()))
                {
                    assert_eq!(a.to_bits(), b.to_bits());
                }
            }
        }
    }

    #[test]
    fn dynamic_rows_preserve_every_impulse_and_velocity_bit() {
        check_exact([0, 1], false);
    }

    #[test]
    fn fallback_rows_preserve_world_alias_and_mixed_body_semantics() {
        check_exact([0, u32::MAX], false);
        check_exact([1, 1], false);
        check_exact([0, 1], true);
    }

    #[test]
    fn empty_rows_are_a_noop() {
        let mut bodies = SolverBodies::default();
        solve_cached_joint_rows(&mut [], &mut bodies, true, true);
        assert!(bodies.vels.is_empty());
    }
}
