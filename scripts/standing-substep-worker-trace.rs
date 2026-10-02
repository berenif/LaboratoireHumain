// Injected by build-standing-substep-native.mjs into the isolated Rapier copy.
// Every read happens on worker 0 at an existing or explicitly-added barrier.
unsafe fn standing_trace_state(
    ctx: &SharedCtx,
    group: &super::GroupLayout,
    substep_id: usize,
    phase: &str,
) {
    if !crate::dynamics::solver::standing_trace::enabled() {
        return;
    }
    let velocity_solver = unsafe { &*ctx.velocity_solver };
    for slot in group.bodies.clone() {
        let handle = ctx.island_bodies[slot];
        let (index, generation) = handle.into_raw_parts();
        let velocity = velocity_solver.solver_bodies.vels[slot];
        crate::dynamics::solver::standing_trace::push(std::format!(
            "body\t{phase}\t{substep_id}\t{slot}\t{index}\t{generation}\t{}\t{}\t{}\t{}\t{}\t{}",
            velocity.linear.x,
            velocity.linear.y,
            velocity.linear.z,
            velocity.angular.x,
            velocity.angular.y,
            velocity.angular.z,
        ));
    }

    let joints = unsafe { &*ctx.joint_constraints };
    for (row_index, row) in joints.velocity_constraints.iter().enumerate() {
        let joint_index = row.joint_id[0];
        if joint_index >= ctx.num_joints {
            continue;
        }
        let joint = unsafe { &*ctx.joints.add(joint_index) };
        let (parent_index, parent_generation) = joint.weight.body1.into_raw_parts();
        let (child_index, child_generation) = joint.weight.body2.into_raw_parts();
        crate::dynamics::solver::standing_trace::push(std::format!(
            "joint\t{phase}\t{substep_id}\t{row_index}\t{joint_index}\t{:?}\t{parent_index}\t{parent_generation}\t{child_index}\t{child_generation}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
            row.writeback_id,
            row.impulse,
            row.impulse_bounds[0],
            row.impulse_bounds[1],
            row.rhs,
            row.rhs_wo_bias,
            row.inv_lhs,
            row.cfm_coeff,
            row.cfm_gain,
            row.solver_vel1[0],
            row.solver_vel2[0],
        ));
    }

    if !ctx.use_twist {
        for chunk_id in group.chunks.clone() {
            let constraint = unsafe { &*ctx.coulomb_constraints.add(chunk_id) };
            let slot1 = constraint.solver_vel1[0];
            let slot2 = constraint.solver_vel2[0];
            let handle_parts = |slot: u32| {
                if slot == u32::MAX {
                    (u32::MAX, u32::MAX)
                } else {
                    ctx.island_bodies[slot as usize].into_raw_parts()
                }
            };
            let (body1_index, body1_generation) = handle_parts(slot1);
            let (body2_index, body2_generation) = handle_parts(slot2);
            for point in 0..constraint.num_contacts as usize {
                let normal = &constraint.normal_part[point];
                let tangent = &constraint.tangent_part[point];
                let tangent_total = tangent.total_impulse();
                crate::dynamics::solver::standing_trace::push(std::format!(
                    "contact\t{phase}\t{substep_id}\t{chunk_id}\t{point}\t{:?}\t{slot1}\t{slot2}\t{body1_index}\t{body1_generation}\t{body2_index}\t{body2_generation}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
                    constraint.manifold_id[0],
                    normal.impulse.extract(0),
                    normal.total_impulse().extract(0),
                    tangent.impulse.x.extract(0),
                    tangent.impulse.y.extract(0),
                    tangent_total.x.extract(0),
                    tangent_total.y.extract(0),
                    normal.rhs.extract(0),
                    normal.rhs_wo_bias.extract(0),
                    normal.r.extract(0),
                    constraint.dir1.x.extract(0),
                    constraint.dir1.y.extract(0),
                    constraint.dir1.z.extract(0),
                ));
            }
        }
    } else {
        for chunk_id in group.chunks.clone() {
            let constraint = unsafe { &*ctx.twist_constraints.add(chunk_id) };
            let slot1 = constraint.solver_vel1[0];
            let slot2 = constraint.solver_vel2[0];
            let handle_parts = |slot: u32| {
                if slot == u32::MAX {
                    (u32::MAX, u32::MAX)
                } else {
                    ctx.island_bodies[slot as usize].into_raw_parts()
                }
            };
            let (body1_index, body1_generation) = handle_parts(slot1);
            let (body2_index, body2_generation) = handle_parts(slot2);
            let tangent_total = constraint.tangent_part.impulse_accumulator
                + constraint.tangent_part.impulse;
            let twist_total = constraint.twist_part.impulse_accumulator
                + constraint.twist_part.impulse;
            for point in 0..constraint.num_contacts as usize {
                let normal = &constraint.normal_part[point];
                crate::dynamics::solver::standing_trace::push(std::format!(
                    "contact\t{phase}\t{substep_id}\t{chunk_id}\t{point}\t{:?}\t{slot1}\t{slot2}\t{body1_index}\t{body1_generation}\t{body2_index}\t{body2_generation}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
                    constraint.manifold_id[0],
                    normal.impulse.extract(0),
                    normal.total_impulse().extract(0),
                    constraint.tangent_part.impulse.x.extract(0),
                    constraint.tangent_part.impulse.y.extract(0),
                    tangent_total.x.extract(0),
                    tangent_total.y.extract(0),
                    constraint.twist_part.impulse.extract(0),
                    twist_total.extract(0),
                    normal.rhs.extract(0),
                    normal.rhs_wo_bias.extract(0),
                    normal.r.extract(0),
                    constraint.dir1.x.extract(0),
                    constraint.dir1.y.extract(0),
                    constraint.dir1.z.extract(0),
                ));
            }
        }
    }
}
