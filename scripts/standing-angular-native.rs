use rapier3d::prelude::*;
use rapier3d::dynamics::solver::{AngularLimitParams, JointConstraint, JointConstraintHelper, JointSolverBody, WritebackId};
use serde_json::{json, Value};
use std::io::Write;

#[repr(C)]
#[derive(Default)]
struct ProcessMemory { cb:u32, page_faults:u32, peak_rss:usize, rss:usize, peak_paged:usize, paged:usize,
    peak_nonpaged:usize, nonpaged:usize, pagefile:usize, peak_pagefile:usize }
#[link(name="psapi")]
unsafe extern "system" { fn GetProcessMemoryInfo(process:isize, counters:*mut ProcessMemory, cb:u32)->i32; }
fn memory() -> Value {
    let mut counters = ProcessMemory::default(); counters.cb=std::mem::size_of::<ProcessMemory>() as u32;
    assert_ne!(unsafe {GetProcessMemoryInfo(-1,&mut counters,std::mem::size_of::<ProcessMemory>() as u32)},0);
    assert!(counters.peak_rss <= 768*1024*1024);
    json!({"rss":counters.rss,"peakRss":counters.peak_rss,"method":"Windows GetProcessMemoryInfo"})
}

fn body(rb: &RigidBody) -> JointSolverBody<f32, 1> {
    let mp = rb.mass_properties();
    JointSolverBody { im: mp.effective_inv_mass, ii: mp.effective_world_inv_inertia,
        world_com: mp.world_com, solver_vel: [0] }
}
fn row(r: &JointConstraint<f32, 1>) -> Value {
    json!({"kind": format!("{:?}", r.writeback_id), "linearJacobian": r.lin_jac,
        "parentAngularJacobian": r.ang_jac1, "childAngularJacobian": r.ang_jac2,
        "parentInverseInertiaJacobian": r.ii_ang_jac1, "childInverseInertiaJacobian": r.ii_ang_jac2,
        "rhs": r.rhs, "rhsWithoutBias": r.rhs_wo_bias, "inverseLhs": r.inv_lhs,
        "cfmCoefficient": r.cfm_coeff, "cfmGain": r.cfm_gain,
        "impulseBounds": r.impulse_bounds.map(|v| if v.is_infinite() {v.signum()*f32::MAX} else {v}),
        "infiniteBounds": r.impulse_bounds.map(|v| v.is_infinite())})
}
fn inspect(input: &str) -> Value {
    let bytes = std::fs::read(input).unwrap();
    let world: PhysicsWorld = bincode::deserialize(&bytes).unwrap();
    let sections = [bincode::serialize(&world.gravity).unwrap(), bincode::serialize(&world.integration_parameters).unwrap(),
        bincode::serialize(&world.islands).unwrap(), bincode::serialize(&world.broad_phase).unwrap(),
        bincode::serialize(&world.narrow_phase).unwrap(), bincode::serialize(&world.bodies).unwrap(),
        bincode::serialize(&world.colliders).unwrap(), bincode::serialize(&world.impulse_joints).unwrap(),
        bincode::serialize(&world.multibody_joints).unwrap()];
    let joint_value = serde_json::to_value(&world.impulse_joints).unwrap();
    let mut wakes: Vec<RigidBodyHandle> = serde_json::from_value(joint_value["to_wake_up"].clone()).unwrap();
    let mut joins: Vec<(RigidBodyHandle,RigidBodyHandle)> = serde_json::from_value(joint_value["to_join"].clone()).unwrap();
    let joint_start: usize = sections[..7].iter().map(Vec::len).sum();
    let joint_end = joint_start + sections[7].len();
    let join_start = joint_end - bincode::serialize(&joins).unwrap().len();
    let wake_start = join_start - bincode::serialize(&wakes).unwrap().len();
    let mut original_wakes: Vec<RigidBodyHandle> = bincode::deserialize(&bytes[wake_start..join_start]).unwrap();
    let mut original_joins: Vec<(RigidBodyHandle,RigidBodyHandle)> = bincode::deserialize(&bytes[join_start..joint_end]).unwrap();
    wakes.sort_by_key(|h|h.into_raw_parts()); original_wakes.sort_by_key(|h|h.into_raw_parts());
    joins.sort_by_key(|(a,b)|(a.into_raw_parts(),b.into_raw_parts())); original_joins.sort_by_key(|(a,b)|(a.into_raw_parts(),b.into_raw_parts()));
    let serialized: Vec<u8> = sections.into_iter().flatten().collect();
    assert_eq!(bytes[..wake_start], serialized[..wake_start]); assert_eq!(bytes[joint_end..], serialized[joint_end..]);
    assert_eq!(wakes,original_wakes); assert_eq!(joins,original_joins);
    let p = world.integration_parameters;
    // Rapier subdivides dt by num_solver_iterations for row construction.
    let mut substep_params = p; substep_params.dt /= p.num_solver_iterations as f32;
    let joints: Vec<_> = world.impulse_joints.iter().map(|(handle,joint)| {
        let parent = &world.bodies[joint.body1()]; let child = &world.bodies[joint.body2()];
        let b1 = body(parent); let b2 = body(child);
        let f1 = parent.position()*joint.data.local_frame1; let f2 = child.position()*joint.data.local_frame2;
        let h = JointConstraintHelper::<f32>::new(&f1,&f2,&b1.world_com,&b2.world_com,joint.data.locked_axes.bits());
        let relative = h.ang_err;
        let coords: Vec<_> = (0..3).map(|axis| {
            let v = [relative.x,relative.y,relative.z][axis];
            json!({"axis":axis+3, "limitCoordinate":h.recentered_angle(axis,&AngularLimitParams::new(0.0,0.0)),
                "motorCoordinate":2.0*v.clamp(-1.0,1.0).asin()})
        }).collect();
        let limits = std::array::from_fn(|axis| AngularLimitParams::new(joint.data.limits[axis+3].min,joint.data.limits[axis+3].max));
        let raw: Vec<_> = (3..6).flat_map(|axis| {
            let mut rows = Vec::new(); let bit = 1<<axis;
            if joint.data.locked_axes.bits() & bit != 0 {
                rows.push(row(&h.lock_angular(&substep_params,[0],&b1,&b2,axis-3,WritebackId::Dof(axis),joint.data.softness.erp_inv_dt(substep_params.dt),joint.data.softness.cfm_coeff(substep_params.dt))));
            } else {
                if joint.data.motor_axes.bits() & bit != 0 { rows.push(row(&h.motor_angular([0],&b1,&b2,axis-3,&joint.data.motors[axis].motor_params(substep_params.dt),WritebackId::Motor(axis)))); }
                if joint.data.limit_axes.bits() & bit != 0 { rows.push(row(&h.limit_angular(&substep_params,[0],&b1,&b2,axis-3,limits[axis-3],WritebackId::Limit(axis),joint.data.softness.erp_inv_dt(substep_params.dt),joint.data.softness.cfm_coeff(substep_params.dt)))); }
            } rows
        }).collect();
        let seed = h.lock_linear(&substep_params,[0],&b1,&b2,0,WritebackId::Dof(0),0.0,0.0);
        let mut rows = vec![seed;24];
        let count = JointConstraint::<f32, 1>::update(&substep_params,0,&b1,&b2,&f1,&f2,&joint.data,&limits,&mut rows);
        let motors: Vec<_> = joint.data.motors.iter().enumerate().map(|(i,m)| json!({"axis":i,"targetPosition":m.target_pos,"targetVelocity":m.target_vel,"stiffness":m.stiffness,"damping":m.damping,"maxForce":m.max_force,"storedImpulse":m.impulse,"model":m.model})).collect();
        json!({"handle":handle.into_raw_parts(),"parent":joint.body1().into_raw_parts(),"child":joint.body2().into_raw_parts(),
            "parentFrame":f1,"childFrame":f2,"relativeQuaternion":[relative.x,relative.y,relative.z,relative.w],
            "lockedAxes":joint.data.locked_axes.bits(),"motorAxes":joint.data.motor_axes.bits(),"limitAxes":joint.data.limit_axes.bits(),"coupledAxes":joint.data.coupled_axes.bits(),
            "limits":joint.data.limits,"motors":motors,"storedDofImpulses":joint.impulses,
            "coordinates":coords,"rawAngularRows":raw,"finalizedRows":rows[..count].iter().map(row).collect::<Vec<_>>()})
    }).collect();
    json!({"input":input,"serializationPayloadExact":true,"unorderedMembersExact":true,
        "integrationParameters":p,"rowDt":substep_params.dt,"joints":joints,
        "qualification":"Frozen-pose row reconstruction using native helper/update including row orthogonalization. Not the transient rows solved during WASM integration. Stored impulses belong to the snapshot and are not whole-step delivered torques."})
}
fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(),3,"standing-angular-native input.bin output.json");
    let mut result = inspect(&args[1]);
    result["processMemory"] = memory();
    let mut out=std::fs::OpenOptions::new().write(true).create_new(true).open(&args[2]).unwrap();
    writeln!(out,"{}",serde_json::to_string_pretty(&result).unwrap()).unwrap();
}
