//! Explicit whole-history worker. Never runs through production or normal tests.
use super::{contacts, full_trajectory, observe, velocity};
use crate::{Simulation, measurement, recovery};
use lh_contracts::Motion;
use rapier3d::prelude::Rotation;
use serde::Serialize;
use serde_json::{Value, json};
use std::io::{BufWriter, Write};

#[derive(Default)]
struct StateHash(u64);
impl Write for StateHash {
    fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
        for byte in bytes {
            self.0 = (self.0 ^ u64::from(*byte)).wrapping_mul(1099511628211);
        }
        Ok(bytes.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}
fn hash<T: Serialize>(value: &T) -> String {
    let mut sink = StateHash(14695981039346656037);
    serde_json::to_writer(&mut sink, value).unwrap();
    format!("{:016x}", sink.0)
}

#[test]
#[ignore = "explicit bounded campaign worker; opt-in reference and new output required"]
fn run_whole_reference_native_history() {
    let output = std::path::PathBuf::from(std::env::var("LH_WHOLE_OUT").unwrap());
    assert!(!output.exists(), "Do not overwrite a native history");
    let source: recovery::trajectory_program::Program =
        serde_json::from_slice(&std::fs::read(std::env::var("LH_WHOLE_PROGRAM").unwrap()).unwrap())
            .unwrap();
    let profile = crate::recovery_contract::pinned_profile();
    let fixture = serde_json::from_str::<recovery::FixtureSet>(recovery::FIXTURES)
        .unwrap()
        .cases
        .into_iter()
        .find(|f| f.id == "landed-crouch-left")
        .unwrap();
    let heading = std::env::var("LH_WHOLE_HEADING_DEG")
        .ok()
        .map(|s| s.parse::<f32>().unwrap().to_radians())
        .unwrap_or(fixture.heading);
    let perturb = std::env::var("LH_WHOLE_PERTURB_DEG")
        .ok()
        .map(|s| s.parse::<f32>().unwrap().to_radians())
        .unwrap_or(0.0);
    assert!(heading == 0.0 || (heading - std::f32::consts::FRAC_PI_3).abs() < 1e-6);
    assert!(perturb == 0.0 || (perturb - 2.0_f32.to_radians()).abs() < 1e-6);
    let idle_validation = std::env::var("LH_WHOLE_IDLE_VALIDATION").is_ok_and(|s| s == "1");
    let controller_control = std::env::var("LH_WHOLE_CONTROLLER_CONTROL").is_ok_and(|s| s == "1");
    let mut poses = fixture.poses.clone();
    // Optional qualification perturbation is part of the initial fixture only.
    if heading != fixture.heading || perturb != 0.0 {
        let rotate = Rotation::from_rotation_y(heading - fixture.heading)
            * Rotation::from_rotation_x(perturb);
        for p in &mut poses {
            p.position = crate::vec3(rotate * crate::vector(p.position));
            p.rotation = crate::quat(rotate * crate::rotation(p.rotation));
            p.linear_velocity = crate::vec3(rotate * crate::vector(p.linear_velocity));
            p.angular_velocity = crate::vec3(rotate * crate::vector(p.angular_velocity));
        }
    }
    let mut s = Simulation::build(heading, profile, true, None, Some(&poses)).unwrap();
    let program = source.compile(&s.model).unwrap();
    let duration = program.duration_substeps();
    // No rest-target rebasing, pose transplant, simulation fork or reset.
    let initial = s.snapshot();
    let bodies = s.bodies.clone();
    let joints = s.joints.clone();
    let colliders = s.colliders.clone();
    let generation = s.generation;
    std::fs::create_dir_all(&output).unwrap();
    let mut numeric = BufWriter::new(std::fs::File::create(output.join("substeps.jsonl")).unwrap());
    let mut frames = BufWriter::new(std::fs::File::create(output.join("frames.jsonl")).unwrap());
    let started = std::time::Instant::now();
    let wall_limit = std::env::var("LH_WHOLE_WALL_S")
        .ok()
        .map(|v| v.parse::<f64>().unwrap())
        .unwrap_or(14400.0);
    assert!(wall_limit > 0.0 && wall_limit <= 14400.0);
    let mut prior = velocity(&s);
    let mut first_recovery = None;
    let mut first_standing = None;
    let mut minimum_height = s.world.bodies[s.bodies[0]].translation().y;
    let mut hold = 0_u64;
    let mut failure = None;
    let mut observer_steps = 0_u64;
    let mut maximum_loads = [0.0_f64; 25];
    let mut maximum_acceleration = 0.0_f32;
    let mut maximum_delivered = 0.0_f32;
    let mut maximum_actual_ratio = 0.0_f32;
    let contract = crate::recovery_contract::contract();
    let limit = contract.acceptance.recovery_limit_s + contract.acceptance.stable_observation_s;
    let required_hold =
        crate::recovery_contract::substeps(contract.acceptance.stable_observation_s);
    if !controller_control {
        full_trajectory::activate_history(program, s.substeps_completed);
    }
    for _ in 0..(limit * 60.0).ceil() as usize {
        let result = s.advance_tick_observed(|state, substep| {
            if state.bodies != bodies || state.joints != joints || state.colliders != colliders
                || state.generation != generation
            {
                return Err(measurement::Failure::new(state.tick, substep, "whole-history-ownership",
                    "native assembly changed".into(), 1.0, 0.0));
            }
            observer_steps += 1;
            let time = state.substeps_completed as f64 / 240.0;
            let height = state.world.bodies[state.bodies[0]].translation().y - state.support_height();
            minimum_height = minimum_height.min(height);
            let current = velocity(state);
            let acceleration = (current - prior) / state.profile.dt_s;
            maximum_acceleration = maximum_acceleration.max(acceleration.length());
            let resolved = contacts(state);
            let mut loads = [0.0_f64; 25];
            for c in &resolved {
                let i = state.model.segments.iter().position(|p| p.id == c["id"]).unwrap();
                loads[i] += c["normalLoadN"].as_f64().unwrap();
            }
            for i in 0..25 {maximum_loads[i] = maximum_loads[i].max(loads[i]);}
            let mut delivered = 0.0_f32;
            let mut cap_sum = 0.0_f32;
            for (i, h) in state.joints.iter().enumerate() {
                let Some(joint) = h.and_then(|h| state.world.impulse_joints.get(h)) else {continue};
                for axis in &state.model.segments[i].joint_profile.as_ref().unwrap().axes {
                    let k = match axis.coordinate {'x'=>0,'y'=>1,_=>2};
                    let motor = &joint.data.motors[k+3];
                    delivered += motor.delivered_abs_impulse/state.profile.dt_s;
                    cap_sum += motor.max_force;
                    maximum_actual_ratio = maximum_actual_ratio.max(motor.max_force/axis.max_motor_torque_nm);
                }
            }
            maximum_delivered = maximum_delivered.max(delivered);
            let standing = height > 0.93 && measurement::standing_support(state,substep).is_ok();
            // A standing start in the crouch cannot substitute for the full reference route.
            let route_standing = standing && minimum_height < 0.60;
            if route_standing && first_standing.is_none() {first_standing = Some(time);}
            if state.counters.recoveries > 0 && first_recovery.is_none() {first_recovery = Some(time);}
            let still = state.bodies.iter().all(|h| {
                let b = &state.world.bodies[*h];
                b.linvel().length() <= contract.recovery_acceptance.stable_linear_mps
                    && b.angvel().length() <= contract.recovery_acceptance.stable_angular_radps
            });
            let original_hold = route_standing && still && state.motion == Motion::Upright
                && state.counters.recoveries == 1;
            hold = if original_hold {hold+1} else {0};
            let row = json!({"substep":state.substeps_completed,"timeS":time,"heightM":height,
                "com":crate::vec3(state.center_of_mass()),"velocity":crate::vec3(current),
                "acceleration":crate::vec3(acceleration),"loadsN":loads,"capSumNm":cap_sum,
                "deliveredAbsoluteNm":delivered,"standing":standing,"routeStanding":route_standing,
                "quietHoldSubsteps":hold,"recoveries":state.counters.recoveries,"motion":state.motion,
                "bodiesHash":hash(&state.world.bodies),"motorsHash":hash(&state.world.impulse_joints),
                "contactsHash":hash(&state.world.narrow_phase)});
            serde_json::to_writer(&mut numeric,&row).unwrap();numeric.write_all(b"\n").unwrap();
            if state.substeps_completed == 1 || state.substeps_completed % 12 == 0 {
                let record = json!({"physical":observe(state,prior),"resolvedLoadsN":loads});
                serde_json::to_writer(&mut frames,&record).unwrap();frames.write_all(b"\n").unwrap();
            }
            prior=current;
            if started.elapsed().as_secs_f64() >= wall_limit {
                return Err(measurement::Failure::new(state.tick,substep,"whole-history-wall-budget",
                    "closed campaign wall budget reached".into(),started.elapsed().as_secs_f32(),wall_limit as f32));
            }
            Ok(())
        });
        if let Err(f) = result {
            failure = Some(f);
            break;
        }
        if hold >= required_hold {
            break;
        }
    }
    let (released, cap, ratio) = full_trajectory::finish_history();
    numeric.flush().unwrap();
    frames.flush().unwrap();
    let success = failure.is_none()
        && s.counters.recoveries == 1
        && s.motion == Motion::Upright
        && first_recovery.is_some_and(|t| t <= contract.acceptance.recovery_limit_s)
        && hold >= required_hold
        && first_standing.is_some();
    let mut idle_failure = None;
    let mut idle_steps = 0_u64;
    let mut quiet = measurement::QuietMetrics::default();
    if success && idle_validation {
        // Keep original terminal conditions throughout the same-world idle check.
        let terminal = |state: &Simulation, substep| {
            if state.motion != Motion::Upright
                || state.counters.recoveries != 1
                || state.world.bodies[state.bodies[0]].translation().y - state.support_height()
                    <= 0.93
            {
                return Err(measurement::Failure::new(
                    state.tick,
                    substep,
                    "whole-idle-terminal",
                    "original upright terminal lost".into(),
                    1.0,
                    0.0,
                ));
            }
            measurement::standing_support(state, substep)
        };
        for _ in 0..(contract.acceptance.idle_settling_s * 60.0).ceil() as usize {
            if let Err(f) = s.advance_tick_observed(|state, substep| {
                if state.bodies != bodies
                    || state.joints != joints
                    || state.colliders != colliders
                    || state.generation != generation
                {
                    return Err(measurement::Failure::new(
                        state.tick,
                        substep,
                        "whole-idle-ownership",
                        "native assembly changed".into(),
                        1.0,
                        0.0,
                    ));
                }
                terminal(state, substep)
            }) {
                idle_failure = Some(f);
                break;
            }
        }
        let rest = s.snapshot();
        if idle_failure.is_none() {
            for _ in 0..(contract.acceptance.idle_duration_s * 60.0).ceil() as usize {
                if let Err(f) = s.advance_tick_observed(|state,substep| {
                    if state.bodies!=bodies || state.joints!=joints || state.colliders!=colliders
                        || state.generation!=generation {
                        return Err(measurement::Failure::new(state.tick,substep,"whole-idle-ownership",
                            "native assembly changed".into(),1.0,0.0));
                    }
                    terminal(state,substep)?;
                    quiet.observe(state,&rest,substep)?;
                    idle_steps+=1;
                    let row=json!({"substep":state.substeps_completed,"idle":true,
                        "bodiesHash":hash(&state.world.bodies),"motorsHash":hash(&state.world.impulse_joints),
                        "contactsHash":hash(&state.world.narrow_phase)});
                    serde_json::to_writer(&mut numeric,&row).unwrap();numeric.write_all(b"\n").unwrap();
                    if state.substeps_completed%12==0 {
                        serde_json::to_writer(&mut frames,&json!({"physical":observe(state,prior),"idle":true})).unwrap();
                        frames.write_all(b"\n").unwrap();
                    }
                    prior=velocity(state);
                    Ok(())
                }) {idle_failure=Some(f);break;}
            }
        }
    }
    numeric.flush().unwrap();
    frames.flush().unwrap();
    let report: Value = json!({"scope":"Whole actual native history, same common command interpreter",
        "physicalSubsteps":s.substeps_completed,"observedValidSubsteps":observer_steps,
        "physicalSeconds":s.substeps_completed as f64/240.0,"wallSeconds":started.elapsed().as_secs_f64(),
        "firstRefusal":failure,"fullRecoverySuccess":success,"firstNativeRecoveryS":first_recovery,
        "headingRadians":heading,"initialPerturbationDegrees":perturb.to_degrees(),
        "originalControllerControl":controller_control,
        "idleValidationRequested":idle_validation,"idleFailure":idle_failure,"idleObservedSubsteps":idle_steps,
        "idleMetrics":quiet,"validationPassed":success&&(!idle_validation || idle_failure.is_none()&&idle_steps>=crate::recovery_contract::substeps(contract.acceptance.idle_duration_s)),
        "firstRouteStandingS":first_standing,"minimumPelvisHeightM":minimum_height,
        "quietHoldSubsteps":hold,"requiredHoldSubsteps":required_hold,"programDurationS":duration as f64/240.0,
        "releasedToOriginal":released,"maximumAllocatedCapSumNm":cap,"maximumAnatomicalCapRatio":ratio,
        "maximumActualAnatomicalRatio":maximum_actual_ratio,"maximumDeliveredAbsoluteNm":maximum_delivered,
        "maximumComAccelerationMps2":maximum_acceleration,"maximumLoadsN":maximum_loads,
        "nativeHandlesAndGenerationPreserved":true,"noBodyCommandsOrWorldReset":true,
        "initial":initial,"final":s.snapshot(),"originalRecoveries":s.counters.recoveries,
        "qualification":"Four independent final validations still required after frozen success"});
    std::fs::write(
        output.join("report.json"),
        serde_json::to_vec_pretty(&report).unwrap(),
    )
    .unwrap();
    println!("{}", serde_json::to_string(&report).unwrap());
}
