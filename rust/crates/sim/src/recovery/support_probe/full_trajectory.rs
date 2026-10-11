//! One motor command installer for full-program search and replay.
//! Not connected to the production controller or a search campaign.
use super::offline_trajectory::{Allocation, install_targets};
use crate::{Simulation, recovery::trajectory_program::CompiledProgram};
use serde_json::Value;
use std::cell::{Cell, RefCell};

thread_local! { static ACTIVE: RefCell<Option<(CompiledProgram,u64)>> = const { RefCell::new(None) }; }
thread_local! {
    static AUTO_RELEASE: Cell<bool> = const { Cell::new(false) };
    static RELEASED: Cell<bool> = const { Cell::new(false) };
    static CAP_STATS: RefCell<(f64,f32)> = const { RefCell::new((0.0,0.0)) };
}

pub(super) fn activate_history(program: CompiledProgram, start: u64) {
    ACTIVE.with(|slot| {
        assert!(slot.borrow().is_none());
        *slot.borrow_mut() = Some((program, start));
    });
    AUTO_RELEASE.set(true);
    RELEASED.set(false);
    CAP_STATS.with(|slot| *slot.borrow_mut() = (0.0, 0.0));
}

pub(super) fn finish_history() -> (bool, f64, f32) {
    ACTIVE.with(|slot| slot.borrow_mut().take());
    AUTO_RELEASE.set(false);
    let (cap, ratio) = CAP_STATS.with(|slot| *slot.borrow());
    (RELEASED.get(), cap, ratio)
}

pub(crate) fn before_step(
    s: &mut Simulation,
    substep: u32,
) -> Result<(), crate::measurement::Failure> {
    ACTIVE.with(|slot| {
        let active = slot.borrow();
        let Some((program, start)) = active.as_ref() else {
            return Ok(());
        };
        if AUTO_RELEASE.get()
            && (RELEASED.get()
                || s.counters.recoveries > 0
                || s.recovery.as_ref().is_some_and(|r| r.handed_off()))
        {
            RELEASED.set(true);
            return Ok(());
        }
        let command_step =
            u32::try_from(s.substeps_completed.checked_sub(*start).ok_or_else(|| {
                crate::measurement::Failure::new(
                    s.tick,
                    substep,
                    "program-clock",
                    "clock rewind".into(),
                    1.0,
                    0.0,
                )
            })?)
            .map_err(|_| {
                crate::measurement::Failure::new(
                    s.tick,
                    substep,
                    "program-clock",
                    "clock overflow".into(),
                    1.0,
                    0.0,
                )
            })?;
        let (cap, ratio, _) = install(program, command_step, s).map_err(|error| {
            crate::measurement::Failure::new(s.tick, substep, "program-command", error, 1.0, 0.0)
        })?;
        CAP_STATS.with(|slot| {
            let mut stats = slot.borrow_mut();
            stats.0 = stats.0.max(cap);
            stats.1 = stats.1.max(ratio);
        });
        Ok(())
    })
}

struct ActiveGuard;
impl Drop for ActiveGuard {
    fn drop(&mut self) {
        ACTIVE.with(|slot| {
            slot.borrow_mut().take();
        });
    }
}

fn install(
    program: &CompiledProgram,
    step: u32,
    s: &mut Simulation,
) -> Result<(f64, f32, Vec<Value>), String> {
    if s.profile.dt_s != lh_contracts::SUBSTEP_S || s.profile.tick_substeps != 4 {
        return Err("Program requires the original 240 Hz clock".into());
    }
    let command = program.command(step);
    install_targets(
        s,
        &command.positions,
        &command.velocities,
        Allocation::OwnedRequests,
    )
}

#[test]
fn full_program_installs_equal_search_replay_motors_without_body_mutation() {
    let mut search =
        Simulation::new(0.0, crate::recovery_contract::pinned_profile(), true).unwrap();
    let mut replay =
        Simulation::new(0.0, crate::recovery_contract::pinned_profile(), true).unwrap();
    let source = crate::recovery::trajectory_program::neutral_program(&search.model);
    let program = source.clone().compile(&search.model).unwrap();
    let loaded = serde_json::from_str::<crate::recovery::trajectory_program::Program>(
        &serde_json::to_string(&source).unwrap(),
    )
    .unwrap()
    .compile(&search.model)
    .unwrap();
    for step in [0, 1, 959, 960, 1919, 1920, 2160] {
        let before = serde_json::to_value(&search.world.bodies).unwrap();
        let a = install(&program, step, &mut search).unwrap();
        let b = install(&loaded, step, &mut replay).unwrap();
        assert_eq!(a, b);
        assert_eq!(before, serde_json::to_value(&search.world.bodies).unwrap());
        assert_eq!(
            serde_json::to_value(&search.world.impulse_joints).unwrap(),
            serde_json::to_value(&replay.world.impulse_joints).unwrap()
        );
        assert_eq!(
            serde_json::to_value(&search.profile).unwrap(),
            serde_json::to_value(&replay.profile).unwrap()
        );
    }
    assert_eq!(program.source().schema_version, 1);
    // This test installs commands only. Integration equivalence remains a gate
    // until the usable reference is connected through advance_tick_observed.
}

#[test]
fn full_program_native_search_replay_preserve_every_substep_body_motor_and_contact() {
    fn history(program: CompiledProgram) -> Vec<Value> {
        let mut s = Simulation::new(0.0, crate::recovery_contract::pinned_profile(), true).unwrap();
        ACTIVE.with(|slot| {
            assert!(slot.borrow().is_none());
            *slot.borrow_mut() = Some((program, s.substeps_completed));
        });
        let _guard = ActiveGuard;
        let handles = s.bodies.clone();
        let generation = s.generation;
        let mut result = Vec::new();
        for _ in 0..3 {
            s.advance_tick_observed(|sim,_| {
                assert_eq!(handles,sim.bodies);assert_eq!(generation,sim.generation);
                result.push(serde_json::json!({"bodies":sim.world.bodies,"joints":sim.world.impulse_joints,
                    "contactsSerialized":serde_json::to_string(&sim.world.narrow_phase).unwrap(),"metrics":super::observe(sim,super::velocity(sim))}));
                Ok(())
            }).unwrap();
        }
        result
    }
    let model = lh_model::Model::canonical().unwrap();
    let mut source = crate::recovery::trajectory_program::neutral_program(&model);
    let mut middle = source.keyframes[0].clone();
    middle.substep = 8;
    let lumbar = model
        .segments
        .iter()
        .position(|p| p.id == "lumbar")
        .unwrap();
    middle.positions[lumbar][0] = 0.001;
    source.keyframes.insert(1, middle);
    let searched = source.clone().compile(&model).unwrap();
    assert_ne!(searched.command(0), searched.command(8));
    let replayed: crate::recovery::trajectory_program::Program =
        serde_json::from_str(&serde_json::to_string(&source).unwrap()).unwrap();
    let a = history(searched);
    let b = history(replayed.compile(&model).unwrap());
    assert_eq!(a.len(), 12);
    assert_eq!(a, b);
}

#[test]
#[ignore = "explicit numeric-reference geometry audit; no physical integration"]
fn audit_external_reference_geometry_without_integration() {
    use rapier3d::prelude::{Pose, Rotation};
    let directory = std::path::PathBuf::from(
        std::env::var("LH_REFERENCE_RETARGET_DIR").expect("retarget directory"),
    );
    let data: Value = serde_json::from_str(
        &std::fs::read_to_string(directory.join("candidate-geometry.json")).unwrap(),
    )
    .unwrap();
    let source: crate::recovery::trajectory_program::Program = serde_json::from_str(
        &std::fs::read_to_string(directory.join("candidate-program.json")).unwrap(),
    )
    .unwrap();
    let model = lh_model::Model::canonical().unwrap();
    let command_refusal = source.compile(&model).err();
    let mut rows = Vec::new();
    for degrees in [0.0_f32, 60.0_f32] {
        // Geometry-only world: poses are installed for inspection, never stepped.
        // This is not a candidate history and cannot certify resolved support.
        let mut s =
            Simulation::new(degrees, crate::recovery_contract::pinned_profile(), true).unwrap();
        let handles = s.bodies.clone();
        let heading = Rotation::from_rotation_y(degrees.to_radians());
        for frame in data["frames"].as_array().unwrap() {
            let poses: Vec<lh_model::Pose> =
                serde_json::from_value(frame["poses"].clone()).unwrap();
            crate::recovery::validate_seed(&s.model, &poses).unwrap();
            for (i, p) in poses.iter().enumerate() {
                s.world.bodies[s.bodies[i]].set_position(
                    Pose::from_parts(
                        heading * crate::vector(p.position),
                        heading * crate::rotation(p.rotation),
                    ),
                    false,
                );
            }
            s.world
                .bodies
                .propagate_modified_body_positions_to_colliders(&mut s.world.colliders);
            let mut diagnostics = lh_contracts::Diagnostics::default();
            let result = crate::measurement::inspect_world(&s, &s.world, &mut diagnostics, 0);
            rows.push(
                serde_json::json!({"headingDegrees":degrees,"substep":frame["substep"],
                "geometryPassed":result.is_ok(),"firstRefusal":result.err(),
                "observationsBeforeFirstRefusal":diagnostics}),
            );
            assert_eq!(s.bodies, handles);
            assert_eq!(s.substeps_completed, 0);
        }
    }
    let accepted = rows.iter().filter(|r| r["geometryPassed"] == true).count();
    let report = serde_json::json!({"geometryOnly":true,"physicalIntegrationSubsteps":0,
        "originalThresholdsPreserved":true,"rows":rows,"geometryPassedCount":accepted,
        "geometryEvaluationCount":rows.len(),"commandCompileRefusal":command_refusal,
        "nativeSearchStarted":false,"nativePhysicalSuccess":false});
    std::fs::write(
        directory.join("native-geometry-audit.json"),
        serde_json::to_string_pretty(&report).unwrap(),
    )
    .unwrap();
    println!(
        "geometry inspected: {accepted}/{}; command gate: {:?}",
        rows.len(),
        report["commandCompileRefusal"]
    );
}
