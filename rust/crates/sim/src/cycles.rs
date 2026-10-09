//! Five uninterrupted physical striker/fall/get-up cycles, with no resets.
use crate::{Profile, measurement, runtime::Runtime};
use lh_contracts::{Action, Command, Mode, Motion, Stamp};
use serde_json::{Value, json};

pub fn trial(profile: &Profile) -> Result<Value, String> {
    if !profile.recovery_enabled {
        return Err("Cycle fixture requires recovery_enabled".into());
    }
    let mut runtime = Runtime::new(Mode::Protocol, 0.0, profile.clone())?;
    let initial = runtime.advance(0).map_err(|e| format!("{e:?}"))?.snapshot;
    let mut snapshot = initial.clone();
    let mut cycles = Vec::new();
    let mut trace = Vec::new();
    let mut original = None;
    let mut first_failure = None;
    for cycle in 1..=5 {
        runtime
            .enqueue(Command {
                stamp: Stamp::new(snapshot.stamp.generation, cycle, snapshot.stamp.tick),
                action: Action::Strike,
            })
            .map_err(|e| format!("{e:?}"))?;
        let began_tick = snapshot.stamp.tick;
        let mut fall_tick = None;
        let mut recovery_tick = None;
        let mut stable_substeps = 0;
        let mut minimum_pelvis_y = f32::INFINITY;
        for _ in 0..1800 {
            let reply = runtime
                .advance_observed(1, |s, substep| {
                    let originals = original.get_or_insert_with(|| s.bodies.clone());
                    if s.bodies != *originals || s.generation != initial.stamp.generation {
                        return Err(measurement::Failure::new(
                            s.tick,
                            substep,
                            "uninterrupted-cycle-ownership",
                            "body/generation replacement".into(),
                            1.0,
                            0.0,
                        ));
                    }
                    minimum_pelvis_y =
                        minimum_pelvis_y.min(s.world.bodies[s.bodies[0]].translation().y);
                    if s.counters.falls >= cycle as u32 {
                        fall_tick.get_or_insert(s.tick);
                    }
                    if s.counters.recoveries >= cycle as u32 {
                        recovery_tick.get_or_insert(s.tick);
                        measurement::standing_support(s, substep)?;
                        if s.motion != Motion::Upright {
                            return Err(measurement::Failure::new(
                                s.tick,
                                substep,
                                "cycle-standing-handoff",
                                "motion".into(),
                                0.0,
                                1.0,
                            ));
                        }
                        stable_substeps += 1;
                    }
                    Ok(())
                })
                .map_err(|e| format!("{e:?}"))?;
            snapshot = reply.snapshot;
            if reply.failure.is_some() {
                first_failure = reply.failure;
                break;
            }
            if snapshot.stamp.tick % 15 == 0 {
                trace.push(snapshot.clone());
            }
            if stable_substeps >= 240
                || fall_tick.is_some_and(|tick| snapshot.stamp.tick - tick > 1500)
            {
                break;
            }
        }
        let passed = first_failure.is_none()
            && minimum_pelvis_y < 0.55
            && stable_substeps >= 240
            && recovery_tick
                .zip(fall_tick)
                .is_some_and(|(recovered, fallen)| recovered - fallen <= 1500);
        cycles.push(json!({"cycle":cycle,"passed":passed,"beganTick":began_tick,"fallTick":fall_tick,"recoveryTick":recovery_tick,"minimumPelvisY":minimum_pelvis_y,"stableSubsteps":stable_substeps}));
        if !passed {
            break;
        }
    }
    Ok(
        json!({"passed":cycles.len() == 5 && cycles.iter().all(|c| c["passed"] == true),"requiredCycles":5,
        "completedCycles":cycles.iter().filter(|c| c["passed"] == true).count(),"cycles":cycles,"firstFailure":first_failure,"initial":initial,"final":snapshot,"trace":trace}),
    )
}
