//! JSONL process interface: reset/observe/initializedAction/step/checkpoint/load.
//! Input checkpoint is the already-integrated quiet standing GetUp baseline.
use lh_sim::getup::balance::{ComputeBudget, Experiment};
use serde::Deserialize;
use serde_json::json;
use std::{
    fs,
    io::{self, BufRead, Write},
};

#[derive(Deserialize)]
#[serde(tag = "op", rename_all = "camelCase", deny_unknown_fields)]
enum Command {
    Reset {},
    EpisodeLimit { seconds: u32 },
    Observe {},
    InitializedAction {},
    ReferenceAction {
        now: [f32; 25],
        ahead: [f32; 25],
    },
    Diagnostic {},
    ActorAction {
        path: String,
    },
    Step {
        action: [f32; 21],
        #[serde(default)]
        torso_force_n: [f32; 2],
    },
    StepMotionProfile {
        action: [f32; 21], torques_nm: [f32; 21], qdd: [f32; 21],
    },
    Checkpoint {
        path: String,
    },
    Load {
        path: String,
    },
    Snapshot {},
}
fn main() -> Result<(), String> {
    let args: Vec<_> = std::env::args().collect();
    if args.len() != 3 && !(args.len() == 4 && args[3] == "--upright-entry") {
        return Err("getup_stationary_balance GETUP_CHECKPOINT PHYSICS_WALL_BUDGET_S [--upright-entry]; then JSONL commands on stdin".into());
    }
    let seed = fs::read(&args[1]).map_err(|e| e.to_string())?;
    let mut budget = ComputeBudget::new(args[2].parse().map_err(|_| "Invalid budget")?)?;
    let reset = if args.len() == 4 {
        Experiment::reset_from_upright_checkpoint
    } else {
        Experiment::reset_from_getup_checkpoint
    };
    let mut experiment = reset(&seed)?;
    let stdin = io::stdin();
    let mut out = io::stdout().lock();
    for line in stdin.lock().lines() {
        let result = (|| -> Result<serde_json::Value, String> {
            let command: Command = serde_json::from_str(&line.map_err(|e| e.to_string())?)
                .map_err(|e| e.to_string())?;
            match command {
                Command::Reset {} => {
                    experiment = reset(&seed)?;
                    Ok(serde_json::to_value(experiment.observe()?).unwrap())
                }
                Command::EpisodeLimit { seconds } => { experiment.set_episode_limit_seconds(seconds)?; Ok(json!({"episodeLimitSeconds":experiment.episode_limit_seconds(),"physicsIntegrations":0})) },
                Command::Observe {} => Ok(serde_json::to_value(experiment.observe()?).unwrap()),
                Command::InitializedAction {} => Ok(
                    json!({"action":experiment.initialized_action()?,"trainedStationaryPolicy":false,"source":"architecture-validated-expanded-SLOW-initialization"}),
                ),
                Command::ReferenceAction { now, ahead } => experiment.reference_action(now, ahead),
                Command::Diagnostic {} => experiment.diagnostic(),
                Command::ActorAction { path } => {
                    let bytes = fs::read(path).map_err(|e| e.to_string())?;
                    let actor = lh_sim::getup::actor::Actor::stationary_from_bytes(&bytes)?;
                    Ok(json!({"action":actor.infer(&experiment.observe()?.observation)?.action}))
                }
                Command::Step {
                    action,
                    torso_force_n,
                } => Ok(serde_json::to_value(experiment.step_with_force(
                    action,
                    torso_force_n,
                    &mut budget,
                )?)
                .unwrap()),
                Command::StepMotionProfile { action, torques_nm, qdd } => Ok(serde_json::to_value(
                    experiment.step_motion_profile(action, torques_nm, qdd, &mut budget)?
                ).unwrap()),
                Command::Checkpoint { path } => {
                    let bytes = experiment.checkpoint()?;
                    let restored = Experiment::from_checkpoint(&bytes)?;
                    let same = serde_json::to_value(experiment.observe()?).unwrap()
                        == serde_json::to_value(restored.observe()?).unwrap()
                        && experiment.episode_limit_seconds() == restored.episode_limit_seconds()
                        && serde_json::to_value(experiment.snapshot()).unwrap()
                            == serde_json::to_value(restored.snapshot()).unwrap();
                    if !same {
                        return Err(
                            "Balance checkpoint roundtrip changed observations or snapshot".into(),
                        );
                    }
                    fs::write(&path, &bytes).map_err(|e| e.to_string())?;
                    Ok(
                        json!({"path":path,"bytes":bytes.len(),"roundtripPassed":true,"physicsIntegrations":0}),
                    )
                }
                Command::Load { path } => {
                    experiment =
                        Experiment::from_checkpoint(&fs::read(path).map_err(|e| e.to_string())?)?;
                    Ok(serde_json::to_value(experiment.observe()?).unwrap())
                }
                Command::Snapshot {} => Ok(serde_json::to_value(experiment.snapshot()).unwrap()),
            }
        })();
        let response = match result {
            Ok(value) => {
                json!({"ok":true,"result":value,"processPhysicsWallS":budget.used_s(),"processNativeSubsteps":budget.native_substeps})
            }
            Err(error) => {
                json!({"ok":false,"error":error,"processPhysicsWallS":budget.used_s(),"processNativeSubsteps":budget.native_substeps})
            }
        };
        writeln!(out, "{response}").map_err(|e| e.to_string())?;
        out.flush().map_err(|e| e.to_string())?;
    }
    Ok(())
}
