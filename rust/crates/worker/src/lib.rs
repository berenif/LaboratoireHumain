//! WASM worker boundary. The renderer never owns or imports this simulation.
#[cfg(target_arch = "wasm32")]
mod browser {
    use lh_contracts::{Command, Mode, SCHEMA_VERSION};
    use lh_sim::{
        Profile,
        profiling::{Recorder, TickTiming},
        runtime::{Reply, Runtime},
    };
    use serde::Serialize;
    use wasm_bindgen::prelude::*;

    fn error(value: impl std::fmt::Display) -> JsValue {
        JsValue::from_str(&value.to_string())
    }
    #[wasm_bindgen]
    extern "C" {
        #[wasm_bindgen(js_namespace = performance, js_name = now)]
        fn clock_ms() -> f64;
    }
    #[derive(Serialize)]
    #[serde(rename_all = "camelCase")]
    struct ProfiledReply<'a> {
        reply: &'a Reply,
        timings: &'a [TickTiming],
        clock_valid: bool,
        runtime_ms: f64,
        runtime_non_tick_ms: f64,
    }
    #[wasm_bindgen]
    pub fn wire_schema() -> u16 {
        SCHEMA_VERSION
    }
    #[wasm_bindgen]
    pub fn calibrate_tangent_contact(
        model: &str,
        heading: f32,
        warmstart: f32,
        scenario: &str,
        friction: f32,
        excitation: &str,
    ) -> Result<String, JsValue> {
        let report = match excitation {
            "stress" => {
                lh_sim::calibration::tangent::run(model, heading, warmstart, scenario, friction)
            }
            "bounded" => lh_sim::calibration::tangent::run_bounded(
                model, heading, warmstart, scenario, friction,
            ),
            _ => return Err(error("Unknown tangent calibration excitation")),
        }
        .map_err(error)?;
        serde_json::to_string(&report).map_err(error)
    }
    #[wasm_bindgen]
    pub fn calibrate_foundation(kind: &str, profile_json: &str) -> Result<String, JsValue> {
        if profile_json.len() > 65536 {
            return Err(error("Foundation profile exceeds bound"));
        }
        let profile: Profile = serde_json::from_str(profile_json).map_err(error)?;
        profile.validate().map_err(error)?;
        let cases = match kind {
            "primitive" => {
                let mut reports = Vec::new();
                for warmstart in [0.0, 0.5, 1.0] {
                    for scenario in ["rest", "changing", "unloading", "contact-loss"] {
                        reports.push(
                            lh_sim::calibration::contact_load_with_profile(
                                warmstart, scenario, &profile,
                            )
                            .map_err(error)?,
                        );
                    }
                }
                serde_json::to_value(reports).map_err(error)?
            }
            "canonical" => serde_json::to_value(
                lh_sim::calibration::canonical_contact_load_with_profile(&profile)
                    .map_err(error)?,
            )
            .map_err(error)?,
            "joints" => serde_json::to_value(
                lh_sim::calibration::isolated_joints_with_profile(&profile).map_err(error)?,
            )
            .map_err(error)?,
            "feet" => serde_json::to_value(
                lh_sim::calibration::individual_feet_with_profile(true, &profile).map_err(error)?,
            )
            .map_err(error)?,
            "chains" => serde_json::to_value(
                lh_sim::calibration::loaded_leg_chains_with_profile(&profile).map_err(error)?,
            )
            .map_err(error)?,
            _ => return Err(error("Unknown foundation calibration")),
        };
        serde_json::to_string(&serde_json::json!({"kind":kind,"profile":profile,"cases":cases,"releaseAccepted":false})).map_err(error)
    }

    #[wasm_bindgen]
    pub struct WorkerRuntime {
        runtime: Runtime,
        native_output_ms: f64,
    }
    #[wasm_bindgen]
    impl WorkerRuntime {
        #[wasm_bindgen(constructor)]
        pub fn new(
            mode: &str,
            heading: f32,
            generation: Option<String>,
            floor_enabled: Option<bool>,
            profile_json: Option<String>,
            playground_json: Option<String>,
        ) -> Result<WorkerRuntime, JsValue> {
            let mode = match mode {
                "playground" => Mode::Playground,
                "protocol" => Mode::Protocol,
                _ => return Err(error("Unknown mode")),
            };
            let generation = generation
                .map(|value| value.parse::<u64>().map_err(error))
                .transpose()?
                .unwrap_or(1);
            let profile = profile_json
                .map(|json| {
                    if json.len() > 65536 {
                        return Err(error("Profile exceeds worker input bound"));
                    }
                    serde_json::from_str::<Profile>(&json).map_err(error)
                })
                .transpose()?
                .unwrap_or_default();
            Ok(Self {
                runtime: Runtime::new_trial_with_settings(
                    mode,
                    heading,
                    profile,
                    generation,
                    floor_enabled.unwrap_or(true),
                    playground_json
                        .map(|json| serde_json::from_str(&json).map_err(error))
                        .transpose()?
                        .unwrap_or_default(),
                )
                .map_err(error)?,
                native_output_ms: 0.0,
            })
        }
        pub fn enqueue(&mut self, json: &str) -> Result<(), JsValue> {
            if json.len() > 65536 {
                return Err(error("Command exceeds worker input bound"));
            }
            let command: Command = serde_json::from_str(json).map_err(error)?;
            self.runtime
                .enqueue(command)
                .map_err(|reject| error(format!("{reject:?}")))
        }
        pub fn advance(&mut self, ticks: u32) -> Result<String, JsValue> {
            let reply = self
                .runtime
                .advance(ticks)
                .map_err(|reject| error(format!("{reject:?}")))?;
            serde_json::to_string(&reply).map_err(error)
        }
        pub fn enable_quiet_measurement(&mut self) -> Result<(), JsValue> {
            self.runtime
                .enable_quiet_measurement()
                .map_err(|reject| error(format!("{reject:?}")))
        }
        pub fn quiet_progress(&self) -> Result<String, JsValue> {
            serde_json::to_string(&self.runtime.quiet_progress()).map_err(error)
        }
        pub fn enable_quiet_drift_trace(&mut self) -> Result<(), JsValue> {
            self.runtime
                .enable_quiet_drift_trace()
                .map_err(|reject| error(format!("{reject:?}")))
        }
        pub fn quiet_report(&self) -> Result<String, JsValue> {
            serde_json::to_string(&self.runtime.quiet_report()).map_err(error)
        }
        pub fn advance_profiled(&mut self, ticks: u32) -> Result<String, JsValue> {
            let mut recorder = Recorder::new(clock_ms);
            let started = clock_ms();
            let mut reply = self
                .runtime
                .advance_traced(ticks, |_, event| recorder.record(event))
                .map_err(|reject| error(format!("{reject:?}")))?;
            let runtime_ms = clock_ms() - started;
            let (timings, clock_valid) = recorder.finish();
            let runtime_non_tick_ms =
                runtime_ms - timings.iter().map(|t| t.elapsed_ms).sum::<f64>();
            if clock_valid && let Some(last) = timings.iter().rev().find(|t| t.complete) {
                // Core update includes boundaries, all four substeps and
                // inspection. Output/transport costs are separate fields.
                reply.snapshot.diagnostics.controller_ms = Some(last.controller_ms);
                reply.snapshot.diagnostics.update_ms = Some(last.elapsed_ms);
            }
            let output_started = clock_ms();
            let output = serde_json::to_string(&ProfiledReply {
                reply: &reply,
                timings: &timings,
                clock_valid,
                runtime_ms,
                runtime_non_tick_ms,
            })
            .map_err(error)?;
            self.native_output_ms = clock_ms() - output_started;
            Ok(output)
        }
        pub fn native_output_ms(&self) -> f64 {
            self.native_output_ms
        }
        pub fn enqueue_batch(&mut self, json: &str) -> Result<(), JsValue> {
            if json.len() > 65536 {
                return Err(error("Command packet exceeds worker input bound"));
            }
            let commands: Vec<Command> = serde_json::from_str(json).map_err(error)?;
            self.runtime
                .enqueue_batch(commands)
                .map_err(|reject| error(format!("{reject:?}")))
        }
        pub fn set_visible(&mut self, visible: bool) -> Result<(), JsValue> {
            self.runtime
                .set_visible(visible)
                .map_err(|reject| error(format!("{reject:?}")))
        }
        pub fn shutdown(&mut self) {
            self.runtime.shutdown();
        }
    }
}
#[cfg(target_arch = "wasm32")]
pub use browser::*;
