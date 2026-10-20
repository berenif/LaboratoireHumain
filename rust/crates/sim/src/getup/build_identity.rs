//! Read-only compiled capability identity; constructs no Simulation.
pub fn capabilities() -> serde_json::Value {
    let mut features: Vec<&str> = Vec::new();
    if cfg!(feature = "coordination-transfer-worker") { features.push("coordination-transfer-worker"); }
    if cfg!(feature = "coordination-worker") { features.push("coordination-worker"); }
    if cfg!(feature = "experimental-joint-velocity-cache") { features.push("experimental-joint-velocity-cache"); }
    if cfg!(feature = "experimental-simd8") { features.push("experimental-simd8"); }
    if cfg!(feature = "experimental-solver-step-observation") { features.push("experimental-solver-step-observation"); }
    if cfg!(feature = "getup-physical-roll-experiment") { features.push("getup-physical-roll-experiment"); }
    if cfg!(feature = "getup-policy-adaptation-experiment") { features.push("getup-policy-adaptation-experiment"); }
    if cfg!(feature = "getup-repeat-cycle-experiment") { features.push("getup-repeat-cycle-experiment"); }
    if cfg!(feature = "getup-terminal-balance-experiment") { features.push("getup-terminal-balance-experiment"); }
    if cfg!(feature = "getup-terminal-base-ready-experiment") { features.push("getup-terminal-base-ready-experiment"); }
    if cfg!(feature = "getup-terminal-bumpless-experiment") { features.push("getup-terminal-bumpless-experiment"); }
    if cfg!(feature = "getup-terminal-tail-observer-experiment") { features.push("getup-terminal-tail-observer-experiment"); }
    if cfg!(feature = "numeric-parity-worker") { features.push("numeric-parity-worker"); }
    if cfg!(feature = "root-translation-carry-experiment") { features.push("root-translation-carry-experiment"); }
    if cfg!(feature = "standing-dynamics-experiment") { features.push("standing-dynamics-experiment"); }
    if cfg!(feature = "stationary-balance-experiment") { features.push("stationary-balance-experiment"); }
    if cfg!(feature = "unit-root-rotation-experiment") { features.push("unit-root-rotation-experiment"); }
    serde_json::json!({"schema":"lh-getup-compiled-capabilities/v1",
        "effectiveSimFeatures":features,"physicsIntegrations":0,"constructsRuntime":false})
}
