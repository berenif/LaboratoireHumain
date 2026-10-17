//! Current acceptance contract; historical v1 remains available for old evidence.
use serde::{Deserialize, Serialize};
use std::sync::OnceLock;

pub const FROZEN_JSON: &str = include_str!("../data/recovery-contract-v2.json");
pub const PINNED_PROFILE_JSON: &str = include_str!("../data/recovery-profile-v24.json");

pub fn pinned_profile() -> crate::Profile {
    serde_json::from_str(PINNED_PROFILE_JSON).expect("frozen recovery qualification profile")
}

pub fn profile_matches_pinned(profile: &crate::Profile) -> bool {
    serde_json::to_value(profile).expect("serializable profile")
        == serde_json::to_value(pinned_profile()).expect("serializable pinned profile")
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct Contract {
    pub version: u32,
    pub source: String,
    pub source_sha256: String,
    pub acceptance: Acceptance,
    pub recovery_acceptance: RecoveryAcceptance,
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct Acceptance {
    pub fixed_dt_s: f64,
    pub max_joint_separation_m: f32,
    /// None: depth is measured but never rejects a trajectory by itself.
    pub max_floor_penetration_m: Option<f32>,
    pub max_non_excluded_self_penetration_m: Option<f32>,
    pub trajectory_position_tolerance_m: f32,
    pub trajectory_rotation_tolerance_degrees: f32,
    pub trajectory_velocity_tolerance: f32,
    pub recovery_limit_s: f64,
    pub stable_observation_s: f64,
    pub idle_settling_s: f64,
    pub idle_duration_s: f64,
    pub idle_root_drift_m: f32,
    pub idle_planted_foot_drift_m: f32,
    pub idle_max_linear_mps: f32,
    pub idle_max_angular_radps: f32,
    pub maximum_structural_limit_error_rad: f32,
    pub repeated_cycles: u32,
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct RecoveryAcceptance {
    pub normal_y: f32,
    pub contact_distance_m: f32,
    pub minimum_load_n: f32,
    pub load_persistence_s: f64,
    pub landing_persistence_s: f64,
    pub settle_persistence_s: f64,
    pub settle_linear_mps: f32,
    pub settle_angular_radps: f32,
    pub stable_persistence_s: f64,
    pub stable_linear_mps: f32,
    pub stable_angular_radps: f32,
    pub stable_up_dot: f32,
    pub phase_minimum_s: f64,
    pub stall_s: f64,
    pub support_loss_s: f64,
    pub assistance_force_n: f32,
    pub assistance_torque_nm: f32,
}

pub fn contract() -> &'static Contract {
    static CONTRACT: OnceLock<Contract> = OnceLock::new();
    CONTRACT.get_or_init(|| serde_json::from_str(FROZEN_JSON).expect("frozen recovery contract"))
}

/// Durations use the configured native integration clock, never tick labels.
pub fn substeps(seconds: f64) -> u64 {
    (seconds * f64::from(lh_contracts::TICK_HZ * lh_contracts::SUBSTEPS)).ceil() as u64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frozen_golden_deadline_and_hold_are_independent_of_profiles() {
        let c = contract();
        assert_eq!(c.version, 2);
        assert_eq!(c.source, "rust/crates/sim/data/recovery-contract-v1.json");
        assert_eq!(c.source_sha256.len(), 64);
        assert!(c.acceptance.max_floor_penetration_m.is_none());
        assert!(c.acceptance.max_non_excluded_self_penetration_m.is_none());
        assert_eq!(c.acceptance.recovery_limit_s, 25.0);
        assert_eq!(c.acceptance.stable_observation_s, 1.0);
        assert_eq!(substeps(c.acceptance.recovery_limit_s), 20000);
        assert_eq!(substeps(c.acceptance.stable_observation_s), 800);
        assert_eq!(c.acceptance.repeated_cycles, 5);
        assert_eq!(substeps(c.recovery_acceptance.load_persistence_s), 40);

        let mut old: serde_json::Value =
            serde_json::from_str(include_str!("../data/recovery-contract-v1.json")).unwrap();
        let current: serde_json::Value = serde_json::from_str(FROZEN_JSON).unwrap();
        old["acceptance"]["maxFloorPenetrationM"] = serde_json::Value::Null;
        old["acceptance"]["maxNonExcludedSelfPenetrationM"] = serde_json::Value::Null;
        assert_eq!(current["acceptance"], old["acceptance"]);
        assert_eq!(current["recoveryAcceptance"], old["recoveryAcceptance"]);
    }
}
