//! Serializable, read-only evidence from one completed native contact boundary.
//!
//! Manifold/point indices and feature IDs are diagnostics local to this boundary,
//! never durable material identities. Unselected point impulses may be cached;
//! only admitted contacts in the active solver collection contribute support.
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct HandleIdentity {
    pub index: u32,
    pub generation: u32,
}

impl From<(u32, u32)> for HandleIdentity {
    fn from((index, generation): (u32, u32)) -> Self {
        Self { index, generation }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ContactBoundary {
    pub schema_version: u32,
    pub native_substep: u64,
    pub enclosing_native_dt_s: f64,
    pub floor_collider: HandleIdentity,
    /// One entry per anatomical segment, including absent floor contact pairs.
    pub pairs: Vec<PairBoundary>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PairBoundary {
    pub segment: usize,
    pub segment_collider: HandleIdentity,
    pub pair_present: bool,
    pub pair_colliders: Option<[HandleIdentity; 2]>,
    pub collider_poses: [Option<PoseEvidence>; 2],
    pub segment_is_collider1: Option<bool>,
    pub solver_uses_clusters: bool,
    pub raw_manifolds: Vec<ManifoldBoundary>,
    pub solver_clusters: Vec<ManifoldBoundary>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PoseEvidence {
    pub translation_m: [f64; 3],
    pub rotation_xyzw: [f64; 4],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ManifoldBoundary {
    /// Index within raw_manifolds or solver_clusters of this pair only.
    pub manifold_index: usize,
    pub is_current_solver_collection: bool,
    pub subshape1: u32,
    pub subshape2: u32,
    pub subshape_pose1: Option<PoseEvidence>,
    pub subshape_pose2: Option<PoseEvidence>,
    pub local_normal1: [f64; 3],
    pub local_normal2: [f64; 3],
    pub world_normal: [f64; 3],
    pub upward_normal_on_segment: f64,
    pub solver_flags_bits: u32,
    pub computes_impulses: bool,
    pub rigid_body1: Option<HandleIdentity>,
    pub rigid_body2: Option<HandleIdentity>,
    pub relative_dominance: i16,
    pub friction: f64,
    pub restitution: f64,
    /// Reporting impulse on collider1; this is not a per-point world tangent.
    pub reported_tangent_impulse_ns: [f64; 3],
    pub points: Vec<ManifoldPointBoundary>,
    pub selected_contacts: Vec<SelectedContactBoundary>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ManifoldPointBoundary {
    pub point_index: usize,
    pub local_p1_m: [f64; 3],
    pub local_p2_m: [f64; 3],
    /// Raw geometric points resolved through current collider/subshape poses.
    /// They are not solver anchors and never become support just by existing.
    pub current_world_p1_m: Option<[f64; 3]>,
    pub current_world_p2_m: Option<[f64; 3]>,
    pub live_geometric_separation_m: Option<f64>,
    /// Stored narrow-phase distance, not separation through current body poses.
    pub stored_distance_m: f64,
    pub feature_id1_raw: u32,
    pub feature_id2_raw: u32,
    /// May be stale when this point is unselected or its collection is inactive.
    pub normal_impulse_ns: f64,
    /// Components in Rapier's tangent basis, not world XYZ.
    pub tangent_impulse_ns: [f64; 2],
    pub warmstart_normal_impulse_ns: f64,
    pub warmstart_tangent_impulse_ns: [f64; 2],
    pub warmstart_twist_impulse_ns: f64,
    pub warmstart_tangent_world_ns: [f64; 3],
    pub solver_dp1_m: [f64; 3],
    pub solver_dp2_m: [f64; 3],
    pub selected_contact_indices: Vec<usize>,
    /// Selection-level reasons. Exact live geometry filtering is recorded in
    /// selected_contacts, whose anchors differ from raw narrow-phase points.
    pub filtered_reasons: Vec<ContactExclusion>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SelectedContactBoundary {
    pub selected_contact_index: usize,
    pub manifold_point_index: u32,
    /// Includes Rapier's new-contact bit. It is not a durable point ID.
    pub contact_id_raw: u32,
    pub anchor1_raw: [f64; 3],
    pub anchor2_raw: [f64; 3],
    pub stored_distance_m: f64,
    pub tangent_velocity_mps: [f64; 3],
    pub world_anchor1_m: [f64; 3],
    pub world_anchor2_m: [f64; 3],
    pub segment_local_point_m: [f64; 3],
    pub segment_world_point_m: [f64; 3],
    pub live_separation_m: f64,
    pub normal_impulse_ns: Option<f64>,
    pub upward_normal_on_segment: f64,
    pub admitted_as_current_contact: bool,
    pub current_upward_load_n: Option<f64>,
    pub filtered_reasons: Vec<ContactExclusion>,
    /// Inactive/cached diagnostic data cannot add a new failure to observation.
    /// The existing observer still rejects invalid active selected contacts.
    pub measurement_error: Option<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContactExclusion {
    InactiveSolverCollection,
    SolverImpulsesDisabled,
    UnselectedManifoldPoint,
    UpwardNormalBelowMinimum,
    ZeroImpulse,
    LiveSeparationExceeded,
}

#[derive(Clone, Copy, Debug)]
pub struct SelectedContactSample {
    pub local_point_m: [f64; 3],
    pub world_point_m: [f64; 3],
    pub upward_normal: f64,
    pub impulse_ns: f64,
    pub live_separation_m: f64,
    pub enclosing_native_dt_s: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct ContactAdmissionPolicy {
    pub minimum_upward_normal: f64,
    pub maximum_live_separation_m: f64,
    pub required_native_dt_s: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SelectedContactClassification {
    pub current_upward_load_n: Option<f64>,
    pub filtered_reasons: Vec<ContactExclusion>,
}

/// Shared with native_observation::selected_contact: diagnostics cannot silently
/// diverge from its existing normal, impulse, distance or native-dt rule.
pub fn classify_selected_contact(
    sample: SelectedContactSample,
    policy: ContactAdmissionPolicy,
) -> Result<SelectedContactClassification, String> {
    if sample
        .local_point_m
        .iter()
        .chain(&sample.world_point_m)
        .any(|v| !v.is_finite())
        || !sample.upward_normal.is_finite()
        || !(-1.0..=1.0).contains(&sample.upward_normal)
        || !sample.impulse_ns.is_finite()
        || sample.impulse_ns < 0.0
        || !sample.live_separation_m.is_finite()
        || sample.enclosing_native_dt_s != policy.required_native_dt_s
        || !policy.minimum_upward_normal.is_finite()
        || !(-1.0..=1.0).contains(&policy.minimum_upward_normal)
        || !policy.maximum_live_separation_m.is_finite()
        || policy.maximum_live_separation_m < 0.0
        || !policy.required_native_dt_s.is_finite()
        || policy.required_native_dt_s <= 0.0
    {
        return Err("Invalid selected native contact measurement".into());
    }
    let mut filtered_reasons = Vec::new();
    if sample.upward_normal < policy.minimum_upward_normal {
        filtered_reasons.push(ContactExclusion::UpwardNormalBelowMinimum);
    }
    if sample.impulse_ns == 0.0 {
        filtered_reasons.push(ContactExclusion::ZeroImpulse);
    }
    if sample.live_separation_m > policy.maximum_live_separation_m {
        filtered_reasons.push(ContactExclusion::LiveSeparationExceeded);
    }
    let current_upward_load_n = if filtered_reasons.is_empty() {
        let load = sample.upward_normal * sample.impulse_ns / sample.enclosing_native_dt_s;
        if !load.is_finite() {
            return Err("Nonfinite native selected contact load".into());
        }
        Some(load)
    } else {
        None
    };
    Ok(SelectedContactClassification {
        current_upward_load_n,
        filtered_reasons,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn policy() -> ContactAdmissionPolicy {
        ContactAdmissionPolicy {
            minimum_upward_normal: 0.65,
            maximum_live_separation_m: 0.012,
            required_native_dt_s: 1.0 / 240.0,
        }
    }

    fn sample() -> SelectedContactSample {
        SelectedContactSample {
            local_point_m: [0.0; 3],
            world_point_m: [0.0; 3],
            upward_normal: 1.0,
            impulse_ns: 1.0,
            live_separation_m: 0.0,
            enclosing_native_dt_s: 1.0 / 240.0,
        }
    }

    #[test]
    fn classification_reports_each_filter_without_native_integration() {
        let mut s = sample();
        assert_eq!(
            classify_selected_contact(s, policy())
                .unwrap()
                .current_upward_load_n,
            Some(240.0)
        );
        s.impulse_ns = 0.0;
        assert_eq!(
            classify_selected_contact(s, policy())
                .unwrap()
                .filtered_reasons,
            vec![ContactExclusion::ZeroImpulse]
        );
        s.upward_normal = 0.64;
        s.live_separation_m = 0.012_001;
        assert_eq!(
            classify_selected_contact(s, policy())
                .unwrap()
                .filtered_reasons,
            vec![
                ContactExclusion::UpwardNormalBelowMinimum,
                ContactExclusion::ZeroImpulse,
                ContactExclusion::LiveSeparationExceeded
            ]
        );
    }

    #[test]
    fn classification_preserves_boundary_and_rejects_invalid_measurements_without_native_integration()
     {
        let mut s = sample();
        s.upward_normal = policy().minimum_upward_normal;
        s.live_separation_m = policy().maximum_live_separation_m;
        assert!(
            classify_selected_contact(s, policy())
                .unwrap()
                .current_upward_load_n
                .is_some()
        );
        s.impulse_ns = -1.0;
        assert!(classify_selected_contact(s, policy()).is_err());
        s.impulse_ns = 1.0;
        s.world_point_m[0] = f64::NAN;
        assert!(classify_selected_contact(s, policy()).is_err());
        s = sample();
        s.enclosing_native_dt_s *= 2.0;
        assert!(classify_selected_contact(s, policy()).is_err());
    }

    #[test]
    fn classifier_matches_original_selection_table_without_native_integration() {
        let p = policy();
        for upward_normal in [-1.0, 0.64, 0.65, 1.0] {
            for impulse_ns in [0.0, 0.02, 1.0] {
                for live_separation_m in [-0.1, 0.0, 0.012, 0.02] {
                    let s = SelectedContactSample {
                        upward_normal,
                        impulse_ns,
                        live_separation_m,
                        ..sample()
                    };
                    // Literal original observer predicates and load equation.
                    let original_load =
                        if upward_normal < 0.65 || impulse_ns == 0.0 || live_separation_m > 0.012 {
                            None
                        } else {
                            Some(upward_normal * impulse_ns / s.enclosing_native_dt_s)
                        };
                    assert_eq!(
                        classify_selected_contact(s, p)
                            .unwrap()
                            .current_upward_load_n,
                        original_load
                    );
                }
            }
        }
    }

    #[test]
    fn absent_pair_and_handle_generations_round_trip_without_native_integration() {
        let boundary = ContactBoundary {
            schema_version: 1,
            native_substep: 78,
            enclosing_native_dt_s: 1.0 / 240.0,
            floor_collider: (0, 2).into(),
            pairs: vec![PairBoundary {
                segment: 19,
                segment_collider: (20, 3).into(),
                pair_present: false,
                pair_colliders: None,
                collider_poses: [None, None],
                segment_is_collider1: None,
                solver_uses_clusters: false,
                raw_manifolds: vec![],
                solver_clusters: vec![],
            }],
        };
        let json = serde_json::to_string(&boundary).unwrap();
        let restored: ContactBoundary = serde_json::from_str(&json).unwrap();
        assert_eq!(restored.floor_collider.generation, 2);
        assert_eq!(restored.pairs[0].segment_collider.generation, 3);
        assert!(!restored.pairs[0].pair_present);
        assert_eq!(serde_json::to_string(&restored).unwrap(), json);
    }
}
