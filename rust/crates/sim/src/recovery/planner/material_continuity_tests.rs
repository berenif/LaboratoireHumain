//! Only constructed observations and archived JSON. No simulation/model/FK calls.
use super::super::{
    BODY_COUNT, BodyMotion, ContactMode, MaterialTracker, SegmentContactState, SegmentRole,
};
use super::*;

fn contact(p: [f64; 3]) -> CurrentContact {
    CurrentContact {
        local_point_m: p,
        world_point_m: p,
        upward_normal: 1.0,
        current_upward_load_n: 5.0,
    }
}
fn observation() -> SupportObservation {
    let bodies = (0..BODY_COUNT)
        .map(|segment| BodyMotion {
            segment,
            position_m: [0.0; 3],
            rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
            linear_velocity_mps: [0.0; 3],
            angular_velocity_radps: [0.0; 3],
        })
        .collect();
    let mut segments: Vec<_> = (0..BODY_COUNT)
        .map(|segment| SegmentContactState {
            segment,
            id: format!("segment{segment}"),
            role: SegmentRole::Other,
            persistent_load_n: 0.0,
            loaded_substeps: 0,
            contacts: Vec::new(),
        })
        .collect();
    segments[0].role = SegmentRole::Forefoot;
    segments[0].persistent_load_n = 20.0;
    segments[0].loaded_substeps = 1000;
    segments[0].contacts = vec![
        contact([0.0; 3]),
        contact([-0.03, 0.0, -0.03]),
        contact([0.03, 0.0, -0.03]),
        contact([0.0, 0.0, 0.04]),
    ];
    SupportObservation {
        native_substep: 36,
        time_s: 36.0 / 240.0,
        weight_n: 100.0,
        center_of_mass_m: [0.0, 0.8, 0.0],
        center_of_mass_velocity_mps: [0.0; 3],
        support_height_m: 0.0,
        bodies,
        segments,
        ccd_impulses: Vec::new(),
        native_contact_boundary: None,
    }
}
fn acquired() -> (SupportObservation, MaterialPatch) {
    let mut obs = observation();
    let mut tracker = MaterialTracker::default();
    tracker.acquire(&obs, 0, 0, ContactMode::Hold).unwrap();
    let patch = tracker.active.remove(0);
    assert!(patch.reference_surface.is_some());
    obs.native_substep += 1;
    obs.time_s = obs.native_substep as f64 / 240.0;
    obs.segments[0].contacts.remove(0);
    (obs, patch)
}
fn reason(obs: &SupportObservation, patch: &MaterialPatch) -> ContinuityRefusal {
    let evidence = patch.material_continuity(obs, 0.012).unwrap();
    assert!(!evidence.supported());
    evidence.refusal_reason.unwrap()
}

#[test]
fn original_interior_point_has_loaded_surface_certificate_without_reanchoring() {
    let (obs, mut patch) = acquired();
    let original = (
        patch.local_point_m,
        patch.world_anchor_m,
        patch.reference_surface.clone(),
    );
    assert!(patch.verify_current_support(&obs, 0.012).unwrap());
    patch.observe(&obs, 0.012).unwrap();
    assert!(!patch.current_contact_present);
    assert!(patch.has_current_support());
    assert_eq!(
        (
            patch.local_point_m,
            patch.world_anchor_m,
            patch.reference_surface.clone()
        ),
        original
    );
    let evidence = patch.continuity_evidence.as_ref().unwrap();
    assert_eq!(
        evidence.kind,
        MaterialContinuityKind::EquivalentLoadedSurface
    );
    assert_eq!(evidence.witness_contact_indices, vec![0, 1, 2]);
    assert_eq!(evidence.total_current_surface_load_n, 15.0);
    assert_eq!(evidence.nearest_loaded_contact_index, Some(2));
    assert_eq!(evidence.nearest_loaded_local_distance_m, Some(0.04));
}

#[test]
fn original_direct_rule_preserved_and_false_flag_not_overridden_by_direct_cache() {
    let mut obs = observation();
    obs.segments[0].contacts.truncate(1);
    obs.segments[0].contacts[0].current_upward_load_n = 0.01;
    obs.segments[0].contacts[0].upward_normal = MIN_NORMAL_Y;
    let mut tracker = MaterialTracker::default();
    tracker.acquire(&obs, 0, 0, ContactMode::Hold).unwrap();
    let patch = &mut tracker.active[0];
    assert!(patch.reference_surface.is_none());
    patch.observe(&obs, 0.012).unwrap();
    assert_eq!(
        patch.continuity_evidence.as_ref().unwrap().kind,
        MaterialContinuityKind::DirectLoadedPoint
    );
    patch.current_contact_present = false;
    assert!(!patch.has_current_support());
    assert!(patch.verify_current_support(&obs, 0.012).unwrap());
}

#[test]
fn legacy_import_has_no_surface_and_imported_certificate_is_discarded() {
    let (obs, mut patch) = acquired();
    patch.observe(&obs, 0.012).unwrap();
    let mut json = serde_json::to_value(&patch).unwrap();
    json.as_object_mut().unwrap().remove("reference_surface");
    let mut imported: MaterialPatch = serde_json::from_value(json).unwrap();
    assert!(imported.reference_surface.is_none());
    assert!(imported.continuity_evidence.is_none());
    assert!(!imported.has_current_support());
    imported.observe(&obs, 0.012).unwrap();
    assert_eq!(
        reason(&obs, &imported),
        ContinuityRefusal::NoAcquiredSurface
    );
    assert!(imported.reference_surface.is_none());
}

#[test]
fn cached_surface_must_be_recomputed_against_current_observation() {
    let (mut obs, mut patch) = acquired();
    patch.observe(&obs, 0.012).unwrap();
    obs.segments[0].contacts.clear();
    // Cache intentionally still says what the previous observation proved.
    assert!(patch.has_current_support());
    assert!(!patch.verify_current_support(&obs, 0.012).unwrap());
    patch.observe(&obs, 0.012).unwrap();
    assert!(!patch.has_current_support());
}

#[test]
fn local_and_world_anchor_or_acquisition_identity_cannot_be_changed() {
    let (obs, patch) = acquired();
    let mut modified = patch.clone();
    modified.local_point_m[0] += 0.001;
    assert_eq!(
        reason(&obs, &modified),
        ContinuityRefusal::AcquiredMaterialReferenceChanged
    );
    let mut modified = patch.clone();
    modified.world_anchor_m[0] += 0.001;
    assert_eq!(
        reason(&obs, &modified),
        ContinuityRefusal::AcquiredMaterialReferenceChanged
    );
    let mut modified = patch;
    modified.acquired_substep -= 1;
    assert_eq!(
        reason(&obs, &modified),
        ContinuityRefusal::AcquiredMaterialReferenceChanged
    );
}

#[test]
fn outside_hull_is_refused_even_inside_axis_aligned_bounds() {
    let (mut obs, patch) = acquired();
    obs.segments[0].contacts = vec![
        contact([-0.04, 0.0, 0.02]),
        contact([0.04, 0.0, 0.02]),
        contact([0.04, 0.0, -0.02]),
    ];
    // Use an original origin outside a triangular corner but inside its AABB.
    obs.segments[0].contacts[0].local_point_m[2] = 0.04;
    obs.segments[0].contacts[0].world_point_m[2] = 0.04;
    assert_eq!(
        reason(&obs, &patch),
        ContinuityRefusal::OriginalPointOutsideLoadedHull
    );
}

#[test]
fn hull_tolerance_is_numerical_and_never_contact_match_distance() {
    let (mut obs, patch) = acquired();
    for (offset, expected) in [(0.5e-6, true), (2e-6, false), (0.011, false)] {
        obs.segments[0].contacts = vec![
            contact([offset, 0.0, -0.03]),
            contact([offset + 0.03, 0.0, 0.0]),
            contact([offset, 0.0, 0.03]),
        ];
        assert_eq!(patch.verify_current_support(&obs, 0.012).unwrap(), expected);
    }
}

#[test]
fn collinear_loaded_points_do_not_define_surface_support() {
    let (mut obs, patch) = acquired();
    obs.segments[0].contacts = vec![
        contact([-0.03, 0.0, 0.04]),
        contact([0.0, 0.0, 0.04]),
        contact([0.03, 0.0, 0.04]),
    ];
    assert_eq!(
        reason(&obs, &patch),
        ContinuityRefusal::CollinearLoadedSurface
    );
}

#[test]
fn foreign_material_plane_cannot_replace_acquired_face() {
    let (mut obs, patch) = acquired();
    for c in &mut obs.segments[0].contacts {
        c.local_point_m[1] = 0.002;
        c.world_point_m[1] = 0.002;
    }
    assert_eq!(
        reason(&obs, &patch),
        ContinuityRefusal::DifferentMaterialPlane
    );
}

#[test]
fn contacts_require_flat_ground_normals_with_numeric_tolerance() {
    let (mut obs, patch) = acquired();
    for c in &mut obs.segments[0].contacts {
        c.upward_normal = 1.0 - 0.5e-6;
    }
    assert!(patch.verify_current_support(&obs, 0.012).unwrap());
    for c in &mut obs.segments[0].contacts {
        c.upward_normal = 0.99;
    }
    assert_eq!(reason(&obs, &patch), ContinuityRefusal::NonFlatGroundNormal);
}

#[test]
fn persistence_and_subthreshold_vertex_loads_cannot_enlarge_hull() {
    let (mut obs, patch) = acquired();
    for load in [0.0, 2.999] {
        for c in &mut obs.segments[0].contacts {
            c.current_upward_load_n = load;
        }
        obs.segments[0].loaded_substeps = 1_000_000;
        obs.segments[0].persistent_load_n = 1_000_000.0;
        assert_eq!(
            reason(&obs, &patch),
            ContinuityRefusal::FewerThanThreeLoadedSurfaceContacts
        );
    }
    for c in &mut obs.segments[0].contacts {
        c.current_upward_load_n = 3.0;
    }
    assert!(patch.verify_current_support(&obs, 0.012).unwrap());
}

#[test]
fn lifted_material_point_cannot_be_certified_by_ground_claims() {
    let (mut obs, patch) = acquired();
    obs.bodies[0].position_m[1] = 0.012001;
    for c in &mut obs.segments[0].contacts {
        c.world_point_m = transform_point(&obs.bodies[0], c.local_point_m);
    }
    assert_eq!(
        reason(&obs, &patch),
        ContinuityRefusal::OriginalPointOutsideGroundDistance
    );
}

#[test]
fn inconsistent_local_world_contacts_refuse_surface_certificate() {
    let (mut obs, patch) = acquired();
    obs.segments[0].contacts[1].world_point_m[0] += 0.00001;
    assert_eq!(
        reason(&obs, &patch),
        ContinuityRefusal::InconsistentLocalWorldPoint
    );
}

#[test]
fn acquired_face_cannot_turn_over_while_preserving_ground_normal_claims() {
    let (mut obs, patch) = acquired();
    obs.bodies[0].rotation_xyzw = [1.0, 0.0, 0.0, 0.0];
    for c in &mut obs.segments[0].contacts {
        c.world_point_m = transform_point(&obs.bodies[0], c.local_point_m);
    }
    assert_eq!(
        reason(&obs, &patch),
        ContinuityRefusal::AcquiredFaceNormalNotUpward
    );
}

#[test]
fn acquisition_without_loaded_coplanar_triangle_never_adds_surface_later() {
    let mut obs = observation();
    obs.segments[0].contacts.truncate(1);
    let mut tracker = MaterialTracker::default();
    tracker.acquire(&obs, 0, 0, ContactMode::Hold).unwrap();
    let mut patch = tracker.active.remove(0);
    obs = observation();
    obs.native_substep += 1;
    obs.segments[0].contacts.remove(0);
    patch.observe(&obs, 0.012).unwrap();
    assert_eq!(reason(&obs, &patch), ContinuityRefusal::NoAcquiredSurface);
    assert!(patch.reference_surface.is_none());
}

#[test]
fn support_on_other_segment_cannot_supply_material_surface() {
    let (mut obs, patch) = acquired();
    obs.segments[1].contacts = std::mem::take(&mut obs.segments[0].contacts);
    assert_eq!(
        reason(&obs, &patch),
        ContinuityRefusal::FewerThanThreeLoadedSurfaceContacts
    );
}

#[test]
fn static_certificate_does_not_reset_drift_or_grant_a_drift_waiver() {
    let (mut obs, mut patch) = acquired();
    let anchor = patch.world_anchor_m;
    obs.bodies[0].position_m[0] = 0.011;
    for c in &mut obs.segments[0].contacts {
        c.world_point_m = transform_point(&obs.bodies[0], c.local_point_m);
    }
    patch.observe(&obs, 0.012).unwrap();
    assert!(patch.has_current_support());
    assert_eq!(patch.world_anchor_m, anchor);
    assert!((patch.tangential_drift_m - 0.011).abs() < 1e-12);
    // The adapter's existing 10 mm guard must still reject this observation.
}

#[test]
fn archived_78_is_refused_even_if_weak_third_vertex_is_synthetically_strengthened() {
    let fixture:serde_json::Value=serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"),
        "/../../../docs/evidence/coordination-contact-continuity-2026-10-15/continuity-archive-36-78.json"))).unwrap();
    let acquisition: SupportObservation =
        serde_json::from_value(fixture["acquisition"].clone()).unwrap();
    let mut failed: SupportObservation = serde_json::from_value(fixture["failed"].clone()).unwrap();
    let segment = fixture["segment"].as_u64().unwrap() as usize;
    let contact_index = fixture["acquiredContactIndex"].as_u64().unwrap() as usize;
    let mut tracker = MaterialTracker::default();
    tracker
        .acquire(&acquisition, segment, contact_index, ContactMode::Hold)
        .unwrap();
    let patch = &mut tracker.active[0];
    assert!(patch.reference_surface.is_some());
    patch.observe(&failed, 0.012).unwrap();
    assert!(!patch.current_contact_present);
    assert!(!patch.has_current_support());
    assert_eq!(
        reason(&failed, patch),
        ContinuityRefusal::FewerThanThreeLoadedSurfaceContacts
    );
    assert!((patch.nearest_current_local_distance_m.unwrap() - 0.09).abs() < 1e-6);
    assert_eq!(
        patch
            .continuity_evidence
            .as_ref()
            .unwrap()
            .eligible_contact_indices,
        vec![0, 2]
    );
    // This alteration is a pure discriminating fixture, NOT another physical run.
    for c in &mut failed.segments[segment].contacts {
        c.current_upward_load_n = 5.0;
    }
    assert_eq!(
        reason(&failed, patch),
        ContinuityRefusal::OriginalPointOutsideLoadedHull
    );
    let p = patch.local_point_m;
    let c = &failed.segments[segment].contacts;
    let a = c[0].local_point_m;
    let b = c[1].local_point_m;
    let edge = sub(b, a);
    let t = (dot(sub(p, a), edge) / dot(edge, edge)).clamp(0.0, 1.0);
    let closest = std::array::from_fn(|i| a[i] + t * edge[i]);
    assert!((norm(sub(p, closest)) - 0.06999278470891934).abs() < 1e-6);
}

#[test]
fn equivalent_surface_is_only_for_held_foot_material_not_pivot_or_other_roles() {
    let (mut obs, mut patch) = acquired();
    patch.observe(&obs, 0.012).unwrap();
    assert!(patch.has_current_support());
    for mode in [ContactMode::Pivot, ContactMode::Unload, ContactMode::Free] {
        patch.mode = mode;
        assert!(!patch.has_current_support());
        assert_eq!(
            reason(&obs, &patch),
            ContinuityRefusal::UnsupportedContactMode
        );
    }
    patch.mode = ContactMode::Hold;
    for role in [SegmentRole::Hand, SegmentRole::Shin, SegmentRole::Other] {
        obs.segments[0].role = role;
        assert_eq!(
            reason(&obs, &patch),
            ContinuityRefusal::UnsupportedSegmentRole
        );
    }
}

#[test]
fn pivot_and_nonfoot_acquisition_never_create_equivalent_surface_reference() {
    let mut obs = observation();
    let mut tracker = MaterialTracker::default();
    tracker.acquire(&obs, 0, 0, ContactMode::Pivot).unwrap();
    assert!(tracker.active[0].reference_surface.is_none());
    obs.segments[0].role = SegmentRole::Hand;
    tracker.acquire(&obs, 0, 0, ContactMode::Hold).unwrap();
    assert!(tracker.active[1].reference_surface.is_none());
}

#[test]
fn cached_surface_receipt_must_match_patch_identity_and_observation_clock() {
    let (obs, mut patch) = acquired();
    patch.observe(&obs, 0.012).unwrap();
    assert!(patch.has_current_support());
    let mut stale = patch.clone();
    stale.continuity_evidence.as_mut().unwrap().native_substep -= 1;
    assert!(!stale.has_current_support());
    let mut wrong_id = patch.clone();
    wrong_id.continuity_evidence.as_mut().unwrap().patch_id += 1;
    assert!(!wrong_id.has_current_support());
    let mut wrong_segment = patch;
    wrong_segment.continuity_evidence.as_mut().unwrap().segment += 1;
    assert!(!wrong_segment.has_current_support());
}
