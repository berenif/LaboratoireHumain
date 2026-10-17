//! Conservative, observation-recomputed continuity of one acquired material point.
//! This is contact evidence only: it grants no native execution, drift waiver,
//! support release, load-transfer success or recovery qualification.
use super::{
    BodyMotion, ContactMode, CurrentContact, MIN_LOAD_N, MIN_NORMAL_Y, MaterialPatch, SegmentRole,
    SupportObservation, current_loaded, finite, norm, sub, transform_point,
};
use serde::{Deserialize, Serialize};

pub const SURFACE_NUMERIC_TOLERANCE_M: f64 = 1e-6;
pub const SURFACE_GROUND_DISTANCE_M: f64 = 0.012;
pub const FLAT_NORMAL_TOLERANCE: f64 = 1e-6;

/// Acquired once, from a loaded noncollinear face; never reconstructed after loss.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AcquiredMaterialSurface {
    pub origin_local_m: [f64; 3],
    pub unit_normal_local: [f64; 3],
    pub original_world_anchor_m: [f64; 3],
    pub acquired_substep: u64,
    pub support_height_m: f64,
    pub acquired_witness_contact_indices: [usize; 3],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum MaterialContinuityKind {
    DirectLoadedPoint,
    EquivalentLoadedSurface,
    Refused,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum ContinuityRefusal {
    NoAcquiredSurface,
    UnsupportedContactMode,
    UnsupportedSegmentRole,
    AcquiredMaterialReferenceChanged,
    InvalidAcquiredSurface,
    GroundPlaneChanged,
    AcquiredFaceNormalNotUpward,
    OriginalPointOutsideGroundDistance,
    FewerThanThreeLoadedSurfaceContacts,
    NonFlatGroundNormal,
    InconsistentLocalWorldPoint,
    DifferentMaterialPlane,
    ContactOutsideGroundDistance,
    CollinearLoadedSurface,
    OriginalPointOutsideLoadedHull,
}

/// Serializable diagnostics, deliberately NOT deserializable authorization.
/// Consumers with observations must recompute with `material_continuity`.
#[derive(Clone, Debug, Serialize)]
pub struct MaterialContinuityEvidence {
    pub native_substep: u64,
    pub patch_id: u64,
    pub segment: usize,
    pub kind: MaterialContinuityKind,
    pub nearest_loaded_local_distance_m: Option<f64>,
    pub nearest_loaded_contact_index: Option<usize>,
    pub witness_contact_indices: Vec<usize>,
    pub eligible_contact_indices: Vec<usize>,
    pub rejected_contacts: Vec<(usize, ContinuityRefusal)>,
    pub total_current_surface_load_n: f64,
    pub original_point_ground_distance_m: f64,
    pub refusal_reason: Option<ContinuityRefusal>,
}
impl MaterialContinuityEvidence {
    pub fn supported(&self) -> bool {
        self.kind != MaterialContinuityKind::Refused
    }
    fn refuse(mut self, reason: ContinuityRefusal) -> Self {
        self.kind = MaterialContinuityKind::Refused;
        self.refusal_reason = Some(reason);
        self
    }
}

impl MaterialPatch {
    /// Cached diagnostics only. A forged or stale direct receipt cannot override
    /// an explicitly absent direct point. Imported receipts are discarded.
    pub fn has_current_support(&self) -> bool {
        self.current_contact_present
            || (self.mode == ContactMode::Hold
                && self.continuity_evidence.as_ref().is_some_and(|e| {
                    e.kind == MaterialContinuityKind::EquivalentLoadedSurface
                        && e.native_substep == self.last_observed_substep
                        && e.patch_id == self.id
                        && e.segment == self.segment
                }))
    }

    /// Actual boundary: recompute all support evidence from the given observation.
    pub fn verify_current_support(
        &self,
        observation: &SupportObservation,
        material_match_tolerance_m: f64,
    ) -> Result<bool, String> {
        Ok(self
            .material_continuity(observation, material_match_tolerance_m)?
            .supported())
    }

    pub fn material_continuity(
        &self,
        observation: &SupportObservation,
        material_match_tolerance_m: f64,
    ) -> Result<MaterialContinuityEvidence, String> {
        observation.validate()?;
        if self.segment >= observation.bodies.len()
            || !material_match_tolerance_m.is_finite()
            || material_match_tolerance_m < 0.0
            || observation.native_substep < self.acquired_substep
            || observation.native_substep < self.last_observed_substep
            || !finite(self.local_point_m)
            || !finite(self.world_anchor_m)
        {
            return Err("Invalid material-continuity observation request".into());
        }
        let body = &observation.bodies[self.segment];
        let contacts = &observation.segments[self.segment].contacts;
        let original_world = transform_point(body, self.local_point_m);
        if !finite(original_world) {
            return Err("Nonfinite material-continuity transform".into());
        }
        let nearest = contacts
            .iter()
            .enumerate()
            .filter(|(_, c)| current_loaded(c))
            .map(|(i, c)| (i, norm(sub(c.local_point_m, self.local_point_m))))
            .min_by(|a, b| a.1.total_cmp(&b.1));
        if nearest.is_some_and(|(_, d)| !d.is_finite()) {
            return Err("Nonfinite material-continuity correspondence".into());
        }
        let mut evidence = MaterialContinuityEvidence {
            native_substep: observation.native_substep,
            patch_id: self.id,
            segment: self.segment,
            kind: MaterialContinuityKind::Refused,
            nearest_loaded_local_distance_m: nearest.map(|(_, d)| d),
            nearest_loaded_contact_index: nearest.map(|(i, _)| i),
            witness_contact_indices: Vec::new(),
            eligible_contact_indices: Vec::new(),
            rejected_contacts: Vec::new(),
            total_current_surface_load_n: 0.0,
            original_point_ground_distance_m: (original_world[1] - observation.support_height_m)
                .abs(),
            refusal_reason: None,
        };
        // Preserve the prior direct loaded-point rule exactly, including its
        // existing normal threshold and caller's correspondence tolerance.
        if let Some((index, distance)) = nearest.filter(|(_, d)| *d <= material_match_tolerance_m) {
            let _ = distance;
            evidence.kind = MaterialContinuityKind::DirectLoadedPoint;
            evidence.witness_contact_indices.push(index);
            return Ok(evidence);
        }
        if self.mode != ContactMode::Hold {
            return Ok(evidence.refuse(ContinuityRefusal::UnsupportedContactMode));
        }
        if !matches!(
            observation.segments[self.segment].role,
            SegmentRole::Hindfoot | SegmentRole::Forefoot
        ) {
            return Ok(evidence.refuse(ContinuityRefusal::UnsupportedSegmentRole));
        }
        let Some(surface) = &self.reference_surface else {
            return Ok(evidence.refuse(ContinuityRefusal::NoAcquiredSurface));
        };
        if surface.origin_local_m != self.local_point_m
            || surface.original_world_anchor_m != self.world_anchor_m
            || surface.acquired_substep != self.acquired_substep
        {
            return Ok(evidence.refuse(ContinuityRefusal::AcquiredMaterialReferenceChanged));
        }
        if !finite(surface.unit_normal_local)
            || (norm(surface.unit_normal_local) - 1.0).abs() > 1e-6
            || !surface.support_height_m.is_finite()
        {
            return Ok(evidence.refuse(ContinuityRefusal::InvalidAcquiredSurface));
        }
        let rotation_only = BodyMotion {
            position_m: [0.0; 3],
            ..body.clone()
        };
        let face_normal_world = transform_point(&rotation_only, surface.unit_normal_local);
        if !finite(face_normal_world) || face_normal_world[1] < MIN_NORMAL_Y {
            return Ok(evidence.refuse(ContinuityRefusal::AcquiredFaceNormalNotUpward));
        }
        if (observation.support_height_m - surface.support_height_m).abs()
            > SURFACE_NUMERIC_TOLERANCE_M
        {
            return Ok(evidence.refuse(ContinuityRefusal::GroundPlaneChanged));
        }
        if evidence.original_point_ground_distance_m > SURFACE_GROUND_DISTANCE_M {
            return Ok(evidence.refuse(ContinuityRefusal::OriginalPointOutsideGroundDistance));
        }
        for (index, contact) in contacts.iter().enumerate() {
            // Three individually non-negligible vertices: persistent segment load
            // and almost unloaded corners cannot enlarge this evidence hull.
            if contact.current_upward_load_n < MIN_LOAD_N {
                continue;
            }
            let reason =
                contact_rejection(contact, body, observation.support_height_m).or_else(|| {
                    let distance = dot(
                        sub(contact.local_point_m, surface.origin_local_m),
                        surface.unit_normal_local,
                    );
                    (!distance.is_finite() || distance.abs() > SURFACE_NUMERIC_TOLERANCE_M)
                        .then_some(ContinuityRefusal::DifferentMaterialPlane)
                });
            if let Some(reason) = reason {
                evidence.rejected_contacts.push((index, reason));
            } else {
                evidence.eligible_contact_indices.push(index);
                evidence.total_current_surface_load_n += contact.current_upward_load_n;
            }
        }
        if evidence.eligible_contact_indices.len() < 3 {
            let reason = evidence
                .rejected_contacts
                .first()
                .map(|(_, r)| *r)
                .unwrap_or(ContinuityRefusal::FewerThanThreeLoadedSurfaceContacts);
            return Ok(evidence.refuse(reason));
        }
        if !evidence.total_current_surface_load_n.is_finite() {
            return Err("Nonfinite current material-surface load".into());
        }
        let mut noncollinear = false;
        let indices = &evidence.eligible_contact_indices;
        for a in 0..indices.len() - 2 {
            for b in a + 1..indices.len() - 1 {
                for c in b + 1..indices.len() {
                    let triangle = [indices[a], indices[b], indices[c]];
                    let vertices = triangle.map(|i| contacts[i].local_point_m);
                    if triangle_normal(vertices).is_none() {
                        continue;
                    }
                    noncollinear = true;
                    if triangle_contains(vertices, self.local_point_m, surface.unit_normal_local) {
                        evidence.kind = MaterialContinuityKind::EquivalentLoadedSurface;
                        evidence.witness_contact_indices = triangle.to_vec();
                        return Ok(evidence);
                    }
                }
            }
        }
        Ok(evidence.refuse(if noncollinear {
            ContinuityRefusal::OriginalPointOutsideLoadedHull
        } else {
            ContinuityRefusal::CollinearLoadedSurface
        }))
    }
}

/// Acquisition requires a complete observed coplanar loaded set, with a robust
/// triangle including the selected original point. Failure leaves legacy/direct
/// support intact; no later observe call can manufacture this reference.
pub(super) fn acquire_surface(
    observation: &SupportObservation,
    segment: usize,
    contact_index: usize,
    mode: ContactMode,
) -> Option<AcquiredMaterialSurface> {
    if mode != ContactMode::Hold
        || !matches!(
            observation.segments.get(segment)?.role,
            SegmentRole::Hindfoot | SegmentRole::Forefoot
        )
    {
        return None;
    }
    let body = observation.bodies.get(segment)?;
    let contacts = &observation.segments.get(segment)?.contacts;
    let selected = contacts.get(contact_index)?;
    if selected.current_upward_load_n < MIN_LOAD_N
        || contact_rejection(selected, body, observation.support_height_m).is_some()
    {
        return None;
    }
    let eligible: Vec<_> = contacts
        .iter()
        .enumerate()
        .filter(|(_, c)| c.current_upward_load_n >= MIN_LOAD_N)
        .map(|(i, c)| (i, c))
        .collect();
    if eligible.len() < 3
        || eligible
            .iter()
            .any(|(_, c)| contact_rejection(c, body, observation.support_height_m).is_some())
    {
        return None;
    }
    for a in 0..eligible.len() {
        for b in a + 1..eligible.len() {
            let (ia, ca) = eligible[a];
            let (ib, cb) = eligible[b];
            if ia == contact_index || ib == contact_index {
                continue;
            }
            let Some(mut normal) =
                triangle_normal([selected.local_point_m, ca.local_point_m, cb.local_point_m])
            else {
                continue;
            };
            let rotation_only = BodyMotion {
                position_m: [0.0; 3],
                ..body.clone()
            };
            let world_normal = transform_point(&rotation_only, normal);
            if world_normal[1] < 0.0 {
                normal = normal.map(|v| -v);
            }
            if world_normal[1].abs() < MIN_NORMAL_Y {
                continue;
            }
            if eligible.iter().any(|(_, c)| {
                dot(sub(c.local_point_m, selected.local_point_m), normal).abs()
                    > SURFACE_NUMERIC_TOLERANCE_M
            }) {
                continue;
            }
            return Some(AcquiredMaterialSurface {
                origin_local_m: selected.local_point_m,
                unit_normal_local: normal,
                original_world_anchor_m: selected.world_point_m,
                acquired_substep: observation.native_substep,
                support_height_m: observation.support_height_m,
                acquired_witness_contact_indices: [contact_index, ia, ib],
            });
        }
    }
    None
}

fn contact_rejection(
    c: &CurrentContact,
    body: &BodyMotion,
    support_height: f64,
) -> Option<ContinuityRefusal> {
    if c.upward_normal < 1.0 - FLAT_NORMAL_TOLERANCE {
        return Some(ContinuityRefusal::NonFlatGroundNormal);
    }
    let transformed = transform_point(body, c.local_point_m);
    if !finite(transformed) || norm(sub(transformed, c.world_point_m)) > SURFACE_NUMERIC_TOLERANCE_M
    {
        return Some(ContinuityRefusal::InconsistentLocalWorldPoint);
    }
    if (c.world_point_m[1] - support_height).abs() > SURFACE_GROUND_DISTANCE_M {
        return Some(ContinuityRefusal::ContactOutsideGroundDistance);
    }
    None
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn triangle_normal(v: [[f64; 3]; 3]) -> Option<[f64; 3]> {
    let a = sub(v[1], v[0]);
    let b = sub(v[2], v[0]);
    let c = sub(v[2], v[1]);
    let n = cross(a, b);
    let area = norm(n);
    let edge = norm(a).max(norm(b)).max(norm(c));
    if !area.is_finite()
        || !edge.is_finite()
        || edge <= SURFACE_NUMERIC_TOLERANCE_M
        || area <= SURFACE_NUMERIC_TOLERANCE_M * edge
    {
        return None;
    }
    Some(n.map(|v| v / area))
}
fn triangle_contains(v: [[f64; 3]; 3], p: [f64; 3], normal: [f64; 3]) -> bool {
    let orientation = dot(cross(sub(v[1], v[0]), sub(v[2], v[0])), normal).signum();
    orientation != 0.0
        && (0..3).all(|i| {
            let edge = sub(v[(i + 1) % 3], v[i]);
            let length = norm(edge);
            let signed = orientation * dot(cross(edge, sub(p, v[i])), normal);
            signed.is_finite()
                && length.is_finite()
                && length > 0.0
                && signed >= -SURFACE_NUMERIC_TOLERANCE_M * length
        })
}

#[cfg(test)]
#[path = "material_continuity_tests.rs"]
mod tests;
