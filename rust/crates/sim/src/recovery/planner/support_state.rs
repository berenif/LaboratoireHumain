//! Pure support bookkeeping for the experimental planner.
//!
//! Inputs are measured observations, never requested contacts. Whole-substep CCD
//! impulses are kept separate from current solver points and cannot certify a
//! retained support. These diagnostics do not replace the recovery observer.
use serde::{Deserialize, Serialize};

pub const BODY_COUNT: usize = 25;
pub const NATIVE_HZ: u64 = 240;
pub const MIN_LOAD_N: f64 = 3.0;
pub const LOAD_SUBSTEPS: u64 = 12;
pub const STABLE_SUBSTEPS: u64 = 132;
pub const MIN_SUPPORT_FRACTION: f64 = 0.8;
pub const MAX_RELEASE_FRACTION: f64 = 0.25;
pub const SUPPORT_MARGIN_M: f64 = 0.003;
pub const MAX_LINEAR_MPS: f64 = 0.22;
pub const MAX_ANGULAR_RADPS: f64 = 0.65;
pub const MIN_NORMAL_Y: f64 = 0.65;
pub const HEEL_LOCAL_Z_M: f64 = -0.02;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ContactMode {
    Hold,
    Pivot,
    Unload,
    Free,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SegmentRole {
    Hindfoot,
    Forefoot,
    Hand,
    Shin,
    Other,
}

impl SegmentRole {
    fn admitted_support(self) -> bool {
        !matches!(self, Self::Other)
    }

    fn foot(self) -> bool {
        matches!(self, Self::Hindfoot | Self::Forefoot)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BodyMotion {
    pub segment: usize,
    /// Body-frame translation and quaternion, not the center-of-mass frame.
    pub position_m: [f64; 3],
    pub rotation_xyzw: [f64; 4],
    pub linear_velocity_mps: [f64; 3],
    pub angular_velocity_radps: [f64; 3],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CurrentContact {
    pub local_point_m: [f64; 3],
    pub world_point_m: [f64; 3],
    pub upward_normal: f64,
    /// Current selected solver impulse divided by the enclosing native dt,
    /// matching the original observer. This is not a whole-step average force
    /// when CCD has several intervals. Earlier intervals belong in ccd_impulses.
    pub current_upward_load_n: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SegmentContactState {
    pub segment: usize,
    pub id: String,
    pub role: SegmentRole,
    pub persistent_load_n: f64,
    /// Original segment-level consecutive loaded-substep count.
    pub loaded_substeps: u64,
    /// Only still-present points selected by the current solver boundary.
    pub contacts: Vec<CurrentContact>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CcdImpulse {
    pub native_substep: u64,
    pub ccd_interval: usize,
    pub segment: usize,
    pub interval_s: f64,
    pub normal_ns: [f64; 3],
    pub tangent_ns: [f64; 3],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SupportObservation {
    /// End-of-substep observation, consecutive within a diagnostic history.
    pub native_substep: u64,
    /// Elapsed time in this history, including its physically replayed prefix.
    pub time_s: f64,
    pub weight_n: f64,
    pub center_of_mass_m: [f64; 3],
    pub center_of_mass_velocity_mps: [f64; 3],
    pub support_height_m: f64,
    /// Both arrays have exactly 25 entries in canonical segment order.
    pub bodies: Vec<BodyMotion>,
    pub segments: Vec<SegmentContactState>,
    pub ccd_impulses: Vec<CcdImpulse>,
}

impl SupportObservation {
    pub fn validate(&self) -> Result<(), String> {
        if self.bodies.len() != BODY_COUNT || self.segments.len() != BODY_COUNT {
            return Err("Support observation requires all 25 bodies and segments".into());
        }
        if !self.time_s.is_finite()
            || self.time_s < 0.0
            || !self.weight_n.is_finite()
            || self.weight_n <= 0.0
            || !finite(self.center_of_mass_m)
            || !finite(self.center_of_mass_velocity_mps)
            || !self.support_height_m.is_finite()
        {
            return Err("Invalid support observation state".into());
        }
        for (i, (body, segment)) in self.bodies.iter().zip(&self.segments).enumerate() {
            if body.segment != i
                || segment.segment != i
                || segment.id.is_empty()
                || self.segments[..i]
                    .iter()
                    .any(|other| other.id == segment.id)
            {
                return Err("Support observation canonical segment order mismatch".into());
            }
            if !finite(body.position_m)
                || !body.rotation_xyzw.iter().all(|v| v.is_finite())
                || (body.rotation_xyzw.iter().map(|v| v * v).sum::<f64>() - 1.0).abs() > 1e-5
                || !finite(body.linear_velocity_mps)
                || !finite(body.angular_velocity_radps)
                || !norm(body.linear_velocity_mps).is_finite()
                || !norm(body.angular_velocity_radps).is_finite()
                || !segment.persistent_load_n.is_finite()
                || segment.persistent_load_n < 0.0
            {
                return Err(format!("Invalid body/support state for {}", segment.id));
            }
            for contact in &segment.contacts {
                if !finite(contact.local_point_m)
                    || !finite(contact.world_point_m)
                    || !contact.upward_normal.is_finite()
                    || !(-1.0..=1.0).contains(&contact.upward_normal)
                    || !contact.current_upward_load_n.is_finite()
                    || contact.current_upward_load_n < 0.0
                {
                    return Err(format!("Invalid current support point for {}", segment.id));
                }
            }
        }
        let current_total: f64 = self
            .segments
            .iter()
            .flat_map(|s| &s.contacts)
            .map(|c| c.current_upward_load_n)
            .sum();
        let persistent_total: f64 = self.segments.iter().map(|s| s.persistent_load_n).sum();
        if !current_total.is_finite()
            || !persistent_total.is_finite()
            || !finite(self.capture_point_m())
        {
            return Err("Nonfinite derived support observation".into());
        }
        for impulse in &self.ccd_impulses {
            if impulse.native_substep != self.native_substep
                || impulse.segment >= BODY_COUNT
                || !impulse.interval_s.is_finite()
                || impulse.interval_s <= 0.0
                || !finite(impulse.normal_ns)
                || !finite(impulse.tangent_ns)
            {
                return Err("Invalid or stale CCD impulse journal".into());
            }
        }
        Ok(())
    }

    pub fn capture_point_m(&self) -> [f64; 3] {
        let scale = ((self.center_of_mass_m[1] - self.support_height_m).max(0.2) / 9.81).sqrt();
        std::array::from_fn(|i| {
            self.center_of_mass_m[i] + self.center_of_mass_velocity_mps[i] * scale
        })
    }
}

/// A remembered material point. The anchor is never refreshed from a new
/// manifold point. Pivot replacement is an explicit, archived operation.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MaterialPatch {
    pub id: u64,
    pub segment: usize,
    pub mode: ContactMode,
    pub local_point_m: [f64; 3],
    pub world_anchor_m: [f64; 3],
    pub last_world_point_m: [f64; 3],
    pub acquired_substep: u64,
    pub last_observed_substep: u64,
    pub displacement_from_anchor_m: [f64; 3],
    pub tangential_drift_m: f64,
    pub nearest_current_local_distance_m: Option<f64>,
    pub current_contact_present: bool,
}

impl MaterialPatch {
    pub fn observe(
        &mut self,
        observation: &SupportObservation,
        material_match_tolerance_m: f64,
    ) -> Result<(), String> {
        observation.validate()?;
        if self.segment >= BODY_COUNT
            || !material_match_tolerance_m.is_finite()
            || material_match_tolerance_m < 0.0
            || observation.native_substep < self.last_observed_substep
        {
            return Err("Invalid material-patch observation request".into());
        }
        let body = &observation.bodies[self.segment];
        let last_world_point_m = transform_point(body, self.local_point_m);
        let displacement_from_anchor_m: [f64; 3] =
            std::array::from_fn(|i| last_world_point_m[i] - self.world_anchor_m[i]);
        let tangential_drift_m = displacement_from_anchor_m[0].hypot(displacement_from_anchor_m[2]);
        if !finite(last_world_point_m)
            || !finite(displacement_from_anchor_m)
            || !tangential_drift_m.is_finite()
        {
            return Err("Nonfinite material-patch displacement".into());
        }
        let nearest_current_local_distance_m = observation.segments[self.segment]
            .contacts
            .iter()
            .filter(|contact| current_loaded(contact))
            .map(|contact| norm(sub(contact.local_point_m, self.local_point_m)))
            .min_by(f64::total_cmp);
        if nearest_current_local_distance_m.is_some_and(|distance| !distance.is_finite()) {
            return Err("Nonfinite material-patch correspondence".into());
        }
        self.last_world_point_m = last_world_point_m;
        self.displacement_from_anchor_m = displacement_from_anchor_m;
        self.tangential_drift_m = tangential_drift_m;
        self.nearest_current_local_distance_m = nearest_current_local_distance_m;
        self.current_contact_present = nearest_current_local_distance_m
            .is_some_and(|distance| distance <= material_match_tolerance_m);
        self.last_observed_substep = observation.native_substep;
        Ok(())
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct MaterialTracker {
    pub active: Vec<MaterialPatch>,
    pub archived: Vec<MaterialPatch>,
    next_id: u64,
}

impl MaterialTracker {
    pub fn acquire(
        &mut self,
        observation: &SupportObservation,
        segment: usize,
        contact_index: usize,
        mode: ContactMode,
    ) -> Result<u64, String> {
        observation.validate()?;
        if matches!(mode, ContactMode::Free) {
            return Err("Cannot acquire a free support patch".into());
        }
        let contact = observation
            .segments
            .get(segment)
            .and_then(|s| s.contacts.get(contact_index))
            .filter(|c| current_loaded(c))
            .ok_or("Acquisition requires a currently loaded observed contact")?;
        let id = self.next_id;
        self.next_id = self.next_id.checked_add(1).ok_or("Patch id exhausted")?;
        self.active.push(MaterialPatch {
            id,
            segment,
            mode,
            local_point_m: contact.local_point_m,
            world_anchor_m: contact.world_point_m,
            last_world_point_m: contact.world_point_m,
            acquired_substep: observation.native_substep,
            last_observed_substep: observation.native_substep,
            displacement_from_anchor_m: [0.0; 3],
            tangential_drift_m: 0.0,
            nearest_current_local_distance_m: Some(0.0),
            current_contact_present: true,
        });
        Ok(id)
    }

    pub fn replace_in_pivot(
        &mut self,
        old_id: u64,
        observation: &SupportObservation,
        contact_index: usize,
    ) -> Result<u64, String> {
        let index = self
            .active
            .iter()
            .position(|patch| patch.id == old_id && patch.mode == ContactMode::Pivot)
            .ok_or("Only an explicitly pivoting patch can be replaced")?;
        let mut archived = self.active[index].clone();
        // Validate the complete replacement before changing either collection.
        archived.observe(observation, 0.0)?;
        let new_id = self.acquire(
            observation,
            self.active[index].segment,
            contact_index,
            ContactMode::Pivot,
        )?;
        // Preserve the old material trajectory in the same observation boundary.
        // Matching is only diagnostic, so a zero tolerance does not authorize
        // the replacement or alter its old local point.
        self.active.remove(index);
        self.archived.push(archived);
        Ok(new_id)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TransferEvidence {
    pub releasing: Vec<usize>,
    pub retained_segments: Vec<usize>,
    pub released_load_n: f64,
    pub remaining_current_load_n: f64,
    pub support_points_xz_m: Vec<[f64; 2]>,
    pub com_inside: bool,
    pub capture_inside: bool,
    pub allowed: bool,
}

/// Same support exclusions for hands, shins, or an entire released leg.
/// Current load + original segment persistence + inset hull are evidence for
/// this gate, not a general dynamic or actuator-feasibility certificate.
pub fn supported_transfer(
    observation: &SupportObservation,
    releasing: &[usize],
) -> Result<TransferEvidence, String> {
    observation.validate()?;
    if releasing.is_empty()
        || releasing.iter().any(|segment| *segment >= BODY_COUNT)
        || releasing
            .iter()
            .enumerate()
            .any(|(i, s)| releasing[..i].contains(s))
    {
        return Err("Release must name distinct canonical segments".into());
    }
    let released_load_n = releasing
        .iter()
        .map(|i| persistent_load(&observation.segments[*i]))
        .sum();
    let retained_segments: Vec<_> = observation
        .segments
        .iter()
        .filter(|segment| {
            !releasing.contains(&segment.segment)
                && segment.role.admitted_support()
                && persistent_load(segment) >= MIN_LOAD_N
        })
        .map(|segment| segment.segment)
        .collect();
    let (remaining_current_load_n, support_points_xz_m) =
        current_support(observation, &retained_segments);
    let com_inside = inside_support(&support_points_xz_m, xz(observation.center_of_mass_m));
    let capture_inside = inside_support(&support_points_xz_m, xz(observation.capture_point_m()));
    Ok(TransferEvidence {
        releasing: releasing.to_vec(),
        retained_segments,
        released_load_n,
        remaining_current_load_n,
        support_points_xz_m,
        com_inside,
        capture_inside,
        allowed: released_load_n <= MAX_RELEASE_FRACTION * observation.weight_n
            && remaining_current_load_n >= MIN_SUPPORT_FRACTION * observation.weight_n
            && com_inside
            && capture_inside,
    })
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct RetainedSupports {
    /// Unload requests remain active tasks until a release proof succeeds.
    pub tasks: Vec<(usize, ContactMode)>,
}

impl RetainedSupports {
    pub fn request_release(
        &mut self,
        observation: &SupportObservation,
        releasing: &[usize],
    ) -> Result<TransferEvidence, String> {
        if releasing.iter().any(|i| {
            !self
                .tasks
                .iter()
                .any(|(segment, mode)| segment == i && *mode != ContactMode::Free)
        }) {
            return Err("Requested release is not an active retained task".into());
        }
        let proof = supported_transfer(observation, releasing)?;
        if proof.allowed {
            self.tasks.retain(|(i, _)| !releasing.contains(i));
        } else {
            for (i, mode) in &mut self.tasks {
                if releasing.contains(i) {
                    *mode = ContactMode::Unload;
                }
            }
        }
        Ok(proof)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LocalSupportSample {
    pub native_substep: u64,
    pub heel_loads_n: [f64; 2],
    pub heel_loaded_substeps: [u64; 2],
    pub heel_persistent: [bool; 2],
    pub support_load_n: f64,
    pub support_fraction: f64,
    pub nonfoot_persistent_segments: Vec<usize>,
    pub nonfoot_current_load_n: f64,
    pub com_inside: bool,
    pub capture_inside: bool,
    pub maximum_linear_mps: f64,
    pub maximum_angular_radps: f64,
    pub all_conditions_now: bool,
    pub consecutive_stable_substeps: u64,
    pub first_local_conditions_s: Option<f64>,
    pub passed: bool,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct LocalSupportDiagnostic {
    pub heel_loaded_substeps: [u64; 2],
    pub consecutive_stable_substeps: u64,
    /// Start of the currently uninterrupted stable interval.
    pub stable_started_s: Option<f64>,
    pub first_local_conditions_s: Option<f64>,
    pub passed: bool,
    last_native_substep: Option<u64>,
    last_time_s: Option<f64>,
}

impl LocalSupportDiagnostic {
    pub fn observe(
        &mut self,
        observation: &SupportObservation,
    ) -> Result<LocalSupportSample, String> {
        observation.validate()?;
        if self
            .last_native_substep
            .is_some_and(|last| last.checked_add(1) != Some(observation.native_substep))
            || self.last_time_s.is_some_and(|last| {
                (observation.time_s - last - 1.0 / NATIVE_HZ as f64).abs() > 1e-7
            })
        {
            return Err("Local support diagnostic requires consecutive native substeps".into());
        }
        let mut heel_loads_n = [0.0; 2];
        for (side, id) in ["leftFoot", "rightFoot"].iter().enumerate() {
            let segment = observation
                .segments
                .iter()
                .find(|s| s.id == *id)
                .ok_or_else(|| format!("Missing canonical hindfoot {id}"))?;
            if segment.role != SegmentRole::Hindfoot {
                return Err(format!("Canonical foot {id} has incorrect role"));
            }
            heel_loads_n[side] = segment
                .contacts
                .iter()
                .filter(|c| current_loaded(c) && c.local_point_m[2] < HEEL_LOCAL_Z_M)
                .map(|c| c.current_upward_load_n)
                .sum();
        }
        for (count, load) in self.heel_loaded_substeps.iter_mut().zip(heel_loads_n) {
            *count = if load >= MIN_LOAD_N {
                count.saturating_add(1)
            } else {
                0
            };
        }
        let heel_persistent = self
            .heel_loaded_substeps
            .map(|steps| steps >= LOAD_SUBSTEPS);
        let feet: Vec<_> = observation
            .segments
            .iter()
            .filter(|s| s.role.foot() && persistent_load(s) >= MIN_LOAD_N)
            .map(|s| s.segment)
            .collect();
        let (support_load_n, points) = current_support(observation, &feet);
        let nonfoot_persistent_segments: Vec<_> = observation
            .segments
            .iter()
            .filter(|s| !s.role.foot() && persistent_load(s) >= MIN_LOAD_N)
            .map(|s| s.segment)
            .collect();
        let nonfoot_current_load_n = observation
            .segments
            .iter()
            .filter(|s| !s.role.foot())
            .flat_map(|s| s.contacts.iter())
            .filter(|c| current_loaded(c))
            .map(|c| c.current_upward_load_n)
            .sum();
        let com_inside = inside_support(&points, xz(observation.center_of_mass_m));
        let capture_inside = inside_support(&points, xz(observation.capture_point_m()));
        let maximum_linear_mps = observation
            .bodies
            .iter()
            .map(|b| norm(b.linear_velocity_mps))
            .fold(0.0_f64, f64::max);
        let maximum_angular_radps = observation
            .bodies
            .iter()
            .map(|b| norm(b.angular_velocity_radps))
            .fold(0.0_f64, f64::max);
        let all_conditions_now = heel_persistent.iter().all(|loaded| *loaded)
            && support_load_n >= MIN_SUPPORT_FRACTION * observation.weight_n
            && nonfoot_persistent_segments.is_empty()
            && com_inside
            && capture_inside
            && maximum_linear_mps <= MAX_LINEAR_MPS
            && maximum_angular_radps <= MAX_ANGULAR_RADPS;
        if all_conditions_now {
            self.consecutive_stable_substeps = self.consecutive_stable_substeps.saturating_add(1);
            self.stable_started_s.get_or_insert(observation.time_s);
            self.first_local_conditions_s
                .get_or_insert(observation.time_s);
        } else {
            self.consecutive_stable_substeps = 0;
            self.stable_started_s = None;
        }
        // A new stable interval beginning after the deadline cannot inherit an
        // earlier, interrupted acquisition. 0.55 s remains 132 full substeps.
        self.passed |= self.consecutive_stable_substeps >= STABLE_SUBSTEPS
            && self.stable_started_s.is_some_and(|time| time < 2.0);
        self.last_native_substep = Some(observation.native_substep);
        self.last_time_s = Some(observation.time_s);
        Ok(LocalSupportSample {
            native_substep: observation.native_substep,
            heel_loads_n,
            heel_loaded_substeps: self.heel_loaded_substeps,
            heel_persistent,
            support_load_n,
            support_fraction: support_load_n / observation.weight_n,
            nonfoot_persistent_segments,
            nonfoot_current_load_n,
            com_inside,
            capture_inside,
            maximum_linear_mps,
            maximum_angular_radps,
            all_conditions_now,
            consecutive_stable_substeps: self.consecutive_stable_substeps,
            first_local_conditions_s: self.first_local_conditions_s,
            passed: self.passed,
        })
    }
}

fn persistent_load(segment: &SegmentContactState) -> f64 {
    if segment.loaded_substeps >= LOAD_SUBSTEPS {
        segment.persistent_load_n
    } else {
        0.0
    }
}

fn current_loaded(contact: &CurrentContact) -> bool {
    contact.upward_normal >= MIN_NORMAL_Y && contact.current_upward_load_n > 0.0
}

fn current_support(observation: &SupportObservation, segments: &[usize]) -> (f64, Vec<[f64; 2]>) {
    let mut load = 0.0;
    let mut points = Vec::new();
    for &segment in segments {
        for contact in observation.segments[segment]
            .contacts
            .iter()
            .filter(|c| current_loaded(c))
        {
            load += contact.current_upward_load_n;
            points.push(xz(contact.world_point_m));
        }
    }
    (load, points)
}

/// Monotone-chain hull and 3 mm inset, with the same geometry as the original
/// gate. Neither a point, a line nor an axis-aligned bounding box is support.
fn inside_support(points: &[[f64; 2]], point: [f64; 2]) -> bool {
    if !point.iter().all(|v| v.is_finite()) || points.iter().flatten().any(|v| !v.is_finite()) {
        return false;
    }
    let mut points = points.to_vec();
    points.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    points.dedup();
    if points.len() < 3 {
        return false;
    }
    let cross = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    let mut hull = Vec::new();
    for point in &points {
        while hull.len() >= 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], *point) <= 0.0 {
            hull.pop();
        }
        hull.push(*point);
    }
    let lower = hull.len();
    for point in points.iter().rev().skip(1) {
        while hull.len() > lower && cross(hull[hull.len() - 2], hull[hull.len() - 1], *point) <= 0.0
        {
            hull.pop();
        }
        hull.push(*point);
    }
    hull.pop();
    hull.len() >= 3
        && hull.iter().zip(hull.iter().cycle().skip(1)).all(|(a, b)| {
            let length = (b[0] - a[0]).hypot(b[1] - a[1]);
            let signed_area = cross(*a, *b, point);
            length.is_finite()
                && length > 0.0
                && signed_area.is_finite()
                && signed_area >= SUPPORT_MARGIN_M * length
        })
}

fn finite(point: [f64; 3]) -> bool {
    point.iter().all(|v| v.is_finite())
}
fn norm(v: [f64; 3]) -> f64 {
    v[0].hypot(v[1]).hypot(v[2])
}
fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] - b[i])
}
fn xz(point: [f64; 3]) -> [f64; 2] {
    [point[0], point[2]]
}
fn transform_point(body: &BodyMotion, local: [f64; 3]) -> [f64; 3] {
    let [x, y, z, w] = body.rotation_xyzw;
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let uv = cross([x, y, z], local);
    let uuv = cross([x, y, z], uv);
    std::array::from_fn(|i| body.position_m[i] + local[i] + 2.0 * (w * uv[i] + uuv[i]))
}

#[cfg(test)]
mod tests {
    use super::*;

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
        for (i, id) in ["leftFoot", "rightFoot"].iter().enumerate() {
            segments[i].id = (*id).into();
            segments[i].role = SegmentRole::Hindfoot;
            segments[i].persistent_load_n = 50.0;
            segments[i].loaded_substeps = 12;
            let x = if i == 0 { -0.1 } else { 0.1 };
            for z in [-0.1, 0.1] {
                segments[i].contacts.push(CurrentContact {
                    local_point_m: [x, 0.0, z],
                    world_point_m: [x, 0.0, z],
                    upward_normal: 1.0,
                    current_upward_load_n: 25.0,
                });
            }
        }
        SupportObservation {
            native_substep: 0,
            time_s: 0.0,
            weight_n: 100.0,
            center_of_mass_m: [0.0, 0.7, 0.0],
            center_of_mass_velocity_mps: [0.0; 3],
            support_height_m: 0.0,
            bodies,
            segments,
            ccd_impulses: Vec::new(),
        }
    }

    fn tick(state: &mut SupportObservation) {
        state.native_substep += 1;
        state.time_s = state.native_substep as f64 / NATIVE_HZ as f64;
    }

    #[test]
    fn ccd_impulses_alone_never_prove_support() {
        let mut state = observation();
        for segment in &mut state.segments {
            segment.contacts.clear();
        }
        state.ccd_impulses.push(CcdImpulse {
            native_substep: 0,
            ccd_interval: 0,
            segment: 0,
            interval_s: 1.0 / 240.0,
            normal_ns: [0.0, 100.0, 0.0],
            tangent_ns: [0.0; 3],
        });
        let proof = supported_transfer(&state, &[2]).unwrap();
        assert!(!proof.allowed);
        assert_eq!(proof.remaining_current_load_n, 0.0);
        assert!(proof.support_points_xz_m.is_empty());
    }

    #[test]
    fn withdrawn_hand_cannot_supply_hull_or_load_and_remains_a_task() {
        let mut state = observation();
        for i in 0..2 {
            state.segments[i]
                .contacts
                .retain(|c| c.world_point_m[2] < 0.0);
            state.segments[i].contacts[0].current_upward_load_n = 45.0;
        }
        state.segments[2].role = SegmentRole::Hand;
        state.segments[2].persistent_load_n = 10.0;
        state.segments[2].loaded_substeps = 12;
        state.segments[2].contacts.push(CurrentContact {
            local_point_m: [0.0, 0.0, 0.2],
            world_point_m: [0.0, 0.0, 0.2],
            upward_normal: 1.0,
            current_upward_load_n: 10.0,
        });
        assert!(supported_transfer(&state, &[3]).unwrap().allowed);
        let mut retained = RetainedSupports {
            tasks: vec![(2, ContactMode::Hold)],
        };
        let proof = retained.request_release(&state, &[2]).unwrap();
        assert!(!proof.allowed);
        assert_eq!(proof.remaining_current_load_n, 90.0);
        assert_eq!(proof.support_points_xz_m.len(), 2);
        assert_eq!(retained.tasks, vec![(2, ContactMode::Unload)]);
    }

    #[test]
    fn successful_release_removes_task_only_after_remaining_support_proof() {
        let state = observation();
        let mut retained = RetainedSupports {
            tasks: vec![(2, ContactMode::Hold)],
        };
        assert!(retained.request_release(&state, &[2]).unwrap().allowed);
        assert!(retained.tasks.is_empty());
    }

    #[test]
    fn new_contact_never_reanchors_old_material_point() {
        let mut state = observation();
        let mut tracker = MaterialTracker::default();
        let id = tracker.acquire(&state, 0, 0, ContactMode::Hold).unwrap();
        let original_anchor = tracker.active[0].world_anchor_m;
        state.bodies[0].position_m[0] = 0.48;
        state.segments[0].contacts[0].local_point_m[0] -= 0.48;
        tracker.active[0].observe(&state, 0.001).unwrap();
        assert_eq!(tracker.active[0].world_anchor_m, original_anchor);
        assert!((tracker.active[0].tangential_drift_m - 0.48).abs() < 1e-12);
        assert!(!tracker.active[0].current_contact_present);
        assert!(tracker.replace_in_pivot(id, &state, 0).is_err());
        tracker.active[0].mode = ContactMode::Pivot;
        let next = tracker.replace_in_pivot(id, &state, 0).unwrap();
        assert_ne!(next, id);
        assert_eq!(tracker.archived[0].world_anchor_m, original_anchor);
        assert!((tracker.archived[0].tangential_drift_m - 0.48).abs() < 1e-12);
    }

    #[test]
    fn heel_has_own_persistence_and_full_stable_interval() {
        let mut state = observation();
        let mut diagnostic = LocalSupportDiagnostic::default();
        for step in 1..=142 {
            tick(&mut state);
            let sample = diagnostic.observe(&state).unwrap();
            assert_eq!(sample.heel_persistent, [step >= 12; 2]);
            assert!(!sample.passed);
        }
        tick(&mut state);
        let sample = diagnostic.observe(&state).unwrap();
        assert_eq!(sample.consecutive_stable_substeps, 132);
        assert!(sample.passed);
    }

    #[test]
    fn posterior_segment_label_and_old_segment_persistence_are_insufficient() {
        let mut state = observation();
        for segment in &mut state.segments[..2] {
            segment.contacts[0].local_point_m[2] = HEEL_LOCAL_Z_M;
            segment.loaded_substeps = 1000;
        }
        let mut diagnostic = LocalSupportDiagnostic::default();
        for _ in 0..20 {
            tick(&mut state);
            assert_eq!(
                diagnostic.observe(&state).unwrap().heel_loaded_substeps,
                [0; 2]
            );
        }
        for segment in &mut state.segments[..2] {
            segment.contacts[0].local_point_m[2] = -0.021;
        }
        tick(&mut state);
        assert_eq!(
            diagnostic.observe(&state).unwrap().heel_loaded_substeps,
            [1; 2]
        );
    }

    #[test]
    fn diagnostic_rejects_missing_body_skipped_step_and_nonfoot_dependency() {
        let mut state = observation();
        let mut diagnostic = LocalSupportDiagnostic::default();
        state.bodies[24].angular_velocity_radps[0] = 0.651;
        for _ in 0..12 {
            tick(&mut state);
            diagnostic.observe(&state).unwrap();
        }
        assert_eq!(diagnostic.consecutive_stable_substeps, 0);
        state.bodies[24].angular_velocity_radps[0] = 0.0;
        state.segments[2].persistent_load_n = 3.0;
        state.segments[2].loaded_substeps = 12;
        tick(&mut state);
        let sample = diagnostic.observe(&state).unwrap();
        assert_eq!(sample.nonfoot_persistent_segments, vec![2]);
        assert!(!sample.all_conditions_now);
        state.native_substep += 2;
        assert!(diagnostic.observe(&state).is_err());
        state.bodies.pop();
        assert!(state.validate().is_err());
    }

    #[test]
    fn interrupted_early_interval_cannot_authorize_late_success() {
        let mut state = observation();
        let mut diagnostic = LocalSupportDiagnostic::default();
        for _ in 0..30 {
            tick(&mut state);
            diagnostic.observe(&state).unwrap();
        }
        state.bodies[24].linear_velocity_mps[0] = 1.0;
        while state.native_substep < 480 {
            tick(&mut state);
            diagnostic.observe(&state).unwrap();
        }
        state.bodies[24].linear_velocity_mps[0] = 0.0;
        for _ in 0..132 {
            tick(&mut state);
            diagnostic.observe(&state).unwrap();
        }
        assert_eq!(diagnostic.consecutive_stable_substeps, 132);
        assert!(!diagnostic.passed);
    }

    #[test]
    fn hull_uses_inset_polygon_not_bounding_rectangle() {
        assert!(!inside_support(
            &[[-1.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
            [0.8, 0.8]
        ));
        assert!(!inside_support(
            &[[-1.0, 0.0], [0.0, 0.0], [1.0, 0.0]],
            [0.0, 0.0]
        ));
        let square = [[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]];
        assert!(inside_support(&square, [0.0, 0.0]));
        assert!(!inside_support(&square, [0.099, 0.0]));
    }

    #[test]
    fn finite_inputs_cannot_hide_overflow_in_support_evidence() {
        let mut state = observation();
        state.segments[0].contacts[0].current_upward_load_n = f64::MAX;
        state.segments[1].contacts[0].current_upward_load_n = f64::MAX;
        assert!(state.validate().is_err());
        state = observation();
        state.center_of_mass_m[1] = f64::MAX;
        state.support_height_m = -f64::MAX;
        assert!(state.validate().is_err());
        let giant = [
            [-1e160, -1e160],
            [1e160, -1e160],
            [1e160, 1e160],
            [-1e160, 1e160],
        ];
        assert!(!inside_support(&giant, [0.0, 0.0]));
    }

    #[test]
    fn failed_material_measurement_keeps_last_observation_unchanged() {
        let mut state = observation();
        let mut tracker = MaterialTracker::default();
        tracker.acquire(&state, 0, 0, ContactMode::Hold).unwrap();
        tracker.active[0].local_point_m[0] = f64::MAX;
        state.bodies[0].position_m[0] = f64::MAX;
        let previous = serde_json::to_value(&tracker).unwrap();
        assert!(tracker.active[0].observe(&state, 0.001).is_err());
        assert_eq!(serde_json::to_value(&tracker).unwrap(), previous);
    }
}
