//! Measured-state intentions before any whole-foot withdrawal.
//!
//! No Simulation/Model, FK, motor installer or native admission lives here.
//! Targets are geometric guidance through the shared 46-axis task compiler;
//! measured contacts and immutable material obligations are reported separately.
use serde::Serialize;

use super::{
    geometric_tasks as tasks,
    support_schedule::{ReleaseGroup, RequiredValidation},
    support_state::{
        self, ContactMode, CurrentContact, HEEL_LOCAL_Z_M, LOAD_SUBSTEPS, LocalSupportDiagnostic,
        LocalSupportSample, MIN_LOAD_N, MIN_NORMAL_Y, MaterialPatch, MaterialTracker, NATIVE_HZ,
        SegmentRole, SupportObservation, TransferEvidence,
    },
};

const POSITION_GAIN: f64 = 2.0;
const ROTATION_GAIN: f64 = 2.0;
const SPEED_LIMIT_MPS: f64 = 0.15;
const MATERIAL_MATCH_M: f64 = 0.012;
pub const HEEL_REFERENCE_SHA256: &str =
    "a96d03d736ca185a0457ae29c33846b69af28ecd5eb81609f60d281463756dd6";
pub const HEEL_REFERENCE_PATH: &str =
    "docs/evidence/coordination-prototype-2026-10-12/continuation-tasks.json";

#[derive(Clone, Debug, Serialize)]
pub struct HeelIntent {
    pub segment: usize,
    /// A vertex from the named archived reference, not a requested contact.
    pub local_point_m: [f64; 3],
    pub source_path: String,
    pub source_sha256: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum AfterHeels {
    ContinueBilateral,
    RecenterForWithdrawal(ReleaseGroup),
}

#[derive(Clone, Debug, Serialize)]
pub struct TransferIntent {
    pub heels: [HeelIntent; 2],
    pub after_heels: AfterHeels,
    pub deadline_substep: u64,
}

impl TransferIntent {
    pub fn from_archived_heel_reference(after_heels: AfterHeels, deadline_substep: u64) -> Self {
        Self {
            heels: [18, 23].map(|segment| HeelIntent {
                segment,
                local_point_m: [
                    -0.04399999976158142,
                    -0.04500000178813934,
                    -0.07500000298023224,
                ],
                source_path: HEEL_REFERENCE_PATH.into(),
                source_sha256: HEEL_REFERENCE_SHA256.into(),
            }),
            after_heels,
            deadline_substep,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum Stage {
    AcquireBilateralHeels,
    RecenterBilateral,
    BilateralContinuationReady,
    RecenterForWithdrawal,
    WithdrawalGateReady,
}

#[derive(Clone, Debug, Serialize)]
pub enum Obstacle {
    InteriorTargetUnavailable,
    HeelNotPersistentlyObserved { segment: usize },
    RetainedMaterialContactMissing { patch_id: u64 },
    UntrackedSupport { segment: usize },
    NonfootDependency { segment: usize },
    HeelPitchCannotLowerPoint { segment: usize },
}

#[derive(Clone, Debug, Serialize)]
pub struct HeelObservation {
    pub segment: usize,
    pub region_load_n: f64,
    pub region_loaded_substeps: u64,
    pub goal_matching_contact_index: Option<usize>,
    pub goal_matching_loaded_substeps: u64,
    pub first_material_local_point_m: Option<[f64; 3]>,
}

#[derive(Clone, Debug, Serialize)]
pub struct HeelTaskIntent {
    pub provenance: HeelIntent,
    pub observed_point_world_m: [f64; 3],
    pub height_above_observed_support_m: f64,
    pub world_velocity_mps: [f64; 3],
    pub world_angular_velocity_radps: [f64; 3],
}

#[derive(Clone, Debug, Serialize)]
pub struct MaterialIntent {
    pub patch_id: u64,
    /// Intent only; does not modify the observed patch or reanchor it.
    pub requested_mode: ContactMode,
}

#[derive(Clone, Debug, Serialize)]
pub struct TransferProposal {
    pub native_substep: u64,
    pub stage: Stage,
    pub continue_bilateral: bool,
    pub continuation_event_substep: Option<u64>,
    pub first_stable_substep: Option<u64>,
    pub required_validation: RequiredValidation,
    pub material_obligations: Vec<MaterialPatch>,
    pub material_intents: Vec<MaterialIntent>,
    pub heel_observations: [HeelObservation; 2],
    pub heel_intents: Vec<HeelTaskIntent>,
    pub measured_local_diagnostic: LocalSupportSample,
    pub target_support_segments: Vec<usize>,
    pub target_support_points_xz_m: Vec<[f64; 2]>,
    /// Deterministic center of the measured convex hull (or its degenerate
    /// points), never a requested heel/hand point added to the observed hull.
    pub com_capture_target_xz_m: [f64; 2],
    pub target_inside_original_margin: bool,
    pub com_velocity_mps: [f64; 3],
    pub measured_capture_xz_m: [f64; 2],
    pub original_transfer_gate: Option<TransferEvidence>,
    pub remaining_load_deficit_n: Option<f64>,
    pub released_load_excess_n: Option<f64>,
    pub obstacles: Vec<Obstacle>,
}

impl TransferProposal {
    /// Only task intentions change. This does not touch a motor, reference q/dq,
    /// command bound, owner clock, material journal or physical-admission gate.
    pub(crate) fn apply_to_goals(&self, goals: &mut tasks::TaskGoals) -> Result<(), String> {
        if self
            .obstacles
            .iter()
            .any(|o| matches!(o, Obstacle::RetainedMaterialContactMissing { .. }))
        {
            return Err("A retained material obligation lost its current contact".into());
        }
        if matches!(self.stage, Stage::BilateralContinuationReady) {
            return Ok(());
        }
        if self.material_obligations.is_empty()
            || !finite(self.com_velocity_mps)
            || self
                .heel_intents
                .iter()
                .any(|h| !finite(h.world_velocity_mps) || !finite(h.world_angular_velocity_radps))
        {
            return Err("Invalid load-transfer task proposal".into());
        }
        goals.com_velocity_mps = self.com_velocity_mps;
        goals.com_weight = 1.0;
        // Remove conflicting prior foot orientations while acquiring support;
        // whole-body posture and all nonfoot tasks remain shared and unchanged.
        goals
            .orientation_tasks
            .retain(|g| ![18, 19, 23, 24].contains(&g.segment));
        goals.point_tasks.retain(|g| ![18, 23].contains(&g.segment));
        for heel in &self.heel_intents {
            goals.point_tasks.push(tasks::PointGoal {
                segment: heel.provenance.segment,
                local_point_m: heel.provenance.local_point_m,
                world_velocity_mps: heel.world_velocity_mps,
                weight: 1.0,
            });
            goals.orientation_tasks.push(tasks::OrientationGoal {
                segment: heel.provenance.segment,
                world_angular_velocity_radps: heel.world_angular_velocity_radps,
                weight: 1.0,
            });
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Serialize)]
struct PointPersistence {
    first_local: [f64; 3],
    contact_index: usize,
    steps: u64,
}

#[derive(Clone, Debug, Serialize)]
pub struct Policy {
    intent: TransferIntent,
    initial_material_obligations: Vec<MaterialPatch>,
    latest: SupportObservation,
    diagnostic: LocalSupportDiagnostic,
    sample: LocalSupportSample,
    matching: [Option<PointPersistence>; 2],
    continuation_event_substep: Option<u64>,
    first_stable_substep: Option<u64>,
}

impl Policy {
    pub fn new(
        observation: &SupportObservation,
        retained: &MaterialTracker,
        intent: TransferIntent,
    ) -> Result<Self, String> {
        validate(observation, &intent)?;
        if intent.deadline_substep <= observation.native_substep {
            return Err("Load-transfer deadline must follow the initial observation".into());
        }
        // Import validation is pure and does not refresh the caller's anchors.
        super::support_schedule::SupportSchedule::from_retained(observation, retained)?;
        let mut diagnostic = LocalSupportDiagnostic::default();
        let sample = diagnostic.observe(observation)?;
        if retained
            .active
            .iter()
            .any(|p| [18, 23].contains(&p.segment) && p.local_point_m[2] >= HEEL_LOCAL_Z_M)
        {
            return Err(
                "Acquisition must not commit a front hindfoot point as a fixed heel pivot".into(),
            );
        }
        let mut policy = Self {
            intent,
            initial_material_obligations: retained.active.clone(),
            latest: observation.clone(),
            diagnostic,
            sample,
            matching: [None, None],
            continuation_event_substep: None,
            first_stable_substep: None,
        };
        policy.update_matching();
        Ok(policy)
    }

    pub fn observe(&mut self, observation: &SupportObservation) -> Result<(), String> {
        validate(observation, &self.intent)?;
        let mut next = self.clone();
        // The existing diagnostic checks the full 240 Hz clock and all bodies.
        next.sample = next.diagnostic.observe(observation)?;
        next.latest = observation.clone();
        next.update_matching();
        if next.continuation_event_substep.is_none()
            && next.sample.passed
            && next.sample.all_conditions_now
            && next
                .matching
                .iter()
                .all(|p| p.as_ref().is_some_and(|p| p.steps >= LOAD_SUBSTEPS))
        {
            next.continuation_event_substep = Some(observation.native_substep);
            next.first_stable_substep = Some(
                observation
                    .native_substep
                    .checked_sub(next.sample.consecutive_stable_substeps - 1)
                    .ok_or("Stable interval precedes the native clock")?,
            );
        }
        *self = next;
        Ok(())
    }

    /// First measured local milestone, latched through the subsequent rise.
    pub fn continuation_event(&self) -> Option<u64> {
        self.continuation_event_substep
    }
    pub fn first_stable_substep(&self) -> Option<u64> {
        self.first_stable_substep
    }

    pub fn propose(&self, retained: &MaterialTracker) -> Result<TransferProposal, String> {
        let observation = &self.latest;
        if observation.native_substep > self.intent.deadline_substep
            && self.continuation_event_substep.is_none()
        {
            return Err(
                "Load-transfer acquisition/stability deadline reached without measured success"
                    .into(),
            );
        }
        if retained.active.is_empty()
            || retained.active.iter().any(|p| {
                p.segment >= 25
                    || p.mode == ContactMode::Free
                    || p.last_observed_substep != observation.native_substep
                    || p.acquired_substep > p.last_observed_substep
                    || !finite(p.local_point_m)
                    || !finite(p.world_anchor_m)
            })
        {
            return Err("Load transfer requires current, explicit material obligations".into());
        }
        // This policy plans no release or reanchoring. Its original identities
        // must survive even if the caller owns the mutable tracking structure.
        for initial in &self.initial_material_obligations {
            if !retained.active.iter().any(|current| {
                current.id == initial.id
                    && current.segment == initial.segment
                    && current.local_point_m == initial.local_point_m
                    && current.world_anchor_m == initial.world_anchor_m
                    && current.acquired_substep == initial.acquired_substep
            }) {
                return Err(
                    "An original load-transfer obligation was removed or reanchored".into(),
                );
            }
        }
        for (index, patch) in retained.active.iter().enumerate() {
            if retained.active[..index]
                .iter()
                .any(|p| p.id == patch.id || p.segment == patch.segment)
            {
                return Err("Duplicate load-transfer material obligation".into());
            }
            if !self
                .initial_material_obligations
                .iter()
                .any(|p| p.id == patch.id)
            {
                let side = self
                    .intent
                    .heels
                    .iter()
                    .position(|h| h.segment == patch.segment)
                    .ok_or("Only measured heel acquisition can add a load-transfer obligation")?;
                if self.matching[side].as_ref().is_none_or(|p| {
                    p.steps < LOAD_SUBSTEPS
                        || distance(p.first_local, patch.local_point_m) > MATERIAL_MATCH_M
                }) || patch.local_point_m[2] >= HEEL_LOCAL_Z_M
                {
                    return Err("New heel obligation lacks its own material load history".into());
                }
            }
        }
        let heel_observations = std::array::from_fn(|side| HeelObservation {
            segment: self.intent.heels[side].segment,
            region_load_n: self.sample.heel_loads_n[side],
            region_loaded_substeps: self.sample.heel_loaded_substeps[side],
            goal_matching_contact_index: self.matching[side].as_ref().map(|p| p.contact_index),
            goal_matching_loaded_substeps: self.matching[side].as_ref().map_or(0, |p| p.steps),
            first_material_local_point_m: self.matching[side].as_ref().map(|p| p.first_local),
        });
        let heels_acquired = heel_observations.iter().all(|h| {
            h.region_loaded_substeps >= LOAD_SUBSTEPS
                && h.goal_matching_loaded_substeps >= LOAD_SUBSTEPS
        });
        let withdrawing = match self.intent.after_heels {
            AfterHeels::ContinueBilateral => None,
            AfterHeels::RecenterForWithdrawal(group) => Some(group),
        };
        let gate = withdrawing
            .map(|g| support_state::supported_transfer(observation, &g.segments()))
            .transpose()?;
        let mut stage = if !heels_acquired {
            Stage::AcquireBilateralHeels
        } else if withdrawing.is_some() {
            Stage::RecenterForWithdrawal
        } else {
            Stage::RecenterBilateral
        };
        if withdrawing.is_none() && self.continuation_event_substep.is_some() {
            stage = Stage::BilateralContinuationReady;
        }
        if heels_acquired && gate.as_ref().is_some_and(|g| g.allowed) {
            stage = Stage::WithdrawalGateReady;
        }
        // Prioritize bilateral heel acquisition. A later explicit withdrawal
        // changes the desired support target, never the measured obligations.
        let excluded = if heels_acquired {
            withdrawing.map(ReleaseGroup::segments)
        } else {
            None
        };
        let target_support_segments: Vec<_> = observation
            .segments
            .iter()
            .filter(|s| {
                matches!(s.role, SegmentRole::Hindfoot | SegmentRole::Forefoot)
                    && s.loaded_substeps >= LOAD_SUBSTEPS
                    && s.persistent_load_n >= MIN_LOAD_N
                    && excluded.is_none_or(|group| !group.contains(&s.segment))
            })
            .map(|s| s.segment)
            .collect();
        let points: Vec<_> = target_support_segments
            .iter()
            .flat_map(|&segment| {
                observation.segments[segment]
                    .contacts
                    .iter()
                    .filter(|c| current(c))
                    .map(|c| [c.world_point_m[0], c.world_point_m[2]])
            })
            .collect();
        let (target, inside) = support_center(&points)?;
        let capture = observation.capture_point_m();
        let com_velocity_mps = bounded(
            [
                POSITION_GAIN * (target[0] - capture[0]),
                0.0,
                POSITION_GAIN * (target[1] - capture[2]),
            ],
            SPEED_LIMIT_MPS,
        )?;
        let mut obstacles = Vec::new();
        if !inside {
            obstacles.push(Obstacle::InteriorTargetUnavailable);
        }
        for h in &heel_observations {
            if h.region_loaded_substeps < LOAD_SUBSTEPS
                || h.goal_matching_loaded_substeps < LOAD_SUBSTEPS
            {
                obstacles.push(Obstacle::HeelNotPersistentlyObserved { segment: h.segment });
            }
        }
        for patch in &retained.active {
            if !patch.current_contact_present
                || !observation.segments[patch.segment]
                    .contacts
                    .iter()
                    .any(|c| {
                        current(c)
                            && distance(c.local_point_m, patch.local_point_m) <= MATERIAL_MATCH_M
                    })
            {
                obstacles.push(Obstacle::RetainedMaterialContactMissing { patch_id: patch.id });
            }
        }
        for segment in target_support_segments
            .iter()
            .filter(|_| stage == Stage::WithdrawalGateReady)
        {
            if !retained.active.iter().any(|p| p.segment == *segment) {
                obstacles.push(Obstacle::UntrackedSupport { segment: *segment });
            }
        }
        for segment in &self.sample.nonfoot_persistent_segments {
            obstacles.push(Obstacle::NonfootDependency { segment: *segment });
        }
        let mut heel_intents = Vec::new();
        if stage != Stage::BilateralContinuationReady {
            for side in 0..2 {
                if heel_observations[side].goal_matching_loaded_substeps >= LOAD_SUBSTEPS {
                    continue;
                }
                let goal = &self.intent.heels[side];
                let body = &observation.bodies[goal.segment];
                let offset = rotate(body.rotation_xyzw, goal.local_point_m);
                let point = std::array::from_fn(|i| body.position_m[i] + offset[i]);
                let height = point[1] - observation.support_height_m;
                let forward = rotate(body.rotation_xyzw, [0., 0., 1.]);
                let pitch_axis = rotate(body.rotation_xyzw, [1., 0., 0.]);
                // Existing 2/s orientation feedback, restricted to the observed
                // pitch axis. No yaw/roll correction or new gain tuning.
                let pitch = forward[1].atan2(forward[0].hypot(forward[2]));
                let omega = pitch_axis.map(|v| v * ROTATION_GAIN * pitch);
                let heel_pitch_velocity = cross(omega, offset);
                if height > 0.0 && heel_pitch_velocity[1] >= 0.0 {
                    obstacles.push(Obstacle::HeelPitchCannotLowerPoint {
                        segment: goal.segment,
                    });
                }
                if !finite(point) || !finite(omega) || !height.is_finite() {
                    return Err("Nonfinite measured heel intent".into());
                }
                heel_intents.push(HeelTaskIntent {
                    provenance: goal.clone(),
                    observed_point_world_m: point,
                    height_above_observed_support_m: height,
                    // A target point below the measured plane is not lifted or
                    // declared contacting by this purely descending intent.
                    world_velocity_mps: [
                        0.,
                        (-POSITION_GAIN * height.max(0.)).max(-SPEED_LIMIT_MPS),
                        0.,
                    ],
                    world_angular_velocity_radps: omega,
                });
            }
        }
        let remaining_load_deficit_n = gate.as_ref().map(|g| {
            (support_state::MIN_SUPPORT_FRACTION * observation.weight_n
                - g.remaining_current_load_n)
                .max(0.)
        });
        let released_load_excess_n = gate.as_ref().map(|g| {
            (g.released_load_n - support_state::MAX_RELEASE_FRACTION * observation.weight_n).max(0.)
        });
        Ok(TransferProposal {
            native_substep: observation.native_substep,
            stage,
            continue_bilateral: stage == Stage::BilateralContinuationReady,
            continuation_event_substep: self.continuation_event_substep,
            first_stable_substep: self.first_stable_substep,
            required_validation: RequiredValidation::PreviewRequired,
            material_obligations: retained.active.clone(),
            material_intents: retained
                .active
                .iter()
                .map(|p| MaterialIntent {
                    patch_id: p.id,
                    requested_mode: if [19, 24].contains(&p.segment) {
                        ContactMode::Pivot
                    } else {
                        ContactMode::Hold
                    },
                })
                .collect(),
            heel_observations,
            heel_intents,
            measured_local_diagnostic: self.sample.clone(),
            target_support_segments,
            target_support_points_xz_m: points,
            com_capture_target_xz_m: target,
            target_inside_original_margin: inside,
            com_velocity_mps,
            measured_capture_xz_m: [capture[0], capture[2]],
            original_transfer_gate: gate,
            remaining_load_deficit_n,
            released_load_excess_n,
            obstacles,
        })
    }

    fn update_matching(&mut self) {
        for side in 0..2 {
            let goal = &self.intent.heels[side];
            let contacts = &self.latest.segments[goal.segment].contacts;
            let select = |first: Option<[f64; 3]>| {
                contacts
                    .iter()
                    .enumerate()
                    .filter(|(_, c)| {
                        current(c)
                            && c.current_upward_load_n >= MIN_LOAD_N
                            && c.local_point_m[2] < HEEL_LOCAL_Z_M
                            && distance(c.local_point_m, goal.local_point_m) <= MATERIAL_MATCH_M
                            && first
                                .is_none_or(|p| distance(c.local_point_m, p) <= MATERIAL_MATCH_M)
                    })
                    .max_by(|(i, a), (j, b)| {
                        a.current_upward_load_n
                            .total_cmp(&b.current_upward_load_n)
                            .then_with(|| j.cmp(i))
                    })
                    .map(|(index, _)| index)
            };
            let continuing = self.matching[side]
                .as_ref()
                .and_then(|p| select(Some(p.first_local)));
            if let Some(index) = continuing {
                let memory = self.matching[side].as_mut().unwrap();
                memory.contact_index = index;
                memory.steps = memory.steps.saturating_add(1);
            } else {
                self.matching[side] = select(None).map(|index| PointPersistence {
                    first_local: contacts[index].local_point_m,
                    contact_index: index,
                    steps: 1,
                });
            }
        }
    }
}

fn validate(observation: &SupportObservation, intent: &TransferIntent) -> Result<(), String> {
    observation.validate()?;
    for (segment, id, role) in [
        (18, "leftFoot", SegmentRole::Hindfoot),
        (19, "leftForefoot", SegmentRole::Forefoot),
        (23, "rightFoot", SegmentRole::Hindfoot),
        (24, "rightForefoot", SegmentRole::Forefoot),
    ] {
        if observation.segments[segment].id != id || observation.segments[segment].role != role {
            return Err("Load transfer requires the canonical foot identities".into());
        }
    }
    for (side, segment) in [18, 23].into_iter().enumerate() {
        let goal = &intent.heels[side];
        if goal.segment != segment
            || !finite(goal.local_point_m)
            || goal.local_point_m[2] >= HEEL_LOCAL_Z_M
            || goal.source_path.is_empty()
            || goal.source_sha256.len() != 64
            || !goal.source_sha256.bytes().all(|b| b.is_ascii_hexdigit())
            || observation.segments[segment].role != SegmentRole::Hindfoot
        {
            return Err("Heel intent requires canonical material-point provenance".into());
        }
    }
    Ok(())
}
fn finite(p: [f64; 3]) -> bool {
    p.iter().all(|v| v.is_finite())
}
fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2])
}
fn current(c: &CurrentContact) -> bool {
    c.upward_normal >= MIN_NORMAL_Y && c.current_upward_load_n > 0.
}
fn bounded(v: [f64; 3], limit: f64) -> Result<[f64; 3], String> {
    let length = v[0].hypot(v[1]).hypot(v[2]);
    if !finite(v) || !length.is_finite() {
        return Err("Nonfinite capture guidance".into());
    }
    Ok(if length > limit {
        v.map(|x| x * (limit / length))
    } else {
        v
    })
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn rotate(q: [f64; 4], p: [f64; 3]) -> [f64; 3] {
    let xyz = [q[0], q[1], q[2]];
    let c = cross(xyz, p);
    let cc = cross(xyz, c);
    std::array::from_fn(|i| p[i] + 2. * (q[3] * c[i] + cc[i]))
}

/// Convex hull from measured world points only. The center is an intention;
/// the original 3 mm inset is reported separately and never relaxed.
fn support_center(points: &[[f64; 2]]) -> Result<([f64; 2], bool), String> {
    if points.is_empty() || points.iter().flatten().any(|v| !v.is_finite()) {
        return Err("No finite current support points for a load-transfer target".into());
    }
    let mut points = points.to_vec();
    points.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    points.dedup();
    let cross2 = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    let mut hull = Vec::new();
    for point in &points {
        while hull.len() >= 2 && cross2(hull[hull.len() - 2], hull[hull.len() - 1], *point) <= 0. {
            hull.pop();
        }
        hull.push(*point);
    }
    let lower = hull.len();
    for point in points.iter().rev().skip(1) {
        while hull.len() > lower && cross2(hull[hull.len() - 2], hull[hull.len() - 1], *point) <= 0.
        {
            hull.pop();
        }
        hull.push(*point);
    }
    if hull.len() > 1 {
        hull.pop();
    }
    let target = [
        hull.iter().map(|p| p[0] / hull.len() as f64).sum(),
        hull.iter().map(|p| p[1] / hull.len() as f64).sum(),
    ];
    let inside = hull.len() >= 3
        && hull.iter().zip(hull.iter().cycle().skip(1)).all(|(a, b)| {
            let length = (b[0] - a[0]).hypot(b[1] - a[1]);
            let area = cross2(*a, *b, target);
            length.is_finite()
                && length > 0.
                && area.is_finite()
                && area >= support_state::SUPPORT_MARGIN_M * length
        });
    if target.iter().any(|v| !v.is_finite()) {
        return Err("Nonfinite measured hull target".into());
    }
    Ok((target, inside))
}

/// Pure fixture for adapter tests. The history excludes the returned initial
/// observation and contains 142 consecutive observations, so the local
/// milestone is acquired at start_substep + 142. No world/FK is constructed.
#[cfg(test)]
pub(crate) fn stable_fixture_history(
    start_substep: u64,
) -> (
    SupportObservation,
    MaterialTracker,
    TransferIntent,
    Vec<SupportObservation>,
) {
    tests::stable_fixture_history(start_substep)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{Value, json};

    fn archives() -> Value {
        serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"),
            "/../../../docs/evidence/coordination-support-reformulation-2026-10-13/archived-step16-fixtures.json"))).unwrap()
    }
    fn intent(next: AfterHeels) -> TransferIntent {
        TransferIntent::from_archived_heel_reference(next, 480)
    }
    fn tracker(state: &SupportObservation) -> MaterialTracker {
        super::super::support_schedule::SupportSchedule::bootstrap_observed(state, &[19, 24])
            .unwrap()
            .tracker()
            .clone()
    }
    fn synthetic() -> SupportObservation {
        let fixture = archives();
        let mut s: SupportObservation =
            serde_json::from_value(fixture["cases"][0]["support"].clone()).unwrap();
        s.native_substep = 0;
        s.time_s = 0.;
        s.weight_n = 100.;
        s.center_of_mass_m = [0., 0.7, 0.];
        s.center_of_mass_velocity_mps = [0.; 3];
        s.support_height_m = 0.;
        s.ccd_impulses.clear();
        for b in &mut s.bodies {
            b.position_m = [0.; 3];
            b.rotation_xyzw = [0., 0., 0., 1.];
            b.linear_velocity_mps = [0.; 3];
            b.angular_velocity_radps = [0.; 3];
        }
        for segment in &mut s.segments {
            segment.contacts.clear();
            segment.loaded_substeps = 0;
            segment.persistent_load_n = 0.;
        }
        for (segment, point) in [
            (18, [-0.1, 0., -0.1]),
            (19, [-0.1, 0., 0.1]),
            (23, [0.1, 0., -0.1]),
            (24, [0.1, 0., 0.1]),
        ] {
            let local = if [18, 23].contains(&segment) {
                intent(AfterHeels::ContinueBilateral).heels[0].local_point_m
            } else {
                point
            };
            s.bodies[segment].position_m = std::array::from_fn(|i| point[i] - local[i]);
            let seg = &mut s.segments[segment];
            seg.loaded_substeps = 100;
            seg.persistent_load_n = 25.;
            seg.contacts.push(CurrentContact {
                local_point_m: local,
                world_point_m: point,
                upward_normal: 1.,
                current_upward_load_n: 25.,
            });
        }
        s
    }
    pub(super) fn stable_fixture_history(
        start_substep: u64,
    ) -> (
        SupportObservation,
        MaterialTracker,
        TransferIntent,
        Vec<SupportObservation>,
    ) {
        let mut initial = synthetic();
        initial.native_substep = start_substep;
        initial.time_s = start_substep as f64 / NATIVE_HZ as f64;
        let patches = tracker(&initial);
        let transfer_intent = TransferIntent::from_archived_heel_reference(
            AfterHeels::ContinueBilateral,
            start_substep
                .checked_add(480)
                .expect("Synthetic fixture clock overflow"),
        );
        let history = (1..=142)
            .map(|offset| {
                let mut observation = initial.clone();
                observation.native_substep = start_substep
                    .checked_add(offset)
                    .expect("Synthetic fixture clock overflow");
                observation.time_s = observation.native_substep as f64 / NATIVE_HZ as f64;
                observation
            })
            .collect();
        (initial, patches, transfer_intent, history)
    }
    fn observe_next(
        policy: &mut Policy,
        s: &mut SupportObservation,
        patches: &mut MaterialTracker,
    ) {
        s.native_substep += 1;
        s.time_s = s.native_substep as f64 / NATIVE_HZ as f64;
        policy.observe(s).unwrap();
        for patch in &mut patches.active {
            patch.observe(s, MATERIAL_MATCH_M).unwrap();
        }
    }
    fn goals() -> tasks::TaskGoals {
        tasks::TaskGoals {
            nominal: crate::recovery::trajectory_program::Command {
                positions: [[0.0; 3]; 25],
                velocities: [[0.0; 3]; 25],
            },
            com_velocity_mps: [0., 0.1, 0.],
            com_weight: 0.5,
            point_tasks: vec![],
            orientation_tasks: vec![],
            posture_weight: 0.25,
            regularization: 0.05,
            retained_velocity_tolerance_mps: 0.005,
        }
    }

    #[test]
    fn pure_archived_b16_generates_intents_without_withdrawal_or_native_admission() {
        let fixture = archives();
        let mut results = Vec::new();
        for case in fixture["cases"].as_array().unwrap() {
            let s: SupportObservation = serde_json::from_value(case["support"].clone()).unwrap();
            let patches: MaterialTracker = serde_json::from_value(case["patches"].clone()).unwrap();
            let before = json!(patches);
            let policy = Policy::new(
                &s,
                &patches,
                intent(AfterHeels::RecenterForWithdrawal(ReleaseGroup::LeftFoot)),
            )
            .unwrap();
            let proposal = policy.propose(&patches).unwrap();
            assert_eq!(proposal.stage, Stage::AcquireBilateralHeels);
            assert_eq!(
                proposal.required_validation,
                RequiredValidation::PreviewRequired
            );
            assert!(!proposal.continue_bilateral);
            assert!(proposal.continuation_event_substep.is_none());
            assert!(!proposal.target_inside_original_margin);
            assert_eq!(proposal.target_support_segments, [19, 24]);
            assert!(!proposal.original_transfer_gate.as_ref().unwrap().allowed);
            assert!(proposal.remaining_load_deficit_n.unwrap() > 0.);
            assert!(proposal.released_load_excess_n.unwrap() > 0.);
            assert_eq!(json!(proposal.material_obligations), before["active"]);
            assert_eq!(json!(patches), before);
            assert_eq!(proposal.heel_intents.len(), 2);
            for h in &proposal.heel_intents {
                assert!(h.world_velocity_mps[1] < 0.);
                assert!(h.world_angular_velocity_radps[0] < 0.);
            }
            assert!(distance(proposal.com_velocity_mps, [0.; 3]) <= SPEED_LIMIT_MPS + 1e-15);
            assert!(
                proposal
                    .heel_observations
                    .iter()
                    .all(|h| h.region_load_n == 0. && h.goal_matching_loaded_substeps == 0)
            );
            let mut tasks = goals();
            proposal.apply_to_goals(&mut tasks).unwrap();
            assert_eq!(tasks.nominal.positions, [[0.; 3]; 25]);
            assert_eq!(tasks.nominal.velocities, [[0.; 3]; 25]);
            assert_eq!(tasks.retained_velocity_tolerance_mps, 0.005);
            assert_eq!(tasks.posture_weight, 0.25);
            assert_eq!(tasks.regularization, 0.05);
            assert_eq!(tasks.point_tasks.len(), 2);
            assert_eq!(tasks.orientation_tasks.len(), 2);
            results.push(json!({"feedbackScale":case["feedbackScale"],"proposal":proposal}));
        }
        if let Some(path) = std::env::var_os("LH_LOAD_TRANSFER_STATIC_EXPORT") {
            let file = std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(path)
                .unwrap();
            serde_json::to_writer_pretty(file,&json!({
                "schema":"load-transfer-static-proposals-v1","scope":"Archived observations, no native admission",
                "nativeSteps":0,"modelConstructions":0,"forwardKinematics":0,"proposals":results,
            })).unwrap();
        }
    }

    #[test]
    fn pure_archived_step36_keeps_front_hindfoot_contacts_uncommitted() {
        let x:Value=serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"),
            "/../../../docs/evidence/coordination-transfer-candidate-2026-10-13/observed-entry-step36.json"))).unwrap();
        let s: SupportObservation =
            serde_json::from_value(x["archivedNativeRow"]["support"].clone()).unwrap();
        let patches = tracker(&s);
        let policy = Policy::new(&s, &patches, intent(AfterHeels::ContinueBilateral)).unwrap();
        let proposal = policy.propose(&patches).unwrap();
        assert_eq!(proposal.stage, Stage::AcquireBilateralHeels);
        assert!(proposal.original_transfer_gate.is_none());
        assert_eq!(
            proposal
                .material_obligations
                .iter()
                .map(|p| p.segment)
                .collect::<Vec<_>>(),
            [19, 24]
        );
        assert!(
            proposal.target_support_segments.contains(&18)
                && proposal.target_support_segments.contains(&23)
        );
        assert!(
            proposal
                .heel_observations
                .iter()
                .all(|h| h.region_load_n == 0.)
        );
        assert_eq!(proposal.heel_intents.len(), 2);
    }

    #[test]
    fn pure_heel_reference_matches_archived_material_provenance() {
        let archive: Value = serde_json::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../docs/evidence/coordination-prototype-2026-10-12/continuation-tasks.json"
        )))
        .unwrap();
        for h in intent(AfterHeels::ContinueBilateral).heels {
            let p = archive["materialTargets"]
                .as_array()
                .unwrap()
                .iter()
                .find(|p| p["segmentIndex"] == h.segment)
                .unwrap();
            assert_eq!(p["localPointM"], json!(h.local_point_m));
            assert_eq!(h.source_sha256, HEEL_REFERENCE_SHA256);
        }
    }

    #[test]
    fn pure_local_milestone_is_latched_and_does_not_block_subsequent_rising_motion() {
        let mut s = synthetic();
        let mut patches = tracker(&s);
        let mut policy = Policy::new(&s, &patches, intent(AfterHeels::ContinueBilateral)).unwrap();
        for _ in 1..143 {
            assert!(policy.continuation_event().is_none());
            observe_next(&mut policy, &mut s, &mut patches);
        }
        assert_eq!(policy.continuation_event(), Some(142));
        assert_eq!(policy.first_stable_substep(), Some(11));
        let proposal = policy.propose(&patches).unwrap();
        assert!(proposal.continue_bilateral);
        s.bodies[0].linear_velocity_mps[1] = 0.3;
        observe_next(&mut policy, &mut s, &mut patches);
        let proposal = policy.propose(&patches).unwrap();
        assert!(!proposal.measured_local_diagnostic.all_conditions_now);
        assert!(proposal.continue_bilateral);
        assert_eq!(proposal.continuation_event_substep, Some(142));
        let mut task = goals();
        proposal.apply_to_goals(&mut task).unwrap();
        assert_eq!(task.com_velocity_mps, [0., 0.1, 0.]);
        assert_eq!(task.com_weight, 0.5);
    }

    #[test]
    fn pure_segment_history_cannot_replace_new_heel_material_persistence() {
        let mut s = synthetic();
        let mut patches = tracker(&s);
        let mut policy = Policy::new(&s, &patches, intent(AfterHeels::ContinueBilateral)).unwrap();
        for _ in 1..11 {
            observe_next(&mut policy, &mut s, &mut patches);
        }
        assert!(
            policy
                .propose(&patches)
                .unwrap()
                .heel_observations
                .iter()
                .all(|h| h.goal_matching_loaded_substeps == 11)
        );
        for segment in [18, 23] {
            s.segments[segment].contacts[0].local_point_m[0] += 0.48;
        }
        observe_next(&mut policy, &mut s, &mut patches);
        let proposal = policy.propose(&patches).unwrap();
        assert!(
            proposal
                .heel_observations
                .iter()
                .all(|h| h.region_loaded_substeps == 12 && h.goal_matching_loaded_substeps == 0)
        );
        assert_eq!(proposal.stage, Stage::AcquireBilateralHeels);
        assert!(policy.continuation_event().is_none());
    }

    #[test]
    fn pure_observed_heel_contact_does_not_authorize_unrequested_whole_foot_release() {
        let mut s = synthetic();
        let mut patches = tracker(&s);
        let mut policy = Policy::new(&s, &patches, intent(AfterHeels::ContinueBilateral)).unwrap();
        for _ in 1..12 {
            observe_next(&mut policy, &mut s, &mut patches);
        }
        let proposal = policy.propose(&patches).unwrap();
        assert_eq!(proposal.stage, Stage::RecenterBilateral);
        assert!(proposal.original_transfer_gate.is_none());
        assert!(
            proposal
                .material_obligations
                .iter()
                .all(|p| p.mode == ContactMode::Hold)
        );
        assert_eq!(patches.active.len(), 2);
        assert!(patches.archived.is_empty());
    }

    #[test]
    fn pure_clock_failures_are_atomic_and_missing_material_blocks_goal_application() {
        let mut s = synthetic();
        let mut patches = tracker(&s);
        let mut policy = Policy::new(&s, &patches, intent(AfterHeels::ContinueBilateral)).unwrap();
        let before = json!(policy);
        assert!(policy.observe(&s).is_err());
        assert_eq!(json!(policy), before);
        let mut bad = s.clone();
        bad.native_substep += 2;
        bad.time_s += 2. / 240.;
        assert!(policy.observe(&bad).is_err());
        assert_eq!(json!(policy), before);
        s.segments[19].contacts[0].local_point_m[0] += 0.48;
        observe_next(&mut policy, &mut s, &mut patches);
        let proposal = policy.propose(&patches).unwrap();
        assert!(proposal.apply_to_goals(&mut goals()).is_err());
    }

    #[test]
    fn pure_original_anchors_survive_and_new_heel_obligations_need_own_history() {
        let mut s = synthetic();
        let mut patches = tracker(&s);
        let mut policy = Policy::new(&s, &patches, intent(AfterHeels::ContinueBilateral)).unwrap();
        let mut changed = patches.clone();
        changed.active[0].world_anchor_m[0] += 0.48;
        assert!(policy.propose(&changed).is_err());
        changed = patches.clone();
        changed.active.remove(0);
        assert!(policy.propose(&changed).is_err());
        let mut early = patches.clone();
        early.acquire(&s, 18, 0, ContactMode::Hold).unwrap();
        assert!(policy.propose(&early).is_err());
        for _ in 1..12 {
            observe_next(&mut policy, &mut s, &mut patches);
        }
        patches.acquire(&s, 18, 0, ContactMode::Hold).unwrap();
        assert!(policy.propose(&patches).is_ok());
    }

    #[test]
    fn pure_degenerate_support_is_guidance_only_and_ccd_cannot_fill_it() {
        let (point, valid) = support_center(&[[0., 0.], [1., 0.]]).unwrap();
        assert_eq!(point, [0.5, 0.]);
        assert!(!valid);
        let (_, valid) =
            support_center(&[[-0.1, -0.1], [-0.1, 0.1], [0.1, -0.1], [0.1, 0.1]]).unwrap();
        assert!(valid);
        let s = synthetic();
        let patches = tracker(&s);
        let mut policy = Policy::new(&s, &patches, intent(AfterHeels::ContinueBilateral)).unwrap();
        let mut no_contacts = s.clone();
        for seg in &mut no_contacts.segments {
            seg.contacts.clear();
        }
        no_contacts.native_substep += 1;
        no_contacts.time_s += 1. / 240.;
        no_contacts.ccd_impulses.push(support_state::CcdImpulse {
            native_substep: 1,
            ccd_interval: 0,
            segment: 19,
            interval_s: 1. / 240.,
            normal_ns: [0., 100., 0.],
            tangent_ns: [0.; 3],
        });
        policy.observe(&no_contacts).unwrap();
        let mut updated = patches.clone();
        for p in &mut updated.active {
            p.observe(&no_contacts, MATERIAL_MATCH_M).unwrap();
        }
        assert!(policy.propose(&updated).is_err());
    }
}
