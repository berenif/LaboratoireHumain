//! Pure, measured-support bookkeeping for proposed foot transitions.
//!
//! This module never installs a command, constructs a world, or certifies a
//! native trajectory. Even a successful current-state transfer gate leaves the
//! proposal `PreviewRequired`. Requested goals never become observed contacts.
use serde::Serialize;

use super::support_state::{
    ContactMode, CurrentContact, LOAD_SUBSTEPS, MIN_LOAD_N, MIN_NORMAL_Y, MaterialPatch,
    MaterialTracker, NATIVE_HZ, SegmentRole, SupportObservation, TransferEvidence,
    supported_transfer,
};

/// Existing owner correspondence tolerance, not a new slip/arrival threshold.
const MATERIAL_MATCH_M: f64 = 0.012;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum ReleaseGroup {
    LeftFoot,
    RightFoot,
}

impl ReleaseGroup {
    /// A whole-foot withdrawal always excludes hindfoot AND forefoot.
    pub fn segments(self) -> [usize; 2] {
        match self {
            Self::LeftFoot => [18, 19],
            Self::RightFoot => [23, 24],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum Phase {
    Retain,
    Unload,
    Released,
    Reposition,
    Acquire,
}

/// There is deliberately no successful/native-admission variant here.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum RequiredValidation {
    PreviewRequired,
}

#[derive(Clone, Debug, Serialize)]
pub struct RepositionGoal {
    pub segment: usize,
    pub local_point_m: [f64; 3],
    pub world_goal_m: [f64; 3],
    pub deadline_substep: u64,
}

#[derive(Clone, Debug, Serialize)]
pub struct ReleaseDecision {
    pub evidence: TransferEvidence,
    /// The planned obligation was removed only after the measured gate passed.
    pub obligation_released: bool,
    pub refusal: Option<String>,
    pub required_validation: RequiredValidation,
}

#[derive(Clone, Debug, Serialize)]
pub struct AcquisitionDecision {
    pub patch_id: u64,
    pub first_loaded_substep: u64,
    pub loaded_substeps: u64,
    /// Desired-target error only; no geometric arrival/contact certification.
    pub world_goal_residual_m: [f64; 3],
    pub required_validation: RequiredValidation,
}

#[derive(Clone, Debug, Serialize)]
struct AcquisitionCandidate {
    /// Fixed through this run. Incremental matching cannot hide rolling drift.
    first_local_point_m: [f64; 3],
    first_loaded_substep: u64,
    loaded_substeps: u64,
    contact_index: usize,
}

#[derive(Clone, Debug, Serialize)]
pub struct SupportSchedule {
    phase: Phase,
    group: Option<ReleaseGroup>,
    tracker: MaterialTracker,
    latest: SupportObservation,
    goal: Option<RepositionGoal>,
    candidate: Option<AcquisitionCandidate>,
    released_at_substep: Option<u64>,
    cleared_substep: Option<u64>,
}

impl SupportSchedule {
    /// Imports an observed owner history without refreshing any anchor/clock.
    pub fn from_retained(
        observation: &SupportObservation,
        tracker: &MaterialTracker,
    ) -> Result<Self, String> {
        validate_observation(observation)?;
        if tracker.active.is_empty() {
            return Err("A retained schedule requires observed material support".into());
        }
        validate_ids(tracker)?;
        for (index, patch) in tracker.active.iter().enumerate() {
            if !is_foot(patch.segment)
                || !matches!(patch.mode, ContactMode::Hold | ContactMode::Pivot)
                || tracker.active[..index]
                    .iter()
                    .any(|p| p.segment == patch.segment)
                || patch.acquired_substep > patch.last_observed_substep
                || patch.last_observed_substep != observation.native_substep
                || !patch.has_current_support()
                || !patch_finite(patch)
                || !persistent_segment(observation, patch.segment)
                || !has_matching_material_support(observation, patch)?
            {
                return Err(
                    "Invalid/stale retained foot patch; history cannot be reanchored".into(),
                );
            }
        }
        Ok(Self {
            phase: Phase::Retain,
            group: None,
            tracker: tracker.clone(),
            latest: observation.clone(),
            goal: None,
            candidate: None,
            released_at_substep: None,
            cleared_substep: None,
        })
    }

    /// Bootstrap uses current loaded points on historically loaded segments.
    /// It does NOT claim historical persistence of these particular points.
    /// The independent heel diagnostic must establish its own 12 observations.
    pub fn bootstrap_observed(
        observation: &SupportObservation,
        segments: &[usize],
    ) -> Result<Self, String> {
        validate_observation(observation)?;
        if segments.is_empty()
            || segments
                .iter()
                .enumerate()
                .any(|(i, s)| !is_foot(*s) || segments[..i].contains(s))
        {
            return Err("Bootstrap requires distinct canonical foot segments".into());
        }
        let mut tracker = MaterialTracker::default();
        for &segment in segments {
            if !persistent_segment(observation, segment) {
                return Err("Bootstrap segment lacks the original persistent load".into());
            }
            let contact = select_contact(observation, segment, None, None)
                .ok_or("Bootstrap requires a currently observed contact of at least 3 N")?;
            tracker.acquire(observation, segment, contact, ContactMode::Hold)?;
        }
        Self::from_retained(observation, &tracker)
    }

    pub fn phase(&self) -> Phase {
        self.phase
    }
    pub fn tracker(&self) -> &MaterialTracker {
        &self.tracker
    }
    pub fn observation(&self) -> &SupportObservation {
        &self.latest
    }
    pub fn reposition_goal(&self) -> Option<&RepositionGoal> {
        self.goal.as_ref()
    }
    pub fn obligations(&self) -> Vec<usize> {
        self.tracker.active.iter().map(|p| p.segment).collect()
    }

    /// New actual observations only. Errors leave the complete schedule intact.
    pub fn observe(&mut self, observation: &SupportObservation) -> Result<(), String> {
        validate_observation(observation)?;
        if self.latest.native_substep.checked_add(1) != Some(observation.native_substep)
            || (observation.time_s - self.latest.time_s - 1.0 / NATIVE_HZ as f64).abs() > 1e-7
        {
            return Err("Support schedule requires consecutive native observations".into());
        }
        let mut next = self.clone();
        for patch in &mut next.tracker.active {
            patch.observe(observation, MATERIAL_MATCH_M)?;
        }
        next.latest = observation.clone();
        if let (Some(group), Some(released_at)) = (next.group, next.released_at_substep) {
            if observation.native_substep > released_at
                && group.segments().iter().all(|&segment| {
                    observation.segments[segment]
                        .contacts
                        .iter()
                        .all(|contact| {
                            contact.upward_normal < MIN_NORMAL_Y
                                || contact.current_upward_load_n <= 0.0
                        })
                })
            {
                next.cleared_substep
                    .get_or_insert(observation.native_substep);
            }
        }
        if next.phase == Phase::Acquire {
            next.update_candidate()?;
        }
        *self = next;
        Ok(())
    }

    /// Intent only. Pivot retains the original material point and world anchor.
    /// Rolling to another point needs a separately proved transition; there is
    /// deliberately no replacement/reanchor accessor on this schedule.
    pub fn request_pivot(&mut self, group: ReleaseGroup) -> Result<RequiredValidation, String> {
        if self.phase != Phase::Retain || !self.has_group(group) {
            return Err("Pivot requires a retained foot obligation".into());
        }
        for patch in &mut self.tracker.active {
            if group.segments().contains(&patch.segment) {
                patch.mode = ContactMode::Pivot;
            }
        }
        Ok(RequiredValidation::PreviewRequired)
    }

    pub fn request_unload(&mut self, group: ReleaseGroup) -> Result<RequiredValidation, String> {
        if self.phase != Phase::Retain || !self.has_group(group) {
            return Err("Unload requires a retained foot obligation".into());
        }
        for patch in &mut self.tracker.active {
            if group.segments().contains(&patch.segment) {
                patch.mode = ContactMode::Unload;
            }
        }
        self.group = Some(group);
        self.phase = Phase::Unload;
        self.goal = None;
        self.candidate = None;
        self.released_at_substep = None;
        self.cleared_substep = None;
        Ok(RequiredValidation::PreviewRequired)
    }

    /// Necessary measured gate from the original observer, plus retained
    /// material provenance. This does not prove the subsequent native motion.
    pub fn try_release(&mut self) -> Result<ReleaseDecision, String> {
        if self.phase != Phase::Unload {
            return Err("Release requires an explicit unload phase".into());
        }
        let group = self.group.ok_or("Missing release group")?;
        let evidence = supported_transfer(&self.latest, &group.segments())?;
        let mut retained_material_present = true;
        for patch in self.tracker.active.iter()
            .filter(|p| !group.segments().contains(&p.segment))
        {
            retained_material_present &= patch.has_current_support()
                && patch.last_observed_substep == self.latest.native_substep
                && has_matching_material_support(&self.latest, patch)?;
        }
        // This first implementation schedules feet only. It cannot silently
        // take credit for an untracked hand/shin/new segment in the load/hull.
        let all_support_tracked = evidence
            .retained_segments
            .iter()
            .all(|s| self.tracker.active.iter().any(|p| p.segment == *s));
        let refusal = if !evidence.allowed {
            Some("Original remaining-support gate refused the whole-foot withdrawal".into())
        } else if !retained_material_present || !all_support_tracked {
            Some("Remaining support lacks a retained current material obligation".into())
        } else {
            None
        };
        if refusal.is_none() {
            let (released, retained): (Vec<_>, Vec<_>) = self
                .tracker
                .active
                .drain(..)
                .partition(|p| group.segments().contains(&p.segment));
            self.tracker.archived.extend(released);
            self.tracker.active = retained;
            self.phase = Phase::Released;
            self.released_at_substep = Some(self.latest.native_substep);
        }
        Ok(ReleaseDecision {
            evidence,
            obligation_released: refusal.is_none(),
            refusal,
            required_validation: RequiredValidation::PreviewRequired,
        })
    }

    pub fn begin_reposition(&mut self, goal: RepositionGoal) -> Result<RequiredValidation, String> {
        if self.phase != Phase::Released {
            return Err("Reposition requires a permitted planned withdrawal".into());
        }
        if !self
            .group
            .ok_or("Missing release group")?
            .segments()
            .contains(&goal.segment)
            || !finite(goal.local_point_m)
            || !finite(goal.world_goal_m)
            || goal.deadline_substep <= self.latest.native_substep
        {
            return Err("Invalid reposition goal or expired deadline".into());
        }
        self.goal = Some(goal);
        self.phase = Phase::Reposition;
        Ok(RequiredValidation::PreviewRequired)
    }

    pub fn begin_acquire(&mut self) -> Result<RequiredValidation, String> {
        if self.phase != Phase::Reposition {
            return Err("Acquisition requires an explicit reposition goal".into());
        }
        // This route is a whole-foot withdrawal. Without observed loss of its
        // current support, creating a new identity would merely reset an old
        // anchor after a phase-label round trip. A rolling/pivot transition
        // needs its own future formulation and is not silently substituted.
        if self.cleared_substep.is_none() {
            return Err(
                "Whole-foot support has not disappeared after the planned withdrawal".into(),
            );
        }
        let goal = self.goal.as_ref().ok_or("Missing acquisition goal")?;
        if goal.deadline_substep < self.latest.native_substep {
            return Err("Acquisition deadline expired".into());
        }
        let mut next = self.clone();
        next.phase = Phase::Acquire;
        next.candidate = None;
        next.update_candidate()?;
        *self = next;
        Ok(RequiredValidation::PreviewRequired)
    }

    /// Acquire only a newly persistent, measured point corresponding to the
    /// requested local material region. Proximity to world_goal cannot do this.
    pub fn acquire_observed(&mut self) -> Result<AcquisitionDecision, String> {
        if self.phase != Phase::Acquire {
            return Err("Not in acquisition phase".into());
        }
        let goal = self.goal.as_ref().ok_or("Missing acquisition goal")?;
        let candidate = self
            .candidate
            .as_ref()
            .ok_or("No loaded material acquisition candidate")?;
        if self.latest.native_substep > goal.deadline_substep
            || candidate.loaded_substeps < LOAD_SUBSTEPS
            || !persistent_segment(&self.latest, goal.segment)
        {
            return Err(
                "New contact has not met its own 12-step load history before deadline".into(),
            );
        }
        let contact = &self.latest.segments[goal.segment].contacts[candidate.contact_index];
        let world_goal_residual_m =
            std::array::from_fn(|i| contact.world_point_m[i] - goal.world_goal_m[i]);
        if !finite(world_goal_residual_m) {
            return Err("Nonfinite reposition residual".into());
        }
        let mut tracker = self.tracker.clone();
        let patch_id = tracker.acquire(
            &self.latest,
            goal.segment,
            candidate.contact_index,
            ContactMode::Hold,
        )?;
        validate_ids(&tracker)?;
        let decision = AcquisitionDecision {
            patch_id,
            first_loaded_substep: candidate.first_loaded_substep,
            loaded_substeps: candidate.loaded_substeps,
            world_goal_residual_m,
            required_validation: RequiredValidation::PreviewRequired,
        };
        self.tracker = tracker;
        self.phase = Phase::Retain;
        self.group = None;
        self.candidate = None;
        // Keep the last explicit goal for the external command preparer/audit.
        Ok(decision)
    }

    fn has_group(&self, group: ReleaseGroup) -> bool {
        self.tracker
            .active
            .iter()
            .any(|p| group.segments().contains(&p.segment))
    }

    fn update_candidate(&mut self) -> Result<(), String> {
        let goal = self.goal.as_ref().ok_or("Missing acquisition goal")?;
        if self.latest.native_substep > goal.deadline_substep {
            self.candidate = None;
            return Ok(());
        }
        let continuing = self.candidate.as_ref().and_then(|candidate| {
            select_contact(
                &self.latest,
                goal.segment,
                Some(goal.local_point_m),
                Some(candidate.first_local_point_m),
            )
        });
        if let Some(contact_index) = continuing {
            let candidate = self.candidate.as_mut().ok_or("Missing candidate history")?;
            candidate.loaded_substeps = candidate.loaded_substeps.saturating_add(1);
            candidate.contact_index = contact_index;
        } else {
            self.candidate =
                select_contact(&self.latest, goal.segment, Some(goal.local_point_m), None).map(
                    |contact_index| AcquisitionCandidate {
                        first_local_point_m: self.latest.segments[goal.segment].contacts
                            [contact_index]
                            .local_point_m,
                        first_loaded_substep: self.latest.native_substep,
                        loaded_substeps: 1,
                        contact_index,
                    },
                );
        }
        Ok(())
    }
}

fn validate_observation(observation: &SupportObservation) -> Result<(), String> {
    observation.validate()?;
    for (segment, id, role) in [
        (18, "leftFoot", SegmentRole::Hindfoot),
        (19, "leftForefoot", SegmentRole::Forefoot),
        (23, "rightFoot", SegmentRole::Hindfoot),
        (24, "rightForefoot", SegmentRole::Forefoot),
    ] {
        let actual = &observation.segments[segment];
        if actual.id != id || actual.role != role {
            return Err("Foot transition requires the canonical hindfoot/forefoot mapping".into());
        }
    }
    Ok(())
}

fn is_foot(segment: usize) -> bool {
    [18, 19, 23, 24].contains(&segment)
}
fn finite(values: [f64; 3]) -> bool {
    values.iter().all(|v| v.is_finite())
}
fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2])
}
fn persistent_segment(observation: &SupportObservation, segment: usize) -> bool {
    let s = &observation.segments[segment];
    s.loaded_substeps >= LOAD_SUBSTEPS && s.persistent_load_n >= MIN_LOAD_N
}
fn loaded(contact: &CurrentContact) -> bool {
    contact.upward_normal >= MIN_NORMAL_Y && contact.current_upward_load_n >= MIN_LOAD_N
}
fn has_matching_load(observation: &SupportObservation, segment: usize, local: [f64; 3]) -> bool {
    select_contact(observation, segment, Some(local), None).is_some()
}
fn has_matching_material_support(
    observation: &SupportObservation,
    patch: &MaterialPatch,
) -> Result<bool, String> {
    // Keep the original 3 N direct acquisition/retention requirement. The
    // equivalent-surface certificate independently requires three 3 N witnesses.
    use super::support_state::MaterialContinuityKind;
    let evidence = patch.material_continuity(observation, MATERIAL_MATCH_M)?;
    Ok(match evidence.kind {
        MaterialContinuityKind::DirectLoadedPoint =>
            has_matching_load(observation, patch.segment, patch.local_point_m),
        MaterialContinuityKind::EquivalentLoadedSurface => true,
        MaterialContinuityKind::Refused => false,
    })
}
fn select_contact(
    observation: &SupportObservation,
    segment: usize,
    goal_local: Option<[f64; 3]>,
    fixed_local: Option<[f64; 3]>,
) -> Option<usize> {
    observation.segments[segment]
        .contacts
        .iter()
        .enumerate()
        .filter(|(_, c)| {
            loaded(c)
                && goal_local.is_none_or(|p| distance(c.local_point_m, p) <= MATERIAL_MATCH_M)
                && fixed_local.is_none_or(|p| distance(c.local_point_m, p) <= MATERIAL_MATCH_M)
        })
        .max_by(|(ai, a), (bi, b)| {
            a.current_upward_load_n
                .total_cmp(&b.current_upward_load_n)
                .then_with(|| bi.cmp(ai))
        })
        .map(|(index, _)| index)
}
fn validate_ids(tracker: &MaterialTracker) -> Result<(), String> {
    let all: Vec<_> = tracker.active.iter().chain(&tracker.archived).collect();
    if all
        .iter()
        .enumerate()
        .any(|(i, p)| all[..i].iter().any(|q| q.id == p.id))
    {
        return Err("Material patch identities are not unique".into());
    }
    Ok(())
}
fn patch_finite(patch: &MaterialPatch) -> bool {
    finite(patch.local_point_m)
        && finite(patch.world_anchor_m)
        && finite(patch.last_world_point_m)
        && finite(patch.displacement_from_anchor_m)
        && patch.tangential_drift_m.is_finite()
        && patch.tangential_drift_m >= 0.0
        && patch
            .nearest_current_local_distance_m
            .is_some_and(|v| v.is_finite() && v >= 0.0
                && (!patch.current_contact_present || v <= MATERIAL_MATCH_M))
}

#[cfg(test)]
mod tests {
    use super::super::support_state::{BODY_COUNT, BodyMotion, SegmentContactState};
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
                contacts: vec![],
            })
            .collect();
        for (segment, id, role) in [
            (18, "leftFoot", SegmentRole::Hindfoot),
            (19, "leftForefoot", SegmentRole::Forefoot),
            (23, "rightFoot", SegmentRole::Hindfoot),
            (24, "rightForefoot", SegmentRole::Forefoot),
        ] {
            segments[segment].id = id.into();
            segments[segment].role = role;
        }
        let mut state = SupportObservation {
            native_contact_boundary: None,
            native_substep: 12,
            time_s: 12.0 / 240.0,
            weight_n: 100.0,
            center_of_mass_m: [0.0, 0.7, 0.0],
            center_of_mass_velocity_mps: [0.0; 3],
            support_height_m: 0.0,
            bodies,
            segments,
            ccd_impulses: vec![],
        };
        set_load(&mut state, 19, 20.0);
        set_load(&mut state, 24, 80.0);
        state
    }
    fn set_load(state: &mut SupportObservation, segment: usize, load: f64) {
        let s = &mut state.segments[segment];
        s.persistent_load_n = load;
        s.loaded_substeps = 100;
        s.contacts = [
            [-0.1, 0.0, -0.1],
            [-0.1, 0.0, 0.1],
            [0.1, 0.0, -0.1],
            [0.1, 0.0, 0.1],
        ]
        .into_iter()
        .map(|point| CurrentContact {
            local_point_m: point,
            world_point_m: point,
            upward_normal: 1.0,
            current_upward_load_n: load / 4.0,
        })
        .collect();
    }
    fn tick(state: &mut SupportObservation) {
        state.native_substep += 1;
        state.time_s = state.native_substep as f64 / 240.0;
    }
    fn json(value: &impl Serialize) -> serde_json::Value {
        serde_json::to_value(value).unwrap()
    }
    #[test]
    fn pure_surface_cache_cannot_bypass_recomputed_direct_load_requirement() {
        let mut state = observation();
        state.segments[19].contacts.insert(0, CurrentContact {
            local_point_m: [0.0; 3],
            world_point_m: [0.0; 3],
            upward_normal: 1.0,
            current_upward_load_n: 10.0,
        });
        let mut schedule = SupportSchedule::bootstrap_observed(&state, &[19, 24]).unwrap();
        state.segments[19].contacts.remove(0);
        tick(&mut state);
        schedule.observe(&state).unwrap();
        let retained = schedule.tracker();
        assert!(!retained.active[0].current_contact_present);
        assert!(retained.active[0].has_current_support());
        assert!(has_matching_material_support(&state, &retained.active[0]).unwrap());
        // A different same-clock import must not use the old surface cache to
        // evade the schedule's original 3 N direct-retention requirement.
        state.segments[19].contacts.insert(0, CurrentContact {
            local_point_m: [0.0; 3],
            world_point_m: [0.0; 3],
            upward_normal: 1.0,
            current_upward_load_n: 0.01,
        });
        assert!(!has_matching_material_support(&state, &retained.active[0]).unwrap());
        assert!(SupportSchedule::from_retained(&state, retained).is_err());
    }
    fn acquiring() -> (SupportObservation, SupportSchedule) {
        let mut state = observation();
        let mut schedule = SupportSchedule::bootstrap_observed(&state, &[19, 24]).unwrap();
        schedule.request_unload(ReleaseGroup::LeftFoot).unwrap();
        assert!(schedule.try_release().unwrap().obligation_released);
        schedule
            .begin_reposition(RepositionGoal {
                segment: 19,
                local_point_m: [-0.1, 0.0, -0.1],
                world_goal_m: [-0.1, 0.0, -0.1],
                deadline_substep: 100,
            })
            .unwrap();
        state.segments[19].contacts.clear();
        state.segments[19].persistent_load_n = 0.0;
        state.segments[19].loaded_substeps = 0;
        tick(&mut state);
        schedule.observe(&state).unwrap();
        schedule.begin_acquire().unwrap();
        set_load(&mut state, 19, 20.0);
        tick(&mut state);
        schedule.observe(&state).unwrap();
        (state, schedule)
    }

    #[test]
    fn pure_measured_transition_keeps_history_and_requires_preview() {
        let (mut state, mut schedule) = acquiring();
        let old = schedule.tracker().archived[0].clone();
        assert_eq!(old.acquired_substep, 12);
        assert_eq!(schedule.obligations(), [24]);
        for _ in 1..LOAD_SUBSTEPS {
            assert!(schedule.acquire_observed().is_err());
            tick(&mut state);
            schedule.observe(&state).unwrap();
        }
        let acquired = schedule.acquire_observed().unwrap();
        assert_eq!(acquired.loaded_substeps, 12);
        assert_eq!(acquired.first_loaded_substep, 14);
        assert_eq!(
            acquired.required_validation,
            RequiredValidation::PreviewRequired
        );
        assert_ne!(acquired.patch_id, old.id);
        assert_eq!(json(&schedule.tracker().archived[0]), json(&old));
        assert_eq!(schedule.tracker().active[1].acquired_substep, 25);
        assert_eq!(schedule.phase(), Phase::Retain);
    }

    #[test]
    fn pure_archived_b16_rejects_both_whole_foot_withdrawals_without_mutation() {
        let fixture: serde_json::Value = serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"),
            "/../../../docs/evidence/coordination-support-reformulation-2026-10-13/archived-step16-fixtures.json"))).unwrap();
        let mut decisions = Vec::new();
        for case in fixture["cases"].as_array().unwrap() {
            let observation: SupportObservation =
                serde_json::from_value(case["support"].clone()).unwrap();
            let tracker: MaterialTracker = serde_json::from_value(case["patches"].clone()).unwrap();
            for group in [ReleaseGroup::LeftFoot, ReleaseGroup::RightFoot] {
                let mut schedule = SupportSchedule::from_retained(&observation, &tracker).unwrap();
                assert_eq!(json(schedule.tracker()), json(&tracker));
                schedule.request_unload(group).unwrap();
                let before = json(&schedule);
                let obligations_before = schedule.obligations();
                let tracker_before = json(schedule.tracker());
                let refusal = schedule.try_release().unwrap();
                assert!(!refusal.evidence.allowed);
                assert!(!refusal.obligation_released);
                assert_eq!(refusal.evidence.releasing, group.segments());
                assert_eq!(json(&schedule), before);
                assert!(
                    schedule
                        .begin_reposition(RepositionGoal {
                            segment: group.segments()[1],
                            local_point_m: [0.0; 3],
                            world_goal_m: [0.0; 3],
                            deadline_substep: 100
                        })
                        .is_err()
                );
                assert_eq!(schedule.obligations(), [19, 24]);
                decisions.push(serde_json::json!({
                    "feedbackScale": case["feedbackScale"],
                    "nativeSubstep": observation.native_substep,
                    "observationTimeS": observation.time_s,
                    "weightN": observation.weight_n,
                    "group": group,
                    "decision": refusal,
                    "obligationsBefore": obligations_before,
                    "obligationsAfter": schedule.obligations(),
                    "trackerBefore": tracker_before,
                    "trackerAfter": schedule.tracker(),
                    "completeScheduleUnchangedAfterRefusal": json(&schedule) == before,
                }));
            }
        }
        if let Some(path) = std::env::var_os("LH_SUPPORT_SCHEDULE_STATIC_EXPORT") {
            let receipt = serde_json::json!({
                "schema": "support-schedule-static-release-evidence-v1",
                "scope": "Pure replay of archived observations; no physical admission",
                "fixture": "archived-step16-fixtures.json",
                "sourcePlanningSha256": fixture["sourcePlanningSha256"],
                "sourceTraceSha256": fixture["sourceTraceSha256"],
                "nativeSubstepsExecuted": 0,
                "modelOrSimulationConstructions": 0,
                "forwardKinematicsEvaluations": 0,
                "requiredValidation": RequiredValidation::PreviewRequired,
                "decisions": decisions,
            });
            // An explicitly requested new evidence file only. Never overwrite
            // a prior receipt and never mutate the archived native evidence.
            let file = std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(path)
                .expect("Static support evidence requires a new file");
            serde_json::to_writer_pretty(file, &receipt).unwrap();
        }
    }

    #[test]
    fn pure_whole_foot_cannot_count_its_own_hindfoot_as_remaining_support() {
        let mut state = observation();
        set_load(&mut state, 18, 80.0);
        set_load(&mut state, 19, 12.0);
        set_load(&mut state, 24, 12.0);
        assert!(supported_transfer(&state, &[19]).unwrap().allowed);
        let mut schedule = SupportSchedule::bootstrap_observed(&state, &[18, 19, 24]).unwrap();
        schedule.request_unload(ReleaseGroup::LeftFoot).unwrap();
        let refusal = schedule.try_release().unwrap();
        assert!(!refusal.obligation_released);
        assert_eq!(refusal.evidence.releasing, [18, 19]);
        assert_eq!(refusal.evidence.retained_segments, [24]);
        assert_eq!(schedule.obligations(), [18, 19, 24]);
    }

    #[test]
    fn pure_pivot_preserves_48cm_drift_instead_of_reanchoring() {
        let mut state = observation();
        let mut schedule = SupportSchedule::bootstrap_observed(&state, &[19, 24]).unwrap();
        let anchor = schedule.tracker().active[0].world_anchor_m;
        let local = schedule.tracker().active[0].local_point_m;
        let id = schedule.tracker().active[0].id;
        state.bodies[19].position_m[0] = 0.48;
        for point in &mut state.segments[19].contacts {
            point.local_point_m[0] -= 0.48;
        }
        tick(&mut state);
        schedule.observe(&state).unwrap();
        assert_eq!(
            schedule.request_pivot(ReleaseGroup::LeftFoot).unwrap(),
            RequiredValidation::PreviewRequired
        );
        let patch = &schedule.tracker().active[0];
        assert_eq!(patch.world_anchor_m, anchor);
        assert_eq!(patch.local_point_m, local);
        assert_eq!(patch.id, id);
        assert_eq!(patch.acquired_substep, 12);
        assert!((patch.tangential_drift_m - 0.48).abs() < 1e-12);
        assert!(!patch.current_contact_present);
        assert_eq!(schedule.obligations(), [19, 24]);
        assert!(schedule.tracker().archived.is_empty());
    }

    #[test]
    fn pure_phase_round_trip_cannot_reanchor_a_foot_that_never_left_support() {
        let mut state = observation();
        let mut schedule = SupportSchedule::bootstrap_observed(&state, &[19, 24]).unwrap();
        schedule.request_unload(ReleaseGroup::LeftFoot).unwrap();
        assert!(schedule.try_release().unwrap().obligation_released);
        schedule
            .begin_reposition(RepositionGoal {
                segment: 19,
                local_point_m: [-0.1, 0.0, -0.1],
                world_goal_m: [-0.1, 0.0, -0.1],
                deadline_substep: 100,
            })
            .unwrap();
        for _ in 0..12 {
            tick(&mut state);
            schedule.observe(&state).unwrap();
            let before = json(&schedule);
            assert!(schedule.begin_acquire().is_err());
            assert_eq!(json(&schedule), before);
        }
        // Losing only the forefoot cannot stand in for clearing a whole foot.
        state.segments[19].contacts.clear();
        set_load(&mut state, 18, 4.0);
        tick(&mut state);
        schedule.observe(&state).unwrap();
        assert!(schedule.begin_acquire().is_err());
        state.segments[18].contacts.clear();
        tick(&mut state);
        schedule.observe(&state).unwrap();
        assert_eq!(
            schedule.begin_acquire().unwrap(),
            RequiredValidation::PreviewRequired
        );
        assert!(schedule.acquire_observed().is_err());
    }

    #[test]
    fn pure_new_contact_cannot_inherit_segment_persistence_or_goal_proximity() {
        let (mut state, mut schedule) = acquiring();
        for _ in 0..10 {
            tick(&mut state);
            schedule.observe(&state).unwrap();
        }
        assert!(schedule.acquire_observed().is_err());
        // Move the selected material point far away but leave its world point
        // exactly on the requested target. Segment history still says 100.
        for c in &mut state.segments[19].contacts {
            c.local_point_m[0] += 0.48;
        }
        for _ in 0..12 {
            tick(&mut state);
            schedule.observe(&state).unwrap();
        }
        assert!(schedule.acquire_observed().is_err());
        assert_eq!(schedule.obligations(), [24]);
        set_load(&mut state, 19, 20.0);
        for _ in 0..11 {
            tick(&mut state);
            schedule.observe(&state).unwrap();
        }
        assert!(schedule.acquire_observed().is_err());
        tick(&mut state);
        schedule.observe(&state).unwrap();
        assert_eq!(schedule.acquire_observed().unwrap().loaded_substeps, 12);
    }

    #[test]
    fn pure_clock_and_nonfinite_rejections_are_atomic() {
        let state = observation();
        let mut schedule = SupportSchedule::bootstrap_observed(&state, &[19, 24]).unwrap();
        let before = json(&schedule);
        assert!(schedule.observe(&state).is_err());
        let mut skipped = state.clone();
        tick(&mut skipped);
        tick(&mut skipped);
        assert!(schedule.observe(&skipped).is_err());
        let mut invalid = state.clone();
        tick(&mut invalid);
        invalid.bodies[24].position_m[0] = f64::NAN;
        assert!(schedule.observe(&invalid).is_err());
        assert_eq!(json(&schedule), before);
    }

    #[test]
    fn pure_missing_remaining_patch_or_untracked_hand_cannot_authorize_release() {
        let mut state = observation();
        let mut schedule = SupportSchedule::bootstrap_observed(&state, &[19, 24]).unwrap();
        for contact in &mut state.segments[24].contacts {
            contact.local_point_m[0] += 0.48;
        }
        tick(&mut state);
        schedule.observe(&state).unwrap();
        schedule.request_unload(ReleaseGroup::LeftFoot).unwrap();
        let refused = schedule.try_release().unwrap();
        assert!(refused.evidence.allowed);
        assert!(!refused.obligation_released);
        let mut state = observation();
        state.segments[1].role = SegmentRole::Hand;
        set_load(&mut state, 1, 20.0);
        let mut schedule = SupportSchedule::bootstrap_observed(&state, &[19, 24]).unwrap();
        schedule.request_unload(ReleaseGroup::LeftFoot).unwrap();
        let refused = schedule.try_release().unwrap();
        assert!(refused.evidence.allowed);
        assert!(!refused.obligation_released);
    }

    #[test]
    fn pure_reposition_goal_is_explicit_and_deadline_cannot_create_contact() {
        let (mut state, mut schedule) = acquiring();
        assert_eq!(schedule.reposition_goal().unwrap().segment, 19);
        for _ in 0..89 {
            tick(&mut state);
            schedule.observe(&state).unwrap();
        }
        assert!(schedule.acquire_observed().is_err());
        assert_eq!(schedule.phase(), Phase::Acquire);
        assert_eq!(schedule.obligations(), [24]);
        assert_eq!(schedule.tracker().archived.len(), 1);
    }
}
