//! One candidate owner from observed command history to bilateral continuation.
//!
//! Pure preparation is separate from native installation. A copied branch may
//! receive a short preview window only from the budgeted native preview driver.
//! Real execution requires exact before-step native identities from a completed
//! accepted horizon. No JSON observation can manufacture that native receipt.
use super::{
    command_proposal,
    continuation::Timing,
    coordinated_runtime as runtime,
    load_transfer::{self, Policy, TransferIntent, TransferProposal},
    observed_raccord::{Bridge, LocalReadyReceipt, ObservedCommandEntry, PreparedRaccord},
    support_state::{ContactMode, MaterialTracker, SupportObservation},
    whole_body_tasks::Problem,
};
use crate::recovery::trajectory_program::{
    Command, CompiledProgram, Coordinates, validate_command,
};
use lh_model::Model;
use serde_json::{Value, json};

const CONTROL_STEPS: u64 = 4;
const REAL_WINDOW_STEPS: usize = 24;
const MATERIAL_MATCH_M: f64 = 0.012;

#[derive(Clone)]
struct NativeRecord {
    at: u64,
    identity: std::sync::Arc<[u8]>,
    command: Option<Command>,
}

#[derive(Clone)]
struct NativeCheckpoint {
    at: u64,
    identity: std::sync::Arc<[u8]>,
}

/// Untrusted prepared payload. Only native_plan_preview may wrap this in its
/// private-constructor receipt after independently checking the native branch.
pub(super) struct NativeWindowData {
    records: Vec<NativeRecord>,
    horizon_steps: u32,
    checkpoints: Vec<NativeCheckpoint>,
    entry_identity: std::sync::Arc<[u8]>,
}

pub(super) struct NativeTechnicalData {
    records: Vec<NativeRecord>,
    entry_identity: std::sync::Arc<[u8]>,
    end_identity: std::sync::Arc<[u8]>,
    instance: usize,
    scope: Vec<u8>,
}

/// Pure clock observer; this records the original measured terminal event,
/// never the requested standing reference. The additional second is bounded
/// from that first event and cannot restart on later observations.
#[derive(Clone, Debug, serde::Serialize)]
struct TerminalClock {
    absolute_origin: u64,
    first_success_at: Option<u64>,
    last_observed_at: Option<u64>,
}
impl TerminalClock {
    fn new(absolute_origin: u64) -> Self {
        Self {
            absolute_origin,
            first_success_at: None,
            last_observed_at: None,
        }
    }
    fn observe(&mut self, at: u64, terminal: bool) -> Result<(), String> {
        let deadline = self
            .absolute_origin
            .checked_add(6000)
            .ok_or("Transfer deadline overflow")?;
        if at < self.absolute_origin || self.last_observed_at.is_some_and(|old| at < old) {
            return Err("Original terminal observation clock rewound".into());
        }
        let success = match self.first_success_at {
            Some(first) => {
                if !terminal {
                    return Err(
                        "Original terminal success was lost during additional monitoring".into(),
                    );
                }
                first
            }
            None if terminal => {
                if at > deadline {
                    return Err(
                        "Original native terminal success arrived after the deadline".into(),
                    );
                }
                at
            }
            None => {
                if at >= deadline {
                    return Err(
                        "Original native recovery did not succeed before its deadline".into(),
                    );
                }
                self.last_observed_at = Some(at);
                return Ok(());
            }
        };
        if at > Self::monitoring_end(success)? {
            return Err("Additional original terminal monitoring second exhausted".into());
        }
        self.first_success_at = Some(success);
        self.last_observed_at = Some(at);
        Ok(())
    }
    fn monitoring_end(success: u64) -> Result<u64, String> {
        // Original stability remains at least 240 substeps. Tick finalization
        // adds zero to three observations without restarting that interval.
        let earliest = success
            .checked_add(240)
            .ok_or("Terminal monitoring clock overflow")?;
        earliest
            .checked_add(3)
            .map(|v| v / 4 * 4)
            .ok_or("Terminal monitoring tick overflow".into())
    }
    fn completed(&self, at: u64) -> Result<bool, String> {
        self.first_success_at
            .map(Self::monitoring_end)
            .transpose()
            .map(|end| end == Some(at))
    }
    fn require_next_step(&self, at: u64) -> Result<(), String> {
        if self.last_observed_at != Some(at) {
            return Err(
                "Native transfer action lacks a current original terminal observation".into(),
            );
        }
        let end = match self.first_success_at {
            Some(first) => Self::monitoring_end(first)?,
            None => self
                .absolute_origin
                .checked_add(6000)
                .ok_or("Terminal admission clock overflow")?,
        };
        if at >= end {
            return Err("No native step remains in the original terminal monitoring window".into());
        }
        Ok(())
    }
}

#[derive(Clone)]
enum Authorization {
    None,
    Technical {
        records: Vec<NativeRecord>,
        next: usize,
        instance: usize,
        end_identity: std::sync::Arc<[u8]>,
    },
    Preview {
        instance: usize,
        end: u64,
    },
    Real {
        records: Vec<NativeRecord>,
        next: usize,
        window_end: usize,
        checkpoints: Vec<NativeCheckpoint>,
    },
}

#[derive(Clone)]
pub(crate) struct Adapter {
    policy: Policy,
    tracker: MaterialTracker,
    entry: ObservedCommandEntry,
    local_bridge: Bridge,
    source: CompiledProgram,
    timing: Timing,
    absolute_origin: u64,
    raccord: Option<PreparedRaccord>,
    last_observed: u64,
    previous_dq: Coordinates,
    queue: Vec<Command>,
    queue_start: u64,
    last_proposal: Value,
    authorization: Authorization,
    native_records: Vec<NativeRecord>,
    native_checkpoints: Vec<NativeCheckpoint>,
    last_observation: SupportObservation,
    queue_rates: super::whole_body_tasks::Rates,
    passive_since: Option<u64>,
    preview_entry_identity: Option<std::sync::Arc<[u8]>>,
    preview_start: Option<u64>,
    terminal_clock: TerminalClock,
    immutable_scope: std::sync::Arc<[u8]>,
}

impl Adapter {
    pub(crate) fn new(
        model: &Model,
        observation: &SupportObservation,
        tracker: MaterialTracker,
        entry: ObservedCommandEntry,
        source: CompiledProgram,
        timing: Timing,
        intent: TransferIntent,
        absolute_origin: u64,
    ) -> Result<Self, String> {
        if observation.native_substep != entry.native_substep
            || entry.native_substep % CONTROL_STEPS != 0
            || absolute_origin > entry.native_substep
            || timing.recentered_hold_completed != 432
        {
            return Err(
                "Transfer entry must be the observed completed tick and original cut".into(),
            );
        }
        let immutable_scope=serde_json::to_vec(&json!({"schema":"observed-transfer-v1",
            "entry":entry,"source":source.source(),"intent":intent,"initialMaterial":tracker,
            "timing":[timing.heels_acquired,timing.recentered_hold_started,timing.recentered_hold_completed,timing.standing_reference,timing.hold_completed],
            "absoluteOrigin":absolute_origin})).map_err(|e|e.to_string())?.into();
        let policy = Policy::new(observation, &tracker, intent)?;
        let local_bridge = Bridge::prepare(model, &entry, &source.command(432))?;
        Ok(Self {
            policy,
            tracker,
            previous_dq: entry.previous_dq,
            last_observed: observation.native_substep,
            queue_start: observation.native_substep,
            entry,
            local_bridge,
            source,
            timing,
            absolute_origin,
            raccord: None,
            queue: vec![],
            last_proposal: Value::Null,
            authorization: Authorization::None,
            native_records: vec![],
            native_checkpoints: vec![],
            last_observation: observation.clone(),
            queue_rates: [0.; 46],
            passive_since: None,
            preview_entry_identity: None,
            preview_start: None,
            terminal_clock: TerminalClock::new(absolute_origin),
            immutable_scope,
        })
    }

    /// Complete decision state is copied by value. Authorization and recorded
    /// validation history are displayed separately because they never enter the
    /// task equations. Native identity matching excludes only those two fields.
    pub(crate) fn evidence(&self) -> Value {
        let authorization = match &self.authorization {
            Authorization::None => json!({"kind":"unarmed"}),
            Authorization::Technical {
                records,
                next,
                instance,
                ..
            } => {
                json!({"kind":"technical-four-only","records":records.len(),"next":next,"instance":instance})
            }
            Authorization::Preview { instance, end } => {
                json!({"kind":"preview","instance":instance,"end":end})
            }
            Authorization::Real {
                records,
                next,
                window_end,
                checkpoints,
            } => {
                json!({"kind":"native-window","next":next,"windowEnd":window_end,"records":records.len(),
                "checkpoints":checkpoints.iter().map(|c|c.at).collect::<Vec<_>>()})
            }
        };
        json!({"decision":self.decision_evidence(),"authorization":authorization,
            "nativeRecords":self.native_records.iter().map(|r|json!({"at":r.at,"identityBytes":r.identity.len(),
                "command":r.command.as_ref().map(|c|json!({"positions":c.positions,"velocities":c.velocities}))})).collect::<Vec<_>>()})
    }

    pub(crate) fn decision_evidence(&self) -> Value {
        json!({"policy":self.policy,"tracker":self.tracker,"entry":self.entry,
            "localBridge":self.local_bridge.evidence(),"source":self.source.source(),
            "timing":[self.timing.heels_acquired,self.timing.recentered_hold_started,
                self.timing.recentered_hold_completed,self.timing.standing_reference,self.timing.hold_completed],
            "absoluteOrigin":self.absolute_origin,"raccord":self.raccord.as_ref().map(|r|r.evidence()),
            "lastObserved":self.last_observed,"previousDq":self.previous_dq,"queueStart":self.queue_start,
            "queue":self.queue.iter().map(|c|json!({"positions":c.positions,"velocities":c.velocities})).collect::<Vec<_>>(),
            "lastProposal":self.last_proposal,"lastObservation":self.last_observation,
            "queueRates":self.queue_rates.to_vec(),"passiveSince":self.passive_since,
            "terminalClock":self.terminal_clock})
    }

    pub(super) fn immutable_scope(&self) -> &[u8] {
        &self.immutable_scope
    }

    pub(crate) fn observe_native_terminal(
        &mut self,
        at: u64,
        terminal: bool,
    ) -> Result<(), String> {
        self.terminal_clock.observe(at, terminal)
    }

    /// Endpoint/capture checks do not alter the complete-tick identity. The
    /// actual owner updates the same clock immediately before configure identity.
    pub(crate) fn check_native_terminal(&self, at: u64, terminal: bool) -> Result<(), String> {
        self.terminal_clock.clone().observe(at, terminal)
    }

    pub(crate) fn terminal_monitoring_complete(
        &self,
        at: u64,
        terminal: bool,
    ) -> Result<bool, String> {
        let mut clock = self.terminal_clock.clone();
        clock.observe(at, terminal)?;
        clock.completed(at)
    }

    pub(crate) fn tracker(&self) -> &MaterialTracker {
        &self.tracker
    }
    pub(crate) fn policy(&self) -> &Policy {
        &self.policy
    }

    /// Imported observation at the entry is already counted once. Every later
    /// observation is counted exactly once; rejection is atomic.
    pub(crate) fn observe(
        &mut self,
        observation: &SupportObservation,
    ) -> Result<TransferProposal, String> {
        let mut next = self.clone();
        observation.validate()?;
        if observation.native_substep == next.last_observed
            && serde_json::to_vec(observation).map_err(|e| e.to_string())?
                != serde_json::to_vec(&next.last_observation).map_err(|e| e.to_string())?
        {
            return Err("Different transfer observation at an already counted clock".into());
        }
        if observation.native_substep != next.last_observed {
            if next.last_observed.checked_add(1) != Some(observation.native_substep) {
                return Err("Transfer observation clock is discontinuous".into());
            }
            for patch in &mut next.tracker.active {
                patch.observe(observation, MATERIAL_MATCH_M)?;
            }
            next.policy.observe(observation)?;
            next.last_observed = observation.native_substep;
            next.last_observation = observation.clone();
        }
        // Ten millimetres is the existing planted-foot idle envelope, reused
        // conservatively for THIS candidate, not a universal dynamic necessity.
        let drift_limit = f64::from(
            crate::recovery_contract::contract()
                .acceptance
                .idle_planted_foot_drift_m,
        );
        if next
            .tracker
            .active
            .iter()
            .any(|p| !p.current_contact_present || p.tangential_drift_m > drift_limit)
        {
            return Err("Retained material contact lost or candidate's original planted-foot drift envelope exceeded".into());
        }
        let proposed = next.policy.propose(&next.tracker)?;
        if !proposed
            .measured_local_diagnostic
            .nonfoot_persistent_segments
            .is_empty()
        {
            return Err(
                "Bilateral candidate acquired a persistent nonfoot support dependency".into(),
            );
        }
        // Add a heel only after THIS policy's own point correspondence has
        // persisted. Existing forefoot ids and anchors are never replaced.
        for heel in &proposed.heel_observations {
            if heel.goal_matching_loaded_substeps >= 12
                && !next
                    .tracker
                    .active
                    .iter()
                    .any(|p| p.segment == heel.segment)
            {
                next.tracker.acquire(
                    observation,
                    heel.segment,
                    heel.goal_matching_contact_index
                        .ok_or("Persistent heel lacks current material point")?,
                    ContactMode::Hold,
                )?;
            }
        }
        let proposed = next.policy.propose(&next.tracker)?;
        next.last_proposal = serde_json::to_value(&proposed).map_err(|e| e.to_string())?;
        *self = next;
        Ok(proposed)
    }

    pub(crate) fn nominal(
        &mut self,
        model: &Model,
        memory: &runtime::Memory,
        at: u64,
    ) -> Result<Command, String> {
        if at != memory.next_substep || at != self.last_observed {
            return Err("Transfer nominal does not match measured and command clocks".into());
        }
        if self.raccord.is_none() && self.policy.continuation_event().is_some() {
            // The entry is captured NOW from the last installed command. The
            // old A36 command is never reused after waiting for physical load.
            self.raccord = Some(PreparedRaccord::prepare(
                model,
                &ObservedCommandEntry {
                    previous_q: memory.previous,
                    previous_dq: self.previous_dq,
                    native_substep: at,
                },
                &self.source,
                self.timing,
                Some(LocalReadyReceipt {
                    native_substep: at,
                    first_stable_substep: self
                        .policy
                        .first_stable_substep()
                        .ok_or("Missing measured local interval")?,
                }),
                self.absolute_origin,
            )?);
        }
        if let Some(raccord) = &self.raccord {
            return raccord.command(at);
        }
        let index = at
            .checked_sub(self.entry.native_substep)
            .ok_or("Transfer reference rewind")?
            + 1;
        let duration = u64::from(self.local_bridge.duration_substeps());
        let mut command = self.local_bridge.command(index.min(duration) as u32)?;
        if index > duration {
            command.velocities = [[0.; 3]; 25];
        }
        Ok(command)
    }

    pub(crate) fn reference_substep(&self, at: u64) -> u64 {
        self.raccord
            .as_ref()
            .map_or(432, |r| {
                let elapsed = at.saturating_sub(r.timing.start_native_substep);
                if elapsed < u64::from(r.timing.bridge_substeps) {
                    432
                } else {
                    433 + elapsed - u64::from(r.timing.bridge_substeps)
                }
            })
            .min(u64::from(self.source.duration_substeps()))
    }

    pub(crate) fn terminal_requested(&self, at: u64) -> bool {
        self.raccord
            .as_ref()
            .is_some_and(|r| at >= r.timing.standing_command_substep)
    }

    /// Pure preparation: the exact four-command f32 representation is retained.
    pub(crate) fn compile_batch(
        &mut self,
        model: &Model,
        memory: &runtime::Memory,
        problem: &Problem,
        at: u64,
    ) -> Result<(), String> {
        if at % CONTROL_STEPS != 0 || at != memory.next_substep || at != self.last_observed {
            return Err("Transfer compilation outside the common 60 Hz boundary".into());
        }
        let proposal = command_proposal::prepare(model, memory, problem, &self.tracker.active, at)?;
        self.queue = proposal.commands_for_native_preview().to_vec();
        self.queue_start = at;
        self.queue_rates = proposal.rates;
        self.last_proposal["commandCompilation"] = json!({"physicalAdmission":proposal.physical_admission,
            "guides":proposal.material_guides,"residuals":proposal.guide_residuals,
            "continuousConstraintError":proposal.continuous_qp_constraint_error});
        Ok(())
    }

    pub(crate) fn proposed_command(
        &self,
        model: &Model,
        memory: &runtime::Memory,
        at: u64,
    ) -> Result<Command, String> {
        let offset = at
            .checked_sub(self.queue_start)
            .ok_or("Transfer queue rewind")?;
        let c = self
            .queue
            .get(offset as usize)
            .ok_or("Missing certified command; no fallback")?
            .clone();
        if offset >= CONTROL_STEPS || memory.next_substep != at {
            return Err("Stale transfer command queue".into());
        }
        validate_command(model, &c)?;
        for i in 0..25 {
            for k in 0..3 {
                if c.velocities[i][k].to_bits()
                    != ((c.positions[i][k] - memory.previous[i][k]) / lh_contracts::SUBSTEP_S)
                        .to_bits()
                {
                    return Err("Transfer emitted q/dq differs from actual command history".into());
                }
            }
        }
        Ok(c)
    }

    pub(crate) fn commit_command(
        &mut self,
        memory: &mut runtime::Memory,
        command: &Command,
        at: u64,
    ) -> Result<(), String> {
        if memory.next_substep != at {
            return Err("Transfer command already consumed".into());
        }
        let offset = at
            .checked_sub(self.queue_start)
            .ok_or("Transfer commit queue rewind")?;
        if offset >= CONTROL_STEPS
            || self
                .queue
                .get(offset as usize)
                .is_none_or(|expected| !command_bits_equal(expected, command))
        {
            return Err("Transfer commit differs from its certified command queue".into());
        }
        for i in 0..25 {
            for k in 0..3 {
                if command.velocities[i][k].to_bits()
                    != ((command.positions[i][k] - memory.previous[i][k]) / lh_contracts::SUBSTEP_S)
                        .to_bits()
                {
                    return Err("Transfer commit q/dq differs from actual command history".into());
                }
            }
        }
        let next = at.checked_add(1).ok_or("Transfer clock overflow")?;
        memory.previous = command.positions;
        memory.next_substep = next;
        memory.rates = self.queue_rates;
        self.previous_dq = command.velocities;
        Ok(())
    }

    // Only called from StaticPreviewBranch after its durable four-step reserve.
    pub(super) fn begin_native_preview(&mut self, start: u64, outer_identity: Vec<u8>) {
        self.native_records.clear();
        self.native_checkpoints.clear();
        self.preview_start = Some(start);
        self.preview_entry_identity = Some(outer_identity.into());
    }

    /// Completed-tick checkpoint, deliberately distinct from before-configure
    /// identities (native tick bookkeeping clears delivery accumulators first).
    pub(super) fn record_preview_post_tick(
        &mut self,
        at: u64,
        outer_identity: Vec<u8>,
        horizon_end: bool,
    ) -> Result<(), String> {
        let start = self
            .preview_start
            .ok_or("Native preview has no outer entry")?;
        let elapsed = at
            .checked_sub(start)
            .ok_or("Native post-tick checkpoint rewind")?;
        if elapsed == 0 || elapsed % 4 != 0 || elapsed != self.native_records.len() as u64 {
            return Err("Post-window native trace incomplete".into());
        }
        if elapsed % REAL_WINDOW_STEPS as u64 == 0 || horizon_end {
            if self.native_checkpoints.last().is_some_and(|c| c.at >= at) {
                return Err("Duplicate or rewound native checkpoint".into());
            }
            self.native_checkpoints.push(NativeCheckpoint {
                at,
                identity: outer_identity.into(),
            });
        }
        Ok(())
    }

    pub(super) fn preview_window(&mut self, instance: usize, start: u64) -> Result<(), String> {
        if self.passive_since.is_none()
            && start != self.last_observed
            && start != self.last_observed + 1
        {
            return Err("Preview authorization clock mismatch".into());
        }
        self.authorization = Authorization::Preview {
            instance,
            end: start.checked_add(4).ok_or("Preview window overflow")?,
        };
        Ok(())
    }

    pub(crate) fn require_native_command(
        &mut self,
        instance: usize,
        at: u64,
        identity: Vec<u8>,
        command: &Command,
    ) -> Result<(), String> {
        self.require_native_action(instance, at, identity, Some(command))
    }

    pub(crate) fn require_native_passive(
        &mut self,
        instance: usize,
        at: u64,
        identity: Vec<u8>,
    ) -> Result<(), String> {
        self.require_native_action(instance, at, identity, None)?;
        self.passive_since.get_or_insert(at);
        Ok(())
    }

    fn require_native_action(
        &mut self,
        instance: usize,
        at: u64,
        identity: Vec<u8>,
        command: Option<&Command>,
    ) -> Result<(), String> {
        self.terminal_clock.require_next_step(at)?;
        match &mut self.authorization {
            Authorization::Technical {
                records,
                next,
                instance: expected_instance,
                ..
            } => {
                let expected = records
                    .get(*next)
                    .ok_or("Technical transfer receipt exhausted after four steps")?;
                let same = match (&expected.command, command) {
                    (Some(a), Some(b)) => command_bits_equal(a, b),
                    (None, None) => true,
                    _ => false,
                };
                if instance != *expected_instance
                    || expected.at != at
                    || expected.identity.as_ref() != identity.as_slice()
                    || !same
                {
                    return Err(
                        "Technical transfer counterpart differs from its bound four-step forecast"
                            .into(),
                    );
                }
                *next += 1;
            }
            Authorization::None => {
                return Err(
                    "No budgeted native transfer window; static feasibility is not admission"
                        .into(),
                );
            }
            Authorization::Preview {
                instance: expected,
                end,
            } => {
                if instance != *expected || at >= *end {
                    return Err("Preview instance/window mismatch".into());
                }
                if self.native_records.len() >= 480
                    || self
                        .preview_start
                        .and_then(|start| start.checked_add(self.native_records.len() as u64))
                        != Some(at)
                {
                    return Err(
                        "Native preview action is not the next bounded forecast state".into(),
                    );
                }
                self.native_records.push(NativeRecord {
                    at,
                    identity: identity.into(),
                    command: command.cloned(),
                });
            }
            Authorization::Real {
                records,
                next,
                window_end,
                ..
            } => {
                if *next >= *window_end {
                    return Err("Native transfer window needs its completed-tick checkpoint before continuing".into());
                }
                let expected = records
                    .get(*next)
                    .ok_or("Native transfer lease exhausted; new complete preview required")?;
                let same = match (&expected.command, command) {
                    (Some(a), Some(b)) => command_bits_equal(a, b),
                    (None, None) => true,
                    _ => false,
                };
                if expected.at != at || expected.identity.as_ref() != identity.as_slice() || !same {
                    return Err("Actual transfer diverged from its exact native forecast before installation".into());
                }
                *next += 1;
            }
        }
        Ok(())
    }

    pub(crate) fn require_window_checkpoint(&self, at: u64, identity: &[u8]) -> Result<(), String> {
        if let Authorization::Technical {
            records,
            next,
            end_identity,
            ..
        } = &self.authorization
        {
            if *next != 4
                || records.first().and_then(|r| r.at.checked_add(4)) != Some(at)
                || end_identity.as_ref() != identity
            {
                return Err("Technical transfer tick lacks its exact completed checkpoint".into());
            }
        }
        if let Authorization::Real {
            records,
            next,
            window_end,
            checkpoints,
        } = &self.authorization
        {
            if *next != *window_end
                || records.first().and_then(|r| r.at.checked_add(*next as u64)) != Some(at)
                || checkpoints
                    .iter()
                    .find(|c| c.at == at)
                    .is_none_or(|c| c.identity.as_ref() != identity)
            {
                return Err("Prior native lease lacks its exact AFTER-window checkpoint".into());
            }
        }
        Ok(())
    }

    /// Continue only already predicted states. This neither creates a new
    /// forecast nor refunds or grants any integration budget.
    pub(crate) fn continue_validated_window(
        &mut self,
        at: u64,
        identity: &[u8],
    ) -> Result<(), String> {
        self.require_window_checkpoint(at, identity)?;
        let Authorization::Real {
            records,
            next,
            window_end,
            ..
        } = &mut self.authorization
        else {
            return Err("No completed native forecast to continue".into());
        };
        if *next >= records.len() {
            return Err("Completed native forecast exhausted".into());
        }
        *window_end = (*next + REAL_WINDOW_STEPS).min(records.len());
        Ok(())
    }

    pub(crate) fn admit_real_window(
        &mut self,
        receipt: super::native_plan_preview::NativeWindowReceipt,
        at: u64,
        identity: &[u8],
    ) -> Result<(), String> {
        self.admit_window_data(receipt.into_data(), at, identity)
    }

    fn admit_window_data(
        &mut self,
        receipt: NativeWindowData,
        at: u64,
        identity: &[u8],
    ) -> Result<(), String> {
        self.require_window_checkpoint(at, identity)?;
        let steps = receipt.horizon_steps as usize;
        if !(120..=480).contains(&steps)
            || steps % 4 != 0
            || receipt.records.len() != steps
            || receipt
                .records
                .iter()
                .enumerate()
                .any(|(i, r)| at.checked_add(i as u64) != Some(r.at))
        {
            return Err("Incomplete native horizon receipt".into());
        }
        let end = at
            .checked_add(steps as u64)
            .ok_or("Native horizon receipt clock overflow")?;
        let mut required: Vec<u64> = (REAL_WINDOW_STEPS..=steps)
            .step_by(REAL_WINDOW_STEPS)
            .map(|i| at + i as u64)
            .collect();
        if steps % REAL_WINDOW_STEPS != 0 {
            required.push(end);
        }
        if receipt.checkpoints.iter().map(|c| c.at).collect::<Vec<_>>() != required {
            return Err("Native horizon receipt lacks complete ordered window checkpoints".into());
        }
        if receipt.entry_identity.as_ref() != identity {
            return Err("Native receipt does not begin at the current complete state".into());
        }
        self.authorization = Authorization::Real {
            records: receipt.records,
            next: 0,
            window_end: REAL_WINDOW_STEPS,
            checkpoints: receipt.checkpoints,
        };
        Ok(())
    }

    pub(super) fn technical_data(
        &self,
        start: u64,
        instance: usize,
        scope: Vec<u8>,
    ) -> Result<NativeTechnicalData, String> {
        if self.preview_start != Some(start)
            || self.native_records.len() != 4
            || self
                .native_records
                .iter()
                .enumerate()
                .any(|(i, r)| start.checked_add(i as u64) != Some(r.at))
            || self.native_checkpoints.len() != 1
            || start.checked_add(4) != Some(self.native_checkpoints[0].at)
        {
            return Err(
                "Technical receipt needs exactly four completed native forecast actions".into(),
            );
        }
        Ok(NativeTechnicalData {
            records: self.native_records.clone(),
            entry_identity: self
                .preview_entry_identity
                .clone()
                .ok_or("Missing technical entry identity")?,
            end_identity: self.native_checkpoints[0].identity.clone(),
            instance,
            scope,
        })
    }

    pub(crate) fn admit_technical_tick(
        &mut self,
        receipt: super::native_plan_preview::NativeTechnicalReceipt,
        instance: usize,
        at: u64,
        identity: &[u8],
        scope: &[u8],
    ) -> Result<(), String> {
        self.admit_technical_data(receipt.into_data(), instance, at, identity, scope)
    }
    fn admit_technical_data(
        &mut self,
        data: NativeTechnicalData,
        instance: usize,
        at: u64,
        identity: &[u8],
        scope: &[u8],
    ) -> Result<(), String> {
        if !matches!(self.authorization, Authorization::None)
            || data.instance != instance
            || data.scope != scope
            || data.records.len() != 4
            || data
                .records
                .iter()
                .enumerate()
                .any(|(i, r)| at.checked_add(i as u64) != Some(r.at))
            || data.entry_identity.as_ref() != identity
        {
            return Err("Technical receipt is stale, foreign, or already used; it cannot renew an execution lease".into());
        }
        self.authorization = Authorization::Technical {
            records: data.records,
            next: 0,
            instance,
            end_identity: data.end_identity,
        };
        Ok(())
    }

    pub(super) fn complete_native_preview(
        &self,
        start: u64,
        steps: u32,
        final_observation: &SupportObservation,
    ) -> Result<NativeWindowData, String> {
        final_observation.validate()?;
        if final_observation.segments.iter().any(|s| {
            !matches!(
                s.role,
                super::support_state::SegmentRole::Hindfoot
                    | super::support_state::SegmentRole::Forefoot
            ) && s.loaded_substeps >= 12
                && s.persistent_load_n >= 3.
        }) {
            return Err(
                "Bilateral native horizon ended with a persistent nonfoot dependency".into(),
            );
        }
        if !(120..=480).contains(&steps)
            || steps % 4 != 0
            || self.preview_start != Some(start)
            || final_observation.native_substep != start + u64::from(steps)
            || self.native_records.len() != steps as usize
            || self
                .native_records
                .iter()
                .enumerate()
                .any(|(i, r)| r.at != start + i as u64)
        {
            return Err("Native transfer horizon incomplete or not consecutive".into());
        }
        let mut checked = self.clone();
        let last = checked.observe(final_observation)?;
        if !last.continue_bilateral {
            return Err(
                "Native horizon did not establish the measured bilateral continuation event".into(),
            );
        }
        Ok(NativeWindowData {
            records: self.native_records.clone(),
            horizon_steps: steps,
            checkpoints: self.native_checkpoints.clone(),
            entry_identity: self
                .preview_entry_identity
                .clone()
                .ok_or("Missing outer entry native checkpoint")?,
        })
    }
}

fn command_bits_equal(a: &Command, b: &Command) -> bool {
    a.positions
        .iter()
        .flatten()
        .chain(a.velocities.iter().flatten())
        .zip(
            b.positions
                .iter()
                .flatten()
                .chain(b.velocities.iter().flatten()),
        )
        .all(|(a, b)| a.to_bits() == b.to_bits())
}

#[cfg(test)]
#[path = "transfer_adapter_tests.rs"]
mod tests;
