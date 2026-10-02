//! Worker-ready lifecycle owner. No clocks, DOM, renderer, or pose setters.
//! Station changes construct fresh physical trials before acknowledging them.
use crate::{Profile, Simulation, measurement::Failure};
use lh_contracts::{
    Acknowledgement, Action, Command, MAX_ADVANCE_TICKS, Mode, Reject, SCHEMA_VERSION, Snapshot,
    Stamp, Timeline,
};
use serde::Serialize;

#[derive(Serialize)]
pub struct Reply {
    pub completed_ticks: u32,
    pub paused: bool,
    pub visible: bool,
    pub mode: Mode,
    pub acknowledgements: Vec<Acknowledgement>,
    pub snapshot: Snapshot,
    pub failure: Option<Failure>,
}
pub struct Runtime {
    simulation: Simulation,
    quiet: Option<crate::quiet::QuietObservation>,
    timeline: Timeline,
    heading: f32,
    mode: Mode,
    paused: bool,
    visible: bool,
    disposed: bool,
    highest_press: u64,
    observation_sequence: u64,
    playground_settings: lh_contracts::PlaygroundSettings,
    acknowledgements: Vec<Acknowledgement>,
}
fn acknowledgement(
    command: &Command,
    tick: Option<u64>,
    rejected: Option<Reject>,
) -> Acknowledgement {
    Acknowledgement {
        stamp: command.stamp,
        applied_tick: tick,
        rejected,
    }
}
impl Runtime {
    pub fn new(mode: Mode, heading: f32, profile: Profile) -> Result<Self, String> {
        Self::new_trial(mode, heading, profile, 1, true)
    }
    /// A replacement worker starts an empty trial in a fresh generation. This
    /// initializes bodies once; it cannot transfer active input or running poses.
    pub fn new_trial(
        mode: Mode,
        heading: f32,
        profile: Profile,
        generation: u64,
        floor_enabled: bool,
    ) -> Result<Self, String> {
        Self::new_trial_with_settings(
            mode,
            heading,
            profile,
            generation,
            floor_enabled,
            Default::default(),
        )
    }
    pub fn new_trial_with_settings(
        mode: Mode,
        heading: f32,
        profile: Profile,
        generation: u64,
        floor_enabled: bool,
        playground_settings: lh_contracts::PlaygroundSettings,
    ) -> Result<Self, String> {
        if generation == 0 || generation > 9_007_199_254_740_991 {
            return Err("Invalid initial generation".into());
        }
        let mut simulation = match mode {
            Mode::Protocol => Simulation::new_protocol(heading, profile, floor_enabled)?,
            Mode::Playground => {
                Simulation::new_playground(heading, profile, floor_enabled, playground_settings)?
            }
        };
        simulation.generation = generation;
        Ok(Self {
            timeline: Timeline::new(simulation.generation),
            simulation,
            quiet: None,
            heading,
            mode,
            paused: false,
            visible: true,
            disposed: false,
            highest_press: 0,
            observation_sequence: 0,
            playground_settings,
            acknowledgements: Vec::new(),
        })
    }
    /// None means queued, not applied. Every command receives a later application
    /// or rejection acknowledgement at its exact requested tick boundary.
    pub fn enqueue(&mut self, command: Command) -> Result<(), Reject> {
        if self.disposed {
            return Err(Reject::Invalid);
        }
        if self.quiet.is_some() {
            return Err(Reject::Unsupported);
        }
        if command.stamp.schema != SCHEMA_VERSION {
            return Err(Reject::Schema);
        }
        if command.stamp.generation != self.timeline.generation {
            return Err(Reject::Generation);
        }
        if matches!(command.action, Action::Strike)
            && self.mode == Mode::Protocol
            && (self.paused
                || !self.visible
                || self.simulation.first_failure.is_some()
                || self
                    .simulation
                    .striker
                    .as_ref()
                    .is_none_or(|s| !s.available()))
        {
            return Err(Reject::Cancelled);
        }
        if (!self.visible
            || self.paused
            || !matches!(
                self.simulation.motion,
                lh_contracts::Motion::Upright | lh_contracts::Motion::Reacting
            ))
            && matches!(
                command.action,
                Action::GrabBegin { .. } | Action::GrabMove { .. }
            )
        {
            return Err(Reject::Cancelled);
        }
        if matches!(
            command.action,
            Action::Pause
                | Action::Resume
                | Action::Reset
                | Action::Initialize { .. }
                | Action::SelectStation { .. }
                | Action::SetDifficulty { .. }
                | Action::Strike
        ) && command.stamp.tick != self.timeline.tick
        {
            return Err(Reject::Late);
        }
        let press = match &command.action {
            Action::GrabBegin { press, .. } => Some(*press),
            _ => None,
        };
        self.timeline.enqueue(command)?;
        if let Some(press) = press {
            self.highest_press = self.highest_press.max(press);
        }
        Ok(())
    }
    fn cancel_pointer_intent(&mut self) {
        self.simulation.cancel_grab();
        self.simulation.last_press = self.simulation.last_press.max(self.highest_press);
        for command in self.timeline.cancel_grabs() {
            self.acknowledgements
                .push(acknowledgement(&command, None, Some(Reject::Cancelled)));
        }
    }
    /// Enqueue is atomic across a worker packet. Rejection cannot leave hidden
    /// intent waiting to execute after the caller receives a failed packet.
    pub fn enqueue_batch(&mut self, commands: Vec<Command>) -> Result<(), Reject> {
        if self.disposed {
            return Err(Reject::Invalid);
        }
        if commands.len() > lh_contracts::MAX_QUEUED_COMMANDS {
            return Err(Reject::QueueFull);
        }
        let before = self.timeline.clone();
        let highest_press = self.highest_press;
        for command in commands {
            if let Err(reject) = self.enqueue(command) {
                self.timeline = before;
                self.highest_press = highest_press;
                return Err(reject);
            }
        }
        Ok(())
    }
    pub fn set_visible(&mut self, visible: bool) -> Result<(), Reject> {
        if self.disposed {
            return Err(Reject::Invalid);
        }
        if self.quiet.is_some() && !visible {
            return Err(Reject::Unsupported);
        }
        if !visible {
            self.cancel_pointer_intent();
        }
        self.visible = visible;
        Ok(())
    }
    pub fn shutdown(&mut self) {
        self.cancel_pointer_intent();
        for command in self.timeline.clear() {
            self.acknowledgements
                .push(acknowledgement(&command, None, Some(Reject::Cancelled)));
        }
        self.disposed = true;
    }
    fn replace(
        &mut self,
        mode: Mode,
        heading: f32,
        settings: lh_contracts::PlaygroundSettings,
    ) -> Result<(), Reject> {
        let generation = self
            .timeline
            .generation
            .checked_add(1)
            .ok_or(Reject::Invalid)?;
        let floor = self.simulation.world.colliders[self.simulation.floor].is_enabled();
        let profile = self.simulation.profile.clone();
        let mut next = match mode {
            Mode::Protocol => Simulation::new_protocol(heading, profile, floor),
            Mode::Playground => Simulation::new_playground(heading, profile, floor, settings),
        }
        .map_err(|_| Reject::Invalid)?;
        next.generation = generation;
        next.last_press = self.highest_press;
        // Construct first so a failed initialization preserves the old world.
        self.cancel_pointer_intent();
        for command in self.timeline.clear() {
            self.acknowledgements
                .push(acknowledgement(&command, None, Some(Reject::Generation)));
        }
        self.simulation = next;
        self.timeline = Timeline::new(generation);
        self.heading = heading;
        self.mode = mode;
        self.playground_settings = settings;
        self.observation_sequence = 0;
        Ok(())
    }
    fn apply(&mut self, command: &Command) -> Result<(), Reject> {
        if command.stamp.generation != self.timeline.generation {
            return Err(Reject::Generation);
        }
        match command.action {
            Action::Pause => {
                self.paused = true;
                self.cancel_pointer_intent();
            }
            Action::Resume => {
                self.paused = false;
            }
            Action::CancelGrab => self.cancel_pointer_intent(),
            Action::Reset => self.replace(self.mode, self.heading, self.playground_settings)?,
            Action::Initialize { mode, heading } => {
                self.replace(mode, heading, self.playground_settings)?
            }
            Action::SetFloorEnabled { enabled } => self
                .simulation
                .set_floor_enabled(enabled)
                .map_err(|_| Reject::Invalid)?,
            Action::SelectStation { station } if self.mode == Mode::Playground => {
                self.replace(
                    self.mode,
                    self.heading,
                    lh_contracts::PlaygroundSettings {
                        station,
                        ..self.playground_settings
                    },
                )?;
            }
            Action::SetDifficulty { difficulty } if self.mode == Mode::Playground => {
                self.replace(
                    self.mode,
                    self.heading,
                    lh_contracts::PlaygroundSettings {
                        difficulty,
                        ..self.playground_settings
                    },
                )?;
            }
            Action::Strike if self.mode == Mode::Protocol => {
                if self.paused
                    || !self.visible
                    || !self
                        .simulation
                        .request_strike()
                        .map_err(|_| Reject::Invalid)?
                {
                    return Err(Reject::Cancelled);
                }
            }
            Action::Strike | Action::SelectStation { .. } | Action::SetDifficulty { .. } => {
                return Err(Reject::Unsupported);
            }
            _ => {
                if (!self.visible
                    || self.paused
                    || !matches!(
                        self.simulation.motion,
                        lh_contracts::Motion::Upright | lh_contracts::Motion::Reacting
                    ))
                    && matches!(
                        command.action,
                        Action::GrabBegin { .. } | Action::GrabMove { .. }
                    )
                {
                    return Err(Reject::Cancelled);
                }
                self.simulation.apply_grab_action(&command.action)?;
            }
        }
        if command.stamp.generation == self.timeline.generation {
            self.timeline.last_applied = self.timeline.last_applied.max(command.stamp.sequence);
            self.simulation.applied_sequence = self.timeline.last_applied;
        }
        Ok(())
    }
    fn boundary(&mut self) {
        for command in self.timeline.boundary() {
            let tick = self.timeline.tick;
            let result = self.apply(&command);
            self.acknowledgements.push(acknowledgement(
                &command,
                result.as_ref().ok().map(|_| tick),
                result.err(),
            ));
        }
    }
    pub fn advance(&mut self, ticks: u32) -> Result<Reply, Reject> {
        self.advance_traced(ticks, |_, _| {})
    }
    /// Only an untouched, visible trial on the flat floor can be measured.
    /// No body state or controller setting is changed by this observer.
    pub fn enable_quiet_measurement(&mut self) -> Result<(), Reject> {
        if self.disposed
            || self.quiet.is_some()
            || self.paused
            || !self.visible
            || self.simulation.tick != 0
            || self.timeline.depth() != 0
            || self.timeline.last_applied != 0
            || self.simulation.first_failure.is_some()
            || self.mode == Mode::Playground
                && self.playground_settings.station != lh_contracts::Station::Flat
            || !self.simulation.world.colliders[self.simulation.floor].is_enabled()
        {
            return Err(Reject::Invalid);
        }
        self.quiet = Some(crate::quiet::QuietObservation::new(
            &self.simulation,
            self.heading,
        ));
        Ok(())
    }
    pub fn quiet_progress(&self) -> Option<serde_json::Value> {
        self.quiet
            .as_ref()
            .map(|observer| observer.progress(&self.simulation))
    }
    pub fn enable_quiet_drift_trace(&mut self) -> Result<(), Reject> {
        if self.simulation.tick != 0 || self.disposed {
            return Err(Reject::Invalid);
        }
        if !self
            .quiet
            .as_mut()
            .ok_or(Reject::Invalid)?
            .enable_drift_trace(&self.simulation)
        {
            return Err(Reject::Invalid);
        }
        Ok(())
    }
    pub fn quiet_report(&self) -> Option<serde_json::Value> {
        self.quiet.as_ref().map(|observer| {
            let mut report = observer.report(&self.simulation);
            report["mode"] = serde_json::json!(self.mode);
            report
        })
    }
    pub fn advance_observed(
        &mut self,
        ticks: u32,
        observer: impl FnMut(&Simulation, u32) -> Result<(), Failure>,
    ) -> Result<Reply, Reject> {
        self.advance_with(ticks, observer, |_, _| {})
    }
    pub fn advance_traced(
        &mut self,
        ticks: u32,
        trace: impl FnMut(&Simulation, crate::profiling::Event),
    ) -> Result<Reply, Reject> {
        self.advance_with(ticks, |_, _| Ok(()), trace)
    }
    fn advance_with(
        &mut self,
        ticks: u32,
        mut observer: impl FnMut(&Simulation, u32) -> Result<(), Failure>,
        mut trace: impl FnMut(&Simulation, crate::profiling::Event),
    ) -> Result<Reply, Reject> {
        if self.disposed || ticks > MAX_ADVANCE_TICKS {
            return Err(Reject::Invalid);
        }
        let mut completed_ticks = 0;
        // Tick zero is also a lifecycle boundary while hidden or paused.
        if ticks == 0 {
            self.boundary();
        }
        for _ in 0..ticks {
            let event = |s: &Simulation, begin| crate::profiling::Event {
                generation: s.generation,
                tick: s.tick,
                substep: 0,
                phase: crate::profiling::Phase::Tick,
                begin,
            };
            trace(&self.simulation, event(&self.simulation, true));
            self.boundary();
            if self.paused
                || !self.visible
                || self.simulation.first_failure.is_some()
                || (self.quiet.is_some() && self.simulation.tick == crate::quiet::END_TICK)
            {
                trace(&self.simulation, event(&self.simulation, false));
                break;
            }
            let quiet = &mut self.quiet;
            if self
                .simulation
                .advance_tick_traced(
                    |simulation, substep| {
                        if let Some(quiet) = quiet.as_mut() {
                            quiet.observe(simulation, substep)?;
                        }
                        observer(simulation, substep)
                    },
                    &mut trace,
                )
                .is_err()
            {
                self.cancel_pointer_intent();
                trace(&self.simulation, event(&self.simulation, false));
                break;
            }
            self.timeline.tick = self.simulation.tick;
            if matches!(
                self.simulation.motion,
                lh_contracts::Motion::Falling | lh_contracts::Motion::Fallen
            ) {
                self.cancel_pointer_intent();
            }
            if let Some(quiet) = self.quiet.as_mut() {
                quiet.completed_tick(&self.simulation);
            }
            completed_ticks += 1;
            trace(&self.simulation, event(&self.simulation, false));
        }
        self.observation_sequence = self
            .observation_sequence
            .checked_add(1)
            .ok_or(Reject::Invalid)?;
        let mut snapshot = self.simulation.snapshot();
        snapshot.stamp = Stamp::new(
            self.timeline.generation,
            self.observation_sequence,
            self.timeline.tick,
        );
        snapshot.diagnostics.queue_depth = self.timeline.depth() as u32;
        Ok(Reply {
            completed_ticks,
            paused: self.paused,
            visible: self.visible,
            mode: self.mode,
            acknowledgements: std::mem::take(&mut self.acknowledgements),
            snapshot,
            failure: self.simulation.first_failure.clone(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use lh_model::Vec3;
    #[test]
    fn read_only_observer_keeps_every_physical_reply_and_visits_each_substep() {
        let mut plain = runtime();
        let mut observed = runtime();
        let mut visits = Vec::new();
        for _ in 0..6 {
            let expected = plain.advance(4).unwrap();
            let actual = observed
                .advance_observed(4, |simulation, substep| {
                    visits.push((simulation.tick, substep));
                    Ok(())
                })
                .unwrap();
            assert_eq!(
                serde_json::to_value(actual).unwrap(),
                serde_json::to_value(expected).unwrap()
            );
        }
        assert_eq!(
            visits,
            (0..24)
                .flat_map(|tick| (0..4).map(move |substep| (tick, substep)))
                .collect::<Vec<_>>()
        );
    }
    #[test]
    fn observer_failure_halts_at_its_exact_substep_and_cancels_future_grabs() {
        let mut owner = runtime();
        submit(&mut owner, 1, 0, begin(41));
        submit(&mut owner, 2, 2, begin(42));
        let reply = owner
            .advance_observed(4, |simulation, substep| {
                if simulation.tick == 1 && substep == 2 {
                    Err(Failure::new(
                        1,
                        2,
                        "observer-sentinel",
                        "read-only test".into(),
                        1.0,
                        0.0,
                    ))
                } else {
                    Ok(())
                }
            })
            .unwrap();
        assert_eq!(reply.completed_ticks, 1);
        assert_eq!(reply.failure.as_ref().unwrap().check, "observer-sentinel");
        assert_eq!(reply.failure.as_ref().unwrap().substep, 2);
        assert!(owner.simulation.grab.is_none());
        assert_eq!(owner.timeline.depth(), 0);
        assert!(
            reply
                .acknowledgements
                .iter()
                .any(|ack| ack.stamp.sequence == 2 && ack.rejected == Some(Reject::Cancelled))
        );
        let frozen = serde_json::to_value(&reply.snapshot.segments).unwrap();
        let again = owner.advance(4).unwrap();
        assert_eq!(again.completed_ticks, 0);
        assert_eq!(
            serde_json::to_value(&again.snapshot.segments).unwrap(),
            frozen
        );
    }
    #[test]
    fn quiet_trial_rejects_contaminated_start_and_commands() {
        let mut pending = runtime();
        submit(&mut pending, 1, 0, begin(1));
        assert_eq!(pending.enable_quiet_measurement(), Err(Reject::Invalid));
        let mut hidden = runtime();
        hidden.set_visible(false).unwrap();
        assert_eq!(hidden.enable_quiet_measurement(), Err(Reject::Invalid));
        let mut advanced = runtime();
        advanced.advance(1).unwrap();
        assert_eq!(advanced.enable_quiet_measurement(), Err(Reject::Invalid));
        let mut no_floor =
            Runtime::new_trial(Mode::Protocol, 0.0, Profile::default(), 1, false).unwrap();
        assert_eq!(no_floor.enable_quiet_measurement(), Err(Reject::Invalid));
        let mut quiet = runtime();
        quiet.enable_quiet_measurement().unwrap();
        assert_eq!(quiet.enable_quiet_measurement(), Err(Reject::Invalid));
        assert_eq!(
            quiet.enqueue(Command {
                stamp: Stamp::new(1, 1, 0),
                action: begin(1)
            }),
            Err(Reject::Unsupported)
        );
        assert_eq!(quiet.set_visible(false), Err(Reject::Unsupported));
        assert!(quiet.advance(5).is_err());
        assert_eq!(quiet.quiet_progress().unwrap()["quietSubsteps"], 0);
    }
    #[test]
    fn full_quiet_trial_matches_the_independent_native_observer_and_stops_at_1920() {
        let profile = Profile::default();
        let mut direct = Simulation::new_protocol(0.0, profile.clone(), true).unwrap();
        let mut owner = Runtime::new(Mode::Protocol, 0.0, profile).unwrap();
        assert_eq!(owner.enable_quiet_drift_trace(), Err(Reject::Invalid));
        owner.enable_quiet_measurement().unwrap();
        owner.enable_quiet_drift_trace().unwrap();
        assert_eq!(owner.enable_quiet_drift_trace(), Err(Reject::Invalid));
        let initial = direct.snapshot();
        for _ in 0..120 {
            direct.advance_tick().unwrap();
        }
        let settled = direct.snapshot();
        let mut metrics = crate::measurement::QuietMetrics::default();
        for _ in 0..1800 {
            direct
                .advance_tick_observed(|simulation, substep| {
                    metrics.observe(simulation, &settled, substep)
                })
                .unwrap();
        }
        for _ in 0..639 {
            owner.advance(3).unwrap();
        }
        assert_eq!(owner.advance(2).unwrap().completed_ticks, 2);
        assert_eq!(owner.advance(4).unwrap().completed_ticks, 1);
        let report = owner.quiet_report().unwrap();
        assert_eq!(report["progress"]["passed"], true);
        assert_eq!(report["progress"]["settlingSubsteps"], 480);
        assert_eq!(report["progress"]["quietSubsteps"], 7200);
        assert_eq!(report["initial"], serde_json::to_value(initial).unwrap());
        assert_eq!(report["settled"], serde_json::to_value(settled).unwrap());
        assert_eq!(
            report["final"],
            serde_json::to_value(direct.snapshot()).unwrap()
        );
        assert_eq!(
            report["measurements"]["quiet"],
            serde_json::to_value(metrics).unwrap()
        );
        let samples = report["driftTrace"]["samples"].as_array().unwrap();
        assert_eq!(samples.len(), 33);
        assert_eq!(samples[0]["completedSubsteps"], 0);
        assert_eq!(samples[32]["completedSubsteps"], 7680);
        assert_eq!(
            samples[32]["simulationTimeS"],
            report["final"]["simulation_time_s"]
        );
        assert!(
            samples
                .iter()
                .all(|sample| sample["legs"].as_array().unwrap().len() == 10)
        );
        assert_eq!(owner.enable_quiet_drift_trace(), Err(Reject::Invalid));
        assert_eq!(owner.advance(4).unwrap().completed_ticks, 0);
        assert_eq!(owner.quiet_report().unwrap(), report);
    }
    fn runtime() -> Runtime {
        Runtime::new(Mode::Playground, 0.0, Profile::default()).expect("runtime")
    }
    fn submit(r: &mut Runtime, sequence: u64, tick: u64, action: Action) {
        r.enqueue(Command {
            stamp: Stamp::new(r.timeline.generation, sequence, tick),
            action,
        })
        .expect("enqueue");
    }
    fn begin(press: u64) -> Action {
        Action::GrabBegin {
            press,
            segment: 14,
            local_anchor: Vec3::default(),
            target: Vec3 {
                x: 0.2,
                y: 0.9,
                z: 0.0,
            },
        }
    }
    #[test]
    fn replacement_trial_rejects_old_generation_and_starts_with_fresh_intent() {
        let mut baseline = runtime();
        let initial = baseline.advance(0).expect("initial");
        let mut replacement =
            Runtime::new_trial(Mode::Playground, 0.0, Profile::default(), 42, true)
                .expect("replacement");
        let restarted = replacement.advance(0).expect("restarted");
        assert_eq!(restarted.snapshot.stamp.generation, 42);
        assert_eq!(restarted.snapshot.stamp.tick, 0);
        assert_eq!(
            serde_json::to_value(&initial.snapshot.segments).unwrap(),
            serde_json::to_value(&restarted.snapshot.segments).unwrap()
        );
        assert_eq!(
            replacement.enqueue(Command {
                stamp: Stamp::new(1, 1, 0),
                action: begin(41),
            }),
            Err(Reject::Generation)
        );
        let stepped = replacement.advance(1).expect("step");
        assert!(stepped.failure.is_none());
        assert!(replacement.simulation.grab.is_none());
        assert!(Runtime::new_trial(Mode::Playground, 0.0, Profile::default(), 0, true).is_err());
        let mut unsupported =
            Runtime::new_trial(Mode::Playground, 0.0, Profile::default(), 43, false).unwrap();
        assert!(unsupported.advance(4).unwrap().failure.is_none());
        assert!(!unsupported.simulation.world.colliders[unsupported.simulation.floor].is_enabled());
    }
    #[test]
    fn pause_and_visibility_cancel_active_and_future_grabs_without_advancing_bodies() {
        for hidden in [false, true] {
            let mut r = runtime();
            submit(&mut r, 1, 0, begin(41));
            r.advance(1).expect("start");
            submit(&mut r, 2, 3, begin(42));
            if hidden {
                r.set_visible(false).expect("hide");
            } else {
                submit(&mut r, 3, 1, Action::Pause);
            }
            let frozen = r.advance(4).expect("suspend");
            assert_eq!(frozen.completed_ticks, 0);
            assert!(r.simulation.grab.is_none());
            assert_eq!(r.timeline.depth(), 0);
            assert!(
                frozen
                    .acknowledgements
                    .iter()
                    .any(|a| a.rejected == Some(Reject::Cancelled))
            );
            let again = r.advance(4).expect("still suspended");
            assert_eq!(
                serde_json::to_value(&frozen.snapshot.segments).expect("poses"),
                serde_json::to_value(&again.snapshot.segments).expect("poses")
            );
            assert_eq!(
                frozen.snapshot.simulation_time_s,
                again.snapshot.simulation_time_s
            );
            assert!(lh_contracts::accepts_observation(
                frozen.snapshot.stamp,
                again.snapshot.stamp
            ));
            if hidden {
                r.set_visible(true).expect("show");
            } else {
                submit(&mut r, 4, 1, Action::Resume);
            }
            r.advance(0).expect("resume boundary");
            submit(&mut r, 5, 1, begin(42));
            assert_eq!(
                r.advance(0).expect("old press").acknowledgements[0].rejected,
                Some(Reject::Invalid)
            );
            submit(&mut r, 6, 1, begin(43));
            assert!(
                r.advance(1).expect("fresh press").acknowledgements[0]
                    .rejected
                    .is_none()
            );
        }
    }
    #[test]
    fn reset_isolates_queued_commands_and_stale_generations() {
        let mut r = runtime();
        let before = r.advance(1).expect("first").snapshot;
        submit(&mut r, 1, 3, Action::SetFloorEnabled { enabled: false });
        submit(&mut r, 2, 1, Action::Reset);
        let reset = r.advance(0).expect("reset");
        assert_eq!(reset.snapshot.stamp.generation, 2);
        assert_eq!(reset.snapshot.simulation_time_s, 0.0);
        assert_eq!(r.timeline.depth(), 0);
        assert!(
            reset
                .acknowledgements
                .iter()
                .any(|a| a.rejected == Some(Reject::Generation))
        );
        assert_eq!(
            r.enqueue(Command {
                stamp: Stamp::new(1, 3, 0),
                action: Action::Pause
            }),
            Err(Reject::Generation)
        );
        assert_eq!(
            r.enqueue(Command {
                stamp: Stamp::new(1, 4, 99),
                action: Action::Pause
            }),
            Err(Reject::Generation)
        );
        assert!(!lh_contracts::accepts_observation(
            reset.snapshot.stamp,
            before.stamp
        ));
        assert!(r.simulation.world.colliders[r.simulation.floor].is_enabled());
    }
    #[test]
    fn floor_disable_clears_support_at_the_boundary_and_preserves_body_identity() {
        let mut r = runtime();
        r.advance(4).expect("ground");
        let handles = r.simulation.bodies.clone();
        submit(&mut r, 1, 4, Action::SetFloorEnabled { enabled: false });
        let cleared = r.advance(0).expect("disable");
        assert!(cleared.snapshot.contacts.is_empty());
        let com = r.simulation.center_of_mass();
        let falling = r.advance(4).expect("free fall");
        assert!(falling.snapshot.contacts.is_empty());
        assert!(r.simulation.center_of_mass().y < com.y);
        assert_eq!(handles, r.simulation.bodies);
    }
    #[test]
    fn observed_fall_cancels_future_input_and_reset_opens_a_fresh_trial() {
        let mut r = runtime();
        submit(&mut r, 1, 0, Action::SetFloorEnabled { enabled: false });
        submit(&mut r, 2, 20, begin(41));
        let mut acknowledgements = Vec::new();
        for _ in 0..3 {
            acknowledgements.extend(r.advance(4).unwrap().acknowledgements);
        }
        assert_eq!(r.simulation.motion, lh_contracts::Motion::Falling);
        assert_eq!(r.simulation.counters.falls, 1);
        assert_eq!(r.timeline.depth(), 0);
        assert!(acknowledgements.iter().any(|ack| ack.stamp.sequence == 2
            && ack.rejected == Some(Reject::Cancelled)
            && ack.applied_tick.is_none()));
        assert_eq!(
            r.enqueue(Command {
                stamp: Stamp::new(1, 3, 12),
                action: begin(42)
            }),
            Err(Reject::Cancelled)
        );
        submit(&mut r, 4, 12, Action::Reset);
        let reset = r.advance(0).unwrap();
        assert_eq!(reset.snapshot.stamp.generation, 2);
        assert_eq!(reset.snapshot.motion, lh_contracts::Motion::Upright);
        assert_eq!(reset.snapshot.counters.falls, 0);
        submit(&mut r, 5, 0, begin(42));
        assert!(
            r.advance(0)
                .unwrap()
                .acknowledgements
                .iter()
                .all(|ack| ack.rejected.is_none())
        );
    }
    #[test]
    fn preaccepted_future_commands_apply_once_at_their_requested_tick() {
        let mut r = runtime();
        submit(&mut r, 1, 2, begin(41));
        submit(&mut r, 2, 0, Action::SetFloorEnabled { enabled: false });
        let result = r.advance(3).expect("scheduled advances");
        assert_eq!(result.acknowledgements.len(), 2);
        assert_eq!(result.acknowledgements[0].applied_tick, Some(0));
        assert_eq!(result.acknowledgements[1].applied_tick, Some(2));
        assert!(result.acknowledgements.iter().all(|a| a.rejected.is_none()));
        assert!(r.simulation.grab.is_some());
        assert_eq!(result.snapshot.acknowledged_sequence, 2);
        assert!(r.advance(0).expect("no replay").acknowledgements.is_empty());
    }
    #[test]
    fn invalid_batches_teardown_and_unported_actions_cannot_claim_execution() {
        let mut r = runtime();
        assert!(matches!(r.advance(5), Err(Reject::Invalid)));
        assert_eq!(r.simulation.tick, 0);
        submit(&mut r, 1, 0, Action::Strike);
        let reply = r.advance(0).expect("acknowledge");
        assert_eq!(
            reply.acknowledgements[0].rejected,
            Some(Reject::Unsupported)
        );
        assert_eq!(reply.acknowledgements[0].applied_tick, None);
        assert_eq!(reply.snapshot.counters.strikes, 0);
        r.shutdown();
        assert_eq!(r.enqueue_batch(Vec::new()), Err(Reject::Invalid));
        assert!(matches!(r.advance(0), Err(Reject::Invalid)));
        assert_eq!(
            r.enqueue(Command {
                stamp: Stamp::new(1, 2, 0),
                action: Action::Resume
            }),
            Err(Reject::Invalid)
        );
    }
    #[test]
    fn rejected_command_packet_cannot_leave_intent_or_consume_a_press() {
        let mut r = runtime();
        let commands = vec![
            Command {
                stamp: Stamp::new(1, 1, 0),
                action: begin(99),
            },
            Command {
                stamp: Stamp::new(0, 2, 0),
                action: Action::Pause,
            },
        ];
        assert_eq!(r.enqueue_batch(commands), Err(Reject::Generation));
        assert_eq!(r.timeline.depth(), 0);
        assert_eq!(r.highest_press, 0);
        submit(&mut r, 1, 0, begin(1));
        let reply = r.advance(0).expect("fresh packet");
        assert!(reply.acknowledgements[0].rejected.is_none());
        assert!(!reply.paused);
    }
}
