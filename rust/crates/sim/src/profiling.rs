//! Optional, bounded phase observation. The clock and recorder are supplied by
//! the caller; neither can change simulation state or its fixed physical clock.
use serde::Serialize;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "camelCase")]
pub enum Phase {
    Tick,
    Controller,
    Physics,
    Contacts,
    Integrity,
    Observer,
}
impl Phase {
    fn index(self) -> Option<usize> {
        match self {
            Self::Tick => None,
            Self::Controller => Some(0),
            Self::Physics => Some(1),
            Self::Contacts => Some(2),
            Self::Integrity => Some(3),
            Self::Observer => Some(4),
        }
    }
}
#[derive(Clone, Copy, Debug)]
pub struct Event {
    pub generation: u64,
    pub tick: u64,
    pub substep: u32,
    pub phase: Phase,
    pub begin: bool,
}
#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
pub struct TickTiming {
    pub generation: u64,
    pub tick_start: u64,
    pub tick_end: u64,
    pub completed_substeps: u32,
    pub complete: bool,
    pub elapsed_ms: f64,
    pub completed_at_ms: f64,
    pub controller_ms: f64,
    pub physics_ms: f64,
    pub contacts_ms: f64,
    pub integrity_ms: f64,
    pub observer_ms: f64,
    pub controller_substeps_ms: [Option<f64>; 4],
}
struct Working {
    timing: TickTiming,
    started: f64,
    opened: [Option<f64>; 5],
    totals: [f64; 5],
    has_integration_stamp: bool,
}
pub struct Recorder<C> {
    clock: C,
    ticks: Vec<TickTiming>,
    current: Option<Working>,
    valid: bool,
    last_clock: f64,
}
impl<C: FnMut() -> f64> Recorder<C> {
    pub fn new(clock: C) -> Self {
        Self {
            clock,
            ticks: Vec::with_capacity(4),
            current: None,
            valid: true,
            last_clock: f64::NEG_INFINITY,
        }
    }
    pub fn record(&mut self, event: Event) {
        let now = (self.clock)();
        if !now.is_finite() || now < self.last_clock {
            self.valid = false;
            return;
        }
        self.last_clock = now;
        if event.phase == Phase::Tick {
            if event.begin {
                if self.current.is_some() || self.ticks.len() >= 4 {
                    self.valid = false;
                    return;
                }
                self.current = Some(Working {
                    timing: TickTiming {
                        generation: event.generation,
                        tick_start: event.tick,
                        tick_end: event.tick,
                        completed_substeps: 0,
                        complete: false,
                        elapsed_ms: 0.0,
                        completed_at_ms: 0.0,
                        controller_ms: 0.0,
                        physics_ms: 0.0,
                        contacts_ms: 0.0,
                        integrity_ms: 0.0,
                        observer_ms: 0.0,
                        controller_substeps_ms: [None; 4],
                    },
                    started: now,
                    opened: [None; 5],
                    totals: [0.0; 5],
                    has_integration_stamp: false,
                });
            } else if let Some(mut current) = self.current.take() {
                self.valid &= current.opened.iter().all(Option::is_none);
                current.timing.elapsed_ms = now - current.started;
                current.timing.completed_at_ms = now;
                current.timing.tick_end = event.tick;
                current.timing.complete = current.has_integration_stamp
                    && event.generation == current.timing.generation
                    && event.tick == current.timing.tick_start + 1
                    && current.timing.completed_substeps == 4;
                let [controller, physics, contacts, integrity, observer] = current.totals;
                current.timing.controller_ms = controller;
                current.timing.physics_ms = physics;
                current.timing.contacts_ms = contacts;
                current.timing.integrity_ms = integrity;
                current.timing.observer_ms = observer;
                self.ticks.push(current.timing);
            } else {
                self.valid = false;
            }
            return;
        }
        let Some(current) = &mut self.current else {
            self.valid = false;
            return;
        };
        let index = event.phase.index().expect("non-tick phase");
        if event.substep >= 4 {
            self.valid = false;
            return;
        }
        if event.begin {
            if event.phase == Phase::Controller && !current.has_integration_stamp {
                current.timing.generation = event.generation;
                current.timing.tick_start = event.tick;
                current.has_integration_stamp = true;
            }
            self.valid &= current.opened[index].is_none();
            current.opened[index] = Some(now);
        } else if let Some(started) = current.opened[index].take() {
            let elapsed = now - started;
            current.totals[index] += elapsed;
            if event.phase == Phase::Controller {
                current.timing.controller_substeps_ms[event.substep as usize] = Some(elapsed);
            }
            if event.phase == Phase::Physics {
                current.timing.completed_substeps += 1;
            }
        } else {
            self.valid = false;
        }
    }
    pub fn finish(self) -> (Vec<TickTiming>, bool) {
        (self.ticks, self.valid && self.current.is_none())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Profile, runtime::Runtime};
    use lh_contracts::{Action, Command, Mode, Stamp};
    #[test]
    fn every_phase_is_bounded_and_instrumentation_preserves_the_exact_native_reply() {
        let mut plain = Runtime::new(Mode::Playground, 0.0, Profile::default()).unwrap();
        let mut measured = Runtime::new(Mode::Playground, 0.0, Profile::default()).unwrap();
        let mut now = 0.0;
        let mut recorder = Recorder::new(|| {
            now += 0.25;
            now
        });
        let expected = plain.advance(4).unwrap();
        let actual = measured
            .advance_traced(4, |_, event| recorder.record(event))
            .unwrap();
        assert_eq!(
            serde_json::to_value(expected).unwrap(),
            serde_json::to_value(actual).unwrap()
        );
        let (ticks, valid) = recorder.finish();
        assert!(valid);
        assert_eq!(ticks.len(), 4);
        for (index, tick) in ticks.iter().enumerate() {
            assert_eq!(
                (tick.tick_start, tick.tick_end),
                (index as u64, index as u64 + 1)
            );
            assert!(tick.complete && tick.completed_substeps == 4);
            assert_eq!(tick.controller_substeps_ms, [Some(0.25); 4]);
            assert_eq!(tick.controller_ms, 1.0);
            assert_eq!(tick.physics_ms, 1.0);
            assert_eq!(tick.contacts_ms, 1.0);
            assert_eq!(tick.integrity_ms, 1.0);
            assert_eq!(tick.observer_ms, 1.0);
            assert_eq!(tick.elapsed_ms, 10.25);
        }
    }
    #[test]
    fn suspension_records_no_fictitious_completed_update_and_reset_uses_the_new_generation() {
        let mut runtime = Runtime::new(Mode::Protocol, 0.0, Profile::default()).unwrap();
        runtime
            .enqueue(Command {
                stamp: Stamp::new(1, 1, 0),
                action: Action::Pause,
            })
            .unwrap();
        let mut recorder = Recorder::new(|| 1.0);
        let paused = runtime
            .advance_traced(4, |_, event| recorder.record(event))
            .unwrap();
        assert_eq!(paused.completed_ticks, 0);
        let (ticks, valid) = recorder.finish();
        assert!(valid && ticks.len() == 1);
        assert!(!ticks[0].complete && ticks[0].completed_substeps == 0);
        runtime
            .enqueue_batch(vec![
                Command {
                    stamp: Stamp::new(1, 2, 0),
                    action: Action::Resume,
                },
                Command {
                    stamp: Stamp::new(1, 3, 0),
                    action: Action::Reset,
                },
            ])
            .unwrap();
        let mut recorder = Recorder::new(|| 2.0);
        let reset = runtime
            .advance_traced(1, |_, event| recorder.record(event))
            .unwrap();
        let (ticks, valid) = recorder.finish();
        assert!(valid && ticks[0].complete);
        assert_eq!(
            (ticks[0].generation, ticks[0].tick_start, ticks[0].tick_end),
            (2, 0, 1)
        );
        assert_eq!(reset.snapshot.stamp.generation, 2);
    }
    #[test]
    fn a_non_monotonic_clock_is_explicitly_invalid() {
        let mut clock = 3.0;
        let mut recorder = Recorder::new(|| {
            clock -= 1.0;
            clock
        });
        let event = Event {
            generation: 1,
            tick: 0,
            substep: 0,
            phase: Phase::Tick,
            begin: true,
        };
        recorder.record(event);
        recorder.record(Event {
            begin: false,
            ..event
        });
        assert!(!recorder.finish().1);
    }
}
