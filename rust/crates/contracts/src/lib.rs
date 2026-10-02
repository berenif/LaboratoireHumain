//! Tick-indexed, generation-isolated commands and immutable observations.
use lh_model::{Pose, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

pub const SCHEMA_VERSION: u16 = 4;
pub const TICK_HZ: u32 = 60;
pub const SUBSTEPS: u32 = 4;
pub const SUBSTEP_S: f32 = 1.0 / 240.0;
pub const MAX_QUEUED_COMMANDS: usize = 128;
pub const MAX_FUTURE_TICKS: u64 = 120;
pub const MAX_ADVANCE_TICKS: u32 = 4;
pub const MAX_OUTSTANDING_ADVANCES: u32 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Stamp {
    pub schema: u16,
    pub generation: u64,
    pub sequence: u64,
    pub tick: u64,
}
impl Stamp {
    pub fn new(generation: u64, sequence: u64, tick: u64) -> Self {
        Self {
            schema: SCHEMA_VERSION,
            generation,
            sequence,
            tick,
        }
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum Mode {
    Protocol,
    Playground,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum Station {
    Flat,
    Slope,
    Rubble,
    Beam,
    Stones,
    Wobble,
    Hurdles,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum Difficulty {
    Gentle,
    Challenging,
    Extreme,
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub struct PlaygroundSettings {
    pub station: Station,
    pub difficulty: Difficulty,
}
impl Default for PlaygroundSettings {
    fn default() -> Self {
        Self {
            station: Station::Flat,
            difficulty: Difficulty::Challenging,
        }
    }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvironmentSnapshot {
    pub settings: PlaygroundSettings,
    /// Actual integrated environment bodies, in canonical course order.
    pub pieces: Vec<Pose>,
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum StrikePhase {
    Idle,
    Positioning,
    Striking,
    Retracting,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct StrikerSnapshot {
    pub phase: StrikePhase,
    pub body: Pose,
    pub impact_id: u32,
    pub available: bool,
    pub last_impact_impulse_ns: f32,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Action {
    Initialize {
        mode: Mode,
        heading: f32,
    },
    GrabBegin {
        press: u64,
        segment: u8,
        local_anchor: Vec3,
        target: Vec3,
    },
    GrabMove {
        press: u64,
        target: Vec3,
    },
    GrabEnd {
        press: u64,
    },
    CancelGrab,
    Strike,
    Pause,
    Resume,
    Reset,
    SelectStation {
        station: Station,
    },
    SetDifficulty {
        difficulty: Difficulty,
    },
    SetFloorEnabled {
        enabled: bool,
    },
}
impl Action {
    pub fn valid(&self) -> bool {
        match self {
            Self::Initialize { heading, .. } => heading.is_finite(),
            Self::GrabBegin {
                segment,
                local_anchor,
                target,
                ..
            } => *segment < 25 && local_anchor.finite() && target.finite(),
            Self::GrabMove { target, .. } => target.finite(),
            _ => true,
        }
    }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Command {
    pub stamp: Stamp,
    pub action: Action,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Reject {
    Schema,
    Generation,
    Duplicate,
    OutOfOrder,
    Late,
    TooFarAhead,
    QueueFull,
    Invalid,
    Cancelled,
    Unsupported,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Acknowledgement {
    pub stamp: Stamp,
    pub applied_tick: Option<u64>,
    pub rejected: Option<Reject>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum Motion {
    Upright,
    Reacting,
    Falling,
    Fallen,
    Recovering,
    Halted,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Contact {
    pub segment: u8,
    pub normal_load_n: f32,
    pub persistence_s: f32,
}
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Diagnostics {
    /// None until the actual runtime path supplies a timing measurement.
    pub controller_ms: Option<f64>,
    pub update_ms: Option<f64>,
    pub dropped_time_s: f64,
    pub queue_depth: u32,
    /// Sum of absolute bounded generalized native requests. Not delivered torque.
    pub requested_motor_effort_nm: f32,
    pub delivered_motor_impulse_nm_s: Option<f32>,
    pub direct_pelvis_force_n: f32,
    pub direct_pelvis_torque_nm: f32,
    pub max_anchor_separation_m: f32,
    pub max_floor_penetration_m: f32,
    pub max_self_penetration_m: f32,
    pub max_limit_error_rad: f32,
}
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Counters {
    pub strikes: u32,
    pub falls: u32,
    pub recoveries: u32,
    pub steps: u32,
    pub upright_ticks: u64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Snapshot {
    pub stamp: Stamp,
    pub simulation_time_s: f64,
    /// Aggregate contacts describe the last fully completed observation tick.
    /// A halted partial tick may have newer segment state than this timestamp.
    pub contact_time_s: f64,
    pub discontinuity: bool,
    pub segments: Vec<Pose>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub environment: Option<EnvironmentSnapshot>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub striker: Option<StrikerSnapshot>,
    pub motion: Motion,
    pub contacts: Vec<Contact>,
    pub counters: Counters,
    pub diagnostics: Diagnostics,
    pub acknowledged_sequence: u64,
}

/// Commands apply immediately before substep zero of their requested tick.
/// A received high sequence closes all lower sequences. Duplicates never replay.
/// Paused/hidden time has no simulation tick; lifecycle commands use the current
/// boundary. Resets increment generation, clear pending intent, and restart tick 0.
#[derive(Debug, Clone)]
pub struct Timeline {
    pub generation: u64,
    pub tick: u64,
    last_received: u64,
    pub last_applied: u64,
    queue: BTreeMap<(u64, u64), Command>,
}
impl Timeline {
    pub fn new(generation: u64) -> Self {
        Self {
            generation,
            tick: 0,
            last_received: 0,
            last_applied: 0,
            queue: BTreeMap::new(),
        }
    }
    pub fn depth(&self) -> usize {
        self.queue.len()
    }
    pub fn enqueue(&mut self, c: Command) -> Result<(), Reject> {
        if c.stamp.schema != SCHEMA_VERSION {
            return Err(Reject::Schema);
        }
        if c.stamp.generation != self.generation {
            return Err(Reject::Generation);
        }
        if c.stamp.sequence == self.last_received {
            return Err(Reject::Duplicate);
        }
        if c.stamp.sequence < self.last_received {
            return Err(Reject::OutOfOrder);
        }
        if c.stamp.tick < self.tick {
            return Err(Reject::Late);
        }
        if c.stamp.tick > self.tick + MAX_FUTURE_TICKS {
            return Err(Reject::TooFarAhead);
        }
        if self.depth() >= MAX_QUEUED_COMMANDS {
            return Err(Reject::QueueFull);
        }
        if !c.action.valid() {
            return Err(Reject::Invalid);
        }
        self.last_received = c.stamp.sequence;
        self.queue.insert((c.stamp.tick, c.stamp.sequence), c);
        Ok(())
    }
    pub fn boundary(&mut self) -> Vec<Command> {
        let keys: Vec<_> = self
            .queue
            .range(..=(self.tick, u64::MAX))
            .map(|(k, _)| *k)
            .collect();
        keys.into_iter()
            .filter_map(|k| self.queue.remove(&k))
            .collect()
    }
    pub fn reset(&mut self) -> Result<(), Reject> {
        let generation = self.generation.checked_add(1).ok_or(Reject::Invalid)?;
        *self = Self::new(generation);
        Ok(())
    }
    /// Pending pointer intent must not resume after suspension or hiding.
    pub fn cancel_grabs(&mut self) -> Vec<Command> {
        let keys: Vec<_> = self
            .queue
            .iter()
            .filter(|(_, c)| {
                matches!(
                    c.action,
                    Action::GrabBegin { .. } | Action::GrabMove { .. } | Action::GrabEnd { .. }
                )
            })
            .map(|(key, _)| *key)
            .collect();
        keys.into_iter()
            .filter_map(|key| self.queue.remove(&key))
            .collect()
    }
    pub fn clear(&mut self) -> Vec<Command> {
        std::mem::take(&mut self.queue).into_values().collect()
    }
}

/// The main thread holds one detached-display history per generation. This guard
/// also applies to initialization completions and transfer-buffer replies.
pub fn accepts_observation(current: Stamp, incoming: Stamp) -> bool {
    incoming.schema == SCHEMA_VERSION
        && incoming.generation == current.generation
        && incoming.sequence > current.sequence
        && incoming.tick >= current.tick
}

#[cfg(test)]
mod tests {
    use super::*;
    fn c(g: u64, s: u64, t: u64) -> Command {
        Command {
            stamp: Stamp::new(g, s, t),
            action: Action::Strike,
        }
    }
    #[test]
    fn ordering_late_duplicates_and_reset_isolation() {
        let mut q = Timeline::new(1);
        q.enqueue(c(1, 1, 2)).expect("command");
        q.enqueue(c(1, 2, 0)).expect("command");
        assert_eq!(q.enqueue(c(1, 2, 0)), Err(Reject::Duplicate));
        assert_eq!(q.enqueue(c(1, 1, 0)), Err(Reject::OutOfOrder));
        assert_eq!(q.boundary()[0].stamp.sequence, 2);
        q.tick = 1;
        assert_eq!(q.enqueue(c(1, 3, 0)), Err(Reject::Late));
        q.reset().expect("generation");
        assert_eq!(q.depth(), 0);
        assert_eq!(q.enqueue(c(1, 4, 0)), Err(Reject::Generation));
        assert!(!accepts_observation(
            Stamp::new(2, 0, 0),
            Stamp::new(1, 999, 0)
        ));
    }
    #[test]
    fn queue_is_bounded_and_no_nan_inputs() {
        let mut q = Timeline::new(1);
        for s in 1..=MAX_QUEUED_COMMANDS as u64 {
            q.enqueue(c(1, s, 100)).expect("bounded");
        }
        assert_eq!(q.enqueue(c(1, 129, 100)), Err(Reject::QueueFull));
        assert_eq!(
            Timeline::new(1).enqueue(Command {
                stamp: Stamp::new(1, 1, 0),
                action: Action::Initialize {
                    mode: Mode::Protocol,
                    heading: f32::NAN
                }
            }),
            Err(Reject::Invalid)
        );
    }
}
