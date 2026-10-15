//! Pure command references from the LAST INSTALLED q/dq, never measured angles.
//!
//! No world, FK, support query, installer or integration is used here. A local
//! readiness receipt is supplied by the policy; accepting its clock is not a
//! physical admission. Index zero of a Bridge is historical evidence, not a new
//! command. All subsequent dq values are the exact f32 differences of emitted q.
use crate::recovery::trajectory_program::{
    Command, CompiledProgram, Coordinates, validate_command,
};
use lh_model::Model;
use serde::{Deserialize, Serialize};

use super::continuation::Timing;

const HZ: u32 = 240;
const COMPLETE_STEPS: u64 = 25 * HZ as u64;
const SOURCE_CUT: u32 = 432;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub(crate) struct ObservedCommandEntry {
    pub previous_q: Coordinates,
    pub previous_dq: Coordinates,
    /// Next command clock. The archived command was installed at this clock - 1.
    pub native_substep: u64,
}

impl ObservedCommandEntry {
    pub fn command(&self) -> Command {
        Command {
            positions: self.previous_q,
            velocities: self.previous_dq,
        }
    }
}

/// Current policy evidence, not a boolean enabling physics. The caller must
/// obtain this from its measured heel/recenter policy, never from a schedule.
#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub(crate) struct LocalReadyReceipt {
    pub native_substep: u64,
    pub first_stable_substep: u64,
}

#[derive(Clone, Copy, Debug, Serialize)]
pub(crate) struct BridgeBounds {
    /// Necessary displacement/rate bound; it is not a feasibility claim.
    pub displacement_time_lower_bound_s: f64,
    /// Sufficient cubic Bernstein derivative bound, before grid rounding.
    pub certified_duration_floor_s: f64,
    pub duration_substeps: u32,
    pub maximum_continuous_rate: f64,
    pub maximum_emitted_rate: f32,
}

#[derive(Clone)]
pub(crate) struct Bridge {
    commands: Vec<Command>,
    pub bounds: BridgeBounds,
}

/// Cubic coefficients in normalized time u, preserving both endpoint tangents.
fn cubic(q0: f64, v0: f64, q1: f64, duration: f64) -> [f64; 4] {
    let d = q1 - q0;
    [
        q0,
        duration * v0,
        3. * d - 2. * duration * v0,
        -2. * d + duration * v0,
    ]
}

fn position(c: [f64; 4], u: f64) -> f64 {
    ((c[3] * u + c[2]) * u + c[1]) * u + c[0]
}

fn speed(c: [f64; 4], u: f64, duration: f64) -> f64 {
    ((3. * c[3] * u + 2. * c[2]) * u + c[1]) / duration
}

fn stationary_points(c: [f64; 4]) -> Vec<f64> {
    let (a, b, d) = (3. * c[3], 2. * c[2], c[1]);
    if a == 0. {
        return if b == 0. { vec![] } else { vec![-d / b] };
    }
    let discriminant = b * b - 4. * a * d;
    if discriminant < 0. {
        return vec![];
    }
    // Stable quadratic roots avoid cancellation when one stationary point is
    // near an endpoint. These are evaluations, not a duration/variant search.
    let q = -0.5 * (b + discriminant.sqrt().copysign(b));
    if q == 0. {
        vec![-b / (2. * a)]
    } else {
        vec![q / a, d / q]
    }
}

fn emitted(
    model: &Model,
    positions: Coordinates,
    previous: &Coordinates,
) -> Result<Command, String> {
    let mut command = Command {
        positions,
        velocities: [[0.; 3]; 25],
    };
    for i in 0..25 {
        for k in 0..3 {
            let difference = command.positions[i][k] - previous[i][k];
            if !difference.is_finite() || difference.abs() > lh_contracts::SUBSTEP_S {
                return Err(format!(
                    "Raccord emitted slew at segment {i} coordinate {k}"
                ));
            }
            command.velocities[i][k] = difference / lh_contracts::SUBSTEP_S;
        }
    }
    validate_command(model, &command)?;
    Ok(command)
}

fn command_evidence(commands: &[Command]) -> serde_json::Value {
    serde_json::json!(
        commands
            .iter()
            .map(|command| serde_json::json!({
                "positions": command.positions, "velocities": command.velocities,
            }))
            .collect::<Vec<_>>()
    )
}

impl Bridge {
    /// One cubic toward a stationary command. The one-second floor and the
    /// analytic rate bound determine ONE duration; failed anatomy/rounding is
    /// refused, never retried at other durations. Source cut 432 is stationary.
    pub fn prepare(
        model: &Model,
        entry: &ObservedCommandEntry,
        target: &Command,
    ) -> Result<Self, String> {
        if model.segments.len() != 25 || entry.native_substep == 0 {
            return Err("Observed raccord needs 25 segments and a previous command clock".into());
        }
        validate_command(model, &entry.command())?;
        validate_command(model, target)?;
        if target.velocities.iter().flatten().any(|v| *v != 0.) {
            return Err("Raccord bridge requires a stationary target command".into());
        }
        let mut displacement_time: f64 = 0.;
        let mut duration_floor: f64 = 0.;
        for i in 0..25 {
            for k in 0..3 {
                let delta = f64::from(target.positions[i][k]) - f64::from(entry.previous_q[i][k]);
                let v0 = f64::from(entry.previous_dq[i][k]);
                displacement_time = displacement_time.max(delta.abs());
                if delta != 0. {
                    // dq is a quadratic Bernstein polynomial with controls
                    // [v0, 3*delta/T-v0, 0]. Keeping its middle control in
                    // [-1,1] bounds the whole derivative without a sweep.
                    let denominator = 1. + delta.signum() * v0;
                    if denominator <= 0. {
                        return Err(
                            "No finite duration under the fixed Bernstein rate certificate".into(),
                        );
                    }
                    duration_floor = duration_floor.max(3. * delta.abs() / denominator);
                }
            }
        }
        let dt = f64::from(lh_contracts::SUBSTEP_S);
        let steps = (duration_floor / dt).ceil().max(f64::from(HZ));
        if !steps.is_finite() || steps > COMPLETE_STEPS as f64 {
            return Err("Raccord bridge alone exceeds the 25-second command bound".into());
        }
        let steps = steps as u32;
        let duration = f64::from(steps) * dt;
        let mut coefficients = [[[0.; 4]; 3]; 25];
        let mut maximum_continuous_rate: f64 = 0.;
        for (i, segment) in model.segments.iter().enumerate() {
            for k in 0..3 {
                let c = cubic(
                    f64::from(entry.previous_q[i][k]),
                    f64::from(entry.previous_dq[i][k]),
                    f64::from(target.positions[i][k]),
                    duration,
                );
                coefficients[i][k] = c;
                let mut speed_times = vec![0., 1.];
                if c[3] != 0. {
                    speed_times.push(-c[2] / (3. * c[3]));
                }
                for u in speed_times.into_iter().filter(|u| (0.0..=1.0).contains(u)) {
                    let rate = speed(c, u, duration).abs();
                    if !rate.is_finite() || rate > 1. {
                        return Err(format!("Raccord continuous rate at {i}/{k}"));
                    }
                    maximum_continuous_rate = maximum_continuous_rate.max(rate);
                }
                if let Some(axis) = segment
                    .joint_profile
                    .as_ref()
                    .and_then(|p| p.axes.iter().find(|a| a.coordinate == ['x', 'y', 'z'][k]))
                {
                    for u in stationary_points(c)
                        .into_iter()
                        .filter(|u| *u > 0. && *u < 1.)
                    {
                        let q = position(c, u);
                        if !q.is_finite()
                            || q < f64::from(axis.min_radians)
                            || q > f64::from(axis.max_radians)
                        {
                            return Err(format!(
                                "Raccord continuous anatomical overshoot at {i}/{k}"
                            ));
                        }
                    }
                }
            }
        }
        let mut commands = vec![entry.command()];
        let mut maximum_emitted_rate: f32 = 0.;
        for step in 1..=steps {
            let u = f64::from(step) / f64::from(steps);
            let q = if step == steps {
                target.positions
            } else {
                std::array::from_fn(|i| {
                    std::array::from_fn(|k| position(coefficients[i][k], u) as f32)
                })
            };
            let command = emitted(model, q, &commands.last().unwrap().positions)?;
            maximum_emitted_rate = command
                .velocities
                .iter()
                .flatten()
                .fold(maximum_emitted_rate, |m, v| m.max(v.abs()));
            commands.push(command);
        }
        Ok(Self {
            commands,
            bounds: BridgeBounds {
                displacement_time_lower_bound_s: displacement_time,
                certified_duration_floor_s: duration_floor,
                duration_substeps: steps,
                maximum_continuous_rate,
                maximum_emitted_rate,
            },
        })
    }

    pub fn duration_substeps(&self) -> u32 {
        self.bounds.duration_substeps
    }

    /// Zero returns the archived command unchanged, including nonzero dq.
    pub fn command(&self, index: u32) -> Result<Command, String> {
        self.commands
            .get(index as usize)
            .cloned()
            .ok_or_else(|| "Raccord bridge clock outside finite reference".into())
    }

    pub fn evidence(&self) -> serde_json::Value {
        serde_json::json!({"scope":"pure finite command bridge; no physical admission",
            "bounds":self.bounds,"commandsIncludingHistoricalEntry":command_evidence(&self.commands)})
    }
}

#[derive(Clone, Copy, Debug, Serialize)]
pub(crate) struct RaccordTiming {
    pub start_native_substep: u64,
    pub bridge_substeps: u32,
    pub source_cut_substep: u32,
    pub standing_command_substep: u64,
    /// The next clock after the last command; no implicit infinite tail.
    pub end_native_substep_exclusive: u64,
    pub absolute_deadline_native_substep: u64,
}

#[derive(Clone)]
pub(crate) struct PreparedRaccord {
    commands: Vec<Command>,
    pub timing: RaccordTiming,
    pub bridge_bounds: BridgeBounds,
    pub ready_receipt: LocalReadyReceipt,
}

impl PreparedRaccord {
    pub fn prepare(
        model: &Model,
        entry: &ObservedCommandEntry,
        source: &CompiledProgram,
        source_timing: Timing,
        ready: Option<LocalReadyReceipt>,
        absolute_origin_substep: u64,
    ) -> Result<Self, String> {
        let ready = ready.ok_or("Raccord needs current local policy readiness evidence")?;
        if ready.native_substep != entry.native_substep
            || ready.first_stable_substep > ready.native_substep
        {
            return Err("Raccord readiness is stale or inconsistent with the command entry".into());
        }
        if absolute_origin_substep > entry.native_substep {
            return Err("Raccord absolute deadline origin is after the observed entry".into());
        }
        if source_timing.recentered_hold_completed != SOURCE_CUT
            || source_timing.standing_reference <= SOURCE_CUT
            || source_timing.hold_completed != source.duration_substeps()
            || source_timing
                .hold_completed
                .checked_sub(source_timing.standing_reference)
                .is_none_or(|v| v < HZ)
            || source.source().segment_ids
                != model
                    .segments
                    .iter()
                    .map(|s| s.id.clone())
                    .collect::<Vec<_>>()
        {
            return Err(
                "Raccord source must contain the fixed hold cut, standing and an extra second"
                    .into(),
            );
        }
        let bridge = Bridge::prepare(model, entry, &source.command(SOURCE_CUT))?;
        let count = u64::from(bridge.duration_substeps())
            + u64::from(source_timing.hold_completed - SOURCE_CUT);
        let end = entry
            .native_substep
            .checked_add(count)
            .ok_or("Raccord clock overflow")?;
        let deadline = absolute_origin_substep
            .checked_add(COMPLETE_STEPS)
            .ok_or("Raccord deadline overflow")?;
        if end > deadline {
            return Err(
                "Complete raccord and standing hold exceed the absolute 25-second deadline".into(),
            );
        }
        let timing = RaccordTiming {
            start_native_substep: entry.native_substep,
            bridge_substeps: bridge.duration_substeps(),
            source_cut_substep: SOURCE_CUT,
            standing_command_substep: entry.native_substep
                + u64::from(bridge.duration_substeps())
                + u64::from(source_timing.standing_reference - SOURCE_CUT)
                - 1,
            end_native_substep_exclusive: end,
            absolute_deadline_native_substep: deadline,
        };
        let bridge_bounds = bridge.bounds;
        let mut commands = bridge.commands;
        let standing = source.command(source_timing.standing_reference).positions;
        for step in SOURCE_CUT + 1..=source_timing.hold_completed {
            let q = source.command(step).positions;
            let command = emitted(model, q, &commands.last().unwrap().positions)?;
            if step > source_timing.standing_reference
                && (command.positions != standing
                    || command.velocities.iter().flatten().any(|v| *v != 0.))
            {
                return Err(
                    "Raccord source does not preserve the full additional standing second".into(),
                );
            }
            commands.push(command);
        }
        Ok(Self {
            commands,
            timing,
            bridge_bounds,
            ready_receipt: ready,
        })
    }

    pub fn entry_command(&self) -> &Command {
        &self.commands[0]
    }

    pub fn command(&self, native_substep: u64) -> Result<Command, String> {
        if native_substep < self.timing.start_native_substep
            || native_substep >= self.timing.end_native_substep_exclusive
        {
            return Err("Raccord command clock outside the prepared finite reference".into());
        }
        Ok(self.commands[(native_substep - self.timing.start_native_substep + 1) as usize].clone())
    }

    pub fn evidence(&self) -> serde_json::Value {
        serde_json::json!({"scope":"pure finite continuation reference; no physical admission",
            "timing":self.timing,"bridgeBounds":self.bridge_bounds,"readyReceipt":self.ready_receipt,
            "commandsIncludingHistoricalEntry":command_evidence(&self.commands)})
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::recovery::trajectory_program::Program;

    fn input() -> (Model, ObservedCommandEntry, CompiledProgram, Timing) {
        // Serde metadata only: no Model constructor, FK, Simulation or step.
        let model: Model = serde_json::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../model/data/canonical-v1.json"
        )))
        .unwrap();
        let fixture: serde_json::Value = serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"),
            "/../../../docs/evidence/coordination-transfer-candidate-2026-10-13/observed-entry-step36.json"))).unwrap();
        let entry = serde_json::from_value(fixture["commandEntry"].clone()).unwrap();
        let source: Program = serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"),
            "/../../../docs/evidence/coordination-prototype-2026-10-12/continuation-candidate-program.json"))).unwrap();
        let timing = Timing {
            heels_acquired: 168,
            recentered_hold_started: 300,
            recentered_hold_completed: 432,
            standing_reference: 1680,
            hold_completed: 1920,
        };
        let source = source.compile(&model).unwrap();
        (model, entry, source, timing)
    }

    #[test]
    fn archived_entry_velocity_is_preserved_and_emitted_derivatives_are_exact() {
        let (model, entry, source, _) = input();
        assert!(entry.previous_dq.iter().flatten().any(|v| *v != 0.));
        let bridge = Bridge::prepare(&model, &entry, &source.command(SOURCE_CUT)).unwrap();
        assert_eq!(bridge.command(0).unwrap(), entry.command());
        assert_eq!(bridge.duration_substeps(), HZ);
        let mut previous = entry.previous_q;
        for step in 1..=bridge.duration_substeps() {
            let command = bridge.command(step).unwrap();
            for i in 0..25 {
                for k in 0..3 {
                    assert_eq!(
                        command.velocities[i][k].to_bits(),
                        ((command.positions[i][k] - previous[i][k]) / lh_contracts::SUBSTEP_S)
                            .to_bits()
                    );
                }
            }
            previous = command.positions;
        }
        assert_eq!(previous, source.command(SOURCE_CUT).positions);
        assert!(bridge.command(HZ + 1).is_err());
    }

    #[test]
    fn full_reference_requires_current_receipt_and_keeps_the_absolute_deadline() {
        let (model, entry, source, timing) = input();
        // Synthetic receipt tests the interface, not the physical A36 state.
        let ready = LocalReadyReceipt {
            native_substep: entry.native_substep,
            first_stable_substep: 0,
        };
        assert!(PreparedRaccord::prepare(&model, &entry, &source, timing, None, 0).is_err());
        let stale = LocalReadyReceipt {
            native_substep: entry.native_substep - 1,
            ..ready
        };
        assert!(PreparedRaccord::prepare(&model, &entry, &source, timing, Some(stale), 0).is_err());
        let prepared =
            PreparedRaccord::prepare(&model, &entry, &source, timing, Some(ready), 0).unwrap();
        assert_eq!(*prepared.entry_command(), entry.command());
        assert_eq!(
            prepared.timing.end_native_substep_exclusive,
            36 + 240 + 1920 - 432
        );
        assert_eq!(prepared.timing.absolute_deadline_native_substep, 6000);
        for step in prepared.timing.standing_command_substep + 1
            ..prepared.timing.end_native_substep_exclusive
        {
            assert!(
                prepared
                    .command(step)
                    .unwrap()
                    .velocities
                    .iter()
                    .flatten()
                    .all(|v| *v == 0.)
            );
        }
        assert!(
            prepared
                .command(prepared.timing.end_native_substep_exclusive)
                .is_err()
        );
        let mut late = entry.clone();
        late.native_substep = 5900;
        let late_ready = LocalReadyReceipt {
            native_substep: 5900,
            first_stable_substep: 5800,
        };
        assert!(
            PreparedRaccord::prepare(&model, &late, &source, timing, Some(late_ready), 0).is_err()
        );
    }

    #[test]
    fn invalid_or_overshooting_boundary_commands_are_refused_without_retiming() {
        let (model, mut entry, source, _) = input();
        entry.previous_dq[15][0] = f32::NAN;
        assert!(Bridge::prepare(&model, &entry, &source.command(SOURCE_CUT)).is_err());
        entry.previous_dq[15][0] = 1.;
        let axis = model.segments[15]
            .joint_profile
            .as_ref()
            .unwrap()
            .axes
            .iter()
            .find(|a| a.coordinate == 'x')
            .unwrap();
        entry.previous_q[15][0] = axis.max_radians;
        let target = entry.command();
        let target = Command {
            velocities: [[0.; 3]; 25],
            ..target
        };
        assert!(Bridge::prepare(&model, &entry, &target).is_err());
    }
}
