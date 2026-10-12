//! Pure common-rate owner. All 46 q/dq commands are compiled together.
//! This is a proposal generator; native preview admission remains mandatory.
use super::whole_body_tasks::{AXES, HardTask, Problem, Rates, Solution, Task};
use crate::recovery::trajectory_program::{Command, Coordinates, validate_command};
use lh_model::Model;

const TICK_STEPS: u64 = 4;
const MAP_MAX_AGE_STEPS: u64 = 24;

#[derive(Clone, Debug)]
pub(crate) struct Axis {
    pub segment: usize,
    pub coordinate: usize,
    pub minimum: f32,
    pub maximum: f32,
}

#[derive(Clone, Debug)]
pub(crate) struct Memory {
    pub previous: Coordinates,
    pub next_substep: u64,
    pub rates: Rates,
    pub mode_signature: u64,
}

#[derive(Clone, Debug)]
pub(crate) struct ResponseMap {
    pub identified_at: u64,
    pub mode_signature: u64,
    pub baseline: Vec<f64>,
    pub baseline_rates: Rates,
    pub rows: Vec<Rates>,
    /// Effective signed perturbations of TOTAL compiled rate, in rad/s.
    pub amplitudes: Vec<[f64; 2]>,
}

#[derive(Clone, Debug)]
pub(crate) struct OutputTask {
    pub output: usize,
    pub target: f64,
    pub weight: f64,
}
#[derive(Clone, Debug)]
pub(crate) struct OutputConstraint {
    pub name: String,
    pub output: usize,
    pub lower: f64,
    pub upper: f64,
}

#[derive(Clone, Debug)]
pub(crate) struct AxisSamples {
    pub negative: Option<(f64, Vec<f64>)>,
    pub positive: Option<(f64, Vec<f64>)>,
}

impl ResponseMap {
    /// Observations must come from separate, budgeted native copies executing
    /// this SAME command compiler and installer on each of their four steps.
    /// Numerical differentiation does not establish provenance or prediction quality.
    pub(crate) fn from_samples(
        at: u64,
        mode: u64,
        baseline: Vec<f64>,
        baseline_rates: Rates,
        samples: &[AxisSamples],
    ) -> Result<Self, String> {
        if samples.len() != AXES
            || baseline.is_empty()
            || baseline.len() > 256
            || baseline.iter().any(|v| !v.is_finite())
            || baseline_rates
                .iter()
                .any(|v| !v.is_finite() || v.abs() > 1.)
        {
            return Err("Invalid native response dimensions".into());
        }
        let mut rows = vec![[0.; AXES]; baseline.len()];
        let mut amplitudes = Vec::with_capacity(AXES);
        for (axis, s) in samples.iter().enumerate() {
            let mut pair = [(0., baseline.as_slice()); 2];
            for (side, sample) in [s.negative.as_ref(), s.positive.as_ref()]
                .into_iter()
                .enumerate()
            {
                if let Some((delta, values)) = sample {
                    if !delta.is_finite()
                        || *delta == 0.
                        || (baseline_rates[axis] + delta).abs() > 1.
                        || (side == 0 && *delta > 0.)
                        || (side == 1 && *delta < 0.)
                        || values.len() != baseline.len()
                        || values.iter().any(|v| !v.is_finite())
                    {
                        return Err("Invalid effective native perturbation".into());
                    }
                    pair[side] = (*delta, values);
                }
            }
            let width = pair[1].0 - pair[0].0;
            if !width.is_finite() || width < 1e-8 {
                return Err(format!(
                    "No identifiable native rate direction for axis {axis}"
                ));
            }
            for (i, row) in rows.iter_mut().enumerate() {
                row[axis] = (pair[1].1[i] - pair[0].1[i]) / width;
                if !row[axis].is_finite() {
                    return Err("Nonfinite response derivative".into());
                }
            }
            amplitudes.push([pair[0].0, pair[1].0]);
        }
        Ok(Self {
            identified_at: at,
            mode_signature: mode,
            baseline,
            baseline_rates,
            rows,
            amplitudes,
        })
    }

    fn problem(
        &self,
        soft: &[OutputTask],
        hard: &[OutputConstraint],
        lower: Rates,
        upper: Rates,
    ) -> Result<Problem, String> {
        let offset = |i: usize| -> Result<f64, String> {
            let row = self.rows.get(i).ok_or("Unknown native output task")?;
            Ok(self.baseline[i]
                - row
                    .iter()
                    .zip(self.baseline_rates)
                    .map(|(a, x)| a * x)
                    .sum::<f64>())
        };
        let mut p = Problem {
            lower,
            upper,
            hard: vec![],
            soft: vec![],
            regularization: 0.001,
        };
        for task in soft {
            let base = offset(task.output)?;
            p.soft.push(Task {
                row: self.rows[task.output],
                target: task.target - base,
                weight: task.weight,
            });
        }
        for task in hard {
            let base = offset(task.output)?;
            p.hard.push(HardTask {
                name: task.name.clone(),
                row: self.rows[task.output],
                lower: task.lower - base,
                upper: task.upper - base,
            });
        }
        p.validate()?;
        Ok(p)
    }
    pub(crate) fn require_current(&self, at: u64, mode: u64) -> Result<(), String> {
        let age = at
            .checked_sub(self.identified_at)
            .ok_or("Response map from future clock")?;
        if age >= MAP_MAX_AGE_STEPS || mode != self.mode_signature {
            return Err(
                "Native response stale or contact mode changed; reidentify before decision".into(),
            );
        }
        Ok(())
    }
}

pub(crate) fn axes(model: &Model) -> Result<Vec<Axis>, String> {
    if model.segments.len() != 25 {
        return Err("Expected 25 native bodies".into());
    }
    let mut result = Vec::new();
    for (i, s) in model.segments.iter().enumerate() {
        if let Some(p) = &s.joint_profile {
            for a in &p.axes {
                let k = ['x', 'y', 'z']
                    .iter()
                    .position(|c| *c == a.coordinate)
                    .ok_or("Unknown coordinate")?;
                if i == 0 {
                    return Err("Root is not an action".into());
                }
                if !a.min_radians.is_finite()
                    || !a.max_radians.is_finite()
                    || a.min_radians > a.max_radians
                    || result
                        .iter()
                        .any(|v: &Axis| v.segment == i && v.coordinate == k)
                {
                    return Err("Invalid/duplicate anatomical axis".into());
                }
                result.push(Axis {
                    segment: i,
                    coordinate: k,
                    minimum: a.min_radians,
                    maximum: a.max_radians,
                });
            }
        }
    }
    if model.segments.len() != 25 || result.len() != AXES {
        return Err("Expected exactly 25 bodies and 46 anatomical axes".into());
    }
    Ok(result)
}

pub(crate) fn initialize(
    model: &Model,
    q: Coordinates,
    native_substep: u64,
    mode: u64,
) -> Result<Memory, String> {
    axes(model)?;
    validate_command(
        model,
        &Command {
            positions: q,
            velocities: [[0.; 3]; 25],
        },
    )?;
    Ok(Memory {
        previous: q,
        next_substep: native_substep,
        rates: [0.; AXES],
        mode_signature: mode,
    })
}

/// Pure: input memory and problem are not mutated on failure. Caller assembles
/// native-output tasks in the same physical units as the response map.
pub(crate) fn decide(
    model: &Model,
    memory: &Memory,
    native_substep: u64,
    map: &ResponseMap,
    mode: u64,
    soft: &[OutputTask],
    hard: &[OutputConstraint],
    lower: Rates,
    upper: Rates,
) -> Result<(Memory, Solution), String> {
    if native_substep != memory.next_substep || native_substep % TICK_STEPS != 0 {
        return Err("Common decision requires the unique 60 Hz clock".into());
    }
    map.require_current(native_substep, mode)?;
    let end = native_substep
        .checked_add(TICK_STEPS)
        .ok_or("Command decision clock overflow")?;
    let mut problem = map.problem(soft, hard, lower, upper)?;
    let layout = axes(model)?;
    let tick_dt = lh_contracts::SUBSTEP_S as f64 * TICK_STEPS as f64;
    for (i, a) in layout.iter().enumerate() {
        let q = memory.previous[a.segment][a.coordinate] as f64;
        problem.lower[i] = problem.lower[i]
            .max(-1.)
            .max((a.minimum as f64 - q) / tick_dt);
        problem.upper[i] = problem.upper[i]
            .min(1.)
            .min((a.maximum as f64 - q) / tick_dt);
    }
    let solution = problem.solve()?;
    let mut next = memory.clone();
    next.rates = solution.rates;
    next.mode_signature = mode;
    // Check all four f32 commands, not merely the f64 optimizer result.
    let mut check = next.clone();
    for step in native_substep..end {
        let (emitted, after) = command(model, &check, step)?;
        let mut rates = [0.; AXES];
        for (i, a) in layout.iter().enumerate() {
            rates[i] = emitted.velocities[a.segment][a.coordinate] as f64;
        }
        // A f64 solution is not a certificate for rounded f32 commands.
        problem.verify_rates(&rates)?;
        check = after;
    }
    Ok((next, solution))
}

/// The same per-substep compiler is used by a real owner AND every preview.
/// dq is the derivative of the actual emitted f32 q (including rounding).
pub(crate) fn command(
    model: &Model,
    memory: &Memory,
    native_substep: u64,
) -> Result<(Command, Memory), String> {
    if native_substep != memory.next_substep {
        return Err("Duplicate/skipped command clock".into());
    }
    let layout = axes(model)?;
    let mut result = Command {
        positions: memory.previous,
        velocities: [[0.; 3]; 25],
    };
    let dt = lh_contracts::SUBSTEP_S;
    for (i, a) in layout.iter().enumerate() {
        let rate = memory.rates[i];
        if !rate.is_finite() || rate.abs() > 1. {
            return Err("Excess/nonfinite total rate".into());
        }
        let previous = memory.previous[a.segment][a.coordinate];
        let mut q = (previous as f64 + rate * dt as f64) as f32;
        // Move a single rounding ULP toward previous if casting would exceed
        // slew; this is command rounding, not relaxed physical acceptance.
        if (q - previous).abs() > dt {
            q = if q > 0. {
                f32::from_bits(if q > previous {
                    q.to_bits() - 1
                } else {
                    q.to_bits() + 1
                })
            } else if q < 0. {
                f32::from_bits(if q > previous {
                    q.to_bits() + 1
                } else {
                    q.to_bits() - 1
                })
            } else {
                q
            };
        }
        if q < a.minimum || q > a.maximum || (q - previous).abs() > dt {
            return Err("Compiled total command exceeds anatomy/slew".into());
        }
        result.positions[a.segment][a.coordinate] = q;
        result.velocities[a.segment][a.coordinate] = (q - previous) / dt;
    }
    validate_command(model, &result)?;
    let mut next = memory.clone();
    next.previous = result.positions;
    next.next_substep = next
        .next_substep
        .checked_add(1)
        .ok_or("Command clock overflow")?;
    Ok((result, next))
}

#[cfg(test)]
mod tests {
    use super::*;
    fn model() -> Model {
        Model::canonical().unwrap()
    }
    #[test]
    fn total_rate_produces_coherent_q_dq_for_all_axes_without_integration() {
        let model = model();
        let program = crate::recovery::trajectory_program::neutral_program(&model)
            .compile(&model)
            .unwrap();
        let mut memory = initialize(&model, program.command(0).positions, 0, 0).unwrap();
        memory.rates.fill(0.1);
        // Constrain axes resting at their anatomical bounds to move inward.
        for (i, a) in axes(&model).unwrap().iter().enumerate() {
            if memory.previous[a.segment][a.coordinate] + 0.01 > a.maximum {
                memory.rates[i] = -0.1;
            }
        }
        for step in 0..4 {
            let (c, next) = command(&model, &memory, step).unwrap();
            for a in axes(&model).unwrap() {
                assert_eq!(
                    c.velocities[a.segment][a.coordinate],
                    (c.positions[a.segment][a.coordinate]
                        - memory.previous[a.segment][a.coordinate])
                        / lh_contracts::SUBSTEP_S
                );
            }
            assert_eq!(c.positions[0], [0.; 3]);
            memory = next;
        }
        assert!(command(&model, &memory, 3).is_err());
    }
    #[test]
    fn one_sided_response_and_mode_expiry_without_integration() {
        let s = AxisSamples {
            negative: None,
            positive: Some((0.2, vec![1.4])),
        };
        let map = ResponseMap::from_samples(0, 7, vec![1.], [0.; AXES], &vec![s; AXES]).unwrap();
        assert!((map.rows[0][45] - 2.).abs() < 1e-12);
        assert_eq!(map.amplitudes[0], [0., 0.2]);
        assert_eq!(map.baseline, vec![1.]);
        assert!(map.require_current(23, 7).is_ok());
        assert!(map.require_current(24, 7).is_err());
        assert!(map.require_current(0, 8).is_err());
    }
    #[test]
    fn saturated_reference_has_no_extra_rate_budget_without_integration() {
        let model = model();
        let q = crate::recovery::trajectory_program::neutral_program(&model)
            .compile(&model)
            .unwrap()
            .command(0)
            .positions;
        let mut memory = initialize(&model, q, 0, 0).unwrap();
        memory.rates[0] = 1.01;
        assert!(command(&model, &memory, 0).is_err());
    }
    #[test]
    fn common_decision_uses_native_output_map_and_rejects_nan_without_integration() {
        let model = model();
        let memory = initialize(&model, [[0.; 3]; 25], 0, 7).unwrap();
        let sample = AxisSamples {
            negative: Some((-0.2, vec![0.8])),
            positive: Some((0.2, vec![1.2])),
        };
        let map =
            ResponseMap::from_samples(0, 7, vec![1.], [0.; AXES], &vec![sample; AXES]).unwrap();
        let tasks = [OutputTask {
            output: 0,
            target: 1.,
            weight: 1.,
        }];
        let (next, _) = decide(
            &model,
            &memory,
            0,
            &map,
            7,
            &tasks,
            &[],
            [-1.; AXES],
            [1.; AXES],
        )
        .unwrap();
        assert_eq!(next.rates, [0.; AXES]);
        let mut lower = [-1.; AXES];
        lower[1] = f64::NAN;
        assert!(decide(&model, &memory, 0, &map, 7, &tasks, &[], lower, [1.; AXES]).is_err());
        let bad = AxisSamples {
            negative: Some((-f64::MAX, vec![0.])),
            positive: Some((f64::MAX, vec![1.])),
        };
        assert!(ResponseMap::from_samples(0, 7, vec![1.], [0.; AXES], &vec![bad; AXES]).is_err());
    }
}
