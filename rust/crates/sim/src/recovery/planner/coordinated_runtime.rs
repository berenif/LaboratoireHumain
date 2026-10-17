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

/// Certificate for the four actual f32 commands, not only the continuous QP.
#[derive(Clone, Debug)]
pub(crate) struct RepresentationCertificate {
    pub rate_error_bounds: Rates,
    pub maximum_observed_rate_error: f64,
    pub commands: Vec<Command>,
}

/// First certify the direct optimum; if needed reserve a rounding enclosure.
/// In either case verify the exact emitted
/// sequence against the ORIGINAL problem. No physical or numerical acceptance
/// tolerance is increased. The enclosure only tightens the optimization set.
pub(crate) fn compile_problem(
    model: &Model,
    memory: &Memory,
    native_substep: u64,
    problem: &Problem,
) -> Result<(Memory, Solution, RepresentationCertificate), String> {
    compile_problem_traced(
        model,
        memory,
        native_substep,
        problem,
        &mut serde_json::Value::Null,
    )
}

pub(crate) fn compile_problem_traced(
    model: &Model,
    memory: &Memory,
    native_substep: u64,
    problem: &Problem,
    audit: &mut serde_json::Value,
) -> Result<(Memory, Solution, RepresentationCertificate), String> {
    *audit = serde_json::json!({"stage":"representation enclosure","nativeSubstep":native_substep,"previousQ":memory.previous});
    let result = (|| {
        if native_substep != memory.next_substep || native_substep % TICK_STEPS != 0 {
            return Err("Representable decision requires the unique 60 Hz clock".into());
        }
        problem.validate()?;
        let end = native_substep
            .checked_add(TICK_STEPS)
            .ok_or("Command decision clock overflow")?;
        let layout = axes(model)?;
        let dt = f64::from(lh_contracts::SUBSTEP_S);
        let ulp_above = |value: f64| -> Result<f64, String> {
            let value = value as f32;
            if !value.is_finite() || value < 0. {
                return Err("Nonfinite rounding enclosure".into());
            }
            let next = f32::from_bits(
                value
                    .to_bits()
                    .checked_add(1)
                    .ok_or("Rounding enclosure overflow")?,
            );
            let ulp = f64::from(next) - f64::from(value);
            if !ulp.is_finite() || ulp <= 0. {
                return Err("Invalid rounding enclosure".into());
            }
            Ok(ulp)
        };
        let mut guard = [0.; AXES];
        for (i, a) in layout.iter().enumerate() {
            let q = f64::from(memory.previous[a.segment][a.coordinate]).abs();
            // Four bounded steps plus an outward f32 enclosure. The compiler may
            // move one ULP toward the previous value at the exact slew boundary.
            let range = q + 4. * dt;
            let outward = range + 8. * f64::from(f32::EPSILON) * range.max(1.);
            let position_error = 2. * ulp_above(outward)?;
            let difference_error = ulp_above(dt + position_error)?;
            let division_error = ulp_above((dt + position_error + difference_error) / dt)?;
            guard[i] = (position_error + difference_error) / dt + division_error;
        }
        let certify = |solution: &Solution,
                       trace: &mut serde_json::Value|
         -> Result<(Memory, RepresentationCertificate), String> {
            trace["continuousRates"] = serde_json::json!(solution.rates.to_vec());
            trace["continuousCertificate"] = serde_json::json!({"iterations":solution.iterations,"hardError":solution.maximum_constraint_error,"projectedGradientResidual":solution.projected_gradient_residual});
            trace["attemptedCommands"] = serde_json::json!([]);
            let mut next = memory.clone();
            next.rates = solution.rates;
            let mut trial = next.clone();
            let mut commands = Vec::new();
            let mut maximum_error: f64 = 0.;
            for step in native_substep..end {
                let (emitted, after) = command(model, &trial, step)?;
                trace["attemptedCommands"].as_array_mut().unwrap().push(serde_json::json!({"nativeSubstep":step,"positions":emitted.positions,"velocities":emitted.velocities}));
                let rates = std::array::from_fn(|i| {
                    f64::from(emitted.velocities[layout[i].segment][layout[i].coordinate])
                });
                for i in 0..AXES {
                    let error = (rates[i] - solution.rates[i]).abs();
                    maximum_error = maximum_error.max(error);
                    if error > guard[i] {
                        return Err(format!(
                            "Rounding enclosure failed at substep {step} axis {i}: error={error:.17e}, bound={:.17e}",
                            guard[i]
                        ));
                    }
                }
                problem
                    .verify_rates(&rates)
                    .map_err(|e| format!("Exact f32 command at substep {step}: {e}"))?;
                commands.push(emitted);
                trial = after;
            }
            Ok((
                next,
                RepresentationCertificate {
                    rate_error_bounds: guard,
                    maximum_observed_rate_error: maximum_error,
                    commands,
                },
            ))
        };
        audit["rateErrorBounds"] = serde_json::json!(guard.to_vec());
        audit["stage"] = serde_json::json!("direct continuous solve");
        let direct = problem
            .solve()
            .map_err(|e| format!("Direct common solve: {e}"))?;
        let mut direct_trace = serde_json::json!({});
        match certify(&direct, &mut direct_trace) {
            Ok((next, certificate)) => {
                audit["directAttempt"] = direct_trace;
                audit["stage"] = serde_json::json!("certified direct exact f32 commands");
                return Ok((next, direct, certificate));
            }
            Err(error) => {
                direct_trace["refusal"] = serde_json::json!(error);
                audit["directAttempt"] = direct_trace;
                // An invalid enclosure is an implementation/integrity failure,
                // never a reason to try a different numerical candidate.
                if error.starts_with("Rounding enclosure failed") {
                    return Err(error);
                }
            }
        }
        // Exactly one interior repair after an unrepresentable direct optimum.
        // This preserves stationary commands and representable equalities.
        let mut tightened = problem.clone();
        for i in 0..AXES {
            tightened.lower[i] += guard[i];
            tightened.upper[i] -= guard[i];
        }
        for task in &mut tightened.hard {
            let margin = task
                .row
                .iter()
                .zip(guard)
                .map(|(a, e)| a.abs() * e)
                .sum::<f64>();
            task.lower += margin;
            task.upper -= margin;
        }
        audit["rateErrorBounds"] = serde_json::json!(guard.to_vec());
        audit["guardedLower"] = serde_json::json!(tightened.lower.to_vec());
        audit["guardedUpper"] = serde_json::json!(tightened.upper.to_vec());
        audit["guardedHardBounds"] = serde_json::json!(
            tightened
                .hard
                .iter()
                .map(|t| serde_json::json!({"name":t.name,"lower":t.lower,"upper":t.upper}))
                .collect::<Vec<_>>()
        );
        tightened.validate().map_err(|e| {
            format!("Rounding enclosure has no interior (representability not disproven): {e}")
        })?;
        audit["stage"] = serde_json::json!("guarded continuous solve");
        let solution = tightened
            .solve()
            .map_err(|e| format!("Representable common solve: {e}"))?;
        let mut guarded_trace = serde_json::json!({});
        let compiled = certify(&solution, &mut guarded_trace);
        if let Err(error) = &compiled {
            guarded_trace["refusal"] = serde_json::json!(error);
        }
        audit["guardedAttempt"] = guarded_trace;
        let (next, certificate) = compiled?;
        audit["stage"] = serde_json::json!("certified guarded exact f32 commands");
        Ok((next, solution, certificate))
    })();
    if let Err(error) = &result {
        audit["refusal"] = serde_json::json!(error);
    }
    result
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
    let (mut next, solution, _) = compile_problem(model, memory, native_substep, &problem)?;
    next.mode_signature = mode;
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

    fn assert_same_memory(actual: &Memory, expected: &Memory) {
        assert_eq!(
            actual.previous.map(|q| q.map(f32::to_bits)),
            expected.previous.map(|q| q.map(f32::to_bits))
        );
        assert_eq!(
            actual.rates.map(f64::to_bits),
            expected.rates.map(f64::to_bits)
        );
        assert_eq!(actual.next_substep, expected.next_substep);
        assert_eq!(actual.mode_signature, expected.mode_signature);
    }

    fn unconstrained_rates() -> Problem {
        Problem {
            lower: [-1.; AXES],
            upper: [1.; AXES],
            hard: vec![],
            soft: vec![],
            regularization: 0.01,
        }
    }

    #[test]
    fn exact_stationary_equalities_preserve_four_commands_without_integration() {
        let model = model();
        let memory = initialize(&model, [[0.; 3]; 25], 8, 19).unwrap();
        let before = memory.clone();
        let mut problem = unconstrained_rates();
        // Exercise a fixed box bound and a hard equality: neither has an
        // interior to tighten, but zero is exactly representable in f32.
        problem.lower[0] = 0.;
        problem.upper[0] = 0.;
        let mut row = [0.; AXES];
        row[1] = 1.;
        problem.hard.push(HardTask {
            name: "stationary material point".into(),
            row,
            lower: 0.,
            upper: 0.,
        });
        let mut audit = serde_json::Value::Null;
        let (compiled, solution, certificate) =
            compile_problem_traced(&model, &memory, 8, &problem, &mut audit).unwrap();
        assert_same_memory(&memory, &before);
        assert_same_memory(&compiled, &before);
        assert_eq!(solution.rates, [0.; AXES]);
        assert_eq!(certificate.maximum_observed_rate_error, 0.);
        assert_eq!(certificate.commands.len(), 4);
        for command in &certificate.commands {
            assert_eq!(command.positions, before.previous);
            assert_eq!(command.velocities, [[0.; 3]; 25]);
        }
        assert_eq!(audit["stage"], "certified direct exact f32 commands");
        assert_eq!(
            audit["directAttempt"]["attemptedCommands"]
                .as_array()
                .unwrap()
                .len(),
            4
        );
        assert!(audit.get("guardedHardBounds").is_none());
        assert!(audit.get("guardedAttempt").is_none());
    }

    #[test]
    fn unrepresentable_equality_refuses_with_guard_diagnostics_without_integration() {
        let model = model();
        let layout = axes(&model).unwrap();
        let (index, axis) = layout
            .iter()
            .enumerate()
            .find(|(_, a)| a.minimum <= -0.5 && a.maximum >= 0.5)
            .unwrap();
        let mut q = [[0.; 3]; 25];
        q[axis.segment][axis.coordinate] = 0.25;
        let mut memory = initialize(&model, q, 12, 23).unwrap();
        memory.rates[index] = -0.25;
        let before = memory.clone();
        let mut problem = unconstrained_rates();
        let mut row = [0.; AXES];
        row[index] = 1.;
        problem.hard.push(HardTask {
            name: "nonrepresentable rate equality".into(),
            row,
            lower: 0.1,
            upper: 0.1,
        });
        let mut audit = serde_json::Value::Null;
        let error = compile_problem_traced(&model, &memory, 12, &problem, &mut audit).unwrap_err();
        assert_same_memory(&memory, &before);
        assert!(
            error.starts_with("Rounding enclosure has no interior"),
            "{error}"
        );
        assert_eq!(audit["refusal"], error);
        assert!(
            audit["directAttempt"]["refusal"]
                .as_str()
                .unwrap()
                .starts_with("Exact f32 command at substep 12:")
        );
        assert_eq!(
            audit["directAttempt"]["attemptedCommands"]
                .as_array()
                .unwrap()
                .len(),
            1
        );
        // These diagnostics must survive the failed tightened.validate().
        assert_eq!(audit["rateErrorBounds"].as_array().unwrap().len(), AXES);
        assert_eq!(audit["guardedLower"].as_array().unwrap().len(), AXES);
        assert_eq!(audit["guardedUpper"].as_array().unwrap().len(), AXES);
        let guard = audit["rateErrorBounds"][index].as_f64().unwrap();
        assert!(guard > 0.);
        let guarded = &audit["guardedHardBounds"][0];
        assert_eq!(guarded["name"], "nonrepresentable rate equality");
        assert_eq!(guarded["lower"].as_f64().unwrap(), 0.1 + guard);
        assert_eq!(guarded["upper"].as_f64().unwrap(), 0.1 - guard);
        assert!(guarded["lower"].as_f64().unwrap() > guarded["upper"].as_f64().unwrap());
        assert!(audit.get("guardedAttempt").is_none());
    }

    #[test]
    fn negative_zero_and_binade_commands_have_exact_dq_without_integration() {
        let model = model();
        let layout = axes(&model).unwrap();
        let (index, axis) = layout
            .iter()
            .enumerate()
            .find(|(_, a)| a.minimum <= -0.5 && a.maximum >= 0.5)
            .unwrap();
        let below_binade = f32::from_bits(0.25_f32.to_bits() - 1);
        let cases = [
            (-0.25, -1.),
            (0., -1.),
            (-0.25 * lh_contracts::SUBSTEP_S, 0.5),
            (below_binade, 1.),
            (-below_binade, -1.),
        ];
        for (initial_q, desired_rate) in cases {
            let mut q = [[0.; 3]; 25];
            q[axis.segment][axis.coordinate] = initial_q;
            let memory = initialize(&model, q, 20, 31).unwrap();
            let before = memory.clone();
            let mut problem = unconstrained_rates();
            let mut row = [0.; AXES];
            row[index] = 1.;
            problem.soft.push(Task {
                row,
                target: desired_rate * (1. + problem.regularization),
                weight: 1.,
            });
            let (compiled, solution, certificate) =
                compile_problem(&model, &memory, 20, &problem).unwrap();
            assert_same_memory(&memory, &before);
            assert_eq!(certificate.commands.len(), 4);
            let mut replay = compiled;
            for (offset, emitted) in certificate.commands.iter().enumerate() {
                let rates = std::array::from_fn(|i| {
                    f64::from(emitted.velocities[layout[i].segment][layout[i].coordinate])
                });
                problem.verify_rates(&rates).unwrap();
                for (i, a) in layout.iter().enumerate() {
                    let actual_dq = emitted.velocities[a.segment][a.coordinate];
                    let expected_dq = (emitted.positions[a.segment][a.coordinate]
                        - replay.previous[a.segment][a.coordinate])
                        / lh_contracts::SUBSTEP_S;
                    assert_eq!(actual_dq.to_bits(), expected_dq.to_bits());
                    assert!(
                        (rates[i] - solution.rates[i]).abs() <= certificate.rate_error_bounds[i]
                    );
                }
                let (replayed, next) = command(&model, &replay, 20 + offset as u64).unwrap();
                assert_eq!(replayed.positions, emitted.positions);
                assert_eq!(replayed.velocities, emitted.velocities);
                replay = next;
            }
            let final_q = replay.previous[axis.segment][axis.coordinate];
            assert_eq!(replay.next_substep, 24);
            assert!((final_q - initial_q) * desired_rate as f32 > 0.);
            if initial_q == below_binade {
                assert!(final_q > 0.25);
            } else if initial_q == -below_binade {
                assert!(final_q < -0.25);
            } else if desired_rate > 0. {
                assert!(initial_q < 0. && final_q > 0.);
            }
        }
    }
}
