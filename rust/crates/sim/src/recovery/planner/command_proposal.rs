//! Command feasibility is NOT contact feasibility.
//!
//! The archived geometric rows multiply reference rates, not measured joint
//! rates. ForceBased does not make those variables equal. They may guide a
//! candidate, but cannot certify a material support or a floating-base motion.
//! Original command bounds and unrelated hard constraints remain hard. Actual
//! support obligations, anchors and all original physical gates survive this
//! separation and require a separately budgeted native preview.
use super::{coordinated_runtime as runtime, support_state, whole_body_tasks};
use crate::recovery::trajectory_program::Command;
use lh_model::Model;
use serde::Serialize;
use whole_body_tasks::{AXES, HardTask, Problem, Rates, Task};

/// There is deliberately no static constructor for a successful native status.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub(crate) enum PhysicalAdmission {
    NativePreviewRequired,
}

#[derive(Clone, Debug, Serialize)]
pub(crate) struct GuideResidual {
    pub name: String,
    pub original_lower_mps: f64,
    pub original_upper_mps: f64,
    /// Algebraic value under the explicitly unvalidated unit servo response.
    pub heuristic_value_mps: f64,
    pub outside_original_band_mps: f64,
}

#[derive(Clone, Debug, Serialize)]
pub(crate) struct MaterialGuide {
    pub name: String,
    pub original_row: Vec<f64>,
    pub original_lower_mps: f64,
    pub original_upper_mps: f64,
}

/// A proposal can be inspected or supplied to a future preview adapter. It is
/// not an executable recovery command or a proof that any contact will hold.
pub(crate) struct CommandProposal {
    commands: Vec<Command>,
    pub native_substep: u64,
    pub rates: Rates,
    pub material_obligations: Vec<support_state::MaterialPatch>,
    pub material_guides: Vec<MaterialGuide>,
    pub guide_residuals: Vec<Vec<GuideResidual>>,
    pub physical_admission: PhysicalAdmission,
    pub numerical_iterations: usize,
    pub continuous_qp_constraint_error: f64,
}

impl CommandProposal {
    pub(crate) fn commands_for_native_preview(&self) -> &[Command] {
        &self.commands
    }

    /// Fail closed until a future driver supplies complete native evidence.
    /// A geometric optimum, low residual, or synthetic observation is never it.
    pub(crate) fn require_real_execution_admission(&self) -> Result<(), String> {
        Err("Command-feasible proposal has no native trajectory/support admission".into())
    }
}

struct ProposalProblem {
    commands: Problem,
    guides: Vec<HardTask>,
    obligations: Vec<support_state::MaterialPatch>,
}

/// Explicit classification, not a blanket hard-to-soft fallback after failure.
/// Only the complete named material rows belonging to current retained patches
/// may become unvalidated geometric guidance. Unknown hard rows stay hard.
fn separate_command_and_support_domains(
    original: &Problem,
    retained: &[support_state::MaterialPatch],
    at: u64,
) -> Result<ProposalProblem, String> {
    original.validate()?;
    if retained.is_empty() || retained.len() > 64 {
        return Err("Support proposal needs explicit retained material obligations".into());
    }
    for (i, p) in retained.iter().enumerate() {
        if p.mode == support_state::ContactMode::Free
            || p.segment >= support_state::BODY_COUNT
            || p.last_observed_substep != at
            || p.acquired_substep > at
            || !p.current_contact_present
            || p.local_point_m
                .iter()
                .chain(&p.world_anchor_m)
                .chain(&p.last_world_point_m)
                .chain(&p.displacement_from_anchor_m)
                .any(|v| !v.is_finite())
            || !p.tangential_drift_m.is_finite()
            || p.tangential_drift_m < 0.
            || retained[..i].iter().any(|old| old.id == p.id)
        {
            return Err("Invalid, missing or stale material obligation".into());
        }
    }
    let names: Vec<_> = retained
        .iter()
        .flat_map(|p| (0..3).map(move |axis| format!("material-patch-{}-world-{axis}", p.id)))
        .collect();
    if names
        .iter()
        .any(|name| original.hard.iter().filter(|t| &t.name == name).count() != 1)
        || original
            .hard
            .iter()
            .any(|t| t.name.starts_with("material-patch-") && !names.contains(&t.name))
    {
        return Err(
            "Material guidance must exactly cover every retained patch, once per axis".into(),
        );
    }
    let mut commands = original.clone();
    let mut guides = Vec::new();
    commands.hard.retain(|task| {
        if names.contains(&task.name) {
            guides.push(task.clone());
            false
        } else {
            true
        }
    });
    for guide in &guides {
        // Fixed unit-weight geometric intent, not a fitted servo derivative or
        // a relaxed acceptance threshold. Original bands remain in evidence.
        commands.soft.push(Task {
            row: guide.row,
            target: 0.5 * guide.lower + 0.5 * guide.upper,
            weight: 1.,
        });
    }
    commands.validate()?;
    Ok(ProposalProblem {
        commands,
        guides,
        obligations: retained.to_vec(),
    })
}

/// No Simulation, world, geometry reconstruction, motor installation or step.
/// `model` is frozen anatomical metadata used by the original q/dq compiler.
pub(crate) fn prepare(
    model: &Model,
    memory: &runtime::Memory,
    original: &Problem,
    retained: &[support_state::MaterialPatch],
    native_substep: u64,
) -> Result<CommandProposal, String> {
    let separated = separate_command_and_support_domains(original, retained, native_substep)?;
    let (_, solution, representation) =
        runtime::compile_problem(model, memory, native_substep, &separated.commands)?;
    let layout = runtime::axes(model)?;
    let mut residuals = Vec::new();
    for command in &representation.commands {
        let rates: Rates = std::array::from_fn(|i| {
            f64::from(command.velocities[layout[i].segment][layout[i].coordinate])
        });
        separated.commands.verify_rates(&rates)?;
        let mut rows = Vec::new();
        for guide in &separated.guides {
            let value: f64 = guide.row.iter().zip(rates).map(|(a, x)| a * x).sum();
            if !value.is_finite() {
                return Err("Nonfinite uncertified material-guide residual".into());
            }
            rows.push(GuideResidual {
                name: guide.name.clone(),
                original_lower_mps: guide.lower,
                original_upper_mps: guide.upper,
                heuristic_value_mps: value,
                outside_original_band_mps: (guide.lower - value).max(value - guide.upper).max(0.),
            });
        }
        residuals.push(rows);
    }
    Ok(CommandProposal {
        commands: representation.commands,
        native_substep,
        rates: solution.rates,
        material_obligations: separated.obligations,
        material_guides: separated
            .guides
            .iter()
            .map(|g| MaterialGuide {
                name: g.name.clone(),
                original_row: g.row.to_vec(),
                original_lower_mps: g.lower,
                original_upper_mps: g.upper,
            })
            .collect(),
        guide_residuals: residuals,
        physical_admission: PhysicalAdmission::NativePreviewRequired,
        numerical_iterations: solution.iterations,
        continuous_qp_constraint_error: solution.maximum_constraint_error,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{Value, json};

    fn fixtures() -> Value {
        serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"),
            "/../../../docs/evidence/coordination-support-reformulation-2026-10-13/archived-step16-fixtures.json"))).unwrap()
    }
    fn rates(value: &Value) -> Rates {
        assert_eq!(value.as_array().unwrap().len(), AXES);
        std::array::from_fn(|i| value[i].as_f64().unwrap())
    }
    fn problem(case: &Value) -> Problem {
        let p = &case["problem"];
        Problem {
            lower: rates(&p["lower"]),
            upper: rates(&p["upper"]),
            regularization: p["regularization"].as_f64().unwrap(),
            hard: p["hard"]
                .as_array()
                .unwrap()
                .iter()
                .map(|t| HardTask {
                    name: t["name"].as_str().unwrap().into(),
                    row: rates(&t["row"]),
                    lower: t["lower"].as_f64().unwrap(),
                    upper: t["upper"].as_f64().unwrap(),
                })
                .collect(),
            soft: p["soft"]
                .as_array()
                .unwrap()
                .iter()
                .map(|t| Task {
                    row: rates(&t["row"]),
                    target: t["target"].as_f64().unwrap(),
                    weight: t["weight"].as_f64().unwrap(),
                })
                .collect(),
        }
    }
    fn patches(case: &Value) -> Vec<support_state::MaterialPatch> {
        serde_json::from_value(case["patches"]["active"].clone()).unwrap()
    }
    fn metadata() -> Model {
        // Deserialize only. Do not call Model::canonical()/validate(), which
        // would also calculate geometry-dependent mass properties.
        serde_json::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../model/data/canonical-v1.json"
        )))
        .unwrap()
    }

    #[test]
    fn four_archived_states_compile_command_feasible_but_never_support_admitted() {
        let fixture = fixtures();
        let model = metadata();
        let layout = runtime::axes(&model).unwrap();
        let mut cases = Vec::new();
        for case in fixture["cases"].as_array().unwrap() {
            let original = problem(case);
            let retained = patches(case);
            let previous = serde_json::from_value(case["previousQ"].clone()).unwrap();
            let memory = runtime::initialize(&model, previous, 16, 0).unwrap();
            let proposal = prepare(&model, &memory, &original, &retained, 16).unwrap();
            assert_eq!(proposal.commands.len(), 4);
            assert_eq!(
                proposal.physical_admission,
                PhysicalAdmission::NativePreviewRequired
            );
            assert!(proposal.require_real_execution_admission().is_err());
            assert_eq!(
                serde_json::to_value(&retained).unwrap(),
                serde_json::to_value(&proposal.material_obligations).unwrap()
            );
            assert_eq!(memory.next_substep, 16, "No caller clock was advanced");
            assert_eq!(proposal.material_guides.len(), original.hard.len());
            for (guide, original) in proposal.material_guides.iter().zip(&original.hard) {
                assert_eq!(guide.original_row, original.row.to_vec());
                assert_eq!(guide.original_lower_mps.to_bits(), original.lower.to_bits());
                assert_eq!(guide.original_upper_mps.to_bits(), original.upper.to_bits());
            }
            let mut prior = memory.previous;
            for command in proposal.commands_for_native_preview() {
                for (i, a) in layout.iter().enumerate() {
                    let q = command.positions[a.segment][a.coordinate];
                    let dq = command.velocities[a.segment][a.coordinate];
                    assert!(q >= a.minimum && q <= a.maximum);
                    assert!(dq.abs() <= 1.);
                    assert_eq!(
                        dq.to_bits(),
                        ((q - prior[a.segment][a.coordinate]) / lh_contracts::SUBSTEP_S).to_bits()
                    );
                    assert!(
                        f64::from(dq) >= original.lower[i] - 1e-8
                            && f64::from(dq) <= original.upper[i] + 1e-8
                    );
                }
                prior = command.positions;
            }
            let largest = proposal
                .guide_residuals
                .iter()
                .flatten()
                .map(|r| r.outside_original_band_mps)
                .fold(0., f64::max);
            assert!(
                largest > 1e-8,
                "Incompatible old support rows must not disappear from evidence"
            );
            cases.push(json!({"feedbackScale":case["feedbackScale"],"commandFeasible":true,
                "physicalAdmission":proposal.physical_admission,"nativeSubsteps":0,
                "iterations":proposal.numerical_iterations,"continuousQpConstraintError":proposal.continuous_qp_constraint_error,
                "materialObligations":proposal.material_obligations,"materialGuides":proposal.material_guides,"guideResiduals":proposal.guide_residuals,
                "commands":proposal.commands.iter().map(|c|json!({"positions":c.positions,"velocities":c.velocities})).collect::<Vec<_>>() }));
        }
        if let Ok(path) = std::env::var("LH_COMMAND_PROPOSAL_STATIC_EXPORT") {
            use std::io::Write;
            let output = json!({"schema":1,"scope":"pure command compilation of archived coefficients; not support feasibility",
                "sourceFixturePlanningSha256":fixture["sourcePlanningSha256"],"nativeSteps":0,"worldsConstructed":0,"fkEvaluations":0,
                "commandModel":"46 reference rates; geometric unit-response heuristic is explicitly unvalidated", "cases":cases});
            let mut file = std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(path)
                .unwrap();
            file.write_all(&serde_json::to_vec_pretty(&output).unwrap())
                .unwrap();
            file.write_all(b"\n").unwrap();
        }
    }

    #[test]
    fn command_boxes_and_unknown_hard_constraints_are_never_relaxed() {
        let f = fixtures();
        let case = &f["cases"][0];
        let mut p = problem(case);
        let retained = patches(case);
        let mut row = [0.; AXES];
        row[0] = 1.;
        p.hard.push(HardTask {
            name: "protected-integrity-fixture".into(),
            row,
            lower: 2.,
            upper: 3.,
        });
        let separated = separate_command_and_support_domains(&p, &retained, 16).unwrap();
        assert_eq!(p.lower, separated.commands.lower);
        assert_eq!(p.upper, separated.commands.upper);
        assert_eq!(separated.commands.hard.len(), 1);
        assert!(separated.commands.solve().is_err());
    }
    #[test]
    fn stale_missing_free_duplicate_or_nonfinite_obligations_are_refused() {
        let f = fixtures();
        let c = &f["cases"][0];
        let p = problem(c);
        let patches = patches(c);
        for variant in 0..5 {
            let mut bad = patches.clone();
            match variant {
                0 => bad[0].last_observed_substep = 15,
                1 => bad[0].current_contact_present = false,
                2 => bad[0].mode = support_state::ContactMode::Free,
                3 => bad.push(bad[0].clone()),
                _ => bad[0].world_anchor_m[0] = f64::NAN,
            }
            assert!(separate_command_and_support_domains(&p, &bad, 16).is_err());
        }
    }
    #[test]
    fn material_row_omissions_or_renaming_do_not_hide_obligations() {
        let f = fixtures();
        let c = &f["cases"][0];
        let p = problem(c);
        let patches = patches(c);
        for variant in 0..3 {
            let mut bad = p.clone();
            match variant {
                0 => {
                    bad.hard.pop();
                }
                1 => bad.hard[0].name = "material-patch-99-world-0".into(),
                _ => bad.hard.push(bad.hard[0].clone()),
            }
            assert!(separate_command_and_support_domains(&bad, &patches, 16).is_err());
        }
    }
    #[test]
    fn even_zero_geometric_residual_cannot_authorize_a_native_transfer() {
        let f = fixtures();
        let c = &f["cases"][0];
        let mut p = problem(c);
        let patches = patches(c);
        for t in &mut p.hard {
            t.row = [0.; AXES];
            t.lower = -0.005;
            t.upper = 0.005;
        }
        let model = metadata();
        let q = serde_json::from_value(c["previousQ"].clone()).unwrap();
        let memory = runtime::initialize(&model, q, 16, 0).unwrap();
        let proposal = prepare(&model, &memory, &p, &patches, 16).unwrap();
        assert!(
            proposal
                .guide_residuals
                .iter()
                .flatten()
                .all(|r| r.outside_original_band_mps == 0.)
        );
        assert!(proposal.require_real_execution_admission().is_err());
    }
}
