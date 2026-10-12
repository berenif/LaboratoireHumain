//! Command preparation for a complete continuation from the actual fixture.
//!
//! This module has no Simulation, integrator, motor installer, body mutation or
//! recovery counter. A compiled command schedule is deliberately not an admitted
//! support trajectory. Material support and native execution remain separate.
use crate::recovery::trajectory_program::{
    Command, CompiledProgram, Coordinates, Program, validate_command,
};
use crate::{measurement, rotation};
use lh_model::{Model, Pose};

const HZ: u32 = 240;
const LOCAL_DEADLINE: u32 = 2 * HZ;
const COMPLETE_DEADLINE: u32 = 25 * HZ;
const STABLE_PERSISTENCE: u32 = 132; // Original 0.55 s; not standing settle.
const EXTRA_HOLD: u32 = HZ;

/// These are schedule boundaries, never observations or forced transitions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Timing {
    pub heels_acquired: u32,
    pub recentered_hold_started: u32,
    pub recentered_hold_completed: u32,
    pub standing_reference: u32,
    pub hold_completed: u32,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Refusal {
    InvalidFixture(String),
    InvalidCommands(String),
    InitialCommandMismatch { segment: usize, coordinate: usize },
    MissingBoundary(u32),
    HeelDeadline,
    LocalPersistence,
    ContinuationOrder,
    CompleteDeadline,
    MissingStandingHold,
    LocalHoldMoves(u32),
    StandingHoldMoves(u32),
}

/// Its type cannot be confused with physical or static support admission.
pub(crate) struct CommandPreparation {
    pub commands: CompiledProgram,
    pub timing: Timing,
}

/// Derive only modeled coordinates using the same native convention as motors.
/// A stored fixture is initial data, not a post-step pose transplantation.
pub(crate) fn fixture_coordinates(model: &Model, poses: &[Pose]) -> Result<Coordinates, Refusal> {
    crate::recovery::validate_seed(model, poses).map_err(Refusal::InvalidFixture)?;
    if model.segments.len() != 25 {
        return Err(Refusal::InvalidFixture("Expected 25 native bodies".into()));
    }
    let mut q = [[0.0; 3]; 25];
    for (i, segment) in model.segments.iter().enumerate() {
        let Some(joint) = &segment.joint_profile else {
            continue;
        };
        let parent = model
            .segments
            .iter()
            .position(|p| Some(&p.id) == segment.parent.as_ref())
            .ok_or_else(|| Refusal::InvalidFixture("Missing native parent".into()))?;
        q[i] = measurement::coordinates(
            rotation(poses[parent].rotation),
            rotation(poses[i].rotation),
            joint,
        );
        for (k, coordinate) in ['x', 'y', 'z'].iter().enumerate() {
            if !joint.axes.iter().any(|axis| axis.coordinate == *coordinate) {
                if q[i][k].abs() > 1e-5 {
                    return Err(Refusal::InvalidFixture(format!(
                        "Unmodeled fixture rotation {}[{k}]",
                        segment.id
                    )));
                }
                q[i][k] = 0.0;
            }
        }
    }
    validate_command(
        model,
        &Command {
            positions: q,
            velocities: [[0.0; 3]; 25],
        },
    )
    .map_err(Refusal::InvalidCommands)?;
    Ok(q)
}

/// Compile every f32 command at 240 Hz, including interpolation interiors.
/// The conservative timing covers standing plus its extra second inside 25 s.
/// Neither this choice nor the schedule advances the original recovery observer.
pub(crate) fn compile_commands(
    model: &Model,
    initial: Coordinates,
    program: Program,
    timing: Timing,
) -> Result<CommandPreparation, Refusal> {
    validate_command(
        model,
        &Command {
            positions: initial,
            velocities: [[0.0; 3]; 25],
        },
    )
    .map_err(Refusal::InvalidCommands)?;
    if timing.heels_acquired == 0 || timing.heels_acquired >= LOCAL_DEADLINE {
        return Err(Refusal::HeelDeadline);
    }
    if timing.recentered_hold_started < timing.heels_acquired
        || timing.recentered_hold_started >= LOCAL_DEADLINE
        || timing.recentered_hold_completed
            < timing
                .recentered_hold_started
                .saturating_add(STABLE_PERSISTENCE)
    {
        return Err(Refusal::LocalPersistence);
    }
    if timing.standing_reference <= timing.recentered_hold_completed {
        return Err(Refusal::ContinuationOrder);
    }
    if timing.hold_completed > COMPLETE_DEADLINE {
        return Err(Refusal::CompleteDeadline);
    }
    if timing.hold_completed < timing.standing_reference.saturating_add(EXTRA_HOLD) {
        return Err(Refusal::MissingStandingHold);
    }
    for boundary in [
        timing.heels_acquired,
        timing.recentered_hold_started,
        timing.recentered_hold_completed,
        timing.standing_reference,
        timing.hold_completed,
    ] {
        if !program.keyframes.iter().any(|key| key.substep == boundary) {
            return Err(Refusal::MissingBoundary(boundary));
        }
    }
    let compiled = program.compile(model).map_err(Refusal::InvalidCommands)?;
    if compiled.duration_substeps() != timing.hold_completed {
        return Err(Refusal::CompleteDeadline);
    }
    let first = compiled.command(0);
    // Serialized source coordinates and native atan2/wrapping have independent
    // float32 roundoff. This explicit numerical comparison is not an anatomical
    // tolerance: the unchanged native compiler still checks every actual target.
    for i in 0..25 {
        for k in 0..3 {
            if (first.positions[i][k] - initial[i][k]).abs() > 1e-6 {
                return Err(Refusal::InitialCommandMismatch {
                    segment: i,
                    coordinate: k,
                });
            }
        }
    }
    let local_hold = compiled.command(timing.recentered_hold_started).positions;
    for step in timing.recentered_hold_started..=timing.recentered_hold_completed {
        let command = compiled.command(step);
        // Constant nonzero Hermite endpoints can differ by f32 rounding. The
        // native compiler already checks exact velocity/slew limits; require
        // the local hold within four float32 epsilons and zero tangent.
        if command
            .positions
            .iter()
            .flatten()
            .zip(local_hold.iter().flatten())
            .any(|(q, held)| (q - held).abs() > 4.0 * f32::EPSILON)
            || command.velocities != [[0.0; 3]; 25]
        {
            return Err(Refusal::LocalHoldMoves(step));
        }
    }
    let standing = compiled.command(timing.standing_reference).positions;
    for step in timing.standing_reference..=timing.hold_completed {
        let command = compiled.command(step);
        if command.positions != standing || command.velocities != [[0.0; 3]; 25] {
            return Err(Refusal::StandingHoldMoves(step));
        }
    }
    Ok(CommandPreparation {
        commands: compiled,
        timing,
    })
}

/// Missing prerequisite identities, not booleans masquerading as proof.
/// No function in this module converts a command-only plan into permission to
/// execute. These requirements need independently linked evidence; producing
/// command keys does not satisfy any support requirement by itself.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum MissingPreparation {
    AcquisitionAndRecenterKeyframes,
    MaterialPivotAndReplacementSchedule,
    NativeGeometryThroughoutInterpolation,
    RetainedMaterialPointsThroughoutInterpolation,
    SupportFeasibilityThroughoutContinuation,
    OriginalObserverAndHandoffAdapter,
}

pub(crate) const UNSCREENED_REQUIREMENTS: [MissingPreparation; 6] = [
    MissingPreparation::AcquisitionAndRecenterKeyframes,
    MissingPreparation::MaterialPivotAndReplacementSchedule,
    MissingPreparation::NativeGeometryThroughoutInterpolation,
    MissingPreparation::RetainedMaterialPointsThroughoutInterpolation,
    MissingPreparation::SupportFeasibilityThroughoutContinuation,
    MissingPreparation::OriginalObserverAndHandoffAdapter,
];

#[cfg(test)]
mod tests {
    use super::*;
    use crate::recovery::trajectory_program::Keyframe;

    fn command_only() -> (Model, Program, Timing) {
        let model = Model::canonical().unwrap();
        let timing = Timing {
            heels_acquired: 240,
            recentered_hold_started: 240,
            recentered_hold_completed: 372,
            standing_reference: 19 * HZ,
            hold_completed: 20 * HZ,
        };
        let program = Program {
            schema_version: 1,
            segment_ids: model.segments.iter().map(|s| s.id.clone()).collect(),
            keyframes: [0, 240, 372, 19 * HZ, 20 * HZ]
                .into_iter()
                .map(|substep| Keyframe {
                    substep,
                    positions: [[0.0; 3]; 25],
                    velocities: [[0.0; 3]; 25],
                })
                .collect(),
            contacts: vec![],
        };
        (model, program, timing)
    }

    #[test]
    fn continuation_command_validation_does_not_admit_support() {
        let (model, program, timing) = command_only();
        let result = compile_commands(&model, [[0.0; 3]; 25], program, timing).unwrap();
        assert_eq!(
            result.commands.duration_substeps(),
            result.timing.hold_completed
        );
        assert_eq!(UNSCREENED_REQUIREMENTS.len(), 6);
        assert!(
            UNSCREENED_REQUIREMENTS
                .contains(&MissingPreparation::RetainedMaterialPointsThroughoutInterpolation)
        );
    }

    #[test]
    fn continuation_refuses_deadline_and_short_persistence() {
        let (model, program, mut timing) = command_only();
        timing.heels_acquired = 481;
        assert!(matches!(
            compile_commands(&model, [[0.0; 3]; 25], program.clone(), timing),
            Err(Refusal::HeelDeadline)
        ));
        timing.heels_acquired = 241;
        assert!(matches!(
            compile_commands(&model, [[0.0; 3]; 25], program, timing),
            Err(Refusal::LocalPersistence)
        ));
    }

    #[test]
    fn continuation_refuses_motion_during_extra_second() {
        let (model, mut program, timing) = command_only();
        let lumbar = model
            .segments
            .iter()
            .position(|s| s.id == "lumbar")
            .unwrap();
        program.keyframes.last_mut().unwrap().positions[lumbar][0] = 0.01;
        assert!(matches!(
            compile_commands(&model, [[0.0; 3]; 25], program, timing),
            Err(Refusal::StandingHoldMoves(_))
        ));
    }

    #[test]
    fn continuation_derives_real_fixture_without_world_or_steps() {
        let model = Model::canonical().unwrap();
        let fixtures: crate::recovery::FixtureSet =
            serde_json::from_str(crate::recovery::FIXTURES).unwrap();
        let fixture = fixtures
            .cases
            .iter()
            .find(|f| f.id == "landed-crouch-left")
            .unwrap();
        let initial = fixture_coordinates(&model, &fixture.poses).unwrap();
        assert_eq!(initial[0], [0.0; 3]);
        assert!(initial.iter().flatten().any(|q| q.abs() > 0.1));
    }

    #[test]
    fn continuation_new_six_key_candidate_has_native_command_continuity() {
        let model = Model::canonical().unwrap();
        let fixtures: crate::recovery::FixtureSet =
            serde_json::from_str(crate::recovery::FIXTURES).unwrap();
        let fixture = fixtures
            .cases
            .iter()
            .find(|f| f.id == "landed-crouch-left")
            .unwrap();
        let initial = fixture_coordinates(&model, &fixture.poses).unwrap();
        let program: Program = serde_json::from_str(include_str!(
            "../../../../../../docs/evidence/coordination-prototype-2026-10-12/continuation-candidate-program.json"
        )).unwrap();
        let timing = Timing {
            heels_acquired: 168,
            recentered_hold_started: 300,
            recentered_hold_completed: 432,
            standing_reference: 1680,
            hold_completed: 1920,
        };
        compile_commands(&model, initial, program, timing).unwrap();
    }
}
