//! Absolute, immutable commands shared by future whole-history search and replay.
//! Experimental/test-only. This module never writes a body or changes physics.
use lh_model::Model;
use serde::{Deserialize, Serialize};

pub(crate) type Coordinates = [[f32; 3]; 25];
const MIN_STEPS: u32 = 8 * 240;
// User-authorized temporal dilation; physical success still must occur by 25 s.
const MAX_STEPS: u32 = 25 * 240;
const RATE: f32 = 1.0;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub(crate) struct Keyframe {
    pub substep: u32,
    pub positions: Coordinates,
    pub velocities: Coordinates,
}

/// Intent annotation on an existing triangle; it neither creates nor resolves contact.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub(crate) struct ContactSurface {
    pub segment: String,
    pub triangle: u32,
    pub barycentric: [f32; 3],
    pub start_substep: u32,
    pub end_substep: u32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub(crate) struct Program {
    pub schema_version: u16,
    pub segment_ids: Vec<String>,
    pub keyframes: Vec<Keyframe>,
    pub contacts: Vec<ContactSurface>,
}

#[derive(Clone)]
pub(crate) struct CompiledProgram(Program);

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Command {
    pub positions: Coordinates,
    pub velocities: Coordinates,
}

impl Program {
    pub fn compile(self, model: &Model) -> Result<CompiledProgram, String> {
        if self.schema_version != 1
            || self.segment_ids
                != model
                    .segments
                    .iter()
                    .map(|s| s.id.clone())
                    .collect::<Vec<_>>()
            || model.segments.len() != 25
            || self.keyframes.len() < 2
            || self.keyframes.len() > MAX_STEPS as usize + 1
            || self.keyframes[0].substep != 0
        {
            return Err("Program schema/topology/initial clock".into());
        }
        let last = self.keyframes.last().unwrap();
        if !(MIN_STEPS..=MAX_STEPS).contains(&last.substep)
            || self
                .keyframes
                .windows(2)
                .any(|w| w[0].substep >= w[1].substep)
            || [self.keyframes.first().unwrap(), last]
                .iter()
                .any(|k| k.velocities.iter().flatten().any(|v| *v != 0.0))
        {
            return Err("Program duration/order/rest endpoints".into());
        }
        for surface in &self.contacts {
            let s = model
                .segments
                .iter()
                .find(|s| s.id == surface.segment)
                .ok_or("Unknown contact segment")?;
            if surface.triangle as usize >= s.geometry.indices.len() / 3
                || surface.start_substep >= surface.end_substep
                || surface.end_substep > last.substep
                || surface
                    .barycentric
                    .iter()
                    .any(|v| !v.is_finite() || *v < 0.0 || *v > 1.0)
                || (surface.barycentric.iter().sum::<f32>() - 1.0).abs() > 1e-6
            {
                return Err("Contact annotation outside existing native surface/time".into());
            }
        }
        for key in &self.keyframes {
            validate_command(
                model,
                &Command {
                    positions: key.positions,
                    velocities: key.velocities,
                },
            )?;
        }
        let compiled = CompiledProgram(self);
        // Validate the actual f32 command grid, including interpolation interiors.
        let mut previous = compiled.command(0);
        for step in 0..=compiled.duration_substeps() {
            let command = compiled.command(step);
            validate_command(model, &command)?;
            if step > 0
                && command
                    .positions
                    .iter()
                    .flatten()
                    .zip(previous.positions.iter().flatten())
                    .any(|(q, old)| (q - old).abs() > RATE * lh_contracts::SUBSTEP_S)
            {
                return Err(format!("Program target slew at substep {step}"));
            }
            previous = command;
        }
        Ok(compiled)
    }
}

pub(crate) fn validate_command(model: &Model, command: &Command) -> Result<(), String> {
    for (i, segment) in model.segments.iter().enumerate() {
        for k in 0..3 {
            let q = command.positions[i][k];
            let dq = command.velocities[i][k];
            if !q.is_finite() || !dq.is_finite() || dq.abs() > RATE {
                return Err(format!(
                    "Nonfinite/excess rate {}[{k}] measured q={q}, dq={dq}, rate limit={RATE}",
                    segment.id
                ));
            }
            let axis = segment
                .joint_profile
                .as_ref()
                .and_then(|p| p.axes.iter().find(|a| a.coordinate == ['x', 'y', 'z'][k]));
            match axis {
                Some(a) if q < a.min_radians || q > a.max_radians => {
                    return Err(format!(
                        "Anatomical command limit {}[{k}] measured q={q}",
                        segment.id
                    ));
                }
                None if q != 0.0 || dq != 0.0 => {
                    return Err(format!("Unmodeled/root command {}[{k}]", segment.id));
                }
                _ => {}
            }
        }
    }
    Ok(())
}

impl CompiledProgram {
    pub fn duration_substeps(&self) -> u32 {
        self.0.keyframes.last().unwrap().substep
    }
    pub fn source(&self) -> &Program {
        &self.0
    }
    /// No cumulative deltas, cursor or mutable state: identical clocks give identical bits.
    pub fn command(&self, substep: u32) -> Command {
        let keys = &self.0.keyframes;
        let end = keys.last().unwrap();
        if substep >= end.substep {
            return Command {
                positions: end.positions,
                velocities: [[0.0; 3]; 25],
            };
        }
        let right = keys.partition_point(|k| k.substep <= substep);
        let a = &keys[right - 1];
        if a.substep == substep {
            return Command {
                positions: a.positions,
                velocities: a.velocities,
            };
        }
        let b = &keys[right];
        let span = (b.substep - a.substep) as f32 * lh_contracts::SUBSTEP_S;
        let u = (substep - a.substep) as f32 / (b.substep - a.substep) as f32;
        let u2 = u * u;
        let u3 = u2 * u;
        let mut result = Command {
            positions: [[0.0; 3]; 25],
            velocities: [[0.0; 3]; 25],
        };
        for i in 0..25 {
            for k in 0..3 {
                let (qa, qb, va, vb) = (
                    a.positions[i][k],
                    b.positions[i][k],
                    a.velocities[i][k],
                    b.velocities[i][k],
                );
                result.positions[i][k] = (2.0 * u3 - 3.0 * u2 + 1.0) * qa
                    + (u3 - 2.0 * u2 + u) * span * va
                    + (-2.0 * u3 + 3.0 * u2) * qb
                    + (u3 - u2) * span * vb;
                result.velocities[i][k] = ((6.0 * u2 - 6.0 * u) * qa + (-6.0 * u2 + 6.0 * u) * qb)
                    / span
                    + (3.0 * u2 - 4.0 * u + 1.0) * va
                    + (3.0 * u2 - 2.0 * u) * vb;
            }
        }
        result
    }
}

pub(crate) fn neutral_program(model: &Model) -> Program {
    Program {
        schema_version: 1,
        segment_ids: model.segments.iter().map(|s| s.id.clone()).collect(),
        keyframes: vec![
            Keyframe {
                substep: 0,
                positions: [[0.0; 3]; 25],
                velocities: [[0.0; 3]; 25],
            },
            Keyframe {
                substep: MIN_STEPS,
                positions: [[0.0; 3]; 25],
                velocities: [[0.0; 3]; 25],
            },
        ],
        contacts: vec![],
    }
}

#[test]
fn whole_program_search_replay_commands_are_absolute_and_bit_identical() {
    let model = Model::canonical().unwrap();
    let mut p = neutral_program(&model);
    let i = model
        .segments
        .iter()
        .position(|s| s.id == "lumbar")
        .unwrap();
    p.keyframes.insert(
        1,
        Keyframe {
            substep: 960,
            positions: [[0.0; 3]; 25],
            velocities: [[0.0; 3]; 25],
        },
    );
    p.keyframes[1].positions[i][0] = 0.1;
    let searched = p.clone().compile(&model).unwrap();
    let replayed: Program = serde_json::from_str(&serde_json::to_string(&p).unwrap()).unwrap();
    let replayed = replayed.compile(&model).unwrap();
    for step in (0..=2000).rev() {
        assert_eq!(searched.command(step), replayed.command(step));
    }
    assert_eq!(searched.command(960).positions, p.keyframes[1].positions);
    assert_eq!(searched.command(2000).velocities, [[0.0; 3]; 25]);
}

#[test]
fn program_refuses_interior_rate_limits_root_commands_and_foreign_surfaces() {
    let model = Model::canonical().unwrap();
    let p = neutral_program(&model);
    let mut root = p.clone();
    root.keyframes[0].positions[0][0] = 0.01;
    assert!(root.compile(&model).is_err());
    let mut rate = p.clone();
    rate.keyframes.insert(
        1,
        Keyframe {
            substep: 1,
            positions: [[0.0; 3]; 25],
            velocities: [[0.0; 3]; 25],
        },
    );
    let i = model
        .segments
        .iter()
        .position(|s| s.id == "lumbar")
        .unwrap();
    rate.keyframes[1].positions[i][0] = 0.1;
    assert!(rate.compile(&model).is_err());
    let mut surface = p.clone();
    surface.contacts.push(ContactSurface {
        segment: "leftKnee".into(),
        triangle: 0,
        barycentric: [1.0, 0.0, 0.0],
        start_substep: 0,
        end_substep: 10,
    });
    assert!(surface.compile(&model).is_err());
    let mut bad = serde_json::to_value(p).unwrap();
    bad["rootForce"] = serde_json::json!([0, 1, 0]);
    assert!(serde_json::from_value::<Program>(bad).is_err());
}

#[test]
fn program_rejects_hermite_interior_anatomical_overshoot_with_valid_knots() {
    let model = Model::canonical().unwrap();
    let mut p = neutral_program(&model);
    let i = model
        .segments
        .iter()
        .position(|s| s.id == "lumbar")
        .unwrap();
    let bound = model.segments[i]
        .joint_profile
        .as_ref()
        .unwrap()
        .axes
        .iter()
        .find(|a| a.coordinate == 'x')
        .unwrap()
        .max_radians;
    let mut a = p.keyframes[0].clone();
    a.substep = 480;
    a.positions[i][0] = bound - 0.01;
    a.velocities[i][0] = 1.0;
    let mut b = a.clone();
    b.substep = 960;
    b.velocities[i][0] = -1.0;
    p.keyframes.insert(1, a);
    p.keyframes.insert(2, b);
    for key in &p.keyframes {
        validate_command(
            &model,
            &Command {
                positions: key.positions,
                velocities: key.velocities,
            },
        )
        .unwrap();
    }
    assert!(p.compile(&model).is_err());
}
