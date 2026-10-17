//! Explicit orientation mapping into the native quaternion coordinate convention.
//! This is a checked conversion primitive, not a completed Tao retargeting.
use super::trajectory_program::{Command, Coordinates, validate_command};
use crate::{measurement, rotation};
use lh_model::{Model, Quat};
use rapier3d::prelude::Rotation;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase", deny_unknown_fields)]
pub(crate) enum OrientationSource {
    World {
        source: String,
        offset: Quat,
    },
    /// Explicitly specified native joint-frame rotation for a missing source segment.
    ParentRelative {
        rotation: Quat,
    },
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub(crate) struct SegmentMapping {
    pub target: String,
    pub orientation: OrientationSource,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub(crate) struct Mapping {
    pub source_to_native_basis: Quat,
    pub segments: Vec<SegmentMapping>,
}

fn checked(q: Quat) -> Result<Rotation, String> {
    let r = rotation(q);
    if !r.is_finite() || (r.length_squared() - 1.0).abs() > 1e-5 {
        return Err("Nonfinite/nonunit mapping quaternion".into());
    }
    Ok(r)
}

impl Mapping {
    pub fn coordinates(
        &self,
        model: &Model,
        source: &BTreeMap<String, Quat>,
    ) -> Result<Coordinates, String> {
        if model.segments.len() != 25 || self.segments.len() != 25 {
            return Err("Incomplete anatomical mapping".into());
        }
        let basis = checked(self.source_to_native_basis)?;
        let mut world = [Rotation::IDENTITY; 25];
        let mut q = [[0.0; 3]; 25];
        for (i, (part, map)) in model.segments.iter().zip(&self.segments).enumerate() {
            if map.target != part.id {
                return Err("Mapping order/duplicate/unknown target".into());
            }
            let parent = part
                .parent
                .as_ref()
                .map(|id| {
                    model
                        .segments
                        .iter()
                        .position(|p| &p.id == id)
                        .ok_or("Missing anatomical parent")
                })
                .transpose()?;
            if parent.is_some_and(|j| j >= i) {
                return Err("Mapping parent topology".into());
            }
            world[i] = match &map.orientation {
                OrientationSource::World {
                    source: name,
                    offset,
                } => {
                    basis
                        * checked(
                            *source
                                .get(name)
                                .ok_or_else(|| format!("Missing source orientation {name}"))?,
                        )?
                        * checked(*offset)?
                }
                OrientationSource::ParentRelative { rotation: r } => {
                    let j = parent.ok_or("Root must have a source world orientation")?;
                    let p = part
                        .joint_profile
                        .as_ref()
                        .ok_or("Missing native joint profile")?;
                    world[j]
                        * rotation(p.parent_frame.rotation)
                        * checked(*r)?
                        * rotation(p.child_frame.rotation).inverse()
                }
            };
            if let Some(j) = parent {
                let p = part
                    .joint_profile
                    .as_ref()
                    .ok_or("Missing native joint profile")?;
                q[i] = measurement::coordinates(world[j], world[i], p);
                // Only remove roundoff on unmodeled axes; reject a real missing DOF.
                for k in 0..3 {
                    if !p.axes.iter().any(|a| a.coordinate == ['x', 'y', 'z'][k]) {
                        if q[i][k].abs() > 1e-5 {
                            return Err(format!("Unmodeled source rotation {}[{k}]", part.id));
                        }
                        q[i][k] = 0.0;
                    }
                }
            }
        }
        validate_command(
            model,
            &Command {
                positions: q,
                velocities: [[0.0; 3]; 25],
            },
        )?;
        Ok(q)
    }
}

#[test]
fn mapping_roundtrip_uses_native_joint_frames_and_coupled_quaternion_coordinates() {
    let model = Model::canonical().unwrap();
    for heading in [0.0_f32, 60.0_f32.to_radians()] {
        let basis = Rotation::from_rotation_y(heading);
        let mut native = [Rotation::IDENTITY; 25];
        let mut expected = [[0.0; 3]; 25];
        let mut sources = BTreeMap::new();
        let mut maps = Vec::new();
        for (i, part) in model.segments.iter().enumerate() {
            if let Some(p) = &part.joint_profile {
                let j = model
                    .segments
                    .iter()
                    .position(|s| Some(&s.id) == part.parent.as_ref())
                    .unwrap();
                for a in &p.axes {
                    let k = ['x', 'y', 'z']
                        .iter()
                        .position(|c| *c == a.coordinate)
                        .unwrap();
                    expected[i][k] = (a.min_radians + a.max_radians) * 0.1;
                }
                let t = expected[i].map(|v| (v * 0.5).tan());
                native[i] = native[j]
                    * rotation(p.parent_frame.rotation)
                    * Rotation::from_xyzw(t[0], t[1], t[2], 1.0).normalize()
                    * rotation(p.child_frame.rotation).inverse();
            }
            let offset = Rotation::from_rotation_z(0.19);
            let source = basis.inverse() * native[i] * offset.inverse();
            // Antipodal representations must yield the same native coordinates.
            sources.insert(
                part.id.clone(),
                crate::quat(if i % 2 == 0 { -source } else { source }),
            );
            maps.push(SegmentMapping {
                target: part.id.clone(),
                orientation: OrientationSource::World {
                    source: part.id.clone(),
                    offset: crate::quat(offset),
                },
            });
        }
        let mapping = Mapping {
            source_to_native_basis: crate::quat(basis),
            segments: maps,
        };
        let actual = mapping.coordinates(&model, &sources).unwrap();
        for (a, b) in actual.iter().flatten().zip(expected.iter().flatten()) {
            assert!((a - b).abs() < 1e-5);
        }
    }
}

#[test]
fn mapping_rejects_missing_source_bad_quaternion_and_unmodeled_rotation() {
    let model = Model::canonical().unwrap();
    let identity = crate::quat(Rotation::IDENTITY);
    let mut mapping = Mapping {
        source_to_native_basis: identity,
        segments: model
            .segments
            .iter()
            .map(|p| SegmentMapping {
                target: p.id.clone(),
                orientation: OrientationSource::ParentRelative { rotation: identity },
            })
            .collect(),
    };
    mapping.segments[0].orientation = OrientationSource::World {
        source: "root".into(),
        offset: identity,
    };
    let mut source = BTreeMap::new();
    assert!(mapping.coordinates(&model, &source).is_err());
    source.insert("root".into(), identity);
    assert!(mapping.coordinates(&model, &source).is_ok());
    mapping.source_to_native_basis.w = 2.0;
    assert!(mapping.coordinates(&model, &source).is_err());
    mapping.source_to_native_basis = identity;
    let i = model
        .segments
        .iter()
        .position(|p| p.joint_profile.as_ref().is_some_and(|j| j.axes.len() < 3))
        .unwrap();
    let p = model.segments[i].joint_profile.as_ref().unwrap();
    let k = (0..3)
        .find(|k| !p.axes.iter().any(|a| a.coordinate == ['x', 'y', 'z'][*k]))
        .unwrap();
    let axis = [
        rapier3d::prelude::Vector::X,
        rapier3d::prelude::Vector::Y,
        rapier3d::prelude::Vector::Z,
    ][k];
    mapping.segments[i].orientation = OrientationSource::ParentRelative {
        rotation: crate::quat(Rotation::from_axis_angle(axis, 0.2)),
    };
    assert!(mapping.coordinates(&model, &source).is_err());
}
