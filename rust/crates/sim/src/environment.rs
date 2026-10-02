//! Canonical course solids and kinematic decks. Dynamic anatomy is never moved.
use crate::{quat, rotation, vec3, vector};
use lh_contracts::{Difficulty, EnvironmentSnapshot, PlaygroundSettings, Station};
use lh_model::{CoursePiece, Model};
use rapier3d::prelude::*;
use std::collections::HashSet;

pub const STATIONS: [Station; 7] = [
    Station::Flat,
    Station::Slope,
    Station::Rubble,
    Station::Beam,
    Station::Stones,
    Station::Wobble,
    Station::Hurdles,
];
pub const DIFFICULTIES: [Difficulty; 3] = [
    Difficulty::Gentle,
    Difficulty::Challenging,
    Difficulty::Extreme,
];

/// Structural and measured-contact evidence for every authored station. This
/// short diagnostic is not stepping, get-up, performance or release admission.
pub fn trials(profile: &crate::Profile) -> Result<Vec<serde_json::Value>, String> {
    let mut reports = Vec::new();
    for difficulty in DIFFICULTIES {
        for station in STATIONS {
            let settings = PlaygroundSettings {
                station,
                difficulty,
            };
            let mut s = crate::Simulation::new_playground(0.0, profile.clone(), true, settings)?;
            let initial = s.snapshot();
            let handles = s.bodies.clone();
            let mut terrain_load_n = 0.0_f32;
            let mut failure = None;
            for _ in 0..120 {
                if let Err(error) = s.advance_tick() {
                    failure = Some(error);
                    break;
                }
                for collider in &s.colliders {
                    for pair in s.world.narrow_phase.contact_pairs_with(*collider) {
                        let other = if pair.collider1 == *collider {
                            pair.collider2
                        } else {
                            pair.collider1
                        };
                        if s.environment.as_ref().unwrap().contains(other) {
                            terrain_load_n = terrain_load_n.max(
                                crate::measurement::support_normal_impulse(pair, *collider)?
                                    / s.profile.dt_s,
                            );
                        }
                    }
                }
            }
            let ownership =
                handles == s.bodies && s.bodies.iter().all(|h| s.world.bodies[*h].is_dynamic());
            let starts_on_terrain = !matches!(station, Station::Flat | Station::Hurdles);
            reports.push(serde_json::json!({"settings":settings,"initial":initial,"final":s.snapshot(),
            "failure":failure,"peakTerrainLoadN":terrain_load_n,"dynamicOwnership":ownership,
            "startsOnTerrain":starts_on_terrain,
            "passed":failure.is_none() && ownership && (!starts_on_terrain || terrain_load_n >= 3.0)}));
        }
    }
    Ok(reports)
}

pub fn difficulty_name(value: Difficulty) -> &'static str {
    match value {
        Difficulty::Gentle => "gentle",
        Difficulty::Challenging => "challenging",
        Difficulty::Extreme => "extreme",
    }
}

pub fn station_name(value: Station) -> &'static str {
    match value {
        Station::Flat => "flat",
        Station::Slope => "slope",
        Station::Rubble => "rubble",
        Station::Beam => "beam",
        Station::Stones => "stones",
        Station::Wobble => "wobble",
        Station::Hurdles => "hurdles",
    }
}
pub(crate) struct Environment {
    pub settings: PlaygroundSettings,
    pieces: Vec<(CoursePiece, RigidBodyHandle)>,
    pub colliders: Vec<ColliderHandle>,
    handles: HashSet<ColliderHandle>,
}
impl Environment {
    pub fn new(
        world: &mut PhysicsWorld,
        model: &Model,
        settings: PlaygroundSettings,
    ) -> Result<Self, String> {
        let definitions = model.course(difficulty_name(settings.difficulty))?;
        let mut pieces = Vec::new();
        let mut colliders = Vec::new();
        for definition in definitions {
            let builder = if definition.motion.is_some() {
                RigidBodyBuilder::kinematic_position_based()
            } else {
                RigidBodyBuilder::fixed()
            };
            let body = world.bodies.insert(builder.pose(Pose::from_parts(
                vector(definition.position),
                rotation(definition.rotation_at(0.0)),
            )));
            // Match the retained app: convex triangular prisms keep every
            // rubble top triangle and report actual solver contact impulses.
            let hulls = if definition.tiled {
                definition.geometry.triangles[..definition.geometry.surface_triangles.unwrap()]
                    .iter()
                    .map(|triangle| {
                        let top: Vec<_> = triangle
                            .iter()
                            .map(|i| vector(definition.geometry.vertices[*i as usize]))
                            .collect();
                        top.iter()
                            .copied()
                            .chain(top.iter().map(|v| Vector::new(v.x, 0.0, v.z)))
                            .collect::<Vec<_>>()
                    })
                    .collect::<Vec<_>>()
            } else {
                vec![
                    definition
                        .geometry
                        .vertices
                        .iter()
                        .map(|v| vector(*v))
                        .collect(),
                ]
            };
            for hull in hulls {
                let descriptor = ColliderBuilder::convex_hull(&hull)
                    .ok_or_else(|| format!("Invalid course hull {}", definition.id))?
                    .friction(definition.friction)
                    .restitution(0.01);
                colliders.push(world.colliders.insert_with_parent(
                    descriptor,
                    body,
                    &mut world.bodies,
                ));
            }
            pieces.push((definition, body));
        }
        let handles = colliders.iter().copied().collect();
        Ok(Self {
            settings,
            pieces,
            colliders,
            handles,
        })
    }
    pub fn contains(&self, handle: ColliderHandle) -> bool {
        self.handles.contains(&handle)
    }
    pub fn update(&self, world: &mut PhysicsWorld, time_s: f32) {
        for (piece, handle) in &self.pieces {
            if piece.motion.is_some() {
                world.bodies[*handle].set_next_kinematic_position(Pose::from_parts(
                    vector(piece.position),
                    rotation(piece.rotation_at(time_s)),
                ));
            }
        }
    }
    pub fn height_at(&self, world: &PhysicsWorld, x: f32, z: f32, ceiling: f32) -> f32 {
        let ray = Ray::new(Vector::new(x, ceiling, z), -Vector::Y);
        self.colliders
            .iter()
            .filter_map(|handle| {
                let collider = &world.colliders[*handle];
                if !collider.is_enabled() {
                    return None;
                }
                let bounds = collider.compute_aabb();
                if x < bounds.mins.x || x > bounds.maxs.x || z < bounds.mins.z || z > bounds.maxs.z
                {
                    return None;
                }
                collider
                    .shape()
                    .cast_ray_and_get_normal(collider.position(), &ray, ceiling.max(0.0), false)
                    .filter(|hit| hit.normal.y > 0.1)
                    .map(|hit| ceiling - hit.time_of_impact)
            })
            .fold(0.0, f32::max)
    }
    pub fn spawn_offset(&self, world: &PhysicsWorld, model: &Model) -> Result<Vector, String> {
        let position = vector(model.station(station_name(self.settings.station))?);
        let mut height = 0.0_f32;
        for x in [-0.28, 0.0, 0.28] {
            for z in [-0.16, 0.1, 0.28] {
                height = height.max(self.height_at(world, position.x + x, position.z + z, 5.0));
            }
        }
        Ok(Vector::new(position.x, height, position.z))
    }
    pub fn snapshot(&self, world: &PhysicsWorld) -> EnvironmentSnapshot {
        EnvironmentSnapshot {
            settings: self.settings,
            pieces: self
                .pieces
                .iter()
                .map(|(piece, handle)| {
                    let b = &world.bodies[*handle];
                    lh_model::Pose {
                        id: piece.id.clone(),
                        position: vec3(b.translation()),
                        rotation: quat(*b.rotation()),
                        linear_velocity: vec3(b.linvel()),
                        angular_velocity: vec3(b.angvel()),
                    }
                })
                .collect(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Profile, Simulation};
    use lh_contracts::{Action, Command, Mode, Stamp};

    #[test]
    fn all_course_variants_spawn_connected_anatomy_over_the_selected_surface() {
        for difficulty in DIFFICULTIES {
            for station in STATIONS {
                let settings = PlaygroundSettings {
                    station,
                    difficulty,
                };
                let s =
                    Simulation::new_playground(0.0, Profile::default(), true, settings).unwrap();
                let environment = s.environment.as_ref().unwrap();
                let expected = environment.spawn_offset(&s.world, &s.model).unwrap();
                let root = s.world.bodies[s.bodies[0]].translation();
                assert!(
                    (root - vector(s.model.segments[0].initial.position) - expected).length()
                        < 1e-6
                );
                assert_eq!(s.bodies.len(), 25);
                assert_eq!(environment.snapshot(&s.world).pieces.len(), 26);
                assert!(environment.colliders.len() >= 26);
                assert!(s.bodies.iter().all(|h| s.world.bodies[*h].is_dynamic()));
                if !matches!(station, Station::Flat | Station::Hurdles) {
                    assert!(expected.y > 0.01, "{settings:?}");
                } else {
                    assert_eq!(expected.y, 0.0, "{settings:?}");
                }
                for (i, joint) in s.joints.iter().enumerate() {
                    if let Some(handle) = joint {
                        let joint = s.world.impulse_joints.get(*handle).unwrap();
                        let profile = s.model.segments[i].joint_profile.as_ref().unwrap();
                        let a = s.world.bodies[joint.body1()]
                            .position()
                            .transform_point(vector(profile.parent_frame.anchor));
                        let b = s.world.bodies[joint.body2()]
                            .position()
                            .transform_point(vector(profile.child_frame.anchor));
                        assert!(a.distance(b) < 1e-5);
                    }
                }
            }
        }
    }

    #[test]
    fn station_and_difficulty_commands_reset_atomically_and_preserve_pause() {
        let mut r =
            crate::runtime::Runtime::new(Mode::Playground, 0.0, Profile::default()).unwrap();
        r.enqueue(Command {
            stamp: Stamp::new(1, 1, 0),
            action: Action::Pause,
        })
        .unwrap();
        r.advance(0).unwrap();
        r.enqueue(Command {
            stamp: Stamp::new(1, 2, 0),
            action: Action::SelectStation {
                station: Station::Beam,
            },
        })
        .unwrap();
        let reply = r.advance(0).unwrap();
        assert_eq!(reply.snapshot.stamp.generation, 2);
        assert_eq!(reply.snapshot.stamp.tick, 0);
        assert!(reply.paused && reply.acknowledgements[0].rejected.is_none());
        assert_eq!(
            reply.snapshot.environment.unwrap().settings.station,
            Station::Beam
        );
        r.enqueue(Command {
            stamp: Stamp::new(2, 1, 0),
            action: Action::SetDifficulty {
                difficulty: Difficulty::Extreme,
            },
        })
        .unwrap();
        let reply = r.advance(0).unwrap();
        assert_eq!(reply.snapshot.stamp.generation, 3);
        assert!(reply.paused);
        let settings = reply.snapshot.environment.unwrap().settings;
        assert_eq!(
            settings,
            PlaygroundSettings {
                station: Station::Beam,
                difficulty: Difficulty::Extreme
            }
        );
        assert!(
            r.enqueue(Command {
                stamp: Stamp::new(2, 2, 0),
                action: Action::Resume
            })
            .is_err()
        );
        assert!(r.enable_quiet_measurement().is_err());
    }

    #[test]
    fn moving_deck_is_driven_by_integrated_time_and_does_not_write_anatomical_poses() {
        let mut world = PhysicsWorld::default();
        let model = Model::canonical().unwrap();
        let e = Environment::new(&mut world, &model, PlaygroundSettings::default()).unwrap();
        let (definition, handle) = e.pieces.iter().find(|(d, _)| d.motion.is_some()).unwrap();
        let initial = *world.bodies[*handle].position();
        for step in 1..=960 {
            e.update(&mut world, step as f32 / 240.0);
            world.integration_parameters.dt = 1.0 / 240.0;
            world.step();
        }
        let measured = &world.bodies[*handle];
        assert!(
            (measured
                .rotation()
                .dot(rotation(definition.rotation_at(4.0)))
                .abs()
                - 1.0)
                .abs()
                < 1e-5
        );
        assert_eq!(measured.translation(), initial.translation);
        assert!(measured.angvel().length() > 0.001);
        assert!(!measured.is_dynamic());
    }
}
