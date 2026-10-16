//! Unload a trapped arm onto measured back/leg contacts before folding it.
//! All poses here are motor intent; no body or contact is written.
use super::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub(super) enum RollArmStage {
    BackSupport,
    Fold,
    Roll,
}

#[derive(Clone, Serialize)]
pub(super) struct RollArm {
    pub side: usize,
    pub stage: RollArmStage,
    chain: [usize; 5],
    held: [[f32; 3]; 5],
    qualified_substeps: u64,
    root_goal: Option<Rotation>,
    root_legs: [bool; 2],
    released_legs: [bool; 2],
    contact_error_m: Option<f32>,
}

fn back_orientation(torso: Rotation) -> Rotation {
    let angle = (torso * Vector::X).y.atan2((torso * Vector::Z).y);
    torso * Rotation::from_rotation_y(angle * 0.18)
}

fn fold_targets(side: usize) -> [[f32; 3]; 5] {
    // Flex the shoulder and elbow above the chest, with a small lateral
    // clearance. The canonical shoulder frames already mirror flexion.
    let sign = if side == 0 { 1.0 } else { -1.0 };
    [
        [0.15, 0.0, 0.0],
        [1.2, 0.0, sign * 0.25],
        [1.8, 0.0, 0.0],
        [0.0; 3],
        [0.0; 3],
    ]
}

fn leg_chain(s: &Simulation, side: usize) -> [usize; 5] {
    let name = if side == 0 { "left" } else { "right" };
    ["Thigh", "Shin", "Ankle", "Foot", "Forefoot"].map(|part| index(s, &format!("{name}{part}")))
}

/// One material point drives IK; every currently loaded point verifies it.
/// This permits real foot/knee pivots but cannot silently drag a planted sole.
fn roll_leg_plan(
    s: &Simulation,
    side: usize,
    root: Pose,
    contacts: &[(PlantedAnchor, f32)],
) -> Result<Option<(LegPlan, f32)>, String> {
    let Some((anchor, _)) = contacts.iter().max_by(|a, b| a.1.total_cmp(&b.1)) else {
        return Ok(None);
    };
    let targets = planted_leg_targets(s, *anchor, root)?;
    let chain = leg_chain(s, side);
    let measured = measured_coordinates(s)?;
    let angles = chain.map(|i| {
        let q = targets
            .iter()
            .find(|(j, _)| *j == i)
            .map_or(measured[i], |(_, q)| *q);
        let profile = s.model.segments[i]
            .joint_profile
            .as_ref()
            .expect("leg profile");
        std::array::from_fn(|k| {
            profile
                .axes
                .iter()
                .find(|a| a.coordinate == ['x', 'y', 'z'][k])
                .map_or(0.0, |a| q[k].clamp(a.min_radians, a.max_radians))
        })
    });
    let mut error = 0.0_f32;
    for (k, &i) in chain.iter().enumerate() {
        let pose = crate::reaching::endpoint_pose(s, &chain[..=k], root, &angles[..=k])?;
        for (point, _) in contacts.iter().filter(|(p, _)| p.segment == i) {
            error = error.max(
                pose.transform_point(point.local_point)
                    .distance(point.world_point),
            );
        }
        let collider = &s.world.colliders[s.colliders[i]];
        let proposed = collider.shape().compute_aabb(&pose);
        let existing = collider.compute_aabb();
        let floor = s.surface_height(proposed.center().x, proposed.center().z, proposed.maxs.y);
        if proposed.mins.y < floor.min(existing.mins.y) - 0.001 {
            return Ok(None);
        }
    }
    if error > 0.003 {
        return Ok(None);
    }
    Ok(Some((chain.into_iter().zip(angles).collect(), error)))
}

fn roll_pose(s: &Simulation, returning: bool, lead: usize, fraction: f32) -> Pose {
    let measured = *body(s, "pelvis").position();
    let target = if returning {
        back_orientation(measured.rotation)
    } else {
        rolling_orientation(measured.rotation, lead)
    };
    let delta = (target * measured.rotation.inverse())
        .to_scaled_axis()
        .clamp_length_max(0.12)
        * fraction;
    let turn = Rotation::from_scaled_axis(delta);
    // A bounded virtual pivot under the pelvis gives the stance IK a lateral
    // displacement as well as a rotation. It is not asserted to be an appui.
    let pivot = Vector::new(
        measured.translation.x,
        s.support_height(),
        measured.translation.z,
    );
    let mut root = Pose::from_parts(
        pivot + turn * (measured.translation - pivot),
        turn * measured.rotation,
    );
    let pelvis_shape = s.world.colliders[s.colliders[0]].shape();
    let bottom = pelvis_shape.compute_aabb(&root).mins.y;
    root.translation.y += (s.support_height() - bottom).max(0.0).min(0.02);
    root
}

impl Controller {
    pub(super) fn update_roll_arm(
        &mut self,
        s: &Simulation,
        desired: &mut [[f32; 3]],
    ) -> Result<(), String> {
        if self.roll_arm.is_none() {
            let loads = ["leftUpperArm", "rightUpperArm"].map(|id| load(s, id));
            let side = usize::from(loads[1] >= loads[0]);
            if loads[side] < 3.0 {
                return Ok(());
            }
            let name = if side == 0 { "left" } else { "right" };
            let chain = [
                "ShoulderGirdle",
                "UpperArm",
                "Forearm",
                "ForearmTwist",
                "Hand",
            ]
            .map(|part| index(s, &format!("{name}{part}")));
            let measured = measured_coordinates(s)?;
            self.roll_arm = Some(RollArm {
                side,
                stage: RollArmStage::BackSupport,
                chain,
                held: chain.map(|i| measured[i]),
                qualified_substeps: 0,
                root_goal: None,
                root_legs: [false; 2],
                released_legs: [false; 2],
                contact_error_m: None,
            });
        }
        let arm = self.roll_arm.as_mut().ok_or("Missing roll arm")?;
        let release = supported_transfer_with(s, &arm.chain, true)?;
        let fold = fold_targets(arm.side);
        match arm.stage {
            RollArmStage::BackSupport => {
                // Back, pelvis, thighs and the other arm may support a roll.
                // They never become evidence for later knee/standing release.
                arm.qualified_substeps = if release {
                    arm.qualified_substeps + 1
                } else {
                    0
                };
                if arm.qualified_substeps
                    >= crate::recovery_contract::substeps(
                        crate::recovery_contract::contract()
                            .recovery_acceptance
                            .phase_minimum_s,
                    )
                {
                    arm.stage = RollArmStage::Fold;
                    self.hand_anchors[arm.side] = None;
                }
            }
            RollArmStage::Fold => {
                if !release {
                    // Stop advancing the swing reference if its real support
                    // disappears; reacquire it before any further arm motion.
                    arm.held = arm.chain.map(|i| self.targets[i]);
                    arm.stage = RollArmStage::BackSupport;
                    arm.qualified_substeps = 0;
                } else {
                    let measured = measured_coordinates(s)?;
                    let reached = arm.chain.iter().zip(fold).all(|(&i, target)| {
                        measured[i]
                            .iter()
                            .zip(target)
                            .all(|(q, target)| (q - target).abs() < 0.15)
                    });
                    let clear = arm.chain.iter().all(|&i| {
                        let aabb = s.world.colliders[s.colliders[i]].compute_aabb();
                        aabb.mins.y
                            > s.surface_height(aabb.center().x, aabb.center().z, aabb.maxs.y)
                                + 0.015
                            && load(s, &s.model.segments[i].id) < 3.0
                    });
                    if reached && clear {
                        arm.stage = RollArmStage::Roll;
                    }
                }
            }
            RollArmStage::Roll => {}
        }
        let arm_targets = if arm.stage == RollArmStage::BackSupport {
            arm.held
        } else {
            fold
        };
        for (&i, angles) in arm.chain.iter().zip(arm_targets) {
            desired[i] = angles;
        }
        if arm.stage != RollArmStage::BackSupport {
            // The generic bracing loop must not reacquire a grazing palm while
            // the support proof authorizes this whole arm's swing.
            self.hand_anchors[arm.side] = None;
        }
        // The trunk follows the pelvis through its own admissible local
        // coordinates. Never give both spine joints the torso's world pose.
        for id in ["lumbar", "torso"] {
            let i = index(s, id);
            desired[i] = s.rest_targets[i];
        }
        self.coordinate_roll_legs(s, desired)?;
        Ok(())
    }

    fn coordinate_roll_legs(
        &mut self,
        s: &Simulation,
        desired: &mut [[f32; 3]],
    ) -> Result<(), String> {
        let arm = self.roll_arm.as_mut().ok_or("Missing roll leg owner")?;
        arm.root_goal = None;
        arm.root_legs = [false; 2];
        arm.contact_error_m = None;
        let measured = measured_coordinates(s)?;
        let chains = [leg_chain(s, 0), leg_chain(s, 1)];
        let mut contacts = [
            planted_contacts(s, &chains[0])?,
            planted_contacts(s, &chains[1])?,
        ];
        for side in 0..2 {
            // An actual loaded shin earns the replacement support after swing.
            if arm.released_legs[side] && load(s, &s.model.segments[chains[side][1]].id) >= 3.0 {
                arm.released_legs[side] = false;
                self.knee_anchors[side] = planted_contact(s, &[chains[side][1]])?;
            }
            for &i in &chains[side] {
                desired[i] = measured[i];
            }
            if arm.released_legs[side] {
                contacts[side].clear();
                desired[chains[side][0]] = [1.2, 0.0, if side == 0 { 0.15 } else { -0.15 }];
                desired[chains[side][1]] = [1.8, 0.0, 0.0];
            }
        }
        let returning = arm.stage != RollArmStage::Roll;
        for fraction in [1.0, 0.5, 0.25, 0.125] {
            let root = roll_pose(s, returning, self.lead, fraction);
            let mut plans = Vec::new();
            let mut worst = 0.0_f32;
            let mut feasible = true;
            for side in 0..2 {
                if contacts[side].is_empty() {
                    continue;
                }
                match roll_leg_plan(s, side, root, &contacts[side])? {
                    Some((plan, error)) => {
                        plans.push((side, plan));
                        worst = worst.max(error);
                    }
                    None => {
                        feasible = false;
                        break;
                    }
                }
            }
            if feasible && !plans.is_empty() {
                arm.root_goal = Some(root.rotation);
                arm.contact_error_m = Some(worst);
                for (side, plan) in plans {
                    arm.root_legs[side] = true;
                    for (i, q) in plan {
                        desired[i] = q;
                    }
                }
                return Ok(());
            }
        }
        // A fixed-foot trajectory can exhaust hip range before prone. Release
        // one whole leg only after the same real-load/capture exclusion proof;
        // the other leg must retain a native path for the rolling effort.
        for side in [arm.side, 1 - arm.side] {
            if !arm.released_legs[side]
                && !contacts[1 - side].is_empty()
                && supported_transfer_with(s, &chains[side], true)?
            {
                arm.released_legs[side] = true;
                self.knee_anchors[side] = None;
                self.foot_anchors[side] = None;
                break;
            }
        }
        Ok(())
    }

    pub(super) fn roll_root_goal(&self) -> Option<Rotation> {
        self.roll_arm.as_ref().and_then(|arm| arm.root_goal)
    }
    pub(super) fn roll_root_legs(&self) -> [bool; 2] {
        self.roll_arm
            .as_ref()
            .map_or([false; 2], |arm| arm.root_legs)
    }
    pub fn roll_coordination(&self) -> Option<lh_contracts::RollCoordinationObservation> {
        self.roll_arm
            .as_ref()
            .map(|arm| lh_contracts::RollCoordinationObservation {
                pelvis_target: arm.root_goal.map(crate::quat),
                support_legs: arm.root_legs,
                released_legs: arm.released_legs,
                max_contact_error_m: arm.contact_error_m,
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn landed() -> Simulation {
        // Archived browser strike at 4.6667 s. Velocities are zeroed only for
        // this non-integrating geometry fixture, never for a physical run.
        let fixtures: FixtureSet =
            serde_json::from_str(include_str!("../../data/roll-landed-v1.json")).unwrap();
        Simulation::build(
            0.0,
            Profile::default(),
            true,
            None,
            Some(&fixtures.cases[0].poses),
        )
        .unwrap()
    }

    #[test]
    fn pelvis_roll_first_step_preserves_ankle_and_toe_points_within_bounds() {
        let s = landed();
        let before = serde_json::to_value(s.snapshot()).unwrap();
        // Hypothetical material points test the kinematics, not physical load.
        // Production authority requires planted_contacts' real solved impulses.
        let contacts: Vec<_> = [index(&s, "rightAnkle"), index(&s, "rightForefoot")]
            .into_iter()
            .map(|i| {
                let pose = s.world.bodies[s.bodies[i]].position();
                let local = s.model.segments[i]
                    .geometry
                    .vertices
                    .chunks_exact(3)
                    .map(|p| Vector::new(p[0], p[1], p[2]))
                    .min_by(|a, b| {
                        pose.transform_point(*a)
                            .y
                            .total_cmp(&pose.transform_point(*b).y)
                    })
                    .unwrap();
                (
                    PlantedAnchor {
                        segment: i,
                        local_point: local,
                        world_point: pose.transform_point(local),
                    },
                    1.0,
                )
            })
            .collect();
        let mut accepted = None;
        for fraction in [1.0, 0.5, 0.25, 0.125] {
            let root = roll_pose(&s, true, 1, fraction);
            if let Some((plan, error)) = roll_leg_plan(&s, 1, root, &contacts).unwrap() {
                accepted = Some((root, plan, error));
                break;
            }
        }
        let (root, plan, error) = accepted.expect("Initial back rotation has a planted-leg path");
        println!(
            "Recorded-pose root step: rotation={} rad, translation={} m, contact error={} m",
            (root.rotation * body(&s, "pelvis").rotation().inverse())
                .to_scaled_axis()
                .length(),
            root.translation.distance(body(&s, "pelvis").translation()),
            error
        );
        assert!(error <= 0.003);
        let pelvis = body(&s, "pelvis");
        assert!((root.rotation * Vector::Z).y > (*pelvis.rotation() * Vector::Z).y);
        assert!(root.translation.distance(pelvis.translation()) < 0.04);
        for (i, q) in plan {
            for axis in &s.model.segments[i].joint_profile.as_ref().unwrap().axes {
                let k = ['x', 'y', 'z']
                    .iter()
                    .position(|c| *c == axis.coordinate)
                    .unwrap();
                assert!((axis.min_radians..=axis.max_radians).contains(&q[k]));
            }
        }
        assert_eq!(before, serde_json::to_value(s.snapshot()).unwrap());
    }

    #[test]
    fn pelvis_roll_rejects_unreachable_fixed_foot_continuation() {
        let s = landed();
        let i = index(&s, "rightForefoot");
        let pose = s.world.bodies[s.bodies[i]].position();
        let local = Vector::new(0.0, -0.045, 0.06);
        let anchor = PlantedAnchor {
            segment: i,
            local_point: local,
            world_point: pose.transform_point(local),
        };
        let measured = *body(&s, "pelvis").position();
        let root = Pose::from_parts(
            measured.translation,
            measured.rotation * Rotation::from_rotation_y(2.44),
        );
        assert!(
            roll_leg_plan(&s, 1, root, &[(anchor, 1.0)])
                .unwrap()
                .is_none(),
            "Continuing to prone requires a support change, not a dragged sole"
        );
    }

    #[test]
    fn pelvis_roll_replaces_saturated_spine_intent_and_requires_solved_leg_contacts() {
        let mut s = landed();
        let upper = index(&s, "rightUpperArm");
        s.integrated_support_substeps[upper] = 240;
        s.integrated_support_loads_n[upper] = 250.0;
        let before = serde_json::to_value(s.snapshot()).unwrap();
        let mut c = Controller::new(&s).unwrap();
        let mut desired = s.rest_targets.clone();
        c.update_roll_arm(&s, &mut desired).unwrap();
        for id in ["lumbar", "torso"] {
            let i = index(&s, id);
            assert_eq!(desired[i], s.rest_targets[i]);
        }
        assert!(c.roll_root_goal().is_none());
        for id in ["leftThigh", "rightThigh"] {
            let part = &s.model.segments[index(&s, id)];
            assert!(!c.native_root_authority(&s, part));
            assert_eq!(c.root_torque(&s, part), Vector::ZERO);
        }
        assert_eq!(before, serde_json::to_value(s.snapshot()).unwrap());
    }

    #[test]
    fn arm_transfer_back_route_preserves_axis_and_lifts_loaded_side() {
        let torso = Rotation::from_xyzw(-0.68539315, 0.325458, 0.10550473, 0.6427924);
        let back = back_orientation(torso);
        assert!((back * Vector::Y - torso * Vector::Y).length() < 1e-6);
        assert!((back * Vector::Z).y > (torso * Vector::Z).y);
        assert!((back * Vector::X).y.abs() < (torso * Vector::X).y.abs());
    }

    #[test]
    fn arm_transfer_fold_is_bounded_and_clears_both_arms_on_back() {
        let s = Simulation::new(0.0, Profile::default(), true).unwrap();
        let root = Pose::from_parts(
            Vector::new(0.0, 0.14, 0.0),
            Rotation::from_rotation_x(-std::f32::consts::FRAC_PI_2),
        );
        let before = serde_json::to_value(s.snapshot()).unwrap();
        for (side, name) in ["left", "right"].into_iter().enumerate() {
            let chain = [
                "ShoulderGirdle",
                "UpperArm",
                "Forearm",
                "ForearmTwist",
                "Hand",
            ]
            .map(|part| index(&s, &format!("{name}{part}")));
            let angles = fold_targets(side);
            for (k, &i) in chain.iter().enumerate() {
                for axis in &s.model.segments[i].joint_profile.as_ref().unwrap().axes {
                    assert!(
                        (axis.min_radians..=axis.max_radians).contains(
                            &angles[k][['x', 'y', 'z']
                                .iter()
                                .position(|c| *c == axis.coordinate)
                                .unwrap()]
                        )
                    );
                }
                let pose =
                    crate::reaching::endpoint_pose(&s, &chain[..=k], root, &angles[..=k]).unwrap();
                let shape = s.world.colliders[s.colliders[i]].shape();
                assert!(shape.compute_aabb(&pose).mins.y > 0.015);
            }
        }
        assert_eq!(before, serde_json::to_value(s.snapshot()).unwrap());
    }

    #[test]
    fn arm_transfer_cannot_swing_without_measured_alternate_contacts() {
        let mut s = Simulation::new(0.0, Profile::default(), true).unwrap();
        let i = index(&s, "rightUpperArm");
        s.integrated_support_substeps[i] = 240;
        s.integrated_support_loads_n[i] = 250.0;
        let mut c = Controller::new(&s).unwrap();
        c.enter(Phase::Roll);
        let before = serde_json::to_value(s.snapshot()).unwrap();
        let mut desired = s.rest_targets.clone();
        for _ in 0..100 {
            c.update_roll_arm(&s, &mut desired).unwrap();
        }
        let arm = c.roll_arm.unwrap();
        assert_eq!(arm.stage, RollArmStage::BackSupport);
        assert_eq!(arm.qualified_substeps, 0);
        for (&i, held) in arm.chain.iter().zip(arm.held) {
            assert_eq!(desired[i], held);
        }
        assert_eq!(before, serde_json::to_value(s.snapshot()).unwrap());
    }
}
