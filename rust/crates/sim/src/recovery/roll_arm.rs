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
        if arm.stage != RollArmStage::Roll {
            let route = back_orientation(*body(s, "torso").rotation());
            for id in ["lumbar", "torso"] {
                let i = index(s, id);
                let joint = s
                    .world
                    .impulse_joints
                    .get(s.joints[i].ok_or("Missing roll joint")?)
                    .ok_or("Lost roll joint")?;
                desired[i] = measurement::coordinates(
                    *s.world.bodies[joint.body1()].rotation(),
                    route,
                    s.model.segments[i]
                        .joint_profile
                        .as_ref()
                        .ok_or("Missing roll profile")?,
                );
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
