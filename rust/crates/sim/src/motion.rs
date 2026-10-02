//! Observe falls from integrated poses and measured support, never from a UI timer.
//! Recovery completion requires integrated standing and persistent measured support.
use crate::Simulation;
use lh_contracts::Motion;
use rapier3d::prelude::Vector;

#[derive(Default)]
pub(crate) struct Observation {
    unsupported_s: f32,
    settled_s: f32,
}

impl Simulation {
    pub(crate) fn observe_impact(&mut self) {
        if matches!(self.motion, Motion::Upright | Motion::Reacting) {
            self.counters.falls += 1;
        }
        self.motion = Motion::Falling;
        self.recovery = None;
        self.stepping = None;
        self.motion_observation = Observation::default();
        self.cancel_grab();
    }
    pub(crate) fn observe_motion(&mut self, loads_n: &[f32; 25]) {
        if self.motion == Motion::Halted {
            return;
        }
        if self.motion == Motion::Recovering {
            self.observe_recovery();
            return;
        }
        // Calibration also uses an eleven-body loaded leg assembly. Whole-body
        // motion labels do not apply to that deliberately reduced topology.
        let Some(torso) = self
            .model
            .segments
            .iter()
            .position(|part| part.id == "torso")
        else {
            return;
        };
        let foot_support = self.model.segments.iter().enumerate().any(|(i, part)| {
            matches!(part.role.as_str(), "hindfoot" | "forefoot") && loads_n[i] >= 3.0
        });
        self.motion_observation.unsupported_s = if foot_support {
            0.0
        } else {
            self.motion_observation.unsupported_s + self.profile.dt_s
        };
        if matches!(self.motion, Motion::Upright | Motion::Reacting) {
            let pelvis = &self.world.bodies[self.bodies[0]];
            let torso_up = (*self.world.bodies[self.bodies[torso]].rotation() * Vector::Y).y;
            let support_loss_limit = if self.environment.is_some()
                && self.world.colliders[self.floor].is_enabled()
                && self.substeps_completed < 144
            {
                // Inherited playground spawn grace: settle above the complete
                // sloped footprint before requiring a persistent support load.
                0.5
            } else if self
                .stepping
                .as_ref()
                .is_some_and(|step| step.status.phase == "swing")
            {
                0.36
            } else {
                0.14
            };
            // Inherited flat-floor fall criteria; thresholds are not tuned to a trial.
            if pelvis.translation().y - self.support_height() < 0.56
                || torso_up < 1.25_f32.cos()
                || self.motion_observation.unsupported_s > support_loss_limit
            {
                self.motion = Motion::Falling;
                self.counters.falls += 1;
                self.cancel_grab();
            } else {
                self.motion = if self.grab.is_some() {
                    Motion::Reacting
                } else {
                    Motion::Upright
                };
            }
        }
        if self.motion == Motion::Falling {
            let landed = self.model.segments.iter().enumerate().any(|(i, part)| {
                !matches!(part.role.as_str(), "hindfoot" | "forefoot") && loads_n[i] >= 3.0
            });
            let settled = self.bodies.iter().all(|handle| {
                let body = &self.world.bodies[*handle];
                body.linvel().length() <= 0.65 && body.angvel().length() <= 1.8
            });
            self.motion_observation.settled_s = if landed && settled {
                self.motion_observation.settled_s + self.profile.dt_s
            } else {
                0.0
            };
            if self.motion_observation.settled_s >= 0.3 {
                self.motion = Motion::Fallen;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{Profile, Simulation};
    use lh_contracts::Motion;

    #[test]
    fn loaded_leg_calibration_remains_valid_without_a_torso() {
        let reports = crate::calibration::loaded_leg_chains().unwrap();
        assert_eq!(reports.len(), 3);
        assert!(reports.iter().all(|report| report["passed"] == true));
    }

    #[test]
    fn unsupported_fall_is_observed_once_and_cancels_grab_without_pose_writes() {
        let mut s = Simulation::new(0.0, Profile::default(), false).unwrap();
        let hand = s
            .model
            .segments
            .iter()
            .position(|part| part.id == "rightHand")
            .unwrap();
        let target = crate::vec3(s.world.bodies[s.bodies[hand]].translation());
        s.begin_grab(hand, 1, lh_model::Vec3::default(), target)
            .unwrap();
        for _ in 0..12 {
            s.advance_tick().unwrap();
        }
        assert_eq!(s.motion, Motion::Falling);
        assert_eq!(s.counters.falls, 1);
        assert!(s.grab.is_none());
        assert!(
            s.begin_grab(hand, 2, lh_model::Vec3::default(), target)
                .is_err()
        );
        let before = s.snapshot();
        for _ in 0..4 {
            s.advance_tick().unwrap();
        }
        assert_eq!(s.counters.falls, 1);
        assert_eq!(s.counters.recoveries, 0);
        assert!(s.snapshot().segments[0].position.y < before.segments[0].position.y);
    }
}
