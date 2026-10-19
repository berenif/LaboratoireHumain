//! A presentation-only interpolation clock. It never advances physical state.

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct PresentationClock {
    initialized: bool,
    arrival_ms: f64,
    interval_s: f64,
    previous_time_s: f64,
    target_time_s: f64,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct PresentationSample {
    pub alpha: f32,
    pub time_s: f64,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct PresentationRetarget {
    pub snap: bool,
    pub old_alpha: f32,
}

impl PresentationClock {
    pub fn sample(&self, now_ms: f64) -> Result<PresentationSample, &'static str> {
        if !now_ms.is_finite() {
            return Err("Invalid presentation timestamp");
        }
        // A rAF timestamp may precede an intervening snapshot's performance.now.
        // Clamp that elapsed interval rather than extrapolating either endpoint.
        let alpha = if self.interval_s > 0.0 {
            ((now_ms - self.arrival_ms) / 1000.0 / self.interval_s).clamp(0.0, 1.0) as f32
        } else {
            1.0
        };
        Ok(PresentationSample {
            alpha,
            time_s: self.previous_time_s
                + (self.target_time_s - self.previous_time_s) * f64::from(alpha),
        })
    }

    pub fn retarget(
        &mut self,
        target_time_s: f64,
        now_ms: f64,
        force_snap: bool,
    ) -> Result<PresentationRetarget, &'static str> {
        if !target_time_s.is_finite() || target_time_s < 0.0 || !now_ms.is_finite() {
            return Err("Invalid presentation timestamp");
        }
        // Sample the OLD blend, before replacing its duration or arrival. This
        // must not depend on whether a GPU frame happened between two arrivals.
        let old = self.sample(now_ms)?;
        let interval_s = if self.initialized {
            (target_time_s - self.target_time_s).max(0.0)
        } else {
            0.0
        };
        let snap = force_snap || !self.initialized || interval_s == 0.0;
        self.previous_time_s = if snap { target_time_s } else { old.time_s };
        self.target_time_s = target_time_s;
        self.interval_s = interval_s;
        self.arrival_ms = now_ms;
        self.initialized = true;
        Ok(PresentationRetarget {
            snap,
            old_alpha: old.alpha,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Transform;
    use glam::{Quat, Vec3};

    fn close(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-8, "{a} != {b}");
    }

    #[test]
    fn arrivals_advance_the_old_blend_even_without_an_intervening_draw() {
        let mut c = PresentationClock::default();
        assert!(c.retarget(0.0, 0.0, false).unwrap().snap);
        c.retarget(0.025, 25.0, false).unwrap();
        close(c.sample(30.0).unwrap().time_s, 0.005);
        let next = c.retarget(0.050, 35.0, false).unwrap();
        close(f64::from(next.old_alpha), 0.4);
        close(c.sample(35.0).unwrap().time_s, 0.010);
        c.retarget(0.075, 40.0, false).unwrap();
        close(c.sample(40.0).unwrap().time_s, 0.018);
        close(c.sample(45.0).unwrap().time_s, 0.0294);
    }

    #[test]
    fn inserting_or_removing_draw_samples_cannot_change_later_presentation() {
        let mut sparse = PresentationClock::default();
        let mut dense = sparse;
        for (time, now) in [(0.0, 0.0), (0.025, 25.0), (0.05, 35.0), (0.075, 40.0)] {
            for draw in [now - 3.0, now - 2.0, now - 1.0] {
                dense.sample(draw).unwrap();
            }
            sparse.retarget(time, now, false).unwrap();
            dense.retarget(time, now, false).unwrap();
            assert_eq!(sparse, dense);
        }
        assert_eq!(
            sparse.sample(45.0).unwrap().alpha,
            dense.sample(45.0).unwrap().alpha
        );
        close(
            sparse.sample(45.0).unwrap().time_s,
            dense.sample(45.0).unwrap().time_s,
        );
    }

    #[test]
    fn same_timestamp_bursts_preserve_the_sampled_origin_and_newest_target() {
        let mut c = PresentationClock::default();
        c.retarget(0.0, 0.0, false).unwrap();
        c.retarget(0.025, 25.0, false).unwrap();
        c.retarget(0.05, 35.0, false).unwrap();
        let origin = c.sample(35.0).unwrap().time_s;
        for time in [0.075, 0.100, 0.125] {
            c.retarget(time, 35.0, false).unwrap();
            close(c.sample(35.0).unwrap().time_s, origin);
        }
        close(c.sample(60.0).unwrap().time_s, 0.125);
    }

    #[test]
    fn unequal_intervals_use_the_old_duration_then_consecutive_target_times() {
        let mut c = PresentationClock::default();
        c.retarget(0.0, 0.0, false).unwrap();
        c.retarget(0.1, 100.0, false).unwrap();
        c.retarget(0.125, 150.0, false).unwrap();
        close(c.sample(150.0).unwrap().time_s, 0.05);
        close(c.sample(162.5).unwrap().time_s, 0.0875);
        close(c.sample(175.0).unwrap().time_s, 0.125);
    }

    #[test]
    fn samples_clamp_to_observed_endpoints_without_extrapolating() {
        let mut c = PresentationClock::default();
        c.retarget(1.0, 1000.0, false).unwrap();
        c.retarget(1.025, 1025.0, false).unwrap();
        for now in [0.0, 1024.0, 1025.0] {
            close(c.sample(now).unwrap().time_s, 1.0);
        }
        for now in [1050.0, 1100.0, 1e12] {
            close(c.sample(now).unwrap().time_s, 1.025);
        }
    }

    #[test]
    fn pause_reset_discontinuity_geometry_and_zero_time_share_snap_policy() {
        for reason in ["pause", "reset", "discontinuity", "geometry"] {
            let mut c = PresentationClock::default();
            c.retarget(1.0, 1000.0, false).unwrap();
            c.retarget(1.025, 1025.0, false).unwrap();
            let target = if reason == "reset" { 0.0 } else { 1.05 };
            assert!(c.retarget(target, 1030.0, true).unwrap().snap);
            close(c.sample(1030.0).unwrap().time_s, target);
        }
        let mut c = PresentationClock::default();
        c.retarget(1.0, 1000.0, false).unwrap();
        assert!(c.retarget(1.0, 1010.0, false).unwrap().snap);
        close(c.sample(1010.0).unwrap().time_s, 1.0);
        assert!(c.retarget(0.5, 1020.0, false).unwrap().snap);
        close(c.sample(1020.0).unwrap().time_s, 0.5);
    }

    #[test]
    fn invalid_clocks_fail_before_mutation() {
        let mut c = PresentationClock::default();
        c.retarget(1.0, 1000.0, false).unwrap();
        let before = c;
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(c.sample(bad).is_err());
            assert!(c.retarget(2.0, bad, false).is_err());
            assert!(c.retarget(bad, 1010.0, false).is_err());
            assert_eq!(c, before);
        }
        assert!(c.retarget(-1.0, 1010.0, false).is_err());
        assert_eq!(c, before);
    }

    #[test]
    fn shared_alpha_preserves_pose_inputs_and_shortest_rotation() {
        let a = Transform {
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
        };
        let b = Transform {
            position: Vec3::new(1.0, 2.0, 3.0),
            rotation: -Quat::from_rotation_y(0.5),
        };
        let bytes_before = (
            a.position.to_array(),
            a.rotation.to_array(),
            b.position.to_array(),
            b.rotation.to_array(),
        );
        let mut c = PresentationClock::default();
        c.retarget(0.0, 0.0, false).unwrap();
        c.retarget(0.025, 25.0, false).unwrap();
        let alpha = c.retarget(0.05, 35.0, false).unwrap().old_alpha;
        let body = a.interpolate(b, alpha);
        let environment = a.interpolate(b, alpha);
        let striker = a.interpolate(b, alpha);
        assert_eq!(body.position, environment.position);
        assert_eq!(body.position, striker.position);
        assert!((body.rotation.length() - 1.0).abs() < 1e-6);
        assert!((body.rotation.dot(Quat::from_rotation_y(0.2)).abs() - 1.0).abs() < 1e-6);
        assert_eq!(
            bytes_before,
            (
                a.position.to_array(),
                a.rotation.to_array(),
                b.position.to_array(),
                b.rotation.to_array()
            )
        );
    }
}
