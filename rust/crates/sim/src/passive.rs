//! Canonical passive resistance: damping throughout the joint range and a
//! cubic spring near the ends. It supplies no standing or get-up posture.
pub(crate) struct Feedback {
    pub target: f32,
    pub stiffness: f32,
    pub damping: f32,
    pub ceiling: f32,
}
pub(crate) fn feedback(value: f32, axis: &lh_model::Axis, soft_zone: f32) -> Feedback {
    let zone = ((axis.max_radians - axis.min_radians).max(1e-6) * soft_zone).max(1e-5);
    let amount = if value < axis.min_radians + zone {
        ((axis.min_radians + zone - value) / zone).clamp(0.0, 2.0)
    } else if value > axis.max_radians - zone {
        -((value - (axis.max_radians - zone)) / zone).clamp(0.0, 2.0)
    } else {
        0.0
    };
    Feedback {
        target: (value + zone * amount.powi(3)).clamp(axis.min_radians, axis.max_radians),
        stiffness: if amount != 0.0 {
            axis.passive_stiffness_nm_per_rad
        } else {
            0.0
        },
        damping: axis.damping_nms_per_rad,
        ceiling: (axis.max_motor_torque_nm * 0.5)
            .max(axis.passive_stiffness_nm_per_rad * zone * 2.0)
            .min(axis.max_motor_torque_nm),
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn passive_feedback_has_no_central_posture_spring_and_respects_every_axis_ceiling() {
        let model = lh_model::Model::canonical().unwrap();
        for profile in model
            .segments
            .iter()
            .filter_map(|s| s.joint_profile.as_ref())
        {
            for axis in &profile.axes {
                let middle = (axis.min_radians + axis.max_radians) * 0.5;
                let center = feedback(middle, axis, profile.limit_soft_zone_fraction);
                assert_eq!(center.stiffness, 0.0);
                assert_eq!(center.target, middle);
                for value in [
                    axis.min_radians - 0.06,
                    axis.min_radians,
                    axis.max_radians,
                    axis.max_radians + 0.06,
                ] {
                    let r = feedback(value, axis, profile.limit_soft_zone_fraction);
                    assert!(r.ceiling > 0.0 && r.ceiling <= axis.max_motor_torque_nm);
                    assert!((r.target - value) * (middle - value) >= 0.0);
                    assert!(r.damping >= 0.0 && r.stiffness >= 0.0);
                }
            }
        }
    }
}
