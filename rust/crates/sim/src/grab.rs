//! Port of the legacy S1 point-impulse controller. Owns intent, never body poses.
use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Limits {
    pub angular_frequency_rad_s: f32,
    pub target_speed_m_s: f32,
    pub target_acceleration_m_s2: f32,
    pub force_n: f32,
    pub torque_nm: f32,
    pub positive_power_w: f32,
}
impl Default for Limits {
    fn default() -> Self {
        Self {
            angular_frequency_rad_s: 16.0,
            target_speed_m_s: 1.5,
            target_acceleration_m_s2: 8.0,
            force_n: 180.0,
            torque_nm: 12.0,
            positive_power_w: 36.0,
        }
    }
}
impl Limits {
    pub fn valid(&self) -> bool {
        [
            self.angular_frequency_rad_s,
            self.target_speed_m_s,
            self.target_acceleration_m_s2,
            self.force_n,
            self.torque_nm,
            self.positive_power_w,
        ]
        .iter()
        .all(|v| v.is_finite() && *v > 0.0)
    }
}
pub struct Grab {
    pub segment: usize,
    pub press: u64,
    pub local_anchor: Vector,
    pub raw_target: Vector,
    control_target: Vector,
    target_velocity: Vector,
}
fn bounded(v: Vector, max: f32) -> Vector {
    let n = v.length();
    if n > max { v * (max / n) } else { v }
}
impl Grab {
    pub fn control_target(&self) -> Vector {
        self.control_target
    }
    pub fn new(
        segment: usize,
        press: u64,
        local_anchor: Vector,
        target: Vector,
    ) -> Result<Self, String> {
        if segment >= 25 || !local_anchor.is_finite() || !target.is_finite() {
            return Err("Invalid grab".into());
        }
        Ok(Self {
            segment,
            press,
            local_anchor,
            raw_target: target,
            control_target: target,
            target_velocity: Vector::ZERO,
        })
    }
    pub fn apply(
        &mut self,
        body: &mut RigidBody,
        dt: f32,
        limits: &Limits,
    ) -> Result<Vector, String> {
        if !dt.is_finite() || dt <= 0.0 || !limits.valid() || !self.raw_target.is_finite() {
            return Err("Invalid grab interval/limits/target".into());
        }
        let remaining = self.raw_target - self.control_target;
        let distance = remaining.length();
        let speed = limits
            .target_speed_m_s
            .min((2.0 * limits.target_acceleration_m_s2 * distance).sqrt())
            .min(distance / dt);
        let desired = if distance > 0.0 {
            remaining * (speed / distance)
        } else {
            Vector::ZERO
        };
        self.target_velocity += bounded(
            desired - self.target_velocity,
            limits.target_acceleration_m_s2 * dt,
        );
        self.control_target += self.target_velocity * dt;
        let anchor = body.position().transform_point(self.local_anchor);
        let radius = anchor - body.center_of_mass();
        let velocity = body.velocity_at_point(anchor);
        let m = body.mass_properties();
        let response = |j: Vector| {
            m.effective_inv_mass * j
                + (m.effective_world_inv_inertia * (radius.cross(j))).cross(radius)
        };
        let x = response(Vector::X);
        let y = response(Vector::Y);
        let z = response(Vector::Z);
        let determinant = x.dot(y.cross(z));
        if !determinant.is_finite() || determinant <= 1e-12 {
            return Err("Invalid grab effective mass".into());
        }
        let omega = limits.angular_frequency_rad_s;
        let dv = ((self.control_target - anchor) * (omega * omega)
            + (self.target_velocity - velocity) * (2.0 * omega))
            * (dt / (1.0 + 2.0 * omega * dt + omega * omega * dt * dt));
        let mut impulse =
            Vector::new(dv.dot(y.cross(z)), x.dot(dv.cross(z)), x.dot(y.cross(dv))) / determinant;
        impulse = bounded(impulse, limits.force_n * dt);
        let angular = radius.cross(impulse).length();
        if angular > limits.torque_nm * dt {
            impulse *= limits.torque_nm * dt / angular;
        }
        let quadratic = 0.5 * impulse.dot(response(impulse));
        let linear = impulse.dot(velocity);
        let work_limit = limits.positive_power_w * dt;
        if linear + quadratic > work_limit {
            let scale = if quadratic > 1e-12 {
                (-linear + (linear * linear + 4.0 * quadratic * work_limit).sqrt())
                    / (2.0 * quadratic)
            } else if linear > 0.0 {
                work_limit / linear
            } else {
                1.0
            };
            impulse *= scale.clamp(0.0, 1.0);
        }
        if !impulse.is_finite() {
            return Err("Non-finite grab impulse".into());
        }
        body.apply_impulse_at_point(impulse, anchor, true);
        Ok(impulse)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn point_impulse_obeys_force_torque_and_independent_kinetic_energy_caps() {
        for velocity in [Vector::ZERO, Vector::new(5.0, -2.0, 3.0)] {
            let mut body = RigidBodyBuilder::dynamic()
                .additional_mass_properties(MassProperties::new(
                    Vector::ZERO,
                    5.0,
                    Vector::splat(1.0),
                ))
                .linvel(velocity)
                .build();
            let colliders = ColliderSet::new();
            body.recompute_mass_properties_from_colliders(&colliders);
            let mut grab = Grab::new(
                0,
                1,
                Vector::new(0.03, 0.02, 0.01),
                Vector::new(50.0, 2.0, 3.0),
            )
            .expect("fixture");
            let limits = Limits::default();
            let dt = 1.0 / 240.0;
            for _ in 0..240 {
                let before =
                    2.5 * body.linvel().length_squared() + 0.5 * body.angvel().length_squared();
                let anchor = body.position().transform_point(grab.local_anchor);
                let radius = anchor - body.center_of_mass();
                let j = grab.apply(&mut body, dt, &limits).expect("bounded impulse");
                let after =
                    2.5 * body.linvel().length_squared() + 0.5 * body.angvel().length_squared();
                assert!(j.length() <= limits.force_n * dt + 1e-5);
                assert!(radius.cross(j).length() <= limits.torque_nm * dt + 1e-5);
                assert!(after - before <= limits.positive_power_w * dt + 0.0001);
            }
        }
    }
}
