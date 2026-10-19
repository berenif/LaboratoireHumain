//! Bounded point-force controller. Owns intent, never body poses or velocities.
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
    pub fn target_velocity(&self) -> Vector {
        self.target_velocity
    }
    pub(crate) fn reverses(&self, demand: Vector) -> bool {
        // The acceleration-limited filter can briefly overshoot a held target.
        // Changing the planned mover requires a substantial raw pointer change.
        self.target_velocity.dot(demand) < -0.01
            && (self.raw_target - self.control_target).dot(demand) < -0.05 * demand.length()
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
        point_velocity: Vector,
        dt: f32,
        limits: &Limits,
    ) -> Result<Vector, String> {
        if !dt.is_finite()
            || dt <= 0.0
            || !limits.valid()
            || !self.raw_target.is_finite()
            || !point_velocity.is_finite()
        {
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
        // Multibody link velocity caches can lag the integrated generalized
        // velocity. The owner supplies the current physical velocity at anchor.
        let velocity = point_velocity;
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
        // A RigidBody impulse only changes the link's velocity cache, which a
        // reduced-coordinate multibody overwrites. External forces enter its
        // generalized dynamics and transmit to the rest of the articulation.
        // Simulation clears this wrench before every 1/800 s substep.
        body.add_force_at_point(impulse / dt, anchor, true);
        Ok(impulse)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn held_target_overshoot_is_distinct_from_a_real_pointer_reversal() {
        for direction in [Vector::X, -Vector::X] {
            let mut body = RigidBodyBuilder::dynamic()
                .additional_mass_properties(MassProperties::new(
                    Vector::ZERO,
                    5.0,
                    Vector::splat(1.0),
                ))
                .build();
            body.recompute_mass_properties_from_colliders(&ColliderSet::new());
            let mut grab = Grab::new(0, 1, Vector::ZERO, Vector::ZERO).expect("fixture");
            let limits = Limits::default();
            let dt = 1.0 / 240.0;
            let mut saw_overshoot = false;
            for tick in 0..180 {
                grab.raw_target = direction * (0.6 * (tick as f32 / 90.0).min(1.0));
                for _ in 0..4 {
                    body.reset_forces(false);
                    body.reset_torques(false);
                    let previous_velocity = grab.target_velocity();
                    grab.apply(&mut body, Vector::ZERO, dt, &limits)
                        .expect("held target");
                    assert!(grab.target_velocity().length() <= limits.target_speed_m_s + 1e-6);
                    assert!(
                        grab.target_velocity().distance(previous_velocity)
                            <= limits.target_acceleration_m_s2 * dt + 1e-6
                    );
                    saw_overshoot |= grab.target_velocity().dot(direction) < -0.01;
                    assert!(!grab.reverses(direction));
                }
            }
            assert!(saw_overshoot);
            grab.raw_target = -direction * 0.6;
            let mut reversed = false;
            for _ in 0..60 {
                body.reset_forces(false);
                body.reset_torques(false);
                grab.apply(&mut body, Vector::ZERO, dt, &limits)
                    .expect("reversed target");
                reversed |= grab.reverses(direction);
            }
            assert!(reversed);
        }
    }
    #[test]
    fn point_force_obeys_wrench_and_isolated_impulse_equivalent_work_caps() {
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
                body.reset_forces(false);
                body.reset_torques(false);
                let before =
                    2.5 * body.linvel().length_squared() + 0.5 * body.angvel().length_squared();
                let anchor = body.position().transform_point(grab.local_anchor);
                let radius = anchor - body.center_of_mass();
                let point_velocity = body.velocity_at_point(anchor);
                let j = grab
                    .apply(&mut body, point_velocity, dt, &limits)
                    .expect("bounded force");
                // Independent free-body impulse-equivalent energy, not a claim
                // about immediate cache mutation or full articulated energy.
                let after = 2.5 * (velocity + j / 5.0).length_squared()
                    + 0.5 * radius.cross(j).length_squared();
                assert!(j.length() <= limits.force_n * dt + 1e-5);
                assert!(radius.cross(j).length() <= limits.torque_nm * dt + 1e-5);
                assert!(after - before <= limits.positive_power_w * dt + 0.0001);
                assert!(body.user_force().distance(j / dt) < 0.0001);
                assert!(body.user_torque().distance(radius.cross(j / dt)) < 0.0001);
                assert_eq!(body.linvel(), velocity, "force must await integration");
            }
        }
    }

    #[test]
    fn grab_force_reaches_free_multibody_momentum_instead_of_only_link_cache() {
        for direction in [Vector::X, -Vector::X] {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::ZERO;
            world.integration_parameters.dt = 1.0 / 800.0;
            let create = |position| {
                RigidBodyBuilder::dynamic()
                    .translation(position)
                    .can_sleep(false)
                    .additional_mass_properties(MassProperties::new(Vector::ZERO, 1.0, Vector::ONE))
            };
            let root = world.insert_body(create(Vector::ZERO));
            let child = world.insert_body(create(Vector::Y));
            world
                .insert_multibody_joint(
                    root,
                    child,
                    FixedJointBuilder::new().local_anchor1(Vector::Y).build(),
                )
                .expect("articulation");
            world.step();
            let anchor = world.bodies[child].center_of_mass();
            let mut grab = Grab::new(0, 1, Vector::ZERO, anchor + direction * 0.2).expect("grab");
            let impulse = grab
                .apply(
                    &mut world.bodies[child],
                    Vector::ZERO,
                    1.0 / 800.0,
                    &Limits::default(),
                )
                .expect("bounded point force");
            assert!(impulse.dot(direction) > 0.0);
            world.step();
            let id = *world.multibody_joints.rigid_body_link(root).unwrap();
            let multibody = world.multibody_joints.get_multibody(id.multibody).unwrap();
            let mut momentum = Vector::ZERO;
            for link in multibody.links() {
                let twist =
                    multibody.body_jacobian(link.link_id()) * multibody.generalized_velocity();
                let velocity = Vector::new(twist[0], twist[1], twist[2]);
                assert!(velocity.dot(direction) > 0.0, "both links must respond");
                momentum += velocity * world.bodies[link.rigid_body_handle()].mass();
            }
            assert!(
                momentum.distance(impulse) < 0.00001,
                "{momentum:?} vs {impulse:?}"
            );
        }
    }

    #[test]
    fn damping_uses_supplied_current_point_velocity_instead_of_stale_body_cache() {
        let mut body = RigidBodyBuilder::dynamic()
            .additional_mass_properties(MassProperties::new(Vector::ZERO, 1.0, Vector::ONE))
            .linvel(-Vector::X)
            .build();
        body.recompute_mass_properties_from_colliders(&ColliderSet::new());
        let mut grab = Grab::new(0, 1, Vector::ZERO, Vector::ZERO).unwrap();
        let impulse = grab
            .apply(&mut body, Vector::X, 1.0 / 800.0, &Limits::default())
            .unwrap();
        assert!(
            impulse.x < 0.0,
            "damp actual motion, not the opposite cached velocity"
        );
        body.reset_forces(false);
        body.reset_torques(false);
        assert!(
            grab.apply(
                &mut body,
                Vector::splat(f32::NAN),
                1.0 / 800.0,
                &Limits::default()
            )
            .is_err()
        );
        assert_eq!(body.user_force(), Vector::ZERO);
    }
}
