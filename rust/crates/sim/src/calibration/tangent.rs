//! Independent horizontal/vertical momentum fixtures for contact reporting.
use crate::{
    Profile, configure,
    measurement::{floor_normal_impulse, floor_tangent_impulse},
    vec3,
};
use lh_model::Vec3;
use rapier3d::prelude::*;
use serde::Serialize;

pub const SCENARIOS: [&str; 6] = [
    "static",
    "sliding",
    "changing",
    "unloading",
    "contact-loss",
    "frictionless",
];
#[derive(Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Report {
    pub scope: &'static str,
    pub model: String,
    pub heading: f32,
    pub warmstart: f32,
    pub scenario: String,
    pub excitation: &'static str,
    pub requested_friction: f32,
    pub effective_friction: f32,
    pub settling_substeps: u32,
    pub observed_substeps: u32,
    pub max_tangent_momentum_error_ns: f32,
    pub worst_tangent_sample: MomentumSample,
    pub max_normal_momentum_error_ns: f32,
    pub max_friction_bound_excess_ns: f32,
    pub max_horizontal_speed_mps: f32,
    pub mean_tangent_force_n: f32,
    pub zero_signal_when_contact_absent: bool,
    pub final_velocity: Vec3,
    pub passed: bool,
    pub release_accepted: bool,
}
#[derive(Debug, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct MomentumSample {
    pub substep: u32,
    pub before_velocity: Vec3,
    pub after_velocity: Vec3,
    pub external_force_n: Vec3,
    pub reported_tangent_impulse_ns: Vec3,
    pub expected_tangent_impulse_ns: Vec3,
    pub float64_momentum_error_ns: f64,
}
pub fn run(
    model: &str,
    heading: f32,
    warmstart: f32,
    scenario: &str,
    friction: f32,
) -> Result<Report, String> {
    run_fixture(model, heading, warmstart, scenario, friction, false, false)
}
pub fn run_bounded(
    model: &str,
    heading: f32,
    warmstart: f32,
    scenario: &str,
    friction: f32,
) -> Result<Report, String> {
    run_fixture(model, heading, warmstart, scenario, friction, false, true)
}
fn run_fixture(
    model: &str,
    heading: f32,
    warmstart: f32,
    scenario: &str,
    friction: f32,
    force_only: bool,
    bounded: bool,
) -> Result<Report, String> {
    let friction_model = match model {
        "simplified" => FrictionModel::Simplified,
        "coulomb" => FrictionModel::Coulomb,
        _ => return Err("Unknown calibration friction model".into()),
    };
    if !heading.is_finite()
        || !warmstart.is_finite()
        || !(0.0..=1.0).contains(&warmstart)
        || !SCENARIOS.contains(&scenario)
        || ![0.5, 4.0].contains(&friction)
    {
        return Err("Invalid tangent calibration fixture".into());
    }
    let profile = Profile::default();
    let mut world = PhysicsWorld::new();
    configure(&mut world, &profile);
    world.integration_parameters.warmstart_coefficient = warmstart;
    world.integration_parameters.friction_model = friction_model;
    let mu = if scenario == "frictionless" {
        0.0
    } else {
        friction
    };
    let floor = world.insert_collider(
        ColliderBuilder::cuboid(1000.0, 0.5, 1000.0)
            .translation(Vector::new(0.0, -0.5, 0.0))
            .friction(mu)
            .restitution(0.0),
        None,
    );
    let (body, collider) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 0.5, 0.0))
            .rotation(Vector::Y * heading)
            .lock_rotations()
            .can_sleep(false)
            .additional_solver_iterations(profile.additional_body_iterations),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5)
            .mass(10.0)
            .friction(mu)
            .restitution(0.0),
    );
    for _ in 0..480 {
        world.step();
    }
    let direction = Rotation::from_rotation_y(heading) * Vector::new(0.6, 0.0, 0.8);
    let dt = world.integration_parameters.dt;
    let mass = world.bodies[body].mass();
    let mut max_tangent_error = 0.0_f32;
    let mut worst_tangent_sample = MomentumSample::default();
    let mut max_normal_error = 0.0_f32;
    let mut max_bound_excess = 0.0_f32;
    let mut max_speed = 0.0_f32;
    let mut tangent_sum = 0.0_f32;
    let mut zero_when_absent = true;
    for step in 0..480 {
        if (scenario == "contact-loss" || force_only) && step == 0 {
            world.colliders[floor].set_enabled(false);
        }
        let magnitude = match scenario {
            "sliding" if bounded => mu * mass * world.gravity.y.abs() + 20.0,
            "sliding" => 200.0 * friction / 0.5,
            "changing" if step >= 240 => -20.0,
            _ => 20.0,
        };
        let force = direction * magnitude
            + if scenario == "unloading" {
                Vector::Y * 110.0
            } else {
                Vector::ZERO
            };
        let before = world.bodies[body].linvel();
        world.bodies[body].reset_forces(false);
        world.bodies[body].add_force(force, true);
        world.step();
        let after = world.bodies[body].linvel();
        let tangent = floor_tangent_impulse(&world, floor, collider)?;
        let normal = floor_normal_impulse(&world, floor, collider)?;
        let expected = mass * (after - before) - (force + world.gravity * mass) * dt;
        let expected_horizontal = Vector::new(expected.x, 0.0, expected.z);
        let tangent_error = tangent.distance(expected_horizontal);
        if tangent_error >= max_tangent_error {
            let expected_x =
                mass as f64 * (after.x as f64 - before.x as f64) - force.x as f64 * dt as f64;
            let expected_z =
                mass as f64 * (after.z as f64 - before.z as f64) - force.z as f64 * dt as f64;
            worst_tangent_sample = MomentumSample {
                substep: step,
                before_velocity: vec3(before),
                after_velocity: vec3(after),
                external_force_n: vec3(force),
                reported_tangent_impulse_ns: vec3(tangent),
                expected_tangent_impulse_ns: vec3(expected_horizontal),
                float64_momentum_error_ns: ((tangent.x as f64 - expected_x).powi(2)
                    + (tangent.z as f64 - expected_z).powi(2))
                .sqrt(),
            };
        }
        max_tangent_error = max_tangent_error.max(tangent_error);
        max_normal_error = max_normal_error.max((normal - expected.y).abs());
        max_bound_excess = max_bound_excess.max((tangent.length() - mu * normal).max(0.0));
        max_speed = max_speed.max(Vector::new(after.x, 0.0, after.z).length());
        tangent_sum += tangent.length() / dt;
        if normal == 0.0 || scenario == "frictionless" || scenario == "contact-loss" {
            zero_when_absent &= tangent == Vector::ZERO;
        }
    }
    let mean_force = tangent_sum / 480.0;
    let behavior_pass = if force_only {
        zero_when_absent
    } else {
        match scenario {
            "static" | "changing" => max_speed <= 0.001 && (mean_force - 20.0).abs() <= 0.2,
            "sliding" => {
                max_speed > 1.0 && (mean_force - mu * mass * world.gravity.y.abs()).abs() <= 0.5
            }
            _ => zero_when_absent,
        }
    };
    Ok(Report {
        scope: if force_only {
            "Same high-force fixture with floor disabled after settling: external-force f32 momentum control, no contact calibration or release admission"
        } else {
            "Isolated flat-box linear contact reporting; no angular friction, character gate or release admission"
        },
        model: model.into(),
        heading,
        warmstart,
        scenario: scenario.into(),
        excitation: if bounded { "bounded" } else { "stress" },
        requested_friction: friction,
        effective_friction: mu,
        settling_substeps: 480,
        observed_substeps: 480,
        max_tangent_momentum_error_ns: max_tangent_error,
        worst_tangent_sample,
        max_normal_momentum_error_ns: max_normal_error,
        max_friction_bound_excess_ns: max_bound_excess,
        max_horizontal_speed_mps: max_speed,
        mean_tangent_force_n: mean_force,
        zero_signal_when_contact_absent: zero_when_absent,
        final_velocity: vec3(world.bodies[body].linvel()),
        passed: max_tangent_error <= 0.002
            && max_normal_error <= 0.002
            && max_bound_excess <= 0.002
            && zero_when_absent
            && behavior_pass
            && (!bounded || max_speed <= 4.1),
        release_accepted: false,
    })
}
/// A causal diagnostic for the unchanged high-speed sliding fixture. The floor
/// is disabled after settling and all forces, clock and solver settings remain
/// identical. A failure here cannot be caused by tangent impulse reporting.
pub fn force_only_controls() -> Result<Vec<Report>, String> {
    let mut results = Vec::with_capacity(36);
    for model in ["simplified", "coulomb"] {
        for friction in [0.5, 4.0] {
            for heading in [0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0] {
                for warmstart in [0.0, 0.5, 1.0] {
                    results.push(run_fixture(
                        model, heading, warmstart, "sliding", friction, true, false,
                    )?);
                }
            }
        }
    }
    Ok(results)
}
pub fn matrix() -> Result<Vec<Report>, String> {
    matrix_with_excitation(false)
}
pub fn bounded_matrix() -> Result<Vec<Report>, String> {
    matrix_with_excitation(true)
}
fn matrix_with_excitation(bounded: bool) -> Result<Vec<Report>, String> {
    let mut results = Vec::with_capacity(216);
    for model in ["simplified", "coulomb"] {
        for friction in [0.5, 4.0] {
            for heading in [0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0] {
                for warmstart in [0.0, 0.5, 1.0] {
                    for scenario in SCENARIOS {
                        results.push(run_fixture(
                            model, heading, warmstart, scenario, friction, false, bounded,
                        )?);
                    }
                }
            }
        }
    }
    Ok(results)
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn bounded_contact_models_report_linear_impulses_against_independent_momentum() {
        let results = bounded_matrix().unwrap();
        assert_eq!(results.len(), 216);
        for result in results {
            assert!(result.passed, "{}", serde_json::to_string(&result).unwrap());
        }
    }
    #[test]
    fn stress_diagnostics_preserve_failed_cases_and_never_admit_release() {
        let results = matrix().unwrap();
        assert_eq!(results.len(), 216);
        assert!(
            results.iter().any(|r| !r.passed),
            "the high-speed experiment remains unqualified"
        );
        for r in results {
            assert!(!r.release_accepted);
            assert_eq!(r.excitation, "stress");
            if !r.passed {
                assert_eq!(r.scenario, "sliding");
                assert!(r.max_tangent_momentum_error_ns > 0.002);
                assert!(r.worst_tangent_sample.float64_momentum_error_ns > 0.002);
            }
        }
        let controls = force_only_controls().unwrap();
        assert_eq!(controls.len(), 36);
        assert!(controls.iter().any(|r| !r.passed));
        assert!(controls.iter().all(|r| r.zero_signal_when_contact_absent
            && r.worst_tangent_sample.reported_tangent_impulse_ns == Vec3::default()
            && !r.release_accepted));
    }
}
