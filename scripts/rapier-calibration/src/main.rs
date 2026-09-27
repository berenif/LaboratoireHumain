#[cfg(not(feature = "f64"))]
use rapier3d::prelude::*;
#[cfg(feature = "f64")]
use rapier_f64::prelude::*;
use serde::Deserialize;
use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};
use std::io::Write;

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct Metadata {
    body_handles: BTreeMap<String, [u32; 2]>,
    excluded_collider_pairs: Vec<[[u32; 2]; 2]>,
    #[serde(default = "default_steps")]
    steps: usize,
}

fn default_steps() -> usize { 720 }

struct Hooks(BTreeSet<[[u32; 2]; 2]>, bool);
impl PhysicsHooks for Hooks {
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
        if self.1 && context.rigid_body1.is_some_and(|h| context.bodies[h].is_dynamic())
            && context.rigid_body2.is_some_and(|h| context.bodies[h].is_dynamic()) { return None; }
        let (a, ag) = context.collider1.into_raw_parts();
        let (b, bg) = context.collider2.into_raw_parts();
        let mut pair = [[a, ag], [b, bg]];
        pair.sort();
        if self.0.contains(&pair) { None } else { Some(SolverFlags::COMPUTE_IMPULSES) }
    }
}

fn vector(v: Vector) -> Value { json!({"x": v.x as f64, "y": v.y as f64, "z": v.z as f64}) }

// Rapier's f32 world contains u128 user data and f32 scalar physics values.
// Preserve integer tokens unchanged, and widen each shortest f32 float token
// through its exact f32 value. Parsing a shortest f32 token directly as f64
// (e.g. 0.1) would otherwise change the initial physical state.
#[cfg(feature = "f64")]
fn widen_f32(value: &mut Value) {
    match value {
        Value::Number(number) if number.is_f64() => {
            let original: f32 = number.as_str().parse().expect("Invalid f32 field");
            *number = serde_json::Number::from_f64(original as f64).expect("Non-finite f32 field");
        },
        Value::Array(values) => values.iter_mut().for_each(widen_f32),
        Value::Object(values) => values.values_mut().for_each(widen_f32),
        _ => {},
    }
}
#[cfg(feature = "f64")]
fn first_transfer_difference(a: &Value, b: &Value, path: &str) -> Option<String> {
    if a == b { return None; }
    match (a, b) {
        (Value::Number(x), Value::Number(y)) if x.is_f64() && y.is_f64()
            && x.as_f64().unwrap().to_bits() == y.as_f64().unwrap().to_bits() => None,
        (Value::Array(x), Value::Array(y)) if x.len() == y.len() => x.iter().zip(y).enumerate()
            .find_map(|(i, (x, y))| first_transfer_difference(x, y, &format!("{path}[{i}]"))),
        (Value::Object(x), Value::Object(y)) if x.len() == y.len() => x.iter().find_map(|(key, x)|
            y.get(key).map_or_else(|| Some(format!("{path}.{key}: missing")),
                |y| first_transfer_difference(x, y, &format!("{path}.{key}")))),
        _ => Some(format!("{path}: expected {a}, got {b}")),
    }
}
#[cfg(feature = "f64")]
fn remove_zero_pose_padding(value: &mut Value) {
    match value {
        Value::Array(values) => values.iter_mut().for_each(remove_zero_pose_padding),
        Value::Object(values) => {
            // glamx Pose3A uses an explicit u32 padding slot for alignment;
            // DPose3 has no padding field. It carries no physical state.
            if values.len() == 3 && values.contains_key("rotation")
                && values.contains_key("translation") && values.contains_key("padding") {
                assert_eq!(values.remove("padding").unwrap(), json!(0));
            }
            values.values_mut().for_each(remove_zero_pose_padding);
        },
        _ => {},
    }
}
fn measured(world: &PhysicsWorld, metadata: &Metadata) -> Value {
    let mut result = serde_json::Map::new();
    for (id, [index, generation]) in &metadata.body_handles {
        let b = &world.bodies[RigidBodyHandle::from_raw_parts(*index, *generation)];
        let q = b.rotation();
        result.insert(id.clone(), json!({"position": vector(b.translation()), "com": vector(b.center_of_mass()),
            "velocity": vector(b.linvel()), "angularVelocity": vector(b.angvel()),
            "rotation": {"x": q.x as f64, "y": q.y as f64, "z": q.z as f64, "w": q.w as f64}}));
    }
    Value::Object(result)
}

fn self_contacts(world: &PhysicsWorld) -> Value {
    let contacts: Vec<_> = world.narrow_phase.contact_pairs().filter(|pair| {
        [pair.collider1, pair.collider2].iter().all(|h|
            world.colliders[*h].parent().is_some_and(|b| world.bodies[b].is_dynamic()))
    }).filter_map(|pair| pair.find_deepest_contact().map(|(_, point)| {
        json!({"first": pair.collider1.into_raw_parts(), "second": pair.collider2.into_raw_parts(),
            "distance": point.dist as f64, "reportedImpulse": pair.total_impulse_magnitude() as f64})
    })).collect();
    json!(contacts)
}

fn ground_contacts(world: &PhysicsWorld) -> Value {
    let contacts: Vec<_> = world.narrow_phase.contact_pairs().filter(|pair| {
        [pair.collider1, pair.collider2].iter().any(|h|
            world.colliders[*h].parent().is_none_or(|b| world.bodies[b].is_fixed()))
    }).filter_map(|pair| pair.find_deepest_contact().map(|(_, point)| {
        json!({"first": pair.collider1.into_raw_parts(), "second": pair.collider2.into_raw_parts(),
            "distance": point.dist as f64, "reportedImpulse": pair.total_impulse_magnitude() as f64,
            "solverContacts": pair.manifolds.iter().map(|m| m.data.solver_contacts.len()).sum::<usize>()})
    })).collect();
    json!(contacts)
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert!(args.len() == 5, "Usage: rapier-calibration snapshot.bin metadata.json output.json mode");
    let bytes = std::fs::read(&args[1]).unwrap();
    assert!(bytes.len() < 32 * 1024 * 1024, "Unexpectedly large snapshot");
    let source_world: rapier3d::pipeline::PhysicsWorld = bincode::deserialize(&bytes).expect("Pinned Rapier snapshot format mismatch");
    #[cfg(not(feature = "f64"))]
    let mut world: PhysicsWorld = source_world;
    #[cfg(feature = "f64")]
    let mut world: PhysicsWorld = {
        let mut fields = serde_json::to_value(source_world).expect("Cannot represent the native f32 world as JSON");
        widen_f32(&mut fields);
        let transferred: PhysicsWorld = serde_json::from_value(fields.clone())
            .expect("Cannot transfer the exact f32 state into the f64 world");
        remove_zero_pose_padding(&mut fields);
        assert!(first_transfer_difference(&fields, &serde_json::to_value(&transferred).unwrap(), "world")
            .is_none(), "Precision transfer changed a serialized world field: {:?}",
            first_transfer_difference(&fields, &serde_json::to_value(&transferred).unwrap(), "world"));
        transferred
    };
    let metadata: Metadata = serde_json::from_slice(&std::fs::read(&args[2]).unwrap()).unwrap();
    let hooks = Hooks(metadata.excluded_collider_pairs.iter().map(|p| { let mut pair = *p; pair.sort(); pair }).collect(),
        args[4] == "no-self-contact");
    let original_parameters = serde_json::to_value(&world.integration_parameters).unwrap();
    match args[4].as_str() {
        "normal" | "no-self-contact" => {},
        "no-ccd" => world.integration_parameters.max_ccd_substeps = 0,
        "friction-in-bias" => world.integration_parameters.friction_in_bias_pass = true,
        "warmstart-joints" => world.integration_parameters.warmstart_joints = true,
        // Isolate the normal-contact compliance limit on the rigid calibration
        // fixture. This is deliberately outside the frozen application settings.
        "rigid-fixed-contact" => world.integration_parameters.static_contact_softness.natural_frequency = 1.0e6,
        "pgs-convergence" => world.integration_parameters.num_internal_pgs_iterations *= 10,
        "relax-convergence" => world.integration_parameters.num_internal_stabilization_iterations =
            world.integration_parameters.num_internal_pgs_iterations,
        _ => panic!("Unknown isolated calibration intervention"),
    }
    let parameters = serde_json::to_value(&world.integration_parameters).unwrap();
    let initial = measured(&world, &metadata);
    let mut samples = vec![json!({"tick": 0, "measured": initial})];
    assert!(metadata.steps > 0 && metadata.steps <= 1920, "Invalid calibration horizon");
    let mut peak_linear: Real = 0.0;
    let mut peak_angular: Real = 0.0;
    let mut first_ccd_active_tick = None;
    let mut ccd_active_body_steps = 0usize;
    for tick in 1..=metadata.steps {
        world.step_with_events(&hooks, &());
        let active_ccd_bodies = world.bodies.iter().filter(|(_, body)| body.is_ccd_active()).count();
        if active_ccd_bodies > 0 { first_ccd_active_tick.get_or_insert(tick); }
        ccd_active_body_steps += active_ccd_bodies;
        if tick > 120 {
            for (_, body) in world.bodies.iter() {
                peak_linear = peak_linear.max(body.linvel().length());
                peak_angular = peak_angular.max(body.angvel().length());
            }
        }
        if tick <= 10 || tick % 60 == 0 { samples.push(json!({"tick": tick, "measured": measured(&world, &metadata),
            "selfContacts": self_contacts(&world), "groundContacts": ground_contacts(&world),
            "activeCcdBodies": active_ccd_bodies})); }
    }
    let precision = if cfg!(feature = "f64") { "f64 from exact serialized f32 state" } else { "f32" };
    let report = json!({"command": args, "engine": "Rapier 0.35.0 / Parry 0.30.2", "precision": precision,
        "blockSolverFeature": cfg!(feature = "block-solver"),
        "serializedStateTransferVerified": cfg!(feature = "f64"),
        "qualification": "Independent native replay of a JS snapshot. Compare baseline replay before interpreting the isolated solver intervention; never a production acceptance run.",
        "originalParameters": original_parameters, "parameters": parameters,
        "maxLinearAfter120": peak_linear, "maxAngularAfter120": peak_angular,
        "ccdActivity": {"firstActiveTick": first_ccd_active_tick, "activeBodySteps": ccd_active_body_steps},
        "samples": samples});
    let mut output = std::fs::OpenOptions::new().write(true).create_new(true).open(&args[3]).unwrap();
    writeln!(output, "{}", serde_json::to_string_pretty(&report).unwrap()).unwrap();
    println!("{}", json!({"output": args[3], "mode": args[4], "maxLinearAfter120": peak_linear, "maxAngularAfter120": peak_angular}));
}
