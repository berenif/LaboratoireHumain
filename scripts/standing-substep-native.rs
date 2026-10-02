use rapier3d::dynamics::solver::standing_trace;
use rapier3d::prelude::*;
use serde_json::{json, Value};
use std::collections::HashSet;
use std::io::Write;

struct StandingHooks {
    excluded: HashSet<(u32, u32)>,
}

impl StandingHooks {
    fn from_path(path: Option<&String>) -> Self {
        let mut excluded = HashSet::new();
        if let Some(path) = path {
            let value: Value = serde_json::from_slice(&std::fs::read(path).unwrap()).unwrap();
            for pair in value["excludedColliderIndices"].as_array().unwrap() {
                let a = pair[0].as_u64().unwrap() as u32;
                let b = pair[1].as_u64().unwrap() as u32;
                excluded.insert((a.min(b), a.max(b)));
            }
        }
        Self { excluded }
    }
}

impl PhysicsHooks for StandingHooks {
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
        let (a, _) = context.collider1.into_raw_parts();
        let (b, _) = context.collider2.into_raw_parts();
        if self.excluded.contains(&(a.min(b), a.max(b))) {
            None
        } else {
            Some(SolverFlags::COMPUTE_IMPULSES)
        }
    }

    fn filter_intersection_pair(&self, _context: &PairFilterContext) -> bool {
        true
    }
}

fn bodies(world: &PhysicsWorld) -> Vec<Value> {
    world.bodies.iter().map(|(handle, body)| {
        let (index, generation) = handle.into_raw_parts();
        json!({
            "handle": [index, generation],
            "translation": body.translation(),
            "rotation": body.rotation(),
            "linearVelocity": body.linvel(),
            "angularVelocity": body.angvel(),
            "dynamic": body.is_dynamic(),
        })
    }).collect()
}

fn inspect(input: &str, hooks: &StandingHooks) -> Value {
    let bytes = std::fs::read(input).unwrap();
    let mut control: PhysicsWorld = bincode::deserialize(&bytes).unwrap();
    control.step_with_events(hooks, &());
    let control_after = bodies(&control);
    let control_bytes = bincode::serialize(&control).unwrap();
    let mut world: PhysicsWorld = bincode::deserialize(&bytes).unwrap();
    let before = bodies(&world);
    let parameters = world.integration_parameters;
    standing_trace::begin();
    world.step_with_events(hooks, &());
    let events = standing_trace::finish();
    let after = bodies(&world);
    let traced_bytes = bincode::serialize(&world).unwrap();
    json!({
        "input": input,
        "inputBytes": bytes.len(),
        "integrationParameters": parameters,
        "beforeBodies": before,
        "controlAfterBodies": control_after,
        "afterBodies": after,
        "outputNeutral": control_bytes == traced_bytes,
        "controlBytes": control_bytes.len(),
        "tracedBytes": traced_bytes.len(),
        "events": events,
        "qualification": "One native Rapier physical step restored from the supplied snapshot. Trace reads occur only at existing solver barriers; constraint values and body state are not modified. No production source or dependency is changed."
    })
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert!(args.len() == 3 || args.len() == 4,
        "standing-substep-native input.bin output.json [collision-context.json]");
    let hooks = StandingHooks::from_path(args.get(3));
    let result = inspect(&args[1], &hooks);
    let mut output = std::fs::OpenOptions::new().write(true).create_new(true).open(&args[2]).unwrap();
    writeln!(output, "{}", serde_json::to_string_pretty(&result).unwrap()).unwrap();
}
