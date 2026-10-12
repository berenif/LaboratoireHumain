//! Test-only support measurements and explicitly opt-in native experiments.
//! These traces do not qualify recovery.
use super::*;
use serde_json::{Value, json};

pub(crate) mod full_trajectory;
pub(crate) mod getup_ab;
pub(crate) mod hand_support_experiment;
pub(crate) mod offline_trajectory;
mod whole_history;

fn velocity(s: &Simulation) -> Vector {
    s.bodies
        .iter()
        .map(|h| {
            let b = &s.world.bodies[*h];
            b.linvel() * b.mass()
        })
        .sum::<Vector>()
        / s.total_mass()
}

fn contacts(s: &Simulation) -> Vec<Value> {
    let mut result = Vec::new();
    for (i, collider) in s.colliders.iter().enumerate() {
        for pair in s.world.narrow_phase.contact_pairs_with(*collider) {
            let first = pair.collider1 == *collider;
            let other = if first {
                pair.collider2
            } else {
                pair.collider1
            };
            if other != s.floor {
                continue;
            }
            for m in pair.solver_manifolds() {
                if !m.data.solver_flags.contains(SolverFlags::COMPUTE_IMPULSES) {
                    continue;
                }
                let normal = m.data.normal * if first { -1.0 } else { 1.0 };
                if normal.y < 0.65 {
                    continue;
                }
                for c in &m.data.solver_contacts {
                    let p = &m.points[c.contact_indices()[0] as usize];
                    if p.data.impulse <= 0.0 {
                        continue;
                    }
                    let (a, b) = m.data.solver_contact_world_points(c, &s.world.bodies);
                    let world = if first { a } else { b };
                    result.push(json!({"id":s.model.segments[i].id,
                        "point":crate::vec3(s.heading.inverse()*world),
                        "normalLoadN":normal.y*p.data.impulse/s.profile.dt_s,
                        "persistent":load(s,&s.model.segments[i].id)>0.0}));
                }
            }
        }
    }
    result
}

fn observe(s: &Simulation, previous: Vector) -> Value {
    let v = velocity(s);
    let com = s.center_of_mass();
    let capture =
        s.heading.inverse() * (com + v * ((com.y - s.support_height()).max(0.2) / 9.81).sqrt());
    let resolved = contacts(s);
    let points = resolved
        .iter()
        .filter(|c| {
            c["persistent"] == true
                && (c["id"].as_str().unwrap().ends_with("Foot")
                    || c["id"].as_str().unwrap().ends_with("Forefoot"))
        })
        .map(|c| {
            [
                c["point"]["x"].as_f64().unwrap() as f32,
                c["point"]["z"].as_f64().unwrap() as f32,
            ]
        })
        .collect::<Vec<_>>();
    let geometric = standing_balance(s).unwrap();
    let intent = s.recovery.as_ref().map(|r| {
        json!({
        "root":r.stand_root.map(|p|crate::vec3(s.heading.inverse()*p.translation)),
        "goal":r.stand_goal.map(|q|crate::vec3((s.heading.inverse()*q).to_scaled_axis()))})
    });
    json!({"timeS":s.substeps_completed as f64/240.0,
        "com":crate::vec3(s.heading.inverse()*com),"velocity":crate::vec3(s.heading.inverse()*v),
        "acceleration":crate::vec3(s.heading.inverse()*((v-previous)/s.profile.dt_s)),
        "capture":crate::vec3(capture),"resolvedContacts":resolved,
        "resolvedInside":crate::stepping::inside_support(points.clone(),[capture.x,capture.z],0.003),
        "resolvedProjection":crate::stepping::project_support(&points,[capture.x,capture.z],0.003),
        "controllerBalance":geometric.map(|(q,inside)|json!({"inside":inside,"axis":crate::vec3((s.heading.inverse()*q).to_scaled_axis())})),
        "status":s.recovery_status(),"intent":intent,"actuation":diagnostic_actuation(s),
        "delivery":s.joints.iter().enumerate().filter_map(|(i,h)| {
            let j=s.world.impulse_joints.get((*h)?)?;
            Some(json!({"id":s.model.segments[i].id,"axes":j.data.motors[3..].iter().map(|m|json!({
                "signedNm":m.delivered_impulse/s.profile.dt_s,
                "signedNmConvention":"constraint row; not the generalized child torque",
                "body1AngularImpulseWorldNms":crate::vec3(m.delivered_angular_impulse_1),
                "body2AngularImpulseWorldNms":crate::vec3(m.delivered_angular_impulse_2),"absoluteNm":m.delivered_abs_impulse/s.profile.dt_s,"capNm":m.max_force
            })).collect::<Vec<_>>()}))
        }).collect::<Vec<_>>(),
        "snapshot":s.snapshot()})
}

#[test]
#[ignore = "explicit diagnostic writes fresh evidence; no recovery qualification"]
fn capture_support_trace() {
    let output = std::env::var("LH_SUPPORT_OUT").expect("LH_SUPPORT_OUT required");
    let out = std::path::Path::new(&output);
    assert!(!out.exists(), "Refuse existing evidence directory");
    std::fs::create_dir_all(out).unwrap();
    let seconds = std::env::var("LH_SUPPORT_SECONDS")
        .ok()
        .map(|s| s.parse::<f64>().unwrap())
        .unwrap_or(2.0);
    assert!(seconds > 0.0 && seconds <= 25.0);
    let profile: Profile =
        serde_json::from_str(include_str!("../../data/recovery-profile-v24.json")).unwrap();
    let fixtures: FixtureSet = serde_json::from_str(FIXTURES).unwrap();
    for fixture in fixtures
        .cases
        .into_iter()
        .filter(|f| f.id.starts_with("landed-crouch-left"))
    {
        let mut s = Simulation::build(
            fixture.heading,
            profile.clone(),
            true,
            None,
            Some(&fixture.poses),
        )
        .unwrap();
        let bodies = s.bodies.clone();
        let joints = s.joints.clone();
        let colliders = s.colliders.clone();
        let generation = s.generation;
        let initial = s.snapshot();
        let mut prior = velocity(&s);
        let mut trace = Vec::new();
        let mut failure = None;
        for _ in 0..(seconds * 60.0).ceil() as usize {
            if let Err(error) = s.advance_tick_observed(|state, _| {
                assert_eq!(state.bodies, bodies);
                assert_eq!(state.joints, joints);
                assert_eq!(state.colliders, colliders);
                assert_eq!(state.generation, generation);
                trace.push(observe(state, prior));
                prior = velocity(state);
                Ok(())
            }) {
                failure = Some(error);
                break;
            }
        }
        let report = json!({"scope":"Read-only solved support diagnostic; no physical success assertion",
            "id":fixture.id,"heading":fixture.heading,"profile":profile,"initial":initial,"trace":trace,
            "failure":failure,"final":s.snapshot(),"releaseAccepted":false});
        std::fs::write(
            out.join(format!("{}.json", fixture.id)),
            serde_json::to_vec(&report).unwrap(),
        )
        .unwrap();
    }
    std::fs::write(out.join("canonical.json"), lh_model::CANONICAL_JSON).unwrap();
}

#[test]
#[ignore = "read-only geometry inspection of a supplied static plan"]
fn inspect_static_geometry() {
    let input = std::env::var("LH_STATIC_PLAN").expect("LH_STATIC_PLAN required");
    let output = std::env::var("LH_GEOMETRY_OUT").expect("LH_GEOMETRY_OUT required");
    assert!(!std::path::Path::new(&output).exists());
    let plan: Value = serde_json::from_slice(&std::fs::read(input).unwrap()).unwrap();
    let profile: Profile =
        serde_json::from_str(include_str!("../../data/recovery-profile-v24.json")).unwrap();
    let mut reports = Vec::new();
    for (n, row) in plan["rows"].as_array().unwrap().iter().enumerate() {
        let poses: Vec<BodyPose> = serde_json::from_value(row["poses"].clone()).unwrap();
        let mut s = Simulation::build(0.0, profile.clone(), true, None, Some(&poses)).unwrap();
        let before = serde_json::to_value(s.snapshot()).unwrap();
        let failure = measurement::inspect(&mut s, 0).err();
        let after = serde_json::to_value(s.snapshot()).unwrap();
        assert_eq!(before["segments"], after["segments"]);
        assert_eq!(s.substeps_completed, 0);
        let exact_limit_excess = s
            .model
            .segments
            .iter()
            .enumerate()
            .filter_map(|(i, p)| p.joint_profile.as_ref().map(|j| (i, j)))
            .flat_map(|(i, p)| {
                let j = s.world.impulse_joints.get(s.joints[i].unwrap()).unwrap();
                let q = measurement::coordinates(
                    *s.world.bodies[j.body1()].rotation(),
                    *s.world.bodies[s.bodies[i]].rotation(),
                    p,
                );
                p.axes.iter().map(move |a| {
                    let k = match a.coordinate {
                        'x' => 0,
                        'y' => 1,
                        _ => 2,
                    };
                    (q[k] - a.max_radians).max(a.min_radians - q[k]).max(0.0)
                })
            })
            .fold(0.0_f32, f32::max);
        reports.push(json!({"row":n,"failure":failure,"exactLimitExcessRad":exact_limit_excess,"diagnostics":s.diagnostics}));
    }
    std::fs::write(output,serde_json::to_vec_pretty(&json!({"scope":"Native convex geometry inspection at sampled poses only; zero integration, no interpolation or recovery claim","reports":reports})).unwrap()).unwrap();
}
