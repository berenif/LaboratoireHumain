//! Pure bounded arm FK/reference projection. Native bodies are read-only.
use super::*;
use rapier3d::{math::Matrix, na};

#[derive(Default)]
pub(super) struct State {
    last_q: Option<Coordinates>,
    last_step: u64,
}
pub(super) struct Task {
    pub stage: usize,
    pub progress: f32,
    pub reference_torso: Pose,
    pub land_goals: [Vector; 2],
}
struct Kinematics {
    poses: [Pose; 25],
    anchors: [Vector; 25],
    directions: [Matrix; 25],
}
fn index(s: &Simulation, id: &str) -> usize {
    s.model.segments.iter().position(|p| p.id == id).unwrap()
}
fn arm_indices(s: &Simulation, side: &str) -> Vec<usize> {
    [
        "ShoulderGirdle",
        "UpperArm",
        "Forearm",
        "ForearmTwist",
        "Hand",
    ]
    .iter()
    .map(|name| index(s, &format!("{side}{name}")))
    .collect()
}
fn fk(s: &Simulation, q: &Coordinates, arm: &[usize], torso: Pose) -> Kinematics {
    let mut result = Kinematics {
        poses: std::array::from_fn(|i| *s.world.bodies[s.bodies[i]].position()),
        anchors: [Vector::ZERO; 25],
        directions: [Matrix::IDENTITY; 25],
    };
    result.poses[index(s, "torso")] = torso;
    for &i in arm {
        let p = s.model.segments[i].joint_profile.as_ref().unwrap();
        let parent = index(s, s.model.segments[i].parent.as_ref().unwrap());
        let frame = result.poses[parent].rotation * rotation(p.parent_frame.rotation);
        let t = q[i].map(|v| (v * 0.5).tan());
        let relative = Rotation::from_xyzw(t[0], t[1], t[2], 1.0).normalize();
        let vv = relative.xyz();
        let w = relative.w;
        let row = |k: usize| {
            let e = [Vector::X, Vector::Y, Vector::Z][k];
            (w * w * e + w * vv.cross(e) + vv[k] * vv) / (w * w + vv[k] * vv[k])
        };
        let jac_transpose = Matrix::from_cols(row(0), row(1), row(2));
        let inverse = jac_transpose.transpose().inverse();
        result.directions[i] = Matrix::from_cols(
            frame * inverse.col(0),
            frame * inverse.col(1),
            frame * inverse.col(2),
        );
        let anchor = result.poses[parent].translation
            + result.poses[parent].rotation * vector(p.parent_frame.anchor);
        let child_rotation = frame * relative * rotation(p.child_frame.rotation).inverse();
        result.poses[i] = Pose::from_parts(
            anchor - child_rotation * vector(p.child_frame.anchor),
            child_rotation,
        );
        result.anchors[i] = anchor;
    }
    result
}
fn vertices(s: &Simulation, i: usize, pose: Pose) -> Vec<Vector> {
    s.model.segments[i]
        .geometry
        .vertices
        .chunks_exact(3)
        .map(|p| pose.translation + pose.rotation * Vector::new(p[0], p[1], p[2]))
        .collect()
}
// A fixed torso face plane gives a conservative separation certificate.
// Both directions are required: canonical faces do not assume left/right signs.
fn skin_plane(s: &Simulation, k: &Kinematics, upper: usize) -> (f32, Vector, Vector) {
    let torso = index(s, "torso");
    let tv = vertices(s, torso, k.poses[torso]);
    let av = vertices(s, upper, k.poses[upper]);
    let mut best = (f32::NEG_INFINITY, Vector::ZERO, Vector::ZERO);
    for tri in s.model.segments[torso].geometry.indices.chunks_exact(3) {
        let n = (tv[tri[1] as usize] - tv[tri[0] as usize])
            .cross(tv[tri[2] as usize] - tv[tri[0] as usize]);
        if n.length_squared() < 1e-12 {
            continue;
        }
        for n in [n.normalize(), -n.normalize()] {
            let top = tv
                .iter()
                .map(|p| n.dot(*p))
                .fold(f32::NEG_INFINITY, f32::max);
            let point = *av
                .iter()
                .min_by(|a, b| n.dot(**a).total_cmp(&n.dot(**b)))
                .unwrap();
            let gap = n.dot(point) - top;
            if gap > best.0 {
                best = (gap, n, point);
            }
        }
    }
    best
}
fn project(s: &Simulation, q: &mut Coordinates, side: &str, goal: Vector) -> Value {
    let arm = arm_indices(s, side);
    let upper = arm[1];
    let hand = arm[4];
    let torso = *s.world.bodies[s.bodies[index(s, "torso")]].position();
    let skin = s.world.colliders[s.colliders[index(s, "torso")]].contact_skin()
        + s.world.colliders[s.colliders[upper]].contact_skin();
    let axes: Vec<_> = arm
        .iter()
        .flat_map(|&i| {
            s.model.segments[i]
                .joint_profile
                .as_ref()
                .unwrap()
                .axes
                .iter()
                .map(move |a| {
                    let k = match a.coordinate {
                        'x' => 0,
                        'y' => 1,
                        _ => 2,
                    };
                    (i, k, a.min_radians, a.max_radians)
                })
        })
        .collect();
    let mut iterations = 0;
    for _ in 0..12 {
        iterations += 1;
        let kin = fk(s, q, &arm, torso);
        let (gap, normal, point) = skin_plane(s, &kin, upper);
        let error = goal - kin.poses[hand].translation;
        let need = (skin - gap).max(0.0);
        if error.length() < 0.00005 && need < 0.00001 {
            break;
        }
        let mut a = na::SMatrix::<f32, 4, 10>::zeros();
        for (col, &(i, axis, _, _)) in axes.iter().enumerate() {
            let direction = kin.directions[i].col(axis);
            let hand_derivative = direction.cross(kin.poses[hand].translation - kin.anchors[i]);
            for row in 0..3 {
                a[(row, col)] = hand_derivative[row];
            }
            if need > 0.0 && (i == arm[0] || i == upper) {
                a[(3, col)] = normal.dot(direction.cross(point - kin.anchors[i]));
            }
        }
        let rhs = na::SVector::<f32, 4>::new(error.x, error.y, error.z, need);
        let matrix = a * a.transpose() + na::SMatrix::<f32, 4, 4>::identity() * 1e-6;
        let Some(solution) = matrix.lu().solve(&rhs) else {
            break;
        };
        let delta = a.transpose() * solution;
        let scale = (0.12 / delta.amax().max(1e-12)).min(1.0);
        for (col, &(i, axis, low, high)) in axes.iter().enumerate() {
            q[i][axis] = (q[i][axis] + delta[col] * scale).clamp(low, high);
        }
    }
    let kin = fk(s, q, &arm, torso);
    let (gap, _, _) = skin_plane(s, &kin, upper);
    json!({"side":side,"iterations":iterations,"goal":crate::vec3(goal),
        "projectedGoalErrorM":(goal-kin.poses[hand].translation).length(),
        "projectedNativeSkinSeparationLowerBoundM":gap,"nativeCombinedSkinM":skin})
}
#[path = "bounded_arm_clearance.rs"]
mod bounded_arm_clearance;
pub(super) fn chain_clearance_projection(
    s: &Simulation,
    q: &mut Coordinates,
    side: &str,
    extra: f32,
    warm: Option<&Coordinates>,
) -> Result<Value, Value> {
    bounded_arm_clearance::project(s, q, side, extra, warm)
}

#[test]
fn exact_recorded_projection_is_bounded_and_nonpenetrating_without_world_mutation() {
    let record: Value =
        serde_json::from_str(include_str!("prone_roll_projection_regression.json")).unwrap();
    // Geometry-only fixture, never integrated: restore all measured poses and velocities exactly.
    let mut s = Simulation::build(
        0.,
        crate::recovery_contract::pinned_profile(),
        true,
        None,
        None,
    )
    .unwrap();
    let poses: Vec<lh_model::Pose> = serde_json::from_value(record["segments"].clone()).unwrap();
    for (i, p) in poses.iter().enumerate() {
        s.world.bodies[s.bodies[i]].set_position(
            Pose::from_parts(vector(p.position), rotation(p.rotation)),
            false,
        );
        s.world.bodies[s.bodies[i]].set_linvel(vector(p.linear_velocity), false);
        s.world.bodies[s.bodies[i]].set_angvel(vector(p.angular_velocity), false);
    }
    let before = serde_json::to_value(&s.world.bodies).unwrap();
    let targets = s.targets.clone();
    let mut q: Coordinates =
        serde_json::from_value(record["oldTargetCoordinates"].clone()).unwrap();
    let arm = arm_indices(&s, "right");
    let torso = *s.world.bodies[s.bodies[index(&s, "torso")]].position();
    assert!(
        vertices(&s, arm[1], fk(&s, &q, &arm, torso).poses[arm[1]])
            .iter()
            .any(|p| p.y < -0.005)
    );
    let certificate = chain_clearance_projection(&s, &mut q, "right", 0.005, None).unwrap();
    println!(
        "exact-state certificate: {} coordinates: {:?}",
        certificate, q
    );
    assert_eq!(certificate["status"], "certified");
    assert!(certificate["maximumConstraintResidualM"].as_f64().unwrap() <= 0.000001);
    for link in certificate["links"].as_array().unwrap() {
        assert!(
            link["projectedLowestY"].as_f64().unwrap() >= link["requiredLowestY"].as_f64().unwrap()
        );
    }
    assert_eq!(before, serde_json::to_value(&s.world.bodies).unwrap());
    assert_eq!(targets, s.targets);
    let original = q;
    let invalid_warm: Coordinates =
        serde_json::from_value(record["oldTargetCoordinates"].clone()).unwrap();
    let preferred =
        chain_clearance_projection(&s, &mut q, "right", 0.005, Some(&invalid_warm)).unwrap();
    assert_eq!(preferred["start"], 0);
    assert_eq!(q, original);

    let error = chain_clearance_projection(&s, &mut q, "right", 10., None).unwrap_err();
    assert_eq!(error["status"], "not-converged");
    assert_eq!(error["commandCommitted"], false);
    assert_eq!(q, original);
    assert_eq!(before, serde_json::to_value(&s.world.bodies).unwrap());
    assert_eq!(targets, s.targets);
}

#[test]
fn whole_arm_vertex_jacobians_match_native_coordinates_without_mutating_bodies() {
    let fixture = serde_json::from_str::<FixtureSet>(FIXTURES)
        .unwrap()
        .cases
        .into_iter()
        .find(|x| x.id == "landed-crouch-left")
        .unwrap();
    let s = Simulation::build(
        fixture.heading,
        crate::recovery_contract::pinned_profile(),
        true,
        None,
        Some(&fixture.poses),
    )
    .unwrap();
    let original = serde_json::to_value(&s.world.bodies).unwrap();
    let q = offline_trajectory::coordinates(&s).unwrap();
    let arm = arm_indices(&s, "right");
    let torso = *s.world.bodies[s.bodies[index(&s, "torso")]].position();
    let kin = fk(&s, &q, &arm, torso);
    for (row, &body) in arm.iter().enumerate() {
        let vs = &s.model.segments[body].geometry.vertices;
        let local = Vector::new(vs[0], vs[1], vs[2]);
        let point = kin.poses[body].translation + kin.poses[body].rotation * local;
        for &ancestor in &arm[..=row] {
            for axis in &s.model.segments[ancestor]
                .joint_profile
                .as_ref()
                .unwrap()
                .axes
            {
                let k = match axis.coordinate {
                    'x' => 0,
                    'y' => 1,
                    _ => 2,
                };
                let mut plus = q;
                let mut minus = q;
                plus[ancestor][k] += 0.001;
                minus[ancestor][k] -= 0.001;
                let p = fk(&s, &plus, &arm, torso).poses[body];
                let m = fk(&s, &minus, &arm, torso).poses[body];
                let numeric =
                    (p.translation + p.rotation * local - m.translation - m.rotation * local)
                        / 0.002;
                let analytic = kin.directions[ancestor]
                    .col(k)
                    .cross(point - kin.anchors[ancestor]);
                assert!(
                    (numeric - analytic).length() < 0.002,
                    "{} {} {}",
                    s.model.segments[body].id,
                    s.model.segments[ancestor].id,
                    k
                );
            }
        }
    }
    assert_eq!(serde_json::to_value(&s.world.bodies).unwrap(), original);
}

impl State {
    pub(super) fn adjust(
        &mut self,
        s: &Simulation,
        q: &mut Coordinates,
        dq: &mut Coordinates,
        task: Task,
    ) -> Value {
        let started = Instant::now();
        let nominal = *q;
        let mut reports = Vec::new();
        for (side, land_stage, fixed_goal) in [
            ("left", 2, task.land_goals[0]),
            ("right", 5, task.land_goals[1]),
        ] {
            if task.stage < land_stage {
                continue;
            }
            let arm = arm_indices(s, side);
            let goal = if task.stage == land_stage {
                fk(s, &nominal, &arm, task.reference_torso).poses[arm[4]].translation
            } else {
                fixed_goal
            };
            let mut projected = nominal;
            let mut report = project(s, &mut projected, side, goal);
            let blend = if task.stage == land_stage {
                task.progress
            } else {
                1.0
            };
            report["blendWithinOriginalLandDuration"] = json!(blend);
            reports.push(report);
            for &i in &arm {
                for axis in &s.model.segments[i].joint_profile.as_ref().unwrap().axes {
                    let k = match axis.coordinate {
                        'x' => 0,
                        'y' => 1,
                        _ => 2,
                    };
                    q[i][k] = nominal[i][k] + blend * (projected[i][k] - nominal[i][k]);
                    if let Some(previous) = &self.last_q
                        && self.last_step + 1 == s.substeps_completed
                    {
                        dq[i][k] = (q[i][k] - previous[i][k]) / s.profile.dt_s;
                    }
                }
            }
        }
        self.last_q = Some(*q);
        self.last_step = s.substeps_completed;
        json!({"scope":"Same land goals from measured torso, native-skin clearance; motor references only",
            "arms":reports,"projectionWallS":started.elapsed().as_secs_f64()})
    }
}

#[test]
fn arm_fk_and_endpoint_jacobian_match_the_native_coordinate_convention() {
    let fixture = serde_json::from_str::<FixtureSet>(FIXTURES)
        .unwrap()
        .cases
        .into_iter()
        .find(|x| x.id == "landed-crouch-left")
        .unwrap();
    let s = Simulation::build(
        fixture.heading,
        crate::recovery_contract::pinned_profile(),
        true,
        None,
        Some(&fixture.poses),
    )
    .unwrap();
    let q = offline_trajectory::coordinates(&s).unwrap();
    let original = serde_json::to_value(&s.world.bodies).unwrap();
    for side in ["left", "right"] {
        let arm = arm_indices(&s, side);
        let torso = *s.world.bodies[s.bodies[index(&s, "torso")]].position();
        let kin = fk(&s, &q, &arm, torso);
        assert!(
            (kin.poses[arm[4]].translation - s.world.bodies[s.bodies[arm[4]]].translation())
                .length()
                < 1e-5
        );
        for &i in &arm {
            for a in &s.model.segments[i].joint_profile.as_ref().unwrap().axes {
                let k = match a.coordinate {
                    'x' => 0,
                    'y' => 1,
                    _ => 2,
                };
                let mut plus = q;
                let mut minus = q;
                plus[i][k] += 0.001;
                minus[i][k] -= 0.001;
                let numeric = (fk(&s, &plus, &arm, torso).poses[arm[4]].translation
                    - fk(&s, &minus, &arm, torso).poses[arm[4]].translation)
                    / 0.002;
                let analytic = kin.directions[i]
                    .col(k)
                    .cross(kin.poses[arm[4]].translation - kin.anchors[i]);
                assert!((numeric - analytic).length() < 0.002, "{side} {i} {k}");
            }
        }
    }
    assert_eq!(serde_json::to_value(&s.world.bodies).unwrap(), original);
}
