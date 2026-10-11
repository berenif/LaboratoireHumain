//! One paired test-only equilibrium diagnosis; no world-state mutation.
use super::*;
use rapier3d::math::Matrix;

const BRANCH_STEP: u64 = 1268;
const END_STEP: u64 = BRANCH_STEP + 120;
const WALL_BUDGET_S: f64 = 45.0;
struct Audit {
    gravity: bool,
    started: Instant,
    first_stall: Option<measurement::Failure>,
}
thread_local! {static RUN:RefCell<Option<Audit>>=const{RefCell::new(None)};}
pub(super) fn enabled() -> bool {
    RUN.with(|r| r.borrow().is_some())
}
pub(super) fn remember_stall(f: measurement::Failure) {
    RUN.with(|r| {
        let mut r = r.borrow_mut();
        r.as_mut().unwrap().first_stall.get_or_insert(f);
    });
}
pub(super) fn finished(s: &Simulation) -> bool {
    enabled() && s.substeps_completed >= END_STEP
}
pub(super) fn wall_guard(s: &Simulation, substep: u32) -> Result<(), measurement::Failure> {
    RUN.with(|r| {
        if let Some(r) = r.borrow().as_ref()
            && r.started.elapsed().as_secs_f64() > WALL_BUDGET_S
        {
            return Err(measurement::Failure::new(
                s.tick,
                substep,
                "left-hand-audit-wall",
                "paired diagnosis budget".into(),
                r.started.elapsed().as_secs_f64() as f32,
                WALL_BUDGET_S as f32,
            ));
        }
        Ok(())
    })
}
pub(super) fn summary() -> Value {
    RUN.with(|r|r.borrow().as_ref().map_or(Value::Null,|r|json!({"scope":"One same-world first-hand task diagnostic; original refusal retained, no recovery qualification","gravityCompensation":r.gravity,"branchStep":BRANCH_STEP,"endStep":END_STEP,"elapsedNativeWallS":r.started.elapsed().as_secs_f64(),"wallBudgetS":WALL_BUDGET_S,"firstOriginalStall":r.first_stall})))
}
fn frame_jacobian(s: &Simulation, i: usize) -> (Rotation, Matrix, Vector) {
    let p = s.model.segments[i].joint_profile.as_ref().unwrap();
    let h = s.joints[i].unwrap();
    let j = s.world.impulse_joints.get(h).unwrap();
    let parent = &s.world.bodies[j.body1()];
    let child = &s.world.bodies[j.body2()];
    let frame = *parent.rotation() * rotation(p.parent_frame.rotation);
    let rel = frame.inverse() * (*child.rotation() * rotation(p.child_frame.rotation));
    let v = rel.xyz();
    let w = rel.w;
    let jac = |k: usize| {
        let e = [Vector::X, Vector::Y, Vector::Z][k];
        (w * w * e + w * v.cross(e) + v[k] * v) / (w * w + v[k] * v[k])
    };
    (
        frame,
        Matrix::from_cols(jac(0), jac(1), jac(2)),
        parent.translation() + *parent.rotation() * vector(p.parent_frame.anchor),
    )
}
fn subtree(s: &Simulation, i: usize) -> Vec<usize> {
    let id = &s.model.segments[i].id;
    (0..s.bodies.len())
        .filter(|&k| {
            let mut cursor = k;
            loop {
                if s.model.segments[cursor].id == *id {
                    return true;
                }
                let Some(parent) = &s.model.segments[cursor].parent else {
                    return false;
                };
                cursor = s
                    .model
                    .segments
                    .iter()
                    .position(|p| p.id == *parent)
                    .unwrap();
            }
        })
        .collect()
}
fn gravity(s: &Simulation, i: usize) -> Vector {
    let (frame, jac, anchor) = frame_jacobian(s, i);
    let torque = subtree(s, i)
        .iter()
        .map(|&k| {
            let b = &s.world.bodies[s.bodies[k]];
            (b.mass_properties().world_com - anchor).cross(Vector::new(0.0, -9.81 * b.mass(), 0.0))
        })
        .sum::<Vector>();
    jac.inverse() * (frame.inverse() * torque)
}
pub(super) fn adjust(s: &Simulation, dq: &mut Coordinates) -> Result<Value, measurement::Failure> {
    let active = RUN.with(|r| r.borrow().as_ref().is_some_and(|r| r.gravity));
    if !active || s.substeps_completed < BRANCH_STEP {
        return Ok(Value::Null);
    }
    compensate(s, dq, &["left"])
}
pub(super) fn compensate(
    s: &Simulation,
    dq: &mut Coordinates,
    sides: &[&str],
) -> Result<Value, measurement::Failure> {
    let scale = s
        .profile
        .motor_scale
        .min(s.profile.aggregate_motor_cap_nm / s.motor_ceiling_sum);
    let mut rows = Vec::new();
    for (i, p) in s
        .model
        .segments
        .iter()
        .enumerate()
        .filter(|(_, p)| sides.iter().any(|side| p.id.starts_with(side)) && arm(&p.id))
    {
        let torque = -gravity(s, i);
        for a in &p.joint_profile.as_ref().unwrap().axes {
            let k = match a.coordinate {
                'x' => 0,
                'y' => 1,
                _ => 2,
            };
            let damping = a.damping_nms_per_rad * s.profile.posture_damping_multiplier * scale;
            if !damping.is_finite() || damping <= 0.0 || !torque[k].is_finite() {
                return Err(measurement::Failure::new(
                    s.tick,
                    0,
                    "left-hand-gravity-command",
                    p.id.clone(),
                    damping,
                    0.0,
                ));
            }
            let bias = torque[k] / damping;
            dq[i][k] += bias;
            rows.push(json!({"id":p.id,"coordinate":a.coordinate,"gravityCompensationNm":torque[k],"motorVelocityBiasRadps":bias,"unchangedDampingNmsPerRad":damping}));
        }
    }
    Ok(
        json!({"scope":"Gravity feedforward through existing bounded joint motors, no applied body force/torque","axes":rows}),
    )
}
pub(super) fn observe(s: &Simulation) -> Value {
    observe_arms(s, &["left"])
}
pub(super) fn observe_arms(s: &Simulation, sides: &[&str]) -> Value {
    let mut pairs = Vec::new();
    let label = |h: ColliderHandle| {
        s.colliders.iter().position(|c| *c == h).map_or_else(
            || {
                if h == s.floor {
                    "floor".to_string()
                } else {
                    "other".to_string()
                }
            },
            |i| s.model.segments[i].id.clone(),
        )
    };
    for pair in s.world.narrow_phase.contact_pairs() {
        let a = label(pair.collider1);
        let b = label(pair.collider2);
        if !sides
            .iter()
            .any(|side| a.starts_with(side) && arm(&a) || b.starts_with(side) && arm(&b))
        {
            continue;
        }
        for m in pair.solver_manifolds() {
            let mut points = Vec::new();
            let mut impulse = 0.0;
            for c in &m.data.solver_contacts {
                let p = &m.points[c.contact_indices()[0] as usize];
                let (x, y) = m.data.solver_contact_world_points(c, &s.world.bodies);
                impulse += p.data.impulse.max(0.0);
                points.push(json!({"point1":crate::vec3(x),"point2":crate::vec3(y),"distanceM":c.dist,"normalImpulseNms":p.data.impulse,"solverLever1M":crate::vec3(p.data.solver_dp1),"solverLever2M":crate::vec3(p.data.solver_dp2),"rawTangentImpulse":p.data.tangent_impulse}));
            }
            pairs.push(json!({"collider1":a,"collider2":b,"computeImpulses":m.data.solver_flags.contains(SolverFlags::COMPUTE_IMPULSES),"normalForceOn1N":crate::vec3(-m.data.normal*(impulse/s.profile.dt_s)),"tangentForceOn1N":crate::vec3(m.data.reported_tangent_impulse/s.profile.dt_s),"points":points}));
        }
    }
    let mut motors = Vec::new();
    let mut bodies = Vec::new();
    for (i, p) in s
        .model
        .segments
        .iter()
        .enumerate()
        .filter(|(_, p)| sides.iter().any(|side| p.id.starts_with(side)) && arm(&p.id))
    {
        let b = &s.world.bodies[s.bodies[i]];
        let j = s.world.impulse_joints.get(s.joints[i].unwrap()).unwrap();
        let profile = p.joint_profile.as_ref().unwrap();
        let q = measurement::coordinates(
            *s.world.bodies[j.body1()].rotation(),
            *b.rotation(),
            profile,
        );
        let (frame, jac, anchor) = frame_jacobian(s, i);
        let omega = frame.inverse() * (b.angvel() - s.world.bodies[j.body1()].angvel());
        let rates = jac.transpose() * omega;
        bodies.push(json!({"id":p.id,"massKg":b.mass(),"worldComM":crate::vec3(b.mass_properties().world_com),"worldInertiaKgM2":b.mass_properties().effective_world_inv_inertia.inverse(),"angularDamping":b.angular_damping(),"linearDamping":b.linear_damping(),"userForceN":crate::vec3(b.user_force()),"userTorqueNm":crate::vec3(b.user_torque())}));
        let rows=profile.axes.iter().map(|a|{let k=match a.coordinate{'x'=>0,'y'=>1,_=>2};let axis=crate::axis(a.coordinate).unwrap() as usize;let m=&j.data.motors[axis];json!({"coordinate":a.coordinate,"measuredRad":q[k],"measuredRateRadps":rates[k],"installedTargetRad":m.target_pos,"installedVelocityRadps":m.target_vel,"installedStiffnessNmPerRad":m.stiffness,"installedDampingNmsPerRad":m.damping,"installedCapNm":m.max_force,"motorModel":format!("{:?}",m.model),"limitImpulseNms":j.data.limits[axis].impulse,"minRad":a.min_radians,"maxRad":a.max_radians})}).collect::<Vec<_>>();
        motors.push(json!({"id":p.id,"jointBody1IsCanonicalParent":s.model.segments[p.parent.as_ref().and_then(|name|s.model.segments.iter().position(|x|x.id==*name)).unwrap()].id==s.model.segments[s.bodies.iter().position(|h|*h==j.body1()).unwrap()].id,"parentWorldFrameRotation":crate::quat(frame),"parentWorldAnchorM":crate::vec3(anchor),"gravityGeneralizedNm":crate::vec3(gravity(s,i)),"canonicalParentFrame":profile.parent_frame,"canonicalChildFrame":profile.child_frame,"nativeLocalFrame1":j.data.local_frame1,"nativeLocalFrame2":j.data.local_frame2,"jointConstraintImpulses":j.impulses,"axes":rows}));
    }
    json!({"scope":"Read-only actual native motors, mass properties and all selected-arm contacts","motors":motors,"bodies":bodies,"allSelectedArmContactManifolds":pairs})
}
#[test]
#[ignore = "one declared short diagnosis case; run reference before corrected case"]
fn diagnose_left_hand_one() {
    let kind = std::env::var("LH_LEFT_HAND_AUDIT_KIND").expect("kind required");
    assert!(kind == "reference" || kind == "gravity");
    let out = PathBuf::from(std::env::var("LH_LEFT_HAND_AUDIT_OUT").expect("out required"));
    assert!(!out.exists());
    std::fs::create_dir_all(&out).unwrap();
    let plan = Plan::load(&std::env::var("LH_DESCENT_PLAN").expect("C plan required")).unwrap();
    assert!(plan.stop_at_base && plan.phase_gate.is_some());
    let fixture = serde_json::from_str::<FixtureSet>(FIXTURES)
        .unwrap()
        .cases
        .into_iter()
        .find(|x| x.id == "landed-crouch-left")
        .unwrap();
    let started = Instant::now();
    RUN.with(|r| {
        *r.borrow_mut() = Some(Audit {
            gravity: kind == "gravity",
            started,
            first_stall: None,
        })
    });
    let case = run_with_policy(
        &fixture,
        &crate::recovery_contract::pinned_profile(),
        &plan,
        ArmMode::Prepared,
        fixture.heading,
        started,
        ObservationPolicy::ProneRecovery,
    )
    .unwrap();
    std::fs::write(out.join("case.json"), serde_json::to_vec(&case).unwrap()).unwrap();
    std::fs::write(out.join("summary.json"),serde_json::to_vec_pretty(&json!({"caseCount":1,"kind":kind,"physicalSeconds":case["physicalSeconds"],"elapsedNativeWallS":started.elapsed().as_secs_f64(),"physicalGuardFailure":case["failure"],"retainedPreparationRefusal":case["leftHandAuditSummary"]["firstOriginalStall"],"finalPreparationObservation":case["proneFinalObservation"],"recoveryQualified":false})).unwrap()).unwrap();
    RUN.with(|r| *r.borrow_mut() = None);
}
