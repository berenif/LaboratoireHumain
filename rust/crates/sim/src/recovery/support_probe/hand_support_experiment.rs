//! Opt-in A/B native test of an anticipated hand/shin support corridor.
//! Only anatomical joint motors are commanded; the production controller is unchanged.
use super::offline_trajectory::{self, Allocation, Coordinates};
use super::*;
use rapier3d::utils::AngularInertiaOps;
use serde::{Deserialize, Serialize};
use std::{cell::RefCell, path::PathBuf, time::Instant};

#[path = "land_arm_tracking.rs"]
mod land_arm_tracking;
#[path = "left_hand_command_audit.rs"]
mod left_hand_command_audit;
#[path = "prone_chain_execution.rs"]
mod prone_chain_execution;
#[path = "prone_recovery_experiment.rs"]
mod prone_recovery_experiment;

const MAX_WALL_S: f64 = 120.0;

#[derive(Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
struct Row {
    time_s: f32,
    coordinates: Coordinates,
}

#[derive(Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
struct PhaseGate {
    at_time_s: f32,
}

fn default_horizon() -> f32 {
    3.0
}

#[derive(Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
struct Plan {
    rows: Vec<Row>,
    #[serde(default)]
    phase_gate: Option<PhaseGate>,
    #[serde(default = "default_horizon")]
    native_horizon_s: f32,
    #[serde(default)]
    stop_at_base: bool,
}

impl Plan {
    fn load(path: &str) -> Result<Self, String> {
        let plan: Self = serde_json::from_slice(&std::fs::read(path).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
        if plan.rows.len() < 2
            || plan.rows.first().unwrap().time_s != 0.0
            || !plan.native_horizon_s.is_finite()
            || plan.native_horizon_s <= 0.0
            || plan.native_horizon_s > 6.0
            || plan.rows.last().unwrap().time_s > plan.native_horizon_s
            || !plan.rows.iter().all(|r| {
                r.time_s.is_finite() && r.coordinates.iter().flatten().all(|v| v.is_finite())
            })
            || !plan.rows.windows(2).all(|r| r[0].time_s < r[1].time_s)
        {
            return Err("invalid bounded hand corridor rows".into());
        }
        if plan.phase_gate.as_ref().is_some_and(|g| {
            !g.at_time_s.is_finite()
                || g.at_time_s <= 0.0
                || g.at_time_s >= plan.native_horizon_s
                || !plan.rows.iter().any(|r| r.time_s == g.at_time_s)
        }) {
            return Err("invalid declared phase gate".into());
        }
        Ok(plan)
    }
}

#[derive(Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
enum ArmMode {
    Reference,
    Prepared,
}

#[derive(Clone, Copy, PartialEq, Eq, Serialize)]
enum ObservationPolicy {
    StrictCorridor,
    LandingOnly,
    ProneRecovery,
}

fn arm(id: &str) -> bool {
    (id.starts_with("left") || id.starts_with("right"))
        && [
            "ShoulderGirdle",
            "UpperArm",
            "Forearm",
            "ForearmTwist",
            "Hand",
        ]
        .iter()
        .any(|x| id.ends_with(x))
}
fn allowed(id: &str) -> bool {
    id.ends_with("Foot") || id.ends_with("Forefoot") || id.ends_with("Hand") || id.ends_with("Shin")
}

fn command(
    plan: &Plan,
    s: &Simulation,
    initial: &Coordinates,
    mode: ArmMode,
) -> (Coordinates, Coordinates, usize) {
    let t = s.substeps_completed as f32 * s.profile.dt_s;
    command_at_time(plan, s, initial, mode, t)
}

fn command_at_time(
    plan: &Plan,
    s: &Simulation,
    initial: &Coordinates,
    mode: ArmMode,
    t: f32,
) -> (Coordinates, Coordinates, usize) {
    let j = plan
        .rows
        .partition_point(|r| r.time_s <= t)
        .saturating_sub(1)
        .min(plan.rows.len() - 2);
    let a = &plan.rows[j];
    let b = &plan.rows[j + 1];
    let duration = b.time_s - a.time_s;
    let u = ((t - a.time_s) / duration).clamp(0.0, 1.0);
    let u2 = u * u;
    let u3 = u2 * u;
    let mut q = [[0.0; 3]; 25];
    let mut dq = [[0.0; 3]; 25];
    for (i, part) in s.model.segments.iter().enumerate() {
        for k in 0..3 {
            let y0 = a.coordinates[i][k];
            let y1 = b.coordinates[i][k];
            let m0 = if j == 0 {
                0.0
            } else {
                (b.coordinates[i][k] - plan.rows[j - 1].coordinates[i][k])
                    / (b.time_s - plan.rows[j - 1].time_s)
            };
            let m1 = if j + 2 >= plan.rows.len() {
                0.0
            } else {
                (plan.rows[j + 2].coordinates[i][k] - a.coordinates[i][k])
                    / (plan.rows[j + 2].time_s - a.time_s)
            };
            q[i][k] = (2.0 * u3 - 3.0 * u2 + 1.0) * y0
                + (u3 - 2.0 * u2 + u) * duration * m0
                + (-2.0 * u3 + 3.0 * u2) * y1
                + (u3 - u2) * duration * m1;
            dq[i][k] = if t >= b.time_s {
                0.0
            } else {
                ((6.0 * u2 - 6.0 * u) * y0
                    + (3.0 * u2 - 4.0 * u + 1.0) * duration * m0
                    + (-6.0 * u2 + 6.0 * u) * y1
                    + (3.0 * u2 - 2.0 * u) * duration * m1)
                    / duration
            };
            if mode == ArmMode::Reference && arm(&part.id) {
                q[i][k] = initial[i][k];
                dq[i][k] = 0.0;
            }
        }
    }
    (q, dq, j)
}

struct Context {
    plan: Plan,
    initial: Coordinates,
    mode: ArmMode,
    last_command: Value,
    previous_com_velocity: Vector,
    trace: Vec<Value>,
    unsupported_steps: u64,
    terminal_steps: u64,
    max_terminal_steps: u64,
    max_cap_sum_nm: f64,
    max_unplanned_load_n: f32,
    first_hand: Option<Value>,
    first_shin: Option<Value>,
    first_unplanned: Option<Value>,
    reference_steps: u64,
    base_stable_steps: u64,
    max_base_stable_steps: u64,
    base_gate_open: bool,
    base_gate_open_time_s: Option<f32>,
    observation_policy: ObservationPolicy,
    first_corridor_failure: Option<measurement::Failure>,
    rest_steps: u64,
    max_rest_steps: u64,
    prone: Option<prone_recovery_experiment::Controller>,
}
thread_local! {static ACTIVE:RefCell<Option<Context>>=const{RefCell::new(None)};}

pub(crate) fn before_step(s: &mut Simulation, substep: u32) -> Result<(), measurement::Failure> {
    ACTIVE.with(|slot| {
        let mut slot = slot.borrow_mut();
        let Some(ctx) = slot.as_mut() else {
            return Ok(());
        };
        if let Some((q,mut dq,phase))=ctx.prone.as_ref().and_then(|p|p.command(s)) {
            let feedforward=if prone_chain_execution::enabled() {
                left_hand_command_audit::compensate(s,&mut dq,ctx.prone.as_ref().unwrap().gravity_sides())?
            }else{left_hand_command_audit::adjust(s,&mut dq)?};
            let (cap_sum,ratio,requests)=offline_trajectory::install_targets(s,&q,&dq,Allocation::OwnedRequests)
                .map_err(|e|measurement::Failure::new(s.tick,substep,"prone-motor-command",e,1.0,0.0))?;
            ctx.max_cap_sum_nm=ctx.max_cap_sum_nm.max(cap_sum);
            ctx.last_command=json!({"owner":"test-only-prone-recovery","phase":phase,"targetCoordinates":q,"targetVelocities":dq,
                "ownerRequestsAndCaps":requests,"capSumNm":cap_sum,"maxAnatomicalRatio":ratio,"originalMotion":s.motion});
            if !feedforward.is_null(){ctx.last_command["gravityFeedforward"]=feedforward;}
            return Ok(());
        }
        let reference_time = ctx.reference_steps as f32 * s.profile.dt_s;
        let time = if let Some(gate) = &ctx.plan.phase_gate {
            if ctx.base_gate_open {
                reference_time
            } else {
                reference_time.min(gate.at_time_s)
            }
        } else {
            s.substeps_completed as f32 * s.profile.dt_s
        };
        let (q, mut dq, row) = command_at_time(&ctx.plan, s, &ctx.initial, ctx.mode, time);
        if ctx.plan.phase_gate.as_ref().is_some_and(|g| {
            !ctx.base_gate_open && reference_time >= g.at_time_s
        }) {
            dq = [[0.0; 3]; 25];
        }
        let (cap_sum, ratio, requests) =
            offline_trajectory::install_targets(s, &q, &dq, Allocation::OwnedRequests).map_err(
                |e| measurement::Failure::new(s.tick, substep, "hand-motor-command", e, 1.0, 0.0),
            )?;
        ctx.max_cap_sum_nm = ctx.max_cap_sum_nm.max(cap_sum);
        ctx.last_command = json!({"owner":"hand-support-corridor","armMode":ctx.mode,"row":row,"referenceTimeS":time,
            "targetCoordinates":q,"targetVelocities":dq,"ownerRequestsAndCaps":requests,
            "capSumNm":cap_sum,"maxAnatomicalRatio":ratio,"originalMotion":s.motion});
        Ok(())
    })
}

fn contacts(s: &Simulation) -> (Vec<Value>, Vector) {
    let mut result = Vec::new();
    let mut hand_impulse = Vector::ZERO;
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
                let tangent = if first {
                    m.data.reported_tangent_impulse
                } else {
                    -m.data.reported_tangent_impulse
                };
                if s.model.segments[i].id.ends_with("Hand") {
                    let normal_impulse = m
                        .data
                        .solver_contacts
                        .iter()
                        .map(|c| {
                            m.points[c.contact_indices()[0] as usize]
                                .data
                                .impulse
                                .max(0.0)
                        })
                        .sum::<f32>();
                    hand_impulse += normal * normal_impulse + tangent;
                }
                for c in &m.data.solver_contacts {
                    let p = &m.points[c.contact_indices()[0] as usize];
                    if p.data.impulse <= 0.0 {
                        continue;
                    }
                    let (a, b) = m.data.solver_contact_world_points(c, &s.world.bodies);
                    let world = if first { a } else { b };
                    let body = &s.world.bodies[s.bodies[i]];
                    let v = body.linvel()
                        + body
                            .angvel()
                            .cross(world - body.mass_properties().world_com);
                    let local = body.rotation().inverse() * (world - body.translation());
                    let id = &s.model.segments[i].id;
                    result.push(json!({"id":id,"point":crate::vec3(s.heading.inverse()*world),
                        "segmentLocalPoint":crate::vec3(local),"normal":crate::vec3(s.heading.inverse()*normal),
                        "normalLoadN":normal.y*p.data.impulse/s.profile.dt_s,
                        "manifoldTangentImpulseNms":crate::vec3(s.heading.inverse()*tangent),
                        "manifoldTangentForceN":crate::vec3(s.heading.inverse()*(tangent/s.profile.dt_s)),
                        "tangentialSlipMps":(v.x*v.x+v.z*v.z).sqrt(),
                        "originalPersistentLoadN":super::super::load(s,id)}));
                }
            }
        }
    }
    (result, hand_impulse)
}

fn momentum(s: &Simulation) -> Value {
    let com = s.center_of_mass();
    let mut linear = Vector::ZERO;
    let mut angular = Vector::ZERO;
    for handle in &s.bodies {
        let b = &s.world.bodies[*handle];
        let m = b.mass();
        let p = b.linvel() * m;
        linear += p;
        angular += (b.mass_properties().world_com - com).cross(p);
        angular += b.mass_properties().effective_world_inv_inertia.inverse() * b.angvel();
    }
    json!({"linearKgMps":crate::vec3(s.heading.inverse()*linear),
        "angularAboutComKgM2ps":crate::vec3(s.heading.inverse()*angular)})
}

fn contact_sum(contacts: &[Value], id: &str) -> f32 {
    contacts
        .iter()
        .filter(|c| c["id"] == id)
        .filter_map(|c| c["normalLoadN"].as_f64())
        .sum::<f64>() as f32
}

fn observe_step(
    s: &Simulation,
    substep: u32,
    started: Instant,
) -> Result<(), measurement::Failure> {
    ACTIVE.with(|slot|{
        let mut slot=slot.borrow_mut();let ctx=slot.as_mut().expect("hand experiment context");
        let c=crate::recovery_contract::contract();let now=s.substeps_completed as f32*s.profile.dt_s;
        let (contact,hand_impulse)=contacts(s);
        let mut permitted_load=0.0_f32;let mut persistent_permitted_load=0.0_f32;let mut points=Vec::<[f32;2]>::new();
        let mut unexpected:Option<(String,f32)>=None;
        for part in &s.model.segments {
            let actual=contact_sum(&contact,&part.id);
            if allowed(&part.id) {
                permitted_load+=actual;
                persistent_permitted_load+=super::super::load(s,&part.id);
                if actual>=c.recovery_acceptance.minimum_load_n&&super::super::load(s,&part.id)>=c.recovery_acceptance.minimum_load_n {
                    for p in contact.iter().filter(|p|p["id"]==part.id) {
                        if let (Some(x),Some(z))=(p["point"]["x"].as_f64(),p["point"]["z"].as_f64()){points.push([x as f32,z as f32]);}
                    }
                }
            } else if actual>0.0 {
                ctx.max_unplanned_load_n=ctx.max_unplanned_load_n.max(actual);
                if actual>=c.recovery_acceptance.minimum_load_n&&super::super::load(s,&part.id)>=c.recovery_acceptance.minimum_load_n {
                    unexpected=Some((part.id.clone(),actual));
                    if ctx.first_unplanned.is_none(){ctx.first_unplanned=Some(json!({"id":part.id,"timeS":now,"loadN":actual}));}
                }
            }
        }
        let pair=|suffix:&str| ["left","right"].iter().all(|side|{
            let id=format!("{side}{suffix}");contact_sum(&contact,&id)>=c.recovery_acceptance.minimum_load_n
                &&super::super::load(s,&id)>=c.recovery_acceptance.minimum_load_n
        });
        let hands=pair("Hand");let shins=pair("Shin");
        if hands&&ctx.first_hand.is_none(){ctx.first_hand=Some(json!({"timeS":now}));}
        if shins&&ctx.first_shin.is_none(){ctx.first_shin=Some(json!({"timeS":now}));}
        let mass=s.total_mass();let com=s.center_of_mass();let v=super::velocity(s);
        let local_com=s.heading.inverse()*com;
        let capture=s.heading.inverse()*(com+v*((com.y-s.support_height()).max(0.2)/9.81).sqrt());
        let inside=crate::stepping::inside_support(points.clone(),[local_com.x,local_com.z],0.003)
            &&crate::stepping::inside_support(points,[capture.x,capture.z],0.003);
        let all_still=s.bodies.iter().all(|h|{let b=&s.world.bodies[*h];
            b.linvel().length()<=c.recovery_acceptance.stable_linear_mps
                &&b.angvel().length()<=c.recovery_acceptance.stable_angular_radps});
        let rolling_load=s.model.segments.iter().filter(|part|part.id!="head"&&part.id!="neck")
            .map(|part|super::super::load(s,&part.id)).sum::<f32>();
        let all_settled=s.bodies.iter().all(|h|{let b=&s.world.bodies[*h];
            b.linvel().length()<=c.recovery_acceptance.settle_linear_mps
                &&b.angvel().length()<=c.recovery_acceptance.settle_angular_radps});
        // Observe rest after the original corridor refusal, not the low-motion
        // initial crouch in its deterministic replay prefix.
        ctx.rest_steps=if ctx.observation_policy!=ObservationPolicy::StrictCorridor
            &&!ctx.prone.as_ref().is_some_and(|p|p.active())
            &&ctx.first_corridor_failure.is_some()
            &&rolling_load>=c.recovery_acceptance.minimum_load_n&&all_settled {ctx.rest_steps+1}else{0};
        ctx.max_rest_steps=ctx.max_rest_steps.max(ctx.rest_steps);
        let terminal=hands&&shins&&permitted_load>=0.8*mass*9.81&&inside&&all_still&&unexpected.is_none();
        let base_stable=shins&&permitted_load>=0.8*mass*9.81&&inside&&all_still&&unexpected.is_none();
        if let Some(gate)=&ctx.plan.phase_gate {
            let at_gate=ctx.reference_steps as f32*s.profile.dt_s>=gate.at_time_s;
            ctx.base_stable_steps=if at_gate&&base_stable {ctx.base_stable_steps+1}else{0};
            ctx.max_base_stable_steps=ctx.max_base_stable_steps.max(ctx.base_stable_steps);
            if !ctx.plan.stop_at_base&&!ctx.base_gate_open&&ctx.base_stable_steps>=crate::recovery_contract::substeps(c.recovery_acceptance.stable_persistence_s) {
                ctx.base_gate_open=true;ctx.base_gate_open_time_s=Some(now);
            }
            if !at_gate||ctx.base_gate_open {ctx.reference_steps+=1;}
        }
        ctx.terminal_steps=if terminal {ctx.terminal_steps+1}else{0};
        ctx.max_terminal_steps=ctx.max_terminal_steps.max(ctx.terminal_steps);
        ctx.unsupported_steps=if persistent_permitted_load>=c.recovery_acceptance.minimum_load_n {0}else{ctx.unsupported_steps+1};
        let physical=super::observe(s,ctx.previous_com_velocity);
        ctx.previous_com_velocity=v;
        let hand_local=s.heading.inverse()*hand_impulse;
        let mut sample=json!({"timeS":now,"physical":physical,"contacts":contact,"momentum":momentum(s),
            "handHorizontalImpulseNms":{"x":hand_local.x,"z":hand_local.z},
            "handHorizontalForceN":{"x":hand_local.x/s.profile.dt_s,"z":hand_local.z/s.profile.dt_s},
            "command":ctx.last_command,"originalMotion":s.motion,"originalRecoveryCount":s.counters.recoveries,
            "actualAllowedLoadN":permitted_load,"persistentAllowedLoadN":persistent_permitted_load,"handsPersistent":hands,"shinsPersistent":shins,
            "comAndCaptureInsideActualAllowedHull":inside,"all25Still":all_still,
            "terminalConsecutiveSteps":ctx.terminal_steps,"unsupportedConsecutiveSteps":ctx.unsupported_steps,
            "baseStableConsecutiveSteps":ctx.base_stable_steps,"baseGateOpen":ctx.base_gate_open});
        if ctx.observation_policy==ObservationPolicy::LandingOnly {
            sample["landingObservation"]=json!({"persistentRollingLoadExcludingHeadNeckN":rolling_load,
                "all25UnderOriginalSettlingSpeeds":all_settled,"restConsecutiveSteps":ctx.rest_steps});
        }
        let prone_result=if let Some(prone)=ctx.prone.as_mut() {
            let result=prone.observe(s,substep,&contact,all_still);
            if prone.active(){sample["proneRecoveryObservation"]=prone.observation();}
            result
        }else{Ok(())};
        if left_hand_command_audit::enabled()&&now>=2.0 {sample["leftHandCommandAudit"]=left_hand_command_audit::observe(s);}
        if prone_chain_execution::enabled()&&now>=2.0 {sample["bilateralArmCommandAudit"]=left_hand_command_audit::observe_arms(s,&["left","right"]);}
        ctx.trace.push(sample);
        if let Err(f)=prone_result {
            if !(left_hand_command_audit::enabled()&&f.check=="prone-stage-stall") {return Err(f);}
            left_hand_command_audit::remember_stall(f);
        }
        left_hand_command_audit::wall_guard(s,substep)?;
        prone_chain_execution::wall_guard(s,substep)?;
        let corridor_failure=if let Some((id,load))=unexpected {
            Some(measurement::Failure::new(s.tick,substep,"hand-unplanned-support",id,load,c.recovery_acceptance.minimum_load_n))
        } else if ctx.unsupported_steps>=crate::recovery_contract::substeps(c.recovery_acceptance.support_loss_s) {
            Some(measurement::Failure::new(s.tick,substep,"hand-all-support-loss","no declared support".into(),ctx.unsupported_steps as f32*s.profile.dt_s,c.recovery_acceptance.support_loss_s as f32))
        } else {None};
        if let Some(failure)=corridor_failure {
            ctx.first_corridor_failure.get_or_insert_with(||failure.clone());
            if ctx.observation_policy==ObservationPolicy::StrictCorridor {return Err(failure);}
        }
        if started.elapsed().as_secs_f64()>=MAX_WALL_S {
            return Err(measurement::Failure::new(s.tick,substep,"hand-wall-budget","A/B diagnostic wall budget".into(),started.elapsed().as_secs_f64() as f32,MAX_WALL_S as f32));
        }
        Ok(())
    })
}

fn run(
    fixture: &Fixture,
    profile: &Profile,
    plan: &Plan,
    mode: ArmMode,
    heading: f32,
    started: Instant,
) -> Result<Value, String> {
    run_with_policy(
        fixture,
        profile,
        plan,
        mode,
        heading,
        started,
        ObservationPolicy::StrictCorridor,
    )
}

fn run_with_policy(
    fixture: &Fixture,
    profile: &Profile,
    plan: &Plan,
    mode: ArmMode,
    heading: f32,
    started: Instant,
    observation_policy: ObservationPolicy,
) -> Result<Value, String> {
    let mut poses = fixture.poses.clone();
    if heading != fixture.heading {
        let r = Rotation::from_rotation_y(heading - fixture.heading);
        for pose in &mut poses {
            pose.position = crate::vec3(r * vector(pose.position));
            pose.rotation = crate::quat(r * rotation(pose.rotation));
            pose.linear_velocity = crate::vec3(r * vector(pose.linear_velocity));
            pose.angular_velocity = crate::vec3(r * vector(pose.angular_velocity));
        }
    }
    let mut s = Simulation::build(heading, profile.clone(), true, None, Some(&poses))?;
    let initial_q = offline_trajectory::coordinates(&s)?;
    s.rest_targets = initial_q.to_vec();
    let initial = s.snapshot();
    let bodies = s.bodies.clone();
    let joints = s.joints.clone();
    let colliders = s.colliders.clone();
    let generation = s.generation;
    ACTIVE.with(|slot| {
        *slot.borrow_mut() = Some(Context {
            plan: plan.clone(),
            initial: initial_q,
            mode,
            last_command: Value::Null,
            previous_com_velocity: super::velocity(&s),
            trace: Vec::new(),
            unsupported_steps: 0,
            terminal_steps: 0,
            max_terminal_steps: 0,
            max_cap_sum_nm: 0.0,
            max_unplanned_load_n: 0.0,
            first_hand: None,
            first_shin: None,
            first_unplanned: None,
            reference_steps: 0,
            base_stable_steps: 0,
            max_base_stable_steps: 0,
            base_gate_open: false,
            base_gate_open_time_s: None,
            observation_policy,
            first_corridor_failure: None,
            rest_steps: 0,
            max_rest_steps: 0,
            prone: None,
        })
    });
    if observation_policy == ObservationPolicy::ProneRecovery {
        let path = std::env::var("LH_PRONE_PROGRAM").map_err(|e| e.to_string())?;
        let prone = prone_recovery_experiment::Controller::load(&path)?;
        ACTIVE.with(|slot| slot.borrow_mut().as_mut().unwrap().prone = Some(prone));
    }
    let mut failure = None;
    let horizon = if observation_policy == ObservationPolicy::LandingOnly {
        plan.native_horizon_s.min(3.0)
    } else if observation_policy == ObservationPolicy::ProneRecovery {
        if prone_chain_execution::enabled() {
            prone_chain_execution::HORIZON_S
        } else {
            6.0
        }
    } else {
        plan.native_horizon_s
    };
    let horizon_steps = (horizon / s.profile.dt_s).round() as u64;
    for _ in 0..horizon_steps / 4 {
        let result = s.advance_tick_observed(|state, substep| {
            if state.bodies != bodies
                || state.joints != joints
                || state.colliders != colliders
                || state.generation != generation
            {
                return Err(measurement::Failure::new(
                    state.tick,
                    substep,
                    "hand-ownership",
                    "native assembly changed".into(),
                    1.0,
                    0.0,
                ));
            }
            observe_step(state, substep, started)
        });
        if let Err(e) = result {
            failure = Some(e);
            break;
        }
        let base_finished = ACTIVE.with(|slot| {
            let slot = slot.borrow();
            let ctx = slot.as_ref().unwrap();
            plan.stop_at_base
                && ctx.max_base_stable_steps
                    >= crate::recovery_contract::substeps(
                        crate::recovery_contract::contract()
                            .recovery_acceptance
                            .stable_persistence_s,
                    )
        });
        let resting = ACTIVE.with(|slot| {
            slot.borrow().as_ref().unwrap().rest_steps
                >= crate::recovery_contract::substeps(
                    crate::recovery_contract::contract()
                        .recovery_acceptance
                        .settle_persistence_s,
                )
        });
        if observation_policy == ObservationPolicy::ProneRecovery && resting {
            let started = ACTIVE.with(|slot| {
                let mut slot = slot.borrow_mut();
                let ctx = slot.as_mut().unwrap();
                let held: Coordinates =
                    serde_json::from_value(ctx.last_command["targetCoordinates"].clone())
                        .map_err(|e| e.to_string())?;
                let prone = ctx.prone.as_mut().unwrap();
                if !prone.active() {
                    prone.start(&s, &held)
                } else {
                    Ok(())
                }
            });
            if let Err(e) = started {
                failure = Some(measurement::Failure::new(
                    s.tick,
                    0,
                    "prone-entry",
                    e,
                    1.0,
                    0.0,
                ));
                break;
            }
        }
        if left_hand_command_audit::finished(&s) {
            break;
        }
        if observation_policy == ObservationPolicy::ProneRecovery
            && ACTIVE.with(|slot| {
                slot.borrow()
                    .as_ref()
                    .unwrap()
                    .prone
                    .as_ref()
                    .unwrap()
                    .passed()
            })
        {
            break;
        }
        if (observation_policy == ObservationPolicy::StrictCorridor && base_finished)
            || (observation_policy == ObservationPolicy::LandingOnly && resting)
        {
            break;
        }
    }
    let ctx = ACTIVE.with(|slot| slot.borrow_mut().take().unwrap());
    let passed = failure.is_none()
        && ctx.first_corridor_failure.is_none()
        && ctx.max_terminal_steps
            >= crate::recovery_contract::substeps(
                crate::recovery_contract::contract()
                    .recovery_acceptance
                    .stable_persistence_s,
            );
    let base_passed = failure.is_none()
        && ctx.first_corridor_failure.is_none()
        && ctx.max_base_stable_steps
            >= crate::recovery_contract::substeps(
                crate::recovery_contract::contract()
                    .recovery_acceptance
                    .stable_persistence_s,
            );
    Ok(
        json!({"scope":"Diagnostic intermediate hand/shin support only; no recovery qualification",
        "armMode":mode,"headingRad":heading,"initial":initial,"final":s.snapshot(),
        "physicalSteps":s.substeps_completed,"physicalSeconds":s.substeps_completed as f64/240.0,
        "failure":failure,"intermediateSupportPassed":passed,"maxTerminalConsecutiveSteps":ctx.max_terminal_steps,
        "baseSupportPassed":base_passed,"baseSupportScope":"Bilateral shins and allowed supports; same original load, hull, speed and persistence thresholds; not hand/shin transfer or get-up qualification",
        "firstBothHands":ctx.first_hand,"firstBothShins":ctx.first_shin,"firstUnplannedSupport":ctx.first_unplanned,
        "maxUnplannedLoadN":ctx.max_unplanned_load_n,"maxInstalledCapSumNm":ctx.max_cap_sum_nm,
        "originalMotion":s.motion,"originalRecoveryCount":s.counters.recoveries,
        "baseGateOpenTimeS":ctx.base_gate_open_time_s,"maxBaseStableSteps":ctx.max_base_stable_steps,
        "observationPolicy":observation_policy,"firstOriginalCorridorFailure":ctx.first_corridor_failure,
        "diagnosticRestObserved":ctx.max_rest_steps>=crate::recovery_contract::substeps(crate::recovery_contract::contract().recovery_acceptance.settle_persistence_s),
        "maxDiagnosticRestSteps":ctx.max_rest_steps,"recoveryQualified":false,
        "proneMilestonePassed":ctx.prone.as_ref().is_some_and(|p|p.passed()),
        "proneFinalObservation":ctx.prone.as_ref().map(|p|p.observation()),
        "leftHandAuditSummary":left_hand_command_audit::summary(),
        "trace":ctx.trace}),
    )
}

#[test]
fn prepared_arms_change_only_native_motor_targets_and_caps() {
    let fixture = serde_json::from_str::<FixtureSet>(FIXTURES)
        .unwrap()
        .cases
        .into_iter()
        .find(|x| x.id == "landed-crouch-left")
        .unwrap();
    let profile = crate::recovery_contract::pinned_profile();
    let mut a = Simulation::build(
        fixture.heading,
        profile.clone(),
        true,
        None,
        Some(&fixture.poses),
    )
    .unwrap();
    let mut b =
        Simulation::build(fixture.heading, profile, true, None, Some(&fixture.poses)).unwrap();
    let initial = offline_trajectory::coordinates(&a).unwrap();
    let mut end = initial;
    let arm_index = a
        .model
        .segments
        .iter()
        .position(|p| p.id == "leftUpperArm")
        .unwrap();
    end[arm_index][0] += 0.08;
    let plan = Plan {
        phase_gate: None,
        native_horizon_s: 3.0,
        stop_at_base: false,
        rows: vec![
            Row {
                time_s: 0.0,
                coordinates: initial,
            },
            Row {
                time_s: 1.0,
                coordinates: end,
            },
        ],
    };
    a.substeps_completed = 120;
    b.substeps_completed = 120;
    let (qa, va, _) = command(&plan, &a, &initial, ArmMode::Reference);
    let (qb, vb, _) = command(&plan, &b, &initial, ArmMode::Prepared);
    for i in 0..25 {
        if !arm(&a.model.segments[i].id) {
            assert_eq!(qa[i], qb[i]);
            assert_eq!(va[i], vb[i]);
        }
    }
    assert_eq!(qa[arm_index], initial[arm_index]);
    assert_ne!(qb[arm_index], initial[arm_index]);
    let initial_snapshot = serde_json::to_value(a.snapshot()).unwrap();
    let initial_bodies = serde_json::to_value(&a.world.bodies).unwrap();
    let (cap_a, _, _) =
        offline_trajectory::install_targets(&mut a, &qa, &va, Allocation::OwnedRequests).unwrap();
    let (cap_b, _, _) =
        offline_trajectory::install_targets(&mut b, &qb, &vb, Allocation::OwnedRequests).unwrap();
    assert!(cap_a <= 760.0 && cap_b <= 760.0);
    for state in [&a, &b] {
        assert_eq!(
            serde_json::to_value(state.snapshot()).unwrap(),
            initial_snapshot
        );
        assert_eq!(
            serde_json::to_value(&state.world.bodies).unwrap(),
            initial_bodies
        );
        assert_eq!(state.bodies.len(), 25);
        for (i, p) in state.model.segments.iter().enumerate().skip(1) {
            let joint = state
                .world
                .impulse_joints
                .get(state.joints[i].unwrap())
                .unwrap();
            for axis in &p.joint_profile.as_ref().unwrap().axes {
                let motor = &joint.data.motors[crate::axis(axis.coordinate).unwrap() as usize];
                assert!(
                    motor.max_force.is_finite()
                        && motor.max_force >= 0.0
                        && motor.max_force <= axis.max_motor_torque_nm
                );
            }
        }
    }
    assert_eq!(a.bodies, b.bodies);
    assert_eq!(a.joints, b.joints);
    assert_eq!(a.colliders, b.colliders);
}

#[test]
fn closed_phase_gate_holds_targets_without_body_writes() {
    let fixture = serde_json::from_str::<FixtureSet>(FIXTURES)
        .unwrap()
        .cases
        .into_iter()
        .find(|x| x.id == "landed-crouch-left")
        .unwrap();
    let mut s = Simulation::build(
        fixture.heading,
        crate::recovery_contract::pinned_profile(),
        true,
        None,
        Some(&fixture.poses),
    )
    .unwrap();
    let initial = offline_trajectory::coordinates(&s).unwrap();
    let mut base = initial;
    base[15][0] += 0.02;
    let mut end = base;
    end[15][0] += 0.03;
    let plan = Plan {
        rows: vec![
            Row {
                time_s: 0.0,
                coordinates: initial,
            },
            Row {
                time_s: 2.0,
                coordinates: base,
            },
            Row {
                time_s: 3.5,
                coordinates: end,
            },
        ],
        phase_gate: Some(PhaseGate { at_time_s: 2.0 }),
        native_horizon_s: 6.0,
        stop_at_base: false,
    };
    let before = serde_json::to_value(&s.world.bodies).unwrap();
    ACTIVE.with(|slot| {
        *slot.borrow_mut() = Some(Context {
            plan,
            initial,
            mode: ArmMode::Prepared,
            last_command: Value::Null,
            previous_com_velocity: super::velocity(&s),
            trace: Vec::new(),
            unsupported_steps: 0,
            terminal_steps: 0,
            max_terminal_steps: 0,
            max_cap_sum_nm: 0.0,
            max_unplanned_load_n: 0.0,
            first_hand: None,
            first_shin: None,
            first_unplanned: None,
            reference_steps: 480,
            base_stable_steps: 0,
            max_base_stable_steps: 0,
            base_gate_open: false,
            base_gate_open_time_s: None,
            observation_policy: ObservationPolicy::StrictCorridor,
            first_corridor_failure: None,
            rest_steps: 0,
            max_rest_steps: 0,
            prone: None,
        });
    });
    before_step(&mut s, 0).unwrap();
    ACTIVE.with(|slot| {
        let slot = slot.borrow();
        let ctx = slot.as_ref().unwrap();
        assert_eq!(ctx.last_command["referenceTimeS"], 2.0);
        assert_eq!(ctx.last_command["targetCoordinates"], json!(base));
        assert_eq!(
            ctx.last_command["targetVelocities"],
            serde_json::to_value([[0.0_f32; 3]; 25]).unwrap()
        );
    });
    assert_eq!(serde_json::to_value(&s.world.bodies).unwrap(), before);
    ACTIVE.with(|slot| {
        let mut slot = slot.borrow_mut();
        let ctx = slot.as_mut().unwrap();
        ctx.base_gate_open = true;
        ctx.reference_steps = 481;
    });
    before_step(&mut s, 0).unwrap();
    ACTIVE.with(|slot| {
        let ctx = slot.borrow_mut().take().unwrap();
        assert_ne!(ctx.last_command["targetCoordinates"], json!(base));
        assert!(ctx.last_command["referenceTimeS"].as_f64().unwrap() > 2.0);
    });
    assert_eq!(serde_json::to_value(&s.world.bodies).unwrap(), before);
    assert_eq!(s.substeps_completed, 0);
}

#[test]
#[ignore = "explicit bounded tibia-first native comparison against archived B"]
fn controlled_descent_one() {
    let plan_path = std::env::var("LH_DESCENT_PLAN").expect("LH_DESCENT_PLAN required");
    let out = PathBuf::from(std::env::var("LH_DESCENT_OUT").expect("LH_DESCENT_OUT required"));
    assert!(!out.exists(), "Refuse existing evidence directory");
    std::fs::create_dir_all(&out).unwrap();
    let plan = Plan::load(&plan_path).unwrap();
    assert!(plan.phase_gate.is_some());
    assert_eq!(plan.native_horizon_s, 6.0);
    assert!(
        plan.stop_at_base,
        "Verify base reproducibility before progressing"
    );
    let fixture = serde_json::from_str::<FixtureSet>(FIXTURES)
        .unwrap()
        .cases
        .into_iter()
        .find(|x| x.id == "landed-crouch-left")
        .unwrap();
    let profile = crate::recovery_contract::pinned_profile();
    let started = Instant::now();
    let c = run(
        &fixture,
        &profile,
        &plan,
        ArmMode::Prepared,
        fixture.heading,
        started,
    )
    .unwrap();
    std::fs::write(
        out.join("c-controlled.json"),
        serde_json::to_vec(&c).unwrap(),
    )
    .unwrap();
    let mut repeat = Value::Null;
    let mut rotated = Value::Null;
    if c["baseSupportPassed"] == true {
        repeat = run(
            &fixture,
            &profile,
            &plan,
            ArmMode::Prepared,
            fixture.heading,
            started,
        )
        .unwrap();
        std::fs::write(
            out.join("c-repeat.json"),
            serde_json::to_vec(&repeat).unwrap(),
        )
        .unwrap();
        if repeat["baseSupportPassed"] == true {
            rotated = run(
                &fixture,
                &profile,
                &plan,
                ArmMode::Prepared,
                std::f32::consts::PI / 3.0,
                started,
            )
            .unwrap();
            std::fs::write(
                out.join("c-heading-60.json"),
                serde_json::to_vec(&rotated).unwrap(),
            )
            .unwrap();
        }
    }
    std::fs::write(out.join("summary.json"),serde_json::to_vec_pretty(&json!({
        "scope":"One coordinated tibia-first descent vs archived B; stop at stable tibia base before any hand transfer",
        "caseCount":1+usize::from(!repeat.is_null())+usize::from(!rotated.is_null()),
        "nativeHorizonPerCaseS":6,"wallBudgetS":MAX_WALL_S,"elapsedWallS":started.elapsed().as_secs_f64(),
        "cBasePassed":c["baseSupportPassed"],"repeatBasePassed":repeat["baseSupportPassed"],
        "heading60BasePassed":rotated["baseSupportPassed"],
        "handShinTransferPassed":c["intermediateSupportPassed"],
        "repeatTraceExact":if repeat.is_null(){Value::Null}else{json!(c["trace"]==repeat["trace"]&&c["final"]==repeat["final"])},
        "recoveryQualified":false
    })).unwrap()).unwrap();
}

#[test]
#[ignore = "one authorized landing-only continuation of archived C; no qualification"]
fn observe_c_landing_once() {
    let plan_path = std::env::var("LH_DESCENT_PLAN").expect("LH_DESCENT_PLAN required");
    let out = PathBuf::from(std::env::var("LH_LANDING_OUT").expect("LH_LANDING_OUT required"));
    assert!(!out.exists(), "Refuse existing evidence directory");
    std::fs::create_dir_all(&out).unwrap();
    let plan = Plan::load(&plan_path).unwrap();
    assert!(plan.stop_at_base && plan.phase_gate.is_some());
    let fixture = serde_json::from_str::<FixtureSet>(FIXTURES)
        .unwrap()
        .cases
        .into_iter()
        .find(|x| x.id == "landed-crouch-left")
        .unwrap();
    let started = Instant::now();
    let observed = run_with_policy(
        &fixture,
        &crate::recovery_contract::pinned_profile(),
        &plan,
        ArmMode::Prepared,
        fixture.heading,
        started,
        ObservationPolicy::LandingOnly,
    )
    .unwrap();
    std::fs::write(
        out.join("c-landing-observation.json"),
        serde_json::to_vec(&observed).unwrap(),
    )
    .unwrap();
    std::fs::write(out.join("summary.json"),serde_json::to_vec_pretty(&json!({
        "scope":"One landing-only observation; original C corridor failure remains negative",
        "caseCount":1,"horizonS":3.0,"wallBudgetS":MAX_WALL_S,
        "elapsedWallS":started.elapsed().as_secs_f64(),"physicalSeconds":observed["physicalSeconds"],
        "physicalGuardFailure":observed["failure"],"firstOriginalCorridorFailure":observed["firstOriginalCorridorFailure"],
        "diagnosticRestObserved":observed["diagnosticRestObserved"],"recoveryQualified":false
    })).unwrap()).unwrap();
}

#[test]
#[ignore = "explicit native A/B diagnostic; requires archived corridor plan"]
fn hand_support_ab() {
    let plan_path = std::env::var("LH_HAND_PLAN").expect("LH_HAND_PLAN required");
    let out = PathBuf::from(std::env::var("LH_HAND_OUT").expect("LH_HAND_OUT required"));
    assert!(!out.exists());
    std::fs::create_dir_all(&out).unwrap();
    let input = std::fs::read(&plan_path).unwrap();
    let plan = Plan::load(&plan_path).unwrap();
    let profile = crate::recovery_contract::pinned_profile();
    let fixture = serde_json::from_str::<FixtureSet>(FIXTURES)
        .unwrap()
        .cases
        .into_iter()
        .find(|x| x.id == "landed-crouch-left")
        .unwrap();
    let started = Instant::now();
    let a = run(
        &fixture,
        &profile,
        &plan,
        ArmMode::Reference,
        fixture.heading,
        started,
    )
    .unwrap();
    std::fs::write(
        out.join("a-reference-arms.json"),
        serde_json::to_vec(&a).unwrap(),
    )
    .unwrap();
    let b = run(
        &fixture,
        &profile,
        &plan,
        ArmMode::Prepared,
        fixture.heading,
        started,
    )
    .unwrap();
    std::fs::write(
        out.join("b-prepared-arms.json"),
        serde_json::to_vec(&b).unwrap(),
    )
    .unwrap();
    let mut repeat = Value::Null;
    let mut rotated = Value::Null;
    if b["intermediateSupportPassed"] == true {
        repeat = run(
            &fixture,
            &profile,
            &plan,
            ArmMode::Prepared,
            fixture.heading,
            started,
        )
        .unwrap();
        std::fs::write(
            out.join("b-repeat.json"),
            serde_json::to_vec(&repeat).unwrap(),
        )
        .unwrap();
        rotated = run(
            &fixture,
            &profile,
            &plan,
            ArmMode::Prepared,
            std::f32::consts::PI / 3.0,
            started,
        )
        .unwrap();
        std::fs::write(
            out.join("b-heading-60.json"),
            serde_json::to_vec(&rotated).unwrap(),
        )
        .unwrap();
    }
    std::fs::write(out.join("summary.json"),serde_json::to_vec_pretty(&json!({
        "scope":"Native three-second hand/shin acquisition A/B, not get-up qualification",
        "planPath":plan_path,"planBytes":input.len(),"elapsedWallS":started.elapsed().as_secs_f64(),
        "wallBudgetS":MAX_WALL_S,"caseCount":2+usize::from(!repeat.is_null())+usize::from(!rotated.is_null()),
        "aPassed":a["intermediateSupportPassed"],"bPassed":b["intermediateSupportPassed"],
        "repeatPassed":repeat["intermediateSupportPassed"],"heading60Passed":rotated["intermediateSupportPassed"],
        "bRepeatExact":if repeat.is_null(){Value::Null}else{json!(b["trace"]==repeat["trace"]&&b["final"]==repeat["final"])},
        "originalRecoveryQualified":false
    })).unwrap()).unwrap();
}
