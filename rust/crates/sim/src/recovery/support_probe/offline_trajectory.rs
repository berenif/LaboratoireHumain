//! Bounded, opt-in search over whole native motor-command trajectories.
//! All bodies, contacts, limits, gains, and qualification gates remain native.
use super::*;
use serde::{Deserialize, Serialize};
use std::{
    cell::{Cell, RefCell},
    path::PathBuf,
    time::Instant,
};

const MAX_CASES: usize = 48;
const MAX_WALL_S: f64 = 600.0;
const TRACE_STRIDE: u64 = 4;
const SEARCH_VERSION: &str = "offline-trajectory-v1";
pub(in crate::recovery) type Coordinates = [[f32; 3]; 25];

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
struct Parameters {
    pivot_foot_first: bool,
    pivot_s: f32,
    recenter_s: f32,
    rise_s: f32,
    hip_rad: f32,
    knee_rad: f32,
    ankle_rad: f32,
    torso_rad: f32,
    toe_rad: f32,
}

impl Parameters {
    fn baseline() -> Self {
        Self {
            pivot_foot_first: false,
            pivot_s: 0.25,
            recenter_s: 0.60,
            rise_s: 2.0,
            hip_rad: 0.0,
            knee_rad: 0.0,
            ankle_rad: 0.0,
            torso_rad: 0.0,
            toe_rad: 0.0,
        }
    }
    fn within_family(self) -> bool {
        self.pivot_s >= 0.15
            && self.pivot_s <= 0.75
            && self.recenter_s >= 0.25
            && self.recenter_s <= 1.5
            && self.rise_s >= 1.0
            && self.rise_s <= 4.0
            && self.hip_rad.abs() <= 0.28
            && self.knee_rad.abs() <= 0.28
            && self.ankle_rad.abs() <= 0.22
            && self.torso_rad.abs() <= 0.22
            && self.toe_rad.abs() <= 0.24
    }
    fn shifted(mut self, dim: usize, amount: f32) -> Option<Self> {
        let field = match dim {
            0 => &mut self.pivot_s,
            1 => &mut self.recenter_s,
            2 => &mut self.rise_s,
            3 => &mut self.hip_rad,
            4 => &mut self.knee_rad,
            5 => &mut self.ankle_rad,
            6 => &mut self.torso_rad,
            7 => &mut self.toe_rad,
            _ => return None,
        };
        *field += amount;
        self.within_family().then_some(self)
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub(in crate::recovery) enum Allocation {
    Inherited,
    OwnedRequests,
}

struct Plan {
    low: Coordinates,
    stand: Vec<Coordinates>,
    input_bytes: Vec<u8>,
}
impl Plan {
    fn load() -> Result<Self, String> {
        let path = std::env::var("LH_OFFLINE_PLAN")
            .map(PathBuf::from)
            .unwrap_or_else(|_| {
                PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(
                    "../../../evidence/native-constraint-comparison-2026-10-11/static-plan.json",
                )
            });
        let input_bytes = std::fs::read(path).map_err(|e| e.to_string())?;
        let value: Value = serde_json::from_slice(&input_bytes).map_err(|e| e.to_string())?;
        let stand = value["rows"]
            .as_array()
            .ok_or("missing static plan rows")?
            .iter()
            .map(|row| {
                serde_json::from_value::<Coordinates>(row["coordinates"].clone())
                    .map_err(|e| e.to_string())
            })
            .collect::<Result<Vec<_>, _>>()?;
        if stand.len() != 41 {
            return Err("static stand plan must have 41 rows".into());
        }
        Ok(Self {
            low: stand[0],
            stand,
            input_bytes,
        })
    }
}

struct Context {
    params: Parameters,
    allocation: Allocation,
    initial: Coordinates,
    low: Coordinates,
    stand: Vec<Coordinates>,
    rise_start_s: Option<f32>,
    stable_substeps: u64,
    max_stable_substeps: u64,
    unsupported_substeps: u64,
    bilateral_substeps: u64,
    balanced_substeps: u64,
    resolved_inside_substeps: u64,
    highest_stand_row: usize,
    max_pelvis_height_m: f32,
    min_capture_distance_m: f32,
    max_cap_sum_nm: f64,
    max_anatomical_ratio: f32,
    max_command_seconds: f64,
    command_seconds: Vec<f64>,
    released_to_original: bool,
    last_command: Value,
    trace: Vec<Value>,
    previous_velocity: Vector,
    velocity_before_last_observation: Vector,
    last_observed_substep: u64,
    quiet_hold_substeps: u64,
    first_recovery_s: Option<f32>,
    max_nonfoot_load_n: f32,
    first_unplanned_contact: Option<Value>,
    first_persistent_unplanned_support: Option<Value>,
}
thread_local! { static ACTIVE: RefCell<Option<Context>> = const { RefCell::new(None) }; }

fn smooth(from: f32, to: f32, elapsed: f32, duration: f32) -> (f32, f32) {
    let u = (elapsed / duration).clamp(0.0, 1.0);
    let blend = u * u * (3.0 - 2.0 * u);
    let velocity = if elapsed >= duration {
        0.0
    } else {
        (to - from) * 6.0 * u * (1.0 - u) / duration
    };
    (from + (to - from) * blend, velocity)
}
pub(in crate::recovery) fn coordinates(s: &Simulation) -> Result<Coordinates, String> {
    let mut q = [[0.0; 3]; 25];
    for (i, part) in s.model.segments.iter().enumerate().skip(1) {
        let p = part.joint_profile.as_ref().ok_or("missing joint profile")?;
        let joint = s
            .world
            .impulse_joints
            .get(s.joints[i].ok_or("missing joint")?)
            .ok_or("lost joint")?;
        q[i] = measurement::coordinates(
            *s.world.bodies[joint.body1()].rotation(),
            *s.world.bodies[s.bodies[i]].rotation(),
            p,
        );
    }
    Ok(q)
}
fn offset(base: &Coordinates, s: &Simulation, p: Parameters, pivot: bool) -> Coordinates {
    let mut q = *base;
    for (i, part) in s.model.segments.iter().enumerate() {
        let x = if part.id.ends_with("Thigh") {
            if pivot { 0.0 } else { p.hip_rad }
        } else if part.id.ends_with("Shin") {
            if pivot { 0.0 } else { p.knee_rad }
        } else if part.id.ends_with("Ankle") {
            p.ankle_rad
        } else if part.id.ends_with("Forefoot") {
            p.toe_rad
        } else if matches!(part.id.as_str(), "torso" | "chest") {
            if pivot { 0.0 } else { p.torso_rad }
        } else {
            0.0
        };
        if part
            .joint_profile
            .as_ref()
            .is_some_and(|j| j.axes.iter().any(|a| a.coordinate == 'x'))
        {
            q[i][0] += x;
        }
    }
    q
}
fn command(ctx: &Context, s: &Simulation) -> (Coordinates, Coordinates, &'static str, usize) {
    let t = s.substeps_completed as f32 * s.profile.dt_s;
    let coupled_pivot = offset(&ctx.low, s, ctx.params, true);
    let pivot = if ctx.params.pivot_foot_first {
        let mut feet = ctx.initial;
        for (i, part) in s.model.segments.iter().enumerate() {
            if part.id.ends_with("Ankle") || part.id.ends_with("Forefoot") {
                feet[i] = coupled_pivot[i];
            }
        }
        feet
    } else {
        coupled_pivot
    };
    let recenter = offset(&ctx.low, s, ctx.params, false);
    let mut q = [[0.0; 3]; 25];
    let mut v = [[0.0; 3]; 25];
    if let Some(start) = ctx.rise_start_s {
        let u = ((t - start) / ctx.params.rise_s).clamp(0.0, 1.0);
        if u >= 0.10 {
            let w = ((u - 0.10) / 0.90).clamp(0.0, 1.0);
            let progress = w * w * (3.0 - 2.0 * w);
            let progress_rate = if w >= 1.0 {
                0.0
            } else {
                6.0 * w * (1.0 - w) / (ctx.params.rise_s * 0.90)
            };
            let span = (ctx.stand.len() - 1) as f32;
            let row = (progress * span).clamp(0.0, span);
            let j = (row.floor() as usize).min(ctx.stand.len() - 2);
            let a = row - j as f32;
            let a2 = a * a;
            let a3 = a2 * a;
            for i in 0..25 {
                for k in 0..3 {
                    let y0 = ctx.stand[j][i][k];
                    let y1 = ctx.stand[j + 1][i][k];
                    let m0 = if j == 0 {
                        y1 - y0
                    } else {
                        (y1 - ctx.stand[j - 1][i][k]) * 0.5
                    };
                    let m1 = if j + 2 >= ctx.stand.len() {
                        y1 - y0
                    } else {
                        (ctx.stand[j + 2][i][k] - y0) * 0.5
                    };
                    q[i][k] = (2.0 * a3 - 3.0 * a2 + 1.0) * y0
                        + (a3 - 2.0 * a2 + a) * m0
                        + (-2.0 * a3 + 3.0 * a2) * y1
                        + (a3 - a2) * m1;
                    let dqdr = (6.0 * a2 - 6.0 * a) * y0
                        + (3.0 * a2 - 4.0 * a + 1.0) * m0
                        + (-6.0 * a2 + 6.0 * a) * y1
                        + (3.0 * a2 - 2.0 * a) * m1;
                    v[i][k] = dqdr * span * progress_rate;
                }
            }
            return (q, v, if w >= 1.0 { "hold" } else { "rise" }, row as usize);
        }
    }
    let (from, to, elapsed, duration, phase) = if t < ctx.params.pivot_s {
        (&ctx.initial, &pivot, t, ctx.params.pivot_s, "pivot")
    } else if t < ctx.params.pivot_s + ctx.params.recenter_s {
        (
            &pivot,
            &recenter,
            t - ctx.params.pivot_s,
            ctx.params.recenter_s,
            "recenter",
        )
    } else if let Some(start) = ctx.rise_start_s {
        let u = ((t - start) / ctx.params.rise_s).clamp(0.0, 1.0);
        if u < 0.10 {
            (
                &recenter,
                &ctx.low,
                t - start,
                ctx.params.rise_s * 0.10,
                "rise-transfer",
            )
        } else {
            unreachable!("rise rows returned above")
        }
    } else {
        (&recenter, &recenter, 1.0, 1.0, "support-hold")
    };
    for i in 0..25 {
        for k in 0..3 {
            (q[i][k], v[i][k]) = smooth(from[i][k], to[i][k], elapsed, duration);
        }
    }
    // The static-plan rows are piecewise precomputed poses, not motor gains.
    let row = ctx.rise_start_s.map_or(0, |start| {
        let u = ((t - start) / ctx.params.rise_s).clamp(0.0, 1.0);
        (((u - 0.10) / 0.90).clamp(0.0, 1.0) * 40.0) as usize
    });
    (q, v, phase, row)
}

/// Called only from a cfg(test) hook after environment and PGS are prepared.
pub(crate) fn before_step(s: &mut Simulation, substep: u32) -> Result<(), measurement::Failure> {
    ACTIVE.with(|slot| {
        let mut slot=slot.borrow_mut();let Some(ctx)=slot.as_mut() else {return Ok(())};
        let t=s.substeps_completed as f32*s.profile.dt_s;
        let finished=ctx.rise_start_s.is_some_and(|start|t-start>=ctx.params.rise_s);
        let upright=s.world.bodies[s.bodies[0]].translation().y-s.support_height()>0.93;
        let stable=ctx.stable_substeps>=crate::recovery_contract::substeps(crate::recovery_contract::contract().recovery_acceptance.stable_persistence_s);
        if s.counters.recoveries>0 || ctx.released_to_original || finished&&upright&&stable&&measurement::standing_support(s,substep).is_ok() {
            ctx.released_to_original=true;ctx.last_command=json!({"phase":"original-handoff","plannedRiseFinished":finished,"originalMotion":s.motion});return Ok(());
        }
        let start=Instant::now();
        let (targets,velocities,phase,row)=command(ctx,s);
        let (sum,ratio,requests)=install_targets(s,&targets,&velocities,ctx.allocation)
            .map_err(|e|measurement::Failure::new(s.tick,substep,"offline-command-refusal",e,1.0,0.0))?;
        ctx.max_cap_sum_nm=ctx.max_cap_sum_nm.max(sum);
        ctx.max_anatomical_ratio=ctx.max_anatomical_ratio.max(ratio);
        ctx.highest_stand_row=ctx.highest_stand_row.max(row);
        let seconds=start.elapsed().as_secs_f64();ctx.max_command_seconds=ctx.max_command_seconds.max(seconds);ctx.command_seconds.push(seconds);
        ctx.last_command=json!({"phase":phase,"pivotFamily":if ctx.params.pivot_foot_first {"foot-first"}else{"coupled-crouch"},"standRow":row,"targetCoordinates":targets,"targetVelocities":velocities,"capSumNm":sum,"maxAnatomicalRatio":ratio,"allocation":ctx.allocation,"ownerRequestsAndCaps":requests,"originalMotion":s.motion});
        Ok(())
    })
}

pub(in crate::recovery) fn install_targets(
    s: &mut Simulation,
    q: &Coordinates,
    dq: &Coordinates,
    allocation: Allocation,
) -> Result<(f64, f32, Vec<Value>), String> {
    let gain_scale = s
        .profile
        .motor_scale
        .min(s.profile.aggregate_motor_cap_nm / s.motor_ceiling_sum);
    let reserves = crate::recovery::support_effort_reserves(s);
    let mut owned_caps = [[0.0_f32; 3]; 25];
    let mut owner_requests = [[0.0_f32; 3]; 25];
    let mut cap_sum = 0.0_f64;
    for (i, part) in s.model.segments.iter().enumerate().skip(1) {
        let profile = part.joint_profile.as_ref().ok_or("missing joint profile")?;
        let handle = s.joints[i].ok_or("missing anatomical joint")?;
        let parent_handle = s
            .world
            .impulse_joints
            .get(handle)
            .ok_or("lost anatomical joint")?
            .body1();
        let parent = &s.world.bodies[parent_handle];
        let child = &s.world.bodies[s.bodies[i]];
        let measured = measurement::coordinates(*parent.rotation(), *child.rotation(), profile);
        let frame = *parent.rotation() * rotation(profile.parent_frame.rotation);
        let relative =
            frame.inverse() * (*child.rotation() * rotation(profile.child_frame.rotation));
        let omega = frame.inverse() * (child.angvel() - parent.angvel());
        let relative_v = relative.xyz();
        let mut commands = Vec::new();
        for a in &profile.axes {
            let k = match a.coordinate {
                'x' => 0,
                'y' => 1,
                _ => 2,
            };
            let axis = crate::axis(a.coordinate)?;
            let raw = q[i][k];
            let target = raw.clamp(a.min_radians, a.max_radians);
            let target_velocity = if target != raw { 0.0 } else { dq[i][k] };
            let e = [Vector::X, Vector::Y, Vector::Z][k];
            let denominator = relative.w * relative.w + relative_v[k] * relative_v[k];
            if !denominator.is_finite() || denominator < 1e-12 {
                return Err("singular owner motor coordinate".into());
            }
            let jac = (relative.w * relative.w * e
                + relative.w * relative_v.cross(e)
                + relative_v[k] * relative_v)
                / denominator;
            let rate = jac.dot(omega);
            let stiffness = a.passive_stiffness_nm_per_rad
                * s.profile.posture_stiffness_multiplier
                * gain_scale;
            let damping = a.damping_nms_per_rad * s.profile.posture_damping_multiplier * gain_scale;
            let error = (target - measured[k] + std::f32::consts::PI)
                .rem_euclid(std::f32::consts::TAU)
                - std::f32::consts::PI;
            let request = stiffness * error + damping * (target_velocity - rate);
            let anatomy_cap = a.max_motor_torque_nm * s.profile.motor_scale.min(1.0);
            let owned_cap =
                (0.10 * anatomy_cap + 1.25 * request.abs().max(reserves[i][k])).min(anatomy_cap);
            if !target.is_finite()
                || !target_velocity.is_finite()
                || !request.is_finite()
                || !owned_cap.is_finite()
            {
                return Err("non-finite owner motor command".into());
            }
            commands.push((
                k,
                axis,
                target,
                target_velocity,
                stiffness,
                damping,
                request,
                owned_cap,
                anatomy_cap,
            ));
        }
        let joint = s
            .world
            .impulse_joints
            .get_mut(handle, true)
            .ok_or("lost anatomical joint")?;
        for (
            k,
            axis,
            target,
            target_velocity,
            stiffness,
            damping,
            request,
            owned_cap,
            anatomy_cap,
        ) in commands
        {
            let old_cap = joint.data.motors[axis as usize].max_force;
            if !old_cap.is_finite() || old_cap < 0.0 || old_cap > anatomy_cap {
                return Err("inherited cap violates anatomy".into());
            }
            joint
                .data
                .set_motor_model(axis, MotorModel::ForceBased)
                .set_motor(axis, target, target_velocity, stiffness, damping);
            owned_caps[i][k] = owned_cap;
            owner_requests[i][k] = request;
            cap_sum += if allocation == Allocation::Inherited {
                old_cap as f64
            } else {
                owned_cap as f64
            };
        }
    }
    let budget = s.profile.aggregate_motor_cap_nm as f64;
    let owned_scale = if allocation == Allocation::OwnedRequests {
        (budget / cap_sum.max(1e-9)).min(1.0) as f32
    } else {
        1.0
    };
    if allocation == Allocation::Inherited && cap_sum - budget > 1e-3 {
        return Err("inherited cap exceeds aggregate budget materially".into());
    }
    let mut actual_sum = 0.0_f64;
    let mut selected = None;
    let mut maximum_ratio = 0.0_f32;
    let mut requests = Vec::new();
    for (i, part) in s.model.segments.iter().enumerate().skip(1) {
        let profile = part.joint_profile.as_ref().ok_or("missing joint profile")?;
        let joint = s
            .world
            .impulse_joints
            .get_mut(s.joints[i].unwrap(), true)
            .ok_or("lost joint")?;
        for a in &profile.axes {
            let k = match a.coordinate {
                'x' => 0,
                'y' => 1,
                _ => 2,
            };
            let axis = crate::axis(a.coordinate)?;
            let cap = if allocation == Allocation::OwnedRequests {
                owned_caps[i][k] * owned_scale
            } else {
                joint.data.motors[axis as usize].max_force
            };
            if cap < 0.0 || !cap.is_finite() || cap > a.max_motor_torque_nm {
                return Err("owner allocation exceeds anatomy".into());
            }
            joint.data.set_motor_max_force(axis, cap);
            actual_sum += cap as f64;
            maximum_ratio = maximum_ratio.max(cap / a.max_motor_torque_nm);
            requests.push(json!({"id":part.id,"coordinate":a.coordinate,"ownerSpringRequestNm":owner_requests[i][k],"installedCapNm":cap,"anatomicalCapNm":a.max_motor_torque_nm,"canonicalSupportReserveNm":reserves[i][k]}));
            if cap > 0.0 {
                selected = Some((s.joints[i].unwrap(), axis, cap));
            }
        }
    }
    if actual_sum > budget {
        let (handle, axis, cap) = selected.ok_or("no cap to repair")?;
        let exact = cap as f64 - (actual_sum - budget);
        if exact <= 0.0 || actual_sum - budget > 1e-3 {
            return Err("aggregate cap exceeds rounding repair".into());
        }
        let mut next = exact as f32;
        if next as f64 > exact {
            next = f32::from_bits(next.to_bits() - 1);
        }
        s.world
            .impulse_joints
            .get_mut(handle, true)
            .unwrap()
            .data
            .set_motor_max_force(axis, next);
        actual_sum = actual_sum - cap as f64 + next as f64;
    }
    if actual_sum > budget {
        return Err("rounded owner caps exceed aggregate ceiling".into());
    }
    Ok((actual_sum, maximum_ratio, requests))
}

fn capture_distance(physical: &Value) -> Option<f32> {
    let capture = &physical["capture"];
    let projection = physical["resolvedProjection"].as_array()?;
    let dx = capture["x"].as_f64()? as f32 - projection[0].as_f64()? as f32;
    let dz = capture["z"].as_f64()? as f32 - projection[1].as_f64()? as f32;
    Some(dx.hypot(dz))
}

fn resolved_inside(s: &Simulation, contacts: &[Value]) -> bool {
    let points = contacts
        .iter()
        .filter(|c| {
            c["persistent"] == true
                && c["id"]
                    .as_str()
                    .is_some_and(|id| id.ends_with("Foot") || id.ends_with("Forefoot"))
        })
        .filter_map(|c| {
            Some([
                c["point"]["x"].as_f64()? as f32,
                c["point"]["z"].as_f64()? as f32,
            ])
        })
        .collect::<Vec<_>>();
    let com = s.heading.inverse() * s.center_of_mass();
    let v = s.heading.inverse() * super::velocity(s);
    if !com.is_finite() || !v.is_finite() {
        return false;
    }
    let capture = com + v * ((com.y - s.support_height()).max(0.2) / 9.81).sqrt();
    crate::stepping::inside_support(points.clone(), [com.x, com.z], 0.003)
        && crate::stepping::inside_support(points, [capture.x, capture.z], 0.003)
}

fn observe_step(
    s: &Simulation,
    substep: u32,
    started: Instant,
    wall_s: f64,
) -> Result<(), measurement::Failure> {
    ACTIVE.with(|slot| {
        let mut slot=slot.borrow_mut();let ctx=slot.as_mut().expect("offline context");
        let contract=crate::recovery_contract::contract();
        let loaded=["left","right"].iter().all(|side|super::super::foot(s,side));
        ctx.unsupported_substeps=if loaded {0}else{ctx.unsupported_substeps+1};
        if loaded {ctx.bilateral_substeps+=1;}
        let balance=super::super::standing_balance(s).map_err(|e|measurement::Failure::new(s.tick,substep,"offline-balance-measurement",e,1.0,0.0))?.is_some_and(|(_,inside)|inside);
        let contacts=super::contacts(s);
        let physical_balance=resolved_inside(s,&contacts);
        let t=s.substeps_completed as f32*s.profile.dt_s;
        let mut unexpected=Vec::<(String,f32,f32)>::new();
        for part in s.model.segments.iter().filter(|part|!matches!(part.role.as_str(),"hindfoot"|"forefoot")) {
            let actual=contacts.iter().filter(|c|c["id"]==part.id).filter_map(|c|c["normalLoadN"].as_f64()).sum::<f64>() as f32;
            if actual>0.0 {
                ctx.max_nonfoot_load_n=ctx.max_nonfoot_load_n.max(actual);
                if ctx.first_unplanned_contact.is_none(){ctx.first_unplanned_contact=Some(json!({"id":part.id,"timeS":t,"actualNormalLoadN":actual}));}
                let persistent=super::super::load(s,&part.id);
                if actual>=contract.recovery_acceptance.minimum_load_n && persistent>=contract.recovery_acceptance.minimum_load_n {
                    unexpected.push((part.id.clone(),actual,persistent));
                }
            }
        }
        if let Some((id,actual,persistent))=unexpected.first()
            && ctx.first_persistent_unplanned_support.is_none() {
            ctx.first_persistent_unplanned_support=Some(json!({"id":id,"timeS":t,"actualNormalLoadN":actual,"originalPersistentLoadN":persistent}));
        }
        if balance && physical_balance {ctx.balanced_substeps+=1;}
        if physical_balance {ctx.resolved_inside_substeps+=1;}
        let all_still=s.bodies.iter().all(|h|{let b=&s.world.bodies[*h];b.linvel().length()<=contract.recovery_acceptance.stable_linear_mps&&b.angvel().length()<=contract.recovery_acceptance.stable_angular_radps});
        let soles=["left","right"].iter().all(|side|{
            let id=format!("{side}Foot");super::super::load(s,&id)>=contract.recovery_acceptance.minimum_load_n&&super::super::up(s,&id)>=contract.recovery_acceptance.stable_up_dot
        });
        let stable=all_still&&soles&&balance&&physical_balance&&unexpected.is_empty();
        ctx.stable_substeps=if stable {ctx.stable_substeps+1}else{0};
        ctx.max_stable_substeps=ctx.max_stable_substeps.max(ctx.stable_substeps);
        if ctx.rise_start_s.is_none()&&t>=ctx.params.pivot_s+ctx.params.recenter_s
            &&ctx.stable_substeps>=crate::recovery_contract::substeps(contract.recovery_acceptance.stable_persistence_s) {
            ctx.rise_start_s=Some(t);
        }
        ctx.max_pelvis_height_m=ctx.max_pelvis_height_m.max(s.world.bodies[s.bodies[0]].translation().y-s.support_height());
        let original_hold=s.counters.recoveries>0&&s.motion==Motion::Upright
            &&s.world.bodies[s.bodies[0]].translation().y-s.support_height()>0.93&&physical_balance
            &&measurement::standing_support(s,substep).is_ok()&&all_still;
        if s.counters.recoveries>0 && ctx.first_recovery_s.is_none(){ctx.first_recovery_s=Some(t);}
        ctx.quiet_hold_substeps=if original_hold {ctx.quiet_hold_substeps+1}else{0};
        if s.substeps_completed==1||s.substeps_completed%TRACE_STRIDE==0||ctx.quiet_hold_substeps==1||!unexpected.is_empty() {
            let physical=super::observe(s,ctx.previous_velocity);
            if let Some(distance)=capture_distance(&physical){ctx.min_capture_distance_m=ctx.min_capture_distance_m.min(distance);}
            ctx.trace.push(json!({"physical":physical,"command":ctx.last_command,
                "riseStartedS":ctx.rise_start_s,"originalMotion":s.motion,"recoveryCount":s.counters.recoveries,
                "loadedBothFeet":loaded,"balanceInside":balance,"resolvedInside":physical_balance,"stableSubsteps":ctx.stable_substeps,
                "unsupportedSubsteps":ctx.unsupported_substeps,"unexpectedPersistentSupport":unexpected,"maxNonFootLoadN":ctx.max_nonfoot_load_n,"originalQuietHoldSubsteps":ctx.quiet_hold_substeps}));
        }
        ctx.velocity_before_last_observation=ctx.previous_velocity;
        ctx.previous_velocity=super::velocity(s);
        ctx.last_observed_substep=s.substeps_completed;
        if let Some((id,actual,_))=unexpected.first() {
            return Err(measurement::Failure::new(s.tick,substep,"offline-unplanned-support",format!("{id} floor load persists outside feet"),*actual,contract.recovery_acceptance.minimum_load_n));
        }
        if started.elapsed().as_secs_f64()>=wall_s {
            return Err(measurement::Failure::new(s.tick,substep,"offline-search-wall-budget","search wall budget reached".into(),started.elapsed().as_secs_f64() as f32,wall_s as f32));
        }
        if s.counters.recoveries==0&&ctx.unsupported_substeps>=crate::recovery_contract::substeps(contract.recovery_acceptance.support_loss_s) {
            return Err(measurement::Failure::new(s.tick,substep,"offline-bilateral-support-loss","feet-only trajectory lost original bilateral support".into(),ctx.unsupported_substeps as f32*s.profile.dt_s,contract.recovery_acceptance.support_loss_s as f32));
        }
        Ok(())
    })
}

fn quantiles(mut samples: Vec<f64>) -> Value {
    if samples.is_empty() {
        return Value::Null;
    }
    samples.sort_by(f64::total_cmp);
    let at = |p: f64| samples[((samples.len() - 1) as f64 * p).round() as usize];
    json!({"count":samples.len(),"medianS":at(0.50),"p95S":at(0.95),"maxS":samples[samples.len()-1]})
}

#[allow(clippy::too_many_arguments)] // Keep the audited fixture, plan, bounds and cache identity explicit.
fn run_case(
    fixture: &Fixture,
    profile: &Profile,
    plan: &Plan,
    params: Parameters,
    allocation: Allocation,
    heading: f32,
    started: Instant,
    wall_s: f64,
    key: &str,
) -> Result<Value, String> {
    let mut poses = fixture.poses.clone();
    if heading != fixture.heading {
        let rotate = Rotation::from_rotation_y(heading - fixture.heading);
        for pose in &mut poses {
            pose.position = crate::vec3(rotate * vector(pose.position));
            pose.rotation = crate::quat(rotate * rotation(pose.rotation));
            pose.linear_velocity = crate::vec3(rotate * vector(pose.linear_velocity));
            pose.angular_velocity = crate::vec3(rotate * vector(pose.angular_velocity));
        }
    }
    let mut s = Simulation::build(heading, profile.clone(), true, None, Some(&poses))?;
    s.rest_targets = coordinates(&s)?.to_vec();
    let initial = s.snapshot();
    let initial_coordinates = coordinates(&s)?;
    let original_bodies = s.bodies.clone();
    let original_joints = s.joints.clone();
    let original_colliders = s.colliders.clone();
    let generation = s.generation;
    ACTIVE.with(|slot| {
        *slot.borrow_mut() = Some(Context {
            params,
            allocation,
            initial: initial_coordinates,
            low: plan.low,
            stand: plan.stand.clone(),
            rise_start_s: None,
            stable_substeps: 0,
            max_stable_substeps: 0,
            unsupported_substeps: 0,
            bilateral_substeps: 0,
            balanced_substeps: 0,
            resolved_inside_substeps: 0,
            highest_stand_row: 0,
            max_pelvis_height_m: f32::NEG_INFINITY,
            min_capture_distance_m: f32::INFINITY,
            max_cap_sum_nm: 0.0,
            max_anatomical_ratio: 0.0,
            max_command_seconds: 0.0,
            command_seconds: Vec::new(),
            released_to_original: false,
            last_command: Value::Null,
            trace: Vec::new(),
            previous_velocity: super::velocity(&s),
            velocity_before_last_observation: super::velocity(&s),
            last_observed_substep: 0,
            quiet_hold_substeps: 0,
            first_recovery_s: None,
            max_nonfoot_load_n: 0.0,
            first_unplanned_contact: None,
            first_persistent_unplanned_support: None,
        })
    });
    let case_started = Instant::now();
    let mut failure = None;
    let limit = crate::recovery_contract::contract()
        .acceptance
        .recovery_limit_s
        + crate::recovery_contract::contract()
            .acceptance
            .stable_observation_s;
    for _ in 0..(limit * 60.0).ceil() as usize {
        let result = s.advance_tick_observed(|state, substep| {
            if state.bodies != original_bodies
                || state.joints != original_joints
                || state.colliders != original_colliders
                || state.generation != generation
            {
                return Err(measurement::Failure::new(
                    state.tick,
                    substep,
                    "offline-ownership",
                    "native assembly changed".into(),
                    1.0,
                    0.0,
                ));
            }
            observe_step(state, substep, started, wall_s)
        });
        if let Err(e) = result {
            failure = Some(e);
            break;
        }
        let held = ACTIVE.with(|slot| slot.borrow().as_ref().unwrap().quiet_hold_substeps);
        if held
            >= crate::recovery_contract::substeps(
                crate::recovery_contract::contract()
                    .acceptance
                    .stable_observation_s,
            )
        {
            break;
        }
    }
    let mut ctx = ACTIVE.with(|slot| slot.borrow_mut().take().unwrap());
    if ctx
        .trace
        .last()
        .and_then(|x| x["physical"]["timeS"].as_f64())
        .is_none_or(|v| v < s.substeps_completed as f64 / 240.0)
    {
        let previous = if ctx.last_observed_substep == s.substeps_completed {
            ctx.velocity_before_last_observation
        } else {
            ctx.previous_velocity
        };
        let physical = super::observe(&s, previous);
        if let Some(distance) = capture_distance(&physical) {
            ctx.min_capture_distance_m = ctx.min_capture_distance_m.min(distance);
        }
        ctx.trace
            .push(json!({"physical":physical,"command":ctx.last_command,"lastPhysicalState":true}));
    }
    let qualified = failure.is_none()
        && s.counters.recoveries == 1
        && ctx.first_recovery_s.is_some_and(|t| {
            t <= crate::recovery_contract::contract()
                .acceptance
                .recovery_limit_s as f32
        })
        && s.motion == Motion::Upright
        && ctx.quiet_hold_substeps
            >= crate::recovery_contract::substeps(
                crate::recovery_contract::contract()
                    .acceptance
                    .stable_observation_s,
            );
    let physical_steps = s.substeps_completed;
    let report = json!({"scope":"Whole actual native trajectory; diagnostic result requires independent qualification",
        "cacheKey":key,"sourceVersion":SEARCH_VERSION,"parameters":params,"pivotFamily":if params.pivot_foot_first {"foot-first"}else{"coupled-crouch"},"allocation":allocation,"headingRad":heading,
        "intendedFullTrajectory":{"pivotS":params.pivot_s,"recenterS":params.recenter_s,"riseS":params.rise_s,"staticStandRows":41,
            "riseGate":"original bilateral hindfoot load, upright feet, all-25 speed, capture/COM within support for original 0.55 s"},
        "initial":initial,"final":s.snapshot(),"physicalSteps":physical_steps,"physicalSeconds":physical_steps as f64/240.0,
        "firstNativeRecoveryS":ctx.first_recovery_s,"plannedProgramEndS":ctx.rise_start_s.map(|start|start+params.rise_s),
        "caseOutcome":if failure.as_ref().is_some_and(|f|f.check=="offline-search-wall-budget") {"unknown-wall-budget"}else if qualified {"original-qualified"}else if failure.is_some(){"physical-or-command-failure"}else{"full-horizon-unqualified"},
        "wallSeconds":case_started.elapsed().as_secs_f64(),"failure":failure,"originalRecoveryQualified":qualified,
        "originalRecoveries":s.counters.recoveries,"originalMotion":s.motion,"riseStartedS":ctx.rise_start_s,
        "highestStandRow":ctx.highest_stand_row,"maxStableSubsteps":ctx.max_stable_substeps,
        "bilateralSubsteps":ctx.bilateral_substeps,"balancedSubsteps":ctx.balanced_substeps,"resolvedInsideSubsteps":ctx.resolved_inside_substeps,
        "maxNonFootLoadN":ctx.max_nonfoot_load_n,"firstUnplannedContact":ctx.first_unplanned_contact,"firstPersistentUnplannedSupport":ctx.first_persistent_unplanned_support,
        "minimumCaptureDistanceM":ctx.min_capture_distance_m.is_finite().then_some(ctx.min_capture_distance_m),
        "maxPelvisHeightM":ctx.max_pelvis_height_m,"maxCapSumNm":ctx.max_cap_sum_nm,
        "maxAnatomicalRatio":ctx.max_anatomical_ratio,"commandTiming":quantiles(ctx.command_seconds),
        "releasedToOriginal":ctx.released_to_original,"trajectoryTrace":ctx.trace});
    Ok(report)
}

fn hash_key(
    fixture: &Fixture,
    profile: &Profile,
    plan: &Plan,
    params: Parameters,
    allocation: Allocation,
    heading: f32,
) -> String {
    let mut hash = 0xcbf29ce484222325_u64;
    let chunks = [
        include_bytes!("offline_trajectory.rs").to_vec(),
        include_bytes!("../../lib.rs").to_vec(),
        include_bytes!("../../measurement.rs").to_vec(),
        include_bytes!("../../recovery.rs").to_vec(),
        include_bytes!("../support_probe.rs").to_vec(),
        include_bytes!("../../../../../Cargo.lock").to_vec(),
        include_bytes!("../../../../../vendor/rapier3d/src/dynamics/joint/generic_joint.rs")
            .to_vec(),
        lh_model::CANONICAL_JSON.as_bytes().to_vec(),
        crate::recovery_contract::PINNED_PROFILE_JSON
            .as_bytes()
            .to_vec(),
        serde_json::to_vec(&fixture.poses).unwrap(),
        serde_json::to_vec(profile).unwrap(),
        plan.input_bytes.clone(),
        serde_json::to_vec(&(params, allocation, heading)).unwrap(),
        SEARCH_VERSION.as_bytes().to_vec(),
    ];
    for chunk in chunks {
        for byte in chunk {
            hash ^= byte as u64;
            hash = hash.wrapping_mul(0x100000001b3);
        }
    }
    format!("{hash:016x}")
}

fn integrity_violation(report: &Value) -> f64 {
    let failure = &report["failure"];
    let Some(check) = failure["check"].as_str() else {
        return 0.0;
    };
    if !matches!(
        check,
        "floor"
            | "self-contact"
            | "joint-anchor"
            | "joint-limit"
            | "finite"
            | "ownership"
            | "offline-ownership"
            | "offline-command-refusal"
            | "measurement"
    ) {
        return 0.0;
    }
    let measured = failure["measured"].as_f64().unwrap_or(1.0);
    let threshold = failure["threshold"].as_f64().unwrap_or(0.0);
    if threshold > 0.0 && measured.is_finite() {
        (measured / threshold).max(1.0)
    } else {
        1.0
    }
}

fn score(report: &Value) -> [f64; 13] {
    let n = |key: &str| report[key].as_f64().unwrap_or(0.0);
    let finished = report["originalRecoveryQualified"] == true;
    let distance = report["minimumCaptureDistanceM"].as_f64().unwrap_or(1000.0);
    [
        f64::from(finished as u8),
        -integrity_violation(report),
        f64::from((report["caseOutcome"] != "unknown-wall-budget") as u8),
        f64::from((report["failure"]["check"] != "offline-unplanned-support") as u8),
        -n("maxNonFootLoadN"),
        f64::from(report["riseStartedS"].is_number() as u8),
        n("highestStandRow"),
        n("maxStableSubsteps"),
        n("resolvedInsideSubsteps"),
        n("balancedSubsteps"),
        n("bilateralSubsteps"),
        -distance,
        n("maxPelvisHeightM"),
    ]
}

fn better(a: &Value, b: &Value) -> bool {
    for (x, y) in score(a).into_iter().zip(score(b)) {
        if (x - y).abs() > 1e-12 {
            return x > y;
        }
    }
    false
}

fn initial_family() -> Vec<Parameters> {
    let base = Parameters::baseline();
    vec![
        base,
        Parameters {
            pivot_foot_first: true,
            ..base
        },
        Parameters {
            pivot_foot_first: true,
            ankle_rad: -0.10,
            toe_rad: -0.14,
            ..base
        },
        Parameters {
            ankle_rad: -0.10,
            toe_rad: -0.14,
            ..base
        },
        Parameters {
            ankle_rad: 0.10,
            toe_rad: 0.14,
            ..base
        },
        Parameters {
            hip_rad: -0.12,
            knee_rad: 0.10,
            torso_rad: -0.10,
            ..base
        },
        Parameters {
            hip_rad: 0.12,
            knee_rad: -0.10,
            torso_rad: 0.10,
            ..base
        },
        Parameters {
            hip_rad: -0.10,
            knee_rad: 0.12,
            ankle_rad: -0.08,
            toe_rad: -0.10,
            ..base
        },
        Parameters {
            pivot_s: 0.40,
            recenter_s: 0.9,
            torso_rad: -0.12,
            ankle_rad: -0.08,
            ..base
        },
        Parameters {
            pivot_s: 0.18,
            recenter_s: 0.4,
            rise_s: 3.0,
            knee_rad: 0.12,
            toe_rad: 0.12,
            ..base
        },
    ]
}

#[test]
fn smooth_trajectory_respects_endpoints_and_finite_derivatives() {
    for duration in [0.15, 0.6, 4.0] {
        assert_eq!(smooth(-0.2, 0.3, 0.0, duration), (-0.2, 0.0));
        assert_eq!(smooth(-0.2, 0.3, duration, duration), (0.3, 0.0));
        for i in 1..100 {
            let (q, v) = smooth(-0.2, 0.3, duration * i as f32 / 100.0, duration);
            assert!((-0.2..=0.3).contains(&q) && v.is_finite());
        }
    }
    assert_eq!(initial_family().len(), 10);
    assert!(initial_family().iter().all(|p| p.within_family()));
}

#[test]
fn owner_allocation_changes_only_joint_motors_and_respects_caps() {
    let fixture = serde_json::from_str::<FixtureSet>(FIXTURES)
        .unwrap()
        .cases
        .into_iter()
        .find(|x| x.id == "landed-crouch-left")
        .unwrap();
    let profile = crate::recovery_contract::pinned_profile();
    let mut inherited = Simulation::build(
        fixture.heading,
        profile.clone(),
        true,
        None,
        Some(&fixture.poses),
    )
    .unwrap();
    let mut owned =
        Simulation::build(fixture.heading, profile, true, None, Some(&fixture.poses)).unwrap();
    let before = serde_json::to_value(inherited.snapshot()).unwrap();
    let bodies_before = serde_json::to_value(&inherited.world.bodies).unwrap();
    let q = coordinates(&inherited).unwrap();
    let dq = [[0.0; 3]; 25];
    let (inherited_sum, _, _) =
        install_targets(&mut inherited, &q, &dq, Allocation::Inherited).unwrap();
    let (owned_sum, _, requests) =
        install_targets(&mut owned, &q, &dq, Allocation::OwnedRequests).unwrap();
    for s in [&inherited, &owned] {
        assert_eq!(serde_json::to_value(s.snapshot()).unwrap(), before);
        assert_eq!(
            serde_json::to_value(&s.world.bodies).unwrap(),
            bodies_before
        );
        assert_eq!(s.bodies.len(), 25);
    }
    assert_eq!(inherited.bodies, owned.bodies);
    assert_eq!(inherited.joints, owned.joints);
    assert_eq!(inherited.colliders, owned.colliders);
    assert_eq!(inherited.generation, owned.generation);
    assert!(inherited_sum <= 760.0 && owned_sum <= 760.0);
    assert!(!requests.is_empty());
    for (i, part) in owned.model.segments.iter().enumerate().skip(1) {
        let a = inherited
            .world
            .impulse_joints
            .get(inherited.joints[i].unwrap())
            .unwrap();
        let b = owned
            .world
            .impulse_joints
            .get(owned.joints[i].unwrap())
            .unwrap();
        for axis in &part.joint_profile.as_ref().unwrap().axes {
            let id = crate::axis(axis.coordinate).unwrap() as usize;
            let x = &a.data.motors[id];
            let y = &b.data.motors[id];
            assert_eq!(
                (x.target_pos, x.target_vel, x.stiffness, x.damping),
                (y.target_pos, y.target_vel, y.stiffness, y.damping)
            );
            assert!(
                x.max_force <= axis.max_motor_torque_nm && y.max_force <= axis.max_motor_torque_nm
            );
        }
    }
}

#[test]
#[ignore = "explicit bounded whole-trajectory offline search; writes fresh evidence"]
fn offline_trajectory_search() {
    let out = PathBuf::from(std::env::var("LH_OFFLINE_OUT").expect("LH_OFFLINE_OUT required"));
    assert!(
        !out.exists(),
        "offline search requires fresh output directory"
    );
    std::fs::create_dir_all(out.join("cases")).unwrap();
    let external_cache = std::env::var("LH_OFFLINE_CACHE").ok().map(PathBuf::from);
    let max_cases = std::env::var("LH_OFFLINE_MAX_CASES")
        .ok()
        .map(|x| x.parse::<usize>().unwrap())
        .unwrap_or(MAX_CASES);
    assert!((1..=MAX_CASES).contains(&max_cases));
    let wall_s = std::env::var("LH_OFFLINE_MAX_WALL_S")
        .ok()
        .map(|x| x.parse::<f64>().unwrap())
        .unwrap_or(MAX_WALL_S);
    assert!(wall_s > 0.0 && wall_s <= MAX_WALL_S);
    let plan = Plan::load().unwrap();
    let profile = crate::recovery_contract::pinned_profile();
    let fixture = serde_json::from_str::<FixtureSet>(FIXTURES)
        .unwrap()
        .cases
        .into_iter()
        .find(|x| x.id == "landed-crouch-left")
        .unwrap();
    let started = Instant::now();
    let evaluations = Cell::new(0usize);
    let mut reports = Vec::<Value>::new();
    let mut best: Option<Value> = None;
    let best_params = Cell::new(Parameters::baseline());
    let best_rise_started = Cell::new(false);
    let best_physical_seconds = Cell::new(0.0_f64);
    let mut evaluate = |params: Parameters,
                        mode: Allocation,
                        force: bool,
                        heading: f32|
     -> Option<Value> {
        if !params.within_family() || started.elapsed().as_secs_f64() >= wall_s {
            return None;
        }
        let key = hash_key(&fixture, &profile, &plan, params, mode, heading);
        let local = out.join("cases").join(format!("{key}.json"));
        let cached = external_cache
            .as_ref()
            .map(|dir| dir.join(format!("{key}.json")))
            .filter(|path| path.exists());
        let cache_hit = !force && (local.exists() || cached.is_some());
        let report = if !force && local.exists() {
            serde_json::from_slice::<Value>(&std::fs::read(&local).unwrap()).unwrap()
        } else if !force && cached.is_some() {
            let bytes = std::fs::read(cached.as_ref().unwrap()).unwrap();
            std::fs::write(&local, &bytes).unwrap();
            serde_json::from_slice(&bytes).unwrap()
        } else {
            if evaluations.get() >= max_cases {
                return None;
            }
            evaluations.set(evaluations.get() + 1);
            let report = run_case(
                &fixture, &profile, &plan, params, mode, heading, started, wall_s, &key,
            )
            .unwrap();
            std::fs::write(&local, serde_json::to_vec(&report).unwrap()).unwrap();
            report
        };
        assert_eq!(report["cacheKey"], key, "cache collision or corrupt report");
        reports.push(json!({"cacheKey":key,"path":local,"allocation":mode,"pivotFamily":report["pivotFamily"],"parameters":params,"physicalSeconds":report["physicalSeconds"],
            "failure":report["failure"],"integrityViolation":integrity_violation(&report),"originalRecoveryQualified":report["originalRecoveryQualified"],"score":score(&report),"cached":cache_hit}));
        if best.as_ref().is_none_or(|prior| better(&report, prior)) {
            best_params.set(params);
            best_rise_started.set(report["riseStartedS"].is_number());
            best_physical_seconds.set(report["physicalSeconds"].as_f64().unwrap_or(0.0));
            best = Some(report.clone());
        }
        std::fs::write(out.join("search-progress.json"),serde_json::to_vec_pretty(&json!({"evaluations":evaluations.get(),"elapsedWallS":started.elapsed().as_secs_f64(),"bestKey":best.as_ref().map(|x|&x["cacheKey"]),"cases":reports})).unwrap()).unwrap();
        Some(report)
    };
    let baseline = Parameters::baseline();
    evaluate(baseline, Allocation::Inherited, false, fixture.heading);
    evaluate(baseline, Allocation::OwnedRequests, false, fixture.heading);
    for params in initial_family().into_iter().skip(1) {
        if evaluate(params, Allocation::OwnedRequests, false, fixture.heading).is_none() {
            break;
        }
    }
    let steps = [0.10, 0.20, 0.50, 0.08, 0.08, 0.06, 0.08, 0.08];
    let mut skipped_inactive_dimensions = Vec::new();
    for _round in 0..2 {
        for (dim, step) in steps.into_iter().enumerate() {
            let center = best_params.get();
            let reached_rise = best_rise_started.get();
            let reached_recenter = best_physical_seconds.get() >= center.pivot_s as f64;
            if dim == 2 && !reached_rise || matches!(dim, 3 | 4 | 6) && !reached_recenter {
                skipped_inactive_dimensions.push(json!({"dimension":dim,"parameters":center,"reason":if dim==2 {"rise gate never reached"}else{"recenter phase never reached"}}));
                continue;
            }
            for direction in [-1.0, 1.0] {
                if let Some(candidate) = center.shifted(dim, step * direction)
                    && evaluate(candidate, Allocation::OwnedRequests, false, fixture.heading)
                        .is_none()
                {
                    break;
                }
            }
            if evaluations.get() >= max_cases || started.elapsed().as_secs_f64() >= wall_s {
                break;
            }
        }
    }
    // A separate, uncached physical validation is required even when the
    // family has no success. A changed trajectory invalidates any ranking.
    let search_elapsed_s = started.elapsed().as_secs_f64();
    let search_budget_stopped = evaluations.get() >= max_cases || search_elapsed_s >= wall_s;
    let validation=best.as_ref().map(|top|{
        let params:Parameters=serde_json::from_value(top["parameters"].clone()).unwrap();
        let mode:Allocation=serde_json::from_value(top["allocation"].clone()).unwrap();
        let key=hash_key(&fixture,&profile,&plan,params,mode,fixture.heading);
        let fresh=run_case(&fixture,&profile,&plan,params,mode,fixture.heading,Instant::now(),wall_s,&key).unwrap();
        let same=top["physicalSteps"]==fresh["physicalSteps"]&&top["failure"]==fresh["failure"]
            &&top["trajectoryTrace"]==fresh["trajectoryTrace"]&&top["final"]==fresh["final"];
        std::fs::write(out.join("independent-best-repeat.json"),serde_json::to_vec(&fresh).unwrap()).unwrap();
        json!({"samePhysicalTraceAndCommands":same,"path":"independent-best-repeat.json","cacheBypassed":true,"bestKey":key})
    });
    let summary = json!({"scope":"Bounded deterministic whole-trajectory search, not proof of mechanical impossibility",
        "sourceVersion":SEARCH_VERSION,"sourceHashInKeys":"FNV-1a64 over offline source, physics/measurement/recovery/support source, canonical model, frozen contract, seed, profile, plan, parameters, allocation and heading",
        "maxCases":max_cases,"evaluations":evaluations.get(),"searchWallBudgetS":wall_s,"validationWallBudgetS":wall_s,
        "elapsedSearchWallS":search_elapsed_s,"elapsedValidationWallS":started.elapsed().as_secs_f64()-search_elapsed_s,"elapsedTotalWallS":started.elapsed().as_secs_f64(),
        "budgetStopped":search_budget_stopped,"skippedInactiveDimensions":skipped_inactive_dimensions,
        "fullIntendedHorizonS":crate::recovery_contract::contract().acceptance.recovery_limit_s+crate::recovery_contract::contract().acceptance.stable_observation_s,
        "bestKey":best.as_ref().map(|x|&x["cacheKey"]),"bestScore":best.as_ref().map(score),"independentBestRepeat":validation,"cases":reports});
    std::fs::write(
        out.join("search-summary.json"),
        serde_json::to_vec_pretty(&summary).unwrap(),
    )
    .unwrap();
}
