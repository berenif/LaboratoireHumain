//! One frozen A/B executor. No lookahead, body writes, retiming or search.
//! The kinematic feedback is a command generator, not inverse dynamics.
use super::{offline_trajectory, velocity};
use crate::Simulation;
use crate::recovery::{
    self,
    trajectory_program::{Command, CompiledProgram, Coordinates},
};
use rapier3d::prelude::*;
use serde_json::{Value, json};
use std::cell::RefCell;

pub(crate) const PARAMETERS: &str = r#"{"version":"getup-ab-v1","captureGainPerS":4.0,"maximumRelativeComVelocityMps":0.15,"maximumCorrectionRateRadps":0.25,"maximumOffsetRad":0.12,"leastSquaresRegularization":0.001,"contactWeight":4.0,"coordinateSweeps":24,"minimumPersistentLoadN":3.0,"terminalRequest":"nominal program end","progress":"measured nearest point on complete nominal joint path","stallIncrement":0.005}"#;
pub(crate) const COORDINATION_PARAMETERS: &str = r#"{"version":"foot-coordination-v1","witness":"archived B feedback","activeWindows":"unchanged nominal contact annotations; final intent retained during settle","physicalContacts":"all remain enabled; command tasks use only observed intended surfaces","axes":"left/right Thigh Shin Ankle Forefoot x","goals":"hindfoot/forefoot world sagittal pitch zero, shank pitch in ankle range, posterior sole to floor","gainRule":"min native ankle/forefoot posture stiffness divided by damping","maximumTaskAngularRateRadps":0.25,"angularWeightRule":"4 * native half segment length squared","maximumHeelSpeedRule":"measured heel-to-support horizontal lever * 0.25","maximumOffsetRad":0.12,"anatomicalBoundsInsideCoordinateSolve":true,"preferredCorrectionRateRadps":0.25,"forcedCancellation":"minimum nominal cancellation at anatomical/slew boundary; final velocity <=1","floatingRoot":"not commanded; kinematic approximation only"}"#;

struct Runtime {
    program: CompiledProgram,
    enabled: bool,
    coordination: bool,
    start: u64,
    previous_step: Option<u64>,
    previous: Coordinates,
    offsets: Coordinates,
    released: bool,
    best_progress: f32,
    stalled_s: f32,
    unsupported_s: f32,
    last: Value,
    maximum_cap: f64,
    maximum_ratio: f32,
}
thread_local! { static ACTIVE: RefCell<Option<Runtime>> = const { RefCell::new(None) }; }

pub(super) fn activate(program: CompiledProgram, s: &Simulation, enabled: bool) {
    ACTIVE.with(|slot| {
        assert!(slot.borrow().is_none());
        *slot.borrow_mut() = Some(Runtime {
            previous: program.command(0).positions,
            program,
            enabled,
            coordination: false,
            start: s.substeps_completed,
            previous_step: None,
            offsets: [[0.0; 3]; 25],
            released: false,
            best_progress: 0.0,
            stalled_s: 0.0,
            unsupported_s: 0.0,
            last: Value::Null,
            maximum_cap: 0.0,
            maximum_ratio: 0.0,
        });
    });
}
pub(super) fn activate_coordination(program: CompiledProgram, s: &Simulation, enabled: bool) {
    activate(program, s, true);
    ACTIVE.with(|slot| slot.borrow_mut().as_mut().unwrap().coordination = enabled);
}
pub(super) fn finish() -> (bool, f64, f32) {
    ACTIVE.with(|slot| {
        let r = slot.borrow_mut().take().unwrap();
        (r.released, r.maximum_cap, r.maximum_ratio)
    })
}
pub(super) fn last_command() -> Value {
    ACTIVE.with(|slot| {
        slot.borrow().as_ref().map_or(Value::Null, |r| {
            if r.released {
                json!({"motorOwner":"native-handoff-or-ordinary","released":true})
            } else {
                r.last.clone()
            }
        })
    })
}

pub(crate) fn active() -> bool {
    ACTIVE.with(|slot| slot.borrow().is_some())
}

/// dq's world angular generator for the native coupled quaternion chart.
/// q = normalize([tan(qx/2), tan(qy/2), tan(qz/2), 1]).
fn chart_axis(q: [f32; 3], k: usize) -> Vector {
    let t = Vector::new((q[0] * 0.5).tan(), (q[1] * 0.5).tan(), (q[2] * 0.5).tan());
    let e = [Vector::X, Vector::Y, Vector::Z][k];
    (e + t.cross(e)) * ((1.0 + t[k] * t[k]) / (1.0 + t.length_squared()))
}

struct Axis {
    segment: usize,
    coordinate: usize,
    pivot: Vector,
    angular: Vector,
}
fn axes(s: &Simulation, measured: &Coordinates) -> Vec<Axis> {
    let mut result = Vec::new();
    for (i, part) in s.model.segments.iter().enumerate().skip(1) {
        let p = part.joint_profile.as_ref().unwrap();
        let parent = recovery::index(s, part.parent.as_ref().unwrap());
        let b = &s.world.bodies[s.bodies[parent]];
        let pivot = b
            .position()
            .transform_point(crate::vector(p.parent_frame.anchor));
        let frame = *b.rotation() * crate::rotation(p.parent_frame.rotation);
        for a in &p.axes {
            let k = match a.coordinate {
                'x' => 0,
                'y' => 1,
                _ => 2,
            };
            result.push(Axis {
                segment: i,
                coordinate: k,
                pivot,
                angular: frame * chart_axis(measured[i], k),
            });
        }
    }
    result
}
fn descendant(s: &Simulation, mut i: usize, ancestor: usize) -> bool {
    loop {
        if i == ancestor {
            return true;
        }
        let Some(parent) = &s.model.segments[i].parent else {
            return false;
        };
        i = recovery::index(s, parent);
    }
}
fn point_columns(s: &Simulation, axes: &[Axis], segment: usize, point: Vector) -> Vec<Vector> {
    axes.iter()
        .map(|a| {
            if descendant(s, segment, a.segment) {
                a.angular.cross(point - a.pivot)
            } else {
                Vector::ZERO
            }
        })
        .collect()
}

fn intended(r: &Runtime, name: &str, step: u64) -> bool {
    let lookup_step = step.min(u64::from(r.program.duration_substeps().saturating_sub(1)));
    r.program.source().contacts.iter().any(|c| {
        c.segment == name
            && u64::from(c.start_substep) <= lookup_step
            && lookup_step < u64::from(c.end_substep)
    })
}
fn sagittal_axis(s: &Simulation, a: &Axis) -> bool {
    a.coordinate == 0
        && matches!(
            s.model.segments[a.segment].role.as_str(),
            "thigh" | "shin" | "ankle" | "forefoot"
        )
}
type TaskRow = (Vec<f32>, f32, f32);
fn coordination_rows(
    s: &Simulation,
    r: &Runtime,
    axes: &[Axis],
    nominal: &Command,
    contacts: &[Value],
    step: u64,
) -> Result<Vec<TaskRow>, String> {
    let mut rows = Vec::new();
    for side in ["left", "right"] {
        let rear = recovery::index(s, &format!("{side}Foot"));
        let front = recovery::index(s, &format!("{side}Forefoot"));
        let shin = recovery::index(s, &format!("{side}Shin"));
        let ankle = recovery::index(s, &format!("{side}Ankle"));
        if !(intended(r, &s.model.segments[rear].id, step)
            || intended(r, &s.model.segments[front].id, step))
        {
            continue;
        }
        let support = contacts
            .iter()
            .filter(|c| {
                c["segment"]
                    .as_u64()
                    .is_some_and(|i| i as usize == rear || i as usize == front)
            })
            .collect::<Vec<_>>();
        if support.is_empty() {
            continue;
        }
        let gain = [ankle, front]
            .into_iter()
            .map(|i| {
                let a = &s.model.segments[i].joint_profile.as_ref().unwrap().axes[0];
                a.passive_stiffness_nm_per_rad * s.profile.posture_stiffness_multiplier
                    / (a.damping_nms_per_rad * s.profile.posture_damping_multiplier)
            })
            .fold(f32::INFINITY, f32::min);
        for segment in [rear, front, shin] {
            let b = &s.world.bodies[s.bodies[segment]];
            let forward = s.heading.inverse() * (*b.rotation() * Vector::Z);
            let denominator = forward.y * forward.y + forward.z * forward.z;
            if denominator < 1e-8 {
                return Err("Singular sagittal foot coordination chart".into());
            }
            let pitch = (-forward.y).atan2(forward.z);
            let target = if segment == shin {
                let bounds = &s.model.segments[ankle].joint_profile.as_ref().unwrap().axes[0];
                pitch.clamp(bounds.min_radians, bounds.max_radians)
            } else {
                0.0
            };
            let desired = (-gain * (pitch - target)).clamp(-0.25, 0.25);
            let j = axes
                .iter()
                .map(|a| {
                    if sagittal_axis(s, a) && descendant(s, segment, a.segment) {
                        let df = (s.heading.inverse() * a.angular).cross(forward);
                        (-forward.z * df.y + forward.y * df.z) / denominator
                    } else {
                        0.0
                    }
                })
                .collect::<Vec<_>>();
            let rate = j
                .iter()
                .zip(axes)
                .map(|(a, b)| a * nominal.velocities[b.segment][b.coordinate])
                .sum::<f32>();
            let vertices = &s.model.segments[segment].geometry.vertices;
            let (low, high) = vertices
                .chunks_exact(3)
                .map(|p| p[if segment == shin { 1 } else { 2 }])
                .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), x| {
                    (lo.min(x), hi.max(x))
                });
            let lever = (high - low) * 0.5;
            rows.push((j, desired - rate, 4.0 * lever * lever));
        }
        let geometry = &s.model.segments[rear].geometry.vertices;
        let z = geometry
            .chunks_exact(3)
            .map(|p| p[2])
            .fold(f32::INFINITY, f32::min);
        let y = geometry
            .chunks_exact(3)
            .map(|p| p[1])
            .fold(f32::INFINITY, f32::min);
        let heel = s.world.bodies[s.bodies[rear]]
            .position()
            .transform_point(Vector::new(0.0, y, z));
        let mut average = vec![Vector::ZERO; axes.len()];
        let mut support_point = Vector::ZERO;
        for c in &support {
            let point = crate::vector(serde_json::from_value(c["worldPoint"].clone()).unwrap());
            support_point += point / support.len() as f32;
            let columns = point_columns(s, axes, c["segment"].as_u64().unwrap() as usize, point);
            for (a, b) in average.iter_mut().zip(columns) {
                *a += b / support.len() as f32;
            }
        }
        let heel_columns = point_columns(s, axes, rear, heel);
        let j = heel_columns
            .iter()
            .zip(&average)
            .zip(axes)
            .map(|((a, b), axis)| {
                if sagittal_axis(s, axis) {
                    (a - b).y
                } else {
                    0.0
                }
            })
            .collect::<Vec<_>>();
        let mut lever = heel - support_point;
        lever.y = 0.0;
        let max_speed = lever.length() * 0.25;
        let desired = (-gain * (heel.y - s.support_height())).clamp(-max_speed, max_speed);
        let rate = j
            .iter()
            .zip(axes)
            .map(|(a, b)| a * nominal.velocities[b.segment][b.coordinate])
            .sum::<f32>();
        rows.push((j, desired - rate, 4.0));
    }
    Ok(rows)
}

fn correction_bounds(s: &Simulation, r: &Runtime, nominal: &Command, axis: &Axis) -> (f32, f32) {
    let i = axis.segment;
    let k = axis.coordinate;
    let velocity = nominal.velocities[i][k];
    let mut low = (-1.0 - velocity).max(-0.25);
    let mut high = (1.0 - velocity).min(0.25);
    if r.coordination {
        let bound = s.model.segments[i]
            .joint_profile
            .as_ref()
            .unwrap()
            .axes
            .iter()
            .find(|a| a.coordinate == ['x', 'y', 'z'][k])
            .unwrap();
        let q = nominal.positions[i][k] + r.offsets[i][k];
        let dt = s.profile.dt_s;
        let physical_low = (-1.0 - velocity)
            .max((bound.min_radians - q) / dt)
            .max((-0.12 - r.offsets[i][k]) / dt)
            .max((r.previous[i][k] - dt - q) / dt);
        let physical_high = (1.0 - velocity)
            .min((bound.max_radians - q) / dt)
            .min((0.12 - r.offsets[i][k]) / dt)
            .min((r.previous[i][k] + dt - q) / dt);
        low = physical_low.max(-0.25);
        high = physical_high.min(0.25);
        if physical_low <= physical_high && low > high {
            // Minimum cancellation of nominal motion at an anatomical/slew
            // boundary. Final target and total velocity limits stay native.
            let required = 0.0_f32.clamp(physical_low, physical_high);
            low = required;
            high = required;
        }
    }
    (low, high)
}

pub(super) fn foot_geometry(s: &Simulation) -> Vec<Value> {
    ["leftFoot","rightFoot","leftForefoot","rightForefoot"].into_iter().map(|name| {
        let i=recovery::index(s,name);let b=&s.world.bodies[s.bodies[i]];let forward=s.heading.inverse()*(*b.rotation()*Vector::Z);
        let vertices=s.model.segments[i].geometry.vertices.chunks_exact(3).filter(|p|p[2] < -0.02).map(|p|b.position().transform_point(Vector::new(p[0],p[1],p[2])).y-s.support_height()).collect::<Vec<_>>();
        json!({"id":name,"nativeSubstep":s.substeps_completed,"worldPitchRad":(-forward.y).atan2(forward.z),"posteriorMinimumHeightM":vertices.into_iter().reduce(f32::min)})
    }).collect()
}

pub(super) fn motor_delivery(s: &Simulation) -> Vec<Value> {
    s.model.segments.iter().enumerate().filter_map(|(i,part)| {
        let profile=part.joint_profile.as_ref()?;let joint=s.world.impulse_joints.get(s.joints[i]?)?;
        Some(json!({"id":part.id,"nativeSubstep":s.substeps_completed,"axes":profile.axes.iter().map(|a| {
            let k=match a.coordinate {'x'=>0,'y'=>1,_=>2};let motor=&joint.data.motors[k+3];
            json!({"coordinate":a.coordinate,"installedCapNm":motor.max_force,
                "nativeScalarImpulseNms":motor.delivered_impulse,"nativeScalarConvention":"constraint row; not generalized child torque",
                "absoluteImpulseNms":motor.delivered_abs_impulse,
                "parentAngularImpulseWorldNms":crate::vec3(motor.delivered_angular_impulse_1),
                "childAngularImpulseWorldNms":crate::vec3(motor.delivered_angular_impulse_2),
                "nativeScalarCapRatio":motor.delivered_impulse.abs()/(motor.max_force*s.profile.dt_s).max(1e-12)})
        }).collect::<Vec<_>>()}))
    }).collect()
}

pub(super) fn persistent_contacts(s: &Simulation) -> Result<Vec<Value>, String> {
    let mut result = Vec::new();
    for (i, part) in s.model.segments.iter().enumerate() {
        if recovery::load(s, &part.id) < 3.0 {
            continue;
        }
        for (anchor, strength) in recovery::planted_contacts(s, &[i])? {
            result.push(json!({"nativeSubstep":s.substeps_completed,"segment":i,"id":part.id,
                "localPoint":crate::vec3(anchor.local_point),"worldPoint":crate::vec3(anchor.world_point),
                "latestSelectedNormalImpulseNs":strength,
                "originalPersistentLoadN":recovery::load(s,&part.id),
                "integratedSupportSubsteps":s.integrated_support_substeps[i],
                "region":if part.role=="hindfoot" && anchor.local_point.z < -0.02 {"posterior-hindfoot"} else {"native-surface"}}));
        }
    }
    if result.len() > 100 {
        return Err("A/B contact row bound exceeded".into());
    }
    Ok(result)
}

fn feedback(
    s: &Simulation,
    r: &mut Runtime,
    nominal: &Command,
    measured: &Coordinates,
    contacts: &[Value],
    step: u64,
) -> Result<Command, String> {
    let axes = axes(s, measured);
    let n = axes.len();
    let control_contacts = if r.coordination {
        contacts
            .iter()
            .filter(|c| intended(r, c["id"].as_str().unwrap(), step))
            .cloned()
            .collect::<Vec<_>>()
    } else {
        contacts.to_vec()
    };
    let contacts = control_contacts.as_slice();
    let mut rows: Vec<(Vec<f32>, f32, f32)> = Vec::new();
    if !contacts.is_empty() {
        let mut contact_columns = Vec::new();
        let mut support = Vector::ZERO;
        for c in contacts {
            let segment = c["segment"].as_u64().unwrap() as usize;
            let point = crate::vector(serde_json::from_value(c["worldPoint"].clone()).unwrap());
            support += point;
            contact_columns.push(point_columns(s, &axes, segment, point));
        }
        support /= contacts.len() as f32;
        let mut average = vec![Vector::ZERO; n];
        for columns in &contact_columns {
            for (a, b) in average.iter_mut().zip(columns) {
                *a += *b / contacts.len() as f32;
            }
        }
        // Relative geometry cancels an uncommanded common translation. Root
        // rotation/dynamic tracking are not certified by this approximation.
        let mut com_columns = vec![Vector::ZERO; n];
        for (i, h) in s.bodies.iter().enumerate() {
            let b = &s.world.bodies[*h];
            let columns = point_columns(s, &axes, i, b.center_of_mass());
            for (a, bcol) in com_columns.iter_mut().zip(columns) {
                *a += bcol * (b.mass() / s.total_mass());
            }
        }
        let com = s.center_of_mass();
        let capture = com + velocity(s) * ((com.y - s.support_height()).max(0.2) / 9.81).sqrt();
        let points = contacts
            .iter()
            .map(|c| {
                let p = &c["worldPoint"];
                [
                    p["x"].as_f64().unwrap() as f32,
                    p["z"].as_f64().unwrap() as f32,
                ]
            })
            .collect::<Vec<_>>();
        let target = crate::stepping::project_support(&points, [capture.x, capture.z], 0.003)
            .unwrap_or([support.x, support.z]);
        let desired = Vector::new(target[0] - capture.x, 0.0, target[1] - capture.z) * 4.0;
        let desired = desired.clamp_length_max(0.15);
        for k in [0, 2] {
            let j = com_columns
                .iter()
                .zip(&average)
                .map(|(a, b)| (a - b)[k])
                .collect::<Vec<_>>();
            let nominal_rate = j
                .iter()
                .zip(&axes)
                .map(|(a, b)| a * nominal.velocities[b.segment][b.coordinate])
                .sum::<f32>();
            rows.push((j, desired[k] - nominal_rate, 1.0));
        }
        for columns in contact_columns {
            for k in 0..3 {
                let j = columns
                    .iter()
                    .zip(&average)
                    .map(|(a, b)| (a - b)[k])
                    .collect::<Vec<_>>();
                let nominal_rate = j
                    .iter()
                    .zip(&axes)
                    .map(|(a, b)| a * nominal.velocities[b.segment][b.coordinate])
                    .sum::<f32>();
                rows.push((j, -nominal_rate, 4.0 / contacts.len() as f32));
            }
        }
    }
    if r.coordination {
        rows.extend(coordination_rows(s, r, &axes, nominal, contacts, step)?);
    }
    let mut delta = vec![0.0_f32; n];
    let mut residual = rows.iter().map(|(_, b, _)| -b).collect::<Vec<_>>();
    for _ in 0..24 {
        for a in 0..n {
            let mut gradient = 0.001 * delta[a];
            let mut diagonal = 0.001;
            for ((j, _, weight), error) in rows.iter().zip(&residual) {
                gradient += weight * j[a] * error;
                diagonal += weight * j[a] * j[a];
            }
            let (low, high) = correction_bounds(s, r, nominal, &axes[a]);
            if low > high {
                return Err("Empty anticipated foot command intersection".into());
            }
            let next = (delta[a] - gradient / diagonal).clamp(low, high);
            let change = next - delta[a];
            delta[a] = next;
            for ((j, _, _), error) in rows.iter().zip(&mut residual) {
                *error += j[a] * change;
            }
        }
    }
    let mut command = nominal.clone();
    for (a, d) in axes.iter().zip(delta) {
        let i = a.segment;
        let k = a.coordinate;
        let profile = s.model.segments[i].joint_profile.as_ref().unwrap();
        let bounds = profile
            .axes
            .iter()
            .find(|b| b.coordinate == ['x', 'y', 'z'][k])
            .unwrap();
        let q = nominal.positions[i][k];
        let previous_offset = r.offsets[i][k];
        let offset = (previous_offset + d * s.profile.dt_s).clamp(-0.12, 0.12);
        let requested = if offset == 0.0 { q } else { q + offset };
        let mut low = (r.previous[i][k] - s.profile.dt_s)
            .max(bounds.min_radians)
            .max(q - 0.12);
        let mut high = (r.previous[i][k] + s.profile.dt_s)
            .min(bounds.max_radians)
            .min(q + 0.12);
        while r.previous[i][k] - low > s.profile.dt_s {
            low = low.next_up();
        }
        while high - r.previous[i][k] > s.profile.dt_s {
            high = high.next_down();
        }
        if low > high {
            return Err("Empty A/B anatomical/slew intersection".into());
        }
        command.positions[i][k] = if step == 0 {
            q
        } else {
            requested.clamp(low, high)
        };
        r.offsets[i][k] = command.positions[i][k] - q;
        let offset_delta = r.offsets[i][k] - previous_offset;
        command.velocities[i][k] = if step == 0 || offset_delta == 0.0 {
            nominal.velocities[i][k]
        } else {
            (nominal.velocities[i][k] + offset_delta / s.profile.dt_s).clamp(-1.0, 1.0)
        };
    }
    recovery::trajectory_program::validate_command(&s.model, &command)?;
    Ok(command)
}

fn path_progress(s: &Simulation, program: &CompiledProgram, q: &Coordinates) -> f32 {
    let keys = &program.source().keyframes;
    let mut best_distance = f32::INFINITY;
    let mut best = 0.0;
    let mut cumulative = 0.0;
    let mut total = 0.0;
    for pair in keys.windows(2) {
        let mut length = 0.0;
        let mut projection = 0.0;
        for (i, part) in s.model.segments.iter().enumerate().skip(1) {
            for a in &part.joint_profile.as_ref().unwrap().axes {
                let k = match a.coordinate {
                    'x' => 0,
                    'y' => 1,
                    _ => 2,
                };
                let d = pair[1].positions[i][k] - pair[0].positions[i][k];
                length += d * d;
                projection += (q[i][k] - pair[0].positions[i][k]) * d;
            }
        }
        let t = if length > 1e-12 {
            (projection / length).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let mut distance = 0.0;
        for (i, part) in s.model.segments.iter().enumerate().skip(1) {
            for a in &part.joint_profile.as_ref().unwrap().axes {
                let k = match a.coordinate {
                    'x' => 0,
                    'y' => 1,
                    _ => 2,
                };
                distance += (q[i][k]
                    - (pair[0].positions[i][k]
                        + t * (pair[1].positions[i][k] - pair[0].positions[i][k])))
                    .powi(2);
            }
        }
        let arc = length.sqrt();
        if distance < best_distance {
            best_distance = distance;
            best = cumulative + t * arc;
        }
        cumulative += arc;
        total += arc;
    }
    if total > 0.0 { best / total } else { 0.0 }
}

/// Called instead of native target generation while this executor owns motors.
/// At handoff it permanently returns ownership to the original configure path.
pub(crate) fn configure(s: &mut Simulation) -> Result<bool, String> {
    ACTIVE.with(|slot| {
        let mut slot=slot.borrow_mut();let Some(r)=slot.as_mut() else {return Ok(false)};
        if r.released {return Ok(false);}
        let step=s.substeps_completed.checked_sub(r.start).ok_or("A/B clock rewind")?;
        if r.previous_step.is_some_and(|old|step!=old+1) {return Err("A/B clock discontinuity".into());}
        if step>=6000 {return Err("A/B original 25 s recovery deadline".into());}
        if s.recovery.as_ref().is_some_and(|c|c.handed_off()) || s.counters.recoveries>0 {
            r.released=true;r.previous_step=Some(step);return Ok(false);
        }
        let measured=offline_trajectory::coordinates(s)?;
        let progress=path_progress(s,&r.program,&measured);
        if progress>r.best_progress+0.005 {r.best_progress=progress;r.stalled_s=0.0;} else {r.stalled_s+=s.profile.dt_s;}
        r.unsupported_s=if recovery::has_persistent_support(s) {0.0} else {r.unsupported_s+s.profile.dt_s};
        let contract=crate::recovery_contract::contract();
        if f64::from(r.unsupported_s)>=contract.recovery_acceptance.support_loss_s {return Err("A/B original support loss duration".into());}
        if f64::from(r.stalled_s)>=contract.recovery_acceptance.stall_s {return Err("A/B measured path stagnation duration".into());}
        let contacts=persistent_contacts(s)?;
        let nominal=r.program.command(u32::try_from(step).map_err(|_|"A/B clock overflow")?);
        let command=if r.enabled {feedback(s,r,&nominal,&measured,&contacts,step)?} else {nominal.clone()};
        recovery::trajectory_program::validate_command(&s.model,&command)?;
        if s.recovery.is_none() {
            if r.previous_step.is_some() {
                return Err("Original recovery observer refused A/B; no restart".into());
            }
            s.recovery=Some(recovery::Controller::new(s)?);
            s.motion=lh_contracts::Motion::Recovering;s.stepping=None;
        }
        let anchors=["left","right"].map(|side| {
            if recovery::foot(s,side) {recovery::planted_contact(s,&[recovery::index(s,&format!("{side}Foot")),recovery::index(s,&format!("{side}Forefoot"))])} else {Ok(None)}
        });
        let anchors=[anchors[0].clone()?,anchors[1].clone()?];
        let terminal=step>=u64::from(r.program.duration_substeps());
        let controller=s.recovery.as_mut().unwrap();
        // Intent only; original observe_recovery independently checks every
        // physical condition, consecutive settle, measured anchors and hold.
        controller.status.phase=if terminal {recovery::Phase::Stand} else {recovery::Phase::Brace};
        controller.status.elapsed_s=step as f32*s.profile.dt_s;
        controller.status.phase_time_s=controller.status.elapsed_s;
        controller.status.stalled_s=r.stalled_s;controller.status.unsupported_s=r.unsupported_s;
        controller.status.progress=progress;controller.foot_anchors=anchors;
        controller.targets=command.positions.to_vec();controller.velocities=command.velocities.to_vec();
        s.targets=command.positions.to_vec();
        let (cap,ratio,requests)=offline_trajectory::install_targets(s,&command.positions,&command.velocities,offline_trajectory::Allocation::OwnedRequests)?;
        r.maximum_cap=r.maximum_cap.max(cap);r.maximum_ratio=r.maximum_ratio.max(ratio);
        let mut max_error=0.0_f32;
        for (i,part) in s.model.segments.iter().enumerate().skip(1) {for a in &part.joint_profile.as_ref().unwrap().axes {
            let k=match a.coordinate {'x'=>0,'y'=>1,_=>2};max_error=max_error.max((nominal.positions[i][k]-measured[i][k]).abs());
        }}
        r.last=json!({"commandSubstep":step,"measuredAtNativeSubstep":s.substeps_completed,
            "feedbackEnabled":r.enabled,"footCoordinationEnabled":r.coordination,"nominalPositions":nominal.positions,"nominalVelocities":nominal.velocities,
            "positions":command.positions,"velocities":command.velocities,"measuredPositions":measured,
            "maximumNominalTrackingErrorRad":max_error,"contacts":contacts,"measuredPathProgress":progress,
            "stalledS":r.stalled_s,"unsupportedS":r.unsupported_s,"terminalRequested":terminal,
            "capSumNm":cap,"anatomicalCapRatio":ratio,"ownerMotorRequests":requests,"motorOwner":"getup-ab"});
        r.previous=command.positions;r.previous_step=Some(step);
        Ok(true)
    })
}

#[test]
fn coupled_chart_generator_matches_rotation_differences_without_integration() {
    let q = [0.43_f32, -0.31, 0.27];
    let rotation = |v: [f32; 3]| {
        Rotation::from_xyzw(
            (v[0] * 0.5).tan(),
            (v[1] * 0.5).tan(),
            (v[2] * 0.5).tan(),
            1.0,
        )
        .normalize()
    };
    let initial = rotation(q);
    for k in 0..3 {
        let mut plus = q;
        let mut minus = q;
        plus[k] += 0.001;
        minus[k] -= 0.001;
        let fd = (rotation(plus) * initial.inverse()).to_scaled_axis()
            - (rotation(minus) * initial.inverse()).to_scaled_axis();
        assert!((fd / 0.002 - chart_axis(q, k)).length() < 0.0001);
    }
}

#[test]
fn owner_rejects_duplicate_clock_and_releases_at_handoff_without_integration() {
    let mut s = Simulation::new(0.0, crate::recovery_contract::pinned_profile(), true).unwrap();
    let program = recovery::trajectory_program::neutral_program(&s.model)
        .compile(&s.model)
        .unwrap();
    let before = serde_json::to_value(&s.world.bodies).unwrap();
    activate(program, &s, false);
    assert!(configure(&mut s).unwrap());
    assert!(
        configure(&mut s)
            .unwrap_err()
            .contains("clock discontinuity")
    );
    assert_eq!(serde_json::to_value(&s.world.bodies).unwrap(), before);
    // Isolated boundary injection only; no claim of an integrated handoff.
    s.substeps_completed += 1;
    s.recovery.as_mut().unwrap().status.phase = recovery::Phase::Handoff;
    assert!(!configure(&mut s).unwrap());
    assert!(!configure(&mut s).unwrap());
    assert!(finish().0);
    assert_eq!(s.counters.recoveries, 0);
    assert_eq!(serde_json::to_value(&s.world.bodies).unwrap(), before);
}

#[test]
fn zero_feedback_preserves_analytic_nominal_commands_without_integration() {
    let s = Simulation::new(0.0, crate::recovery_contract::pinned_profile(), true).unwrap();
    let mut source = recovery::trajectory_program::neutral_program(&s.model);
    source.keyframes[1].positions[recovery::index(&s, "lumbar")][0] = 0.1;
    let program = source.compile(&s.model).unwrap();
    let measured = offline_trajectory::coordinates(&s).unwrap();
    let before = serde_json::to_value(&s.world.bodies).unwrap();
    activate(program.clone(), &s, true);
    ACTIVE.with(|slot| {
        let mut slot = slot.borrow_mut();
        let r = slot.as_mut().unwrap();
        for step in 0..240 {
            let nominal = program.command(step);
            let command = feedback(&s, r, &nominal, &measured, &[], u64::from(step)).unwrap();
            for i in 0..25 {
                for k in 0..3 {
                    assert_eq!(
                        command.positions[i][k].to_bits(),
                        nominal.positions[i][k].to_bits()
                    );
                    assert_eq!(
                        command.velocities[i][k].to_bits(),
                        nominal.velocities[i][k].to_bits()
                    );
                }
            }
            r.previous = command.positions;
        }
    });
    finish();
    assert_eq!(serde_json::to_value(&s.world.bodies).unwrap(), before);
}

#[test]
fn coordinated_foot_rows_include_upstream_chain_without_body_mutation() {
    let fixtures: recovery::FixtureSet = serde_json::from_str(recovery::FIXTURES).unwrap();
    let seed = fixtures
        .cases
        .iter()
        .find(|c| c.id == "landed-crouch-left")
        .unwrap();
    let s = Simulation::build(
        0.0,
        crate::recovery_contract::pinned_profile(),
        true,
        None,
        Some(&seed.poses),
    )
    .unwrap();
    let before = serde_json::to_value(&s.world.bodies).unwrap();
    let mut source = recovery::trajectory_program::neutral_program(&s.model);
    source
        .contacts
        .push(recovery::trajectory_program::ContactSurface {
            segment: "leftForefoot".into(),
            triangle: 9,
            barycentric: [1.0, 0.0, 0.0],
            start_substep: 0,
            end_substep: 1920,
        });
    let program = source.compile(&s.model).unwrap();
    activate_coordination(program.clone(), &s, true);
    let front = recovery::index(&s, "leftForefoot");
    let point = s.world.bodies[s.bodies[front]]
        .position()
        .transform_point(Vector::new(0.0, -0.045, 0.06));
    // Algebra-only support fixture, never a claimed resolved physical contact.
    let contacts =
        vec![json!({"id":"leftForefoot","segment":front,"worldPoint":crate::vec3(point)})];
    ACTIVE.with(|slot| {
        let slot = slot.borrow();
        let r = slot.as_ref().unwrap();
        let axes = axes(&s, &offline_trajectory::coordinates(&s).unwrap());
        let rows = coordination_rows(&s, r, &axes, &program.command(0), &contacts, 0).unwrap();
        assert_eq!(rows.len(), 4);
        assert!(rows.iter().all(|(_, b, _)| *b < 0.0));
        for name in ["leftThigh", "leftShin", "leftAnkle"] {
            let column = axes
                .iter()
                .position(|a| s.model.segments[a.segment].id == name && a.coordinate == 0)
                .unwrap();
            assert!(rows[0].0[column].abs() > 0.1);
        }
        assert!(
            coordination_rows(&s, r, &axes, &program.command(0), &[], 0)
                .unwrap()
                .is_empty()
        );
    });
    finish();
    assert_eq!(s.counters.recoveries, 0);
    assert_eq!(serde_json::to_value(&s.world.bodies).unwrap(), before);
}

#[test]
fn anticipated_limit_cancels_nominal_motion_with_native_envelope() {
    let s = Simulation::new(0.0, crate::recovery_contract::pinned_profile(), true).unwrap();
    let before = serde_json::to_value(&s.world.bodies).unwrap();
    let program = recovery::trajectory_program::neutral_program(&s.model)
        .compile(&s.model)
        .unwrap();
    activate_coordination(program.clone(), &s, true);
    let measured = offline_trajectory::coordinates(&s).unwrap();
    let axes = axes(&s, &measured);
    let axis = axes
        .iter()
        .find(|a| s.model.segments[a.segment].id == "leftAnkle" && a.coordinate == 0)
        .unwrap();
    ACTIVE.with(|slot| {
        let mut slot = slot.borrow_mut();
        let r = slot.as_mut().unwrap();
        let bound = s.model.segments[axis.segment]
            .joint_profile
            .as_ref()
            .unwrap()
            .axes[0]
            .max_radians;
        let mut nominal = program.command(0);
        nominal.positions[axis.segment][0] = bound - 0.004;
        nominal.velocities[axis.segment][0] = 0.5;
        r.previous[axis.segment][0] = bound;
        r.offsets[axis.segment][0] = 0.006;
        let (low, high) = correction_bounds(&s, r, &nominal, axis);
        assert_eq!(low, high);
        assert!(high < -0.25);
        let q =
            nominal.positions[axis.segment][0] + r.offsets[axis.segment][0] + high * s.profile.dt_s;
        assert!(q <= bound);
        assert!((q - r.previous[axis.segment][0]).abs() <= s.profile.dt_s);
        assert!((nominal.velocities[axis.segment][0] + high).abs() <= 1.0);
    });
    finish();
    assert_eq!(serde_json::to_value(&s.world.bodies).unwrap(), before);
}
