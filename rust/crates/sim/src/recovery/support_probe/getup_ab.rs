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

struct Runtime {
    program: CompiledProgram,
    enabled: bool,
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
            let nominal_rate = nominal.velocities[axes[a].segment][axes[a].coordinate];
            let next = (delta[a] - gradient / diagonal).clamp(
                (-1.0 - nominal_rate).max(-0.25),
                (1.0 - nominal_rate).min(0.25),
            );
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

pub(in crate::recovery) fn path_progress(s: &Simulation, program: &CompiledProgram, q: &Coordinates) -> f32 {
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
        let (cap,ratio,_)=offline_trajectory::install_targets(s,&command.positions,&command.velocities,offline_trajectory::Allocation::OwnedRequests)?;
        r.maximum_cap=r.maximum_cap.max(cap);r.maximum_ratio=r.maximum_ratio.max(ratio);
        let mut max_error=0.0_f32;
        for (i,part) in s.model.segments.iter().enumerate().skip(1) {for a in &part.joint_profile.as_ref().unwrap().axes {
            let k=match a.coordinate {'x'=>0,'y'=>1,_=>2};max_error=max_error.max((nominal.positions[i][k]-measured[i][k]).abs());
        }}
        r.last=json!({"commandSubstep":step,"measuredAtNativeSubstep":s.substeps_completed,
            "feedbackEnabled":r.enabled,"nominalPositions":nominal.positions,"nominalVelocities":nominal.velocities,
            "positions":command.positions,"velocities":command.velocities,"measuredPositions":measured,
            "maximumNominalTrackingErrorRad":max_error,"contacts":contacts,"measuredPathProgress":progress,
            "stalledS":r.stalled_s,"unsupportedS":r.unsupported_s,"terminalRequested":terminal,
            "capSumNm":cap,"anatomicalCapRatio":ratio,"motorOwner":"getup-ab"});
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
