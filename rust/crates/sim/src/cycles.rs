//! Five physical impact/fall/get-up/interaction cycles on one owned assembly.
use crate::{Profile, Simulation, measurement, recovery_contract, runtime::Runtime};
use lh_contracts::{Action, Command, Mode, Motion, Stamp};
use rapier3d::parry::query;
use rapier3d::prelude::*;
use serde::Serialize;
use serde_json::{Value, json};

const DESCENT_HEIGHT_M: f32 = 0.55;
const DRAG_DISTANCE_M: f32 = 0.02;
const DRAG_HOLD_S: f64 = 0.25;
const GESTURE_TOLERANCE_M: f32 = 1e-5;

#[derive(Clone, Copy, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct Point {
    tick: u64,
    substep: u32,
    integrated_substeps: u64,
    time_s: f64,
}
impl Point {
    fn new(s: &Simulation, substep: u32) -> Self {
        Self::at(s.tick, substep, s.substeps_completed)
    }
    fn at(tick: u64, substep: u32, integrated_substeps: u64) -> Self {
        Self {
            tick,
            substep,
            integrated_substeps,
            time_s: integrated_substeps as f64 / 240.0,
        }
    }
}

struct Ownership {
    generation: u64,
    bodies: Vec<RigidBodyHandle>,
    colliders: Vec<ColliderHandle>,
    joints: Vec<Option<ImpulseJointHandle>>,
    joint_bodies: Vec<Option<(RigidBodyHandle, RigidBodyHandle)>>,
    floor: ColliderHandle,
}
impl Ownership {
    fn capture(s: &Simulation) -> Self {
        Self {
            generation: s.generation,
            bodies: s.bodies.clone(),
            colliders: s.colliders.clone(),
            joints: s.joints.clone(),
            floor: s.floor,
            joint_bodies: s
                .joints
                .iter()
                .map(|h| {
                    h.map(|h| {
                        let joint = s.world.impulse_joints.get(h).expect("initialized joint");
                        (joint.body1(), joint.body2())
                    })
                })
                .collect(),
        }
    }
    fn check(&self, s: &Simulation, substep: u32) -> Result<(), measurement::Failure> {
        let invalid = if s.generation != self.generation {
            Some("generation replacement".to_string())
        } else if s.bodies != self.bodies || s.colliders != self.colliders || s.floor != self.floor
        {
            Some("body/collider replacement".to_string())
        } else if s.joints != self.joints {
            Some("joint replacement".to_string())
        } else {
            self.bodies
                .iter()
                .enumerate()
                .find_map(|(i, h)| match s.world.bodies.get(*h) {
                    None => Some(format!("{}: missing body", s.model.segments[i].id)),
                    Some(body) => (!body.is_dynamic()
                        || s.world
                            .colliders
                            .get(self.colliders[i])
                            .is_none_or(|c| c.parent() != Some(*h)))
                    .then(|| {
                        format!(
                            "{}: dynamic body/collider ownership",
                            s.model.segments[i].id
                        )
                    }),
                })
                .or_else(|| {
                    self.joints
                        .iter()
                        .zip(&self.joint_bodies)
                        .enumerate()
                        .find_map(|(i, (h, pair))| match h {
                            Some(h) => (s
                                .world
                                .impulse_joints
                                .get(*h)
                                .map(|j| (j.body1(), j.body2()))
                                != *pair)
                                .then(|| format!("{}: joint ownership", s.model.segments[i].id)),
                            None => None,
                        })
                })
        };
        invalid.map_or(Ok(()), |reason| {
            Err(failure(
                s,
                substep,
                "uninterrupted-cycle-ownership",
                reason,
                1.0,
                0.0,
            ))
        })
    }
    fn report(&self, checked_substeps: u64, passed: bool) -> Value {
        json!({"passed":passed,"capturedBeforeIntegration":true,"generation":self.generation,
            "bodyCount":self.bodies.len(),"jointCount":self.joints.iter().flatten().count(),
            "bodyHandles":self.bodies,"colliderHandles":self.colliders,"jointHandles":self.joints,
            "checkedSubsteps":checked_substeps})
    }
}
fn failure(
    s: &Simulation,
    substep: u32,
    check: &str,
    reason: String,
    value: f32,
    limit: f32,
) -> measurement::Failure {
    measurement::Failure::new(s.tick, substep, check, reason, value, limit)
}

#[derive(Default, Serialize)]
#[serde(rename_all = "camelCase")]
struct StandingSample {
    ordinary_authority: bool,
    bilateral_loaded: [bool; 2],
    bilateral_persistent: [bool; 2],
    loads_n: [f32; 2],
    minimum_up_dot: f32,
    maximum_linear_mps: f32,
    maximum_angular_radps: f32,
    stable: bool,
}
struct Standing {
    loaded_substeps: Vec<u64>,
    last: StandingSample,
}
impl Standing {
    fn new(s: &Simulation) -> Self {
        Self {
            loaded_substeps: vec![0; s.bodies.len()],
            last: StandingSample::default(),
        }
    }
    fn observe(&mut self, s: &Simulation, substep: u32) -> Result<bool, measurement::Failure> {
        let c = &recovery_contract::contract().recovery_acceptance;
        let mut sample = StandingSample {
            minimum_up_dot: 1.0,
            ..StandingSample::default()
        };
        let floor = &s.world.colliders[s.floor];
        for (i, segment) in s.model.segments.iter().enumerate() {
            let body = &s.world.bodies[s.bodies[i]];
            sample.maximum_linear_mps = sample.maximum_linear_mps.max(body.linvel().length());
            sample.maximum_angular_radps = sample.maximum_angular_radps.max(body.angvel().length());
            if matches!(
                segment.id.as_str(),
                "pelvis" | "torso" | "leftFoot" | "rightFoot"
            ) {
                sample.minimum_up_dot = sample.minimum_up_dot.min((*body.rotation() * Vector::Y).y);
            }
            if !matches!(segment.role.as_str(), "hindfoot" | "forefoot") {
                continue;
            }
            let side = usize::from(segment.id.starts_with("right"));
            let collider = &s.world.colliders[s.colliders[i]];
            let distance = query::contact(
                floor.position(),
                floor.shape(),
                collider.position(),
                collider.shape(),
                c.contact_distance_m,
            )
            .map_err(|e| {
                failure(
                    s,
                    substep,
                    "cycle-standing-measurement",
                    format!("{}: {e:?}", segment.id),
                    1.0,
                    0.0,
                )
            })?
            .map_or(f32::INFINITY, |contact| contact.dist);
            let load = s
                .world
                .narrow_phase
                .contact_pair(s.floor, s.colliders[i])
                .map(|pair| measurement::support_normal_impulse(pair, s.colliders[i]))
                .transpose()
                .map_err(|e| failure(s, substep, "cycle-standing-measurement", e, 1.0, 0.0))?
                .unwrap_or(0.0)
                / s.profile.dt_s;
            let loaded =
                floor.is_enabled() && distance <= c.contact_distance_m && load >= c.minimum_load_n;
            self.loaded_substeps[i] = if loaded {
                self.loaded_substeps[i] + 1
            } else {
                0
            };
            sample.bilateral_loaded[side] |= loaded;
            sample.bilateral_persistent[side] |=
                self.loaded_substeps[i] >= recovery_contract::substeps(c.load_persistence_s);
            sample.loads_n[side] += load;
        }
        sample.ordinary_authority = matches!(s.motion, Motion::Upright | Motion::Reacting)
            || s.recovery_status()
                .is_some_and(|status| status.phase == crate::recovery::Phase::Ordinary);
        sample.stable = sample.ordinary_authority
            && sample.bilateral_persistent.iter().all(|x| *x)
            && sample.minimum_up_dot >= c.stable_up_dot
            && sample.maximum_linear_mps <= c.stable_linear_mps
            && sample.maximum_angular_radps <= c.stable_angular_radps;
        self.last = sample;
        Ok(self.last.stable)
    }
}

struct AcceptedInteraction {
    press: u64,
    segment: usize,
    began_target: Vector,
    dragged_target: Option<Vector>,
    held_substeps: u64,
    valid: bool,
}

/// Shared browser/native evidence: it reads physics after each integration.
pub(crate) struct QualificationObservation {
    ownership: Ownership,
    standing: Standing,
    stable_substeps: u64,
    checked_substeps: u64,
    last_substeps: u64,
    initial_press: u64,
    interaction: Option<AcceptedInteraction>,
    release_accepted: bool,
    cycle: Option<RecoveryObservation>,
    cycle_number: u32,
    last_impact_id: u32,
    first_failure: Option<measurement::Failure>,
}
impl QualificationObservation {
    pub(crate) fn new(s: &Simulation) -> Self {
        Self {
            ownership: Ownership::capture(s),
            standing: Standing::new(s),
            stable_substeps: 0,
            checked_substeps: 0,
            last_substeps: s.substeps_completed,
            initial_press: s.last_press,
            interaction: None,
            release_accepted: false,
            cycle: None,
            cycle_number: 0,
            last_impact_id: s
                .striker
                .as_ref()
                .map_or(0, |striker| striker.snapshot(&s.world).impact_id),
            first_failure: None,
        }
    }
    pub(crate) fn observe(
        &mut self,
        s: &Simulation,
        substep: u32,
    ) -> Result<(), measurement::Failure> {
        if let Some(f) = &self.first_failure {
            return Err(f.clone());
        }
        let result = self.ownership.check(s, substep).and_then(|_| {
            if s.substeps_completed != self.last_substeps + 1 {
                return Err(failure(
                    s,
                    substep,
                    "qualification-integrated-clock",
                    "nonconsecutive integration".into(),
                    s.substeps_completed as f32,
                    (self.last_substeps + 1) as f32,
                ));
            }
            self.last_substeps = s.substeps_completed;
            self.checked_substeps += 1;
            self.stable_substeps = if self.standing.observe(s, substep)? {
                self.stable_substeps + 1
            } else {
                0
            };
            let striker = s.striker.as_ref().map(|striker| striker.snapshot(&s.world));
            let new_impact = striker
                .as_ref()
                .filter(|impact| impact.impact_id > self.last_impact_id);
            if let Some(impact) = new_impact {
                if impact.impact_id != self.last_impact_id + 1 {
                    return Err(failure(
                        s,
                        substep,
                        "cycle-striker-impact",
                        "nonconsecutive observed impacts".into(),
                        impact.impact_id as f32,
                        (self.last_impact_id + 1) as f32,
                    ));
                }
                self.last_impact_id = impact.impact_id;
            }
            if let Some(cycle) = &mut self.cycle
                && cycle.completed.is_none()
            {
                let pelvis = &s.world.bodies[s.bodies[0]];
                cycle
                    .observe_return(
                        Point::new(s, substep),
                        new_impact.map(|impact| impact.last_impact_impulse_ns),
                        pelvis.translation().y,
                        pelvis.linvel().y < 0.0,
                        self.standing.last.stable,
                        s.counters.recoveries >= self.cycle_number
                            && matches!(s.motion, Motion::Upright | Motion::Reacting)
                            && s.recovery_status().is_none(),
                    )
                    .map_err(|check| {
                        failure(
                            s,
                            substep,
                            check,
                            "observed strike/fall/stable-return contract".into(),
                            cycle.impact.map_or(0.0, |impact| {
                                (s.substeps_completed - impact.integrated_substeps) as f32 / 240.0
                            }),
                            recovery_contract::contract().acceptance.recovery_limit_s as f32,
                        )
                    })?;
            }
            if let Some(interaction) = &mut self.interaction
                && let Some(target) = interaction.dragged_target
            {
                let held = s.grab.as_ref().is_some_and(|grab| {
                    grab.press == interaction.press
                        && grab.segment == interaction.segment
                        && grab.raw_target == target
                });
                if held && interaction.valid {
                    interaction.held_substeps += 1;
                } else if !self.release_accepted {
                    interaction.valid = false;
                }
            }
            Ok(())
        });
        if let Err(f) = &result {
            self.first_failure.get_or_insert(f.clone());
        }
        result
    }
    pub(crate) fn applied(&mut self, s: &Simulation, action: &Action) {
        match action {
            Action::Strike | Action::StrikeFromBehind => {
                if self.cycle.is_some()
                    && !(self.release_accepted
                        && self.stable_substeps
                            >= recovery_contract::substeps(
                                recovery_contract::contract()
                                    .acceptance
                                    .stable_observation_s,
                            ))
                {
                    self.first_failure.get_or_insert_with(|| failure(s, 0, "cycle-fresh-interaction",
                        "next strike preceded fresh hand interaction and stable postrelease return".into(), 0.0, 1.0));
                }
                self.cycle_number += 1;
                self.cycle = Some(RecoveryObservation::new(Point::new(s, 0)));
                self.interaction = None;
                self.release_accepted = false;
                self.stable_substeps = 0;
            }
            Action::GrabBegin {
                press,
                segment,
                target,
                local_anchor,
            } => {
                let i = usize::from(*segment);
                let actual_anchor = s.world.bodies[s.bodies[i]]
                    .position()
                    .transform_point(crate::vector(*local_anchor));
                let fresh = *press > self.initial_press
                    && crate::vector(*target).distance(actual_anchor) <= GESTURE_TOLERANCE_M
                    && (self.interaction.is_none() || self.release_accepted)
                    && self
                        .cycle
                        .as_ref()
                        .is_some_and(|cycle| cycle.completed.is_some())
                    && self.stable_substeps
                        >= recovery_contract::substeps(
                            recovery_contract::contract()
                                .acceptance
                                .stable_observation_s,
                        )
                    && s.model.segments[i].role == "hand";
                self.interaction = Some(AcceptedInteraction {
                    press: *press,
                    segment: i,
                    began_target: actual_anchor,
                    dragged_target: None,
                    held_substeps: 0,
                    valid: fresh,
                });
                self.release_accepted = false;
            }
            Action::GrabMove { press, target } => {
                if let Some(interaction) = &mut self.interaction
                    && *press == interaction.press
                {
                    let target = crate::vector(*target);
                    interaction.valid &=
                        ((target - interaction.began_target).length() - DRAG_DISTANCE_M).abs()
                            <= GESTURE_TOLERANCE_M;
                    interaction.dragged_target = Some(target);
                    interaction.held_substeps = 0;
                }
            }
            Action::GrabEnd { press } => {
                self.release_accepted = self.interaction.as_ref().is_some_and(|interaction| {
                    interaction.press == *press
                        && interaction.valid
                        && interaction.held_substeps >= recovery_contract::substeps(DRAG_HOLD_S)
                }) && s.grab.is_none();
                self.stable_substeps = 0;
            }
            _ => {}
        }
    }
    pub(crate) fn report(&self, s: &Simulation) -> Value {
        let ownership_unchanged = self.ownership.check(s, 0).is_ok();
        let impact = self.cycle.as_ref().and_then(|cycle| cycle.impact);
        let completed = self.cycle.as_ref().and_then(|cycle| cycle.completed);
        json!({"profile":s.profile,"generation":s.generation,"bodyHandles":s.bodies,"jointHandles":s.joints,
            "colliderHandles":s.colliders,"tick":s.tick,"substepsCompleted":s.substeps_completed,
            "simulationTimeS":s.substeps_completed as f64 / 240.0,"stableSubsteps":self.stable_substeps,
            "standingSupported":self.standing.last.bilateral_persistent.iter().all(|b| *b),
            "standingValid":self.standing.last.stable,"standingEvidence":self.standing.last,
            "ownershipUnchanged":ownership_unchanged,"capturedBeforeIntegration":true,
            "checkedSubsteps":self.checked_substeps,"releaseAccepted":false,
            "interactionReleaseAccepted":self.release_accepted,
            "interactionHeldSubsteps":self.interaction.as_ref().map_or(0, |i| i.held_substeps),
            "cycle":self.cycle_number,"impactSubstep":impact.map(|p|p.integrated_substeps),
            "stableReturnSubstep":completed.map(|p|p.integrated_substeps),
            "impactToStableSubsteps":completed.zip(impact).map(|(end, began)|end.integrated_substeps - began.integrated_substeps),
            "impact":impact,"stableReturn":completed,
            "physicalDescent":self.cycle.as_ref().and_then(|cycle|cycle.descent),
            "recoveryStableSubsteps":self.cycle.as_ref().map_or(0, |cycle|cycle.stable_substeps),
            "impactToStableSeconds":self.cycle.as_ref().and_then(RecoveryObservation::elapsed_s),
            "firstFailure":self.first_failure.as_ref().or(s.first_failure.as_ref()),
            "contract":recovery_contract::contract()})
    }
}

struct RecoveryObservation {
    began: Point,
    impact: Option<Point>,
    impact_impulse_ns: f32,
    descent: Option<Point>,
    minimum_pelvis_y: f32,
    stable_substeps: u64,
    stable_start: Option<Point>,
    completed: Option<Point>,
    interruptions: u32,
}
impl RecoveryObservation {
    fn new(began: Point) -> Self {
        Self {
            began,
            impact: None,
            impact_impulse_ns: 0.0,
            descent: None,
            minimum_pelvis_y: f32::INFINITY,
            stable_substeps: 0,
            stable_start: None,
            completed: None,
            interruptions: 0,
        }
    }
    #[cfg(test)]
    fn observe(
        &mut self,
        point: Point,
        new_impact_impulse_ns: Option<f32>,
        pelvis_y: f32,
        descending: bool,
        stable: bool,
    ) -> Result<(), &'static str> {
        self.observe_return(
            point,
            new_impact_impulse_ns,
            pelvis_y,
            descending,
            stable,
            true,
        )
    }
    fn observe_return(
        &mut self,
        point: Point,
        new_impact_impulse_ns: Option<f32>,
        pelvis_y: f32,
        descending: bool,
        stable: bool,
        returned: bool,
    ) -> Result<(), &'static str> {
        let limit =
            recovery_contract::substeps(recovery_contract::contract().acceptance.recovery_limit_s);
        if self.impact.is_none()
            && let Some(impulse) = new_impact_impulse_ns
        {
            if !impulse.is_finite() || impulse <= 0.25 {
                return Err("cycle-striker-impact");
            }
            self.impact = Some(point);
            self.impact_impulse_ns = impulse;
        }
        if let Some(impact) = self.impact {
            self.minimum_pelvis_y = self.minimum_pelvis_y.min(pelvis_y);
            if pelvis_y < DESCENT_HEIGHT_M && descending {
                self.descent.get_or_insert(point);
            }
            if self.descent.is_some() && stable {
                if self.stable_substeps == 0 {
                    self.stable_start = Some(point);
                }
                self.stable_substeps += 1;
                if returned
                    && self.stable_substeps
                        >= recovery_contract::substeps(
                            recovery_contract::contract()
                                .acceptance
                                .stable_observation_s,
                        )
                {
                    self.completed.get_or_insert(point);
                }
            } else {
                if self.stable_substeps > 0 {
                    self.interruptions += 1;
                }
                self.stable_substeps = 0;
                self.stable_start = None;
                self.completed = None;
            }
            if self
                .completed
                .is_some_and(|end| end.integrated_substeps - impact.integrated_substeps > limit)
                || (point.integrated_substeps - impact.integrated_substeps >= limit
                    && self.completed.is_none())
            {
                return Err("cycle-recovery-deadline");
            }
        } else if point.integrated_substeps - self.began.integrated_substeps >= limit {
            return Err("cycle-striker-impact");
        }
        Ok(())
    }
    fn elapsed_s(&self) -> Option<f64> {
        self.completed
            .zip(self.impact)
            .map(|(end, impact)| end.time_s - impact.time_s)
    }
}

fn apply(runtime: &mut Runtime, sequence: &mut u64, action: Action) -> Value {
    let s = runtime.simulation();
    *sequence += 1;
    let command = Command {
        stamp: Stamp::new(s.generation, *sequence, s.tick),
        action,
    };
    let result = runtime.enqueue(command).and_then(|_| runtime.advance(0));
    match result {
        Ok(reply) => reply.acknowledgements.into_iter().find(|a| a.stamp.sequence == *sequence)
            .map(|ack| json!({"accepted":ack.rejected.is_none() && ack.applied_tick.is_some(), "acknowledgement":ack}))
            .unwrap_or_else(|| json!({"accepted":false,"reason":"missing acknowledgement"})),
        Err(rejected) => json!({"accepted":false,"rejected":rejected}),
    }
}

fn interaction(
    runtime: &mut Runtime,
    sequence: &mut u64,
    ownership: &Ownership,
    checked_substeps: &mut u64,
    standing: &mut Standing,
    trace: &mut Vec<lh_contracts::Snapshot>,
) -> (Value, Option<measurement::Failure>) {
    let mut report = json!({"passed":false,"attempted":true,"dragDistanceM":DRAG_DISTANCE_M,"requiredHoldSeconds":DRAG_HOLD_S});
    let s = runtime.simulation();
    let fail = |s: &Simulation, reason: &str| {
        failure(s, 0, "cycle-fresh-interaction", reason.into(), 0.0, 1.0)
    };
    if let Err(f) = ownership.check(s, 0) {
        return (report, Some(f));
    }
    if s.grab.is_some() || !standing.last.stable {
        return (
            report,
            Some(fail(
                s,
                "fresh press requires stable return and no active grab",
            )),
        );
    }
    let Some(segment) = s
        .model
        .segments
        .iter()
        .position(|part| part.id == "rightHand")
    else {
        return (report, Some(fail(s, "missing right hand")));
    };
    let local = s.model.segments[segment]
        .geometry
        .vertices
        .chunks_exact(3)
        .map(|v| Vector::new(v[0], v[1], v[2]))
        .max_by(|a, b| a.x.total_cmp(&b.x))
        .unwrap_or(Vector::ZERO);
    let target = s.world.bodies[s.bodies[segment]]
        .position()
        .transform_point(local);
    let dragged = target + Vector::X * DRAG_DISTANCE_M;
    let previous_press = s.last_press;
    let press = previous_press + 1;
    report["press"] = json!(press);
    report["previousPress"] = json!(previous_press);
    report["segment"] = json!(s.model.segments[segment].id);
    report["beganTimeS"] = json!(s.substeps_completed as f64 / 240.0);
    report["begin"] = apply(
        runtime,
        sequence,
        Action::GrabBegin {
            press,
            segment: segment as u8,
            local_anchor: crate::vec3(local),
            target: crate::vec3(target),
        },
    );
    if report["begin"]["accepted"] != true {
        return (
            report,
            Some(fail(runtime.simulation(), "fresh hand press rejected")),
        );
    }
    report["move"] = apply(
        runtime,
        sequence,
        Action::GrabMove {
            press,
            target: crate::vec3(dragged),
        },
    );
    if report["move"]["accepted"] != true {
        return (
            report,
            Some(fail(runtime.simulation(), "gentle drag rejected")),
        );
    }
    let hold_began = runtime.simulation().substeps_completed;
    let mut held_substeps = 0;
    let mut last_failure = None;
    while held_substeps < recovery_contract::substeps(DRAG_HOLD_S) {
        let reply = runtime.advance_observed(1, |s, substep| {
            ownership.check(s, substep)?;
            *checked_substeps += 1;
            let valid = s.grab.as_ref().is_some_and(|grab| {
                grab.press == press
                    && grab.segment == segment
                    && grab.local_anchor == local
                    && grab.raw_target == dragged
            });
            if !valid || s.last_press != press {
                return Err(failure(
                    s,
                    substep,
                    "cycle-fresh-interaction",
                    "held press/drag was cancelled or replaced".into(),
                    0.0,
                    1.0,
                ));
            }
            standing.observe(s, substep)?;
            held_substeps += 1;
            Ok(())
        });
        match reply {
            Ok(reply) => {
                if reply.snapshot.stamp.tick % 15 == 0 {
                    trace.push(reply.snapshot);
                }
                if reply.failure.is_some() {
                    last_failure = reply.failure;
                    break;
                }
            }
            Err(e) => {
                last_failure = Some(fail(
                    runtime.simulation(),
                    &format!("hold advance rejected: {e:?}"),
                ));
                break;
            }
        }
    }
    report["heldSubsteps"] = json!(held_substeps);
    report["heldSeconds"] =
        json!((runtime.simulation().substeps_completed - hold_began) as f64 / 240.0);
    if last_failure.is_some() {
        return (report, last_failure);
    }
    report["release"] = apply(runtime, sequence, Action::GrabEnd { press });
    if report["release"]["accepted"] != true || runtime.simulation().grab.is_some() {
        return (
            report,
            Some(fail(
                runtime.simulation(),
                "hand release rejected or still held",
            )),
        );
    }
    if let Err(f) = ownership.check(runtime.simulation(), 0) {
        return (report, Some(f));
    }
    let release_substeps = runtime.simulation().substeps_completed;
    report["releasedTimeS"] = json!(release_substeps as f64 / 240.0);
    let required = recovery_contract::substeps(
        recovery_contract::contract()
            .acceptance
            .stable_observation_s,
    );
    let limit =
        recovery_contract::substeps(recovery_contract::contract().acceptance.recovery_limit_s);
    let mut stable_substeps = 0;
    let mut interruptions = 0;
    let mut returned = None;
    while stable_substeps < required {
        let reply = runtime.advance_observed(1, |s, substep| {
            ownership.check(s, substep)?;
            *checked_substeps += 1;
            if s.grab.is_some() {
                return Err(failure(
                    s,
                    substep,
                    "cycle-fresh-interaction",
                    "grab resumed after release".into(),
                    1.0,
                    0.0,
                ));
            }
            if standing.observe(s, substep)? {
                stable_substeps += 1;
                if stable_substeps == required {
                    returned = Some(Point::new(s, substep));
                }
            } else {
                if stable_substeps > 0 {
                    interruptions += 1;
                }
                stable_substeps = 0;
                returned = None;
            }
            if s.substeps_completed - release_substeps >= limit && returned.is_none() {
                return Err(failure(
                    s,
                    substep,
                    "cycle-post-interaction-stability",
                    "released hand did not return to stable standing".into(),
                    (s.substeps_completed - release_substeps) as f32 / 240.0,
                    limit as f32 / 240.0,
                ));
            }
            Ok(())
        });
        match reply {
            Ok(reply) => {
                if reply.snapshot.stamp.tick % 15 == 0 {
                    trace.push(reply.snapshot);
                }
                if reply.failure.is_some() {
                    last_failure = reply.failure;
                    break;
                }
            }
            Err(e) => {
                last_failure = Some(fail(
                    runtime.simulation(),
                    &format!("return advance rejected: {e:?}"),
                ));
                break;
            }
        }
    }
    report["stableReturn"] = json!(returned);
    report["stableReturnSubsteps"] = json!(stable_substeps);
    report["stableReturnInterruptions"] = json!(interruptions);
    report["standingEvidence"] = json!(standing.last);
    report["passed"] = json!(
        last_failure.is_none()
            && returned.is_some()
            && held_substeps >= recovery_contract::substeps(DRAG_HOLD_S)
    );
    (report, last_failure)
}

pub fn trial(profile: &Profile) -> Result<Value, String> {
    if !profile.recovery_enabled {
        return Err("Cycle fixture requires recovery_enabled".into());
    }
    let mut runtime = Runtime::new(Mode::Protocol, 0.0, profile.clone())?;
    let ownership = Ownership::capture(runtime.simulation());
    let initial = runtime.advance(0).map_err(|e| format!("{e:?}"))?.snapshot;
    let mut snapshot = initial.clone();
    let mut standing = Standing::new(runtime.simulation());
    let mut cycles = Vec::new();
    let mut trace = Vec::new();
    let mut sequence = 0;
    let mut checked_substeps = 0;
    let mut first_failure = None;
    let mut continuity_passed = true;
    for cycle in 1..=recovery_contract::contract().acceptance.repeated_cycles {
        if let Err(f) = ownership.check(runtime.simulation(), 0) {
            continuity_passed = false;
            first_failure = Some(f);
            break;
        }
        let before = runtime.simulation();
        let baseline_impact = before
            .striker
            .as_ref()
            .unwrap()
            .snapshot(&before.world)
            .impact_id;
        let mut observation = RecoveryObservation::new(Point::new(before, 0));
        let mut recovery_point = None;
        let strike = apply(&mut runtime, &mut sequence, Action::Strike);
        if strike["accepted"] != true {
            first_failure = Some(failure(
                runtime.simulation(),
                0,
                "cycle-striker-command",
                "strike rejected".into(),
                0.0,
                1.0,
            ));
        }
        while first_failure.is_none() && observation.completed.is_none() {
            let reply = runtime
                .advance_observed(1, |s, substep| {
                    ownership.check(s, substep)?;
                    checked_substeps += 1;
                    let impact = s.striker.as_ref().unwrap().snapshot(&s.world);
                    if impact.impact_id > baseline_impact + 1 {
                        return Err(failure(
                            s,
                            substep,
                            "cycle-striker-impact",
                            "multiple impacts during one requested strike".into(),
                            impact.impact_id as f32,
                            (baseline_impact + 1) as f32,
                        ));
                    }
                    if s.counters.recoveries >= cycle {
                        recovery_point.get_or_insert(Point::new(s, substep));
                    }
                    let body = &s.world.bodies[s.bodies[0]];
                    let stable = standing.observe(s, substep)?;
                    let observed_impact = (impact.impact_id == baseline_impact + 1
                        && observation.impact.is_none())
                    .then_some(impact.last_impact_impulse_ns);
                    observation
                        .observe_return(
                            Point::new(s, substep),
                            observed_impact,
                            body.translation().y,
                            body.linvel().y < 0.0,
                            stable,
                            s.counters.recoveries >= cycle
                                && matches!(s.motion, Motion::Upright | Motion::Reacting)
                                && s.recovery_status().is_none(),
                        )
                        .map_err(|check| {
                            failure(
                                s,
                                substep,
                                check,
                                if observation.impact.is_none() {
                                    "no measured striker contact"
                                } else {
                                    "impact-to-complete uninterrupted standing exceeded deadline"
                                }
                                .into(),
                                observation.impact.map_or(0.0, |p| {
                                    (s.substeps_completed - p.integrated_substeps) as f32 / 240.0
                                }),
                                recovery_contract::contract().acceptance.recovery_limit_s as f32,
                            )
                        })
                })
                .map_err(|e| format!("{e:?}"))?;
            snapshot = reply.snapshot;
            if snapshot.stamp.tick % 15 == 0 {
                trace.push(snapshot.clone());
            }
            if let Some(f) = reply.failure {
                continuity_passed &= f.check != "uninterrupted-cycle-ownership";
                first_failure = Some(f);
            }
        }
        let mut interaction_report = json!({"passed":false,"attempted":false});
        if first_failure.is_none() {
            let (report, failed) = interaction(
                &mut runtime,
                &mut sequence,
                &ownership,
                &mut checked_substeps,
                &mut standing,
                &mut trace,
            );
            interaction_report = report;
            if let Some(f) = failed {
                continuity_passed &= f.check != "uninterrupted-cycle-ownership";
                first_failure = Some(f);
            }
            snapshot = runtime.advance(0).map_err(|e| format!("{e:?}"))?.snapshot;
        }
        let passed = first_failure.is_none()
            && observation.impact.is_some()
            && observation.descent.is_some()
            && observation.completed.is_some()
            && interaction_report["passed"] == true;
        cycles.push(json!({"cycle":cycle,"passed":passed,"beganTick":observation.began.tick,
            "strike":strike,"impact":observation.impact,"impactImpulseNs":observation.impact_impulse_ns,
            "fallTick":observation.descent.map(|p|p.tick),"physicalDescent":observation.descent,
            "recoveryTick":recovery_point.map(|p|p.tick),"controllerRecovery":recovery_point,
            "minimumPelvisY":observation.minimum_pelvis_y.is_finite().then_some(observation.minimum_pelvis_y),
            "stableStart":observation.stable_start,"stableCompletion":observation.completed,
            "stableSubsteps":observation.stable_substeps,"stableHoldInterruptions":observation.interruptions,
            "stableHoldSeconds":observation.stable_substeps as f64 / 240.0,
            "impactToStableSeconds":observation.elapsed_s(),"interaction":interaction_report,
            "elapsedSinceImpactSeconds":observation.impact.map(|impact|
                runtime.simulation().substeps_completed as f64 / 240.0 - impact.time_s),
            "continuityPassed":continuity_passed,"standingEvidence":standing.last}));
        if !passed {
            break;
        }
    }
    Ok(
        json!({"passed":cycles.len() == recovery_contract::contract().acceptance.repeated_cycles as usize
        && cycles.iter().all(|c| c["passed"] == true),"requiredCycles":recovery_contract::contract().acceptance.repeated_cycles,
        "completedCycles":cycles.iter().filter(|c| c["passed"] == true).count(),"cycles":cycles,
        "contract":recovery_contract::contract(),"firstFailure":first_failure,
        "continuity":ownership.report(checked_substeps, continuity_passed),
        "initial":initial,"final":snapshot,"trace":trace}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    fn point(n: u64) -> Point {
        Point::at(n / 4, (n % 4) as u32, n)
    }
    fn fallen() -> RecoveryObservation {
        let mut observation = RecoveryObservation::new(point(0));
        observation
            .observe(point(17), Some(1.0), 0.9, false, false)
            .unwrap();
        observation
            .observe(point(20), None, 0.5, true, false)
            .unwrap();
        observation
    }
    #[test]
    fn descent_and_upright_labels_without_measured_impact_never_pass() {
        let mut observation = RecoveryObservation::new(point(0));
        for n in 1..6000 {
            observation
                .observe(point(n), None, 0.4, true, true)
                .unwrap();
        }
        assert!(observation.completed.is_none());
        assert_eq!(
            observation.observe(point(6000), None, 0.4, true, true),
            Err("cycle-striker-impact")
        );
    }
    #[test]
    fn one_second_hold_must_finish_within_twenty_five_seconds_of_actual_impact() {
        let mut on_time = fallen();
        for n in 5778..=6017 {
            on_time.observe(point(n), None, 0.9, false, true).unwrap();
        }
        assert_eq!(on_time.completed.unwrap().integrated_substeps, 6017);
        assert_eq!(on_time.elapsed_s(), Some(25.0));
        on_time
            .observe(point(6018), None, 0.9, false, true)
            .unwrap();
        assert_eq!(on_time.elapsed_s(), Some(25.0));
        let mut late = fallen();
        for n in 5779..6017 {
            late.observe(point(n), None, 0.9, false, true).unwrap();
        }
        assert_eq!(
            late.observe(point(6017), None, 0.9, false, true),
            Err("cycle-recovery-deadline")
        );
        assert!(late.completed.is_none());
    }
    #[test]
    fn interrupted_stability_resets_the_entire_hold() {
        let mut observation = fallen();
        for n in 100..339 {
            observation
                .observe(point(n), None, 0.9, false, true)
                .unwrap();
        }
        observation
            .observe(point(339), None, 0.9, false, false)
            .unwrap();
        for n in 340..579 {
            observation
                .observe(point(n), None, 0.9, false, true)
                .unwrap();
        }
        assert!(observation.completed.is_none());
        assert_eq!(observation.interruptions, 1);
        observation
            .observe(point(579), None, 0.9, false, true)
            .unwrap();
        assert_eq!(observation.stable_substeps, 240);
        assert_eq!(observation.completed.unwrap().integrated_substeps, 579);
    }

    #[test]
    fn ordinary_stance_hold_counts_before_upright_label_but_requires_observed_return() {
        let mut observation = fallen();
        for n in 100..340 {
            observation
                .observe_return(point(n), None, 0.95, false, true, false)
                .unwrap();
        }
        assert_eq!(observation.stable_substeps, 240);
        assert!(observation.completed.is_none());
        observation
            .observe_return(point(340), None, 0.95, false, true, true)
            .unwrap();
        assert_eq!(observation.completed.unwrap().integrated_substeps, 340);
    }

    #[test]
    fn impact_and_stable_standing_without_physical_descent_do_not_complete_a_cycle() {
        let mut observation = RecoveryObservation::new(point(0));
        observation
            .observe(point(17), Some(1.0), 0.95, false, true)
            .unwrap();
        for n in 18..6017 {
            observation
                .observe(point(n), None, 0.95, false, true)
                .unwrap();
        }
        assert!(observation.descent.is_none());
        assert!(observation.completed.is_none());
        assert_eq!(
            observation.observe(point(6017), None, 0.95, false, true),
            Err("cycle-recovery-deadline")
        );
    }
    #[test]
    fn ownership_is_frozen_before_the_first_integration_and_rejects_replacement() {
        let mut s = Simulation::new(0.0, Profile::default(), true).unwrap();
        let baseline = Ownership::capture(&s);
        assert_eq!(s.substeps_completed, 0);
        baseline.check(&s, 0).unwrap();
        s.generation += 1;
        assert_eq!(
            baseline.check(&s, 0).unwrap_err().offending,
            "generation replacement"
        );
        s.generation -= 1;
        let handle = s.world.bodies.insert(RigidBodyBuilder::dynamic().build());
        s.bodies[0] = handle;
        assert_eq!(
            baseline.check(&s, 0).unwrap_err().offending,
            "body/collider replacement"
        );
        s.bodies = baseline.bodies.clone();
        let joint =
            s.world
                .impulse_joints
                .insert(s.bodies[0], s.bodies[1], FixedJoint::new(), true);
        s.joints[1] = Some(joint);
        assert_eq!(
            baseline.check(&s, 0).unwrap_err().offending,
            "joint replacement"
        );
    }
    #[test]
    fn airborne_upright_bodies_cannot_manufacture_loaded_persistent_support() {
        let s = Simulation::new(0.0, Profile::default(), true).unwrap();
        let mut standing = Standing::new(&s);
        for _ in 0..240 {
            assert!(!standing.observe(&s, 0).unwrap());
        }
        assert_eq!(standing.last.bilateral_persistent, [false; 2]);
    }
    #[test]
    fn stale_held_press_and_reset_dependent_interaction_are_rejected() {
        let mut runtime = Runtime::new(Mode::Protocol, 0.0, Profile::default()).unwrap();
        let ownership = Ownership::capture(runtime.simulation());
        let mut standing = Standing::new(runtime.simulation());
        standing.last.stable = true;
        let mut sequence = 0;
        let mut checked = 0;
        let mut trace = Vec::new();
        assert_eq!(
            apply(
                &mut runtime,
                &mut sequence,
                Action::GrabBegin {
                    press: 1,
                    segment: 14,
                    local_anchor: lh_model::Vec3::default(),
                    target: lh_model::Vec3::default()
                }
            )["accepted"],
            true
        );
        let (_, failure) = interaction(
            &mut runtime,
            &mut sequence,
            &ownership,
            &mut checked,
            &mut standing,
            &mut trace,
        );
        assert_eq!(failure.unwrap().check, "cycle-fresh-interaction");
        assert_eq!(
            apply(&mut runtime, &mut sequence, Action::Reset)["accepted"],
            true
        );
        let (_, failure) = interaction(
            &mut runtime,
            &mut sequence,
            &ownership,
            &mut checked,
            &mut standing,
            &mut trace,
        );
        assert_eq!(failure.unwrap().check, "uninterrupted-cycle-ownership");
    }

    #[test]
    fn qualification_rejects_offset_press_origin_without_rejecting_public_grab_commands() {
        for initial_offset_m in [0.0, 0.10] {
            let mut s =
                Simulation::new_protocol(0.0, recovery_contract::pinned_profile(), true).unwrap();
            let mut observer = QualificationObservation::new(&s);
            // Isolate gesture admission after an already proved stable return.
            let mut cycle = fallen();
            cycle.completed = Some(point(260));
            observer.cycle = Some(cycle);
            observer.stable_substeps = 240;
            let hand = s
                .model
                .segments
                .iter()
                .position(|part| part.id == "rightHand")
                .unwrap();
            let local_anchor = lh_model::Vec3 {
                x: 0.005,
                y: 0.003,
                z: 0.002,
            };
            let actual_anchor = s.world.bodies[s.bodies[hand]]
                .position()
                .transform_point(crate::vector(local_anchor));
            let initial_target = actual_anchor + Vector::X * initial_offset_m;
            let begin = Action::GrabBegin {
                press: 1,
                segment: hand as u8,
                local_anchor,
                target: crate::vec3(initial_target),
            };
            s.apply_grab_action(&begin)
                .expect("public press remains accepted");
            observer.applied(&s, &begin);
            assert_eq!(
                observer.interaction.as_ref().unwrap().valid,
                initial_offset_m == 0.0
            );
            assert_eq!(
                observer.interaction.as_ref().unwrap().began_target,
                actual_anchor
            );
            let movement = Action::GrabMove {
                press: 1,
                target: crate::vec3(initial_target + Vector::X * DRAG_DISTANCE_M),
            };
            s.apply_grab_action(&movement)
                .expect("public movement remains accepted");
            observer.applied(&s, &movement);
            if initial_offset_m > 0.0 {
                assert!(!observer.interaction.as_ref().unwrap().valid);
            }
            let end = Action::GrabEnd { press: 1 };
            s.apply_grab_action(&end)
                .expect("public release remains accepted");
            observer.applied(&s, &end);
            assert!(!observer.release_accepted);
        }
    }

    #[test]
    fn runtime_measurement_counts_actual_substeps_and_never_rebaselines_after_reset() {
        let profile = recovery_contract::pinned_profile();
        let mut runtime = Runtime::new(Mode::Protocol, 0.0, profile).unwrap();
        runtime.enable_recovery_measurement().unwrap();
        assert_eq!(
            runtime.enable_recovery_measurement(),
            Err(lh_contracts::Reject::Invalid)
        );
        let initial = runtime.recovery_progress().unwrap();
        assert_eq!(initial["substepsCompleted"], 0);
        assert_eq!(initial["capturedBeforeIntegration"], true);
        let reply = runtime.advance(1).unwrap();
        assert!(reply.failure.is_none(), "{:?}", reply.failure);
        let progress = runtime.recovery_progress().unwrap();
        assert_eq!(progress["substepsCompleted"], 4);
        assert_eq!(progress["checkedSubsteps"], 4);
        assert_eq!(progress["simulationTimeS"], 1.0 / 60.0);
        assert_eq!(progress["releaseAccepted"], false);
        let mut sequence = 0;
        assert_eq!(
            apply(&mut runtime, &mut sequence, Action::Pause)["accepted"],
            true
        );
        runtime.advance(4).unwrap();
        assert_eq!(runtime.recovery_progress().unwrap()["checkedSubsteps"], 4);
        assert_eq!(
            apply(&mut runtime, &mut sequence, Action::Resume)["accepted"],
            true
        );
        assert_eq!(
            apply(&mut runtime, &mut sequence, Action::Reset)["accepted"],
            true
        );
        assert_eq!(
            runtime.recovery_progress().unwrap()["ownershipUnchanged"],
            false
        );
        assert_eq!(
            runtime.advance(1).unwrap().failure.unwrap().check,
            "uninterrupted-cycle-ownership"
        );
    }

    #[test]
    fn qualification_rejects_mutated_profile_without_changing_the_trial() {
        for change_id in [false, true] {
            let mut profile = recovery_contract::pinned_profile();
            if change_id {
                profile.id.push_str("-mutated");
            } else {
                profile.solver_iterations += 1;
            }
            let mut runtime = Runtime::new(Mode::Protocol, 0.0, profile).unwrap();
            let before = runtime.advance(0).unwrap().snapshot;
            assert_eq!(
                runtime.enable_recovery_measurement(),
                Err(lh_contracts::Reject::Invalid)
            );
            assert!(runtime.recovery_progress().is_none());
            let after = runtime.advance(0).unwrap().snapshot;
            assert_eq!(before.stamp.generation, after.stamp.generation);
            assert_eq!(before.stamp.tick, after.stamp.tick);
            assert_eq!(before.simulation_time_s, after.simulation_time_s);
            assert_eq!(
                serde_json::to_value(before.segments).unwrap(),
                serde_json::to_value(after.segments).unwrap()
            );
        }
    }
}
