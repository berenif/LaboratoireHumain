//! Read-only acceptance observer used by the actual worker lifecycle owner.
use crate::{
    Simulation,
    measurement::{Failure, QuietMetrics},
};
use lh_contracts::Snapshot;
use serde_json::{Value, json};

pub(crate) const SETTLE_TICKS: u64 = 120;
pub(crate) const END_TICK: u64 = 1920;

pub(crate) struct QuietObservation {
    heading: f32,
    initial: Snapshot,
    settled: Option<Snapshot>,
    metrics: QuietMetrics,
    settling_substeps: u32,
    quiet_substeps: u32,
    drift_trace: Option<Vec<Value>>,
}
impl QuietObservation {
    pub fn new(simulation: &Simulation, heading: f32) -> Self {
        Self {
            heading,
            initial: simulation.snapshot(),
            settled: None,
            metrics: QuietMetrics::default(),
            settling_substeps: 0,
            quiet_substeps: 0,
            drift_trace: None,
        }
    }
    pub fn enable_drift_trace(&mut self, simulation: &Simulation) -> bool {
        if self.drift_trace.is_some() {
            return false;
        }
        self.drift_trace = Some(vec![drift_sample(simulation, None)]);
        true
    }
    pub fn observe(&mut self, simulation: &Simulation, substep: u32) -> Result<(), Failure> {
        let result = self.observe_metrics(simulation, substep);
        if let Some(trace) = &mut self.drift_trace {
            // One physical sample per second, plus the first failing substep.
            // This clock uses completed integrations, including a partial tick.
            if (substep == 3 && (simulation.tick + 1) % 60 == 0) || result.is_err() {
                trace.push(drift_sample(simulation, Some(substep)));
            }
        }
        result
    }
    fn observe_metrics(&mut self, simulation: &Simulation, substep: u32) -> Result<(), Failure> {
        if simulation.tick < SETTLE_TICKS {
            self.settling_substeps += 1;
            return Ok(());
        }
        let reference = self.settled.as_ref().ok_or_else(|| {
            Failure::new(
                simulation.tick,
                substep,
                "quiet-reference",
                "missing settled observation".into(),
                0.0,
                1.0,
            )
        })?;
        self.quiet_substeps += 1;
        self.metrics.observe(simulation, reference, substep)
    }
    pub fn completed_tick(&mut self, simulation: &Simulation) {
        if simulation.tick == SETTLE_TICKS {
            self.settled = Some(simulation.snapshot());
        }
    }
    pub fn progress(&self, simulation: &Simulation) -> Value {
        let passed = simulation.first_failure.is_none()
            && simulation.tick == END_TICK
            && self.settling_substeps == 480
            && self.quiet_substeps == 7200
            && self.settled.is_some();
        json!({"completedTicks": simulation.tick, "settlingSubsteps": self.settling_substeps,
            "quietSubsteps": self.quiet_substeps, "done": simulation.tick == END_TICK || simulation.first_failure.is_some(),
            "passed": passed, "firstFailure": simulation.first_failure})
    }
    pub fn report(&self, simulation: &Simulation) -> Value {
        let mut report = json!({"schema": 1, "scope": "Worker-runtime quiet standing only; no other gate or release admission",
            "heading": self.heading, "profile": simulation.profile, "initial": self.initial,
            "settled": self.settled, "final": simulation.snapshot(),
            "progress": self.progress(simulation), "measurements": {"massKg": simulation.total_mass(), "quiet": self.metrics},
            "releaseAccepted": false});
        if let Some(trace) = &self.drift_trace {
            report["driftTrace"] = json!({"scope": "Read-only once-per-second state and selected-solver-point diagnostic; targets are pre-solve intent, not delivered effort", "samples": trace});
        }
        report
    }
}

fn drift_sample(s: &Simulation, substep: Option<u32>) -> Value {
    use rapier3d::prelude::SolverFlags;
    let legs: Vec<_> = s.model.segments.iter().enumerate().filter_map(|(i, segment)| {
        if !matches!(segment.role.as_str(), "thigh" | "shin" | "ankle" | "hindfoot" | "forefoot") {
            return None;
        }
        let profile = segment.joint_profile.as_ref()?;
        let joint = s.world.impulse_joints.get(s.joints[i]?)?;
        let parent = s.world.bodies.get(joint.body1())?;
        Some(json!({"id": segment.id, "coordinateRadians": crate::measurement::coordinates(*parent.rotation(), *s.world.bodies[s.bodies[i]].rotation(), profile), "preSolveTargetsRadians": s.targets[i]}))
    }).collect();
    let contacts: Vec<_> = s.model.segments.iter().enumerate().filter(|(_, segment)| matches!(segment.role.as_str(), "hindfoot" | "forefoot")).map(|(i, segment)| {
        let mut points = Vec::new();
        if let Some(pair) = s.world.narrow_phase.contact_pair(s.floor, s.colliders[i]) {
            for manifold in pair.solver_manifolds() {
                if !manifold.data.solver_flags.contains(SolverFlags::COMPUTE_IMPULSES) { continue; }
                for contact in &manifold.data.solver_contacts {
                    let index = contact.contact_indices()[0];
                    if let Some(point) = manifold.points.get(index as usize) {
                        let normal = point.data.impulse;
                        let tangent = point.data.tangent_impulse.norm();
                        let cap = normal * manifold.data.friction;
                        let utilization = (cap > 0.0).then_some(tangent / cap);
                        points.push(json!({"normalImpulseNs": normal, "rawReportedTangentImpulseMagnitudeNs": tangent, "friction": manifold.data.friction, "rawReportedFrictionUtilization": utilization}));
                    }
                }
            }
        }
        let tangent = match crate::measurement::floor_tangent_impulse(&s.world, s.floor, s.colliders[i]) {
            Ok(impulse) => json!({"worldImpulseOnFootNs": crate::vec3(impulse)}),
            Err(error) => json!({"measurementError": error}),
        };
        json!({"id": segment.id, "selectedSolverPoints": points, "manifoldTangentDiagnostic": tangent})
    }).collect();
    let snapshot = s.snapshot();
    let bodies: Vec<_> = snapshot
        .segments
        .iter()
        .filter(|segment| {
            matches!(
                segment.id.as_str(),
                "pelvis" | "torso" | "leftFoot" | "leftForefoot" | "rightFoot" | "rightForefoot"
            )
        })
        .collect();
    json!({"tickStart": s.tick, "substep": substep, "completedSubsteps": s.substeps_completed,
        "simulationTimeS": snapshot.simulation_time_s, "com": crate::vec3(s.center_of_mass()),
        "referenceCom": crate::vec3(s.rest_com), "pelvisInHeading": crate::quat(s.heading.inverse() * *s.world.bodies[s.bodies[0]].rotation()),
        "bodies": bodies, "legs": legs, "floorContacts": contacts,
        "tangentReporting": {"solverModel": format!("{:?}", s.world.integration_parameters.friction_model), "admitted": false,
            "reason": "The world-space total passes bounded isolated linear-box calibration. Angular, canonical-foot and generic-multibody tangent reporting remain unqualified, and high-speed stress calibration fails. Simplified per-point zeros do not establish absent friction; diagnostics do not feed control."}})
}
