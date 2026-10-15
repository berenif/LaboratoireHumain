//! One frozen experimental feedback candidate. No learned dynamics or support proof.
use super::{runtime, tasks, ReferenceTasks};
use crate::{Simulation, recovery::trajectory_program::{Command, CompiledProgram}};
use crate::recovery::planner::{native_observation, support_state::{SupportObservation, MaterialTracker, ContactMode, SegmentRole}};
use lh_model::Model;
use rapier3d::prelude::*;
use serde_json::{Value,json};

#[derive(Clone, Default)]
pub(super) struct Feedback { queue: Vec<Command>, start: u64, pub evidence: Value }

fn reference_step(at:u64)->u32 {
    let elapsed=at.saturating_sub(36);
    // Brake and recenter at the A36 posture for two seconds; then replay the
    // complete remaining reference while retaining the same feedback.
    if elapsed<480 {36} else {(36+elapsed-480).min(1920) as u32}
}
fn hull_center(points:&[[f64;2]])->Option<[f64;2]> {
    if points.is_empty() {return None;}
    let mut p=points.to_vec();p.sort_by(|a,b|a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));p.dedup();
    let cross=|a:[f64;2],b:[f64;2],c:[f64;2]|(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);
    let mut hull:Vec<[f64;2]>=vec![];
    for &v in &p {while hull.len()>=2 && cross(hull[hull.len()-2],hull[hull.len()-1],v)<=0. {hull.pop();}hull.push(v);}
    let lower=hull.len();
    for &v in p.iter().rev().skip(1) {while hull.len()>lower && cross(hull[hull.len()-2],hull[hull.len()-1],v)<=0. {hull.pop();}hull.push(v);}
    if hull.len()>1 {hull.pop();}
    Some([hull.iter().map(|p|p[0]).sum::<f64>()/hull.len() as f64,hull.iter().map(|p|p[1]).sum::<f64>()/hull.len() as f64])
}
fn transfer_velocity(com:[f64;3],velocity:[f64;3],floor:f64,center:[f64;2],height:f64)->[f64;3] {
    let omega=(9.81/(com[1]-floor).max(0.2)).sqrt();
    let v=[omega*(center[0]-com[0])-velocity[0],2.*(height-com[1])-velocity[1],omega*(center[1]-com[2])-velocity[2]];
    let norm=v.iter().map(|x|x*x).sum::<f64>().sqrt(); let scale=(0.15/norm.max(0.15)).min(1.);
    v.map(|x|x*scale)
}

fn prepare(model:&Model, observed:&tasks::ObservedState, support:&SupportObservation,
    program:&CompiledProgram, reference:&ReferenceTasks, memory:&runtime::Memory, at:u64)
    ->Result<(Vec<Command>,Value),String> {
    support.validate()?;
    if support.native_substep!=at || observed.native_substep!=at || at!=memory.next_substep || at%4!=0 {
        return Err("Feedback observation/command clock mismatch".into());
    }
    let mut current=MaterialTracker::default();let mut points=vec![];
    for s in &support.segments {
        if !matches!(s.role,SegmentRole::Hindfoot|SegmentRole::Forefoot) {continue;}
        for c in s.contacts.iter().filter(|c|c.current_upward_load_n>=3.) {points.push([c.world_point_m[0],c.world_point_m[2]]);}
        if let Some((index,_))=s.contacts.iter().enumerate().filter(|(_,c)|c.current_upward_load_n>=3.)
            .max_by(|(_,a),(_,b)|a.current_upward_load_n.total_cmp(&b.current_upward_load_n)) {
            // Fresh measured contact, used for one decision only. No persistent anchor gate.
            current.acquire(support,s.segment,index,ContactMode::Hold)?;
        }
    }
    let center=hull_center(&points);
    let source=reference_step(at);let row=&reference.rows[(source/4) as usize];
    let desired=transfer_velocity(support.center_of_mass_m,support.center_of_mass_velocity_mps,
        support.support_height_m,center.unwrap_or([support.center_of_mass_m[0],support.center_of_mass_m[2]]),row.com_position_m[1]);
    let mut nominal=program.command(source);let scale=if at<516 {0.}else{1.};
    for i in 0..25 {for k in 0..3 {nominal.velocities[i][k]=(nominal.velocities[i][k]*scale+2.*(nominal.positions[i][k]-memory.previous[i][k])).clamp(-1.,1.);}}
    let mut goals=tasks::TaskGoals {nominal,com_velocity_mps:desired,com_weight:1.,point_tasks:vec![],orientation_tasks:vec![],posture_weight:0.25,regularization:0.05,retained_velocity_tolerance_mps:0.005};
    // Trunk braking and foot flattening accompany the same CoM task; all are soft
    // intents under measured base motion, not an ideal torque/force controller.
    for segment in [1,2,18,19,23,24] {
        // Canonical heading-zero A36 scene: flat feet and upright trunk.
        let target=Rotation::IDENTITY;
        let mut delta=target*observed.bodies[segment].rotation().inverse();if delta.w<0. {delta=-delta;}
        let angular=delta.to_scaled_axis()*2.-Vector::from_array(observed.bodies[segment].angular_velocity_radps);
        goals.orientation_tasks.push(tasks::OrientationGoal {segment,world_angular_velocity_radps:angular.to_array().map(f64::from),weight:1.});
    }
    let mut problem=tasks::assemble_exploratory_observed(model,observed,&current.active,&goals)?;
    for (i,a) in runtime::axes(model)?.iter().enumerate() {
        let q=f64::from(memory.previous[a.segment][a.coordinate]);let dt=4.*f64::from(lh_contracts::SUBSTEP_S);
        problem.lower[i]=problem.lower[i].max((f64::from(a.minimum)-q)/dt);
        problem.upper[i]=problem.upper[i].min((f64::from(a.maximum)-q)/dt);
    }
    let (_,solution,representation)=runtime::compile_problem(model,memory,at,&problem)?;
    let evidence=json!({"candidate":"loaded-hull-capture-and-trunk-feedback-v1","decisionSubstep":at,
        "phase":if at<516 {"transfer"}else if source<432 {"reference-recentering"}else if source<1680 {"extension"}else{"standing-reference-hold"},
        "referenceSubstep":source,"loadedHullCenterXZ":center,"currentContactGuides":current.active.len(),
        "com":support.center_of_mass_m,"comVelocityMps":support.center_of_mass_velocity_mps,"desiredComVelocityMps":desired,
        "solverIterations":solution.iterations,"constraintError":solution.maximum_constraint_error,
        "physicalAdmission":false,"forceBasedResponseIdentified":false});
    Ok((representation.commands,evidence))
}
impl Feedback {
    pub fn command(&mut self,sim:&Simulation,program:&CompiledProgram,reference:&ReferenceTasks,memory:&runtime::Memory,at:u64)->Result<Command,String> {
        if at%4==0 {
            let support=native_observation::observe(sim,0,&[])?;let observed=tasks::ObservedState::from_simulation(sim)?;
            let (queue,evidence)=prepare(&sim.model,&observed,&support,program,reference,memory,at)?;
            self.queue=queue;self.start=at;self.evidence=evidence;
        }
        let offset=at.checked_sub(self.start).ok_or("Feedback queue rewind")? as usize;
        let command=self.queue.get(offset).ok_or("Feedback queue exhausted")?.clone();
        crate::recovery::trajectory_program::validate_command(&sim.model,&command)?;
        for i in 0..25 {for k in 0..3 {if command.velocities[i][k].to_bits()!=((command.positions[i][k]-memory.previous[i][k])/lh_contracts::SUBSTEP_S).to_bits() {return Err("Feedback velocity history mismatch".into());}}}
        Ok(command)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn pure_feedback_direction_hull_and_finite_schedule() {
        assert_eq!(hull_center(&[[0.,0.],[2.,0.],[0.,2.],[0.2,0.2]]),Some([2./3.,2./3.]));
        assert_eq!(hull_center(&[]),None);
        let v=transfer_velocity([0.,1.,0.04],[0.,0.,0.1],0.,[0.,-0.01],1.);
        assert!(v[2]<0. && v.iter().map(|x|x*x).sum::<f64>().sqrt()<=0.15000000001);
        assert_eq!(reference_step(36),36);assert_eq!(reference_step(515),36);assert_eq!(reference_step(516),36);assert_eq!(reference_step(2160),1680);assert_eq!(reference_step(2400),1920);assert_eq!(reference_step(6239),1920);
    }
    #[test]
    fn pure_feedback_archived_a36_commands_and_contact_free_case() {
        let root=std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../..");
        let read=|p:&str|std::fs::read(root.join(p)).unwrap();
        let model:Model=serde_json::from_slice(&read("rust/crates/model/data/canonical-v1.json")).unwrap();
        let entry:Value=serde_json::from_slice(&read("docs/evidence/coordination-transfer-candidate-2026-10-13/observed-entry-step36.json")).unwrap();
        let state:Value=serde_json::from_slice(&read("docs/evidence/coordination-transfer-candidate-2026-10-13/observed-entry-task-state.json")).unwrap();
        let observed:tasks::ObservedState=serde_json::from_value(state["observed_task_state"].clone()).unwrap();
        let support:SupportObservation=serde_json::from_value(entry["archivedNativeRow"]["support"].clone()).unwrap();
        let program:crate::recovery::trajectory_program::Program=serde_json::from_slice(&read("docs/evidence/coordination-prototype-2026-10-12/continuation-candidate-program.json")).unwrap();
        let program=program.compile(&model).unwrap();
        let reference=serde_json::from_slice(&read("docs/evidence/coordination-prototype-2026-10-12/continuation-tasks.json")).unwrap();
        let q=serde_json::from_value(entry["commandEntry"]["previous_q"].clone()).unwrap();
        let memory=runtime::initialize(&model,q,36,0).unwrap();
        for observation in [support.clone(), {let mut o=support;for s in &mut o.segments {s.contacts.clear();s.persistent_load_n=0.;s.loaded_substeps=0;}o}] {
            let (commands,evidence)=prepare(&model,&observed,&observation,&program,&reference,&memory,36).unwrap();
            assert_eq!(commands.len(),4);assert_eq!(evidence["physicalAdmission"],false);
            let mut previous=memory.previous;
            for c in commands {crate::recovery::trajectory_program::validate_command(&model,&c).unwrap();for i in 0..25 {for k in 0..3 {assert_eq!(c.velocities[i][k].to_bits(),((c.positions[i][k]-previous[i][k])/lh_contracts::SUBSTEP_S).to_bits());}}previous=c.positions;}
        }
    }
}
