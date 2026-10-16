//! Three fixed initial orientations; construction only, followed by real settling.
use crate::{Simulation, getup, measurement};
use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct State {
    pub kind: String,
    pub preparing: bool,
    pub ready: bool,
    pub settled_substeps: u32,
    pub trigger_substep: Option<u64>,
    pub maximum_body_com_speed_mps: f32,
    pub maximum_angular_radps: f32,
    pub ground_load_n: f32,
    pub torso_long_axis_up: f32,
    pub torso_face_up: f32,
    pub torso_side_up: f32,
    pub head_height_m: f32,
}

fn displace_root(s: &mut Simulation, translation: Vector, angular: Vector) -> Result<(), String> {
    let id = *s.world.multibody_joints.rigid_body_link(s.bodies[0]).ok_or("Missing free root")?;
    let mb = s.world.multibody_joints.get_multibody_mut(id.multibody).ok_or("Missing articulation")?;
    if mb.ndofs() != 27 { return Err("Fixture requires the unchanged GetUp articulation".into()); }
    let mut displacement = vec![0.0; mb.ndofs()];
    displacement[..3].copy_from_slice(&translation.to_array());
    displacement[3..6].copy_from_slice(&angular.to_array());
    mb.apply_displacements(&displacement);
    mb.forward_kinematics(&s.world.bodies, false);
    mb.update_rigid_bodies(&mut s.world.bodies, true);
    s.world.bodies.propagate_modified_body_positions_to_colliders(&mut s.world.colliders);
    Ok(())
}

pub fn initialize(s: &mut Simulation, kind: &str) -> Result<(), String> {
    if s.substeps_completed != 0 || s.tick != 0 || s.getup_fixture.is_some()
        || s.getup_controller.is_none() || s.environment.is_some() || s.grab.is_some()
    { return Err("Fixture construction is allowed once, before the first integration".into()); }
    let angular = match kind {
        "back" => Vector::X * -std::f32::consts::FRAC_PI_2,
        "belly" => Vector::X * std::f32::consts::FRAC_PI_2,
        "side" => Vector::Z * std::f32::consts::FRAC_PI_2,
        _ => return Err("Unknown fixed GetUp fixture".into()),
    };
    // All three cases retain the exact same21 joint coordinates. Only the free
    // root orientation and initial ground clearance differ by fixture definition.
    displace_root(s, Vector::ZERO, angular)?;
    let bottom = s.colliders.iter().filter(|h| s.world.colliders[**h].is_enabled())
        .map(|h| s.world.colliders[*h].compute_aabb().mins.y).fold(f32::INFINITY, f32::min);
    displace_root(s, Vector::Y * (0.002-bottom), Vector::ZERO)?;
    for h in &s.bodies {
        s.world.bodies[*h].reset_forces(false);
        s.world.bodies[*h].reset_torques(false);
    }
    s.getup_fixture = Some(State { kind:kind.into(),preparing:true,ready:false,settled_substeps:0,
        trigger_substep:None,maximum_body_com_speed_mps:0.0,maximum_angular_radps:0.0,
        ground_load_n:0.0,torso_long_axis_up:0.0,torso_face_up:0.0,torso_side_up:0.0,head_height_m:0.0 });
    observe(s, &[0.0;25]);
    measurement::inspect(s,0).map_err(|e|format!("Fixture geometry integrity: {e:?}"))?;
    getup::configure(s)
}

pub fn observe(s: &mut Simulation, loads: &[f32;25]) {
    let Some(state) = s.getup_fixture.as_ref() else { return; };
    if !state.preparing { return; }
    let kind = state.kind.clone();
    let Ok(twists) = getup::current_twists(s) else { return; };
    let max_v = twists.iter().map(|(v,_)|v.length()).fold(0.0_f32,f32::max);
    let max_w = twists.iter().map(|(_,w)|w.length()).fold(0.0_f32,f32::max);
    let torso = s.model.segments.iter().position(|p|p.id=="torso").unwrap();
    let head = s.model.segments.iter().position(|p|p.id=="head").unwrap();
    let rotation = s.world.bodies[s.bodies[torso]].rotation().normalize();
    let long = (rotation*Vector::Y).y;
    let face = (rotation*Vector::Z).y;
    let side = (rotation*Vector::X).y;
    let head_y = s.world.bodies[s.bodies[head]].translation().y;
    let load: f32 = loads.iter().sum();
    let orientation = long.abs()<=0.35 && match kind.as_str() {
        "back" => face>=0.8,
        "belly" => face<=-0.8,
        "side" => face.abs()<=0.35 && side>=0.8,
        _ => false,
    };
    let settled = orientation && head_y<=0.5 && load>=0.8*s.total_mass()*9.81
        && max_v<=0.22 && max_w<=0.65;
    let state = s.getup_fixture.as_mut().unwrap();
    state.settled_substeps = if settled { state.settled_substeps+1 } else {0};
    state.ready = state.settled_substeps>=120;
    state.maximum_body_com_speed_mps=max_v;state.maximum_angular_radps=max_w;state.ground_load_n=load;
    state.torso_long_axis_up=long;state.torso_face_up=face;state.torso_side_up=side;state.head_height_m=head_y;
}

pub fn trigger(s: &mut Simulation) -> Result<(), String> {
    let state=s.getup_fixture.as_ref().ok_or("No GetUp fixture")?;
    if !state.preparing || !state.ready { return Err("Fixture must be physically settled in its declared orientation".into()); }
    s.getup_controller.as_mut().ok_or("Missing learned controller")?.fixture_trigger(s.substeps_completed)?;
    let state=s.getup_fixture.as_mut().unwrap();state.preparing=false;state.trigger_substep=Some(s.substeps_completed);
    s.motion=lh_contracts::Motion::Fallen;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn fixed_orientations_preserve_joint_coordinates_mass_and_integrity_without_stepping() {
        for kind in ["back","belly","side"] {
            let mut profile=crate::Profile::default();profile.getup_policy_enabled=true;
            let mut s=Simulation::new_protocol(0.0,profile,true).unwrap();
            let (q,dq)=getup::joint_state(&s).unwrap();let mass=s.total_mass();
            initialize(&mut s,kind).unwrap();let (after,velocity)=getup::joint_state(&s).unwrap();
            for i in 0..21 { assert!((q[i]-after[i]).abs()<1e-5);assert_eq!(dq[i],velocity[i]); }
            assert_eq!(mass,s.total_mass());assert_eq!(s.bodies.len(),25);assert_eq!(s.substeps_completed,0);
            assert!(s.diagnostics.getup.as_ref().unwrap().motor_controls.iter().all(|x|*x==0.0));
            let f=s.getup_fixture.as_ref().unwrap();assert!(!f.ready);
            assert!(f.torso_long_axis_up.abs()<1e-5);
            if kind=="back" { assert!(f.torso_face_up>0.99); }
            if kind=="belly" { assert!(f.torso_face_up< -0.99); }
            if kind=="side" { assert!(f.torso_side_up>0.99); }
            assert!(trigger(&mut s).is_err());assert!(initialize(&mut s,kind).is_err());
            let runtime=crate::runtime::Runtime::new_getup_fixture(kind).unwrap();
            let bytes=runtime.getup_checkpoint().unwrap();
            let restored=crate::runtime::Runtime::from_getup_checkpoint(&bytes).unwrap();
            assert_eq!(runtime.getup_fixture_report().unwrap(),restored.getup_fixture_report().unwrap());
            assert!(runtime.verify_getup_checkpoint(&bytes).is_ok());
        }
    }
}
