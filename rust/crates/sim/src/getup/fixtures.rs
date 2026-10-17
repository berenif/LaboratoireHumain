//! Fixed initial orientations; construction only, followed by real ground contact.
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

// Frozen initial posture, selected by geometry only; no physical tuning runs.
// Flexed legs and a forward lower arm spread natural contacts. Shoulder geometry
// keeps the arm near torso height instead of propping the body above a collapsing
// upper arm. The same 21 hinges are used; no support is imposed after construction.
pub const SIDE_SUPPORTED_Q: [(usize,f32);7] = [(5,-0.65),(6,-1.1),(11,-0.9),(12,-1.45),
    (15,0.616),(16,0.217),(17,-1.5)];
fn set_supported_side_joints(s: &mut Simulation) -> Result<(),String> {
    let (q,_) = getup::joint_state(s)?;
    let id = *s.world.multibody_joints.rigid_body_link(s.bodies[0]).ok_or("Missing free root")?;
    let mb = s.world.multibody_joints.get_multibody_mut(id.multibody).ok_or("Missing articulation")?;
    let mut displacement = vec![0.0;mb.ndofs()];
    for (i,part) in s.model.segments.iter().enumerate() {
        if let Some(h) = part.reference.as_ref().and_then(|r|r.hinge.as_ref()) {
            if let Some((_,target)) = SIDE_SUPPORTED_Q.iter().find(|(j,_)|*j==h.qpos_index) {
                let axis = &part.joint_profile.as_ref().unwrap().axes[0];
                if *target<axis.min_radians || *target>axis.max_radians {return Err("Invalid fixed side joint".into());}
                let link = mb.links().find(|l|l.rigid_body_handle()==s.bodies[i]).ok_or("Missing hinge")?;
                displacement[link.assembly_id()]=*target-q[h.qpos_index];
            }
        }
    }
    mb.apply_displacements(&displacement);mb.forward_kinematics(&s.world.bodies,false);
    mb.update_rigid_bodies(&mut s.world.bodies,true);
    s.world.bodies.propagate_modified_body_positions_to_colliders(&mut s.world.colliders);
    Ok(())
}

fn support_bottom(s: &Simulation) -> f32 {
    s.model.segments.iter().enumerate().flat_map(|(i,p)| {
        let body=&s.world.bodies[s.bodies[i]];
        p.reference.as_ref().unwrap().primitives.iter().flat_map(move |primitive| {
            [primitive.a,primitive.b].map(|endpoint|
                (body.rotation()*crate::vector(endpoint)+body.translation()).y-primitive.radius)
        })
    }).fold(f32::INFINITY,f32::min)
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
        "side" | "side-supported" => Vector::Z * std::f32::consts::FRAC_PI_2,
        _ => return Err("Unknown fixed GetUp fixture".into()),
    };
    if kind=="side-supported" {set_supported_side_joints(s)?;}
    // The original three cases retain the exact same21 joint coordinates. Only the free
    // root orientation and initial ground clearance differ by fixture definition.
    displace_root(s, Vector::ZERO, angular)?;
    // Rotating a compound's local AABB overestimates its downward extent. For
    // the corrected lateral fixture use the actual sphere/capsule endpoints,
    // avoiding an unintended drop before the intended limb contacts the floor.
    let bottom = if kind=="side-supported" { support_bottom(s) } else {
        s.colliders.iter().filter(|h| s.world.colliders[**h].is_enabled())
            .map(|h| s.world.colliders[*h].compute_aabb().mins.y).fold(f32::INFINITY, f32::min)
    };
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
        "side" | "side-supported" => face.abs()<=0.35 && side>=0.8,
        _ => false,
    };
    let grounded = orientation && head_y<=0.5 && load>=0.8*s.total_mass()*9.81;
    let state = s.getup_fixture.as_mut().unwrap();
    (state.settled_substeps, state.ready) = admission_progress(
        &kind, grounded, max_v, max_w, state.settled_substeps);
    state.maximum_body_com_speed_mps=max_v;state.maximum_angular_radps=max_w;state.ground_load_n=load;
    state.torso_long_axis_up=long;state.torso_face_up=face;state.torso_side_up=side;state.head_height_m=head_y;
}

// Lateral entry is a grounded starting condition, not a passive equilibrium.
// Waiting for near-zero speed lets a valid side pose roll prone before control.
// The orientation/actual-load gates are unchanged; 20 native samples are 25 ms.
fn admission_progress(kind: &str, grounded: bool, speed: f32, angular: f32, prior: u32) -> (u32, bool) {
    let lateral = matches!(kind, "side" | "side-supported");
    let admitted = grounded && speed.is_finite() && angular.is_finite()
        && (lateral || (speed<=0.22 && angular<=0.65));
    let count = if admitted { prior+1 } else { 0 };
    (count, count>=if lateral { 20 } else { 120 })
}

pub fn trigger(s: &mut Simulation) -> Result<(), String> {
    let state=s.getup_fixture.as_ref().ok_or("No GetUp fixture")?;
    if !state.preparing || !state.ready { return Err("Fixture must be physically grounded and ready in its declared orientation".into()); }
    s.getup_controller.as_mut().ok_or("Missing learned controller")?.fixture_trigger(s.substeps_completed)?;
    let state=s.getup_fixture.as_mut().unwrap();state.preparing=false;state.trigger_substep=Some(s.substeps_completed);
    s.motion=lh_contracts::Motion::Fallen;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn lateral_trigger_changes_only_lifecycle_without_reset_or_support() {
        let mut profile=crate::Profile::default();profile.getup_policy_enabled=true;
        let mut s=Simulation::new_protocol(0.0,profile,true).unwrap();
        initialize(&mut s,"side-supported").unwrap();
        // Isolate trigger semantics; this synthetic readiness is not a physical trial.
        s.getup_fixture.as_mut().unwrap().ready=true;
        let before=s.snapshot().segments;
        let joints=getup::joint_state(&s).unwrap();
        let articulation= s.world.multibody_joints.iter().count();
        let impulses=s.world.impulse_joints.len();
        trigger(&mut s).unwrap();
        assert_eq!(serde_json::to_value(before).unwrap(),serde_json::to_value(s.snapshot().segments).unwrap());
        assert_eq!(joints,getup::joint_state(&s).unwrap());
        assert_eq!(articulation,s.world.multibody_joints.iter().count());
        assert_eq!(impulses,s.world.impulse_joints.len());
        assert!(!s.getup_fixture.as_ref().unwrap().preparing);
        let saved=serde_json::to_value(&s.getup_fixture).unwrap();
        observe(&mut s,&[0.0;25]);
        assert_eq!(saved,serde_json::to_value(&s.getup_fixture).unwrap());
        assert!(trigger(&mut s).is_err());
    }

    #[test]
    fn lateral_admission_requires_persistent_grounding_but_allows_motion() {
        for kind in ["side", "side-supported"] {
            assert_eq!(admission_progress(kind, true, 0.7, 1.2, 18), (19, false));
            assert_eq!(admission_progress(kind, true, 0.7, 1.2, 19), (20, true));
            // Grounding includes the unchanged lateral orientation and actual-load gates.
            assert_eq!(admission_progress(kind, false, 0.0, 0.0, 40), (0, false));
            assert_eq!(admission_progress(kind, true, f32::NAN, 0.0, 40), (0, false));
            assert_eq!(admission_progress(kind, true, 0.0, f32::INFINITY, 40), (0, false));
        }
        for kind in ["back", "belly"] {
            assert_eq!(admission_progress(kind, true, 0.7, 1.2, 119), (0, false));
            assert_eq!(admission_progress(kind, true, 0.22, 0.65, 118), (119, false));
            assert_eq!(admission_progress(kind, true, 0.22, 0.65, 119), (120, true));
        }
    }

    #[test]
    fn fixed_orientations_preserve_joint_coordinates_mass_and_integrity_without_stepping() {
        for kind in ["back","belly","side","side-supported"] {
            let mut profile=crate::Profile::default();profile.getup_policy_enabled=true;
            let mut s=Simulation::new_protocol(0.0,profile,true).unwrap();
            let (q,dq)=getup::joint_state(&s).unwrap();let mass=s.total_mass();
            initialize(&mut s,kind).unwrap();let (after,velocity)=getup::joint_state(&s).unwrap();
            for i in 0..21 {
                let expected = if kind=="side-supported" {SIDE_SUPPORTED_Q.iter().find(|(j,_)|*j==i).map(|(_,v)|*v).unwrap_or(q[i])} else {q[i]};
                assert!((expected-after[i]).abs()<1e-5);assert_eq!(dq[i],velocity[i]);
            }
            if kind=="side-supported" {
                assert!((support_bottom(&s)-0.002).abs()<1e-5);
                let pelvis=s.model.segments.iter().position(|p|p.id=="pelvis").unwrap();
                // Pelvis is already near its resting side height, not hanging
                // more than 20 cm above the ground on an outstretched arm.
                assert!(s.world.bodies[s.bodies[pelvis]].translation().y<0.23);
            }
            let before = serde_json::to_value(s.snapshot()).unwrap();
            let diagnostic=getup::controller::standing_diagnostic(&s,None).unwrap();
            assert_eq!(diagnostic["physicsIntegrations"],0);
            assert_eq!(before,serde_json::to_value(s.snapshot()).unwrap());
            assert_eq!(mass,s.total_mass());assert_eq!(s.bodies.len(),25);assert_eq!(s.substeps_completed,0);
            assert!(s.diagnostics.getup.as_ref().unwrap().motor_controls.iter().all(|x|*x==0.0));
            let f=s.getup_fixture.as_ref().unwrap();assert!(!f.ready);
            assert!(f.torso_long_axis_up.abs()<1e-5);
            if kind=="back" { assert!(f.torso_face_up>0.99); }
            if kind=="belly" { assert!(f.torso_face_up< -0.99); }
            if kind.starts_with("side") { assert!(f.torso_side_up>0.99); }
            assert!(trigger(&mut s).is_err());assert!(initialize(&mut s,kind).is_err());
            let runtime=crate::runtime::Runtime::new_getup_fixture(kind).unwrap();
            let bytes=runtime.getup_checkpoint().unwrap();
            let restored=crate::runtime::Runtime::from_getup_checkpoint(&bytes).unwrap();
            assert_eq!(runtime.getup_fixture_report().unwrap(),restored.getup_fixture_report().unwrap());
            assert!(runtime.verify_getup_checkpoint(&bytes).is_ok());
        }
    }
}
