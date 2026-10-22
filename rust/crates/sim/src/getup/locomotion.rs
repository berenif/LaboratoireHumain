//! Default-off native GetUp observation and single motor-owner contract.
//! This layer never edits poses/velocities or applies a root support wrench.
//! A passive owner removes active motors, not the model's physical springs.
use crate::*;

pub const FORMAT: &str = "getup-native-owner-v1";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Profile {
    #[serde(rename = "native-joint-owner-v1")]
    NativeJointOwnerV1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Mode { Passive, Active }

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Owner { FixturePreparation, Passive, Balance, Recovery, Step }

#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Application {
    pub native_input: u64,
    pub owner: Owner,
    pub controls: [f32; 21],
}

#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct State {
    format: String,
    generation: u64,
    pub mode: Mode,
    activation_native: Option<u64>,
    pub last_application: Option<Application>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    step: Option<super::native_step::State>,
}

impl State {
    pub fn new(generation: u64) -> Self {
        Self { format: FORMAT.into(), generation, mode: Mode::Active,
            activation_native: None, last_application: None, step: None }
    }
    pub(crate) fn set_initial_generation(&mut self,generation:u64) {self.generation=generation;}
    fn valid(&self, s: &Simulation) -> bool {
        self.format == FORMAT && self.generation == s.generation
            && self.activation_native.is_none_or(|n| n <= s.substeps_completed)
            && self.last_application.as_ref().is_none_or(|a|
                a.native_input <= s.substeps_completed
                && (a.owner != Owner::Step || self.step.is_some())
                && a.controls.iter().all(|v| v.is_finite() && v.abs() <= 1.0))
            && self.step.as_ref().is_none_or(|step|step.valid(s.substeps_completed))
    }
}


pub(crate) fn step_active(s:&Simulation)->bool {
    s.native_locomotion.as_ref().is_some_and(|state|state.step.as_ref().is_some_and(|x|x.active()))
}
#[cfg(feature="getup-native-locomotion-experiment")]
pub(crate) fn begin_step(s:&mut Simulation,side:usize)->Result<(),String> {
    validate(s)?;
    let state=s.native_locomotion.as_ref().ok_or("Native Step requires the explicit pilot profile")?;
    if state.mode!=Mode::Active || step_active(s) || s.first_failure.is_some() || s.motion==Motion::Halted
        || s.getup_fixture.as_ref().is_some_and(|f|f.preparing)
        || !s.getup_controller.as_ref().is_some_and(|c|c.native_balance_ready(s.substeps_completed)
            && !c.native_recovery_owner(s.motion,s.substeps_completed))
        || state.activation_native.is_some_and(|n|s.substeps_completed.saturating_sub(n)<20)
    {return Err("Step needs earned recovered balance and exclusive active motor ownership".into());}
    let mut step=super::native_step::State::begin(s,side)?;
    step.command(s)?; // Read-only algebraic admission, no integration or motor application.
    s.native_locomotion.as_mut().unwrap().step=Some(step);
    Ok(())
}
pub(crate) fn cancel_step_for_recovery(s:&mut Simulation) {
    let recovery=s.getup_fixture.as_ref().is_some_and(|f|f.preparing)
        || s.getup_controller.as_ref().is_some_and(|c|c.native_recovery_owner(s.motion,s.substeps_completed));
    if recovery && let Some(step)=s.native_locomotion.as_mut().and_then(|x|x.step.as_mut()) {step.cancel();}
}
pub(crate) fn step_command(s:&mut Simulation)->Result<[f32;21],String> {
    // Evaluate a clone of the controller state, never a clone of the world.
    // All state transitions are committed atomically with a feasible command.
    let mut step=s.native_locomotion.as_ref().and_then(|x|x.step.clone()).ok_or("Missing Step owner")?;
    let controls=step.command(s)?;
    s.native_locomotion.as_mut().unwrap().step=Some(step);
    Ok(controls)
}

/// Receipt of the REAL bounded external user impulse, not a pointer intention.
/// At command input n it describes interval n -> n+1. After integration it
/// remains explicitly stamped n; it is not relabelled as the next interval.
#[derive(Clone, Serialize)]
pub(crate) struct GrabApplication {
    pub native_input: u64,
    pub press: u64,
    pub body: usize,
    pub anchor_world: [f32; 3],
    pub impulse_ns: [f32; 3],
    pub force_n: [f32; 3],
    pub torque_about_body_com_nm: [f32; 3],
}

pub(crate) fn validate(s: &Simulation) -> Result<(), String> {
    if s.profile.getup_native_locomotion.is_some() != s.native_locomotion.is_some() {
        return Err("Native owner profile/state mismatch".into());
    }
    if let Some(state) = &s.native_locomotion {
        if !cfg!(feature = "getup-native-locomotion-experiment") || !state.valid(s) {
            return Err("Unsupported/invalid native owner state".into());
        }
        #[cfg(feature = "stationary-balance-experiment")]
        if s.balance_residual.is_some() || s.balance_torso_force_n != [0.0; 2] {
            return Err("Native owner cannot coexist with stationary external override".into());
        }
    }
    Ok(())
}

pub(crate) fn passive(s: &Simulation) -> bool {
    s.native_locomotion.as_ref().is_some_and(|x| x.mode == Mode::Passive)
}

/// No command injection: only choose whether the existing joint controllers
/// run. Active re-entry ramps the live controller's command from zero over
/// 20 native samples. Recovery/fixture ownership cannot be interrupted.
pub(crate) fn set_mode(s: &mut Simulation, mode: Mode) -> Result<(), String> {
    validate(s)?;
    if s.first_failure.is_some() || s.motion == Motion::Halted {
        return Err("Cannot change owner in a failed world".into());
    }
    let old = s.native_locomotion.as_ref().ok_or("Native owner is not enabled")?.mode;
    if old == mode { return Ok(()); }
    if s.getup_fixture.as_ref().is_some_and(|f| f.preparing)
        || (mode == Mode::Passive && s.getup_controller.as_ref()
            .is_some_and(|c| c.native_recovery_owner(s.motion,s.substeps_completed)))
    { return Err("Cannot interrupt fixture preparation or incomplete recovery".into()); }
    if step_active(s) {return Err("Step pilot owns motors until recovery or explicit trial stop".into());}
    let state = s.native_locomotion.as_mut().unwrap();
    state.mode = mode;
    state.activation_native = (mode == Mode::Active).then_some(s.substeps_completed);
    Ok(())
}

pub(crate) fn finish_command(s: &mut Simulation, owner: Owner, mut controls: [f32; 21])
    -> Result<[f32; 21], String>
{
    if let Some(state) = &s.native_locomotion {
        if owner == Owner::Step && !step_active(s) { return Err("Step owner has no active explicit pilot".into()); }
        if owner == Owner::Passive && controls != [0.0; 21] {
            return Err("Passive owner attempted active motor torque".into());
        }
        if owner != Owner::FixturePreparation && let Some(start) = state.activation_native {
            let k = s.substeps_completed.checked_sub(start).ok_or("Owner clock reversed")?;
            let alpha = (k.min(20) as f32) / 20.0;
            for control in &mut controls { *control *= alpha; }
        }
    }
    if controls.iter().any(|x| !x.is_finite() || x.abs() > 1.0) {
        return Err("Invalid native owner motor command".into());
    }
    Ok(controls)
}

pub(crate) fn record_application(s: &mut Simulation, owner: Owner, controls: [f32;21]) {
    if let Some(state) = &mut s.native_locomotion {
        state.last_application = Some(Application {native_input:s.substeps_completed, owner, controls});
    }
}

/// Read-only fresh physical state. Contacts are the last COMPLETED native
/// interval, current poses/twists are read now, and grab receipts name their
/// own interval. Floor-only contact scope is explicit, not terrain support.
pub fn observation(s: &Simulation) -> Result<serde_json::Value, String> {
    validate(s)?;
    let (q, dq) = super::joint_state(s)?;
    let twists = super::current_twists(s)?;
    let root = &s.world.bodies[s.bodies[0]];
    let com = s.center_of_mass();
    let mass = s.total_mass();
    let com_velocity = s.bodies.iter().zip(&twists)
        .map(|(h, (v, _))| *v * s.world.bodies[*h].mass()).sum::<Vector>() / mass;
    if q.iter().chain(dq.iter()).any(|x|!x.is_finite()) || !com.is_finite()
        || !com_velocity.is_finite() || twists.iter().any(|(v,w)|!v.is_finite() || !w.is_finite())
    { return Err("Non-finite native observation".into()); }
    let contacts = super::standing::contacts_including_unloaded(s)?;
    if contacts.iter().any(|c| !c.point.is_finite() || !c.floor_point.is_finite()
        || !c.normal.is_finite() || !c.velocity.is_finite() || !c.tangent_impulse_world.is_finite()
        || !c.impulse.is_finite() || c.impulse<0.0 || !c.signed_gap_m.is_finite()) {
        return Err("Non-finite/invalid resolved native contact".into());
    }
    let contacts:Vec<_> = contacts.iter().map(|c|serde_json::json!({
        "body":s.model.segments[c.body].id, "point":c.point.to_array(),
        "surfacePoint":c.floor_point.to_array(), "normal":c.normal.to_array(),
        "normalImpulseNs":c.impulse, "tangentImpulseNs":c.tangent_impulse_world.to_array(),
        "pointVelocityMps":c.velocity.to_array(), "signedGapM":c.signed_gap_m
    })).collect();
    let foot_geometry=match super::native_step::geometry(s) {
        Ok(feet)=>serde_json::json!({"feet":feet,"scope":"flat-floor capsule extrema"}),
        Err(reason)=>serde_json::json!({"unavailable":reason}),
    };
    let bodies:Vec<_> = s.model.segments.iter().enumerate().map(|(i,p)|serde_json::json!({
        "id":p.id,"massKg":p.mass_kg,"position":s.world.bodies[s.bodies[i]].translation().to_array(),
        "rotation":s.world.bodies[s.bodies[i]].rotation().to_array(),
        "comVelocityMps":twists[i].0.to_array(),"angularVelocityRadps":twists[i].1.to_array(),
        "resolvedSupportLoadN":s.integrated_support_loads_n[i]
    })).collect();
    Ok(serde_json::json!({"format":FORMAT,"physicsIntegrations":0,
        "generation":s.generation,"nativeCompleted":s.substeps_completed,"motion":s.motion,
        "coordinateContract":"q/dq in source hinge order; controls in actuator order; world Y up",
        "velocitySource":"current multibody generalized velocities, not rigid-body caches",
        "q":q,"dq":dq,"root":{"body":"torso","position":root.translation().to_array(),
            "rotation":root.rotation().to_array(),"comVelocityMps":twists[0].0.to_array(),
            "angularVelocityRadps":twists[0].1.to_array()},
        "com":com.to_array(),"comVelocityMps":com_velocity.to_array(),"massKg":mass,
        "bodies":bodies,"floorFootContacts":contacts,
        "floorFootGeometry":foot_geometry,
        "footGeometryContract":"capsule extrema independent of impulse; geometric proximity is not load","contactNativeCompleted":s.substeps_completed,
        "contactScope":"flat-floor solver foot contacts including zero impulse; geometry is not load",
        "activeGrab":s.grab.as_ref().map(|g|serde_json::json!({"press":g.press,
            "body":s.model.segments[g.segment].id,"localAnchor":g.local_anchor.to_array(),
            "rawTarget":g.raw_target.to_array(),"filteredTarget":g.control_target().to_array()})),
        "lastAppliedGrab":s.native_grab_application,"ownership":s.native_locomotion,
        "earnedBalanceReady":s.getup_controller.as_ref().is_some_and(|c|c.native_balance_ready(s.substeps_completed)),
        "failure":s.first_failure}))
}

#[cfg(all(test, feature="getup-native-locomotion-experiment"))]
mod tests {
    use super::*;
    fn simulation()->Simulation {
        let mut p=crate::Profile::default();p.getup_policy_enabled=true;
        p.getup_repeated_recovery=Some(GetUpRepeatedRecovery::CompletedCycleConfirmedFallV1);
        p.getup_native_locomotion=Some(Profile::NativeJointOwnerV1);
        Simulation::new(0.0,p,true).unwrap()
    }
    #[test] fn native_step_pure_entry_requires_earned_state_without_mutation() {
        let mut s=simulation();let before=serde_json::to_value(&s).unwrap();
        assert!(begin_step(&mut s,0).is_err());
        assert_eq!(serde_json::to_value(&s).unwrap(),before);assert_eq!(s.substeps_completed,0);
        assert!(finish_command(&mut s,Owner::Step,[0.;21]).is_err());
    }
    #[test] fn native_step_pure_geometry_preserves_zero_load_candidates() {
        let s=simulation();let before=serde_json::to_value(&s).unwrap();
        let feet=super::super::native_step::geometry(&s).unwrap();
        assert!(feet.iter().all(|f|f.load==0. && !f.points.is_empty()));
        assert_eq!(serde_json::to_value(&s).unwrap(),before);assert_eq!(s.substeps_completed,0);
    }
    #[test] fn native_contract_native_observation_is_fresh_and_nonstepping() {
        let s=simulation();let before=serde_json::to_value(s.snapshot()).unwrap();
        let o=observation(&s).unwrap();assert_eq!(o["nativeCompleted"],0);
        assert_eq!(o["physicsIntegrations"],0);assert_eq!(o["q"].as_array().unwrap().len(),21);
        assert_eq!(o["bodies"].as_array().unwrap().len(),25);
        assert_eq!(o["q"],serde_json::to_value(super::super::joint_state(&s).unwrap().0).unwrap());
        assert_eq!(serde_json::to_value(s.snapshot()).unwrap(),before);
    }
    #[test] fn native_contract_passive_zeroes_only_motors_and_activation_starts_at_zero() {
        let mut s=simulation();let before=super::super::joint_state(&s).unwrap();
        set_mode(&mut s,Mode::Passive).unwrap();
        for h in &s.bodies {s.world.bodies[*h].reset_forces(false);s.world.bodies[*h].reset_torques(false);}
        s.configure_motors().unwrap();let a=s.native_locomotion.as_ref().unwrap().last_application.as_ref().unwrap();
        assert_eq!(a.owner,Owner::Passive);assert_eq!(a.controls,[0.0;21]);
        assert_eq!(super::super::joint_state(&s).unwrap(),before);
        set_mode(&mut s,Mode::Active).unwrap();s.configure_motors().unwrap();
        assert_eq!(s.native_locomotion.as_ref().unwrap().last_application.as_ref().unwrap().controls,[0.0;21]);
        assert_eq!(s.substeps_completed,0);
    }
    #[test] fn native_contract_profile_and_state_cannot_be_silently_dropped() {
        let mut s=simulation();s.native_locomotion=None;assert!(validate(&s).is_err());
        let mut s=simulation();s.native_locomotion.as_mut().unwrap().generation+=1;
        assert!(validate(&s).is_err());
    }
    #[test] fn native_contract_unsupported_step_command_and_nonfinite_control_are_rejected() {
        let mut s=simulation();assert!(finish_command(&mut s,Owner::Step,[0.0;21]).is_err());
        assert!(finish_command(&mut s,Owner::Balance,[f32::NAN;21]).is_err());
    }
    #[test] fn native_contract_fixture_preparation_retains_its_explicit_zero_owner() {
        let mut s=simulation();
        s.native_locomotion.as_mut().unwrap().mode=Mode::Passive;
        assert_eq!(finish_command(&mut s,Owner::FixturePreparation,[0.0;21]).unwrap(),[0.0;21]);
        assert_eq!(s.substeps_completed,0);
    }
    #[test] fn native_contract_disabled_profile_remains_byte_omitted() {
        let p=crate::Profile::default();let json=serde_json::to_value(&p).unwrap();
        assert!(json.get("getup_native_locomotion").is_none());
        let s=Simulation::new(0.0,p,true).unwrap();
        assert!(serde_json::to_value(&s).unwrap().get("native_locomotion").is_none());
    }
}
