use lh_sim::{Profile,Simulation};
use serde_json::{Value,json};
fn trial_profile() -> Profile {
 if let Ok(path)=std::env::var("LH_BUDGET_PROFILE") { let profile:Profile=serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap(); profile.validate().unwrap(); return profile; }

    let mut profile = Profile::default();
    match std::env::var("LH_BUDGET_CANDIDATE").as_deref() {
        Ok("1") => {
            profile.id = "getup-800hz-4plus1-qualification-v1".into();
            profile.solver_iterations = 4;
            profile.additional_body_iterations = 1;
        }
        Ok("0") | Err(_) => {}
        Ok(other) => panic!("Unknown budget profile selector: {other}"),
    }
    profile.validate().unwrap();
    profile
}

fn report(name:&str,result:Result<Value,String>,profile:&Profile){
 match result {Ok(cases)=>println!("{}",json!({"kind":name,"profile":profile,"cases":cases,"releaseAccepted":false})),Err(error)=>println!("{}",json!({"kind":name,"profile":profile,"error":error,"releaseAccepted":false}))}
}
fn main(){
 let profile=trial_profile();
 let kind=std::env::args().nth(1).expect("explicit regression category");
 match kind.as_str(){
 "actuators"=>report(&kind,lh_sim::actuators::trials(&profile).and_then(|x|serde_json::to_value(x).map_err(|e|e.to_string())),&profile),
 "joints"=>report(&kind,lh_sim::calibration::isolated_joints_with_profile(&profile).and_then(|x|serde_json::to_value(x).map_err(|e|e.to_string())),&profile),
 "contacts"=>report(&kind,lh_sim::calibration::canonical_contact_load_with_profile(&profile).and_then(|x|serde_json::to_value(x).map_err(|e|e.to_string())),&profile),
 "feet"=>report(&kind,lh_sim::calibration::individual_feet_with_profile(true,&profile).and_then(|x|serde_json::to_value(x).map_err(|e|e.to_string())),&profile),
 "chains"=>report(&kind,lh_sim::calibration::loaded_leg_chains_with_profile(&profile).and_then(|x|serde_json::to_value(x).map_err(|e|e.to_string())),&profile),
 "ballistic"=>{
  for heading in [0.0,std::f32::consts::PI/3.0,-std::f32::consts::PI/4.0] {
   let mut s=Simulation::new(heading,profile.clone(),false).unwrap();let origin=s.center_of_mass();let mut rows=Vec::new();let mut failure=None;
   for tick in 0..=30 {
    if tick>0 {if let Err(f)=s.advance_tick(){failure=Some(f);}}
    let delta=s.center_of_mass()-origin;let snap=s.snapshot();
    rows.push(json!({"tick":tick,"physicalSeconds":snap.simulation_time_s,"deltaM":[delta.x,delta.y,delta.z],"horizontalAbsCoordinatesPass":delta.x.abs()<0.0001&&delta.z.abs()<0.0001,"contacts":snap.contacts.len(),"motion":snap.motion}));
    if failure.is_some(){break;}
   }
   println!("{}",json!({"kind":kind,"profile":profile,"heading":heading,"failure":failure,"horizontalCoordinateToleranceM":0.0001,"rows":rows,"releaseAccepted":false}));
  }
 },
 _=>panic!("Unknown regression category"),
 }
}
