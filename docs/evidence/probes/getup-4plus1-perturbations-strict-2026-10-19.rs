use lh_sim::{Profile,Simulation,getup,measurement};
use serde_json::json;
use std::collections::BTreeMap;
fn profile()->Profile {
 let mut p=Profile::default();
 if std::env::var("LH_BUDGET_CANDIDATE").as_deref()==Ok("1") {p.id="getup-800hz-4plus1-qualification-v1".into();p.solver_iterations=4;p.additional_body_iterations=1;}
 p.validate().unwrap(); p
}
fn main(){
 // Frozen before results: four20N-capped torso pulls at0deg, two3cm hand
 // grabs at0deg, and two20N-capped torso pulls at60deg. All use the real
 // existing point-force grab input, never an impulse/pose/velocity write.
 let cases=[("torso-x+",0.0,"torso",0,1.0,4,20.0),("torso-x-",0.0,"torso",0,-1.0,4,20.0),
 ("torso-z+",0.0,"torso",2,1.0,4,20.0),("torso-z-",0.0,"torso",2,-1.0,4,20.0),
 ("hand-x+",0.0,"left_hand",0,0.03,10,180.0),("hand-z+",0.0,"left_hand",2,0.03,10,180.0),
 ("rotated-torso-x+",std::f32::consts::PI/3.0,"torso",0,1.0,4,20.0),
 ("rotated-torso-x-",std::f32::consts::PI/3.0,"torso",0,-1.0,4,20.0)];
 for (id,heading,body,axis,displacement,held_ticks,force_cap) in cases {
 if std::env::var("LH_PERTURBATION_CASE").ok().is_some_and(|selected| selected != id) { continue; }
 let original_profile=profile();let mut s=Simulation::new_protocol(heading,original_profile.clone(),true).unwrap();
 let release_tick=120+held_ticks;let quiet_tick=release_tick+120;let end_tick=quiet_tick+120;
 let mut anchors=None;let mut quiet_reference=None;let mut quiet=measurement::QuietMetrics::default();
 let mut all_failures=BTreeMap::<String,u64>::new();let mut required=BTreeMap::<String,u64>::new();let mut support=BTreeMap::<String,u64>::new();let mut quiet_failures=BTreeMap::<String,u64>::new();
 let mut slip=[0.0_f64;2];let mut maximum_foot_drift=[0.0_f64;2];let mut minimum_up=1.0_f64;let mut minimum_load=[f64::MAX;2];let mut maximum_linear=0.0_f64;let mut maximum_angular=0.0_f64;let mut samples=0;let mut quiet_samples=0;let mut failure=None;let mut frames=Vec::new();
 for tick in 0..end_tick {
  if tick==120 {
   let snap=s.snapshot();quiet_reference=Some(snap.clone());let index=snap.segments.iter().position(|p|p.id==body).unwrap();
   let mut target=snap.segments[index].position;
   match axis{0=>target.x+=displacement,2=>target.z+=displacement,_=>unreachable!()}
   let mut local=target;local.x=0.0;local.y=0.0;local.z=0.0;
   s.profile.grab_limits.force_n=force_cap;
   s.begin_grab(index,1,local,target).unwrap();
  }
  if tick==release_tick {s.cancel_grab();s.profile.grab_limits.force_n=original_profile.grab_limits.force_n;}
  let quiet_window=tick<120||tick>=quiet_tick;
  let result=s.advance_tick_observed(|s,sub|{
   let d=getup::controller::standing_diagnostic(s,anchors).unwrap();
   for foot in 0..2 {let m=&d["contactMotion"]["feet"][foot];if m["loaded"]==true {if let Some(speed)=m["impulseWeightedTangentSpeedMps"].as_f64(){slip[foot]+=speed*0.00125;}}}
   if s.snapshot().simulation_time_s>=0.5 {
    samples+=1;if anchors.is_none(){anchors=Some(serde_json::from_value(d["feet"].clone()).unwrap());}
    minimum_up=minimum_up.min(d["torsoUp"].as_f64().unwrap());
    for foot in 0..2 {let i=d["footIds"][foot].as_u64().unwrap() as usize;minimum_load[foot]=minimum_load[foot].min(d["loadsN"][i].as_f64().unwrap());if let Some(v)=d["footDriftM"][foot].as_f64(){maximum_foot_drift[foot]=maximum_foot_drift[foot].max(v);}}
    if let Err(f)=measurement::standing_support(s,sub){*support.entry(f.check).or_default()+=1;}
    for (name,value) in d["predicates"].as_object().unwrap(){
     if name=="footDriftFromSuppliedAnchors"&&value.is_null(){continue;}
     if value!=true {
      *all_failures.entry(name.clone()).or_default()+=1;
      if quiet_window||!matches!(name.as_str(),"linearMotion"|"angularMotion"){
       let count=required.entry(name.clone()).or_default();if *count<2 {eprintln!("{}",json!({"case":id,"tick":tick,"substep":sub,"predicate":name,"diagnostic":d}));}*count+=1;
      }
     }
    }
    for m in d["motion"].as_array().unwrap(){maximum_linear=maximum_linear.max(m["linearMps"].as_f64().unwrap());maximum_angular=maximum_angular.max(m["angularRadps"].as_f64().unwrap());}
    if let Some(foot)=slip.iter().position(|x|*x>0.02){
     *required.entry("cumulativeLoadedSlip20mm".into()).or_default()+=1;
     return Err(measurement::Failure::new(s.tick,sub,"loaded-foot-slip-stop",format!("foot-{foot}"),slip[foot] as f32,0.02));
    }
   }
   if tick>=quiet_tick {quiet_samples+=1;if quiet_reference.is_none(){quiet_reference=Some(s.snapshot());}if let Err(f)=quiet.observe(s,quiet_reference.as_ref().unwrap(),sub){*quiet_failures.entry(f.check).or_default()+=1;}}
   Ok(())
  });
  if let Err(f)=result{failure=Some(f);break;}
  if tick==119||tick==release_tick-1||tick==quiet_tick-1||tick==end_tick-1 {frames.push(json!({"tick":tick+1,"snapshot":s.snapshot(),"standing":getup::controller::standing_diagnostic(&s,anchors).unwrap()}));}
 }
 println!("{}",json!({"case":id,"heading":heading,"profile":original_profile,"stimulus":{"body":body,"axis":axis,"targetDisplacementM":displacement,"heldTicks":held_ticks,"durationS":held_ticks as f64/40.0,"pointForceCapN":force_cap,"mechanism":"existing bounded point-force grab; force cap temporarily sets only stimulus amplitude, restored on release"},"schedule":{"initialSettleTicks":120,"postReleaseSettleTicks":120,"finalQuietTicks":120,"quietReference":"pre-input settled snapshot; not reset after input"},"physicalSeconds":s.snapshot().simulation_time_s,"failure":failure,"allPredicateFailureSamples":all_failures,"requiredPredicateFailures":required,"supportFailures":support,"quietFailures":quiet_failures,"nativeSamplesAfter0_5s":samples,"finalQuietSamples":quiet_samples,"loadedSlipIntegralM":slip,"maxFootDriftM":maximum_foot_drift,"minimumTorsoUp":minimum_up,"minimumFootLoadN":minimum_load,"maximumBodyLinearMps":maximum_linear,"maximumBodyAngularRadps":maximum_angular,"quietMetrics":quiet,"finalCounters":s.snapshot().counters,"finalState":s.snapshot(),"frames":frames}));
 }
}
