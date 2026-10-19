use lh_sim::{Profile,Simulation,getup,measurement};
use serde_json::json;
use std::io::Write;
fn trace_run(){
 let args:Vec<_>=std::env::args().collect();let seconds:u32=args.get(1).map_or(Ok(12),|v|v.parse()).unwrap();
 let headings = match args.get(2).map(|s|s.as_str()) {Some("zero")=>vec![0.0], Some("other")=>vec![std::f32::consts::PI/3.0,-std::f32::consts::PI/4.0], _=>vec![0.0,std::f32::consts::PI/3.0,-std::f32::consts::PI/4.0]};
 for heading in headings {
 let mut s=Simulation::new_protocol(heading,trial_profile(),true).unwrap();
 let mut raw = args.get(3).map(|path|std::fs::File::create(path).unwrap());
 if let Some(raw)=&mut raw {writeln!(raw,"{}",json!({"snapshot":s.snapshot(),"standing":getup::controller::standing_diagnostic(&s,None).unwrap()})).unwrap();}
 let mut anchors=None;let mut quiet_reference=None;let mut quiet=measurement::QuietMetrics::default();
 let mut predicates=std::collections::BTreeMap::<String,u64>::new();let mut quiet_failures=std::collections::BTreeMap::<String,u64>::new();let mut support_failures=0;
 let mut minimum_up=1.0_f32; let mut slip=[0.0_f64;2];let mut maximum_foot_drift=[0.0_f64;2];let mut observations=0_u64;let mut maximum_linear=0.0_f64;let mut maximum_angular=0.0_f64;
 let mut failure=None;let mut frames=Vec::new();let began=std::time::Instant::now();
 for tick in 0..seconds*40 {
  let result=s.advance_tick_observed(|s,sub| {
   let d=getup::controller::standing_diagnostic(s,anchors).unwrap();
   for i in 0..2 {
    let foot=&d["contactMotion"]["feet"][i];
    if foot["loaded"]==true {if let Some(speed)=foot["impulseWeightedTangentSpeedMps"].as_f64(){slip[i]+=speed*0.00125;}}
   }
   if s.snapshot().simulation_time_s>=0.5 {
    if anchors.is_none(){anchors=Some(serde_json::from_value(d["feet"].clone()).unwrap());}
    observations+=1;minimum_up=minimum_up.min(d["torsoUp"].as_f64().unwrap() as f32);
    if measurement::standing_support(s,sub).is_err(){support_failures+=1;}
    for (key,value) in d["predicates"].as_object().unwrap(){if key=="footDriftFromSuppliedAnchors"&&value.is_null(){continue} if value!=true{let counter=predicates.entry(key.clone()).or_default();if *counter<12 {eprintln!("{}",json!({"at":s.snapshot().simulation_time_s,"predicate":key,"diagnostic":d}));} *counter+=1;}}
    for motion in d["motion"].as_array().unwrap(){maximum_linear=maximum_linear.max(motion["linearMps"].as_f64().unwrap());maximum_angular=maximum_angular.max(motion["angularRadps"].as_f64().unwrap());}
    for i in 0..2 {
     if let Some(drift)=d["footDriftM"][i].as_f64(){maximum_foot_drift[i]=maximum_foot_drift[i].max(drift);}
    }
   }
   if s.snapshot().simulation_time_s>=3.0 {
    if quiet_reference.is_none(){quiet_reference=Some(s.snapshot());}
    if let Err(f)=quiet.observe(s,quiet_reference.as_ref().unwrap(),sub){*quiet_failures.entry(f.check).or_default()+=1;}
   }
   Ok(())
  });
  if let Err(f)=result {failure=Some(f);break;}
  if tick<200 {if let Some(raw)=&mut raw {writeln!(raw,"{}",json!({"snapshot":s.snapshot(),"standing":getup::controller::standing_diagnostic(&s,anchors).unwrap()})).unwrap();}}
  if tick%40==39 {frames.push(json!({"snapshot":s.snapshot(),"standing":getup::controller::standing_diagnostic(&s,anchors).unwrap()}));}
 }
 println!("{}",json!({"heading":heading,"profile":trial_profile(),"physicalSeconds":s.snapshot().simulation_time_s,"physicsWallSeconds":began.elapsed().as_secs_f64(),"nativeObservedAfter0_5S":observations,"failure":failure,"standingSupportFailureSubsteps":support_failures,"standingPredicateFailureSubsteps":predicates,"quietFailuresAfter3S":quiet_failures,"quietLegacyMetrics":quiet,"minimumTorsoUp":minimum_up,"maximumBodyLinearMps":maximum_linear,"maximumBodyAngularRadps":maximum_angular,"loadedSlipIntegralM":slip,"maximumFootOriginDriftM":maximum_foot_drift,"frames":frames}));std::io::stdout().flush().unwrap();
 }
}

fn trial_profile() -> Profile {
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
fn main() {
    match std::env::args().nth(1).as_deref() {
        Some("profile") => println!("{}", serde_json::to_string_pretty(&trial_profile()).unwrap()),
        Some("benchmark") => {
            for trial in 0..3 {
                let profile = trial_profile();
                let mut simulation = Simulation::new_protocol(0.0, profile.clone(), true).unwrap();
                let began = std::time::Instant::now();
                for _ in 0..120 { simulation.advance_tick().unwrap(); }
                let wall_seconds = began.elapsed().as_secs_f64();
                println!("{}", json!({"trial":trial,"profile":profile,"ticks":120,
                    "physicalSeconds":3.0,"nativeIntegrations":2400,"wallSeconds":wall_seconds,
                    "simulationToWallRatio":3.0/wall_seconds,"snapshot":simulation.snapshot(),
                    "observer":"ordinary advance_tick; no extra diagnostic callback",
                    "timingExcludes":["construction","final snapshot serialization"]}));
            }
        }
        _ => trace_run(),
    }
}
