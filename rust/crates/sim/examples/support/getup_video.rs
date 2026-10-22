//! Single V3 belly video capture. No Step, strike, retry or mid-run reset.
use lh_sim::{Profile,getup,integration_budget as budget,measurement::Failure,runtime::Runtime};
use serde_json::{Value,json};
use std::{fs::OpenOptions,io::{Write,Seek,SeekFrom},path::Path};
fn pose_ids(o:&Value)->Vec<String> {
    o["bodies"].as_array().unwrap().iter().filter(|b|b["massKg"].as_f64().unwrap_or(0.)>0.).map(|b|b["id"].as_str().unwrap().to_string()).collect()
}
fn open_poses(path:&Path,ids:&[String])->Result<std::fs::File,String> {
    if ids.len()!=16 {return Err("Capture requires 16 physical bodies".into());}
    let mut f=OpenOptions::new().write(true).create_new(true).open(path).map_err(|e|e.to_string())?;
    let h=serde_json::to_vec(&json!({"format":"native-poses-f32le-v1","ids":ids,"dtSeconds":0.00125,"strideNative":20,"frameBytes":456})).map_err(|e|e.to_string())?;
    f.write_all(&(h.len() as u32).to_le_bytes()).and_then(|_|f.write_all(&h)).and_then(|_|f.sync_all()).map_err(|e|e.to_string())?;Ok(f)
}
fn write_pose(f:&mut std::fs::File,o:&Value,ids:&[String])->Result<(),String> {
    let mut bytes=Vec::with_capacity(456);
    bytes.extend_from_slice(&o["nativeCompleted"].as_u64().ok_or("Missing pose native")?.to_le_bytes());
    for id in ids {
        let b=o["bodies"].as_array().ok_or("Missing bodies")?.iter().find(|b|b["id"].as_str()==Some(id)).ok_or("Missing capture body")?;
        for (field,n) in [("position",3),("rotation",4)] {
            let values=b[field].as_array().ok_or("Missing pose component")?;if values.len()!=n {return Err("Invalid pose length".into());}
            for v in values {let v=v.as_f64().ok_or("Invalid pose number")? as f32;if !v.is_finite(){return Err("Nonfinite pose".into());}bytes.extend_from_slice(&v.to_le_bytes());}
        }
    }
    if bytes.len()!=456 {return Err("Invalid frame size".into());}
    f.write_all(&bytes).and_then(|_|f.sync_data()).map_err(|e|e.to_string())
}


pub fn capture(out:&Path)->Result<(),String> {
 let mut result_file=OpenOptions::new().write(true).create_new(true).open(out).map_err(|e|e.to_string())?;
 result_file.write_all(b"{\"status\":\"capture started; outcome pending\"}\n").and_then(|_|result_file.sync_all()).map_err(|e|e.to_string())?;
 budget::install_getup_video_capture()?;
 let mut profile:Profile=serde_json::from_str(include_str!("../../../../../docs/evidence/profiles/getup-800hz-3plus1-repeat-cycle-v1.json")).map_err(|e|e.to_string())?;
 profile.getup_native_locomotion=Some(getup::locomotion::Profile::NativeJointOwnerV1);profile.validate()?;
 let mut runtime=Runtime::new_getup_fixture_with_profile("belly",profile.clone())?;
 runtime.set_root_translation_compensation_enabled(true)?; // exact numerical free-root integrator, not a root force
 let initial:Value=serde_json::from_str(&runtime.getup_native_observation()?).map_err(|e|e.to_string())?;
 let ids=pose_ids(&initial);let mut poses=open_poses(&out.with_extension("poses.bin"),&ids)?;write_pose(&mut poses,&initial,&ids)?;
 let mut progress=OpenOptions::new().write(true).create_new(true).open(out.with_extension("progress.jsonl")).map_err(|e|e.to_string())?;
 let mut last_pose=0;let mut trigger=None;let mut stop=None;let mut anchors=None;let mut stable_start=None;let mut stable=0;let mut max_stable=0;let mut earned=None;let mut final_diagnostic=Value::Null;let mut maximum_root_force=0f32;let mut maximum_root_torque=0f32;
 let outcome=(||->Result<(),String>{loop {
  if let Err(e)=budget::available(){stop=Some(e);break;}
  if trigger.is_none() {
   let fixture:Value=serde_json::from_str(&runtime.getup_fixture_report()?).map_err(|e|e.to_string())?;
   if fixture["state"]["ready"]==true {runtime.trigger_getup_fixture()?;trigger=fixture["nativeSubsteps"].as_u64();}
   else if budget::snapshot()?.real>=1600 {stop=Some("belly-fixture-not-admitted".into());break;}
  }
  let reply=runtime.advance_observed(1,|s,substep| {
   let o=getup::locomotion::observation(s).map_err(|e|Failure::new(s.tick,substep,"video-observation",e,0.,0.))?;let n=o["nativeCompleted"].as_u64().unwrap();
   if n-last_pose>=20 {write_pose(&mut poses,&o,&ids).map_err(|e|Failure::new(s.tick,substep,"video-poses",e,0.,0.))?;last_pose=n;}
   if trigger.is_some() {
    let d=getup::controller::standing_diagnostic(s,anchors).map_err(|e|Failure::new(s.tick,substep,"video-standing",e,0.,0.))?;
    let predicates=d["predicates"].as_object().ok_or_else(||Failure::new(s.tick,substep,"video-standing","missing predicates".into(),0.,0.))?;
    let passes=predicates.iter().all(|(k,v)|v==true || (k=="footDriftFromSuppliedAnchors" && anchors.is_none()));
    if passes {if anchors.is_none(){anchors=Some(serde_json::from_value(d["feet"].clone()).map_err(|e|Failure::new(s.tick,substep,"video-anchors",e.to_string(),0.,0.))?);}let start=*stable_start.get_or_insert(n);stable=n-start;max_stable=max_stable.max(stable);}else{anchors=None;stable_start=None;stable=0;}
    if o["earnedBalanceReady"]==true && earned.is_none(){earned=Some(n);}
    final_diagnostic=json!({"native":n,"predicates":d["predicates"],"thresholds":d["thresholds"],"stableIntervals":stable});
   }
   if n%20==0 {
    let line=serde_json::to_vec(&json!({"native":n,"ledger":budget::snapshot().ok(),"motion":o["motion"],"com":o["com"],"root":o["root"],"earned":o["earnedBalanceReady"],"standing":final_diagnostic,"fixtureTriggeredAt":trigger})).map_err(|e|Failure::new(s.tick,substep,"video-progress",e.to_string(),0.,0.))?;
    if progress.metadata().map_err(|e|Failure::new(s.tick,substep,"video-progress",e.to_string(),0.,0.))?.len()+line.len() as u64>900_000{return Err(Failure::new(s.tick,substep,"video-proof-budget","progress reserve exhausted".into(),0.,0.));}
    progress.write_all(&line).and_then(|_|progress.write_all(b"\n")).and_then(|_|progress.sync_data()).map_err(|e|Failure::new(s.tick,substep,"video-progress",e.to_string(),0.,0.))?;
   }
   if o["earnedBalanceReady"]==true && stable>=800 {stop=Some("standing-acquired-and-one-second-stable".into());return Err(Failure::new(s.tick,substep,"video-complete","standing-acquired-and-one-second-stable".into(),0.,0.));}
   Ok(())
  }).map_err(|e|format!("runtime: {e:?}"))?;
  maximum_root_force=maximum_root_force.max(reply.snapshot.diagnostics.direct_pelvis_force_n);maximum_root_torque=maximum_root_torque.max(reply.snapshot.diagnostics.direct_pelvis_torque_nm);
  if maximum_root_force!=0. || maximum_root_torque!=0. {stop=Some("unexpected-root-assistance".into());break;}
  if let Some(f)=reply.failure {if stop.is_none(){stop=Some(format!("{}: {}",f.check,f.offending));}break;}
  if reply.completed_ticks==0 {stop=Some("runtime-no-progress".into());break;}
 } Ok(())} )();
 if let Err(e)=outcome {if stop.is_none(){stop=Some(e);}}
 let final_o:Value=serde_json::from_str(&runtime.getup_native_observation()?).map_err(|e|e.to_string())?;let n=final_o["nativeCompleted"].as_u64().ok_or("missing final native")?;
 if n!=last_pose{write_pose(&mut poses,&final_o,&ids)?;}
 let checkpoint=super::checkpoint::save_compressed_checkpoint_bounded(&runtime,&out.with_extension("stop.bin.gz"),800_000);
 let ledger=budget::snapshot()?;let receipt=json!({"format":"belly-getup-video026-v1","historical":37589,"captureBudget":8000,"cumulativeLimit":45589,"totalConsumed":37589+ledger.real+ledger.forecast,"ledger":ledger,"profile":profile,"initialObservation":initial,"finalObservation":final_o,"triggerNative":trigger,"earnedNative":earned,"maximumContinuousStableSeconds":max_stable as f64*0.00125,"firstStop":stop,"maximumDirectPelvisForceN":maximum_root_force,"maximumDirectPelvisTorqueNm":maximum_root_torque,"stopCheckpoint":checkpoint.as_ref().ok(),"checkpointError":checkpoint.as_ref().err(),"standingSuccess":earned.is_some()&&max_stable>=800,"poses":out.with_extension("poses.bin")});
 let data=serde_json::to_vec(&receipt).map_err(|e|e.to_string())?;if data.len()>250_000{return Err("capture result reserve exceeded".into());}
 result_file.seek(SeekFrom::Start(0)).and_then(|_|result_file.write_all(&data)).and_then(|_|result_file.set_len(data.len() as u64)).and_then(|_|result_file.sync_all()).map_err(|e|e.to_string())?;
 println!("{}",json!({"ledger":ledger,"stop":stop,"standingSuccess":receipt["standingSuccess"],"earnedNative":earned,"output":out}));Ok(())
}
