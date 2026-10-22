//! Prepared replay from native6186: historical flight, explicit reception only.
//! No execution authorized by compilation; --resume-stop requires a separate GO.
//! Flight controller e5151e23; new reception is not attributed to the historical commit.
use lh_sim::{getup,integration_budget as budget,measurement::Failure,runtime::Runtime};
use serde_json::{Value,json};
use std::{fs::OpenOptions,io::{Write,Seek,SeekFrom},path::Path};
#[path="support/checkpoint.rs"]
mod checkpoint;

// Fixed-size frames: native:u64 + 16*(position:3f32, rotation:4f32).
// Metadata is a length-prefixed JSON header; each frame is synced immediately.
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

// The immutable curve is stored once in the input JSON and exactly in the final
// checkpoint. Event observations omit its repeated coefficients, never bodies.
fn compact_observation(o:&Value)->Value {
 let mut v=o.clone();if let Some(p)=v["ownership"]["step"]["placement"].as_object_mut() {p.remove("segments");}v
}

fn run(input:&Path,out:&Path,preflight_only:bool,placement:Option<&Path>)->Result<Value,String> {
    let digest=std::process::Command::new("sha256sum").arg(input).output().map_err(|e|e.to_string())?;
    if !digest.status.success() || String::from_utf8_lossy(&digest.stdout).split_whitespace().next()!=Some("0737e7b3f237336486af699a28f0f92b5b27cf3afa622a96b70d9a563f9224fa") {return Err("Input is not the preserved exact native6186 checkpoint".into());}
    let decoded=std::process::Command::new("gzip").args(["-d","-c"]).arg(input).output().map_err(|e|e.to_string())?;
    if !decoded.status.success() {return Err("Input gzip decoding failed".into());}
    let bytes=decoded.stdout;
    let mut runtime=Runtime::from_getup_checkpoint(&bytes)?;
    let restored:Value=serde_json::from_str(&runtime.verify_getup_checkpoint(&bytes)?).map_err(|e|e.to_string())?;
    let before:Value=serde_json::from_str(&runtime.getup_native_observation()?).map_err(|e|e.to_string())?;
    if restored["passed"]!=true || before["nativeCompleted"]!=6186 || before["failure"]["offending"]!="Step joint allocation did not converge within 256 projections in the current affine region; not global infeasibility" {
        return Err("Resume requires verified saved native6186 refusal".into());
    }
    // This performs the complete reference/posture, passive/dynamic RNEA,
    // cone allocation and unchanged motor-cap admission, without integration.
    if let Some(path)=placement {
        runtime.arm_getup_post_separation_path(&std::fs::read_to_string(path).map_err(|e|e.to_string())?)?;
    }
    let admission=runtime.resume_getup_native_step_6186();
    let after_admission:Value=serde_json::from_str(&runtime.getup_native_observation()?).map_err(|e|e.to_string())?;
    let started=before["ownership"]["step"]["started_native"].as_u64().ok_or("Missing original Step clock")?;
    let native=before["nativeCompleted"].as_u64().ok_or("Missing native clock")?;
    let elapsed=native.checked_sub(started).ok_or("Invalid original Step clock")?;
    let pilot_remaining=2400u64.checked_sub(elapsed).ok_or("Original pilot expired")?;
    let duration=after_admission["ownership"]["step"]["reference"]["duration"].as_f64().ok_or("Missing reference duration")?;
    let reference_elapsed=after_admission["ownership"]["step"]["reference"]["elapsed"].as_f64().ok_or("Missing reference elapsed")?;
    let remaining_seconds=pilot_remaining.min(2000) as f64*0.00125;
    let horizon_error=if admission.is_ok() && duration-reference_elapsed>remaining_seconds {Some(format!("Reference needs {} s, original pilot/reserve has {remaining_seconds} s; no compression/reset/extension",duration-reference_elapsed))} else {None};
    let mut preflight=json!({"restoration":restored,"nativeBefore":before["nativeCompleted"],
        "originalStartedNative":started,"originalPilotRemainingNative":pilot_remaining,"referenceRemainingSeconds":duration-reference_elapsed,"availableSeconds":remaining_seconds,
        "commandAdmitted":admission.is_ok(),"horizonError":horizon_error,"acknowledgement":admission.as_ref().ok().and_then(|s|serde_json::from_str::<Value>(s).ok()),"error":admission.as_ref().err(),"commands":budget::step_command_diagnostics(),
        "ledger":budget::snapshot()?});
    if let Some(path)=placement {
        let plan:Value=serde_json::from_slice(&std::fs::read(path).map_err(|e|e.to_string())?).map_err(|e|e.to_string())?;
        preflight["postSeparationPlacement"]=json!({"entryNative":plan["entryNative"],"durationSeconds":plan["durationSeconds"],"descentAtSeconds":plan["descentAtSeconds"],"fullPostSeparationCommandVerified":false,"runtimeAuthorized":!preflight_only});
    }
    if preflight_only || admission.is_err() || horizon_error.is_some() {
        return Ok(json!({"preflight":preflight,"integrated":false,"firstStop":admission.err().or(horizon_error)}));
    }
    // All later integration, including forecasts, uses the same global reserve.
    budget::begin_step()?;
    let ids=pose_ids(&before);let mut poses=open_poses(&out.with_extension("poses.bin"),&ids)?;
    write_pose(&mut poses,&before,&ids)?;let mut last_pose_native=native;
    let mut progress=OpenOptions::new().write(true).create_new(true).open(out.with_extension("progress.jsonl")).map_err(|e|e.to_string())?;
    let mut events=OpenOptions::new().write(true).create_new(true).open(out.with_extension("events.jsonl")).map_err(|e|e.to_string())?;
    let mut previous_signature:Option<([bool;2],bool,String,bool)>=None;
    let mut stable_anchors=None;let mut stable_start=None;let mut stable_intervals=0u64;let mut max_stable_intervals=0u64;
    let mut stability_last=json!({});let mut reception_start=None;let mut old_deadline_crossed=None;
    let original_deadline=started+2400;

    let mut first_stop=None;let mut separated_seen=false;let mut landed_seen=false;
    let mut positive_clearance_samples=0u64;let mut positive_clearance_run=0u64;let mut maximum_positive_clearance_run=0u64;
    let mut confirmed_separation_samples=0u64;
    let mut samples=0u64;let mut extrema=json!({});let mut brake_exit=None;
    let foot_index=before["bodies"].as_array().ok_or("missing bodies")?.iter().position(|b|b["id"]=="left_foot").ok_or("missing left foot")?;
    let initial_foot:[f64;3]=serde_json::from_value(before["bodies"][foot_index]["position"].clone()).map_err(|e|e.to_string())?;
    let forward:[f64;3]=serde_json::from_value(before["ownership"]["step"]["forward"].clone()).map_err(|e|e.to_string())?;
    let mut max_retreat=0f64;let mut max_lift=0f64;let mut max_clearance=f64::NEG_INFINITY;let mut min_free_load=f64::INFINITY;
    let initial_com:[f64;3]=serde_json::from_value(before["com"].clone()).map_err(|e|e.to_string())?;
    let mut trace=vec![];let mut separation=None;let mut landing=None;let mut minimum_load_sample=None;
    let mut refresh_trace=vec![];let mut last_refresh=None;
    let outcome=(||->Result<(),String> {
        loop {
            if let Err(e)=budget::available(){first_stop=Some(e);break;}
            let reply=runtime.advance_observed(1,|s,substep| {
                let o=getup::locomotion::observation(s).map_err(|e|Failure::new(s.tick,substep,"continuation-observation",e,0.,0.))?;
                samples+=1;
                let native=o["nativeCompleted"].as_u64().ok_or_else(||Failure::new(s.tick,substep,"clock","missing native".into(),0.,0.))?;
                if native-last_pose_native>=20 {write_pose(&mut poses,&o,&ids).map_err(|e|Failure::new(s.tick,substep,"pose-write",e,0.,0.))?;last_pose_native=native;}
                if native>original_deadline && old_deadline_crossed.is_none() {old_deadline_crossed=Some(native);}
                let reception=o["ownership"]["step"]["reception"]["started"].as_u64();
                if reception_start.is_none() {reception_start=reception;}
                if reception.is_some() {
                    let d=getup::controller::standing_diagnostic(s,stable_anchors).map_err(|e|Failure::new(s.tick,substep,"stability",e,0.,0.))?;
                    let predicates=d["predicates"].as_object().ok_or_else(||Failure::new(s.tick,substep,"stability","missing predicates".into(),0.,0.))?;
                    let passes=predicates.iter().all(|(k,v)|v==true || (k=="footDriftFromSuppliedAnchors" && stable_anchors.is_none()));
                    if passes {
                        if stable_anchors.is_none() {stable_anchors=Some(serde_json::from_value(d["feet"].clone()).map_err(|e|Failure::new(s.tick,substep,"stability",e.to_string(),0.,0.))?);}
                        let start=*stable_start.get_or_insert(native);stable_intervals=native-start;
                        max_stable_intervals=max_stable_intervals.max(stable_intervals);
                    } else {stable_start=None;stable_anchors=None;stable_intervals=0;}
                    stability_last=json!({"native":native,"predicates":d["predicates"],"thresholds":d["thresholds"],"continuousIntervals":stable_intervals});
                }

                let phase=o["ownership"]["step"]["phase"].as_str().unwrap_or("missing");
                let loads=std::array::from_fn(|i|o["floorFootGeometry"]["feet"][i]["load"].as_f64().unwrap_or(0.)>=3.);
                let separated=o["floorFootGeometry"]["feet"][0]["clearance"].as_f64().unwrap_or(0.)>0.;
                let path_started=o["ownership"]["step"]["placement"]["started"].as_bool().unwrap_or(false);
                let signature=(loads,separated,phase.to_string(),path_started);
                if previous_signature.as_ref()!=Some(&signature) {
                    let command=budget::last_step_command().unwrap_or(Value::Null);
                    // Full measured boundary, including q/dq, root/body poses and
                    // velocities, current contact points/impulses and owner state.
                    // This is an observation, not a reconstructible Rapier checkpoint.
                    let line=serde_json::to_vec(&json!({"native":native,"loadedFeet":loads,"capsuleSeparated":separated,"observation":compact_observation(&o),"ledger":budget::snapshot().ok(),"command":{"native":command["native"],"controlMode":command["controlMode"],"reason":command["reason"],"requestedCop":command["requestedCopXZ"],"commandedCop":command["commandedCopXZ"],"pointForces":command["pointForces"],"postureTarget":command["postureTarget"],"rawPd":command["rawPd"],"compensationControls":command["compensationControls"],"requestedControls":command["requestedControls"],"controls":command["controls"],"motorLimiting":command["motorLimiting"]}})).map_err(|e|Failure::new(s.tick,substep,"event",e.to_string(),0.,0.))?;
                    if events.metadata().map_err(|e|Failure::new(s.tick,substep,"event",e.to_string(),0.,0.))?.len()+line.len() as u64>1_500_000 {return Err(Failure::new(s.tick,substep,"event-reserve","event reserve exhausted".into(),0.,0.));}
                    events.write_all(&line).and_then(|_|events.write_all(b"\n")).and_then(|_|events.sync_data()).map_err(|e|Failure::new(s.tick,substep,"event",e.to_string(),0.,0.))?;
                    previous_signature=Some(signature);
                }
                separated_seen|=o["ownership"]["step"]["separated"]==true;
                landed_seen|=o["ownership"]["step"]["landed_samples"].as_u64().unwrap_or(0)>0;
                let foot:[f64;3]=serde_json::from_value(o["bodies"][foot_index]["position"].clone()).map_err(|e|Failure::new(s.tick,substep,"foot-observation",e.to_string(),0.,0.))?;
                let retreat=-(0..3).map(|i|(foot[i]-initial_foot[i])*forward[i]).sum::<f64>();
                max_retreat=max_retreat.max(retreat);max_lift=max_lift.max(foot[1]-initial_foot[1]);
                if brake_exit.is_none() && o["ownership"]["step"]["reference"]["braking"]==false {
                    brake_exit=Some(json!({"native":o["nativeCompleted"],"retreatM":retreat,"liftM":foot[1]-initial_foot[1],"feet":o["floorFootGeometry"]}));
                }
                let clearance=o["floorFootGeometry"]["feet"][0]["clearance"].as_f64().ok_or_else(||Failure::new(s.tick,substep,"sole-observation","missing capsule clearance".into(),0.,0.))?;
                let free_load=o["floorFootGeometry"]["feet"][0]["load"].as_f64().ok_or_else(||Failure::new(s.tick,substep,"load-observation","missing load".into(),0.,0.))?;
                landed_seen|=phase=="Reception" && clearance<=0.001 && free_load>=3.;
                max_clearance=max_clearance.max(clearance);
                if clearance>0. {positive_clearance_samples+=1;positive_clearance_run+=1;} else {positive_clearance_run=0;}
                maximum_positive_clearance_run=maximum_positive_clearance_run.max(positive_clearance_run);
                if o["ownership"]["step"]["separated"]==true {confirmed_separation_samples+=1;}
                let measured=json!({"native":o["nativeCompleted"],"com":o["com"],"comVelocityMps":o["comVelocityMps"],"freeLoadN":free_load,"retainedLoadN":o["floorFootGeometry"]["feet"][1]["load"],"wholeSoleClearanceM":clearance,"retreatM":retreat,"phase":phase,"rootRotation":o["bodies"][0]["rotation"],"rootAngularVelocity":o["root"]["angularVelocityRadps"],"stableIntervals":stable_intervals});
                if free_load<min_free_load {min_free_load=free_load;minimum_load_sample=Some(measured.clone());}
                let refresh=o["ownership"]["step"]["preparation"]["attempted"].as_u64().or_else(||o["ownership"]["step"]["preparation"]["updated"].as_u64());
                if refresh!=last_refresh {
                    refresh_trace.push(json!({"observedNative":o["nativeCompleted"],"attemptedNative":refresh,"preparation":o["ownership"]["step"]["preparation"],
                        "comGoal":o["ownership"]["step"]["com"],"footGoal":o["ownership"]["step"]["foot_goal"],
                        "reference":o["ownership"]["step"]["reference"],"measured":measured}));
                    last_refresh=refresh;
                }
                if samples==1 || samples%20==0 {
                    trace.push(measured.clone());
                    let command=budget::last_step_command().unwrap_or(Value::Null);
                    let line=serde_json::to_vec(&json!({"measurement":measured,"ledger":budget::snapshot().ok(),"receptionStarted":reception_start,"compensationActive":command["compensationActive"],"reason":command["reason"],"controlMode":command["controlMode"],"requestedCop":command["requestedCopXZ"],"commandedCop":command["commandedCopXZ"],"copResidual":command["copResidualM"],"motorLimiting":command["motorLimiting"],"controls":o["ownership"]["last_application"]["controls"],"requestedControls":command["requestedControls"],"forceResidual":command["rneaResidualForceN"],"momentResidual":command["rneaResidualMomentNm"],"contactGeometry":o["floorFootContacts"],"placementStarted":path_started,"transitionNative":command["transitionNative"],"stability":stability_last})).map_err(|e|Failure::new(s.tick,substep,"progress",e.to_string(),0.,0.))?;
                    if progress.metadata().map_err(|e|Failure::new(s.tick,substep,"progress",e.to_string(),0.,0.))?.len()+line.len() as u64>3_000_000 {return Err(Failure::new(s.tick,substep,"proof-reserve","progress reserve exhausted".into(),0.,0.));}
                    progress.write_all(&line).and_then(|_|progress.write_all(b"\n")).and_then(|_|progress.sync_data()).map_err(|e|Failure::new(s.tick,substep,"progress",e.to_string(),0.,0.))?;
                }
                if separated_seen && separation.is_none() {separation=Some(measured.clone());}
                if landed_seen && landing.is_none() {landing=Some(measured);}
                extrema=json!({"retreatM":retreat,"liftM":foot[1]-initial_foot[1],"braking":o["ownership"]["step"]["reference"]["braking"],"native":o["nativeCompleted"],"phase":phase,"feet":o["floorFootGeometry"],"separated":o["ownership"]["step"]["separated"],"landedSamples":o["ownership"]["step"]["landed_samples"]});
                let reason=if stable_intervals>=800 {Some("early-reception-one-second-stable-observed")} else if reception_start.is_some_and(|n|native>=n+1200) {Some("reception-window-ended")} else if phase=="Complete" {Some("step-complete")} else if phase=="Cancelled" {Some("step-cancelled")}
                    else if o["ownership"]["last_application"]["owner"]!="step" {Some("step-ownership-lost")}
                    else if matches!(s.motion,lh_contracts::Motion::Falling|lh_contracts::Motion::Fallen|lh_contracts::Motion::Recovering|lh_contracts::Motion::Halted) {Some("step-fall-or-halt")} else {None};
                if let Some(reason)=reason {first_stop=Some(reason.into());return Err(Failure::new(s.tick,substep,"continuation-stop",reason.into(),0.,0.));}
                Ok(())
            }).map_err(|e|format!("runtime rejected: {e:?}"))?;
            if let Some(f)=reply.failure {if first_stop.is_none(){first_stop=Some(format!("{}: {}",f.check,f.offending));}break;}
            if reply.completed_ticks==0 {first_stop=Some("runtime-no-progress".into());break;}
        }
        Ok(())
    })();
    if let Err(e)=outcome {if first_stop.is_none(){first_stop=Some(e);}}
    let final_observation:Value=serde_json::from_str(&runtime.getup_native_observation()?).map_err(|e|e.to_string())?;
    if final_observation["nativeCompleted"].as_u64()!=Some(last_pose_native) {write_pose(&mut poses,&final_observation,&ids)?;}
    writeln!(progress,"{}",json!({"finalNative":final_observation["nativeCompleted"],"ledger":budget::snapshot()?,"firstStop":first_stop})).and_then(|_|progress.sync_all()).map_err(|e|e.to_string())?;
    let stop=checkpoint::save_compressed_checkpoint_bounded(&runtime,&out.with_extension("stop.bin.gz"),800_000);
    Ok(json!({"preflight":preflight,"integrated":budget::snapshot()?.real>0,"firstStop":first_stop,
        "receptionStartedNative":reception_start,"receptionDeadlineNative":reception_start.map(|n|n+1200),"originalDeadlineCrossedNative":old_deadline_crossed,"maximumContinuousStableSeconds":max_stable_intervals as f64*0.00125,"oneSecondStableObserved":max_stable_intervals>=800,"stabilityLast":stability_last,"nominalPlacementSuccess":false,"poses":out.with_extension("poses.bin"),"initialCom":initial_com,"initialComVelocityMps":before["comVelocityMps"],"measuredTrace":if placement.is_some(){json!("durable progress.jsonl")}else{json!(trace)},"minimumLoadSample":minimum_load_sample,"refreshTrace":"omitted; poses/progress saved durably","firstSeparation":separation,"firstLanding":landing,"maximumWholeSoleClearanceM":max_clearance,"minimumFreeLoadN":min_free_load,
        "positiveClearanceSamples":positive_clearance_samples,"maximumPositiveClearanceRunSeconds":maximum_positive_clearance_run as f64*0.00125,"confirmedSeparationSamples":confirmed_separation_samples,"observedSamples":samples,"brakeExit":brake_exit,"maximumRetreatM":max_retreat,"maximumLiftM":max_lift,"separatedSeen":separated_seen,"landingSeen":landed_seen,"lastMeasuredSupport":extrema,
        "stopCheckpoint":stop.as_ref().ok(),"stopCheckpointError":stop.as_ref().err(),
        "commands":budget::step_command_diagnostics(),
        "finalObservation":runtime.getup_native_observation().ok().and_then(|s|serde_json::from_str::<Value>(&s).ok())}))
}
fn main()->Result<(),Box<dyn std::error::Error>> {
    let args:Vec<_>=std::env::args().collect();
    if !(args.len()==4 || args.len()==5) || !matches!(args[1].as_str(),"--preflight-stop"|"--resume-stop") {
        return Err("usage: getup_native_step_continue --preflight-stop|--resume-stop ENTRY.bin NEW_RESULT.json".into());
    }
    let mut file=OpenOptions::new().write(true).create_new(true).open(&args[3])?;
    file.write_all(b"{\"status\":\"started; final outcome pending\"}\n")?;file.sync_all()?;
    if args.len()==5 {budget::install_post_separation_025()?;}else{budget::install_reception010_replay()?;}
    let result=run(Path::new(&args[2]),Path::new(&args[3]),args[1]=="--preflight-stop",args.get(4).map(|p|Path::new(p)));
    let ledger=budget::snapshot()?;
    let mut receipt=json!({"format":"reception024-captured-leg-comparison-from-6186-v1","historicalFlightController":"e5151e23f3c1401abac9aa993364a7a88d30505b","receptionImplementation":"local/reception-010-fix; see build identity","historical":34945,"trialLimit":2000,
        "cumulativeLimit":39600,"totalConsumed":34945+ledger.real+ledger.forecast,
        "remainingGlobalBudget":4655-ledger.real-ledger.forecast,"ledger":ledger,
        "inputCheckpoint":args[2],"result":result.as_ref().ok(),"error":result.as_ref().err(),"motionQualified":false});
    if args.len()==5 {
        receipt["format"]=json!("post-separation-placement-trial025-v1");receipt["historical"]=json!(36249);
        receipt["trialLimit"]=json!(12450);receipt["cumulativeLimit"]=json!(48699);receipt["totalConsumed"]=json!(36249+ledger.real+ledger.forecast);
        receipt["remainingGlobalBudget"]=json!(12450-ledger.real-ledger.forecast);
    }
    let bytes=serde_json::to_vec(&receipt)?;
    if bytes.len()>750_000 {return Err("result exceeds reserved 750000 bytes".into());}
    file.seek(SeekFrom::Start(0))?;file.write_all(&bytes)?;file.set_len(bytes.len() as u64)?;file.sync_all()?;
    println!("{}",json!({"ledger":ledger,"totalConsumed":receipt["totalConsumed"],"error":receipt["error"],"firstStop":receipt["result"]["firstStop"],"output":args[3]}));Ok(())
}
