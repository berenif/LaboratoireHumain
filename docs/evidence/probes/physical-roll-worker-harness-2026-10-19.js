// Inert copied-worker source fragment. Inserted only by the gated isolated maker.
const rollContract = __CONTRACT_JSON__;
const approvedFixtureProfile = __PROFILE_JSON__;
const rollPrefixReference = __PREFIX_REFERENCE_JSON__;
const canonical = value => Array.isArray(value) ? value.map(canonical) : value && typeof value === 'object'
  ? Object.fromEntries(Object.keys(value).sort().map(k => [k, canonical(value[k])])) : value;
const sameJson = (a,b) => JSON.stringify(canonical(a)) === JSON.stringify(canonical(b));
const eventCounters = counters => Object.fromEntries(['falls','recoveries','steps','strikes'].map(k=>[k,counters[k]]));
const rawError = error => ({ name:error?.name ?? null, message:String(error), stack:error?.stack ?? null });
const subset = (value,keys) => Object.fromEntries(keys.map(key => {
  if (!Object.hasOwn(value,key)) throw new Error('Missing frozen prefix field '+key);
  return [key,value[key]];
}));
async function digestJson(value) {
  const bytes = new TextEncoder().encode(JSON.stringify(canonical(value)));
  return Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes))).map(v=>v.toString(16).padStart(2,'0')).join('');
}
let carryActivationEvidence = null;
let rollNativeTrace = [], rollTraceError = null, rollNative0State = null, rollBuildIdentity = null, rollLastRawReply = null;
let rollTerminal = null, rollLastNative = 0, rollFirstPassNative = null, rollHoldEndNative = null;
let rollAnchors = null, rollFirstPassCounters = null, rollFullStandingSamples = 0, rollLoadedMaterialTravelM = [0,0], rollLastUprightTicks = 0;
let rollLatestFixture = null, rollNative0Controller = null, rollProneEntryNative = null;
const rollPrefixEvidence = { native0match:false, matchedNativeRows:0, verifiedThroughNative:0,
  completeBeforeRoll481:false, comparator:'SHA256 canonical JSON of exact frozen common fields; raw carry JSON string preserved',
  baselineArchiveSha256:rollContract.baselinePreparation.sha256, native0BaselineStamp:rollPrefixReference.native0BaselineStamp,
  native0ActualStamp:null, native0StampAccounting:null, triggerReadyAt480:false };
function stopRoll(reason,error=null) {
  if (rollTerminal) return;
  // Latch before any fallible report. No later advance can reach integration.
  rollTerminal = {reason,native:rollLastNative,decisiveNative:rollLastNative,publicationStopNative:null,uncreditedNativeTail:null,firstPassNative:rollFirstPassNative,holdEndNative:rollHoldEndNative,
    loadedMaterialTravelM:[...rollLoadedMaterialTravelM],stopController:null,stopFixture:null,
    error:error ? rawError(error) : null,reportErrors:[],tailObservations:[],tailPredicateOrCounterFailures:[]};
  try { rollTerminal.stopFixture = JSON.parse(runtime.getup_fixture_report()); rollTerminal.native=rollTerminal.stopFixture.nativeSubsteps; rollTerminal.publicationStopNative=rollTerminal.native; rollTerminal.uncreditedNativeTail=rollTerminal.native-rollTerminal.decisiveNative; }
  catch (error) { rollTerminal.reportErrors.push({operation:'getup_fixture_report',...rawError(error)}); }
  try { rollTerminal.stopController = JSON.parse(runtime.getup_report()); }
  catch (error) { rollTerminal.reportErrors.push({operation:'getup_report',...rawError(error)}); }
}
async function verifyRollPrefix(rows) {
  for (const row of rows) {
    if (row.nativeSubsteps > rollContract.triggerNative) continue;
    const n=row.nativeSubsteps;
    if (n !== rollPrefixEvidence.verifiedThroughNative+1) throw new Error('Passive prefix native coverage gap');
    const actual=await digestJson(subset(row,rollContract.baselinePreparation.nativeFields));
    if (actual !== rollPrefixReference.nativeSha256[n]) throw new Error('Frozen passive physical/current/carry prefix mismatch at native'+n);
    rollPrefixEvidence.verifiedThroughNative=n; rollPrefixEvidence.matchedNativeRows++;
  }
}
function inspectRollNativeRows(rows) {
  for (const row of rows) {
    const n=row.nativeSubsteps;
    if (n !== rollLastNative+1 || row.commandInputNativeSubstep !== n-1 || row.current?.nativeSubsteps !== n
      || row.current.physicsIntegrations !== 0 || row.poseWords?.length !== 325 || row.physicalObservationWords?.length !== 67
      || row.motorApplication?.commandInputNativeSubstep !== n-1 || row.motorApplication.overrideApplied !== false
      || row.motorApplication.overriddenActuatorIndices?.length !== 0
      || row.directPelvisForceN !== 0 || row.directPelvisTorqueNm !== 0) throw new Error('Full native observer/accounting/authority failure at'+n);
    rollLastNative=n;
    if(!Number.isSafeInteger(row.counters?.upright_ticks)||row.counters.upright_ticks<rollLastUprightTicks)throw new Error('Original upright tick counter decreased');
    rollLastUprightTicks=row.counters.upright_ticks;
    if (n>480 && !rollPrefixEvidence.completeBeforeRoll481) throw new Error('Roll advanced without complete passive prefix gate');
    const policy=row.getup?.policy;
    if (!policy) throw new Error('Missing original source policy receipt');
    if(policy.phase==='physical-roll-PassiveProneSettle' && rollProneEntryNative===null) rollProneEntryNative=n;
    if(rollProneEntryNative!==null && n>rollProneEntryNative && ['physical-roll-PassiveProneSettle','physical-roll-Admitted'].includes(policy.phase)
      && !row.motorApplication.appliedControls.every(v=>v===0)) throw new Error('Passive prone interval used a nonzero motor');
    const keys=Object.keys(row.current.predicates).sort();
    if (!sameJson(keys,[...rollContract.predicateKeys].sort())) throw new Error('Original ten current predicate schema changed');
    for(const key of rollContract.predicateKeys) {
      const value=row.current.predicates[key];
      if (typeof value !== 'boolean' && !(key==='footDriftFromSuppliedAnchors' && value===null && row.originalStableFeet===null)) throw new Error('Invalid current predicate '+key);
    }
    const full=rollContract.predicateKeys.every(k=>row.current.predicates[k]===true);
    rollFullStandingSamples=full ? rollFullStandingSamples+1 : 0;
    if(rollTerminal) {
      const tail={native:n,standingPassed:policy.prototype_standing_passed,currentPredicates:row.current.predicates,counters:row.counters,originalAnchorsEqual:rollAnchors===null?null:sameJson(row.originalStableFeet,rollAnchors),countersEqual:rollFirstPassCounters===null?null:sameJson(eventCounters(row.counters),rollFirstPassCounters)};
      rollTerminal.tailObservations.push(tail);
      if(!full || (rollFirstPassNative!==null && (policy.prototype_standing_passed!==true || tail.originalAnchorsEqual!==true || tail.countersEqual!==true)))rollTerminal.tailPredicateOrCounterFailures.push(tail);
      continue;
    }
    if (rollFirstPassNative===null && policy.prototype_standing_passed) {
      if (!full || rollFullStandingSamples<800 || n>20480 || !Array.isArray(row.originalStableFeet) || row.originalStableFeet.length!==2)
        throw new Error('Original firstpass lacks800 complete consecutive current predicates inside original deadline');
      rollFirstPassNative=n; rollHoldEndNative=n+24000; rollAnchors=structuredClone(row.originalStableFeet); rollFirstPassCounters=eventCounters(row.counters);
    }
    if (rollFirstPassNative!==null && n>=rollFirstPassNative && n<=rollHoldEndNative) {
      let holdFailure=(!full || !sameJson(row.originalStableFeet,rollAnchors) || !sameJson(eventCounters(row.counters),rollFirstPassCounters))
        ? 'standing-hold-current-predicate-original-anchor-or-counter-failure' : null;
      // Finish BOTH feet of the decisive native before latching its receipt.
      for(let i=0;i<2;i++) {
        const foot=row.current.feet[i],anchor=rollAnchors[i];
        if(!foot || Math.hypot(foot.x-anchor.x,foot.y-anchor.y,foot.z-anchor.z)>0.01)holdFailure??='standing-hold-original-foot-origin-drift';
        if(n>rollFirstPassNative) {
          const contact=row.current.contactMotion?.feet?.[i],speed=contact?.impulseWeightedTangentSpeedMps;
          if(contact?.body!==['left_foot','right_foot'][i] || contact.loaded!==true || !Number.isFinite(speed) || speed<0)
            holdFailure??='standing-hold-missing-loaded-material-motion';
          else rollLoadedMaterialTravelM[i]+=speed/800;
          if(rollLoadedMaterialTravelM[i]>0.02)holdFailure??='standing-hold-loaded-material-travel-exceeded';
        }
      }
      if(holdFailure){stopRoll(holdFailure);continue;}
      if(n===rollHoldEndNative){stopRoll('thirty-second-original-anchored-hold-complete');continue;}
    }
    if(policy.phase==='physical-roll-Rejected'){stopRoll('physical-roll-preparation-rejected');continue;}
    if (policy.refusal) {stopRoll('original-controller-refusal');continue;}
    if (n>=20480 && rollFirstPassNative===null) {stopRoll('original-25-second-recovery-deadline');continue;}
  }
}
function readRollPublication(reply) {
  rollLatestFixture=JSON.parse(runtime.getup_fixture_report());
  const f=rollLatestFixture,n=Math.round(reply.snapshot.simulation_time_s*800),r=f.physicalRoll;
  if(f.nativeSubsteps!==n || !sameJson(f.profile,approvedFixtureProfile)
    || ![20,32].includes(f.solverBudget?.currentInternalPgsIterations)
    || ![20,32].includes(f.solverBudget?.selectedNextInternalPgsIterations)) throw new Error('Frozen complete profile/native solver receipt mismatch');
  const unit=f.rootRotationNormalization;
  if(unit?.algorithm!=='unit-free-root-v1' || unit.enabled!==true || unit.validationError!==null || !sameJson(unit.root,[0,0])
    || !Number.isFinite(unit.rootNormSquared) || Math.abs(unit.rootNormSquared-1)>.001) throw new Error('Actual unit-root normalization receipt invalid');
  if(r) {
    if(r.state.route!=='right-flank-tuck-uncoil-v1' || r.state.startNative!==480 || r.state.capNative!==4000
      || r.impactNative!==480 || r.recoveryDeadlineNative!==20480 || ![0,1].includes(r.sourceForecastsGenerated)) throw new Error('Original roll route/impact/deadline/source count changed');
    if(r.state.phase==='rejected' || r.state.rejectionNative!==null || r.state.rejection!==null) stopRoll('physical-roll-preparation-rejected');
    if(r.state.phase!=='admitted' && n>=r.state.startNative+r.state.capNative) stopRoll('physical-roll-preparation-cap');
    if(r.state.admissionNative===null && (r.sourceStartNative!==null || r.sourceForecastsGenerated!==0)) throw new Error('Original source started before physical prone admission');
    if(r.sourceStartNative!==null && (r.sourceStartNative<r.state.admissionNative || r.sourceForecastsGenerated!==1)) throw new Error('Original source forecast admission/count violation');
    if(!rollTerminal && rollFirstPassNative!==null && (r.sourceForecastsGenerated!==1 || !sameJson(r.standingAnchors,rollAnchors)
      || r.standingPassed!==true || r.stableStandingNativeSamples<800)) throw new Error('Original controller anchors/streak changed after firstpass');
  } else if(n>480) {stopRoll('physical-roll-summary-missing-after-trigger');}
  return { isolatedTestOnly:true,kind:fixedFixture,profile:fixedFixtureProfile,preparationPollingNative:80,
    triggerNative,triggerEvidence,carryActivationEvidence,rollNativeTrace,rollTraceError,rollNative0State,rollBuildIdentity,
    physicalRoll:r??null,fixtureSolverBudget:f.solverBudget,fixtureRootRotationPolicy:unit,
    rollNative0Controller,rollPrefixEvidence:{...rollPrefixEvidence},rollTerminal };
}
