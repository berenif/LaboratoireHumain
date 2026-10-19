// SOURCE ONLY. Pure bounded-memory accounting; importing this file never runs physics.
import assert from 'node:assert/strict';
import { createReadStream, readFileSync, writeFileSync } from 'node:fs';
import { createGunzip } from 'node:zlib';
import { createInterface } from 'node:readline';
import { fileURLToPath } from 'node:url';
import { resolve } from 'node:path';

export const PREDICATES = Object.freeze(['head','torso','bothFeetLoadedAndPersistent','footLoadFraction','onlyFeet',
  'comInSupport','captureInSupport','linearMotion','angularMotion','footDriftFromSuppliedAnchors']);
const canonical = v => Array.isArray(v) ? v.map(canonical) : v && typeof v === 'object'
  ? Object.fromEntries(Object.keys(v).sort().map(k => [k,canonical(v[k])])) : v;
const equal = (a,b) => JSON.stringify(canonical(a)) === JSON.stringify(canonical(b));
const vector = v => v && ['x','y','z'].every(k => Number.isFinite(v[k]));
const array = (v,n) => Array.isArray(v) && v.length === n && v.every(Number.isFinite);
const words = (v,n) => array(v,n) && v.every(x => Number.isInteger(x) && x >= 0 && x <= 0xffffffff);
const distance = (a,b) => Math.hypot(a.x-b.x,a.y-b.y,a.z-b.z);

export function createPhysicalRollAnalyzer(C) {
  for (const [k,v] of Object.entries({nativeHz:800,nativePerPublication:20,triggerNative:480,
    recoveryDeadlineNative:20480,rollCapNative:4000,standingSamples:800,holdNative:24000})) assert.equal(C[k],v,k);
  assert.deepEqual([...C.predicateKeys].sort(),[...PREDICATES].sort());
  const r = { schema:1, observationValid:false, completeCasePassed:false, physicalExecutabilityEstablished:false,
    productionRecoveryEnabled:false, releaseAccepted:false, performanceClaim:false,
    native0Recorded:false, completedNativeRows:0, publications:0,lastNative:0,firstPassNative:null,holdEndNative:null,
    consecutiveFullStandingSamples:0,maximumConsecutiveFullStandingSamples:0,holdObservedNativeSamples:0,
    driftUnavailableNativeSamples:0,firstCurrentPredicateFailure:{},firstHoldFailure:null,originalStandingAnchors:null,
    originalAnchorMaximumDriftM:[0,0],loadedMaterialTravelM:[0,0],unavailableLoadedMaterialRows:[0,0],
    loadedMaterialRows:[0,0],firstPassEventCounters:null,lastUprightTicks:null,terminalTailObservations:[],terminalTailPredicateOrCounterFailures:[],holdSecondSummaries:[],
    counterContinuity:'Recovery/fall/strike/step event counters remain at first pass. upright_ticks is monotone native tick accounting and may increase normally.',
    materialTravelSemantics:'For each loaded foot, current.contactMotion.feet.impulseWeightedTangentSpeedMps / 800, strictly after first pass; unavailable values never receive zero-slip credit.',
    driftSemantics:'Null before original standing anchors is unavailable. Hold uses frozen first-pass controller anchors, with strict 0.01 m displacement bound.',
    finalPhysicalRoll:null,sourceForecastsGenerated:0,rollPrefixEvidence:null,terminal:null,
    cumulativeIntegrityMaxima:{maxAnchorSeparationM:0,maxFloorPenetrationM:0,maxSelfPenetrationM:0,maxLimitErrorRad:0},
    integritySemantics:'Native cumulative maxima are retained as readbacks, not treated as interval-local measurements.',
    noRetry:true,noReset:true,sameWorld:true };
  let streakAnchors = null, previousPolicyStart = null, rollStart = null, lastRollPhase = null, native0 = null;
  let previousNativePolicyPhase = null, passiveZeroIntervals = 0;
  const holdFailure = (n,reason) => { r.firstHoldFailure ??= {native:n,reason}; };
  const checkCurrent = (d,n) => {
    assert.equal(d?.nativeSubsteps,n); assert.equal(d.physicsIntegrations,0);
    assert.deepEqual(Object.keys(d.predicates).sort(),[...PREDICATES].sort());
    for (const key of PREDICATES.slice(0,9)) assert.equal(typeof d.predicates[key],'boolean',key);
    assert.ok(d.predicates.footDriftFromSuppliedAnchors === null || typeof d.predicates.footDriftFromSuppliedAnchors === 'boolean');
    assert.ok(array(d.loadsN,25) && array(d.supportPersistence,25));
    assert.ok(Array.isArray(d.footIds) && d.footIds.length === 2 && new Set(d.footIds).size === 2
      && d.footIds.every(x => Number.isInteger(x) && x >= 0 && x < 25));
    assert.ok(Array.isArray(d.feet) && d.feet.length === 2 && d.feet.every(vector));
    assert.ok(Array.isArray(d.motion) && d.motion.length === 25 && d.motion.every(x => Number.isFinite(x.linearMps) && Number.isFinite(x.angularRadps)));
    const contact = d.contactMotion;
    assert.equal(contact?.physicsIntegrations,0); assert.equal(contact.velocitySource,'current-generalized-motion');
    assert.equal(contact.feet.length,2); assert.deepEqual(contact.feet.map(f => f.body).sort(),['left_foot','right_foot']);
    for (const [i,body] of ['left_foot','right_foot'].entries()) {
      const f = contact.feet.find(x => x.body === body);
      assert.equal(f.loadN,d.loadsN[d.footIds[i]]); assert.ok(Number.isFinite(f.loadN) && f.loadN >= 0 && vector(f.origin));
      assert.ok(vector(f.comVelocityMps) && vector(f.angularVelocityRadps) && Array.isArray(f.contacts));
      assert.ok(f.impulseWeightedTangentSpeedMps === null || Number.isFinite(f.impulseWeightedTangentSpeedMps) && f.impulseWeightedTangentSpeedMps >= 0);
    }
  };
  return {
    consumeNative0(state,snapshot) {
      assert.equal(r.native0Recorded,false); assert.equal(state?.nativeSubsteps,0); assert.equal(state.physicsIntegrations,0);
      assert.equal(state.physicalObservationError,null); assert.equal(state.integratorReadError,null);
      assert.ok(words(state.poseWords,325) && words(state.physicalObservationWords,67));
      assert.equal(typeof state.rootTranslationIntegratorStateJson,'string'); checkCurrent(state.current,0);
      assert.equal(snapshot.simulation_time_s,0); assert.equal(snapshot.stamp.generation,1);
      native0 = { poseWords:state.poseWords,physicalObservationWords:state.physicalObservationWords,
        rootTranslationIntegratorStateJson:state.rootTranslationIntegratorStateJson,current:state.current };
      r.native0Recorded = true;
    },
    consumeRows(rows,terminal = null) {
      assert.ok(r.native0Recorded); assert.ok(Array.isArray(rows) && rows.length <= 20);
      for (const row of rows) {
        const n = row.nativeSubsteps, d = row.current, p = row.getup?.policy;
        assert.equal(n,r.lastNative+1,'Native gap, duplicate or reordering');
        assert.equal(row.commandInputNativeSubstep,n-1); assert.equal(row.physicalSeconds,n/800);
        assert.ok(words(row.poseWords,325) && words(row.physicalObservationWords,67)); checkCurrent(d,n);
        assert.equal(typeof row.rootTranslationIntegratorStateJson,'string');
        assert.equal(row.directPelvisForceN,0); assert.equal(row.directPelvisTorqueNm,0);
        assert.ok(row.getup?.learned_policy_loaded && p && row.counters && typeof row.motion === 'string');
        const eventCounters = Object.fromEntries(['falls','recoveries','steps','strikes'].map(k => {
          assert.ok(Number.isSafeInteger(row.counters[k]) && row.counters[k] >= 0);return [k,row.counters[k]];
        }));
        assert.ok(Number.isSafeInteger(row.counters.upright_ticks) && row.counters.upright_ticks >= (r.lastUprightTicks ?? 0));
        r.lastUprightTicks = row.counters.upright_ticks;
        assert.ok(array(row.getup.motor_controls,21) && row.getup.motor_controls.every(v => Math.abs(v) <= 1));
        assert.ok(array(row.getup.joint_angles_rad,21) && array(row.getup.joint_velocities_radps,21));
        const a = row.motorApplication;
        assert.ok(a && array(a.intendedControls,21) && array(a.appliedControls,21)
          && array(a.intendedTorqueNm,21) && array(a.appliedTorqueNm,21));
        assert.equal(a.commandInputNativeSubstep,n-1); assert.equal(a.overrideApplied,false);
        assert.deepEqual(a.overriddenActuatorIndices,[]); assert.deepEqual(a.intendedControls,row.getup.motor_controls);
        assert.deepEqual(a.appliedControls,a.intendedControls); assert.deepEqual(a.appliedTorqueNm,a.intendedTorqueNm);
        assert.ok(Number.isFinite(row.requestedMotorEffortNm) && row.requestedMotorEffortNm >= 0 && row.requestedMotorEffortNm <= 920.001);
        assert.ok(Math.abs(a.intendedTorqueNm.reduce((s,x)=>s+Math.abs(x),0)-row.requestedMotorEffortNm) <= 0.001);
        if (String(p.phase).startsWith('physical-roll-')) {
          assert.ok(a.appliedControls.every(x => Math.abs(x) <= Math.fround(0.85)));
          assert.ok(row.requestedMotorEffortNm <= 782.001);
        }
        if (previousNativePolicyPhase === 'physical-roll-PassiveProneSettle') {
          assert.ok(a.appliedControls.every(x => x === 0),'Passive-prone quiet samples require actual zero-motor intervals');
          passiveZeroIntervals++;
        } else if (p.phase !== 'physical-roll-Admitted') passiveZeroIntervals = 0;
        if (p.phase === 'physical-roll-Admitted' && previousNativePolicyPhase !== p.phase) {
          assert.ok(passiveZeroIntervals >= 120,'Admission precedes120 zero-motor intervals after prone entry');
        }
        previousNativePolicyPhase = p.phase;
        for (const k of Object.keys(r.cumulativeIntegrityMaxima)) {
          assert.ok(Number.isFinite(row.integrityLimits?.[k]) && row.integrityLimits[k] >= r.cumulativeIntegrityMaxima[k]);
          r.cumulativeIntegrityMaxima[k] = row.integrityLimits[k];
        }
        if (Number.isFinite(p.started_at_s)) {
          previousPolicyStart ??= p.started_at_s; assert.equal(p.started_at_s,previousPolicyStart,'Source epoch renewed');
        }
        if (n > 480 && Number.isFinite(p.since_impact_s)) assert.equal(n-Math.round(p.since_impact_s*800),480,'Original impact epoch renewed');
        const anchors = row.originalStableFeet;
        if (anchors === null) {
          assert.equal(d.footDriftM,null); assert.equal(d.predicates.footDriftFromSuppliedAnchors,null);
          r.driftUnavailableNativeSamples++;
        } else {
          assert.ok(Array.isArray(anchors) && anchors.length === 2 && anchors.every(vector) && array(d.footDriftM,2));
          for (let i=0;i<2;i++) assert.ok(Math.abs(distance(anchors[i],d.feet[i])-d.footDriftM[i]) <= 2e-6);
        }
        const credit = n <= (terminal?.decisiveNative ?? Infinity)
          && n <= (r.holdEndNative ?? Infinity) && r.firstHoldFailure === null;
        const all = PREDICATES.every(k => d.predicates[k] === true);
        for (const k of PREDICATES) if (d.predicates[k] === false) r.firstCurrentPredicateFailure[k] ??= n;
        if (credit && all) {
          assert.ok(anchors); if (streakAnchors) assert.ok(equal(anchors,streakAnchors),'Original anchors slide inside standing interval');
          else streakAnchors = structuredClone(anchors);
          r.consecutiveFullStandingSamples++;
        } else if (credit) { r.consecutiveFullStandingSamples = 0; streakAnchors = null; }
        r.maximumConsecutiveFullStandingSamples = Math.max(r.maximumConsecutiveFullStandingSamples,r.consecutiveFullStandingSamples);
        if (credit && p.prototype_standing_passed && r.firstPassNative === null) {
          assert.ok(r.consecutiveFullStandingSamples >= 800 && n <= 20480,'Original800-consecutive/25-second criterion not met');
          r.firstPassNative = n; r.holdEndNative = n+24000; r.originalStandingAnchors = structuredClone(anchors);
          assert.equal(eventCounters.recoveries,1,'Exactly one original recovery credit required');
          r.firstPassEventCounters = structuredClone(eventCounters);
        }
        if (credit && r.firstPassNative !== null && n > r.firstPassNative) {
          r.holdObservedNativeSamples++;
          if (!all) holdFailure(n,'One or more of the ten current predicates failed or became unavailable');
          if (!equal(anchors,r.originalStandingAnchors)) holdFailure(n,'Original first-pass anchors changed or disappeared');
          if (!equal(eventCounters,r.firstPassEventCounters) || p.prototype_standing_passed !== true) holdFailure(n,'Original recovery/fall/strike/step continuity changed');
          for (const [i,body] of ['left_foot','right_foot'].entries()) {
            const drift = distance(d.feet[i],r.originalStandingAnchors[i]);
            r.originalAnchorMaximumDriftM[i] = Math.max(r.originalAnchorMaximumDriftM[i],drift);
            if (drift > 0.01) holdFailure(n,body+' original anchor displacement exceeds0.01m');
            const f = d.contactMotion.feet.find(x => x.body === body);
            if (f.loadN >= 3 && f.loaded === true) {
              r.loadedMaterialRows[i]++;
              if (f.impulseWeightedTangentSpeedMps === null) { r.unavailableLoadedMaterialRows[i]++; holdFailure(n,body+' loaded material speed unavailable'); }
              else r.loadedMaterialTravelM[i] += f.impulseWeightedTangentSpeedMps/800;
              if (r.loadedMaterialTravelM[i] > 0.02) holdFailure(n,body+' loaded material travel exceeds0.02m');
            } else holdFailure(n,body+' is not currently loaded throughout standing hold');
          }
          if ((n-r.firstPassNative)%800 === 0) {
            assert.ok(r.holdSecondSummaries.length < 30);
            r.holdSecondSummaries.push({holdSecond:(n-r.firstPassNative)/800,native:n,current:structuredClone(d),
              originalStableFeet:structuredClone(anchors),counters:structuredClone(row.counters),
              allTenCurrentPredicatesPassed:all,loadedMaterialTravelM:[...r.loadedMaterialTravelM]});
          }
        }
        if (terminal && n > terminal.decisiveNative) {
          const tail={native:n,standingPassed:p.prototype_standing_passed,currentPredicates:structuredClone(d.predicates),
            counters:structuredClone(row.counters),originalAnchorsEqual:r.originalStandingAnchors===null?null:equal(anchors,r.originalStandingAnchors),
            eventCountersEqual:r.firstPassEventCounters===null?null:equal(eventCounters,r.firstPassEventCounters)};
          r.terminalTailObservations.push(tail);
          if (!all || r.firstPassNative !== null && (p.prototype_standing_passed !== true || tail.originalAnchorsEqual !== true || tail.eventCountersEqual !== true)) r.terminalTailPredicateOrCounterFailures.push(tail);
        }
        r.lastNative = n; r.completedNativeRows++;
      }
    },
    consumePublication(s) {
      assert.equal(s.native,r.lastNative); r.publications++;
      assert.equal(s.rollTraceError,null); assert.equal(s.stamp.generation,1);
      const budget = s.fixtureSolverBudget;
      assert.equal(typeof budget?.sourceRecoveryActive,'boolean');
      const fallPhase = ['Falling','Fallen'].includes(s.motion) || s.motion === 'Recovering' && budget.sourceRecoveryActive;
      assert.equal(budget.selectedNextInternalPgsIterations,fallPhase ? 32 : 20);
      assert.ok([20,32].includes(budget.currentInternalPgsIterations));
      const root = s.fixtureRootRotationPolicy;
      assert.equal(root?.algorithm,'unit-free-root-v1'); assert.equal(root.enabled,true); assert.equal(root.validationError,null);
      assert.deepEqual(root.root,[0,0]); assert.ok(Number.isFinite(root.rootNormSquared));
      const prefix = s.rollPrefixEvidence;
      if (prefix) {
        assert.equal(prefix.native0match,true); assert.equal(prefix.matchedNativeRows,Math.min(s.native,480));
        assert.equal(prefix.verifiedThroughNative,Math.min(s.native,480));
        if (s.native > 480) assert.equal(prefix.completeBeforeRoll481,true);
        r.rollPrefixEvidence = structuredClone(prefix);
      }
      if (s.native > 480) assert.ok(prefix && prefix.completeBeforeRoll481,'No roll481 without exact passive native0..480 prefix');
      const roll = s.physicalRoll;
      if (roll !== null) {
        const state = roll.state;
        rollStart ??= state.startNative; assert.equal(state.startNative,480); assert.equal(rollStart,state.startNative);
        assert.equal(state.capNative,4000); assert.equal(state.route,'right-flank-tuck-uncoil-v1');
        assert.equal(roll.impactNative,480); assert.equal(roll.recoveryDeadlineNative,20480);
        assert.ok(['tuck-roll-right','lateral-uncoil','passive-prone-settle','admitted','rejected'].includes(state.phase));
        if (lastRollPhase === 'admitted') assert.equal(state.phase,'admitted','Roll repeated after admission');
        lastRollPhase = state.phase;
        assert.ok([0,1].includes(roll.sourceForecastsGenerated));
        if (state.admissionNative === null) { assert.equal(roll.sourceForecastsGenerated,0); assert.equal(roll.sourceStartNative,null); }
        if (roll.sourceStartNative !== null) {
          assert.equal(state.phase,'admitted'); assert.ok(roll.sourceStartNative >= state.admissionNative);
          assert.equal(roll.sourceForecastsGenerated,1);
        }
        if (state.phase === 'admitted') {
          assert.ok(state.lateralNative >= 480 && state.proneNative >= state.lateralNative && state.admissionNative >= state.proneNative+120);
          assert.ok(state.admissionNative <= 4480); assert.equal(state.quietProneSamples,120);
        }
        r.sourceForecastsGenerated = roll.sourceForecastsGenerated; r.finalPhysicalRoll = structuredClone(roll);
      } else assert.ok(s.native <= 480,'Physical roll summary vanished after trigger');
      if (s.rollTerminal !== null) {
        assert.equal(r.terminal,null); r.terminal = structuredClone({...s.rollTerminal,stopController:undefined,stopFixture:undefined});
      }
    },
    finish(terminal) {
      assert.ok(r.native0Recorded && r.rollPrefixEvidence?.completeBeforeRoll481);
      assert.equal(terminal.native,r.lastNative); assert.equal(terminal.firstPassNative,r.firstPassNative);
      assert.equal(terminal.publicationStopNative,r.lastNative);
      assert.ok(Number.isInteger(terminal.decisiveNative) && terminal.decisiveNative <= r.lastNative);
      assert.equal(terminal.uncreditedNativeTail,r.lastNative-terminal.decisiveNative);
      assert.ok(terminal.uncreditedNativeTail >= 0 && terminal.uncreditedNativeTail <= 19);
      assert.equal(r.terminalTailObservations.length,terminal.uncreditedNativeTail);
      assert.equal(terminal.tailObservations.length,terminal.uncreditedNativeTail);
      assert.deepEqual(terminal.tailObservations.map(x=>x.native),r.terminalTailObservations.map(x=>x.native));
      assert.deepEqual(terminal.tailPredicateOrCounterFailures.map(x=>x.native),r.terminalTailPredicateOrCounterFailures.map(x=>x.native));
      assert.equal(terminal.holdEndNative,r.holdEndNative);
      assert.ok(array(terminal.loadedMaterialTravelM,2));
      for (let i=0;i<2;i++) assert.ok(Math.abs(terminal.loadedMaterialTravelM[i]-r.loadedMaterialTravelM[i]) <= 1e-10);
      const controller = terminal.stopController, fixture = terminal.stopFixture;
      assert.ok(controller && fixture); assert.equal(fixture.nativeSubsteps,r.lastNative);
      assert.ok(equal(fixture.physicalRoll ?? null,r.finalPhysicalRoll));
      assert.equal(controller.impact_substep,480); assert.equal(controller.started_substep,r.finalPhysicalRoll?.sourceStartNative ?? null);
      r.decisiveNative = terminal.decisiveNative; r.publicationStopNative = terminal.publicationStopNative;
      r.uncreditedNativeTail = terminal.uncreditedNativeTail;
      r.uncreditedNativeTailSemantics = 'Already-integrated suffix of the terminal ordinary20-native publication; fully archived and structurally audited, excluded from qualification credit. No subsequent advance.';
      r.completeCasePassed = terminal.error === null && r.firstPassNative !== null && r.sourceForecastsGenerated === 1
        && terminal.decisiveNative === r.holdEndNative && r.holdObservedNativeSamples === 24000 && r.firstHoldFailure === null
        && r.unavailableLoadedMaterialRows.every(x => x === 0) && r.loadedMaterialTravelM.every(x => x <= 0.02);
      r.physicalExecutabilityEstablished = r.completeCasePassed;
      const roll = r.finalPhysicalRoll;
      r.physicalSpans = {rollStartNative:roll?.state.startNative ?? null,admissionNative:roll?.state.admissionNative ?? null,
        sourceStartNative:roll?.sourceStartNative ?? null,firstPassNative:r.firstPassNative,holdEndNative:r.holdEndNative,
        rollPreparationSeconds:roll?.state.admissionNative == null?null:(roll.state.admissionNative-roll.state.startNative)/800,
        originalImpactToFirstPassSeconds:r.firstPassNative===null?null:(r.firstPassNative-480)/800,
        sourceStartToFirstPassSeconds:r.firstPassNative===null||roll?.sourceStartNative==null?null:(r.firstPassNative-roll.sourceStartNative)/800,
        creditedPostPassHoldSeconds:r.holdObservedNativeSamples/800};
      r.observationValid = true; return structuredClone(r);
    },
    snapshot() { return structuredClone(r); },
    native0PhysicalReceipt() { return structuredClone(native0); },
  };
}

// Offline replay accepts only existing records. No browser, runtime or actors.
if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  assert.equal(process.argv.length,7,'Usage: analyzer CONTRACT TRACE.ndjson.gz PUBLICATIONS.ndjson TERMINAL.json OUTPUT.json');
  const [, , contract,trace,publications,terminal,output] = process.argv;
  const C = JSON.parse(readFileSync(contract));
  assert.equal(C.executionApproved,true,'SOURCE-ONLY: execution gate remains false');
  assert.equal(C.schemaPinned,true,'SOURCE-ONLY: exact schema audit is pending');
  const a = createPhysicalRollAnalyzer(C);
  const rows = createInterface({input:createReadStream(trace).pipe(createGunzip()),crlfDelay:Infinity})[Symbol.asyncIterator]();
  const initial = await rows.next(); assert.equal(initial.done,false);
  const zero = JSON.parse(initial.value); assert.equal(zero.receiptKind,'native0-read-only-state');
  a.consumeNative0(zero,zero.snapshot);
  let native = 0;
  for await (const line of createInterface({input:createReadStream(publications),crlfDelay:Infinity})) {
    const s = JSON.parse(line);
    while (native < s.native) {
      const next = await rows.next(); assert.equal(next.done,false,'Native stream ends before publication');
      const row = JSON.parse(next.value); a.consumeRows([row],s.rollTerminal); native = row.nativeSubsteps;
    }
    a.consumePublication(s);
  }
  assert.equal((await rows.next()).done,true,'Unaccounted native tail');
  writeFileSync(output,JSON.stringify(a.finish(JSON.parse(readFileSync(terminal))),null,2)+'\n');
}
