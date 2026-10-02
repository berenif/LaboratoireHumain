import assert from 'node:assert/strict';
import { closeSync, copyFileSync, existsSync, mkdirSync, openSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { spawnSync } from 'node:child_process';
import { gzipSync, gunzipSync } from 'node:zlib';
import os from 'node:os';
import { fileURLToPath } from 'node:url';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';
import { inventory, json, replayFailure, verifyInventory } from './coordinated-standing-evidence.mjs';

export const CANDIDATE_CONTROLLERS = ['h78-v1','h79-v1','h80-v1'];
export const TRANSFER_TEST_PATTERN = 'slow pulls stay connected and stepping remains available after release and reversal|a planted reversal preserves ownership without a transient unsupported fall';
export const mergeManifest=(base,overlay)=>Object.fromEntries([...new Set([...Object.keys(base),...Object.keys(overlay)])].map(key=>{
  const left=base[key],right=overlay[key];
  return [key,left && right && typeof left==='object' && typeof right==='object' && !Array.isArray(left) && !Array.isArray(right)
    ?mergeManifest(left,right):right===undefined?left:right];
}));
export function loadManifest(path, ancestors=[]) {
  const absolute=resolve(path);
  assert.ok(!ancestors.includes(absolute),`Manifest inheritance cycle: ${[...ancestors,absolute].join(' -> ')}`);
  const source=JSON.parse(readFileSync(absolute,'utf8'));
  const inherited=source.extends?loadManifest(resolve(dirname(absolute),source.extends),[...ancestors,absolute]):null;
  return {manifest:inherited?mergeManifest(inherited.manifest,source):source,
    paths:[absolute,...(inherited?.paths??[])],source};
}
export function initialStages(controllerId) {
  return Object.fromEntries(['feasibility',...(CANDIDATE_CONTROLLERS.includes(controllerId)?['prediction-validation','transfer-regressions']:[]),
    'saved-regressions-and-faults','short-screens','official-standing','structural-integration','sustained-held-out-runtime-memory',
    ...(CANDIDATE_CONTROLLERS.includes(controllerId)?['actual-browser-runtime-memory']:[])].map(name=>[name,{status:'incomplete',reason:'Prerequisites not yet passed'}]));
}
export function childReportStatus(execution,report) {
  if(execution.error||execution.signal||execution.exit===null)return 'incomplete';
  if(!report||!['pass','fail','incomplete'].includes(report.status))return 'incomplete';
  return execution.exit!==0&&report.status==='pass'?'fail':report.status;
}
export function acceptanceStatus(stages,sourceUnchanged,manifestsUnchanged,error) {
  if(!sourceUnchanged||!manifestsUnchanged||Object.values(stages).some(s=>s.status==='fail'))return 'fail';
  return !error&&Object.values(stages).every(s=>s.status==='pass')?'pass':'incomplete';
}
export function guardedStage(stages,name,prerequisites,operation) {
  const blocked=prerequisites.filter(prerequisite=>stages[prerequisite]?.status!=='pass');
  const result=blocked.length?{status:'incomplete',reason:`Required stages have not passed: ${blocked.join(', ')}`}:operation();
  assert.ok(['pass','fail','incomplete'].includes(result.status),`Invalid stage status: ${name}`);
  stages[name]=result;return result;
}
export function verifyFreezeLock(lock,source,manifestDependencies) {
  assert.deepEqual(lock.source,source,'H78 frozen source changed; create a new version instead of changing a frozen experiment');
  const normalized=Object.fromEntries(Object.entries(lock.manifestDependencies??{}).map(([path,hash])=>[resolve(path),hash]));
  assert.deepEqual(normalized,manifestDependencies,'H78 frozen manifest chain changed');
}
export async function runAcceptance(arguments_=process.argv.slice(2)) {
arguments_=[...arguments_];
if(arguments_[0]==='--replay') {
  const directory=resolve(arguments_[1]);
  const files=JSON.parse(readFileSync(join(directory,'artifacts.json')));verifyInventory(directory,files);
  const run=JSON.parse(readFileSync(join(dirname(directory),'run-manifest.json')));
  for(const [path,hash] of Object.entries(run.dependencies))assert.equal(sha256(readFileSync(path)),hash,`Replay dependency changed: ${path}`);
  if(CANDIDATE_CONTROLLERS.includes(run.controllerId)) {
    const failure=JSON.parse(readFileSync(join(directory,'first-failure.json')));
    assert.equal(failure.controllerBefore?.candidateConfiguration?.id,run.controllerId,'Replay candidate identity differs');
    for(const [path,hash]of Object.entries({...run.source,...run.manifestDependencies}))assert.equal(sha256(readFileSync(path)),hash,`H78 replay source changed: ${path}`);
  }
  const {default:RAPIER}=await import('@dimforge/rapier3d-compat');await RAPIER.init();
  console.log(JSON.stringify(await replayFailure(directory,RAPIER)));process.exit(0);
}
let controllerId='h74-v1';
const inlineController=arguments_.findIndex(value=>value.startsWith('--controller='));
if(inlineController>=0)controllerId=arguments_.splice(inlineController,1)[0].slice('--controller='.length);
const splitController=arguments_.indexOf('--controller');
if(splitController>=0)controllerId=arguments_.splice(splitController,2)[1];
assert.ok(['h74-v1','h77-v1',...CANDIDATE_CONTROLLERS].includes(controllerId),'Unknown versioned standing controller');
const contactCandidate=CANDIDATE_CONTROLLERS.includes(controllerId);
const experimentPath=`docs/experiments/standing-${controllerId}.json`;
const loaded=loadManifest(experimentPath),m=loaded.manifest;
const baseExperimentPath=loaded.paths[1]??null;
const manifestDependencies=Object.fromEntries(loaded.paths.map(path=>[path,sha256(readFileSync(path))]));
assert.equal(m.controller.id,controllerId,'Manifest/controller selection differs');
const output=resolve(arguments_[0]??`evidence/standing-${controllerId}/${new Date().toISOString().replaceAll(/[:.]/g,'-')}`);
assert.ok(!existsSync(output),'Output must be a fresh directory; retain previous attempts');mkdirSync(output,{recursive:true});
const started=performance.now(), source=fingerprints();
let freezeLock=null;
if(contactCandidate) {
  const path=`docs/experiments/standing-${controllerId}.lock.json`;
  assert.ok(existsSync(path),'H78 requires its mandatory pretrial freeze lock');
  const contents=readFileSync(path);freezeLock={path,sha256:sha256(contents)};
  verifyFreezeLock(JSON.parse(contents),source,manifestDependencies);
  copyFileSync(path,join(output,'experiment-lock.json'));
}
const dependencies=Object.fromEntries(['node_modules/@dimforge/rapier3d-compat/package.json','node_modules/@dimforge/rapier3d-compat/dist/rapier.mjs',
  'node_modules/@dimforge/rapier3d-compat/dist/rapier_wasm3d_bg.wasm','node_modules/tsx/package.json'].map(p=>[p,sha256(readFileSync(p))]));
json(join(output,'experiment.json'),m);
copyFileSync(experimentPath,join(output,'experiment-source.json'));
if(baseExperimentPath)copyFileSync(baseExperimentPath,join(output,'experiment-base.json'));
for(const [index,path]of loaded.paths.slice(2).entries())copyFileSync(path,join(output,`experiment-ancestor-${index+2}.json`));
json(join(output,'run-manifest.json'),{started:new Date().toISOString(),experimentSha256:sha256(readFileSync(experimentPath)),
  manifestDependencies,freezeLock,resolvedExperimentSha256:sha256(readFileSync(join(output,'experiment.json'))),source,dependencies,
  command:[process.execPath,...process.execArgv,...process.argv.slice(1)],node:process.version,nodeExecutable:process.execPath,nodeExecutableSha256:sha256(readFileSync(process.execPath)),
  cpu:os.cpus()[0].model,os:`${os.type()} ${os.release()} ${os.arch()}`,rapier:'0.20.0',seed:0,
  controllerId, qualification:'Versioned before this attempt; original startup and serial child processes. Candidate promotion is prohibited unless every stage passes.'});
const preservedSource=Object.fromEntries(Object.keys(source).filter(p=>/^(src\/|scripts\/|tests\/|package|tsconfig)/.test(p)).map(p=>[p,readFileSync(p,'utf8')]));
writeFileSync(join(output,'source-snapshot.json.gz'),gzipSync(JSON.stringify(preservedSource)),{flag:'wx'});
const stages=initialStages(controllerId);
const runs=[],commands=[], preflight={};let feasibilitySteps=0;
const invoke=(name,args,environment={},maximumWallMs=Infinity)=>{
  if(contactCandidate&&!args.some(argument=>argument.startsWith('--max-old-space-size=')))args=['--max-old-space-size=384',...args];
  const remaining=m.budget.maximumAcceptanceWallS*1000-(performance.now()-started);
  if(remaining<=0)throw new Error('Total acceptance wall budget exhausted');
  const log=join(output,`${name}.log`),fd=openSync(log,'wx'),begin=performance.now();
  let result;
  try {result=spawnSync(process.execPath,args,{cwd:process.cwd(),env:{...process.env,...environment},stdio:['ignore',fd,fd],windowsHide:true,
    timeout:Math.max(1,Math.ceil(Math.min(remaining,m.budget.childTimeoutS*1000,maximumWallMs)))});}finally{closeSync(fd);}
  const record={name,command:[process.execPath,...args],environment,exit:result.status,signal:result.signal,error:result.error?.message??null,wallMs:performance.now()-begin,log};
  commands.push(record);console.log(JSON.stringify({operation:name,exit:record.exit,wallMs:record.wallMs}));return record;
};
const trial=(name,kind,heading,offset,steps,extra={})=>{
  if(['reference','local'].includes(kind)) {
    if(feasibilitySteps+steps>m.budget.maximumFeasibilitySteps || (performance.now()-started)>m.budget.maximumFeasibilityWallS*1000)throw new Error('Feasibility budget exhausted');
    feasibilitySteps+=steps;
  }
  const dir=join(output,name);mkdirSync(dir);json(join(dir,'scenario.json'),{name,kind,heading,offset,steps,...extra});
  const execution=invoke(name,['--max-old-space-size=384','--import','tsx','scripts/standing-coordinated-trial.mjs',dir],{},
    contactCandidate&&['reference','local'].includes(kind)?m.budget.maximumFeasibilityWallS*1000-(performance.now()-started):Infinity);
  const reportPath=join(dir,'report.json');
  const report=existsSync(reportPath)?JSON.parse(readFileSync(reportPath)):{status:'incomplete',reason:'Missing final report',completedTicks:0};
  if(contactCandidate)report.status=childReportStatus(execution,report);
  else if(execution.exit!==0&&report.status==='pass')report.status='fail';
  const result={name,directory:dir,execution,report};runs.push(result);return result;
};
const historicalInventory=()=>{
  const root='docs/checkpoints/2026-09-27/h71-physical-fallback',archive=JSON.parse(readFileSync(join(root,'manifest.json')));
  const dir=join(output,'saved-inputs');mkdirSync(dir);copyFileSync(join(root,'manifest.json'),join(dir,'archive-manifest.json'));
  for(const original of archive.originals) {
    const parts=original.archives.map(path=>{
      const record=archive.files.find(f=>f.archive===path),bytes=readFileSync(join(root,path));
      assert.equal(sha256(bytes),record.archiveSha256,path);const raw=gunzipSync(bytes);
      assert.equal(raw.length,record.bytes);assert.equal(sha256(raw),record.sha256);return raw;
    });
    const data=Buffer.concat(parts);assert.equal(data.length,original.bytes);assert.equal(sha256(data),original.sha256);
    const base=original.original.split(/[\\/]/).at(-1);
    if(original.original.includes('physical-fallback-authority'))writeFileSync(join(dir,base),data,{flag:'wx'});
    else if(original.original==='production source snapshot at archival time') {
      const sources=JSON.parse(data);
      for(const [path,contents]of Object.entries(sources)) {
        assert.equal(sha256(contents),archive.source[path]);
        const dest=join(dir,'original-source',path);mkdirSync(dirname(dest),{recursive:true});writeFileSync(dest,contents,{flag:'wx'});
      }
    }else writeFileSync(join(dir,original.archives[0].replaceAll(/[\\/]/g,'_').replace('.part-00001.gz','')),data,{flag:'wx'});
  }
  return {status:'pass',originals:archive.originals.length,parts:archive.files.length,archiveSha256:sha256(readFileSync(join(root,'manifest.json'))),
    qualification:'Integrity/preservation only. Native simulation regressions depend on feasibility; historical sources, inputs, outputs and prefix provenance remain separate from the new candidate.'};
};
try {
  try {preflight.historical=historicalInventory();}catch(error){preflight.historical={status:'incomplete',reason:error.message};}
  preflight.policy=invoke('policy-tests',['--test','--test-concurrency=1','tests/coordinated-standing.test.mjs',
    ...(controllerId==='h77-v1'||contactCandidate?['tests/implicit-standing.test.mjs']:[]),
    ...(contactCandidate?['tests/contact-force-standing.test.mjs','tests/standing-acceptance.test.mjs',
      ...(['h79-v1','h80-v1'].includes(controllerId)?['tests/contact-force-solve-h79.test.mjs']:[]),
      ...(controllerId==='h80-v1'?['tests/contact-force-solve-h80.test.mjs']:[])]:[])],
    preflight.historical.status==='pass'?{STANDING_OLD_SOURCE:join(output,'saved-inputs/original-source')} : {});
  if(preflight.policy.exit!==0)throw new Error('Policy/integration preflight failed');
  const faults=m.failurePolicy.injections.map((fault,i)=>trial(`fault-${i}`, 'fault', 0,0,3,{fault,faultTick:2}));
  preflight.injectedFaults={status:faults.every(r=>r.report.status==='pass')?'pass':'fail',cases:faults.length};
  if(preflight.injectedFaults.status!=='pass')throw new Error('Injected failure policy failed');
  let selected;
  for(const [i,offset]of m.reference.forwardOffsetsM.entries()) {
    const result=trial(`reference-${i}`,'reference',0,offset,m.budget.referenceStepsPerTrial);
    if(result.report.status==='pass'&&selected===undefined){selected=offset;if(!contactCandidate)break;}
  }
  let locals=[];
  if(selected!==undefined)locals=m.local.inputs.map((input,i)=>trial(`local-${i}`,'local',0,selected,m.budget.localStepsPerTrial,{inputs:[input]}));
  const feasible=selected!==undefined&&locals.every(r=>r.report.status==='pass');
  const feasibilityIncomplete=contactCandidate&&runs.some(r=>['reference','local'].includes(JSON.parse(readFileSync(join(r.directory,'scenario.json'))).kind)&&r.report.status==='incomplete');
  stages.feasibility={status:feasible?'pass':feasibilityIncomplete?'incomplete':'fail',selectedOffsetM:selected??null,allocatedSteps:feasibilitySteps,
    reason:feasible?'Reference hold and declared local return passed':'No validated reference/local return within the frozen trial budget; predictive work is not authorized by this result'};
  if(!feasible) {
    if(contactCandidate) {
      preflight.contactDiagnostic={status:'incomplete',reason:'Prediction, transfer, and browser work require a passing reference and local return. Retain the first physical failure for the bounded contact-force investigation.'};
    }else if(controllerId==='h77-v1') {
      const traces=[
        ['native-saved-state-substep-report.json','evidence/standing-h77-v1/native-substep-trace-03/report.json'],
        ['native-live-h77-substep-report.json','evidence/standing-h77-v1/native-live-substep-report-02.json'],
      ];
      const records=[];
      for(const [name,path] of traces) {
        assert.ok(existsSync(path),`Required native substep trace is missing: ${path}`);
        const report=JSON.parse(readFileSync(path));
        assert.equal(report.conclusion?.branch,'correct-joint-rows/contact-wrench',`Unexpected evidence branch: ${path}`);
        copyFileSync(path,join(output,name));
        records.push({source:path,copy:name,sha256:sha256(readFileSync(path)),branch:report.conclusion.branch});
      }
      preflight.nativeSubstepTrace={status:'pass',records,
        qualification:'Output-neutral actual Rapier substep traces selected the contact-wrench/load-distribution branch. Native/WASM parity is close but not bit-identical, so this is directional branch evidence, not standing acceptance.'};
    }else if(feasibilitySteps+4320>m.budget.maximumFeasibilitySteps || performance.now()-started>m.budget.maximumFeasibilityWallS*1000) {
      preflight.representation={status:'incomplete',reason:'Feasibility budget exhausted before H42 diagnostic'};
    }else {
      const execution=invoke('h42-representation',['scripts/probe-rigid-assembly-drift.mjs',join(output,'h42-representation.json')],
        {RIGID_MODES:'fixed-joints,fixed-multibody',RIGID_EXPORT_SNAPSHOTS:'1',RIGID_SNAPSHOT_TICK:'1'});
      feasibilitySteps+=4320;preflight.representation={status:execution.exit===0?'pass':'incomplete',execution,
        qualification:'Mechanism diagnostic execution, not articulated standing acceptance; locked fixture changes DOFs only in these separate copies.'};
    }
  }else {
    if(contactCandidate) {
      guardedStage(stages,'prediction-validation',['feasibility'],()=>{
      const predictionDirectory=join(output,'prediction-validation');mkdirSync(predictionDirectory);
      json(join(predictionDirectory,'scenario.json'),{controllerId,selectedOffsetM:selected,
        feasibility:stages.feasibility,localDirectories:locals.map(r=>r.directory)});
      const prediction=invoke('prediction-validation',['--max-old-space-size=384','--import','tsx','scripts/standing-prediction-validation.mjs',predictionDirectory]);
      const predictionPath=join(predictionDirectory,'report.json');
      const predictionReport=existsSync(predictionPath)?JSON.parse(readFileSync(predictionPath)):null;
      return {status:childReportStatus(prediction,predictionReport),execution:prediction,
        report:predictionReport,reason:predictionReport?.reason??(predictionReport?null:'Missing prediction validation report')};
      });
      if(stages['prediction-validation'].status!=='pass')throw new Error('Prediction validation did not pass; transfer acceptance remains blocked');
      guardedStage(stages,'transfer-regressions',['feasibility','prediction-validation'],()=>{
      const transfer=invoke('transfer-regressions',['--max-old-space-size=384','--test','--test-concurrency=1',
        `--test-name-pattern=${TRANSFER_TEST_PATTERN}`,'tests/balance-controller.test.mjs'],
        {STANDING_CANDIDATE:controllerId,STANDING_PREDICTION_VALIDATED:'1'});
      return {status:childReportStatus(transfer,{status:transfer.exit===0?'pass':'fail'}),execution:transfer,
        qualification:'Existing slow-pull and planted-reversal assertions, with only explicit candidate selection changed.'};
      });
      if(stages['transfer-regressions'].status!=='pass')throw new Error('Candidate transfer regressions did not pass');
    }
    if(preflight.historical.status!=='pass')throw new Error('Historical matching-source inputs unavailable');
    const old=invoke('saved-native-replays',['--max-old-space-size=384','--import','tsx','scripts/standing-saved-regressions.mjs',join(output,'saved-inputs')]);
    const saved=[['h66',0,978],['h69-plus',Math.PI/3,621],['h69-minus',-Math.PI/4,497]].map(([name,heading,steps])=>trial(`saved-${name}`,'saved-regression',heading,selected,steps));
    stages['saved-regressions-and-faults']={status:old.exit===0&&saved.every(r=>r.report.status==='pass')?'pass':'fail',originalReplay:old};
    if(stages['saved-regressions-and-faults'].status==='pass') {
      const screens=m.official.headingsRad.map((h,i)=>trial(`screen-${i}`,'screen',h,selected,720));
      stages['short-screens']={status:screens.every(r=>r.report.status==='pass')?'pass':'fail'};
      if(stages['short-screens'].status==='pass') {
        const official=m.official.headingsRad.map((h,i)=>trial(`official-${i}`,'official',h,selected,1920));
        stages['official-standing']={status:official.every(r=>r.report.status==='pass')?'pass':'fail'};
        if(stages['official-standing'].status==='pass') {
          const structural=invoke('structural-tests',['--test','--test-concurrency=1','tests/body-physics.test.mjs','tests/joint-motors.test.mjs',
            'tests/physical-chain-integrity.test.mjs','tests/character-domain.test.mjs','tests/balance-controller.test.mjs','tests/coordinated-standing.test.mjs',
            ...(controllerId==='h77-v1'||contactCandidate?['tests/implicit-standing.test.mjs']:[]),
            ...(contactCandidate?['tests/contact-force-standing.test.mjs','tests/standing-acceptance.test.mjs']:[])]);
          stages['structural-integration']={status:structural.exit===0?'pass':'fail',execution:structural,
            qualification:'Shared update order and structural regression suite; browser performance is a separate required gate.'};
          if(structural.exit===0) {
            const extended=[];
            for(let repeat=0;repeat<m.sustained.repetitions;repeat++)for(const[h,heading]of m.sustained.headingsRad.entries())
              extended.push(trial(`sustained-${repeat}-${h}`,'sustained',heading,selected,7320));
            for(let repeat=0;repeat<m.heldOut.repetitions;repeat++)for(const[h,heading]of m.heldOut.headingsRad.entries())
              extended.push(trial(`heldout-${repeat}-${h}`,'disturbance',heading,selected,720,{seed:m.heldOut.seeds[h],inputs:m.heldOut.inputs}));
            const physical=extended.every(r=>r.report.status==='pass');
            const runtime=extended.every(r=>r.report.runtime?.updateDeadlineMisses===0&&r.report.runtime?.controllerDeadlineMisses===0);
            const memory=extended.every(r=>r.report.memory?.maximumRss<=m.memory.maximumProcessRssMiB*2**20
              &&r.report.memory?.growthAfterWarmup!==null&&r.report.memory?.growthAfterWarmup<=m.memory.maximumPostWarmupGrowthMiB*2**20&&r.report.memory?.wasmMeasured);
            stages['sustained-held-out-runtime-memory']={status:physical&&runtime&&memory?(contactCandidate?'pass':'incomplete'):'fail',physical,runtime,memory,
              application:{status:'incomplete',reason:'An actual browser run with process memory and installed browser fingerprint is required; Node and source parity cannot substitute for it.'}};
            if(contactCandidate&&physical&&runtime&&memory) {
              guardedStage(stages,'actual-browser-runtime-memory',['feasibility','prediction-validation','transfer-regressions',
                'saved-regressions-and-faults','short-screens','official-standing','structural-integration','sustained-held-out-runtime-memory'],()=>{
              const browserDirectory=join(output,'actual-browser-runtime-memory');mkdirSync(browserDirectory);
              json(join(browserDirectory,'scenario.json'),{controllerId,selectedOffsetM:selected,renderers:['canvas2d','webgl']});
              const browser=invoke('actual-browser-runtime-memory',['--max-old-space-size=384','scripts/standing-browser-performance.mjs',browserDirectory]);
              const reportPath=join(browserDirectory,'report.json'),browserReport=existsSync(reportPath)?JSON.parse(readFileSync(reportPath)):null;
              return {status:childReportStatus(browser,browserReport),execution:browser,report:browserReport,
                reason:browserReport?.reason??(browserReport?null:'Missing actual-browser report')};
              });
              stages['sustained-held-out-runtime-memory'].application=stages['actual-browser-runtime-memory'];
            }
          }
        }
      }
    }
  }
}catch(error){preflight.error={message:error.message,stack:error.stack};}
finally {
  const sourceAfter=fingerprints(), sourceUnchanged=JSON.stringify(sourceAfter)===JSON.stringify(source);
  const manifestsUnchanged=Object.entries(manifestDependencies).every(([path,hash])=>existsSync(path)&&sha256(readFileSync(path))===hash)
    &&sha256(readFileSync(join(output,'experiment.json')))===JSON.parse(readFileSync(join(output,'run-manifest.json'))).resolvedExperimentSha256
    &&(!freezeLock||existsSync(freezeLock.path)&&sha256(readFileSync(freezeLock.path))===freezeLock.sha256);
  const status=contactCandidate?acceptanceStatus(stages,sourceUnchanged,manifestsUnchanged,preflight.error)
    :Object.values(stages).every(s=>s.status==='pass')&&sourceUnchanged&&manifestsUnchanged&&!preflight.error?'pass':Object.values(stages).some(s=>s.status==='fail')?'fail':'incomplete';
  const report={status,stages,preflight,sourceUnchanged,manifestsUnchanged,feasibilitySteps,wallMs:performance.now()-started,
    runs:runs.map(r=>({name:r.name,status:r.report.status,completedTicks:r.report.completedTicks,firstFailure:r.report.firstFailure,directory:r.directory})),commands,
    nextDiagnostic:stages.feasibility.status==='fail'?(contactCandidate
      ?'Inspect the retained first contact-force failure. Prediction and transfer acceptance are blocked; a new version requires a falsifiable evidence-supported change within the declared experiment budget.'
      :controllerId==='h77-v1'
      ?'The output-neutral native trace found correct, uncapped joint rows and destabilizing sole/contact response; continue only in the separately versioned coordinated contact-wrench/load-distribution branch.'
      :'Inspect first failing native constraint rows, comparing pose-derived coordinate/rate with native rows on copied snapshots. H42 fixed representation differences do not by themselves identify the articulated failure cause.'):null,
    operatingEnvelope:'Unvalidated unless reference, local return and all dependent gates pass; no permanent-stability or interactive-performance claim.'};
  json(join(output,'report.json'),report);
  const files=inventory(output);verifyInventory(output,files);json(join(output,'artifacts.json'),files);
  console.log(JSON.stringify({output,status,stages,sourceUnchanged}));process.exitCode=status==='pass'?0:1;
}
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))await runAcceptance();
