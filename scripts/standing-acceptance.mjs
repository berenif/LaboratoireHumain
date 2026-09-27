import assert from 'node:assert/strict';
import { closeSync, copyFileSync, existsSync, mkdirSync, openSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { spawnSync } from 'node:child_process';
import { gzipSync, gunzipSync } from 'node:zlib';
import os from 'node:os';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';
import { inventory, json, replayFailure, verifyInventory } from './coordinated-standing-evidence.mjs';

const arguments_=process.argv.slice(2);
if(arguments_[0]==='--replay') {
  const directory=resolve(arguments_[1]);
  const files=JSON.parse(readFileSync(join(directory,'artifacts.json')));verifyInventory(directory,files);
  const run=JSON.parse(readFileSync(join(dirname(directory),'run-manifest.json')));
  for(const [path,hash] of Object.entries(run.dependencies))assert.equal(sha256(readFileSync(path)),hash,`Replay dependency changed: ${path}`);
  const {default:RAPIER}=await import('@dimforge/rapier3d-compat');await RAPIER.init();
  console.log(JSON.stringify(await replayFailure(directory,RAPIER)));process.exit(0);
}
const experimentPath='docs/experiments/standing-h74-v1.json';
const m=JSON.parse(readFileSync(experimentPath));
const output=resolve(arguments_[0]??`evidence/standing-h74-v1/${new Date().toISOString().replaceAll(/[:.]/g,'-')}`);
assert.ok(!existsSync(output),'Output must be a fresh directory; retain previous attempts');mkdirSync(output,{recursive:true});
const started=performance.now(), source=fingerprints();
const dependencies=Object.fromEntries(['node_modules/@dimforge/rapier3d-compat/package.json','node_modules/@dimforge/rapier3d-compat/dist/rapier.mjs',
  'node_modules/@dimforge/rapier3d-compat/dist/rapier_wasm3d_bg.wasm','node_modules/tsx/package.json'].map(p=>[p,sha256(readFileSync(p))]));
copyFileSync(experimentPath,join(output,'experiment.json'));
json(join(output,'run-manifest.json'),{started:new Date().toISOString(),experimentSha256:sha256(readFileSync(experimentPath)),source,dependencies,
  command:[process.execPath,...process.execArgv,...process.argv.slice(1)],node:process.version,nodeExecutable:process.execPath,nodeExecutableSha256:sha256(readFileSync(process.execPath)),
  cpu:os.cpus()[0].model,os:`${os.type()} ${os.release()} ${os.arch()}`,rapier:'0.20.0',seed:0,
  qualification:'Frozen before this attempt; original startup, serial processes, no H73 evaluation.'});
const preservedSource=Object.fromEntries(Object.keys(source).filter(p=>/^(src\/|scripts\/|tests\/|package|tsconfig)/.test(p)).map(p=>[p,readFileSync(p,'utf8')]));
writeFileSync(join(output,'source-snapshot.json.gz'),gzipSync(JSON.stringify(preservedSource)),{flag:'wx'});
const stages=Object.fromEntries(['feasibility','saved-regressions-and-faults','short-screens','official-standing','structural-integration','sustained-held-out-runtime-memory'].map(name=>[name,{status:'incomplete',reason:'Prerequisites not yet passed'}]));
const runs=[],commands=[], preflight={};let feasibilitySteps=0;
const invoke=(name,args,environment={})=>{
  const remaining=m.budget.maximumAcceptanceWallS*1000-(performance.now()-started);
  if(remaining<=0)throw new Error('Total acceptance wall budget exhausted');
  const log=join(output,`${name}.log`),fd=openSync(log,'wx'),begin=performance.now();
  let result;
  try {result=spawnSync(process.execPath,args,{cwd:process.cwd(),env:{...process.env,...environment},stdio:['ignore',fd,fd],windowsHide:true,
    timeout:Math.min(remaining,m.budget.childTimeoutS*1000)});}finally{closeSync(fd);}
  const record={name,command:[process.execPath,...args],environment,exit:result.status,signal:result.signal,error:result.error?.message??null,wallMs:performance.now()-begin,log};
  commands.push(record);console.log(JSON.stringify({operation:name,exit:record.exit,wallMs:record.wallMs}));return record;
};
const trial=(name,kind,heading,offset,steps,extra={})=>{
  if(['reference','local'].includes(kind)) {
    if(feasibilitySteps+steps>m.budget.maximumFeasibilitySteps || (performance.now()-started)>m.budget.maximumFeasibilityWallS*1000)throw new Error('Feasibility budget exhausted');
    feasibilitySteps+=steps;
  }
  const dir=join(output,name);mkdirSync(dir);json(join(dir,'scenario.json'),{name,kind,heading,offset,steps,...extra});
  const execution=invoke(name,['--max-old-space-size=384','--import','tsx','scripts/standing-coordinated-trial.mjs',dir]);
  const reportPath=join(dir,'report.json');
  const report=existsSync(reportPath)?JSON.parse(readFileSync(reportPath)):{status:'incomplete',reason:'Missing final report',completedTicks:0};
  if(execution.exit!==0&&report.status==='pass')report.status='fail';
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
  preflight.policy=invoke('policy-tests',['--test','--test-concurrency=1','tests/coordinated-standing.test.mjs'],
    preflight.historical.status==='pass'?{STANDING_OLD_SOURCE:join(output,'saved-inputs/original-source')} : {});
  if(preflight.policy.exit!==0)throw new Error('Policy/integration preflight failed');
  const faults=m.failurePolicy.injections.map((fault,i)=>trial(`fault-${i}`, 'fault', 0,0,3,{fault,faultTick:2}));
  preflight.injectedFaults={status:faults.every(r=>r.report.status==='pass')?'pass':'fail',cases:faults.length};
  if(preflight.injectedFaults.status!=='pass')throw new Error('Injected failure policy failed');
  let selected;
  for(const [i,offset]of m.reference.forwardOffsetsM.entries()) {
    const result=trial(`reference-${i}`,'reference',0,offset,m.budget.referenceStepsPerTrial);
    if(result.report.status==='pass'){selected=offset;break;}
  }
  let locals=[];
  if(selected!==undefined)locals=m.local.inputs.map((input,i)=>trial(`local-${i}`,'local',0,selected,m.budget.localStepsPerTrial,{inputs:[input]}));
  const feasible=selected!==undefined&&locals.every(r=>r.report.status==='pass');
  stages.feasibility={status:feasible?'pass':'fail',selectedOffsetM:selected??null,allocatedSteps:feasibilitySteps,
    reason:feasible?'Reference hold and declared local return passed':'No validated reference/local return within the frozen trial budget; predictive work is not authorized by this result'};
  if(!feasible) {
    if(feasibilitySteps+4320>m.budget.maximumFeasibilitySteps || performance.now()-started>m.budget.maximumFeasibilityWallS*1000) {
      preflight.representation={status:'incomplete',reason:'Feasibility budget exhausted before H42 diagnostic'};
    }else {
      const execution=invoke('h42-representation',['scripts/probe-rigid-assembly-drift.mjs',join(output,'h42-representation.json')],
        {RIGID_MODES:'fixed-joints,fixed-multibody',RIGID_EXPORT_SNAPSHOTS:'1',RIGID_SNAPSHOT_TICK:'1'});
      feasibilitySteps+=4320;preflight.representation={status:execution.exit===0?'pass':'incomplete',execution,
        qualification:'Mechanism diagnostic execution, not articulated standing acceptance; locked fixture changes DOFs only in these separate copies.'};
    }
  }else {
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
            'tests/physical-chain-integrity.test.mjs','tests/character-domain.test.mjs','tests/balance-controller.test.mjs','tests/coordinated-standing.test.mjs']);
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
            stages['sustained-held-out-runtime-memory']={status:physical&&runtime&&memory?'incomplete':'fail',physical,runtime,memory,
              application:{status:'incomplete',reason:'An actual browser run with process memory and installed browser fingerprint is required; Node and source parity cannot substitute for it.'}};
          }
        }
      }
    }
  }
}catch(error){preflight.error={message:error.message,stack:error.stack};}
finally {
  const sourceAfter=fingerprints(), sourceUnchanged=JSON.stringify(sourceAfter)===JSON.stringify(source);
  const status=Object.values(stages).every(s=>s.status==='pass')&&sourceUnchanged&&!preflight.error?'pass':Object.values(stages).some(s=>s.status==='fail')?'fail':'incomplete';
  const report={status,stages,preflight,sourceUnchanged,feasibilitySteps,wallMs:performance.now()-started,
    runs:runs.map(r=>({name:r.name,status:r.report.status,completedTicks:r.report.completedTicks,firstFailure:r.report.firstFailure,directory:r.directory})),commands,
    nextDiagnostic:stages.feasibility.status==='fail'?'Inspect first failing native constraint rows, comparing pose-derived coordinate/rate with native rows on copied snapshots. H42 fixed representation differences do not by themselves identify the articulated failure cause.':null,
    operatingEnvelope:'Unvalidated unless reference, local return and all dependent gates pass; no permanent-stability or interactive-performance claim.'};
  json(join(output,'report.json'),report);
  const files=inventory(output);verifyInventory(output,files);json(join(output,'artifacts.json'),files);
  console.log(JSON.stringify({output,status,stages,sourceUnchanged}));process.exitCode=status==='pass'?0:1;
}
