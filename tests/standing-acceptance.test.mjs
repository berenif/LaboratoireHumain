import assert from 'node:assert/strict';
import { mkdtempSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import test from 'node:test';
import { acceptanceStatus, CANDIDATE_CONTROLLERS, childReportStatus, guardedStage, initialStages, loadManifest, TRANSFER_TEST_PATTERN, verifyFreezeLock } from '../scripts/standing-acceptance.mjs';
import { assessBrowserRun } from '../scripts/standing-browser-performance.mjs';

test('recursive inheritance preserves ancestor thresholds and replaces arrays without mutating historical manifests',()=>{
  const directory=mkdtempSync(join(tmpdir(),'standing-manifest-test-'));
  try{
    const save=(name,value)=>writeFileSync(join(directory,name),JSON.stringify(value));
    save('base.json',{budget:{wall:900,steps:7320},official:{headings:[0,1,-1],limit:.01},controller:{id:'old'}});
    save('middle.json',{extends:'base.json',budget:{wall:800},controller:{id:'middle'}});
    save('leaf.json',{extends:'middle.json',controller:{id:'new'},official:{headings:[0]}});
    const result=loadManifest(join(directory,'leaf.json'));
    assert.equal(result.paths.length,3);assert.deepEqual(result.manifest.budget,{wall:800,steps:7320});
    assert.deepEqual(result.manifest.official,{headings:[0],limit:.01});assert.equal(result.manifest.controller.id,'new');
    assert.equal(loadManifest(join(directory,'base.json')).manifest.budget.wall,900);
    save('base.json',{extends:'leaf.json'});
    assert.throws(()=>loadManifest(join(directory,'leaf.json')),/inheritance cycle/);
  }finally{rmSync(directory,{recursive:true,force:true});}
});

test('historical stage sets stay unchanged and only an actually created contact candidate is authorized',()=>{
  const historical=['feasibility','saved-regressions-and-faults','short-screens','official-standing','structural-integration','sustained-held-out-runtime-memory'];
  assert.deepEqual(Object.keys(initialStages('h74-v1')),historical);
  assert.deepEqual(Object.keys(initialStages('h77-v1')),historical);
  assert.deepEqual(CANDIDATE_CONTROLLERS,['h78-v1','h79-v1','h80-v1']);
  const next=initialStages('h78-v1');
  assert.ok(next['prediction-validation']);assert.ok(next['transfer-regressions']);assert.ok(next['actual-browser-runtime-memory']);
  assert.ok(Object.values(next).every(stage=>stage.status==='incomplete'));
  const h74=loadManifest('docs/experiments/standing-h74-v1.json').manifest;
  const h77=loadManifest('docs/experiments/standing-h77-v1.json').manifest;
  assert.deepEqual(h77.budget,h74.budget);assert.deepEqual(h77.official,h74.official);
});

test('pretrial freeze rejects changed, added, or removed source and any manifest-chain mismatch',()=>{
  const source={'src/a.ts':'a','scripts/run.mjs':'b'},path=join(process.cwd(),'docs','experiments','standing-h78-v1.json');
  const manifests={[path]:'m'},lock={source,manifestDependencies:manifests};
  assert.doesNotThrow(()=>verifyFreezeLock(lock,{...source},{...manifests}));
  assert.throws(()=>verifyFreezeLock(lock,{...source,'src/a.ts':'changed'},manifests),/frozen source changed/);
  assert.throws(()=>verifyFreezeLock(lock,{...source,'src/new.ts':'new'},manifests),/frozen source changed/);
  assert.throws(()=>verifyFreezeLock(lock,{'src/a.ts':'a'},manifests),/frozen source changed/);
  assert.throws(()=>verifyFreezeLock(lock,source,{[path]:'changed'}),/manifest chain changed/);
  assert.throws(()=>verifyFreezeLock(lock,source,{}),/manifest chain changed/);
});

test('missing reports, killed children and contradictory success reports never qualify',()=>{
  const successful={exit:0,signal:null,error:null};
  assert.equal(childReportStatus(successful,{status:'pass'}),'pass');
  assert.equal(childReportStatus(successful,null),'incomplete');
  assert.equal(childReportStatus(successful,{status:'unknown'}),'incomplete');
  assert.equal(childReportStatus({...successful,exit:null,error:'ETIMEDOUT'},{status:'pass'}),'incomplete');
  assert.equal(childReportStatus({...successful,exit:null,signal:'SIGTERM'},{status:'pass'}),'incomplete');
  assert.equal(childReportStatus({...successful,exit:1},{status:'pass'}),'fail');
  assert.equal(childReportStatus({...successful,exit:1},{status:'incomplete'}),'incomplete');
  assert.equal(childReportStatus({...successful,exit:1},{status:'fail'}),'fail');
});

test('prediction and transfer callbacks stay blocked until their physical prerequisites pass',()=>{
  const stages=initialStages('h78-v1'),called=[];
  const run=name=>()=>{called.push(name);return {status:'pass'};};
  guardedStage(stages,'prediction-validation',['feasibility'],run('prediction'));
  assert.deepEqual(called,[]);
  stages.feasibility={status:'pass'};
  guardedStage(stages,'prediction-validation',['feasibility'],()=>({status:'fail'}));
  guardedStage(stages,'transfer-regressions',['feasibility','prediction-validation'],run('transfer'));
  assert.deepEqual(called,[]);
  guardedStage(stages,'prediction-validation',['feasibility'],run('prediction'));
  guardedStage(stages,'transfer-regressions',['feasibility','prediction-validation'],run('transfer'));
  assert.deepEqual(called,['prediction','transfer']);
  assert.match('slow pulls stay connected and stepping remains available after release and reversal',new RegExp(TRANSFER_TEST_PATTERN));
  assert.match('a planted reversal preserves ownership without a transient unsupported fall',new RegExp(TRANSFER_TEST_PATTERN));
});

test('browser acceptance is mandatory and changed frozen inputs invalidate promotion',()=>{
  const stages=initialStages('h78-v1');
  for(const [name,stage]of Object.entries(stages))if(name!=='actual-browser-runtime-memory')stage.status='pass';
  let browserCalls=0;
  guardedStage(stages,'actual-browser-runtime-memory',['prediction-validation','transfer-regressions'],()=>{browserCalls++;return {status:'incomplete',reason:'Browser unavailable'};});
  assert.equal(browserCalls,1);assert.equal(acceptanceStatus(stages,true,true,null),'incomplete');
  stages['actual-browser-runtime-memory']={status:'pass'};
  assert.equal(acceptanceStatus(stages,true,true,null),'pass');
  assert.equal(acceptanceStatus(stages,false,true,null),'fail');
  assert.equal(acceptanceStatus(stages,true,false,null),'fail');
  assert.equal(acceptanceStatus(stages,true,true,{message:'child failed'}),'incomplete');
  stages.feasibility={status:'fail'};
  assert.equal(acceptanceStatus(stages,true,true,null),'fail');
});

// These records are isolated unit-test fixtures, never physical/browser evidence.
function browserFixture(){
  const manifest=loadManifest('docs/experiments/standing-h74-v1.json').manifest;manifest.controller.id='h78-v1';
  const run={controllerId:'h78-v1',renderer:'canvas2d',actualRenderer:'canvas2d',canvasPresent:true,flatFloor:true,
    browser:{product:'fixture',userAgent:'fixture'},completedTicks:7320,observedWallMs:120000,
    runtime:{fullUpdateMs:{count:7200,max:16},controllerMs:{count:7200,max:7},renderMs:{count:7200,max:5},updateDeadlineMisses:0,controllerDeadlineMisses:0},
    memory:{samples:[{rss:100},{rss:100}],maximumRss:100*2**20,growthAfterWarmup:0,wasmMeasured:true}};
  return {manifest,run};
}
test('browser gate requires both actual execution identity and complete wall-time/process-memory evidence',()=>{
  const {manifest,run}=browserFixture();assert.equal(assessBrowserRun(run,manifest),'pass');
  for(const change of [{observedWallMs:119999},{completedTicks:7319},{actualRenderer:'webgl'},{flatFloor:false},{canvasPresent:false},{controllerId:'legacy'},
    {browser:null},{memory:{...run.memory,wasmMeasured:false}},{memory:{...run.memory,maximumRss:null}},
    {runtime:{...run.runtime,controllerMs:{count:0,max:0}}},{runtime:{...run.runtime,renderMs:null}}]){
    assert.equal(assessBrowserRun({...run,...change},manifest),'incomplete',JSON.stringify(change));
  }
});
test('one runtime miss, physical failure or inherited memory breach fails browser acceptance',()=>{
  const {manifest,run}=browserFixture();
  for(const change of [{firstFailure:{reason:'fallen'}},{runtime:{...run.runtime,updateDeadlineMisses:1}},
    {runtime:{...run.runtime,controllerMs:{count:7200,max:8.001}}},
    {memory:{...run.memory,maximumRss:769*2**20}},{memory:{...run.memory,growthAfterWarmup:65*2**20}}]){
    assert.equal(assessBrowserRun({...run,...change},manifest),'fail',JSON.stringify(change));
  }
});
