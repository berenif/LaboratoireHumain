import assert from 'node:assert/strict';
import test from 'node:test';
import { fingerprint, releaseDecision, requiredReleaseStages, validateCandidate, nativeTestCount, persistReceipt } from '../scripts/rust-release-tools.mjs';
import { rollbackCases, rollbackIdentity, rollbackStages, validateTypescriptRollback } from '../scripts/typescript-rollback-tools.mjs';

test('an interrupted receipt write preserves the previous valid rejection checkpoint',()=>{
  const files=new Map();let fail=false;
  const io={writeFileSync(path,bytes){files.set(path,fail?bytes.slice(0,15):bytes);if(fail)throw new Error('ENOSPC');},
    renameSync(from,to){files.set(to,files.get(from));files.delete(from);}};
  persistReceipt('execution.json',{releaseAccepted:false,complete:false},io);
  fail=true;
  assert.throws(()=>persistReceipt('execution.json',{releaseAccepted:true,complete:true},io),/ENOSPC/);
  assert.deepEqual(JSON.parse(files.get('execution.json')),{releaseAccepted:false,complete:false});
});

test('missing, failed, timed-out and duplicate release coverage cannot be accepted',()=>{
  const stages=requiredReleaseStages.map(name=>({name,passed:true}));
  assert.equal(releaseDecision(stages).releaseAccepted,true);
  assert.equal(releaseDecision(stages.slice(1)).releaseAccepted,false);
  assert.equal(releaseDecision([...stages,{...stages[0]}]).releaseAccepted,false);
  for(const passed of [false,null,undefined,'true']) {
    assert.equal(releaseDecision(stages.map((stage,index)=>index===3?{...stage,passed}:stage)).releaseAccepted,false);
  }
});
test('promotion requires source, candidate and asset identities and every acceptance stage',()=>{
  const assets={'app.wasm':'abc','sim-worker.js':'def'};
  const candidate={schema:1,sourceFingerprint:'source',assetsFingerprint:fingerprint(assets),assets,mount:'/LaboratoireHumain/'};
  candidate.candidateId=fingerprint({source:candidate.sourceFingerprint,assets:candidate.assetsFingerprint,mount:candidate.mount});
  const receipt={...candidate,releaseAccepted:true,sourceUnchanged:true,assetsUnchanged:true,stages:requiredReleaseStages.map(name=>({name,passed:true}))};
  assert.equal(validateCandidate(candidate,receipt,assets).releaseAccepted,true);
  for(const key of ['candidateId','sourceFingerprint','assetsFingerprint']) {
    assert.throws(()=>validateCandidate(candidate,{...receipt,[key]:'different'},assets),/mismatch/);
  }
  assert.throws(()=>validateCandidate(candidate,receipt,{...assets,'app.wasm':'changed'}),/assets changed/);
  assert.throws(()=>validateCandidate(candidate,{...receipt,releaseAccepted:false},assets));
  assert.throws(()=>validateCandidate(candidate,{...receipt,stages:[]},assets),/Unaccepted/);
});
test('identities are independent of manifest insertion order and zero selected tests remain zero',()=>{
  assert.equal(fingerprint({z:{b:2,a:1},a:0}),fingerprint({a:0,z:{a:1,b:2}}));
  assert.equal(nativeTestCount('test result: ok. 0 passed; 0 failed;'),0);
  assert.equal(nativeTestCount('test result: ok. 3 passed; 0 failed;'),3);
  assert.equal(nativeTestCount('test result: FAILED. 3 passed; 1 failed;'),0);
});

function rollbackFixture() {
  const assets = {'index.html': 'a'.repeat(64), '404.html': 'b'.repeat(64), '_next/app.js': 'c'.repeat(64)};
  const candidate = {schema: 1, runtime: 'typescript', mount: '/LaboratoireHumain/',
    publishedUrl: 'https://berenif.github.io/LaboratoireHumain/', assets, assetsFingerprint: fingerprint(assets)};
  candidate.rollbackId = rollbackIdentity(candidate);
  const browser = {passed: true, cases: rollbackCases.map(value => {
    const [mode, renderer] = value.split(':');
    return {mode, renderer, passed: true, finite: true, firstFixedSteps: 1, finalFixedSteps: 2, errors: []};
  })};
  const receipt = {rollbackId: candidate.rollbackId, assetsFingerprint: candidate.assetsFingerprint,
    qualified: true, artifactUnchanged: true, publishedAssets: assets, localBrowser: browser, publishedBrowser: browser,
    stages: rollbackStages.map(name => ({name, passed: true}))};
  return {assets, candidate, receipt};
}

test('first migration accepts only exact qualified TypeScript rollback bytes', () => {
  const {assets, candidate, receipt} = rollbackFixture();
  assert.equal(validateTypescriptRollback(candidate, receipt, assets), true);
  assert.throws(() => validateTypescriptRollback(candidate, receipt, {...assets, 'index.html': 'd'.repeat(64)}), /changed/);
  assert.throws(() => validateTypescriptRollback(candidate, {...receipt, publishedAssets: {}}, assets), /deployed/);
  assert.throws(() => validateTypescriptRollback({...candidate, rollbackId: 'other'}, receipt, assets), /identity/);
  assert.throws(() => validateTypescriptRollback({...candidate, runtime: 'rust'}, receipt, assets));
  for (const stages of [[], [...receipt.stages, receipt.stages[0]],
    receipt.stages.map((stage, i) => i === 0 ? {...stage, passed: false} : stage)]) {
    assert.throws(() => validateTypescriptRollback(candidate, {...receipt, stages}, assets));
  }
});

test('rollback rejects incomplete mode/renderer coverage, frozen simulation and invalid assets', () => {
  const {assets, candidate, receipt} = rollbackFixture();
  for (const browser of [{...receipt.localBrowser, cases: receipt.localBrowser.cases.slice(1)},
    {...receipt.localBrowser, cases: [...receipt.localBrowser.cases, receipt.localBrowser.cases[0]]},
    {...receipt.localBrowser, cases: receipt.localBrowser.cases.map(row => ({...row, finalFixedSteps: 1}))}]) {
    assert.throws(() => validateTypescriptRollback(candidate, {...receipt, localBrowser: browser}, assets));
  }
  for (const name of ['../escape', '/absolute', 'C:/absolute', 'dir\\file']) {
    assert.throws(() => validateTypescriptRollback(candidate, receipt, {...assets, [name]: 'e'.repeat(64)}), /path/);
  }
});
