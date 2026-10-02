import assert from 'node:assert/strict';
import { closeSync, mkdirSync, openSync, readFileSync } from 'node:fs';
import { join } from 'node:path';
import { spawnSync } from 'node:child_process';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';
import { json, inventory } from './coordinated-standing-evidence.mjs';

const output=process.argv[2]; mkdirSync(output,{recursive:false});
const source=fingerprints(), checks=[];
const selectedTests=['tests/contact-force-standing.test.mjs','tests/standing-acceptance.test.mjs',
  'tests/contact-force-solve-h79.test.mjs',
  'tests/contact-force-solve-h80.test.mjs',
  'tests/implicit-standing.test.mjs','tests/coordinated-standing.test.mjs','tests/joint-motors.test.mjs',
  'tests/active-step-fall-gate.test.mjs','tests/landing-capture.test.mjs','tests/landing-plan.test.mjs',
  'tests/physical-chain-integrity.test.mjs'];
for(const [name,args] of [
  ['typecheck',['node_modules/typescript/bin/tsc','--noEmit','--pretty','false']],
  ['lint',['scripts/run-tool.mjs','eslint','.','--ignore-pattern','dist','--ignore-pattern','.next']],
  ['build',['scripts/run-tool.mjs','vinext','build']],
  ['regressions',['--test','--test-concurrency=1',...selectedTests]],
]) {
  const fd=openSync(join(output,`${name}.log`),'wx'),start=performance.now();
  // Physics/test children retain384MiB. Toolchain checks use their ordinary
  // package-script heap configuration; they are not candidate physical runs.
  const physical=name==='regressions',nodeArgs=physical?['--max-old-space-size=384',...args]:args;
  let result;try{result=spawnSync(process.execPath,nodeArgs,{
    stdio:['ignore',fd,fd],windowsHide:true,timeout:300000,env:physical?{...process.env,NODE_OPTIONS:'--max-old-space-size=384'}:process.env});}finally{closeSync(fd);}
  const record={name,command:[process.execPath,...nodeArgs],physical,exit:result.status,
    signal:result.signal,error:result.error?.message??null,status:result.error?'incomplete':result.status===0?'pass':'fail',
    wallMs:performance.now()-start,logSha256:sha256(readFileSync(join(output,`${name}.log`)))};
  checks.push(record);console.log(JSON.stringify(record));
}
const sourceAfter=fingerprints(),unchanged=JSON.stringify(source)===JSON.stringify(sourceAfter);
json(join(output,'report.json'),{status:checks.every(c=>c.status==='pass')&&unchanged?'pass':'fail',checks,source,sourceAfter,unchanged});
json(join(output,'artifacts.json'),inventory(output));
assert.ok(checks.every(c=>c.status==='pass')&&unchanged,'Frozen checks failed; see preserved logs');
