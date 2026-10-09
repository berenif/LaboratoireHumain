// Build once, exercise those bytes, and retain an immutable Pages candidate.
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createWriteStream, existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { fingerprint, sourceHashes, treeHashes } from './rust-release-tools.mjs';
import { workspaceOutput } from './rust-browser-tools.mjs';

const output=workspaceOutput(process.argv[2]),profile=resolve(process.argv[3]);
assert.ok(!existsSync(output),'Use a fresh candidate directory');mkdirSync(output,{recursive:true});
const sources=sourceHashes();
const report={schema:1,sourceFingerprint:fingerprint(sources),stages:[],runs:[]};
const save=()=>writeFileSync(join(output,'qualification.json'),JSON.stringify(report,null,2)+'\n');
async function run(name,script,args,timeoutMs=1800000) {
  const log=createWriteStream(join(output,name+'.log'));
  const result=await new Promise(resolve=>{
    const child=spawn(process.execPath,[script,...args],{windowsHide:true});
    child.stdout.pipe(log,{end:false});child.stderr.pipe(log,{end:false});
    const timeout=setTimeout(()=>child.kill(),timeoutMs);let error;
    child.on('error',value=>{error=String(value);});
    child.on('close',(exit,signal)=>{clearTimeout(timeout);log.end();resolve({exit,signal,error});});
  });
  report.runs.push({name,...result});save();
  return result.exit===0&&!result.signal&&!result.error;
}
async function serve(artifact,mount) {
  const child=spawn(process.execPath,['scripts/serve-rust-preview.mjs',artifact,'0',mount],{windowsHide:true});
  const origin=await new Promise((resolve,reject)=>{
    const timeout=setTimeout(()=>{child.kill();reject(new Error('Preview startup timeout'));},15000);let output='';
    child.stdout.on('data',chunk=>{output+=chunk;const match=output.match(/Rust preview: (http:\/\/127\.0\.0\.1:\d+\/[^\s]*)/);if(match){clearTimeout(timeout);resolve(match[1]);}});
    child.on('error',reject);child.on('exit',code=>reject(new Error(`Preview exited ${code}`)));
  });return {child,origin};
}
try {
  for(const [name,mount] of [['root','/'],['subpath','/LaboratoireHumain/']]) {
    const artifact=join(output,name==='subpath'?'site':'root');
    const build=await run(name+'-build','scripts/build-rust-browser.mjs',[artifact,mount]);
    if(!build){report.stages.push({name:name+'-browser',passed:false,reason:'build failed'});continue;}
    const assets=treeHashes(artifact);
    if(name==='subpath') {
      report.assets=assets;report.assetsFingerprint=fingerprint(assets);
      report.candidateId=fingerprint({source:report.sourceFingerprint,assets:report.assetsFingerprint,mount});
    }
    const server=await serve(artifact,mount);
    try {
      let passed=true;
      for(const [check,script] of [['ui','scripts/verify-rust-browser.mjs'],['faults','scripts/verify-rust-browser-faults.mjs'],
        ['worker','scripts/verify-rust-worker.mjs'],['quiet','scripts/verify-rust-worker-quiet.mjs'],['terrain','scripts/verify-rust-terrain.mjs'],['striker','scripts/verify-rust-striker.mjs']]) {
        passed=(await run(name+'-'+check,script,[join(output,name+'-'+check),server.origin,artifact]))&&passed;
      }
      report.stages.push({name:name+'-browser',passed});
      if(name==='subpath') {
        const physical=await run('wasm-physics','scripts/verify-rust-worker-release.mjs',[join(output,'wasm-physics'),server.origin,artifact,profile]);
        // Physical acceptance is independent of the still experimental cached
        // kernel. The native bit-equivalence unit test alone cannot certify WASM.
        report.stages.push({name:'wasm-physics-equivalence',passed:false,physicalPassed:physical,
          missingCoverage:['WASM cached/scalar impulse and velocity bit equivalence']});
        if(physical) {
          await run('hardware-performance','scripts/probe-rust-browser-performance.mjs',[join(output,'hardware-performance'),server.origin,'all',profile,artifact]);
          const performancePath=join(output,'hardware-performance','report.json');
          const timing=existsSync(performancePath)?JSON.parse(readFileSync(performancePath)):{};
          for(const [stage,backend] of [['WebGPU-performance','webgpu'],['WebGL2-performance','webgl2']]) {
            const cases=timing.cases?.filter(c=>c.backend===backend)??[];
            report.stages.push({name:stage,passed:cases.length===2&&cases.every(c=>c.performancePassed)});
          }
          report.stages.push({name:'soak-120',passed:timing.performancePassed===true});
        }
      }
    } finally {server.child.kill();}
    assert.deepEqual(treeHashes(artifact),assets,'Candidate assets changed during qualification');
  }
  report.stages.push({name:'physical-touchscreen',passed:false,missingCoverage:['physical touchscreen on the frozen Windows device']});
  report.assetsUnchanged=report.assets!==undefined&&fingerprint(treeHashes(join(output,'site')))===report.assetsFingerprint;
  report.stages.push({name:'asset-unchanged',passed:report.assetsUnchanged});
  report.sourceUnchanged=fingerprint(sourceHashes())===report.sourceFingerprint;
  writeFileSync(join(output,'candidate.json'),JSON.stringify({schema:1,candidateId:report.candidateId,
    sourceFingerprint:report.sourceFingerprint,assetsFingerprint:report.assetsFingerprint,assets:report.assets,mount:'/LaboratoireHumain/'},null,2)+'\n');
} catch(error){report.error=String(error);}
finally {save();}
process.exitCode=report.sourceUnchanged&&report.assetsUnchanged&&report.stages.every(stage=>stage.passed)?0:1;
