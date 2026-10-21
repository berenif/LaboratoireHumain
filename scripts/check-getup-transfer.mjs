// Explicit bounded transfer-only pilot; importing this file never loads WASM.
// Initial prone construction is declared. No swing, strike, reset, or restoration.
import assert from 'node:assert/strict';
import {readFileSync,writeFileSync,mkdirSync,openSync,fsyncSync,closeSync,existsSync} from 'node:fs';
import {resolve,dirname,join} from 'node:path';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';
import {execFileSync} from 'node:child_process';
export const FEATURES = ['getup-policy-adaptation-experiment','getup-repeat-balance-acquire-experiment',
  'getup-repeat-cycle-experiment','getup-supported-transfer-experiment','getup-terminal-balance-experiment',
  'getup-terminal-base-ready-experiment','getup-terminal-bumpless-experiment','getup-terminal-tail-observer-experiment',
  'root-translation-carry-experiment','standing-dynamics-experiment','unit-root-rotation-experiment'];
export class Budget {
  constructor(priorCharged=0){assert(Number.isSafeInteger(priorCharged)&&priorCharged>=0);this.priorCharged=priorCharged;this.real=0;this.forecast=0;this.previousForecast=0;this.source=null;this.reserved=0;this.calls=0;}
  get charged(){return this.priorCharged+this.real+this.forecast+this.reserved;}
  reserve(){assert.equal(this.reserved,0,'Uncertain previous call');assert(this.charged+5020<=37000,'37,000 total integration envelope');this.reserved=5020;}
  reconcile(native,forecast,source){
    assert.equal(this.reserved,5020);assert(Number.isSafeInteger(native)&&native>=this.real);
    assert(Number.isSafeInteger(forecast)&&forecast>=0);
    const extra=this.source!==source?forecast:forecast-this.previousForecast;
    assert(extra>=0,'Unexplained local forecast rewind');
    assert(native-this.real<=20 && native-this.real+extra<=this.reserved,'Dispatch exceeded charged bound');
    this.real=native;this.forecast+=extra;this.previousForecast=forecast;this.source=source;this.reserved=0;this.calls++;
  }
}
export function transferEntryReady(snapshot){
  const p=snapshot.diagnostics.getup?.policy;
  return snapshot.motion==='Upright'&&p?.prototype_standing_passed===true&&p.stable_standing_s>=1
    &&p.phase==='experimental-joint-balance-holding';
}
export function rightPilotPrior(prior,previous){
  assert.equal(prior.schema,'lh-transfer-geometry-ledger/v1');
  assert.equal(prior.aggregateLimit,37000);assert.equal(prior.priorCharged,10840);
  assert.equal(prior.real,4800);assert.equal(prior.forecast,560);assert.equal(prior.reserved,0);
  assert.equal(prior.charged,16200);assert.equal(prior.calls,240);
  assert.equal(previous.schema,'lh-supported-transfer-budget/v1');assert.equal(previous.limit,37000);
  assert.equal(previous.priorCharged,5480);assert.equal(previous.real,4800);assert.equal(previous.forecast,560);
  assert.equal(previous.reserved,0);assert.equal(previous.charged,10840);
  assert.equal(previous.charged,previous.priorCharged+previous.real+previous.forecast);
  assert.equal(prior.charged,previous.charged+prior.real+prior.forecast);
  return prior.charged;
}
const sha=x=>createHash('sha256').update(x).digest('hex');
function durable(path,value){
  const created=!existsSync(path),fd=openSync(path,'w');
  try{writeFileSync(fd,JSON.stringify(value)+'\n');fsyncSync(fd);}finally{closeSync(fd);}
  if(created){const d=openSync(dirname(path),'r');try{fsyncSync(d);}finally{closeSync(d);}}
}
export async function main(args){
  assert.equal(args[0],'--execute','No physical execution without explicit --execute');
  const [assets,receiptPath,out,priorLedgerPath,previousTransferLedgerPath]=args.slice(1).map(x=>resolve(x));assert(assets&&receiptPath&&out&&priorLedgerPath&&previousTransferLedgerPath);
  const priorBytes=readFileSync(priorLedgerPath),prior=JSON.parse(priorBytes);
  const previousBytes=readFileSync(previousTransferLedgerPath),previous=JSON.parse(previousBytes);
  assert.equal(sha(previousBytes),prior.priorLedgerSha256);
  rightPilotPrior(prior,previous);
  assert(!existsSync(out),'New output directory required; never resume an attempt');
  const receiptBytes=readFileSync(receiptPath),receipt=JSON.parse(receiptBytes);
  assert.equal(receipt.schema,'lh-supported-transfer-build/v1');
  const git=(...a)=>execFileSync('git',a,{encoding:'utf8'}).trim();
  const harnessCommit=git('rev-parse','HEAD'),harnessTree=git('rev-parse','HEAD^{tree}');
  assert.equal(git('rev-parse',`${receipt.build_local_commit}^{tree}`),receipt.source_tree);
  // This correction changes only the harness/documentation; no worker rebuild
  // or equivalence assumption is needed: every actual Rust input is hash-checked below.
  const allowed=['scripts/check-getup-transfer.mjs','scripts/getup-transfer-budget-test.mjs','docs/experiments/getup-supported-transfer.md'];
  assert(git('diff','--name-only',receipt.build_local_commit,'HEAD').split('\n').filter(Boolean).every(p=>allowed.includes(p)));
  assert.equal(git('diff','HEAD','--',...allowed),'');
  for(const [p,h] of Object.entries(receipt.source_inputs))assert.equal(sha(readFileSync(p)),h,`Changed build input ${p}`);
  const wasm=readFileSync(join(assets,'lh_worker_bg.wasm')),glue=readFileSync(join(assets,'lh_worker.js')),
    profile=readFileSync(join(assets,'profile.json'));
  assert.equal(sha(wasm),receipt.worker_sha256);assert.equal(sha(glue),receipt.glue_sha256);assert.equal(sha(profile),receipt.profile_sha256);
  assert.deepEqual(receipt.effective_sim_features,FEATURES);
  const p=JSON.parse(profile);assert.equal(p.getup_supported_transfer,'gravity-only-supported-transfer-v1');
  assert.equal(p.dt_s,0.00125);assert.equal(p.tick_substeps,20);
  mkdirSync(out);const directory=openSync(dirname(out),'r');try{fsyncSync(directory);}finally{closeSync(directory);}
  const start=Date.now(),budget=new Budget(prior.charged),trace=[],completed=[],requests=[];
  const ledger=()=>({schema:'lh-supported-transfer-budget/v1',startedAt:new Date(start).toISOString(),
    limit:37000,wallLimitSeconds:584,previousTransferLedgerSha256:sha(previousBytes),priorLedgerSha256:sha(priorBytes),priorSourceTree:prior.sourceTree,...budget,charged:budget.charged,sourceTree:harnessTree,buildSourceTree:receipt.source_tree});
  durable(join(out,'ledger.json'),ledger());
  let w,reply,triggered=false,reason='unstarted',error=null,side=1,lastKey='',lastTransfer=null;
  try{
    const mod=await import(pathToFileURL(join(assets,'lh_worker.js')));mod.initSync({module:wasm});
    assert.deepEqual(JSON.parse(mod.getup_build_capabilities()).effectiveSimFeatures,FEATURES);
    w=mod.WorkerRuntime.getup_fixture_with_profile('belly',profile.toString());
    w.set_root_translation_compensation_enabled(true);reply=JSON.parse(w.advance(0));
    assert.equal(JSON.parse(w.getup_transfer_report()).native,0);
    while(true){
      assert(Date.now()-start<584000,'Remaining 584-second wall deadline');
      const s=reply.snapshot,status=s.diagnostics.getup?.policy,t=JSON.parse(w.getup_transfer_report());lastTransfer=t;
      assert.equal(s.stamp.generation,1);assert.equal(s.diagnostics.direct_pelvis_force_n,0);assert.equal(s.diagnostics.direct_pelvis_torque_nm,0);
      const key=JSON.stringify([s.motion,s.counters,t.transfer?.phase,status?.phase]);
      if(key!==lastKey||budget.calls%20===0){trace.push({native:t.native,motion:s.motion,counters:s.counters,
        stable:status?.stable_standing_s,policyPhase:status?.phase,refusal:status?.refusal,failure:reply.failure,transfer:t.transfer});lastKey=key;}
      if(reply.failure){reason='physical-failure';break;}
      if(t.transfer&&['failed','cancelled'].includes(t.transfer.phase)){reason='transfer-stopped';break;}
      if(!triggered&&t.native===400){assert(JSON.parse(w.getup_fixture_report()).state.ready);
        w.trigger_getup_fixture();triggered=true;reply=JSON.parse(w.advance(0));continue;}
      if(triggered&&t.transfer?.phase==='passed'){completed.push(t.transfer);reason='right-transfer-and-return-passed';break;}
      if(triggered&&(!t.transfer||t.transfer.phase==='passed')&&transferEntryReady(s)){
        const report=JSON.parse(w.getup_report()),standing=JSON.parse(w.getup_standing_diagnostic('null'));
        requests.push({side,native:t.native,reference:report.terminal_balance.reference,com:standing.com,
          capture:standing.capture,feet:standing.contactMotion.feet});w.begin_getup_transfer(side);continue;
      }
      budget.reserve();durable(join(out,'ledger.json'),ledger());
      try{
        reply=JSON.parse(w.advance(1));
        const n=JSON.parse(w.getup_transfer_report()).native,st=reply.snapshot.diagnostics.getup?.policy;
        budget.reconcile(n,st?.forecast_substeps??0,st?.started_at_s??null);
      }catch(e){
        // A thrown call can still have advanced. Read-only reconciliation only;
        // if that fails, retain the entire 5,020 reservation and never retry.
        try{const n=JSON.parse(w.getup_transfer_report()).native,c=JSON.parse(w.getup_report());
          budget.reconcile(n,c?.status?.forecast_substeps??0,c?.status?.started_at_s??null);
        }catch{}
        throw e;
      }finally{durable(join(out,'ledger.json'),ledger());}
    }
  }catch(e){error=String(e?.stack??e);reason='bounded-or-control-stop';}
  finally{
    if(w){try{lastTransfer=JSON.parse(w.getup_transfer_report());}catch{}w.free();}
    const result={passed:reason==='right-transfer-and-return-passed',reason,error,
      sourceCommit:harnessCommit,sourceTree:harnessTree,buildSourceCommit:receipt.source_commit,buildSourceTree:receipt.source_tree,harnessSha256:sha(readFileSync(new URL(import.meta.url))),buildReceiptSha256:sha(receiptBytes),
      workerSha256:sha(wasm),glueSha256:sha(glue),profileSha256:sha(profile),initialProne:true,
      constructors:w?1:0,resets:0,restores:0,strikes:0,liftCommands:0,gravityOnly:true,browserValidated:false,
      ...ledger(),schema:'lh-supported-transfer-pilot/v1',finalFailure:reply?.failure,finalPolicy:reply?.snapshot?.diagnostics?.getup?.policy,elapsedWallSeconds:(Date.now()-start)/1000,requests,completed,lastTransfer,
      activeStateResumable:false,requestedSides:[1],leftNotAttempted:true,trace};
    const bytes=JSON.stringify(result)+'\n';assert(Buffer.byteLength(bytes)<=1024*1024,'Compact result exceeds 1 MiB');
    durable(join(out,'result.json'),result);console.log(JSON.stringify({passed:result.passed,reason,charged:budget.charged,
      real:budget.real,forecast:budget.forecast,uncertainReservation:budget.reserved,completed:completed.length,error}));
  }
}
if(process.argv[1]&&import.meta.url===pathToFileURL(resolve(process.argv[1])).href)await main(process.argv.slice(2));
