// ONE generic solve of immutable saved QP. No engine load, forward, or integration.
import fs from 'node:fs';import assert from 'node:assert/strict';import crypto from 'node:crypto';import {gunzipSync} from 'node:zlib';
import {constrainedDirectLS} from './pull-direct-qr.mjs';import {directCost} from './pull-whole-body.mjs';import {certificate,dot} from './pull-constrained-ls.mjs';
const output=process.argv[2];assert(output&&!fs.existsSync(output),'Single recorded algebraic qualification only');
const sha=b=>crypto.createHash('sha256').update(b).digest('hex'),bytes=fs.readFileSync('web/proofs/pull-step-whole-body-002/initial-qp.json.gz');assert.equal(sha(bytes),'18456fd8d6c5428d03213787e6f0ab4d0608a87231a3226a10d3af595d43fb39');
const qp=JSON.parse(gunzipSync(bytes)),{C,d}=directCost(qp),joints=JSON.parse(fs.readFileSync('web/proofs/pull-step-coupled-pure/result.json')).joints;
const reference=[.40395077286568576,-.14064402956367772,.005129213052267494,-.08639027038538212,-.05460409325636366,.11238204200488314,-.22245736092828272,-.1976786873193464,1,.08057250666716222,-.02314670555400123,.06109928795895335,-.058545704174085625,-.1813345177688219,1,-.19799183738517404,.18369238595902188,.0388751457767249,.16089706833451972,-.030771817583678765,.06865337904139106];
const report={sourceQP:sha(bytes),solves:1,nativeForwards:0,ticks:0,reference:{cost:.054575667166374846,maxCostDifference:1e-9,maxMotorDifferenceNm:1e-5},genericSelection:true,files:Object.fromEntries(['scripts/pull-direct-qr.mjs','scripts/pull-whole-body.mjs'].map(p=>[p,sha(fs.readFileSync(p))]))};
try{
 assert.equal(C.length,135);assert.equal(C[0].length,45);assert.equal(qp.rows.length,124);
 // Verification only: never supplied to the QR solve.
 const gram=Array.from({length:45},(_,i)=>Array.from({length:45},(_,j)=>dot(C.map(r=>r[i]),C.map(r=>r[j])))),rhs=Array.from({length:45},(_,i)=>dot(C.map(r=>r[i]),d));
 report.gramRelativeMax=Math.max(...gram.flat().map((v,i)=>Math.abs(v-qp.H[i])))/Math.max(...qp.H.map(Math.abs));report.rhsRelativeMax=Math.max(...rhs.map((v,i)=>Math.abs(v-qp.b[i])))/Math.max(...qp.b.map(Math.abs));assert(report.gramRelativeMax<1e-12&&report.rhsRelativeMax<1e-12);
 const t=performance.now(),solve=constrainedDirectLS(C,d,qp.rows,{maxPivots:64,tolerance:1e-10});report.solveMs=performance.now()-t;report.solve=solve;
 assert(solve.solved,solve.reason);report.originalHCertificate=certificate(qp.H,qp.b,qp.rows,solve.x,solve.lambda);assert(report.originalHCertificate.passed);
 report.motorDifferenceNm=Math.max(...reference.map((v,i)=>Math.abs((solve.x[i]-v)*joints[i].gear)));report.costDifference=Math.abs(solve.cost-report.reference.cost);assert(report.motorDifferenceNm<=report.reference.maxMotorDifferenceNm&&report.costDifference<=report.reference.maxCostDifference);
 assert(solve.x.slice(0,21).every(v=>Math.abs(v)<=1+1e-10));report.passed=true;
}catch(e){report.passed=false;report.error=String(e);process.exitCode=1;}
finally{fs.writeFileSync(output,JSON.stringify(report));console.log(JSON.stringify({passed:report.passed,error:report.error,solves:1,ticks:0,nativeForwards:0,solveMs:report.solveMs,pivots:report.solve?.pivots,cost:report.solve?.cost,motorDifferenceNm:report.motorDifferenceNm,backwardError:report.solve?.backwardError,primal:report.solve?.normalizedPrimalViolation,complementarity:report.solve?.complementarity,active:report.solve?.active}));}
