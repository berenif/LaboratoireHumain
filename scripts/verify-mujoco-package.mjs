// Pure packaging/policy checks. Never imports a physics runtime or integrates.
import fs from 'node:fs';
import assert from 'node:assert/strict';
import crypto from 'node:crypto';
import {DenseActor,controls} from '../web/dist/local-getup/slow.js';

const read=p=>fs.readFileSync(p),sha=b=>crypto.createHash('sha256').update(b).digest('hex');
const root='web/dist/',assets={
 'getup-assets/weak-final.json':['web/proofs/getup-weak-seed10/weak-final.json','8b9ef9f76bd26ec5047c5b1d13ecbf4715820ef3c9a8324568dd7cdbc9473281'],
 'getup-assets/reference.json':['web/proofs/getup-continuous-slow/reference.json','ef1aaae307153c55afd9f8da37d63999e65ee06f51e1e694e12e7badbc436dee'],
 'getup-assets/student.bin':['rust/crates/sim/data/getup-policy/student.bin','c489faed55bdab1669c51dab71a102db91ff2a55143789f682e45cd8169d6dbb'],
 'getup-assets/getup-v1.json':['rust/crates/model/data/getup-v1.json','1611ec8c988c1006b0a819ab21e2b7c410adaaf512080be425d6d6e235a8926a'],
};
for(const [name,[source,hash]] of Object.entries(assets)){
 assert.equal(sha(read(root+name)),hash,name);assert(read(root+name).equals(read(source)),name);
}
assert.equal(sha(read(root+'humanoid.xml')),'7a6e3dfc83a9e30bebe81e3e8c6e6b146f8447d1dbfaacd83de90f4937ea5765');
for(const file of ['app.js','engine.js','index.html','mvp.css','LICENSE-MuJoCo.txt','local-getup/cycle.js','local-getup/continuation.js','local-getup/slow.js'])assert(read(root+file).equals(read('web/mujoco/'+file)),file);
for(const file of ['mujoco.js','mujoco.wasm'])assert(read(root+'vendor/'+file).equals(read('node_modules/@mujoco/mujoco/'+file)),file);
assert.equal(JSON.parse(read('node_modules/@mujoco/mujoco/package.json')).version,'3.17.0');
assert.match(read(root+'index.html').toString(),/data-getup="known-entry"/);
const app=read(root+'app.js').toString();
assert(app.includes("import('./local-getup/cycle.js')")&&app.includes("loadCycleAssets(getupAssets)"));
assert(!app.includes("import('/web/"));
assert(!app.includes('pullExperiment')&&!app.includes('enablePullStep'),'Experimental controller cannot activate in release');
assert.match(read(root+'index.html').toString(),/Récupération des genoux validée sur le scénario testé/);
assert.match(read(root+'index.html').toString(),/Transfert de poids et pas non validés/);
assert.match(read(root+'index.html').toString(),/pas utilisable en temps réel/);
// Freeze the already executed controller, observer, engine and policy math.
for(const [file,marker,hash] of [
 ['local-getup/cycle.js','export class CycleObserver','983d62ff367f2c77a99a298b8fee75f8565e1d1227dc0c8deedf0227341171c3'],
 ['local-getup/continuation.js','export class SlowContinuation','1fd083a81cd761a015a5561d0c548a1d9cbb8deb3f806e9008b5a8d5b4902f17'],
 ['local-getup/slow.js',null,'069f5cf1222e93f2ce7c06a8133b4764ec98c2ec142cbe2a24ff4f418e2bde81'],
 ['engine.js',null,'8e6f0383161a0506f510b236b1d1e262be8efef327710d3d6ea441aea02d7d40'],
]){let bytes=read(root+file);if(marker)bytes=bytes.subarray(bytes.indexOf(marker));assert.equal(sha(bytes),hash,file);}
const fixture=JSON.parse(read('rust/crates/sim/data/getup-policy/parity.json'));
const binary=read(root+'getup-assets/student.bin'),actor=new DenseActor(binary.buffer.slice(binary.byteOffset,binary.byteOffset+binary.length));
let raw=0,action=0,pd=0;
for(const f of fixture.actors.filter(f=>f.actor==='student')){
 const actual=actor.infer(f.input);raw=Math.max(raw,...actual.raw.map((v,i)=>Math.abs(v-f.raw[i])));action=Math.max(action,...actual.action.map((v,i)=>Math.abs(v-f.action[i])));
}
for(const f of fixture.pd){const actual=controls(f.reference,f.residual,f.q,f.dq,f.holding);pd=Math.max(pd,...actual.map((v,i)=>Math.abs(v-f.controls[i])));}
assert(raw<=2e-4&&action<=5e-5&&pd<=2e-7);
const files=fs.readdirSync(root,{recursive:true}).filter(p=>fs.statSync(root+p).isFile());
assert.equal(files.length,16);assert(!files.some(p=>/proofs|\.mp4$|\.png$|teacher\.bin/.test(p)));
console.log(JSON.stringify({passed:true,integrations:0,files:files.length,assetBytes:files.reduce((n,p)=>n+fs.statSync(root+p).size,0),controllerUnchanged:true,studentFixtures:fixture.actors.filter(f=>f.actor==='student').length,pdFixtures:fixture.pd.length,raw,action,pd}));
