// Pure WASM actor parity. No WorkerRuntime or physical world is constructed.
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { initSync, getup_actor } from '../rust/dist/app/lh_worker.js';
const fixtures=JSON.parse(readFileSync('rust/crates/sim/data/getup-policy/parity.json'));
initSync({module:readFileSync('rust/dist/app/lh_worker_bg.wasm')});
let maximumAction=0, maximumRaw=0;
for(const fixture of fixtures.actors){
 const actual=JSON.parse(getup_actor(fixture.actor,JSON.stringify(fixture.input)));
 for(let i=0;i<21;i++)maximumAction=Math.max(maximumAction,Math.abs(actual.action[i]-fixture.action[i]));
 for(let i=0;i<42;i++)maximumRaw=Math.max(maximumRaw,Math.abs(actual.raw[i]-fixture.raw[i]));
}
assert.ok(maximumAction<=fixtures.actionTolerance);
assert.ok(maximumRaw<=fixtures.rawTolerance);
assert.throws(()=>getup_actor('student','[0]'));
console.log(JSON.stringify({passed:true,fixtures:fixtures.actors.length,maximumAction,maximumRaw,physicsIntegrations:0}));
