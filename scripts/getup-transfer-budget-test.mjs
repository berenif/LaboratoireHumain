import {test} from 'node:test';
import assert from 'node:assert/strict';
import {Budget,main,transferEntryReady} from './check-getup-transfer.mjs';
test('forecast accounting survives controller-local cycle reset',()=>{
 const b=new Budget();b.reserve();b.reconcile(20,1000,0.025);b.reserve();b.reconcile(40,1000,0.025);
 b.reserve();b.reconcile(60,0,null);b.reserve();b.reconcile(80,500,0.1);
 assert.equal(b.charged,1580);assert.equal(b.forecast,1500);
});
test('unknown or malformed completion retains charged reservation and forbids retry',()=>{
 const b=new Budget();b.reserve();assert.throws(()=>b.reconcile(21,0,null));
 assert.equal(b.charged,5020);assert.throws(()=>b.reserve());
});
test('last dispatch requires its whole unchanged 5020 envelope',()=>{
 const b=new Budget();b.real=32000;assert.throws(()=>b.reserve());assert.equal(b.charged,32000);
 b.real=31980;b.reserve();assert.equal(b.charged,37000);
});
test('no runtime entry without explicit execute',async()=>{await assert.rejects(main([]),/explicit --execute/);});

test('closed 400 integrations are carried, never imported as runtime state or refunded',()=>{
 const b=new Budget(400);b.real=31600;assert.throws(()=>b.reserve());
 b.real=31580;b.reserve();assert.equal(b.charged,37000);
 b.reconcile(31600,0,null);assert.equal(b.charged,32000);assert.equal(b.priorCharged,400);
});

test('earned recovery alone is not completed terminal blend',()=>{
 const s={motion:'Upright',diagnostics:{getup:{policy:{prototype_standing_passed:true,stable_standing_s:1.005,phase:'slow-holding'}}}};
 assert.equal(transferEntryReady(s),false);
 s.diagnostics.getup.policy.phase='experimental-joint-balance-blending';assert.equal(transferEntryReady(s),false);
 s.diagnostics.getup.policy.phase='experimental-joint-balance-holding';assert.equal(transferEntryReady(s),true);
 s.diagnostics.getup.policy.stable_standing_s=0.99;assert.equal(transferEntryReady(s),false);
});
test('second closed ledger is carried once against the original cap',()=>{
 const b=new Budget(5480);b.real=26520;assert.throws(()=>b.reserve());
 b.real=26500;b.reserve();assert.equal(b.charged,37000);
});
