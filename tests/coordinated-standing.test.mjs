import assert from 'node:assert/strict';
import test, { after } from 'node:test';
import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import { pathToFileURL } from 'node:url';
import { register } from 'tsx/esm/api';
const unregister=register();after(unregister);
const {createEmbodiedCharacter}=await import('../src/character/index.ts');
const {characterFrame}=await import('../src/character/character-frame.ts');
const {coordinatedStandingOptions}=await import('../src/character/standing-selection.ts');
const {CoordinatedStandingController,COORDINATED_STANDING}=await import('../src/character/CoordinatedStandingController.ts');
const {SEGMENT_BY_ID}=await import('../src/core/humanoid.ts');
const {jointFrameAxesWorld}=await import('../src/character/joint-coordinates.ts');
const {scale}=await import('../src/character/math.ts');
const manifest=JSON.parse(readFileSync('docs/experiments/standing-h74-v1.json'));

async function sampleInput() {
  const c=await createEmbodiedCharacter('canvas2d',coordinatedStandingOptions('h74-v1'));
  let input;
  const poses=new Map([...c.poses].map(([id,p])=>[id,structuredClone(p)]));
  c.coordinatedStanding.update=i=>{input={...i,poses};return null;};
  characterFrame(c,1/60,null,'canvas2d');
  return {c,input,poses};
}
test('manifest and shared controller configuration match before physics',()=>{
  for(const[k,v]of Object.entries(COORDINATED_STANDING))assert.equal(manifest.controller[k],v,k);
  assert.equal(manifest.physical.dtS,1/60);assert.equal(manifest.physical.settlingS,2);
  assert.equal(manifest.local.backupRegion,null);
  assert.throws(()=>coordinatedStandingOptions('h73'));
  assert.throws(()=>coordinatedStandingOptions('h74-v1',0.02));
});
test('all task terms produce a capped command even when optional headroom reserve is missed',async()=>{
  const {c,input,poses}=await sampleInput();
  try {
    const controller=new CoordinatedStandingController(poses,0,0,()=>0);
    const gravity=new Map();
    for(const nominal of input.nominal) {
      const d=SEGMENT_BY_ID.get(nominal.id),a=d.jointProfile.axes[0];
      if(a)gravity.set(d.id,scale(jointFrameAxesWorld(poses.get(d.parent).rotation,d.jointProfile)[a.coordinate],a.maxMotorTorqueNm*2));
    }
    const reference=JSON.stringify([...controller.reference]);
    const commands=controller.update({...input,gravity});
    assert.equal(commands.length,24);assert.ok(controller.diagnostics().requestedHeadroom<0.1);
    const result=c.nativeMotors.apply(c.ragdollBodies,c.jointsByChild,commands);
    for(const[id,motor]of result)assert.ok(Math.hypot(...Object.values(motor.torqueWorld))<=Math.hypot(...SEGMENT_BY_ID.get(id).jointProfile.axes.map(a=>a.maxMotorTorqueNm))+1e-6);
    assert.deepEqual(Object.keys(controller.diagnostics().objective),['feet','pelvis','motion','posture','effort']);
    assert.equal(JSON.stringify([...controller.reference]),reference);
  }finally{c.dispose();}
});
test('deadline, infeasibility, contact, model and backup failures latch a real authority transition',async()=>{
  const {c,input,poses}=await sampleInput();
  try {
    for(const reason of manifest.failurePolicy.injections) {
      const controller=new CoordinatedStandingController(poses,0,0,()=>0);
      assert.equal(controller.update({...input,fault:reason}),null);
      assert.equal(controller.update(input),null);
      assert.equal(controller.diagnostics().reason,reason);
      assert.equal(controller.serialize().backupRegion,null);
    }
    let t=0;
    const slow=new CoordinatedStandingController(poses,0,0,()=>{t+=9;return t;});
    assert.equal(slow.update(input),null);assert.equal(slow.diagnostics().reason,'timeout');
    const invalidContact=new CoordinatedStandingController(poses,0,0,()=>0);
    assert.equal(invalidContact.update({...input,tick:121,contacts:[]}),null);
    assert.equal(invalidContact.diagnostics().reason,'invalid-contact');
    const infeasible=new CoordinatedStandingController(poses,0,0,()=>0);
    assert.equal(infeasible.update({...input,nominal:input.nominal.map(c=>({...c,damping:0}))}),null);
    assert.equal(infeasible.diagnostics().reason,'infeasible');
  }finally{c.dispose();}
});
test('shared frame executes candidate before native motors and observes contacts after integration for both renderers',async()=>{
  let golden;
  for(const renderer of ['canvas2d','webgl']) {
    const c=await createEmbodiedCharacter(renderer,coordinatedStandingOptions('h74-v1'));
    try {
      // Deterministic clock only for order/pose parity, never an acceptance run.
      c.coordinatedStanding.clock=()=>0;
      const order=[];
      for(const[obj,method,label]of [[c.balance,'update','balance'],[c.coordinatedStanding,'update','candidate'],
        [c.nativeMotors,'apply','motors'],[c.world,'step','rapier'],[c,'readPhysicsPoses','poses'],[c,'observeContacts','contacts']]) {
        const original=obj[method].bind(obj);obj[method]=(...a)=>{order.push(label);return original(...a);};
      }
      const before=[...c.ragdollBodies.values()];
      const snapshot=characterFrame(c,1/60,null,renderer);
      assert.deepEqual(order,['balance','candidate','motors','rapier','poses','contacts']);
      assert.deepEqual([...c.ragdollBodies.values()],before);
      if(golden)assert.deepEqual(snapshot.segments,golden);else golden=snapshot.segments;
      c.coordinatedStanding.handoff('invalid-model');
      const calls=[];c.nativeMotors.apply=((original)=>(...args)=>{calls.push(args[2]);return original(...args);})(c.nativeMotors.apply.bind(c.nativeMotors));
      characterFrame(c,1/60,null,renderer);
      assert.equal(calls.length,1);assert.ok(calls[0].some(command=>command.stiffness>0));
      assert.equal(c.fixedSteps,2);assert.equal(c.diagnostics().coordinatedStanding.mode,'transition');
    }finally{c.dispose();}
  }
});
test('disabled candidate preserves thirty original-source physical steps exactly', {skip:!process.env.STANDING_OLD_SOURCE?'Original source supplied by npm run test:standing':false},async()=>{
  const original=await import(pathToFileURL(join(process.env.STANDING_OLD_SOURCE,'src/character/index.ts')).href);
  const a=await createEmbodiedCharacter(),expected=[];
  try {
    for(let tick=0;tick<30;tick++) {
      const snapshot=characterFrame(a,1/60,null,'canvas2d');
      expected.push(structuredClone({segments:snapshot.segments,commands:a.standingCommands,contacts:a.lastContacts,state:snapshot.state}));
    }
  }finally{a.dispose();}
  const b=await original.createEmbodiedCharacter();
  try { for(const x of expected) {
    const y=characterFrame(b,1/60,null,'canvas2d');assert.deepEqual(x.segments,y.segments);assert.deepEqual(x.commands,b.standingCommands);
    assert.deepEqual(x.contacts,b.lastContacts);assert.equal(x.state,y.state);
  }}finally{b.dispose();}
});
