import assert from 'node:assert/strict';
import test, { after } from 'node:test';
import { register } from 'tsx/esm/api';
import RAPIER from '@dimforge/rapier3d-compat';
const unregister = register(); after(unregister);
const { createEmbodiedCharacter } = await import('../src/character/index.ts');
const { SEGMENTS, SEGMENT_BY_ID, TOTAL_MASS_KG } = await import('../src/core/humanoid.ts');
const { flattenGeometryVertices, flattenGeometryIndices } = await import('../src/core/geometry.ts');
const { jointCoordinates, jointRotationFromCoordinates } = await import('../src/character/joint-coordinates.ts');
const { configureHumanoidWorld } = await import('../src/character/physics-settings.ts');
const { massState } = await import('../src/character/BalanceController.ts');
const { distributeSupportLoad } = await import('../src/character/support-loads.ts');
const { add, sub, scale, rotate, quatInverse, quatFromAxisAngle, length } = await import('../src/character/math.ts');
const DT = 1/60, ZERO = {x:0,y:0,z:0}, UP = {x:0,y:1,z:0};
const rows = [];
function report(name, metrics) { rows.push({name,...metrics}); console.log('BODY_METRICS '+JSON.stringify(rows.at(-1))); }
function pairEnabled(c,a,b) { return c.physicsHooks.filterContactPair(c.ragdollColliders.get(a).handle,c.ragdollColliders.get(b).handle)!==null; }
function bodyMass(c) {
  let mass=0, com=ZERO, velocity=ZERO;
  for(const b of c.ragdollBodies.values()) { mass+=b.mass(); com=add(com,scale(b.worldCom(),b.mass())); velocity=add(velocity,scale(b.linvel(),b.mass())); }
  return {mass,com:scale(com,1/mass),velocity:scale(velocity,1/mass)};
}
function rawStep(c) { c.world.step(c.eventQueue,c.physicsHooks); c.readPhysicsPoses(); c.observeContacts(DT); }

for(const heading of [0,Math.PI/2,Math.PI]) test(`body orientation, collider pose and actual CoM at heading ${heading}`,async()=>{
 const c=await createEmbodiedCharacter('canvas2d',{heading}), reference=await createEmbodiedCharacter('canvas2d');
 try {
  const yaw=quatFromAxisAngle(UP,heading), snapshot=c.getSnapshot('canvas2d');
  let positionError=0, colliderError=0, footError=0, penetration=0;
  for(const d of SEGMENTS) {
   const p=c.ragdollBodies.get(d.id), b=reference.ragdollBodies.get(d.id), col=c.ragdollColliders.get(d.id);
   positionError=Math.max(positionError,length(sub(p.translation(),rotate(yaw,b.translation()))));
   colliderError=Math.max(colliderError,length(sub(p.translation(),col.translation())),length(sub(p.translation(),snapshot.segments.find(s=>s.id===d.id).position)));
   const inertia=p.principalInertia(); assert.ok([inertia.x,inertia.y,inertia.z].every(v=>Number.isFinite(v)&&v>0),d.id+' inertia');
   if(d.role==='hindfoot') { const forward=rotate(quatInverse(yaw),rotate(p.rotation(),{x:0,y:0,z:1}));footError=Math.max(footError,Math.abs(Math.atan2(forward.x,forward.z))*180/Math.PI); }
  }
  for(let i=0;i<SEGMENTS.length;i++) for(let j=i+1;j<SEGMENTS.length;j++) {
   const a=SEGMENTS[i].id,b=SEGMENTS[j].id;if(!pairEnabled(c,a,b))continue;
   const contact=c.ragdollColliders.get(a).contactCollider(c.ragdollColliders.get(b),0);
   if(contact)penetration=Math.max(penetration,-contact.distance);
  }
  const actual=bodyMass(c), computed=massState(new Map(snapshot.segments.map(p=>[p.id,p])));
  const comError=length(sub(actual.com,computed.position));
  report('orientation',{heading,positionErrorM:positionError,colliderErrorM:colliderError,footFacingErrorDeg:footError,initialPenetrationM:penetration,massKg:actual.mass,comErrorM:comError});
  assert.ok(positionError<=.0001);assert.ok(colliderError<1e-6);assert.ok(footError<2);
  assert.ok(penetration<=.001);assert.ok(Math.abs(actual.mass-TOTAL_MASS_KG)<.0001);assert.ok(comError<1e-6);
 }finally{c.dispose();reference.dispose();}
});

test('upper-arm surfaces are closed, outward and retain trunk contact',async()=>{
 const c=await createEmbodiedCharacter();try{
  for(const side of ['left','right']) {
   const d=SEGMENT_BY_ID.get(`${side}UpperArm`),edges=new Map();
   for(const tri of d.geometry.triangles)for(let i=0;i<3;i++) {const a=tri[i],b=tri[(i+1)%3],key=[Math.min(a,b),Math.max(a,b)].join(':');const e=edges.get(key)??{count:0,sign:0};e.count++;e.sign+=a<b?1:-1;edges.set(key,e);}
   assert.ok([...edges.values()].every(e=>e.count===2&&e.sign===0),side+' closed oriented 2-manifold');
   assert.ok(pairEnabled(c,d.id,'torso'));assert.ok(pairEnabled(c,d.id,'pelvis'));
  }
 }finally{c.dispose();}
});

for(const heading of [0,Math.PI/2,Math.PI]) test(`native positive flexion follows anatomy under actual dynamics ${heading}`,async()=>{
 for(const id of ['leftForearm','rightForearm','leftThigh','rightThigh','leftShin','rightShin','leftAnkle','rightAnkle']) {
  const c=await createEmbodiedCharacter('canvas2d',{heading});try{
   c.floorCollider.setEnabled(false);c.world.gravity=ZERO;c.nativeMotors.disable(c.jointsByChild);
   const d=SEGMENT_BY_ID.get(id), profile=d.jointProfile, parent=c.ragdollBodies.get(d.parent),child=c.ragdollBodies.get(id);
   const target={x:.25,y:0,z:0};
   for(let tick=0;tick<45;tick++) {c.nativeMotors.apply(c.ragdollBodies,c.jointsByChild,[{id,targetLocalRotation:jointRotationFromCoordinates(target,profile),stiffness:60,damping:8,strengthScale:1}]);rawStep(c);}
   const achieved=jointCoordinates(parent.rotation(),child.rotation(),profile);
   const relativeDown=rotate(quatInverse(parent.rotation()),rotate(child.rotation(),{x:0,y:-1,z:0}));
   const sign=d.role==='shin'?-1:1;
   report('flexion',{heading,id,angleRad:achieved.x,physicalDistalForward:relativeDown.z});
   assert.ok(achieved.x>.05,`${id} flexion ${achieved.x}`);
   assert.ok(sign*relativeDown.z>0,`${id} physical axis reversed`);
  }finally{c.dispose();}
 }
});

test('structurally locked hindfoot pitch resists applied torque, not a zero-width limit',async()=>{
 const c=await createEmbodiedCharacter();try {
  c.floorCollider.setEnabled(false);c.world.gravity=ZERO;c.nativeMotors.disable(c.jointsByChild);
  let error=0;
  for(let tick=0;tick<120;tick++) {
   for(const side of ['left','right']) {
    const parent=c.ragdollBodies.get(`${side}Ankle`),child=c.ragdollBodies.get(`${side}Foot`),impulse=rotate(parent.rotation(),{x:10*DT,y:0,z:0});
    child.applyTorqueImpulse(impulse,true);parent.applyTorqueImpulse(scale(impulse,-1),true);
   }
   rawStep(c);
   for(const side of ['left','right']) {const d=SEGMENT_BY_ID.get(`${side}Foot`);error=Math.max(error,Math.abs(jointCoordinates(c.ragdollBodies.get(d.parent).rotation(),c.ragdollBodies.get(d.id).rotation(),d.jointProfile).x));}
  }
  report('structural-foot-lock',{maxForbiddenPitchRad:error});assert.ok(error<.06);
 }finally{c.dispose();}
});

test('native CoM follows gravity when posture and environment support are disabled',async()=>{
 const c=await createEmbodiedCharacter();try{
  c.floorCollider.setEnabled(false);c.nativeMotors.disable(c.jointsByChild);
  const initial=bodyMass(c);let maxAccelerationError=0,maxSeparation=0;let previous=initial.velocity;
  for(let tick=0;tick<90;tick++) {rawStep(c);const current=bodyMass(c);maxAccelerationError=Math.max(maxAccelerationError,Math.abs((current.velocity.y-previous.y)/DT+9.81));previous=current.velocity;maxSeparation=Math.max(maxSeparation,c.maximumJointSeparation());assert.equal(c.lastContacts.filter(x=>x.loadBearing).length,0);}
  const final=bodyMass(c);report('passive-freefall',{dropM:initial.com.y-final.com.y,maxAccelerationErrorMps2:maxAccelerationError,maxJointSeparationM:maxSeparation});
  assert.ok(maxAccelerationError<.1);assert.ok(initial.com.y-final.com.y>9);assert.ok(maxSeparation<.01);
 }finally{c.dispose();}
});

test('support pressure retains measured owners, height and nonnegative normalized load',async()=>{
 const c=await createEmbodiedCharacter();try{
  for(let tick=0;tick<120;tick++)c.fixedUpdate(DT,null);
  const contacts=c.lastContacts.filter(x=>x.loadBearing);assert.ok(contacts.length);
  for(const requested of [bodyMass(c).com,{x:2,y:0,z:2},{x:-2,y:0,z:-2}]) {
   const loads=distributeSupportLoad(contacts,requested);assert.ok(loads.length);
   assert.ok(Math.abs(loads.reduce((s,l)=>s+l.share,0)-1)<1e-9);
   for(const l of loads) {assert.ok(l.share>=0);assert.ok(contacts.some(ct=>ct.segment===l.segment&&(ct.points??[ct.point]).some(p=>length(sub(p,l.point))<1e-9)));}
  }
  assert.deepEqual(distributeSupportLoad([],ZERO),[]);
  report('measured-support',{contacts:contacts.map(c=>({segment:c.segment,rawForceN:c.forceN,persistenceS:c.persistenceS})),measured:c.diagnostics().balance.measuredSupportingFeet});
 }finally{c.dispose();}
});

// Independent two-body contact fixtures use the SAME visible convex surfaces,
// segment masses and collision-pair policy. They supplement, not replace, the
// full articulated hand-across-chest case below.
for(const active of [false,true]) test(`anatomical self-contact blocks and slides, passive=${!active}`,async()=>{
 const c=await createEmbodiedCharacter();
 try {
  for(const [first,second] of [['rightHand','torso'],['rightForearm','torso'],['rightUpperArm','torso'],['leftHand','head'],['leftForearm','pelvis'],['leftForearm','rightForearm'],['rightForearm','leftThigh'],['leftThigh','rightThigh']])for(const speed of [.35,4]) {
   assert.ok(pairEnabled(c,first,second),`${first}/${second} filtered`);
   const w=new RAPIER.World(ZERO);configureHumanoidWorld(w);
   try {
    const a=SEGMENT_BY_ID.get(first),b=SEGMENT_BY_ID.get(second),gap=b.geometry.localBounds.max.z-a.geometry.localBounds.min.z+.025;
    const moving=w.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0,0,gap).setLinvel(0,0,-speed).setCcdEnabled(true).setCanSleep(false));
    const receiving=w.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setCanSleep(false));
    const make=(d,body)=>w.createCollider(RAPIER.ColliderDesc.convexMesh(flattenGeometryVertices(d.geometry),flattenGeometryIndices(d.geometry)).setMass(d.massKg).setFriction(.45).setRestitution(.02).setContactSkin(.0015),body);
    const ac=make(a,moving),bc=make(b,receiving);let touched=false,peak=0,sustained=0;const history=[];
    for(let tick=0;tick<90;tick++) {
     if(active&&tick>0&&tick<60) {const delta=sub(receiving.translation(),moving.translation());const force=scale(delta,120);moving.applyImpulse({x:Math.min(8,Math.max(-8,force.x))*DT,y:0,z:Math.min(20,Math.max(-20,force.z))*DT},true);}
     if(tick===45)moving.applyImpulse({x:.1*a.massKg,y:0,z:0},true);
     w.step();w.contactPair(ac,bc,m=>{if(m.numContacts())touched=true;});
     const ct=ac.contactCollider(bc,0),depth=ct?Math.max(0,-ct.distance):0;peak=Math.max(peak,depth);history.push(depth);if(history.length>12)history.shift();if(history.length===12)sustained=Math.max(sustained,Math.min(...history));
    }
    report('pair-contact',{first,second,speedMps:speed,active,contact:touched,peakPenetrationM:peak,sustainedPenetrationM:sustained,receiverTravelM:length(receiving.translation())});
    assert.ok(touched);assert.ok(peak<=.015);assert.ok(sustained<=.005);assert.ok(length(receiving.translation())>0);
   }finally{w.free();}
  }
 }finally{c.dispose();}
});

for(const side of ['right','left'])test(`actual ${side} hand drag across chest transmits load without sustained penetration`,async()=>{
 const c=await createEmbodiedCharacter();try {
  for(let tick=0;tick<120;tick++)c.fixedUpdate(DT,null);
  const id=`${side}Hand`,start={...c.ragdollBodies.get(id).translation()},root={...c.ragdollBodies.get('pelvis').translation()},torso={...c.ragdollBodies.get('torso').translation()};
  const target={x:side==='right'?-.25:.25,y:torso.y,z:torso.z+.01};let peak=0,sustained=0,joints=0,touched=false;const depths=[],states=new Set(),phases=new Set();
  c.fixedUpdate(DT,{kind:'begin',pointerId:301,region:id,segment:id,localAnchor:ZERO,worldTarget:start,timestampMs:0});
  for(let tick=1;tick<=240;tick++) {
   const amount=Math.min(tick/120,1),position=add(start,scale(sub(target,start),amount));
   c.fixedUpdate(DT,{kind:'move',pointerId:301,worldTarget:position,timestampMs:tick*1000/60});
   states.add(c.state);phases.add(c.step?.phase??'none');joints=Math.max(joints,c.maximumJointSeparation());
   let depth=0;
   for(const segment of [id,`${side}Forearm`,`${side}UpperArm`]) {
    const a=c.ragdollColliders.get(segment),b=c.ragdollColliders.get('torso');c.world.contactPair(a,b,m=>{if(m.numContacts())touched=true;});
    const contact=a.contactCollider(b,0);depth=Math.max(depth,contact?Math.max(0,-contact.distance):0);
   }
   peak=Math.max(peak,depth);depths.push(depth);if(depths.length>12)depths.shift();if(depths.length===12)sustained=Math.max(sustained,Math.min(...depths));
  }
  report('articulated-chest-drag',{side,contact:touched,peakPenetrationM:peak,sustainedPenetrationM:sustained,maxJointSeparationM:joints,rootTravelM:length(sub(c.ragdollBodies.get('pelvis').translation(),root)),states:[...states],phases:[...phases]});
  assert.ok(touched);assert.ok(peak<=.015);assert.ok(sustained<=.005);assert.ok(joints<=.01);
 }finally{c.dispose();}
});

test('support disappears on actual liftoff and returns only after real touchdown',async()=>{
 const c=await createEmbodiedCharacter();try{
  for(let tick=0;tick<120;tick++)c.fixedUpdate(DT,null);
  assert.ok(c.getSnapshot('canvas2d').support.planted.length>0);
  c.nativeMotors.disable(c.jointsByChild);c.floorCollider.setEnabled(false);
  for(let tick=0;tick<6;tick++) {rawStep(c);assert.equal(c.lastContacts.filter(ct=>ct.loadBearing).length,0);assert.deepEqual(c.getSnapshot('canvas2d').support.planted,[]);}
  c.floorCollider.setEnabled(true);let firstContact=-1,firstLoaded=-1;const transitions=[];
  for(let tick=0;tick<60;tick++) {
   rawStep(c);const contacts=c.lastContacts.filter(ct=>['leftFoot','rightFoot','leftForefoot','rightForefoot'].includes(ct.segment));
   if(contacts.length&&firstContact<0)firstContact=tick;
   if(contacts.some(ct=>ct.loadBearing)&&firstLoaded<0)firstLoaded=tick;
   const planted=c.getSnapshot('canvas2d').support.planted;
   for(const foot of planted)assert.ok(contacts.some(ct=>ct.loadBearing&&ct.segment.startsWith(foot.startsWith('left')?'left':'right')));
   transitions.push({tick,planted,loads:contacts.map(ct=>({segment:ct.segment,forceN:ct.forceN,persistenceS:ct.persistenceS,loadBearing:ct.loadBearing}))});
  }
  report('liftoff-touchdown',{firstContactTick:firstContact,firstLoadedTick:firstLoaded,transitions});
  assert.ok(firstContact>=0);assert.ok(firstLoaded>=firstContact);
 }finally{c.dispose();}
});
