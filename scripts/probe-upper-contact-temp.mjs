import { createEmbodiedCharacter } from '../src/character/index.ts';
import { lerp, quatInverse, rotate, sub, worldPoint } from '../src/character/math.ts';

const dt = 1 / 60;
for (const [limb, route, heading] of [
  ['leftHand', 'chest', 0],
  ['leftFoot', 'midline', Math.PI / 3],
  ['rightFoot', 'midline', Math.PI / 3],
]) {
  const c = await createEmbodiedCharacter('canvas2d', { heading });
  for (let i = 0; i < 60; i++) c.fixedUpdate(dt, null);
  const initial = c.getSnapshot('canvas2d');
  const piece = initial.segments.find(p => p.id === limb);
  const start = piece.position;
  c.fixedUpdate(dt, { kind: 'begin', pointerId: 701, region: limb, segment: limb, localAnchor: {x:0,y:0,z:0}, worldTarget: start, timestampMs:1000 });
  const side = limb.startsWith('left') ? 1 : -1;
  const ref = initial.segments.find(p => p.id === (limb.endsWith('Hand') ? 'torso' : 'pelvis'));
  const local = route === 'chest' ? { x: side * .18, y: 0, z: .14 } : {x: side * .16, y: -.85, z: .13};
  const target = worldPoint(ref.position, ref.rotation, local);
  let worst = null;
  for (let frame = 1; frame <= 90; frame++) {
    c.fixedUpdate(dt, {kind:'move',pointerId:701,worldTarget:lerp(start,target,frame/90),timestampMs:(60+frame)*dt*1000});
    const torso = c.ragdollColliders.get('torso');
    for (const side of ['left','right']) {
      const id = `${side}UpperArm`;
      const arm = c.ragdollColliders.get(id);
      const contact = torso.contactCollider(arm, .03);
      if (!contact || (worst && contact.distance >= worst.distance)) continue;
      const armBody = c.ragdollBodies.get(id);
      const localArmPoint = rotate(quatInverse(armBody.rotation()), sub(contact.point2, armBody.translation()));
      worst = { frame, distance: contact.distance, id, localArmPoint, state:c.getSnapshot('canvas2d').state,
        torsoPosition:torso.translation(), armPosition:arm.translation(),
        armSkin:arm.contactSkin(), armVelocity:armBody.linvel() };
    }
  }
  c.fixedUpdate(dt, {kind:'end',pointerId:701,timestampMs:151*dt*1000});
  console.log(JSON.stringify({limb,route,heading,worst,diagnostics:c.diagnostics().maxSelfPenetrationM}));
  c.dispose();
}
