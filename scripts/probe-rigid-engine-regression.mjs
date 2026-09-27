import assert from 'node:assert/strict';
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { register } from 'tsx/esm/api';
import RAPIER from '@dimforge/rapier3d-compat';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';

const [mode, recipePath, output] = process.argv.slice(2);
assert.ok(mode === 'capture' || mode === 'run');
assert.ok(recipePath && (mode === 'capture' || output));
const destination = mode === 'capture' ? recipePath : output;
assert.ok(!existsSync(destination), 'Output must be fresh');
const unregister = register();
const { SEGMENTS } = await import('../src/core/humanoid.ts');
const { flattenGeometryVertices, flattenGeometryIndices } = await import('../src/core/geometry.ts');
const { quatMultiply, quatInverse } = await import('../src/character/math.ts');
const sourceBefore = fingerprints();
const vector = value => ({ x: value.x, y: value.y, z: value.z });
const rotation = value => ({ ...vector(value), w: value.w });
const bodyState = body => ({ position: vector(body.translation()), rotation: rotation(body.rotation()),
  com: vector(body.worldCom()), velocity: vector(body.linvel()), angularVelocity: vector(body.angvel()),
  mass: body.mass(), localCom: vector(body.localCom()), principalInertia: vector(body.principalInertia()),
  principalFrame: rotation(body.principalInertiaLocalFrame()) });
const parameters = ['dt', 'lengthUnit', 'numSolverIterations', 'numInternalPgsIterations', 'maxCcdSubsteps',
  'normalizedAllowedLinearError', 'normalizedPredictionDistance'];
const pair = (a, b) => a < b ? `${a}|${b}` : `${b}|${a}`;
const recipes = [], results = [];
try {
  await RAPIER.init();
  if (mode === 'capture') {
    assert.equal(RAPIER.version(), '0.20.0');
    const { createEmbodiedCharacter } = await import('../src/character/index.ts');
    for (const heading of [0, Math.PI / 3, -Math.PI / 4]) {
      const character = await createEmbodiedCharacter('canvas2d', { heading });
      try {
        const bodies = SEGMENTS.map(definition => {
          const body = character.ragdollBodies.get(definition.id), collider = character.ragdollColliders.get(definition.id);
          return { id: definition.id, initial: bodyState(body), damping: { linear: body.linearDamping(), angular: body.angularDamping() },
            ccd: body.isCcdEnabled(), softCcdPrediction: body.softCcdPrediction(), additionalSolverIterations: body.additionalSolverIterations(),
            vertices: [...flattenGeometryVertices(definition.geometry)], indices: [...flattenGeometryIndices(definition.geometry)],
            collider: { mass: collider.mass(), friction: collider.friction(), frictionCombineRule: collider.frictionCombineRule(),
              restitution: collider.restitution(), contactSkin: collider.contactSkin(), collisionGroups: collider.collisionGroups() } };
        });
        const byId = new Map(bodies.map(body => [body.id, body]));
        const joints = SEGMENTS.filter(d => d.parent).map(definition => ({ parent: definition.parent, child: definition.id,
          anchor1: definition.jointProfile.parentFrame.anchor, anchor2: definition.jointProfile.childFrame.anchor,
          frame1: quatMultiply(quatInverse(byId.get(definition.parent).initial.rotation), byId.get(definition.id).initial.rotation),
          frame2: { x: 0, y: 0, z: 0, w: 1 } }));
        const floor = character.floorCollider;
        recipes.push({ heading, gravity: character.world.gravity,
          parameters: Object.fromEntries(parameters.map(name => [name, character.world.integrationParameters[name]])),
          observedContactErp: character.world.integrationParameters.contact_erp,
          floor: { position: vector(floor.translation()), rotation: rotation(floor.rotation()), halfExtents: vector(floor.halfExtents()),
            friction: floor.friction(), restitution: floor.restitution(), collisionGroups: floor.collisionGroups() },
          bodies, joints, excludedPairs: [...new Set(SEGMENTS.flatMap(d => d.collisionExclusions.map(id => pair(d.id, id))))] });
      } finally { character.dispose(); }
    }
    writeFileSync(recipePath, JSON.stringify({ generatedAt: new Date().toISOString(), source: sourceBefore, recipes,
      qualification: 'Initial build recipes from the production 0.20.0 anatomy. The comparison constructs new unactuated fixed-joint fixtures, not the live articulated controller.' }, null, 2) + '\n', { flag: 'wx' });
  } else {
    const recipeBytes = readFileSync(recipePath), input = JSON.parse(recipeBytes);
    for (const recipe of input.recipes) {
      const world = new RAPIER.World(recipe.gravity), queue = new RAPIER.EventQueue(true);
      const bodies = new Map(), colliderIds = new Map(), exclusions = new Set(recipe.excludedPairs);
      const hooks = { filterContactPair: (a, b) => exclusions.has(pair(colliderIds.get(a), colliderIds.get(b))) ? null : RAPIER.SolverFlags.COMPUTE_IMPULSE,
        filterIntersectionPair: () => true };
      try {
        for (const [name, value] of Object.entries(recipe.parameters)) {
          world.integrationParameters[name] = value;
          assert.equal(world.integrationParameters[name], value, `Parameter readback: ${name}`);
        }
        const floor = recipe.floor;
        const floorBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(floor.position.x, floor.position.y, floor.position.z).setRotation(floor.rotation));
        world.createCollider(RAPIER.ColliderDesc.cuboid(floor.halfExtents.x, floor.halfExtents.y, floor.halfExtents.z)
          .setFriction(floor.friction).setRestitution(floor.restitution).setCollisionGroups(floor.collisionGroups), floorBody);
        world.step(queue, hooks); queue.clear();
        for (const data of recipe.bodies) {
          const initial = data.initial;
          const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic()
            .setTranslation(initial.position.x, initial.position.y, initial.position.z).setRotation(initial.rotation)
            .setLinvel(initial.velocity.x, initial.velocity.y, initial.velocity.z).setAngvel(initial.angularVelocity)
            .setCanSleep(false).setLinearDamping(data.damping.linear).setAngularDamping(data.damping.angular)
            .setCcdEnabled(data.ccd).setSoftCcdPrediction(data.softCcdPrediction).setAdditionalSolverIterations(data.additionalSolverIterations));
          const p = data.collider, descriptor = RAPIER.ColliderDesc.convexMesh(new Float32Array(data.vertices), new Uint32Array(data.indices));
          assert.ok(descriptor);
          const collider = world.createCollider(descriptor.setMass(p.mass).setFriction(p.friction).setFrictionCombineRule(p.frictionCombineRule)
            .setRestitution(p.restitution).setContactSkin(p.contactSkin).setCollisionGroups(p.collisionGroups)
            .setActiveHooks(RAPIER.ActiveHooks.FILTER_CONTACT_PAIRS), body);
          body.recomputeMassPropertiesFromColliders();
          bodies.set(data.id, body); colliderIds.set(collider.handle, data.id);
        }
        for (const joint of recipe.joints) world.createImpulseJoint(RAPIER.JointData.fixed(joint.anchor1, joint.frame1, joint.anchor2, joint.frame2),
          bodies.get(joint.parent), bodies.get(joint.child), true).setContactsEnabled(false);
        assert.equal(world.impulseJoints.len(), 24);
        const state = () => Object.fromEntries([...bodies].map(([id, body]) => [id, bodyState(body)]));
        const initial = state(), samples = [];
        for (const data of recipe.bodies) {
          const measured = initial[data.id];
          assert.ok(Math.abs(measured.mass - data.initial.mass) <= 1e-5 * data.initial.mass, `Mass mismatch: ${data.id}`);
          for (const axis of ['x', 'y', 'z']) assert.equal(measured.position[axis], data.initial.position[axis]);
        }
        for (let tick = 1; tick <= 720; tick++) {
          world.step(queue, hooks); queue.clear();
          if (tick <= 10 || tick % 60 === 0) samples.push({ tick, measured: state() });
        }
        results.push({ heading: recipe.heading, parameters: Object.fromEntries(parameters.map(name => [name, world.integrationParameters[name]])),
          observedContactErp: world.integrationParameters.contact_erp, initial, samples });
        console.log(JSON.stringify({ version: RAPIER.version(), heading: recipe.heading,
          pelvisAt120: samples.find(s => s.tick === 120).measured.pelvis.position, pelvisAt720: samples.at(-1).measured.pelvis.position }));
      } finally { queue.free(); world.free(); }
    }
    writeFileSync(output, JSON.stringify({ generatedAt: new Date().toISOString(), recipePath, recipeSha256: sha256(recipeBytes),
      runtime: globalThis[Symbol.for('laboratoire.rapierRuntimeOverride')], version: RAPIER.version(), sourceBefore, sourceAfter: fingerprints(), results,
      qualification: 'Unactuated fixed-joint fixture rebuilt from common recipes. Public configured parameters match; internal defaults and floating-point mass-property calculations may differ across engine versions. Not articulated standing acceptance.' }, null, 2) + '\n', { flag: 'wx' });
  }
  assert.deepEqual(fingerprints(), sourceBefore);
} finally { unregister(); }
