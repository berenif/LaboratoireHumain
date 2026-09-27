import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { closeSync, existsSync, mkdirSync, openSync, writeFileSync, writeSync } from 'node:fs';
import { join } from 'node:path';
import { register } from 'tsx/esm/api';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';
import { stepLoadDiagnostics } from './balance-probe-diagnostics.mjs';
import { installHybridStandingConstraints } from './hybrid-standing-constraints.mjs';
import { holdPatchStandingLoads, preferHindfootStandingLoads, projectHindfootStandingLoads, projectHindfootHullStandingLoads, projectMeasuredContourStandingLoads, projectEqualContourStandingLoads, projectMeasuredPressureStandingLoads, projectStandingLoads } from './standing-support-projection.mjs';

const unregister = register();
const { createEmbodiedCharacter } = await import('../src/character/index.ts');
const { SEGMENT_BY_ID, TOTAL_MASS_KG } = await import('../src/core/humanoid.ts');
const { jointCoordinates, clampJointCoordinates, jointCoordinateTargetError, jointFrameAxesWorld } = await import('../src/character/joint-coordinates.ts');
const { add, angularVelocity, clamp, clampLength, cross, dot, length, quatInverse, quatMultiply, scale, sub, worldPoint } = await import('../src/character/math.ts');
const { measureMassState } = await import('../src/character/mass-state.ts');
const { distributeSupportLoad } = await import('../src/character/support-loads.ts');
const { recoverySupportHull } = await import('../src/character/recovery-support.ts');
const { composeUprightPose } = await import('../src/character/pose.ts');
const { createCenteredStandingCharacter } = await import('./centered-standing-initialization.mjs');
const { installStandingResponseModel } = await import('./standing-response-model.mjs');
const { installForefootAuthority } = await import('./standing-forefoot-authority.mjs');
const { installDistalPreview } = await import('./standing-distal-preview.mjs');
const { installIntegratedInertia } = await import('./standing-inertia-calibration.mjs');
const zero = { x: 0, y: 0, z: 0 }, identity = { ...zero, w: 1 }, dt = 1 / 60;
const output = process.argv[2];
if (!output || existsSync(join(output, 'report.json'))) throw new Error('Specify a fresh output directory.');
mkdirSync(output, { recursive: true });
const frames = Number(process.env.STANDING_FRAMES ?? 240);
const startTick = Number(process.env.STANDING_START_TICK ?? 121);
const durationTicks = Number(process.env.STANDING_PULSE_TICKS ?? 6);
const traceStartTick = Number(process.env.STANDING_TRACE_START_TICK ?? 1);
const traceEndTick = Number(process.env.STANDING_TRACE_END_TICK ?? frames);
const traceContacts = process.env.STANDING_TRACE_CONTACTS === '1';
const traceKinematics = process.env.STANDING_TRACE_KINEMATICS === '1';
const amplitude = Number(process.env.STANDING_PULSE_NM ?? 0.5);
const headings = JSON.parse(process.env.STANDING_HEADINGS ?? '[0,1.0471975511965976,-0.7853981633974483]');
const modes = (process.env.STANDING_MODES ?? 'plain,observe,observe-repeat,pulse-plus,pulse-minus,no-posture,no-gravity,no-pelvis').split(',');
const allowedModes = new Set(['plain', 'observe', 'observe-repeat', 'pulse-plus', 'pulse-minus', 'no-posture', 'no-gravity', 'no-pelvis',
  'arm-world-damping', 'hold-idle-references', 'held-world-damping', 'hold-leg-gravity', 'hold-foot-gravity', 'project-loads', 'projected-world-damping',
  'hold-patch-loads', 'held-patch-world-damping', 'project-prior-loads', 'projected-prior-world-damping', 'held-prior-world-damping',
  'hindfoot-loads', 'hindfoot-world-damping', 'projected-hindfoot-loads', 'projected-hindfoot-world-damping',
  'contour-hindfoot-loads', 'contour-hindfoot-world-damping', 'held-contour-hindfoot-world-damping',
  'measured-contour-loads', 'measured-contour-world-damping', 'tracking-held-contour-hindfoot-world-damping',
  'servo-contour-hindfoot-world-damping', 'distal-contour-hindfoot-world-damping', 'equal-contour-loads', 'equal-contour-world-damping',
  'pressure-contour-loads', 'pressure-contour-world-damping', 'held-pressure-contour-world-damping', 'quiet-held-pressure-contour-world-damping',
  'servo-quiet-held-pressure-contour-world-damping', 'whole-servo-quiet-held-pressure-contour-world-damping',
  'displacement-servo-quiet-held-pressure-contour-world-damping', 'fixed-leg-quiet-held-pressure-contour-world-damping', 'hybrid-translations', 'hybrid-hinges',
  'centered-quiet-held-pressure-contour-world-damping', 'smooth-centered-quiet-held-pressure-contour-world-damping',
  'initial-centered-quiet-held-pressure-contour-world-damping', 'integral-quiet-held-pressure-contour-world-damping',
  'interval-quiet-held-pressure-contour-world-damping', 'model-observe-quiet-held-pressure-contour-world-damping',
  'inertia-plain', 'inertia-quiet-held-pressure-contour-world-damping', 'forefoot-frame-quiet-held-pressure-contour-world-damping',
  'authority-observe-quiet-held-pressure-contour-world-damping', 'authority-pulse-quiet-held-pressure-contour-world-damping',
  'authority-distal-observe-quiet-held-pressure-contour-world-damping', 'authority-distal-pulse-quiet-held-pressure-contour-world-damping',
  'preview-distal-quiet-held-pressure-contour-world-damping', 'preview-horizontal-quiet-held-pressure-contour-world-damping',
  'preview-leg-quiet-held-pressure-contour-world-damping', 'preview-pelvis-quiet-held-pressure-contour-world-damping',
  'preview-pelvis-full-angular-quiet-held-pressure-contour-world-damping',
  'preview-pelvis-full-angular-arms-quiet-held-pressure-contour-world-damping',
  'preview-pelvis-full-angular-arms-position-quiet-held-pressure-contour-world-damping',
  'preview-pelvis-full-angular-arms-position-coupled-quiet-held-pressure-contour-world-damping']);
assert.ok(modes.every(mode => allowedModes.has(mode)), 'Unknown experimental mode');
assert.ok(!modes.some(mode => mode.startsWith('hybrid-')) || startTick === 1, 'Hybrid constraints are an initialization intervention');
assert.ok(!modes.some(mode => mode.startsWith('inertia-')) || startTick === 1, 'Inertia correction is an initialization intervention');
assert.ok([frames, startTick, durationTicks].every(value => Number.isInteger(value) && value > 0), 'Frame counts must be positive integers');
assert.ok([traceStartTick, traceEndTick].every(value => Number.isInteger(value) && value > 0)
  && traceStartTick <= traceEndTick && traceEndTick <= frames, 'Invalid trace range');
assert.ok(Number.isFinite(amplitude) && amplitude > 0 && headings.length > 0 && headings.every(Number.isFinite), 'Invalid pulse or heading');
const sourceBefore = fingerprints(), started = new Date().toISOString();
const copy = value => JSON.parse(JSON.stringify(value));
const horizontalDistance = (a, b) => Math.hypot(a.x - b.x, a.z - b.z);
const physical = c => ({ poses: [...c.poses.values()], contacts: c.lastContacts, state: c.state, step: c.step, stepCount: c.stepCount });
const references = c => copy({ feet: c.balance.feet, rotations: c.balance.footRotations, supportTarget: c.balance.supportTarget,
  nominalHeight: c.balance.nominalHeight, neutralComOffset: c.balance.neutralComOffset, neutralRootFromCom: c.balance.neutralRootFromCom });

function groundManifolds(c) {
  const result = [];
  for (const id of ['leftFoot', 'leftForefoot', 'rightFoot', 'rightForefoot']) {
    const body = c.ragdollBodies.get(id), collider = c.ragdollColliders.get(id);
    c.world.contactPair(collider, c.floorCollider, manifold => {
      const impulses = Array.from({ length: manifold.numContacts() }, (_, index) => ({
        normal: manifold.contactImpulse(index), tangentX: manifold.contactTangentImpulseX(index),
        tangentY: manifold.contactTangentImpulseY(index), distance: manifold.contactDist(index) }));
      result.push({ segment: id, normal: manifold.normal(), friction: manifold.friction(), impulses,
        points: Array.from({ length: manifold.numSolverContacts() }, (_, index) => {
          const point = manifold.solverContactPoint(index);
          return { point, velocity: point ? body.velocityAtPoint(point) : null, distance: manifold.solverContactDist(index) };
        }) });
    });
  }
  return result;
}

function instrument(c, mode, trace) {
  const stanceIntent = copy(c.balance.feet);
  const holding = ['hold-idle-references', 'held-world-damping', 'held-prior-world-damping', 'held-contour-hindfoot-world-damping', 'tracking-held-contour-hindfoot-world-damping', 'held-pressure-contour-world-damping', 'quiet-held-pressure-contour-world-damping', 'servo-quiet-held-pressure-contour-world-damping', 'whole-servo-quiet-held-pressure-contour-world-damping', 'displacement-servo-quiet-held-pressure-contour-world-damping', 'fixed-leg-quiet-held-pressure-contour-world-damping', 'centered-quiet-held-pressure-contour-world-damping', 'smooth-centered-quiet-held-pressure-contour-world-damping', 'initial-centered-quiet-held-pressure-contour-world-damping', 'integral-quiet-held-pressure-contour-world-damping', 'interval-quiet-held-pressure-contour-world-damping', 'model-observe-quiet-held-pressure-contour-world-damping', 'forefoot-frame-quiet-held-pressure-contour-world-damping'].includes(mode) || mode.startsWith('authority-') || mode.startsWith('preview-');
  const interventionActive = () => c.fixedSteps >= startTick && c.fixedSteps < startTick + durationTicks;
  let centeredNeutral;
  if (mode.includes('centered-')) {
    const hull = recoverySupportHull(['leftFoot', 'leftForefoot', 'rightFoot', 'rightForefoot'].flatMap(id => {
      const body = c.ragdollBodies.get(id);
      return SEGMENT_BY_ID.get(id).geometry.supportPatch.map(point => worldPoint(body.translation(), body.rotation(), point));
    }));
    let area2 = 0, x = 0, z = 0;
    for (let i = 0; i < hull.length; i++) {
      const a = hull[i], b = hull[(i + 1) % hull.length], signed = a.x * b.z - b.x * a.z;
      area2 += signed; x += (a.x + b.x) * signed; z += (a.z + b.z) * signed;
    }
    assert.ok(Math.abs(area2) > 1e-6, 'Degenerate planned support hull');
    const centroid = { x: x / (3 * area2), y: 0, z: z / (3 * area2) };
    centeredNeutral = { originalOffset: copy(c.balance.neutralComOffset),
      targetOffset: { ...sub(centroid, c.balance.supportTarget), y: 0 }, centroid, hull };
  }
  if (holding) {
    const mayHold = () => interventionActive() && !c.step && !c.activeGrab && c.stepCount === 0;
    for (const name of ['feet', 'footRotations']) {
      c.balance[name] = new Proxy(c.balance[name], { set(target, key, value) {
        if (!mayHold()) target[key] = value;
        return true;
      } });
    }
    let heldTarget = c.balance.supportTarget;
    Object.defineProperty(c.balance, 'supportTarget', { get: () => heldTarget, set: value => {
      if (!mayHold()) heldTarget = value;
    } });
  }
  const gravity = new Map();
  const previousGravity = new Map();
  const originalGravity = c.gravityCompensation.bind(c);
  c.gravityCompensation = definition => {
    const value = originalGravity(definition);
    const selected = mode === 'hold-leg-gravity' && ['thigh', 'shin', 'ankle', 'hindfoot', 'forefoot'].includes(definition.role)
      || mode === 'hold-foot-gravity' && ['hindfoot', 'forefoot'].includes(definition.role);
    const applied = selected && interventionActive() ? previousGravity.get(definition.id) ?? value : value;
    previousGravity.set(definition.id, value);
    gravity.set(definition.id, applied);
    if (selected && interventionActive()) {
      trace.gravityIntervention ??= {};
      trace.gravityIntervention[definition.id] = { unmodifiedWorld: value, appliedWorld: applied };
    }
    return applied;
  };
  let inertia = new Map();
  const originalDynamics = c.legTargetDynamics.sample.bind(c.legTargetDynamics);
  c.legTargetDynamics.sample = (...args) => {
    inertia = originalDynamics(...args);
    return inertia;
  };
  const originalBalance = c.balance.update.bind(c.balance);
  c.balance.update = input => {
    if (centeredNeutral) {
      const applied = interventionActive() && !c.step && !c.activeGrab && c.stepCount === 0;
      const u = clamp((c.fixedSteps - startTick) * dt / 2, 0, 1);
      const progress = mode.startsWith('smooth-centered-') ? u ** 3 * (10 + u * (-15 + 6 * u)) : 1;
      const currentOffset = progress === 1 ? centeredNeutral.targetOffset : add(centeredNeutral.originalOffset,
        scale(sub(centeredNeutral.targetOffset, centeredNeutral.originalOffset), progress));
      c.balance.neutralComOffset = applied ? currentOffset : centeredNeutral.originalOffset;
      trace.centeredNeutral = { ...centeredNeutral, applied, progress, currentOffset: copy(c.balance.neutralComOffset) };
    }
    const before = references(c), stepBefore = copy(c.step);
    const result = originalBalance(input);
    const after = references(c);
    trace.references = { before, after, rootTarget: result.rootTarget, reactionOffset: result.reactionOffset,
      kneeFlexion: result.kneeFlexion, measuredLegFrame: true,
      reasons: Object.fromEntries(['leftFoot', 'rightFoot'].map(foot => [foot,
        holding && interventionActive() ? 'experiment: hold idle posture reference' : stepBefore?.foot === foot ? 'step state machine' : input.contacts?.some(contact => contact.loadBearing && contact.segment === foot)
          ? 'adopt measured horizontal sole position; flat-ground height and yaw' : 'hold previous reference'])),
      supportTargetReason: holding && interventionActive() ? 'experiment: hold idle posture references; measured contacts unchanged' : result.step ? 'transfer/swing state machine' : result.stepCount === 0 ? 'rebase to measured support ankle center each tick' : 'hold after completed step',
      legRootReason: 'leg IK uses measured physical pelvis; virtual root target drives height and pelvis posture' };
    return result;
  };
  const originalCommands = c.motorCommands.bind(c);
  const contributions = new Map();
  const previousTargets = new Map();
  const initialLegTargets = new Map();
  const postureIntegral = new Map();
  if (mode.startsWith('integral-')) trace.postureIntegralSummary = {
    integrationTimeS: 2, activeTicks: 0, saturationUpdatesRejected: 0, maxIntegralTorqueNm: 0,
  };
  const previousBodyRotations = new Map([...c.poses].map(([id, pose]) => [id, { ...pose.rotation }]));
  if (mode.startsWith('interval-')) trace.intervalDampingSummary = {
    activeTicks: 0, maxAxisCorrectionNm: 0, maxPublishedRelativeSpeed: 0, maxIntervalRelativeSpeed: 0,
  };
  const previousFootPositions = new Map(['leftFoot', 'rightFoot'].map(id => [id, { ...c.poses.get(id).position }]));
  let previousPlan;
  c.motorCommands = targets => {
    const measuredAngularRates = new Map();
    if (mode.startsWith('interval-')) for (const [id, pose] of c.poses) {
      measuredAngularRates.set(id, angularVelocity(previousBodyRotations.get(id), pose.rotation, dt));
      previousBodyRotations.set(id, { ...pose.rotation });
    }
    const intervalEnabled = mode.startsWith('interval-') && interventionActive() && !c.step && !c.activeGrab && c.stepCount === 0;
    if (intervalEnabled) trace.intervalDampingSummary.activeTicks++;
    const loadedSides = new Set(c.lastContacts.filter(contact => contact.loadBearing && contact.forceN >= 3
      && contact.normalY >= 0.65 && contact.persistenceS >= 0.05
      && ['hindfoot', 'forefoot'].includes(SEGMENT_BY_ID.get(contact.segment)?.role))
      .map(contact => SEGMENT_BY_ID.get(contact.segment).side));
    const integralEnabled = mode.startsWith('integral-') && interventionActive()
      && !c.step && !c.activeGrab && c.stepCount === 0 && loadedSides.has('left') && loadedSides.has('right');
    if (!integralEnabled) postureIntegral.clear();
    else trace.postureIntegralSummary.activeTicks++;
    const measuredFootRates = new Map();
    for (const id of ['leftFoot', 'rightFoot']) {
      const position = c.poses.get(id).position;
      measuredFootRates.set(id, scale(sub(position, previousFootPositions.get(id)), 1 / dt));
      previousFootPositions.set(id, { ...position });
    }
    if (interventionActive() && mode.includes('quiet-held-') && !c.step && !c.activeGrab) {
      const quiet = composeUprightPose({ rootTranslation: trace.references.rootTarget, heading: c.heading,
        kneeFlexion: c.kneeFlexion, reactionOffset: c.reactionOffset, simulationTime: 0, activeGrab: c.activeGrab,
        supportFeet: c.supportFeet, supportFootRotations: c.supportFootRotations,
        measuredPoses: c.poses, measuredLegFrame: true, step: c.step }).poses;
      const revised = new Map(targets);
      for (const [id, pose] of quiet) {
        if (['shoulder-girdle', 'upper-arm', 'forearm', 'forearm-twist', 'hand'].includes(SEGMENT_BY_ID.get(id).role)) revised.set(id, pose);
        else assert.deepEqual(pose, targets.get(id), 'Idle oscillator intervention changed a non-arm target');
      }
      trace.idleTargetIntervention = { simulationTimeForArms: 0, actualSimulationTime: c.simulationTime };
      targets = revised;
    }
    delete trace.unmodifiedContactPlan;
    delete trace.loadProjection;
    delete trace.gravityIntervention;
    delete trace.forefootFrame;
    if (interventionActive() && ['project-loads', 'projected-world-damping'].includes(mode)) {
      const contacts = c.lastContacts.filter(contact => c.contactLoadPlan.loads.some(load => load.segment === contact.segment));
      trace.unmodifiedContactPlan = copy(c.contactLoadPlan);
      const projection = projectStandingLoads(c.contactLoadPlan, contacts, distributeSupportLoad);
      c.contactLoadPlan = projection.plan;
      trace.loadProjection = { reason: projection.reason, iterations: projection.iterations };
    }
    if (interventionActive() && ['hold-patch-loads', 'held-patch-world-damping'].includes(mode)) {
      const contacts = c.lastContacts.filter(contact => c.contactLoadPlan.loads.some(load => load.segment === contact.segment));
      trace.unmodifiedContactPlan = copy(c.contactLoadPlan);
      const projection = holdPatchStandingLoads(c.contactLoadPlan, contacts, distributeSupportLoad, previousPlan);
      c.contactLoadPlan = projection.plan;
      trace.loadProjection = { reason: projection.reason, iterations: projection.iterations };
    }
    if (interventionActive() && ['project-prior-loads', 'projected-prior-world-damping', 'held-prior-world-damping'].includes(mode)) {
      const contacts = c.lastContacts.filter(contact => c.contactLoadPlan.loads.some(load => load.segment === contact.segment));
      trace.unmodifiedContactPlan = copy(c.contactLoadPlan);
      const projection = previousPlan ? projectStandingLoads(c.contactLoadPlan, contacts, distributeSupportLoad, previousPlan)
        : { plan: c.contactLoadPlan, reason: 'initialize' };
      c.contactLoadPlan = projection.plan;
      trace.loadProjection = { reason: projection.reason, iterations: projection.iterations };
    }
    previousPlan = copy(c.contactLoadPlan);
    if (interventionActive() && (mode.startsWith('measured-contour') || mode.startsWith('equal-contour') || mode.includes('pressure-contour'))) {
      const contacts = c.lastContacts.filter(contact => c.contactLoadPlan.loads.some(load => load.segment === contact.segment));
      trace.unmodifiedContactPlan = copy(c.contactLoadPlan);
      const solve = mode.includes('pressure-contour') ? projectMeasuredPressureStandingLoads
        : mode.startsWith('equal-') ? projectEqualContourStandingLoads : projectMeasuredContourStandingLoads;
      const projection = solve(c.contactLoadPlan, contacts, distributeSupportLoad, null, recoverySupportHull);
      c.contactLoadPlan = projection.plan;
      trace.loadProjection = { reason: projection.reason, iterations: projection.iterations };
    }
    if (interventionActive() && mode.includes('hindfoot')) {
      const contacts = c.lastContacts.filter(contact => c.contactLoadPlan.loads.some(load => load.segment === contact.segment));
      trace.unmodifiedContactPlan = copy(c.contactLoadPlan);
      const projection = mode.includes('contour-hindfoot') ? projectHindfootHullStandingLoads(c.contactLoadPlan, contacts, distributeSupportLoad, null, recoverySupportHull)
        : mode.startsWith('projected-') ? projectHindfootStandingLoads(c.contactLoadPlan, contacts, distributeSupportLoad)
        : preferHindfootStandingLoads(c.contactLoadPlan, contacts, distributeSupportLoad);
      c.contactLoadPlan = projection.plan;
      trace.loadProjection = { reason: projection.reason };
    }
    const commands = originalCommands(targets);
    const active = c.fixedSteps >= startTick && c.fixedSteps < startTick + durationTicks;
    trace.interventionActive = active;
    contributions.clear();
    return commands.map(original => {
      const definition = SEGMENT_BY_ID.get(original.id);
      let pelvis = zero;
      if (definition.role === 'thigh' && c.contactLoadPlan.loads.length) {
        const pose = c.poses.get('pelvis');
        const total = c.contactLoadPlan.loads.reduce((sum, load) => sum + load.measuredForceN, 0);
        const side = c.contactLoadPlan.loads.reduce((sum, load) => sum + (SEGMENT_BY_ID.get(load.segment)?.side === definition.side ? load.measuredForceN : 0), 0);
        const error = angularVelocity(pose.rotation, targets.get('pelvis').rotation, 1);
        pelvis = scale(clampLength(add(scale(error, 250), scale(pose.angularVelocity, -30)), 100), total > 0 ? -side / total : 0);
      }
      const grav = gravity.get(original.id) ?? zero, inertial = inertia.get(original.id) ?? zero;
      const trajectory = sub(sub(sub(original.feedforwardWorld ?? zero, grav), pelvis), inertial);
      const terms = { gravityLoad: grav, pelvis, inertial, trajectory, perturbation: zero, trackingFrameDamping: zero, standingTargetRate: zero, stanceTask: zero, postureIntegral: zero, intervalDamping: zero };
      let command = original;
      const forefootWorldFrame = mode.startsWith('forefoot-frame-') && active && definition.role === 'forefoot'
        && !c.step && !c.activeGrab && c.stepCount === 0;
      if (forefootWorldFrame) {
        // The forefoot is beyond the IK hindfoot endpoint. Its world sole
        // orientation can compensate hindfoot tracking error without changing
        // any proximal leg target or the planned hindfoot endpoint.
        command = { ...command, targetLocalRotation: quatMultiply(
          quatInverse(c.poses.get(definition.parent).rotation), targets.get(command.id).rotation) };
        trace.forefootFrame ??= {};
        trace.forefootFrame[command.id] = { originalTargetLocalRotation: original.targetLocalRotation,
          revisedTargetLocalRotation: command.targetLocalRotation, worldTargetRotation: targets.get(command.id).rotation,
          measuredParentRotation: c.poses.get(definition.parent).rotation };
      }
      if (mode.startsWith('fixed-leg-') && ['thigh', 'shin', 'ankle', 'hindfoot', 'forefoot'].includes(definition.role)) {
        if (!initialLegTargets.has(command.id)) initialLegTargets.set(command.id, copy(command.targetLocalRotation));
        if (active && !c.step && !c.activeGrab) command = { ...command, targetLocalRotation: initialLegTargets.get(command.id) };
      }
      if (active && mode === 'no-posture') command = { ...command, stiffness: 0 };
      if (active && mode === 'no-gravity') {
        command = { ...command, feedforwardWorld: sub(command.feedforwardWorld ?? zero, grav) };
        terms.gravityLoad = zero;
      }
      if (active && mode === 'no-pelvis') {
        command = { ...command, feedforwardWorld: sub(command.feedforwardWorld ?? zero, pelvis) };
        terms.pelvis = zero;
      }
      if (active && mode.endsWith('world-damping') && (['shoulder-girdle', 'upper-arm', 'forearm', 'forearm-twist', 'hand'].includes(definition.role)
        || mode.startsWith('whole-') && !['thigh', 'shin', 'ankle', 'hindfoot', 'forefoot'].includes(definition.role)
        || mode.startsWith('distal-') && ['ankle', 'hindfoot', 'forefoot'].includes(definition.role)
        || forefootWorldFrame)) {
        terms.trackingFrameDamping = scale(c.poses.get(definition.parent).angularVelocity, -command.damping);
        command = { ...command, feedforwardWorld: add(command.feedforwardWorld ?? zero, terms.trackingFrameDamping) };
      }
      const currentTarget = clampJointCoordinates(jointCoordinates(identity, command.targetLocalRotation, definition.jointProfile), definition.jointProfile);
      const priorTarget = previousTargets.get(command.id);
      if (intervalEnabled && ['thigh', 'shin', 'ankle', 'hindfoot', 'forefoot'].includes(definition.role)) {
        const parent = c.poses.get(definition.parent), child = c.poses.get(command.id);
        const publishedRelative = sub(child.angularVelocity, parent.angularVelocity);
        const intervalRelative = sub(measuredAngularRates.get(command.id), measuredAngularRates.get(definition.parent));
        terms.intervalDamping = scale(sub(publishedRelative, intervalRelative), command.damping);
        command = { ...command, feedforwardWorld: add(command.feedforwardWorld ?? zero, terms.intervalDamping) };
        const basis = jointFrameAxesWorld(parent.rotation, definition.jointProfile);
        for (const axis of definition.jointProfile.axes) {
          trace.intervalDampingSummary.maxAxisCorrectionNm = Math.max(trace.intervalDampingSummary.maxAxisCorrectionNm,
            Math.abs(dot(terms.intervalDamping, basis[axis.coordinate])) * clamp(command.strengthScale, 0, 1));
          trace.intervalDampingSummary.maxPublishedRelativeSpeed = Math.max(trace.intervalDampingSummary.maxPublishedRelativeSpeed,
            Math.abs(dot(publishedRelative, basis[axis.coordinate])));
          trace.intervalDampingSummary.maxIntervalRelativeSpeed = Math.max(trace.intervalDampingSummary.maxIntervalRelativeSpeed,
            Math.abs(dot(intervalRelative, basis[axis.coordinate])));
        }
      }
      if (integralEnabled && ['thigh', 'shin', 'ankle', 'hindfoot', 'forefoot'].includes(definition.role)) {
        const parent = c.poses.get(definition.parent), child = c.poses.get(command.id);
        const measured = jointCoordinates(parent.rotation, child.rotation, definition.jointProfile);
        const error = jointCoordinateTargetError(measured, currentTarget, definition.jointProfile);
        const basis = jointFrameAxesWorld(parent.rotation, definition.jointProfile);
        const relativeVelocity = sub(child.angularVelocity, parent.angularVelocity);
        for (const axis of definition.jointProfile.axes) {
          const key = `${command.id}:${axis.coordinate}`, cap = axis.maxMotorTorqueNm;
          const previous = postureIntegral.get(key) ?? 0;
          const increment = command.stiffness * error[axis.coordinate] * dt / trace.postureIntegralSummary.integrationTimeS;
          let next = clamp(previous + increment, -cap, cap);
          const withoutIntegral = command.stiffness * error[axis.coordinate]
            - command.damping * dot(relativeVelocity, basis[axis.coordinate])
            + dot(command.feedforwardWorld ?? zero, basis[axis.coordinate]);
          if (Math.abs(withoutIntegral + next) > cap && increment * (withoutIntegral + next) > 0) {
            next = previous;
            trace.postureIntegralSummary.saturationUpdatesRejected++;
          }
          assert.ok(Number.isFinite(next) && Math.abs(next) <= cap);
          postureIntegral.set(key, next);
          terms.postureIntegral = add(terms.postureIntegral, scale(basis[axis.coordinate], next));
          trace.postureIntegralSummary.maxIntegralTorqueNm = Math.max(trace.postureIntegralSummary.maxIntegralTorqueNm,
            Math.abs(next) * clamp(command.strengthScale, 0, 1));
        }
        command = { ...command, feedforwardWorld: add(command.feedforwardWorld ?? zero, terms.postureIntegral) };
      }
      if (active && mode.startsWith('tracking-held-') && !c.step && priorTarget
        && ['thigh', 'shin', 'ankle', 'hindfoot', 'forefoot'].includes(definition.role)) {
        const delta = jointCoordinateTargetError(priorTarget, currentTarget, definition.jointProfile);
        const basis = jointFrameAxesWorld(c.poses.get(definition.parent).rotation, definition.jointProfile);
        terms.standingTargetRate = definition.jointProfile.axes.reduce((sum, axis) => add(sum,
          scale(basis[axis.coordinate], clamp(command.damping * delta[axis.coordinate] / dt,
            -axis.maxMotorTorqueNm, axis.maxMotorTorqueNm))), zero);
        command = { ...command, feedforwardWorld: add(command.feedforwardWorld ?? zero, terms.standingTargetRate) };
      }
      previousTargets.set(command.id, currentTarget);
      if (active && mode.includes('servo-') && !c.step && !c.activeGrab
        && ['thigh', 'shin', 'ankle', 'hindfoot'].includes(definition.role)) {
        const foot = `${definition.side}Foot`, pose = c.poses.get(foot);
        const qualified = c.lastContacts.filter(contact => contact.loadBearing && contact.forceN >= 3 && contact.normalY >= .65
          && ['hindfoot', 'forefoot'].includes(SEGMENT_BY_ID.get(contact.segment)?.role));
        const total = qualified.reduce((sum, contact) => sum + contact.forceN, 0);
        const side = qualified.filter(contact => SEGMENT_BY_ID.get(contact.segment)?.side === definition.side)
          .reduce((sum, contact) => sum + contact.forceN, 0);
        if (side > 0 && total > 0) {
          const error = sub(stanceIntent[foot], pose.position);
          const velocity = mode.startsWith('displacement-') ? measuredFootRates.get(foot) : pose.linearVelocity;
          const acceleration = clampLength({ x: 34 * error.x - 8.5 * velocity.x, y: 0,
            z: 34 * error.z - 8.5 * velocity.z }, 3.6);
          const force = scale(acceleration, TOTAL_MASS_KG * side / total);
          const jointPose = c.poses.get(definition.id);
          const anchor = worldPoint(jointPose.position, jointPose.rotation, definition.jointProfile.childFrame.anchor);
          terms.stanceTask = cross(sub(pose.position, anchor), force);
          command = { ...command, feedforwardWorld: add(command.feedforwardWorld ?? zero, terms.stanceTask) };
        }
      }
      if (active && original.id === 'leftThigh' && mode.startsWith('pulse-')) {
        const basis = jointFrameAxesWorld(c.poses.get(definition.parent).rotation, definition.jointProfile);
        terms.perturbation = scale(basis.x, mode === 'pulse-plus' ? amplitude : -amplitude);
        command = { ...command, feedforwardWorld: add(command.feedforwardWorld ?? zero, terms.perturbation) };
      }
      contributions.set(command.id, terms);
      return command;
    });
  };
  const originalApply = c.nativeMotors.apply.bind(c.nativeMotors);
  c.nativeMotors.apply = (bodies, joints, commands) => {
    if (commands !== c.standingCommands) {
      postureIntegral.clear();
      for (const [id, pose] of c.poses) previousBodyRotations.set(id, { ...pose.rotation });
      // Recovery owns its own command decomposition. Forward it unchanged;
      // stale standing contributions cannot describe a different controller.
      trace.actuationPhase = 'non-standing-controller';
      trace.interventionActive = false;
      trace.motors = [];
      trace.nonStandingCommands = copy(commands);
      const results = originalApply(bodies, joints, commands);
      trace.nonStandingResults = copy([...results]);
      return results;
    }
    trace.actuationPhase = 'standing';
    delete trace.nonStandingCommands;
    delete trace.nonStandingResults;
    trace.motors = commands.map(command => {
      const definition = SEGMENT_BY_ID.get(command.id), profile = definition.jointProfile;
      const parent = bodies.get(definition.parent), child = bodies.get(command.id);
      const coordinates = jointCoordinates(parent.rotation(), child.rotation(), profile);
      const target = clampJointCoordinates(jointCoordinates(identity, command.targetLocalRotation, profile), profile);
      const error = jointCoordinateTargetError(coordinates, target, profile);
      const basis = jointFrameAxesWorld(parent.rotation(), profile);
      const strength = clamp(command.strengthScale, 0, 1), kp = Math.max(0, command.stiffness) * strength, kd = Math.max(0, command.damping) * strength;
      const relativeVelocity = sub(child.angvel(), parent.angvel());
      const axes = profile.axes.map(axis => {
        const coordinate = axis.coordinate, cap = axis.maxMotorTorqueNm * strength;
        const terms = Object.fromEntries(Object.entries(contributions.get(command.id)).map(([name, value]) => [name, dot(value, basis[coordinate]) * strength]));
        const feedforward = dot(command.feedforwardWorld ?? zero, basis[coordinate]) * strength;
        const velocity = kd > 0 ? feedforward / kd : 0;
        const posture = kp * error[coordinate], damping = -kd * dot(relativeVelocity, basis[coordinate]);
        const requested = kp * error[coordinate] + kd * (velocity - dot(relativeVelocity, basis[coordinate]));
        const sum = posture + damping + Object.values(terms).reduce((a, b) => a + b, 0);
        assert.ok(Math.abs(sum - requested) < 1e-8, `Contribution reconstruction differs from native request: ${JSON.stringify({
          tick: c.fixedSteps, state: c.state, id: command.id, coordinate, sum, requested,
          commandIsStanding: commands === c.standingCommands, terms, feedforward, kp, kd })}`);
        return { coordinate, basisWorld: basis[coordinate], measuredCoordinate: coordinates[coordinate], targetCoordinate: target[coordinate], error: error[coordinate],
          relativeSpeed: dot(relativeVelocity, basis[coordinate]), posture, damping, ...terms, requested, cap,
          cappedRequest: clamp(requested, -cap, cap), curtailed: requested - clamp(requested, -cap, cap),
          atLimit: coordinates[coordinate] <= axis.minRadians + 0.001 || coordinates[coordinate] >= axis.maxRadians - 0.001 };
      });
      return { id: command.id, torqueSource: 'native-request-not-delivered-impulse', axes };
    });
    trace.contactPlan = copy(c.contactLoadPlan);
    const results = originalApply(bodies, joints, commands);
    for (const motor of trace.motors) {
      const reconstructed = motor.axes.reduce((sum, axis) => add(sum, scale(axis.basisWorld, axis.cappedRequest)), zero);
      assert.ok(length(sub(reconstructed, results.get(motor.id).torqueWorld)) < 1e-8, 'Capped native request differs from trace');
    }
    return results;
  };
}

let characterPrototype;
async function run(heading, mode, index) {
  const centered = mode.startsWith('initial-centered-')
    ? await createCenteredStandingCharacter(createEmbodiedCharacter, characterPrototype, heading, join(output, `${index}-centered-initialization.json`)) : null;
  const c = centered?.character ?? await createEmbodiedCharacter('canvas2d', { heading });
  characterPrototype ??= Object.getPrototypeOf(c);
  const integratedInertia = mode.startsWith('inertia-') ? installIntegratedInertia(c, join(output, `${index}-integrated-inertia.json`)) : null;
  let hybrid;
  if (mode === 'hybrid-translations' || mode === 'hybrid-hinges') {
    try { hybrid = installHybridStandingConstraints(c, output, index, mode === 'hybrid-hinges' ? 'hinges' : 'translations'); }
    catch (error) { c.dispose(); throw error; }
  }
  const trace = {};
  if (mode !== 'plain') instrument(c, mode.replace(/^inertia-/, ''), trace);
  const responseModel = mode.startsWith('model-observe-') ? installStandingResponseModel(c, output, index) : null;
  const forefootAuthority = mode.startsWith('authority-')
    ? installForefootAuthority(c, output, index, heading, mode.includes('-observe-') ? 'observe' : 'pulse', trace,
      mode.includes('-distal-') ? 'distal' : 'forefoot') : null;
  const distalPreview = mode.startsWith('preview-') ? installDistalPreview(c, output, index, trace,
    mode.startsWith('preview-horizontal-') || mode.startsWith('preview-leg-') || mode.startsWith('preview-pelvis-'),
    mode.startsWith('preview-leg-') || mode.startsWith('preview-pelvis-'), mode.startsWith('preview-pelvis-'),
    mode.startsWith('preview-pelvis-full-angular-'), mode.startsWith('preview-pelvis-full-angular-arms-'),
    mode.startsWith('preview-pelvis-full-angular-arms-position-'),
    mode.startsWith('preview-pelvis-full-angular-arms-position-coupled-')) : null;
  const fd = openSync(join(output, `${index}-${mode}.jsonl`), 'wx');
  const hashes = [], initialHash = sha256(JSON.stringify(physical(c))), response = [], peaks = {};
  const bodyRefs = new Map(c.ragdollBodies), colliderRefs = new Map(c.ragdollColliders);
  const verticalRanges = {};
  const projectionReasons = {}, traceDigest = createHash('sha256');
  let traceRows = 0;
  let firstSupportLossTick = null, firstNonUprightTick = null;
  let anchor, firstMotion = null, maxPelvisExcursionM = 0, maxFootExcursionM = 0, maxJointSeparationM = 0, maxSelfPenetrationM = 0, maxFloorPenetrationM = 0;
  let currentTick = 0, lastSnapshot;
  const digest = createHash('sha256');
  try {
    for (let tick = 1; tick <= frames; tick++) {
      currentTick = tick;
      const before = mode !== 'plain' ? copy(physical(c)) : null;
      c.fixedUpdate(dt, null);
      const measured = copy(physical(c));
      const hash = sha256(JSON.stringify(measured));
      hashes.push(hash); digest.update(hash);
      const snapshot = c.getSnapshot('canvas2d');
      lastSnapshot = snapshot;
      assert.equal(snapshot.diagnostics.finite, true);
      assert.deepEqual(snapshot.diagnostics.errors, []);
      assert.equal(snapshot.diagnostics.physicsOwnership, 'rapier-dynamic');
      for (const [id, body] of bodyRefs) assert.equal(c.ragdollBodies.get(id), body, 'Body identity changed');
      for (const [id, collider] of colliderRefs) assert.equal(c.ragdollColliders.get(id), collider, 'Collider identity changed');
      const speeds = snapshot.segments.map(segment => ({ id: segment.id, linear: length(segment.linearVelocity), angular: length(segment.angularVelocity) }));
      const maxLinear = speeds.reduce((a, b) => b.linear > a.linear ? b : a);
      const maxAngular = speeds.reduce((a, b) => b.angular > a.angular ? b : a);
      if (tick === 120) anchor = snapshot;
      if (tick > 120) {
        if (snapshot.support.planted.length !== 2) firstSupportLossTick ??= tick;
        if (!['upright', 'reacting'].includes(snapshot.state)) firstNonUprightTick ??= tick;
        for (const segment of snapshot.segments) {
          const range = verticalRanges[segment.id] ?? { min: segment.position.y, max: segment.position.y };
          verticalRanges[segment.id] = { min: Math.min(range.min, segment.position.y), max: Math.max(range.max, segment.position.y) };
        }
        for (const [name, value] of [['linear', maxLinear.linear], ['angular', maxAngular.angular]]) {
          if (!peaks[name] || value > peaks[name].value) peaks[name] = { value, tick, segment: name === 'linear' ? maxLinear.id : maxAngular.id };
        }
        if (!firstMotion && (maxLinear.linear > 0.1 || maxAngular.angular > 0.5)) firstMotion = { tick, maxLinear, maxAngular };
        if (anchor) {
          maxPelvisExcursionM = Math.max(maxPelvisExcursionM, horizontalDistance(snapshot.rootPosition, anchor.rootPosition));
          maxFootExcursionM = Math.max(maxFootExcursionM, ...['leftFoot', 'leftForefoot', 'rightFoot', 'rightForefoot'].map(id => horizontalDistance(snapshot.segments.find(s => s.id === id).position, anchor.segments.find(s => s.id === id).position)));
        }
      }
      maxJointSeparationM = Math.max(maxJointSeparationM, snapshot.diagnostics.maxJointSeparationM);
      maxSelfPenetrationM = Math.max(maxSelfPenetrationM, snapshot.diagnostics.maxSelfPenetrationM);
      maxFloorPenetrationM = Math.max(maxFloorPenetrationM, snapshot.diagnostics.maxFloorPenetrationM);
      const child = c.poses.get('leftThigh'), parent = c.poses.get('pelvis');
      const basis = jointFrameAxesWorld(parent.rotation, SEGMENT_BY_ID.get('leftThigh').jointProfile);
      const coordinates = jointCoordinates(parent.rotation, child.rotation, SEGMENT_BY_ID.get('leftThigh').jointProfile);
      const mass = measureMassState(c.poses.values());
      response.push({ tick, relativeSpeedX: dot(sub(child.angularVelocity, parent.angularVelocity), basis.x), coordinateX: coordinates.x,
        maxLinear: maxLinear.linear, maxAngular: maxAngular.angular, centerOfMass: mass.position, comVelocity: mass.velocity,
        footAngularSpeeds: Object.fromEntries(speeds.filter(s => /Foot|Forefoot/.test(s.id)).map(s => [s.id, s.angular])),
        loads: ['left', 'right'].map(side => c.lastContacts.filter(contact => SEGMENT_BY_ID.get(contact.segment)?.side === side).reduce((sum, contact) => sum + contact.forceN * contact.normalY, 0)),
        state: snapshot.state, planted: snapshot.support.planted, stepCount: c.stepCount,
        ...(traceKinematics ? { feet: Object.fromEntries(measured.poses.filter(pose => /Foot|Forefoot/.test(pose.id))
          .map(pose => [pose.id, { position: pose.position, centerOfMass: pose.centerOfMass,
            rotation: pose.rotation, linearVelocity: pose.linearVelocity, angularVelocity: pose.angularVelocity }])) } : {}),
        ...(trace.loadProjection ? { projection: trace.loadProjection.reason } : {}) });
      if (trace.loadProjection) projectionReasons[trace.loadProjection.reason] = (projectionReasons[trace.loadProjection.reason] ?? 0) + 1;
      if (mode !== 'plain' && tick >= traceStartTick && tick <= traceEndTick) {
        const row = JSON.stringify({ tick, commandTimeS: (tick - 1) * dt, responseTimeS: tick * dt,
          input: before, ...trace, output: measured, ...(traceContacts ? { groundManifolds: groundManifolds(c) } : {}), standingChain: snapshot.diagnostics.standingChain,
          stepLoads: stepLoadDiagnostics(snapshot, SEGMENT_BY_ID), diagnostics: { physicsOwnership: snapshot.diagnostics.physicsOwnership,
            maxJointSeparationM: snapshot.diagnostics.maxJointSeparationM, maxSelfPenetrationM: snapshot.diagnostics.maxSelfPenetrationM,
            maxFloorPenetrationM: snapshot.diagnostics.maxFloorPenetrationM, errors: snapshot.diagnostics.errors, finite: snapshot.diagnostics.finite,
            joints: snapshot.diagnostics.jointDiagnostics },
          maxLinear, maxAngular, bodyWeightN: TOTAL_MASS_KG * 9.81 }) + '\n';
        writeSync(fd, row); traceDigest.update(row); traceRows++;
      }
    }
    const end = c.getSnapshot('canvas2d');
    if (forefootAuthority) assert.equal(forefootAuthority.samples.length, 1, 'Authority sample was not completed');
    return { heading, mode, ...(hybrid ? { hybrid } : {}), ...(centered ? { centeredInitialization: centered.receipt } : {}),
      ...(integratedInertia ? { integratedInertia } : {}),
      ...(trace.postureIntegralSummary ? { postureIntegralSummary: trace.postureIntegralSummary } : {}),
      ...(trace.intervalDampingSummary ? { intervalDampingSummary: trace.intervalDampingSummary } : {}),
      ...(responseModel ? { responseModel } : {}),
      ...(forefootAuthority ? { forefootAuthority } : {}),
      ...(distalPreview ? { distalPreview } : {}),
      initialHash, trajectoryHash: digest.digest('hex'), hashes, response, peaks, firstMotion, projectionReasons,
      trace: { file: `${index}-${mode}.jsonl`, rows: traceRows, sha256: traceDigest.digest('hex'), ticks: [traceStartTick, traceEndTick] },
      maxPelvisExcursionM, maxFootExcursionM, maxJointSeparationM, maxSelfPenetrationM, maxFloorPenetrationM, finalState: end.state, planted: end.support.planted, steps: c.stepCount,
      firstSupportLossTick, firstNonUprightTick, verticalRanges,
      endpointPelvisDriftM: anchor ? horizontalDistance(end.rootPosition, anchor.rootPosition) : null,
      endpointFootDriftM: anchor ? Math.max(...['leftFoot', 'leftForefoot', 'rightFoot', 'rightForefoot'].map(id => horizontalDistance(end.segments.find(s => s.id === id).position, anchor.segments.find(s => s.id === id).position))) : null };
  } catch (error) {
    writeFileSync(join(output, `${index}-${mode}-failure.json`), JSON.stringify({
      heading, mode, currentTick, error: String(error), stack: error.stack,
      initialHash, partialTrajectoryHash: digest.copy().digest('hex'), hashes, response, peaks,
      snapshot: lastSnapshot, physical: physical(c), hybrid,
      nativeBodies: [...c.ragdollBodies].map(([id, body]) => {
        // Native getters can expose reusable buffers. Copy before the next
        // getter, and retain non-finite values explicitly in this failure log.
        const position = { ...body.translation() }, rotation = { ...body.rotation() };
        const linearVelocity = { ...body.linvel() }, angularVelocity = { ...body.angvel() };
        const centerOfMass = { ...body.worldCom() }, principalInertia = { ...body.principalInertia() };
        const matrix = body.effectiveWorldInvInertia();
        const effectiveWorldInvInertia = Object.fromEntries(['m11', 'm12', 'm13', 'm22', 'm23', 'm33'].map(key => [key, matrix[key]]));
        return { id, position, rotation, linearVelocity, angularVelocity, centerOfMass, mass: body.mass(), principalInertia, effectiveWorldInvInertia };
      }),
      qualification: 'Failed run stopped at the original guard. Partial histories and physical readback only; no acceptance.'
    }, (_key, value) => typeof value === 'number' && !Number.isFinite(value) ? String(value) : value, 2) + '\n', { flag: 'wx' });
    throw error;
  } finally { closeSync(fd); c.dispose(); }
}

const runs = [];
try {
  for (const heading of headings) for (const mode of modes) {
    const result = await run(heading, mode, runs.length);
    runs.push(result);
    console.log(JSON.stringify({ heading, mode, peaks: result.peaks, finalState: result.finalState }));
  }
  const comparisons = [];
  for (const heading of headings) {
    const group = runs.filter(run => run.heading === heading), control = group.find(run => run.mode === 'plain') ?? group.find(run => run.mode === 'observe');
    if (!control) continue;
    for (const variant of group.filter(run => run !== control)) {
      const firstDifference = variant.hashes.findIndex((hash, i) => hash !== control.hashes[i]);
      const beforeEqual = variant.initialHash === control.initialHash && variant.hashes.slice(0, startTick - 1).every((hash, i) => hash === control.hashes[i]);
      const observer = variant.mode.startsWith('observe');
      assert.ok(beforeEqual, `Unequal pre-intervention state: ${heading} ${variant.mode}`);
      if (observer) assert.equal(variant.trajectoryHash, control.trajectoryHash, 'Instrumentation/replay changed physical behavior');
      const window = variant.response.slice(startTick - 1, startTick + 29), baseline = control.response.slice(startTick - 1, startTick + 29);
      comparisons.push({ heading, mode: variant.mode, beforeEqual, firstDifferenceTick: firstDifference < 0 ? null : firstDifference + 1,
        firstTickRelativeSpeedDelta: variant.response[startTick - 1].relativeSpeedX - control.response[startTick - 1].relativeSpeedX,
        maxLinearRatio: Math.max(...window.map(frame => frame.maxLinear)) / Math.max(...baseline.map(frame => frame.maxLinear)),
        maxAngularRatio: Math.max(...window.map(frame => frame.maxAngular)) / Math.max(...baseline.map(frame => frame.maxAngular)) });
    }
  }
  const sourceAfter = fingerprints();
  assert.deepEqual(sourceAfter, sourceBefore, 'Source changed during diagnostic');
  const report = { started, finished: new Date().toISOString(), command: [process.execPath, ...process.execArgv, ...process.argv.slice(1)],
    nodeVersion: process.version, runtimeOverride: globalThis[Symbol.for('laboratoire.rapierRuntimeOverride')] ?? null,
    parameters: { frames, startTick, durationTicks, amplitude, headings, modes, traceStartTick, traceEndTick, traceContacts, traceKinematics }, sourceBefore, sourceAfter,
    sourceFingerprint: sha256(JSON.stringify(sourceBefore)), comparisons, runs: runs.map(run => Object.fromEntries(Object.entries(run).filter(([key]) => key !== 'hashes'))),
    acceptance: 'diagnostic-only; no behavioral checkpoint accepted' };
  writeFileSync(join(output, 'report.json'), JSON.stringify(report, null, 2) + '\n');
  console.log(JSON.stringify({ comparisons, report: join(output, 'report.json') }));
} catch (error) {
  writeFileSync(join(output, 'failure.json'), JSON.stringify({ started, finished: new Date().toISOString(),
    command: [process.execPath, ...process.execArgv, ...process.argv.slice(1)], nodeVersion: process.version,
    parameters: { frames, startTick, durationTicks, amplitude, headings, modes, traceStartTick, traceEndTick, traceContacts, traceKinematics },
    sourceBefore, sourceAfter: fingerprints(), error: String(error), stack: error.stack,
    completedRuns: runs.map(run => Object.fromEntries(Object.entries(run).filter(([key]) => key !== 'hashes'))),
    acceptance: 'diagnostic failed; no behavioral checkpoint accepted' }, null, 2) + '\n', { flag: 'wx' });
  throw error;
} finally { unregister(); }
