import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { writeFileSync } from 'node:fs';
import { join } from 'node:path';
import RAPIER from '@dimforge/rapier3d-compat';
import { SEGMENT_BY_ID } from '../src/core/humanoid.ts';
import { clampJointCoordinates, jointCoordinates, jointFrameAxesWorld } from '../src/character/joint-coordinates.ts';
import { clamp, dot } from '../src/character/math.ts';
import { sha256 } from './capture-physics-baseline.mjs';
import { solveDampedAngularStep } from './standing-forefoot-authority.mjs';
import { pelvisPoseRates, pelvisBoundedResidual } from './standing-pelvis-task.mjs';
import { coupledSpeedMetrics, searchCoupledSpeedPreview } from './standing-coupled-speed.mjs';

const zero = { x: 0, y: 0, z: 0 }, identity = { ...zero, w: 1 };
const feet = ['leftFoot', 'leftForefoot', 'rightFoot', 'rightForefoot'];
const distalControls = ['leftAnkle', 'leftFoot', 'leftForefoot', 'rightAnkle', 'rightFoot', 'rightForefoot'];
const armControls = [...SEGMENT_BY_ID.values()].filter(d => ['shoulder-girdle', 'upper-arm', 'forearm', 'forearm-twist', 'hand'].includes(d.role)).map(d => d.id);

/** Diagnostic online preview. Only native motor biases cross into the live world. */
export function installDistalPreview(character, output, index, trace, correctHorizontal = false, fullLegs = false, quietPelvis = false, fullAngular = false, quietArms = false, positionFeedback = false, coupledSpeeds = false) {
  if (quietPelvis) assert.ok(correctHorizontal && fullLegs);
  if (fullAngular) assert.ok(quietPelvis);
  if (quietArms) assert.ok(fullAngular);
  if (positionFeedback) assert.ok(quietArms);
  if (coupledSpeeds) assert.ok(positionFeedback);
  const world = character.world, originalStep = world.step.bind(world);
  const controls = fullLegs ? [...distalControls, 'leftShin', 'rightShin', 'leftThigh', 'rightThigh', ...(quietArms ? armControls : [])] : distalControls;
  const handles = [...character.ragdollBodies].map(([id, body]) => [id, body.handle]);
  const footReference = positionFeedback ? Object.fromEntries(feet.map(id => [id, { ...character.ragdollBodies.get(id).translation() }])) : null;
  const report = { correctHorizontal, fullLegs, horizontalDeadbandMS: 0.0002, triggerRadS: 0.4, reserveRadS: 0.3, predictedSteps: 0, skippedSteps: 0,
    horizontalTriggers: 0, horizontalCorrections: 0, horizontalMisses: 0,
    correctedSteps: 0, reserveMisses: 0, infeasibleSteps: 0, totalCopies: 0, totalMs: 0, maxMs: 0, samples: [],
    qualification: 'One-step online diagnostic; original native ceilings and physical ownership. No state transfer. Base motor request telemetry excludes the separately recorded bias.' };
  const caseDigest = fullLegs ? createHash('sha256') : null;
  if (quietPelvis) Object.assign(report, { quietPelvis, pelvisLinearBoundMS: 0.01, pelvisAngularBoundRadS: 0.05, pelvisMisses: 0 });
  if (fullAngular) report.fullAngular = true;
  if (quietArms) Object.assign(report, { quietArms, armLinearReserveMS: 0.08, armAngularReserveRadS: 0.4,
    armTriggers: 0, armCorrections: 0, armMisses: 0 });
  if (positionFeedback) Object.assign(report, { positionFeedback, footReturnTimeS: 1, footPositionReference: footReference, trackingMisses: 0,
    horizontalMissDefinition: 'Actual pose rate above 0.2 mm/s; tracking error relative to the return target is reported separately.' });
  if (coupledSpeeds) Object.assign(report, { coupledSpeeds, coupledTriggers: 0, coupledCorrections: 0,
    coupledMisses: 0, coupledReserveMisses: 0, coupledLinearReserveMS: 0.08, coupledAngularReserveRadS: 0.4 });
  if (fullLegs) {
    report.caseLog = { file: `${index}-leg-preview-cases.jsonl`, rows: 0, sha256: null };
    writeFileSync(join(output, report.caseLog.file), '', { flag: 'wx' });
  }
  const capture = current => Object.fromEntries(handles.map(([id, handle]) => {
    const body = current.getRigidBody(handle);
    return [id, { position: { ...body.translation() }, rotation: { ...body.rotation() },
      velocity: { ...body.linvel() }, angularVelocity: { ...body.angvel() },
      centerOfMass: { ...body.worldCom() }, mass: body.mass() }];
  }));
  const metrics = bodies => {
    const finite = Object.values(bodies).every(body => Object.values(body).flatMap(v =>
      typeof v === 'number' ? [v] : Object.values(v)).every(Number.isFinite));
    const residual = feet.flatMap(id => Object.values(bodies[id].angularVelocity));
    const maxFootAngular = Math.max(...feet.map(id => Math.hypot(...Object.values(bodies[id].angularVelocity))));
    const maxLinear = Math.max(...Object.values(bodies).map(b => Math.hypot(...Object.values(b.velocity))));
    const maxOtherAngular = Math.max(...Object.entries(bodies).filter(([id]) => !feet.includes(id))
      .map(([, b]) => Math.hypot(...Object.values(b.angularVelocity))));
    const armResidual = quietArms ? armControls.flatMap(id => [
      ...Object.values(bodies[id].velocity).map(v => v / 0.1), ...Object.values(bodies[id].angularVelocity).map(v => v / 0.5)]) : null;
    return { finite, residual, score: residual.reduce((sum, v) => sum + v * v, 0),
      maxFootAngular, maxLinear, maxOtherAngular,
      ...(coupledSpeeds ? coupledSpeedMetrics(bodies) : {}),
      ...(quietArms ? { armResidual, armScore: armResidual.reduce((s, v) => s + v * v, 0),
        maxArmLinear: Math.max(...armControls.map(id => Math.hypot(...Object.values(bodies[id].velocity)))),
        maxArmAngular: Math.max(...armControls.map(id => Math.hypot(...Object.values(bodies[id].angularVelocity)))) } : {}),
      withinOtherSpeedBounds: finite && maxLinear <= 0.1 && maxOtherAngular <= 0.5 };
  };
  world.step = (...args) => {
    delete trace.distalPreviewOverride;
    if (character.activeGrab || character.step || character.stepCount !== 0
      || !['upright', 'reacting'].includes(character.state)) {
      report.skippedSteps++;
      return originalStep(...args);
    }
    const started = performance.now(), tick = character.fixedSteps;
    const snapshot = world.takeSnapshot(), snapshotSha256 = sha256(snapshot), initial = capture(world);
    if (positionFeedback && report.predictedSteps === 0) {
      assert.deepEqual(footReference, Object.fromEntries(feet.map(id => [id, initial[id].position])),
        'Initialization foot reference differs from the first native pre-step state');
    }
    const axes = controls.flatMap(id => {
      const command = character.standingCommands.find(c => c.id === id), definition = SEGMENT_BY_ID.get(id);
      if (!fullLegs) assert.equal(definition.jointProfile.axes.length, 1);
      const strength = clamp(command.strengthScale, 0, 1), damping = command.damping * strength;
      assert.ok(damping > 0);
      const basis = jointFrameAxesWorld(character.ragdollBodies.get(definition.parent).rotation(), definition.jointProfile);
      const target = clampJointCoordinates(jointCoordinates(identity, command.targetLocalRotation, definition.jointProfile), definition.jointProfile);
      return definition.jointProfile.axes.map(axis => {
        const coordinate = axis.coordinate;
        if (!fullLegs) assert.equal(coordinate, definition.role === 'hindfoot' ? 'z' : 'x');
        return { id, handle: character.jointsByChild.get(id).handle, abiAxis: { x: 3, y: 4, z: 5 }[coordinate],
          stiffness: command.stiffness * strength, damping, targetPosition: target[coordinate],
          targetVelocity: dot(command.feedforwardWorld ?? zero, basis[coordinate]) * strength / damping,
          unchangedTorqueCeilingNm: axis.maxMotorTorqueNm * strength };
      });
    });
    const configure = (current, delta) => axes.forEach((axis, i) => {
      if (delta[i] !== 0) current.impulseJoints.raw.jointConfigureMotor(axis.handle, axis.abiAxis,
        axis.targetPosition, axis.targetVelocity + delta[i] / axis.damping, axis.stiffness, axis.damping);
    });
    const legAxisCount = quietArms ? 14 : axes.length;
    if (quietArms) assert.equal(axes.length, 34);
    const cases = [];
    const horizontalMetrics = bodies => {
      const horizontalResidual = feet.flatMap(id => ['x', 'z'].map(axis =>
        (bodies[id].position[axis] - initial[id].position[axis]) / world.timestep));
      const maxHorizontalRate = Math.max(...feet.map((_, i) => Math.hypot(...horizontalResidual.slice(i * 2, i * 2 + 2))));
      const trackingResidual = positionFeedback ? feet.flatMap((id, i) => ['x', 'z'].map((axis, j) => horizontalResidual[i * 2 + j]
        + (initial[id].position[axis] - footReference[id][axis]) / report.footReturnTimeS)) : horizontalResidual;
      const pelvis = quietPelvis ? pelvisPoseRates(initial.pelvis, bodies.pelvis, world.timestep) : null;
      const taskResidual = pelvis ? [...trackingResidual, ...pelvisBoundedResidual(pelvis)] : null;
      return { horizontalResidual, horizontalScore: horizontalResidual.reduce((sum, v) => sum + v * v, 0), maxHorizontalRate,
        ...(positionFeedback ? { trackingResidual,
          maxTrackingRate: Math.max(...feet.map((_, i) => Math.hypot(...trackingResidual.slice(i * 2, i * 2 + 2)))),
          maxFootPositionErrorM: Math.max(...feet.map(id => Math.hypot(bodies[id].position.x - footReference[id].x, bodies[id].position.z - footReference[id].z))) } : {}),
        ...(pelvis ? { ...pelvis, taskResidual, taskScore: taskResidual.reduce((s, v, i) => s + (v / (i < 11 ? 0.1 : 0.5)) ** 2, 0) } : {}) };
    };
    const simulate = (name, delta) => {
      const copy = RAPIER.World.restoreSnapshot(snapshot), queue = new RAPIER.EventQueue(true);
      try {
        assert.deepEqual(capture(copy), initial, 'Restored preview changed physical initial fields');
        configure(copy, delta); copy.step(queue, character.physicsHooks); queue.clear();
        const bodies = capture(copy), measured = metrics(bodies);
        const result = { name, deltaTorqueNm: [...delta], ...measured,
          ...(fullLegs ? { predictedBodySha256: sha256(JSON.stringify(bodies)) } : {}),
          ...(correctHorizontal ? horizontalMetrics(bodies) : {}) };
        cases.push(result); return { ...result, bodies };
      } finally { queue.free(); copy.free(); }
    };
    const baseline = simulate('baseline', axes.map(() => 0));
    assert.equal(baseline.finite, true);
    let best = baseline;
    const triggered = baseline.maxFootAngular > report.triggerRadS;
    if (triggered) {
      for (let iteration = 0; iteration < 16
        && !(best.withinOtherSpeedBounds && best.maxFootAngular <= report.reserveRadS); iteration++) {
        const columns = [];
        let bestProbe;
        const improvement = fullAngular ? 1e-14 : 1e-12;
        for (let axis = 0; axis < (fullLegs ? distalControls.length : axes.length); axis++) {
          const plus = [...best.deltaTorqueNm], minus = [...best.deltaTorqueNm], cap = axes[axis].unchangedTorqueCeilingNm;
          plus[axis] = clamp(plus[axis] + 1, -cap, cap); minus[axis] = clamp(minus[axis] - 1, -cap, cap);
          const positive = simulate(`iteration-${iteration}-axis-${axis}-plus`, plus);
          const negative = simulate(`iteration-${iteration}-axis-${axis}-minus`, minus);
          assert.ok(positive.finite && negative.finite, 'Non-finite preview probe');
          if (fullAngular) for (const probe of [positive, negative]) {
            if (probe.withinOtherSpeedBounds && probe.score < best.score - improvement
              && (!bestProbe || probe.score < bestProbe.score)) bestProbe = probe;
          }
          columns.push(positive.residual.map((v, i) => (v - negative.residual[i]) / (plus[axis] - minus[axis])));
        }
        const step = fullAngular
          ? solveDampedAngularStep(columns.map((column, i) => column.map(v => v * axes[i].unchangedTorqueCeilingNm / 0.5)),
            best.residual.map(v => v / 0.5)).map((v, i) => v * axes[i].unchangedTorqueCeilingNm)
          : solveDampedAngularStep(columns, best.residual);
        const trust = fullAngular ? Math.max(1, ...step.map((v, i) => Math.abs(v) / axes[i].unchangedTorqueCeilingNm))
          : Math.max(1, ...step.map(Math.abs));
        let next;
        for (const fraction of fullAngular ? [1, 0.5, 0.25, 0.125, 0.0625, 0.03125] : [1, 0.5, 0.25, 0.125]) {
          const paddedStep = fullLegs ? [...step, ...Array(axes.length - step.length).fill(0)] : step;
          const delta = paddedStep.map((v, i) => clamp(best.deltaTorqueNm[i] + fraction * v / trust,
            -axes[i].unchangedTorqueCeilingNm, axes[i].unchangedTorqueCeilingNm));
          const candidate = simulate(`iteration-${iteration}-line-${fraction}`, delta);
          if (candidate.withinOtherSpeedBounds && candidate.score < best.score - improvement) { next = candidate; break; }
        }
        if (bestProbe && (!next || bestProbe.score < next.score)) next = bestProbe;
        if (!next) break;
        best = next;
      }
    }
    const armStart = best;
    const needsArm = c => quietArms && (c.maxArmLinear > report.armLinearReserveMS || c.maxArmAngular > report.armAngularReserveRadS);
    const armTriggered = needsArm(best);
    if (armTriggered) {
      report.armTriggers++;
      for (let iteration = 0; iteration < 8 && needsArm(best); iteration++) {
        const columns = []; let bestProbe;
        for (let axis = legAxisCount; axis < axes.length; axis++) {
          const plus = [...best.deltaTorqueNm], minus = [...best.deltaTorqueNm], cap = axes[axis].unchangedTorqueCeilingNm;
          plus[axis] = clamp(plus[axis] + 1, -cap, cap); minus[axis] = clamp(minus[axis] - 1, -cap, cap);
          const positive = simulate(`arm-${iteration}-axis-${axis}-plus`, plus);
          const negative = simulate(`arm-${iteration}-axis-${axis}-minus`, minus);
          assert.ok(positive.finite && negative.finite, 'Non-finite arm preview probe');
          for (const probe of [positive, negative]) {
            if (probe.withinOtherSpeedBounds && probe.maxFootAngular <= report.triggerRadS
              && probe.armScore < best.armScore - 1e-14 && (!bestProbe || probe.armScore < bestProbe.armScore)) bestProbe = probe;
          }
          columns.push(positive.armResidual.map((v, i) => (v - negative.armResidual[i]) / (plus[axis] - minus[axis])));
        }
        const step = solveDampedAngularStep(columns.map((column, i) => column.map(v => v * axes[legAxisCount + i].unchangedTorqueCeilingNm)),
          best.armResidual).map((v, i) => v * axes[legAxisCount + i].unchangedTorqueCeilingNm);
        const trust = Math.max(1, ...step.map((v, i) => Math.abs(v) / axes[legAxisCount + i].unchangedTorqueCeilingNm));
        let next;
        for (const fraction of [1, 0.5, 0.25, 0.125, 0.0625, 0.03125]) {
          const delta = axes.map((axis, i) => clamp(best.deltaTorqueNm[i] + fraction * (step[i - legAxisCount] ?? 0) / trust,
            -axis.unchangedTorqueCeilingNm, axis.unchangedTorqueCeilingNm));
          const candidate = simulate(`arm-${iteration}-line-${fraction}`, delta);
          if (candidate.withinOtherSpeedBounds && candidate.maxFootAngular <= report.triggerRadS
            && candidate.armScore < best.armScore - 1e-14) { next = candidate; break; }
        }
        if (bestProbe && (!next || bestProbe.armScore < next.armScore)) next = bestProbe;
        if (!next) break; best = next;
      }
      if (best !== armStart) report.armCorrections++;
    }
    const beforeHorizontal = best;
    const loadedSides = new Set(character.lastContacts.filter(c => c.loadBearing && c.normalY > 0.75)
      .map(c => SEGMENT_BY_ID.get(c.segment)?.side));
    const pelvisMiss = c => quietPelvis && (c.maxPelvisLinearRate > report.pelvisLinearBoundMS || c.maxPelvisAngularRate > report.pelvisAngularBoundRadS);
    const needsTask = c => (positionFeedback ? c.maxTrackingRate : c.maxHorizontalRate) > report.horizontalDeadbandMS || pelvisMiss(c);
    const taskResidual = c => quietPelvis ? c.taskResidual : c.horizontalResidual;
    const taskScore = c => quietPelvis ? c.taskScore : c.horizontalScore;
    const taskScale = i => quietPelvis && i >= 11 ? 0.5 : 0.1;
    const horizontalTriggered = correctHorizontal && loadedSides.has('left') && loadedSides.has('right') && needsTask(best);
    if (horizontalTriggered) {
      report.horizontalTriggers++;
      for (let iteration = 0; iteration < (fullLegs ? 8 : 4) && needsTask(best); iteration++) {
        const columns = [];
        let bestProbe;
        for (let axis = 0; axis < legAxisCount; axis++) {
          const plus = [...best.deltaTorqueNm], minus = [...best.deltaTorqueNm], cap = axes[axis].unchangedTorqueCeilingNm;
          plus[axis] = clamp(plus[axis] + 1, -cap, cap); minus[axis] = clamp(minus[axis] - 1, -cap, cap);
          const positive = simulate(`horizontal-${iteration}-axis-${axis}-plus`, plus);
          const negative = simulate(`horizontal-${iteration}-axis-${axis}-minus`, minus);
          assert.ok(positive.finite && negative.finite, 'Non-finite horizontal preview probe');
          if (fullLegs) for (const probe of [positive, negative]) {
            if (probe.withinOtherSpeedBounds && probe.maxFootAngular <= report.triggerRadS
              && taskScore(probe) < taskScore(best) - 1e-14
              && (!bestProbe || taskScore(probe) < taskScore(bestProbe))) bestProbe = probe;
          }
          columns.push(taskResidual(positive).map((v, i) => (v - taskResidual(negative)[i]) / (plus[axis] - minus[axis])));
        }
        const step = fullLegs
          ? solveDampedAngularStep(columns.map((column, i) => column.map((v, j) => v * axes[i].unchangedTorqueCeilingNm / taskScale(j))),
            taskResidual(best).map((v, j) => v / taskScale(j))).map((v, i) => v * axes[i].unchangedTorqueCeilingNm)
          : solveDampedAngularStep(columns, best.horizontalResidual);
        const trust = fullLegs ? Math.max(1, ...step.map((v, i) => Math.abs(v) / axes[i].unchangedTorqueCeilingNm))
          : Math.max(1, ...step.map(Math.abs));
        let next;
        for (const fraction of fullLegs ? [1, 0.5, 0.25, 0.125, 0.0625, 0.03125] : [1, 0.5, 0.25, 0.125]) {
          const delta = axes.map((axis, i) => clamp(best.deltaTorqueNm[i] + fraction * (step[i] ?? 0) / trust,
            -axis.unchangedTorqueCeilingNm, axis.unchangedTorqueCeilingNm));
          const candidate = simulate(`horizontal-${iteration}-line-${fraction}`, delta);
          if (candidate.withinOtherSpeedBounds && candidate.maxFootAngular <= report.triggerRadS
            && taskScore(candidate) < taskScore(best) - 1e-14) { next = candidate; break; }
        }
        if (bestProbe && (!next || taskScore(bestProbe) < taskScore(next))) next = bestProbe;
        if (!next) break;
        best = next;
      }
      if (best !== beforeHorizontal) report.horizontalCorrections++;
      if (best.maxHorizontalRate > report.horizontalDeadbandMS) report.horizontalMisses++;
    }
    const afterHorizontal = best;
    const coupledTriggered = coupledSpeeds && !(best.withinOtherSpeedBounds && best.maxFootAngular <= 0.5);
    let coupledResult;
    if (coupledTriggered) {
      report.coupledTriggers++;
      coupledResult = searchCoupledSpeedPreview(axes, best, simulate);
      best = coupledResult.best;
      if (best !== afterHorizontal) report.coupledCorrections++;
      if (!coupledResult.guardFound) report.coupledMisses++;
      if (!coupledResult.reserveReached) report.coupledReserveMisses++;
    }
    if (pelvisMiss(best)) report.pelvisMisses++;
    if (needsArm(best)) report.armMisses++;
    if (positionFeedback && best.maxTrackingRate > report.horizontalDeadbandMS) report.trackingMisses++;
    const corrected = best !== baseline;
    if (corrected) {
      configure(world, best.deltaTorqueNm);
      trace.distalPreviewOverride = { tick, deltaTorqueNm: best.deltaTorqueNm, axes, qualification: report.qualification };
      report.correctedSteps++;
    }
    const result = originalStep(...args), actual = capture(world);
    assert.deepEqual(actual, best.bodies, 'Live world differs from the exact 25-body preview');
    const withinReserve = best.withinOtherSpeedBounds && best.maxFootAngular <= report.reserveRadS;
    const withinBounds = best.withinOtherSpeedBounds && best.maxFootAngular <= 0.5;
    if (triggered && !withinReserve) report.reserveMisses++;
    if (!withinBounds) report.infeasibleSteps++;
    report.predictedSteps++; report.totalCopies += cases.length;
    const elapsedMs = performance.now() - started;
    report.totalMs += elapsedMs; report.maxMs = Math.max(report.maxMs, elapsedMs);
    const sample = { tick, snapshotSha256, triggered, corrected, withinReserve, withinBounds,
      baselineMaxFootAngular: baseline.maxFootAngular, bestMaxFootAngular: best.maxFootAngular,
      ...(correctHorizontal ? { horizontalTriggered, horizontalCorrected: afterHorizontal !== beforeHorizontal,
        baselineMaxHorizontalRate: baseline.maxHorizontalRate, bestMaxHorizontalRate: best.maxHorizontalRate } : {}),
      maxLinear: best.maxLinear, maxOtherAngular: best.maxOtherAngular, cases: cases.length,
      ...(quietPelvis ? { maxPelvisLinearRate: best.maxPelvisLinearRate, maxPelvisAngularRate: best.maxPelvisAngularRate } : {}),
      ...(quietArms ? { armTriggered, armCorrected: beforeHorizontal !== armStart,
        armStageMaxLinear: beforeHorizontal.maxArmLinear, armStageMaxAngular: beforeHorizontal.maxArmAngular,
        maxArmLinear: best.maxArmLinear, maxArmAngular: best.maxArmAngular } : {}),
      ...(positionFeedback ? { baselineMaxTrackingRate: baseline.maxTrackingRate, bestMaxTrackingRate: best.maxTrackingRate,
        maxFootPositionErrorM: best.maxFootPositionErrorM } : {}),
      ...(coupledSpeeds ? { coupledTriggered, coupledCorrected: best !== afterHorizontal,
        coupledSearchCases: coupledResult?.cases ?? 0, coupledGuardFound: coupledResult?.guardFound ?? null,
        coupledSearchBestScore: coupledResult?.searchBest.coupledScore ?? null,
        coupledReserveReached: coupledResult?.reserveReached ?? null } : {}),
      deltaTorqueNm: best.deltaTorqueNm, exactPredictedBodies: handles.length,
      actualSha256: sha256(JSON.stringify(actual)), elapsedMs };
    if (fullLegs) {
      if (tick === 1 || tick % 120 === 0 || !withinBounds) {
        sample.snapshot = `${index}-leg-preview-${tick}.bin`;
        writeFileSync(join(output, sample.snapshot), snapshot, { flag: 'wx' });
      }
      sample.caseLogLine = report.caseLog.rows + 1;
      const withoutBodies = value => Object.fromEntries(Object.entries(value).filter(([key]) => key !== 'bodies'));
      const row = JSON.stringify({ ...sample, initial, axes, baseline: withoutBodies(baseline),
        best: withoutBodies(best), actual, cases, qualification: report.qualification }) + '\n';
      writeFileSync(join(output, report.caseLog.file), row, { flag: 'a' });
      caseDigest.update(row); report.caseLog.rows++; report.caseLog.sha256 = caseDigest.copy().digest('hex');
    } else if (triggered || horizontalTriggered) {
      const stem = `${index}-distal-preview-${tick}`;
      writeFileSync(join(output, `${stem}.bin`), snapshot, { flag: 'wx' });
      const data = { ...sample, initial, axes, baseline, best, actual, cases, qualification: report.qualification };
      const bytes = JSON.stringify(data, null, 2) + '\n';
      writeFileSync(join(output, `${stem}.json`), bytes, { flag: 'wx' });
      sample.file = `${stem}.json`; sample.sha256 = sha256(bytes); sample.snapshot = `${stem}.bin`;
    }
    report.samples.push(sample);
    if (tick % 120 === 0) console.log(JSON.stringify({ preview: index, tick, corrected: report.correctedSteps,
      infeasible: report.infeasibleSteps, copies: report.totalCopies }));
    return result;
  };
  return report;
}
