// Experimental only. Project measured patch shares onto the current feasible
// wrench using every real manifold point, without choosing loads by hull vertex
// membership. Never expands support or changes force/actuator budgets.
const dot = (a, b) => a.reduce((sum, x, i) => sum + x * b[i], 0);

export function preferHindfootStandingLoads(plan, contacts, distribute) {
  if (!plan.pressureFeasible || !contacts.length) return { plan, reason: 'infeasible-or-empty' };
  const preferredSegments = new Set(contacts.filter(contact => ['leftFoot', 'rightFoot'].includes(contact.segment)).map(contact => contact.segment));
  const points = distribute(contacts, plan.pressurePoint, { rotationInvariant: true, preferredSegments });
  const loads = contacts.map(contact => {
    const selected = points.filter(load => load.segment === contact.segment), share = selected.reduce((sum, load) => sum + load.share, 0);
    const point = Object.fromEntries(['x', 'y', 'z'].map(axis => [axis, share > 1e-12
      ? selected.reduce((sum, load) => sum + load.point[axis] * load.share, 0) / share : contact.point[axis]]));
    return { segment: contact.segment, point,
      plannedForce: Object.fromEntries(['x', 'y', 'z'].map(axis => [axis, plan.allocatedForce[axis] * share])),
      measuredForceN: contact.forceN, measuredPoint: { ...(contact.measuredPressurePoint ?? contact.point) } };
  });
  return { plan: { ...plan, loads }, reason: points.some(load => !preferredSegments.has(load.segment) && load.share > 1e-12)
    ? 'hindfoot-pressure-infeasible-all-current-support' : 'current-hindfoot-pressure-feasible' };
}

export function projectHindfootStandingLoads(plan, contacts, distribute) {
  const preferred = preferHindfootStandingLoads(plan, contacts, distribute);
  if (preferred.reason !== 'current-hindfoot-pressure-feasible') return preferred;
  const hindfeet = contacts.filter(contact => ['leftFoot', 'rightFoot'].includes(contact.segment));
  const projection = projectStandingLoads(preferred.plan, hindfeet, distribute, 'equal-patches');
  if (projection.reason !== 'projected-current-measured-points') return { plan, reason: projection.reason };
  return { plan: { ...projection.plan, loads: contacts.map(contact => projection.plan.loads.find(load => load.segment === contact.segment)
    ?? preferred.plan.loads.find(load => load.segment === contact.segment)) },
  reason: 'projected-current-hindfoot-points', iterations: projection.iterations };
}

export function projectHindfootHullStandingLoads(plan, contacts, distribute, previous, hull) {
  void previous;
  return projectHindfootStandingLoads(plan, measuredPatchContours(contacts, hull), distribute);
}

export function projectMeasuredContourStandingLoads(plan, contacts, distribute, previous, hull) {
  void previous;
  return projectStandingLoads(plan, measuredPatchContours(contacts, hull), distribute, 'measured');
}

export function projectEqualContourStandingLoads(plan, contacts, distribute, previous, hull) {
  void previous;
  return projectStandingLoads(plan, measuredPatchContours(contacts, hull), distribute, 'equal-patches');
}

export function projectMeasuredPressureStandingLoads(plan, contacts, distribute, previous, hull) {
  void previous;
  return projectStandingLoads(plan, measuredPatchContours(contacts, hull), distribute, 'measured-pressure');
}

function measuredPatchContours(contacts, hull) {
  return contacts.map(contact => {
    const points = contact.points?.length ? contact.points : [contact.point];
    return { ...contact, points: hull(points).map(vertex => {
      const point = points.find(point => Math.abs(point.x - vertex.x) < 1e-10 && Math.abs(point.z - vertex.z) < 1e-10);
      if (!point) throw new Error('Patch contour lost measured ownership');
      return point;
    }) };
  });
}

function solve(matrix, rhs) {
  const a = matrix.map((row, i) => [...row, rhs[i]]);
  for (let col = 0; col < 3; col++) {
    let pivot = col;
    for (let row = col + 1; row < 3; row++) if (Math.abs(a[row][col]) > Math.abs(a[pivot][col])) pivot = row;
    if (Math.abs(a[pivot][col]) < 1e-12) return null;
    [a[col], a[pivot]] = [a[pivot], a[col]];
    const divisor = a[col][col];
    for (let j = col; j <= 3; j++) a[col][j] /= divisor;
    for (let row = 0; row < 3; row++) if (row !== col) {
      const factor = a[row][col];
      for (let j = col; j <= 3; j++) a[row][j] -= factor * a[col][j];
    }
  }
  return a.map(row => row[3]);
}

export function projectStandingLoads(plan, contacts, distribute, reference = 'measured') {
  if (!plan.pressureFeasible || !plan.loads.length) return { plan, reason: 'infeasible-or-empty' };
  const previous = typeof reference === 'object' ? reference : null;
  if (previous && (!previous.pressureFeasible || previous.loads.length !== contacts.length
    || contacts.some(contact => !previous.loads.some(load => load.segment === contact.segment)))) {
    return { plan, reason: 'prior-reference-unavailable-or-support-changed' };
  }
  const previousTotal = previous?.loads.reduce((sum, load) => sum + load.plannedForce.y, 0);
  if (previous && !(previousTotal > 0)) return { plan, reason: 'no-prior-load-reference' };
  const total = reference === 'equal-patches' ? contacts.length : contacts.reduce((sum, contact) => sum + contact.forceN, 0);
  const candidates = contacts.flatMap((contact, owner) => {
    const points = contact.points?.length ? contact.points : [contact.point];
    if (reference === 'measured-pressure') {
      const projected = distribute([contact], contact.measuredPressurePoint ?? contact.point, { rotationInvariant: true });
      return points.map(point => ({ point, owner,
        seed: contact.forceN / total * projected.filter(load =>
          Math.hypot(point.x - load.point.x, point.y - load.point.y, point.z - load.point.z) < 1e-9)
          .reduce((sum, load) => sum + load.share, 0),
        metric: 1 / contacts.length / points.length }));
    }
    if (previous) {
      const prior = previous.loads.find(load => load.segment === contact.segment);
      const projected = distribute([contact], prior.point, { rotationInvariant: true });
      return points.map(point => ({ point, owner,
        seed: prior.plannedForce.y / previousTotal * projected.filter(load =>
          Math.hypot(point.x - load.point.x, point.y - load.point.y, point.z - load.point.z) < 1e-9)
          .reduce((sum, load) => sum + load.share, 0),
        metric: 1 / contacts.length / points.length }));
    }
    return points.map(point => ({ point, owner, seed: (reference === 'equal-patches' ? 1 : contact.forceN) / total / points.length }));
  });
  const width = Math.max(0.01, ...candidates.map(({ point }) => Math.hypot(point.x - plan.pressurePoint.x, point.z - plan.pressurePoint.z)));
  const columns = candidates.map(({ point }) => [1, (point.x - plan.pressurePoint.x) / width, (point.z - plan.pressurePoint.z) / width]);
  const seed = candidates.map(candidate => candidate.seed), free = seed.map(() => true);
  const metric = candidates.map(candidate => candidate.metric ?? candidate.seed);
  const initial = distribute(contacts, plan.pressurePoint, { rotationInvariant: true });
  let weights = seed.map(() => 0);
  for (const load of initial) {
    const index = candidates.findIndex(({ point, owner }) => contacts[owner].segment === load.segment
      && Math.hypot(point.x - load.point.x, point.y - load.point.y, point.z - load.point.z) < 1e-9);
    if (index < 0) return { plan, reason: 'missing-measured-owner' };
    weights[index] += load.share;
  }
  for (let iteration = 0; iteration < 4 * seed.length * seed.length; iteration++) {
    const matrix = [0, 1, 2].map(row => [0, 1, 2].map(col => columns.reduce((sum, a, i) => sum + (free[i] ? metric[i] * a[row] * a[col] : 0), 0)));
    const rhs = [1, 0, 0].map((value, row) => value - columns.reduce((sum, a, i) => sum + (free[i] ? seed[i] * a[row] : 0), 0));
    const lambda = solve(matrix, rhs);
    if (!lambda) return { plan, reason: 'rank-deficient-boundary' };
    const optimum = columns.map((a, i) => free[i] ? previous || reference === 'measured-pressure'
      ? seed[i] + metric[i] * dot(a, lambda) : seed[i] * (1 + dot(a, lambda)) : 0);
    let fraction = 1, blocked = -1;
    for (let i = 0; i < seed.length; i++) if (free[i] && optimum[i] < -1e-12) {
      const amount = Math.max(0, weights[i]) / (weights[i] - optimum[i]);
      if (amount < fraction) { fraction = amount; blocked = i; }
    }
    if (blocked >= 0) {
      weights = weights.map((value, i) => Math.max(0, value + fraction * (optimum[i] - value)));
      free[blocked] = false;
      continue;
    }
    weights = optimum.map(value => Math.max(0, value));
    let release = -1, multiplier = -1e-10;
    for (let i = 0; i < seed.length; i++) if (!free[i]) {
      const gradient = -seed[i] / metric[i] - dot(columns[i], lambda);
      if (gradient < multiplier) { multiplier = gradient; release = i; }
    }
    if (release >= 0) { free[release] = true; continue; }
    const residual = [1, 0, 0].map((value, row) => value - columns.reduce((sum, a, i) => sum + weights[i] * a[row], 0));
    if (Math.max(...residual.map(Math.abs)) > 1e-8 || weights.some(w => !Number.isFinite(w))) throw new Error('Experimental projection violated force/moment constraints');
    const loads = contacts.map((contact, owner) => {
      const share = candidates.reduce((sum, candidate, i) => sum + (candidate.owner === owner ? weights[i] : 0), 0);
      const point = Object.fromEntries(['x', 'y', 'z'].map(axis => [axis, share > 1e-12
        ? candidates.reduce((sum, candidate, i) => sum + (candidate.owner === owner ? candidate.point[axis] * weights[i] : 0), 0) / share : contact.point[axis]]));
      return { segment: contact.segment, point, plannedForce: Object.fromEntries(['x', 'y', 'z'].map(axis => [axis, plan.allocatedForce[axis] * share])),
        measuredForceN: contact.forceN, measuredPoint: { ...(contact.measuredPressurePoint ?? contact.point) } };
    });
    return { plan: { ...plan, loads, pressurePoint: Object.fromEntries(['x', 'y', 'z'].map(axis => [axis,
      candidates.reduce((sum, candidate, i) => sum + weights[i] * candidate.point[axis], 0)])) }, reason: 'projected-current-measured-points', iterations: iteration + 1 };
  }
  return { plan, reason: 'active-set-limit' };
}

// H8: hold aggregate patch command references only while they remain feasible
// on today's real manifold. A dual convex solve enforces each patch sum and
// global pressure, with nonnegative point weights throughout the final result.
export function holdPatchStandingLoads(plan, contacts, distribute, previous) {
  if (!previous) return { plan, reason: 'initialize' };
  if (!plan.pressureFeasible || !previous.pressureFeasible) return { plan, reason: 'pressure-infeasible' };
  if (!contacts.length || contacts.length !== previous.loads.length
    || contacts.some(c => !previous.loads.some(l => l.segment === c.segment))) {
    return { plan, reason: 'support-membership-changed' };
  }
  const priorTotal = previous.loads.reduce((sum, load) => sum + load.plannedForce.y, 0);
  if (priorTotal <= 0) return { plan, reason: 'no-prior-load-reference' };
  const shares = contacts.map(c => previous.loads.find(l => l.segment === c.segment).plannedForce.y / priorTotal);
  const candidates = contacts.flatMap((contact, owner) => {
    const prior = previous.loads.find(l => l.segment === contact.segment);
    const points = contact.points?.length ? contact.points : [contact.point];
    const seedLoads = distribute([contact], prior.point, { rotationInvariant: true });
    const weights = points.map(() => 0);
    for (const load of seedLoads) {
      const index = points.findIndex(point => Math.hypot(point.x - load.point.x, point.y - load.point.y, point.z - load.point.z) < 1e-9);
      if (index < 0) throw new Error('H8 lost measured point ownership');
      weights[index] += load.share * shares[owner];
    }
    return points.map((point, i) => ({ point, owner, seed: weights[i], metric: Math.max(shares[owner], 1e-8) / points.length }));
  });
  const width = Math.max(.01, ...candidates.map(c => Math.hypot(c.point.x - plan.pressurePoint.x, c.point.z - plan.pressurePoint.z)));
  const columns = candidates.map(c => [...contacts.map((_, i) => i === c.owner ? 1 : 0),
    (c.point.x - plan.pressurePoint.x) / width, (c.point.z - plan.pressurePoint.z) / width]);
  const target = [...shares, 0, 0], n = target.length;
  const state = lambda => {
    const raw = candidates.map((c, i) => c.seed + c.metric * dot(columns[i], lambda));
    const weights = raw.map(w => Math.max(0, w));
    const gradient = target.map((value, row) => columns.reduce((sum, a, i) => sum + a[row] * weights[i], 0) - value);
    const value = weights.reduce((sum, w, i) => sum + w * w / (2 * candidates[i].metric), 0) - dot(target, lambda);
    return { raw, weights, gradient, value };
  };
  function solveDual(matrix, rhs) {
    const a = matrix.map((row, i) => [...row, rhs[i]]), pivots = [];
    let rank = 0;
    for (let col = 0; col < n; col++) {
      let pivot = rank;
      for (let row = rank + 1; row < n; row++) if (Math.abs(a[row][col]) > Math.abs(a[pivot][col])) pivot = row;
      if (Math.abs(a[pivot][col]) < 1e-12) continue;
      [a[rank], a[pivot]] = [a[pivot], a[rank]];
      const divisor = a[rank][col];
      for (let j = col; j <= n; j++) a[rank][j] /= divisor;
      for (let row = 0; row < n; row++) if (row !== rank) {
        const factor = a[row][col];
        for (let j = col; j <= n; j++) a[row][j] -= factor * a[rank][j];
      }
      pivots.push(col); rank++;
    }
    if (a.slice(rank).some(row => Math.abs(row[n]) > 1e-9)) return null;
    const result = target.map(() => 0);
    pivots.forEach((col, row) => { result[col] = a[row][n]; });
    return result;
  }
  let lambda = target.map(() => 0);
  for (let iteration = 0; iteration < 64; iteration++) {
    const current = state(lambda);
    if (Math.max(...current.gradient.map(Math.abs)) < 1e-9) {
      const loads = contacts.map((contact, owner) => {
        const share = candidates.reduce((sum, c, i) => sum + (c.owner === owner ? current.weights[i] : 0), 0);
        const point = Object.fromEntries(['x', 'y', 'z'].map(axis => [axis, share > 1e-12
          ? candidates.reduce((sum, c, i) => sum + (c.owner === owner ? c.point[axis] * current.weights[i] : 0), 0) / share : contact.point[axis]]));
        return { segment: contact.segment, point, plannedForce: Object.fromEntries(['x', 'y', 'z'].map(axis => [axis, plan.allocatedForce[axis] * share])),
          measuredForceN: contact.forceN, measuredPoint: { ...(contact.measuredPressurePoint ?? contact.point) } };
      });
      return { plan: { ...plan, loads }, reason: 'held-patch-references-feasible', iterations: iteration };
    }
    const matrix = target.map((_, row) => target.map((_, col) => columns.reduce((sum, a, i) => sum
      + (current.raw[i] >= -1e-12 ? candidates[i].metric * a[row] * a[col] : 0), 0)));
    const direction = solveDual(matrix, current.gradient.map(value => -value));
    if (!direction) return { plan, reason: 'held-patch-shares-infeasible' };
    let step = 1;
    while (step > 1e-8) {
      const trial = lambda.map((value, i) => value + step * direction[i]);
      if (state(trial).value <= current.value + 1e-4 * step * dot(current.gradient, direction) + 1e-14) {
        lambda = trial; break;
      }
      step *= .5;
    }
    if (step <= 1e-8) return { plan, reason: 'held-patch-dual-stalled' };
  }
  return { plan, reason: 'held-patch-iteration-limit' };
}
