import assert from 'node:assert/strict';
import { solveDampedAngularStep } from './standing-forefoot-authority.mjs';

const feet = ['leftFoot', 'leftForefoot', 'rightFoot', 'rightForefoot'];
const excess = (vector, bound, normalization) => {
  const values = [vector.x, vector.y, vector.z], norm = Math.hypot(...values);
  const factor = norm > bound ? (1 - bound / norm) / normalization : 0;
  return values.map(v => v * factor);
};

/** Copied-world search metrics, independent of the physical acceptance criteria. */
export function coupledSpeedMetrics(bodies) {
  const residual = Object.values(bodies).flatMap(b => [...excess(b.velocity, 0.08, 0.1), ...excess(b.angularVelocity, 0.4, 0.5)]);
  const finite = Object.values(bodies).every(b => Object.values(b).flatMap(v => typeof v === 'number' ? [v] : Object.values(v)).every(Number.isFinite));
  const linear = Math.max(...Object.values(bodies).map(b => Math.hypot(...Object.values(b.velocity))));
  const angular = Math.max(...Object.values(bodies).map(b => Math.hypot(...Object.values(b.angularVelocity))));
  const footAngular = Math.max(...feet.map(id => Math.hypot(...Object.values(bodies[id].angularVelocity))));
  return { coupledResidual: residual, coupledScore: residual.reduce((s, v) => s + v * v, 0),
    coupledGuard: finite && linear <= 0.1 && angular <= 0.5 && footAngular <= 0.4,
    coupledReserve: finite && linear <= 0.08 && angular <= 0.4 };
}

/** Search copies only. Return a guarded candidate, or retain the original command. */
export function searchCoupledSpeedPreview(axes, start, simulate) {
  const cases = [];
  const query = (...args) => { const candidate = simulate(...args); cases.push(candidate); return candidate; };
  let best = start;
  for (let iteration = 0; iteration < 8 && !best.coupledReserve; iteration++) {
    const probeStart = cases.length;
    const columns = axes.map((a, i) => {
      const p = [...best.deltaTorqueNm], m = [...best.deltaTorqueNm];
      p[i] = Math.min(a.unchangedTorqueCeilingNm, p[i] + 1); m[i] = Math.max(-a.unchangedTorqueCeilingNm, m[i] - 1);
      const plus = query(`coupled-${iteration}-${i}-plus`, p), minus = query(`coupled-${iteration}-${i}-minus`, m);
      assert.ok(plus.finite && minus.finite);
      return plus.coupledResidual.map((v, j) => (v - minus.coupledResidual[j]) / (p[i] - m[i]));
    });
    const step = solveDampedAngularStep(columns.map((c, i) => c.map(v => v * axes[i].unchangedTorqueCeilingNm)),
      best.coupledResidual).map((v, i) => v * axes[i].unchangedTorqueCeilingNm);
    const trust = Math.max(1, ...step.map((v, i) => Math.abs(v) / axes[i].unchangedTorqueCeilingNm));
    let bestProbe = cases.slice(probeStart).filter(c => c.finite && c.coupledScore < best.coupledScore - 1e-14)
      .sort((a, b) => a.coupledScore - b.coupledScore)[0];
    if (bestProbe) {
      const direction = bestProbe.deltaTorqueNm.map((v, i) => v - best.deltaTorqueNm[i]);
      const seen = new Set([JSON.stringify(bestProbe.deltaTorqueNm)]);
      for (const factor of [2, 4, 8, 16, 32, 64, 128]) {
        const delta = axes.map((a, i) => Math.max(-a.unchangedTorqueCeilingNm, Math.min(a.unchangedTorqueCeilingNm,
          best.deltaTorqueNm[i] + factor * direction[i])));
        const key = JSON.stringify(delta); if (seen.has(key)) continue; seen.add(key);
        const candidate = query(`coupled-${iteration}-probe-expansion-${factor}`, delta);
        if (candidate.finite && candidate.coupledScore < bestProbe.coupledScore) bestProbe = candidate;
      }
    }
    let next;
    for (const fraction of [1, 0.5, 0.25, 0.125, 0.0625, 0.03125]) {
      const delta = axes.map((a, i) => Math.max(-a.unchangedTorqueCeilingNm, Math.min(a.unchangedTorqueCeilingNm,
        best.deltaTorqueNm[i] + fraction * step[i] / trust)));
      const candidate = query(`coupled-${iteration}-line-${fraction}`, delta);
      if (candidate.finite && candidate.coupledScore < best.coupledScore - 1e-14) { next = candidate; break; }
    }
    if (bestProbe && (!next || bestProbe.coupledScore < next.coupledScore)) next = bestProbe;
    if (!next) break; best = next;
  }
  const guarded = best.coupledGuard ? best : [start, ...cases].filter(c => c.coupledGuard)
    .sort((a, b) => a.coupledScore - b.coupledScore)[0];
  return { best: guarded ?? start, searchBest: best, cases: cases.length,
    guardFound: Boolean(guarded), reserveReached: guarded?.coupledReserve ?? false };
}
