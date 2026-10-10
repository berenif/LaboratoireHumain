import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFileSync } from 'node:fs';

export function deriveRecoveryBrowserLimits(contract, profile) {
  const substepHz = Math.round(1 / profile.dt_s), substepsPerTick = profile.tick_substeps;
  assert.equal(substepHz, 240, 'Pinned recovery integrated clock changed');
  assert.ok(Number.isFinite(profile.dt_s) && Math.abs(profile.dt_s * substepHz - 1) <= 1e-6, 'Invalid pinned substep duration');
  assert.ok(Number.isSafeInteger(substepsPerTick) && substepsPerTick > 0);
  const tickHz = substepHz / substepsPerTick;
  assert.ok(Number.isSafeInteger(tickHz) && Math.abs(tickHz * contract.acceptance.fixedDtS - 1) <= 1e-9,
    'Frozen tick clock differs from the pinned profile');
  const integrated = seconds => {
    const count = seconds * substepHz;
    assert.ok(Number.isSafeInteger(count) && count > 0, 'Frozen interval is not an exact integrated substep count');
    return count;
  };
  assert.ok(Number.isSafeInteger(contract.acceptance.repeatedCycles) && contract.acceptance.repeatedCycles > 0);
  return Object.freeze({ substepHz, substepsPerTick, tickHz,
    recoveryLimitSubsteps: integrated(contract.acceptance.recoveryLimitS),
    stableSubsteps: integrated(contract.acceptance.stableObservationS),
    stableTicks: Math.ceil(integrated(contract.acceptance.stableObservationS) / substepsPerTick),
    stableActualMs: contract.acceptance.stableObservationS * 1000,
    repeatedCycles: contract.acceptance.repeatedCycles,
    dragMetres: 0.02, dragActualMs: 250, dragSubsteps: integrated(0.25) });
}
const contractBytes = readFileSync('rust/crates/sim/data/recovery-contract-v1.json');
export const recoveryContractSha256 = createHash('sha256').update(contractBytes).digest('hex');
export const recoveryBrowserLimits = deriveRecoveryBrowserLimits(JSON.parse(contractBytes),
  JSON.parse(readFileSync('rust/crates/sim/data/recovery-profile-v24.json')));
