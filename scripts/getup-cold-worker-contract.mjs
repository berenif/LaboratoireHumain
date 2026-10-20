import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
export const coldWorkerContract=JSON.parse(readFileSync(new URL('./getup-cold-worker-contract.json',import.meta.url)));
export function checkColdWorkerCapabilities(value){
 assert.equal(value.schema,'lh-getup-compiled-capabilities/v1');
 assert.equal(value.physicsIntegrations,0);assert.equal(value.constructsRuntime,false);
 assert.deepEqual(value.effectiveSimFeatures,coldWorkerContract.effective_sim_features,
  'Wrong compiled controller feature closure; no physical entry permitted');
 return value;
}
