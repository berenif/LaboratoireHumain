// Match the actual historical browser archive boundary. JSON number−0 becomes0;
// raw Uint32 pose/physical words and raw carry JSON strings remain exact.
import assert from 'node:assert/strict';
export function assertExactArchiveRow(actual, expectedRawLine, label='native row') {
 const expected=JSON.parse(expectedRawLine);
 for (const key of ['poseWords','physicalObservationWords','rootTranslationIntegratorStateJson'])
  assert.deepStrictEqual(actual[key],expected[key],label+' exact raw state '+key);
 assert.equal(JSON.stringify(actual),expectedRawLine,label+' exact historical JSON archive bytes');
}
