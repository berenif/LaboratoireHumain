import test from 'node:test';
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';

test('private experiment backup contracts, synthetic only', () => {
  const result = spawnSync('python3', ['scripts/experiment-backup-test.py'], {
    encoding: 'utf8', timeout: 30_000,
  });
  assert.equal(result.status, 0, `${result.stdout}\n${result.stderr}`);
});
