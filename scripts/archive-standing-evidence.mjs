import assert from 'node:assert/strict';
import { existsSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { basename, dirname, isAbsolute, join, relative, resolve, sep } from 'node:path';
import { gzipSync, gunzipSync } from 'node:zlib';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';

const peakWindows = process.argv.includes('--peak-windows');
const [mode, output, ...inputs] = process.argv.slice(2).filter(arg => arg !== '--peak-windows');
if (mode === 'verify') {
  const manifest = JSON.parse(readFileSync(output, 'utf8'));
  const root = resolve(dirname(output));
  for (const file of manifest.files) {
    const path = resolve(root, file.archive), offset = relative(root, path);
    assert.ok(offset && !isAbsolute(offset) && offset !== '..' && !offset.startsWith(`..${sep}`));
    const bytes = readFileSync(path), unpacked = gunzipSync(bytes);
    assert.equal(sha256(bytes), file.archiveSha256, file.archive);
    assert.equal(sha256(unpacked), file.sha256, file.original);
    assert.equal(unpacked.length, file.bytes);
  }
  console.log(JSON.stringify({ verified: manifest.files.length, manifest: output }));
} else if (mode === 'create' && output && inputs.length && !existsSync(output)) {
  mkdirSync(output, { recursive: true });
  const files = [];
  for (const input of inputs) {
    const name = basename(resolve(input));
    mkdirSync(join(output, name));
    const report = existsSync(join(input, 'report.json')) ? JSON.parse(readFileSync(join(input, 'report.json'), 'utf8')) : null;
    for (const entry of readdirSync(input, { withFileTypes: true })) {
      if (!entry.isFile() || !/\.(json|jsonl|bin|tgz)$/.test(entry.name)) continue;
      const original = join(input, entry.name), originalBytes = readFileSync(original);
      let bytes = originalBytes, extraction;
      if (peakWindows && entry.name.endsWith('.jsonl') && bytes.length) {
        const run = report?.runs.find(run => run.trace?.file === entry.name);
        assert.ok(run, 'Trace must be identified in its run report');
        const centers = [175, 176, ...Object.values(run.peaks).map(peak => peak.tick)];
        const rows = bytes.toString('utf8').trim().split('\n').filter(line => {
          const { tick } = JSON.parse(line);
          return centers.some(center => Math.abs(tick - center) <= 5);
        });
        bytes = Buffer.from(rows.join('\n') + '\n');
        extraction = { originalBytes: originalBytes.length, originalSha256: sha256(originalBytes),
          tickCenters: centers, radiusTicks: 5, rows: rows.length,
          fullTraceAvailability: 'local-only at original path; report retains full response history' };
      }
      // Plain controls have no event trace by design; their complete physical
      // history and trajectory digest live in report.json.
      if (!bytes.length) continue;
      const archive = `${name}/${entry.name}.gz`, packed = gzipSync(bytes);
      writeFileSync(join(output, archive), packed, { flag: 'wx' });
      files.push({ original, archive, bytes: bytes.length, sha256: sha256(bytes), archiveSha256: sha256(packed), ...(extraction ? { extraction } : {}) });
    }
  }
  const source = fingerprints();
  const production = Object.fromEntries(Object.keys(source).filter(path => path.startsWith('src/')
    || ['package.json', 'package-lock.json'].includes(path)).map(path => [path, readFileSync(path, 'utf8')]));
  const bytes = Buffer.from(JSON.stringify(production)), packed = gzipSync(bytes), archive = 'production-source.json.gz';
  writeFileSync(join(output, archive), packed, { flag: 'wx' });
  files.push({ original: 'production source snapshot at archival time; compare hashes with each report', archive,
    bytes: bytes.length, sha256: sha256(bytes), archiveSha256: sha256(packed) });
  const scripts = Buffer.from(JSON.stringify(Object.fromEntries(Object.keys(source).filter(path => path.startsWith('scripts/'))
    .map(path => [path, readFileSync(path, 'utf8')]))));
  const scriptArchive = 'diagnostic-source.json.gz', scriptPacked = gzipSync(scripts);
  writeFileSync(join(output, scriptArchive), scriptPacked, { flag: 'wx' });
  files.push({ original: 'diagnostic source snapshot at archival time; compare hashes with each report', archive: scriptArchive,
    bytes: scripts.length, sha256: sha256(scripts), archiveSha256: sha256(scriptPacked) });
  writeFileSync(join(output, 'manifest.json'), JSON.stringify({ generatedAt: new Date().toISOString(), source,
    qualification: 'New diagnostic evidence, including failed experiments. Not the missing historical fixture or production acceptance.', files }, null, 2) + '\n', { flag: 'wx' });
  console.log(JSON.stringify({ files: files.length, output }));
} else throw new Error('Usage: create fresh-output-directory run-directory... | verify manifest.json');
