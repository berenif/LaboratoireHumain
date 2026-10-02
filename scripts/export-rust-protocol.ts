import { createHash } from 'node:crypto';
import { readFileSync, writeFileSync } from 'node:fs';
import { PROTOCOL_ROOM, PROTOCOL_STRIKER_PIECES, STRIKER_HEAD, STRIKER_STOW_HEIGHT } from '../src/core/protocol';

// Additive canonical protocol asset; the frozen anatomy/course export is untouched.
const path = 'rust/crates/model/data/protocol-v1.json';
const bytes = JSON.stringify({ room: PROTOCOL_ROOM, strikerHead: STRIKER_HEAD,
  stowHeight: STRIKER_STOW_HEIGHT, pieces: PROTOCOL_STRIKER_PIECES }) + '\n';
const digest = (value: string | Buffer) => createHash('sha256').update(value).digest('hex');
writeFileSync(path, bytes, { flag: 'wx' });
const sources = ['src/core/protocol.ts', 'src/core/playground.ts', 'scripts/export-rust-protocol.ts'];
writeFileSync('rust/crates/model/data/protocol-provenance-v1.json', JSON.stringify({
  generatedAt: new Date().toISOString(), command: 'node --import tsx scripts/export-rust-protocol.ts',
  sha256: digest(bytes), sources: Object.fromEntries(sources.map(path => [path, digest(readFileSync(path))])),
}, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ path, sha256: digest(bytes), pieces: PROTOCOL_STRIKER_PIECES.length }));
