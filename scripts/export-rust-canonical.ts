import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { SEGMENTS, HUMAN_PROPORTIONS, TOTAL_MASS_KG } from '../src/core/humanoid';
import { flattenGeometryVertices, flattenGeometryIndices } from '../src/core/geometry';
import { integrateConvexMass } from '../src/core/geometry-mass';
import { PLAYGROUND_STATIONS, getPlaygroundCourse } from '../src/core/playground';
import { PROTOCOL_ROOM, STRIKER_HEAD } from '../src/core/protocol';
import { createEmbodiedCharacter } from '../src/character/EmbodiedCharacter';

const directory = 'rust/crates/model/data';
mkdirSync(directory, { recursive: true });
const path = `${directory}/canonical-v1.json`;
if (existsSync(path)) throw new Error('Canonical versions are immutable; choose a new version');
const character = await createEmbodiedCharacter('canvas2d');
const poses = character.getSnapshot('canvas2d').segments;
character.dispose();
const data = { schemaVersion: 1, totalMassKg: TOTAL_MASS_KG, statureM: HUMAN_PROPORTIONS.totalHeightM,
  segments: SEGMENTS.map(segment => {
    const vertices = Array.from(flattenGeometryVertices(segment.geometry));
    const indices = Array.from(flattenGeometryIndices(segment.geometry));
    return { ...segment, geometry: { vertices, indices },
      goldenMass: integrateConvexMass(vertices, indices, segment.massKg),
      initial: poses.find(pose => pose.id === segment.id) };
  }), playground: { stations: PLAYGROUND_STATIONS,
    courses: Object.fromEntries(['gentle', 'challenging', 'extreme'].map(d => [d, getPlaygroundCourse(d as 'gentle' | 'challenging' | 'extreme')])) },
  protocol: { room: PROTOCOL_ROOM, strikerHead: STRIKER_HEAD } };
const bytes = JSON.stringify(data) + '\n';
writeFileSync(path, bytes, { flag: 'wx' });
const sourcePaths = ['src/core/humanoid.ts','src/core/geometry.ts','src/core/geometry-mass.ts','src/core/playground.ts','src/core/protocol.ts',
  'src/character/EmbodiedCharacter.ts'];
const digest = (bytes: string | Buffer) => createHash('sha256').update(bytes).digest('hex');
writeFileSync(`${directory}/provenance-v1.json`, JSON.stringify({ schema: 1, generatedAt: new Date().toISOString(),
  command: 'node --import tsx scripts/export-rust-canonical.ts', canonicalSha256: digest(bytes),
  sources: Object.fromEntries(sourcePaths.map(p => [p, digest(readFileSync(p))])),
  note: 'Exact canonical f32 collider vertices and indices; prescribed masses; unstepped actual legacy initialization. Golden mass tensor used only for independent port verification.' }, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ path, segments: data.segments.length, massKg: data.totalMassKg, bytes: bytes.length, sha256: digest(bytes) }));
