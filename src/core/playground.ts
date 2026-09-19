import type { Quat, Vec3 } from "./types";

export type PlaygroundDifficulty = "gentle" | "challenging" | "extreme";
export type PlaygroundStation = "flat" | "slope" | "rubble" | "beam" | "stones" | "wobble" | "hurdles";

export interface PlaygroundConfig {
  difficulty: PlaygroundDifficulty;
  station: PlaygroundStation;
}

export interface PlaygroundTrial {
  uprightSeconds: number;
  bestSeconds: number;
  falls: number;
}

export const DEFAULT_PLAYGROUND: Readonly<PlaygroundConfig> = { difficulty: "challenging", station: "flat" };
export const ARENA = { width: 20, depth: 17 } as const;

export const PLAYGROUND_STATIONS: ReadonlyArray<{
  id: PlaygroundStation; name: string; subtitle: string; instruction: string; color: string; position: Vec3;
}> = [
  { id: "flat", name: "Base camp", subtitle: "Find your feet", instruction: "Pull a hand or the torso to test recovery. Pick a station to raise the stakes.", color: "#70dfc2", position: { x: 0, y: 0, z: 3.3 } },
  { id: "slope", name: "The incline", subtitle: "Uphill / downhill", instruction: "An angled foothold. Pull uphill, downhill, or sideways and watch the ankles work.", color: "#f6b96c", position: { x: -5, y: 0, z: -2.1 } },
  { id: "rubble", name: "Broken ground", subtitle: "Roll / pitch / recover", instruction: "A rolling, banked surface with uneven footing. Small pulls expose the limits of support.", color: "#9eacf5", position: { x: 5, y: 0, z: -2.1 } },
  { id: "beam", name: "Knife edge", subtitle: "A very small margin", instruction: "A narrow, elevated beam. A sideways pull quickly takes the body outside its support.", color: "#70d6ed", position: { x: 0, y: 0, z: -1.6 } },
  { id: "stones", name: "Islands", subtitle: "Mind the gaps", instruction: "Offset stepping stones, different heights, and gaps. Guide the torso toward the next island.", color: "#c4a4f5", position: { x: -5, y: 0, z: 1.5 } },
  { id: "wobble", name: "Sea legs", subtitle: "The floor fights back", instruction: "The deck rolls and pitches under real contact. Try to stay upright as the tilt builds.", color: "#f48ca6", position: { x: 0, y: 0, z: -5.8 } },
  { id: "hurdles", name: "Trip wire", subtitle: "Steps / barriers / slalom", instruction: "Low barriers and offset blocks interrupt corrective steps. Pull the body into the course.", color: "#e4d572", position: { x: 5, y: 0, z: 1.15 } },
];

export const DIFFICULTY_LABELS: Readonly<Record<PlaygroundDifficulty, string>> = {
  gentle: "Gentle", challenging: "Challenging", extreme: "Extreme",
};

export interface CourseGeometry {
  vertices: readonly Vec3[];
  triangles: readonly (readonly [number, number, number])[];
  surfaceTriangles?: number;
}

export interface CoursePiece {
  id: string;
  geometry: CourseGeometry;
  position: Vec3;
  color: string;
  friction: number;
  tiled?: boolean;
  motion?: { pitch: number; roll: number; speed: number };
}

export interface CourseTransform { position: Vec3; rotation: Quat }

const BOX_TRIANGLES: CourseGeometry["triangles"] = [
  [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
  [0, 1, 5], [0, 5, 4], [3, 7, 6], [3, 6, 2],
  [0, 4, 7], [0, 7, 3], [1, 2, 6], [1, 6, 5],
];

/** The renderers and Rapier consume these same surfaces, including every slope. */
export function boxGeometry(width: number, height: number, depth: number): CourseGeometry {
  const x = width / 2, y = height / 2, z = depth / 2;
  return { vertices: [
    { x: -x, y: -y, z: -z }, { x, y: -y, z: -z }, { x, y: -y, z }, { x: -x, y: -y, z },
    { x: -x, y, z: -z }, { x, y, z: -z }, { x, y, z }, { x: -x, y, z },
  ], triangles: BOX_TRIANGLES.map(([a, b, c]) => [a, c, b] as const) };
}

function terrainGeometry(width: number, depth: number, columns: number, rows: number,
  height: (x: number, z: number) => number): CourseGeometry {
  const vertices: Vec3[] = [];
  const triangles: [number, number, number][] = [];
  for (let row = 0; row <= rows; row++) for (let column = 0; column <= columns; column++) {
    const x = width * (column / columns - 0.5), z = depth * (row / rows - 0.5);
    vertices.push({ x, y: height(x, z), z });
  }
  for (let row = 0; row < rows; row++) for (let column = 0; column < columns; column++) {
    const a = row * (columns + 1) + column, b = a + 1, c = a + columns + 1, d = c + 1;
    triangles.push([a, c, b], [b, c, d]);
  }
  // Close the perimeter down to the floor so the terrain has physical sides.
  const perimeter = [
    ...Array.from({ length: columns }, (_, i) => i),
    ...Array.from({ length: rows }, (_, i) => i * (columns + 1) + columns),
    ...Array.from({ length: columns }, (_, i) => rows * (columns + 1) + columns - i),
    ...Array.from({ length: rows }, (_, i) => (rows - i) * (columns + 1)),
  ];
  for (let i = 0; i < perimeter.length; i++) {
    const a = perimeter[i], b = perimeter[(i + 1) % perimeter.length];
    const c = vertices.length, d = c + 1;
    vertices.push({ ...vertices[a], y: 0 }, { ...vertices[b], y: 0 });
    triangles.push([a, b, c], [b, d, c]);
  }
  return { vertices, triangles, surfaceTriangles: columns * rows * 2 };
}

const courses = new Map<PlaygroundDifficulty, readonly CoursePiece[]>();

export function getPlaygroundCourse(difficulty: PlaygroundDifficulty): readonly CoursePiece[] {
  const cached = courses.get(difficulty);
  if (cached) return cached;
  const level = difficulty === "gentle" ? 0 : difficulty === "extreme" ? 2 : 1;
  const pieces: CoursePiece[] = [];
  const box = (id: string, x: number, y: number, z: number, w: number, h: number, d: number, color: string, friction = 1.25) => {
    pieces.push({ id, position: { x, y, z }, geometry: boxGeometry(w, h, d), color, friction });
  };
  const angle = (10 + level * 9) * Math.PI / 180;
  pieces.push({ id: "incline", position: { x: -5, y: 0, z: -2.1 }, color: "#c9975c", friction: 1.3,
    geometry: terrainGeometry(3.2, 4.2, 4, 8, (_x, z) => 0.025 + (2.1 - z) * Math.tan(angle)) });
  // A separate cross-slope makes lateral loading visible alongside the main ramp.
  pieces.push({ id: "cross-slope", position: { x: -7.65, y: 0, z: -2.1 }, color: "#a97c4e", friction: 1.15, tiled: true,
    geometry: terrainGeometry(1.6, 4.2, 3, 8, (x, z) => 0.035 + (x + 0.8) * Math.tan(angle) + 0.09 * (1 + Math.sin(z * 1.6))) });
  pieces.push({ id: "broken-ground", position: { x: 5, y: 0, z: -2.1 }, color: "#818fab", friction: 1.1, tiled: true,
    geometry: terrainGeometry(3.6, 4.2, 8, 10, (x, z) => {
      const edge = Math.min(1, (1.8 - Math.abs(x)) * 2.8, (2.1 - Math.abs(z)) * 2.8);
      return 0.035 + Math.max(0, edge) * (0.23 + level * 0.07
        + (0.065 + level * 0.035) * Math.sin(x * 4.5 + z * 2.6)
        + (0.065 + level * 0.025) * Math.cos(z * 3.7 - x));
    }) });
  const beamWidth = [0.64, 0.42, 0.26][level];
  const beamTop = [0.3, 0.52, 0.76][level];
  box("beam", 0, beamTop - 0.10, -1.6, beamWidth, 0.2, 4.2, "#5ca6b8");
  for (const z of [-3.2, 0]) box(`beam-leg:${z}`, 0, (beamTop - 0.2) / 2, z, 0.23, beamTop - 0.2, 0.25, "#304955");
  box("beam-start", 0, 0.08, 0.72, 1.3, 0.16, 0.45, "#497688");
  for (let i = 0; i < 6; i++) {
    const h = 0.12 + (i % 3) * (0.065 + level * 0.035);
    const w = [0.9, 0.72, 0.56][level];
    box(`island:${i}`, -5 + (i === 0 ? 0 : Math.sin(i * 2.1) * (0.26 + level * 0.15)), h / 2,
      1.5 + i * (0.81 + level * 0.065), w, h, 0.63 - level * 0.06, i % 2 ? "#8b7da6" : "#a291c4");
  }
  box("wobble-pedestal", 0, 0.14, -5.8, 0.8, 0.28, 0.8, "#73495b");
  pieces.push({ id: "wobble-deck", geometry: boxGeometry(2.8, 0.16, 2.5), position: { x: 0, y: 0.64, z: -5.8 },
    color: "#ba7188", friction: 1.35, motion: {
      pitch: [0.07, 0.15, 0.23][level], roll: [0.09, 0.18, 0.29][level], speed: [0.7, 1, 1.3][level],
    } });
  for (let i = 0; i < 4; i++) {
    const h = 0.1 + level * 0.04 + i * 0.055;
    box(`hurdle:${i}`, 5 + (i % 2 ? 0.25 : -0.25), h / 2, 2 + i * 0.9,
      1.9 - level * 0.15, h, 0.16 + i * 0.05, i % 2 ? "#b9a858" : "#d0be71");
  }
  for (const [i, x, z] of [[0, 3.7, 2.5], [1, 6.35, 3.4], [2, 3.7, 4.6]]) {
    box(`bollard:${i}`, x, 0.3 + level * 0.06, z, 0.4, 0.6 + level * 0.12, 0.4, "#b8ab69");
  }
  // Low perimeter curbs are collidable too; the ground extends beyond them.
  box("curb:north", 0, 0.07, -8, 18, 0.14, 0.14, "#385262");
  box("curb:south", 0, 0.07, 7.5, 18, 0.14, 0.14, "#385262");
  box("curb:west", -9, 0.07, -0.25, 0.14, 0.14, 15.5, "#385262");
  box("curb:east", 9, 0.07, -0.25, 0.14, 0.14, 15.5, "#385262");
  courses.set(difficulty, pieces);
  return pieces;
}

/** A two-second ramp-in gives the subject time to load its feet before motion. */
export function courseTransform(piece: CoursePiece, time: number): CourseTransform {
  if (!piece.motion) return { position: piece.position, rotation: { x: 0, y: 0, z: 0, w: 1 } };
  const ramp = Math.min(1, Math.max(0, (time - 1) / 2));
  const t = Math.max(0, time - 1) * piece.motion.speed;
  const pitch = Math.sin(t * 1.3) * piece.motion.pitch * ramp;
  const roll = Math.sin(t * 0.93) * piece.motion.roll * ramp;
  const sx = Math.sin(pitch / 2), cx = Math.cos(pitch / 2), sz = Math.sin(roll / 2), cz = Math.cos(roll / 2);
  return { position: piece.position, rotation: { x: sx * cz, y: -sx * sz, z: cx * sz, w: cx * cz } };
}

export function playgroundStation(id: PlaygroundStation) {
  return PLAYGROUND_STATIONS.find(station => station.id === id) ?? PLAYGROUND_STATIONS[0];
}
