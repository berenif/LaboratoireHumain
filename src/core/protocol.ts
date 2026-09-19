import { boxGeometry, type CourseGeometry } from "./playground";
import type { Vec3 } from "./types";

/** Dimensions shared by the physical room and both presentation adapters. */
export const PROTOCOL_ROOM = Object.freeze({ width: 10, depth: 10, height: 4.2 });

/** Half extents of the kinematic impact head, in its local frame. Local +Z strikes. */
export const STRIKER_HEAD = Object.freeze({ x: 0.26, y: 0.21, z: 0.32 });

export const STRIKER_STOW_HEIGHT = 3.45;

export interface ProtocolVisualPiece {
  id: string;
  geometry: CourseGeometry;
  position: Vec3;
  color: string;
  opacity?: number;
}

/** All positions are local to the physical impact head; local +Z is forward. */
export const PROTOCOL_STRIKER_PIECES: readonly ProtocolVisualPiece[] = [
  { id: "head", geometry: boxGeometry(STRIKER_HEAD.x * 2, STRIKER_HEAD.y * 2, STRIKER_HEAD.z * 2), position: { x: 0, y: 0, z: 0 }, color: "#edb421" },
  { id: "collar", geometry: boxGeometry(0.68, 0.48, 0.16), position: { x: 0, y: 0, z: -0.35 }, color: "#39433b" },
  { id: "shaft", geometry: boxGeometry(0.22, 0.22, 0.65), position: { x: 0, y: 0, z: -0.69 }, color: "#c89724" },
  { id: "actuator", geometry: boxGeometry(0.48, 0.46, 0.42), position: { x: 0, y: 0, z: -1.19 }, color: "#d5a025" },
];
