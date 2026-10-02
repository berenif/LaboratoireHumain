import { boxGeometry } from "../core/playground";
import type { ProtocolVisualPiece } from "../core/protocol";
export { PROTOCOL_STRIKER_PIECES } from "../core/protocol";
export type { ProtocolVisualPiece } from "../core/protocol";

/** Fixed overview used by the protocol room and its visual replay. */
export const PROTOCOL_ARENA_CAMERA = Object.freeze({
  position: { x: 1, y: 3.7, z: 7.8 },
  target: { x: 0, y: 1.4, z: -1 },
});

/** The interior faces coincide with the Rapier room dimensions. */
export function protocolRoomPieces(room: { width: number; depth: number; height: number }): readonly ProtocolVisualPiece[] {
  const { width, depth, height } = room;
  const wall = 0.18;
  return [
    { id: "floor", geometry: boxGeometry(width, 0.12, depth), position: { x: 0, y: -0.06, z: 0 }, color: "#d9d9cd" },
    { id: "back-wall", geometry: boxGeometry(width + wall * 2, height, wall), position: { x: 0, y: height / 2, z: -depth / 2 - wall / 2 }, color: "#e7e7dc" },
    { id: "left-wall", geometry: boxGeometry(wall, height, depth), position: { x: -width / 2 - wall / 2, y: height / 2, z: 0 }, color: "#e0e1d7" },
    { id: "right-wall", geometry: boxGeometry(wall, height, depth), position: { x: width / 2 + wall / 2, y: height / 2, z: 0 }, color: "#d4d7ce", opacity: 0.13 },
    { id: "front-wall", geometry: boxGeometry(width + wall * 2, height, wall), position: { x: 0, y: height / 2, z: depth / 2 + wall / 2 }, color: "#d4d7ce", opacity: 0.09 },
    ...[-1, 1].flatMap(sign => [
      { id: `boundary-x-${sign}`, geometry: boxGeometry(width - 0.5, 0.008, 0.025),
        position: { x: 0, y: 0.006, z: sign * (depth / 2 - 0.25) }, color: "#7a9990" },
      { id: `boundary-z-${sign}`, geometry: boxGeometry(0.025, 0.008, depth - 0.5),
        position: { x: sign * (width / 2 - 0.25), y: 0.006, z: 0 }, color: "#7a9990" },
    ]),
  ];
}
