import type { RegionId } from "../core/types";

export const BACKGROUND = "#f5f3ee";
export const SELECTION = "#16877f";
export const BODY_COLORS: Record<RegionId, string> = {
  head: "#b49c81", torso: "#799c9e", pelvis: "#8e98aa", leftHand: "#bd978b",
  rightHand: "#bd978b", leftFoot: "#8eaa94", rightFoot: "#8eaa94",
};
export const JOINT_COLOR = "#697673";

/** Muted display colors; leave course definitions and canonical physics untouched. */
export function sceneryColor(hex: string): string {
  const rgb = [1, 3, 5].map(start => parseInt(hex.slice(start, start + 2), 16));
  const mean = (rgb[0] + rgb[1] + rgb[2]) / 3;
  return `#${rgb.map(value => Math.round(value * 0.35 + mean * 0.25 + 240 * 0.4).toString(16).padStart(2, "0")).join("")}`;
}
