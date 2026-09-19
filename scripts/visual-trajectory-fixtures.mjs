/** Input-only visual replays of the frozen physics-acceptance trajectories. */
import { PULL_FIXTURES } from "./physics-fixtures.ts";
import { add, lerp, quatFromAxisAngle, rotate, worldPoint } from "../src/character/math.ts";

const DT = 1 / 60;
const ZERO = { x: 0, y: 0, z: 0 }, UP = { x: 0, y: 1, z: 0 };
export const VISUAL_TRAJECTORY_FIXTURES = [
  ...PULL_FIXTURES.map(fixture => ({ id: fixture.id, kind: "pull", heading: fixture.initial.heading, fixture })),
  ...[0, Math.PI / 3].flatMap(heading => ["leftHand", "rightHand", "leftFoot", "rightFoot"].flatMap(limb =>
    (limb.endsWith("Hand") ? ["chest", "abdomen", "opposite-shoulder"] : ["midline"]).map(route => ({
      id: `cross-body-${limb}-${route}-${heading === 0 ? "front" : "rotated"}`,
      kind: "cross-body", heading, limb, route,
    })))),
];

const pose = (character, id) => character.getSnapshot("webgl").segments.find(segment => segment.id === id);
const warmup = (character, frames) => { for (let frame = 0; frame < frames; frame++) character.fixedUpdate(DT, null); };
function begin(character, region, pointerId, localAnchor = ZERO) {
  const segment = pose(character, region);
  const start = worldPoint(segment.position, segment.rotation, localAnchor);
  return { start, command: { kind: "begin", pointerId, region, segment: region, localAnchor, worldTarget: start,
    timestampMs: character.getSnapshot("webgl").simulationTime * 1000 } };
}
function offsetAt(fixture, frame) {
  const next = fixture.targets.findIndex(target => target.frame >= frame);
  if (next < 0) return fixture.targets.at(-1).offset;
  if (next === 0) return fixture.targets[0].offset;
  const a = fixture.targets[next - 1], b = fixture.targets[next];
  return lerp(a.offset, b.offset, (frame - a.frame) / (b.frame - a.frame));
}

/** Starts at harness frame zero, immediately after its begin command. */
export function prepareVisualTrajectory(character, descriptor) {
  if (descriptor.kind === "pull") {
    const fixture = descriptor.fixture;
    warmup(character, fixture.warmupFrames ?? 0);
    const grab = begin(character, fixture.region, 41, fixture.localAnchor);
    if (fixture.initialTargetWorldOffset) {
      grab.start = add(pose(character, fixture.region).position, fixture.initialTargetWorldOffset);
      grab.command.worldTarget = grab.start;
    }
    if (fixture.beginTimestampMs !== undefined) grab.command.timestampMs = fixture.beginTimestampMs;
    character.fixedUpdate(DT, grab.command);
    let release = fixture.releaseFrame, releasedInSwing = false;
    return { lastFrame: fixture.observeUntilFrame, acceptanceThroughFrame: fixture.observeUntilFrame,
      reviewFrames: fixture.id === "slow-hand-forward" ? [105, 115, 120, 125, 130, 135, 140, 143, 150, 180, 240, 330, 420] : [],
      commandAt(frame) {
        if (fixture.releaseAtFirstSwing && !releasedInSwing && character.getSnapshot("webgl").support.swingFoot) {
          release = frame; releasedInSwing = true;
        }
        if (frame === release) return { kind: "end", pointerId: 41, timestampMs: frame * DT * 1000 };
        if (frame > release || fixture.holdWithoutCommandsAfterLastTarget && frame > fixture.targets.at(-1).frame) return null;
        return { kind: "move", pointerId: 41,
          worldTarget: add(grab.start, rotate(quatFromAxisAngle(UP, fixture.initial.heading), offsetAt(fixture, frame))),
          timestampMs: frame * DT * 1000 };
      },
    };
  }
  warmup(character, 60);
  const grab = begin(character, descriptor.limb, 701);
  character.fixedUpdate(DT, grab.command);
  const side = descriptor.limb.startsWith("left") ? 1 : -1;
  // As in acceptance, this reference is sampled AFTER the begin update.
  const reference = pose(character, descriptor.limb.endsWith("Hand") ? "torso" : "pelvis");
  const localTarget = descriptor.route === "chest" ? { x: side * 0.18, y: 0, z: 0.14 }
    : descriptor.route === "abdomen" ? { x: side * 0.16, y: -0.24, z: 0.13 }
      : descriptor.route === "opposite-shoulder" ? { x: side * 0.28, y: 0.12, z: 0.10 }
        : { x: side * 0.16, y: -0.85, z: 0.13 };
  const target = worldPoint(reference.position, reference.rotation, localTarget);
  return { lastFrame: 151, acceptanceThroughFrame: 91,
    reviewFrames: [30, 45, 60, 75, 85, 90, 91, 105, 120, 151],
    commandAt(frame) {
      if (frame === 91) return { kind: "end", pointerId: 701, timestampMs: 151 * DT * 1000 };
      if (frame > 91) return null;
      return { kind: "move", pointerId: 701, worldTarget: lerp(grab.start, target, frame / 90),
        timestampMs: (60 + frame) * DT * 1000 };
    },
  };
}
