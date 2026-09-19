import RAPIER, { type Collider, type RigidBody, type World } from "@dimforge/rapier3d-compat";
import { PROTOCOL_ROOM, PROTOCOL_STRIKER_PIECES, STRIKER_HEAD, STRIKER_STOW_HEIGHT } from "../core/protocol";
import type { PoseSnapshot, Quat, SegmentId, Vec3 } from "../core/types";
import { add, clamp, normalize, quatFromAxisAngle, quatFromTo, quatMultiply, rotate, scale, sub } from "./math";

type Phase = NonNullable<PoseSnapshot["striker"]>["phase"];
const IDENTITY: Quat = { x: 0, y: 0, z: 0, w: 1 };
// Calibrated against the real torso contact: it topples the neutral subject
// while keeping the impulsive shoulder motion within structural joint limits.
const STRIKE_SPEED_MPS = 4.0;
const STANDOFF_M = 1.28;
const MIN_STANDOFF_M = 0.72;
const MAX_TRAVEL_M = 1.60;
const POSITION_OVERHEAD_S = 0.03;
const POSITION_DESCEND_S = 0.03;
const RETRACT_TO_START_S = 0.08;
const RETRACT_STOW_S = 0.06;
const RETRACT_HOME_S = 0.08;
const ROOM_MARGIN_M = 0.02;
const HOME: Vec3 = { x: 0, y: STRIKER_STOW_HEIGHT, z: 2.1 };
const ASSEMBLY_VERTICES = PROTOCOL_STRIKER_PIECES.flatMap(piece =>
  piece.geometry.vertices.map(vertex => add(piece.position, vertex)));
const ASSEMBLY_PARTS = PROTOCOL_STRIKER_PIECES.map(piece => ({
  offset: piece.position,
  x: Math.max(...piece.geometry.vertices.map(vertex => Math.abs(vertex.x))),
  y: Math.max(...piece.geometry.vertices.map(vertex => Math.abs(vertex.y))),
  z: Math.max(...piece.geometry.vertices.map(vertex => Math.abs(vertex.z))),
}));
const ASSEMBLY_MIN_Y = Math.min(...ASSEMBLY_VERTICES.map(vertex => vertex.y));
const ASSEMBLY_MAX_Y = Math.max(...ASSEMBLY_VERTICES.map(vertex => vertex.y));

/** A single Rapier kinematic obstacle. Only its solver contacts create impacts. */
export class PhysicsStriker {
  private readonly body: RigidBody;
  private readonly collider: Collider;
  private readonly clearanceShape = new RAPIER.Cuboid(
    STRIKER_HEAD.x + 0.012, STRIKER_HEAD.y + 0.012, STRIKER_HEAD.z + 0.012,
  );
  private readonly assemblyClearance = ASSEMBLY_PARTS.map(part => ({
    offset: part.offset,
    shape: new RAPIER.Cuboid(part.x + 0.012, part.y + 0.012, part.z + 0.012),
  }));
  private readonly assemblyPrediction = ASSEMBLY_PARTS.map(part => ({
    offset: part.offset,
    shape: new RAPIER.Cuboid(part.x + 0.09, part.y + 0.09, part.z + 0.09),
  }));
  private anatomyHandles = new Set<number>();
  private phaseValue: Phase = "idle";
  private phaseTime = 0;
  private position: Vec3 = HOME;
  private rotation: Quat = IDENTITY;
  private start: Vec3 = this.position;
  private direction: Vec3 = { x: 0, y: 0, z: -1 };
  private target: Vec3 = { x: 0, y: 1.2, z: 0 };
  private travel = 0;
  private travelLimit = MAX_TRAVEL_M;
  private positionOrigin: Vec3 = this.position;
  private retractOrigin: Vec3 = this.position;
  private retractFromObstruction = false;
  private impactIdValue = 0;
  private impactedThisStroke = false;
  private readonly roomBodies: RigidBody[] = [];
  readonly roomColliders: Collider[] = [];

  constructor(private readonly world: World) {
    const wallThickness = 0.16;
    const { width, depth, height } = PROTOCOL_ROOM;
    for (const [x, z, hx, hz] of [
      [-width / 2 - wallThickness, 0, wallThickness, depth / 2 + wallThickness],
      [width / 2 + wallThickness, 0, wallThickness, depth / 2 + wallThickness],
      [0, -depth / 2 - wallThickness, width / 2 + wallThickness, wallThickness],
      [0, depth / 2 + wallThickness, width / 2 + wallThickness, wallThickness],
    ]) {
      const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(x, height / 2, z));
      const collider = world.createCollider(RAPIER.ColliderDesc.cuboid(hx, height / 2, hz)
        .setFriction(1.1).setRestitution(0.03)
        .setCollisionGroups((0x0002 << 16) | 0x0001), body);
      this.roomBodies.push(body);
      this.roomColliders.push(collider);
    }
    const ceiling = world.createRigidBody(RAPIER.RigidBodyDesc.fixed()
      .setTranslation(0, height + wallThickness, 0));
    const ceilingCollider = world.createCollider(RAPIER.ColliderDesc.cuboid(
      width / 2 + wallThickness, wallThickness, depth / 2 + wallThickness,
    ).setFriction(1.1).setRestitution(0.03)
      .setCollisionGroups((0x0002 << 16) | 0x0001), ceiling);
    this.roomBodies.push(ceiling);
    this.roomColliders.push(ceilingCollider);
    this.body = world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased()
      .setTranslation(this.position.x, this.position.y, this.position.z));
    this.collider = world.createCollider(RAPIER.ColliderDesc.cuboid(
      STRIKER_HEAD.x, STRIKER_HEAD.y, STRIKER_HEAD.z,
    ).setFriction(0.7).setRestitution(0.02)
      .setCollisionGroups((0x0002 << 16) | 0x0001)
      .setEnabled(false), this.body);
  }

  get available(): boolean { return this.phaseValue === "idle"; }
  get phase(): Phase { return this.phaseValue; }
  get impactId(): number { return this.impactIdValue; }
  get strikeDirection(): Vec3 { return this.direction; }

  request(torso: Vec3, characterColliders?: ReadonlyMap<SegmentId, Collider>): boolean {
    if (!this.available) return false;
    if (![torso.x, torso.y, torso.z].every(Number.isFinite)) return false;
    // Keep the entire rotated head inside the inner wall faces, including a
    // missed stroke. The maximum half extent is a conservative yaw bound.
    const safeX = PROTOCOL_ROOM.width / 2 - ROOM_MARGIN_M;
    const safeZ = PROTOCOL_ROOM.depth / 2 - ROOM_MARGIN_M;
    if (Math.abs(torso.x) >= safeX || Math.abs(torso.z) >= safeZ) return false;
    // Near a wall, shorten the standoff and approach obliquely. A candidate
    // must still point inward and start clear of the torso; there is no
    // outside-wall fallback that could fake a legal repositioning.
    const inward = normalize({ x: -torso.x, y: 0, z: -torso.z }, { x: 0, y: 0, z: -1 });
    const candidates: { direction: Vec3; standoff: number; score: number }[] = [];
    for (let i = 0; i < 2048; i++) {
      const angle = i * Math.PI * 2 / 2048;
      const direction = { x: Math.sin(angle), y: 0, z: Math.cos(angle) };
      const score = direction.x * inward.x + direction.z * inward.z;
      if (score <= 0.001) continue;
      const rotation = quatFromTo({ x: 0, y: 0, z: 1 }, direction);
      if (!this.assemblyInside(this.position, rotation)) continue;
      let low = MIN_STANDOFF_M, high = STANDOFF_M;
      for (const vertex of ASSEMBLY_VERTICES) {
        const offset = rotate(rotation, vertex);
        for (const [value, component, bound] of [
          [torso.x + offset.x, direction.x, safeX],
          [torso.z + offset.z, direction.z, safeZ],
        ]) {
          if (Math.abs(component) < 1e-9) {
            if (Math.abs(value) > bound) low = Infinity;
          } else {
            const first = (value - bound) / component;
            const second = (value + bound) / component;
            low = Math.max(low, Math.min(first, second));
            high = Math.min(high, Math.max(first, second));
          }
        }
        if (low > high) break;
      }
      if (low > high) continue;
      const standoff = high;
      candidates.push({ direction, standoff, score });
    }
    const anatomy = new Set([...(characterColliders?.values() ?? [])].map(collider => collider.handle));
    candidates.sort((a, b) => b.score - a.score || b.standoff - a.standoff);
    const startFor = ({ direction, standoff }: typeof candidates[number]): Vec3 => {
      const raw = sub(torso, scale(direction, standoff));
      return { x: raw.x, y: clamp(raw.y, -ASSEMBLY_MIN_Y + ROOM_MARGIN_M,
        PROTOCOL_ROOM.height - ASSEMBLY_MAX_Y - ROOM_MARGIN_M), z: raw.z };
    };
    const selected = candidates.find(candidate => {
      if (anatomy.size === 0) return true;
      const start = startFor(candidate);
      const rotation = quatFromTo({ x: 0, y: 0, z: 1 }, candidate.direction);
      const parked = { x: start.x, y: STRIKER_STOW_HEIGHT, z: start.z };
      if (!this.assemblyInside(start, rotation)
        || !this.assemblyInside(parked, rotation)) return false;
      return this.assemblyPathClear(this.position, parked, rotation, anatomy)
        && this.assemblyPathClear(parked, start, rotation, anatomy)
        && (!characterColliders || this.predictedPositioningPathClear(
          this.position, parked, start, rotation, characterColliders));
    });
    if (!selected) return false;
    this.anatomyHandles = anatomy;
    this.target = { ...torso };
    this.direction = selected.direction;
    this.rotation = quatFromTo({ x: 0, y: 0, z: 1 }, this.direction);
    this.start = startFor(selected);
    this.travelLimit = MAX_TRAVEL_M;
    for (const vertex of ASSEMBLY_VERTICES) {
      const offset = rotate(this.rotation, vertex);
      for (const [value, component, bound] of [
        [this.start.x + offset.x, this.direction.x, safeX],
        [this.start.z + offset.z, this.direction.z, safeZ],
      ]) {
        if (component > 1e-9) this.travelLimit = Math.min(this.travelLimit, (bound - value) / component);
        else if (component < -1e-9) this.travelLimit = Math.min(this.travelLimit, (bound + value) / -component);
      }
    }
    this.positionOrigin = { ...this.position };
    this.phaseValue = "positioning";
    this.phaseTime = 0;
    this.travel = 0;
    this.retractFromObstruction = false;
    this.impactedThisStroke = false;
    this.collider.setEnabled(false);
    return true;
  }

  /** Advances before the same fixed Rapier integration as the character. */
  beforeStep(dt: number): void {
    if (this.phaseValue === "idle") return;
    this.phaseTime += dt;
    if (this.phaseValue === "positioning") {
      // The head moves over the subject while stowed, then lowers at standoff.
      // It is re-enabled only after reaching the clear locked strike start.
      const horizontal = clamp(this.phaseTime / POSITION_OVERHEAD_S, 0, 1);
      const descend = clamp((this.phaseTime - POSITION_OVERHEAD_S) / POSITION_DESCEND_S, 0, 1);
      const parked = { x: this.start.x, y: STRIKER_STOW_HEIGHT, z: this.start.z };
      const nextPosition = {
        x: this.positionOrigin.x + (parked.x - this.positionOrigin.x) * horizontal,
        y: STRIKER_STOW_HEIGHT + (this.start.y - STRIKER_STOW_HEIGHT) * descend,
        z: this.positionOrigin.z + (parked.z - this.positionOrigin.z) * horizontal,
      };
      if (!this.assemblyPathClear(this.position, nextPosition, this.rotation, this.anatomyHandles)) {
        this.retractOrigin = { ...this.position };
        this.retractFromObstruction = true;
        this.phaseValue = "retracting";
        this.phaseTime = 0;
        this.collider.setEnabled(false);
      } else {
        this.position = nextPosition;
      }
      if (this.phaseValue === "positioning"
        && this.phaseTime >= POSITION_OVERHEAD_S + POSITION_DESCEND_S) {
        this.position = this.start;
        // Avoid turning the collision-free lowering movement into an
        // artificial kinematic impact on the first enabled solver step.
        this.body.setTranslation(this.start, true);
        this.body.setRotation(this.rotation, true);
        this.phaseValue = "striking";
        this.phaseTime = 0;
        this.collider.setEnabled(true);
      }
    } else if (this.phaseValue === "striking") {
      this.travel = Math.min(this.travelLimit, this.travel + STRIKE_SPEED_MPS * dt);
      this.position = add(this.start, scale(this.direction, this.travel));
    } else {
      if (this.retractFromObstruction) {
        // The locked start has become obstructed. Never move toward it again:
        // withdraw vertically from the last verified position, while disabled.
        const rise = clamp(this.phaseTime / RETRACT_STOW_S, 0, 1);
        const home = clamp((this.phaseTime - RETRACT_STOW_S) / RETRACT_HOME_S, 0, 1);
        const next = { x: this.retractOrigin.x + (HOME.x - this.retractOrigin.x) * home,
          y: this.retractOrigin.y + (STRIKER_STOW_HEIGHT - this.retractOrigin.y) * rise,
          z: this.retractOrigin.z + (HOME.z - this.retractOrigin.z) * home };
        if (this.pathClearLeavingOverlap(this.position, next, this.rotation, this.anatomyHandles)) {
          this.position = next;
        } else this.phaseTime = Math.max(0, this.phaseTime - dt);
      } else {
        const back = clamp(this.phaseTime / RETRACT_TO_START_S, 0, 1);
        const rise = clamp((this.phaseTime - RETRACT_TO_START_S) / RETRACT_STOW_S, 0, 1);
        const home = clamp((this.phaseTime - RETRACT_TO_START_S - RETRACT_STOW_S) / RETRACT_HOME_S, 0, 1);
        const moved = add(this.retractOrigin, scale(sub(this.start, this.retractOrigin), back));
        const stowed = { x: moved.x, y: moved.y + (STRIKER_STOW_HEIGHT - moved.y) * rise, z: moved.z };
        const next = { x: stowed.x + (HOME.x - stowed.x) * home,
          y: stowed.y, z: stowed.z + (HOME.z - stowed.z) * home };
        if (back < 1 || this.pathClear(this.position, next, this.rotation, this.anatomyHandles)) {
          this.position = next;
        } else {
          // The locked start became occupied after contact. Rise from the
          // current position instead of waiting against the obstruction.
          this.retractOrigin = { ...this.position };
          this.retractFromObstruction = true;
          this.phaseTime = 0;
          this.collider.setEnabled(false);
        }
        if (back >= 1) this.collider.setEnabled(false);
      }
      if (this.phaseTime >= (this.retractFromObstruction
        ? RETRACT_STOW_S + RETRACT_HOME_S
        : RETRACT_TO_START_S + RETRACT_STOW_S + RETRACT_HOME_S)) {
        this.phaseValue = "idle";
        this.phaseTime = 0;
      }
    }
    this.body.setNextKinematicRotation(this.rotation);
    this.body.setNextKinematicTranslation(this.position);
  }

  private pathClear(from: Vec3, to: Vec3, rotation: Quat, anatomy: ReadonlySet<number>): boolean {
    if (!anatomy.size) return true;
    const length = Math.hypot(to.x - from.x, to.y - from.y, to.z - from.z);
    const samples = Math.max(1, Math.ceil(length / 0.06));
    for (let index = 0; index <= samples; index++) {
      const blend = index / samples;
      const point = { x: from.x + (to.x - from.x) * blend,
        y: from.y + (to.y - from.y) * blend,
        z: from.z + (to.z - from.z) * blend };
      let blocked = false;
      this.world.intersectionsWithShape(point, rotation, this.clearanceShape,
        collider => { if (anatomy.has(collider.handle)) blocked = true; return !blocked; });
      if (blocked) return false;
    }
    return true;
  }

  /** Check every drawn machine volume during collision-free repositioning. */
  private assemblyPathClear(from: Vec3, to: Vec3, rotation: Quat,
    anatomy: ReadonlySet<number>): boolean {
    if (!anatomy.size) return true;
    const parts = this.assemblyClearance.map(part => ({
      offset: rotate(rotation, part.offset), shape: part.shape,
    }));
    const length = Math.hypot(to.x - from.x, to.y - from.y, to.z - from.z);
    const samples = Math.max(1, Math.ceil(length / 0.06));
    for (let index = 0; index <= samples; index++) {
      const blend = index / samples;
      const origin = { x: from.x + (to.x - from.x) * blend,
        y: from.y + (to.y - from.y) * blend,
        z: from.z + (to.z - from.z) * blend };
      for (const part of parts) {
        let blocked = false;
        this.world.intersectionsWithShape(add(origin, part.offset), rotation, part.shape,
          collider => { if (anatomy.has(collider.handle)) blocked = true; return !blocked; });
        if (blocked) return false;
      }
    }
    return true;
  }

  /** A withdrawal may begin in contact; each sample must reduce that overlap. */
  private pathClearLeavingOverlap(from: Vec3, to: Vec3, rotation: Quat,
    anatomy: ReadonlySet<number>): boolean {
    if (this.pathClear(from, to, rotation, anatomy)) return true;
    const existing = new Map<number, number>();
    this.world.intersectionsWithShape(from, rotation, this.clearanceShape, collider => {
      if (anatomy.has(collider.handle)) {
        existing.set(collider.handle, this.clearanceShape.contactShape(from, rotation,
          collider.shape, collider.translation(), collider.rotation(), 0)?.distance ?? 0);
      }
      return true;
    });
    if (!existing.size) return false;
    const length = Math.hypot(to.x - from.x, to.y - from.y, to.z - from.z);
    const samples = Math.max(1, Math.ceil(length / 0.06));
    for (let index = 1; index <= samples; index++) {
      const blend = index / samples;
      const point = { x: from.x + (to.x - from.x) * blend,
        y: from.y + (to.y - from.y) * blend,
        z: from.z + (to.z - from.z) * blend };
      const stillOverlapping = new Set<number>();
      let blocked = false;
      this.world.intersectionsWithShape(point, rotation, this.clearanceShape, collider => {
        if (!anatomy.has(collider.handle)) return true;
        stillOverlapping.add(collider.handle);
        const previous = existing.get(collider.handle);
        const distance = this.clearanceShape.contactShape(point, rotation,
          collider.shape, collider.translation(), collider.rotation(), 0)?.distance ?? 0;
        if (previous === undefined || distance < previous - 0.001) blocked = true;
        else existing.set(collider.handle, distance);
        return !blocked;
      });
      if (blocked) return false;
      for (const handle of existing.keys()) if (!stillOverlapping.has(handle)) existing.delete(handle);
    }
    return true;
  }

  /** Reject a candidate when a moving limb is expected to meet its descent. */
  private predictedPositioningPathClear(
    origin: Vec3, parked: Vec3, start: Vec3, rotation: Quat,
    characterColliders: ReadonlyMap<SegmentId, Collider>,
  ): boolean {
    const duration = POSITION_OVERHEAD_S + POSITION_DESCEND_S;
    const samples = Math.ceil(duration * 120);
    const parts = this.assemblyPrediction.map(part => ({
      offset: rotate(rotation, part.offset), shape: part.shape,
    }));
    for (let index = 0; index <= samples; index++) {
      const time = duration * index / samples;
      const horizontal = clamp(time / POSITION_OVERHEAD_S, 0, 1);
      const descend = clamp((time - POSITION_OVERHEAD_S) / POSITION_DESCEND_S, 0, 1);
      const machine = {
        x: origin.x + (parked.x - origin.x) * horizontal,
        y: STRIKER_STOW_HEIGHT + (start.y - STRIKER_STOW_HEIGHT) * descend,
        z: origin.z + (parked.z - origin.z) * horizontal,
      };
      for (const collider of characterColliders.values()) {
        if (!collider.isEnabled()) continue;
        const body = collider.parent();
        const velocity = body?.linvel() ?? { x: 0, y: 0, z: 0 };
        const spin = body?.angvel() ?? { x: 0, y: 0, z: 0 };
        const position = add(collider.translation(), scale(velocity, time));
        const spinSpeed = Math.hypot(spin.x, spin.y, spin.z);
        const predictedRotation = spinSpeed > 1e-8
          ? quatMultiply(quatFromAxisAngle(scale(spin, 1 / spinSpeed), spinSpeed * time), collider.rotation())
          : collider.rotation();
        for (const part of parts) {
          if (part.shape.intersectsShape(add(machine, part.offset), rotation,
            collider.shape, position, predictedRotation)) return false;
        }
      }
    }
    return true;
  }

  private assemblyInside(position: Vec3, rotation: Quat): boolean {
    const boundX = PROTOCOL_ROOM.width / 2 - ROOM_MARGIN_M;
    const boundZ = PROTOCOL_ROOM.depth / 2 - ROOM_MARGIN_M;
    return ASSEMBLY_VERTICES.every(vertex => {
      const point = add(position, rotate(rotation, vertex));
      return Math.abs(point.x) <= boundX + 1e-8
        && Math.abs(point.z) <= boundZ + 1e-8
        && point.y >= ROOM_MARGIN_M - 1e-8
        && point.y <= PROTOCOL_ROOM.height - ROOM_MARGIN_M + 1e-8;
    });
  }

  /** Returns true only for a newly measured solver impulse on an anatomical body. */
  afterStep(characterColliders: ReadonlyMap<SegmentId, Collider>): boolean {
    if (this.phaseValue !== "striking") return false;
    let impulse = 0;
    for (const collider of characterColliders.values()) {
      this.world.contactPair(this.collider, collider, manifold => {
        for (let i = 0; i < manifold.numContacts(); i++) impulse += Math.abs(manifold.contactImpulse(i));
      });
    }
    const impact = !this.impactedThisStroke && impulse > 0.25;
    if (impact) {
      this.impactedThisStroke = true;
      this.impactIdValue += 1;
    }
    if (impact || this.travel >= this.travelLimit) {
      this.retractOrigin = { ...this.position };
      this.retractFromObstruction = false;
      this.phaseValue = "retracting";
      this.phaseTime = 0;
    }
    return impact;
  }

  snapshot(): NonNullable<PoseSnapshot["striker"]> {
    const translation = this.body.translation();
    const rotation = this.body.rotation();
    return { phase: this.phaseValue,
      position: { x: translation.x, y: translation.y, z: translation.z },
      rotation: { x: rotation.x, y: rotation.y, z: rotation.z, w: rotation.w },
      impactId: this.impactIdValue,
      available: this.available };
  }

  reset(): void {
    this.phaseValue = "idle";
    this.phaseTime = 0;
    this.position = HOME;
    this.rotation = IDENTITY;
    this.start = this.position;
    this.positionOrigin = this.position;
    this.travelLimit = MAX_TRAVEL_M;
    this.retractFromObstruction = false;
    this.collider.setEnabled(false);
    this.body.setTranslation(this.position, true);
    this.body.setRotation(this.rotation, true);
    this.body.setNextKinematicTranslation(this.position);
    this.body.setNextKinematicRotation(this.rotation);
    this.impactIdValue = 0;
    this.impactedThisStroke = false;
    this.anatomyHandles.clear();
  }

  dispose(): void {
    this.world.removeRigidBody(this.body);
    for (const body of this.roomBodies) this.world.removeRigidBody(body);
  }
}
