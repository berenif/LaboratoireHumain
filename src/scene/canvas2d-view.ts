import { V3 } from "../core/math";
import { ARENA, courseTransform, getPlaygroundCourse, PLAYGROUND_STATIONS } from "../core/playground";
import {
  PASSIVE_COLOR,
  PRIMARY_SEGMENT_BY_REGION,
  REGION_COLORS,
  SEGMENTS,
  SEGMENT_BY_ID,
} from "../core/humanoid";
import type { PoseSnapshot, PoseView, SegmentDefinition, SegmentId, SegmentPose, Vec3 } from "../core/types";
import { SharedCameraProjection } from "./camera";
import type { SceneViewOptions } from "./options";
import { interpolatePoseSnapshot, rotateVector, transformLocalPoint } from "./pose";
import { PROTOCOL_STRIKER_PIECES, protocolRoomPieces, type ProtocolVisualPiece } from "./protocol-visuals";

interface ProjectedPoint {
  x: number;
  y: number;
  depth: number;
}

interface DrawableSegment {
  definition: SegmentDefinition;
  pose: SegmentPose;
  depth: number;
}

function convexHull(points: ReadonlyArray<ProjectedPoint>): ProjectedPoint[] {
  if (points.length <= 3) return [...points];
  const sorted = [...points].sort((a, b) => a.x - b.x || a.y - b.y);
  const cross = (origin: ProjectedPoint, a: ProjectedPoint, b: ProjectedPoint) =>
    (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x);
  const lower: ProjectedPoint[] = [];
  for (const point of sorted) {
    while (lower.length >= 2 && cross(lower.at(-2)!, lower.at(-1)!, point) <= 0) lower.pop();
    lower.push(point);
  }
  const upper: ProjectedPoint[] = [];
  for (let index = sorted.length - 1; index >= 0; index -= 1) {
    const point = sorted[index];
    while (upper.length >= 2 && cross(upper.at(-2)!, upper.at(-1)!, point) <= 0) upper.pop();
    upper.push(point);
  }
  lower.pop();
  upper.pop();
  return lower.concat(upper);
}

function average(a: Vec3, b: Vec3): Vec3 {
  return V3.scale(V3.add(a, b), 0.5);
}

export class Canvas2DView implements PoseView {
  readonly mode = "canvas2d" as const;
  readonly canvas: HTMLCanvasElement;

  private readonly ownsCanvas: boolean;
  private readonly projection: SharedCameraProjection;
  private context: CanvasRenderingContext2D | null = null;
  private container: HTMLElement | null = null;
  private snapshot: PoseSnapshot | null = null;
  private cssWidth = 1;
  private cssHeight = 1;
  private pixelRatio = 1;
  private disposed = false;

  constructor(options: SceneViewOptions = {}) {
    if (typeof document === "undefined" && !options.canvas) {
      throw new Error("Canvas2DView requires a browser canvas.");
    }
    this.ownsCanvas = !options.canvas;
    this.canvas = options.canvas ?? document.createElement("canvas");
    this.projection = options.camera ?? new SharedCameraProjection();
    this.canvas.className = options.className ?? "embodied-scene embodied-scene--canvas2d";
    this.canvas.dataset.renderer = "canvas2d";
    this.canvas.style.display = "block";
    this.canvas.style.width = "100%";
    this.canvas.style.height = "100%";
    this.canvas.style.touchAction = "none";
  }

  mount(container: HTMLElement): void {
    this.assertUsable();
    this.container = container;
    if (this.canvas.parentElement !== container) container.append(this.canvas);
    this.context = this.canvas.getContext("2d", { alpha: false });
    if (!this.context) throw new Error("Canvas2D context creation failed.");
    const rect = container.getBoundingClientRect();
    this.resize(rect.width || container.clientWidth || 1, rect.height || container.clientHeight || 1,
      typeof window === "undefined" ? 1 : window.devicePixelRatio || 1);
  }

  setSnapshot(previous: PoseSnapshot, current: PoseSnapshot, alpha: number): void {
    this.snapshot = interpolatePoseSnapshot(previous, current, alpha);
  }

  render(): void {
    const context = this.context;
    if (!context || this.disposed) return;
    context.save();
    context.setTransform(this.pixelRatio, 0, 0, this.pixelRatio, 0, 0);
    this.drawBackdrop(context, Boolean(this.snapshot?.room));
    if (this.snapshot?.room) this.drawProtocolRoom(context, this.snapshot.room);
    else this.drawFloor(context);
    if (this.snapshot) {
      if (!this.snapshot.room) this.drawSupport(context, this.snapshot);
      this.drawJointLinks(context, this.snapshot);
      this.drawSegments(context, this.snapshot);
      this.drawJointMarkers(context, this.snapshot);
      if (!this.snapshot.room) this.drawSelection(context, this.snapshot);
    }
    context.restore();
  }

  resize(width: number, height: number, devicePixelRatio: number): void {
    this.assertUsable();
    this.cssWidth = Math.max(1, Math.round(width));
    this.cssHeight = Math.max(1, Math.round(height));
    this.pixelRatio = Math.max(1, Math.min(3, Number.isFinite(devicePixelRatio) ? devicePixelRatio : 1));
    const bufferWidth = Math.max(1, Math.round(this.cssWidth * this.pixelRatio));
    const bufferHeight = Math.max(1, Math.round(this.cssHeight * this.pixelRatio));
    if (this.canvas.width !== bufferWidth) this.canvas.width = bufferWidth;
    if (this.canvas.height !== bufferHeight) this.canvas.height = bufferHeight;
    this.canvas.style.width = `${this.cssWidth}px`;
    this.canvas.style.height = `${this.cssHeight}px`;
    this.projection.setViewport(this.cssWidth, this.cssHeight);
  }

  getProjection(): SharedCameraProjection {
    return this.projection;
  }

  getElement(): HTMLCanvasElement {
    return this.canvas;
  }

  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    this.snapshot = null;
    this.context?.clearRect(0, 0, this.canvas.width, this.canvas.height);
    this.context = null;
    if (this.ownsCanvas) this.canvas.remove();
    this.container = null;
  }

  private drawBackdrop(context: CanvasRenderingContext2D, protocol: boolean): void {
    const gradient = context.createLinearGradient(0, 0, 0, this.cssHeight);
    gradient.addColorStop(0, protocol ? "#aebcb5" : "#101a2b");
    gradient.addColorStop(0.66, protocol ? "#d8ddd2" : "#17233a");
    gradient.addColorStop(1, protocol ? "#9fae9f" : "#0a111d");
    context.fillStyle = gradient;
    context.fillRect(0, 0, this.cssWidth, this.cssHeight);
  }

  private drawProtocolRoom(context: CanvasRenderingContext2D, room: NonNullable<PoseSnapshot["room"]>): void {
    const identity = { x: 0, y: 0, z: 0, w: 1 };
    const pieces = protocolRoomPieces(room);
    for (const piece of pieces) this.drawProtocolPiece(context, piece, { x: 0, y: 0, z: 0 }, identity);
    context.lineWidth = 1;
    const halfWidth = room.width / 2;
    const halfDepth = room.depth / 2;
    for (const inset of [0.3, 0.34]) {
      const color = inset === 0.3 ? "rgba(73,80,69,0.45)" : "rgba(241,181,27,0.6)";
      this.strokeWorldLine(context, { x: -halfWidth + inset, y: 0.005, z: -halfDepth + inset }, { x: halfWidth - inset, y: 0.005, z: -halfDepth + inset }, color);
      this.strokeWorldLine(context, { x: -halfWidth + inset, y: 0.005, z: halfDepth - inset }, { x: halfWidth - inset, y: 0.005, z: halfDepth - inset }, color);
    }
  }

  private drawProtocolPiece(context: CanvasRenderingContext2D, piece: ProtocolVisualPiece, parentPosition: Vec3, parentRotation: PoseSnapshot["rootRotation"]): void {
    const center = V3.add(parentPosition, rotateVector(parentRotation, piece.position));
    const vertices = piece.geometry.vertices.map(vertex => V3.add(center, rotateVector(parentRotation, vertex)));
    const projected = vertices.map(vertex => this.projection.project(vertex));
    const faces = piece.geometry.triangles.flatMap(([a, b, c]) => {
      const points = [projected[a], projected[b], projected[c]];
      if (points.some(point => point.depth <= 0)) return [];
      const normal = V3.normalize(V3.cross(V3.sub(vertices[b], vertices[a]), V3.sub(vertices[c], vertices[a])));
      const shade = Math.max(0.63, Math.min(1.11, 0.88 + normal.y * 0.13 - normal.x * 0.09 + normal.z * 0.05));
      return [{ points, depth: points.reduce((sum, point) => sum + point.depth, 0) / 3, shade }];
    }).sort((a, b) => b.depth - a.depth);
    const opacity = piece.opacity ?? 1;
    context.save();
    context.globalAlpha = opacity;
    for (const face of faces) {
      const rgb = [1, 3, 5].map(start => Math.min(255, Math.round(parseInt(piece.color.slice(start, start + 2), 16) * face.shade)));
      context.beginPath();
      context.moveTo(face.points[0].x, face.points[0].y);
      context.lineTo(face.points[1].x, face.points[1].y);
      context.lineTo(face.points[2].x, face.points[2].y);
      context.closePath();
      context.fillStyle = `rgb(${rgb.join(",")})`;
      context.fill();
    }
    if (piece.id === "head" || piece.id === "collar") {
      const hull = convexHull(projected.filter(point => point.depth > 0));
      if (hull.length > 2) {
        context.beginPath();
        context.moveTo(hull[0].x, hull[0].y);
        for (let index = 1; index < hull.length; index++) context.lineTo(hull[index].x, hull[index].y);
        context.closePath();
        context.strokeStyle = "#28322b";
        context.lineWidth = 2;
        context.stroke();
      }
    }
    context.restore();
  }

  private drawFloor(context: CanvasRenderingContext2D): void {
    const corners = [
      { x: -ARENA.width / 2, y: 0, z: -ARENA.depth / 2 },
      { x: ARENA.width / 2, y: 0, z: -ARENA.depth / 2 },
      { x: ARENA.width / 2, y: 0, z: ARENA.depth / 2 },
      { x: -ARENA.width / 2, y: 0, z: ARENA.depth / 2 },
    ].map((point) => this.projection.project(point));
    if (corners.every((point) => point.depth > 0)) {
      context.beginPath();
      context.moveTo(corners[0].x, corners[0].y);
      for (let index = 1; index < corners.length; index += 1) context.lineTo(corners[index].x, corners[index].y);
      context.closePath();
      context.fillStyle = "rgba(24, 38, 58, 0.72)";
      context.fill();
    }

    context.lineWidth = 1;
    for (let grid = -20; grid <= 20; grid += 1) {
      const x = grid * 0.5;
      this.strokeWorldLine(context, { x, y: 0.002, z: -8.5 }, { x, y: 0.002, z: 8.5 },
        grid === 0 ? "rgba(98, 213, 255, 0.34)" : "rgba(135, 157, 188, 0.15)");
    }
    for (let grid = -17; grid <= 17; grid += 1) {
      const z = grid * 0.5;
      this.strokeWorldLine(context, { x: -10, y: 0.002, z }, { x: 10, y: 0.002, z },
        grid === 0 ? "rgba(124, 131, 255, 0.32)" : "rgba(135, 157, 188, 0.15)");
    }

    context.lineWidth = 2;
    const bounds: ReadonlyArray<readonly [Vec3, Vec3]> = [
      [{ x: -10, y: 0.008, z: -8.5 }, { x: 10, y: 0.008, z: -8.5 }],
      [{ x: 10, y: 0.008, z: -8.5 }, { x: 10, y: 0.008, z: 8.5 }],
      [{ x: 10, y: 0.008, z: 8.5 }, { x: -10, y: 0.008, z: 8.5 }],
      [{ x: -10, y: 0.008, z: 8.5 }, { x: -10, y: 0.008, z: -8.5 }],
    ];
    for (const [from, to] of bounds) this.strokeWorldLine(context, from, to, "rgba(112, 221, 255, 0.4)");
  }

  private drawSupport(context: CanvasRenderingContext2D, snapshot: PoseSnapshot): void {
    const poses = new Map(snapshot.segments.map((pose) => [pose.id, pose]));
    for (const foot of ["leftFoot", "rightFoot"] as const) {
      const pose = poses.get(foot);
      if (!pose) continue;
      const point = this.projection.project({ x: pose.position.x, y: pose.position.y - 0.025, z: pose.position.z });
      if (point.depth <= 0) continue;
      const planted = snapshot.support.planted.includes(foot);
      const swinging = snapshot.support.swingFoot === foot;
      const radius = this.projection.worldRadiusToPixels({ x: pose.position.x, y: 0.012, z: pose.position.z }, 0.16);
      context.beginPath();
      context.ellipse(point.x, point.y, radius, Math.max(2, radius * 0.32), 0, 0, Math.PI * 2);
      context.fillStyle = swinging
        ? "rgba(255, 209, 102, 0.2)"
        : planted ? "rgba(128, 231, 168, 0.2)" : "rgba(184, 197, 217, 0.08)";
      context.strokeStyle = swinging ? "#ffd166" : planted ? "#80e7a8" : "rgba(184, 197, 217, 0.35)";
      context.lineWidth = swinging ? 2.5 : 1.5;
      context.fill();
      context.stroke();
    }
  }

  private drawJointLinks(context: CanvasRenderingContext2D, snapshot: PoseSnapshot): void {
    const poses = new Map(snapshot.segments.map((pose) => [pose.id, pose]));
    context.lineCap = "round";
    for (const definition of SEGMENTS) {
      if (!definition.parent) continue;
      const child = poses.get(definition.id);
      const parent = poses.get(definition.parent);
      if (!child || !parent) continue;
      const parentDefinition = SEGMENT_BY_ID.get(definition.parent);
      const parentPoint = definition.jointAnchorParent
        ? transformLocalPoint(parent, definition.jointAnchorParent)
        : parent.position;
      const childPoint = definition.jointAnchorChild
        ? transformLocalPoint(child, definition.jointAnchorChild)
        : child.position;
      context.lineWidth = parentDefinition?.role === "pelvis" || parentDefinition?.role === "ribcage" ? 5 : 4;
      this.strokeWorldLine(context, parentPoint, childPoint, snapshot.room ? "rgba(48,58,55,0.75)" : "rgba(175, 198, 225, 0.58)");
    }
  }

  private drawSegments(context: CanvasRenderingContext2D, snapshot: PoseSnapshot): void {
    const drawables: { depth: number; draw: () => void }[] = [];
    for (const pose of snapshot.segments) {
      const definition = SEGMENT_BY_ID.get(pose.id);
      if (!definition) continue;
      const projected = this.projection.project(pose.position);
      if (projected.depth > 0) drawables.push({ depth: projected.depth,
        draw: () => this.drawSegment(context, { definition, pose, depth: projected.depth }, snapshot) });
    }
    if (snapshot.striker) {
      for (const piece of PROTOCOL_STRIKER_PIECES) {
        const center = V3.add(snapshot.striker.position, rotateVector(snapshot.striker.rotation, piece.position));
        const projected = this.projection.project(center);
        if (projected.depth > 0) drawables.push({ depth: projected.depth,
          draw: () => this.drawProtocolPiece(context, piece, snapshot.striker!.position, snapshot.striker!.rotation) });
      }
    }
    if (snapshot.playground) {
      for (const piece of getPlaygroundCourse(snapshot.playground.difficulty)) {
        const transform = courseTransform(piece, snapshot.simulationTime);
        const vertices = piece.geometry.vertices.map(vertex => V3.add(transform.position, rotateVector(transform.rotation, vertex)));
        const projected = vertices.map(vertex => this.projection.project(vertex));
        for (const [a, b, c] of piece.geometry.triangles) {
          const points = [projected[a], projected[b], projected[c]];
          if (points.some(point => point.depth <= 0)) continue;
          const normal = V3.normalize(V3.cross(V3.sub(vertices[b], vertices[a]), V3.sub(vertices[c], vertices[a])));
          const shade = Math.max(0.38, Math.min(1.15, 0.7 + normal.y * 0.3 - normal.x * 0.12 + normal.z * 0.1));
          const color = [1, 3, 5].map(start => Math.min(255, Math.round(parseInt(piece.color.slice(start, start + 2), 16) * shade)));
          drawables.push({ depth: points.reduce((sum, point) => sum + point.depth, 0) / 3, draw: () => {
            context.beginPath();
            context.moveTo(points[0].x, points[0].y);
            context.lineTo(points[1].x, points[1].y);
            context.lineTo(points[2].x, points[2].y);
            context.closePath();
            context.fillStyle = `rgb(${color.join(",")})`;
            context.fill();
            context.strokeStyle = "rgba(10,20,30,0.16)";
            context.lineWidth = 0.7;
            context.stroke();
          } });
        }
      }
    }
    drawables.sort((a, b) => b.depth - a.depth);
    for (const drawable of drawables) drawable.draw();
    if (snapshot.playground) for (const [index, station] of PLAYGROUND_STATIONS.entries()) {
      const point = this.projection.project({ ...station.position, y: 0.025,
        z: station.id === "flat" ? 4.55 : station.id === "wobble" ? -7.3
          : station.id === "stones" || station.id === "hurdles" ? 6.7 : 0.28 });
      if (!point.visible) continue;
      context.font = `600 ${Math.max(9, Math.min(15, 180 / point.depth))}px system-ui`;
      context.fillStyle = station.color;
      context.textAlign = "center";
      context.fillText(`${String(index + 1).padStart(2, "0")} / ${station.name.toUpperCase()}`, point.x, point.y);
    }
  }

  private drawSegment(
    context: CanvasRenderingContext2D,
    drawable: DrawableSegment,
    snapshot: PoseSnapshot,
  ): void {
    const { definition, pose } = drawable;
    const color = snapshot.room
      ? ["lumbar", "neck", "shoulder-girdle", "forearm-twist", "ankle"].includes(definition.role) ? "#303a38" : "#f1efdf"
      : definition.region ? REGION_COLORS[definition.region] : PASSIVE_COLOR;
    const exactSelection = snapshot.diagnostics.selectedSegment;
    const selected = exactSelection
      ? definition.id === exactSelection
      : definition.region !== null && definition.region === snapshot.diagnostics.selectedRegion;
    const fill = snapshot.room ? color : selected ? color : `${color}df`;
    const outline = snapshot.room ? "rgba(41,51,45,0.78)" : selected ? "#ffffff" : "rgba(6, 12, 24, 0.82)";
    const points = definition.geometry.vertices.map((vertex) => {
      const world = V3.add(pose.position, rotateVector(pose.rotation, vertex));
      return this.projection.project(world);
    });
    const faces = definition.geometry.triangles.flatMap(([a, b, c]) => {
      const triangle = [points[a], points[b], points[c]] as const;
      if (triangle.some((point) => point.depth <= 0)) return [];
      return [{ triangle, depth: (triangle[0].depth + triangle[1].depth + triangle[2].depth) / 3 }];
    }).sort((a, b) => b.depth - a.depth);
    context.fillStyle = fill;
    for (const { triangle } of faces) {
      context.beginPath();
      context.moveTo(triangle[0].x, triangle[0].y);
      context.lineTo(triangle[1].x, triangle[1].y);
      context.lineTo(triangle[2].x, triangle[2].y);
      context.closePath();
      context.fill();
    }

    const hull = convexHull(points.filter((point) => point.depth > 0));
    if (hull.length < 3) return;
    context.beginPath();
    context.moveTo(hull[0].x, hull[0].y);
    for (let index = 1; index < hull.length; index += 1) context.lineTo(hull[index].x, hull[index].y);
    context.closePath();
    context.strokeStyle = outline;
    context.lineWidth = snapshot.room ? 1.3 : selected ? 4 : 2;
    context.stroke();
  }

  private drawJointMarkers(context: CanvasRenderingContext2D, snapshot: PoseSnapshot): void {
    const poses = new Map(snapshot.segments.map((pose) => [pose.id, pose]));
    for (const definition of SEGMENTS) {
      if (!definition.parent) continue;
      const child = poses.get(definition.id);
      const parent = poses.get(definition.parent);
      if (!child || !parent) continue;
      const parentPoint = definition.jointAnchorParent
        ? transformLocalPoint(parent, definition.jointAnchorParent)
        : parent.position;
      const childPoint = definition.jointAnchorChild
        ? transformLocalPoint(child, definition.jointAnchorChild)
        : child.position;
      const projected = this.projection.project(average(parentPoint, childPoint));
      if (!projected.visible) continue;
      const radius = Math.max(2, Math.min(5, 23 / projected.depth));
      context.beginPath();
      context.arc(projected.x, projected.y, radius, 0, Math.PI * 2);
      context.fillStyle = "rgba(10, 18, 30, 0.88)";
      context.strokeStyle = snapshot.room ? "rgba(48,58,55,0.72)" : "rgba(225, 238, 255, 0.72)";
      context.lineWidth = 1.25;
      context.fill();
      context.stroke();
    }
  }

  private drawSelection(context: CanvasRenderingContext2D, snapshot: PoseSnapshot): void {
    const region = snapshot.diagnostics.selectedRegion;
    if (!region) return;
    const segment: SegmentId | undefined = snapshot.diagnostics.selectedSegment
      ?? PRIMARY_SEGMENT_BY_REGION.get(region);
    const pose = segment ? snapshot.segments.find((candidate) => candidate.id === segment) : undefined;
    if (!pose) return;
    const point = this.projection.project(pose.position);
    if (!point.visible) return;
    const pulse = snapshot.diagnostics.activeGrab ? 1 + Math.sin(snapshot.simulationTime * 12) * 0.08 : 1;
    context.beginPath();
    context.arc(point.x, point.y, 17 * pulse, 0, Math.PI * 2);
    context.strokeStyle = snapshot.diagnostics.activeGrab ? "#ffffff" : "rgba(255,255,255,0.68)";
    context.lineWidth = snapshot.diagnostics.activeGrab ? 2.5 : 1.5;
    context.setLineDash(snapshot.diagnostics.activeGrab ? [] : [4, 4]);
    context.stroke();
    context.setLineDash([]);
    context.beginPath();
    context.arc(point.x, point.y, 2.5, 0, Math.PI * 2);
    context.fillStyle = "#ffffff";
    context.fill();
  }

  private strokeWorldLine(
    context: CanvasRenderingContext2D,
    from: Vec3,
    to: Vec3,
    strokeStyle: string,
  ): void {
    const a = this.projection.project(from);
    const b = this.projection.project(to);
    if (a.depth <= 0 || b.depth <= 0) return;
    context.beginPath();
    context.moveTo(a.x, a.y);
    context.lineTo(b.x, b.y);
    context.strokeStyle = strokeStyle;
    context.stroke();
  }

  private assertUsable(): void {
    if (this.disposed) throw new Error("Canvas2DView has been disposed.");
  }
}
