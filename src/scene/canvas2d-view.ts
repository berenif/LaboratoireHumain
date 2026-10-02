import { V3 } from "../core/math";
import { ARENA, boxGeometry, courseTransform, getPlaygroundCourse, PLAYGROUND_STATIONS, type CoursePiece } from "../core/playground";
import { PRIMARY_SEGMENT_BY_REGION, SEGMENTS, SEGMENT_BY_ID } from "../core/humanoid";
import type { PoseSnapshot, PoseView, Vec3, Quat, RenderQuality, RenderMetrics } from "../core/types";
import { SharedCameraProjection } from "./camera";
import type { SceneViewOptions } from "./options";
import { PresentationBuffer, rotateVector, transformLocalPoint } from "./pose";
import { PROTOCOL_STRIKER_PIECES, protocolRoomPieces, type ProtocolVisualPiece } from "./protocol-visuals";
import { qualitySettings } from "./quality";
import { BACKGROUND, BODY_COLORS, JOINT_COLOR, SELECTION, sceneryColor } from "./palette";

interface DrawCommand {
  depth: number;
  path: Path2D;
  fill?: string;
  stroke?: string;
  opacity: number;
  lineWidth?: number;
  text?: string;
  font?: string;
  x?: number;
  y?: number;
  bounds?: readonly [number, number, number, number];
}
interface RasterLayer { canvas: HTMLCanvasElement; x: number; y: number; width: number; height: number; end: number; depth: number }
const IDENTITY: Quat = { x: 0, y: 0, z: 0, w: 1 };
const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const farFirst = (a: DrawCommand, b: DrawCommand) => b.depth - a.depth;
type Point = ReturnType<SharedCameraProjection["project"]>;

/** Exact projected silhouette of the canonical convex surface, no substitute shape. */
function silhouette(points: Point[]): Point[] {
  points.sort((a, b) => a.x - b.x || a.y - b.y);
  const hull: Point[] = [];
  const cross = (a: Point, b: Point, c: Point) => (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  for (const point of points) {
    while (hull.length >= 2 && cross(hull[hull.length - 2], hull[hull.length - 1], point) <= 0) hull.pop();
    hull.push(point);
  }
  const lower = hull.length;
  for (let i = points.length - 2; i >= 0; i--) {
    while (hull.length > lower && cross(hull[hull.length - 2], hull[hull.length - 1], points[i]) <= 0) hull.pop();
    hull.push(points[i]);
  }
  hull.pop();
  return hull;
}

/** Cached projected scenery and a separately sorted dynamic stream are merged by depth. */
export class Canvas2DView implements PoseView {
  readonly mode = "canvas2d" as const;
  readonly canvas: HTMLCanvasElement;
  private readonly ownsCanvas: boolean;
  private readonly projection: SharedCameraProjection;
  private readonly presentation = new PresentationBuffer();
  private context: CanvasRenderingContext2D | null = null;
  private snapshot: PoseSnapshot | null = null;
  private cssWidth = 1;
  private cssHeight = 1;
  private sized = false;
  private pixelRatio = 1;
  private deviceRatio = 1;
  private quality: RenderQuality = "auto";
  private autoRatio = 1.25;
  private presentationStart = 0;
  private presentationCpuMs = 0;
  private disposed = false;
  private geometryKey = "";
  private cacheKey = "";
  private geometryBuilds = 0;
  private projectionBuilds = 0;
  private pieces: readonly ProtocolVisualPiece[] = [];
  private moving: readonly CoursePiece[] = [];
  private readonly staticCommands: DrawCommand[] = [];
  private readonly floorCommands: DrawCommand[] = [];
  private readonly dynamicCommands: DrawCommand[] = [];
  private rasterKey = "";
  private floorLayer: RasterLayer | null = null;
  private readonly staticLayers = new Map<number, RasterLayer>();

  constructor(options: SceneViewOptions = {}) {
    if (typeof document === "undefined" && !options.canvas) throw new Error("Canvas2DView requires a browser canvas.");
    this.ownsCanvas = !options.canvas;
    this.canvas = options.canvas ?? document.createElement("canvas");
    this.projection = options.camera ?? new SharedCameraProjection();
    this.canvas.className = options.className ?? "embodied-scene embodied-scene--canvas2d";
    this.canvas.dataset.renderer = "canvas2d";
    Object.assign(this.canvas.style, { display: "block", width: "100%", height: "100%", touchAction: "none" });
  }

  mount(container: HTMLElement): void {
    this.assertUsable();
    if (this.canvas.parentElement !== container) container.append(this.canvas);
    this.context = this.canvas.getContext("2d", { alpha: false });
    if (!this.context) throw new Error("Canvas2D context creation failed.");
    const rect = container.getBoundingClientRect();
    this.resize(rect.width || 1, rect.height || 1, container.ownerDocument.defaultView?.devicePixelRatio || 1);
  }

  setSnapshot(previous: PoseSnapshot, current: PoseSnapshot, alpha: number): void {
    const start = performance.now();
    this.snapshot = this.presentation.update(previous, current, alpha);
    this.presentationStart = performance.now() - start;
  }

  setQuality(quality: RenderQuality, autoPixelRatio = 1.25): void {
    this.quality = quality;
    this.autoRatio = autoPixelRatio;
    if (this.sized) this.resize(this.cssWidth, this.cssHeight, this.deviceRatio);
  }

  resize(width: number, height: number, devicePixelRatio: number): void {
    this.assertUsable();
    this.sized = true;
    this.cssWidth = Math.max(1, Math.round(width));
    this.cssHeight = Math.max(1, Math.round(height));
    this.deviceRatio = devicePixelRatio;
    this.pixelRatio = qualitySettings(this.quality, devicePixelRatio, this.autoRatio).pixelRatio;
    const bufferWidth = Math.max(1, Math.round(this.cssWidth * this.pixelRatio));
    const bufferHeight = Math.max(1, Math.round(this.cssHeight * this.pixelRatio));
    if (this.canvas.width !== bufferWidth) this.canvas.width = bufferWidth;
    if (this.canvas.height !== bufferHeight) this.canvas.height = bufferHeight;
    this.canvas.style.width = `${this.cssWidth}px`;
    this.canvas.style.height = `${this.cssHeight}px`;
    this.projection.setViewport(this.cssWidth, this.cssHeight);
  }

  getProjection(): SharedCameraProjection { return this.projection; }
  getElement(): HTMLCanvasElement { return this.canvas; }
  getMetrics(): RenderMetrics & { geometryBuilds: number; projectionBuilds: number } {
    return { presentationCpuMs: this.presentationCpuMs, effectivePixelRatio: this.pixelRatio,
      shadowResolution: 0, drawCalls: 0, geometries: this.pieces.length + this.moving.length,
      textures: 0, geometryBuilds: this.geometryBuilds, projectionBuilds: this.projectionBuilds };
  }

  render(): void {
    const context = this.context;
    if (!context || this.disposed) return;
    const start = performance.now();
    context.setTransform(this.pixelRatio, 0, 0, this.pixelRatio, 0, 0);
    context.globalAlpha = 1;
    context.fillStyle = BACKGROUND;
    context.fillRect(0, 0, this.cssWidth, this.cssHeight);
    const snapshot = this.snapshot;
    if (snapshot) {
      this.cacheScenery(snapshot);
      this.dynamicCommands.length = 0;
      this.projectBody(snapshot);
      for (const piece of this.moving) {
        const transform = courseTransform(piece, snapshot.simulationTime);
        this.projectPiece(this.dynamicCommands, { ...piece, color: sceneryColor(piece.color), position: ZERO }, transform.position, transform.rotation);
      }
      if (snapshot.striker) for (const piece of PROTOCOL_STRIKER_PIECES) {
        this.projectPiece(this.dynamicCommands, piece, snapshot.striker.position, snapshot.striker.rotation);
      }
      this.dynamicCommands.sort(farFirst);
      if (this.floorLayer) this.drawLayer(context, this.floorLayer);
      else for (const command of this.floorCommands) this.draw(context, command);
      // Do not flatten scenery into a foreground bitmap: moving bodies must be
      // able to pass behind and in front of cached walls and course obstacles.
      let staticIndex = 0, dynamicIndex = 0;
      while (staticIndex < this.staticCommands.length || dynamicIndex < this.dynamicCommands.length) {
        const a = this.staticCommands[staticIndex], b = this.dynamicCommands[dynamicIndex];
        const layer = this.staticLayers.get(staticIndex);
        // A cached slice is safe only if no moving command intersects its depth
        // range. Otherwise replay its individual commands in the exact order.
        if (layer && (!b || layer.depth >= b.depth)) {
          this.drawLayer(context, layer); staticIndex = layer.end;
        } else if (!b || (a && a.depth >= b.depth)) { this.draw(context, a); staticIndex++; }
        else { this.draw(context, b); dynamicIndex++; }
      }
      this.drawSelection(context, snapshot);
    }
    context.globalAlpha = 1;
    this.presentationCpuMs = performance.now() - start + this.presentationStart;
    this.presentationStart = 0;
  }

  private cacheScenery(snapshot: PoseSnapshot): void {
    const room = snapshot.room;
    const geometryKey = room ? `room:${room.width}:${room.depth}:${room.height}` : `course:${snapshot.playground?.difficulty ?? "none"}`;
    if (geometryKey !== this.geometryKey) {
      this.geometryKey = geometryKey;
      this.geometryBuilds++;
      const course = snapshot.playground ? getPlaygroundCourse(snapshot.playground.difficulty) : [];
      this.pieces = room ? protocolRoomPieces(room) : course.filter(piece => !piece.motion).map(piece => ({ ...piece, color: sceneryColor(piece.color) }));
      this.moving = course.filter(piece => piece.motion);
    }
    const key = `${geometryKey}:${this.projection.version}:${snapshot.playground?.station ?? ""}`;
    if (key === this.cacheKey) { this.cacheRasters(); return; }
    this.cacheKey = key;
    this.projectionBuilds++;
    this.staticCommands.length = this.floorCommands.length = 0;
    for (const piece of this.pieces) {
      this.projectPiece(piece.id === "floor" || piece.id.startsWith("boundary-") ? this.floorCommands : this.staticCommands, piece);
    }
    if (!room) {
      this.projectPiece(this.floorCommands, { id: "floor", position: { x: 0, y: -0.015, z: 0 },
        geometry: boxGeometry(ARENA.width, 0.02, ARENA.depth), color: "#e9e9e2" });
    }
    const width = room?.width ?? ARENA.width, depth = room?.depth ?? ARENA.depth;
    for (let x = -width / 2; x <= width / 2; x += 0.5) this.projectLine(this.floorCommands,
      { x, y: 0.003, z: -depth / 2 }, { x, y: 0.003, z: depth / 2 }, "#c8cfca");
    for (let z = -depth / 2; z <= depth / 2; z += 0.5) this.projectLine(this.floorCommands,
      { x: -width / 2, y: 0.003, z }, { x: width / 2, y: 0.003, z }, "#c8cfca");
    if (snapshot.playground) for (const [index, station] of PLAYGROUND_STATIONS.entries()) {
      const point = this.projection.project({ ...station.position, y: 0.025,
        z: station.id === "flat" ? 4.55 : station.id === "wobble" ? -7.3 : station.id === "stones" || station.id === "hurdles" ? 6.7 : 0.28 });
      if (point.visible) this.staticCommands.push({ depth: point.depth, path: new Path2D(), opacity: 1, fill: "#596965",
        text: `${String(index + 1).padStart(2, "0")} / ${station.name.toUpperCase()}`,
        font: `600 ${Math.max(9, Math.min(15, 180 / point.depth))}px system-ui`, x: point.x, y: point.y,
        bounds: [point.x - 180, point.y - 20, point.x + 180, point.y + 5] });
    }
    this.staticCommands.sort(farFirst);
    this.cacheRasters();
  }

  private cacheRasters(): void {
    const key = `${this.cacheKey}:${this.pixelRatio}`;
    if (this.rasterKey === key) return;
    this.clearRasters();
    this.rasterKey = key;
    // Canvas injection in non-browser tests can still replay projected commands.
    if (!this.canvas.ownerDocument) return;
    this.floorLayer = this.rasterize(this.floorCommands, 0, this.floorCommands.length);
    for (let i = 0; i < this.staticCommands.length; i += 32) {
      const layer = this.rasterize(this.staticCommands, i, Math.min(i + 32, this.staticCommands.length));
      if (layer) this.staticLayers.set(i, layer);
    }
  }

  private rasterize(commands: DrawCommand[], start: number, end: number): RasterLayer | null {
    let left = this.cssWidth, top = this.cssHeight, right = 0, bottom = 0;
    for (let i = start; i < end; i++) {
      const bounds = commands[i].bounds;
      if (!bounds) return null;
      left = Math.min(left, bounds[0]); top = Math.min(top, bounds[1]);
      right = Math.max(right, bounds[2]); bottom = Math.max(bottom, bounds[3]);
    }
    left = Math.max(0, Math.floor(left - 2)); top = Math.max(0, Math.floor(top - 2));
    right = Math.min(this.cssWidth, Math.ceil(right + 2)); bottom = Math.min(this.cssHeight, Math.ceil(bottom + 2));
    if (right <= left || bottom <= top) return null;
    const canvas = this.canvas.ownerDocument.createElement("canvas");
    canvas.width = Math.ceil((right - left) * this.pixelRatio);
    canvas.height = Math.ceil((bottom - top) * this.pixelRatio);
    const context = canvas.getContext("2d");
    if (!context) return null;
    context.setTransform(this.pixelRatio, 0, 0, this.pixelRatio, -left * this.pixelRatio, -top * this.pixelRatio);
    for (let i = start; i < end; i++) this.draw(context, commands[i]);
    return { canvas, x: left, y: top, width: canvas.width / this.pixelRatio, height: canvas.height / this.pixelRatio,
      end, depth: commands[end - 1].depth };
  }

  private drawLayer(context: CanvasRenderingContext2D, layer: RasterLayer): void {
    context.globalAlpha = 1;
    context.drawImage(layer.canvas, layer.x, layer.y, layer.width, layer.height);
  }

  private clearRasters(): void {
    if (this.floorLayer) this.floorLayer.canvas.width = this.floorLayer.canvas.height = 1;
    for (const layer of this.staticLayers.values()) layer.canvas.width = layer.canvas.height = 1;
    this.floorLayer = null;
    this.staticLayers.clear();
  }

  private projectPiece(commands: DrawCommand[], piece: ProtocolVisualPiece, parentPosition = ZERO, rotation = IDENTITY): void {
    const center = V3.add(parentPosition, rotateVector(rotation, piece.position));
    const vertices = piece.geometry.vertices.map(vertex => V3.add(center, rotateVector(rotation, vertex)));
    const projected = vertices.map(vertex => this.projection.project(vertex));
    const rgb = [1, 3, 5].map(start => parseInt(piece.color.slice(start, start + 2), 16));
    for (const [a, b, c] of piece.geometry.triangles) {
      const p = projected[a], q = projected[b], r = projected[c];
      if (p.depth <= 0 || q.depth <= 0 || r.depth <= 0) continue;
      const normal = V3.normalize(V3.cross(V3.sub(vertices[b], vertices[a]), V3.sub(vertices[c], vertices[a])));
      const shade = Math.max(0.6, Math.min(1.12, 0.88 + normal.y * 0.14 - normal.x * 0.09 + normal.z * 0.05));
      const path = new Path2D();
      path.moveTo(p.x, p.y); path.lineTo(q.x, q.y); path.lineTo(r.x, r.y); path.closePath();
      commands.push({ depth: (p.depth + q.depth + r.depth) / 3, path,
        fill: `rgb(${rgb.map(value => Math.min(255, Math.round(value * shade))).join(",")})`, opacity: piece.opacity ?? 1,
        bounds: [Math.min(p.x, q.x, r.x), Math.min(p.y, q.y, r.y), Math.max(p.x, q.x, r.x), Math.max(p.y, q.y, r.y)] });
    }
  }

  private projectLine(commands: DrawCommand[], a: Vec3, b: Vec3, color: string, lineWidth = 1): void {
    const p = this.projection.project(a), q = this.projection.project(b);
    if (p.depth <= 0 || q.depth <= 0) return;
    const path = new Path2D(); path.moveTo(p.x, p.y); path.lineTo(q.x, q.y);
    commands.push({ depth: (p.depth + q.depth) / 2, path, stroke: color, opacity: 1, lineWidth,
      bounds: [Math.min(p.x, q.x), Math.min(p.y, q.y), Math.max(p.x, q.x), Math.max(p.y, q.y)] });
  }

  private projectBody(snapshot: PoseSnapshot): void {
    for (const pose of snapshot.segments) {
      const definition = SEGMENT_BY_ID.get(pose.id);
      if (!definition) continue;
      const selected = snapshot.diagnostics.selectedSegment ? pose.id === snapshot.diagnostics.selectedSegment
        : definition.region !== null && definition.region === snapshot.diagnostics.selectedRegion;
      const center = this.projection.project(pose.position);
      if (center.depth <= 0) continue;
      // Every body segment is convex. The union of its filled triangles is its
      // projected hull, so one path retains its exact outline without hundreds
      // of overlapping fills. Segment depth ordering matches the original view.
      const points = silhouette(definition.geometry.vertices.map(vertex =>
        this.projection.project(V3.add(pose.position, rotateVector(pose.rotation, vertex))))
        .filter(point => point.depth > 0));
      if (points.length < 3) continue;
      const path = new Path2D();
      path.moveTo(points[0].x, points[0].y);
      for (let i = 1; i < points.length; i++) path.lineTo(points[i].x, points[i].y);
      path.closePath();
      this.dynamicCommands.push({ depth: center.depth, path, opacity: 1,
        fill: selected ? SELECTION : definition.region ? BODY_COLORS[definition.region] : "#a5b1ad",
        stroke: selected ? SELECTION : "#697673", lineWidth: selected ? 2 : 0.8 });
    }
    const poses = this.presentation.byId;
    for (const definition of SEGMENTS) {
      if (!definition.parent) continue;
      const child = poses.get(definition.id), parent = poses.get(definition.parent);
      if (!child || !parent) continue;
      const a = definition.jointAnchorParent ? transformLocalPoint(parent, definition.jointAnchorParent) : parent.position;
      const b = definition.jointAnchorChild ? transformLocalPoint(child, definition.jointAnchorChild) : child.position;
      this.projectLine(this.dynamicCommands, a, b, JOINT_COLOR, 3);
      const point = this.projection.project(V3.scale(V3.add(a, b), 0.5));
      if (point.visible) {
        const path = new Path2D(); path.arc(point.x, point.y, Math.max(1.2, Math.min(4, 23 / point.depth)), 0, Math.PI * 2);
        this.dynamicCommands.push({ depth: point.depth - 0.025, path, opacity: 1, fill: JOINT_COLOR });
      }
    }
    if (!snapshot.room) for (const foot of ["leftFoot", "rightFoot"] as const) {
      const pose = poses.get(foot);
      if (!pose) continue;
      const point = this.projection.project(pose.position);
      if (!point.visible) continue;
      const radius = this.projection.worldRadiusToPixels(pose.position, 0.16);
      const path = new Path2D(); path.ellipse(point.x, point.y, radius, Math.max(2, radius * 0.32), 0, 0, Math.PI * 2);
      this.dynamicCommands.push({ depth: point.depth, path, opacity: 0.65, stroke: snapshot.support.swingFoot === foot ? "#ad873e"
        : snapshot.support.planted.includes(foot) ? SELECTION : "#87948e", lineWidth: 1.5 });
    }
  }

  private draw(context: CanvasRenderingContext2D, command: DrawCommand): void {
    context.globalAlpha = command.opacity;
    if (command.fill) {
      context.fillStyle = command.fill;
      if (command.text) {
        context.font = command.font!; context.textAlign = "center";
        context.fillText(command.text, command.x!, command.y!);
      } else context.fill(command.path);
    }
    if (command.stroke) {
      context.strokeStyle = command.stroke;
      context.lineWidth = command.lineWidth ?? 1;
      context.stroke(command.path);
    }
  }

  private drawSelection(context: CanvasRenderingContext2D, snapshot: PoseSnapshot): void {
    const id = snapshot.diagnostics.selectedSegment ?? (snapshot.diagnostics.selectedRegion ? PRIMARY_SEGMENT_BY_REGION.get(snapshot.diagnostics.selectedRegion) : null);
    const pose = id ? this.presentation.byId.get(id) : null;
    if (!pose) return;
    const point = this.projection.project(pose.position);
    if (!point.visible) return;
    context.globalAlpha = 1;
    context.beginPath(); context.arc(point.x, point.y, 17, 0, Math.PI * 2);
    context.strokeStyle = SELECTION; context.lineWidth = 2;
    context.stroke();
  }

  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    this.snapshot = null;
    this.context?.clearRect(0, 0, this.canvas.width, this.canvas.height);
    this.context = null;
    this.clearRasters();
    this.pieces = this.moving = [];
    this.staticCommands.length = this.dynamicCommands.length = this.floorCommands.length = 0;
    if (this.ownsCanvas) this.canvas.remove();
  }

  private assertUsable(): void {
    if (this.disposed) throw new Error("Canvas2DView has been disposed.");
  }
}
