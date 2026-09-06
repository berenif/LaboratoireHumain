import { REGION_IDS } from "../core/types";
import type { CameraProjection, PickResult, PoseSnapshot, Ray, RegionId } from "../core/types";

export interface ProjectedRegionDiagnostic {
  region: RegionId;
  clientX: number;
  clientY: number;
  depthM: number;
  visible: boolean;
  pickedRegion: RegionId | null;
  centerSelectsSelf: boolean;
}

/** Read-only evidence from the same camera and picker used by pointer input. */
export function projectVisibleRegions(
  snapshot: PoseSnapshot,
  projection: CameraProjection,
  rect: DOMRectReadOnly,
  pick: (ray: Ray) => PickResult | null,
): ProjectedRegionDiagnostic[] {
  const camera = projection.getState();
  return REGION_IDS.flatMap((region) => {
    const segment = snapshot.segments.find((pose) => pose.id === region);
    if (!segment) return [];
    const point = projection.project(segment.position);
    // Scale projected CSS coordinates to the actual event target rectangle.
    // Visibility deliberately excludes the presentation camera's clip margin.
    const localX = point.x * rect.width / camera.viewportWidth;
    const localY = point.y * rect.height / camera.viewportHeight;
    const clientX = rect.left + localX;
    const clientY = rect.top + localY;
    const visible =
      Number.isFinite(clientX) && Number.isFinite(clientY) &&
      point.depth >= camera.near && point.depth <= camera.far &&
      localX >= 0 && localX < rect.width && localY >= 0 && localY < rect.height;
    const hit = visible ? pick(projection.screenToRay(clientX, clientY, rect)) : null;
    return [{
      region, clientX, clientY, depthM: point.depth, visible,
      pickedRegion: hit?.region ?? null,
      centerSelectsSelf: visible && hit?.region === region,
    }];
  });
}

/** Does not advance simulation, move the camera, or change the selected body. */
export function inspectScenePresentation(
  snapshot: PoseSnapshot,
  projection: CameraProjection,
  canvas: HTMLCanvasElement,
  pick: (ray: Ray) => PickResult | null,
) {
  const rect = canvas.getBoundingClientRect();
  const regions = projectVisibleRegions(snapshot, projection, rect, pick);
  const dragCandidates = new Set<RegionId>(["torso", "pelvis"]);
  if (snapshot.diagnostics.selectedRegion) dragCandidates.add(snapshot.diagnostics.selectedRegion);
  const ownerWindow = canvas.ownerDocument.defaultView;
  return {
    snapshotSequence: snapshot.sequence,
    simulationTime: snapshot.simulationTime,
    viewport: { width: ownerWindow?.innerWidth ?? 0, height: ownerWindow?.innerHeight ?? 0 },
    devicePixelRatio: ownerWindow?.devicePixelRatio ?? 1,
    canvasCss: { width: rect.width, height: rect.height, left: rect.left, top: rect.top },
    internalResolution: { width: canvas.width, height: canvas.height },
    renderedPixelRatio: {
      x: rect.width > 0 ? canvas.width / rect.width : 0,
      y: rect.height > 0 ? canvas.height / rect.height : 0,
    },
    camera: projection.getState(),
    regions,
    allSevenCentersSelectSelf: regions.length === REGION_IDS.length && regions.every((region) => region.centerSelectsSelf),
    continuedDragVisibleSelectable: regions.some((region) => dragCandidates.has(region.region) && region.centerSelectsSelf),
  };
}
