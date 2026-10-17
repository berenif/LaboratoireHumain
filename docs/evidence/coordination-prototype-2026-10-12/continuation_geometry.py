#!/usr/bin/env python3
"""New sagittal geometry family; never steps physics or commands a root.

Every FK/native request is charged before evaluation against the parent's fixed
5000-inspection reservation. Geometry roots are inspection variables only.
"""
import importlib.util
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
TOKEN = "d9649c8e-c891-4b5c-8045-8171437911ac"
JOURNAL = HERE / "continuation-geometry-ledger.json"
spec = importlib.util.spec_from_file_location("continuation_initial", HERE/"continuation_prepare.py")
initial = importlib.util.module_from_spec(spec)
spec.loader.exec_module(initial)


def charge(kind):
    global_ledger = json.loads((HERE/"budget/ledger.json").read_text())["payload"]
    reservation = next(r for r in global_ledger["reservations"] if r["id"] == TOKEN)
    ceiling = reservation["inspections"]
    if reservation["status"] != "reserved" or not (5000 <= ceiling <= 8000):
        raise RuntimeError("No active parent inspection reservation")
    if JOURNAL.exists():
        ledger = json.loads(JOURNAL.read_text())
    else:
        ledger = {"parentReservationId": TOKEN, "ceiling": ceiling,
                  "chargedFk": 0, "chargedNativeGeometry": 0, "physicalIntegrations": 0}
    ledger["ceiling"] = ceiling
    if ledger["chargedFk"] + ledger["chargedNativeGeometry"] >= ceiling:
        raise RuntimeError("Static inspection ceiling reached")
    ledger["chargedFk" if kind == "fk" else "chargedNativeGeometry"] += 1
    initial.dump(JOURNAL, ledger)


model = json.loads((ROOT/"rust/crates/model/data/canonical-v1.json").read_text())
segments = model["segments"]
ids = [s["id"] for s in segments]
seed = json.loads((HERE/"continuation-seed.json").read_text())
q0 = seed["initial"]["coordinatesF64Diagnostic"]
root0 = (seed["initial"]["rootPositionM"], seed["initial"]["rootRotationXyzw"])
# Both front edges share y/z. One fixed vertex per forefoot defines the pivot
# line; the corresponding second foot point is independently checked below.
anchors = {side: seed["initial"]["inspection"]["feet"][side+"Forefoot"]["lowestWorldPointM"]
           for side in ("left", "right")}
locals_ = {}
for side in ("left", "right"):
    i = ids.index(side+"Forefoot")
    vertex = seed["initial"]["inspection"]["feet"][side+"Forefoot"]["lowestMeshVertex"]
    locals_[side] = segments[i]["geometry"]["vertices"][3*vertex:3*vertex+3]


def target(hip, knee, ankle, lumbar=0.0, arm=0.0):
    q = [[0.0]*3 for _ in segments]
    for side in ("left", "right"):
        for suffix, angle in (("Thigh", hip), ("Shin", knee), ("Ankle", ankle), ("UpperArm", arm)):
            q[ids.index(side+suffix)][0] = angle
    q[ids.index("lumbar")][0] = lumbar
    return q


def inspect(q, foot_pitch):
    charge("fk")
    # This relation follows native sagittal frames, not Euler decomposition of
    # arbitrary 3D coordinates. The chosen family keeps both legs sagittal.
    pitch = foot_pitch + q[ids.index("leftThigh")][0] - q[ids.index("leftShin")][0] + q[ids.index("leftAnkle")][0]
    root_rotation = [math.sin(pitch/2), 0.0, 0.0, math.cos(pitch/2)]
    poses = initial.fk(segments, q, ([0.0]*3, root_rotation))
    i = ids.index("leftForefoot")
    point = initial.add(poses[i][0], initial.rotate(poses[i][1], locals_["left"]))
    shift = [a-b for a, b in zip(anchors["left"], point)]
    poses = [(initial.add(p, shift), r) for p, r in poses]
    summary = initial.inspect_mesh(segments, poses)
    drift = {}
    for side in ("left", "right"):
        i = ids.index(side+"Forefoot")
        point = initial.add(poses[i][0], initial.rotate(poses[i][1], locals_[side]))
        drift[side] = math.dist(point, anchors[side])
    return {"rootPosition": shift, "rootRotation": root_rotation,
            "coordinates": q, "materialDriftM": drift, "inspection": summary}


if __name__ == "__main__":
    results = {"heels": inspect(target(.54, .66, .32), 0.),
               "recenter": inspect(target(.42, .66, .24), 0.),
               "standing": inspect(target(0., 0., 0.), 0.)}
    initial.dump(HERE/"continuation-key-exploration.json", results)
    for name, r in results.items():
        print(name, json.dumps({"root": r["rootPosition"], "com": r["inspection"]["centerOfMassM"],
                               "lowest": r["inspection"]["lowestMeshPoint"], "drift": r["materialDriftM"]}))
