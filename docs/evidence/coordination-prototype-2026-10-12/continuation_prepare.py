#!/usr/bin/env python3
"""Two pure FK inspections; never imports a physics engine or third-party code.

Run once after the enclosing preparation ledger reserves two inspections.
Refuses to overwrite an existing local ledger, including an interrupted one.
Outputs a partial reference, not an admissible motor program or launch permit.
"""
import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
LEDGER = HERE / "continuation-ledger.json"


def dump(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False) + "\n")


def vec(obj):
    return [obj[k] for k in ("x", "y", "z")]


def quat(obj):
    return [obj[k] for k in ("x", "y", "z", "w")]


def mul(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def inv(q):
    norm = sum(x*x for x in q)
    return [-q[0]/norm, -q[1]/norm, -q[2]/norm, q[3]/norm]


def rotate(q, v):
    return mul(mul(q, v+[0.0]), inv(q))[:3]


def add(a, b):
    return [x+y for x, y in zip(a, b)]


def fk(segments, q, root):
    # Pure implementation of trajectory_geometry.rs, evaluated in f64 here.
    # This is not a replacement for the required native f32 geometry screen.
    poses = [root]
    indices = {s["id"]: i for i, s in enumerate(segments)}
    for i, segment in enumerate(segments[1:], 1):
        joint = segment["jointProfile"]
        parent = poses[indices[segment["parent"]]]
        r = [math.tan(x/2) for x in q[i]] + [1.0]
        norm = math.sqrt(sum(x*x for x in r))
        r = [x/norm for x in r]
        orientation = mul(mul(mul(parent[1], quat(joint["parentFrame"]["rotation"])), r),
                          inv(quat(joint["childFrame"]["rotation"])))
        anchor = add(parent[0], rotate(parent[1], vec(joint["parentFrame"]["anchor"])))
        position = add(anchor, [-x for x in rotate(orientation, vec(joint["childFrame"]["anchor"]))])
        poses.append((position, orientation))
    return poses


def inspect_mesh(segments, poses):
    points, feet, com = [], {}, [0.0]*3
    mass = sum(s["massKg"] for s in segments)
    for s, (p, r) in zip(segments, poses):
        local_com = vec(s["goldenMass"]["centerOfMass"])
        world_com = add(p, rotate(r, local_com))
        com = add(com, [x*s["massKg"]/mass for x in world_com])
        raw = s["geometry"]["vertices"]
        vertices = [raw[k:k+3] for k in range(0, len(raw), 3)]
        transformed = [add(p, rotate(r, v)) for v in vertices]
        points.extend((v[1], s["id"], i) for i, v in enumerate(transformed))
        if s["id"] in ("leftFoot", "rightFoot", "leftForefoot", "rightForefoot"):
            low = min(range(len(transformed)), key=lambda i: transformed[i][1])
            entry = {"lowestMeshVertex": low, "lowestWorldPointM": transformed[low],
                     "orientationXyzw": r}
            if s["role"] == "hindfoot":
                heel = min((i for i, v in enumerate(vertices) if v[2] < -0.02),
                           key=lambda i: transformed[i][1])
                entry.update({"posteriorMeshVertex": heel,
                              "posteriorLocalPointM": vertices[heel],
                              "posteriorWorldPointM": transformed[heel]})
            feet[s["id"]] = entry
    return {"centerOfMassM": com, "lowestMeshPoint": min(points), "feet": feet,
            "scope": "mesh/FK only; vertices are not native solved contacts",
            "nativeCollisionScreen": "not performed", "supportProof": "missing"}


def main():
    if LEDGER.exists():
        raise SystemExit("Refused: local inspection reservation already exists; no automatic renewal")
    ledger = {"scope": "continuation preparation; historical ledgers remain closed",
              "sharedCeiling": 20000, "delegatedCeiling": 8000,
              "reservedInspections": 2, "completedInspections": 0,
              "chargedInspections": 2, "physicalIntegrations": 0,
              "status": "reserved-before-first-FK"}
    dump(LEDGER, ledger)
    inputs = {"model": "rust/crates/model/data/canonical-v1.json",
              "fixture": "rust/crates/sim/data/recovery-v1.json",
              "profile": "rust/crates/sim/data/recovery-profile-v24.json",
              "contract": "rust/crates/sim/data/recovery-contract-v1.json"}
    loaded = {key: json.loads((ROOT/path).read_text()) for key, path in inputs.items()}
    hashes = {path: hashlib.sha256((ROOT/path).read_bytes()).hexdigest() for path in inputs.values()}
    segments = loaded["model"]["segments"]
    fixture = next(f for f in loaded["fixture"]["cases"] if f["id"] == "landed-crouch-left")
    source = fixture["poses"]
    ids = [s["id"] for s in segments]
    assert ids == [p["id"] for p in source] and len(ids) == 25
    assert all(v == 0 for p in source for field in ("linearVelocity", "angularVelocity") for v in vec(p[field]))
    q = [[0.0]*3 for _ in segments]
    for i, s in enumerate(segments[1:], 1):
        joint = s["jointProfile"]
        parent = source[ids.index(s["parent"])]
        relative = mul(inv(mul(quat(parent["rotation"]), quat(joint["parentFrame"]["rotation"]))),
                       mul(quat(source[i]["rotation"]), quat(joint["childFrame"]["rotation"])))
        if relative[3] < 0:
            relative = [-v for v in relative]
        for k, coordinate in enumerate("xyz"):
            value = (2*math.atan2(relative[k], relative[3])+math.pi) % (2*math.pi)-math.pi
            axis = next((a for a in joint["axes"] if a["coordinate"] == coordinate), None)
            if axis is None:
                assert abs(value) < 1e-5
            else:
                assert axis["minRadians"] <= value <= axis["maxRadians"]
                q[i][k] = value
    root = (vec(source[0]["position"]), quat(source[0]["rotation"]))
    initial_fk = fk(segments, q, root)
    initial = inspect_mesh(segments, initial_fk)
    initial["maximumFixturePositionReconstructionErrorM"] = max(
        math.dist(p[0], vec(s["position"])) for p, s in zip(initial_fk, source))
    ledger["completedInspections"] = 1
    dump(LEDGER, ledger)
    terminal_root = (vec(segments[0]["initial"]["position"]), quat(segments[0]["initial"]["rotation"]))
    terminal_q = [[0.0]*3 for _ in segments]
    terminal = inspect_mesh(segments, fk(segments, terminal_q, terminal_root))
    ledger.update(completedInspections=2, status="closed-no-physical-authorization")
    dump(LEDGER, ledger)
    result = {"schemaVersion": 1, "fixtureId": fixture["id"], "inputsSha256": hashes,
              "profileId": loaded["profile"]["id"], "segmentIds": ids,
              "initial": {"rootPositionM": root[0], "rootRotationXyzw": root[1],
                          "coordinatesF64Diagnostic": q, "inspection": initial},
              "terminalObjectiveOnly": {"coordinates": terminal_q,
                          "rootForInspectionOnly": terminal_root, "inspection": terminal},
              "commandProgram": None, "nativeF32ProgramCompiled": False,
              "completeNumericTrajectoryReady": False, "newNativeHistoryAllowed": False,
              "timingPolicy": {"heelsAcquiredBySubstep": 480, "localPersistenceSubsteps": 132,
                               "standingPlusHoldBySubstep": 6000, "extraHoldSubsteps": 240},
              "blockers": ["No acquisition/recenter numerical keyframes",
                           "No material pivot/replacement schedule from the initial feet",
                           "No interpolation geometry or retained-point audit",
                           "No supported continuation to standing",
                           "No original-observer/handoff adapter proven"],
              "physicalIntegrations": 0, "pureFkInspections": 2,
              "note": "Two endpoints do not define a route. No interpolation or contact intent is invented. F64 diagnostic coordinates must be rederived by fixture_coordinates in native f32."}
    dump(HERE/"continuation-seed.json", result)
    print(json.dumps({"completeNumericTrajectoryReady": False, "physicalIntegrations": 0,
                      "pureFkInspections": 2, "fixtureReconstructionErrorM": initial["maximumFixturePositionReconstructionErrorM"],
                      "initialComM": initial["centerOfMassM"],
                      "posteriorHeelHeightsM": {s: v["posteriorWorldPointM"][1] for s, v in initial["feet"].items() if "posteriorWorldPointM" in v}}))


if __name__ == "__main__":
    main()
