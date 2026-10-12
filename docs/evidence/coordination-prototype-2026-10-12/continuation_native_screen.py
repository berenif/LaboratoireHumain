#!/usr/bin/env python3
"""Full native static-grid screen; ignored geometry test never integrates."""
import gzip
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import argparse

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("continuation_geometry",HERE/"continuation_geometry.py")
g=importlib.util.module_from_spec(spec); spec.loader.exec_module(g)
BINARY=g.ROOT/"rust/target/debug/deps/lh_sim-68e67b1c237792f1"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def point(pose, local):
    return g.initial.add(g.initial.vec(pose["position"]),g.initial.rotate(g.initial.quat(pose["rotation"]),local))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--heading-deg",type=int,choices=(0,60),default=0)
    heading=parser.parse_args().heading_deg
    suffix="" if heading==0 else "-60deg"
    OUTPUT=HERE/("continuation-native-geometry"+suffix+".jsonl.gz")
    if OUTPUT.exists(): raise SystemExit("Refused: native screen already exists; no automatic rerun")
    angle=math.radians(heading)
    heading_rotation=[0.,math.sin(angle/2),0.,math.cos(angle/2)]
    binary_hash=sha(BINARY)
    env=dict(os.environ,LH_GEOMETRY_PROGRAM=str(HERE/"continuation-candidate-program.json"))
    process=subprocess.Popen([str(BINARY),"--exact","recovery::trajectory_geometry::serve_trajectory_geometry",
                              "--ignored","--nocapture","--test-threads=1"],
                             stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
                             text=True,env=env)
    count=0; refusals=[]; minfloor=[math.inf,None]; maxdrift=0.; maxheeldrift=0.
    initial_rotation_error=None
    route=json.loads((HERE/"continuation-route-audit.json").read_text())
    heel_anchors={side:g.initial.rotate(heading_rotation,route["milestones"]["168"]["inspection"]["feet"][side+"Foot"]["posteriorWorldPointM"]) for side in ("left","right")}
    pivot_anchors={side:g.initial.rotate(heading_rotation,anchor) for side,anchor in g.anchors.items()}
    try:
        with gzip.open(OUTPUT,"wt") as output, (HERE/"continuation-route-frames.jsonl").open() as frames:
            for line in frames:
                frame=json.loads(line); step=frame["substep"]
                request={"programSubstep":step,"rootPosition":g.initial.rotate(heading_rotation,frame["rootPosition"]),
                         "rootRotation":g.initial.mul(heading_rotation,frame["rootRotation"])}
                g.charge("native")
                process.stdin.write(json.dumps(request)+"\n"); process.stdin.flush()
                while True:
                    reply=process.stdout.readline()
                    if not reply: raise RuntimeError("Native geometry worker ended: "+process.stderr.read())
                    if "GEOMETRY " in reply:
                        value=json.loads(reply.split("GEOMETRY ",1)[1]); break
                if "error" in value: raise RuntimeError(value["error"])
                if value.get("physicalIntegrations") != 0: raise RuntimeError("Unexpected integration marker")
                count+=1
                if value["nativeRefusal"] is not None: refusals.append({"substep":step,"refusal":value["nativeRefusal"]})
                floor=min(value["floorBounds"])
                if floor<minfloor[0]: minfloor=[floor,step]
                poses={p["id"]:p for p in value["poses"]}
                for side in ("left","right"):
                    maxdrift=max(maxdrift,math.dist(point(poses[side+"Forefoot"],g.locals_[side]),pivot_anchors[side]))
                    if step>=168:
                        local=g.seed["initial"]["inspection"]["feet"][side+"Foot"]["posteriorLocalPointM"]
                        maxheeldrift=max(maxheeldrift,math.dist(point(poses[side+"Foot"],local),heel_anchors[side]))
                output.write(json.dumps({"substep":step,"query":request,"result":value},allow_nan=False)+"\n")
        process.stdin.close()
        trailer=process.stdout.read(); stderr=process.stderr.read(); code=process.wait(timeout=30)
        if code: raise RuntimeError(f"Native worker exit {code}: {stderr}")
        if sha(BINARY)!=binary_hash: raise RuntimeError("Native binary changed during static screen")
    except BaseException:
        process.kill(); process.wait(); raise
    receipt={"scope":"native f32 program compilation and complete static geometry grid",
             "headingDegrees":heading,"binarySha256":binary_hash,"programSha256":sha(HERE/"continuation-candidate-program.json"),
             "compressedTraceSha256":sha(OUTPUT),"parentReservationId":g.TOKEN,
             "nativeGeometryQueries":count,"physicalIntegrations":0,
             "nativeCommandCompilerPassed":True,"nativeGeometryRefusals":refusals,
             "minimumNativeFloorBoundM":minfloor,
             "maximumNativeMaterialPivotDriftM":maxdrift,
             "maximumNativeRetainedHeelDriftAfterAcquisitionM":maxheeldrift,
             "newNativeHistoryAllowed":False,
             "limitations":["static roots are inspection states, not actions",
                            "virtual points do not prove loaded contacts or static force/torque feasibility",
                            "native execution and original recovery terminal remain unproven"]}
    g.initial.dump(HERE/("continuation-native-audit"+suffix+".json"),receipt)
    print(json.dumps(receipt))


if __name__=="__main__": main()
