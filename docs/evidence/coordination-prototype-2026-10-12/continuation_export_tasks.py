#!/usr/bin/env python3
"""Extract 60 Hz tasks from archived FK results: zero new FK/inspection/physics."""
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent


def main():
    frames_path=HERE/"continuation-route-frames.jsonl"
    frames=[json.loads(line) for line in frames_path.read_text().splitlines()]
    seed=json.loads((HERE/"continuation-seed.json").read_text())
    program=json.loads((HERE/"continuation-candidate-program.json").read_text())
    model=json.loads((HERE.parents[2]/"rust/crates/model/data/canonical-v1.json").read_text())
    ids=program["segmentIds"]
    material=[]
    for surface in program["contacts"]:
        i=ids.index(surface["segment"]); segment=model["segments"][i]
        triangle=segment["geometry"]["indices"][3*surface["triangle"]:3*surface["triangle"]+3]
        vertex=triangle[surface["barycentric"].index(1.)]
        local=segment["geometry"]["vertices"][3*vertex:3*vertex+3]
        foot=frames[surface["startSubstep"]]["inspection"]["feet"][surface["segment"]]
        world=foot["posteriorWorldPointM"] if surface["segment"].endswith("Foot") else foot["lowestWorldPointM"]
        material.append(dict(surface,segmentIndex=i,localPointM=local,plannedAnchorWorldM=world,
                             scope="geometric material target; native acquisition must be observed"))
    rows=[]
    for step in range(0,1921,4):
        frame=frames[step]
        lo=max(0,step-1); hi=min(1920,step+1)
        dt=(hi-lo)/240
        low=frames[lo]["inspection"]["centerOfMassM"]
        high=frames[hi]["inspection"]["centerOfMassM"]
        velocity=[(b-a)/dt for a,b in zip(low,high)]
        phase="acquire-heels" if step<168 else "recenter" if step<300 else "hold-recentered" if step<432 else "rise" if step<1680 else "standing-hold"
        feet=[]
        for side in ("left","right"):
            hindfoot=frame["inspection"]["feet"][side+"Foot"]
            forefoot=frame["inspection"]["feet"][side+"Forefoot"]
            feet.append({"side":side,"hindfootRotationXyzw":hindfoot["orientationXyzw"],
                         "forefootRotationXyzw":forefoot["orientationXyzw"],
                         "posteriorMaterialTargetWorldM":hindfoot["posteriorWorldPointM"]})
        rows.append({"substep":step,"plannedPhase":phase,
                     "comPositionM":frame["inspection"]["centerOfMassM"],"comVelocityMps":velocity,
                     "rootRotationForPostureOnlyXyzw":frame["rootRotation"],"feet":feet})
    result={"schemaVersion":1,"nativeHz":240,"controlHz":60,"firstSubstep":0,"lastSubstep":1920,
            "programSha256":hashlib.sha256((HERE/"continuation-candidate-program.json").read_bytes()).hexdigest(),
            "sourceFramesSha256":hashlib.sha256(frames_path.read_bytes()).hexdigest(),
            "materialTargets":material,"rows":rows,"newFkInspections":0,"physicalIntegrations":0,
            "semantics":["coordinates/qdot come only from the shared native Program compiler",
                         "root rotation is a posture objective and never an actuator",
                         "world positions/vectors and quaternions must be rotated by the requested heading",
                         "plannedPhase and material target intervals never establish actual contact or load",
                         "comVelocityMps is a central finite difference of archived geometry, not measured dynamics"]}
    path=HERE/"continuation-tasks.json"
    path.write_text(json.dumps(result,separators=(',',':'),allow_nan=False)+"\n")
    print(json.dumps({"path":str(path),"rows":len(rows),"sha256":hashlib.sha256(path.read_bytes()).hexdigest(),"newFkInspections":0}))


if __name__=="__main__":main()
