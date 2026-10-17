#!/usr/bin/env python3
"""Construct and inspect one new six-key sagittal route; no physical integration."""
import importlib.util
import json
import math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("continuation_geometry", HERE/"continuation_geometry.py")
g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)


def cross(o, a, b):
    return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])


def hull(points):
    p = sorted(set(tuple(x) for x in points))
    lower, upper = [], []
    for part, result in ((p, lower), (list(reversed(p)), upper)):
        for point in part:
            while len(result) >= 2 and cross(result[-2], result[-1], point) <= 0:
                result.pop()
            result.append(point)
    return lower[:-1] + upper[:-1]


def support_margin(result):
    # Flat-foot support is geometric/virtual. No force or solved contact claim.
    points = []
    feet = result["inspection"]["feet"]
    for side in ("left", "right"):
        for suffix in ("Foot", "Forefoot"):
            name = side+suffix
            i = g.ids.index(name)
            raw = g.segments[i]["geometry"]["vertices"]
            low = feet[name]["lowestMeshVertex"]
            local_low = raw[3*low:3*low+3]
            translation = [v-d for v, d in zip(feet[name]["lowestWorldPointM"],
                                             g.initial.rotate(feet[name]["orientationXyzw"], local_low))]
            for k in range(0, len(raw), 3):
                world = g.initial.add(translation, g.initial.rotate(feet[name]["orientationXyzw"], raw[k:k+3]))
                if abs(world[1] - .0009999985139407044) < 1e-6:
                    points.append([world[0], world[2]])
    polygon = hull(points)
    if len(polygon) < 3:
        return None
    com = result["inspection"]["centerOfMassM"]
    p = [com[0], com[2]]
    return min(cross(a, b, p)/math.dist(a, b) for a, b in zip(polygon, polygon[1:]+polygon[:1]))


def command(keys, step):
    # Same operation order and f32 arithmetic as the native zero-tangent Hermite
    # program. Native service independently compiles and samples this program.
    if step >= keys[-1][0]:
        return keys[-1][1], np.zeros((25, 3), dtype=np.float32)
    right = next(i for i, k in enumerate(keys) if k[0] > step)
    a, b = keys[right-1], keys[right]
    if step == a[0]:
        return a[1], np.zeros((25, 3), dtype=np.float32)
    f = np.float32
    u = f(step-a[0])/f(b[0]-a[0]); u2=u*u; u3=u2*u
    span=f(b[0]-a[0])*f(1/240)
    q=(f(2)*u3-f(3)*u2+f(1))*a[1]+(-f(2)*u3+f(3)*u2)*b[1]
    dq=((f(6)*u2-f(6)*u)*a[1]+(-f(6)*u2+f(6)*u)*b[1])/span
    return q, dq


def main():
    frames_path=HERE/"continuation-route-frames.jsonl"
    if frames_path.exists():
        raise SystemExit("Refused: candidate frames already exist; no silent rerun")
    keys=[(0,g.q0), (168,g.target(.54,.66,.32)), (300,g.target(.52,.62,.30)),
          (432,g.target(.52,.62,.30)), (1680,g.target(0.,0.,0.)), (1920,g.target(0.,0.,0.))]
    keys=[(s,np.asarray(q,dtype=np.float32)) for s,q in keys]
    surfaces=[]
    for side in ("left","right"):
        for suffix,start in (("Forefoot",0),("Foot",168)):
            name=side+suffix; i=g.ids.index(name)
            vertex=g.seed["initial"]["inspection"]["feet"][name].get("posteriorMeshVertex",8)
            indices=g.segments[i]["geometry"]["indices"]
            offset=indices.index(vertex); triangle=offset//3; bary=[0.,0.,0.]; bary[offset%3]=1.
            surfaces.append({"segment":name,"triangle":triangle,"barycentric":bary,
                             "startSubstep":start,"endSubstep":1920})
    program={"schemaVersion":1,"segmentIds":g.ids,
             "keyframes":[{"substep":s,"positions":q.tolist(),"velocities":[[0.,0.,0.] for _ in range(25)]} for s,q in keys],
             "contacts":surfaces}
    g.initial.dump(HERE/"continuation-candidate-program.json",program)
    minimum_margin=(math.inf,None); minimum_floor=(math.inf,None)
    maximum_drift=0.; maximum_rate=0.; maximum_slew=0.; previous=None
    milestones={}; heel_maximum_after_acquisition=0.
    with frames_path.open("x") as stream:
        for step in range(1921):
            q,dq=command(keys,step)
            if step<=168:
                u=step/168; foot_pitch=.2*(1-3*u*u+2*u*u*u)
            else:
                foot_pitch=0.
            r=g.inspect(q.tolist(),foot_pitch)
            r["substep"]=step
            margin=support_margin(r)
            r["virtualFootSupportMarginM"]=margin
            if step>=168 and (margin is None or margin<minimum_margin[0]):
                minimum_margin=(margin if margin is not None else -math.inf,step)
            low=r["inspection"]["lowestMeshPoint"][0]
            if low<minimum_floor[0]: minimum_floor=(low,step)
            maximum_drift=max(maximum_drift,max(r["materialDriftM"].values()))
            maximum_rate=max(maximum_rate,float(np.max(np.abs(dq))))
            if previous is not None: maximum_slew=max(maximum_slew,float(np.max(np.abs(q-previous))))
            previous=q
            if step>=168:
                heel_maximum_after_acquisition=max(heel_maximum_after_acquisition,*[r["inspection"]["feet"][side+"Foot"]["posteriorWorldPointM"][1] for side in ("left","right")])
            if step in [k[0] for k in keys]: milestones[str(step)]=r
            stream.write(json.dumps(r,allow_nan=False)+"\n")
    receipt={"candidate":"new-six-key-sagittal-material-pivot", "parentReservationId":g.TOKEN,
             "substeps":1920,"seconds":8.,"pureFkSamples":1921,"nativeGeometryQueries":0,
             "maximumMaterialPivotDriftM":maximum_drift,
             "minimumVirtualFootSupportMarginAfterHeelsM":minimum_margin,
             "minimumMeshFloorHeightM":minimum_floor,
             "maximumPosteriorHeelHeightAfterAcquisitionM":heel_maximum_after_acquisition,
             "maximumTargetRateRadPerS":maximum_rate,"maximumTargetDeltaRad":maximum_slew,
             "milestones":milestones,"physicalIntegrations":0,
             "preparedGeometryOnly":True,"newNativeHistoryAllowed":False,
             "limitations":["root poses are geometry variables, never motor actions",
                            "the initial pivot has only a line of virtual support; dynamic acquisition is unproven",
                            "virtual CoM margin does not prove load, friction, torque or capture margin",
                            "native collision and f32 program validation pending",
                            "original recovery observer and handoff adaptation unproven"]}
    g.initial.dump(HERE/"continuation-route-audit.json",receipt)
    print(json.dumps({k:v for k,v in receipt.items() if k not in ('milestones','limitations')}))


if __name__=="__main__":
    main()
