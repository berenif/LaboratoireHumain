#!/usr/bin/env python3
"""Zero-integration raw quaternion versus normalized source FK closure."""
import argparse,importlib.util,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from source_engine import World,ROOT
from native_env import atomic_json
from source_math import P
s=importlib.util.spec_from_file_location('native_raw_fk',ROOT/'scripts/getup-step-reference.py');fk=importlib.util.module_from_spec(s);s.loader.exec_module(fk)

def main():
    p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);a=p.parse_args();x=json.loads((a.base/'native-quiet-entry.json').read_text());parts={p['id']:p for p in x['nativeSnapshot']['segments']};physical=np.array(x['nativePhysicalObservation']);kin=fk.Kinematics(json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text()));root=fk.transform(parts['torso']['position'],parts['torso']['rotation']);unit=root.copy();unit[:3,:3]=Rotation.from_quat([parts['torso']['rotation'][k]for k in'xyzw']).as_matrix();raw_fk=kin.forward(physical[:21],root);unit_fk=kin.forward(physical[:21],unit);world=World(a.base)
    try:
        world.state(x['nativeSnapshot']['segments'],physical);rows=[]
        for name in kin.model['referenceMetadata']['sourceBodyOrder']:
            body=parts[name]
            native=fk.xyz(body['position']);source=P.T@world.arr(3,(17,3))[world.ids[name]];rows.append(dict(body=name,nativeQuaternionNormSquared=sum(body['rotation'][k]**2 for k in'xyzw'),nativePositionWorldM=native.tolist(),sourcePositionInNativeWorldM=source.tolist(),rawFKNativeErrorM=float(np.linalg.norm(raw_fk[name][:3,3]-native)),normalizedFKSourceErrorM=float(np.linalg.norm(unit_fk[name][:3,3]-source)),sourceNativePositionErrorM=float(np.linalg.norm(source-native)),rawUnitFKDifferenceM=float(np.linalg.norm(raw_fk[name][:3,3]-unit_fk[name][:3,3]))))
        result=dict(physicsIntegrations=world.dim(6),explicitStaticForwardCalls=1,rootQuaternionNormSquared=rows[0]['nativeQuaternionNormSquared'],maximumRawFKNativeErrorM=max(r['rawFKNativeErrorM']for r in rows),maximumNormalizedFKSourceErrorM=max(r['normalizedFKSourceErrorM']for r in rows),maximumNativeSourcePositionErrorM=max(r['sourceNativePositionErrorM']for r in rows),rows=rows,scope='representation diagnosis only; native application unchanged; only a new diagnostic source world receives transferred q/qvel; zero integrations and no parity gate relaxed')
        assert world.dim(6)==0;atomic_json(a.base/'geometry-representation-diagnosis.json',result);print(json.dumps({k:v for k,v in result.items()if k!='rows'},indent=2))
    finally:world.close()
if __name__=='__main__':main()
