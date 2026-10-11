#!/usr/bin/env python3
"""Audit exported numeric data without loading a policy or executable objects."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile
import numpy as np


def audit(file, metadata):
    if file.stat().st_size > 256*1024*1024:
        raise ValueError('Numeric archive > 256 MiB')
    with zipfile.ZipFile(file) as z:
        if sum(i.file_size for i in z.infolist()) > 512*1024*1024 or len(z.infolist()) > 64:
            raise ValueError('Excessive expanded archive')
    meta=json.loads(metadata.read_text())
    if hashlib.sha256(file.read_bytes()).hexdigest()!=meta['numericFileSha256']:
        raise ValueError('Numeric archive checksum')
    with np.load(file,allow_pickle=False) as z:
        arrays={k:z[k] for k in z.files}
    for name,value in arrays.items():
        if value.dtype.kind not in 'fiu' or not np.isfinite(value).all():
            raise ValueError(f'Non-numeric/nonfinite array {name}')
    t=arrays['time_s']; n=len(t); bodies=len(meta['bodyNames'])
    for key,shape in [('body_pos',(n,bodies,3)),('body_quat_xyzw',(n,bodies,4)),
                      ('com_pos',(n,3)),('com_vel',(n,3)),('floor_load_n',(n,bodies))]:
        if arrays[key].shape!=shape: raise ValueError(f'Shape {key}')
    if n<2 or n>30001 or np.any(np.diff(t)<=0) or t[-1]-t[0]>30:
        raise ValueError('Clock/order/duration')
    if not np.allclose(np.diff(t),meta['timeStepS'],rtol=0,atol=1e-10):
        raise ValueError('Irregular physics clock')
    if np.max(np.abs(np.linalg.norm(arrays['body_quat_xyzw'],axis=2)-1))>1e-5:
        raise ValueError('Quaternion normalization')
    offsets=arrays['contact_offsets']; rows=arrays['contact_rows']
    if offsets.shape!=(n+1,) or offsets[0]!=0 or offsets[-1]!=len(rows) or np.any(np.diff(offsets)<0):
        raise ValueError('Resolved contact offsets')
    names=meta['bodyNames']; loads=arrays['floor_load_n']; times=t-t[0]
    # 3 N is an analysis threshold, not source/native standing qualification.
    support={name:{'maxResolvedFloorNormalN':float(loads[:,i].max()),
                   'totalTimeAbove3NS':float(np.count_nonzero(loads[:,i]>=3)*meta['timeStepS'])}
             for i,name in enumerate(names) if np.any(loads[:,i]>=3)}
    acceleration=np.diff(arrays['com_vel'],axis=0)/np.diff(t)[:,None]
    result={'numericAuditPassed':True,'samples':n,'bodyCount':bodies,
            'durationS':float(times[-1]),'sourceTotalMassKg':float(arrays['body_mass_kg'].sum()),
            'quaternionFormat':'xyzw','sourceUpAxis':'+Z','actualContactRows':len(rows),
            'supportBodiesAbove3N':support,
            'maxAbsJointVelocityRadS':float(np.abs(arrays['qvel'][:,6:]).max()),
            'maxAbsActuatorScalarForce':float(np.abs(arrays['actuator_force']).max()),
            'maxAbsGeneralizedJointTorqueNm':float(np.abs(arrays['qfrc_actuator'][:,6:]).max()),
            'maxComAccelerationMps2':float(np.linalg.norm(acceleration,axis=1).max()),
            'firstSampleHasFreshSolvedContacts':False,
            'nativeMappingPassed':False,'nativeGeometryPassed':False,'nativePhysicalSuccess':False,
            'limits':['Source contact snapshots follow each actual integration substep except initial reset sample',
                      'A solved source contact is not a native contact plan or support proof']}
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('file',type=Path);p.add_argument('metadata',type=Path)
    p.add_argument('--out',type=Path);a=p.parse_args();result=audit(a.file,a.metadata)
    content=json.dumps(result,indent=2)+'\n'
    if a.out:a.out.write_text(content)
    print(content)
