#!/usr/bin/env python3
"""Explicit isolated impratio=2 or 8 comparison; never changes the selected preset.

Only contact impedance ratio changes. A new XML-bound cold cache is rebuilt
from identical selected qpos/qvel. No original full cache enters this model.
Unchanged-original-model qualification is deliberately not claimed.
"""
import argparse
import ctypes as C
import json
import shutil
import time
from pathlib import Path

import numpy as np

from source_step_probe import World,sha,ptr,PTR
from source_selected_step import prepare,state_sha,load_preset
from source_cadenced_ten_steps import CadencedSteps
from source_arm_knee_mobility import native_bytes

RATIO=2.


def options(w,inspector):
    ro=C.CDLL(str(inspector));ro.lh_ro_model.argtypes=[C.c_void_p,PTR];ro.lh_ro_model.restype=None
    before=native_bytes(w);out=np.zeros(22);ro.lh_ro_model(w.h,ptr(out))
    assert native_bytes(w)==before
    return out


def prepare_impedance(output,inspector,ratio=RATIO):
    assert ratio in (2.,8.)
    baseline,helper,selection=prepare(output)
    original=output/'baseline-model-session';baseline.rename(original)
    session=output/'model-session';(session/'source-model').mkdir(parents=True)
    for src in (original/'source-model').iterdir():
        if src.is_file():shutil.copyfile(src,session/'source-model'/src.name)
    for name in ('source_bridge_v2.so','dependency'):
        (session/name).symlink_to((original/name).resolve(),target_is_directory=(original/name).is_dir())
    xml=session/'source-model/humanoid_high_freq.xml';old=xml.read_text()
    token='<option timestep=".00125"/>';changed=f'<option timestep=".00125" impratio="{ratio:g}"/>'
    assert old.count(token)==1 and changed not in old
    xml.write_text(old.replace(token,changed));assert xml.read_text().replace(changed,token)==old
    (session/'slow-continuation').mkdir()
    a,b=World(original,helper),World(session,helper)
    try:
        a.restore(original/'slow-continuation/final.bin');entry=a.packet();b.install_cold(entry)
        assert a.l.lh_dim(a.h,6)==b.l.lh_dim(b.h,6)==0
        for key,shape in ((0,(28,)),(1,(27,)),(10,(17,)),(14,(21,6))):
            np.testing.assert_array_equal(a.arr(key,shape),b.arr(key,shape))
        np.testing.assert_array_equal(a.limits,b.limits)
        np.testing.assert_array_equal(a.read('lh_mass_matrix',(27,27)),b.read('lh_mass_matrix',(27,27)))
        assert a.friction_parameters()==b.friction_parameters()
        oa,ob=options(a,inspector),options(b,inspector)
        assert oa[2]==1. and ob[2]==ratio
        np.testing.assert_array_equal(np.delete(oa,2),np.delete(ob,2))
        assert oa[0]==213 and oa[1]==.00125 and oa[3]==0
        b.checkpoint(session/'slow-continuation/final.bin')
        entry_hash=state_sha(entry)
        assert entry_hash==selection['initialQposQvelSHA256']==load_preset()['initialQposQvelSHA256']
        receipt=dict(schema=1,experiment='contact-impratio-only',baselineImpratio=1.,candidateImpratio=ratio,
            baselineModelSHA256=sha(original/'source-model/humanoid_high_freq.xml'),modelSHA256=sha(xml),
            helperSHA256=sha(helper),initialQposQvelSHA256=entry_hash,
            coldCacheSHA256=sha(session/'slow-continuation/final.bin'),
            originalFullCacheImportedIntoCandidate=False,modelBoundColdCacheRebuilt=True,
            initialNativeCounter=0,physicsIntegrations=0,onlyXMLDifference=f'option impratio: absent(default1) -> {ratio:g}',
            originalAndCandidateOptions= [oa.tolist(),ob.tolist()],
            originalMuGeometryMassMotorJointLimitsPreserved=True,originalSolverSolrefSolimpConeAndClockPreserved=True,
            selectedPresetChanged=False,originalModelQualificationClaim=False,
            inspectorSHA256=sha(inspector),runnerSHA256=sha(__file__))
    finally:a.close();b.close()
    selection.update(variant=f'hip10-knee27_5-impratio{ratio:g}',baselineModelSHA256=receipt['baselineModelSHA256'],
        modelSHA256=receipt['modelSHA256'],contactImpedanceChanged=True,contactImpratio=ratio,
        originalModelQualificationClaim=False)
    (output/'selection.json').write_text(json.dumps(selection,indent=2)+'\n')
    (output/'contact-impedance-model.json').write_text(json.dumps(receipt,indent=2)+'\n')
    return session,helper,receipt


class ImpedanceSteps(CadencedSteps):
    def __init__(self,base,helper,output,ratio=RATIO):
        assert ratio in (2.,8.);self.contact_impratio=ratio
        super().__init__(base,helper,output,10,.18,'loaded-settle',False)
        self.code_at_start[Path(__file__).name]=sha(__file__)

    def save(self,reason):
        super().save(reason)
        if hasattr(self,'folder'):
            path=self.folder/'cadenced-step-config.json';config=json.loads(path.read_text())
            config['contactImpedanceExperiment']=dict(impratio=self.contact_impratio,baselineImpratio=1.,muChanged=False,
                originalModelQualificationClaim=False,separatelyModelBoundColdCache=True)
            path.write_text(json.dumps(config,indent=2)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--inspector',type=Path,required=True);p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--execute-prepared',action='store_true');p.add_argument('--impratio',type=float,choices=(2.,8.),default=RATIO);a=p.parse_args()
    assert not(a.prepare_only and a.execute_prepared)
    if a.execute_prepared:
        receipt=json.loads((a.output/'contact-impedance-model.json').read_text())
        session=a.output/'model-session';helper=Path(__file__).resolve().parents[2]/load_preset()['helper']
        assert receipt['candidateImpratio']==a.impratio
        assert receipt['modelSHA256']==sha(session/'source-model/humanoid_high_freq.xml')
        assert receipt['coldCacheSHA256']==sha(session/'slow-continuation/final.bin')
        assert receipt['helperSHA256']==sha(helper) and receipt['runnerSHA256']==sha(__file__)
        check=World(session,helper)
        try:
            check.restore(session/'slow-continuation/final.bin');assert check.l.lh_dim(check.h,6)==0
            assert options(check,a.inspector)[2]==a.impratio
        finally:check.close()
    else:session,helper,receipt=prepare_impedance(a.output,a.inspector,a.impratio)
    if a.prepare_only:
        print(json.dumps(receipt,indent=2));return
    live=ImpedanceSteps(session,helper,a.output/'run',a.impratio);live.paused=False;started=time.monotonic()
    try:
        while not live.finished and not live.failure:
            assert time.monotonic()-started<1200.,'1200s process cap';live.advance(4)
            if live.steps%800==0:print('progress',live.steps*.00125,'step',live.step_index+1,live.phase,'slip',live.slip,'quiet',live.quiet_run*.00125,flush=True)
        print(live.failure or f'ten18cm-plus5squiet-under-explicit-impratio{a.impratio:g}',flush=True)
    except Exception as e:live.failure=repr(e);raise
    finally:live.close()
    raise SystemExit(0 if live.finished else 1)


if __name__=='__main__':main()
