#!/usr/bin/env python3
"""Probe option-wiring smoke tests; Native construction is stubbed, no physics."""
import datetime,importlib.util,sys,tempfile,unittest
import numpy as np
from pathlib import Path
from unittest.mock import patch
sp=importlib.util.spec_from_file_location('probe_cli',Path(__file__).with_name('probe-getup-contact-feedback.py'));cli=importlib.util.module_from_spec(sp);sp.loader.exec_module(cli)
class NativeConstructionReached(Exception):pass
class ProbeCLITests(unittest.TestCase):
 def test_step_stance_target_preserves_planned_height(self):
  class Refs:
   def reference(self,progress):return None,None,np.array([.1,.80006124,.2])
  goal=cli.stance_com_target([0.,.81,0.],[1.,0.,0.],.1,Refs());np.testing.assert_array_equal(goal,[.1,.80006124,.2]);np.testing.assert_array_equal(cli.stance_com_target([0.,.81,0.],[1.,0.,0.],.1),[.1,.81,0.])
 def test_optional_future_reserve_reaches_native_boundary_without_attribute_error(self):
  for extra in [[],['--swing-clearance-reserve-m','.00025','--whole-capsule-swing-guard','--balance-guard','--capture-governor','--recovery-step','--tracking-priority','balance-first','--qp-solver','clarabel'],['--track-arms','--posture-tracking-band'],['--joint-stop-mode','native-bound-release'],['--tracking-priority','physical-first','--friction-mode','active-sticking','--contact-mode','unilateral-soft'],['--joint-reserve-recovery','--joint-viability','--joint-reserve-margin-rad','.0001'],['--joint-viability','--joint-reserve-margin-rad','.0001'],['--force-jacobian','native-frozen'],['--terminal-unload','--stage-unload-after-readiness','--recovery-step','--unload-left','--motion-profile']]:
   with self.subTest(options=extra),tempfile.TemporaryDirectory() as tmp:
    deadline=(datetime.datetime.now(datetime.timezone.utc)+datetime.timedelta(minutes=1)).isoformat()
    args=['probe','--binary','unused-binary','--checkpoint','unused-checkpoint','--output',str(Path(tmp)/'probe'),'--physics-budget','0','--deadline',deadline,'--flat-soles','--retain-balance','--seconds','8']+extra
    with patch.object(sys,'argv',args),patch.object(cli.probe,'Native',side_effect=NativeConstructionReached) as native:
     with self.assertRaises(NativeConstructionReached):cli.main()
     native.assert_called_once()
if __name__=='__main__':unittest.main()
