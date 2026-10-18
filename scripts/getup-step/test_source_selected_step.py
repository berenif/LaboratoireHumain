"""Selected recipe routing, cache identity and controller parity; zero steps."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import source_selected_step as selected
from source_step_probe import World, ROOT
from source_arm_knee_mobility import ArmKneeMultiStep, native_bytes
from source_multistep_probe import MultiStep

BASE = ROOT/'outputs/getup-source-engine/session-20261017T233404Z'
HELPER = ROOT/'outputs/getup-source-step/first-liftoff/source_dynamics-v9.so'
WIDER = ROOT/'outputs/getup-source-step/hip-mobility-pair01/wider-session'


class Selection(unittest.TestCase):
    def test_default_cli_selects_combined_and_keeps_explicit_baseline(self):
        preset = selected.load_preset()
        args = selected.parser().parse_args(['--output','new-run'])
        self.assertEqual(selected.resolve_variant(args.variant,preset),'hip10-knee27_5')
        self.assertEqual(selected.resolve_variant('baseline',preset),'baseline')
        with self.assertRaises(AssertionError):
            selected.resolve_variant('arms',preset)

    def test_dispatch_uses_existing_runners_without_enabling_arms(self):
        with patch.object(selected,'ArmKneeMultiStep') as combined, patch.object(selected,'MultiStep') as baseline:
            selected.create_live('session','helper','run','hip10-knee27_5')
            combined.assert_called_once_with('session','helper','run','knees')
            baseline.assert_not_called()
            selected.create_live('session','helper','run','baseline')
            baseline.assert_called_once_with('session','helper','run',(.03,.06,.09,.045),True,2.)


@unittest.skipUnless(BASE.exists() and HELPER.exists() and WIDER.exists(),'retained native assets absent')
class NativeSelection(unittest.TestCase):
    def test_model_cache_binding_and_cold_state_match_validated_recipe(self):
        model_before = (BASE/'source-model/humanoid_high_freq.xml').read_bytes()
        cache_before = (BASE/'slow-continuation/final.bin').read_bytes()
        with tempfile.TemporaryDirectory() as tmp:
            session,helper,receipt = selected.prepare(Path(tmp)/'selected')
            self.assertEqual(receipt['variant'],'hip10-knee27_5')
            self.assertFalse(receipt['armsExperimentEnabled'])
            self.assertEqual((session/'slow-continuation/final.bin').read_bytes()[:104],(WIDER/'slow-continuation/final.bin').read_bytes()[:104])
            w = World(session,helper)
            try:
                w.restore(session/'slow-continuation/final.bin'); before = native_bytes(w)
                reference = World(WIDER,helper)
                try:
                    reference.restore(WIDER/'slow-continuation/final.bin')
                    for kind,shape in [(0,(28,)),(1,(27,)),(3,(17,3)),(14,(21,6))]:
                        np.testing.assert_array_equal(w.arr(kind,shape),reference.arr(kind,shape))
                    np.testing.assert_array_equal(w.read('lh_mass_matrix',(27,27)),reference.read('lh_mass_matrix',(27,27)))
                    np.testing.assert_array_equal(w.read('lh_bias',(27,)),reference.read('lh_bias',(27,)))
                    self.assertEqual(w.friction_parameters(),reference.friction_parameters())
                finally:
                    reference.close()
                with self.assertRaises(AssertionError):
                    w.restore(BASE/'slow-continuation/final.bin')
                self.assertEqual(native_bytes(w),before)
                self.assertEqual(w.l.lh_dim(w.h,6),0)
            finally:
                w.close()
        self.assertEqual((BASE/'source-model/humanoid_high_freq.xml').read_bytes(),model_before)
        self.assertEqual((BASE/'slow-continuation/final.bin').read_bytes(),cache_before)

    def test_default_selected_and_baseline_commands_equal_existing_validated_runners(self):
        with tempfile.TemporaryDirectory() as tmp:
            for variant,reference_base in [('selected',WIDER),('baseline',BASE)]:
                with self.subTest(variant=variant):
                    output = Path(tmp)/variant
                    session,helper,receipt = selected.prepare(output,variant)
                    live = selected.create_live(session,helper,output/'run',receipt['variant'])
                    reference = (ArmKneeMultiStep(reference_base,helper,output/'reference','knees') if variant == 'selected'
                                 else MultiStep(reference_base,helper,output/'reference',(.03,.06,.09,.045),True,2.))
                    try:
                        for t in (0.,.5):
                            a,ai = live.controller.solve(t,'transfer'); b,bi = reference.controller.solve(t,'transfer')
                            np.testing.assert_array_equal(a,b)
                            self.assertEqual(ai,bi)
                        if variant == 'selected':
                            self.assertTrue(live.controller.knees_enabled)
                            self.assertFalse(live.controller.arms_enabled)
                        self.assertEqual(live.world.l.lh_dim(live.world.h,6),0)
                        self.assertEqual(reference.world.l.lh_dim(reference.world.h,6),0)
                    finally:
                        live.close(); reference.close()


if __name__ == '__main__':
    unittest.main()
