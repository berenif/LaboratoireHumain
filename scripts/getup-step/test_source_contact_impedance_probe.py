"""Pure/mocked preparation and retained-byte checks; no native integrations.

No World constructor, compiler, shared library or integration function is used.
Optional retained-artifact tests inspect file bytes and exercise the existing
foreign-cache guard on a World allocated without invoking its constructor.
"""
from contextlib import redirect_stdout, redirect_stderr
import hashlib
import inspect
import io
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

import source_contact_impedance_probe as probe


XML = ('<mujoco><option timestep=".00125"/>'
       '<worldbody><geom type="plane" size="1 1 .2" friction="1 .005 .0001"/>'
       '<body><joint range="-35 15"/><geom size=".027"/></body></worldbody>'
       '<actuator><motor gear="40" ctrlrange="-1 1"/></actuator></mujoco>')


class MockPreparation(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='impratio-pure-')
        self.root = Path(self.temporary.name)
        self.output = self.root/'experiment'
        self.helper = self.root/'helper.so'; self.helper.write_bytes(b'helper-stub')
        self.inspector = self.root/'inspector.so'; self.inspector.write_bytes(b'inspector-stub')
        self.entry = dict(qpos=np.arange(28, dtype=float).tolist(),
                          qvel=(np.arange(27, dtype=float)/100.).tolist())
        self.entry_hash = probe.state_sha(self.entry)
        self.worlds = []
        self.options_delta = None
        self.source_xml = XML

    def tearDown(self):
        self.temporary.cleanup()

    def fake_prepare(self, output):
        session = output/'model-session'
        (session/'source-model').mkdir(parents=True)
        (session/'source-model/humanoid_high_freq.xml').write_text(self.source_xml)
        (session/'source-model/included.xml').write_text('<mujoco><asset/></mujoco>')
        (session/'source_bridge_v2.so').write_bytes(b'bridge-stub')
        (session/'dependency').mkdir()
        (session/'dependency/library.bin').write_bytes(b'engine-stub')
        (session/'slow-continuation').mkdir()
        (session/'slow-continuation/final.bin').write_bytes(b'original-model-bound-cache')
        selection = dict(variant='hip10-knee27_5',
                         initialQposQvelSHA256=self.entry_hash,
                         modelSHA256=probe.sha(session/'source-model/humanoid_high_freq.xml'))
        (output/'selection.json').write_text(json.dumps(selection))
        return session, self.helper, selection

    def fake_world(self, base, helper):
        case = self
        class FakeWorld:
            def __init__(self):
                self.base = Path(base)
                self.helper = helper
                self.h = object()
                self.l = SimpleNamespace(lh_dim=lambda *_: 0)
                self.limits = np.tile([-1., 1.], (21, 1))
                self.arrays = {0: np.array(case.entry['qpos']),
                               1: np.array(case.entry['qvel']),
                               10: np.arange(17, dtype=float),
                               14: np.arange(126, dtype=float).reshape(21, 6)}
                self.restores, self.installs, self.checkpoints = [], [], []
                self.closed = False
            def restore(self, path):
                self.restores.append(Path(path))
                if self.base.name != 'baseline-model-session':
                    raise AssertionError('candidate must not import baseline full cache')
                case.assertEqual(Path(path), self.base/'slow-continuation/final.bin')
            def packet(self):
                return dict(qpos=self.arrays[0].tolist(), qvel=self.arrays[1].tolist())
            def install_cold(self, entry):
                self.installs.append(entry)
                case.assertEqual(entry, case.entry)
                self.arrays[0] = np.array(entry['qpos'])
                self.arrays[1] = np.array(entry['qvel'])
            def arr(self, key, shape):
                return self.arrays[key].reshape(shape)
            def read(self, name, shape):
                case.assertEqual(name, 'lh_mass_matrix')
                return np.eye(27)
            def friction_parameters(self):
                return {'compiledGeoms': [1., .005, .0001], 'contactCondim': 3}
            def checkpoint(self, path):
                self.checkpoints.append(Path(path))
                Path(path).write_bytes(b'fresh-cold-cache-for-'+bytes.fromhex(
                    probe.sha(self.base/'source-model/humanoid_high_freq.xml')))
            def close(self):
                self.closed = True
        world = FakeWorld(); self.worlds.append(world)
        return world

    def fake_options(self, world, inspector):
        self.assertEqual(inspector, self.inspector)
        result = np.arange(22, dtype=float)
        result[0], result[1], result[2], result[3] = 213., .00125, 1., 0.
        if world.base.name == 'model-session':
            option = ET.fromstring((world.base/'source-model/humanoid_high_freq.xml').read_text()).find('option')
            result[2] = float(option.attrib['impratio'])
            if self.options_delta is not None:
                result[self.options_delta] += 1.
        return result

    def prepare(self, ratio=None):
        with patch.object(probe, 'prepare', self.fake_prepare), \
             patch.object(probe, 'World', self.fake_world), \
             patch.object(probe, 'options', self.fake_options), \
             patch.object(probe, 'load_preset', return_value={
                 'initialQposQvelSHA256': self.entry_hash}):
            if ratio is None:
                return probe.prepare_impedance(self.output, self.inspector)
            return probe.prepare_impedance(self.output, self.inspector, ratio)

    def test_exactly_one_xml_attribute_changes_and_dependencies_are_shared(self):
        session, helper, receipt = self.prepare()
        baseline = self.output/'baseline-model-session'
        original = (baseline/'source-model/humanoid_high_freq.xml').read_text()
        candidate = (session/'source-model/humanoid_high_freq.xml').read_text()
        self.assertEqual(original, XML)
        self.assertEqual(candidate.replace('<option timestep=".00125" impratio="2"/>',
                                           '<option timestep=".00125"/>'), original)
        self.assertEqual(candidate.count('impratio="2"'), 1)
        self.assertEqual((baseline/'source-model/included.xml').read_bytes(),
                         (session/'source-model/included.xml').read_bytes())
        for name in ('source_bridge_v2.so', 'dependency'):
            self.assertTrue((session/name).is_symlink())
            self.assertEqual((session/name).resolve(), (baseline/name).resolve())
        self.assertEqual(helper, self.helper)
        self.assertNotEqual(receipt['modelSHA256'], receipt['baselineModelSHA256'])

    def test_candidate_only_installs_identical_cold_qpos_qvel_and_builds_own_cache(self):
        session, _, receipt = self.prepare()
        a, b = self.worlds
        self.assertEqual(len(a.restores), 1)
        self.assertFalse(a.installs)
        self.assertFalse(b.restores)
        self.assertEqual(b.installs, [self.entry])
        self.assertEqual(b.checkpoints, [session/'slow-continuation/final.bin'])
        self.assertTrue(all(w.closed for w in self.worlds))
        self.assertEqual(receipt['physicsIntegrations'], 0)
        self.assertEqual(receipt['initialNativeCounter'], 0)
        self.assertFalse(receipt['originalFullCacheImportedIntoCandidate'])
        self.assertTrue(receipt['modelBoundColdCacheRebuilt'])
        self.assertEqual(receipt['initialQposQvelSHA256'], self.entry_hash)

    def test_receipt_and_local_selection_explicitly_label_altered_model(self):
        session, _, receipt = self.prepare()
        saved = json.loads((self.output/'contact-impedance-model.json').read_text())
        selection = json.loads((self.output/'selection.json').read_text())
        self.assertEqual(saved, receipt)
        self.assertEqual(receipt['baselineImpratio'], 1.)
        self.assertEqual(receipt['candidateImpratio'], 2.)
        self.assertFalse(receipt['selectedPresetChanged'])
        self.assertFalse(receipt['originalModelQualificationClaim'])
        self.assertFalse(selection['originalModelQualificationClaim'])
        self.assertTrue(selection['contactImpedanceChanged'])
        self.assertEqual(selection['variant'], 'hip10-knee27_5-impratio2')
        self.assertEqual(selection['modelSHA256'], probe.sha(session/'source-model/humanoid_high_freq.xml'))
        self.assertEqual(receipt['coldCacheSHA256'], probe.sha(session/'slow-continuation/final.bin'))

    def test_explicit_eight_changes_only_option_and_records_its_own_provenance(self):
        session, _, receipt = self.prepare(8.)
        baseline = self.output/'baseline-model-session'
        candidate = (session/'source-model/humanoid_high_freq.xml').read_text()
        self.assertEqual(candidate.replace('<option timestep=".00125" impratio="8"/>',
                                           '<option timestep=".00125"/>'), XML)
        self.assertEqual((baseline/'source-model/humanoid_high_freq.xml').read_text(), XML)
        self.assertEqual(receipt['candidateImpratio'], 8.)
        self.assertEqual(receipt['onlyXMLDifference'], 'option impratio: absent(default1) -> 8')
        a, b = np.array(receipt['originalAndCandidateOptions'])
        self.assertEqual((a[2], b[2]), (1., 8.))
        np.testing.assert_array_equal(np.delete(a, 2), np.delete(b, 2))
        selection = json.loads((self.output/'selection.json').read_text())
        self.assertEqual(selection['variant'], 'hip10-knee27_5-impratio8')
        self.assertEqual(selection['contactImpratio'], 8.)
        self.assertFalse(selection['originalModelQualificationClaim'])
        self.assertFalse(self.worlds[1].restores)
        self.assertEqual(self.worlds[1].installs, [self.entry])
        self.assertEqual(receipt['initialNativeCounter'], 0)

    def test_unapproved_ratios_are_rejected_before_any_preparation(self):
        for ratio in (1., 5., 2.0001, 8.0001, float('nan'), float('inf')):
            with self.subTest(ratio=ratio):
                with self.assertRaises(AssertionError):
                    self.prepare(ratio)
                self.assertFalse(self.output.exists())
                self.assertFalse(self.worlds)

    def test_any_second_compiled_option_change_is_rejected_before_cache_save(self):
        self.options_delta = 3  # friction cone changed in addition to impratio
        with self.assertRaises(AssertionError):
            self.prepare()
        self.assertTrue(all(w.closed for w in self.worlds))
        self.assertFalse(self.worlds[1].checkpoints)
        self.assertFalse((self.output/'contact-impedance-model.json').exists())

    def test_unexpected_or_duplicate_option_token_is_rejected(self):
        for xml in (XML.replace('<option timestep=".00125"/>', ''),
                    XML.replace('<option timestep=".00125"/>',
                                '<option timestep=".00125"/><option timestep=".00125"/>'),
                    XML.replace('<option timestep=".00125"/>',
                                '<option timestep=".00125" impratio="2"/>')):
            with self.subTest(xml=xml), tempfile.TemporaryDirectory(prefix='impratio-invalid-') as folder:
                self.output = Path(folder)/'experiment'
                self.source_xml = xml
                with self.assertRaises(AssertionError):
                    self.prepare()
                self.assertFalse(self.worlds)


class ExplicitRunConfiguration(unittest.TestCase):
    def test_runner_keeps_ten_180mm_loaded_settle_steps_and_legacy_arm_mode(self):
        captured = []
        def init(instance, *args):
            captured.append(args); instance.code_at_start = {}
        with patch.object(probe.CadencedSteps, '__init__', init):
            live = probe.ImpedanceSteps(Path('/unused/model'), Path('/unused/helper'), Path('/unused/run'))
        self.assertEqual(captured, [(Path('/unused/model'), Path('/unused/helper'), Path('/unused/run'),
                                    10, .18, 'loaded-settle', False)])
        self.assertIn('source_contact_impedance_probe.py', live.code_at_start)
        self.assertEqual(live.contact_impratio, 2.)
        self.assertEqual(inspect.signature(probe.prepare_impedance).parameters['ratio'].default, 2.)
        self.assertEqual(inspect.signature(probe.ImpedanceSteps.__init__).parameters['ratio'].default, 2.)

    def test_explicit_eight_keeps_identical_cadence_arguments(self):
        captured = []
        def init(instance, *args):
            captured.append(args); instance.code_at_start = {}
        with patch.object(probe.CadencedSteps, '__init__', init):
            live = probe.ImpedanceSteps(Path('/unused/model8'), Path('/unused/helper'), Path('/unused/run8'), 8.)
        self.assertEqual(captured, [(Path('/unused/model8'), Path('/unused/helper'), Path('/unused/run8'),
                                    10, .18, 'loaded-settle', False)])
        self.assertEqual(live.contact_impratio, 8.)

    def test_saved_config_marks_experiment_without_overwriting_existing_gates(self):
        with tempfile.TemporaryDirectory(prefix='impratio-save-') as folder:
            live = probe.ImpedanceSteps.__new__(probe.ImpedanceSteps); live.folder = Path(folder)
            live.contact_impratio = probe.RATIO
            original = dict(finalQuietRequiredS=5., globalLoadedSlipLimitM=.01,
                            originalContactJointMotorCollisionCaptureGates=True)
            def save(instance, reason):
                (instance.folder/'cadenced-step-config.json').write_text(json.dumps(original))
            with patch.object(probe.CadencedSteps, 'save', save):
                live.save('test-only')
            result = json.loads((live.folder/'cadenced-step-config.json').read_text())
            for key, value in original.items():
                self.assertEqual(result[key], value)
            experiment = result['contactImpedanceExperiment']
            self.assertEqual(experiment['impratio'], 2.)
            self.assertFalse(experiment['muChanged'])
            self.assertFalse(experiment['originalModelQualificationClaim'])
            self.assertTrue(experiment['separatelyModelBoundColdCache'])

    def test_explicit_eight_saved_config_labels_eight_without_changing_gates(self):
        with tempfile.TemporaryDirectory(prefix='impratio8-save-') as folder:
            live = probe.ImpedanceSteps.__new__(probe.ImpedanceSteps)
            live.folder, live.contact_impratio = Path(folder), 8.
            original = dict(finalQuietRequiredS=5., globalLoadedSlipLimitM=.01,
                            originalContactJointMotorCollisionCaptureGates=True)
            def save(instance, reason):
                (instance.folder/'cadenced-step-config.json').write_text(json.dumps(original))
            with patch.object(probe.CadencedSteps, 'save', save):
                live.save('test-only')
            result = json.loads((live.folder/'cadenced-step-config.json').read_text())
            self.assertEqual({key: result[key] for key in original}, original)
            self.assertEqual(result['contactImpedanceExperiment']['impratio'], 8.)
            self.assertFalse(result['contactImpedanceExperiment']['muChanged'])
            self.assertFalse(result['contactImpedanceExperiment']['originalModelQualificationClaim'])

    def test_prepare_only_cannot_construct_a_live_runner(self):
        args = ['probe', '--output', '/unused/output', '--inspector', '/unused/inspector', '--prepare-only']
        receipt = {'physicsIntegrations': 0}
        with patch('sys.argv', args), \
             patch.object(probe, 'prepare_impedance', return_value=(Path('/unused/model'), Path('/unused/helper'), receipt)), \
             patch.object(probe, 'ImpedanceSteps', side_effect=AssertionError('must not launch')), \
             redirect_stdout(io.StringIO()) as output:
            probe.main()
        self.assertEqual(json.loads(output.getvalue()), receipt)

    def test_cli_explicit_eight_prepare_only_passes_eight_without_running(self):
        args = ['probe', '--output', '/unused/output8', '--inspector', '/unused/inspector',
                '--impratio', '8', '--prepare-only']
        receipt = {'candidateImpratio': 8., 'physicsIntegrations': 0}
        with patch('sys.argv', args), \
             patch.object(probe, 'prepare_impedance', return_value=(Path('/unused/model8'), Path('/unused/helper'), receipt)) as prepare, \
             patch.object(probe, 'ImpedanceSteps', side_effect=AssertionError('must not launch')), \
             redirect_stdout(io.StringIO()) as output:
            probe.main()
        prepare.assert_called_once_with(Path('/unused/output8'), Path('/unused/inspector'), 8.)
        self.assertEqual(json.loads(output.getvalue()), receipt)

    def test_cli_rejects_five_without_creating_a_model(self):
        args = ['probe', '--output', '/unused/output5', '--inspector', '/unused/inspector',
                '--impratio', '5', '--prepare-only']
        with patch('sys.argv', args), patch.object(probe, 'prepare_impedance') as prepare, \
             redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as caught:
            probe.main()
        self.assertEqual(caught.exception.code, 2)
        prepare.assert_not_called()


PREPARED = Path('/tmp/cadence-impratio-2')
PREPARED_EIGHT = Path('/tmp/cadence-impratio-8')


class _PreparedIdentityChecks:
    def test_real_model_pair_differs_only_in_impratio_option(self):
        a = self.prepared/'baseline-model-session/source-model'
        b = self.prepared/'model-session/source-model'
        self.assertEqual({p.name for p in a.iterdir()}, {p.name for p in b.iterdir()})
        for path in a.iterdir():
            if path.name == 'humanoid_high_freq.xml':
                candidate = (b/path.name).read_text()
                self.assertEqual(candidate.replace(f'<option timestep=".00125" impratio="{self.ratio:g}"/>',
                                                   '<option timestep=".00125"/>'), path.read_text())
            else:
                self.assertEqual((b/path.name).read_bytes(), path.read_bytes())

    def test_actual_cache_hashes_bind_each_cache_to_its_own_model_and_engine(self):
        receipt = json.loads((self.prepared/'contact-impedance-model.json').read_text())
        self.assertEqual(receipt['candidateImpratio'], self.ratio)
        self.assertFalse(receipt['originalModelQualificationClaim'])
        bindings = []
        for name in ('baseline-model-session', 'model-session'):
            session = self.prepared/name
            blob = (session/'slow-continuation/final.bin').read_bytes()
            binding = b''.join(bytes.fromhex(probe.sha(path)) for path in (
                session/'source-model/humanoid_high_freq.xml', session/'source_bridge_v2.so',
                session/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3'))
            self.assertEqual(blob[:8], b'LHSC002!')
            self.assertEqual(blob[8:104], binding)
            self.assertEqual(blob[104:136], hashlib.sha256(blob[136:]).digest())
            bindings.append(blob[8:40])
        self.assertNotEqual(bindings[0], bindings[1])
        self.assertEqual(bindings[1].hex(), receipt['modelSHA256'])
        self.assertEqual(probe.sha(self.prepared/'model-session/slow-continuation/final.bin'), receipt['coldCacheSHA256'])

    def test_real_restore_rejects_baseline_cache_before_any_native_call(self):
        # Allocate a World without __init__: a poisoned native interface proves
        # binding rejection happens before any cache import or native access.
        class NoNative:
            def __getattr__(self, name):
                raise RuntimeError('unexpected native call: '+name)
        world = object.__new__(probe.World)
        world.base = self.prepared/'model-session'
        world.xml = world.base/'source-model/humanoid_high_freq.xml'
        world.l, world.h = NoNative(), None
        with self.assertRaises(AssertionError):
            world.restore(self.prepared/'baseline-model-session/slow-continuation/final.bin')


@unittest.skipUnless((PREPARED/'contact-impedance-model.json').is_file(),
                     'optional retained prepared ratio2 comparison absent')
class RetainedPreparedIdentity(_PreparedIdentityChecks, unittest.TestCase):
    prepared, ratio = PREPARED, 2.

    def test_old_ratio_two_launcher_keeps_its_original_frozen_receipt_identity(self):
        frozen = self.prepared/'frozen-source/source_contact_impedance_probe.py'
        if not frozen.exists():
            self.skipTest('optional frozen ratio2 launcher absent')
        receipt = json.loads((self.prepared/'contact-impedance-model.json').read_text())
        expected = '51c5dca0f9a11c723e3137551b1f7573e14b60ac0a93d503ccbb8982ad0eec21'
        self.assertEqual(probe.sha(frozen), expected)
        self.assertEqual(receipt['runnerSHA256'], expected)


@unittest.skipUnless((PREPARED_EIGHT/'contact-impedance-model.json').is_file(),
                     'optional retained prepared ratio8 comparison absent')
class RetainedPreparedEightIdentity(_PreparedIdentityChecks, unittest.TestCase):
    prepared, ratio = PREPARED_EIGHT, 8.

    def test_prepared_eight_receipt_matches_current_runner_without_rebinding_old_two(self):
        receipt = json.loads((self.prepared/'contact-impedance-model.json').read_text())
        self.assertEqual(receipt['runnerSHA256'], probe.sha(Path(probe.__file__)))
        a, b = np.array(receipt['originalAndCandidateOptions'])
        self.assertEqual((a[2], b[2]), (1., 8.))
        np.testing.assert_array_equal(np.delete(a, 2), np.delete(b, 2))
        selection = json.loads((self.prepared/'selection.json').read_text())
        self.assertEqual(selection['variant'], 'hip10-knee27_5-impratio8')
        self.assertFalse(selection['originalModelQualificationClaim'])


if __name__ == '__main__':
    unittest.main()
