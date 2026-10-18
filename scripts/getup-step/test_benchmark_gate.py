import json,tempfile,unittest
from pathlib import Path
try:import torch
except ImportError:torch=None
@unittest.skipIf(torch is None,'existing Torch runtime')
class BenchmarkGateTests(unittest.TestCase):
    def test_failed500_gate_rejects_before_world_or_budget_admission(self):
        import benchmark_stabilization as b
        with tempfile.TemporaryDirectory()as folder:
            stage=Path(folder);(stage/'fit-500-summary.json').write_text(json.dumps(dict(admitted=False,reconstructionBitExact=True)));(stage/'source200-entry-preflight.json').write_text(json.dumps(dict(passed=True)))
            with self.assertRaisesRegex(ValueError,'prerequisites failed'):b.prepare(stage,stage/'source-does-not-exist')
            self.assertFalse((stage/'ledger.json').exists());self.assertFalse((stage/'benchmark-preflight.json').exists())
if __name__=='__main__':unittest.main()
