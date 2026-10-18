import copy,json,unittest
from dataclasses import asdict
from unittest.mock import patch
from pathlib import Path
import numpy as np
from source_observer import model_context,ValidatedSample
from slow_task import SourceSLOWCore

ROOT=Path(__file__).resolve().parents[2]
class ObserverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.row=json.loads((ROOT/'docs/fixtures/getup-step/source-SLOW-final-frame.json').read_text())['frame'];cls.model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())
    def rows(self):
        rows=[]
        for n in range(1,21):
            row=copy.deepcopy(self.row);row['sourceStep']+=n;row['timeS']+=n*.00125;row['loadedSlipSpeedMps']=[0.,0.];rows.append(row)
        return rows
    def test_shared_model_is_owned_immutable_and_hash_bound(self):
        supplied=copy.deepcopy(self.model);a=SourceSLOWCore(self.row,supplied);b=SourceSLOWCore(self.row,self.model)
        self.assertIs(a._context,b._context);self.assertIs(copy.deepcopy(a)._context,a._context)
        with self.assertRaises(TypeError):a.model['segments'][0]['massKg']=0
        with self.assertRaises(TypeError):a.model['segments']+=(1,)
        old=a.model['segments'][0]['massKg'];supplied['segments'][0]['massKg']+=1
        self.assertEqual(a.model['segments'][0]['massKg'],old);self.assertIsNot(model_context(supplied),a._context)
    def test_validated_row_token_hash_model_and_readonly_metrics(self):
        core=SourceSLOWCore(self.row,self.model);rows=self.rows();samples=[core.validate(r)for r in rows];before=copy.deepcopy(core.__dict__)
        with self.assertRaises(ValueError):ValidatedSample({},'',core._context,None)
        with self.assertRaises(ValueError):samples[0].metrics['physical'][0]=0
        with self.assertRaises(ValueError):samples[0].metrics['physical'].setflags(write=True)
        with self.assertRaises(TypeError):samples[0].native_id=0
        rows[0]['timeS']+=.00125
        with self.assertRaises(ValueError):core.consume(rows,validated_samples=samples)
        self.assertEqual(core.native_id,before['native_id']);self.assertEqual(core.quiet_run,before['quiet_run']);np.testing.assert_array_equal(core.slip,before['slip'])
        rows=self.rows();swapped=samples[::-1]
        with self.assertRaises(ValueError):core.consume(rows,validated_samples=swapped)
        foreign=copy.deepcopy(self.model);foreign['segments'][0]['massKg']+=.01;context=model_context(foreign)
        from source_observer import checked_metrics
        with self.assertRaises(ValueError):checked_metrics(samples[0],rows[0],context)
    def test_preview_and_consume_rollback_mutable_schema(self):
        core=SourceSLOWCore(self.row,self.model);trial=core.preview_state();trial.slip+=1;trial.feet_run[0]+=9;trial.quiet_run+=5
        np.testing.assert_array_equal(core.slip,[0,0]);self.assertEqual(core.feet_run,[0,0]);self.assertEqual(core.quiet_run,0)
        rows=self.rows();before=copy.deepcopy(core.__dict__);samples=[core.validate(r)for r in rows]
        from task import TaskHistory
        original=TaskHistory.evaluate
        def fail(history,*a,**k):
            original(history,*a,**k)
            raise RuntimeError('post-history-mutation fixture')
        with patch.object(TaskHistory,'evaluate',fail):
            with self.assertRaises(RuntimeError):core.consume(rows,validated_samples=samples)
        for key in core.MUTABLE_CONSUME_FIELDS:
            if key=='history':continue
            if isinstance(before[key],np.ndarray):np.testing.assert_array_equal(getattr(core,key),before[key])
            else:self.assertEqual(getattr(core,key),before[key])
        self.assertEqual(asdict(core.history),asdict(before['history']));self.assertIs(core._context,before['_context']);self.assertIs(core.model,before['model'])
    def test_decode_reuse_and_unoptimized_api_equal(self):
        a=SourceSLOWCore(self.row,self.model);b=SourceSLOWCore(self.row,self.model);rows=self.rows();samples=[a.validate(r)for r in rows]
        self.assertEqual(a.consume(rows,validated_samples=samples),b.consume(rows))

if __name__=='__main__':unittest.main()
