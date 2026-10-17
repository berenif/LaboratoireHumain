import datetime as dt
import json
from pathlib import Path
import tempfile
import unittest
from native_env import PilotLedger, atomic_json

class LedgerTests(unittest.TestCase):
    def make(self, path):
        now=dt.datetime.now(dt.timezone.utc)
        atomic_json(path,dict(startedUtc=now.isoformat(),deadlineUtc=(now+dt.timedelta(hours=1)).isoformat(),
             newAggregatePhysicsLimitSeconds=300,newPhysicsWallSeconds=0.,newNativeSubsteps=0,
             stagePhysicsCeilings=dict(knownSourceParity=10,frozenReference=50,dataLearning=120,validationReserved=100,headroom=20)))
        return PilotLedger(path)
    def test_stage_and_validation_reserve_admission(self):
        with tempfile.TemporaryDirectory() as d:
            ledger=self.make(Path(d)/'ledger.json');ledger.admit('knownSourceParity',10.)
            with self.assertRaises(ValueError):ledger.admit('knownSourceParity',10.001)
            ledger.charge('dataLearning',120.,800);ledger.admit('frozenReference',50.)
            ledger.charge('frozenReference',50.,100)
            with self.assertRaises(ValueError):ledger.admit('headroom',31.)
            ledger.admit('validationReserved',90.)
            ledger.close();reloaded=PilotLedger(ledger.path);self.assertEqual(reloaded.data['newNativeSubsteps'],900);reloaded.close()
    def test_counters_never_reset_or_go_backwards(self):
        with tempfile.TemporaryDirectory() as d:
            ledger=self.make(Path(d)/'ledger.json');ledger.charge('knownSourceParity',.1,20)
            with self.assertRaises(ValueError):ledger.charge('knownSourceParity',-.1,0)
            ledger.close();loaded=PilotLedger(ledger.path);self.assertEqual(loaded.data['newPhysicsWallSeconds'],.1)
    def test_expired_wall_deadline_rejects_admission(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'ledger.json';ledger=self.make(path);data=ledger.data
            data['deadlineUtc']='2020-01-01T00:00:00Z';atomic_json(path,data);ledger.close()
            with self.assertRaises(TimeoutError):PilotLedger(path)

# Fake children exercise the transport/accounting failure paths with ZERO native
# integration or external executable. unittest discovery imports this file.
import io
from unittest.mock import patch
from native_env import SourceNative

class FakeChild:
    def __init__(self,rows,exception=None,dead=False):
        self.pid=123456789;self.returncode=0 if dead else None;self.stdin=io.StringIO()
        self.stdout=io.StringIO(''.join(x if isinstance(x,str)else json.dumps(x)+'\n'for x in rows))
        if exception is not None:
            class BadRead:
                def readline(self):raise exception
            self.stdout=BadRead()
        self.kills=0;self.waits=0
    def poll(self):return self.returncode
    def kill(self):
        import signal
        assert signal.getitimer(signal.ITIMER_REAL)[0]==0
        self.kills+=1;self.returncode=-9
    def wait(self,timeout=None):self.waits+=1;self.returncode=0 if self.returncode is None else self.returncode;return self.returncode

class TransportTests(unittest.TestCase):
    make=LedgerTests.make
    def session(self,root,child):
        ledger=self.make(root/'ledger.json');binary=root/'binary';seed=root/'seed';binary.write_bytes(b'fake');seed.write_bytes(b'fake')
        with patch('native_env.subprocess.Popen',return_value=child):
            native=SourceNative(binary,seed,root/'out',ledger,stage='knownSourceParity',physics_cap=1.)
        return ledger,native,binary,seed
    def test_success_and_error_replies_charge_once(self):
        rows=[dict(ok=True,result={'x':1},processPhysicsWallS=.1,processNativeSubsteps=20),dict(ok=False,error='known',processPhysicsWallS=.2,processNativeSubsteps=40)]
        with tempfile.TemporaryDirectory()as d:
            ledger,n,*_=self.session(Path(d),FakeChild(rows))
            try:
                n.command(dict(op='stepSourceTeacher',controls=[0]*21));n.command(dict(op='stepSourceTeacher',controls=[2]*21),expect_ok=False)
                self.assertEqual(ledger.data['newPhysicsWallSeconds'],.2);self.assertEqual(ledger.data['newNativeSubsteps'],40)
                self.assertFalse(ledger.data['uncertainRequests']);self.assertIsNone(n.record['requestOutstanding'])
            finally:n.close();ledger.close()
    def test_nonfinite_request_writes_nothing(self):
        with tempfile.TemporaryDirectory()as d:
            child=FakeChild([]);ledger,n,*_=self.session(Path(d),child)
            try:
                with self.assertRaises(ValueError):n.command(dict(op='stepSourceTeacher',controls=[float('nan')]*21))
                self.assertEqual(child.stdin.getvalue(),'');self.assertEqual(ledger.data['newNativeSubstepsUpperBound'],0)
            finally:n.close();ledger.close()
    def test_malformed_missing_nan_and_backward_counter_charge_uncertainty(self):
        invalid=['not-json\n',dict(ok=True),dict(ok=True,processPhysicsWallS=float('nan'),processNativeSubsteps=20),dict(ok=True,processPhysicsWallS=-1,processNativeSubsteps=0)]
        for row in invalid:
            with self.subTest(row=row),tempfile.TemporaryDirectory()as d:
                child=FakeChild([row]);ledger,n,*_=self.session(Path(d),child)
                try:
                    with self.assertRaises((ValueError,KeyError)):n.command(dict(op='stepSourceTeacher',controls=[0]*21))
                    self.assertEqual(len(ledger.data['uncertainRequests']),1);self.assertEqual(ledger.data['newNativeSubstepsUpperBound'],20)
                    self.assertGreater(ledger.data['newPhysicsWallSeconds'],0);self.assertGreater(child.waits,0)
                finally:n.close();ledger.close()
    def test_native_counter_cannot_rewind_after_known_integration(self):
        rows=[dict(ok=True,result={},processPhysicsWallS=.1,processNativeSubsteps=20),dict(ok=True,result={},processPhysicsWallS=.2,processNativeSubsteps=0)]
        with tempfile.TemporaryDirectory()as d:
            ledger,n,*_=self.session(Path(d),FakeChild(rows))
            try:
                n.command(dict(op='stepSourceTeacher',controls=[0]*21))
                with self.assertRaises(ValueError):n.command(dict(op='stepSourceTeacher',controls=[0]*21))
                self.assertEqual(ledger.data['newNativeSubsteps'],20);self.assertEqual(ledger.data['newNativeSubstepsUpperBound'],40)
            finally:n.close();ledger.close()
    def test_interruption_and_already_dead_child_are_reaped_and_charged(self):
        for exception in [KeyboardInterrupt(),SystemExit(3),None]:
            with self.subTest(exception=exception),tempfile.TemporaryDirectory()as d:
                child=FakeChild([],exception=exception,dead=exception is None);ledger,n,*_=self.session(Path(d),child)
                try:
                    with self.assertRaises(BaseException):n.command(dict(op='stepSourceTeacher',controls=[0]*21))
                    self.assertEqual(len(ledger.data['uncertainRequests']),1);self.assertEqual(ledger.data['newNativeSubstepsUpperBound'],20)
                finally:n.close();ledger.close()
    def test_exclusive_ledger_and_process_handle(self):
        with tempfile.TemporaryDirectory()as d:
            root=Path(d);ledger,n,binary,seed=self.session(root,FakeChild([]))
            try:
                with self.assertRaises(RuntimeError):PilotLedger(ledger.path)
                with self.assertRaises(RuntimeError):SourceNative(binary,seed,root/'second',ledger,stage='knownSourceParity',physics_cap=1.)
            finally:n.close();ledger.close()
    def test_failure_trace_error_cannot_skip_reaping_or_charging(self):
        class BadTrace:
            def write(self,*_):raise OSError('disk full')
            def flush(self):raise OSError('disk full')
        with tempfile.TemporaryDirectory()as d:
            child=FakeChild(['bad-json\n']);ledger,n,*_=self.session(Path(d),child);original=n.trace;n.trace=BadTrace()
            try:
                with self.assertRaises(ValueError):n.command(dict(op='stepSourceTeacher',controls=[0]*21))
                self.assertGreater(child.waits,0);self.assertEqual(len(ledger.data['uncertainRequests']),1)
                self.assertEqual(ledger.data['newNativeSubstepsUpperBound'],20)
            finally:n.trace=original;n.close();ledger.close()
    def test_unresolved_marker_blocks_restart_even_if_status_closed(self):
        with tempfile.TemporaryDirectory()as d:
            ledger=self.make(Path(d)/'ledger.json');ledger.data['processes']=[dict(status='closed',requestOutstanding={'op':'step'})];ledger.save();ledger.close()
            with self.assertRaisesRegex(RuntimeError,'reconciliation'):PilotLedger(ledger.path)
    def test_unfinished_process_prevents_restart_without_reconciliation(self):
        with tempfile.TemporaryDirectory()as d:
            ledger=self.make(Path(d)/'ledger.json');ledger.data['processes']=[dict(status='running',requestOutstanding=dict(maximumAdditionalNativeSubsteps=20))];ledger.save();ledger.close()
            with self.assertRaisesRegex(RuntimeError,'reconciliation'):PilotLedger(ledger.path)

if __name__=='__main__':unittest.main()
