#!/usr/bin/env python3
"""Bounded OS-only supervision checks. Compiles an inert C fixture, never Rust or physics."""
import ctypes
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('prototype_runner_inert_check', ROOT / 'scripts/run-coordination-prototype.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
# Reap deliberately forked inert grandchildren locally if the group leader dies.
assert ctypes.CDLL(None).prctl(36, 1, 0, 0, 0) == 0  # PR_SET_CHILD_SUBREAPER
results = []
started = time.monotonic()
with tempfile.TemporaryDirectory(prefix='lh-local-pair-inert-supervision-') as temporary:
    temporary = Path(temporary)
    binary = temporary / 'inert-process'
    subprocess.run(['cc', '-O0', '-pthread', str(EVIDENCE / 'process-supervision-inert.c'), '-o', str(binary)], check=True)
    for case in ('normal', 'alarm', 'sigterm', 'constructor-alarm', 'thread', 'child'):
        os.environ['LH_INERT_MODE'] = case if case in ('thread', 'child') else 'hang' if case != 'normal' else 'normal'
        output = temporary / case
        output.mkdir()
        worker = None
        constructed = []
        error = None
        per_case = time.monotonic()
        try:
            with runner.hard_deadline(per_case + 0.3, lambda: worker):
                if case == 'constructor-alarm':
                    constructed.append(runner.NativeWorker(binary, output))
                    time.sleep(1)  # signal must interrupt before assignment
                    raise AssertionError('Constructor alarm did not fire')
                worker = runner.NativeWorker(binary, output)
                constructed.append(worker)
                ready = worker.receive(per_case + 0.3)
                assert ready == {'event': 'ready', 'pid': worker.pid}
                if case == 'alarm':
                    time.sleep(1)
                    raise AssertionError('Alarm did not fire')
                if case == 'sigterm':
                    os.kill(os.getpid(), signal.SIGTERM)
                    raise AssertionError('SIGTERM did not interrupt')
                if case in ('thread', 'child'):
                    raise AssertionError('Forbidden process shape was accepted')
                assert worker.stop() == 0
        except runner.RunnerError as exc:
            error = str(exc)
        finally:
            for process in constructed:
                if not process.process.stdout.closed:
                    process.stop(error or 'inert harness cleanup')
        # Reap only descendants of this harness; no unrelated process is touched.
        while True:
            try:
                pid, _ = os.waitpid(-1, os.WNOHANG)
                if pid == 0:
                    raise AssertionError('An inert descendant is still live')
            except ChildProcessError:
                break
        assert not runner.supervision.child_pids(), 'An inert descendant remains'
        assert all(not Path('/proc', str(p.pid)).exists() for p in constructed)
        if case == 'normal':
            assert error is None, error
        else:
            assert error is not None, case
            expected = 'contract violated' if case in ('thread', 'child') else 'SIGTERM' if case == 'sigterm' else 'Hard protocol deadline'
            assert expected in error, (case, error)
        results.append({'case': case, 'passed': True, 'elapsedSeconds': time.monotonic() - per_case,
                        'error': error, 'descendantsRemaining': 0, 'physicalIntegrations': 0})
print(json.dumps({'schema': 1, 'scope': 'OS-only inert C fixture; no simulation, FK or physics',
                  'physicalIntegrations': 0, 'elapsedSeconds': time.monotonic() - started,
                  'cases': results}, indent=2))
