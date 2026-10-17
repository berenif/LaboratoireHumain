"""Bounded JSONL source bridge. Every real integration stays in native Rapier."""
from __future__ import annotations
import datetime as dt
import hashlib
import fcntl
import math
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomic_json(path, value):
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.tmp')
    with temp.open('w') as file:
        file.write(json.dumps(value, indent=2, allow_nan=False) + '\n'); file.flush(); os.fsync(file.fileno())
    os.replace(temp, path)


class PilotLedger:
    def __init__(self, path):
        self.path = Path(path)
        self._lock = self.path.with_suffix(self.path.suffix+'.lock').open('a+')
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self._lock.close(); raise RuntimeError('another pilot ledger writer is active')
        self.active_process = None
        self.data = json.loads(self.path.read_text())
        self.data.setdefault('stagePhysicsUsedSeconds', {})
        self.data.setdefault('processes', [])
        self.data.setdefault('uncertainRequests', [])
        if any(p.get('status') in ['running','unresolved-process'] or p.get('requestOutstanding') is not None for p in self.data['processes']):
            self.close(); raise RuntimeError('unfinished native process requires explicit reconciliation before restart')
        self.deadline = dt.datetime.fromisoformat(self.data['deadlineUtc'].replace('Z', '+00:00')).timestamp()
        self.check_deadline()

    def close(self):
        if getattr(self,'_lock',None) is not None and not self._lock.closed:
            fcntl.flock(self._lock,fcntl.LOCK_UN);self._lock.close()

    def __del__(self):
        self.close()

    def check_deadline(self):
        if time.time() >= self.deadline:
            raise TimeoutError('aggregate pilot wall deadline reached')

    def admit(self, stage, cap):
        self.check_deadline()
        if any(p.get('requestOutstanding') is not None or p.get('status')=='unresolved-process' for p in self.data['processes']):
            raise RuntimeError('unresolved native request requires reconciliation')
        remaining = self.data['newAggregatePhysicsLimitSeconds'] - self.data['newPhysicsWallSeconds']
        stage_remaining = self.data['stagePhysicsCeilings'][stage] - self.data['stagePhysicsUsedSeconds'].get(stage, 0.)
        # Validation is a protected reserve; other stages cannot consume it.
        reserve = 0. if stage == 'validationReserved' else max(0., self.data['stagePhysicsCeilings']['validationReserved'] - self.data['stagePhysicsUsedSeconds'].get('validationReserved', 0.))
        if not 0 <= cap <= min(90., stage_remaining, remaining - reserve):
            raise ValueError('requested process physics cap exceeds stage/aggregate/reserved allowance')

    def save(self):
        self.data['newNativeSubstepsUpperBound'] = self.data['newNativeSubsteps'] + sum(r['maximumAdditionalNativeSubsteps'] for r in self.data['uncertainRequests'])
        self.data['lastUpdatedUtc'] = dt.datetime.now(dt.timezone.utc).isoformat()
        self.data['totalElapsedWallSeconds'] = max(0., time.time() - dt.datetime.fromisoformat(self.data['startedUtc'].replace('Z', '+00:00')).timestamp())
        atomic_json(self.path, self.data)

    def charge(self, stage, physics, substeps):
        if physics < 0 or substeps < 0:
            raise ValueError('process counters moved backwards')
        self.data['newPhysicsWallSeconds'] += physics
        self.data['newNativeSubsteps'] += substeps
        used = self.data['stagePhysicsUsedSeconds']
        used[stage] = used.get(stage, 0.) + physics
        self.save()


class SourceNative:
    def __init__(self, binary, checkpoint, output, ledger, *, stage, physics_cap, upright=False):
        if ledger.active_process is not None:
            raise RuntimeError('one native process at a time per pilot ledger')
        ledger.admit(stage, physics_cap)
        self.ledger, self.stage, self.cap = ledger, stage, physics_cap
        self.output = Path(output); self.output.mkdir(parents=True, exist_ok=True)
        (self.output/'started-once').open('x').close()
        self.physics, self.substeps, self.closed = 0., 0, False
        self.trace = (self.output/'protocol.ndjson').open('w')
        self.stderr = (self.output/'stderr.log').open('w')
        args = [str(Path(binary).resolve()), str(Path(checkpoint).resolve()), str(physics_cap)]
        if upright:
            args.append('--upright-entry')
        self.proc = subprocess.Popen(args, cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.stderr, text=True)
        self.record = dict(pid=self.proc.pid, stage=stage, physicsCapSeconds=physics_cap, binarySHA256=sha256(binary),
                           checkpointSHA256=sha256(checkpoint), output=str(self.output), status='running')
        ledger.active_process=self
        ledger.data['processes'].append(self.record); ledger.save()

    def command(self, request, *, expect_ok=True):
        self.ledger.check_deadline()
        wire = json.dumps(request, allow_nan=False)
        is_step = request.get('op', '').startswith('step')
        timeout = min(15., self.ledger.deadline - time.time())
        if is_step:
            timeout = min(timeout, max(.001, self.cap - self.physics + .1))
        began = time.monotonic()
        self.record['requestOutstanding']=dict(request=request,startedUnix=time.time(),maximumAdditionalNativeSubsteps=20 if is_step else 0)
        self.ledger.save()  # Write-ahead marker survives a killed Python process.
        def alarm(*_):raise TimeoutError('native response timeout')
        old_handler = signal.signal(signal.SIGALRM, alarm)
        signal.setitimer(signal.ITIMER_REAL, max(.001, timeout))
        raw=None
        try:
            self.proc.stdin.write(wire + '\n'); self.proc.stdin.flush()
            raw = self.proc.stdout.readline()
            if not raw:
                raise RuntimeError('native process ended without response')
            response = json.loads(raw)
            wall, steps = response['processPhysicsWallS'], response['processNativeSubsteps']
            if type(response['ok']) is not bool or type(wall) not in (int,float) or not math.isfinite(wall) or wall<self.physics or type(steps)is not int or steps<self.substeps or steps-self.substeps>(20 if is_step else 0):
                raise ValueError('invalid/nonfinite/backward native response counters')
        except BaseException as error:
            # Disable alarm BEFORE cleanup, then reap before measuring/charging.
            signal.setitimer(signal.ITIMER_REAL,0)
            reaped=False
            try:
                if self.proc.poll() is None:self.proc.kill()
                self.proc.wait(timeout=5.);reaped=True
            finally:
                if is_step:
                    elapsed=time.monotonic()-began
                    self.ledger.data['uncertainRequests'].append(dict(output=str(self.output),request=request,
                        conservativeChargedPhysicsSeconds=elapsed,maximumAdditionalNativeSubsteps=20,error=repr(error)))
                    self.ledger.charge(self.stage,elapsed,0)
                self.record['uncertain']=is_step
                if reaped:
                    self.record['requestOutstanding']=None
                    self.record['status']='failed-charged' if is_step else 'failed-read-only'
                else:
                    self.record['status']='unresolved-process'  # Keep write-ahead reservation.
                self.ledger.save()
            # Failure logging cannot preempt kill/reap/accounting.
            try:
                self.trace.write(json.dumps(dict(request=request,failedResponse=raw,error=repr(error)),allow_nan=False)+'\n');self.trace.flush()
            except OSError:
                pass
            raise
        finally:
            signal.setitimer(signal.ITIMER_REAL,0)
            signal.signal(signal.SIGALRM,old_handler)
        self.ledger.charge(self.stage,wall-self.physics,steps-self.substeps)
        self.physics,self.substeps=wall,steps
        self.record['requestOutstanding']=None
        self.record['lastKnownPhysicsWallSeconds']=wall;self.record['lastKnownNativeSubsteps']=steps
        self.ledger.save()
        self.trace.write(json.dumps(dict(request=request,response=response),allow_nan=False)+'\n');self.trace.flush()
        if response['ok'] != expect_ok:
            raise RuntimeError(response.get('error', 'unexpected native response'))
        return response.get('result', response)

    def checkpoint(self, path, controller_state):
        path = Path(path)
        result = self.command(dict(op='checkpoint', path=str(path.resolve())))
        atomic_json(path.with_suffix('.controller.json'), dict(nativeSHA256=sha256(path), controller=controller_state,
                    ledger=str(self.ledger.path.resolve()), nativeSubsteps=self.substeps))
        return result

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=5.)
        except (BrokenPipeError, subprocess.TimeoutExpired):
            self.proc.kill(); self.proc.wait()
        self.trace.close(); self.stderr.close()
        self.ledger.active_process=None
        status='unresolved-process' if self.record.get('requestOutstanding') is not None or self.record.get('status')=='unresolved-process' else ('closed-with-uncertainty' if self.record.get('uncertain') else 'closed')
        self.record.update(status=status, exitCode=self.proc.returncode, physicsWallSeconds=self.physics, nativeSubsteps=self.substeps)
        self.ledger.save()
