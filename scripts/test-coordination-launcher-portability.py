#!/usr/bin/env python3
"""Real-process launcher tests using a tiny C fixture; physical integrations=0.

No Rust executable, Rapier library, native experiment manifest, or historical
register is opened for execution. Protocol counts 12/44 are fabricated. All
manifests/claims/registers belong to temporary SYNTHETIC authorization roots.
The runner's NativeWorker, inspector, /proc scan, signals and alarm are real.
Only test constants (authorization identity and a shorter wall deadline) change.
"""
import argparse
import ctypes
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / 'scripts/run-coordination-numeric-parity.py'
SOURCE = ROOT / 'scripts/fixtures/coordination-fake-worker.c'
TEST_AUTHORIZATION = 'SYNTHETIC-launcher-portability-physical-integrations-zero'
TEST_WALL_SECONDS = 2


def load_runner():
    spec = importlib.util.spec_from_file_location('real_launcher_under_test', RUNNER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # These overrides are confined to this test process, never written to source.
    module.AUTHORIZATION_ID = TEST_AUTHORIZATION
    module.WALL_SECONDS = TEST_WALL_SECONDS
    return module


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True) + '\n')


def receipt(**fields):
    return dict(physicalIntegrations=0, protocolCountsAreSynthetic=True,
                nativeRustWorkerExecuted=False, **fields)


def helper(spec_path):
    """Separate actual Python launcher process, to receive external signals."""
    spec = json.loads(Path(spec_path).read_text())
    runner = load_runner()
    handlers = {s: signal.getsignal(s) for s in (signal.SIGALRM, signal.SIGINT, signal.SIGTERM)}
    old_alarm = signal.getitimer(signal.ITIMER_REAL)
    unrelated = None
    result, error = None, None
    try:
        if spec.get('unrelated'):
            environment = dict(os.environ, LH_FAKE_WORKER_MODE='hang-ready')
            unrelated = subprocess.Popen([spec['binary']], stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=environment)
            Path(spec['unrelatedPidPath']).write_text(str(unrelated.pid))
        if spec.get('unavailableProc'):
            runner.PROC = Path(spec['emptyProcPath'])
        if spec.get('existingAlarm'):
            signal.setitimer(signal.ITIMER_REAL, 30)
        factory = runner.NativeWorker
        if spec.get('constructionWindow'):
            def factory(binary, output):
                real_worker = runner.NativeWorker(binary, output)
                Path(spec['constructorMarker']).write_text(str(real_worker.pid))
                # Deliberately expose the real Popen -> assignment window.
                # Process, descriptors, inspector and alarm remain genuine.
                time.sleep(30)
                return real_worker
        result = runner.run(spec['manifest'], execute=spec.get('execute', True),
                            batch_root=Path(spec['batchRoot']), worker_factory=factory)
    except BaseException as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        unrelated_survived = unrelated is not None and unrelated.poll() is None
        if unrelated is not None:
            unrelated.kill()
            unrelated.wait(timeout=2)
            unrelated.stdin.close()
        if spec.get('existingAlarm'):
            signal.setitimer(signal.ITIMER_REAL, 0)
        restored = all(signal.getsignal(s) == previous for s, previous in handlers.items())
        alarm_restored = signal.getitimer(signal.ITIMER_REAL) == old_alarm
        write(spec['helperResult'], receipt(result=result, error=error,
            signalHandlersRestored=restored, alarmRestored=alarm_restored,
            unrelatedSurvivedCleanup=unrelated_survived if spec.get('unrelated') else None))
    return 0


def make_manifest(runner, case, binary, output_name='run'):
    case.mkdir()
    batch = case / 'batch'
    frozen = case / 'frozen'
    batch.mkdir()
    frozen.mkdir()
    files = []
    def add(path, role):
        files.append({'path': str(path.resolve()), 'sha256': sha(path), 'role': role})
    add(binary, 'binary')
    for code in (RUNNER, SOURCE, Path(__file__).resolve()):
        add(code, 'code')
    inputs = {}
    for role in ('fixture', 'program', 'tasks', 'model', 'profile', 'criteria', 'validation'):
        path = frozen / (role + '.json')
        write(path, receipt(role=role, fixtureOnly=True))
        add(path, role)
        if role in ('fixture', 'program', 'tasks'):
            inputs[role + 'Path'] = str(path.resolve())
    protected = []
    for i in range(7):
        path = frozen / f'SYNTHETIC-protected-{i}.txt'
        path.write_text(f'Synthetic placeholder {i}; no historical artifact; zero physics.\n')
        protected.append({'path': str(path.resolve()), 'sha256': sha(path)})
    preparation = frozen / 'preparation.json'
    write(preparation, receipt(approvedNativeSubsteps=runner.STEPS,
        approvedNativeWallSeconds=TEST_WALL_SECONDS, nativeExecutionStarted=False,
        oldRegisters=protected[:6], originalReleaseWorker=protected[6]))
    add(preparation, 'preparation')
    manifest = case / 'manifest.json'
    document = {'schema': runner.SCHEMA, 'authorizationId': TEST_AUTHORIZATION,
        'outputPath': str(batch / output_name), 'binaryPath': str(binary),
        'inputs': inputs, 'files': files,
        'prerequisites': dict.fromkeys(runner.PREREQUISITES, True)}
    write(manifest, document)
    return manifest, batch, document, protected


def proc_state(pid):
    try:
        rest = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return rest[0]
    except FileNotFoundError:
        return None


def recorded_pids(path):
    if not path.exists():
        return []
    return list(dict.fromkeys(int(v) for v in path.read_text().split()))


def wait_for(predicate, timeout=8):
    limit = time.monotonic() + timeout
    while time.monotonic() < limit:
        if predicate():
            return True
        time.sleep(.01)
    return False


def has_permit(batch):
    path = batch / 'run/protocol.ndjson'
    try:
        return any(json.loads(line)['direction'] == 'parent-to-worker'
                   for line in path.read_text().splitlines())
    except (FileNotFoundError, json.JSONDecodeError):
        return False


def run_case(runner, temporary, binary, destination, name, mode='success', **options):
    case = temporary / name
    manifest, batch, document, protected = make_manifest(runner, case, binary)
    pid_path = case / 'fake-pids.txt'
    marker = case / 'constructor.marker'
    spec = {'manifest': str(manifest), 'batchRoot': str(batch), 'binary': str(binary),
        'helperResult': str(case / 'helper-result.json'), 'constructorMarker': str(marker),
        'unrelatedPidPath': str(case / 'unrelated.pid'), 'emptyProcPath': str(case / 'empty-proc'),
        **options}
    Path(spec['emptyProcPath']).mkdir()
    spec_path = case / 'helper-spec.json'
    write(spec_path, spec)
    environment = dict(os.environ, LH_FAKE_WORKER_MODE=mode, LH_FAKE_PID_PATH=str(pid_path))
    started = time.monotonic()
    launcher = subprocess.Popen([sys.executable, str(Path(__file__).resolve()),
        '--helper-spec', str(spec_path)], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        env=environment, start_new_session=True)
    signal_sent = None
    assertions = []
    try:
        if options.get('signal'):
            target_ready = marker.exists if options.get('constructionWindow') else lambda: has_permit(batch)
            if not wait_for(target_ready):
                assertions.append('Requested signal window was not reached')
            elif launcher.poll() is not None:
                assertions.append('Launcher exited before requested signal')
            else:
                signal_sent = options['signal']
                launcher.send_signal(signal_sent)
        stdout, stderr = launcher.communicate(timeout=8)
    except subprocess.TimeoutExpired:
        assertions.append('Launcher failed outer eight-second test watchdog')
        launcher.kill()
        stdout, stderr = launcher.communicate(timeout=2)
    elapsed = time.monotonic() - started
    helper_path = Path(spec['helperResult'])
    result = json.loads(helper_path.read_text()) if helper_path.exists() else None
    pids = recorded_pids(pid_path)
    if marker.exists() and int(marker.read_text()) not in pids:
        pids.append(int(marker.read_text()))
    states = {str(pid): proc_state(pid) for pid in pids}
    survivors = [pid for pid in pids if states[str(pid)] not in (None, 'Z')]
    if survivors:
        assertions.append(f'Fixture processes still running after launcher: {survivors}')
    # Only fixture PIDs recorded by our C binary are eligible for cleanup.
    for pid in pids:
        if proc_state(pid) not in (None, 'Z'):
            try:
                if Path(os.readlink(f'/proc/{pid}/exe')) == binary:
                    os.kill(pid, signal.SIGKILL)
            except FileNotFoundError:
                pass
        try:
            os.waitpid(pid, os.WNOHANG)
        except ChildProcessError:
            pass
    if result is None:
        assertions.append('Missing helper receipt')
    else:
        if not result['signalHandlersRestored']:
            assertions.append('Signal handlers were not restored')
        if not result['alarmRestored']:
            assertions.append('Alarm was not restored/disarmed')
        if options.get('unrelated') and result['unrelatedSurvivedCleanup'] is not True:
            assertions.append('Pre-existing unrelated fixture was killed by runner cleanup')
    output = batch / 'run'
    ledger_path = output / 'physical-ledger.json'
    ledger = json.loads(ledger_path.read_text())['payload'] if ledger_path.exists() else None
    no_launch = options.get('execute') is False or options.get('unavailableProc') or options.get('existingAlarm')
    success = mode == 'success' and not options.get('signal') and not no_launch
    if no_launch:
        if pids or (batch / 'authorization-claim.json').exists() or output.exists():
            assertions.append('Preflight refusal/read-only check created claim/output or spawned fixture')
        if options.get('execute') is False and (result or {}).get('error'):
            assertions.append('Read-only preflight should have passed')
        if options.get('execute') is not False and not (result or {}).get('error'):
            assertions.append('Unavailable capability should refuse')
    else:
        if not ledger or ledger['status'] != ('closed-success' if success else 'closed-failure'):
            assertions.append('Ledger did not close with expected status')
        if ledger and (ledger['reservedNativeSteps'] != 44 or ledger['refundNativeSteps'] != 0):
            assertions.append('Synthetic reservation changed/refunded')
        if result and bool(result['error']) == success:
            assertions.append('Helper result disagrees with expected success/refusal')
        if mode in ('success', 'hang-eof') and ledger and ledger['actualNativeStepsKnown'] != 44:
            assertions.append('Fabricated 44-step receipt was not preserved')
        if mode == 'refusal' and ledger and ledger['actualNativeStepsKnown'] != 12:
            assertions.append('Fabricated 12-step partial receipt was not preserved')
        if mode in ('child', 'two-threads') and has_permit(batch):
            assertions.append('Permit sent despite process/thread violation at ready')
    for record in protected:
        if sha(record['path']) != record['sha256']:
            assertions.append('Synthetic protected file changed')
    duplicate = None
    if success:
        before_count = len(pids)
        claim = batch / 'authorization-claim.json'
        before_claim = sha(claim)
        duplicate_manifest = case / 'duplicate-manifest.json'
        second = dict(document, outputPath=str(batch / 'second-run'))
        write(duplicate_manifest, second)
        try:
            runner.run(duplicate_manifest, execute=True, batch_root=batch)
            duplicate = 'unexpected success'
            assertions.append('Duplicate claim executed')
        except BaseException as exc:
            duplicate = f'{type(exc).__name__}: {exc}'
        if len(recorded_pids(pid_path)) != before_count or sha(claim) != before_claim:
            assertions.append('Duplicate claim spawned or changed immutable claim')
    case_output = destination / name
    case_output.mkdir()
    if output.exists():
        shutil.copytree(output, case_output / 'runner-artifacts')
    for src in (manifest, helper_path, batch / 'authorization-claim.json'):
        if src.exists():
            shutil.copy2(src, case_output / src.name)
    (case_output / 'launcher.stderr.txt').write_bytes(stderr)
    (case_output / 'launcher.stdout.txt').write_bytes(stdout)
    summary = receipt(case=name, fixtureMode=mode, testWallSeconds=TEST_WALL_SECONDS,
        passed=not assertions, failures=assertions, helperExitCode=launcher.returncode,
        elapsedSeconds=elapsed, sentSignal=signal_sent, fixturePids=pids,
        statesAfterLauncher=states, activeSurvivors=survivors, helperReceipt=result,
        duplicateClaimRefusal=duplicate, temporaryManifestPathsAreHistorical=True,
        oldNativeRegistersTouched=False)
    write(case_output / 'receipt.json', summary)
    print(json.dumps({'case': name, 'passed': not assertions, 'physicalIntegrations': 0,
                      'failures': assertions}), flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--helper-spec')
    parser.add_argument('--output-dir')
    args = parser.parse_args()
    if args.helper_spec:
        return helper(args.helper_spec)
    if not args.output_dir:
        parser.error('--output-dir is required and must not already exist')
    destination = Path(args.output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    runner = load_runner()
    # Adopt fixture descendants solely to reap them after deliberate group kills.
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(36, 1, 0, 0, 0) != 0:  # Linux PR_SET_CHILD_SUBREAPER
        raise OSError(ctypes.get_errno(), 'Cannot enable fixture descendant reaping')
    initial_runner_sha = sha(RUNNER)
    host_children = Path(f'/proc/{os.getpid()}/task/{os.getpid()}/children')
    host = receipt(platform=sys.platform, procChildrenInterfaceExists=host_children.exists(),
        procStatExists=Path(f'/proc/{os.getpid()}/stat').exists(),
        processInspection=runner.inspect_process(os.getpid()))
    cases = [
        ('read-only-preflight', 'success', {'execute': False}),
        ('unavailable-proc-preflight', 'success', {'unavailableProc': True}),
        ('existing-alarm-preflight', 'success', {'existingAlarm': True}),
        ('success-and-duplicate-claim', 'success', {}),
        ('partial-refusal', 'refusal', {}),
        ('crash-before-ready', 'crash-ready', {}),
        ('crash-after-permit', 'crash-permit', {}),
        ('timeout-before-ready', 'hang-ready', {}),
        ('timeout-after-permit', 'hang', {}),
        ('timeout-after-result', 'hang-eof', {}),
        ('sigterm-after-permit', 'hang', {'signal': signal.SIGTERM}),
        ('sigint-after-permit', 'hang', {'signal': signal.SIGINT}),
        ('sigterm-during-construction', 'hang', {'signal': signal.SIGTERM, 'constructionWindow': True}),
        ('child-process-refused', 'child', {}),
        ('second-thread-refused', 'two-threads', {}),
        ('unrelated-child-preserved', 'hang', {'unrelated': True}),
    ]
    with tempfile.TemporaryDirectory(prefix='lh-launcher-C-only-') as tmp:
        temporary = Path(tmp)
        binary = temporary / 'fake-worker'
        command = ['cc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-pthread', str(SOURCE), '-o', str(binary)]
        compiled = subprocess.run(command, capture_output=True, text=True, check=True)
        write(destination / 'fixture-build.json', receipt(command=command, exitCode=compiled.returncode,
            stdout=compiled.stdout, stderr=compiled.stderr, sourceSha256=sha(SOURCE), binarySha256=sha(binary)))
        summaries = [run_case(runner, temporary, binary, destination, name, mode, **options)
                     for name, mode, options in cases]
    unchanged = sha(RUNNER) == initial_runner_sha
    summary = receipt(passed=all(v['passed'] for v in summaries) and unchanged,
        tests=len(summaries), passedTests=sum(v['passed'] for v in summaries),
        runnerSha256=initial_runner_sha, runnerUnchangedThroughout=unchanged,
        harnessSha256=sha(__file__), sourceSha256=sha(SOURCE), host=host,
        caseReceipts=[f'{v["case"]}/receipt.json' for v in summaries],
        scope='Real Linux processes, pipes, /proc, alarms and signals; fabricated C protocol only. No native physical qualification.')
    write(destination / 'summary.json', summary)
    print(json.dumps(summary), flush=True)
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
