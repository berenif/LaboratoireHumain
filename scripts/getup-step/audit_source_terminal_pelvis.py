#!/usr/bin/env python3
"""Independent terminal-only pelvis profile over the unchanged reviewed audit.

Every physical/arm/causal-support reducer and retained transfer witness is still
run by ``core.audit``. This policy changes only final-stand body-4 orientation
arithmetic. Its three changed-task QP witnesses are read-only local replays, not
integrations or a proof of all unrecorded interventions or visual improvement.
"""
import argparse
from collections import deque
import copy
import ctypes as C
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

import audit_source_reviewed_walk as core
from audit_source_cadenced_steps import capture_margin, quiet

PROFILE = 'terminal-pelvis-soft-v1'
DURATION = 4.
MAX_ANGLE_RAD = math.radians(6.)
MAX_WEIGHT = .5
THRESHOLDS = (.5, 4., 4.1)
ACTIVATION_FIELDS = frozenset('''schema profile body stepIndex finalStepIndex sourceStep
sourceTimeS standEntry initialRotationSource yawRad uprightRotationSource axisWorldRad
boundedEndpointRotationSource correctionAngleRad durationS maximumWeight
measuredFinalLanding loadedHistory activationSHA256'''.split())
ACTIVE_FIELDS = frozenset(('profile', 'activation', 'activationSHA256', 'clock',
    'elapsedS', 'complete', 'endpointHeld', 'orientation'))


def terminal_policy():
    """Independent literal declaration; never imported from the producer."""
    return dict(schema=1, profile=PROFILE, enabled=True, body=4,
        activation='once at final measured settle-to-stand handoff',
        measuredFinalLandingRequired=True, minimumLoadedDoubleSupportS=.1,
        minimumLoadedHistorySamples=81, minimumEachFootLoadN=3.,
        minimumTotalWeightFraction=.8, actualNativeHistoryOnly=True,
        target='same-yaw upright, left-trivialized SO(3) arc from frozen actual pelvis',
        maximumCorrectionAngleRad=MAX_ANGLE_RAD, durationS=DURATION,
        maximumWeight=MAX_WEIGHT, orientationKp=40., orientationKd=12.,
        referenceAndWeightClock='same 4s quintic', endpointHeldAfterCompletion=True,
        quietStarts='only integrations after first complete endpoint command',
        torsoComJointFootArmBarrierUnloadReferencesUnchanged=True,
        transferForecastOnStand=False, optimizerEnabled=False, hardConeEnabled=False,
        kneeReferenceChanged=False, riskHistoryChanged=False,
        originalPhysicalGatesUnchanged=True, originalPhaseDeadlinesUnchanged=True)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def rotation(value):
    R = core.array(value, (3, 3))
    require(core.close(R.T@R, np.eye(3), 1e-10)
        and abs(np.linalg.det(R)-1.) <= 1e-10, 'invalid proper rotation')
    return R


def terminal_geometry(initial):
    """Same-yaw upright goal with a world SO(3) displacement clipped to 6 deg."""
    R0 = rotation(initial)
    require(math.hypot(R0[0, 0], R0[1, 0]) > 1e-8, 'singular yaw chart')
    yaw = math.atan2(R0[1, 0], R0[0, 0])
    upright = Rotation.from_rotvec([0., 0., yaw]).as_matrix()
    axis = Rotation.from_matrix(upright@R0.T).as_rotvec()
    angle = float(np.linalg.norm(axis))
    if angle > MAX_ANGLE_RAD:
        axis *= MAX_ANGLE_RAD/angle
    endpoint = Rotation.from_rotvec(axis).as_matrix()@R0
    return yaw, upright, axis, endpoint


def terminal_orientation(activation, source_time):
    initial = rotation(activation['initialRotationSource'])
    _, _, axis, _ = terminal_geometry(initial)
    elapsed = float(source_time)-float(activation['sourceTimeS'])
    require(math.isfinite(elapsed) and elapsed >= -1e-10, 'terminal reference predates activation')
    s, ds, dds = core.clock(elapsed, DURATION)
    return dict(body=4,
        rotation=(Rotation.from_rotvec(axis*s).as_matrix()@initial).reshape(-1).tolist(),
        velocity=(axis*ds).tolist(), acceleration=(axis*dds).tolist(),
        kp=40., kd=12., weight=MAX_WEIGHT*s)


class PelvisFK:
    """Independent body-4 rotation from bound XML joint order and saved qpos.

    Only rotations along torso -> pelvis are needed. This uses no runtime FK,
    controller, native engine, checkpoint import, or physical integration.
    """
    def __init__(self, xml_path, model):
        tree = ET.parse(xml_path)
        world = tree.getroot().find('worldbody')
        bodies = list(world.iter('body'))
        require(len(bodies) >= 4 and bodies[3].get('name') == 'pelvis', 'native body 4 is not pelvis')
        parents = {child: parent for parent in world.iter() for child in parent}
        node = bodies[3]; chain = []
        while node.tag == 'body':
            chain.append(node); node = parents[node]
        chain.reverse()
        require(chain[0].get('name') == 'torso', 'unexpected pelvis root')
        hinges = {segment['reference']['hinge']['name']: segment['reference']['hinge']['qposIndex']
            for segment in model['segments'] if segment['reference'].get('hinge')}
        self.chain = []
        for body in chain[1:]:
            require(not any(body.get(name) for name in ('euler', 'axisangle', 'xyaxes', 'zaxis')),
                'unsupported XML body orientation')
            quat = np.fromstring(body.get('quat', '1 0 0 0'), sep=' ')
            require(quat.shape == (4,) and np.isfinite(quat).all() and np.linalg.norm(quat) > 0., 'invalid body quaternion')
            fixed = Rotation.from_quat(quat[[1, 2, 3, 0]]).as_matrix()
            joints = []
            for joint in body.findall('joint'):
                require(joint.get('type', 'hinge') == 'hinge' and joint.get('name') in hinges, 'unexpected pelvis hinge')
                axis = np.fromstring(joint.get('axis', '0 0 1'), sep=' ')
                require(axis.shape == (3,) and np.isfinite(axis).all() and np.linalg.norm(axis) > 0., 'invalid hinge axis')
                joints.append((hinges[joint.get('name')], axis/np.linalg.norm(axis)))
            self.chain.append((fixed, joints))

    def __call__(self, row):
        q = core.array(row['qpos'], (28,))
        R = Rotation.from_quat(q[[4, 5, 6, 3]]).as_matrix()
        for fixed, joints in self.chain:
            R = R@fixed
            for index, axis in joints:
                R = R@Rotation.from_rotvec(axis*q[7+index]).as_matrix()
        return R


def audit_activation(activation, anchor, final_index, landing, history, mass, pelvis_fk):
    checks = {}
    def check(key, value): checks[key] = bool(value)
    try:
        check('exactTerminalActivationSchema', isinstance(activation, dict) and set(activation) == ACTIVATION_FIELDS
            and activation['schema'] == 1 and activation['profile'] == PROFILE)
        check('terminalActivationDigest', core.is_sha(activation.get('activationSHA256'))
            and core.value_sha({k: v for k, v in activation.items() if k != 'activationSHA256'}) == activation['activationSHA256'])
        check('terminalActivationOnlyAtFinalStandEntry', activation['body'] == 4
            and type(activation['stepIndex']) is int and activation['stepIndex'] == activation['finalStepIndex'] == final_index
            and type(activation['sourceStep']) is int and activation['sourceStep'] >= 0
            and activation['standEntry'] is True and anchor['phase'] == 'settle'
            and anchor['stepIndex'] == final_index and activation['sourceStep'] == anchor['sourceStep']
            and core.close(activation['sourceTimeS'], anchor['sourceTimeS'], 1e-12))
        R0 = rotation(activation['initialRotationSource']); measured = pelvis_fk(anchor)
        check('terminalAnchorIsActualBody4FK', core.close(R0, measured, 1e-10)
            and core.close(anchor.get('actualPelvisRotationSource'), measured, 1e-10))
        yaw, upright, axis, endpoint = terminal_geometry(R0)
        check('independentSameYawBoundedSO3Goal', core.close(activation['yawRad'], yaw, 1e-12)
            and core.close(rotation(activation['uprightRotationSource']), upright, 1e-10)
            and core.close(activation['axisWorldRad'], axis, 1e-10)
            and core.close(rotation(activation['boundedEndpointRotationSource']), endpoint, 1e-10)
            and core.close(activation['correctionAngleRad'], np.linalg.norm(axis), 1e-12)
            and np.linalg.norm(axis) <= MAX_ANGLE_RAD+1e-12
            and activation['durationS'] == DURATION and activation['maximumWeight'] == MAX_WEIGHT)
        # The summary uses the integration counter clock; activation and native
        # rows use the accumulated simulator clock. Bind each to its own source
        # without widening the exact native-row comparison for long runs.
        check('terminalLandingSummaryMatchesStepCounterClock',
            core.close(landing['landingTimeS'], landing['landingNativeStep']*core.DT, 1e-12))
        expected_landing = dict(sourceStep=landing['landingNativeStep'], sourceTimeS=landing['actualLandingSourceTimeS'],
            stepIndex=final_index, landingConsecutiveSamples=landing.get('landingConsecutiveSamples'),
            feetLoadN=landing.get('feetLoadN'), forwardM=landing['achievedAtLoadAcquisitionM'],
            requestedForwardM=landing['requestedDistanceM'])
        saved_landing = activation['measuredFinalLanding']
        check('terminalAnchorFollowsMeasuredFinalLanding', saved_landing['sourceStep'] == expected_landing['sourceStep']
            and core.close(saved_landing['sourceTimeS'], expected_landing['sourceTimeS'], 1e-12)
            and saved_landing['stepIndex'] == final_index
            and type(saved_landing['landingConsecutiveSamples']) is int
            and saved_landing['landingConsecutiveSamples'] == landing['actualLandingConsecutiveSamples'] >= 80
            and core.equal(saved_landing['feetLoadN'], landing['actualLandingFeetLoadN'])
            and all(value >= 3. for value in saved_landing['feetLoadN'])
            and core.close(saved_landing['forwardM'], expected_landing['forwardM'])
            and saved_landing['forwardM'] >= saved_landing['requestedForwardM'] == .18
            and saved_landing['sourceStep'] <= anchor['sourceStep'])
        h = activation['loadedHistory']; samples = h['samples']
        check('terminalActualLoadedHistoryPolicy', h['minimumDurationS'] == .1
            and h['minimumEachFootLoadN'] == 3. and h['minimumTotalWeightFraction'] == .8
            and core.close(h['bodyWeightN'], mass*9.81))
        check('terminalHistoryBoundToEveryActualNativeSample', len(samples) >= 81
            and core.equal(samples, history)
            and all(b['sourceStep']-a['sourceStep'] == 1 and core.close(b['sourceTimeS']-a['sourceTimeS'], core.DT, 1e-12)
                for a, b in zip(samples, samples[1:]))
            and all(min(s['feetLoadN']) >= 3. and sum(s['feetLoadN']) >= .8*mass*9.81 for s in samples)
            and h['sourceStartStep'] == samples[0]['sourceStep'] >= saved_landing['sourceStep']
            and h['sourceEndStep'] == samples[-1]['sourceStep'] == anchor['sourceStep']
            and core.close(h['sourceStartTimeS'], samples[0]['sourceTimeS'], 1e-12)
            and core.close(h['sourceEndTimeS'], samples[-1]['sourceTimeS'], 1e-12)
            and core.close(h['durationS'], samples[-1]['sourceTimeS']-samples[0]['sourceTimeS'], 1e-12)
            and h['durationS'] >= .1-1e-12)
    except (KeyError, TypeError, ValueError, IndexError, ArithmeticError):
        checks['completeIndependentTerminalActivationEvidence'] = False
    return dict(passed=bool(checks) and all(checks.values()), checks=checks,
        failedChecks=[k for k, v in checks.items() if not v])


def active_context_checks(receipt, note, activation):
    checks = dict(explicitTerminalFeatureEveryCommand=note.get('reviewedWalk', {}).get('featureProfile') == PROFILE,
        noOtherChangedReferenceFeature=receipt.get('config', {}).get('coupledPosture') is None
            and receipt.get('config', {}).get('boundedSettle') is None)
    try:
        active = receipt['causal'].get('terminalPelvis')
        stand = receipt['phase'] == 'stand'
        checks['terminalOverlayOnlyInFinalStand'] = ((active is not None) == stand
            and (not stand or receipt['step_index'] == activation['finalStepIndex']))
        if not stand:
            checks['preterminalReferenceContextUnchanged'] = ('terminalPelvis' not in receipt['causal']
                and 'terminalPelvis' not in receipt['diagnostics'] and len(receipt['orientations']) == 1)
        else:
            now = float(receipt['source_time']); elapsed = now-activation['sourceTimeS']
            orientation = terminal_orientation(activation, now)
            complete = elapsed >= DURATION-1e-10
            checks['exactFrozenTerminalOverlayEvidence'] = (set(active) == ACTIVE_FIELDS
                and active['profile'] == PROFILE and core.equal(active['activation'], activation)
                and active['activationSHA256'] == activation['activationSHA256']
                and core.equal(active['clock'], core.clock(elapsed, DURATION))
                and core.close(active['elapsedS'], elapsed, 1e-12)
                and active['complete'] is complete and active['endpointHeld'] is complete
                and core.equal(active['orientation'], orientation)
                and core.equal(receipt['diagnostics'].get('terminalPelvis'), active))
            checks['onlyOneAdditionalBody4Orientation'] = (len(receipt['orientations']) == 2
                and receipt['orientations'][0]['body'] == 1 and receipt['orientations'][1]['body'] == 4
                and core.equal(receipt['orientations'][1], orientation))
    except (KeyError, TypeError, ValueError, IndexError, ArithmeticError):
        checks['completeFiniteTerminalReferenceContext'] = False
    return checks


def audit_terminal_journal(rows, notes, entry, activation, summary, mass, pelvis_fk):
    """Rebuild the handoff, load history and fresh endpoint-held quiet counter."""
    checks = {}; commands = iter(notes); next_note = next(commands, None)
    def check(key, value): checks[key] = checks.get(key, True) and bool(value)
    previous = entry; count = quiet_run = 0; anchor = None; anchor_feet = None
    history = deque(); actual_history = None; first_endpoint = None; landing_row = None
    final_index = len(summary.get('steps', [])); records = {r['index']: r for r in summary.get('steps', [])}
    final_record = copy.deepcopy(records.get(final_index, {})); activation_proof = None
    current_note = None; quiet_start = None; stand_count = 0; landing_run = 0; anchor_history = None
    for row in rows:
        count += 1
        try:
            sid, now = row['sourceStep'], row['sourceTimeS']
            check('terminalEveryNativeSampleOrdered', sid == previous['sourceStep']+1
                and core.close(now-previous['sourceTimeS'], core.DT, 1e-12))
            # Commands describe the preceding state, including the endpoint task
            # first installed before the native integration represented by row.
            if (count-1) % 4 == 0:
                current_note = next_note; next_note = next(commands, None)
                check('terminalAllActualCommandsBound', isinstance(current_note, dict)
                    and current_note.get('stepIndex') == row['stepIndex'] and current_note.get('phase') == row['phase']
                    and core.close(current_note.get('t'), previous['sourceStep']*core.DT, 1e-10))
                if isinstance(current_note, dict):
                    receipt = current_note.get('referenceContext', {})
                    for key, good in active_context_checks(receipt, current_note, activation).items(): check(key, good)
                    if row['phase'] == 'stand' and receipt.get('source_time', -math.inf) >= activation.get('sourceTimeS', math.inf)+DURATION-1e-10:
                        if first_endpoint is None:
                            first_endpoint = dict(sourceStep=receipt['source_step'], sourceTimeS=receipt['source_time'])
            if row['stepIndex'] == final_index and row['phase'] == 'land':
                acquired = (min(f['loadN'] for f in row['feet']) >= 3.
                    and row['feet'][row['swingSide']]['clearance'] <= .003
                    and row['swingForwardDisplacementM'] >= .18)
                landing_run = landing_run+1 if acquired else 0
            if row['stepIndex'] == final_index and sid == final_record.get('landingNativeStep'):
                final_record['actualLandingConsecutiveSamples'] = landing_run
                landing_row = row
                final_record['actualLandingSourceTimeS'] = now
                final_record['actualLandingFeetLoadN'] = [f['loadN'] for f in row['feet']]
            loaded = (row['stepIndex'] == final_index
                and min(f['loadN'] for f in row['feet']) >= 3.
                and sum(f['loadN'] for f in row['feet']) >= .8*mass*9.81)
            if not loaded or sid < final_record.get('landingNativeStep', math.inf):
                history.clear()
            else:
                history.append(dict(sourceStep=sid, sourceTimeS=now, feetLoadN=[f['loadN'] for f in row['feet']]))
                # The receipt may retain a little more than .1 s, but never
                # trusts loads absent from the independently saved native rows.
                while history and history[0]['sourceStep'] < activation.get('loadedHistory', {}).get('sourceStartStep', -1):
                    history.popleft()
            if sid == activation.get('sourceStep'):
                anchor_history = list(history)
            if row['phase'] == 'stand':
                stand_count += 1
                if anchor is None:
                    anchor = previous; actual_history = anchor_history or []
                    positions = core.array([f['position'] for f in row['feet']], (2, 3)); anchor_feet = positions.copy()
                    activation_proof = audit_activation(activation, anchor, final_index, final_record,
                        actual_history, mass, pelvis_fk)
                    for key, good in activation_proof['checks'].items(): check(key, good)
                measured = pelvis_fk(row)
                check('everyTerminalActualPelvisRotationMatchesIndependentFK', core.close(row.get('actualPelvisRotationSource'), measured, 1e-10))
                drift = float(np.max(np.linalg.norm(np.asarray([f['position'] for f in row['feet']])-anchor_feet, axis=1)))
                eligible = (first_endpoint is not None and sid > first_endpoint['sourceStep']
                    and now > activation['sourceTimeS']+DURATION+1e-10)
                isquiet = eligible and quiet(row, mass, drift, capture_margin(row))
                quiet_run = quiet_run+1 if isquiet else 0
                if quiet_run == 1: quiet_start = sid
                telemetry = row.get('terminalPelvis', {})
                check('freshTerminalQuietCountMatchesEveryNativeSample', telemetry.get('quietCountedSamples') == quiet_run
                    and core.close(row.get('quietConsecutiveS'), quiet_run*core.DT, 1e-10))
                check('terminalQuietEligibilityAfterHeldEndpointCommand', telemetry.get('quietEligible') is eligible
                    and telemetry.get('rampComplete') is (now >= activation['sourceTimeS']+DURATION-1e-10)
                    and telemetry.get('endpointCommandSourceStep') == (None if first_endpoint is None else first_endpoint['sourceStep'])
                    and core.equal(telemetry.get('endpointCommandSourceTimeS'), None if first_endpoint is None else first_endpoint['sourceTimeS']))
                check('terminalRowAnchorAndClockFrozen', telemetry.get('profile') == PROFILE
                    and telemetry.get('enabled') is True
                    and telemetry.get('activationSHA256') == activation['activationSHA256']
                    and telemetry.get('activated') is True
                    and telemetry.get('activationSourceStep') == activation['sourceStep']
                    and core.close(telemetry.get('activationSourceTimeS'), activation['sourceTimeS'], 1e-12)
                    and core.close(telemetry.get('elapsedS'), now-activation['sourceTimeS'], 1e-12))
            else:
                check('noPreterminalQuietCredit', row.get('terminalPelvis', {}).get('quietCountedSamples', 0) == 0)
        except (KeyError, TypeError, ValueError, IndexError, ArithmeticError):
            check('completeIndependentTerminalJournalEvidence', False)
        previous = row
    check('terminalNonemptyNativeJournal', count > 0)
    check('terminalCompleteCommandSchedule', next_note is None and next(commands, None) is None)
    check('finalStandActivationObserved', anchor is not None and landing_row is not None and stand_count > 0)
    check('terminalFinal4000NativeSamplesQuietAfterRamp', quiet_run >= 4000 and first_endpoint is not None
        and previous.get('stepIndex') == final_index and previous.get('phase') == 'stand')
    check('terminalQuietSummaryMatchesIndependentCounter', core.close(summary.get('finalQuietS'), quiet_run*core.DT, 1e-10)
        and summary.get('actualIntegrations') == count and final_record.get('completedNativeStep') == previous.get('sourceStep')
        and core.close(final_record.get('quietConsecutiveS'), quiet_run*core.DT, 1e-10)
        and summary.get('passed') is True)
    return dict(passed=bool(checks) and all(checks.values()), checks=checks,
        failedChecks=[k for k, v in checks.items() if not v], actualRecordedIntegrations=count,
        finalQuietSamples=quiet_run, finalQuietS=quiet_run*core.DT, finalQuietStartSourceStep=quiet_start,
        firstEndpointCommand=first_endpoint, activationReplay=activation_proof,
        physicsIntegrations=0, independentReferenceArithmetic=True)


class TerminalPelvisPolicy:
    profile = PROFILE

    def __init__(self, folder):
        self.folder = Path(folder); self.checks = {}; self.native = None; self.seen = set(); self.native_results = []
        read = lambda name: json.loads((self.folder/name).read_text())
        self.config = read('reviewed-walk-config.json'); self.run = read('reviewed-walk-run.json')
        self.summary = read('multistep-summary.json'); self.initial = read('summary.json')
        model = json.loads((Path(__file__).resolve().parents[2]/'rust/crates/model/data/getup-v1.json').read_text())
        self.mass = model['totalMassKg']
        self.fk = PelvisFK(self.folder.parents[1]/'model-session/source-model/humanoid_high_freq.xml', model)
        activation_path = self.folder/'terminal-pelvis-activation.json'
        self.activation = read('terminal-pelvis-activation.json') if activation_path.is_file() else {}
        self.input_hashes = {name: core.file_sha(self.folder/name) for name in
            ('terminal-pelvis-activation.json',) if (self.folder/name).is_file()}
        with (self.folder/'frames.ndjson').open() as stream:
            self.journal = audit_terminal_journal((json.loads(line) for line in stream),
                core.json_array_items(self.folder/'control.json'), self.initial['entry'], self.activation,
                self.summary, self.mass, self.fk)
        self.samples = {}; self.manifest = {}; self.manifest_errors = []
        path = self.folder/'terminal-pelvis-native/manifest.json'
        try:
            require(path.is_file() and not path.is_symlink() and path.stat().st_size <= 256*1024, 'missing terminal native manifest')
            self.manifest = json.loads(path.read_text()); m = self.manifest
            require(m.get('schema') == 1 and m.get('profile') == PROFILE
                and m.get('stepIndex') == self.config['requestedSteps'] and m.get('physicsIntegrations') == 0
                and m.get('actualCommandNotYetApplied') is True
                and m.get('payloadEncoding') == 'zlib-compressed exact lh_checkpoint_export payload', 'terminal manifest protocol mismatch')
            samples = m.get('samples', [])
            require([s.get('thresholdS') for s in samples] == list(THRESHOLDS), 'missing or duplicated terminal native thresholds')
            for sample in samples:
                sid = sample.get('sourceStep'); name = sample.get('filename')
                require(type(sid) is int and name == Path(name).name and name.endswith('.native-payload.zlib'), 'unsafe terminal witness path')
                payload_path = path.parent/name
                require(payload_path.is_file() and not payload_path.is_symlink()
                    and 0 < payload_path.stat().st_size <= 4*1024*1024
                    and type(sample.get('payloadBytes')) is int and 0 < sample['payloadBytes'] <= 32*1024*1024
                    and all(core.is_sha(sample.get(k)) for k in ('payloadSHA256', 'compressedSHA256', 'contextSHA256', 'actualMotorControlSHA256')), 'invalid terminal payload metadata')
                require(sid not in self.samples, 'duplicate terminal source step')
                self.samples[sid] = sample, payload_path
            require({p.name for p in path.parent.glob('*.native-payload.zlib')} == {s['filename'] for s in samples}, 'extra terminal payload files')
            self.input_hashes[str(path.relative_to(self.folder))] = core.file_sha(path)
            for _, payload_path in self.samples.values():
                self.input_hashes[str(payload_path.relative_to(self.folder))] = core.file_sha(payload_path)
        except (OSError, KeyError, TypeError, ValueError) as error:
            self.manifest_errors.append(str(error))

    def _record(self, checks):
        for key, value in checks.items(): self.checks[key] = self.checks.get(key, True) and bool(value)
        return checks

    def references(self, config, causal, elapsed, phase, hypothetical):
        expected = core.independent_references(config, causal, elapsed, phase, hypothetical)
        active = causal.get('terminalPelvis')
        if active is not None:
            require(phase == 'stand' and not hypothetical, 'terminal pelvis overlay outside actual stand')
            now = float(causal['arm']['nativeSourceTimeS'])
            expected['orientations'].append(terminal_orientation(self.activation, now))
        return expected

    def bound_reference(self, reference, receipt): return reference
    def correction_clock(self, note, original): return original
    def ready(self, note, row): return True
    def settle_checks(self, receipt, row, state): return {}
    def reference_diagnostic(self, receipt): return receipt['causal']['continuousCom']['diagnostic']
    def goal(self, note, reference):
        return np.asarray(reference['com1'])+np.asarray(note['robustSupportAdmission'].get('comCorrectionM', np.zeros(3)))
    def expected_com(self, note, reference):
        r = note['referenceContext']
        return core.independent_references(r['config'], r['causal'], r['elapsed'], r['phase'])['com'][0]
    def install_native_windows(self, native): self.native = native
    def context_checks(self, receipt, note, row, reference):
        return self._record(active_context_checks(receipt, note, self.activation))

    def observe(self, note, row, reference, elapsed, native):
        self.native = native
        sid = row['sourceStep']
        if sid in self.samples:
            result = self.replay_terminal_witness(note, row, reference)
            self.native_results.append(result)
            return dict(activeTerminalNativeWitnessReplay=result['passed'])
        return {}

    def replay_terminal_witness(self, note, row, reference):
        result = dict(passed=False, sourceStep=row['sourceStep'], physicsIntegrations=0)
        try:
            from source_reference_context import ReferenceContext
            from source_step_probe import Controller
            from source_arm_knee_mobility import native_bytes
            native = self.native; sid = row['sourceStep']; sample, path = self.samples[sid]
            require(sid not in self.seen, 'terminal witness replay duplicated')
            require(not self.manifest_errors, 'invalid terminal native manifest')
            native._check_sources()
            require(self.manifest.get('activationSHA256') == self.activation['activationSHA256']
                and all(self.manifest.get(k) == v for k, v in native.bindings.items()), 'terminal native model/helper/bridge/engine binding mismatch')
            receipt = note['referenceContext']; context = ReferenceContext.from_receipt(receipt)
            now = row['sourceTimeS']; offset = now-self.activation['sourceTimeS']
            require(note['phase'] == context.phase == sample.get('phase') == 'stand'
                and note['stepIndex'] == context.step_index == self.activation['finalStepIndex']
                and context.source_step == sid and core.close(context.source_time, now, 1e-12)
                and core.close(sample.get('sourceTimeS'), now, 1e-12)
                and core.close(sample.get('elapsedS'), offset, 1e-12)
                and sample['thresholdS'] <= offset+1e-12 < sample['thresholdS']+.00500001,
                'terminal witness not first command at bound threshold')
            require(all(active_context_checks(receipt, note, self.activation).values()), 'terminal witness independent orientation/context evidence fails')
            planned = self.references(receipt['config'], receipt['causal'], context.elapsed, 'stand', False)
            require(core.jet_matches(receipt['com'], planned['com'], 3)
                and core.jet_matches(receipt['joints'], planned['joints'], 21)
                and core.equal(receipt['orientations'], planned['orientations']), 'terminal witness independently rebuilt task mismatch')
            require(len(receipt['feet']) == 2 and all(core.jet_matches(value['translation'], expected[:3], 3)
                and core.close(np.asarray(value['orientation']['rotation']).reshape(3, 3), expected[3])
                and core.close(value['orientation']['velocity'], expected[4])
                and core.close(value['orientation']['acceleration'], expected[5])
                for value, expected in zip(receipt['feet'], planned['feet'])), 'terminal witness independent foot task mismatch')
            require(sample['contextSHA256'] == receipt.get('contextSHA256')
                and core.value_sha({k:v for k,v in receipt.items() if k != 'contextSHA256'}) == receipt['contextSHA256']
                and sample['actualMotorControlSHA256'] == note['actualMotorControlSHA256']
                and sample['qpAssemblySHA256'] == note['qpAssemblySHA256'], 'terminal witness note/digest mismatch')
            w = native._ensure_world(); payload = native._payload(sample, path)
            require(w.l.lh_checkpoint_size(w.h) == len(payload), 'terminal checkpoint size mismatch')
            buf = C.create_string_buffer(payload)
            require(w.l.lh_checkpoint_import(w.h, buf, len(payload)) == len(payload), 'terminal checkpoint import rejected')
            del buf
            require(native_bytes(w) == payload, 'terminal exact checkpoint import/export mismatch')
            native._compare('terminal qpos', w.arr(0, (28,)), row['qpos'], (28,), 0.)
            native._compare('terminal qvel', w.arr(1, (27,)), row['qvel'], (27,), 0.)
            counter = w.l.lh_dim(w.h, 6); stamp = w.l.lh_time(w.h)
            require(counter == sid and core.close(stamp, now, 1e-12), 'terminal imported native counter/time mismatch')
            native._compare('terminal anchor body4 FK', w.arr(5, (17, 3, 3))[4], self.fk(row), (3, 3), 1e-10)
            probe = Controller.__new__(Controller); probe.w = w
            for name, value in receipt['config'].items():
                if name == 'targets': value = [(np.asarray(p), np.asarray(R)) for p, R in value]
                elif name == 'flatRotations': value = [np.asarray(R) for R in value]
                elif isinstance(value, list): value = np.asarray(value, dtype=float)
                elif isinstance(value, dict): value = copy.deepcopy(value)
                setattr(probe, name, value)
            probe._reviewed_step_index = context.step_index; probe._reference_epoch = context.epoch
            probe._prepared_reference_context = context
            constraints = core.independent_native_constraints(w, receipt, row, reference, note.get('robustSupportAdmission', {}))
            require(len(constraints) == len(context.capture_constraints)
                and all(core.close(a, b, 1e-8) and core.close(x, y, 1e-7)
                    for (a, x), (b, y) in zip(constraints, context.capture_constraints)), 'terminal immutable capture/arm constraints mismatch')
            control, info = Controller.solve(probe, context.elapsed, 'stand')
            native._compare('terminal actual motor control', control, note['actualMotorControl'], (21,), 1e-9)
            require(core.command_sha(note['actualMotorControl']) == note['actualMotorControlSHA256']
                and np.max(np.abs(control)) <= .95000001
                and info.get('qpAssemblySHA256') == note['qpAssemblySHA256'], 'terminal QP assembly/command mismatch')
            require(native_bytes(w) == payload and w.l.lh_dim(w.h, 6) == counter
                and w.l.lh_time(w.h) == stamp, 'terminal QP replay changed native bytes/counter/time')
            self.seen.add(sid)
            result.update(passed=True, thresholdS=sample['thresholdS'], nativeVerified=True,
                independentTerminalOrientationRebuilt=True, nativeBytesCounterTimeUnchanged=True,
                actualReferenceAndCommandVerified=True, qpAssemblyVerified=True)
        except Exception as error:
            result['error'] = str(error)
        return result

    def declarations(self, config, receipt, cadence):
        return (config.get('profile') == receipt.get('profile') == PROFILE
            and config.get('mode') == receipt.get('mode') == 'continuous-com-causal'
            and config.get('requestedSteps') in (1, 10)
            and config.get('terminalPelvisEnabled') is True and receipt.get('terminalPelvisEnabled') is True
            and config.get('featureProfile') == receipt.get('featureProfile') == PROFILE
            and core.equal(config.get('terminalPelvis'), terminal_policy())
            and core.equal(receipt.get('terminalPelvis'), terminal_policy())
            and core.equal(cadence.get('terminalPelvis'), terminal_policy())
            and core.equal(config.get('terminalPelvisActivation'), self.activation)
            and core.equal(receipt.get('terminalPelvisActivation'), self.activation)
            and config.get('inheritedPhysicalGatesUnchanged') is True
            and receipt.get('inheritedPhysicalGatesUnchanged') is True
            and config.get('oldProfileQualificationClaim') is False and receipt.get('oldProfileQualificationClaim') is False
            and config.get('finalPlanQualificationClaim') is False and receipt.get('finalPlanQualificationClaim') is False
            and cadence.get('reviewedReferenceProfile', {}).get('featureProfile') == PROFILE
            and 'source_terminal_pelvis.py' in receipt.get('codeSHA256AtStart', {}))

    def finish(self, result):
        terminal_checks = dict(self.checks)
        terminal_checks.update(independentTerminalActivationReferenceAndQuiet=self.journal['passed'],
            terminalEvidenceUnchangedDuringAudit=bool(self.input_hashes) and all((self.folder/name).is_file()
                and core.file_sha(self.folder/name) == digest for name, digest in self.input_hashes.items()))
        reference_pass = bool(terminal_checks) and all(terminal_checks.values())
        native_pass = (not self.manifest_errors and len(self.samples) == len(THRESHOLDS)
            and self.seen == set(self.samples) and len(self.native_results) == len(THRESHOLDS)
            and all(r['passed'] for r in self.native_results))
        native_proof = dict(passed=native_pass, expectedWitnesses=3, verifiedWitnesses=len(self.seen),
            physicsIntegrations=0, oneIsolatedScratchWorld=True, localWitnessClaimOnly=True,
            errors=self.manifest_errors, results=self.native_results)
        result['checks']['independentTerminalReferenceAndQuietReplay'] = reference_pass
        result['checks']['activeTerminalNativeAssemblyAndCommandReplay'] = native_pass
        physical = all(result['checks'].values())
        result.update(profile=PROFILE, reviewedWalkPilotPhysicalPass=False,
            reviewedWalkDiagnosticPass=False, reviewedWalkTenStepPhysicalPass=False,
            reviewedWalkFinalPlanPass=False, terminalPelvisReferenceAndQuietPass=reference_pass,
            terminalPelvisPilotPhysicalPass=physical and result['requestedSteps'] == 1,
            terminalPelvisTenStepPhysicalPass=physical and result['requestedSteps'] == 10,
            terminalPelvisFinalPlanPass=False, terminalReferenceReplay=self.journal,
            terminalFeatureChecks=terminal_checks, terminalNativeWitnessReplay=native_proof,
            improvements=None, finalVisualComparisonIndependentlyAudited=False)
        result['failedChecks'] = [k for k,v in result['checks'].items() if not v]
        result['hashes'].update(self.input_hashes)
        result['auditorSHA256'] = core.file_sha(Path(__file__))
        result['coreReviewedAuditorSHA256'] = core.file_sha(Path(core.__file__))
        result['evidenceLimit'] += (' Terminal body-4 references, activation and fresh endpoint-held quiet are independently reconstructed; three exact changed-task QP witnesses establish only their local saved assemblies. Walking-phase pelvis alignment and final visual improvement are not established.')
        return result


def audit(folder):
    return core.audit(folder, reference_policy=TerminalPelvisPolicy(folder))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path); parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); result = audit(args.folder)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    keys = ('terminalPelvisReferenceAndQuietPass', 'terminalPelvisPilotPhysicalPass',
        'terminalPelvisTenStepPhysicalPass', 'terminalPelvisFinalPlanPass')
    print(json.dumps({key: result[key] for key in keys}|dict(failedChecks=result['failedChecks']), indent=2))
    raise SystemExit(0 if result['terminalPelvisPilotPhysicalPass'] or result['terminalPelvisTenStepPhysicalPass'] else 1)


if __name__ == '__main__': main()
