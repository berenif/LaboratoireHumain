#!/usr/bin/env python3
"""Bounded, terminal-only soft pelvis experiment; no walking reference changes.

The actual pelvis orientation is frozen once at the final settle-to-stand
handoff, after measured landing and 100 ms of measured loaded double support.
Only an extra soft body-4 Orientation is added. This is an unqualified native
pilot until the independent terminal-profile audit and physical run pass.
"""
import argparse
from collections import deque
import copy
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
from pathlib import Path
import shutil
import time
import zlib

import numpy as np
from scipy.spatial.transform import Rotation

from source_contact_impedance_probe import prepare_impedance
from source_arm_knee_mobility import native_bytes
from source_reference_context import Orientation, canonical, digest
from source_reviewed_walk import ReviewedController, ReviewedSteps, wall_cap_seconds
from source_step_probe import clock, sha

PROFILE = 'terminal-pelvis-soft-v1'
MODE = 'continuous-com-causal'
DURATION_S = 4.
MAXIMUM_ANGLE_RAD = math.radians(6.)
MAXIMUM_WEIGHT = .5
PHYSICS_STEP_S = .00125
RUNTIME = ('source_terminal_pelvis.py',)


def terminal_policy():
    return dict(schema=1, profile=PROFILE, enabled=True, body=4,
        activation='once at final measured settle-to-stand handoff',
        measuredFinalLandingRequired=True, minimumLoadedDoubleSupportS=.1,
        minimumLoadedHistorySamples=81, minimumEachFootLoadN=3.,
        minimumTotalWeightFraction=.8, actualNativeHistoryOnly=True,
        target='same-yaw upright, left-trivialized SO(3) arc from frozen actual pelvis',
        maximumCorrectionAngleRad=MAXIMUM_ANGLE_RAD, durationS=DURATION_S,
        maximumWeight=MAXIMUM_WEIGHT, orientationKp=40., orientationKd=12.,
        referenceAndWeightClock='same 4s quintic', endpointHeldAfterCompletion=True,
        quietStarts='only integrations after first complete endpoint command',
        torsoComJointFootArmBarrierUnloadReferencesUnchanged=True,
        transferForecastOnStand=False, optimizerEnabled=False, hardConeEnabled=False,
        kneeReferenceChanged=False, riskHistoryChanged=False,
        originalPhysicalGatesUnchanged=True, originalPhaseDeadlinesUnchanged=True)


@dataclass(frozen=True)
class LoadedSample:
    source_step: int
    source_time: float
    feet_load_n: tuple

    def receipt(self):
        return dict(sourceStep=self.source_step, sourceTimeS=self.source_time,
            feetLoadN=list(self.feet_load_n))


@dataclass(frozen=True)
class TerminalActivation:
    """Owned immutable scalar data; no live measured rotation can replace it."""
    receipt_json: str

    def receipt(self):
        return json.loads(self.receipt_json)

    @property
    def key(self):
        return self.receipt()['activationSHA256']

    def sample(self, source_time):
        receipt = self.receipt()
        elapsed = float(source_time)-receipt['sourceTimeS']
        if not math.isfinite(elapsed) or elapsed < -1e-10:
            raise ValueError('terminal reference precedes its frozen activation')
        s, ds, dds = clock(max(0., elapsed), DURATION_S)
        axis = np.array(receipt['axisWorldRad'])
        R0 = np.array(receipt['initialRotationSource'])
        R = Rotation.from_rotvec(axis*s).as_matrix()@R0
        orientation = Orientation.make(4, R, axis*ds, axis*dds,
            kp=40., kd=12., weight=MAXIMUM_WEIGHT*s)
        complete = elapsed >= DURATION_S-1e-10
        return dict(profile=PROFILE, activation=receipt, activationSHA256=self.key,
            clock=[s, ds, dds], elapsedS=max(0., elapsed), complete=complete,
            endpointHeld=complete, orientation=asdict(orientation))


class TerminalPelvisController(ReviewedController):
    def install_reviewed(self, step_index, mode):
        if mode != MODE:
            raise ValueError('terminal pelvis experiment requires unchanged causal COM mode')
        super().install_reviewed(step_index, mode)
        self._terminal_final_step_index = None
        self._terminal_landing_json = None
        self._terminal_loaded_history = deque()
        self._terminal_activation = None
        self._terminal_endpoint_command = None
        self._terminal_saved_witnesses = set()
        self._terminal_witnesses = []

    def _command_identity(self, t, phase):
        return super()._command_identity(t, phase)+(PROFILE,
            None if self._terminal_activation is None else self._terminal_activation.key,)

    def _terminal_current_row(self, row):
        if (row.get('stepIndex') != self._reviewed_step_index
                or row.get('sourceStep') != self.w.l.lh_dim(self.w.h, 6)
                or abs(row.get('sourceTimeS', -1.)-self.w.l.lh_time(self.w.h)) > 1e-10):
            raise ValueError('terminal history must use the current actual native sample')

    def record_terminal_native_sample(self, row):
        """Read-only new history; never writes the existing robust risk history."""
        if self._terminal_final_step_index != self._reviewed_step_index:
            return
        self._terminal_current_row(row)
        loads = tuple(float(f['loadN']) for f in row['feet'])
        if len(loads) != 2 or not all(math.isfinite(x) for x in loads):
            raise ValueError('invalid terminal measured foot loads')
        sample = LoadedSample(int(row['sourceStep']), float(row['sourceTimeS']), loads)
        history = self._terminal_loaded_history
        if history and sample.source_step == history[-1].source_step:
            if sample != history[-1]:
                raise ValueError('terminal native history sample was replaced')
            return
        if history and (sample.source_step != history[-1].source_step+1
                or abs(sample.source_time-history[-1].source_time-PHYSICS_STEP_S) > 1e-10):
            history.clear()
        if min(loads) < 3. or sum(loads) < .8*self.w.mass*9.81:
            history.clear()
            return
        history.append(sample)
        while len(history) > 1 and history[1].source_time <= sample.source_time-.1+1e-10:
            history.popleft()

    def prepare_phase_handoff(self, old_phase, new_phase, row, *, landing_consecutive_samples):
        super().prepare_phase_handoff(old_phase, new_phase, row,
            landing_consecutive_samples=landing_consecutive_samples)
        if self._terminal_final_step_index != self._reviewed_step_index:
            return
        if new_phase not in ('settle', 'stand'):
            return
        self._terminal_current_row(row)
        if new_phase == 'settle':
            if (old_phase != 'land' or row.get('phase') != 'land'
                    or self._terminal_landing_json is not None
                    or type(landing_consecutive_samples) is not int or landing_consecutive_samples < 80
                    or any(f['loadN'] < 3. for f in row['feet'])
                    or row['swingForwardDisplacementM'] < row['requestedForwardM']):
                raise ValueError('invalid or replaced terminal final measured landing')
            self._terminal_landing_json = canonical(dict(sourceStep=row['sourceStep'],
                sourceTimeS=row['sourceTimeS'], stepIndex=self._reviewed_step_index,
                landingConsecutiveSamples=landing_consecutive_samples,
                feetLoadN=[float(f['loadN']) for f in row['feet']],
                forwardM=row['swingForwardDisplacementM'], requestedForwardM=row['requestedForwardM']))
            return
        if (old_phase != 'settle' or row.get('phase') != 'settle'
                or self._terminal_activation is not None or self._terminal_landing_json is None):
            raise ValueError('invalid or replaced terminal stand activation')
        history = list(self._terminal_loaded_history)
        landing = json.loads(self._terminal_landing_json)
        if (len(history) < 81 or history[-1].source_step != row['sourceStep']
                or abs(history[-1].source_time-row['sourceTimeS']) > 1e-10
                or history[-1].source_time-history[0].source_time < .1-1e-10
                or history[0].source_step < landing['sourceStep']
                or history[0].source_time < landing['sourceTimeS']-1e-10):
            raise ValueError('terminal stand entry lacks 100ms measured loaded double support after landing')
        # This is the only read that becomes the frozen orientation anchor.
        R0 = np.array(self.w.arr(5, (17, 3, 3))[4], dtype=float, copy=True)
        if (not np.isfinite(R0).all() or np.max(abs(R0.T@R0-np.eye(3))) > 1e-8
                or abs(np.linalg.det(R0)-1.) > 1e-8):
            raise ValueError('invalid actual pelvis rotation at terminal entry')
        yaw = math.atan2(R0[1, 0], R0[0, 0])
        upright = Rotation.from_euler('Z', yaw).as_matrix()
        axis = Rotation.from_matrix(upright@R0.T).as_rotvec()
        angle = float(np.linalg.norm(axis))
        if angle > MAXIMUM_ANGLE_RAD:
            axis *= MAXIMUM_ANGLE_RAD/angle
        receipt = dict(schema=1, profile=PROFILE, body=4,
            stepIndex=self._reviewed_step_index, finalStepIndex=self._terminal_final_step_index,
            sourceStep=row['sourceStep'], sourceTimeS=row['sourceTimeS'], standEntry=True,
            initialRotationSource=R0.tolist(), yawRad=yaw, uprightRotationSource=upright.tolist(),
            axisWorldRad=axis.tolist(),
            boundedEndpointRotationSource=(Rotation.from_rotvec(axis).as_matrix()@R0).tolist(),
            correctionAngleRad=float(np.linalg.norm(axis)), durationS=DURATION_S,
            maximumWeight=MAXIMUM_WEIGHT, measuredFinalLanding=landing,
            loadedHistory=dict(minimumDurationS=.1, minimumEachFootLoadN=3.,
                minimumTotalWeightFraction=.8, bodyWeightN=self.w.mass*9.81,
                sourceStartStep=history[0].source_step, sourceEndStep=history[-1].source_step,
                sourceStartTimeS=history[0].source_time, sourceEndTimeS=history[-1].source_time,
                durationS=history[-1].source_time-history[0].source_time,
                samples=[sample.receipt() for sample in history]))
        receipt['activationSHA256'] = digest(receipt)
        self._terminal_activation = TerminalActivation(canonical(receipt))
        self._command_cache = None
        if hasattr(self, '_terminal_evidence_path'):
            path = self._terminal_evidence_path
            if path.exists():
                raise ValueError('terminal frozen activation evidence already exists')
            path.write_text(json.dumps(receipt, indent=2)+'\n')

    def transform_reviewed_reference_context(self, context):
        # Exact preterminal object identity is intentional: no task or QP field
        # changes during transfer/lift/translate/land/settle, on any step.
        if context.phase != 'stand':
            return context
        if self._terminal_activation is None:
            raise ValueError('terminal stand reference lacks qualified frozen activation')
        if context.hypothetical_changes:
            raise ValueError('terminal stand cannot be used as a transfer forecast')
        if any(o.body == 4 for o in context.orientations):
            raise ValueError('terminal pelvis would replace an existing orientation owner')
        sample = self._terminal_activation.sample(context.source_time)
        orientation = Orientation(**{name: tuple(value) if isinstance(value, list) else value
            for name, value in sample['orientation'].items()})
        causal = json.loads(context.causal_json)
        causal['terminalPelvis'] = sample
        diagnostics = json.loads(context.diagnostics_json)
        diagnostics['terminalPelvis'] = copy.deepcopy(sample)
        return replace(context, orientations=context.orientations+(orientation,),
            causal_json=canonical(causal), diagnostics_json=canonical(diagnostics))

    def _forecast_unload_risk(self, *args, **kwargs):
        context = kwargs.get('reference_context', self._prepared_reference_context)
        if context is not None and context.phase == 'stand':
            raise ValueError('transfer forecast is forbidden on terminal stand')
        return super()._forecast_unload_risk(*args, **kwargs)

    def solve(self, t, phase='transfer'):
        u, info = super().solve(t, phase)
        info['reviewedWalk']['featureProfile'] = PROFILE
        activation = self._terminal_activation
        sample = None if activation is None or phase != 'stand' else activation.sample(self.w.l.lh_time(self.w.h))
        if sample is not None and sample['complete'] and self._terminal_endpoint_command is None:
            self._terminal_endpoint_command = (self.w.l.lh_dim(self.w.h, 6), self.w.l.lh_time(self.w.h))
        info['terminalPelvis'] = dict(schema=1, profile=PROFILE, mode=MODE,
            finalStepIndex=self._terminal_final_step_index, activated=activation is not None,
            activationSHA256=None if activation is None else activation.key,
            sample=sample, endpointCommand=None if self._terminal_endpoint_command is None else
                dict(sourceStep=self._terminal_endpoint_command[0], sourceTimeS=self._terminal_endpoint_command[1]),
            originalPhysicalGatesUnchanged=True, walkingReferencesUnchanged=True)
        if sample is not None and hasattr(self, '_terminal_native_folder'):
            self._retain_terminal_witnesses(sample, info)
        return u, info

    def _retain_terminal_witnesses(self, sample, info):
        for threshold in (.5, 4., 4.1):
            if sample['elapsedS'] < threshold-1e-10 or threshold in self._terminal_saved_witnesses:
                continue
            payload = native_bytes(self.w)
            folder = self._terminal_native_folder
            folder.mkdir(exist_ok=True)
            counter = self.w.l.lh_dim(self.w.h, 6)
            name = f'after-{threshold:.1f}s-source-{counter:06d}.native-payload.zlib'
            path = folder/name
            path.write_bytes(zlib.compress(payload, 1))
            self._terminal_witnesses.append(dict(thresholdS=threshold,
                sourceStep=counter, sourceTimeS=self.w.l.lh_time(self.w.h),
                elapsedS=sample['elapsedS'], phase='stand', payloadBytes=len(payload),
                payloadSHA256=hashlib.sha256(payload).hexdigest(), filename=name,
                compressedSHA256=sha(path), contextSHA256=info['referenceContext']['contextSHA256'],
                actualMotorControlSHA256=info['actualMotorControlSHA256'],
                qpAssemblySHA256=info['qpAssemblySHA256']))
            manifest = dict(schema=1, profile=PROFILE,
                modelSHA256=sha(self.w.xml), helperSHA256=sha(self.w.helper),
                bridgeSHA256=sha(self.w.base/'source_bridge_v2.so'),
                engineSHA256=sha(self.w.base/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3'),
                stepIndex=self._reviewed_step_index,
                activationSHA256=self._terminal_activation.key, samples=self._terminal_witnesses,
                physicsIntegrations=0, actualCommandNotYetApplied=True,
                payloadEncoding='zlib-compressed exact lh_checkpoint_export payload')
            (folder/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
            self._terminal_saved_witnesses.add(threshold)


class TerminalPelvisSteps(ReviewedSteps):
    reviewed_controller_class = TerminalPelvisController

    def __init__(self, base, helper, output, count=1):
        if count not in (1, 10):
            raise ValueError('terminal pelvis requests are one-step pilot or ten-step final only')
        super().__init__(base, helper, output, count, MODE)
        self.code_at_start.update({name: sha(Path(__file__).with_name(name)) for name in RUNTIME})

    def begin_step(self):
        super().begin_step()
        self.controller._terminal_final_step_index = len(self.distances)
        self.controller._terminal_evidence_path = self.folder/'terminal-pelvis-activation.json'
        self.controller._terminal_native_folder = self.folder/'terminal-pelvis-native'

    def decorate(self, row):
        row = super().decorate(row)
        controller = getattr(self, 'controller', None)
        row['actualPelvisRotationSource'] = self.world.arr(5, (17, 3, 3))[4].copy().tolist()
        if not isinstance(controller, TerminalPelvisController):
            return row
        if row['sourceStep'] > 0:
            controller.record_terminal_native_sample(row)
        activation = controller._terminal_activation
        sample = None if activation is None else activation.sample(row['sourceTimeS'])
        endpoint = controller._terminal_endpoint_command
        eligible = bool(row['phase'] == 'stand' and sample is not None and sample['complete']
            and row['sourceTimeS'] > activation.receipt()['sourceTimeS']+DURATION_S+1e-10
            and endpoint is not None and row['sourceStep'] > endpoint[0])
        # The inherited quiet predicate still owns counting. Suppress it until
        # the correction and an actually applied endpoint command are complete.
        if row['phase'] == 'stand' and not eligible:
            self.quiet_run = 0
            row['quietConsecutiveS'] = 0.
        receipt = None if activation is None else activation.receipt()
        row['terminalPelvis'] = dict(profile=PROFILE, enabled=True, activated=receipt is not None,
            activationSHA256=None if receipt is None else receipt['activationSHA256'],
            activationSourceStep=None if receipt is None else receipt['sourceStep'],
            activationSourceTimeS=None if receipt is None else receipt['sourceTimeS'],
            elapsedS=None if sample is None else sample['elapsedS'],
            rampComplete=False if sample is None else sample['complete'], quietEligible=eligible,
            endpointCommandSourceStep=None if endpoint is None else endpoint[0],
            endpointCommandSourceTimeS=None if endpoint is None else endpoint[1],
            quietCountedSamples=self.quiet_run if row['phase'] == 'stand' else 0)
        return row

    def save(self, reason):
        super().save(reason)
        if not hasattr(self, 'folder'):
            return
        controller = getattr(self, 'controller', None)
        activation = getattr(controller, '_terminal_activation', None)
        receipt = None if activation is None else activation.receipt()
        for name in ('reviewed-walk-run.json', 'reviewed-walk-config.json'):
            path = self.folder/name
            if not path.exists():
                continue
            value = json.loads(path.read_text())
            value.update(profile=PROFILE, featureProfile=PROFILE, terminalPelvis=terminal_policy(),
                terminalPelvisActivation=receipt, terminalPelvisEnabled=True,
                finalPlanQualificationClaim=False, oldProfileQualificationClaim=False,
                inheritedPhysicalGatesUnchanged=True)
            path.write_text(json.dumps(value, indent=2)+'\n')
        path = self.folder/'cadenced-step-config.json'
        if path.exists():
            value = json.loads(path.read_text())
            value['reviewedReferenceProfile']['featureProfile'] = PROFILE
            value['terminalPelvis'] = terminal_policy()
            path.write_text(json.dumps(value, indent=2)+'\n')


def argument_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--inspector', type=Path, required=True)
    parser.add_argument('--count', type=int, choices=(1, 10), default=1)
    parser.add_argument('--wall-cap-s', type=wall_cap_seconds, default=3600.,
        help='Compute-time cap only; native clocks and deadlines remain unchanged (default: 3600).')
    return parser


def main():
    args = argument_parser().parse_args()
    session, helper, preparation = prepare_impedance(args.output, args.inspector, 8.)
    retained = args.output/'contact-inspector.so'
    shutil.copyfile(args.inspector, retained)
    if sha(retained) != preparation['inspectorSHA256']:
        raise ValueError('retained read-only inspector differs from preparation')
    live = TerminalPelvisSteps(session, helper, args.output/'run', args.count)
    live.paused = False
    started = time.monotonic()
    live.execution_wall_cap_s = args.wall_cap_s
    live.execution_started = started
    try:
        while not live.finished and not live.failure:
            if time.monotonic()-started >= args.wall_cap_s:
                raise RuntimeError(f'{args.wall_cap_s:g}s process cap')
            live.advance(4)
            if live.steps % 800 == 0:
                print('progress', live.steps*PHYSICS_STEP_S, 'step', live.step_index+1,
                    live.phase, 'slip', live.slip, 'quiet', live.quiet_run*PHYSICS_STEP_S, flush=True)
        print(live.failure or 'terminal-pelvis-physical-sequence-plus5squiet', flush=True)
    except Exception as error:
        live.failure = repr(error)
        raise
    finally:
        live.close()
    raise SystemExit(0 if live.finished else 1)


if __name__ == '__main__':
    main()
