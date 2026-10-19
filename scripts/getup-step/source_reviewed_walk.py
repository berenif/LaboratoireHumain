#!/usr/bin/env python3
"""Opt-in implementation of the five-reviewed physical gait correction plan.

The first mode is reference-infrastructure parity, with unchanged motion. Later
modes are isolated factors. No mode changes model friction, motors, admissions,
loaded-settle readiness, global slip accounting, or native integration cadence.
"""
import argparse
from collections import deque
import copy
import hashlib
import json
import shutil
from pathlib import Path
import time
import zlib
import ctypes as C
from dataclasses import asdict

import numpy as np

from source_step_probe import Controller, PTR, clock, ptr, sha
from source_cadenced_ten_steps import CadencedSteps
from source_robust_support_steps import RobustSupportController, actual_stance_patch
from source_contact_impedance_probe import prepare_impedance
from source_opposed_arm_swing import limb_telemetry
from source_arm_knee_mobility import native_bytes
from source_solved_wrench_diagnostics import reader as read_solved_wrenches
from source_continuous_com import QuinticSegment, plan_forward, plan_projected, validate_reference
from source_reference_context import (REFERENCE_VERSION, canonical, digest, make_context,
    nominal_com, prospective_unload_context)
from source_soft_load_preference import AcquiredLoad, prepare_preference
from source_causal_com_overlap import certify_overlap

MODES = ('baseline-context', 'continuous-com', 'continuous-com-projected', 'continuous-com-load', 'continuous-com-causal')
CONFIG_FIELDS = ('q0','com0','com1','targets','rootR','flatRotations','radii','floorTargets',
    'swing_side','stance_side','swing_body','self_avoidance','B','heading','landing',
    'step_distance','finalCom','lean_axis','phase_durations','joint_reserve','posture_start',
    'shear_weight','trajectory_time_scale','arm_kp','arm_kd','arm_weight','leg_origin',
    'arm_start','placement','arm_placement_duration','loaded_settle_capture_barrier',
    'knee_goal')


def serial(value):
    return json.loads(json.dumps(value, default=lambda x: x.tolist(), allow_nan=False))


def control_sha(control):
    return hashlib.sha256(np.ascontiguousarray(control, dtype='<f8').tobytes()).hexdigest()


def causal_selection_policy():
    return dict(schema=1, policy='actual-loaded-history-no-nominal-wait-v1',
        stateOnlySupportSelection=True,
        nominalRiskHistoryTimerRequired=False, nominalSelectionTimerRequired=False,
        minimumLoadedHistoryS=.1, minimumHistorySamples=21,
        minimumActualStanceWeightFraction=.8, minimumRiskAgreementFrequency=.8,
        actualSurvivorsRequired=True, minimumCorrectionDurationS=1.,
        maximumCorrectionM=.012, maximumCorrectionSpeedMps=.025,
        maximumCorrectionAccelerationMps2=.15,
        maximumTotalSpeedMps=.19260552331163003,
        maximumTotalAccelerationMps2=.3901818436690813,
        originalMinimumTransferAndMeasuredLiftGatesUnchanged=True,
        transferReferenceValidation='every command after state-qualified selection; unchanged legacy transfer before selection')


class ReviewedController(RobustSupportController):
    def _complete_reference_context(self, context, t, phase):
        """Pure optional visual-factor seam; original safety/history owners stay.

        The effective context is finalized BEFORE either actual or prospective
        QP consumption. Feature implementations must declare and independently
        replay their references; this seam alone grants no qualification.
        """
        transform = getattr(self, 'transform_reviewed_reference_context', None)
        if transform is None:
            return context
        result = transform(context)
        result.validate(self, t, phase)
        if (result.arms is not context.arms or result.arm_gains != context.arm_gains
                or result.capture_constraints != context.capture_constraints
                or result.unload_admitted != context.unload_admitted
                or result.hypothetical_changes != context.hypothetical_changes):
            raise ValueError('visual reference hook changed an existing safety/history owner')
        return result

    def install_reviewed(self, step_index, mode):
        self._reviewed_step_index = int(step_index)
        self._reviewed_mode = mode
        self._reference_epoch = 0
        self._measurement_cache = None
        self._command_cache = None
        self._observed_command = None
        self._prepared_reference_context = None
        self._forecast_records = []
        self._robust_history = deque()
        self.robust_support = dict(selected=False, causalHistoryOnly=True)
        self.robust_support_barrier = []
        self.capture_barrier = []
        self.arm_range_barrier = []
        self._state_only_support_selection = mode == 'continuous-com-causal'
        if self._state_only_support_selection:
            self._causal_overlap_plan = None
        if mode == 'continuous-com-load':
            self._soft_load_anchor = None
            self._prepared_soft_load_preference = None
            self._soft_load_saved_targets = set()
            self._soft_load_witnesses = []

    def admit_robust_reference(self, t, delta, duration):
        if not self._state_only_support_selection:
            return True
        certificate = certify_overlap(self.com1-self.com0,
            self.phase_duration('transfer'), delta, t, duration)
        if not certificate['passed']:
            self.robust_support.update(lastAdmissionRejection='summed causal COM jets exceed frozen envelope',
                lastRejectedCausalOverlap=certificate)
            return False
        self._causal_overlap_plan = certificate
        return True

    def prepare_phase_handoff(self, old_phase, new_phase, row, *, landing_consecutive_samples):
        if self._reviewed_mode != 'continuous-com-load' or new_phase != 'settle':
            return
        if (old_phase != 'land' or self._soft_load_anchor is not None
                or row.get('phase') != 'land' or row.get('stepIndex') != self._reviewed_step_index
                or type(landing_consecutive_samples) is not int or landing_consecutive_samples < 80
                or any(foot['loadN'] < 3. for foot in row['feet'])
                or row['sourceStep'] != self.w.l.lh_dim(self.w.h, 6)
                or abs(row['sourceTimeS']-self.w.l.lh_time(self.w.h)) > 1e-10):
            raise ValueError('invalid or replaced acquired-load handoff')
        self._soft_load_anchor = AcquiredLoad(row['sourceStep'], row['sourceTimeS'],
            self._reviewed_step_index, self.swing_side,
            row['feet'][self.swing_side]['loadN'], self.w.mass*9.81,
            landing_consecutive_samples)

    def prepared_arm_measurement(self):
        w = self.w
        key = (w.l.lh_dim(w.h, 6), w.l.lh_time(w.h), self._reviewed_step_index)
        if self._measurement_cache is not None and self._measurement_cache[0] == key:
            return tuple(x.copy() for x in self._measurement_cache[1])
        legs, rate, _ = limb_telemetry(w, 'leg')
        anchor = dict(previousTimeS=self.previous_time,
            previousLegRateRadps=None if self.previous_leg_rate is None else self.previous_leg_rate.tolist())
        dt = None if self.previous_time is None else key[1]-self.previous_time
        acceleration = (rate-self.previous_leg_rate)/dt if dt is not None and dt > 1e-8 else np.zeros(2)
        if dt is not None and dt < 0.:
            raise ValueError('arm measurement history is not causal')
        self.previous_time, self.previous_leg_rate = key[1], rate.copy()
        self._arm_causal = dict(anchor=anchor, measuredLegPitchRad=legs.tolist(),
            measuredLegPitchRateRadps=rate.tolist(), measuredLegPitchAccelerationRadps2=acceleration.tolist(),
            nativeSourceStep=key[0], nativeSourceTimeS=key[1], measuredOncePerNativeCommand=True)
        self._measurement_cache = (key, (legs.copy(), rate.copy(), acceleration.copy()))
        return legs, rate, acceleration

    def _command_identity(self, t, phase):
        # Same-time reference edits invalidate pure assembly, never derivatives.
        config = {k: getattr(self, k) for k in CONFIG_FIELDS if hasattr(self, k)}
        config['unload_admitted'] = bool(getattr(self, 'unload_admitted', False))
        config['referenceVersion'] = REFERENCE_VERSION
        config['mode'] = self._reviewed_mode
        if self._reviewed_mode == 'continuous-com-load':
            config['softLoadAcquisition'] = (None if self._soft_load_anchor is None
                else asdict(self._soft_load_anchor))
        return (self.w.l.lh_dim(self.w.h, 6), self.w.l.lh_time(self.w.h), self._reviewed_step_index,
            phase, float(t), self._reference_epoch, digest(serial(config)))

    def _prepared_inputs_token(self, t, phase):
        """Bound every mutable task/barrier input across the read-only observer.

        Forecast/rejection telemetry and causal vote history are intentionally
        excluded: they are not task inputs. Native q/v, held command, measured
        loads/support and cached transforms cover the actual geometry; the model
        stays source/checkpoint bound and the observer's forecast checks all
        native bytes. A changed support/correction/reference never reuses draft.
        """
        robust = {name: getattr(self, name, None) for name in
            ('_robust_delta', '_robust_start', '_robust_duration', '_robust_clock',
             '_robust_weak', '_robust_survivors', '_robust_initial_reserve')}
        robust['selected'] = bool(self.robust_support.get('selected'))
        tasks = {name: getattr(self, name, None) for name in
            ('arm_posture', 'arm_velocity', 'arm_acceleration', '_seek_jet',
             '_com_path_plan', '_com_reference_diagnostic')}
        tasks.update(robust=robust, mappingRanges=self._mapping.ranges,
            mappingNeutral=self._mapping.neutral_pitch,
            path=self._com_path.receipt() if hasattr(self, '_com_path') else None,
            settle=self._com_settle.receipt() if hasattr(self, '_com_settle') else None)
        if self._reviewed_mode == 'continuous-com-load':
            tasks['softLoadPreference'] = (None if self._prepared_soft_load_preference is None
                else self._prepared_soft_load_preference.receipt())
        if self._state_only_support_selection:
            tasks['causalOverlap'] = self._causal_overlap_plan
        w = self.w
        native = tuple(w.arr(index, shape).tobytes() for index, shape in
            ((0, (28,)), (1, (27,)), (2, (21,)), (3, (17, 3)),
             (5, (17, 3, 3)), (6, (17, 3)), (20, (17,)), (21, (128,))))
        return (self._command_identity(t, phase), digest(serial(tasks)),
                w.l.lh_support_count(w.h), native)

    def corrected_com_reference(self, t, phase):
        target, velocity, acceleration = nominal_com(self, t, phase)
        if self.robust_support.get('selected', False):
            if phase == 'transfer':
                s, ds, dds = clock(t-self._robust_start, self._robust_duration)
            elif phase == 'settle':
                s, ds, dds = clock(t, self.phase_duration('settle')); s, ds, dds = 1.-s, -ds, -dds
            elif phase == 'stand':
                s, ds, dds = 0., 0., 0.
            else:
                s, ds, dds = 1., 0., 0.
            self._robust_clock = (s, ds, dds)
            target = target+s*self._robust_delta
            velocity = velocity+ds*self._robust_delta
            acceleration = acceleration+dds*self._robust_delta
        if self._reviewed_mode not in ('continuous-com', 'continuous-com-projected', 'continuous-com-load', 'continuous-com-causal'):
            return target, velocity, acceleration
        now = self.w.l.lh_time(self.w.h)
        if phase in ('translate', 'land'):
            if not hasattr(self, '_com_path'):
                if phase != 'translate':
                    raise ValueError('continuous COM phase history unavailable')
                planner = plan_projected if self._reviewed_mode == 'continuous-com-projected' else plan_forward
                self._com_path, self._com_path_plan = planner(self, now-t)
            p, v, a = self._com_path.evaluate(now)
            target, velocity, acceleration = target+p, velocity+v, acceleration+a
        elif phase == 'settle' and hasattr(self, '_com_path'):
            if not hasattr(self, '_com_settle'):
                start = now-t
                p, v, a = self._com_path.evaluate(start)
                initial = (self.com1+self._robust_delta+p, v, a)
                self._com_settle = QuinticSegment.between(start, self.phase_duration('settle'),
                    initial, (self.finalCom.copy(), np.zeros(3), np.zeros(3)))
            target, velocity, acceleration = self._com_settle.evaluate(now)
        if phase in ('translate', 'land', 'settle', 'stand'):
            self._com_reference_diagnostic = validate_reference(self, phase, (target, velocity, acceleration))
        elif (self._reviewed_mode == 'continuous-com-causal' and phase == 'transfer'
                and self.robust_support.get('selected', False)):
            # Before selection the historical transfer remains unchanged,
            # including the inherited zero-counter contact-cache startup. Once
            # the measured loaded window exists, validate the complete jet
            # against current actual support on every transfer command too.
            self._com_reference_diagnostic = validate_reference(self, phase, (target, velocity, acceleration))
        return target, velocity, acceleration

    def prepare_reference_context(self, t, phase):
        # Arm targets are already prepared by the historical delayed-arm law.
        before_seek = dict(timeS=getattr(self, '_seek_time', None), depthM=getattr(self, '_seek_depth', 0.),
            rateMps=getattr(self, '_seek_rate', 0.))
        seek = self.landing_adjustment(t, phase)
        draft_com = self.corrected_com_reference(t, phase)
        causal = dict(arm=self._arm_causal, contactSeekBefore=before_seek,
            contactSeekAfter=dict(timeS=self._seek_time, jet=serial(self._seek_jet), **serial(self.contact_seek)),
            robustSelectedBefore=bool(self.robust_support.get('selected')),
            preparedQ0=self.q0.tolist(), armPosture=self.arm_posture.tolist(),
            armVelocity=self.arm_velocity.tolist(), armAcceleration=self.arm_acceleration.tolist(),
            armHistoryAfter=dict(previousTimeS=self.previous_time, previousLegRateRadps=self.previous_leg_rate.tolist()),
            persistentLegOrigin=self.leg_origin.tolist(), armMappingNeutralPitch=self._mapping.neutral_pitch.tolist(),
            robustCorrection=dict(selected=bool(self.robust_support.get('selected')),
                deltaM=getattr(self, '_robust_delta', np.zeros(3)).tolist(),
                startTransferS=getattr(self, '_robust_start', None),
                durationS=getattr(self, '_robust_duration', None),
                clock=serial(getattr(self, '_robust_clock', (0., 0., 0.)))))
        causal['continuousCom'] = dict(path=getattr(self, '_com_path', None).receipt() if hasattr(self, '_com_path') else None,
            settle=getattr(self, '_com_settle', None).receipt() if hasattr(self, '_com_settle') else None,
            plan=getattr(self, '_com_path_plan', None), diagnostic=getattr(self, '_com_reference_diagnostic', None))
        config = {k: getattr(self, k) for k in CONFIG_FIELDS if hasattr(self, k)}
        config.update(mode=self._reviewed_mode, referenceVersion=REFERENCE_VERSION,
            armMappingRanges=self._mapping.ranges)
        if self._state_only_support_selection:
            config['causalSupportSelectionPolicy'] = causal_selection_policy()
            causal['causalOverlap'] = self._causal_overlap_plan
        if self._reviewed_mode == 'continuous-com-load':
            self._prepared_soft_load_preference = prepare_preference(self._soft_load_anchor,
                phase, t, self.w.l.lh_dim(self.w.h, 6), self.w.l.lh_time(self.w.h),
                self.phase_duration('settle'))
            causal['softLoadAcquisition'] = (None if self._soft_load_anchor is None
                else asdict(self._soft_load_anchor))
            config['softLoadPreference'] = (None if self._prepared_soft_load_preference is None
                else self._prepared_soft_load_preference.receipt())
        self._prepared_reference_context = self._complete_reference_context(
            make_context(self, t, phase, draft_com, causal=causal,
                config=serial(config), seek=seek), t, phase)
        draft_token = self._prepared_inputs_token(t, phase)
        observation_key = (self.w.l.lh_dim(self.w.h, 6), self._reviewed_step_index, phase)
        selected_before = bool(self.robust_support.get('selected'))
        if self._observed_command != observation_key:
            self._forecast_records = []
            self._observe_support(t, phase)
            self._observed_command = observation_key
        if self._prepared_inputs_token(t, phase) == draft_token:
            # Frozen context is already exact. The observer may add descriptive
            # evidence, but that cannot require another identical task assembly.
            final = self._prepared_reference_context
        else:
            final_com = self.corrected_com_reference(t, phase)
            causal['robustCorrection'].update(selected=bool(self.robust_support.get('selected')),
                deltaM=getattr(self, '_robust_delta', np.zeros(3)).tolist(),
                startTransferS=getattr(self, '_robust_start', None), durationS=getattr(self, '_robust_duration', None),
                clock=serial(getattr(self, '_robust_clock', (0., 0., 0.))))
            if self._state_only_support_selection:
                causal['causalOverlap'] = self._causal_overlap_plan
                causal['continuousCom']['diagnostic'] = getattr(self, '_com_reference_diagnostic', None)
            final = self._complete_reference_context(make_context(self, t, phase, final_com,
                causal=causal, config=serial(config), seek=seek), t, phase)
        self._prepared_reference_context = final
        # A newly selected triangle changes inequalities immediately. Bind an
        # additional pure forecast to the FINAL context, separate from voting.
        if not selected_before and self.robust_support.get('selected'):
            corners, _, clearance = actual_stance_patch(self.w, self.stance_side)
            _, parity = self._forecast_unload_risk(t, corners, clearance, role='final-context-parity')
            self.robust_support['selectionCommandFinalContextForecast'] = parity
        diagnostics = json.loads(final.diagnostics_json)
        self.capture_barrier = diagnostics['captureBarrier']
        self.arm_range_barrier = diagnostics['opposedArmRangeBarrier']
        self.robust_support_barrier = diagnostics['robustSupportBarriers']

    def _forecast_unload_risk(self, t, corners, clearance, role='causal-observer', reference_context=None):
        """Pure shared-reference QP; no recast, arm update, seek or rollout."""
        w = self.w; before = native_bytes(w); counter = w.l.lh_dim(w.h, 6)
        context = prospective_unload_context(self, self._prepared_reference_context if reference_context is None else reference_context)
        u, info = Controller.solve(self, t, 'transfer', reference_context=context)
        forces = np.array(info['predictedForcesN']).reshape(2, 4, 3)[self.stance_side, :, 2]
        w.l.lh_motor_response.argtypes = [C.c_void_p, PTR, PTR]; w.l.lh_motor_response.restype = None
        response = w.read('lh_motor_response', (33,), ptr(np.ascontiguousarray(u)))
        foot = w.feet()[self.stance_side]; body = foot['body']; v = w.arr(1, (27,))
        Jbody = w.read('lh_body_jac', (6, 27), body); jet = w.read('lh_jdot', (6,), body)
        omega = Jbody[3:]@v; origin = w.arr(3, (17, 3))[body]
        rates, accelerations = [], []
        for point in corners:
            J = w.read('lh_point_jac', (3, 27), body, ptr(np.ascontiguousarray(point)))
            r = point-origin
            bias = jet[:3]+np.cross(jet[3:], r)+np.cross(omega, np.cross(omega, r))
            rates.append(float(J[2]@v)); accelerations.append(float(J[2]@response[:27]+bias[2]))
        horizon = .1; forecast = clearance+horizon*np.array(rates)+.5*horizon**2*np.array(accelerations)
        pressure_risk, clearance_risk = int(np.argmin(forces)), int(np.argmax(forecast))
        if native_bytes(w) != before or w.l.lh_dim(w.h, 6) != counter:
            raise ValueError('shared-reference forecast mutated native state')
        native_digest = hashlib.sha256(before).hexdigest()
        if not hasattr(self, '_robust_payloads'):
            self._robust_payloads = deque(maxlen=22)
        if role == 'causal-observer':
            self._robust_payloads.append(dict(sourceStep=counter, sourceTimeS=w.l.lh_time(w.h),
                payloadSHA256=native_digest, payloadBytes=len(before), compressed=zlib.compress(before, 1)))
        fields = {name: getattr(self, name) for name in CONFIG_FIELDS if hasattr(self, name)}
        fields.update(arm_posture=self.arm_posture, arm_velocity=self.arm_velocity,
            arm_acceleration=self.arm_acceleration, unload_admitted=True)
        telemetry = dict(schema=2, role=role, virtualUnloadNormalForcesN=forces.tolist(),
            nativeUnloadEndpointVerticalAccelerationMps2=accelerations, nativeUnloadEndpointVerticalVelocityMps=rates,
            localClearanceForecastM=forecast.tolist(), predictionHorizonS=horizon,
            pressureRiskCorner=pressure_risk, clearanceRiskCorner=clearance_risk,
            proposedMotorControl=u.tolist(), proposedMotorControlSHA256=control_sha(u), nativeProposedResponse=response.tolist(),
            probeControllerInput=dict(fields=serial(fields), armMappingRanges=self._mapping.ranges.tolist(),
                elapsedTransferS=t, phase='transfer', referenceContext=context.receipt(),
                coreEntryPoint='Controller.solve with immutable reviewed reference context; no preparation'),
            qpAssemblySHA256=info['qpAssemblySHA256'], nativePayloadSHA256=native_digest,
            nativePayloadSourceStep=counter, currentMatchedSupportCoordinatesSource=corners.tolist(),
            currentEndpointClearanceM=clearance.tolist(), proposedMotorMaximumFraction=float(max(abs(u))),
            proposedControlApplied=False, physicsRolloutIntegrations=0, nativeBytesAndCounterUnchanged=True)
        self._forecast_records.append(telemetry)
        return pressure_risk if pressure_risk == clearance_risk else -1, telemetry

    def solve(self, t, phase='transfer'):
        key = self._command_identity(t, phase)
        if self._command_cache is not None and self._command_cache[0] == key:
            return self._command_cache[1].copy(), copy.deepcopy(self._command_cache[2])
        u, info = super().solve(t, phase)
        info['reviewedWalk'] = dict(schema=2, mode=self._reviewed_mode,
            referenceVersion=REFERENCE_VERSION, immutablePreparedContext=True,
            forecasts=copy.deepcopy(self._forecast_records))
        info['actualMotorControl'] = u.tolist()
        info['actualMotorControlSHA256'] = control_sha(u)
        if (self._reviewed_mode == 'continuous-com-load' and phase == 'settle'
                and hasattr(self, '_soft_load_evidence_folder')):
            for threshold in (.1, .5):
                if t < threshold or threshold in self._soft_load_saved_targets:
                    continue
                payload = native_bytes(self.w)
                folder = self._soft_load_evidence_folder
                folder.mkdir(exist_ok=True)
                name = f'after-{threshold:.1f}s-source-{self.w.l.lh_dim(self.w.h, 6):06d}.native-payload.zlib'
                path = folder/name; path.write_bytes(zlib.compress(payload, 1))
                self._soft_load_witnesses.append(dict(thresholdS=threshold,
                    sourceStep=self.w.l.lh_dim(self.w.h, 6), sourceTimeS=self.w.l.lh_time(self.w.h),
                    elapsedS=t, phase=phase, payloadSHA256=hashlib.sha256(payload).hexdigest(),
                    payloadBytes=len(payload), filename=name, compressedSHA256=sha(path),
                    contextSHA256=info['referenceContext']['contextSHA256'],
                    actualMotorControlSHA256=info['actualMotorControlSHA256'],
                    qpAssemblySHA256=info['qpAssemblySHA256']))
                manifest = dict(schema=1, profile='retained-acquired-load-hinge-v1',
                    modelSHA256=sha(self.w.xml), helperSHA256=sha(self.w.helper),
                    bridgeSHA256=sha(self.w.base/'source_bridge_v2.so'),
                    engineSHA256=sha(self.w.base/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3'),
                    stepIndex=self._reviewed_step_index, samples=self._soft_load_witnesses,
                    physicsIntegrations=0, actualCommandNotYetApplied=True,
                    payloadEncoding='zlib-compressed exact lh_checkpoint_export payload')
                (folder/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
                self._soft_load_saved_targets.add(threshold)
        # Preparation may select support, but its planning config is unchanged.
        self._command_cache = (self._command_identity(t, phase), u.copy(), copy.deepcopy(info))
        return u, info


class ReviewedSteps(CadencedSteps):
    def __init__(self, base, helper, output, count=1, mode='baseline-context'):
        if count not in (1, 2, 7, 10) or mode not in MODES:
            raise ValueError('unsupported reviewed pilot/diagnostic/final request')
        self.contact_impratio = 8.
        self.reviewed_mode = mode
        self._wrench_last_source_step = 0
        self._wrench_summary = dict(schema=1, nativeSamples=0, physicsStepS=.00125, perStepPhase={},
            footSideOrder=['left', 'right'], jointOrder='source qpos21; actuator-order permutation removed with B',
            thresholds=dict(nearMotorFraction=.925, nearJointMarginRad=.02, nearNetShearUtilizationLowerBound=.9),
            individualContactNonsaturationEstablished=False,
            forceStage='actual cached mj_step2 solve before integration; current geometry and material-point slip remain separately identified')
        super().__init__(base, helper, output, count, .18, 'loaded-settle', False)
        self.code_at_start.update({name: sha(Path(__file__).with_name(name)) for name in
            ('source_reviewed_walk.py', 'source_reference_context.py', 'source_robust_support_steps.py',
             'source_contact_impedance_probe.py', 'source_continuous_com.py',
             'source_solved_wrench_diagnostics.py', 'source_soft_load_preference.py',
             'source_causal_com_overlap.py')})

    def begin_step(self):
        previous = getattr(self, 'controller', None) if self.step_index > 0 else None
        super().begin_step()
        controller_class = getattr(self, 'reviewed_controller_class', ReviewedController)
        if not isinstance(controller_class, type) or not issubclass(controller_class, ReviewedController):
            raise ValueError('reviewed controller factory must preserve the shared reference owner')
        self.controller.__class__ = controller_class
        self.controller.install_reviewed(self.step_index+1, self.reviewed_mode)
        self.controller._robust_evidence_folder = self.folder/f'robust-risk-step-{self.step_index+1:02d}'
        self.controller._robust_step_index = self.step_index+1
        if self.reviewed_mode == 'continuous-com-load':
            self.controller._soft_load_evidence_folder = self.folder/f'soft-load-step-{self.step_index+1:02d}'
        # Inherited setup solves are descriptive. Their history is discarded;
        # only actual commands contribute causal derivative/support observations.
        if previous is None:
            self.controller.previous_time = None
            self.controller.previous_leg_rate = None
        else:
            self.controller.previous_time = previous.previous_time
            self.controller.previous_leg_rate = previous.previous_leg_rate.copy()
        record = {k: v for k, v in vars(self.controller).items() if k != 'w' and not k.startswith('_')}
        (self.folder/f'controller-step-{self.step_index+1:02d}.json').write_text(json.dumps(serial(record))+'\n')

    def decorate(self, row):
        row = super().decorate(row)
        diagnostic = read_solved_wrenches(self.world, row)
        row['solvedWrenchDiagnostics'] = diagnostic
        if row['sourceStep'] == 0:
            return row
        if row['sourceStep'] != self._wrench_last_source_step+1:
            raise ValueError('wrench journal must cover every native sample exactly once')
        self._wrench_last_source_step = row['sourceStep']
        self._wrench_summary['nativeSamples'] += 1
        key = str(row.get('stepIndex', 1))+':'+row['phase']
        stats = self._wrench_summary['perStepPhase'].setdefault(key, dict(nativeSamples=0,
            loadedSlipM=[0., 0.], netShearImpulseNs=[0., 0.], normalImpulseNs=[0., 0.],
            maximumNetShearUtilizationLowerBound=[None, None], nearNetShearLowerBoundS=[0., 0.],
            nearMotorFractionS=[0.]*21, nearJointLimitS=[0.]*21,
            maximumActualMotorFractionByJoint=[0.]*21, minimumJointMarginRad=[1e300]*21))
        stats['nativeSamples'] += 1
        for side, foot in enumerate(diagnostic['feet']):
            stats['loadedSlipM'][side] += foot['transportedLoadedSlipIncrementM']
            stats['netShearImpulseNs'][side] += foot['netShearForceN']*.00125
            stats['normalImpulseNs'][side] += foot['normalForceN']*.00125
            ratio = foot['netShearUtilizationLowerBound']
            if ratio is not None:
                old = stats['maximumNetShearUtilizationLowerBound'][side]
                stats['maximumNetShearUtilizationLowerBound'][side] = ratio if old is None else max(old, ratio)
                if ratio >= .9: stats['nearNetShearLowerBoundS'][side] += .00125
        gear = np.max(np.abs(self.controller.B[6:]), axis=1)
        fractions = np.abs(np.array(row['actualMotorTorquesNm']))/gear
        margins = np.array(row['jointLimitMarginsRad'])
        for joint in range(21):
            stats['maximumActualMotorFractionByJoint'][joint] = max(stats['maximumActualMotorFractionByJoint'][joint], float(fractions[joint]))
            stats['minimumJointMarginRad'][joint] = min(stats['minimumJointMarginRad'][joint], float(margins[joint]))
            if fractions[joint] >= .925: stats['nearMotorFractionS'][joint] += .00125
            if margins[joint] <= .02: stats['nearJointLimitS'][joint] += .00125
        return row

    def save(self, reason):
        super().save(reason)
        if not hasattr(self, 'folder') or not hasattr(self, 'step_records'):
            return
        hashes = getattr(self, 'code_at_start', {})
        receipt = dict(schema=2, profile='five-reviewed-walk', mode=self.reviewed_mode,
            referenceVersion=REFERENCE_VERSION, requestedSteps=len(self.distances), stopped=reason,
            finished=self.finished, actualIntegrations=self.steps, finalQuietS=self.quiet_run*.00125,
            physicsStepS=.00125, torqueUpdateS=.005, codeSHA256AtStart=hashes,
            codeSHA256AtEnd={name: sha(Path(__file__).with_name(name)) for name in hashes},
            codeUnchangedDuringRun=all(sha(Path(__file__).with_name(name)) == value for name, value in hashes.items()),
            contactImpratio=8., originalModelQualificationClaim=False, selectedPresetChanged=False,
            rootAssistance=False, physicalStateInstallationsAfterInitialization=0,
            loadedSlipResetBetweenSteps=False, implicitArms=False, wrenchDiagnosticsEnabled=True)
        if self.reviewed_mode == 'continuous-com-load':
            receipt['softLoadPreferenceEnabled'] = True
        if self.reviewed_mode == 'continuous-com-causal':
            receipt['causalSupportSelectionEnabled'] = True
        (self.folder/'reviewed-walk-run.json').write_text(json.dumps(receipt, indent=2)+'\n')
        reviewed_config = dict(schema=2,
            profile='five-reviewed-walk', mode=self.reviewed_mode, requestedSteps=len(self.distances),
            referenceVersion=REFERENCE_VERSION, loadedSettleProtocolUnchanged=True,
            originalModelQualificationClaim=False, finalPlanQualificationClaim=False,
            originalPhysicalGatesUnchanged=True, wrenchDiagnosticsEnabled=True)
        if self.reviewed_mode == 'continuous-com-load':
            reviewed_config['softLoadPreference'] = dict(enabled=True,
                policy='retained-acquired-load-hinge-v1', weightPerNewton=.0005,
                maximumAcquiredWeightFraction=.1, activationS=.1,
                fadeDurationS=self.controller.phase_duration('settle'),
                acquiredFrom='actual qualified land-to-settle native sample',
                noHardForceFloor=True, originalCornerNormalAndShearCostsUnchanged=True)
        if self.reviewed_mode == 'continuous-com-causal':
            reviewed_config['causalSupportSelectionPolicy'] = causal_selection_policy()
        (self.folder/'reviewed-walk-config.json').write_text(json.dumps(reviewed_config, indent=2)+'\n')
        self._wrench_summary.update(actualIntegrations=self.steps, sourceStepLast=self._wrench_last_source_step,
            allNativeSamplesCovered=self._wrench_summary['nativeSamples'] == self.steps,
            stopped=reason, globalLoadedSlipM=self.slip.tolist())
        (self.folder/'reviewed-wrench-summary.json').write_text(json.dumps(self._wrench_summary, indent=2)+'\n')
        path = self.folder/'cadenced-step-config.json'
        config = json.loads(path.read_text())
        config['contactImpedanceExperiment'] = dict(impratio=8., baselineImpratio=1., muChanged=False,
            originalModelQualificationClaim=False, separatelyModelBoundColdCache=True)
        config['robustSupportAdmission'] = dict(enabled=True, causalWeakCornerHistoryS=.1,
            minimumFrequency=.8, supportPatchConstruction='three-unique-nearest-actual-support-xy',
            riskSignal='planned-unload-QP-minimum-normal-pressure-and-native-100ms-clearance-agreement',
            forecastHorizonS=.1, forecastControlApplied=False, forecastRolloutIntegrations=0,
            actualSurvivorsRequired=True, maximumComCorrectionM=.012, measuredAdmissionReserveM=.003,
            allFrameOriginalCaptureGuardUnchanged=True, arbitraryCornerLossRobustnessClaim=False)
        config['reviewedReferenceProfile'] = dict(schema=2, version=REFERENCE_VERSION, mode=self.reviewed_mode)
        path.write_text(json.dumps(config, indent=2)+'\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--inspector', type=Path, required=True)
    p.add_argument('--count', type=int, choices=(1, 2, 7, 10), default=1)
    p.add_argument('--mode', choices=MODES, default='baseline-context')
    a = p.parse_args()
    session, helper, preparation = prepare_impedance(a.output, a.inspector, 8.)
    retained_inspector = a.output/'contact-inspector.so'
    shutil.copyfile(a.inspector, retained_inspector)
    if sha(retained_inspector) != preparation['inspectorSHA256']:
        raise ValueError('retained read-only inspector differs from preparation')
    live = ReviewedSteps(session, helper, a.output/'run', a.count, a.mode)
    live.paused = False; started = time.monotonic()
    try:
        while not live.finished and not live.failure:
            if time.monotonic()-started >= 1200.:
                raise RuntimeError('1200s process cap')
            live.advance(4)
            if live.steps % 800 == 0:
                print('progress', live.steps*.00125, 'step', live.step_index+1, live.phase,
                    'slip', live.slip, 'quiet', live.quiet_run*.00125, flush=True)
        print(live.failure or 'reviewed-walk-physical-sequence-plus5squiet', flush=True)
    except Exception as error:
        live.failure = repr(error); raise
    finally:
        live.close()
    raise SystemExit(0 if live.finished else 1)


if __name__ == '__main__':
    main()
