#!/usr/bin/env python3
"""Compact saved-journal gait quality measurements. Never integrates physics.

Quality measurements are not a full physical qualification. Landing means the
first measured land row with 80 consecutive loaded samples, requested own-foot
advance, <=3 mm clearance and the unchanged nominal phase clock completed.
Completion times (which may include settle/quiet) are never used as landings.
"""
import argparse
from collections import defaultdict, deque
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull, QhullError

DT = .00125
BODIES = ('pelvis', 'torso', 'torsoRelativePelvis')
SLIP_LIMIT_M = .020


def location(row):
    return {key: row[key] for key in ('sourceStep', 'sourceTimeS', 'elapsedS', 'stepIndex', 'phase')}


def stats(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return dict(samples=0, rms=None, p95=None, maximum=None)
    return dict(samples=len(values), rms=float(np.sqrt(np.mean(values**2))),
                p95=float(np.percentile(values, 95)), maximum=float(np.max(values)))


class Margin:
    def __init__(self):
        self.count = self.undefined = self.negative = self.below3mm = 0
        self.minimum = self.at = self.first_undefined = None

    def add(self, value, row, context=None):
        self.count += 1
        if value is None or not math.isfinite(value):
            self.undefined += 1
            if self.first_undefined is None:
                self.first_undefined = location(row)
            return
        self.negative += value < 0.
        self.below3mm += value < .003
        if self.minimum is None or value < self.minimum:
            self.minimum = float(value)
            self.at = dict(location(row), supportPointsXY=row.get('supportPointsXY'),
                           com=row.get('com'), comVelocity=row.get('comVelocity'))
            if context:
                self.at.update(context)

    def report(self):
        return dict(samples=self.count, undefinedSamples=self.undefined,
                    firstUndefinedAt=self.first_undefined, negativeSamples=self.negative,
                    below3mmSamples=self.below3mm, minimumM=self.minimum, minimumAt=self.at,
                    allFrameDefinedNonnegativePass=bool(self.count and not self.undefined and not self.negative),
                    allFrame3mmQualityPass=bool(self.count and not self.undefined and not self.below3mm))


def separate_support_margins(row):
    """Signed COM and LIP-capture distances to current recorded support hull."""
    try:
        com, velocity = np.asarray(row['com']), np.asarray(row['comVelocity'])
        points = np.unique(np.asarray(row['supportPointsXY']).reshape(-1, 2), axis=0)
        if not (np.isfinite(com).all() and np.isfinite(velocity).all()
                and np.isfinite(points).all()) or com[2] < 0:
            return None, None
        equations = ConvexHull(points).equations
        cp = com[:2]+velocity[:2]*math.sqrt(com[2]/9.81)
        distances = [float(np.min(-(equations[:, :2]@p+equations[:, 2])))
                     for p in (com[:2], cp)]
        return tuple(distances)
    except (KeyError, QhullError, ValueError, TypeError, IndexError):
        return None, None


def archived_reference(summary, source):
    """Keep archived measurements explicit; a missing raw journal is not rebuilt."""
    old = summary.get('stopOnly', {})
    completed = old.get('completedTimesS', [])
    # Only explicit archived landing times can supply comparable intervals.
    landing = old.get('loadedLandingTimesS', old.get('landingTimesS'))
    return dict(source=str(source), rawJournalAvailable=False,
                evidenceLimit='Archived compact measurements only; lost journal metrics cannot be recomputed.',
                loadedLandingTimesS=landing,
                loadedLandingIntervalsS=np.diff(landing).tolist() if landing else None,
                loadedLandingIntervalsAvailable=bool(landing),
                archivedCompletionTimesS=completed,
                archivedCompletionIntervalsS=np.diff(completed).tolist(),
                completionIntervalsDefinition='Completion may include settle and final quiet; not loaded-landing cadence.',
                archivedCOMHorizontalTravelM=old.get('comHorizontalTravelM'),
                archivedOwnFootAdvancesM=old.get('achievedFootAdvancesM'),
                archivedMinimumCombinedCaptureMarginM=old.get('minimumCaptureMarginM'))


def analyze_rows(rows, entry, references, records=(), expected_steps=10, walking_start_s=0.):
    heading = np.asarray(references[1]['heading'], dtype=float).copy()
    heading[2] = 0.
    heading /= np.linalg.norm(heading)
    left = np.cross([0., 0., 1.], heading)
    initial_com = np.asarray(entry['com'])
    previous_com = initial_com.copy()
    tilt = defaultdict(lambda: {body: [] for body in BODIES})
    margins = {key: Margin() for key in ('recordedCombined', 'reconstructedCOM', 'reconstructedCapture')}
    phase_margins = defaultdict(lambda: {key: Margin() for key in margins})
    quiet = deque(maxlen=4000)
    runs, landings, per_step = [], {}, {}
    landed_runs = defaultdict(int)
    walking_count = near_zero = count = mismatch = 0
    horizontal_path = 0.
    lateral = []
    maximum_slip = np.zeros(2)
    first_slip_exceedance = None
    first, last = None, None
    for row in rows:
        count += 1
        if row['sourceStep'] != entry['sourceStep']+count:
            raise ValueError('noncontiguous sourceStep')
        if abs(row['elapsedS']-count*DT) > 1e-10 or abs(row['sourceTimeS']-entry['sourceTimeS']-count*DT) > 1e-8:
            raise ValueError('native time does not match contiguous 800 Hz rows')
        if first is None:
            first = row
        last = row
        phase, index, side = row['phase'], row['stepIndex'], row['swingSide']
        reference = references[index]
        if not runs or (runs[-1]['stepIndex'], runs[-1]['phase']) != (index, phase):
            runs.append(dict(stepIndex=index, phase=phase, firstNativeStep=row['sourceStep'],
                             firstTimeS=row['elapsedS'], samples=0))
        run = runs[-1]
        run.update(lastNativeStep=row['sourceStep'], lastTimeS=row['elapsedS'], samples=run['samples']+1)
        if phase in ('transfer', 'lift', 'translate', 'land', 'settle') and row['elapsedS'] > walking_start_s:
            # Both transfer and loaded-settle waiting belong to the gait.
            # Exclude only initial arm placement and the terminal stand.
            walking_count += 1
            near_zero += abs(float(np.asarray(row['comVelocity'])@heading)) < .005
        com = np.asarray(row['com'])
        horizontal_path += float(np.linalg.norm((com-previous_com)[:2]))
        previous_com = com
        lateral.append(float((com-initial_com)@left))
        for body in BODIES:
            tilt[phase][body].append(row['uprightMeasured'][body]['tiltDeg'])
        com_margin, cp_margin = separate_support_margins(row)
        values = dict(recordedCombined=row.get('minimumComCaptureMarginM'),
                      reconstructedCOM=com_margin, reconstructedCapture=cp_margin)
        margin_context = dict(reconstructedCOMMarginM=com_margin, reconstructedCaptureMarginM=cp_margin)
        for key, value in values.items():
            margins[key].add(value, row, margin_context)
            phase_margins[phase][key].add(value, row, margin_context)
        reconstructed = min(com_margin, cp_margin) if com_margin is not None and cp_margin is not None else None
        recorded = values['recordedCombined']
        mismatch += ((reconstructed is None) != (recorded is None)
                     or (reconstructed is not None and recorded is not None
                         and (not math.isfinite(recorded) or abs(reconstructed-recorded) > 1e-10)))
        slip = np.asarray(row['loadedSlipDistanceM'])
        maximum_slip = np.maximum(maximum_slip, slip)
        if np.any(slip > SLIP_LIMIT_M) and first_slip_exceedance is None:
            first_slip_exceedance = dict(location(row), perFootM=slip.tolist())
        forward = float((np.asarray(row['feet'][side]['position'])-np.asarray(reference['targets'][side][0]))@np.asarray(reference['heading']))
        if abs(forward-row['swingForwardDisplacementM']) > 1e-10:
            raise ValueError('own-foot forward telemetry disagrees with measured position')
        per_step[index] = dict(stepIndex=index, swingSide='left' if side == 0 else 'right',
                               finalOwnFootAdvanceM=forward, requestedM=row['requestedForwardM'])
        landed = (phase == 'land' and all(f['loadN'] >= 3 for f in row['feet'])
                  and row['feet'][side]['clearance'] <= .003 and forward >= row['requestedForwardM'])
        landed_runs[index] = landed_runs[index]+1 if landed else 0
        nominal = reference.get('phase_durations', {}).get('land', reference.get('trajectory_time_scale', 1.))
        if index not in landings and landed_runs[index] >= 80 and row['phaseElapsedS']-DT >= nominal-1e-10:
            landings[index] = dict(location(row), ownFootAdvanceM=forward,
                                  consecutiveLoadedSamples=landed_runs[index])
        quiet.append(row)
    if last is None:
        raise ValueError('empty source journal')
    for run in runs:
        run['durationS'] = run['samples']*DT
        nominal = references[run['stepIndex']].get('phase_durations', {}).get(run['phase'])
        run['nominalDurationS'] = nominal
        run['beyondNominalS'] = max(0., run['durationS']-nominal) if nominal is not None else None
    landing_rows = [landings[i] for i in sorted(landings)]
    intervals = np.diff([r['elapsedS'] for r in landing_rows]).tolist()
    record_matches = len(records) == len(landings) and all(index in landings and record.get('landingNativeStep') == landings[index]['sourceStep']
                         and abs(record.get('landingTimeS', -1)-landings[index]['elapsedS']) < 1e-10
                         for record in records for index in [record['index']])
    tail = list(quiet)
    final_delta = np.asarray(last['com'])-initial_com
    advances = [per_step[i]['finalOwnFootAdvanceM'] for i in sorted(per_step)]
    for index, measurement in per_step.items():
        measurement['loadedLandingOwnFootAdvanceM'] = landings.get(index, {}).get('ownFootAdvanceM')
    transfers = [run for run in runs if run['phase'] == 'transfer']
    measured_margins = {key: value.report() for key, value in margins.items()}
    safety_pass = all(value['allFrameDefinedNonnegativePass'] for value in measured_margins.values()) and mismatch == 0
    quality_pass = all(value['allFrame3mmQualityPass'] for value in measured_margins.values()) and mismatch == 0
    return dict(schema=1, savedDataOnly=True, physicsIntegrations=0,
                evidenceScope='Gait quality measurements only; no full physical qualification implied.',
                actualRecordedIntegrations=count, physicsSimSeconds=count*DT,
                nativeRowRange=[first['sourceStep'], last['sourceStep']],
                loadedLandings=landing_rows, loadedLandingIntervalsS=intervals,
                medianLoadedLandingIntervalS=float(np.median(intervals)) if intervals else None,
                allNineLoadedLandingIntervalsAvailable=sorted(landings) == list(range(1,11)) and expected_steps == 10,
                landingSummaryMatchesMeasuredRows=bool(records) and record_matches,
                cadenceMedianBelow7_63S=bool(len(landing_rows) == expected_steps and intervals and np.median(intervals) < 7.63),
                phaseRuns=runs,
                phaseTotalDurationS={phase: len(values[BODIES[0]])*DT for phase, values in tilt.items()},
                maximumTransfer=max(transfers, key=lambda run: run['durationS']) if transfers else None,
                walkingForwardCOMSpeed=dict(definition='abs(measured COM velocity dot fixed initial heading) < 0.005 m/s; transfer/lift/translate/land/settle rows after arm placement, before terminal stand',
                    initialArmPlacementExcludedUntilS=walking_start_s,
                    thresholdMps=.005, walkingSamples=walking_count, nearZeroSamples=int(near_zero),
                    walkingDurationS=walking_count*DT, nearZeroDurationS=near_zero*DT,
                    nearZeroFraction=float(near_zero/walking_count) if walking_count else None),
                travel=dict(netCOMForwardM=float(final_delta@heading), netCOMHorizontalM=float(np.linalg.norm(final_delta[:2])),
                    COMHorizontalPathLengthM=horizontal_path, initialHeading=heading.tolist(),
                    ownFootAdvanceDefinition='Each swing foot from its own measured step-entry origin, not separation beyond the stance foot.',
                    perStep=list(per_step.values()), sumOwnFootAdvancesM=sum(advances),
                    sumLoadedLandingOwnFootAdvancesM=sum(item['ownFootAdvanceM'] for item in landing_rows),
                    netCOMForwardPerSumOwnFootAdvances=float(final_delta@heading)/sum(advances) if sum(advances) else None,
                    maximumAbsLateralCOMDisplacementM=max(abs(x) for x in lateral), lateralCOMPeakToPeakM=max(lateral)-min(lateral)),
                tiltDegByPhase={phase: {body: stats(values[body]) for body in BODIES} for phase, values in tilt.items()},
                last4000=dict(samples=len(tail), full4000RowsPresent=len(tail) == 4000,
                    nativeRowRange=[tail[0]['sourceStep'], tail[-1]['sourceStep']],
                    intervalStartS=tail[0]['elapsedS']-DT, intervalEndS=tail[-1]['elapsedS'], durationS=len(tail)*DT,
                    allStand=all(r['phase'] == 'stand' for r in tail),
                    tiltDeg={body: stats([r['uprightMeasured'][body]['tiltDeg'] for r in tail]) for body in BODIES},
                    firstTiltDeg={body: tail[0]['uprightMeasured'][body]['tiltDeg'] for body in BODIES},
                    finalTiltDeg={body: tail[-1]['uprightMeasured'][body]['tiltDeg'] for body in BODIES}),
                capture=dict(margins=measured_margins, telemetryMismatchSamples=int(mismatch),
                    byPhase={phase: {key: value.report() for key, value in values.items()} for phase, values in phase_margins.items()},
                    allFrameDefinedNonnegativePass=safety_pass, allFrame3mmQualityPass=quality_pass,
                    definition='Signed distances to recorded current-support convex hull; capture = COMxy + COMvelocityxy*sqrt(COMz/9.81). Undefined is failure, never excluded from pass.'),
                slip=dict(cumulativePerFootLimitM=SLIP_LIMIT_M, strictExceedanceRequired=True, equalityAllowed=True,
                    maximumPerFootM=maximum_slip.tolist(), finalPerFootM=last['loadedSlipDistanceM'],
                    firstStrictExceedance=first_slip_exceedance, within20mmPass=first_slip_exceedance is None))


def analyze(folder, reference_summary=None):
    folder = Path(folder)
    summary = json.loads((folder/'summary.json').read_text())
    multi = json.loads((folder/'multistep-summary.json').read_text())
    config_path = folder/'cadenced-step-config.json'
    config = json.loads(config_path.read_text()) if config_path.exists() else {}
    references = {int(p.stem.rsplit('-', 1)[1]): json.loads(p.read_text())
                  for p in folder.glob('controller-step-*.json')}
    journal_hash = hashlib.sha256()
    def rows():
        with (folder/'frames.ndjson').open('rb') as source:
            for line in source:
                journal_hash.update(line)
                yield json.loads(line)
    mapping_path = folder/'arm-mapping.json'
    walking_start = json.loads(mapping_path.read_text()).get('placementDurationS', 0.) if mapping_path.exists() else 0.
    result = analyze_rows(rows(), summary['entry'], references, multi.get('steps', []), config.get('requestedSteps', 10), walking_start)
    result['sourceJournalSHA256'] = journal_hash.hexdigest()
    result['finalSummaryAccountingMatches'] = result['actualRecordedIntegrations'] == summary['actualIntegrations'] == multi['actualIntegrations']
    if reference_summary:
        result['archivedReference'] = archived_reference(json.loads(Path(reference_summary).read_text()), reference_summary)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference-summary', type=Path)
    args = parser.parse_args()
    result = analyze(args.folder, args.reference_summary)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: result[key] for key in ('actualRecordedIntegrations', 'loadedLandingIntervalsS',
                     'medianLoadedLandingIntervalS', 'walkingForwardCOMSpeed', 'finalSummaryAccountingMatches')}, indent=2))
