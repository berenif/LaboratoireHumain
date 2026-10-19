#!/usr/bin/env python3
"""Saved-data-only replay of the current cumulative walking-slip criterion."""
import argparse
import hashlib
import json
from pathlib import Path

from source_walk_limits import GLOBAL_LOADED_SLIP_LIMIT_M,loaded_slip_exceeded,validate_saved_slip_limit


def audit(folder,limit=GLOBAL_LOADED_SLIP_LIMIT_M):
    limit=validate_saved_slip_limit(limit)
    previous=[0.,0.];count=0;error=0.;first=None;legacy_first=None;minimum_capture=None;capture_at=None
    h=hashlib.sha256()
    with(Path(folder)/'frames.ndjson').open('rb')as f:
        for line in f:
            h.update(line);row=json.loads(line);count+=1
            assert row['sourceStep']==count and abs(row['elapsedS']-count/800)<1e-10
            slip=row['loadedSlipDistanceM']
            for i,foot in enumerate(row['feet']):
                delta=foot['loadedSlipSpeedMps']/800
                assert delta>=0 and slip[i]>=previous[i]
                error=max(error,abs(slip[i]-previous[i]-delta))
            if first is None and loaded_slip_exceeded(slip,limit):first=dict(timeS=row['elapsedS'],nativeStep=count,step=row['stepIndex'])
            if legacy_first is None and loaded_slip_exceeded(slip,.01):legacy_first=dict(timeS=row['elapsedS'],nativeStep=count,step=row['stepIndex'])
            margin=row['minimumComCaptureMarginM']
            if margin is not None and(minimum_capture is None or margin<minimum_capture):
                minimum_capture=margin;capture_at=dict(timeS=row['elapsedS'],nativeStep=count,step=row['stepIndex'],phase=row['phase'])
            previous=slip
    assert count and error<1e-10
    return dict(schema=1,savedDataOnly=True,physicsIntegrations=0,currentProspectiveThresholdAppliedToRecordedTrajectory=True,
        assessedCumulativePerFootLimitM=limit,boundary='allowed at equality; stop strictly above the limit',
        recordedNativeFrames=count,recordedPhysicalDurationS=count/800,finalCumulativePerFootSlipM=previous,
        maximumAccumulatorReconstructionErrorM=error,slipCriterionPassed=first is None,firstCurrentLimitExceedance=first,
        historicalTenMillimetreFirstExceedance=legacy_first,minimumCaptureMarginM=minimum_capture,captureMinimumAt=capture_at,
        captureReserveM=.003,captureReservePassed=minimum_capture is not None and minimum_capture>=.003,
        fullGaitQualificationClaim=False,sourceJournalSHA256=h.hexdigest())


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--limit-m',type=float,default=GLOBAL_LOADED_SLIP_LIMIT_M)
    a=p.parse_args();r=audit(a.folder,a.limit_m);a.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
    return 0 if r['slipCriterionPassed']else 1


if __name__=='__main__':raise SystemExit(main())
