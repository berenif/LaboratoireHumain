#!/usr/bin/env python3
"""One bounded serialization-fix replacement under the 2026-10-13 10:06Z approval.

Reuse the verified supervisor without changing its limits or the closed batch.
Default is read-only preflight. This entrypoint cannot resume or refund a run.
"""
import argparse
import importlib.util
import json
from pathlib import Path

SOURCE = Path(__file__).with_name('run-coordination-numeric-parity.py')
spec = importlib.util.spec_from_file_location('numeric_parity_supervisor', SOURCE)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
runner.AUTHORIZATION_ID = 'coordination-numeric-parity-serialization-2026-10-13T10:06Z'
BATCH_ROOT = runner.REPO / 'docs/evidence/coordination-native-serialization-2026-10-13'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    try:
        result = runner.run(args.manifest, execute=args.execute, batch_root=BATCH_ROOT)
        print(json.dumps(result, allow_nan=False))
    except (runner.Refusal, OSError, ValueError) as exc:
        parser.exit(1, f'Refused: {exc}\n')


if __name__ == '__main__':
    main()
