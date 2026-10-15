#!/usr/bin/env python3
"""Run an explicit whitelist of arithmetic/archive unit tests, never a worker."""
import argparse
import hashlib
import json
import re
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
PREFIXES = [
    "recovery::planner::support_state::material_continuity::tests::",
    "recovery::planner::support_state::tests::",
    "recovery::planner::support_schedule::tests::",
    "recovery::planner::load_transfer::tests::",
    "recovery::planner::transfer_adapter::tests::",
    "recovery::planner::command_proposal::tests::",
    "recovery::planner::native_contact_diagnostics::tests::",
    "recovery::planner::native_observation::tests::",
]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--phase", required=True)
    parser.add_argument("--filter", action="append", default=[])
    args = parser.parse_args()
    assert re.fullmatch(r"[a-z0-9-]+", args.phase)
    binary = args.binary.resolve()
    assert binary.parent == ROOT / "rust/target/debug/deps"
    assert binary.name.startswith("lh_sim-") and binary.is_file()
    output = HERE / (args.phase + "-pure-tests")
    output.mkdir(exist_ok=False)
    physical = ROOT / "rust/target/release/deps/coordination_transfer_worker-8daed3b7e6b04b70"
    physical_before = sha(physical)
    assert physical_before == "63e0e035d38d02f762c69b9dacb65d61e179c72ced50f5facd0bc29c64b9e8b5"
    listed = subprocess.run([str(binary), "--list", "--format", "terse"],
                            cwd=ROOT, check=True, capture_output=True, text=True, timeout=60)
    all_names = [line.removesuffix(": test") for line in listed.stdout.splitlines()
                 if line.endswith(": test")]
    filters = args.filter or PREFIXES
    assert all(any(prefix.startswith(allowed) for allowed in PREFIXES) for prefix in filters)
    selected = [name for name in all_names if any(name.startswith(p) for p in filters)]
    assert selected
    receipt = {
        "schema": 1,
        "startedUtc": datetime.now(timezone.utc).isoformat(),
        "scope": "Whitelisted pure unit tests only; no scene, world, FK or native integration",
        "testBinary": str(binary), "testBinarySha256": sha(binary),
        "physicalBinarySha256Before": physical_before,
        "selectedTestNames": selected,
        "runs": [], "newNativeSteps": 0, "newFkOrWorldInspections": 0,
    }
    for prefix in filters:
        names = [name for name in selected if name.startswith(prefix)]
        assert names, prefix
        label = prefix.replace("recovery::planner::", "").strip(":").replace("::", "-")
        command = [str(binary), prefix, "--test-threads=1", "--nocapture"]
        start = time.monotonic()
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=60)
        log = output / (label + ".log")
        log.write_text(result.stdout + result.stderr)
        receipt["runs"].append({"filter": prefix, "command": command,
            "expectedTests": len(names), "exitCode": result.returncode,
            "wallSeconds": time.monotonic() - start, "log": str(log.relative_to(ROOT)),
            "logSha256": sha(log)})
        print(f"{label}: {len(names)} tests, exit {result.returncode}", flush=True)
    receipt["physicalBinarySha256After"] = sha(physical)
    assert receipt["physicalBinarySha256After"] == physical_before
    receipt["allPassed"] = all(run["exitCode"] == 0 for run in receipt["runs"])
    receipt["selectedTestCount"] = len(selected)
    receipt["finishedUtc"] = datetime.now(timezone.utc).isoformat()
    (output / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return 0 if receipt["allPassed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
