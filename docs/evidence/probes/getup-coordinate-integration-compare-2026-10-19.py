#!/usr/bin/env python3
"""Read-only exact state parity, excluding only named wall-cost diagnostics."""
import hashlib
import json
from pathlib import Path
import struct
import sys

baseline, observer, output = map(Path, sys.argv[1:])
EXCLUDED = {"controller_ms", "update_ms"}
removed = []
def normalized(value, path=""):
    if isinstance(value, dict):
        result = {}
        for key, item in value.items():
            if key in EXCLUDED:
                removed.append(path + "/" + key)
            else:
                result[key] = normalized(item, path + "/" + key)
        return result
    if isinstance(value, list):
        return [normalized(item, path + "/" + str(i)) for i, item in enumerate(value)]
    return value

def split_checkpoint(path):
    # runtime.rs getup_checkpoint uses bincode fixint (little endian), tuple:
    # (schema:u32, model:String, binary_world:Vec<u8>, auxiliary:String).
    data = path.read_bytes()
    schema = struct.unpack_from("<I", data, 0)[0]
    offset = 4
    chunks = []
    for _ in range(3):
        size = struct.unpack_from("<Q", data, offset)[0]
        offset += 8
        chunks.append(data[offset:offset+size])
        assert len(chunks[-1]) == size
        offset += size
    assert offset == len(data), "Unexpected checkpoint tail/layout"
    assert schema == 1
    return schema, chunks[0], chunks[1], json.loads(chunks[2])

checks = {}
for name in ["initial-snapshot.json", "initial-diagnostic.json", "initial-controller.json", "initial-fixture.json",
             "final-reply.json", "final-snapshot.json", "final-diagnostic.json", "final-controller.json", "final-fixture.json"]:
    left = normalized(json.loads((baseline/name).read_text()), "baseline/"+name)
    right = normalized(json.loads((observer/name).read_text()), "observer/"+name)
    checks[name] = left == right
b = split_checkpoint(baseline/"final-checkpoint.bin")
o = split_checkpoint(observer/"final-checkpoint.bin")
checks["checkpoint_schema"] = b[0] == o[0]
checks["checkpoint_canonical_model_bytes"] = b[1] == o[1]
checks["checkpoint_binary_world_bytes"] = b[2] == o[2]
checks["checkpoint_auxiliary_without_wallcost"] = normalized(b[3],"baseline/checkpoint_auxiliary") == normalized(o[3],"observer/checkpoint_auxiliary")
contexts = [json.loads((p/"run-context.json").read_text()) for p in [baseline, observer]]
checks["input_sha_equal"] = contexts[0]["inputSha256"] == contexts[1]["inputSha256"] == "fd4f0140290c0963af0da6941c181564bd6d7b95c381236d60a0b70fecf4bfd1"
checks["expected_capture_modes"] = contexts[0]["observerCaptureCompiled"] is False and contexts[1]["observerCaptureCompiled"] is True
summary = json.loads((observer/"summary.json").read_text())
checks["exact_500_internal_rows"] = summary["internalRows"] == 500
result = {"passed":all(checks.values()), "checks":checks,
          "excludedWallCostKeys":sorted(EXCLUDED), "removedPaths":sorted(set(removed)),
          "baselineWorldSha256":hashlib.sha256(b[2]).hexdigest(),
          "observerWorldSha256":hashlib.sha256(o[2]).hexdigest(),
          "attributionValidOnlyIfParityPasses":True,
          "observerSummary":summary}
output.write_text(json.dumps(result,indent=2)+"\n")
print(json.dumps({"passed":result["passed"],"checks":checks},indent=2))
raise SystemExit(0 if result["passed"] else 1)
