#!/usr/bin/env python3
"""Numeric-only audit regressions: corruption and object deserialization refusal."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np

spec=importlib.util.spec_from_file_location('audit',Path(__file__).with_name('recovery-reference-audit.py'))
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
class NumericAudit(unittest.TestCase):
 def test_object_array_is_refused_before_any_deserialization(self):
  with tempfile.TemporaryDirectory() as folder:
   file=Path(folder)/'bad.npz';np.savez(file,payload=np.array([{'executableObject':False}],dtype=object))
   metadata=Path(folder)/'bad.json';metadata.write_text(json.dumps({'numericFileSha256':hashlib.sha256(file.read_bytes()).hexdigest()}))
   with self.assertRaisesRegex(ValueError,'Object arrays cannot be loaded'):audit.audit(file,metadata)
 def test_checksum_refusal_precedes_archive_contents(self):
  with tempfile.TemporaryDirectory() as folder:
   file=Path(folder)/'bad.npz';np.savez(file,value=np.array([1.0]))
   metadata=Path(folder)/'bad.json';metadata.write_text(json.dumps({'numericFileSha256':'bad'}))
   with self.assertRaisesRegex(ValueError,'checksum'):audit.audit(file,metadata)
if __name__=='__main__':unittest.main()
