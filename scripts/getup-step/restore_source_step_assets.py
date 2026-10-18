#!/usr/bin/env python3
"""Rebuild local-only pinned source-step assets after executor loss.

Downloads the four original XMLs and the official MuJoCo 2.1.3 wheel, checking
their recorded SHA256s. No upstream assets are copied into tracked source.
Restores the already recorded q/qvel into a new cold cache, with zero steps.
Run in the lightweight environment from requirements.txt plus clarabel.
"""
import hashlib
import json
import subprocess
import sys
import urllib.request
from pathlib import Path
from zipfile import ZipFile

from source_step_probe import ROOT, World, sha
from source_selected_step import load_preset, state_sha

PIN='5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e'
MODEL_HASH='69a7e87fed168e1f4ad7dee43dfdd8df6bd74b3d4af06d99c991b46b941106db'


def checked_download(url, path, expected):
    if not path.exists():
        path.parent.mkdir(parents=True,exist_ok=True)
        with urllib.request.urlopen(url, timeout=60) as response:
            data=response.read()
        assert hashlib.sha256(data).hexdigest()==expected, 'download hash mismatch'
        path.write_bytes(data)
    assert sha(path)==expected, f'local hash mismatch: {path}'


def restore():
    preset=load_preset(); base=ROOT/preset['sourceSession']; helper=ROOT/preset['helper']
    downloads=json.loads((ROOT/'docs/evidence/getup-source-engine-2026-10-17/source-download-receipt.json').read_text())
    dependencies=json.loads((ROOT/'docs/evidence/getup-source-engine-2026-10-17/official-dependency-receipt.json').read_text())
    for name in ('skybox.xml','visual.xml','materials.xml','humanoid_high_freq.xml'):
        expected=MODEL_HASH if name=='humanoid_high_freq.xml' else downloads[name]['sha256']
        checked_download(f'https://raw.githubusercontent.com/tianxintao/get_up_control/{PIN}/data/{name}',base/'source-model'/name,expected)
    wheel=base/'dependency'/dependencies['wheelFilename']
    if not wheel.exists():
        subprocess.run([sys.executable,'-m','pip','download','--no-deps','--only-binary=:all:',
            '--python-version=310','--platform=manylinux2014_x86_64','--implementation=cp','--abi=cp310',
            'mujoco==2.1.3','-d',str(wheel.parent)],check=True)
    assert sha(wheel)==dependencies['wheelSHA256']
    target=base/'dependency/official-wheel'
    library=target/'mujoco/libmujoco_nogl.so.2.1.3'
    if not library.exists():
        with ZipFile(wheel) as archive:
            # Never overwrite a mapped engine library. The pinned official
            # wheel's incompatible CPython extension is never imported.
            archive.extractall(target)
    assert sha(library)==dependencies['loadedLibrarySHA256']
    if not helper.exists():
        subprocess.run(['bash',str(ROOT/'scripts/getup-step/build_source_step_probe.sh'),str(base),str(helper)],check=True)
    assert sha(helper)==preset['helperSHA256'], 'compiler output differs from pinned helper; do not reuse a cache'
    bridge=base/'source_bridge_v2.so'
    if not bridge.exists():
        subprocess.run(['cc','-shared','-fPIC','-O2','-Wall','-Wextra','-Werror',
            '-I'+str(library.parent/'include'),str(ROOT/'scripts/getup-step/source_bridge.c'),
            str(library),'-Wl,-rpath,'+str(library.parent),'-o',str(bridge)],check=True)
    entry=json.loads((ROOT/'docs/fixtures/getup-step/source-SLOW-final-frame.json').read_text())['frame']
    assert state_sha(entry)==preset['initialQposQvelSHA256']
    cache=base/'slow-continuation/final.bin';cache.parent.mkdir(parents=True,exist_ok=True)
    w=World(base,helper)
    try:
        if cache.exists():
            w.restore(cache)
        else:
            w.install_cold(entry);w.checkpoint(cache)
        assert state_sha(w.packet())==preset['initialQposQvelSHA256']
        assert w.l.lh_dim(w.h,6)==0
    finally:w.close()
    receipt=dict(schema=1,physicsIntegrations=0,coldCache=True,
        sourceCommit=PIN,sourceModelSHA256=sha(base/'source-model/humanoid_high_freq.xml'),
        helperSHA256=sha(helper),engineSHA256=sha(library),initialQposQvelSHA256=state_sha(entry),
        originalAssetsLocalOnly=True,pythonExtensionLoaded=False)
    (base/'restoration-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(receipt,indent=2))


if __name__=='__main__':restore()
