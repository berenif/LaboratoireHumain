#!/usr/bin/env python3
"""One offline shared-target worker build; no stepping, network, or trial authority."""
import argparse,json,os,shutil,subprocess
from pathlib import Path
from experiment_backup import atomic_json,sha256
from getup_worker_build import ROOT,CONTRACT,CONTRACT_PATH,rust_inputs,check_capabilities

def build(output):
    output=output.resolve()
    if not output.is_relative_to(ROOT/'outputs/cold-start') or output.exists():
        raise ValueError('New owned cold-start artifact directory required; never overwrite old evidence')
    env=dict(os.environ,CARGO_HOME=str(ROOT/'.toolchain/cargo'),RUSTUP_HOME=str(ROOT/'.toolchain/rustup'),
        CARGO_TARGET_DIR=str(ROOT/'rust/target'),CARGO_NET_OFFLINE='true',RUSTUP_TOOLCHAIN='1.89.0')
    env['PATH']=str(ROOT/'.toolchain/cargo/bin')+os.pathsep+env.get('PATH','')
    bindgen=ROOT/'.sites-runtime/rust-browser-tools/wasm-bindgen-0.2.108-x86_64-unknown-linux-musl/wasm-bindgen'
    def run(args):
        p=subprocess.run([str(x) for x in args],cwd=ROOT,env=env,text=True,capture_output=True,check=True)
        return p.stdout.strip()
    toolchain={'rustc':run(['rustc','-Vv']),'cargo':run(['cargo','-V']),
        'wasm_bindgen':run([bindgen,'--version']),'wasm_bindgen_binary_sha256':sha256(bindgen)}
    if not toolchain['rustc'].startswith('rustc 1.89.0 ') or toolchain['wasm_bindgen']!='wasm-bindgen 0.2.108':
        raise ValueError('Pinned toolchain differs')
    features=','.join(CONTRACT['root_features'])
    common=['--manifest-path','rust/Cargo.toml','--offline','--locked','--no-default-features','--features',features]
    metadata=json.loads(run(['cargo','metadata',*common,'--format-version=1','--filter-platform','wasm32-unknown-unknown']))
    packages={p['id']:p for p in metadata['packages']};nodes={n['id']:n for n in metadata['resolve']['nodes']}
    worker_id=next(i for i,p in packages.items() if p['name']=='lh-worker');ids=set()
    def visit(i):
        if i in ids:return
        ids.add(i)
        for d in nodes[i]['deps']:visit(d['pkg'])
    visit(worker_id)
    closure=[{'package':packages[i]['name'],'version':packages[i]['version'],'id':i,'features':sorted(nodes[i]['features'])} for i in sorted(ids)]
    effective=next(x['features'] for x in closure if x['package']=='lh-sim')
    if effective!=CONTRACT['effective_sim_features']:raise ValueError('Resolved feature mismatch')
    # Existing single shared target; never duplicate caches or target trees.
    print(json.dumps({'event':'BUILD','output':str(output),'free_disk_bytes':shutil.disk_usage(ROOT).free,
        'root_features':CONTRACT['root_features'],'effective_sim_features':effective}),flush=True)
    source=rust_inputs();output.mkdir(parents=True);worker=output/'worker';worker.mkdir()
    messages=run(['cargo','build',*common,'--release','--target','wasm32-unknown-unknown','-p','lh-worker','--message-format=json'])
    raw=ROOT/'rust/target/wasm32-unknown-unknown/release/lh_worker.wasm'
    run([bindgen,raw,'--target','web','--out-dir',worker,'--out-name','lh_worker'])
    capabilities=check_capabilities(json.loads(run(['node','scripts/getup-cold-worker-capabilities.mjs',worker])))
    artifacts=[json.loads(line) for line in messages.splitlines() if line.startswith('{')]
    selected={x['target']['name']:x for x in artifacts if x.get('reason')=='compiler-artifact'
        and x.get('target',{}).get('name') in ('lh_worker','lh_sim','rapier3d')}
    fingerprints=[]
    for p in (ROOT/'rust/target/wasm32-unknown-unknown/release/.fingerprint').glob('*/lib-*.json'):
        value=json.loads(p.read_text())
        name=p.stem.removeprefix('lib-')
        if name in selected and json.loads(value['features'])==sorted(selected[name]['features']):
            fingerprints.append({'path':str(p.relative_to(ROOT)),'sha256':sha256(p),'receipt':value})
    if len(fingerprints)!=3:raise ValueError('Exact selected worker/sim/rapier fingerprint set required')
    recipe=['scripts/build-getup-cold-worker.py','scripts/getup_worker_build.py',
        'scripts/getup-cold-worker-capabilities.mjs','scripts/getup-cold-worker-contract.mjs']
    receipt={'schema':'lh-cold-worker-build/v1','source_commit':run(['git','rev-parse','HEAD']),
        'source_dirty':bool(run(['git','diff','HEAD','--name-only'])),'source_inputs':source,
        'contract_sha256':sha256(CONTRACT_PATH),'root_features':CONTRACT['root_features'],
        'effective_sim_features':effective,'dependency_closure':closure,'toolchain':toolchain,
        'lock_sha256':sha256(ROOT/'rust/Cargo.lock'),'fingerprints':fingerprints,
        'cargo_artifacts':[x for x in artifacts if x.get('reason')=='compiler-artifact' and x.get('target',{}).get('name') in ('lh_worker','lh_sim','rapier3d')],
        'raw_wasm_sha256':sha256(raw),'worker_sha256':sha256(worker/'lh_worker_bg.wasm'),
        'glue_sha256':sha256(worker/'lh_worker.js'),'compiled_capabilities':capabilities,
        'recipe_sources':{p:sha256(ROOT/p) for p in recipe},
        'transformation':{'command':['wasm-bindgen','--target=web','--out-name=lh_worker'],'version':toolchain['wasm_bindgen'],
            'input_sha256':sha256(raw),'output_sha256':sha256(worker/'lh_worker_bg.wasm'),'other_transformations':[]},
        'historical_binary_equivalence_claimed':False,'physics_integrations':0,'trial_authorized':False}
    atomic_json(output/'build-receipt.json',receipt)
    print(json.dumps({'event':'BUILT','worker_sha256':receipt['worker_sha256'],'receipt_sha256':sha256(output/'build-receipt.json'),
        'worker_bytes':(worker/'lh_worker_bg.wasm').stat().st_size,'physics_integrations':0}),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output',type=Path);build(p.parse_args().output)
