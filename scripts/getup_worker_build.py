"""Exact semantic worker gate. Pure checks; never constructs or steps a Runtime."""
import hashlib,json,subprocess
from pathlib import Path
from getup_learning_loop import ContractError
from experiment_backup import sha256
ROOT=Path(__file__).resolve().parents[1]
CONTRACT_PATH=ROOT/'scripts/getup-cold-worker-contract.json'
CONTRACT=json.loads(CONTRACT_PATH.read_text())

def check_capabilities(value):
    if (value.get('schema')!='lh-getup-compiled-capabilities/v1'
            or value.get('physicsIntegrations')!=0 or value.get('constructsRuntime') is not False
            or value.get('effectiveSimFeatures')!=CONTRACT['effective_sim_features']):
        raise ContractError('Wrong compiled controller feature closure; hashes alone are insufficient')
    return value

def rust_inputs(root=ROOT):
    names=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','rust'],cwd=root,text=True).splitlines()
    # Source and lock only, never build outputs/caches or ignored checkpoints.
    names=sorted(set(n for n in names if not set(Path(n).parts)&{'target','target-wasm','dist'}))
    return {name:sha256(root/name) for name in names if (root/name).is_file()}

def validate_build_receipt(path,worker,profile):
    receipt=json.loads(Path(path).read_text());worker=Path(worker)
    if receipt.get('schema')!='lh-cold-worker-build/v1' or receipt.get('contract_sha256')!=sha256(CONTRACT_PATH):
        raise ContractError('Reviewed semantic worker recipe missing or changed')
    if receipt.get('root_features')!=CONTRACT['root_features'] or receipt.get('source_inputs')!=rust_inputs():
        raise ContractError('Worker recipe/source inputs changed')
    if receipt.get('lock_sha256')!=sha256(ROOT/'rust/Cargo.lock'):
        raise ContractError('Worker dependency lock changed')
    if sha256(profile)!=CONTRACT['profile_sha256']:
        raise ContractError('Frozen numerical profile changed')
    for name,digest in CONTRACT['model_sha256'].items():
        if sha256(ROOT/'rust/crates/sim/data/getup-policy'/name)!=digest:
            raise ContractError('Frozen WEAK/SLOW policy changed')
    if (receipt.get('worker_sha256')!=sha256(worker/'lh_worker_bg.wasm')
            or receipt.get('glue_sha256')!=sha256(worker/'lh_worker.js')
            or receipt['worker_sha256']==CONTRACT['wrong_variant_wasm_sha256']):
        raise ContractError('Wrong or substituted worker artifact')
    if receipt.get('effective_sim_features')!=CONTRACT['effective_sim_features']:
        raise ContractError('Cargo resolved wrong feature closure')
    if (receipt.get('compiled_capabilities')!=check_capabilities(receipt.get('compiled_capabilities',{}))
            or not receipt.get('toolchain') or not receipt.get('dependency_closure')
            or not receipt.get('fingerprints') or not receipt.get('raw_wasm_sha256')
            or receipt.get('transformation',{}).get('command',[])[:2]!=['wasm-bindgen','--target=web']):
        raise ContractError('Incomplete compiler/dependency/transformation provenance')
    sim=[x for x in receipt['dependency_closure'] if x['package']=='lh-sim']
    artifacts={x['target']['name']:x for x in receipt.get('cargo_artifacts',[])}
    fingerprints=receipt['fingerprints']
    if (len(sim)!=1 or sim[0]['features']!=CONTRACT['effective_sim_features']
            or set(artifacts)!={'lh_worker','lh_sim','rapier3d'} or len(fingerprints)!=3
            or artifacts['lh_sim']['features']!=CONTRACT['effective_sim_features']):
        raise ContractError('Selected Cargo dependency/artifact closure differs')
    for item in fingerprints:
        name=Path(item['path']).stem.removeprefix('lib-')
        if name not in artifacts or json.loads(item['receipt']['features'])!=sorted(artifacts[name]['features']):
            raise ContractError('Fingerprint is not the selected compiled artifact')
    if (receipt['transformation'].get('input_sha256')!=receipt['raw_wasm_sha256']
            or receipt['transformation'].get('output_sha256')!=receipt['worker_sha256']
            or receipt['transformation'].get('other_transformations')!=[]
            or not receipt['toolchain'].get('rustc','').startswith('rustc 1.89.0 ')
            or receipt['toolchain'].get('wasm_bindgen')!='wasm-bindgen 0.2.108'):
        raise ContractError('Toolchain/WASM transformation provenance differs')
    for name,digest in receipt.get('recipe_sources',{}).items():
        if sha256(ROOT/name)!=digest:raise ContractError('Worker recipe implementation changed')
    if set(receipt.get('recipe_sources',{}))!={'scripts/build-getup-cold-worker.py','scripts/getup_worker_build.py',
            'scripts/getup-cold-worker-capabilities.mjs','scripts/getup-cold-worker-contract.mjs'}:
        raise ContractError('Incomplete worker recipe inventory')
    return receipt
