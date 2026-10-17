"""Three archived native poses only: no FK, collision query or integration."""
from pathlib import Path
import hashlib
import json
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/lh-recorded-parity-mpl')
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
source = HERE / 'native-run/native-trace.jsonl'
ledger = json.loads((HERE / 'native-run/physical-ledger.json').read_text())['payload']
assert ledger['status'] == 'closed-success'
assert hashlib.sha256(source.read_bytes()).hexdigest() == ledger['trace']['sha256']
selected = {}
for line in source.open():
    row = json.loads(line)
    if row['kind'] == 'initial':
        selected['initial'] = row['state']['snapshot']
    if row['kind'] == 'prefix' and row['frame']['nativeSubstep'] == 12:
        selected['prefix'] = row['frame']['snapshot']
    if row['kind'] == 'actual' and row['frame']['nativeSubstep'] == 16:
        if row['frame']['owner']['feedbackScale'] == 1:
            selected['actual'] = row['frame']['snapshot']
assert len(selected) == 3
parts = json.loads((ROOT / 'rust/crates/model/data/canonical-v1.json').read_text())['segments']
fig = plt.figure(figsize=(12, 5), facecolor='white')
titles = ['État initial archivé', 'Préfixe vérifié · 12 pas', 'Facteur 1 · 4 pas exécutés']
for index, (key, title) in enumerate(zip(['initial', 'prefix', 'actual'], titles)):
    snapshot = selected[key]
    ax = fig.add_subplot(1, 3, index + 1, projection='3d')
    for part, pose in zip(parts, snapshot['segments']):
        assert part['id'] == pose['id']
        v = np.array(part['geometry']['vertices']).reshape(-1, 3)
        triangles = np.array(part['geometry']['indices']).reshape(-1, 3)
        q = [pose['rotation'][k] for k in 'xyzw']
        p = np.array([pose['position'][k] for k in 'xyz'])
        rendered = (Rotation.from_quat(q).apply(v) + p)[:, [0, 2, 1]]
        ax.add_collection3d(Poly3DCollection(rendered[triangles], facecolor='#537d99',
                                           edgecolor='none', alpha=.92))
    ground = [[[-.7,-.6,0],[.7,-.6,0],[.7,.8,0],[-.7,.8,0]]]
    ax.add_collection3d(Poly3DCollection(ground, facecolor='#d8dee5', alpha=.4))
    ax.set(xlim=(-.7,.7), ylim=(-.6,.8), zlim=(0,2.05))
    ax.set_box_aspect([1.4,1.4,2.05]);ax.view_init(elev=12,azim=-52)
    ax.set_title(f"{title}\nt = {snapshot['simulation_time_s']:.4f} s", fontsize=10)
    ax.tick_params(labelsize=6)
fig.suptitle('Parité native vérifiée sur une fenêtre courte', fontsize=15)
fig.text(.5,.025,'Rendu des 25 poses archivées · aucune interpolation ni nouvelle physique · aucun relevage qualifié',
         ha='center',fontsize=9)
fig.tight_layout(rect=(0,.07,1,.92))
output = HERE / 'recorded-poses.png'
fig.savefig(output,dpi=150);plt.close(fig)
receipt = {'sourceSha256':ledger['trace']['sha256'],'usesArchivedNativePoses':True,
           'meshTransformsOnly':True,'additionalPhysicsIntegrations':0,'fkQueries':0,
           'collisionQueries':0,'poses':3,'fullRecoveryQualified':False,
           'outputSha256':hashlib.sha256(output.read_bytes()).hexdigest()}
(HERE / 'render-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(receipt))
