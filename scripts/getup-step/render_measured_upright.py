#!/usr/bin/env python3
"""Render archived measured source qpos with independent FK, never physics.

Streams a finalized 800 Hz journal. Each display interval uses a distinct
recorded interval-end pose. Exact rational FPS preserves the source duration;
there is no pose interpolation or repeated final tail. Source ray origins use
the same coordinate transform as the meshes and are independently FK checked.
"""
import argparse
from collections import deque
from fractions import Fraction
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]


FULL_PHYSICAL_FIELDS = ('reviewedWalkTenStepPhysicalPass', 'reviewedWalkPilotPhysicalPass',
                        'terminalPelvisTenStepPhysicalPass', 'terminalPelvisPilotPhysicalPass')


def select_source_steps(native_rows, display_fps):
    """Distinct measured interval-end samples, exact 1x duration and final row."""
    frames = round(native_rows*display_fps/800)
    if not 0 < frames <= native_rows:
        raise ValueError('display sampling must select at least one distinct source pose')
    fps = Fraction(frames*800, native_rows)
    selected = [round(Fraction(i*native_rows, frames)) for i in range(1, frames+1)]
    assert len(set(selected)) == frames and selected[-1] == native_rows
    assert all(a < b for a, b in zip(selected, selected[1:]))
    assert all(abs(Fraction(step, 800)-Fraction(i, 1)/fps) <= Fraction(1, 1600)
               for i, step in enumerate(selected, 1))
    return selected, fps


def gate_verdict(minimum_walking_joint, minimum_capture, undefined_capture,
                 audit=None, native_rows=None, journal_sha256=None):
    """Renderer observations are limited; only a bound physical audit qualifies."""
    defined = bool(undefined_capture == 0 and minimum_capture is not None
                   and math.isfinite(minimum_capture))
    joints = bool(minimum_walking_joint is not None and math.isfinite(minimum_walking_joint)
                  and minimum_walking_joint >= 0.)
    audit_count = None if audit is None else audit.get('actualRecordedIntegrations',
        audit.get('referenceReplay', {}).get('actualRecordedIntegrations'))
    bound = bool(audit and audit.get('savedDataOnly') is True and audit.get('physicsIntegrations') == 0
                 and native_rows is not None and audit_count == native_rows
                 and journal_sha256 and audit.get('hashes', {}).get('frames.ndjson') == journal_sha256)
    field = next((field for field in FULL_PHYSICAL_FIELDS if audit and audit.get(field) is True), None)
    return dict(walkingJointAndCaptureGatesPass=bool(joints and defined and minimum_capture >= .003),
                walkingJointAndCaptureGatesScope='LIMITED: walking joint reserve and all-frame defined combined COM/capture >=3 mm only; not a physical qualification.',
                undefinedCaptureSamples=undefined_capture,
                allFrameDefinedNonnegativeCapturePass=bool(defined and minimum_capture >= 0.),
                allFrame3mmCaptureQualityPass=bool(defined and minimum_capture >= .003),
                independentPhysicalAuditBoundToJournal=bound,
                independentPhysicalAuditVerdictField=field,
                fullPhysicalPass=bool(bound and field and joints and defined and minimum_capture >= 0.))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('folder', type=Path)
    ap.add_argument('--model', type=Path, required=True)
    ap.add_argument('--preview-only', action='store_true')
    ap.add_argument('--subtitle')
    ap.add_argument('--title',default='Grands pas · essai de contrôle du bassin et du tronc')
    ap.add_argument('--audit',type=Path)
    ap.add_argument('--display-fps',type=int,choices=(5,10,20),default=20,
                    help='Display sampling only; rational encoded FPS still preserves exact source duration.')
    ap.add_argument('--width',type=int,choices=(800,1280),default=1280)
    ap.add_argument('--terminal-interrupted-journal', action='store_true',help='Explicitly confirmed terminal interrupted journal; derive duration from its actual rows, never invent a success summary.')
    ap.add_argument('--expected-model-sha')
    a = ap.parse_args()
    folder = a.folder.resolve()
    out = folder/'render'
    out.mkdir(exist_ok=True)
    os.environ['MPLCONFIGDIR'] = str(out/'.mpl-cache')
    os.environ['XDG_CACHE_HOME'] = str(out/'.cache')
    (out/'.cache').mkdir(exist_ok=True)
    import numpy as np
    from scipy.spatial.transform import Rotation
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection, LineCollection
    from PIL import Image, ImageDraw
    from source_math import P

    has_summary=(folder/'summary.json').exists()
    if has_summary:
        summary = json.loads((folder/'summary.json').read_text())
        multi = json.loads((folder/'multistep-summary.json').read_text())
        reported_native_integrations=summary['actualIntegrations']
        recorded=summary.get('recordedIntegrations',reported_native_integrations)
        if recorded!=reported_native_integrations:
            assert a.terminal_interrupted_journal and a.expected_model_sha, 'Explicit terminal recorded-prefix rendering requires the exact model hash.'
            assert 0<recorded<reported_native_integrations
            summary=dict(summary,actualIntegrations=recorded,physicsSimSeconds=recorded/800)
            multi=dict(multi,passed=False)
    else:
        assert a.terminal_interrupted_journal and a.expected_model_sha,'Missing final summary: explicit confirmed terminal interruption and exact model hash required.'
        with (folder/'frames.ndjson').open('rb') as f:recorded_count=sum(1 for line in f if line.strip())
        summary=dict(actualIntegrations=recorded_count,physicsSimSeconds=recorded_count/800,modelSHA256=a.expected_model_sha,stopped='deliberately-interrupted-after-independent-audit-failure',events=[])
        multi=dict(passed=False,steps=[],finalQuietS=0.)
        reported_native_integrations=None
    assert summary.get('physicalStateWritesAfterInitialization',0) == 0
    n = summary['actualIntegrations']
    duration = n/800
    assert abs(duration-summary['physicsSimSeconds']) < 1e-10
    source_xml = a.model.resolve()
    xml_sha = hashlib.sha256(source_xml.read_bytes()).hexdigest()
    assert xml_sha == summary['modelSHA256']
    if a.expected_model_sha:assert xml_sha==a.expected_model_sha
    geometry_path = ROOT/'rust/crates/model/data/getup-v1.json'
    model = json.loads(geometry_path.read_text())
    spec = importlib.util.spec_from_file_location('upright_render_fk', ROOT/'scripts/getup-step-reference.py')
    fk = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fk)
    kin = fk.Kinematics(model)
    mapping = json.loads((folder/'arm-mapping.json').read_text())
    controller = json.loads((folder/'controller-step-01.json').read_text())
    heading = np.array(controller['heading']); heading[2] = 0; heading /= np.linalg.norm(heading)
    source_steps, fps = select_source_steps(n,a.display_fps)
    selected = np.asarray(source_steps,dtype=int);frames=len(selected)
    assert len(set(selected)) == frames and selected[-1] == n
    wanted = set(selected.tolist()+[1])
    # Exact phase-onset and representative lift samples for inspection.
    for event in summary.get('events',[]):
        wanted.add(max(1,min(n,round(event['timeS']*800))))
        if event['event'] in ('translate','land'):
            wanted.add(max(1,min(n,round((event['timeS']-1)*800))))
    samples = {}; journal_hash = hashlib.sha256(); count=0;last_per_step={}; observed_events=[]
    last_phase=None; max_tilt={b:0. for b in ('torso','pelvis','torsoRelativePelvis')}
    quiet = deque(maxlen=4000); min_margin=1.; last_outside=None
    ray_invalid=0; first_lift=None; min_walking=None; min_capture=None; undefined_capture=0
    startup_end=None; startup_over=True; target_identity_error=0.; max_arm_tracking_error=0.
    with (folder/'frames.ndjson').open('rb') as f:
        for raw in f:
            journal_hash.update(raw)
            r = json.loads(raw); count += 1
            assert r['sourceStep'] == count
            assert abs(r['elapsedS']-count/800) < 1e-10
            assert abs(r['sourceTimeS']-count/800) < 1e-8
            if first_lift is None and r['phase']=='lift':first_lift=r['elapsedS']
            margin=r['jointLimitMinimumMarginRad']; min_margin=min(min_margin,margin)
            if margin<0:last_outside=r['elapsedS']
            if startup_over:
                if margin<0:startup_end=r['elapsedS']
                else:startup_over=False
            if first_lift is not None:min_walking=margin if min_walking is None else min(min_walking,margin)
            capture=r['minimumComCaptureMarginM']
            if capture is None or not math.isfinite(capture):undefined_capture+=1
            else:min_capture=capture if min_capture is None else min(min_capture,capture)
            rays = r['jointGroundRays']; assert len(rays)==22
            for ray in rays:
                valid=ray['hit'] and ray['ownBodyExcluded'] and not ray['anchorGroundDistanceIsColliderClearance']
                valid=valid and np.max(np.abs(np.array(ray['originSource'])+np.array(ray['directionSource'])*ray['distanceM']-ray['pointSource']))<1e-9
                ray_invalid += not valid
            for body in max_tilt:max_tilt[body]=max(max_tilt[body],r['uprightMeasured'][body]['tiltDeg'])
            swing=r['opposedArmSwing']
            target=np.array(mapping['neutralArmPitchRad'])-np.array(swing['legPitchRad'])+controller['leg_origin']
            target_identity_error=max(target_identity_error,float(np.max(abs(target-swing['targetArmPitchRad']))))
            if r['phase']in ('lift','translate','land'):
                max_arm_tracking_error=max(max_arm_tracking_error,float(np.max(abs(np.array(swing['armPitchRad'])-swing['targetArmPitchRad']))))
            if r['phase']!=last_phase:
                wanted.add(count);last_phase=r['phase']
                observed_events.append(dict(event=r['phase'],timeS=r['elapsedS'],stepIndex=r['stepIndex']))
            if count in wanted:samples[count]=r
            last_per_step[r['stepIndex']]=r
            quiet.append(r)
    assert count==n and ray_invalid==0
    final=r
    audit=None
    if a.audit:
        audit=json.loads(a.audit.read_text())
        verdict=gate_verdict(min_walking,min_capture,undefined_capture,audit,n,journal_hash.hexdigest())
        assert verdict['independentPhysicalAuditBoundToJournal'], 'Audit must be saved-data-only and match the exact journal hash and row count.'
    verdict=gate_verdict(min_walking,min_capture,undefined_capture,audit,n,journal_hash.hexdigest())
    if not has_summary:
        summary['events']=observed_events
        for step,last in sorted(last_per_step.items()):
            if step<final['stepIndex']:
                multi['steps'].append(dict(index=step,swingSide='left'if last['swingSide']==0 else'right',requestedDistanceM=last['requestedForwardM'],achievedForwardM=last['swingForwardDisplacementM'],completedNativeStep=last['sourceStep'],completedTimeS=last['elapsedS'],qualification='Cycle recorded before next step; independent dynamic gates failed.'))
    assert abs(final.get('quietConsecutiveS',0)-multi.get('finalQuietS',0))<1e-9
    quiet_rows=list(quiet)
    def is_base_quiet(r):
        return (r['phase']=='stand' and all(f['loadN']>=3 for f in r['feet'])
                and sum(f['loadN']for f in r['feet'])>=.8*model['totalMassKg']*9.81
                and r['torsoTiltDeg']<=5 and r['maximumBodyLinearMps']<=.22
                and r['maximumBodyAngularRadps']<=.65 and r['minimumComCaptureMarginM']is not None
                and r['minimumComCaptureMarginM']>=.003 and .780625<=r['com'][2]<=.829976
                and r.get('standingAnchorDriftM',999)<=.01 and r['maximumSelfPenetrationM']<=.005
                and r['jointLimitMinimumMarginRad']>=0)
    def is_quiet(r):
        u=r['uprightMeasured']
        return is_base_quiet(r)and u['pelvis']['tiltDeg']<=5 and u['torsoRelativePelvis']['tiltDeg']<=5
    final_base_quiet_verified=len(quiet_rows)==4000 and all(is_base_quiet(r)for r in quiet_rows)
    final_quiet_verified=len(quiet_rows)==4000 and all(is_quiet(r) for r in quiet_rows)
    if multi.get('sequenceAndQuietCompleted',multi.get('passed')):
        assert len(quiet_rows)==4000
        assert all(r['phase']=='stand' and abs(r['quietConsecutiveS']-(i+1)/800)<1e-9 for i,r in enumerate(quiet_rows))
        assert all(all(f['loadN']>=3 for f in r['feet']) and r['standingAnchorDriftM']<=.01 for r in quiet_rows)
    print(json.dumps({'loadedRows':count,'selectedActualPoses':frames,'durationS':duration,'xmlSHA256':xml_sha}),flush=True)

    meshes=[(p['id'],np.array(p['geometry']['vertices']).reshape(-1,3),np.array(p['geometry']['indices']).reshape(-1,3)) for p in model['segments'] if p['geometry']['vertices']]
    hinge_parts={p['reference']['hinge']['name']:p for p in model['segments'] if p['reference'].get('hinge')}
    colors={'left_leg':np.array([.85,.40,.18]),'right_leg':np.array([.20,.43,.72]),'left_arm':np.array([.80,.29,.53]),'right_arm':np.array([.25,.60,.40]),'torso':np.array([.24,.55,.55]),'pelvis':np.array([.47,.38,.61])}
    def body_color(name):
        if any(v in name for v in ('arm','hand')):return colors['left_arm' if name.startswith('left') else 'right_arm']
        if any(v in name for v in ('thigh','shin','foot')):return colors['left_leg' if name.startswith('left') else 'right_leg']
        return colors.get(name,colors['torso'])
    def orientation(R):
        yaw,pitch,roll=Rotation.from_matrix(R).as_euler('ZYX')
        return dict(yawDeg=np.rad2deg(yaw),pitchDeg=np.rad2deg(pitch),rollDeg=np.rad2deg(roll),tiltDeg=np.rad2deg(np.arccos(np.clip(R[2,2],-1,1))))
    def poses_for(r):
        q=np.array(r['qpos']); assert len(q)==28 and np.isfinite(q).all()
        root=np.eye(4); root[:3,:3]=P.T@Rotation.from_quat(q[[4,5,6,3]]).as_matrix()@P; root[:3,3]=P.T@q[:3]
        return kin.forward(q[7:],root)
    poses={}; com_error=foot_error=clearance_error=ray_error=angle_error=limb_pitch_error=0.
    for i,r in samples.items():
        ps=poses_for(r);poses[i]=ps
        com=sum(p['massKg']*(ps[p['id']]@np.r_[fk.xyz(p['goldenMass']['centerOfMass']),1])[:3] for p in model['segments'])/model['totalMassKg']
        com_error=max(com_error,float(np.max(abs(P@com-r['com']))))
        for j,side in enumerate(('left','right')):
            foot_error=max(foot_error,float(np.max(abs(P@ps[side+'_foot'][:3,3]-r['feet'][j]['position']))))
            clearance_error=max(clearance_error,abs(kin.bottom(side+'_foot',ps[side+'_foot'])-r['feet'][j]['clearance']))
        for ray in r['jointGroundRays']:
            if ray['joint']=='root':anchor=ps['torso'][:3,3]
            else:
                p=hinge_parts[ray['joint']]
                anchor=(ps[p['parent']]@kin.parent_frames[p['id']])[:3,3]
            ray_error=max(ray_error,float(np.max(abs(P@anchor-ray['originSource']))))
        rt=P@ps['torso'][:3,:3]@P.T; rp=P@ps['pelvis'][:3,:3]@P.T
        for side_index,side in enumerate(('right','left')):
            for kind,bodies in [('arm',('_upper_arm','_hand')),('leg',('_thigh','_shin'))]:
                delta=rt.T@P@(ps[side+bodies[1]][:3,3]-ps[side+bodies[0]][:3,3])
                measured=np.arctan2(delta[0],-delta[2])
                limb_pitch_error=max(limb_pitch_error,abs(measured-r['opposedArmSwing'][kind+'PitchRad'][side_index]))
        for body,R in [('torso',rt),('pelvis',rp),('torsoRelativePelvis',rp.T@rt)]:
            obs=orientation(R)
            angle_error=max(angle_error,max(abs((obs[k]-r['uprightMeasured'][body][k]+180)%360-180)for k in obs))
    assert max(com_error,foot_error,clearance_error,ray_error)<1e-9,(com_error,foot_error,clearance_error,ray_error)
    assert angle_error<1e-8,angle_error
    assert limb_pitch_error<1e-10 and target_identity_error<1e-12
    up=np.array([0.,0.,1.]); left=np.cross(up,heading)
    def basis(depth,el):
        el=np.deg2rad(el);x=np.cross(up,depth);return np.column_stack([x,np.cos(el)*up-np.sin(el)*depth,np.cos(el)*depth+np.sin(el)*up])
    views=[basis(np.cos(.17)*heading-np.sin(.17)*left,5),basis(-left,3)]
    extents=[[] for _ in views]
    for ps in poses.values():
        for name,v,t in meshes:
            world=(v@ps[name][:3,:3].T+ps[name][:3,3])@P.T
            for j,B in enumerate(views):extents[j].append(world@B)
    extents=[np.concatenate(e) for e in extents]
    ylo=min(e[:,1].min()for e in extents)-.075; yhi=max(e[:,1].max()for e in extents)+.06
    ylo=min(ylo,-.12); yhi=max(yhi,1.65); span=yhi-ylo
    panel_ratio=(.445*12.8)/(.595*7.2); xspan=span*panel_ratio
    bounds=[]
    for e in extents:
        center=(e[:,0].min()+e[:,0].max())/2;bounds.append((center-xspan/2,center+xspan/2))
    fig=plt.figure(figsize=(12.8,7.2),dpi=a.width/12.8,facecolor='#f7f8f5')
    axes=[];collections=[];rays_artist=[];dots=[];hits=[];bodyaxes=[]
    for j,B in enumerate(views):
        ax=fig.add_axes([.035+j*.495,.215,.445,.595]);ax.set_aspect('equal');ax.set_xlim(*bounds[j]);ax.set_ylim(ylo,yhi);ax.axis('off')
        for val in np.arange(-3,3.01,.1):
            for endpoints in [np.array([[-3,val,0],[3,val,0]]),np.array([[val,-3,0],[val,3,0]])]:
                z=endpoints@B;ax.plot(z[:,0],z[:,1],color='#dce3de',lw=.45,zorder=0)
        poly=PolyCollection([],edgecolors='none',zorder=3);ax.add_collection(poly);collections.append(poly)
        lines=LineCollection([],colors='#6a8d88',linewidths=.65,alpha=.3,zorder=4);ax.add_collection(lines);rays_artist.append(lines)
        dots.append(ax.scatter([],[],s=11,c='#536965',edgecolors='#f7f8f5',linewidths=.45,zorder=6))
        hits.append(ax.scatter([],[],s=9,c='#7b9990',marker='+',linewidths=.6,zorder=2))
        for body,color in [('torso','#157d86'),('pelvis','#845ca2')]:
            line,=ax.plot([],[],color=color,lw=2,zorder=7);vertical,=ax.plot([],[],color=color,lw=.8,ls=':',alpha=.8,zorder=6)
            bodyaxes.append((j,body,line,vertical))
        axes.append(ax)
        fig.text(.257+j*.495,.815,'Face · rayons articulaires en surimpression' if j==0 else 'Profil · axes tronc / bassin et verticale',ha='center',fontsize=10,color='#526963')
    fig.text(.5,.959,a.title,ha='center',fontsize=18,color='#254b48')
    completed=[s for s in multi.get('steps',[])if s.get('completedNativeStep')]
    successful=bool(multi.get('sequenceAndQuietCompleted',multi.get('passed')))
    dynamic_valid=verdict['walkingJointAndCaptureGatesPass']
    quiet_label=' + 5 s calmes finales vérifiées' if final_quiet_verified else ''
    qualification=' · contrôle limité articulations/réserve capture 3 mm '+('réussi' if dynamic_valid else 'non validé')
    qualification+=' · audit physique indépendant '+('validé' if verdict['fullPhysicalPass'] else 'non validé')
    subtitle=a.subtitle or (f"{len(completed)} pas enregistré{'s' if len(completed)!=1 else ''}{quiet_label}{qualification}" if successful else f"Essai arrêté · {len(completed)} pas terminé{'s' if len(completed)!=1 else ''} · fin réelle conservée{qualification}")
    fig.text(.5,.916,subtitle,ha='center',fontsize=12,color='#365f57')
    phase_text=fig.text(.5,.868,'',ha='center',fontsize=12,color='#805838')
    clock=fig.text(.5,.177,'',ha='center',fontsize=11,color='#315b55')
    metrics=fig.text(.5,.137,'',ha='center',fontsize=10.5,color='#315b55')
    motion=fig.text(.5,.100,'',ha='center',fontsize=10,color='#4f655e')
    fig.text(.5,.064,'G : jambe orange / bras rose · D : jambe bleue / bras vert · points = origines des 22 rayons (axes parfois confondus)',ha='center',fontsize=8.5,color='#61716b')
    warning=f"Départ : cheville hors limite {max(0,-np.rad2deg(samples[1]['jointLimitMinimumMarginRad'])):.2f}° ({1000*(startup_end or 0):.2f} ms)"
    if not dynamic_valid:
        joint_text=f"{max(0,-np.rad2deg(min_walking)):.2f}°" if min_walking is not None else 'indéfinie'
        capture_text=f"{min_capture*1000:+.4f} mm" if min_capture is not None else 'indéfinie'
        warning+=f" · marche : excursion articulaire {joint_text}, capture min. {capture_text}, {undefined_capture} indéfinie(s)"
    else:warning+=' · phases de marche dans les limites articulaires'
    fig.text(.5,.027,warning+' · vitesse réelle 1×, FK seul',ha='center',fontsize=8.5,color='#9a593c')
    light=np.array([-.2,-.4,1.]);light/=np.linalg.norm(light)
    phase_names={'transfer':'transfert d’appui','lift':'levée','translate':'avance','land':'descente','settle':'stabilisation','stand':'maintien calme'}
    def update(i):
        r=samples[i];ps=poses[i]
        tris=[];facecolors=[]
        for name,v,t in meshes:
            world=(v@ps[name][:3,:3].T+ps[name][:3,3])@P.T;tri=world[t]
            norm=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);norm/=np.maximum(np.linalg.norm(norm,axis=1)[:,None],1e-12)
            shade=.57+.43*np.maximum(0,norm@light);tris.append(tri);facecolors.append(shade[:,None]*body_color(name))
        tri=np.concatenate(tris);color=np.concatenate(facecolors)
        # Deduplicate coincident hinge-axis origins without inventing ray offsets.
        rr=[];seen=set()
        for ray in r['jointGroundRays']:
            key=tuple(np.round(ray['originSource'],10))
            if key not in seen:rr.append(ray);seen.add(key)
        for j,B in enumerate(views):
            projected=tri@B;order=np.argsort(projected[:,:,2].mean(axis=1));collections[j].set_verts(projected[order,:,:2]);collections[j].set_facecolors(color[order])
            if j==0:
                segments=np.array([[x['originSource'],x['pointSource']]for x in rr])@B
                rays_artist[j].set_segments(segments[:,:,:2]);dots[j].set_offsets(segments[:,0,:2]);hits[j].set_offsets(segments[:,1,:2])
        for j,body,line,vertical in bodyaxes:
            center=P@ps[body][:3,3];direction=P@ps[body][:3,1]
            z=np.array([center,center+direction*.24])@views[j];line.set_data(z[:,0],z[:,1])
            z=np.array([center,center+up*.24])@views[j];vertical.set_data(z[:,0],z[:,1])
        side='gauche' if r['swingSide']==0 else 'droit'
        if r['elapsedS']<=mapping['placementDurationS']:
            phase_text.set_text('0–4 s : placement physique des bras, avant le premier pas')
        else:
            phase_text.set_text(f"Pas {r['stepIndex']} · pied {side} · {phase_names.get(r['phase'],r['phase'])} · demandé {r['requestedForwardM']*100:.0f} cm / mesuré {r['swingForwardDisplacementM']*100:.2f} cm")
        u=r['uprightMeasured']
        clock.set_text(f"Temps physique : {r['elapsedS']:.3f} / {duration:.3f} s · inclinaison tronc {u['torso']['tiltDeg']:.2f}° · bassin {u['pelvis']['tiltDeg']:.2f}° · tronc/bassin {u['torsoRelativePelvis']['tiltDeg']:.2f}°")
        metrics.set_text(f"Dégagement sous le pied : G {r['feet'][0]['clearance']*1000:.1f} / D {r['feet'][1]['clearance']*1000:.1f} mm · glissement cumulé G/D {r['loadedSlipDistanceM'][0]*1000:.2f}/{r['loadedSlipDistanceM'][1]*1000:.2f} mm · calme {r.get('quietConsecutiveS',0):.2f} s")
        swing=r['opposedArmSwing'];idx=1-r['swingSide'];leg=np.rad2deg(swing['legPitchRad'][idx]-controller['leg_origin'][idx]);arm=np.rad2deg(swing['armPitchRad'][idx]-mapping['neutralArmPitchRad'][idx]);err=np.rad2deg(swing['armPitchRad'][idx]-swing['targetArmPitchRad'][idx])
        if r['elapsedS']<=4:motion.set_text('Placement mesuré des bras ; couplage opposé ensuite · distance articulaire au sol ≠ dégagement du pied')
        else:motion.set_text(f"Variations mesurées du côté actif : bras {arm:+.2f}° / cuisse {leg:+.2f}° · erreur de suivi {err:+.3f}° · rayons : distance au sol ≠ dégagement")
        return r
    # Save representative true poses before expensive encoding.
    inspect_ids=[1,min(samples,key=lambda i:abs(i/800-2))]
    for event in summary.get('events',[]):
        if event['event']in ('land','translate'):
            target=max(1,round((event['timeS']-1)*800));inspect_ids.append(min(samples,key=lambda i:abs(i-target)))
    inspect_ids.append(n);inspect_ids=list(dict.fromkeys(inspect_ids))
    thumb_paths=[]
    for j,i in enumerate(inspect_ids):
        update(i);p=out/f'pose-{j:02d}-source-{i:06d}.png';fig.savefig(p,dpi=100);thumb_paths.append(p)
    tile_w,tile_h=640,360;cols=2;rows=(len(thumb_paths)+1)//2
    sheet=Image.new('RGB',(cols*tile_w,rows*tile_h),'#f7f8f5')
    for j,p in enumerate(thumb_paths):sheet.paste(Image.open(p).convert('RGB').resize((tile_w,tile_h)),((j%2)*tile_w,(j//2)*tile_h))
    sheet_path=out/'measured-upright-contact-sheet.png';sheet.save(sheet_path)
    video=out/'laboratoirehumain-grands-pas-debout-mesures.mp4'
    if not a.preview_only:
        cmd=['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pixel_format','rgba','-video_size',f'{a.width}x{a.width*9//16}','-framerate',str(fps),'-i','-','-an','-c:v','libx264','-preset','medium','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(video)]
        proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
        try:
            for j,i in enumerate(selected):
                update(int(i));fig.canvas.draw();proc.stdin.write(fig.canvas.buffer_rgba())
                if j%200==0:print(f'Render {j+1}/{frames}',flush=True)
            proc.stdin.close();assert proc.wait()==0
        except BaseException:
            proc.kill();raise
        probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','stream=width,height,nb_frames,duration,r_frame_rate','-of','json',str(video)]))
        stream=probe['streams'][0];assert int(stream['nb_frames'])==frames and abs(float(stream['duration'])-duration)<1e-5
        for tag,k in [('placement',max(0,round(2*float(fps)))),('opposition',max(0,int(frames*.55))),('last',frames-1)]:
            subprocess.run(['ffmpeg','-y','-loglevel','error','-i',str(video),'-vf',f'select=eq(n\\,{k})','-frames:v','1',str(out/f'decoded-{tag}.png')],check=True)
    plt.close(fig)
    report=dict(video=str(video),contactSheet=str(sheet_path),sourceJournal=str(folder/'frames.ndjson'),sourceModel=str(source_xml),sourceXMLSHA256=xml_sha,sourceRecordedRows=n,sourcePhysicalDurationS=duration,encodedDurationS=duration,videoFrames=frames,exactVideoFPS=str(fps),videoSourceSteps=selected.tolist(),sourceJournalSHA256=journal_hash.hexdigest(),additionalPhysicsIntegrations=0,simulatorImportedOrRun=False,plannedPosesUsed=False,poseInterpolation=False,duplicatedVideoPoses=0,terminalTailFramesAdded=0,FKBodyCOMMaxErrorM=com_error,FKFootOriginMaxErrorM=foot_error,FKWholeFootClearanceMaxErrorM=clearance_error,FKJointRayAnchorMaxErrorM=ray_error,FKOrientationMaxWrappedErrorDeg=angle_error,validGroundRaysEveryRow=ray_invalid==0,groundRayCountPerRow=22,distinctOriginsAtFirstSample=len({tuple(np.round(x['originSource'],10))for x in samples[1]['jointGroundRays']}),measuredMaxTiltDeg=max_tilt,minimumAllSampleJointReserveDeg=float(np.rad2deg(min_margin)),startupJointOverrunEndsS=startup_end,lastOutsideJointLimitS=last_outside,minimumWalkingJointReserveDeg=float(np.rad2deg(min_walking))if min_walking is not None else None,minimumCaptureMarginM=min_capture,finalMeasuredQuietS=final.get('quietConsecutiveS',0),finalQuietIndependentlyVerified=final_quiet_verified,completedSteps=completed,stopped=summary['stopped'],inputSHA256={str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in [folder/'summary.json',folder/'multistep-summary.json',geometry_path,source_xml,Path(__file__).resolve()] if p.exists()})
    report.update(verdict)
    report['maximumDisplayIntervalEndSampleErrorS']=max(abs(float(Fraction(i,1)/fps)-step/800)
        for i,step in enumerate(source_steps,1))
    report.update(finalBaseQuietIndependentlyVerified=final_base_quiet_verified,
                  finalUprightQuietIndependentlyVerified=final_quiet_verified,
                  encodedVideoCreated=not a.preview_only,displayWidthPx=a.width,
                  displaySamplingFPS=a.display_fps,
                  sourceNativeIntegrationsReported=reported_native_integrations,
                  unjournaledTerminalIntegrations=(reported_native_integrations-n)if reported_native_integrations is not None else None,
                  FKLimbSagittalPitchMaxErrorRad=limb_pitch_error,armTargetIdentityMaxErrorRad=target_identity_error,
                  maximumGaitArmTrackingErrorDeg=float(np.rad2deg(max_arm_tracking_error)),sourceFinalSummaryPresent=has_summary,
                  finalRecordedStepIndex=final['stepIndex'],finalRecordedPhase=final['phase'],finalRecordedForwardM=final['swingForwardDisplacementM'])
    if a.audit:
        report.update(independentAudit=str(a.audit.resolve()),independentAuditSHA256=hashlib.sha256(a.audit.read_bytes()).hexdigest(),
                      independentlyAuditedMotionPass=audit.get('auditedMotionPass'),independentlyAuditedUprightPass=audit.get('auditedUprightStepPass'))
    (out/'measured-upright-render-report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items()if k not in ('videoSourceSteps','completedSteps','inputSHA256')}),flush=True)


if __name__=='__main__':main()
