#!/usr/bin/env python3
"""Bounded diagnostic only: pinned MuJoCo 2.1.3 versus archived Rapier.
Preparation performs forward kinematics/dynamics, never time integration.
Run mode requires a separately reserved, frozen cumulative ledger.
"""
import argparse,ctypes as ct,datetime,hashlib,json,math,os,pathlib,signal,time
PROCESS_STARTED=time.monotonic()
import numpy as np
from scipy.spatial.transform import Rotation
import torch

ROOT=pathlib.Path(__file__).resolve().parents[2]
OUT=ROOT/'outputs/getup-mujoco-comparison-2026-10-16'
LEDGER=ROOT/'docs/evidence/getup-mujoco-comparison-2026-10-16.json'
ARCHIVE=ROOT/'outputs/getup-policy-native-clock-2026-10-16/reference-report.json'
MODEL=ROOT/'rust/crates/model/data/getup-v1.json'
WEIGHTS=ROOT/'outputs/getup-policy-2026-10-16/source/teacher.pt'
POWER=.663420431
C=np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
sha=lambda p:hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()
vec=lambda x:np.array([x[k] for k in 'xyz'])
rot=lambda x:Rotation.from_quat([x[k] for k in 'xyzw'])

def write(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')

def actor():
    assert sha(WEIGHTS)=='9935efee1b1d46e796bb241dc1240f1905ad40e0d20284909cc063575606b244'
    assert tuple(map(int,torch.__version__.split('+')[0].split('.')[:2]))>=(2,6)
    torch.set_num_threads(1)
    net=torch.nn.Sequential(torch.nn.Linear(68,1024),torch.nn.ReLU(),torch.nn.Linear(1024,1024),torch.nn.ReLU(),torch.nn.Linear(1024,42))
    weights=torch.load(WEIGHTS,map_location='cpu',weights_only=True)
    assert all(type(t) is torch.Tensor and t.dtype==torch.float32 and torch.isfinite(t).all() for t in weights.values())
    net.load_state_dict({k.removeprefix('trunk.'):v for k,v in weights.items()},strict=True);net.eval()
    def infer(physical):
        x=torch.from_numpy(np.r_[physical,np.float32(POWER)].astype(np.float32))[None,:]
        with torch.inference_mode():return (torch.tanh(net(x)[0,:21])*x[0,-1]).numpy().copy()
    return infer

class World:
    forwards=0
    def __init__(self,xml):
        self.lib=ct.CDLL(str(OUT/'mujoco_bridge.so'));l=self.lib
        for name,args,ret in [('lh_version',[],ct.c_int),('lh_open',[ct.c_char_p,ct.c_char_p,ct.c_int],ct.c_void_p),('lh_close',[ct.c_void_p],None),('lh_dim',[ct.c_void_p,ct.c_int],ct.c_int),('lh_dt',[ct.c_void_p],ct.c_double),('lh_time',[ct.c_void_p],ct.c_double),('lh_warnings',[ct.c_void_p],ct.c_int),('lh_id',[ct.c_void_p,ct.c_int,ct.c_char_p],ct.c_int),('lh_name',[ct.c_void_p,ct.c_int,ct.c_int],ct.c_char_p),('lh_array',[ct.c_void_p,ct.c_int],ct.POINTER(ct.c_double)),('lh_set_state',[ct.c_void_p,ct.POINTER(ct.c_double),ct.POINTER(ct.c_double)],None),('lh_forward',[ct.c_void_p],None),('lh_step',[ct.c_void_p,ct.c_int],ct.c_int),('lh_contact',[ct.c_void_p,ct.c_int,ct.POINTER(ct.c_double)],ct.c_int)]:
            f=getattr(l,name);f.argtypes=args;f.restype=ret
        assert l.lh_version()==213
        error=ct.create_string_buffer(1024);self.h=l.lh_open(os.fsencode(xml),error,1024)
        if not self.h:raise RuntimeError(error.value.decode())
        assert [self.dim(i) for i in range(6)]==[28,27,21,17,20,22]
        assert self.dim(8)==0,'Only source Euler/dm_control split-step supported'
        self.names=[l.lh_name(self.h,1,i).decode() if i else 'world' for i in range(17)]
        self.ids={n:i for i,n in enumerate(self.names)}
    def close(self):self.lib.lh_close(self.h);self.h=None
    def dim(self,k):return self.lib.lh_dim(self.h,k)
    def arr(self,k,shape):return np.ctypeslib.as_array(self.lib.lh_array(self.h,k),shape=(math.prod(shape),)).reshape(shape)
    def state(self,segments,physical):
        torso=next(p for p in segments if p['id']=='torso')
        q=Rotation.from_matrix(C.T@rot(torso['rotation']).as_matrix()@C).as_quat()
        qpos=np.r_[C.T@vec(torso['position']),q[[3,0,1,2]],physical[:21]].astype(np.float64)
        qvel=np.asarray(physical[40:67],dtype=np.float64)
        self.lib.lh_set_state(self.h,qpos.ctypes.data_as(ct.POINTER(ct.c_double)),qvel.ctypes.data_as(ct.POINTER(ct.c_double)));World.forwards+=1
        assert self.dim(6)==0
        return qpos,qvel
    def observation(self):
        xyz=self.arr(3,(17,3));frame=self.arr(5,(17,3,3))[self.ids['torso']];origin=xyz[self.ids['torso']]
        extremities=np.concatenate([(xyz[self.ids[name]]-origin)@frame for name in ['left_hand','left_foot','right_hand','right_foot']])
        return np.r_[self.arr(0,(28,))[7:],xyz[self.ids['head'],2],extremities,frame[2,:],self.arr(7,(17,3))[self.ids['torso']],self.arr(1,(27,))]
    def contacts(self):
        rows=[];buf=np.zeros(8,dtype=np.float64)
        for i in range(self.dim(7)):
            assert self.lib.lh_contact(self.h,i,buf.ctypes.data_as(ct.POINTER(ct.c_double)))==0
            rows.append({'geom1':int(buf[0]),'geom2':int(buf[1]),'body1':self.names[int(buf[2])],'body2':self.names[int(buf[3])],'distanceM':float(buf[4]),'positionSource':buf[5:8].tolist()})
        return rows
    def snapshot(self,step,control):
        xyz=self.arr(3,(17,3));quats=self.arr(4,(17,4));contacts=self.contacts()
        selfdepth=max([0.]+[-p['distanceM'] for p in contacts if 'world' not in [p['body1'],p['body2']]])
        pairdepth=max([0.]+[-p['distanceM'] for p in contacts if {p['body1'],p['body2']}=={'left_thigh','left_foot'}])
        obs=self.observation()
        assert np.isfinite(obs).all() and self.lib.lh_warnings(self.h)==0
        return {'nativeStep':step,'timeS':self.lib.lh_time(self.h),'qpos':self.arr(0,(28,)).copy().tolist(),'qvel':self.arr(1,(27,)).copy().tolist(),'observation':obs.tolist(),'control':np.asarray(control).tolist(),'headHeightM':float(obs[21]),'comSource':self.arr(6,(17,3))[self.ids['torso']].copy().tolist(),'maximumSelfPenetrationM':selfdepth,'leftThighFootPenetrationM':pairdepth,'selfContactQualificationGateExceeded':selfdepth>.005,'contacts':contacts,'bodyPosesSource':{name:{'position':xyz[i].tolist(),'quaternionWxyz':quats[i].tolist()} for i,name in enumerate(self.names) if i},'qfrcActuator':self.arr(8,(27,)).copy().tolist(),'qfrcPassive':self.arr(9,(27,)).copy().tolist()}

def prepare(infer,reference):
    model=json.loads(MODEL.read_text());parts={p['id']:p for p in model['segments']}
    w=World(OUT/'source-model/humanoid_high_freq.xml');rows=[]
    try:
        assert abs(w.lib.lh_dt(w.h)-.00125)<1e-12
        bodymass=w.arr(10,(17,));ipos=w.arr(11,(17,3));inertia=w.arr(12,(17,3));iquat=w.arr(13,(17,4));masserr=inertiaerr=0.
        for name in model['referenceMetadata']['sourceBodyOrder']:
            i=w.ids[name];p=parts[name];masserr=max(masserr,abs(bodymass[i]-p['massKg']))
            R=Rotation.from_quat(iquat[i][[1,2,3,0]]).as_matrix();tensor=C@R@np.diag(inertia[i])@R.T@C.T
            inertiaerr=max(inertiaerr,float(np.max(np.abs(tensor-np.array(p['goldenMass']['inertia'])))))
        assert masserr<1e-6 and inertiaerr<1e-6,(masserr,inertiaerr)
        for i,name in enumerate(model['referenceMetadata']['actuatorOrder']):assert w.lib.lh_name(w.h,18,i).decode()==name
        assert np.array_equal(w.arr(14,(21,6))[:,0],np.array([next(p['reference']['hinge']['gear'] for p in parts.values() if p['reference']['hinge'] and p['reference']['hinge']['actuatorIndex']==i) for i in range(21)]))
        for f in reference['teacher_frames']:
            physical=np.array(f['observation']);q,dq=w.state(f['before'],physical);obs=w.observation();poses={p['id']:p for p in f['before']};xyz=w.arr(3,(17,3));quats=w.arr(4,(17,4));position=angle=0.
            for name in model['referenceMetadata']['sourceBodyOrder']:
                i=w.ids[name];position=max(position,float(np.linalg.norm(C@xyz[i]-vec(poses[name]['position']))));R=C@Rotation.from_quat(quats[i][[1,2,3,0]]).as_matrix()@C.T;angle=max(angle,float(Rotation.from_matrix(R.T@rot(poses[name]['rotation']).as_matrix()).magnitude()))
            errors=[float(np.max(np.abs(obs[a:b]-physical[a:b]))) for a,b in [(0,21),(21,37),(37,40),(40,67)]]
            actionerror=float(np.max(np.abs(infer(obs)-np.array(f['action']))))
            row={'archivedTimeS':f['time_s'],'maximumBodyPositionErrorM':position,'maximumBodyAngleErrorRad':angle,'observationBlockErrorsQGeometryComVelocityGeneralizedVelocity':errors,'maximumActionErrorFromMatchedMujocoState':actionerror,'maximumActionErrorOnExactArchivedObservation':float(np.max(np.abs(infer(physical)-np.array(f['action'])))),'leftThighFootSourceDepthAtArchivedRapierPoseM':max([0.]+[-c['distanceM'] for c in w.contacts() if {c['body1'],c['body2']}=={'left_thigh','left_foot'}])};rows.append(row)
            assert position<.0002 and angle<2e-6 and max(errors)<.0003 and errors[2]<.0001 and errors[3]<1e-6,row
        # Inspect the final refused Rapier geometry without integrating it.
        last=reference['teacher_frames'][-1];physical=np.array(last['observation']);physical[:21]=last['target'][:21]
        w.state(last['snapshot']['segments'],physical)
        finalcontacts=w.contacts();depth=max([0.]+[-c['distanceM'] for c in finalcontacts if {c['body1'],c['body2']}=={'left_thigh','left_foot'}])
        result={'passed':True,'engineVersion':213,'physicsIntegrations':0,'staticForwardCalls':World.forwards,'massMaximumErrorKg':masserr,'inertiaMaximumTensorErrorKgM2':inertiaerr,'sourceMassKg':float(bodymass.sum()),'rows':rows,'sourceDepthAtFinalArchivedRapierConfigurationM':depth,'nativeReportedDepthM':reference['teacher_failure']['measured'],'stateMatchingLimit':'Normalized free-root quaternion and generalized q/qdot match; native per-body quaternion drift yields <=0.2mm FK tolerance. Solver/contact warm-start state cannot be transferred across engines.','weightsOnly':True}
        before=w.observation().copy()
        assert w.lib.lh_step(w.h,0)==-1 and w.dim(6)==0
        assert np.array_equal(w.observation(),before),'Exhausted step bound changed state'
        result['zeroBudgetStepRejectedWithoutIntegration']=True
        source=World(OUT/'source-model/humanoid_static.xml')
        try:
            source.state(reference['teacher_initial']['segments'],reference['teacher_frames'][0]['observation'])
            assert abs(source.lib.lh_dt(source.h)-.005)<1e-12 and source.dim(6)==0
            result['source200ClockVerifiedWithoutIntegration']=True
        finally:source.close()
        result['staticForwardCalls']=World.forwards
        write(OUT/'preflight.json',result);return result
    finally:w.close()

def run(infer,reference):
    ledger=json.loads(LEDGER.read_text());assert ledger['status']=='reserved'
    assert ledger['nativeChargedAfter']==ledger['nativeChargedBefore']+540<=300000
    assert ledger['wallChargedAfterSeconds']==ledger['wallChargedBeforeSeconds']+120<=7200
    prior=ROOT/ledger['priorClosedRecord'];assert sha(prior)==ledger['priorClosedRecordSha256']
    for p,h in ledger['inputs'].items():assert sha(ROOT/p)==h,p
    assert json.loads((OUT/'preflight.json').read_text())['passed']
    (OUT/'started-once').open('x').close();started=PROCESS_STARTED
    signal.signal(signal.SIGALRM,lambda *_:(_ for _ in ()).throw(TimeoutError('120s diagnostic bound')));signal.alarm(max(1,int(115-(time.monotonic()-started))))
    result={'startedUtc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'nativeSteps':0,'histories':[],'qualification':False,'sourceCoordinateSystem':'source +Z up, root quaternion wxyz','maximumNativeSteps':540,'maximumWallSeconds':120}
    try:
        for case in ledger['histories']:
            w=World(OUT/'source-model'/case['model']);n=case['maximumSteps'];dt=case['nativeStepS'];repeat=round(.025/dt);assert abs(w.lib.lh_dt(w.h)-dt)<1e-12
            folder=OUT/case['id'];folder.mkdir();first=reference['teacher_frames'][0];w.state(reference['teacher_initial']['segments'],first['observation']);commands=[];control=np.zeros(21);rows=[]
            try:
                with (folder/'states.ndjson').open('x') as stream:
                    row=w.snapshot(0,control);rows.append(row);stream.write(json.dumps(row)+'\n')
                    for step in range(n):
                        assert result['nativeSteps']<540 and time.monotonic()-started<115
                        if step%repeat==0:
                            index=step//repeat;obs=w.observation();predicted=infer(obs);native=np.array(reference['teacher_frames'][index]['action']);control=native if case['mode']=='exact-archived-Rapier-controls' else predicted
                            commands.append({'decision':index,'timeS':step*dt,'observation':obs.tolist(),'predictedAction':predicted.tolist(),'appliedAction':control.tolist(),'archivedRapierAction':native.tolist(),'maximumActionDifferenceFromRapier':float(np.max(np.abs(control-native)))})
                        w.arr(2,(21,))[:]=control
                        result['nativeSteps']+=1;write(OUT/'result.json',result)
                        assert w.lib.lh_step(w.h,n)==step+1
                        row=w.snapshot(step+1,control);assert abs(row['timeS']-(step+1)*dt)<1e-10;rows.append(row);stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush()
                write(folder/'commands.json',commands)
                crossing=next((r for r in rows if r['selfContactQualificationGateExceeded']),None)
                comparison=rows[round(.29/dt)]
                summary={'id':case['id'],'steps':n,'nativeStepS':dt,'policyDecisions':len(commands),'firstGateExceedanceTimeS':None if crossing is None else crossing['timeS'],'maximumSelfPenetrationM':max(r['maximumSelfPenetrationM'] for r in rows),'maximumLeftThighFootPenetrationM':max(r['leftThighFootPenetrationM'] for r in rows),'headHeightAt029M':comparison['headHeightM'],'leftThighFootDepthAt029M':comparison['leftThighFootPenetrationM'],'finalHeadHeightM':rows[-1]['headHeightM'],'maximumCommandDifferenceFromArchivedRapier':max(c['maximumActionDifferenceFromRapier'] for c in commands),'warnings':w.lib.lh_warnings(w.h),'worldCountedSteps':w.dim(6)}
                assert w.dim(6)==n;result['histories'].append(summary);write(OUT/'result.json',result)
            finally:w.close()
        assert result['nativeSteps']==540;result['completed']=True
    except BaseException as error:
        result['completed']=False;result['error']=repr(error);raise
    finally:
        signal.alarm(0);result['finishedUtc']=datetime.datetime.now(datetime.timezone.utc).isoformat();result['wallSeconds']=time.monotonic()-started;result['staticForwardCalls']=World.forwards;write(OUT/'result.json',result);print(json.dumps(result,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('mode',choices=['prepare','run']);args=parser.parse_args()
    ref=json.loads(ARCHIVE.read_text());infer=actor()
    if args.mode=='prepare':print(json.dumps(prepare(infer,ref),indent=2))
    else:run(infer,ref)
