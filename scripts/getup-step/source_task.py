"""Source-engine stabilization environment core. Collection and learning stay OFF.

The actual SLOW endpoint is a training-only reset candidate. This module never
calls an engine or installs a planned pose. A future admitted driver supplies
counted direct-torque integrations; consume() also works on recorded packets for
measurement-only checks. Recorded SLOW controls are NOT torque-teacher replay.
Only combined actual support is available in bridge v2: no right hull is invented.
"""
from dataclasses import asdict
import copy, hashlib, json, math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from audit_metrics import fk, support_margin, task_errors
from reference import PhaseClock
from source_math import P, ORDER, conditioned_teacher_observation
from task import TaskHistory, load_config

CONFIG_PATH=Path(__file__).with_name('task-config-mujoco-stabilization-v1.json')


def _vector(value,width,name):
    x=np.asarray(value,dtype=float)
    if x.shape!=(width,) or not np.isfinite(x).all():raise ValueError('invalid '+name)
    return x


def capsule_separation(poses,parts):
    def ends(name,c):return[(poses[name]@np.r_[fk.xyz(c[e]),1])[:3]for e in ['a','b']]
    minimum=math.inf
    for l in parts['left_foot']['reference']['primitives']:
        a,b=ends('left_foot',l);d=b-a
        for r in parts['right_foot']['reference']['primitives']:
            c,e=ends('right_foot',r);v=e-c;offset=a-c;candidates=[]
            for t in [0.,1.]:
                u=float(np.clip(v@(offset+t*d)/(v@v),0,1))if v@v>0 else 0.;candidates.append((t,u))
            for u in [0.,1.]:
                t=float(np.clip(-d@(offset-u*v)/(d@d),0,1))if d@d>0 else 0.;candidates.append((t,u))
            matrix=np.column_stack((d,-v));normal=matrix.T@matrix
            if np.linalg.det(normal)>1e-18:
                t,u=np.linalg.solve(normal,-matrix.T@offset)
                if 0<=t<=1 and 0<=u<=1:candidates.append((t,u))
            minimum=min(minimum,min(float(np.linalg.norm(offset+t*d-u*v))-l['radius']-r['radius']for t,u in candidates))
    return minimum

def decode_source_frame(row,model):
    """Post-integration q/velocity; previous solved loads/transported anchors.

    No stale geometry contacts are used to manufacture support. World coordinates
    returned for generic task errors use native Y-up; source targets remain Z-up.
    """
    json.dumps(row,allow_nan=False)
    if type(row.get('timeS'))not in(int,float)or not math.isfinite(row['timeS'])or row['timeS']<0:raise ValueError('invalid source engine time')
    if type(row.get('sourceStep'))is not int or row['sourceStep']<0:raise ValueError('invalid source integration ID')
    q=_vector(row['qpos'],28,'source qpos');v=_vector(row['qvel'],27,'source qvel');physical=_vector(row['physicalObservation'],67,'physical67');com=_vector(row['comSource'],3,'source COM')
    if abs(np.linalg.norm(q[3:7])-1)>1e-6 or np.max(abs(q[7:]-physical[:21]))>1e-10 or np.max(abs(v-physical[40:67]))>1e-10:raise ValueError('source pose/observer mismatch')
    kin=fk.Kinematics(model);root=np.eye(4);root[:3,3]=P.T@q[:3];root[:3,:3]=P.T@Rotation.from_quat(q[[4,5,6,3]]).as_matrix()@P
    poses=kin.forward(q[7:],root)
    geometry=np.r_[poses['head'][1,3],np.concatenate([P@root[:3,:3].T@(poses[name][:3,3]-root[:3,3])for name in ['left_hand','left_foot','right_hand','right_foot']]),P@root[:3,:3].T@np.array([0.,1.,0.])]
    if np.max(abs(geometry-physical[21:37]))>2e-6:raise ValueError('source physical geometry mismatch')
    parts={p['id']:p for p in model['segments']};mass=sum(p['massKg']for p in parts.values())
    true_com=sum(p['massKg']*(poses[name]@np.r_[fk.xyz(p['goldenMass']['centerOfMass']),1])[:3]for name,p in parts.items())/mass
    if np.max(abs(P@true_com-com))>2e-6:raise ValueError('source/FK COM mismatch')
    loads=_vector(row['loadsN'],2,'foot loads');all_loads=_vector(row['allBodyFloorLoadsN'],17,'body loads');qfrc=_vector(row['qfrcActuator'],27,'actual actuator force');speed=_vector(row['loadedSlipSpeedMps'],2,'actual loaded slip speed')
    if min(loads)<0 or min(all_loads)<0 or min(speed)<0:raise ValueError('negative source force/slip')
    order=['world',*model['referenceMetadata']['sourceBodyOrder']];feet=[order.index(x)for x in ['left_foot','right_foot']]
    if np.max(abs(all_loads[feet]-loads))>1e-8:raise ValueError('foot/body load mismatch')
    nonfoot=bool(any(x>=3 for i,x in enumerate(all_loads)if i not in [0,*feet]))
    if type(row.get('nonfootSupport'))is not bool or row['nonfootSupport']!=nonfoot:raise ValueError('source support predicate mismatch')
    measured=row.get('sourceQuiet',{})
    points=np.asarray(measured['solvedPositiveForcePointsCurrentXY'],dtype=float).reshape(-1,2)
    linear=measured['maximumBodyLinearMps'];angular=measured['maximumBodyAngularRadps']
    for name,x in [('linear speed',linear),('angular speed',angular),('penetration',row['maximumSelfPenetrationM'])]:
        if type(x)not in(int,float)or not math.isfinite(x)or x<0:raise ValueError('invalid '+name)
    if type(row['sourceWarnings'])is not int or row['sourceWarnings']<0:raise ValueError('invalid source warning count')
    capture=com[:2]+physical[37:39]*math.sqrt(max(com[2],0)/9.81)
    margin,dimension=support_margin(points,com[:2]);capture_margin,other_dimension=support_margin(points,capture)
    if dimension!=other_dimension:raise ValueError('support dimension mismatch')
    gear=np.zeros(21)
    for p in parts.values():
        h=p['reference'].get('hinge')
        if h:gear[h['qposIndex']]=h['gear']
    controls=qfrc[6:][ORDER]/gear[ORDER]
    if np.max(abs(controls))>1.000001:raise ValueError('source original gear cap exceeded')
    clearances=[kin.bottom(name,poses[name])for name in ['left_foot','right_foot']]
    return dict(nativeId=row['sourceStep'],physical=physical,comWorld=P.T@com,comVelocityWorld=P.T@physical[37:40],transforms=poses,controls=np.clip(controls,-1,1),wholeFootClearanceM=float(clearances[0]),clearances=clearances,loads=loads,nonfoot=nonfoot,combinedDimension=dimension,combinedCOMMargin=margin,combinedCaptureMargin=capture_margin,maximumLinear=float(linear),maximumAngular=float(angular),interfootSeparationM=capsule_separation(poses,parts),penetration=float(row['maximumSelfPenetrationM']),warnings=row['sourceWarnings'],slipSpeed=speed,mass=mass)


def fixed_stabilization_target(measured):
    """Measured entry pose is a REFERENCE only, never a post-reset state write."""
    x=measured
    return dict(q=x['physical'][:21].copy(),qvel=np.zeros(27),com=P@x['comWorld'],com_ori=x['physical'][34:37].copy(),comVelocity=np.zeros(3),leftFoot=x['transforms']['left_foot'].copy(),clearance=x['wholeFootClearanceM'])


def validate_rng(state):
    if not isinstance(state,dict)or state.get('bit_generator')not in ['PCG64','PCG64DXSM','MT19937','Philox','SFC64']:raise ValueError('complete supported RNG state required')
    temporary=getattr(np.random,state['bit_generator'])();temporary.state=state


def _plain(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,dict):return{k:_plain(v)for k,v in value.items()}
    if isinstance(value,(list,tuple)):return[_plain(v)for v in value]
    if isinstance(value,np.generic):return value.item()
    return value


class SourceStabilization:
    """Actual-event/reward core with atomic packet consumption and hybrid resume.

    Stabilize phase advances in actual source time even before quiet is achieved.
    It has a fixed entry reference, no commanded foot movement, and no QP. A full
    step phasechain is deliberately not enabled by this stabilization config.
    """
    def __init__(self,initial_row,model,*,config_path=CONFIG_PATH):
        self.config,self.config_sha=load_config(config_path);self.model=model;self.entry_packet=copy.deepcopy(initial_row)
        if self.config['engine']['fullStepCurriculumEnabled']or self.config['timing']['nativeHz']!=200:raise ValueError('not the stabilization-only source preset')
        self.initial=decode_source_frame(initial_row,model);self.target=fixed_stabilization_target(self.initial);self.native_id=self.initial['nativeId'];self.start_id=self.native_id;self.start_time=float(initial_row['timeS'])
        self.clock=PhaseClock(0.,1.,self.native_id,sample_seconds=.005);self.history=TaskHistory(self.native_id);self.feet_run=[0,0];self.quiet_run=0;self.slip=np.zeros(2);self.previous_controls=self.initial['controls'].copy();self.anchors=np.array([self.initial['transforms'][name][:3,3]for name in ['left_foot','right_foot']]);self.terminal=False
        heading=self.initial['transforms']['torso'][:3,2].copy();heading[1]=0
        if np.linalg.norm(heading)<1e-8:raise ValueError('undefined entry forward heading')
        self.heading=heading/np.linalg.norm(heading);self.entry_left=self.anchors[0].copy();self.last_packet_hash=self.packet_hash(initial_row)
    @staticmethod
    def packet_hash(row):return hashlib.sha256(json.dumps(row,sort_keys=True,allow_nan=False,separators=(',',':')).encode()).hexdigest()
    def observation(self,physical,power=1.):
        # For a held stabilization reference the now and +125ms targets coincide.
        t=np.r_[self.target['q'],self.target['com'][2],self.target['com_ori']]
        return conditioned_teacher_observation(physical,t,t,power)
    def _failure_and_quiet(self,x):
        cfg=self.config;fail=None
        if x['warnings']or x['penetration']>.005:fail='native_integrity'
        elif max(x['loads'])<3:fail='flight'
        elif x['nonfoot']:fail='nonfoot_support'
        elif x['interfootSeparationM']<0:fail='interfoot_overlap'
        elif not cfg['termination']['comHeightMinimumM']<=x['comWorld'][1]<=cfg['termination']['comHeightMaximumM']:fail='com_height'
        elif max(self.slip)>cfg['termination']['loadedSlipMaximumM']:fail='loaded_slip'
        positions=np.array([x['transforms'][name][:3,3]for name in ['left_foot','right_foot']]);drift=float(np.max(np.linalg.norm(positions-self.anchors,axis=1)))
        if drift>.01:fail=fail or'fixed_anchor_drift'
        self.feet_run=[run+1 if load>=3 else 0 for run,load in zip(self.feet_run,x['loads'])]
        quiet=bool(not fail and x['physical'][21]>1.4 and x['physical'][36]>=math.cos(math.radians(5))and min(self.feet_run)>=self.config['phaseGates']['persistentNativeSamples']and sum(x['loads'])>=.8*x['mass']*9.81 and x['combinedDimension']==2 and min(x['combinedCOMMargin'],x['combinedCaptureMargin'])>=.003 and x['maximumLinear']<=.22 and x['maximumAngular']<=.65)
        self.quiet_run=self.quiet_run+1 if quiet else 0
        return fail,quiet,drift
    def consume(self,rows,*,extrinsic_cutoff=False,expected_teacher_controls=None):
        if self.terminal:raise ValueError('cannot step a terminal task')
        if type(extrinsic_cutoff)is not bool:raise ValueError('invalid collection cutoff')
        # Decode and validate every sample before changing ANY hybrid state.
        xs=[decode_source_frame(r,self.model)for r in rows];n=len(xs);maximum=self.config['timing']['nativeSamplesPerTransition']
        if not 1<=n<=maximum or [x['nativeId']for x in xs]!=list(range(self.native_id+1,self.native_id+1+n)):raise ValueError('requires consecutive new source integrations')
        if expected_teacher_controls is not None:
            expected=_vector(expected_teacher_controls,21,'held torque-teacher control')
            if np.max(abs(expected))>1 or any(np.max(abs(x['controls']-expected))>1e-6 for x in xs):raise ValueError('executed controls do not match held torque-teacher action')
        if any(abs(row['timeS']-(self.start_time+(x['nativeId']-self.start_id)*.005))>1e-9 for row,x in zip(rows,xs)):raise ValueError('source integration clock/timestep mismatch')
        backup=copy.deepcopy(self.__dict__)
        try:
            failure=None;conditions=[]
            for index,x in enumerate(xs):
                self.slip+=x['slipSpeed']*.005;f,quiet,drift=self._failure_and_quiet(x);conditions.append(dict(nativeId=x['nativeId'],quiet=quiet,anchorDriftM=drift))
                if f:
                    if index!=n-1:raise ValueError('driver continued after a physical terminal')
                    failure=f
            if n<maximum and failure is None:raise ValueError('incomplete nonterminal requests excluded')
            self.clock.advance(xs[-1]['nativeId'],True,self.config['timing']['maximumSuffixSeconds'])
            if not failure and self.quiet_run<self.config['phaseGates']['finalQuietConsecutiveNativeSamples']and (xs[-1]['nativeId']-self.start_id)*.005>=self.config['timing']['maximumSuffixSeconds']:failure='phase_timeout'
            events=['quietComplete']if self.quiet_run>=self.config['phaseGates']['finalQuietConsecutiveNativeSamples']and not failure else[]
            errors=task_errors(xs[-1],self.target,self.heading,self.entry_left)
            slip_increment=float(np.sum(self.slip-backup['slip']))
            result=self.history.evaluate([x['nativeId']for x in xs],errors,xs[-1]['controls'],self.previous_controls,slip_increment,confirmed_events=events,physical_failure=failure,truncated=extrinsic_cutoff,config=self.config)
            self.native_id=xs[-1]['nativeId'];self.previous_controls=xs[-1]['controls'].copy();self.last_packet_hash=self.packet_hash(rows[-1]);self.terminal=bool(result['terminated']or result['truncated'])
            result.update(phase='stabilize',referenceClock=self.clock.state(),quietConsecutiveSamples=self.quiet_run,loadedSlipM=self.slip.tolist(),nativeConditions=conditions,actualFinalObservation118=self.observation(xs[-1]['physical']).tolist(),taskDataCollectionEnabled=False,optimizerEnabled=False,measurementOnly=True,heldTorqueTeacherActionVerified=expected_teacher_controls is not None,replayEligible=False)
            json.dumps(_plain(result),allow_nan=False);return _plain(result)
        except BaseException:self.__dict__.clear();self.__dict__.update(backup);raise
    def terminal_in_prefix(self,rows):
        """Nonmutating per-native safety preview for the counted engine driver."""
        trial=copy.deepcopy(self)
        for offset,row in enumerate(rows,1):
            x=decode_source_frame(row,self.model)
            if x['nativeId']!=self.native_id+offset:raise ValueError('nonconsecutive preview')
            trial.slip+=x['slipSpeed']*.005
            failure,_,_=trial._failure_and_quiet(x)
            if failure:return failure
        return None
    def checkpoint(self,native_blob,bindings,rng_state):
        if not isinstance(native_blob,bytes)or not native_blob or not isinstance(bindings,dict)or not bindings:raise ValueError('native cache and fingerprints required')
        state={k:_plain(getattr(self,k))for k in ['native_id','start_id','start_time','feet_run','quiet_run','slip','previous_controls','anchors','terminal','heading','entry_left','last_packet_hash','target']}
        state.update(clock=self.clock.state(),history=asdict(self.history))
        side=dict(schema=1,entryPacket=self.entry_packet,entryPacketSHA256=self.packet_hash(self.entry_packet),configSHA256=self.config_sha,nativeSHA256=hashlib.sha256(native_blob).hexdigest(),bindings=bindings,rng=_plain(rng_state),state=state,noLedgerRefund=True,scope='stabilization-only hybrid task; engine dynamics parity still requires admitted paired continuation')
        validate_rng(rng_state)
        side['controllerSHA256']=hashlib.sha256(json.dumps(side,sort_keys=True,allow_nan=False,separators=(',',':')).encode()).hexdigest()
        return side
    def restore(self,side,native_blob,actual_restored_row,*,expected_bindings):
        unsigned={k:v for k,v in side.items()if k!='controllerSHA256'}
        if side.get('controllerSHA256')!=hashlib.sha256(json.dumps(unsigned,sort_keys=True,allow_nan=False,separators=(',',':')).encode()).hexdigest():raise ValueError('controller sidecar checksum mismatch')
        if side.get('schema')!=1 or side.get('configSHA256')!=self.config_sha or side.get('bindings')!=expected_bindings or side.get('nativeSHA256')!=hashlib.sha256(native_blob).hexdigest():raise ValueError('hybrid native/config/code binding mismatch')
        if side.get('entryPacketSHA256')!=self.packet_hash(self.entry_packet)or self.packet_hash(side['entryPacket'])!=side['entryPacketSHA256']:raise ValueError('original episode entry packet mismatch')
        s=side['state'];actual=decode_source_frame(actual_restored_row,self.model)
        if actual['nativeId']!=s['native_id']or self.packet_hash(actual_restored_row)!=s['last_packet_hash']:raise ValueError('native/controller restored state mismatch')
        clock=PhaseClock(**s['clock']);history=TaskHistory(**s['history'])
        if clock.last_native_substep!=actual['nativeId']or history.last_native_id!=actual['nativeId']:raise ValueError('hybrid clock/history mismatch')
        trial=copy.deepcopy(self)
        for k in ['native_id','start_id','start_time','feet_run','quiet_run','terminal','last_packet_hash']:setattr(trial,k,s[k])
        for k in ['slip','previous_controls','anchors','heading','entry_left']:setattr(trial,k,np.asarray(s[k],dtype=float))
        trial.target={k:np.asarray(v,dtype=float)if isinstance(v,list)else v for k,v in s['target'].items()};trial.clock=clock;trial.history=history
        elapsed=s['native_id']-s['start_id']
        if any(type(s[k])is not int or s[k]<0 for k in ['native_id','start_id','quiet_run'])or type(s['terminal'])is not bool or elapsed<0:raise ValueError('invalid restored task counters')
        if s['start_id']!=self.start_id or s['start_time']!=self.start_time or clock.sample_seconds!=.005 or clock.rate!=1 or clock.target_rate!=1 or abs(clock.tau-min(elapsed*.005,self.config['timing']['maximumSuffixSeconds']))>1e-10:raise ValueError('restored source phase clock mismatch')
        if not isinstance(s['feet_run'],list)or len(s['feet_run'])!=2 or any(type(v)is not int or not 0<=v<=elapsed for v in s['feet_run'])or s['quiet_run']>elapsed:raise ValueError('invented restored persistence')
        for k,shape in [('slip',(2,)),('previous_controls',(21,)),('anchors',(2,3)),('heading',(3,)),('entry_left',(3,))]:
            if np.shape(getattr(trial,k))!=shape or not np.isfinite(getattr(trial,k)).all():raise ValueError('invalid restored geometry/control')
        if min(trial.slip)<0 or np.max(abs(trial.previous_controls))>1 or np.max(abs(trial.previous_controls-actual['controls']))>1e-6:raise ValueError('invalid restored slip/control boundary')
        for k in ['anchors','heading','entry_left']:
            if not np.array_equal(getattr(trial,k),getattr(self,k)):raise ValueError('resume cannot re-anchor episode')
        if set(trial.target)!=set(self.target)or any(not np.array_equal(trial.target[k],self.target[k])for k in self.target):raise ValueError('restored episode reference changed')
        goal=history.awarded_events==['quietComplete']
        if history.awarded_events not in [[],['quietComplete']]or goal!=(trial.quiet_run>=self.config['phaseGates']['finalQuietConsecutiveNativeSamples'])or goal and not trial.terminal:raise ValueError('restored terminal/event mismatch')
        validate_rng(side['rng'])
        json.dumps(trial.checkpoint(native_blob,expected_bindings,side['rng']),allow_nan=False)
        self.__dict__.clear();self.__dict__.update(trial.__dict__)
        return copy.deepcopy(side['rng'])
