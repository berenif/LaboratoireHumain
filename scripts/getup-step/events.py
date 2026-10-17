"""Pure measured-event state machine. Target time cannot fabricate a contact.

Preparation reference time must not be frozen merely because its future3mm
lift-admission inset is not attained yet. Those counters only admit the lift.
Inputs require per-native real support margins. A missing geometry measurement
rejects the packet; a prior hull or prospective contact is never substituted.
This is scaffolding, not a task collection runner or a success certificate.
"""
from dataclasses import dataclass,field,asdict
import math
import numpy as np
from task import load_config

PHASES=['prepare','lift','advance','lower','acquire','settle','hold']

@dataclass
class MeasuredEvents:
    native_id:int
    body_weight_n:float
    heading:list
    entry_left:list
    phase:str='prepare'
    unload_run:int=0
    clearance_run:int=0
    touchdown_run:int=0
    load_run:int=0
    quiet_run:int=0
    awarded:list=field(default_factory=list)
    hold_anchors:list|None=None

    def __post_init__(self):
        if type(self.native_id)is not int or self.native_id<0 or self.phase not in PHASES:raise ValueError('invalid event state')
        if not math.isfinite(self.body_weight_n)or self.body_weight_n<=0:raise ValueError('invalid body weight')
        if np.shape(self.heading)!=(3,)or np.shape(self.entry_left)!=(3,)or not np.isfinite([self.heading,self.entry_left]).all()or abs(np.linalg.norm(self.heading)-1)>1e-8 or abs(self.heading[1])>1e-8:raise ValueError('invalid frozen entry geometry')
        order=['unloaded','clearance','forward','touchdown','loadAcquired','quietComplete']
        expected={'prepare':0,'lift':1,'advance':2,'lower':3,'acquire':4,'settle':5,'hold':5}[self.phase]
        if self.awarded!=order[:len(self.awarded)]or len(self.awarded)not in ([5,6]if self.phase=='hold'else [expected]):raise ValueError('event prefix/phase mismatch')
        for run in [self.unload_run,self.clearance_run,self.touchdown_run,self.load_run,self.quiet_run]:
            if type(run)is not int or run<0:raise ValueError('invalid event timer')

    def observe(self,samples,*,phase_reference_complete,actual_loaded_slip_m,stopping_clearance_ready=False):
        self.__post_init__();cfg=load_config()[0];gate=cfg['phaseGates'];events=[];required=gate['persistentNativeSamples']
        if not isinstance(phase_reference_complete,bool)or not isinstance(stopping_clearance_ready,bool):raise ValueError('invalid phase completion flags')
        if np.shape(actual_loaded_slip_m)!=(2,)or not np.isfinite(actual_loaded_slip_m).all()or min(actual_loaded_slip_m)<0:raise ValueError('invalid actual native cumulative slip')
        # Validate the whole packet BEFORE mutation; reject missing native proof.
        for offset,s in enumerate(samples,1):
            if type(s.get('native_substep'))is not int or s['native_substep']!=self.native_id+offset:raise ValueError('missing/duplicate/nonconsecutive event native IDs')
            for name in ['loads_n','clearance_m','positions']:
                if name not in s:raise ValueError('missing measured '+name)
            for prefix in ['right','combined']:
                dimension=s.get(prefix+'_support_dimension');margins=[s.get(prefix+'_'+q+'_margin_m')for q in ['com','capture']]
                if type(dimension)is not int or dimension not in [-1,0,1,2]:raise ValueError('missing/invalid measured support dimension')
                if dimension==2 and not all(type(m)in(int,float)and math.isfinite(m)for m in margins):raise ValueError('polygon needs finite measured margins')
                if dimension<2 and not all(m is None for m in margins):raise ValueError('point/segment cannot claim polygon margins')
            for name in ['torso_up','com_height_m','right_com_margin_m','right_capture_margin_m','combined_com_margin_m','combined_capture_margin_m']:
                if name not in s:raise ValueError('missing measured '+name)
                if s[name]is None and name.endswith('_margin_m'):
                    prefix=name.split('_')[0]
                    if s.get(prefix+'_support_dimension')not in [-1,0,1]:raise ValueError('undefined margin without measured lower-dimensional support')
                elif type(s[name])not in(int,float)or not math.isfinite(s[name]):raise ValueError('nonfinite measured '+name)
            if np.shape(s['loads_n'])!=(2,)or np.shape(s['clearance_m'])!=(2,):raise ValueError('invalid foot metrics')
            if not np.isfinite([s['loads_n'],s['clearance_m']]).all()or min(s['loads_n'])<0:raise ValueError('invalid measured loads/clearances')
            for name in ['nonfoot_support','standing_without_foot_anchor']:
                if type(s.get(name))is not bool:raise ValueError('missing native predicate')
            if len(s['positions'])!=2 or any(not all(k in p and math.isfinite(p[k])for k in 'xyz')for p in s['positions']):raise ValueError('invalid measured positions')
        if not samples:return dict(events=[],phase=self.phase,physicalFailure=None,transitioned=False)
        failure=None
        if max(actual_loaded_slip_m)>cfg['termination']['loadedSlipMaximumM']:failure='loaded_slip'
        for s in samples:
            left,right=s['loads_n'];total=left+right
            if left<3 and right<3:failure=failure or'flight'
            if s['nonfoot_support']:failure=failure or'nonfoot_support'
            if not cfg['termination']['comHeightMinimumM']<=s['com_height_m']<=cfg['termination']['comHeightMaximumM']:failure=failure or'com_height'
            if self.phase=='advance'and s['clearance_m'][0]<gate['nativeAdvanceClearanceMinimumM']:failure=failure or'advance_clearance'
            support=right>=.8*self.body_weight_n and s['right_com_margin_m']is not None and s['right_capture_margin_m']is not None and s['right_com_margin_m']>=.003 and s['right_capture_margin_m']>=.003
            unloaded=left<3 and support and not s['nonfoot_support']
            self.unload_run=self.unload_run+1 if unloaded else 0
            self.clearance_run=self.clearance_run+1 if s['clearance_m'][0]>=.015 and support else 0
            touchdown=left>=3 and right>=.5*self.body_weight_n and total>=.8*self.body_weight_n and not s['nonfoot_support']
            self.touchdown_run=self.touchdown_run+1 if touchdown else 0
            acquired=touchdown and left>=.1*self.body_weight_n and s['combined_com_margin_m']is not None and s['combined_capture_margin_m']is not None and s['combined_com_margin_m']>=.003 and s['combined_capture_margin_m']>=.003
            self.load_run=self.load_run+1 if acquired else 0
            if self.phase=='hold':
                if self.hold_anchors is None:raise ValueError('hold requires fixed measured anchors')
                drift=max(np.linalg.norm(np.array([p[k]for k in 'xyz'])-a)for p,a in zip(s['positions'],self.hold_anchors))
                quiet=s['standing_without_foot_anchor']and s['torso_up']>=math.cos(math.radians(5))and drift<=.01
                self.quiet_run=self.quiet_run+1 if quiet else 0
            self.native_id=s['native_substep']
        old_phase=self.phase;last=samples[-1];left=np.array([last['positions'][0][k]for k in 'xyz']);delta=left-self.entry_left;forward=float(delta@np.array(self.heading));lateral=float(delta@np.cross(np.array([0.,1.,0.]),self.heading))
        if not failure:
            if self.phase=='prepare'and phase_reference_complete and self.unload_run>=required:self.phase='lift';events=['unloaded']
            elif self.phase=='lift'and phase_reference_complete and self.clearance_run>=required and stopping_clearance_ready:self.phase='advance';events=['clearance']
            elif self.phase=='advance'and phase_reference_complete and forward>=.05 and abs(lateral)<=.02:self.phase='lower';events=['forward']
            elif self.phase=='lower'and phase_reference_complete and self.touchdown_run>=required:self.phase='acquire';events=['touchdown']
            elif self.phase=='acquire'and phase_reference_complete and self.load_run>=required:self.phase='settle';events=['loadAcquired']
            elif self.phase=='settle'and phase_reference_complete and last['standing_without_foot_anchor']and last['torso_up']>=math.cos(math.radians(5)):
                self.phase='hold';self.hold_anchors=[[p[k]for k in 'xyz']for p in last['positions']];self.quiet_run=0
            elif self.phase=='hold'and self.quiet_run>=4000 and'quietComplete'not in self.awarded:events=['quietComplete']
            self.awarded.extend(events)
        return dict(events=events,phase=self.phase,physicalFailure=failure,transitioned=self.phase!=old_phase,signedForwardM=forward,lateralM=lateral,quietConsecutiveSamples=self.quiet_run)

    def state(self):return asdict(self)
