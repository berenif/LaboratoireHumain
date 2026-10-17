#!/usr/bin/env python3
"""Contact-gated reference scheduler; only the ordinary motor QP actuates it."""
import numpy as np

def smooth(u):
    u=np.clip(u,0.,1.);return float(u*u*(3-2*u))

def arc(start,end,t,duration):
    u=np.clip(t/duration,0.,1.);delta=np.asarray(end)-start
    return start+smooth(u)*delta,6*u*(1-u)/duration*delta,(6-12*u)/duration**2*delta if 0<=t<duration else np.zeros(3)

class RecoveryStep:
    def __init__(self,initial_com,stance_com,ramp_s,kin,transfer_timeout_s=9.5):
        assert 9.5<=transfer_timeout_s<=12.;self.transfer_timeout_s=transfer_timeout_s
        self.initial_com=np.array(initial_com);self.stance_com=np.array(stance_com);self.ramp_s=ramp_s;self.kin=kin
        self.phase='transfer';self.phase_start=0.;self.unloaded=0;self.capture_run=0.;self.origin=None;self.rotation=None;self.delta=None;self.log=[];self.failed=None;self.return_start=None;self.stand_start=None;self.land_run=0;self.lift_clearance_seen=False
    def foot_task(self,position,velocity=None,acceleration=None):
        return dict(body='left_foot',position=np.asarray(position),rotation=self.rotation,velocity=np.zeros(3) if velocity is None else velocity,acceleration=np.zeros(3) if acceleration is None else acceleration)
    def change(self,phase,elapsed,**details):
        self.log.append(dict(event=phase,timeS=elapsed,**details));self.phase=phase;self.phase_start=elapsed
    def update(self,elapsed,state,model,diagnostic,last_samples,right_distance,lift_ready=True):
        pose=model['geometry'][0]['left_foot'];position=pose[:3,3];loads=state['metrics']['foot_loads_n'];clearance=state['metrics']['foot_clearance_m'][0]
        for sample in last_samples:
            self.unloaded=self.unloaded+1 if sample['loads_n'][0]<3. and sample['loads_n'][1]>=.8*self.kin.model['totalMassKg']*9.81 else 0
        self.capture_run=self.capture_run+.025 if right_distance<=.003 else 0.
        com=self.initial_com+smooth((elapsed-1.)/self.ramp_s)*(self.stance_com-self.initial_com)
        swing=None;allow_unload=self.phase=='transfer'
        if self.phase=='transfer':
            if loads[0]<3. and self.origin is None:
                self.origin=position.copy();self.rotation=pose[:3,:3]/np.cbrt(np.linalg.det(pose[:3,:3]));self.log.append(dict(event='measured-unload-foot-hold',timeS=elapsed))
            if self.origin is not None:swing=self.foot_task(self.origin)
            if self.unloaded>=80 and self.capture_run>=.1 and lift_ready:
                self.origin=position.copy();self.rotation=pose[:3,:3]/np.cbrt(np.linalg.det(pose[:3,:3]));capture=np.array([diagnostic['capture'][k] for k in 'xyz']);delta=capture-self.origin;delta[1]=0.;distance=np.linalg.norm(delta);self.delta=delta/max(distance,1e-12)*min(.06,distance)
                self.bottom=self.kin.bottom('left_foot',pose);self.high=self.origin+np.array([0.,.035,0.]);self.landing=self.origin+self.delta;self.landing[1]-=self.bottom+.001
                self.change('lift',elapsed,targetDisplacementM=float(np.linalg.norm(self.delta)),captureDirected=True);com=self.stance_com.copy();swing=self.foot_task(self.origin)
            elif elapsed>=self.transfer_timeout_s:
                self.failed='no-qualified-unload';self.change('stop',elapsed)
        else:com=self.stance_com.copy()
        dt=elapsed-self.phase_start
        if self.phase=='lift':
            p,v,a=arc(self.origin,self.high,dt,.35);swing=self.foot_task(p,v,a)
            native_clear=bool(last_samples and all(row['clearance_m'][0]>=.015 and row['loads_n'][0]<3. for row in last_samples))
            if dt>=.35 and native_clear:
                self.lift_clearance_seen=True;self.change('translate',elapsed)
            elif dt>=.85:
                self.failed='clearance-not-reached';self.landing=position.copy();self.landing[1]-=self.kin.bottom('left_foot',pose)+.001;self.lower_origin=position.copy();self.change('land',elapsed)
        elif self.phase=='translate':
            native_clear=bool(last_samples and all(row['clearance_m'][0]>=.015 and row['loads_n'][0]<3. for row in last_samples))
            # Never advance a horizontal target through measured recontact.
            if not native_clear:
                self.failed='clearance-lost-during-placement';self.landing=position.copy();self.landing[1]-=self.kin.bottom('left_foot',pose)+.001;self.lower_origin=position.copy();self.change('land',elapsed);swing=self.foot_task(position)
            else:
                p,v,a=arc(self.high,self.high+self.delta,dt,.4);swing=self.foot_task(p,v,a)
                if dt>=.4 and np.linalg.norm((position-(self.high+self.delta))[[0,2]])<=.005:
                    self.lower_origin=self.high+self.delta;self.change('land',elapsed)
                elif dt>=.9:
                    self.failed='placement-not-reached';self.landing=position.copy();self.landing[1]-=self.kin.bottom('left_foot',pose)+.001;self.lower_origin=position.copy();self.change('land',elapsed)
        elif self.phase=='land':
            p,v,a=arc(self.lower_origin,self.landing,dt,.35);swing=self.foot_task(p,v,a)
            for row in last_samples:self.land_run=self.land_run+1 if row['loads_n'][0]>=3. and row['loads_n'][1]>=.5*self.kin.model['totalMassKg']*9.81 and row['clearance_m'][0]<=.003 else 0
            if dt>=.35 and self.land_run>=80:
                self.return_origin=self.initial_com+.5*(self.landing-self.origin);self.return_origin[1]=self.initial_com[1];self.change('return',elapsed,measuredLanding=True)
            elif dt>=1.2:self.failed='landing-load-not-restored'
        elif self.phase=='return':
            com=self.stance_com+smooth(dt/3.)*(self.return_origin-self.stance_com)
            if self.origin is not None:swing=self.foot_task(self.landing if self.delta is not None else self.origin)
            if dt>=3.:
                self.change('stand',elapsed);self.stand_start=elapsed
        elif self.phase=='stand':
            com=self.return_origin.copy()
            if self.origin is not None:swing=self.foot_task(self.landing if self.delta is not None else self.origin)
        return com,swing,allow_unload


class TerminalUnloadGoal:
    """Bounded requested load transfer after actual stance support is qualified.

    This constrains only mathematical contact-force allocation. Native measured
    unloading still gates every physical swing; it never lifts a loaded foot.
    """
    def __init__(self,duration=.5):
        assert duration>=.5
        self.duration=duration;self.anchor=None;self.elapsed=0.;self.failed=False
    def update(self,qualified,measured_left_load_n,dt=.025):
        if self.failed:return None
        if not qualified:
            if self.anchor is not None:self.failed=True
            return None
        if self.anchor is None:
            self.anchor=max(0.,float(measured_left_load_n));self.elapsed=0.
        else:self.elapsed=min(self.duration,self.elapsed+dt)
        return self.anchor*(1.-smooth(self.elapsed/self.duration))
