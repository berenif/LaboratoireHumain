"""Reference data/timing only; never installs planned states into physics."""
from dataclasses import dataclass, asdict
import math
import numpy as np
from scipy.interpolate import interp1d
from source_math import NATIVE_DT, POLICY_DT, P, target


def source_resample_columns(values, speed):
    """Pinned source cubic, uniform teacher frames, ceil(N/speed) output rows.

    This does not normalize quaternions/vertical or differentiate qpos. Source
    qvel is independently interpolated by this function, then multiplied by speed.
    """
    x=np.asarray(values,dtype=np.float64)
    if x.ndim!=2 or len(x)<4 or not np.isfinite(x).all() or not math.isfinite(speed) or not 0<speed<=1:
        raise ValueError('source cubic requires >=4 uniform finite rows and speed in (0,1]')
    old=np.arange(len(x),dtype=float)
    return interp1d(old,x,kind='cubic',axis=0)(np.linspace(0,len(x)-1,math.ceil(len(x)/speed)))


def source_retime(trajectory,speed):
    result={k:source_resample_columns(trajectory[k],speed)for k in ['qpos','qvel','com','com_ori']}
    result['qvel']*=speed
    return result


@dataclass
class PhaseClock:
    tau:float
    rate:float
    last_native_substep:int
    target_rate:float=1.
    response_seconds:float=.1

    def __post_init__(self):
        if not all(math.isfinite(x)for x in [self.tau,self.rate,self.target_rate,self.response_seconds]) or self.tau<0 or not 0<=self.rate<=1 or self.target_rate not in [0.,1.] or self.response_seconds!=.1:
            raise ValueError('invalid/restored reference clock scalars')
        if not isinstance(self.last_native_substep,(int,np.integer))or self.last_native_substep<0:
            raise ValueError('native clock must be a nonnegative integer')

    def advance(self,native_substep,admitted,phase_end):
        self.__post_init__()
        if not isinstance(native_substep,(int,np.integer))or native_substep<self.last_native_substep or not math.isfinite(phase_end)or phase_end<self.tau or not isinstance(admitted,(bool,np.bool_)):
            raise ValueError('invalid/restored reference clock')
        dt=(native_substep-self.last_native_substep)*NATIVE_DT
        old_target=self.target_rate
        decay=math.exp(-dt/self.response_seconds)
        increment=old_target*dt+(self.rate-old_target)*self.response_seconds*(1-decay)
        self.tau=min(phase_end,self.tau+increment)
        self.rate=old_target+(self.rate-old_target)*decay
        self.target_rate=1. if admitted else 0.
        self.last_native_substep=native_substep
        return self.tau,self.rate,(self.target_rate-self.rate)/self.response_seconds

    def ahead(self,phase_end):
        self.__post_init__()
        if not math.isfinite(phase_end)or phase_end<self.tau:raise ValueError('invalid phase endpoint')
        return min(phase_end,self.tau+5*POLICY_DT*self.rate)

    def state(self):return asdict(self)


class UniformReference:
    """Actual or explicitly planned uniform source-format arrays plus terminal.

    Planned files stay tagged; only qualified measured WEAK rollouts are teacher
    training data. Target sentinels are never passed as physical reset states.
    """
    def __init__(self,trajectory,terminal,*,kind):
        if kind not in ['measured-qualified-weak','planned-unqualified','source-fixture']:
            raise ValueError('reference evidence kind required')
        self.kind=kind;self.data={}
        length=None
        for name,width in [('qpos',28),('qvel',27),('com',3),('com_ori',3)]:
            x=np.asarray(trajectory[name],dtype=np.float64)
            if x.ndim!=2 or x.shape[1]!=width or len(x)<1 or not np.isfinite(x).all():raise ValueError('invalid source trajectory '+name)
            if length is not None and len(x)!=length:raise ValueError('unaligned trajectory row counts')
            length=len(x);self.data[name]=x.copy()
        if kind!='source-fixture' and not np.allclose(np.linalg.norm(self.data['qpos'][:,3:7],axis=1),1.,atol=1e-6,rtol=0.):
            raise ValueError('new references require valid normalized root quaternions')
        self.length=length
        self.terminal=np.asarray(terminal,dtype=np.float64)
        if self.terminal.shape!=(25,)or not np.isfinite(self.terminal).all():raise ValueError('invalid terminal25')

    def target_at(self,decision):
        if not isinstance(decision,(int,np.integer))or decision<0:raise ValueError('decision index must be nonnegative integer')
        if decision>=self.length:return self.terminal.copy()
        return target(self.data['qpos'][decision,7:],self.data['com'][decision,2],self.data['com_ori'][decision])

    def now_ahead(self,decision):return self.target_at(decision),self.target_at(decision+5)


def native_pose_to_source_qpos(position,rotation,hinges):
    from scipy.spatial.transform import Rotation
    position=np.asarray(position,dtype=float);rotation=np.asarray(rotation,dtype=float);hinges=np.asarray(hinges,dtype=float)
    if position.shape!=(3,)or rotation.shape!=(3,3)or hinges.shape!=(21,)or not all(np.isfinite(x).all()for x in [position,rotation,hinges]):raise ValueError('invalid native pose')
    # Basis conjugation first; scipy quaternion is xyzw, source MuJoCo is wxyz.
    quat=Rotation.from_matrix(P@rotation@P.T).as_quat()
    return np.r_[P@position,quat[[3,0,1,2]],hinges]
