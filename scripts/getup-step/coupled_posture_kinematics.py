"""Independent analytic FK jets + archived verified arm spline, without native IO.

Only source-model frame data and spline coefficients are consumed. No fitting,
new lookup points, optimizer, table extrapolation, state writes, or integration.
"""
from dataclasses import dataclass
import math
import numpy as np
from coupled_posture_reference import Jet, OrientationJet, frozen

P = np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]])
SIDES = ('right','left')  # Feet elsewhere remain [left,right].


@dataclass(frozen=True)
class MatrixJet:
    position: tuple
    velocity: tuple
    acceleration: tuple

    def __post_init__(self):
        for field in ('position','velocity','acceleration'):
            object.__setattr__(self,field,frozen(getattr(self,field),16))
        T,D,H = self.arrays(); R = T[:3,:3]
        if (np.max(abs(T[3]-[0.,0.,0.,1.])) > 1e-10
                or np.max(abs(D[3])) > 1e-10 or np.max(abs(H[3])) > 1e-10
                or np.max(abs(R.T@R-np.eye(3))) > 1e-8 or abs(np.linalg.det(R)-1.) > 1e-8
                or np.max(abs(R.T@D[:3,:3]+D[:3,:3].T@R)) > 1e-8
                or np.max(abs(R.T@H[:3,:3]+H[:3,:3].T@R+2*D[:3,:3].T@D[:3,:3])) > 1e-8):
            raise ValueError('pose jet is not a rigid homogeneous transform jet')

    @classmethod
    def rest(cls, M):
        return cls(frozen(M),(0.,)*16,(0.,)*16)

    def arrays(self):
        return tuple(np.array(v).reshape(4,4) for v in
            (self.position,self.velocity,self.acceleration))

    @property
    def T(self):
        return self.arrays()[0]

    @property
    def Tdot(self):
        return self.arrays()[1]

    @property
    def Tddot(self):
        return self.arrays()[2]

    def __matmul__(self, other):
        a,b,c = self.arrays(); x,y,z = other.arrays()
        return MatrixJet(frozen(a@x),frozen(b@x+a@y),frozen(c@x+2*b@y+a@z))

    def inverse(self):
        a,b,c = self.arrays(); x = np.linalg.inv(a)
        return MatrixJet(frozen(x),frozen(-x@b@x),frozen(2*x@b@x@b@x-x@c@x))


@dataclass(frozen=True)
class BaseFrameSample:
    """Shared FK/IK handoff: all pose matrices in MODEL/FK Y-up axes.

    baseline_joints carries source-indexed all21 joints, including whichever
    exact arm law the producer declared. A frame_path supplies one such value
    with analytic derivatives at the requested source time. No frozen-root or
    zero-derivative assumption is made, and no root pose is applied to physics.
    """
    root: MatrixJet
    baseline_joints: Jet
    stance: MatrixJet

    def __post_init__(self):
        if len(self.baseline_joints.position) != 21:
            raise ValueError('moving-frame handoff requires all21 joint jets')

    @property
    def root_model(self):
        return self.root

    @property
    def joints(self):
        return self.baseline_joints

    @property
    def stance_pose_model(self):
        return self.stance


@dataclass(frozen=True)
class SourcePoseJet:
    translation: Jet
    rotation: tuple
    angular_velocity_world: tuple
    angular_acceleration_world: tuple


def skew(vector):
    x,y,z = frozen(vector,3)
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def pose_jet_from_source(translation, Rsource, omega_world, alpha_world):
    """Convert source spatial jets to model T/Tdot/Tddot, including w×w."""
    R = np.asarray(Rsource,dtype=float)
    if (R.shape != (3,3) or not np.isfinite(R).all()
            or np.max(abs(R.T@R-np.eye(3))) > 1e-9 or abs(np.linalg.det(R)-1.) > 1e-9):
        raise ValueError('invalid source pose rotation')
    p,v,a = translation.arrays()
    if p.shape != (3,):
        raise ValueError('source pose translation must have three components')
    W,A = skew(omega_world),skew(alpha_world)
    T = np.eye(4); D = np.zeros((4,4)); H = np.zeros((4,4))
    T[:3,:3] = P.T@R@P; T[:3,3] = P.T@p
    D[:3,:3] = P.T@W@R@P; D[:3,3] = P.T@v
    H[:3,:3] = P.T@(A+W@W)@R@P; H[:3,3] = P.T@a
    return MatrixJet(frozen(T),frozen(D),frozen(H))


def pose_jet_to_source(pose):
    T,D,H = pose.arrays()
    R,dR,ddR = [P@x[:3,:3]@P.T for x in (T,D,H)]
    W = dR@np.linalg.inv(R); A = ddR@np.linalg.inv(R)-W@W
    vee = lambda K: np.array([K[2,1]-K[1,2],K[0,2]-K[2,0],K[1,0]-K[0,1]])/2.
    return SourcePoseJet(Jet(*(P@x[:3,3] for x in (T,D,H))),frozen(R),
        frozen(vee(W)),frozen(vee(A)))


def xyz(value):
    return np.array([value[k] for k in 'xyz'])


def model_transform(frame):
    x,y,z,w = [frame['rotation'][k] for k in 'xyzw']
    n = x*x+y*y+z*z+w*w
    M = np.eye(4)
    # Preserve the source's quaternion norm convention, as existing independent FK.
    M[:3,:3] = [[n-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),n-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),n-2*(x*x+y*y)]]
    M[:3,3] = xyz(frame['anchor'])
    return M


def hinge_jet(angle, velocity, acceleration):
    c,s = math.cos(angle),math.sin(angle)
    R = np.eye(4); D = np.zeros((4,4)); H = np.zeros((4,4))
    R[1:3,1:3] = [[c,-s],[s,c]]
    D[1:3,1:3] = [[-s,-c],[c,-s]]
    H[1:3,1:3] = [[-c,s],[-s,-c]]
    return MatrixJet(frozen(R),frozen(D*velocity),frozen(H*velocity**2+D*acceleration))


@dataclass(frozen=True)
class ArmMap:
    ranges: tuple
    grids: tuple
    coefficients: tuple
    neutral_pitch: tuple
    goal: tuple

    @classmethod
    def from_archived_table(cls, table):
        receipt = table['originalReceipt']
        if (receipt['sideOrder'] != list(SIDES) or receipt['angularGain'] != -1.
                or receipt.get('clippingEnabled') is not False
                or receipt.get('balanceArmOffsetEnabled') is not False):
            raise ValueError('wrong mandatory arm mapping contract')
        ranges = tuple(frozen(row,2) for row in receipt['verifiedSwingDeltaRangeBySideRad'])
        grids = tuple(frozen(grid) for grid in table['gridsBySideRad'])
        coefficients = tuple(tuple(tuple(frozen(vector,3) for vector in row)
            for row in side) for side in table['splineCoefficientsBySide'])
        for i in range(2):
            g = np.array(grids[i]); c = np.array(coefficients[i])
            if (len(g) < 2 or np.any(np.diff(g) <= 0.) or c.shape != (4,len(g)-1,3)
                    or abs(g[0]-ranges[i][0]) > 1e-14 or abs(g[-1]-ranges[i][1]) > 1e-14):
                raise ValueError('invalid archived spline dimensions or verified range')
        return cls(ranges,grids,coefficients,frozen(receipt['neutralArmPitchRad'],2),
            frozen(receipt['goalRad'],6))

    def evaluate(self, delta, reserve=.02):
        """Chain rule F(delta), F' delta_dot, F'' delta_dot²+F' delta_ddot."""
        p,v,a = delta.arrays()
        if len(p) != 2 or reserve < 0. or not math.isfinite(reserve):
            raise ValueError('right/left two-thigh jet required')
        out = [[],[],[]]
        for i in range(2):
            lo,hi = self.ranges[i]
            if p[i] < lo+reserve-1e-12 or p[i] > hi-reserve+1e-12:
                raise ValueError('future thigh outside original verified map/reserve; no clipping')
            grid = self.grids[i]; k = min(len(grid)-2,max(0,int(np.searchsorted(grid,p[i],side='right')-1)))
            x = p[i]-grid[k]; c = np.array(self.coefficients[i])[:,k,:]
            mapped = ((c[0]*x+c[1])*x+c[2])*x+c[3]
            jac = (3*c[0]*x+2*c[1])*x+c[2]; hessian = 6*c[0]*x+2*c[1]
            out[0].extend(mapped); out[1].extend(jac*v[i]); out[2].extend(hessian*v[i]**2+jac*a[i])
        return Jet(*out)


@dataclass(frozen=True)
class Part:
    name: str
    parent: str
    qindex: object
    parent_frame: MatrixJet
    child_inverse: MatrixJet
    mass: float
    com: tuple


@dataclass(frozen=True)
class FKResult:
    full_joints: Jet
    future_thigh_right_left: Jet
    future_arm_delta_right_left: Jet
    com_source: Jet
    thorax: OrientationJet
    pelvis: OrientationJet
    root_model: MatrixJet
    poses_model: tuple

    def pose(self, name):
        return dict(self.poses_model)[name]

    def base_frame(self, stance_side):
        if stance_side not in (0,1):
            raise ValueError('feet order is [left,right]')
        return BaseFrameSample(self.root_model,self.full_joints,
            self.pose(('left_foot','right_foot')[stance_side]))


@dataclass(frozen=True)
class ModelFK:
    parts: tuple
    total_mass: float
    limits: tuple

    @classmethod
    def from_model(cls, model):
        parts = []; limits = np.zeros((21,2))
        for p in model['segments']:
            h = (p.get('reference') or {}).get('hinge')
            index = None if not h else h['qposIndex']
            if p['id'] == 'torso':
                parent = child = np.eye(4)
            else:
                j = p['jointProfile']
                parent = model_transform(j['parentFrame'])
                child = np.linalg.inv(model_transform(j['childFrame']))
                if h:
                    axis = j['axes'][0]; limits[index] = axis['minRadians'],axis['maxRadians']
            parts.append(Part(p['id'],p.get('parent'),index,MatrixJet.rest(parent),
                MatrixJet.rest(child),float(p['massKg']),frozen(xyz(p['goldenMass']['centerOfMass']),3)))
        return cls(tuple(parts),float(model['totalMassKg']),tuple(frozen(row,2) for row in limits))

    def forward(self, joints, root=None):
        q,v,a = joints.arrays()
        if len(q) != 21:
            raise ValueError('source FK requires 21 joints')
        out = {'torso':MatrixJet.rest(np.eye(4)) if root is None else root}
        for p in self.parts[1:]:
            hinge = MatrixJet.rest(np.eye(4)) if p.qindex is None else hinge_jet(q[p.qindex],v[p.qindex],a[p.qindex])
            out[p.name] = out[p.parent]@p.parent_frame@hinge@p.child_inverse
        return out

    def thigh_jet(self, lower_joints):
        if len(lower_joints.position) != 15:
            raise ValueError('lower reference must be source joints0..14')
        full = Jet(*(np.r_[x,np.zeros(6)] for x in lower_joints.arrays()))
        poses = self.forward(full); values = [[],[],[]]
        for side in SIDES:
            thigh = poses[side+'_thigh'].arrays(); shin = poses[side+'_shin'].arrays()
            d,dd,ddd = [P@(b[:3,3]-a[:3,3]) for a,b in zip(thigh,shin)]
            x,z = d[[0,2]]; dx,dz = dd[[0,2]]; ddx,ddz = ddd[[0,2]]
            den = x*x+z*z
            if den < 1e-12:
                raise ValueError('future thigh pitch singular')
            numerator = -z*dx+x*dz
            values[0].append(math.atan2(x,-z))
            values[1].append(numerator/den)
            values[2].append((-z*ddx+x*ddz)/den-numerator*2*(x*dx+z*dz)/den**2)
        return Jet(*values)

    def coupled(self, lower_joints, stance_side, stance_pose_model, arm_map, leg_origin):
        """Triangular future-thigh mapping then full COM, anchored to given foot.

        stance_pose_model is a MatrixJet in the MODEL coordinate system. This may
        be a rest foot, or a separately C2 planned foot. It never reads live state.
        Runtime delayed arms remain a different, measured causal task.
        """
        if stance_side not in (0,1):
            raise ValueError('feet order is [left,right]')
        theta = self.thigh_jet(lower_joints); p,v,a = theta.arrays()
        delta = Jet(-(p-np.array(frozen(leg_origin,2))),-v,-a)
        arms = arm_map.evaluate(delta)
        full = Jet(*(np.r_[x,y] for x,y in zip(lower_joints.arrays(),arms.arrays())))
        poses = self.forward(full)
        foot = ('left_foot','right_foot')[stance_side]
        root = stance_pose_model@poses[foot].inverse()
        poses = self.forward(full,root)
        com = [np.zeros(3),np.zeros(3),np.zeros(3)]
        for part in self.parts:
            for i,M in enumerate(poses[part.name].arrays()):
                com[i] += part.mass*(P@(M@np.r_[part.com,1.])[:3])/self.total_mass
        def orientation(name,body):
            T,V,A = poses[name].arrays()
            R,dR,ddR = [P@x[:3,:3]@P.T for x in (T,V,A)]
            omega_hat = dR@np.linalg.inv(R)
            alpha_hat = ddR@np.linalg.inv(R)-omega_hat@omega_hat
            vee = lambda K: np.array([K[2,1]-K[1,2],K[0,2]-K[2,0],K[1,0]-K[0,1]])/2.
            return OrientationJet(body,frozen(R),frozen(vee(omega_hat)),frozen(vee(alpha_hat)))
        return FKResult(full,theta,delta,Jet(*com),orientation('torso',1),orientation('pelvis',4),
            root,tuple(poses.items()))
