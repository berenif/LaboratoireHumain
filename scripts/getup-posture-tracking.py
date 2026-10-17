"""Moving joint-reference tracking constraints; no physics or pose writes."""
import numpy as np


def clock(elapsed,duration):
    if elapsed<=0.:return 0.,0.,0.
    if elapsed>=duration:return 1.,0.,0.
    u=elapsed/duration
    return u*u*(3.-2.*u),6.*u*(1.-u)/duration,(6.-12.*u)/duration**2


class TrackingReference:
    def __init__(self,references,limits,include_arms=False):
        self.references=references;self.include_arms=include_arms
        # Scalar PCHIP preserves each coordinate's knotwise range. Consequently
        # these bounds cover the complete interpolated forward joint path.
        knots=references.curve(references.curve.x)
        margin=np.minimum(knots[:,:15].min(axis=0)-limits[:15,0],limits[:15,1]-knots[:,:15].max(axis=0))
        assert np.all(margin>0.),'Certified forward tracking path must be interior'
        # Optional arm bands are tracking priorities. The actual seed may be at
        # a native arm bound; no interior-path containment is claimed for arms.
        self.radius=np.r_[np.minimum(.03,.5*margin),np.full(6,.03)if include_arms else np.zeros(6)]
        if include_arms:assert min(np.r_[knots[-1,15:]-limits[15:,0],limits[15:,1]-knots[-1,15:]])>=.0999,'Interior terminal arm rest required; actual seed may begin at a native bound'

    def reference(self,elapsed,ramp,phase,phase_elapsed,initial_q,landed_left_q=None):
        curve=self.references.curve
        if phase=='transfer':
            w,rate,acceleration=clock(elapsed-1.,ramp)
            position=curve(w);velocity=curve.derivative()(w)*rate
            joint_acceleration=curve.derivative(2)(w)*rate**2+curve.derivative()(w)*acceleration
            indices=list(range(21 if self.include_arms else 15))
        else:
            position=curve(1.);velocity=np.zeros(21);joint_acceleration=np.zeros(21);indices=list(range(9))+(list(range(15,21))if self.include_arms else [])
            if phase in ['return','stand']:
                w,rate,acceleration=(1.,0.,0.)if phase=='stand'else clock(phase_elapsed,3.)
                delta=np.asarray(initial_q)-position
                if self.include_arms:delta[15:]=0.
                position=position+w*delta;velocity=rate*delta;joint_acceleration=acceleration*delta
                if landed_left_q is not None:
                    position[9:15]=landed_left_q;velocity[9:15]=0.;joint_acceleration[9:15]=0.
        return dict(indices=indices,radius=self.radius.copy(),position=position,velocity=velocity,acceleration=joint_acceleration)


def admissibility(q,velocity,tracking):
    indices=np.asarray(tracking['indices']);radius=tracking['radius'][indices]
    error=np.asarray(q)[indices]-tracking['position'][indices]
    relative_velocity=np.asarray(velocity)[indices]-tracking['velocity'][indices]
    return dict(minimumPositionMarginRad=float(min(radius-abs(error))),minimumFirstOrderBarrierMarginRadps=float(min(np.r_[4*(radius-error)-relative_velocity,4*(radius+error)+relative_velocity])))
