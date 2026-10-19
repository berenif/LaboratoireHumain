"""Per-sample exact body-node reuse in the ORIGINAL strict reference owner.

This is a cache-only adapter. WholeBodyCycle.sample, StanceRealizedIK.derive,
ModelFK.coupled/thigh_jet, with_arms, hinge_jet, MatrixJet operations and all
strict arm-map checks remain their original implementations. No plain-triple
FK, changed arithmetic, finite differences, normalization, or admission bypass.
"""
import copy
from dataclasses import dataclass, field
import numpy as np
from coupled_posture_kinematics import ModelFK, MatrixJet, hinge_jet

VERSION = 'original-strict-per-sample-body-cache-v1'


@dataclass
class _BodyCacheState:
    capacity: int = 256
    cache: dict = field(default_factory=dict)
    forward_calls: int = 0
    requests: int = 0
    hits: int = 0
    misses: int = 0
    evictions: int = 0


def _pose_bytes(pose):
    # Every float64 bit, including signed zero, in all three 4x4 matrices.
    return tuple(np.asarray(value, dtype=np.float64).tobytes()
                 for value in (pose.position, pose.velocity, pose.acceleration))


class _SampleBodyModel(ModelFK):
    """Original immutable model fields plus one bounded, ephemeral body cache."""
    def __init__(self, original, capacity=256):
        super().__init__(original.parts, original.total_mass, original.limits)
        object.__setattr__(self, 'state', _BodyCacheState(capacity=capacity))

    def forward(self, joints, root=None):
        # ORIGINAL ModelFK.forward loop and exact miss expression. Only key
        # lookup/insertion surrounds it; original MatrixJet validation remains.
        q,v,a = joints.arrays()
        if len(q) != 21:
            raise ValueError('source FK requires 21 joints')
        out = {'torso':MatrixJet.rest(np.eye(4)) if root is None else root}
        self.state.forward_calls += 1
        for index,p in enumerate(self.parts[1:],1):
            hinge_key = (None if p.qindex is None else
                np.asarray([q[p.qindex],v[p.qindex],a[p.qindex]],dtype=np.float64).tobytes())
            key = (index,*_pose_bytes(out[p.parent]),hinge_key)
            self.state.requests += 1
            if key in self.state.cache:
                self.state.hits += 1
                out[p.name] = self.state.cache[key]
                continue
            self.state.misses += 1
            hinge = MatrixJet.rest(np.eye(4)) if p.qindex is None else hinge_jet(q[p.qindex],v[p.qindex],a[p.qindex])
            out[p.name] = out[p.parent]@p.parent_frame@hinge@p.child_inverse
            self.state.cache[key] = out[p.name]
            if len(self.state.cache) > self.state.capacity:
                self.state.cache.pop(next(iter(self.state.cache)))
                self.state.evictions += 1
        # A new dict is returned on every forward. Its values are ORIGINAL
        # frozen MatrixJets, which are safe to share within this sample only.
        return out


class StrictReferenceCache:
    """Unchanged sample API, with all caches destroyed on return or exception.

    Reusing this stateless wrapper does not reuse any posture, branch seed,
    model result, arm state, command, or cache from a preceding sample.
    """
    def sample(self, path, phase, elapsed, *, runtime_arms=None,
               landing_adjustment=None):
        local = copy.copy(path)
        local.realizer = copy.copy(path.realizer)
        local.realizer.model = _SampleBodyModel(path.realizer.model)
        return local.sample(phase, elapsed, runtime_arms=runtime_arms,
                            landing_adjustment=landing_adjustment)
