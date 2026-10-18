"""PURE prototype: source SLOW holding-torque base + pre-tanh mean correction.

Explicit architecture adaptation, not the original torque-teacher network.
Uses the inherited epsilon-regularized source log probability, not an exact
normalized density near saturation. No extra state-shift action Jacobian applies.
Single-MLP binary export is unsupported; future optimizers must bind correction
parameters only. No fitting/collection/engine invocation or activation in the current failed-fit
stage. Power1 only; original source hold targets (including height1.4) stay inside
frozen SLOW base. New task references condition only the trainable correction.
"""
import json,math
import numpy as np
import torch
from actors import DenseActor
from native_env import ROOT
from source_math import ORDER
from stabilization_init import logstd_raw_bias
from training import source_module


class AnalyticSLOWHoldingActor(torch.nn.Module):
    single_mlp_binary_export_supported=False
    def __init__(self,*,epsilon=1e-6,seed=0):
        super().__init__()
        if epsilon!=1e-6:raise ValueError('prototype saturation epsilon is fixed')
        self.epsilon=epsilon;source=source_module();self.source=source;torch.set_num_threads(1);torch.manual_seed(seed)
        self.frozen_slow=source.Actor(118,21,-5,2)
        safe=DenseActor.load(ROOT/'rust/crates/sim/data/getup-policy/student.bin')
        with torch.no_grad():
            for index,(w,b)in zip([0,2,4],safe.layers):self.frozen_slow.trunk[index].weight.copy_(torch.from_numpy(w));self.frozen_slow.trunk[index].bias.copy_(torch.from_numpy(b))
        self.frozen_slow.requires_grad_(False)
        self.correction=source.Actor(118,21,-5,2)
        with torch.no_grad():
            # Nonzero hidden layers, zero final mean head: delta(s)==0 for every
            # input without killing the entire network's learning gradients.
            self.correction.trunk[4].weight.zero_();self.correction.trunk[4].bias[:21].zero_();self.correction.trunk[4].bias[21:].fill_(logstd_raw_bias())
        standing=torch.tensor(json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())['referenceMetadata']['standingPose'],dtype=torch.float32)
        self.register_buffer('source_standing',standing);self.register_buffer('source_hold_target',torch.cat((standing,torch.tensor([1.4,0.,0.,1.]))));self.register_buffer('actuator_order',torch.tensor(ORDER,dtype=torch.long));self.outputs={}
    def _validate(self,state):
        if state.ndim!=2 or state.shape[1]!=118 or state.dtype!=torch.float32 or not torch.isfinite(state).all()or not torch.all(state[:,-1]==1):raise ValueError('finite source118 float32/power1 batch required')
    def base_torque(self,state):
        self._validate(state)
        with torch.no_grad():
            target=self.source_hold_target.expand(len(state),-1);slow_state=torch.cat((state[:,:67],target,target,torch.ones_like(state[:,-1:])),dim=1)
            raw=self.frozen_slow.trunk(slow_state)
            if not torch.isfinite(raw).all():raise ValueError('nonfinite frozen SLOW output')
            residual=torch.tanh(raw[:,:21])
            qpos_control=torch.clamp(self.source_standing+residual-state[:,:21]-.2*state[:,46:67],-1.,1.)
            return qpos_control[:,self.actuator_order]
    def latent_components(self,state):
        base=self.base_torque(state);interior=torch.clamp(base,-1+self.epsilon,1-self.epsilon);latent=torch.atanh(interior);raw=self.correction.trunk(state);mean=latent+raw[:,:21];logstd=-1.5+3.5*torch.tanh(raw[:,21:])
        if not torch.isfinite(mean).all()or not torch.isfinite(logstd).all():raise ValueError('nonfinite analytic actor')
        return mean,logstd,base,interior
    def forward(self,state,compute_pi=True,compute_log_pi=True):
        if compute_log_pi and not compute_pi:raise ValueError('log probability requires a stochastic sample')
        mean,logstd,base,interior=self.latent_components(state)
        if compute_pi:
            noise=torch.randn_like(mean);sample=mean+logstd.exp()*noise
            logp=self.source.gaussian_logprob(noise,logstd)if compute_log_pi else None
        else:sample=None;logp=None
        mu,pi,logp=self.source.squash(mean,sample,logp,state[:,-1])
        self.outputs=dict(baseTorque=base.detach(),baseSaturation=(base!=interior).detach(),latentMean=mean.detach())
        return mu,pi,logp,logstd
    @property
    def trainable_parameters(self):return self.correction.parameters()
