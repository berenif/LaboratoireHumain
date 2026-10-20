"""Production cold-start phases; only the clock, bridge and backup I/O are ports.

The same phases run in a disposable SOFTWARE rehearsal. Synthetic records and
receipts remain explicitly synthetic and cannot open a real experiment gate.
No Runtime, clock or learner is created on import.
"""
from __future__ import annotations
import copy
from dataclasses import asdict, dataclass, replace
import hashlib
import json
from pathlib import Path
import random

from experiment_backup import atomic_json, sha256
from getup_cold_start import ColdPhase, FreshMapper, PROTOCOL, artifact
from getup_cold_start_verify import assert_state_equal, verify_replay_mapping
from getup_learning_loop import ContractError, EpisodeReplay, FixedWitness, Gaussian, Record, UpdateGuard, diagnostic_membership


def encoded(value):
    return json.loads(json.dumps(value, default=lambda x: x.tolist(), allow_nan=False))


def tuples(value):
    return tuple(map(tuples,value)) if isinstance(value,list) else value


@dataclass(frozen=True)
class PhasePlan:
    started: float
    deadline: float
    work_end: float
    evaluation_end: float
    parity_seconds: float = 60.
    bootstrap_seconds: float = 180.
    backup_seconds: float = 180.
    warmup_seconds: float = 120.
    collection_seconds: float = 120.
    updates_seconds: float = 120.
    bootstrap_integrations: int = 75000
    block_integrations: int = 20140
    parity_integrations: int = 15060
    evaluation_integrations: int = 147000
    backup_payload_envelope_bytes: int = 17*1024**2
    backup_archive_envelope_bytes: int = 18*1024**2

    def validate(self):
        if (self.deadline-self.started!=3600 or self.deadline-self.evaluation_end<300
                or self.evaluation_end-self.work_end<1200
                or self.work_end!=self.started+2100 or self.evaluation_end!=self.started+3300
                or not self.started<self.work_end<self.evaluation_end<self.deadline
                or self.evaluation_integrations!=147000
                or self.backup_payload_envelope_bytes>64*1024**2
                or self.backup_archive_envelope_bytes>32*1024**2):
            raise ContractError('Immutable absolute work/evaluation/retention cutoffs required')
        for key,value in asdict(self).items():
            if not __import__('math').isfinite(value) or value<=0:
                raise ContractError('Invalid finite positive phase envelope: '+key)
        return self

    def stage_cutoff(self,name,cycle=0):
        fixed={'immutable-input-restore':120,'zero-correction-qualification':180,
            'transition-parity':240,'bootstrap':600,'warmup':900,
            'backup:bootstrap-before-any-optimizer-update':600,
            'backup:critic-warmup-complete':900,'backup:collection-00':1500,
            'backup:updates-01':1500,'backup:collection-01':2100,
            'backup:updates-02':2100,'evaluation':3300,'backup:terminal-stop':3600}
        if name in ('collection','updates'):
            if cycle not in (0,1):raise ContractError('Exactly two blocks; unused capacity grants no extra cycle')
            return self.started+(1500 if cycle==0 else 2100)
        if name not in fixed:raise ContractError('Unknown fixed pilot stage: '+name)
        return self.started+fixed[name]

    def assert_inputs_ready(self,now):
        if now>=self.stage_cutoff('immutable-input-restore'):
            raise ContractError('Absolute two-minute input restoration cutoff reached')

    def sequence_seconds(self, blocks, *, bootstrap=True, warmup=True):
        return ((self.parity_seconds+self.bootstrap_seconds+self.backup_seconds if bootstrap else 0)
                +(self.warmup_seconds+self.backup_seconds if warmup else 0)
                +blocks*(self.collection_seconds+self.updates_seconds+2*self.backup_seconds))

    def admit_sequence(self, now, ledger, *, maximum_blocks=2, prefix_seconds=0., prefix_integrations=0, cumulative_uploaded_bytes=0, retained_payload_bytes=0, replacement_slots=False):
        self.validate()
        if prefix_seconds<0 or prefix_integrations<0:
            raise ContractError('Prefix allowance cannot refund wall or integrations')
        if ledger.get('pendingCall') or ledger.get('uncertainCalls') or ledger.get('uncertainIntegrations',0):
            raise ContractError('Unresolved execution is terminal; never retry or refund')
        for blocks in range(min(2,maximum_blocks),1,-1):
            steps=(ledger['nativeIntegrations']+prefix_integrations+self.parity_integrations+self.bootstrap_integrations
                   +blocks*self.block_integrations+self.evaluation_integrations)
            backups=3+2*blocks
            storage_fits=(cumulative_uploaded_bytes+backups*self.backup_archive_envelope_bytes<=160*1024**2
                and retained_payload_bytes+(1 if replacement_slots else backups)*self.backup_payload_envelope_bytes<=160*1024**2)
            if storage_fits and steps<=300000 and now+prefix_seconds+self.sequence_seconds(blocks)<=self.work_end:
                return blocks
        raise ContractError('Complete two-block cadence, backups and final reserves do not fit')


@dataclass
class PhaseCursor:
    phase: str = 'ready'
    episode_number: int = 0
    episode_id: str | None = None
    case: str | None = None
    active: bool = False
    bootstrap_complete: bool = False
    transition_parity: bool = False
    coherent_before_updates: bool = False
    planned_blocks: int = 0
    completed_cycles: int = 0


class ColdPilot:
    """Existing algorithm, with explicit paused state and bounded phase methods."""
    def __init__(self, *, output, learner, learner_factory, builder_factory, bridge,
                 recovery, clock, plan, identity, synthetic=False):
        self.output=Path(output);self.output.mkdir(parents=True,exist_ok=True)
        self.learner=learner;self.learner_factory=learner_factory;self.builder_factory=builder_factory
        self.bridge=bridge;self.recovery=recovery;self.clock=clock;self.plan=plan.validate()
        self.identity=copy.deepcopy(identity);self.synthetic=synthetic
        if bridge.synthetic is not synthetic or recovery.synthetic is not synthetic:
            raise ContractError('Scripted and actual ports cannot be mixed')
        self.cursor=PhaseCursor();self.state=None;self.mapper=None;self.replay=None;self.phase=None
        self.bootstrap_rows=[];self.mapping=[];self.bootstrap_indices=[];self.excluded_lines=[]
        self.recent=[];self.witness=None;self.witness_indices=[];self.policy_history=[];self.failure_latched=False
        self.phase_deadline=plan.work_end;self.phase_native_end=300000-147000
        self.report={'schema':'lh-cold-phase-report/v1','synthetic':synthetic,
            'softwareRehearsal':synthetic,'actualTransitionParityPassed':False,
            'actualCoherentRecoveryPassed':False,'physicalQualificationPassed':False,
            'trainingExecuted':False,'syntheticOptimizerMechanicsExercised':False,
            'candidateAccepted':False,'releaseAccepted':False,'accepted':0,'rejected':0,
            'costs':{'controlledPhysicsSeconds':0.,'forecastMixedSeconds':0.,'bridgeSeconds':0.,
                'learningSeconds':0.,'serializationSeconds':0.,'transportRestoreSeconds':0.},
            'forecastOnlyDurationMeasured':False}

    def durable_ledger(self):
        try:return self.bridge.call({'op':'ledger'})
        except BaseException:
            path=self.output/'budget-ledger.json'
            if not path.is_file():raise
            return json.loads(path.read_text())

    def ledger(self):
        value=self.bridge.call({'op':'ledger'})
        if (value.get('deadlineUnixMs')!=round(self.plan.deadline*1000)
                or value.get('limit')!=300000 or value.get('nativeIntegrations',-1)<0
                or not 0<=value.get('trainingDecisions',-1)<=4096
                or value.get('bootstrapDecisions',0)>1024 or value.get('iterativeDecisions',0)>3072
                or value['nativeIntegrations']>300000):
            raise ContractError('Bridge ledger identity/caps malformed')
        if value.get('pendingCall') or value.get('uncertainCalls') or value.get('uncertainIntegrations',0):
            raise ContractError('Uncertain command remains charged; no retry')
        return value

    def enter(self, name, seconds, integrations=0, *, evaluation=False, retention=False):
        end=self.plan.stage_cutoff(name,self.phase.cycle if self.phase else 0)
        end=min(end,self.plan.deadline if retention else self.plan.evaluation_end if evaluation else self.plan.work_end)
        now=self.clock.now()
        if self.failure_latched and not retention:raise ContractError('Failure latched; no next phase')
        if now+seconds>end:raise ContractError('Full phase allowance exceeds absolute cutoff: '+name)
        ledger=self.durable_ledger() if retention else self.ledger()
        reserve=0 if evaluation or retention else self.plan.evaluation_integrations
        if ledger['nativeIntegrations']+integrations+reserve>300000:
            raise ContractError('Phase envelope would consume protected evaluation integrations')
        self.phase_deadline=now+seconds;self.phase_native_end=ledger['nativeIntegrations']+integrations
        self.cursor.phase=name
        if not retention:
            self.bridge.call({'op':'reserve','integrations':reserve,'wallSeconds':self.plan.deadline-end})

    def check_time(self):
        if self.clock.now()>=self.phase_deadline:raise ContractError('Phase hard wall exhausted')

    def call(self, query, *, physical=False, upper_bound=5020):
        self.check_time()
        before=self.ledger() if physical else None
        if physical and before['nativeIntegrations']+upper_bound>self.phase_native_end:
            raise ContractError('Next native command exceeds predeclared phase envelope')
        old_deadline=getattr(self.bridge,'deadline',None)
        if old_deadline is not None:self.bridge.deadline=min(old_deadline,self.phase_deadline)
        began=self.clock.now()
        try:
            value=self.bridge.call(query)
            self.check_time()
            if physical:
                after=self.ledger()
                if (after['nativeIntegrations']<before['nativeIntegrations']
                        or after['nativeIntegrations']-before['nativeIntegrations']>upper_bound
                        or after['trainingDecisions']<before['trainingDecisions']):
                    raise ContractError('Bridge refunded or exceeded conservative command charge')
            category=('forecastMixedSeconds' if after['forecastIntegrations']>before['forecastIntegrations']
                      else 'controlledPhysicsSeconds') if physical else 'bridgeSeconds'
            self.report['costs'][category]+=self.clock.now()-began
            return value
        except BaseException:
            self.failure_latched=True
            raise
        finally:
            if old_deadline is not None:self.bridge.deadline=old_deadline

    def raw_lines(self):
        path=self.output/'transitions.ndjson'
        return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []

    def reset(self, case, policy, head=None):
        self.cursor.episode_number+=1
        self.cursor.episode_id=f'cold-{case}-{self.cursor.episode_number:03d}'
        self.cursor.case=case
        self.state=self.call({'op':'reset','case':case,'label':self.cursor.episode_id,
            'external':head is None,'prepared':True,'head':head},physical=True)
        self.cursor.active=True
        self.mapper=FreshMapper(self.builder_factory(),self.cursor.episode_id,policy,case,
            len(self.raw_lines()),synthetic=self.synthetic)

    def step(self, action, collection_phase):
        source=self.state
        if not isinstance(source,dict) or not self.cursor.active:
            raise ContractError('No active source state')
        result=self.call({'op':'step','c':action,'training':bool(source.get('preview')),
            'collectionPhase':collection_phase,'episodeId':self.cursor.episode_id,
            'policyId':self.mapper.behavior},physical=True)
        if not isinstance(result,dict) or not {'state','native','reward','terminal','success','failure'}<=result.keys():
            raise ContractError('Malformed bridge transition')
        # A native refusal can legitimately terminate before integrating a sample.
        # Retain its observed state/raw evidence and original failure, never turn
        # that zero-step refusal into replay or mask it with the +20 row contract.
        self.state=result['state']
        if result['failure'] is not None and (not isinstance(result['failure'],dict)
                or not isinstance(result['failure'].get('check'),str) or not result['failure']['check']):
            raise ContractError('Malformed bridge failure diagnostic')
        if result['failure'] and result['failure'].get('check')!='original25s-deadline':
            post=result['state']
            if result['native']==source['native'] or (isinstance(post,dict) and post.get('native')==source['native']):
                fields={'native','impactEpoch','sourceEpoch','preview'}
                valid=(isinstance(post,dict) and fields<=source.keys() and fields<=post.keys()
                    and result['native']==post['native']==source['native']
                    and source['impactEpoch'] is not None and post['impactEpoch']==source['impactEpoch']
                    and source['sourceEpoch'] is None and post['sourceEpoch'] is None
                    and source['preview'] is None and post['preview'] is None and action is None
                    and result['terminal'] is True and result['success'] is False
                    and result.get('nativeTraceSha256')==hashlib.sha256(b'[]').hexdigest()
                    and self.mapper.builder.pending is None and self.mapper.pending_origin is None
                    and self.mapper.pending_line is None)
                if not valid:
                    raise ContractError('Malformed zero-step refusal diagnostic contract')
            raise ContractError('Physical integrity failure: '+json.dumps(result['failure']))
        row=self.mapper.push(source,action,result)
        return row,result

    def parity(self):
        self.enter('transition-parity',self.plan.parity_seconds,self.plan.parity_integrations)
        self.reset('belly','baseline-deterministic-zero')
        source=copy.deepcopy(self.state)
        saved=self.call({'op':'checkpoint','name':'parity-runtime.bin'})
        bridge_state=self.call({'op':'bridgeState'})
        action=[0.]*21 if source.get('preview') else None
        query={'op':'step','c':action,'training':bool(source.get('preview')),
            'collectionPhase':'bootstrap','episodeId':self.cursor.episode_id,
            'policyId':self.mapper.behavior}
        first=self.call(query,physical=True)
        restored=self.call({'op':'coldRestore','path':str(self.output/'parity-runtime.bin'),
            'sha256':saved['sha256'],'bridge':bridge_state})
        if restored['physicsIntegrations']!=0 or restored['state']!=source:
            raise ContractError('Runtime restore changed source state')
        second=self.call(query,physical=True)
        if first!=second:raise ContractError('Next native trace/reward/terminal parity failed')
        # Exercise two independent REAL SourceTransition/FreshMapper instances.
        first_mapper=FreshMapper(self.builder_factory(),self.cursor.episode_id,
            self.mapper.behavior,'belly',len(self.raw_lines())-2,synthetic=self.synthetic)
        second_mapper=FreshMapper(self.builder_factory(),self.cursor.episode_id,
            self.mapper.behavior,'belly',len(self.raw_lines())-1,synthetic=self.synthetic)
        a=first_mapper.push(source,action,first);b=second_mapper.push(source,action,second)
        if encoded(asdict(a) if a else None)!=encoded(asdict(b) if b else None):
            raise ContractError('Independent next-transition mapping parity failed')
        if encoded(first_mapper.builder.pending)!=encoded(second_mapper.builder.pending):
            raise ContractError('Pending folded-tail restore differs')
        self.mapper=second_mapper;self.state=second['state']
        self.excluded_lines.append(len(self.raw_lines())-2)
        self.cursor.transition_parity=True
        self.report['actualTransitionParityPassed']=not self.synthetic
        self.report['syntheticTransitionParityPassed']=self.synthetic

    def bootstrap(self):
        self.enter('bootstrap',self.plan.bootstrap_seconds,self.plan.bootstrap_integrations)
        for case in ('belly','heldout','original'):
            if case!='belly':self.reset(case,'baseline-deterministic-zero')
            while not self.mapper.finished:
                used=self.ledger().get('bootstrapDecisions',0)
                if used>=1024 and self.mapper.builder.pending is None:break
                action=[0.]*21 if self.state.get('preview') else None
                if action is not None and used>=1024:
                    raise ContractError('Bootstrap cap before pending transition closes')
                self.step(action,'bootstrap')
            if case!='original':rows=self.mapper.completed_rows(preservation=True)
            elif self.mapper.finished:rows=self.mapper.completed_rows()
            else:
                if not self.mapper.rows or self.mapper.builder.pending is not None:
                    raise ContractError('Incomplete bootstrap source transition')
                rows=[replace(row,episode_outcome='truncation',complete_episode=False,
                    truncated=i==len(self.mapper.rows)-1).validate() for i,row in enumerate(self.mapper.rows)]
            offset=len(self.bootstrap_rows);self.bootstrap_rows.extend(rows)
            self.mapping.extend(dict(x,replay_index=x['replay_index']+offset) for x in self.mapper.mapping)
        self.replay=EpisodeReplay(self.bootstrap_rows,fresh_allowance=3072)
        self.bootstrap_indices=list(range(len(self.bootstrap_rows)))
        self.phase=ColdPhase(self.ledger()['bootstrapDecisions'],self.cursor.planned_blocks)
        self.cursor.bootstrap_complete=True
        self.clear_runtime()

    def clear_runtime(self):
        self.call({'op':'coldDrop'})
        self.cursor.active=False;self.state=None;self.mapper=None

    def backup(self, boundary, *, retention=False):
        self.enter('backup:'+boundary,self.plan.backup_seconds,retention=retention)
        proof=self.recovery.checkpoint(self,boundary,self.phase_deadline)
        self.check_time()
        if proof.get('passed') is not True or proof.get('synthetic') is not self.synthetic:
            raise ContractError('Coherent recovery proof absent or wrong mode')
        if boundary=='bootstrap-before-any-optimizer-update':
            self.cursor.coherent_before_updates=True
            self.report['actualCoherentRecoveryPassed']=not self.synthetic
        self.report.setdefault('recoveryPoints',[]).append(proof)

    def optimizer_gate(self):
        if self.replay is not None and self.replay.diagnostic_only:
            raise ContractError('Qualification diagnostic replay never authorizes optimizer')
        if not (self.cursor.bootstrap_complete and self.cursor.transition_parity
                and self.cursor.coherent_before_updates):
            raise ContractError('Real parity/coherent replay gates must precede optimizer')
        expected='synthetic_rehearsal' if self.synthetic else 'fresh_wasm'
        if any(row.source!=expected for row in self.replay.rows):
            raise ContractError('Synthetic and actual replay cannot be mixed for optimization')

    def warmup(self):
        if not (self.cursor.bootstrap_complete and self.cursor.transition_parity
                and self.cursor.coherent_before_updates):
            raise ContractError('Real parity/coherent replay gates must precede optimizer')
        self.optimizer_gate()
        self.enter('warmup',self.plan.warmup_seconds)
        while self.phase.phase=='warmup':
            self.check_time();began=self.clock.now();self.learner.critic_attempt(self.learner.batch(self.replay));self.learner.polyak()
            if not self.learner.finite():raise ContractError('Nonfinite critic warmup')
            self.phase.warmup_done();self.clock.work('critic');self.report['costs']['learningSeconds']+=self.clock.now()-began;self.check_time()
            self.report['trainingExecuted']=not self.synthetic
            self.report['syntheticOptimizerMechanicsExercised']=self.synthetic

    def collect(self):
        self.enter('collection',self.plan.collection_seconds,self.plan.block_integrations)
        policy=self.learner.policy_id();self.phase.begin_collection(policy)
        self.policy_history.append(policy);self.recent=[]
        while self.phase.phase=='collection' or self.mapper and self.mapper.builder.pending is not None:
            if self.state is None:self.reset('original',policy)
            self.mapper.behavior=policy
            if self.learner.policy_id()!=policy:raise ContractError('Policy changed inside collection')
            action=self.learner.action(self.state['preview']['features']) if self.state.get('preview') else None
            row,result=self.step(action,'iterative')
            if action is not None:self.phase.collected(policy)
            if row:
                index=self.replay.add(row);self.recent.append(index)
                self.mapping.append(dict(self.mapper.mapping[-1],replay_index=index))
            if result['terminal']:
                self.replay.finish_episode(self.cursor.episode_id,'success' if result['success'] else 'failure')
                self.clear_runtime()
            if self.phase.phase=='updates' and (self.mapper is None or self.mapper.builder.pending is None):break
        if len(self.recent)!=256:raise ContractError('Every full block needs exactly256 mapped decisions')
        ids=list(self.recent)
        for category in ('back_pre_holding','baseline_preservation'):
            groups={}
            for index in self.bootstrap_indices:
                row=self.replay.rows[index]
                if row.category==category:groups.setdefault(row.episode,[]).append(index)
            episodes=sorted(groups)
            if not episodes:raise ContractError('Missing fixed bootstrap witness stratum')
            for _ in range(64):
                rows=groups[episodes[self.replay.rng.randrange(len(episodes))]]
                ids.append(rows[self.replay.rng.randrange(len(rows))])
        features=[self.replay.rows[i].features for i in ids]
        self.witness_indices=ids
        self.witness=FixedWitness(features,self.learner.gaussians(features),policy)

    def updates(self):
        self.optimizer_gate()
        self.enter('updates',self.plan.updates_seconds)
        # Runtime-free boundaries deliberately make no state/checkpoint calls.
        before=self.call({'op':'state'}) if self.cursor.active else None
        ledger_before=self.ledger()
        guard=UpdateGuard(self.learner,self.replay,self.phase,self.witness)
        for _ in range(64):
            self.check_time();began=self.clock.now()
            _,_,accepted=guard.attempt(lambda:self.learner.batch(self.replay))
            self.report['accepted' if accepted else 'rejected']+=1;self.clock.work('joint');self.report['costs']['learningSeconds']+=self.clock.now()-began;self.check_time()
        if self.cursor.active and self.call({'op':'state'})!=before:
            raise ContractError('Optimizer pause advanced controller state')
        if self.ledger()!=ledger_before:raise ContractError('Optimizer changed physical ledger')
        self.phase.finish_block();self.cursor.completed_cycles=self.phase.cycle

    def evaluate(self):
        if len(self.policy_history)<2 or self.policy_history[0]==self.policy_history[1]:
            raise ContractError('No fresh collection after accepted updated actor')
        if self.phase.phase!='finished':raise ContractError('No completed admitted candidate')
        self.enter('evaluation',self.plan.evaluation_end-self.plan.work_end,
            self.plan.evaluation_integrations,evaluation=True)
        head=self.learner.actor.exported()
        for index in self.recent:
            features=self.replay.rows[index].features
            result=self.call({'op':'headparity','head':head,'features':features})
            if result['physicsIntegrations']!=0 or max(abs(a-b) for a,b in zip(
                    result['correction'],self.learner.action(features,deterministic=True)))>2e-6:
                raise ContractError('Deterministic head export parity')
        results=[]
        for case in ('original','heldout','belly'):
            self.reset(case,self.learner.policy_id(),head=head)
            while True:
                before=self.state;result=self.call({'op':'step'},physical=True);self.state=result['state']
                if before.get('preview'):
                    applied=self.state['lastCorrection']
                    expected=self.learner.action(before['preview']['features'],deterministic=True)
                    if applied['native_substep']!=before['native'] or max(abs(a-b) for a,b in
                            zip(applied['correction'],expected))>2e-6:
                        raise ContractError('Final applied head differs')
                if result['terminal']:break
            receipt={'case':case,'success':result['success'],'failure':result['failure']}
            if result['success']:
                first=self.call({'op':'standingReceipt'})
                receipt['hold']=self.call({'op':'hold','firstSuccessReceipt':first},
                    physical=True,upper_bound=29020)
                self.state=self.call({'op':'state'})
            results.append(receipt)
            if not receipt['success'] or not receipt.get('hold',{}).get('passed'):break
        passed=len(results)==3 and all(x['success'] and x.get('hold',{}).get('passed') for x in results)
        self.report.update(final=results,physicalQualificationPassed=passed and not self.synthetic,
            syntheticEvaluationPassed=passed and self.synthetic,interleavedCadenceVerified=True)
        # Evaluation is not replay; its raw rows are explicitly excluded.
        covered={i for x in self.mapping for i in range(x['transition_line_start'],x['transition_line_end']+1)}
        self.excluded_lines=sorted(set(range(len(self.raw_lines())))-covered)
        self.mapper=None

    def admit_sequence(self,**prefix):
        return self.plan.admit_sequence(self.clock.now(),self.ledger(),
            **self.recovery.storage_usage(),**prefix)

    def run(self):
        try:
            self.cursor.planned_blocks=self.admit_sequence()
            self.parity();self.bootstrap();self.backup('bootstrap-before-any-optimizer-update')
            self.warmup();self.backup('critic-warmup-complete')
            while self.phase.phase=='ready_collection':
                # Admission covers all remaining meaningful cadence+backups,
                # not just a cheap next update with expensive retention omitted.
                remaining=self.phase.planned_blocks-self.phase.cycle
                if self.clock.now()+self.plan.sequence_seconds(remaining,bootstrap=False,warmup=False)>self.plan.work_end:
                    raise ContractError('Remaining interleaved sequence no longer fits')
                usage=self.recovery.storage_usage();backups=remaining*2+1
                if (usage['cumulative_uploaded_bytes']+backups*self.plan.backup_archive_envelope_bytes>160*1024**2
                        or usage['retained_payload_bytes']+(1 if usage.get('replacement_slots') else backups)*self.plan.backup_payload_envelope_bytes>160*1024**2):
                    raise ContractError('Remaining cadence backup byte envelopes no longer fit')
                self.collect();self.backup(f'collection-{self.phase.cycle:02d}')
                self.updates();self.backup(f'updates-{self.phase.cycle:02d}')
            self.evaluate();self.report['stopped']='single-candidate-complete-no-promotion'
        except BaseException as error:
            self.failure_latched=True;self.report.update(stopped='safe-stop',error=str(error))
        finally:
            try:self.backup('terminal-stop',retention=True)
            except BaseException as error:self.report['terminalRetentionError']=str(error)
            try:self.report['ledger']=self.bridge.call({'op':'ledger'})
            except BaseException:
                path=self.output/'budget-ledger.json'
                self.report['ledger']=json.loads(path.read_text()) if path.exists() else {'unavailable':True}
            self.report['policyHistory']=self.policy_history
            atomic_json(self.output/'phase-report.json',self.report)
        return self.report

    def write_snapshot(self, boundary):
        """One paused generation, no immutable/model/build-input duplication."""
        import uuid
        nonce=uuid.uuid4().hex
        rows=list(self.replay.rows if self.replay else self.bootstrap_rows)
        mapping=list(self.mapping)
        if self.replay is None and self.mapper:
            known={(row.episode,row.pre_native,row.post_native) for row in rows}
            for row,entry in zip(self.mapper.rows,self.mapper.mapping):
                if (row.episode,row.pre_native,row.post_native) not in known:
                    mapping.append(dict(entry,replay_index=len(rows)));rows.append(row)
        ledger=self.durable_ledger()
        unresolved=bool(ledger.get('pendingCall') or ledger.get('uncertainCalls') or ledger.get('uncertainIntegrations',0))
        runtime_active=self.cursor.active and not unresolved
        runtime_receipt=None;bridge_state=None;runtime_capture_error=None
        if runtime_active:
            try:
                current=self.call({'op':'state'})
                if current!=self.state:raise ContractError('Cached state differs before snapshot')
                runtime_receipt=self.call({'op':'checkpoint','name':'phase-runtime.bin'})
                bridge_state=self.call({'op':'bridgeState'})
            except BaseException as error:
                if boundary!='terminal-stop':raise
                runtime_active=False;runtime_capture_error=str(error)
        mapper=None
        if self.mapper:
            index={(r.episode,r.pre_native,r.post_native):i for i,r in enumerate(rows)}
            indices=[index[(r.episode,r.pre_native,r.post_native)] for r in self.mapper.rows]
            mapper={key:encoded(getattr(self.mapper,key)) for key in
                ('episode','behavior','case','pending_origin','raw_calls','first_raw_line',
                 'pending_line','mapping','finished','synthetic')}
            mapper.update(row_indices=indices,pending_transition=encoded(self.mapper.builder.pending))
        self.learner.torch.save({'snapshot_id':nonce,'identity':self.identity,
            'state':self.learner.capture_learning_state()},self.output/'phase-learner.pt')
        arrays={key:self.learner.np.asarray([getattr(row,key) for row in rows],dtype='float32').reshape(
            len(rows),21 if key=='correction' else 1024) for key in ('features','correction','next_features')}
        self.learner.np.savez_compressed(self.output/'phase-replay.npz',snapshot_id=nonce,**arrays)
        metadata=[{k:v for k,v in asdict(row).items() if k not in arrays} for row in rows]
        atomic_json(self.output/'phase-replay-metadata.json',{'snapshot_id':nonce,'rows':metadata})
        atomic_json(self.output/'phase-mapping.json',{'snapshot_id':nonce,'mapping':mapping})
        if not (self.output/'transitions.ndjson').exists():(self.output/'transitions.ndjson').write_text('')
        witness=None
        if self.witness:
            witness={'indices':self.witness_indices,'reference':[asdict(x) for x in self.witness.reference],
                'policy_id':self.witness.policy_id,'sha256':self.witness.sha256}
        cursor={'schema':'lh-cold-coherent-cursor/v1','snapshot_id':nonce,'boundary':boundary,
            'identity':self.identity,'synthetic':self.synthetic,'plan':asdict(self.plan),
            'phase_cursor':asdict(self.cursor),'loop_phase':vars(self.phase) if self.phase else None,
            'state':self.state,'runtime_active':runtime_active,'runtime_omitted_unresolved':unresolved,
            'runtime_capture_error':runtime_capture_error,
            'runtime_sha256':runtime_receipt['sha256'] if runtime_receipt else None,'bridge':bridge_state,
            'ledger':ledger,'policy_id':self.learner.policy_id(),'policy_history':self.policy_history,
            'mapper':mapper,'replay_exists':self.replay is not None,
            'replay_capacity':self.replay.capacity if self.replay else None,
            'replay_diagnostic_only':self.replay.diagnostic_only if self.replay else False,
            'replay_groups':self.replay.groups if self.replay else None,
            'replay_rng':encoded(self.replay.rng.getstate()) if self.replay else None,
            'bootstrap_indices':self.bootstrap_indices,'excluded_raw_lines':self.excluded_lines,
            'raw_sha256':sha256(self.output/'transitions.ndjson'),'recent':self.recent,'witness':witness,
            'failure_latched':self.failure_latched,'automatic_restart_authorized':False}
        atomic_json(self.output/'phase-cursor.json',cursor)
        atomic_json(self.output/'phase-snapshot-report.json',{'snapshot_id':nonce,'report':self.report})
        atomic_json(self.output/'phase-ledger.json',ledger)
        roles={'learner':'phase-learner.pt','replay':'phase-replay.npz',
            'replay_metadata':'phase-replay-metadata.json','mapping':'phase-mapping.json',
            'cursor':'phase-cursor.json','report':'phase-snapshot-report.json',
            'ledger':'phase-ledger.json','transitions':'transitions.ndjson'}
        if runtime_active:roles['runtime']='phase-runtime.bin'
        eligible=self.replay is not None and self.cursor.transition_parity and not unresolved and not runtime_capture_error and not self.failure_latched
        return roles,cursor,eligible


@dataclass
class RestoredPilotState:
    learner: object
    replay: object
    mapper: object
    controller: object
    cursor: dict
    rows: list
    mapping: list
    phase: object
    witness: object


def restore_coherent(restored, manifest, *, expected_identity, expected_plan, expected_ledger,
                     learner_factory, builder_factory, controller_factory, synthetic):
    """Construct fresh objects from one exact generation; never mutate a live pilot."""
    files={entry['role']:Path(restored)/entry['path'] for entry in manifest['files']}
    cursor=json.loads(files['cursor'].read_text());nonce=cursor['snapshot_id']
    if (cursor['identity']!=expected_identity or cursor['plan']!=asdict(expected_plan)
            or cursor['synthetic'] is not synthetic or manifest['synthetic'] is not synthetic
            or manifest['config'].get('cold_start',manifest['config'])['identity']!=expected_identity
            or manifest['config'].get('cold_start',manifest['config'])['snapshot_id']!=nonce):
        raise ContractError('Source/artifact/mode/generation identity differs')
    ledger=json.loads(files['ledger'].read_text())
    if (ledger!=cursor['ledger'] or ledger!=expected_ledger
            or ledger['deadlineUnixMs']!=round(expected_plan.deadline*1000)
            or ledger.get('pendingCall') or ledger.get('uncertainCalls')
            or ledger.get('uncertainIntegrations',0)):
        raise ContractError('Mixed, reduced, renewed or uncertain ledger')
    if cursor.get('automatic_restart_authorized') is not False:
        raise ContractError('Archive cannot authorize an automatic retry')
    learner=learner_factory()
    learning=learner.torch.load(files['learner'],weights_only=False,map_location='cpu')
    if learning['snapshot_id']!=nonce or learning['identity']!=expected_identity:
        raise ContractError('Mixed learner generation')
    learner.restore_learning_state(learning['state'])
    assert_state_equal(learning['state'],learner.capture_learning_state())
    if learner.policy_id()!=cursor['policy_id']:raise ContractError('Restored policy identity differs')
    arrays=learner.np.load(files['replay'],allow_pickle=False)
    metadata=json.loads(files['replay_metadata'].read_text())
    lineage=json.loads(files['mapping'].read_text())
    if arrays['snapshot_id'].item()!=nonce or metadata['snapshot_id']!=nonce or lineage['snapshot_id']!=nonce:
        raise ContractError('Mixed replay/mapping generation')
    if sha256(files['transitions'])!=cursor['raw_sha256']:
        raise ContractError('Raw transition lineage differs')
    raw=[json.loads(line) for line in files['transitions'].read_text().splitlines()]
    tensor_arrays={key:arrays[key] for key in ('features','correction','next_features')}
    class Arrays(dict):
        @property
        def files(self):return list(self)
    audit=verify_replay_mapping(Arrays(tensor_arrays),metadata['rows'],lineage['mapping'],raw,
                                __import__('math').exp(-.025/25))
    pending=set()
    if cursor['mapper'] and cursor['mapper']['pending_line'] is not None:
        data=cursor['mapper'];pending=set(range(data['pending_line'],
                                              data['first_raw_line']+data['raw_calls']))
    excluded=set(cursor['excluded_raw_lines'])
    if excluded&pending or sorted(excluded|pending)!=audit['unmappedRawLines']:
        raise ContractError('Unmapped raw execution lacks exact exclusion/pending lineage')
    for i in excluded:
        if raw[i].get('training') and (i+1>=len(raw) or raw[i]!=raw[i+1]
                or not cursor['phase_cursor']['transition_parity']):
            raise ContractError('Executed source decision was silently excluded')
    rows=[]
    allowed=set(cursor['policy_history'])|{'baseline-deterministic-zero',cursor['policy_id']}
    for i,meta in enumerate(metadata['rows']):
        row=Record(**meta,**{key:tuple(arrays[key][i].tolist()) for key in tensor_arrays}).validate()
        if row.source!=('synthetic_rehearsal' if synthetic else 'fresh_wasm') or row.behavior not in allowed:
            raise ContractError('Replay mode or policy provenance differs')
        rows.append(row)
    replay=None
    if cursor['replay_exists']:
        replay=EpisodeReplay(rows,fresh_allowance=0,diagnostic_only=cursor.get('replay_diagnostic_only',False))
        if replay.diagnostic_only and cursor.get('diagnostic_membership')!=diagnostic_membership(rows,lineage['mapping']):
            raise ContractError('Diagnostic case/completion/membership differs')
        if cursor['replay_capacity']<len(rows):raise ContractError('Restored replay capacity reduced')
        replay.capacity=cursor['replay_capacity'];replay.rng.setstate(tuples(cursor['replay_rng']))
        if encoded(replay.groups)!=cursor['replay_groups']:
            raise ContractError('Replay groups differ from reconstructed episode strata')
    mapper=None
    if cursor['mapper']:
        data=cursor['mapper']
        mapper=FreshMapper(builder_factory(),data['episode'],data['behavior'],data['case'],
            data['first_raw_line'],synthetic=synthetic)
        for key in ('pending_origin','raw_calls','pending_line','mapping','finished'):
            setattr(mapper,key,data[key])
        mapper.rows=[rows[i] for i in data['row_indices']]
        mapper.builder.pending=data['pending_transition']
        if (len(mapper.mapping)!=len(mapper.rows) or mapper.raw_calls<0
                or mapper.first_raw_line+mapper.raw_calls>len(raw)
                or bool(mapper.pending_origin)!=bool(mapper.builder.pending)
                or (mapper.pending_line is not None and not mapper.first_raw_line<=mapper.pending_line<len(raw))):
            raise ContractError('Pending mapper/cursor lineage is incoherent')
        if pending:
            segment=[raw[i] for i in sorted(pending)];origin,action=mapper.pending_origin
            values=learner.np.asarray(origin['preview']['features'],dtype='<f4')
            reward=sum(builder_factory().gamma**i*x['reward'] for i,x in enumerate(segment))
            if (hashlib.sha256(values.tobytes()).hexdigest()!=segment[0]['sourceFeaturesSha256']
                    or segment[0]['preNative']!=origin['native']
                    or not learner.np.array_equal(learner.np.asarray(action,'float32'),
                                                  learner.np.asarray(segment[0]['correction'],'float32'))
                    or any(x['episodeId']!=mapper.episode or x['behaviorPolicyId']!=mapper.behavior for x in segment)
                    or abs(reward-mapper.builder.pending[3])>1e-10
                    or len(segment)!=mapper.builder.pending[5]):
                raise ContractError('Pending source features/action/folded reward differs from raw execution')
    phase=None
    if cursor['loop_phase']:
        data=cursor['loop_phase'];phase=ColdPhase(data['bootstrap_decisions'],data['planned_blocks'])
        if data.get('pending_attempt'):raise ContractError('Unresolved optimizer attempt')
        phase.__dict__.update(data)
    witness=None
    if cursor['witness']:
        data=cursor['witness']
        witness=FixedWitness([rows[i].features for i in data['indices']],
            [Gaussian(tuple(x['mean']),tuple(x['log_std'])) for x in data['reference']],data['policy_id'])
        if witness.sha256!=data['sha256']:raise ContractError('Fixed collecting witness changed')
    if cursor['runtime_active']:
        if 'runtime' not in files or sha256(files['runtime'])!=cursor['runtime_sha256']:
            raise ContractError('Mixed runtime generation')
        controller=controller_factory(files['runtime'],cursor['bridge'],cursor['state'],ledger)
    else:
        if 'runtime' in files or cursor['state'] is not None:
            raise ContractError('Runtime-free boundary contains a stale active state')
        controller=controller_factory(None,None,None,ledger)
    return RestoredPilotState(learner,replay,mapper,controller,cursor,rows,lineage['mapping'],phase,witness)


class CoherentRecovery:
    """Same serialization/restore transaction for real Drive and local rehearsal."""
    def __init__(self, transport, controller_factory, *, synthetic):
        self.transport=transport;self.controller_factory=controller_factory;self.synthetic=synthetic
        self.last_restored=None

    def storage_usage(self):return self.transport.storage_usage()

    def checkpoint(self,pilot,boundary,deadline):
        before=pilot.learner.capture_learning_state()
        began=pilot.clock.now()
        roles,cursor,eligible=pilot.write_snapshot(boundary)
        pilot.report['costs']['serializationSeconds']+=pilot.clock.now()-began
        artifacts=[artifact(role,pilot.output/name,'cold-coherent/v1') for role,name in roles.items()]
        if sum(Path(x['path']).stat().st_size for x in artifacts)>pilot.plan.backup_payload_envelope_bytes:
            raise ContractError('Measured generation exceeds predeclared cadence payload envelope')
        if self.synthetic:
            for item in artifacts:item['provenance']='DISPOSABLE SOFTWARE REHEARSAL; not physical experience'
        config={'identity':pilot.identity,'snapshot_id':cursor['snapshot_id'],
            'plan':asdict(pilot.plan),'synthetic':self.synthetic,'resumeAuthorized':eligible}
        diagnostic_restore=bool(getattr(pilot,'diagnostic_restore_required',False))
        def validate(restored,manifest):
            if eligible or diagnostic_restore:
                state=restore_coherent(restored,manifest,expected_identity=pilot.identity,
                    expected_plan=pilot.plan,expected_ledger=cursor['ledger'],
                    learner_factory=pilot.learner_factory,builder_factory=pilot.builder_factory,
                    controller_factory=self.controller_factory,synthetic=self.synthetic)
                if (state.learner is pilot.learner or state.replay is pilot.replay
                        or state.mapper is not None and state.mapper is pilot.mapper):
                    raise ContractError('Independent reconstruction reused a live object')
                assert_state_equal(before,state.learner.capture_learning_state())
                if diagnostic_restore:
                    if not state.replay.diagnostic_only:
                        raise ContractError('Qualification replay lost its nontrainable status')
                    live_rng=pilot.replay.rng.getstate()
                    try:
                        if state.replay.diagnostic_sample_indices(16)!=pilot.replay.diagnostic_sample_indices(16):
                            raise ContractError('Fresh diagnostic membership/sample/RNG differs')
                    finally:pilot.replay.rng.setstate(live_rng)
                self.last_restored=state
            else:
                # Terminal partial evidence is useful but cannot become a resume gate.
                files={entry['role']:Path(restored)/entry['path'] for entry in manifest['files']}
                if (json.loads(files['cursor'].read_text())!=cursor
                        or json.loads(files['ledger'].read_text())!=cursor['ledger']):
                    raise ContractError('Partial terminal diagnostic restoration differs')
                if manifest['application_resume_eligible'] is not False:
                    raise ContractError('Incomplete terminal state claimed resumable')
            return {'passed':True,'physics_integrations':0,'coherent':eligible or diagnostic_restore,'synthetic':self.synthetic}
        began=pilot.clock.now()
        try:
            proof=self.transport.checkpoint(artifacts,config,boundary,deadline,validate,eligible)
            return dict(proof,synthetic=self.synthetic,coherent=eligible or diagnostic_restore)
        finally:
            pilot.report['costs']['transportRestoreSeconds']+=pilot.clock.now()-began
            # Even a rejected fresh restore must not reset the live global RNG.
            pilot.learner.restore_learning_state(before)


class DrivePhaseTransport:
    """Keep the existing operator-mediated official transfer handshake alive."""
    synthetic=False
    def __init__(self,exchange,transfer,clock):
        self.exchange=exchange;self.transfer=transfer;self.clock=clock;self.failed=False
        self.synthetic=exchange.synthetic

    def storage_usage(self):
        return {'cumulative_uploaded_bytes':self.exchange.remote_uploaded_bytes,
            'replacement_slots':hasattr(self.exchange,'pointer'),
            'retained_payload_bytes':self.exchange.shared_bytes+self.exchange.seed_bytes+
                self.exchange.retired_payload_bytes+sum(max(x['payload_bytes'],17*1024**2) if hasattr(self.exchange,'pointer')
                else x['payload_bytes'] for x in self.exchange.verified)}

    def checkpoint(self,artifacts,config,boundary,deadline,validator,eligible):
        if self.failed:raise ContractError('Failed/uncertain transfer; no retry')
        try:
            request=self.exchange.snapshot(artifacts,config,boundary,
                wall_reserve=self.exchange.deadline-deadline)
            details=json.loads(request.read_text())
            if details['archive_bytes']>config['plan']['backup_archive_envelope_bytes']:
                raise ContractError('Measured archive exceeds predeclared cadence envelope before transfer')
            self.transfer(request,validator)
            if self.clock.now()>=deadline:raise ContractError('Backup exceeded its full phase allowance')
        except BaseException:
            self.failed=True
            raise
        value=json.loads(request.read_text())
        return {'passed':True,'archive_bytes':value['archive_bytes'],
            'manifest_sha256':value['manifest_sha256'],'boundary':boundary,
            'actualRemoteWrite':not self.synthetic}
