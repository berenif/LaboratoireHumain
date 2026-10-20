"""Disposable ports for SOFTWARE tests of the production ColdPilot path.

Never loads WASM, starts a real trial clock, contacts Drive, or produces a
physical candidate. The learner/mapper/replay/archive implementation is real.
"""
from __future__ import annotations
import copy
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil

from experiment_backup import create_snapshot, restore_bundle, sha256, verify_bundle
from getup_learning_loop import ContractError
from getup_cold_start_engine import PhasePlan,DrivePhaseTransport
from getup_cold_start import DriveExchange
from experiment_backup import atomic_json

EXISTING_REMOTE_BYTES=16167158
EXISTING_IMMUTABLE_PAYLOAD=18052670


class ScriptClock:
    def __init__(self,now=1900000000.):
        self.value=now
    def now(self):return self.value
    def advance(self,seconds):self.value+=seconds
    def work(self,kind):self.advance(.005 if kind=='critic' else .01)
    def sleep(self,seconds):
        self.advance(seconds)
        if hasattr(self,'on_sleep'):self.on_sleep()


class ScriptedPhysics:
    synthetic=True
    def __init__(self,output,clock,plan,*,episode_decisions=32,forecast_per_reset=0,
                 fail_call=None,malformed_call=None,empty_holding=False,pending_tail=False):
        self.output=Path(output);self.output.mkdir(parents=True,exist_ok=True)
        self.clock=clock;self.plan=plan;self.deadline=plan.deadline
        self.episode_decisions=episode_decisions;self.forecast_per_reset=forecast_per_reset
        self.fail_call=fail_call;self.malformed_call=malformed_call
        self.empty_holding=empty_holding;self.pending_tail=pending_tail
        self.state=None;self.index=0;self.case=None;self.label=None;self.head=None
        self.previous=[0.]*21;self.calls=[];self.physical_calls=0;self.raw=[]
        self.ledger={'schema':1,'synthetic':True,'deadlineUnixMs':round(plan.deadline*1000),
            'nativeIntegrations':0,'realIntegrations':0,'forecastIntegrations':0,
            'trainingDecisions':0,'bootstrapDecisions':0,'iterativeDecisions':0,
            'completedCalls':0,'uncertainCalls':[],'uncertainIntegrations':0,'limit':300000}

    def features(self):
        import numpy as np
        return np.asarray([((self.index+k)%8-4)/8 for k in range(1024)],dtype='float32').tolist()

    def current(self,preview=True,action=None,pre_native=None):
        return {'native':340+20*self.index,'forecast':self.forecast_per_reset,
            'impactEpoch':340,'sourceEpoch':340,'status':{'reference_frames':2,'phase':'script'},
            'preview':{'features':self.features(),'observation':[0.]*118,
                       'referenceDecision':0 if self.empty_holding else self.index} if preview else None,
            'lastCorrection':{'native_substep':pre_native,'correction':action} if action is not None else None,
            'current':{'synthetic':True}}

    def charge(self,real=20,forecast=0):
        self.physical_calls+=1
        if self.ledger.get('pendingCall'):raise ContractError('No retry after unresolved synthetic command')
        self.ledger['pendingCall']={'chargedUpperBound':5020}
        self.ledger['nativeIntegrations']+=5020;self.ledger['uncertainIntegrations']+=5020
        if self.physical_calls==self.fail_call:
            self.ledger['uncertainCalls'].append({'chargedUpperBound':5020})
            atomic_json(self.output/'budget-ledger.json',self.ledger)
            raise ContractError('scripted uncertain execution')
        self.ledger['nativeIntegrations']+=real+forecast-5020
        self.ledger['uncertainIntegrations']-=5020
        self.ledger['realIntegrations']+=real;self.ledger['forecastIntegrations']+=forecast
        self.ledger['completedCalls']+=1;del self.ledger['pendingCall']
        self.clock.advance(.01)

    def checkpoint(self):
        return {'state':self.state,'index':self.index,'case':self.case,'label':self.label,
            'head':self.head,'previous':self.previous,'episode_decisions':self.episode_decisions,
            'forecast_per_reset':self.forecast_per_reset,'empty_holding':self.empty_holding,
            'pending_tail':self.pending_tail}

    def load(self,payload,bridge,ledger=None):
        for key,value in payload.items():setattr(self,key,copy.deepcopy(value))
        self.previous=copy.deepcopy(bridge['previous'])
        if ledger is not None:self.ledger=copy.deepcopy(ledger)

    def head_action(self,head,features):
        import numpy as np
        return np.tanh(np.asarray(head['weights'],dtype='float32').reshape(21,1024)
                       @np.asarray(features,dtype='float32')+np.asarray(head['bias'],dtype='float32')).tolist()

    def call(self,q):
        op=q['op'];self.calls.append(op)
        if op=='ledger':return copy.deepcopy(self.ledger)
        if op=='reserve':
            self.ledger.update(evaluationReserveIntegrations=q['integrations'],
                               evaluationReserveWallSeconds=q['wallSeconds'])
            return {}
        if op=='coldDrop':self.state=None;return {'physicsIntegrations':0}
        if op=='state':
            if self.state is None:raise ContractError('Runtime-free state call')
            return copy.deepcopy(self.state)
        if op=='bridgeState':
            if self.state is None:raise ContractError('Runtime-free bridge call')
            return {'previous':copy.deepcopy(self.previous),'label':self.label,'synthetic':True}
        if op=='checkpoint':
            if self.state is None:raise ContractError('Runtime-free checkpoint call')
            path=self.output/q['name'];path.write_text(json.dumps(self.checkpoint(),sort_keys=True)+'\n')
            return {'sha256':sha256(path),'bytes':path.stat().st_size}
        if op=='coldRestore':
            if sha256(q['path'])!=q['sha256']:raise ContractError('Synthetic checkpoint hash mismatch')
            self.load(json.loads(Path(q['path']).read_text()),q['bridge'])
            return {'physicsIntegrations':0,'state':copy.deepcopy(self.state)}
        if op=='reset':
            self.case=q['case'];self.label=q['label'];self.head=q.get('head')
            self.index=0;self.previous=[0.]*21
            self.charge(forecast=self.forecast_per_reset)
            self.state=self.current()
            return copy.deepcopy(self.state)
        if op=='headparity':
            return {'physicsIntegrations':0,'correction':self.head_action(q['head'],q['features'])}
        if op=='standingReceipt':return {'native':self.state['native'],'stableNative':800,'anchors':'synthetic'}
        if op=='hold':
            self.charge(real=20)
            return {'passed':True,'synthetic':True,'longestMeasuredInterval':800}
        if op!='step':raise ContractError('Unsupported scripted bridge operation: '+op)
        if self.state is None:raise ContractError('No synthetic source state')
        pre=copy.deepcopy(self.state)
        action=q.get('c')
        if self.head is not None and pre.get('preview'):action=self.head_action(self.head,pre['preview']['features'])
        if action is None:action=[0.]*21
        self.charge();self.index+=1
        target=3 if self.case!='original' or self.head is not None else (
            4 if q.get('collectionPhase')=='bootstrap' else self.episode_decisions)
        tail=self.pending_tail and self.index>=target and self.head is None
        terminal=self.index>target if tail else self.index>=target
        self.state=self.current(preview=not terminal and not tail,action=action,pre_native=pre['native'])
        reward=1. if terminal else .1
        feature_hash=lambda values:hashlib.sha256(__import__('numpy').asarray(
            values if values is not None else [0.]*1024,dtype='<f4').tobytes()).hexdigest()
        raw={'episodeId':q.get('episodeId',self.label),'behaviorPolicyId':q.get('policyId'),
            'preNative':pre['native'],'postNative':self.state['native'],'impactEpoch':340,
            'preObservation':pre['preview']['observation'] if pre.get('preview') else None,
            'correction':action,'reward':reward,'terminal':terminal,
            'sourceFeaturesSha256':feature_hash(pre['preview']['features'] if pre.get('preview') else None),
            'nextFeaturesSha256':feature_hash(self.state['preview']['features'] if self.state.get('preview') else None),
            'training':q.get('training') is True,'synthetic':True}
        self.raw.append(raw)
        with (self.output/'transitions.ndjson').open('a') as stream:stream.write(json.dumps(raw,sort_keys=True)+'\n')
        if q.get('training'):
            self.ledger['trainingDecisions']+=1
            self.ledger[q['collectionPhase']+'Decisions']+=1
        self.previous=action
        result={'state':copy.deepcopy(self.state),'reward':reward,'terminal':terminal,'success':terminal,
            'failure':None,'native':self.state['native'],'longestMeasuredInterval':800 if terminal else 0,
            'nativeTraceSha256':hashlib.sha256(json.dumps([pre['native'],self.state,action],sort_keys=True).encode()).hexdigest()}
        if self.physical_calls==self.malformed_call:return {'broken':True}
        return result

    def independent_controller(self,path,bridge,state,ledger):
        fresh=ScriptedPhysics(self.output/'independent-controller',ScriptClock(self.clock.now()),self.plan)
        if path is None:
            if state is not None:raise ContractError('Runtime-free restore has state')
            fresh.ledger=copy.deepcopy(ledger)
        else:
            fresh.load(json.loads(Path(path).read_text()),bridge,ledger)
            if fresh.call({'op':'state'})!=state:raise ContractError('Fresh controller state differs')
        return fresh


class LocalRehearsalTransport:
    """Actual DriveExchange/DrivePhaseTransport; ONLY external transfer is scripted.

    Synthetic mode is explicit in archive/request/receipt/application marker.
    Such receipts can never satisfy a real exchange. No network occurs.
    """
    synthetic=True
    def __init__(self,root,clock,*,delay=1.,fail_boundary=None,initial_cumulative=EXISTING_REMOTE_BYTES):
        self.root=Path(root);self.root.mkdir(parents=True,exist_ok=True);self.clock=clock
        self.delay=delay;self.fail_boundary=fail_boundary;self.failed=False
        self.cumulative=initial_cumulative;self.retained=[];self.sizes=[];self.requests=[];self.after_verified=None
        self.exchange=DriveExchange(self.root,'software-rehearsal',
            {'commit':'a'*40,'dirty':False,'description':'DISPOSABLE SOFTWARE REHEARSAL'},
            clock.now()+3600,shared_sha256='b'*64,clock=clock,synthetic=True)
        self.exchange.shared_bytes=EXISTING_IMMUTABLE_PAYLOAD
        self.exchange.remote_uploaded_bytes=initial_cumulative
        self.exchange.seed_sha256='c'*64
        self.port=DrivePhaseTransport(self.exchange,self.transfer,clock)
        self.clock.on_sleep=self.acknowledge_prune

    def acknowledge_prune(self):
        for path in self.root.glob('*.prune-request.json'):
            request=json.loads(path.read_text());receipt=Path(request['receipt'])
            if receipt.exists():continue
            application=json.loads(Path(request['application_verified']).read_text())
            if application.get('passed') is not True or application.get('synthetic') is not True:
                raise ContractError('Prune cannot precede synthetic application restore')
            atomic_json(receipt,{'synthetic':True,'replacement_manifest_sha256':request['replacement_manifest_sha256'],
                'deleted_drive_file_ids':request['drive_file_ids']})

    def transfer(self,request_path,validator):
        request=json.loads(request_path.read_text());archive=Path(request['archive_path'])
        downloaded=self.root/'download.zip';shutil.copyfile(archive,downloaded)
        self.cumulative+=request['archive_bytes'];self.clock.advance(self.delay)
        if request['boundary']==self.fail_boundary or self.clock.now()>=request['deadline_unix']:
            downloaded.unlink();raise ContractError('Scripted backup failure/delay; no retry')
        receipt=request_path.with_name(request_path.name.replace('.request.json','.receipt.json'))
        atomic_json(receipt,{key:request[key] for key in
            ('run_id','kind','archive_sha256','archive_bytes','manifest_sha256')}|{
            'schema':'lh-private-drive-receipt/v1','synthetic':True,'owner_only':True,'shared':False,
            'download_completed':True,'downloaded_path':str(downloaded),
            'drive_file_id':'synthetic-local-only-'+request['manifest_sha256']})
        try:
            manifest=self.exchange.verify_receipt(request_path,receipt,validator)
            if self.after_verified:self.after_verified(archive,manifest,request['boundary'])
        finally:downloaded.unlink(missing_ok=True)

    def storage_usage(self):return self.port.storage_usage()

    def checkpoint(self,artifacts,config,boundary,deadline,validator,eligible):
        if self.failed:raise ContractError('Prior failed/uncertain backup; no automatic retry')
        self.requests.append(boundary)
        try:proof=self.port.checkpoint(artifacts,config,boundary,deadline,validator,eligible)
        except BaseException:
            self.failed=True;raise
        self.retained=[{'archive':Path(x['archive_path']),'payload':x['payload_bytes'],
            'manifest':x['manifest_sha256']} for x in self.exchange.verified]
        self.sizes.append(proof['archive_bytes'])
        return dict(proof,projectedCumulativeBytes=self.cumulative)


def make_plan(clock):
    start=clock.now()
    return PhasePlan(start,start+3600,start+2100,start+3300)
