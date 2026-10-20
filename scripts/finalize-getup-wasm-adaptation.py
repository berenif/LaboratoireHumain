#!/usr/bin/env python3
"""Read-only audit and compact evidence for the single frozen WASM pilot."""
import argparse,datetime,hashlib,json,math,struct
from pathlib import Path
import numpy as np
import torch

def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def checkpoint(p):
    data=p.read_bytes();schema=struct.unpack_from('<I',data)[0];offset=4;fields=[]
    for _ in range(3):
        n=struct.unpack_from('<Q',data,offset)[0];offset+=8;fields.append(data[offset:offset+n]);offset+=n
    assert offset==len(data)
    aux=fields[2].decode()
    while schema in (3,4):
        wrap=json.loads(aux);schema=wrap['inner_schema'];aux=wrap['auxiliary']
    if schema==2:aux=json.loads(aux)['runtime']
    else:assert schema==1
    rt=json.loads(aux);sim=rt['simulation'];return sim,sim['getup_controller']
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--evidence',type=Path,required=True);a=p.parse_args();out=a.output;report=read(out/'pilot-report.json');config=read(out/'frozen-config.json')
    assert report['trainingDecisions']==8192 and report['criticOnlyUpdates']==256 and report['jointUpdates']==256
    updates=read(out/'updates.json');assert len(updates)==512 and all(all(math.isfinite(v) for v in row.values()) for row in updates)
    state=torch.load(out/'candidate-training.pt',map_location='cpu',weights_only=True)
    for name in ['actor','critic','target']:
        assert all(torch.isfinite(value).all() for value in state[name].values())
    replay=np.load(out/'replay.npz');assert len(replay['0'])==8361 and len(replay['success'])==169
    assert all(np.isfinite(replay[str(i)]).all() for i in range(5)) and np.max(np.abs(replay['1']))<=1
    rows=[json.loads(line) for line in (out/'transitions.ndjson').open()];training=[r for r in rows if r['label']=='training-original' and r['preObservation'] is not None]
    assert len(training)==8192
    max_residual_delta=0.
    for r in training:
        assert len(r['preObservation'])==118 and r['preObservation'][-1]==1 and len(r['correction'])==21
        assert all(math.isfinite(x) and -1<=x<=1 for x in r['correction'])
        delta=max(abs(a-b) for a,b in zip(r['appliedResidual'],r['originalResidual']));max_residual_delta=max(max_residual_delta,delta);assert delta<=.050001
    def metric(label):
        longest=tail=count=0
        for row in rows:
            if row['label']!=label:continue
            for word in row['intervalPredicateWords']:
                count+=1;tail=tail+1 if word==63 else 0;longest=max(longest,tail)
        return longest,tail,count
    comparisons=[];external=0
    for index,case in enumerate(['original','heldout']):
        pair=[]
        for name,source in [('baseline',report['baseline']),('candidate',report['final'])]:
            row=source[index];label=name+'-'+case;extension=out/'fixed25'/f'{label}-fixed25-summary.json'
            if extension.exists():
                d=read(extension);assert d['complete'] and d['endNative']==(480 if case=='original' else 560)+20000
                seed=metric(label);longest,tail=seed[:2];samples=0
                for line in (out/'fixed25'/f'{label}-continuation.ndjson').open():
                    r=json.loads(line)
                    for word in r['intervalPredicateWords']:
                        samples+=1;tail=tail+1 if word==63 else 0;longest=max(longest,tail)
                assert samples==d['nativeIntegrations']==d['endNative']-d['startNative'];assert d.get('forecastIntegrations',0)==0
                assert longest==d['longestMeasuredInterval'];external+=d['nativeIntegrations']
                sim,c=checkpoint(out/'fixed25'/f'{label}-fixed25-final.bin');assert sim['substeps_completed']==d['endNative'] and c['impact_substep']==(480 if case=='original' else 560)
                pair.append({'longestNativeSamples':longest,'passed800Gate':row['success'],'endpointNative':d['endNative'],'finalCheckpointSha256':sha(out/'fixed25'/f'{label}-fixed25-final.bin')})
            else:
                assert not row['success'] and row['failure']['check']=='original25s-deadline'
                m=metric(label);assert m[0]==row['longestMeasuredInterval'];pair.append({'longestNativeSamples':m[0],'passed800Gate':False,'endpointNative':row['native'],'finalCheckpointSha256':sha(out/(label+'-final.bin'))})
        comparisons.append({'case':case,'baseline':pair[0],'candidate':pair[1],'improvementNativeSamples':pair[1]['longestNativeSamples']-pair[0]['longestNativeSamples']})
    holds=[]
    for name,source in [('baseline',report['baseline']),('candidate',report['final'])]:
        row=source[2];assert row['success'] and row['failure'] is None and row['thirtySecondHold']['passed']
        first=next(r for r in rows if r['label']==name+'-belly' and r['success'] and r['terminal'])
        initial_count=round(first['status']['stable_standing_s']*800);sim,c=checkpoint(out/(name+'-belly-final.bin'))
        assert c['stable_substeps']-initial_count==24000 and sim['substeps_completed']-first['postNative']==24000
        holds.append({'label':name,'firstSuccessObservedNative':first['postNative'],'stableSamplesAtObservation':initial_count,'finalStableSamples':c['stable_substeps'],'holdNativeSamples':24000,'holdSeconds':30,'maximumOriginalFootOriginDriftM':row['thirtySecondHold']['maximumOriginDriftM'],'maximumAnchoredFootMaterialPointExcursionM':row['thirtySecondHold']['maximumMaterialSlipM'],'finalCheckpointSha256':sha(out/(name+'-belly-final.bin'))})
    ledger=read(out/'budget-ledger.json');total=ledger['nativeIntegrations']+external;assert total<=600000
    uncertainty=sum(r['chargedUpperBound'] for r in ledger['uncertainCalls']);assert ledger['realIntegrations']+ledger['forecastIntegrations']+uncertainty==ledger['nativeIntegrations']
    ended=datetime.datetime.now(datetime.timezone.utc);start=datetime.datetime.fromisoformat(ledger['startedAt'].replace('Z','+00:00'));elapsed=(ended-start).total_seconds();assert elapsed<3600
    physical=all(r['improvementNativeSamples']>=80 for r in comparisons)
    result={'schema':1,'date':'2026-10-20','experiment':'bounded-frozen-feature-actual-WASM-correction-v1','engineeringPassed':True,'physicalSignal':physical,'candidateAccepted':False,'releaseAccepted':False,'reason':'Original BACK has zero qualifying measured interval before and after; both-case80-native improvement criterion fails. No wider correction, gain, horizon, candidate selection or retry.','caps':config,'actual':{'trainingDecisions':8192,'replayEntries':8361,'successfulRehearsalEntries':169,'criticOnlyUpdates':256,'jointUpdates':256,'allUpdateMetricsAndParametersFinite':True,'maximumCollectedSameStateResidualChangeRad':max_residual_delta},'adapterParity':read(out/'adapter-parity.json'),'headExportParity':{'pureFeatureCases':32,'pureMaximumCorrectionError':report['pureExportParityMaxError'],'rolloutMaximumCorrectionError':report['exportParityMaxCorrectionError'],'tolerance':2e-6},'fixed25Comparisons':comparisons,'bellyNonRegression':holds,'budget':{'startedAt':ledger['startedAt'],'endedAt':ended.isoformat(),'elapsedWallSecondsIncludingEngineeringPausesAndAudit':elapsed,'limitWallSeconds':3600,'mainChargedIntegrations':ledger['nativeIntegrations'],'additionalBaselineIntegrations':14800,'additionalFinalCandidateIntegrations':16860,'actualRealIntegrations':ledger['realIntegrations']+external,'forecastIntegrations':ledger['forecastIntegrations'],'uncertainOutstandingConservativeCharge':uncertainty,'totalChargedIntegrations':total,'limitIntegrations':600000},'limitations':['Nearby heldout uses same initial BACK construction with continued passive contact settling to560 rather than480; no broad independent-fall generalization.','First20native source decision is original/uncorrected on every training and final branch; candidate head starts next40Hz boundary.','Successful BACK metrics originally stopped at first800success; retained same-world continuations correct both evaluations to fixed original25s endpoints. Original censored report is retained and superseded for acceptance.','Diagnostic interval does not replace all800-native standing predicates or30s anchor hold.','Material metric is maximum anchored excursion, separate from10mm foot-origin drift, not cumulative travel.','Single-cycle adapter only: existing completed-cycle reset drops the head; adapted repeated-fall lifecycle is unqualified.','Actual WASM under Node; no candidate browser/mobile/public qualification.','All trained weights, replay and checkpoints remain local; base-policy redistribution rights unresolved.'],'provenance':{'baseCommit':'cdb8c21b7ffc19f3c82c3bc5fb320f9681640cf8','candidateWasmSha256':report['identity']['candidateWasmSha256'],'frozenBaselineWasmSha256':report['identity']['baselineWasmSha256'],'candidateHeadLocalSha256':sha(out/'candidate-head.json'),'frozenConfigSha256':sha(out/'frozen-config.json'),'fixed25MetricCorrectionSha256':sha(out/'fixed25-evaluation-correction-freeze.json'),'rawPilotReportSha256':sha(out/'pilot-report.json'),'transitionsSha256':sha(out/'transitions.ndjson'),'updatesSha256':sha(out/'updates.json'),'replayLocalSha256':sha(out/'replay.npz'),'sourcePolicyChanged':False,'productionDefaultChanged':False}}
    a.evidence.parent.mkdir(parents=True,exist_ok=True);a.evidence.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'physicalSignal':physical,'totalChargedIntegrations':total,'comparisons':comparisons,'elapsedWallSeconds':elapsed},indent=2))
if __name__=='__main__':main()
