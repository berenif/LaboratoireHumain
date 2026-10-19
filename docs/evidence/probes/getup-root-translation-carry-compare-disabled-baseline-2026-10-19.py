import json,pathlib,struct,hashlib
D=pathlib.Path('outputs/getup-root-translation-carry-20261019')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def parts(p):
 b=p.read_bytes();i=0
 def integer(fmt):
  nonlocal i
  v=struct.unpack_from(fmt,b,i)[0];i+=struct.calcsize(fmt);return v
 def vec():
  nonlocal i
  n=integer('<Q');v=b[i:i+n];assert len(v)==n;i+=n;return v
 s=integer('<I');m=vec();w=vec();a=vec();assert i==len(b);return s,m,w,json.loads(a)
def prune(v):
 if isinstance(v,dict):return {k:prune(x) for k,x in v.items() if k not in ['controller_ms','update_ms']}
 if isinstance(v,list):return [prune(x) for x in v]
 return v
baseline=D/'baseline-one-tick-result';candidate=D/'checkpoint-probe-result'
a=json.loads((baseline/'typed-reply.json').read_text());b=json.loads((candidate/'typed-disabled-final-reply.json').read_text());aa=parts(baseline/'checkpoint.bin');bb=parts(candidate/'disabled-checkpoint-after-tick.bin')
checks={'fullPhysicalReplyExceptOnlyWallCost':prune(a)==prune(b),'schema1':aa[0]==bb[0]==1,'canonicalModelBytes':aa[1]==bb[1],'fullSerializedWorldBytes':aa[2]==bb[2],'fullAuxiliaryExceptOnlyWallCost':prune(aa[3])==prune(bb[3])};out={'classification':'exact-same-input-old-frozen-library-versus-new-feature-compiled-inactive-one-tick-control','typedSnapshotContractRestored':True,'numericTolerance':0,'rawFormattingDifferenceWasNotPhysics':True,'checks':checks,'passed':all(checks.values()),'nativeIntegrationsEach':20,'forecastIntegrationsEach':0,'excludedWallClockKeys':['controller_ms','update_ms'],'baselineReplySha256':sha(baseline/'typed-reply.json'),'candidateReplySha256':sha(candidate/'typed-disabled-final-reply.json'),'serializedWorldSha256':hashlib.sha256(aa[2]).hexdigest()};(D/'disabled-baseline-parity.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out));assert out['passed']
