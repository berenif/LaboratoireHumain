"""Exclusive new-stage physics/interval accounting, independent of engine rewind.

Every call is one charged native integration. Timers bound cooperative C calls,
not a hard realtime promise. An unresolved call prevents restart. All elapsed
cost/possible integration is retained on interruption, before an exception escapes.
"""
import datetime as dt,fcntl,json,math,signal,time
from native_env import atomic_json


class StageBudget:
    def __init__(self,path):
        self.path=path;self.lock=(path.parent/'stage.lock').open('a')
        try:fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BaseException:self.lock.close();self.lock=None;raise
        self.data=json.loads(path.read_text())
        if self.data.get('requestOutstanding'):self.close();raise ValueError('unresolved source request: reconciliation required')
        self.deadline=dt.datetime.fromisoformat(self.data['deadlineUtc'].replace('Z','+00:00'));self.profile=self.data.get('profile','stabilization')
        if self.profile not in ['stabilization','analytic-diagnostic']:self.close();raise ValueError('unknown admitted budget profile')
        if self.profile=='analytic-diagnostic'and(self.data['aggregatePhysicsLimitSeconds']!=1 or self.data['validationWallReservedSeconds']!=0 or self.data['allIntervalLimit']!=64):self.close();raise ValueError('wrong diagnostic limits')
        for key in ['physicsSeconds','sourceIntegrations','totalPolicyIntervals','stagePhysicsSeconds','stagePolicyIntervals']:
            if key not in self.data:self.close();raise ValueError('missing admitted budget counters')
        if any(not math.isfinite(v)or v<0 for v in self.data['stagePhysicsSeconds'].values()):self.close();raise ValueError('invalid phase physics counters')
    def close(self):
        if getattr(self,'lock',None):fcntl.flock(self.lock,fcntl.LOCK_UN);self.lock.close();self.lock=None
    def __enter__(self):return self
    def __exit__(self,*_):self.close()
    def remaining_wall(self):return(self.deadline-dt.datetime.now(dt.timezone.utc)).total_seconds()
    def start_benchmark_clock(self):
        if 'benchmarkDeadlineUtc'not in self.data:
            now=dt.datetime.now(dt.timezone.utc);end=min(now+dt.timedelta(seconds=120),self.deadline-dt.timedelta(seconds=self.data['validationWallReservedSeconds']))
            self.data.update(benchmarkStartedUtc=now.isoformat(),benchmarkDeadlineUtc=end.isoformat());atomic_json(self.path,self.data)
        return self.data['benchmarkDeadlineUtc']
    def allowance(self,stage):
        if self.data.get('requestOutstanding'):raise ValueError('unresolved source request blocks admission')
        if stage not in ['benchmark','training','validation']:raise ValueError('unknown stage')
        d=self.data;diagnostic=self.profile=='analytic-diagnostic';limit=({'benchmark':1.,'training':0.,'validation':0.}if diagnostic else{'benchmark':1.,'training':6.,'validation':3.})[stage]
        reserved=0. if diagnostic or stage=='validation'else max(0.,3.-d['stagePhysicsSeconds']['validation'])
        wall_reserved=0. if stage=='validation'else d['validationWallReservedSeconds']
        result=min(limit-d['stagePhysicsSeconds'][stage],d['aggregatePhysicsLimitSeconds']-d['physicsSeconds']-reserved,self.remaining_wall()-wall_reserved-.05)
        if stage=='benchmark':
            if 'benchmarkDeadlineUtc'not in d:raise ValueError('benchmark local clock has not been admitted')
            local=dt.datetime.fromisoformat(d['benchmarkDeadlineUtc'].replace('Z','+00:00'));result=min(result,(local-dt.datetime.now(dt.timezone.utc)).total_seconds()-.05)
        if result<=0:raise TimeoutError('source stage must protect physics/wall reserve')
        return result
    def begin_interval(self,stage):
        self.allowance(stage);d=self.data
        diagnostic=self.profile=='analytic-diagnostic';cap=({'benchmark':64,'training':0,'validation':0}if diagnostic else{'benchmark':64,'training':796,'validation':640})[stage];reserve=0 if diagnostic or stage=='validation'else max(0,640-d['stagePolicyIntervals']['validation'])
        if d['stagePolicyIntervals'][stage]>=cap or d['totalPolicyIntervals']+1+reserve>(64 if diagnostic else 1500):raise TimeoutError('source interval/validation reserve exhausted')
        d['stagePolicyIntervals'][stage]+=1;d['totalPolicyIntervals']+=1;d['status']='running-'+stage;atomic_json(self.path,d)
    def integrate(self,stage,step_one,counter):
        if self.data.get('requestOutstanding'):raise ValueError('outstanding source request')
        remaining=self.allowance(stage);before=counter()
        if type(before)is not int or before<0:raise ValueError('invalid pre-integration counter')
        d=self.data;d.update(requestOutstanding=True,pendingPossibleNativeIntegrations=1,pendingStage=stage,pendingPhysicsReserveSeconds=remaining);atomic_json(self.path,d)
        def alarm(*_):raise TimeoutError('cooperative source call resource limit')
        previous=signal.signal(signal.SIGALRM,alarm);started=time.perf_counter();error=None;actual=None
        try:
            signal.setitimer(signal.ITIMER_REAL,remaining);step_one(before+1)
        except BaseException as caught:error=caught
        finally:
            signal.setitimer(signal.ITIMER_REAL,0);elapsed=time.perf_counter()-started;signal.signal(signal.SIGALRM,previous)
            try:
                after=counter()
                if type(after)is not int or after-before not in [0,1]:raise ValueError('invalid post-integration counter')
                actual=after-before
            except BaseException as caught:error=error or caught
            d['physicsSeconds']+=elapsed;d['stagePhysicsSeconds'][stage]+=elapsed
            # An unreadable counter has one conservative possible integration.
            d['sourceIntegrations']+=1 if actual is None else actual
            d.update(requestOutstanding=actual is None,pendingPossibleNativeIntegrations=1 if actual is None else 0,lastCallPhysicsSeconds=elapsed,lastCallStage=stage,pendingPhysicsReserveSeconds=remaining if actual is None else 0.)
            if actual is None:d['status']='unresolved-counter-after-source-call'
            atomic_json(self.path,d)
        if error:raise error
        if actual!=1:raise ValueError('source callback did not execute exactly one integration')
        self.allowance(stage)
        return before+1
