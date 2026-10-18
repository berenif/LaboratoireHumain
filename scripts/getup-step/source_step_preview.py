#!/usr/bin/env python3
"""Opt-in loopback preview adapter; original engine, live controls, existing UI.

No background integrations, pose tape, new framework, or public network listener.
"""
import argparse,copy,importlib.util,json,math,mimetypes,threading,time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit,unquote
import numpy as np
from scipy.spatial import ConvexHull,QhullError
from scipy.spatial.transform import Rotation
from source_step_probe import World,Controller,ROOT,sha
from source_math import P
spec=importlib.util.spec_from_file_location('preview_fk',ROOT/'scripts/getup-step-reference.py');fk=importlib.util.module_from_spec(spec);spec.loader.exec_module(fk)
class LiveStep:
 def __init__(self,base,helper,output,maximum_integrations=24000):
  self.base=base;self.helper=helper;self.output=output;output.mkdir(parents=True,exist_ok=True);self.maximum=maximum_integrations;self.total=0;self.generation=0;self.request=0;self.observation_sequence=0;self.command_sequence=0;self.world=None;self.journal=None;self.lock=threading.RLock();self.model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());self.kin=fk.Kinematics(self.model)
  seed=World(base,helper)
  try:seed.restore(base/'slow-continuation/final.bin');self.initial=seed.packet()
  finally:seed.close()
  self.reset()
 def reset(self):
  if self.world:
   self.save('explicit-user-reset');self.journal.close();self.world.close()
  self.world=World(self.base,self.helper);self.world.install_cold(self.initial);self.controller=Controller(self.world);self.generation+=1;self.steps=0;self.phase='transfer';self.phase_start=0.;self.paused=True;self.visible=True;self.finished=False;self.failure=None;self.u=np.zeros(21);self.notes=[];self.events=[];self.slip=np.zeros(2);self.clear_run=self.land_run=self.quiet_run=0;self.anchors=None;self.max_clear=-1.;self.max_motor=0.;self.loaded=np.zeros(2,dtype=int);self.entry=self.world.packet();self.row=self.decorate(self.entry.copy());self.folder=self.output/f'generation-{self.generation:03d}';self.folder.mkdir();self.journal=(self.folder/'frames.ndjson').open('x');self.command_sequence=0
 def decorate(self,row):
  row.update(elapsedS=self.steps*.00125,phase=self.phase,phaseElapsedS=self.steps*.00125-self.phase_start,loadedSlipDistanceM=self.slip.tolist(),forwardFootDisplacementM=float((np.array(row['feet'][0]['position'])-self.controller.targets[0][0])@self.controller.heading))
  margin=None;com=np.array(row['com']);capture=com[:2]+np.array(row['comVelocity'])[:2]*math.sqrt(com[2]/9.81)
  try:
   eq=ConvexHull(np.unique(np.array(row['supportPointsXY']).reshape(-1,2),axis=0)).equations;margin=float(min(np.min(-(eq[:,:2]@com[:2]+eq[:,2])),np.min(-(eq[:,:2]@capture+eq[:,2]))))
  except (QhullError,ValueError):pass
  row['minimumComCaptureMarginM']=margin
  if self.phase=='stand':
   positions=np.array([f['position']for f in row['feet']]);self.anchors=positions.copy()if self.anchors is None else self.anchors;drift=float(np.max(np.linalg.norm(positions-self.anchors,axis=1)));row['standingAnchorDriftM']=drift
   quiet=all(f['loadN']>=3 for f in row['feet'])and sum(f['loadN']for f in row['feet'])>=.8*self.world.mass*9.81 and row['torsoTiltDeg']<=5 and row['maximumBodyLinearMps']<=.22 and row['maximumBodyAngularRadps']<=.65 and margin is not None and margin>=.003 and .780625<=com[2]<=.829976 and drift<=.01 and row['maximumSelfPenetrationM']<=.005
   self.quiet_run=self.quiet_run+1 if quiet else 0;row['quietConsecutiveS']=self.quiet_run*.00125
  return row
 def advance(self,ticks):
  if type(ticks)is not int or not 0<=ticks<=4:raise ValueError('advance requires0..4 observation ticks')
  before=self.steps
  if self.paused or not self.visible or self.finished:return 0
  w=self.world;c=self.controller
  for _ in range(ticks*20):
   if self.total>=self.maximum or self.steps>=12800:self.failure='bounded-live-integration-budget';self.paused=True;break
   t=self.steps*.00125;elapsed=t-self.phase_start
   if self.steps%4==0:
    self.u,info=c.solve(elapsed,self.phase);self.notes.append(dict(t=t,phase=self.phase,**info));self.max_motor=max(self.max_motor,float(np.max(abs(self.u))))
   w.arr(2,(21,))[:]=self.u;expected=self.steps+1;self.total+=1
   actual=w.l.lh_step_audited(w.h,12800)
   if actual!=expected:raise ValueError('counted live integration mismatch')
   self.steps+=1;row=w.packet();self.slip+=np.array([f['loadedSlipSpeedMps']for f in row['feet']])*.00125;self.loaded=np.where(np.array([f['loadN']for f in row['feet']])>=3,self.loaded+1,0);self.row=self.decorate(row);self.max_clear=max(self.max_clear,row['feet'][0]['clearance']);self.journal.write(json.dumps(row,allow_nan=False)+'\n')
   if row['warnings']or row['nonfootLoadN']>=3 or all(f['loadN']<3 for f in row['feet']):self.failure='support-or-warning-guard'
   elif max(self.slip)>.01:self.failure='loaded-slip-10mm'
   elif row['maximumSelfPenetrationM']>.005:self.failure='self-penetration-5mm'
   elif row['com'][2]<.65:self.failure='height-guard'
   if self.failure:self.paused=True;break
   nextphase=None
   if self.phase=='transfer'and elapsed>=2.:
    com=np.array(row['com']);capture=com[:2]+np.array(row['comVelocity'])[:2]*math.sqrt(com[2]/9.81)
    if np.linalg.norm((com-c.com1)[:2])<.012 and np.linalg.norm(capture-c.com1[:2])<.015:nextphase='lift'
   elif self.phase=='lift':
    self.clear_run=self.clear_run+1 if row['feet'][0]['clearance']>=.025 and row['feet'][0]['loadN']<3 and row['feet'][1]['loadN']>=.8*w.mass*9.81 else 0
    if elapsed>=1. and self.clear_run>=80:nextphase='translate'
   elif self.phase=='translate'and elapsed>=1.:
    if row['forwardFootDisplacementM']>=.06 and row['feet'][0]['clearance']>=.025 and row['feet'][0]['loadN']<3:nextphase='land'
   elif self.phase=='land':
    self.land_run=self.land_run+1 if row['feet'][0]['loadN']>=3 and row['feet'][1]['loadN']>=3 and row['feet'][0]['clearance']<=.003 and row['forwardFootDisplacementM']>=.06 else 0
    if elapsed>=1. and self.land_run>=80:nextphase='settle'
   elif self.phase=='settle'and elapsed>=2.:nextphase='stand'
   elif self.phase=='stand'and self.quiet_run>=4000 and row['forwardFootDisplacementM']>=.06:self.finished=True;self.paused=True;break
   if nextphase:
    self.events.append(dict(timeS=self.steps*.00125,event=nextphase,measured=row));self.phase=nextphase;self.phase_start=self.steps*.00125;c.q0=np.clip(w.arr(0,(28,))[7:].copy(),w.limits[:,0]+.002,w.limits[:,1]-.002)
   elif self.phase!='stand'and elapsed>{'transfer':2.,'lift':1.,'translate':1.,'land':1.,'settle':2.}[self.phase]+2.:self.failure='phase-timeout-'+self.phase;self.paused=True;break
  self.journal.flush()
  if self.finished or self.failure:self.save('measured-forward-step-plus5squiet'if self.finished else self.failure)
  return (self.steps-before+19)//20
 def save(self,reason):
  if not self.world:return
  summary=dict(stopped=reason,actualIntegrations=self.steps,physicsSimSeconds=self.steps*.00125,events=self.events,maximumWholeFootClearanceM=self.max_clear,finalQuietConsecutiveS=self.quiet_run*.00125,loadedSlipDistanceM=self.slip.tolist(),maximumMotorFraction=self.max_motor,entry=self.entry,final=self.row,liveIntegrationsAcrossResets=self.total,originalWarmStartCacheRestored=False,explicitColdInitialPoseAndVelocity=True,modelSHA256=sha(self.world.xml),helperSHA256=sha(self.helper),adapterSHA256=sha(__file__),rootAssistance=False,physicalStateWritesAfterInitialization=0)
  (self.folder/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');(self.folder/'control.json').write_text(json.dumps(self.notes)+'\n')
 def poses(self):
  q=np.array(self.row['qpos']);v=np.array(self.row['qvel']);epsilon=1e-5
  def fk_at(dt):
   root=np.eye(4);sourceR=Rotation.from_quat(q[[4,5,6,3]]).as_matrix()@Rotation.from_rotvec(v[3:6]*dt).as_matrix();root[:3,:3]=P.T@sourceR@P;root[:3,3]=P.T@(q[:3]+v[:3]*dt);return self.kin.forward(q[7:]+v[6:]*dt,root)
  a=fk_at(0.);b=fk_at(epsilon);result=[]
  def vec(x):return dict(zip('xyz',np.asarray(x,dtype=np.float32).tolist()))
  for part in self.model['segments']:
   name=part['id'];T=a[name];U=b[name];quat=Rotation.from_matrix(T[:3,:3]).as_quat();result.append(dict(id=name,position=vec(T[:3,3]),rotation=dict(zip('xyzw',np.asarray(quat,dtype=np.float32).tolist())),linearVelocity=vec((U[:3,3]-T[:3,3])/epsilon),angularVelocity=vec(Rotation.from_matrix(U[:3,:3]@T[:3,:3].T).as_rotvec()/epsilon)))
  return result
 def reply(self,completed=0,acks=None):
  self.observation_sequence+=1;row=self.row;poses=self.poses();diagnostics=dict(controller_ms=None,update_ms=None,dropped_time_s=0.,queue_depth=0,requested_motor_effort_nm=float(sum(abs(x)for x in row['actualMotorTorquesNm'])),delivered_motor_impulse_nm_s=None,direct_pelvis_force_n=0.,direct_pelvis_torque_nm=0.,max_anchor_separation_m=0.,max_floor_penetration_m=max(0.,-min(f['clearance']for f in row['feet'])),max_self_penetration_m=row['maximumSelfPenetrationM'],max_limit_error_rad=max(0.,-row['jointLimitMinimumMarginRad']),getup=dict(controller='source-phasewise-torque-lab',learned_policy_loaded=False,source_bodies=16,coordinate_frames=9,active_hinges=21,joint_angles_rad=row['qpos'][7:],joint_velocities_radps=row['qvel'][6:],motor_controls=self.u.tolist(),physics_step_s=.00125,policy_step_s=.005,observation=[]))
  snapshot=dict(stamp=dict(schema=4,generation=self.generation,sequence=self.observation_sequence,tick=self.steps//20),simulation_time_s=self.steps*.00125,contact_time_s=self.steps*.00125,discontinuity=self.steps==0,segments=poses,motion='Halted'if self.failure else'Upright',contacts=[dict(segment=next(i for i,p in enumerate(self.model['segments'])if p['id']==name),normal_load_n=row['feet'][j]['loadN'],persistence_s=self.loaded[j]*.00125)for j,name in enumerate(['left_foot','right_foot'])if row['feet'][j]['loadN']>=3],counters=dict(strikes=0,falls=0,recoveries=0,steps=int(self.finished),upright_ticks=self.quiet_run//20),diagnostics=diagnostics,acknowledged_sequence=self.command_sequence)
  lab=dict(engine='original MuJoCo2.1.3, native loopback validation',generation=self.generation,nativeSteps=self.steps,totalNativeSteps=self.total,phase=self.phase,finished=self.finished,quietSeconds=self.quiet_run*.00125,forwardM=row['forwardFootDisplacementM'],leftClearanceM=row['feet'][0]['clearance'],loadedSlipM=self.slip.tolist(),jointGroundRaysWorld=[dict(joint=r['joint'],hit=r['hit'],origin=(P.T@r['originSource']).tolist(),point=(P.T@r['pointSource']).tolist()if r['hit']else None)for r in row['jointGroundRays']],activeJointGroundBarriers=self.notes[-1]['activeJointGroundBarriers']if self.notes else 0)
  failure=None if not self.failure else dict(check=self.failure,tick=self.steps//20,substep=self.steps%20,measured=1.,limit=0.,detail=self.failure)
  return dict(completed_ticks=completed,paused=self.paused,visible=self.visible,mode='protocol',acknowledgements=acks or[],snapshot=snapshot,failure=failure,sourceStepLab=lab)
 def handle(self,message):
  with self.lock:
   request=message.get('requestId');assert type(request)is int and request>self.request,'stale request';self.request=request;kind=message.get('type');acks=[];completed=0
   if kind=='initialize':pass
   elif kind=='advance':completed=self.advance(message.get('ticks'))
   elif kind=='visible':self.visible=message.get('visible')is True
   elif kind=='commands':
    commands=message.get('commands');assert isinstance(commands,list)and len(commands)<=32
    for command in commands:
     stamp=command['stamp'];assert stamp['generation']==self.generation and stamp['sequence']>self.command_sequence,'stale command';self.command_sequence=stamp['sequence'];action=command['action']
     if action=='Pause':self.paused=True
     elif action=='Resume':
      if not self.finished and not self.failure:self.paused=False
     elif action=='Reset':self.reset()
     elif action=='CancelGrab':pass
     else:raise ValueError('source lab supports Step/Pause/Reset only')
     acks.append(dict(stamp=stamp,applied_tick=self.steps//20,rejected=None))
   elif kind=='shutdown':self.paused=True
   else:raise ValueError('unknown source-lab request')
   return self.reply(completed,acks)
 def close(self):
  with self.lock:self.save('measured-forward-step-plus5squiet'if self.finished else self.failure or'preview-closed');self.journal.close();self.world.close();self.world=None

def serve(live,webroot,port):
 class Handler(BaseHTTPRequestHandler):
  def check(self):
   expected=f'http://127.0.0.1:{self.server.server_port}'
   return self.headers.get('Host')==expected[7:]and(self.command!='POST'or self.headers.get('Origin')==expected)
  def send(self,status,body,kind='application/json'):
   self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.end_headers();self.wfile.write(body)
  def do_GET(self):
   if not self.check():self.send(403,b'Forbidden');return
   path=unquote(urlsplit(self.path).path)
   if path=='/__source_step/state':
    with live.lock:self.send(200,json.dumps(live.reply(),allow_nan=False).encode())
    return
   if path=='/__source_step/worker.js':file=ROOT/'scripts/getup-step/source_step_preview_worker.js'
   else:
    file=(webroot/('./'+(path.lstrip('/')or'index.html'))).resolve()
    if not file.is_relative_to(webroot)or file.suffix not in ['.html','.js','.css','.wasm','.png','.svg','.ico']:self.send(404,b'Not found');return
   if not file.is_file():self.send(404,b'Existing app build missing');return
   self.send(200,file.read_bytes(),{'.wasm':'application/wasm','.js':'application/javascript'}.get(file.suffix,mimetypes.guess_type(file.name)[0]or'application/octet-stream'))
  def do_POST(self):
   if not self.check()or self.path!='/__source_step/action'or self.headers.get('Content-Type')!='application/json':self.send(403,b'Forbidden');return
   try:
    length=int(self.headers.get('Content-Length','0'));assert 0<length<=32768;body=json.loads(self.rfile.read(length));reply=live.handle(body);self.send(200,json.dumps(reply,allow_nan=False).encode())
   except (ValueError,AssertionError,KeyError,TypeError)as error:self.send(409,json.dumps(dict(error=str(error))).encode())
  def log_message(self,*args):pass
 server=ThreadingHTTPServer(('127.0.0.1',port),Handler);return server
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);p.add_argument('--helper',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--webroot',type=Path,default=ROOT/'rust/dist/app');p.add_argument('--port',type=int,default=5186);a=p.parse_args();assert 1024<=a.port<=65535;live=LiveStep(a.base,a.helper,a.output);server=serve(live,a.webroot.resolve(),a.port);print(f'Loopback live preview http://127.0.0.1:{a.port}/?sourceStepLab=1',flush=True)
 try:server.serve_forever()
 finally:server.server_close();live.close()
