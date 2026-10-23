// Capsule projection and painter ordering adapted from rust/browser/minimal/app.js.
// Coordinates come exclusively from MuJoCo geom_xpos/geom_xmat; no pose animation.
import {Physics,add,sub,mul} from './engine.js';
const $=s=>document.querySelector(s),canvas=$('#simulation'),ctx=canvas.getContext('2d');
let getup=null,getupBusy=false,getupLoading=false,getupStarted=false,getupModule=null;
const localGetup=['127.0.0.1','localhost','[::1]'].includes(location.hostname)&&new URLSearchParams(location.search).get('getup')==='local';
let physics=null,paused=true,failed=false,yaw=.65,pitch=.28,zoom=1,last=performance.now(),accumulator=0,pointer=null,parts=[];
const vworld=p=>[p[0],p[2],-p[1]], fromView=p=>[p[0],-p[2],p[1]];
function project(p){const center=getup&&physics?physics.data.subtree_com.slice(3,5):[0,0];const v=vworld([p[0]-center[0],p[1]-center[1],p[2]]),c=Math.cos(yaw),s=Math.sin(yaw),x=v[0]*c-v[2]*s,z=v[0]*s+v[2]*c,y=(v[1]-.75)*Math.cos(pitch)-z*Math.sin(pitch),depth=z*Math.cos(pitch)+(v[1]-.75)*Math.sin(pitch),scale=Math.min(canvas.width/(canvas.clientWidth<500?2.5:4),canvas.height/2.7)*zoom*5/Math.max(1,5+depth);return {x:canvas.width/2+x*scale,y:canvas.height*.53-y*scale,scale,depth};}
function line(a,b,color,width){ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();}
function draw(){const ratio=Math.min(devicePixelRatio,2),w=Math.round(canvas.clientWidth*ratio),h=Math.round(canvas.clientHeight*ratio);if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h;}ctx.fillStyle='#e7e3d8';ctx.fillRect(0,0,w,h);ctx.lineCap='round';for(let i=-6;i<=6;i++){line(project([i*.5,-3,0]),project([i*.5,3,0]),'#c7c9bb',ratio);line(project([-3,i*.5,0]),project([3,i*.5,0]),'#c7c9bb',ratio);}parts=[];if(!physics)return;
 for(const g of physics.geometries()){const a=project(g.a),b=project(g.b);parts.push({...g,pa:a,pb:b,r:g.radius*(a.scale+b.scale)/2,depth:(a.depth+b.depth)/2});}
 // Soft floor marks locate the real body; no effect on physics.
 for(const p of parts){const a=project([p.a[0],p.a[1],.001]),b=project([p.b[0],p.b[1],.001]);line(a,b,'#8b998833',p.r*1.7);}
 parts.sort((a,b)=>b.depth-a.depth);for(const p of parts){const active=physics.grab?.body===p.body;line(p.pa,p.pb,'#31554e',p.r*2+2*ratio);const color=active?'#e6b471':p.name.includes('head')?'#d8cbb1':p.name.includes('foot')?'#6d9481':'#a0c8bb';line(p.pa,p.pb,color,p.r*2);if(Math.hypot(p.pa.x-p.pb.x,p.pa.y-p.pb.y)<.1){ctx.fillStyle=color;ctx.beginPath();ctx.arc(p.pa.x,p.pa.y,p.r,0,Math.PI*2);ctx.fill();}}
 if(physics.grab){const a=project(physics.worldPoint(physics.grab.body,physics.grab.local)),b=project(physics.grab.target);line(a,b,'#b96e39',2*ratio);ctx.strokeStyle='#b96e39';ctx.beginPath();ctx.arc(b.x,b.y,5*ratio,0,Math.PI*2);ctx.stroke();}}
function refresh(){if(!physics)return;refreshGetup();$('#pause').textContent=paused?'Démarrer / reprendre':'Pause';$('#status').textContent=getup?(getup.done?'Essai GetUp terminé · en pause':paused?'Essai GetUp · en pause':getup.phase==='passive-fall'?'Essai GetUp · chute passive':getup.phase==='SLOW-moving'?'Essai GetUp · relevage':'Essai GetUp · réception / observation'):failed?'Simulation arrêtée':physics.grab?'Membre saisi':paused?'En pause':physics.fallen?'Chute · commande relâchée':physics.mode==='balance'?'Équilibre · double appui':'Passif';$('#time').textContent=physics.data.time.toFixed(2)+' s';$('#contacts').textContent=physics.data.ncon;$('#force').textContent=physics.lastForce.toFixed(0)+' N';}
function fail(e){if(getup&&!getup.done)getup.finish('Erreur — arrêt de l’essai : '+String(e));failed=true;paused=true;physics?.release();$('#error').hidden=false;$('#error').textContent=String(e);refresh();}
function release(){if(pointer&&canvas.hasPointerCapture(pointer.id))canvas.releasePointerCapture(pointer.id);pointer=null;physics?.release();canvas.classList.remove('grabbing');}
function xy(e){const r=canvas.getBoundingClientRect();return [(e.clientX-r.left)*canvas.width/r.width,(e.clientY-r.top)*canvas.height/r.height];}
function hit(x,y){for(const p of [...parts].reverse()){const dx=p.pb.x-p.pa.x,dy=p.pb.y-p.pa.y,t=Math.max(0,Math.min(1,((x-p.pa.x)*dx+(y-p.pa.y)*dy)/(dx*dx+dy*dy||1)));if(Math.hypot(x-p.pa.x-t*dx,y-p.pa.y-t*dy)<=p.r+5)return {p,t};}return null;}
canvas.addEventListener('pointerdown',e=>{if(!physics||failed||getupLoading||pointer||e.button>0)return;e.preventDefault();const [x,y]=xy(e),h=getup?null:hit(x,y);canvas.setPointerCapture(e.pointerId);canvas.classList.add('grabbing');if(h){const point=add(h.p.a,mul(sub(h.p.b,h.p.a),h.t));const toward=fromView([-Math.sin(yaw)*Math.cos(pitch),-Math.sin(pitch),-Math.cos(yaw)*Math.cos(pitch)]);const surface=add(point,mul(toward,h.p.radius));physics.beginGrab(h.p.body,surface);pointer={id:e.pointerId,type:'grab',x,y,start:surface,scale:project(surface).scale};}else pointer={id:e.pointerId,type:'orbit',x,y};refresh();});
canvas.addEventListener('pointermove',e=>{if(!pointer||e.pointerId!==pointer.id)return;const [x,y]=xy(e);if(pointer.type==='orbit'){yaw+=(x-pointer.x)*.008;pitch=Math.max(-.1,Math.min(1.1,pitch+(y-pointer.y)*.005));pointer.x=x;pointer.y=y;}else{const dx=(x-pointer.x)/pointer.scale,dy=-(y-pointer.y)/pointer.scale;const right=fromView([Math.cos(yaw),0,-Math.sin(yaw)]),up=fromView([-Math.sin(yaw)*Math.sin(pitch),Math.cos(pitch),-Math.cos(yaw)*Math.sin(pitch)]);physics.moveGrab(add(pointer.start,add(mul(right,dx),mul(up,dy))));}});
for(const event of ['pointerup','pointercancel','lostpointercapture'])canvas.addEventListener(event,()=>{pointer=null;physics?.release();canvas.classList.remove('grabbing');refresh();});
canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.5,Math.min(2.5,zoom*Math.exp(-e.deltaY*.001)));},{passive:false});
$('#zoom-in').onclick=()=>zoom=Math.min(2.5,zoom*1.15);$('#zoom-out').onclick=()=>zoom=Math.max(.5,zoom/1.15);$('#camera-reset').onclick=()=>{yaw=.65;pitch=.28;zoom=1;};
$('#pause').onclick=()=>{if(!physics||failed||getupLoading||getupBusy||getup?.done)return;paused=!paused;release();accumulator=0;refresh();};
$('#mode').onchange=()=>{if(physics&&!getup&&!getupLoading){physics.setMode($('#mode').value);refresh();}};
$('#small-push').onclick=()=>{if(!physics||failed||getup||getupLoading)return;physics.smallPush($('#push-direction').value);paused=false;refresh();};
$('#push').onclick=()=>{if(!physics||failed||getup||getupLoading)return;physics.pushBody();paused=false;refresh();};
$('#reset').onclick=()=>{if(!physics||getupBusy||getupLoading)return;if(getup){if(!getup.done)getup.finish('Arrêt demandé — retour au MVP');getup.dispose();getup=null;physics.model.opt.timestep=.00125;}release();physics.reset();paused=true;failed=false;accumulator=0;$('#error').hidden=true;refresh();};
window.addEventListener('blur',()=>{release();paused=true;accumulator=0;refresh();});document.addEventListener('visibilitychange',()=>{if(document.hidden){release();paused=true;accumulator=0;refresh();}});
window.addEventListener('keydown',e=>{if(e.target.matches('input,select,button'))return;if(e.code==='Space'){e.preventDefault();$('#pause').click();}if(e.code==='KeyP')$('#push').click();if(e.code==='KeyR')$('#reset').click();});
// During the local cycle, this is the ONLY stepping path. Physics.step()/balance
// never run alongside SLOW; drawing and read-only diagnostics do not call forward.
async function frame(now){
 const elapsed=Math.min(.05,(now-last)/1000);last=now;
 try{
  if(physics&&!paused&&!failed&&!document.hidden){
   accumulator=Math.min(.05,accumulator+elapsed);
   if(getup){
    if(accumulator>=.025&&!getup.done){
     getupBusy=true;
     await window.__lhGetupAudit?.({kind:'reserve',ticks:getup.ticks,pending:getup.nextTicks});
     const row=getup.advance();
     await window.__lhGetupAudit?.({kind:'sample',row,summary:getup.summary()});
     accumulator-=.025;
     if(getup.done){paused=true;accumulator=0;await window.__lhGetupAudit?.({kind:'final',summary:getup.summary(),state:getup.stateVector(),spec:getup.assets.entry.spec});}
    }
   }else{while(accumulator>=physics.dt){physics.step();accumulator-=physics.dt;}physics.forward();}
  }
  draw();refresh();
 }catch(e){fail(e);}finally{getupBusy=false;requestAnimationFrame(frame);}
}requestAnimationFrame(frame);
try{physics=await Physics.create();for(const id of ['pause','push','small-push','reset'])$('#'+id).disabled=false;refresh();}catch(e){fail(e);}
// Read-only diagnostics for browser acceptance; physical interactions use UI handlers.
window.__lhMujoco={get state(){return physics?{...physics.state(),paused,failed,camera:{yaw,pitch,zoom},geometries:physics.geometries(),getup:getup?.summary()||null,screen:parts.map(p=>({body:p.body,name:p.name,x:(p.pa.x+p.pb.x)/2/canvas.width*canvas.clientWidth,y:(p.pa.y+p.pb.y)/2/canvas.height*canvas.clientHeight}))}:null;}};

function refreshGetup(){
 const locked=!!getup||getupLoading;
 for(const id of ['mode','push','small-push','push-direction'])$('#'+id).disabled=locked;
 $('#reset').disabled=getupBusy||getupLoading;$('#pause').disabled=getupBusy||getupLoading||!!getup?.done;
 if(!getup)return;
 const s=getup.summary(),o=s.last,labels={'passive-fall':'Chute passive','SLOW-moving':'Relevage SLOW','SLOW-holding':'Réception et observation'};
 $('#getup-result').hidden=false;
 $('#getup-result').textContent=`${getup.done?'EN PAUSE · '+getup.reason:labels[getup.phase]}\n${s.ticks} / 10 200 pas · t=${physics.data.time.toFixed(3)} s\n${o?.air?'En vol':o?.bilateral?'Deux pieds chargés':o?.feet.some(f=>f>=3)?'Un pied chargé':'Autres contacts'} · ${s.receptionTime===null?'réception non encore observée':'réception observée à '+s.receptionTime.toFixed(3)+' s'}\nCalme continu : ${s.calmSeconds.toFixed(3)} s · nouveaux envols après réception : ${s.newFlightSamples} échantillons\n${getup.done?(s.observedHold?'Réception et calme observés sur ce cas uniquement.':'Critères de réception calme non tous observés.'):'Traction, poussées et sélection de commande indisponibles pendant le scénario.'}\n${getup.done?'Aucune intégration après ce bilan. Recommencer quitte cet essai et réinitialise explicitement le MVP.':''}`;
}
if(localGetup){
 document.body.classList.add('local-getup');
 $('#getup-panel').hidden=false;
 $('#getup-start').onclick=async()=>{
  if(!physics||getupStarted||getupBusy||getupLoading)return;
  paused=true;release();getupLoading=true;$('#getup-start').disabled=true;
  try{
   getupModule=await import('/web/mujoco/local-getup/cycle.js');
   const assets=await getupModule.loadCycleAssets();
   const nextPhysics=new Physics(physics.mj,assets.xml);
   const nextCycle=new getupModule.GetUpCycle(nextPhysics,assets);
   physics.dispose();physics=nextPhysics;getup=nextCycle;getupStarted=true;
   await window.__lhGetupAudit?.({kind:'start',row:getup.last,summary:getup.summary(),identities:assets.identities});
   accumulator=0;last=performance.now();failed=false;paused=false;
  }catch(e){fail(e);}finally{getupLoading=false;refresh();}
 };
}
