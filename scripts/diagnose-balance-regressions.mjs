import { register } from 'tsx/esm/api';
const unregister = register();
const { createEmbodiedCharacter } = await import('../src/character/index.ts');
const { worldPoint } = await import('../src/character/math.ts');
const mode = process.argv[2] ?? 'slow';
const c = await createEmbodiedCharacter('canvas2d');
try {
  const hand = c.getSnapshot('canvas2d').segments.find(p => p.id === 'rightHand');
  const anchor = mode === 'slow' ? {x:0,y:0,z:0} : {x:.025,y:.015,z:.01};
  const start = worldPoint(hand.position,hand.rotation,anchor);
  c.fixedUpdate(1/60,{kind:'begin',pointerId:41,region:'rightHand',segment:'rightHand',localAnchor:anchor,worldTarget:start,timestampMs:0});
  let prior='';
  for(let tick=1;tick<=510;tick++) {
    let command=null;
    if(mode==='strong') {
      const f=Math.min(1,tick/5);
      if(tick<50)command={kind:'move',worldTarget:{x:start.x+1.25*f,y:start.y+.12*f,z:start.z}};
      else if(tick===50)command={kind:'end'};
    } else if(mode==='reversal') {
      const x=tick<=90?.75*tick/90:tick<=180?.75-1.5*(tick-90)/90:-.75;
      if(tick<300)command={kind:'move',worldTarget:{x:start.x+x,y:start.y+.02*Math.min(tick,90)/90,z:start.z}};
      else if(tick===300)command={kind:'end'};
    } else {
      if(tick<=90)command={kind:'move',worldTarget:{x:start.x+.6*tick/90,y:start.y,z:start.z+.08*tick/90}};
      else if(tick<=180)command={kind:'move',worldTarget:{x:start.x+.6-(tick-90)*.9/90,y:start.y,z:start.z+.08}};
      else if(tick===181)command={kind:'end'};
    }
    c.fixedUpdate(1/60,command?{...command,pointerId:41,timestampMs:tick*1000/60}:null);
    const s=c.getSnapshot('canvas2d'),d=s.diagnostics,phase=s.state+'/'+(c.step?.phase??'none');
    if(tick%15===0||phase!==prior)console.log(JSON.stringify({mode,tick,state:s.state,phase:c.step?.phase,elapsed:c.step?.elapsed,root:s.rootPosition,lean:d.leanRadians,fall:d.balanceFall,steps:d.stepCount,force:d.grabControl.force,work:d.grabControl.cumulativeInjectedWorkJ,balance:d.balance,feet:s.segments.filter(p=>p.id==='leftFoot'||p.id==='rightFoot').map(p=>({id:p.id,position:p.position,velocity:p.linearVelocity})),support:s.support}));
    prior=phase;
    if(['falling','fallen','recovering'].includes(s.state))break;
  }
} finally {c.dispose();unregister();}
