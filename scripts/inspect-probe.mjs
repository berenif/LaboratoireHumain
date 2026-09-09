import {readFileSync} from 'node:fs';
const rows=JSON.parse(readFileSync(process.argv[2],'utf8'));
for(const index of [0,5,15,30,60,120]){
 const s=rows[index];if(!s)continue;
 console.log(JSON.stringify({i:index,phase:s.diagnostics.recovery.phase,root:s.rootPosition,rootq:s.rootRotation,com:s.diagnostics.recovery.centerOfMass,contacts:s.diagnostics.recovery.contacts.filter(c=>c.loadBearing).map(c=>({id:c.segment,p:c.point,f:c.forceN})),parts:s.segments.filter(p=>['torso','leftFoot','rightFoot','leftShin','rightShin'].includes(p.id)).map(p=>({id:p.id,p:p.position,q:p.rotation}))}));
}
