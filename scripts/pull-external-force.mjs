// Same arithmetic AND additive writes as Physics.apply. Uses the frozen,
// measured body COM so a dynamics copy needs no extra kinematics pass.
import {cross,sub} from '../web/dist/engine.js';
export function accumulateInteraction(data,body,{point,bodyCOM,force}){
 const torque=cross(sub(point,bodyCOM),force);
 for(let i=0;i<3;i++){data.xfrc_applied[6*body+i]+=force[i];data.xfrc_applied[6*body+3+i]+=torque[i];}
}
