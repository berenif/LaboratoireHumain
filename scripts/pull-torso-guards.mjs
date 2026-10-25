// Diagnostic mathematics only. These guards do not command any motor.
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const rot=(R,v)=>[0,1,2].map(k=>dot(R.slice(3*k,3*k+3),v));
export function torsoKinematics(R,omega,alpha){
 const up=rot(R,[0,0,1]),upDot=rot(R,cross(omega,[0,0,1]));
 const tangent=cross(alpha,[0,0,1]),centripetal=cross(omega,cross(omega,[0,0,1]));
 const upDD=rot(R,tangent.map((v,i)=>v+centripetal[i]));
 const theta=Math.acos(Math.max(-1,Math.min(1,up[2]))),thetaDot=-upDot[2]/Math.sin(theta);
 return {up,upDot,upDD,theta,thetaDot,thetaDD:-(upDD[2]+Math.cos(theta)*thetaDot**2)/Math.sin(theta),orientationSpeedEnergyDerivative:dot(upDot,upDD)};
}
export function axisResiduals(omega,alpha,h){
 // Local body angular velocity, first-order proxy over the actual substep.
 // Both signs also cover omega=0. This is not a contact-persistence theorem.
 return omega.flatMap((w,i)=>[-1,1].map(sign=>({label:`angular-axis-${i}-${sign}`,value:sign*(w+h*alpha[i])-Math.abs(w)})));
}
export function robustNonpositiveLimit(center,relative,tolerance){
 return Math.min(...robustNonpositiveRows(center,relative,tolerance).map(r=>r.bound/r.factor));
}
export function robustNonpositiveRows(center,relative,tolerance){
 if(!(Number.isFinite(center)&&relative>=0&&relative<1&&Number.isFinite(relative)&&tolerance>=0&&Number.isFinite(tolerance)))throw Error('Expected finite scalar and declared error allowance');
 // gp + max(tol, r*abs(gp-center)) <= 0, for ANY sign of center.
 // Three exact affine inequalities; no added tolerance or new gain.
 return [{factor:1,bound:-tolerance},{factor:1+relative,bound:relative*center},{factor:1-relative,bound:-relative*center}];
}
export function recenteredControlBound(u0,center,trust,sign){
 return Math.min(trust-sign*(center-u0),1-sign*center);
}
