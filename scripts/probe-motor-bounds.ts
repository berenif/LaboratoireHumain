import {solveRecoveryMotorTorques}from'../src/character/recovery-motors';
import type{SegmentId,Vec3}from'../src/core/types';
const ids:SegmentId[]=['pelvis','torso','neck','head'];
const values=[{m11:1,m12:.1,m13:0,m22:.8,m23:0,m33:.7},{m11:.5,m12:.1,m13:.1,m22:.6,m23:0,m33:.8},{m11:300,m12:10,m13:5,m22:220,m23:4,m33:250},{m11:20,m12:1,m13:2,m22:24,m23:1,m33:23}];
const tensors=new Map(ids.map((id,i)=>[id,values[i]]));
const desired:Vec3[]=[{x:48,y:36,z:0},{x:-1,y:2,z:3},{x:0,y:13.2,z:17.6},{x:2,y:-1,z:.5}];
const kp=[12000,4500,1500,1500],kd=[80,85,12,12],caps=[60,110,22,22],lambda=[.2,0,5,0],dt=1/60;
const total=ids.map(()=>({x:0,y:0,z:0}));
for(let j=0;j<4;j++)for(const axis of ['x','y','z']as const){total[j][axis]+=desired[j][axis];if(j)total[j-1][axis]-=desired[j][axis];}
const acceleration=total.map((t,i)=>{const m=values[i];return{x:m.m11*t.x+m.m12*t.y+m.m13*t.z,y:m.m12*t.x+m.m22*t.y+m.m23*t.z,z:m.m13*t.x+m.m23*t.y+m.m33*t.z}});
const motors=ids.map((id,i)=>{const k=kd[i]*dt+kp[i]*dt*dt;return{id,parent:i?ids[i-1]:null,kp:kp[i],kd:kd[i],cap:caps[i],feedforward:{x:0,y:0,z:0},velocity:{x:0,y:0,z:0},error:Object.fromEntries((['x','y','z']as const).map(axis=>[axis,(desired[i][axis]/k+acceleration[i][axis]-(i?acceleration[i-1][axis]:0)+lambda[i]*desired[i][axis])*k/kp[i]]))as unknown as Vec3}});
const start=performance.now(),solved=solveRecoveryMotorTorques(motors,tensors,dt);console.log({duration:performance.now()-start,errors:ids.map((id,i)=>Math.hypot(solved.get(id)!.x-desired[i].x,solved.get(id)!.y-desired[i].y,solved.get(id)!.z-desired[i].z)),solved:[...solved]});
