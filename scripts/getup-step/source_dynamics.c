/* Read-only exact MuJoCo inverse-dynamics telemetry. Uses the original LH ABI.
 * No forces, constraints, model edits, or integrations are added. */
#include "source_bridge.c"
void lh_mass_matrix(LH*h,double*out){mj_fullM(h->m,out,h->d->qM);}
void lh_bias(LH*h,double*out){for(int i=0;i<h->m->nv;i++)out[i]=h->d->qfrc_bias[i]-h->d->qfrc_passive[i];}
void lh_com_jac(LH*h,double*out){double j[81],mass=0;memset(out,0,81*sizeof(double));for(int b=1;b<h->m->nbody;b++){mj_jacBodyCom(h->m,h->d,j,NULL,b);double m=h->m->body_mass[b];mass+=m;for(int i=0;i<81;i++)out[i]+=m*j[i];}for(int i=0;i<81;i++)out[i]/=mass;}
void lh_body_jac(LH*h,int body,double*out){mj_jacBody(h->m,h->d,out,out+3*h->m->nv,body);}
void lh_point_jac(LH*h,int body,const double*p,double*out){mj_jac(h->m,h->d,out,NULL,p,body);}
void lh_geom(LH*h,int geom,double*out){out[0]=h->m->geom_bodyid[geom];out[1]=h->m->geom_type[geom];for(int i=0;i<3;i++){out[2+i]=h->m->geom_size[3*geom+i];out[5+i]=h->d->geom_xpos[3*geom+i];}for(int i=0;i<9;i++)out[8+i]=h->d->geom_xmat[9*geom+i];}
void lh_joint_limits(LH*h,double*out){for(int i=1;i<h->m->njnt;i++){out[2*(i-1)]=h->m->jnt_range[2*i];out[2*(i-1)+1]=h->m->jnt_range[2*i+1];}}
/* Directional kinematic derivative on scratch data, no mj_step calls. */
void lh_jdot(LH*h,int body,double*out){
 double a[162],b[162],eps=1e-5;int rows=body<0?3:6;
 if(body<0)lh_com_jac(h,a);else lh_body_jac(h,body,a);
 mjData*d=mj_makeData(h->m);mju_copy(d->qpos,h->d->qpos,h->m->nq);mju_copy(d->qvel,h->d->qvel,h->m->nv);mj_integratePos(h->m,d->qpos,d->qvel,eps);mj_kinematics(h->m,d);mj_comPos(h->m,d);
 LH copy=*h;copy.d=d;if(body<0)lh_com_jac(&copy,b);else lh_body_jac(&copy,body,b);
 for(int r=0;r<rows;r++){out[r]=0;for(int c=0;c<27;c++)out[r]+=(b[27*r+c]-a[27*r+c])*h->d->qvel[c]/eps;}
 mj_deleteData(d);
}
void lh_motion_metrics(LH*h,double*out){out[0]=out[1]=out[2]=0;for(int b=1;b<h->m->nbody;b++){double v[6];mj_objectVelocity(h->m,h->d,mjOBJ_BODY,b,v,0);double linear=mju_norm(v+3,3),angular=mju_norm(v,3);if(linear>out[0])out[0]=linear;if(angular>out[1])out[1]=angular;}for(int i=0;i<h->d->ncon;i++){mjContact*c=&h->d->contact[i];if(h->m->geom_bodyid[c->geom1]&&h->m->geom_bodyid[c->geom2]&&-c->dist>out[2])out[2]=-c->dist;}}
/* Ground-only rays explicitly exclude every mannequin geom. The installed
 * source floor belongs to worldbody; mesh/hfield routes use engine queries too.
 * Return geom id, distance, and world hit position. No hit is -1, not a floor. */
void lh_ground_ray(LH*h,const double*origin,const double*direction,double*out){
 int id=-1;double nearest=-1;
 for(int i=0;i<h->m->ngeom;i++){
  if(h->m->geom_bodyid[i]!=0)continue;
  int type=h->m->geom_type[i];double distance;
  if(type==mjGEOM_HFIELD)distance=mj_rayHfield(h->m,h->d,i,origin,direction);
  else if(type==mjGEOM_MESH)distance=mj_rayMesh(h->m,h->d,i,origin,direction);
  else distance=mju_rayGeom(h->d->geom_xpos+3*i,h->d->geom_xmat+9*i,h->m->geom_size+3*i,origin,direction,type);
  if(distance>=0&&(nearest<0||distance<nearest)){nearest=distance;id=i;}
 }
 out[0]=id;out[1]=nearest;for(int k=0;k<3;k++)out[2+k]=id<0?0:origin[k]+nearest*direction[k];
}
void lh_joint_anchor(LH*h,int joint,double*out){const double*p=joint==0?h->d->xpos+3:h->d->xanchor+3*joint;for(int k=0;k<3;k++)out[k]=p[k];}
const char*lh_joint_name(LH*h,int joint){return mj_id2name(h->m,mjOBJ_JOINT,joint);}
/* Kinematic decomposition only, at current body origin and contact locations.
 * out: origin linear xyz, angular xyz, then current contact count and each
 * ground contact's world point xyz and tangent xyz (unweighted current geometry). */
void lh_foot_motion(LH*h,int body,double*out){
 double j[162];lh_body_jac(h,body,j);for(int k=0;k<6;k++)out[k]=mju_dot(j+27*k,h->d->qvel,27);int count=0;
 for(int i=0;i<h->d->ncon&&count<8;i++){mjContact*c=&h->d->contact[i];int a=h->m->geom_bodyid[c->geom1],b=h->m->geom_bodyid[c->geom2];if(!((a==0&&b==body)||(b==0&&a==body)))continue;double jac[81],v[3];mj_jac(h->m,h->d,jac,NULL,c->pos,body);for(int k=0;k<3;k++)v[k]=mju_dot(jac+27*k,h->d->qvel,27);double vn=mju_dot(v,c->frame,3);for(int k=0;k<3;k++){out[7+6*count+k]=c->pos[k];out[10+6*count+k]=v[k]-vn*c->frame[k];}count++;}out[6]=count;
}
int lh_joint_body(LH*h,int joint){return h->m->jnt_bodyid[joint];}
void lh_geom_friction(LH*h,int geom,double*out){for(int k=0;k<3;k++)out[k]=h->m->geom_friction[3*geom+k];out[3]=h->m->geom_condim[geom];out[4]=h->m->geom_priority[geom];}
void lh_contact_friction(LH*h,double*out){int count=0;for(int i=0;i<h->d->ncon&&count<16;i++){mjContact*c=&h->d->contact[i];int a=h->m->geom_bodyid[c->geom1],b=h->m->geom_bodyid[c->geom2];if(!((a==0&&(b==h->feet[0]||b==h->feet[1]))||(b==0&&(a==h->feet[0]||a==h->feet[1]))))continue;double*p=out+1+9*count;p[0]=a?a:b;p[1]=c->geom1;p[2]=c->geom2;p[3]=c->dim;for(int k=0;k<5;k++)p[4+k]=c->friction[k];count++;}out[0]=count;}
static double clamp01(double x){return x<0?0:x>1?1:x;}
/* Read-only capsule/sphere separation for native collision-eligible body pairs. */
void lh_self_separation(LH*h,double*out){
 int count=0;out[0]=0;
 for(int ga=1;ga<h->m->ngeom;ga++)for(int gb=ga+1;gb<h->m->ngeom;gb++){
  int a=h->m->geom_bodyid[ga],b=h->m->geom_bodyid[gb],wa=h->m->body_weldid[a],wb=h->m->body_weldid[b];
  if(!a||!b||wa==wb)continue;
  int pa=h->m->body_weldid[h->m->body_parentid[wa]],pb=h->m->body_weldid[h->m->body_parentid[wb]];if(pa==wb||pb==wa)continue;
  if(!((h->m->geom_contype[ga]&h->m->geom_conaffinity[gb])||(h->m->geom_contype[gb]&h->m->geom_conaffinity[ga])))continue;
  int lo=a<b?a:b,hi=a<b?b:a,signature=((lo+1)<<16)+(hi+1),excluded=0;for(int x=0;x<h->m->nexclude;x++)if(h->m->exclude_signature[x]==signature)excluded=1;if(excluded)continue;
  int ta=h->m->geom_type[ga],tb=h->m->geom_type[gb];if((ta!=mjGEOM_CAPSULE&&ta!=mjGEOM_SPHERE)||(tb!=mjGEOM_CAPSULE&&tb!=mjGEOM_SPHERE))continue;
  double p[3],q[3],d[3],e[3],r[3];double ha=ta==mjGEOM_CAPSULE?h->m->geom_size[3*ga+1]:0,hb=tb==mjGEOM_CAPSULE?h->m->geom_size[3*gb+1]:0;
  for(int k=0;k<3;k++){double xa=h->d->geom_xmat[9*ga+3*k+2],xb=h->d->geom_xmat[9*gb+3*k+2];p[k]=h->d->geom_xpos[3*ga+k]-ha*xa;q[k]=h->d->geom_xpos[3*gb+k]-hb*xb;d[k]=2*ha*xa;e[k]=2*hb*xb;r[k]=p[k]-q[k];}
  double aa=mju_dot(d,d,3),bb=mju_dot(d,e,3),ee=mju_dot(e,e,3),cc=mju_dot(d,r,3),ff=mju_dot(e,r,3),s=0,t=0;
  if(aa<1e-16&&ee<1e-16){}else if(aa<1e-16)t=clamp01(ff/ee);else if(ee<1e-16)s=clamp01(-cc/aa);else{double den=aa*ee-bb*bb;s=den>1e-16?clamp01((bb*ff-cc*ee)/den):0;t=(bb*s+ff)/ee;if(t<0){t=0;s=clamp01(-cc/aa);}else if(t>1){t=1;s=clamp01((bb-cc)/aa);}}
  double x[3],y[3],n[3];for(int k=0;k<3;k++){x[k]=p[k]+s*d[k];y[k]=q[k]+t*e[k];n[k]=x[k]-y[k];}double length=mju_norm(n,3),ra=h->m->geom_size[3*ga],rb=h->m->geom_size[3*gb],gap=length-ra-rb;if(gap>.08)continue;if(length<1e-12)continue;
  double*z=out+1+14*count;z[0]=a;z[1]=b;z[2]=ga;z[3]=gb;z[4]=gap;for(int k=0;k<3;k++){n[k]/=length;z[5+k]=x[k]-n[k]*ra;z[8+k]=y[k]+n[k]*rb;z[11+k]=n[k];}if(++count>=64){out[0]=count;return;}
 }out[0]=count;
}
/* Last solved generalized constraint force; step1 does not overwrite it. */
void lh_solved_constraint(LH*h,double*out){mju_copy(out,h->d->qfrc_constraint,h->m->nv);}
/* Exact source contact forward solve on scratch mjData, no mj_step/integration. */
static void motor_response(LH*h,mjData*d,const double*u,double*out){
 mj_copyData(d,h->m,h->d);mju_copy(d->ctrl,u,h->m->nu);mj_fwdActuation(h->m,d);mj_fwdAcceleration(h->m,d);mj_fwdConstraint(h->m,d);mju_copy(out,d->qacc,h->m->nv);mju_zero(out+27,6);
 for(int i=0;i<d->ncon;i++){mjContact*c=&d->contact[i];int a=h->m->geom_bodyid[c->geom1],b=h->m->geom_bodyid[c->geom2];if((a==0)==(b==0))continue;int body=a?a:b,side=body==h->feet[0]?0:body==h->feet[1]?1:-1;if(side<0)continue;double f[6],world[3];mj_contactForce(h->m,d,i,f);mju_mulMatTVec(world,c->frame,f,3,3);for(int k=0;k<3;k++)out[27+3*side+k]+=(a?-1:1)*world[k];}
}
void lh_motor_response(LH*h,const double*u,double*out){mjData*d=mj_makeData(h->m);motor_response(h,d,u,out);mj_deleteData(d);}
void lh_motor_linearization(LH*h,const double*u,double*out){mjData*d=mj_makeData(h->m);double up[21],fm[33],fp[33],eps=.001;motor_response(h,d,u,out);mju_copy(up,u,21);for(int j=0;j<21;j++){up[j]=u[j]-eps;motor_response(h,d,up,fm);up[j]=u[j]+eps;motor_response(h,d,up,fp);up[j]=u[j];for(int k=0;k<33;k++)out[33+k*21+j]=(fp[k]-fm[k])/(2*eps);}mj_deleteData(d);}
