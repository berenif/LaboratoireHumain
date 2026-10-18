/* Isolated inspection only. No model/live-data writes and no integration APIs.
 * The two-pointer prefix is the pinned repository LH ABI. All forward solver
 * calls target a fresh mjData copy; contact geometry/velocities and fresh
 * forces are reported from that same unchanged-time causal state. */
#include <mujoco.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#define EXPORT __attribute__((visibility("default")))
typedef struct { mjModel *m; mjData *d; } LHView;
EXPORT uint64_t lh_ro_model_fingerprint(const LHView*h){uint64_t z=1469598103934665603ULL;const unsigned char*p=(const unsigned char*)h->m;for(size_t i=0;i<sizeof(mjModel);i++){z^=p[i];z*=1099511628211ULL;}p=(const unsigned char*)h->m->buffer;for(int i=0;i<h->m->nbuffer;i++){z^=p[i];z*=1099511628211ULL;}return z;}
EXPORT void lh_ro_model(const LHView*h,double*out){const mjOption*o=&h->m->opt;out[0]=mj_version();out[1]=o->timestep;out[2]=o->impratio;out[3]=o->cone;out[4]=o->solver;out[5]=o->iterations;out[6]=o->tolerance;out[7]=o->noslip_iterations;out[8]=o->noslip_tolerance;out[9]=o->integrator;out[10]=o->disableflags;out[11]=o->enableflags;out[12]=h->m->ngeom;out[13]=h->m->nv;out[14]=o->o_margin;for(int j=0;j<2;j++)out[15+j]=o->o_solref[j];for(int j=0;j<5;j++)out[17+j]=o->o_solimp[j];}
/* out header: count,time,fresh,solver_iter,nefc, qacc27, qfrc_constraint27.
 * Contacts begin at64, stride112. Cached force buffers are deliberately NaN:
 * step1 has refreshed geometry since their previous solve, so they are not
 * paired with a current frame and never used for utilization conclusions. */
EXPORT int lh_ro_ground_contacts(const LHView*h,int fresh,double*out,int capacity){
 if(capacity<64+112*64||h->m->nv!=27)return -1;memset(out,0,capacity*sizeof(double));mjData*scratch=NULL;const mjData*d=h->d;
 if(fresh){scratch=mj_makeData(h->m);if(!scratch)return -2;mj_copyData(scratch,h->m,h->d);mj_fwdActuation(h->m,scratch);mj_fwdAcceleration(h->m,scratch);mj_fwdConstraint(h->m,scratch);d=scratch;}
 out[1]=d->time;out[2]=fresh;out[3]=d->solver_iter;out[4]=d->nefc;for(int j=0;j<27;j++){out[5+j]=d->qacc[j];out[32+j]=d->qfrc_constraint[j];}
 int count=0;
 for(int i=0;i<d->ncon&&count<64;i++){
  const mjContact*c=&d->contact[i];int a=h->m->geom_bodyid[c->geom1],b=h->m->geom_bodyid[c->geom2];if((a==0)==(b==0))continue;int body=a?a:b;double*p=out+64+112*count;p[0]=i;p[1]=body;p[2]=c->geom1;p[3]=c->geom2;p[4]=c->dim;p[5]=c->exclude;p[6]=c->efc_address;p[7]=c->dist;p[8]=c->includemargin;p[9]=c->mu;
  for(int j=0;j<3;j++)p[10+j]=c->pos[j];for(int j=0;j<9;j++)p[13+j]=c->frame[j];for(int j=0;j<6;j++)p[22+j]=NAN;if(fresh)mj_contactForce(h->m,d,i,p+22);for(int j=0;j<5;j++)p[28+j]=c->friction[j];for(int j=0;j<2;j++)p[33+j]=c->solref[j];for(int j=0;j<5;j++)p[35+j]=c->solimp[j];
  double J[81],v[3],normal; mj_jac(h->m,d,J,NULL,c->pos,body);for(int j=0;j<3;j++){v[j]=mju_dot(J+27*j,d->qvel,27);p[40+j]=v[j];}normal=mju_dot(v,c->frame,3);for(int j=0;j<3;j++){p[43+j]=mju_dot(c->frame+3*j,v,3);p[46+j]=v[j]-normal*c->frame[j];p[49+j]=h->m->geom_friction[3*c->geom1+j];p[52+j]=h->m->geom_friction[3*c->geom2+j];}
  p[55]=h->m->geom_condim[c->geom1];p[56]=h->m->geom_condim[c->geom2];p[57]=h->m->geom_priority[c->geom1];p[58]=h->m->geom_priority[c->geom2];p[59]=h->m->geom_solmix[c->geom1];p[60]=h->m->geom_solmix[c->geom2];int nr=c->dim==1?1:h->m->opt.cone==mjCONE_PYRAMIDAL?2*(c->dim-1):c->dim;p[61]=nr;
  for(int j=0;j<nr&&j<10;j++){int k=c->efc_address+j;if(k>=0&&k<d->nefc){p[62+j]=d->efc_R[k];p[72+j]=d->efc_vel[k];p[82+j]=d->efc_aref[k];p[92+j]=fresh?d->efc_force[k]:NAN;p[102+j]=fresh?d->efc_state[k]:-1;}}
  count++;
 }
 out[0]=count;if(scratch)mj_deleteData(scratch);return count;
}
