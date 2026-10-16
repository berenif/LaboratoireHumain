/* Diagnostic-only ABI adapter for the official MuJoCo 2.1.3 headers/library.
 * The only integration entry point counts before each mj_step2/mj_step1 pair.
 */
#include <mujoco.h>
#include <stdlib.h>
#include <string.h>

typedef struct { mjModel *m; mjData *d; int steps; } LH;
int lh_version(void) { return mj_version(); }
void *lh_open(const char *xml, char *error, int length) {
    mjModel *m = mj_loadXML(xml, NULL, error, length);
    if (!m) return NULL;
    mjData *d = mj_makeData(m);
    if (!d) { mj_deleteModel(m); return NULL; }
    LH *h = calloc(1, sizeof(LH)); h->m = m; h->d = d; return h;
}
void lh_close(LH *h) { if(h) { mj_deleteData(h->d); mj_deleteModel(h->m); free(h); } }
int lh_dim(LH *h, int k) {
    switch(k) { case 0:return h->m->nq; case 1:return h->m->nv; case 2:return h->m->nu;
    case 3:return h->m->nbody; case 4:return h->m->ngeom; case 5:return h->m->njnt;
    case 6:return h->steps; case 7:return h->d->ncon; case 8:return h->m->opt.integrator;
    default:return -1; }
}
double lh_dt(LH *h) { return h->m->opt.timestep; }
double lh_time(LH *h) { return h->d->time; }
int lh_warnings(LH *h) {
    int total=0;
    for(int i=0;i<mjNWARNING;i++) total+=h->d->warning[i].number;
    return total;
}
int lh_id(LH *h, int type, const char *name) { return mj_name2id(h->m, type, name); }
const char *lh_name(LH *h, int type, int id) { return mj_id2name(h->m,type,id); }
double *lh_array(LH *h, int k) {
    switch(k) {
    case 0:return h->d->qpos; case 1:return h->d->qvel; case 2:return h->d->ctrl;
    case 3:return h->d->xpos; case 4:return h->d->xquat; case 5:return h->d->xmat;
    case 6:return h->d->subtree_com; case 7:return h->d->subtree_linvel;
    case 8:return h->d->qfrc_actuator; case 9:return h->d->qfrc_passive;
    case 10:return h->m->body_mass; case 11:return h->m->body_ipos;
    case 12:return h->m->body_inertia; case 13:return h->m->body_iquat;
    case 14:return h->m->actuator_gear; case 15:return h->d->qacc;
    case 16:return h->m->jnt_range; case 17:return h->m->dof_damping;
    case 18:return h->m->dof_armature; case 19:return h->m->jnt_stiffness;
    default:return NULL; }
}
void lh_set_state(LH *h, const double *qpos, const double *qvel) {
    mj_resetData(h->m,h->d);
    memcpy(h->d->qpos,qpos,h->m->nq*sizeof(double));
    memcpy(h->d->qvel,qvel,h->m->nv*sizeof(double));
    mj_forward(h->m,h->d);
}
void lh_forward(LH *h) { mj_forward(h->m,h->d); }
int lh_step(LH *h, int maximum) {
    if(h->steps>=maximum) return -1;
    h->steps++;
    mj_step2(h->m,h->d);
    mj_step1(h->m,h->d);
    return h->steps;
}
int lh_contact(LH *h, int i, double *out) {
    if(i<0 || i>=h->d->ncon) return -1;
    mjContact *c=&h->d->contact[i];
    out[0]=c->geom1; out[1]=c->geom2; out[2]=h->m->geom_bodyid[c->geom1];
    out[3]=h->m->geom_bodyid[c->geom2]; out[4]=c->dist;
    for(int k=0;k<3;k++) out[5+k]=c->pos[k];
    return 0;
}
