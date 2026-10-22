//! Explicit native pilot only. Read measured state, propose references, check
//! inverse dynamics, then return joint controls. Never advance or write the world.
use crate::*;
use rapier3d::na::{DMatrix, DVector, Vector3};

const FORMAT: &str = "native-step-pilot-v2";
const DURATION: f32 = 0.8;
const REFERENCE_ACCELERATION: f32 = 2.;
const LIMIT_NATIVE: u64 = 2400;

fn pilot_budget_available(started:u64,completed:u64)->bool {
    completed.checked_sub(started).is_some_and(|elapsed|elapsed<LIMIT_NATIVE)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Phase { Retraction, Swing, Landing, Complete, Cancelled }

#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Reference {
    pub q: [f32;21],
    pub dq: [f32;21],
    start: [f32;21],
    velocity: [f32;21],
    target: [f32;21],
    elapsed: f32,
    braking: bool,
    #[serde(default="reference_duration")]
    duration: f32,
    #[serde(default)]
    limit_braking: [bool;21],
}
fn reference_duration()->f32 {DURATION}
impl Reference {
    fn new(q:[f32;21],dq:[f32;21],target:[f32;21])->Self {
        Self {q,dq,start:q,velocity:dq,target,elapsed:0.,braking:false,duration:DURATION,limit_braking:[false;21]}
    }
    /// Quintic Hermite in physical seconds, not one entire displacement per
    /// control evaluation. The measured state is used by feedback, not as a
    /// replacement for the persistent trajectory origin.
    fn proposal(&self,dt:f32,bounds:&[(f32,f32);21])->Result<(Self,[f32;21]),String> {
        if !dt.is_finite() || dt<=0. || dt>0.025 {return Err("Invalid Step reference interval".into());}
        if self.braking {
            if self.dq.iter().any(|v|*v!=0.) {return self.braking_proposal(dt,bounds);}
            // A completed whole-reference brake may resume the unfinished
            // intention from continuous q/dq. The measured world is untouched.
            let mut resumed=Self::new(self.q,self.dq,self.target);
            resumed.limit_braking=self.limit_braking;
            return resumed.proposal(dt,bounds);
        }
        let t=(self.elapsed+dt).min(self.duration);let x=t/self.duration;
        let h=10.*x.powi(3)-15.*x.powi(4)+6.*x.powi(5);
        let dh=(30.*x.powi(2)-60.*x.powi(3)+30.*x.powi(4))/self.duration;
        let ddh=(60.*x-180.*x.powi(2)+120.*x.powi(3))/self.duration.powi(2);
        let v=x-6.*x.powi(3)+8.*x.powi(4)-3.*x.powi(5);
        let dv=1.-18.*x.powi(2)+32.*x.powi(3)-15.*x.powi(4);
        let ddv=(-36.*x+96.*x.powi(2)-60.*x.powi(3))/self.duration;
        let mut next=self.clone();next.elapsed=t;let mut ddq=[0.;21];
        for i in 0..21 {
            let delta=self.target[i]-self.start[i];
            next.q[i]=self.start[i]+delta*h+self.velocity[i]*self.duration*v;
            next.dq[i]=delta*dh+self.velocity[i]*dv;
            ddq[i]=delta*ddh+self.velocity[i]*ddv;
            // A tiny inward return to one joint stop must not brake all 21
            // coordinates or prevent the support phase from progressing.
            if self.limit_braking[i] || needs_braking(self.q[i],self.dq[i],bounds[i],dt)
                || next.q[i]<bounds[i].0 || next.q[i]>bounds[i].1
                || needs_braking(next.q[i],next.dq[i],bounds[i],dt) {
                let (q,v,a)=brake_reference(self.q[i],self.dq[i],bounds[i],dt)?;
                next.q[i]=q;next.dq[i]=v;ddq[i]=a;next.limit_braking[i]=true;
            }
            if !next.q[i].is_finite() || !next.dq[i].is_finite() || !ddq[i].is_finite()
                || next.dq[i].abs()>0.5 || ddq[i].abs()>REFERENCE_ACCELERATION {
                return Err("Step reference exceeds pilot speed/acceleration envelope".into());
            }
        }
        Ok((next,ddq))
    }
    /// Explicit alpha-zero fallback: brake current reference velocities and
    /// hold once stopped. Unlike nominal proposal this never resumes a target.
    fn braking_proposal(&self,dt:f32,bounds:&[(f32,f32);21])->Result<(Self,[f32;21]),String> {
        if !dt.is_finite() || dt<=0. || dt>0.025 {return Err("Invalid Step brake interval".into());}
        let mut next=self.clone();next.braking=true;let mut acceleration=[0.;21];
        for i in 0..21 {
            let (q,v,a)=brake_reference(self.q[i],self.dq[i],bounds[i],dt)?;
            next.q[i]=q;next.dq[i]=v;acceleration[i]=a;
        }
        next.elapsed=(self.elapsed+dt).min(self.duration);
        Ok((next,acceleration))
    }

}

#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct State {
    format:String,
    pub phase:Phase,
    side:usize,
    last_native:u64,
    started_native:u64,
    reference:Reference,
    reference_acceleration:[f32;21],
    com:Vector,
    forward:Vector,
    foot_goal:Vector,
    separated:bool,
    separation_samples:u32,
    landed_samples:u32,
    #[serde(default)]
    last_reactions:Vec<Reaction>,
}
impl State {
    pub fn active(&self)->bool { self.phase!=Phase::Cancelled }
    pub fn valid(&self,native:u64)->bool {
        self.format==FORMAT && self.side<2 && self.started_native<=self.last_native
            && self.last_native<=native && self.separation_samples<=3 && self.landed_samples<=20
            && self.com.is_finite() && self.forward.is_finite()
            && self.foot_goal.is_finite() && self.reference.elapsed.is_finite()
            && self.reference.duration.is_finite() && self.reference.duration>0. && self.reference.duration<=DURATION
            && (0.0..=self.reference.duration).contains(&self.reference.elapsed)
            && self.last_reactions.iter().all(|(_,p,f)|p.is_finite() && f.is_finite())
            && self.reference_acceleration.iter().all(|x|x.is_finite() && x.abs()<=REFERENCE_ACCELERATION)
            && self.reference.q.iter().chain(&self.reference.dq).chain(&self.reference.start)
                .chain(&self.reference.velocity).chain(&self.reference.target).all(|x|x.is_finite())
    }
    pub fn cancel(&mut self) {self.phase=Phase::Cancelled;}
    #[cfg(feature="getup-native-locomotion-experiment")]
    pub fn begin(s:&Simulation,side:usize)->Result<Self,String> {
        if side>1 || s.environment.is_some() || s.grab.is_some() {return Err("Step pilot requires a named foot, flat floor and released grab".into());}
        let feet=feet(s)?;let support=geometry(s)?;
        if support.iter().any(|f|f.clearance>0. || f.load<3.) {
            return Err("Step pilot entry requires measured bilateral support".into());
        }
        let (mut q,mut dq)=super::joint_state(s)?;
        initialize_reference(s,&mut q,&mut dq);
        let mut forward=*s.world.bodies[s.bodies[0]].rotation()*Vector::Z;forward.y=0.;
        let forward=forward.try_normalize().ok_or("Invalid Step heading")?;
        let foot=s.world.bodies[s.bodies[feet[side]]].translation();
        let retained=s.world.bodies[s.bodies[feet[1-side]]].translation();
        let mut com=s.center_of_mass();let shift=retained-com;
        com+=Vector::new(shift.x,0.,shift.z).clamp_length_max(0.01);
        // Retraction starts while BOTH unilateral contacts remain available.
        let foot_goal=foot-forward*0.005+Vector::Y*0.005;
        let target=posture(s,side,com,foot_goal)?;
        Ok(Self {format:FORMAT.into(),phase:Phase::Retraction,side,last_native:s.substeps_completed,
            started_native:s.substeps_completed,reference:Reference::new(q,dq,target),reference_acceleration:[0.;21],com,forward,
            foot_goal,separated:false,separation_samples:0,landed_samples:0,last_reactions:vec![]})
    }
    pub fn command(&mut self,s:&Simulation)->Result<[f32;21],String> {
        if s.grab.is_some() {return Err("Step pilot external grab dynamics are not qualified".into());}
        let elapsed=s.substeps_completed.checked_sub(self.last_native).ok_or("Step clock reversed")?;
        if elapsed>1 {return Err("Step missed a native command boundary".into());}
        if !pilot_budget_available(self.started_native,s.substeps_completed) {return Err("Step pilot exceeded 2400 native intervals".into());}
        let support=geometry(s)?;
        if support[1-self.side].load<3. || support[1-self.side].clearance>0. {return Err("Step lost measured retained-foot support".into());}
        let foot=&support[self.side];
        if elapsed>0 {update_separation(&mut self.separated,&mut self.separation_samples,foot.clearance,foot.load);}
        if elapsed>0 && !self.reference.braking && self.phase==Phase::Retraction && self.separated {
            self.phase=Phase::Swing;self.foot_goal+=self.forward*0.025+Vector::Y*0.015;
            self.reference=Reference::new(self.reference.q,self.reference.dq,posture(s,self.side,self.com,self.foot_goal)?);
        } else if elapsed>0 && !self.reference.braking && self.reference.elapsed>=self.reference.duration {
            match self.phase {
                Phase::Retraction=>return Err("Step retraction ended without whole-capsule separation and unloading".into()),
                Phase::Swing=>{
                    self.phase=Phase::Landing;self.foot_goal.y-=0.020;
                    self.reference=Reference::new(self.reference.q,self.reference.dq,posture(s,self.side,self.com,self.foot_goal)?);
                },
                Phase::Landing=>{
                    if elapsed>0 && foot.load>=3. && foot.clearance<=0.001 {self.landed_samples=(self.landed_samples+1).min(20);} else {self.landed_samples=0;}
                    // Keep ownership after landing: admission to legacy standing
                    // is a separate handover, never a sudden target reset.
                    if self.landed_samples>=20 {self.phase=Phase::Complete;}
                },
                _=>{},
            }
        }
        let bounds=joint_bounds(s);
        let nominal=if elapsed==0 {(self.reference.clone(),self.reference_acceleration)}
            else {self.reference.proposal(s.profile.dt_s,&bounds)?};
        // Alpha zero advances time and brakes existing reference velocity. It
        // never freezes q while discarding dq or resets the measured state.
        let baseline=if elapsed==0 {nominal.clone()} else {
            self.reference.braking_proposal(s.profile.dt_s,&bounds)?
        };
        let (controls,candidate,ddq,reactions)=command_for_family(s,&baseline,&nominal,self.side,self.separated,&self.last_reactions)?;
        // This State itself is a clone owned by locomotion::step_command.
        // Reference, acceleration and intended allocation commit together.
        self.reference=candidate;self.reference_acceleration=ddq;
        self.last_reactions=reactions;self.last_native=s.substeps_completed;
        Ok(controls)
    }
}

/// Initial reference only: measured state stays untouched even beyond a stop.
fn initialize_reference(s:&Simulation,q:&mut [f32;21],dq:&mut [f32;21]) {
    for (i,(lo,hi)) in joint_bounds(s).into_iter().enumerate() {
        q[i]=q[i].clamp(lo,hi);
        if (q[i]<=lo && dq[i]<0.) || (q[i]>=hi && dq[i]>0.) {dq[i]=0.;}
    }
}
fn joint_bounds(s:&Simulation)->[(f32,f32);21] {
    let mut bounds=[(0.,0.);21];
    for p in &s.model.segments {if let Some(h)=p.reference.as_ref().and_then(|r|r.hinge.as_ref()) {
        let a=&p.joint_profile.as_ref().unwrap().axes[0];bounds[h.qpos_index]=(a.min_radians,a.max_radians);
    }}bounds
}
fn needs_braking(q:f32,v:f32,(lo,hi):(f32,f32),dt:f32)->bool {
    if v==0. {return false;}
    let distance=if v>0. {hi-q}else{q-lo};
    distance<=v*v/(2.*REFERENCE_ACCELERATION)+v.abs()*dt+1e-6
}
/// Continuous q/dq, constant bounded deceleration, then hold. This is reference
/// braking, not an impact solver. Motor feasibility is checked separately before
/// commit; if insufficient, report failure without altering the physical state.
fn brake_reference(q:f32,v:f32,(lo,hi):(f32,f32),dt:f32)->Result<(f32,f32,f32),String> {
    let t=dt.min(v.abs()/REFERENCE_ACCELERATION);
    let a=-v.signum()*REFERENCE_ACCELERATION;
    let next=q+v*t+0.5*a*t*t;
    let speed=if t<dt {0.}else{v+a*t};
    if !next.is_finite() || next<lo || next>hi {return Err("Reference cannot brake inside joint bounds".into());}
    Ok((next,speed,((speed-v)/dt).clamp(-REFERENCE_ACCELERATION,REFERENCE_ACCELERATION)))
}

fn update_separation(separated:&mut bool,count:&mut u32,clearance:f32,load:f32) {
    if !clearance.is_finite() || !load.is_finite() || clearance<=0.001 || load>=3. {
        *separated=false;*count=0;
    } else if clearance>=0.003 && load<=1. {
        *count=(*count+1).min(3);if *count>=3 {*separated=true;}
    } else if !*separated {*count=0;}
}
fn feet(s:&Simulation)->Result<[usize;2],String> {
    let find=|name|s.model.segments.iter().position(|p|p.id==name).ok_or_else(||format!("Missing {name}"));
    Ok([find("left_foot")?,find("right_foot")?])
}

#[derive(Serialize)]
pub(crate) struct FootGeometry {body:usize,pub clearance:f32,pub load:f32,pub points:Vec<Vector>}
/// Capsule extrema remain observable even with zero solver impulse or no
/// manifold. This does not claim a geometric point carries any load.
pub(crate) fn geometry(s:&Simulation)->Result<[FootGeometry;2],String> {
    let floor=s.world.colliders[s.floor].compute_aabb();
    let make=|body:usize|->Result<FootGeometry,String>{
        let b=&s.world.bodies[s.bodies[body]];
        let mut result=FootGeometry {body,clearance:f32::INFINITY,load:s.integrated_support_loads_n[body],points:vec![]};
        for p in &s.model.segments[body].reference.as_ref().ok_or("Missing foot primitives")?.primitives {
            for local in [p.a,p.b] {
                let point=b.translation()+b.rotation().normalize()*vector(local)-Vector::Y*p.radius;
                if point.x<floor.mins.x || point.x>floor.maxs.x || point.z<floor.mins.z || point.z>floor.maxs.z {
                    return Err("Step capsule extends outside flat-floor scope".into());
                }
                result.clearance=result.clearance.min(point.y-floor.maxs.y);
                result.points.push(point);
            }
        }
        if !result.clearance.is_finite() || !result.load.is_finite() || result.load<0. {return Err("Invalid capsule observation".into());}
        Ok(result)
    };
    let ids=feet(s)?;Ok([make(ids[0])?,make(ids[1])?])
}

fn parent(s:&Simulation,i:usize)->Result<usize,String> {
    s.model.segments.iter().position(|p|Some(&p.id)==s.model.segments[i].parent.as_ref())
        .filter(|p|*p<i).ok_or_else(||"Non-topological native Step tree".into())
}
fn axis_anchor(s:&Simulation,i:usize,p:usize)->Result<(Vector,Vector),String> {
    let j=s.model.segments[i].joint_profile.as_ref().ok_or("Missing Step joint")?;
    let b=&s.world.bodies[s.bodies[p]];let r=b.rotation().normalize();
    Ok(((r*rotation(j.parent_frame.rotation)*Vector::X).normalize(),b.translation()+r*vector(j.parent_frame.anchor)))
}
fn jacobian(s:&Simulation,body:usize,point:Vector)->Result<DMatrix<f32>,String> {
    let mut j=DMatrix::zeros(6,27);let root=s.world.bodies[s.bodies[0]].translation();
    for k in 0..3 {let axis=[Vector::X,Vector::Y,Vector::Z][k];j[(k,k)]=1.;let v=axis.cross(point-root);
        for r in 0..3 {j[(r,k+3)]=v[r];j[(r+3,k+3)]=axis[r];}}
    let mut i=body;while i>0 {let p=parent(s,i)?;
        if let Some(h)=s.model.segments[i].reference.as_ref().and_then(|r|r.hinge.as_ref()) {
            let (axis,anchor)=axis_anchor(s,i,p)?;let v=axis.cross(point-anchor);
            for r in 0..3 {j[(r,6+h.qpos_index)]=v[r];j[(r+3,6+h.qpos_index)]=axis[r];}
        }i=p;
    }Ok(j)
}
/// Whole-body posture intention with the six predicted root coordinates
/// eliminated by the retained-foot constraint, never actuated. Failed intention is not a proof of global infeasibility.
fn posture(s:&Simulation,side:usize,com_goal:Vector,foot_goal:Vector)->Result<[f32;21],String> {
    let retained=feet(s)?[1-side];
    let stance=jacobian(s,retained,s.world.bodies[s.bodies[retained]].translation())?;
    let root_map=stance.columns(0,6).into_owned().lu().solve(&(-stance.columns(6,21).into_owned()))
        .ok_or("Singular retained-foot root elimination")?;
    let mut elimination=DMatrix::zeros(27,21);
    elimination.view_mut((0,0),(6,21)).copy_from(&root_map);
    for i in 0..21 {elimination[(6+i,i)]=1.;}
    if (&stance*&elimination).norm()>1e-4 {return Err("Retained-foot elimination residual".into());}
    let mut normal=DMatrix::identity(21,21)*0.001;let mut rhs=DVector::zeros(21);
    let mut add=|task:DMatrix<f32>,b:DVector<f32>,w:f32| {
        let a=task*&elimination;normal+=a.transpose()*&a*w;rhs+=a.transpose()*b*w;
    };
    for (n,i) in feet(s)?.into_iter().enumerate() {
        let point=s.world.bodies[s.bodies[i]].translation();let delta=if n==side {foot_goal-point}else{Vector::ZERO};
        let mut target=DVector::zeros(6);for r in 0..3 {target[r]=delta[r];}
        add(jacobian(s,i,point)?,target,100.);
    }
    let root=jacobian(s,0,s.world.bodies[s.bodies[0]].translation())?;
    add(root.rows(3,3).into_owned(),DVector::zeros(3),10.);
    let mut com=DMatrix::zeros(3,27);
    for (i,h) in s.bodies.iter().enumerate(){let b=&s.world.bodies[*h];
        let p=b.translation()+b.rotation().normalize()*b.local_center_of_mass();
        com+=jacobian(s,i,p)?.rows(0,3)*(b.mass()/s.total_mass());}
    add(com,DVector::from_column_slice(&(com_goal-s.center_of_mass()).to_array()),10.);
    let delta=normal.lu().solve(&rhs).ok_or("Step posture allocation failed (not global infeasibility)")?;
    let (q,_)=super::joint_state(s)?;let mut target=q;
    for part in &s.model.segments {if let Some(h)=part.reference.as_ref().and_then(|r|r.hinge.as_ref()) {
        let a=&part.joint_profile.as_ref().unwrap().axes[0];let i=h.qpos_index;
        if !delta[i].is_finite() || delta[i].abs()>0.15 {return Err("Step posture outside pilot envelope".into());}
        target[i]=(q[i]+delta[i]).clamp(a.min_radians,a.max_radians);
    }}Ok(target)
}

type Reaction=(usize,Vector,Vector);
/// World origin/angular accelerations from measured twists and a predicted
/// free-root acceleration. Pure algebra, no multibody state mutation.
fn accelerations(s:&Simulation,dq:&[f32;21],ddq:&[f32;21],root:(Vector,Vector))
    ->Result<(Vec<Vector>,Vec<Vector>),String> {
    let n=s.bodies.len();let twists=super::current_twists(s)?;
    let mut linear=vec![Vector::ZERO;n];let mut angular=vec![Vector::ZERO;n];
    linear[0]=root.0;angular[0]=root.1;
    for i in 1..n {
        let p=parent(s,i)?;let origin=s.world.bodies[s.bodies[i]].translation();
        let po=s.world.bodies[s.bodies[p]].translation();let wp=twists[p].1;
        if let Some(h)=s.model.segments[i].reference.as_ref().and_then(|r|r.hinge.as_ref()) {
            let (axis,anchor)=axis_anchor(s,i,p)?;
            angular[i]=angular[p]+axis*ddq[h.qpos_index]+wp.cross(axis*dq[h.qpos_index]);
            let a=anchor-po;let b=origin-anchor;let w=twists[i].1;
            linear[i]=linear[p]+angular[p].cross(a)+wp.cross(wp.cross(a))+angular[i].cross(b)+w.cross(w.cross(b));
        } else {angular[i]=angular[p];let r=origin-po;linear[i]=linear[p]+angular[p].cross(r)+wp.cross(wp.cross(r));}
    }
    Ok((linear,angular))
}
fn retained_accelerations(s:&Simulation,dq:&[f32;21],ddq:&[f32;21],retained:usize)
    ->Result<(Vec<Vector>,Vec<Vector>),String> {
    let (bias,alpha)=accelerations(s,dq,ddq,(Vector::ZERO,Vector::ZERO))?;
    let offset=s.world.bodies[s.bodies[retained]].translation()-s.world.bodies[s.bodies[0]].translation();
    let root_alpha=-alpha[retained];let root_linear=-bias[retained]-root_alpha.cross(offset);
    let result=accelerations(s,dq,ddq,(root_linear,root_alpha))?;
    if result.0[retained].length()>1e-4 || result.1[retained].length()>1e-4 {
        return Err("Retained-foot predicted acceleration residual".into());
    }Ok(result)
}
/// Matches staged_island_solver/init.rs: multibody scenes form one solve group.
fn solver_h(s:&Simulation)->f32 {
    let extra=s.bodies.iter().map(|h|s.world.bodies[*h].additional_solver_iterations()).max().unwrap_or(0);
    s.world.integration_parameters.dt/(s.world.integration_parameters.num_solver_iterations+extra) as f32
}
/// Rapier subtracts Kq + (D+hK)v and augments M by armature+hD+h²K.
/// Add those terms once to the requested motor torque to compensate them;
/// do not apply a second passive force or alter Rapier's coefficients.
fn passive_inverse(k:f32,d:f32,armature:f32,q:f32,v:f32,a:f32,h:f32)->f32 {
    k*q+(d+h*k)*v+(armature+h*d+h*h*k)*a
}

/// Measured-state recursive Newton/Euler. Acceleration is affine; root is free
/// and its residual must be supplied by feasible ground intentions, not motors.
fn rnea(s:&Simulation,q:&[f32;21],dq:&[f32;21],ddq:&[f32;21],support:&[Reaction],retained:usize)
    ->Result<([f32;21],Vector,Vector),String> {
    let n=s.bodies.len();let twists=super::current_twists(s)?;
    let (linear,angular)=retained_accelerations(s,dq,ddq,retained)?;
    let h_step=solver_h(s);
    let mut forces=vec![Vector::ZERO;n];let mut moments=vec![Vector::ZERO;n];
    for i in 0..n {
        let b=&s.world.bodies[s.bodies[i]];let r=b.rotation().normalize();let offset=r*b.local_center_of_mass();let w=twists[i].1;
        let acceleration=linear[i]+angular[i].cross(offset)+w.cross(w.cross(offset));
        forces[i]=(acceleration+Vector::Y*9.81)*b.mass();
        let tensor=&s.model.segments[i].golden_mass.inertia;
        let inertia=|v:Vector| {let u=r.inverse()*v;let a: [f32;3]=std::array::from_fn(|row|(0..3).map(|col|tensor[row][col] as f32*u[col]).sum());r*Vector::from_array(a)};
        moments[i]=inertia(angular[i])+w.cross(inertia(w))+offset.cross(forces[i]);
    }
    for (i,point,force) in support {forces[*i]-=*force;moments[*i]-=(*point-s.world.bodies[s.bodies[*i]].translation()).cross(*force);}
    let mut torque=[0.;21];
    for i in (1..n).rev() {
        let p=parent(s,i)?;let origin=s.world.bodies[s.bodies[i]].translation();
        if let Some(h)=s.model.segments[i].reference.as_ref().and_then(|r|r.hinge.as_ref()) {
            let (axis,anchor)=axis_anchor(s,i,p)?;let a=&s.model.segments[i].joint_profile.as_ref().unwrap().axes[0];
            torque[h.actuator_index]=axis.dot(moments[i]+(origin-anchor).cross(forces[i]))
                +passive_inverse(a.passive_stiffness_nm_per_rad,a.damping_nms_per_rad,h.armature,
                    q[h.qpos_index],dq[h.qpos_index],ddq[h.qpos_index],h_step);
        }
        let f=forces[i];let m=moments[i]+(origin-s.world.bodies[s.bodies[p]].translation()).cross(f);
        forces[p]+=f;moments[p]+=m;
    }Ok((torque,forces[0],moments[0]))
}

/// Desired acceleration is an intention, not evidence that contact vanished.
/// Keep geometric contacts through bilateral unloading. Conversely a positive
/// actual gap carries no reaction, even while phase hysteresis is pending.
fn contact_can_react(gap:f32,normal_acceleration:f32)->Result<bool,String> {
    if !gap.is_finite() || !normal_acceleration.is_finite() {return Err("Invalid contact intention".into());}
    Ok(gap<=0.)
}

/// Solve the small unconstrained wrench allocation directly, without the bias
/// introduced by ridge-regularized normal equations. This is NOT a unilateral
/// solver; the existing cone checks still decide admission afterwards.
fn allocate_wrench(a:&DMatrix<f32>,desired:&DVector<f32>)->Result<DVector<f32>,String> {
    allocate_wrench_recorded(a,desired,&mut serde_json::json!({}))
}
fn allocate_wrench_recorded(a:&DMatrix<f32>,desired:&DVector<f32>,record:&mut serde_json::Value)->Result<DVector<f32>,String> {
    // Coordinates: world forces in N; world moments about torso origin in Nm.
    // Normalize moments by a fixed 1 m length for the algebraic solve. Check
    // force and moment residuals separately in their physical units below.
    let matrix=a.map(|x|x as f64);let target=desired.map(|x|x as f64);
    let svd=matrix.clone().svd(true,true);
    let cutoff=svd.singular_values.max()*1e-10;
    record["matrixRows"]=serde_json::json!(a.nrows());record["matrixColumns"]=serde_json::json!(a.ncols());
    record["matrixColumnMajor"]=serde_json::json!(a.as_slice());record["desiredWrenchN_Nm"]=serde_json::json!(desired.as_slice());
    record["singularValues"]=serde_json::json!(svd.singular_values.as_slice());
    record["rank"]=serde_json::json!(svd.singular_values.iter().filter(|x|**x>cutoff).count());
    let allocated=svd.solve(&target,cutoff).map_err(|e|format!("Step wrench SVD: {e}"))?.map(|x|x as f32);
    let residual=&matrix*allocated.map(|x|x as f64)-target;
    let force=residual.rows(0,3).norm();let moment=residual.rows(3,3).norm();
    record["allocatedForcesN"]=serde_json::json!(allocated.as_slice());
    record["residualN_Nm"]=serde_json::json!(residual.as_slice());
    if !allocated.iter().all(|x|x.is_finite()) || !force.is_finite() || !moment.is_finite() || force>0.01 || moment>0.01 {
        return Err(format!("Step contact allocation residual: force={force} N moment={moment} Nm; no global infeasibility claim"));
    }
    Ok(allocated)
}

fn force_in_cone(force:Vector,normal:Vector)->bool {
    let f=Vector3::new(force.x as f64,force.y as f64,force.z as f64);
    let n=Vector3::new(normal.x as f64,normal.y as f64,normal.z as f64).normalize();
    let load=f.dot(&n);
    f.iter().all(|x|x.is_finite()) && load>=0. && (f-n*load).norm()<=load
}

/// Common allocator: wrench equality, individual Coulomb cones, all motor
/// bands and one progression interval. The 256 bound counts whole projection
/// cycles. Anderson accelerates the same fixed point; every proposed result
/// still passes the original physical constraints after f32 conversion.
fn allocate_contact_wrench(a:&DMatrix<f32>,desired:&DVector<f32>,normals:&[Vector],record:&mut serde_json::Value)
    ->Result<DVector<f32>,String> {
    allocate_motor_wrench(a,desired,&DVector::zeros(6),normals,&DMatrix::zeros(0,a.ncols()),
        &DVector::zeros(0),&DVector::zeros(0),&DVector::zeros(0),(1.,1.),None,record).map(|x|x.0)
}
fn allocate_motor_wrench(a:&DMatrix<f32>,w0:&DVector<f32>,dw:&DVector<f32>,normals:&[Vector],
    contact:&DMatrix<f32>,t0:&DVector<f32>,dt:&DVector<f32>,caps:&DVector<f32>,
    interval:(f32,f32),warm:Option<&DVector<f32>>,record:&mut serde_json::Value)
    ->Result<(DVector<f32>,f32),String> {
    let nf=a.ncols();let nm=caps.len();let n=nf+1;let scale=400.;
    if a.nrows()!=6 || nf!=normals.len()*3 || normals.is_empty() || w0.len()!=6 || dw.len()!=6
        || contact.shape()!=(nm,nf) || t0.len()!=nm || dt.len()!=nm
        || interval.0<0. || interval.1>1. || interval.0>interval.1
        || normals.iter().any(|n|!n.is_finite() || n.length()<0.5)
        || a.iter().chain(w0.iter()).chain(dw.iter()).chain(contact.iter()).chain(t0.iter()).chain(dt.iter()).chain(caps.iter()).any(|x|!x.is_finite()) {
        return Err("Invalid joint contact/motor allocation inputs".into());
    }
    let ns:Vec<_>=normals.iter().map(|v|Vector3::new(v.x as f64,v.y as f64,v.z as f64).normalize()).collect();
    let mut eq=DMatrix::<f64>::zeros(6,n);eq.columns_mut(0,nf).copy_from(&a.map(|v|v as f64));
    eq.set_column(nf,&dw.map(|v|-(v as f64)/scale));let target=w0.map(|v|v as f64);
    let svd=eq.clone().svd(true,true);let cutoff=svd.singular_values.max()*1e-10;
    let inverse=svd.solve(&DMatrix::identity(6,6),cutoff).map_err(|e|format!("Joint affine projector: {e}"))?;
    let mut bands=DMatrix::<f64>::zeros(nm,n);bands.columns_mut(0,nf).copy_from(&contact.map(|v|v as f64));
    bands.set_column(nf,&dt.map(|v|v as f64/scale));
    let initial=allocate_wrench_recorded(a,&(w0+dw*interval.1),record)?;
    let mut x=DVector::zeros(n);x.rows_mut(0,nf).copy_from(&warm.filter(|w|w.len()==nf && w.iter().all(|v|v.is_finite())).unwrap_or(&initial).map(|v|v as f64));
    x[nf]=scale*interval.1 as f64;
    record["allocationMethod"]=serde_json::json!("bounded-dykstra-anderson-wrench-cones-motors-progression");
    record["frictionCoefficient"]=serde_json::json!(1.);record["normals"]=serde_json::json!(normals.iter().map(|n|n.to_array()).collect::<Vec<_>>());
    record["unconstrainedForcesN"]=serde_json::json!(initial.as_slice());
    record["wrenchBaseline"]=serde_json::json!(w0.as_slice());record["wrenchDelta"]=serde_json::json!(dw.as_slice());
    record["motorContactColumnMajor"]=serde_json::json!(contact.as_slice());record["motorBaselineNm"]=serde_json::json!(t0.as_slice());
    record["motorDeltaNm"]=serde_json::json!(dt.as_slice());record["motorCapsNm"]=serde_json::json!(caps.as_slice());
    record["alphaInterval"]=serde_json::json!([interval.0,interval.1]);record["warmStart"]=serde_json::json!(warm.is_some());
    let sets=nm+3;let mut state=DVector::zeros(n*(sets+1));state.rows_mut(0,n).copy_from(&x);
    let mut history:Vec<(DVector<f64>,DVector<f64>)>=vec![];
    for iteration in 0..=256 {
        if iteration>0 {
            let before=state.clone();let mut out=state.clone();x=state.rows(0,n).into_owned();
            for set in 0..sets {
                let offset=n*(set+1);let shifted=&x+state.rows(offset,n);
                let mut next=shifted.clone();
                if set==0 {next+=&inverse*(&target-&eq*&shifted);}
                else if set==1 {for (k,no) in ns.iter().enumerate() {
                    let f=Vector3::new(shifted[3*k],shifted[3*k+1],shifted[3*k+2]);let load=f.dot(no);let tangent=f-no*load;let t=tangent.norm();
                    let projected=if t<=load {f} else if t<=-load {Vector3::zeros()} else {(no+tangent/t)*((load+t)*0.5)};
                    for j in 0..3 {next[3*k+j]=projected[j];}
                }} else if set==sets-1 {next[nf]=next[nf].clamp(scale*interval.0 as f64,scale*interval.1 as f64);}
                else {
                    let j=set-2;let row=bands.row(j).transpose();let norm=row.norm_squared();
                    // Inward numerical reserve only, derived from f32 roundoff.
                    // The final RNEA still checks the unchanged physical cap.
                    let cap=(caps[j] as f64-16.*f32::EPSILON as f64*(1.+caps[j].abs() as f64)).max(0.);
                    let value=t0[j] as f64+row.dot(&shifted);
                    if norm>1e-24 {next+=row*((value.clamp(-cap,cap)-value)/norm);}
                }
                out.rows_mut(offset,n).copy_from(&(&shifted-&next));x=next;
            }
            out.rows_mut(0,n).copy_from(&x);let residual=&out-&before;
            history.push((out.clone(),residual.clone()));if history.len()>9 {history.remove(0);}
            state=out.clone();
            if history.len()>1 {
                let mut dr=DMatrix::zeros(state.len(),history.len()-1);let mut dout=dr.clone();
                for (j,(o,r)) in history.iter().take(history.len()-1).enumerate() {dr.set_column(j,&(r-&residual));dout.set_column(j,&(o-&out));}
                let svd=dr.svd(true,true);let tol=(svd.singular_values.max()*1e-10).max(1e-12);
                if let Ok(c)=svd.solve(&(-residual),tol) {
                    let accelerated=&out+dout*c;
                    if accelerated.iter().all(|v|v.is_finite()) {state=accelerated;}
                }
            }
        }
        let alpha=(x[nf]/scale) as f32;
        let mut force=x.rows(0,nf).map(|v|v as f32).into_owned();
        for (k,no) in ns.iter().enumerate() {
            let f=Vector3::new(force[3*k] as f64,force[3*k+1] as f64,force[3*k+2] as f64);let load=f.dot(no);let deficit=(f-no*load).norm()-load;
            if deficit>0. || load<0. {
                let rounded=f+no*(deficit.max(-load)+f32::EPSILON as f64*(1.+load.abs()));
                for j in 0..3 {force[3*k+j]=rounded[j] as f32;}
            }
        }
        let wanted=w0+dw*alpha;let residual=a.map(|v|v as f64)*force.map(|v|v as f64)-wanted.map(|v|v as f64);
        let torque=t0+dt*alpha+contact*&force;
        record["coneIterations"]=serde_json::json!(iteration);record["alpha"]=serde_json::json!(alpha);
        record["allocatedForcesN"]=serde_json::json!(force.as_slice());record["desiredWrenchN_Nm"]=serde_json::json!(wanted.as_slice());
        record["residualN_Nm"]=serde_json::json!(residual.as_slice());record["allocatedMotorTorqueNm"]=serde_json::json!(torque.as_slice());
        if alpha>=interval.0 && alpha<=interval.1 && force.iter().all(|v|v.is_finite())
            && residual.rows(0,3).norm()<0.0001 && residual.rows(3,3).norm()<0.0001
            && (0..normals.len()).all(|k|force_in_cone(Vector::new(force[3*k],force[3*k+1],force[3*k+2]),normals[k]))
            && torque.iter().zip(caps.iter()).all(|(t,c)|t.is_finite() && t.abs()<=*c) {return Ok((force,alpha));}
    }
    Err("Step joint allocation did not converge within 256 projections in the current affine region; not global infeasibility".into())
}

/// Exact affine region of clamp(PD(alpha), -2, 2) containing alpha=1.
/// At a breakpoint choose the regime approached from below. No search across
/// other regions: failure here never claims that the alpha=0 brake is infeasible.
fn acceleration_region(raw0:&[f32;21],raw1:&[f32;21])->([f32;21],[f32;21],f32) {
    let mut a0=[0.;21];let mut da=[0.;21];let mut low=0f32;
    for i in 0..21 {
        let v=raw1[i];let d=v-raw0[i];
        let saturated=if v>REFERENCE_ACCELERATION || (v==REFERENCE_ACCELERATION && d<0.) {Some(REFERENCE_ACCELERATION)}
            else if v< -REFERENCE_ACCELERATION || (v== -REFERENCE_ACCELERATION && d>0.) {Some(-REFERENCE_ACCELERATION)} else {None};
        if let Some(bound)=saturated {a0[i]=bound;} else {a0[i]=raw0[i];da[i]=d;}
        if d!=0. {for bound in [-REFERENCE_ACCELERATION,REFERENCE_ACCELERATION] {
            let crossing=(bound-raw0[i])/d;if crossing>0. && crossing<1. {low=low.max(crossing);}
        }}
    }(a0,da,low)
}
fn blend_reference(baseline:&(Reference,[f32;21]),nominal:&(Reference,[f32;21]),alpha:f32,dt:f32)->Result<(Reference,[f32;21]),String> {
    if alpha==1. {return Ok(nominal.clone());}
    if alpha==0. {return Ok(baseline.clone());}
    let mut r=nominal.0.clone();let a=std::array::from_fn(|i|baseline.1[i]+alpha*(nominal.1[i]-baseline.1[i]));
    for i in 0..21 {r.q[i]=baseline.0.q[i]+alpha*(nominal.0.q[i]-baseline.0.q[i]);r.dq[i]=baseline.0.dq[i]+alpha*(nominal.0.dq[i]-baseline.0.dq[i]);}
    if !r.braking {
        // Rebase the remaining quintic from the accepted continuous q/dq.
        // Preserve its physical deadline; never jump back to the old origin.
        let remaining=r.duration-r.elapsed;
        if remaining<=dt {return Err("Step cannot rebase a partial reference within its remaining duration".into());}
        r.duration=remaining;r.elapsed=0.;r.start=r.q;r.velocity=r.dq;
    }
    Ok((r,a))
}

type CommandProposal=([f32;21],Reference,[f32;21],Vec<Reaction>);
fn command_for_family(s:&Simulation,baseline:&(Reference,[f32;21]),nominal:&(Reference,[f32;21]),side:usize,exclusive:bool,warm:&[Reaction])->Result<CommandProposal,String> {
    let mut record=serde_json::json!({"native":s.substeps_completed,"nominalReference":nominal.0,
        "baselineReference":baseline.0,"side":side,"exclusive":exclusive,"units":"world N / Nm about torso origin"});
    let result=command_for_family_recorded(s,baseline,nominal,side,exclusive,warm,&mut record);
    record["error"]=serde_json::json!(result.as_ref().err());
    record["controls"]=serde_json::json!(result.as_ref().ok().map(|x|x.0));
    #[cfg(feature="numeric-integration-budget")]
    crate::integration_budget::record_step_command(record);
    result
}
fn command_for_family_recorded(s:&Simulation,baseline:&(Reference,[f32;21]),nominal:&(Reference,[f32;21]),side:usize,exclusive:bool,warm:&[Reaction],record:&mut serde_json::Value)->Result<CommandProposal,String> {
    let (q,dq)=super::joint_state(s)?;let geometry=geometry(s)?;let retained=feet(s)?[1-side];
    let raw=|r:&(Reference,[f32;21])|std::array::from_fn(|i|r.1[i]+16.*(r.0.q[i]-q[i])+8.*(r.0.dq[i]-dq[i]));
    let (a0,da,alpha_min)=acceleration_region(&raw(baseline),&raw(nominal));
    let desired_acceleration=std::array::from_fn(|i|a0[i]+da[i]);
    record["q"]=serde_json::json!(q);record["dq"]=serde_json::json!(dq);record["geometry"]=serde_json::json!(geometry);
    let ddq=&desired_acceleration;
    let (_,force,moment)=rnea(s,&q,&dq,ddq,&[],retained)?;
    let mut points=vec![];let mut candidates=vec![];let mut normals=vec![];
    let observed_contacts=super::standing::contacts_including_unloaded(s)?;
    if s.world.colliders[s.floor].friction()!=1. || geometry.iter().any(|g|s.world.colliders[s.colliders[g.body]].friction()!=1.) {
        return Err("Step cone allocator requires unchanged mu=1 foot/floor materials".into());
    }
    let (linear,angular)=retained_accelerations(s,&dq,ddq,retained)?;
    let twists=super::current_twists(s)?;
    let floor=s.world.colliders[s.floor].compute_aabb().maxs.y;
    for (n,g) in geometry.iter().enumerate() {
        if exclusive && n==side {continue;}
        for point in &g.points {
            let offset=*point-s.world.bodies[s.bodies[g.body]].translation();let omega=twists[g.body].1;
            let acceleration=linear[g.body]+angular[g.body].cross(offset)+omega.cross(omega.cross(offset));
            let used=contact_can_react(point.y-floor,acceleration.y)?;
            candidates.push(serde_json::json!({"body":g.body,"point":point.to_array(),"gap":point.y-floor,"desiredNormalAcceleration":acceleration.y,"used":used}));
            if used {
                // Same foot's closest resolved point supplies only its normal,
                // never its measured impulse. Pure flat-floor geometry is +Y
                // when no solver manifold exists (zero load is not deletion).
                let normal=observed_contacts.iter().filter(|c|c.body==g.body)
                    .min_by(|a,b|(a.point-*point).length_squared().total_cmp(&(b.point-*point).length_squared()))
                    .map_or(Vector::Y,|c|c.normal);
                points.push((g.body,*point));normals.push(normal);
            }
        }
    }
    record["candidates"]=serde_json::json!(candidates);
    if points.is_empty(){return Err("Step has no geometric support candidates".into());}
    let root=s.world.bodies[s.bodies[0]].translation();let mut a=DMatrix::zeros(6,3*points.len());
    for (k,(_,point)) in points.iter().enumerate(){for j in 0..3 {
        let e=[Vector::X,Vector::Y,Vector::Z][j];let m=(*point-root).cross(e);a[(j,3*k+j)]=1.;
        for r in 0..3 {a[(r+3,3*k+j)]=m[r];}
    }}
    let vector6=|f:Vector,m:Vector|DVector::from_column_slice(&[f.x,f.y,f.z,m.x,m.y,m.z]);
    let (t0,f0,m0)=rnea(s,&q,&dq,&a0,&[],retained)?;
    let (t1,_,_)=rnea(s,&q,&dq,ddq,&[],retained)?;
    let w0=vector6(f0,m0);let dw=vector6(force,moment)-&w0;
    let t0=DVector::from_column_slice(&t0);let dt=DVector::from_column_slice(&t1)-&t0;
    let mut contact=DMatrix::zeros(21,3*points.len());let mut caps=DVector::zeros(21);
    for (k,(body,point)) in points.iter().enumerate() {
        let jac=jacobian(s,*body,*point)?;
        for part in &s.model.segments {if let Some(h)=part.reference.as_ref().and_then(|r|r.hinge.as_ref()) {
            for axis in 0..3 {contact[(h.actuator_index,3*k+axis)]=-jac[(axis,6+h.qpos_index)];}
            caps[h.actuator_index]=part.joint_profile.as_ref().unwrap().axes[0].max_motor_torque_nm.min(h.gear.abs());
        }}
    }
    // Reuse previous intended forces only for matching live geometric points.
    // These are solver intentions, never recycled measured contact impulses.
    let mut seed=DVector::zeros(3*points.len());let mut matched=0;
    for (k,(body,point)) in points.iter().enumerate() {
        if let Some((_,_,f))=warm.iter().filter(|(b,p,_)|b==body && (*p-*point).length()<0.01)
            .min_by(|a,b|(a.1-*point).length_squared().total_cmp(&(b.1-*point).length_squared())) {
            for j in 0..3 {seed[3*k+j]=f[j];}matched+=1;
        }
    }
    record["rootOrigin"]=serde_json::json!(root.to_array());
    let (allocated,alpha)=allocate_motor_wrench(&a,&w0,&dw,&normals,&contact,&t0,&dt,&caps,(alpha_min,1.),
        if matched>0 {Some(&seed)} else {None},record)?;
    let (reference,reference_ddq)=blend_reference(baseline,nominal,alpha,s.profile.dt_s)?;
    let full_acceleration:[f32;21]=std::array::from_fn(|i|(reference_ddq[i]+16.*(reference.q[i]-q[i])+8.*(reference.dq[i]-dq[i])).clamp(-REFERENCE_ACCELERATION,REFERENCE_ACCELERATION));
    if (0..21).any(|i|(full_acceleration[i]-(a0[i]+alpha*da[i])).abs()>0.00001) {
        return Err("Step candidate left its certified affine PD region".into());
    }
    record["proposedReference"]=serde_json::json!(reference);record["referenceAcceleration"]=serde_json::json!(reference_ddq);
    record["desiredAcceleration"]=serde_json::json!(full_acceleration);
    let ddq=&full_acceleration;
    let mut support=vec![];
    for (k,(body,point)) in points.into_iter().enumerate() {
        let f=Vector::new(allocated[3*k],allocated[3*k+1],allocated[3*k+2]);
        // Conservative intention cone (mu <= existing foot coefficient 1).
        if !force_in_cone(f,normals[k]) {
            return Err(format!("Step constrained allocation violates unilateral/friction constraints: point={k} force_N={:?}; not global infeasibility",f.to_array()));
        }support.push((body,point,f));
    }
    // Both evaluations use the SAME measured q/dq and SAME desired allocation.
    let (bias,_,_)=rnea(s,&q,&dq,&[0.;21],&support,retained)?;
    let (dynamic,residual_f,residual_m)=rnea(s,&q,&dq,ddq,&support,retained)?;
    record["rneaResidualForceN"]=serde_json::json!(residual_f.to_array());
    record["rneaResidualMomentNm"]=serde_json::json!(residual_m.to_array());
    record["biasTorqueNm"]=serde_json::json!(bias);record["dynamicTorqueNm"]=serde_json::json!(dynamic);
    if residual_f.length()>0.01 || residual_m.length()>0.01 {return Err("Step unactuated-root residual".into());}
    let mut controls=[0.;21];
    for part in &s.model.segments {if let Some(h)=part.reference.as_ref().and_then(|r|r.hinge.as_ref()) {
        let axis=&part.joint_profile.as_ref().unwrap().axes[0];let i=h.qpos_index;let j=h.actuator_index;
        if reference.q[i]<axis.min_radians || reference.q[i]>axis.max_radians {
            return Err("Step proposed reference outside unchanged joint bounds".into());
        }
        let acceleration_torque=dynamic[j]-bias[j];
        let raw=(bias[j]+acceleration_torque)/h.gear;
        // Recompute the motor cap for the PROPOSED reference, before commit.
        // No clipping of measured q/dq, no impulse/limit reconstruction.
        let cap=(axis.max_motor_torque_nm/h.gear.abs()).min(1.);
        if !raw.is_finite() || raw.abs()>cap {return Err(format!("Step proposed-reference torque cap at hinge {i}"));}
        controls[j]=raw;
    }}Ok((controls,reference,reference_ddq,support))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test] fn native_step_pure_reference_single_stop_preserves_other_motion() {
        let mut r=Reference::new([0.;21],[0.02;21],[0.005;21]);
        r.q[20]=0.9999997;r.start[20]=r.q[20];r.velocity[20]=-0.000000007;r.dq[20]=r.velocity[20];r.target[20]=1.;
        // Use an outward speed at the upper stop to trigger its local brake.
        r.dq[20]=0.000000007;r.velocity[20]=r.dq[20];
        let next=r.proposal(0.00125,&[(-1.,1.);21]).unwrap().0;
        assert!(next.limit_braking[20]);assert!(!next.braking);
        assert!(next.dq[0]>0.019);assert!(next.q[0]>r.q[0]);assert_eq!(next.dq[20],0.);
    }
    #[test] fn native_step_pure_reference_global_brake_resumes_but_alpha_zero_holds() {
        let mut r=Reference::new([0.;21],[0.005;21],[0.005;21]);r.braking=true;
        r=r.proposal(0.00125,&[(-1.,1.);21]).unwrap().0;
        r=r.proposal(0.00125,&[(-1.,1.);21]).unwrap().0;
        assert!(r.dq.iter().all(|v|*v==0.));let stopped=r.clone();
        let held=r.braking_proposal(0.00125,&[(-1.,1.);21]).unwrap().0;
        assert!(held.braking);assert_eq!(held.q,stopped.q);assert_eq!(held.dq,stopped.dq);
        let next=r.proposal(0.00125,&[(-1.,1.);21]).unwrap().0;
        assert!(!next.braking);assert!(next.q[0]>=stopped.q[0]);assert!(next.dq[0]>0.);
        assert_eq!(next.start,stopped.q);assert_eq!(next.velocity,stopped.dq);
    }
    #[test]
    #[ignore="requires recorded trial observation; evolves references only, never Rapier"]
    fn native_step_pure_reference_recorded_4757_resumes_unfinished_transfer() {
        let path=std::env::var("LH_STEP_TRIAL").unwrap();let data:serde_json::Value=serde_json::from_slice(&std::fs::read(path).unwrap()).unwrap();
        let mut reference:Reference=serde_json::from_value(data["result"]["finalObservation"]["ownership"]["step"]["reference"].clone()).unwrap();
        let entry:Reference=serde_json::from_value(data["result"]["commands"]["firstAdmitted"]["proposedReference"].clone()).unwrap();
        let model=lh_model::Model::canonical().unwrap();
        let mut bounds=[(0.,0.);21];for part in &model.segments {if let Some(h)=part.reference.as_ref().and_then(|r|r.hinge.as_ref()) {
            let a=&part.joint_profile.as_ref().unwrap().axes[0];bounds[h.qpos_index]=(a.min_radians,a.max_radians);
        }}
        let mut from_entry=entry;let mut first_mask=None;
        for step in 1..=217 {let next=from_entry.proposal(0.00125,&bounds).unwrap().0;
            if first_mask.is_none() && next.limit_braking.iter().any(|x|*x) {first_mask=Some((step,next.limit_braking));}from_entry=next;}
        assert_eq!(first_mask.as_ref().unwrap().1.iter().filter(|x|**x).count(),1);assert!(first_mask.as_ref().unwrap().1[20]);
        let initial_distance: f32=(0..21).map(|i|(reference.target[i]-reference.q[i]).abs()).sum();
        let mut resumed_at=None;let mut nonzero_after=false;
        for step in 1..=700 {let next=reference.proposal(0.00125,&bounds).unwrap().0;
            assert!((0..21).all(|i|(next.q[i]-reference.q[i]).abs()<=0.5*0.00125+1e-6));
            if resumed_at.is_none() && !next.braking {resumed_at=Some(step);}
            if resumed_at.is_some() && next.dq[6].abs()>0.001 {nonzero_after=true;}
            reference=next;
        }
        let final_distance:f32=(0..21).map(|i|(reference.target[i]-reference.q[i]).abs()).sum();
        assert!(resumed_at.unwrap()<=24);assert!(nonzero_after);assert!(final_distance<initial_distance*0.1);
        println!("REFERENCE_ONLY {}",serde_json::json!({"firstLimitedCoordinateFromEntry":first_mask,"resumedAfterReferenceUpdates":resumed_at,"initialTargetDistanceRad":initial_distance,"finalTargetDistanceRad":final_distance,"newPhysicsIntegrations":0}));
    }
    #[test] fn native_step_pure_joint_allocator_progression_and_motor_limit() {
        let mut a=DMatrix::zeros(6,3);a[(1,1)]=1.;
        let mut w0=DVector::zeros(6);w0[1]=80.;let mut dw=DVector::zeros(6);dw[1]=20.;
        let mut motor=DMatrix::zeros(1,3);motor[(0,1)]=1.;let mut record=serde_json::json!({});
        let (f,alpha)=allocate_motor_wrench(&a,&w0,&dw,&[Vector::Y],&motor,&DVector::zeros(1),
            &DVector::from_element(1,100.),&DVector::from_element(1,100.),(0.,1.),None,&mut record).unwrap();
        assert!(alpha>0.1 && alpha<0.17,"{record}");
        assert!((f[1]-(80.+20.*alpha)).abs()<0.0001);assert!(f[1]+100.*alpha<=100.);
        assert!(force_in_cone(Vector::new(f[0],f[1],f[2]),Vector::Y));
    }
    #[test] fn native_step_pure_joint_alpha_brakes_and_rebases_without_velocity_reset() {
        let r=Reference::new([0.;21],[0.02;21],[0.005;21]);let nominal=r.proposal(0.00125,&[(-1.,1.);21]).unwrap();
        let mut brake=r.clone();brake.braking=true;let baseline=brake.proposal(0.00125,&[(-1.,1.);21]).unwrap();
        let (zero,_)=blend_reference(&baseline,&nominal,0.,0.00125).unwrap();assert!(zero.q[0]>0. && zero.dq[0]>0. && zero.dq[0]<r.dq[0]);
        let (half,_)=blend_reference(&baseline,&nominal,0.5,0.00125).unwrap();assert!(half.dq[0]>zero.dq[0]);
        assert_eq!(half.start,half.q);assert_eq!(half.velocity,half.dq);assert_eq!(half.elapsed,0.);
        assert!((half.duration+nominal.0.elapsed-DURATION).abs()<1e-7);
        let next=half.proposal(0.00125,&[(-1.,1.);21]).unwrap().0;
        assert!(next.q[0]>half.q[0]);assert!((next.dq[0]-half.dq[0]).abs()<0.001);
    }
    #[test] fn native_step_pure_joint_partial_terminal_reference_cannot_jump_to_target() {
        let mut nominal=Reference::new([0.;21],[0.;21],[0.01;21]);nominal.elapsed=nominal.duration;
        nominal.q=nominal.target;let mut baseline=nominal.clone();baseline.q=[0.;21];baseline.braking=true;
        assert!(blend_reference(&(baseline.clone(),[0.;21]),&(nominal.clone(),[0.;21]),0.5,0.00125).err().unwrap().contains("remaining duration"));
        nominal.elapsed=nominal.duration-0.0005;
        assert!(blend_reference(&(baseline,[0.;21]),&(nominal,[0.;21]),0.5,0.00125).is_err());
    }
    #[test] fn native_step_pure_joint_clipped_pd_region_includes_endpoint() {
        let mut raw0=[0.;21];let mut raw1=[0.;21];raw0[0]=-3.;raw1[0]=3.;
        raw0[1]=3.;raw1[1]=2.; // endpoint saturation approached from above
        let (a,da,low)=acceleration_region(&raw0,&raw1);
        assert!((low-5./6.).abs()<1e-6);
        for alpha in [low,(low+1.)*0.5,1.] {for i in 0..21 {
            let real=(raw0[i]+alpha*(raw1[i]-raw0[i])).clamp(-2.,2.);
            assert!((real-(a[i]+alpha*da[i])).abs()<1e-6);
        }}
    }
    #[test] fn native_step_pure_constrained_exact_refusal_4756() {
        let fixture:serde_json::Value=serde_json::from_str(include_str!("../../tests/fixtures/native-step-friction-4756.json")).unwrap();
        let data:Vec<f32>=serde_json::from_value(fixture["matrixColumnMajor"].clone()).unwrap();
        let wrench:Vec<f32>=serde_json::from_value(fixture["desiredWrenchN_Nm"].clone()).unwrap();
        let ns:Vec<[f32;3]>=serde_json::from_value(fixture["normals"].clone()).unwrap();
        let normals:Vec<_>=ns.into_iter().map(Vector::from_array).collect();
        let a=DMatrix::from_column_slice(6,24,&data);let b=DVector::from_column_slice(&wrench);
        let old:Vec<f32>=serde_json::from_value(fixture["allocatedForcesN"].clone()).unwrap();
        assert!(!force_in_cone(Vector::new(old[0],old[1],old[2]),normals[0]));
        let mut receipt=serde_json::json!({});let f=allocate_contact_wrench(&a,&b,&normals,&mut receipt).unwrap();
        let residual=a.map(|v|v as f64)*f.map(|v|v as f64)-b.map(|v|v as f64);
        assert!(residual.rows(0,3).norm()<0.0001);assert!(residual.rows(3,3).norm()<0.0001);
        for k in 0..8 {
            let n=Vector3::new(normals[k].x as f64,normals[k].y as f64,normals[k].z as f64).normalize();
            let force=Vector3::new(f[3*k] as f64,f[3*k+1] as f64,f[3*k+2] as f64);
            let load=force.dot(&n);assert!(load>=0.);assert!((force-n*load).norm()<=load);
        }
        assert!(receipt["coneIterations"].as_u64().unwrap()<=256);
        let repeated=allocate_contact_wrench(&a,&b,&normals,&mut serde_json::json!({})).unwrap();
        assert_eq!(f.as_slice(),repeated.as_slice());
        assert_eq!(receipt["frictionCoefficient"],1.);
    }
    #[test] fn native_step_pure_constrained_zero_and_tensile_wrench() {
        let mut a=DMatrix::zeros(6,3);for i in 0..3 {a[(i,i)]=1.;}
        let normals=[Vector::Y];
        assert_eq!(allocate_contact_wrench(&a,&DVector::zeros(6),&normals,&mut serde_json::json!({})).unwrap(),DVector::zeros(3));
        let mut receipt=serde_json::json!({});let b=DVector::from_column_slice(&[0.,-1.,0.,0.,0.,0.]);
        let error=allocate_contact_wrench(&a,&b,&normals,&mut receipt).unwrap_err();
        assert!(error.contains("not global infeasibility"));assert_eq!(receipt["coneIterations"],256);
    }
    #[test] fn native_step_pure_min_norm_removes_ridge_bias_without_admitting_tension() {
        // Algebra-only regression fixture derived from recorded extrema at
        // native4541, not a checkpoint or replay of the lost Rapier world.
        let root=Vector::new(-0.48699132,1.2573017,0.51681453);
        let points=[Vector::new(-0.29174295,-0.00026358292,0.69674224),
            Vector::new(-0.4980225,-0.0001147259,0.7528557),Vector::new(-0.31804687,-0.0007823948,0.64281785)];
        let mut a=DMatrix::zeros(6,9);
        for (k,p) in points.iter().enumerate() {for j in 0..3 {
            let e=[Vector::X,Vector::Y,Vector::Z][j];let m=(*p-root).cross(e);a[(j,3*k+j)]=1.;
            for r in 0..3 {a[(r+3,3*k+j)]=m[r];}
        }}
        let b=DVector::from_column_slice(&[2.9607882,404.15958,11.857897,2.8018434,-0.2523685,9.532738]);
        let m=a.map(|x|x as f64);let target=b.map(|x|x as f64);
        let old=m.transpose()*(&m*m.transpose()+DMatrix::identity(6,6)*1e-6).lu().solve(&target).unwrap();
        assert!((&m*old-&target).norm()>0.08);
        let force=allocate_wrench(&a,&b).unwrap();
        assert!((&m*force.map(|x|x as f64)-target).norm()<0.001);
        // Solving the equality cannot conceal the unilateral violation.
        assert!(force[1] < -1500.);
    }
    #[test] fn native_step_pure_bilateral_allocation_keeps_geometric_candidates() {
        // Six touching extrema from the saved observation; no Rapier world.
        // Checks candidate retention and wrench allocation only, NOT realization
        // of desired accelerations or dynamic unloading/transfer.
        let root=Vector::new(-0.48699132,1.2573017,0.51681453);
        let points=[Vector::new(-0.66118604,-0.00002062507,0.34502047),
            Vector::new(-0.6517557,-0.00027231313,0.36265582),Vector::new(-0.45713937,-0.0005102437,0.2812652),
            Vector::new(-0.29174295,-0.00026358292,0.69674224),Vector::new(-0.4980225,-0.0001147259,0.7528557),
            Vector::new(-0.31804687,-0.0007823948,0.64281785)];
        let mut a=DMatrix::zeros(6,18);
        for (k,p) in points.iter().enumerate() {
            assert!(contact_can_react(p.y,if k<3 {0.2}else{0.00006}).unwrap());
            for j in 0..3 {let e=[Vector::X,Vector::Y,Vector::Z][j];let m=(*p-root).cross(e);a[(j,3*k+j)]=1.;
                for r in 0..3 {a[(r+3,3*k+j)]=m[r];}}
        }
        let b=DVector::from_column_slice(&[2.9607882,404.15958,11.857897,2.8018434,-0.2523685,9.532738]);
        let mut receipt=serde_json::json!({});let forces=allocate_wrench_recorded(&a,&b,&mut receipt).unwrap();
        assert_eq!(receipt["rank"],6);assert_eq!(receipt["matrixColumnMajor"].as_array().unwrap().len(),108);
        for k in 0..6 {let f=Vector::new(forces[3*k],forces[3*k+1],forces[3*k+2]);
            assert!(f.y>0. && Vector::new(f.x,0.,f.z).length()<f.y);}
        assert!(!contact_can_react(0.004,0.2).unwrap());
    }
    #[test] fn native_step_pure_min_norm_rejects_unrepresentable_wrench() {
        let mut a=DMatrix::zeros(6,3);for i in 0..3 {a[(i,i)]=1.;}
        let b=DVector::from_column_slice(&[0.,400.,0.,1.,0.,0.]);
        assert!(allocate_wrench(&a,&b).is_err());
    }
    #[test] fn native_step_pure_budget_refuses_command_for_integration_2401() {
        assert!(pilot_budget_available(4800,4800));
        assert!(pilot_budget_available(4800,7199));
        assert!(!pilot_budget_available(4800,7200));
        assert!(!pilot_budget_available(4800,4799));
    }
    #[test] fn native_step_pure_rnea_is_affine_at_fixed_state_and_allocation() {
        let s=Simulation::new(0.,crate::Profile::default(),true).unwrap();
        let before=serde_json::to_value(&s).unwrap();let (q,dq)=super::super::joint_state(&s).unwrap();
        let ids=feet(&s).unwrap();let allocation=vec![(ids[0],s.world.bodies[s.bodies[ids[0]]].translation(),Vector::Y*20.)];
        let (zero,f0,m0)=rnea(&s,&q,&dq,&[0.;21],&allocation,ids[0]).unwrap();
        let (full,f1,m1)=rnea(&s,&q,&dq,&[0.1;21],&allocation,ids[0]).unwrap();
        let (half,fh,mh)=rnea(&s,&q,&dq,&[0.05;21],&allocation,ids[0]).unwrap();
        for i in 0..21 {assert!((half[i]-(zero[i]+full[i])*0.5).abs()<1e-3);}
        assert!((fh-(f0+f1)*0.5).length()<1e-3);assert!((mh-(m0+m1)*0.5).length()<1e-3);
        assert_eq!(serde_json::to_value(&s).unwrap(),before);assert_eq!(s.substeps_completed,0);
    }
    #[test] fn native_step_pure_reference_is_cadence_independent() {
        let start=Reference::new([0.;21],[0.;21],[0.005;21]);let mut fine=start.clone();let mut coarse=start;
        for _ in 0..20 {fine=fine.proposal(0.00125,&[(-1.,1.);21]).unwrap().0;}
        coarse=coarse.proposal(0.025,&[(-1.,1.);21]).unwrap().0;
        assert!((fine.q[0]-coarse.q[0]).abs()<1e-8);assert!(fine.q[0]<0.00001);
        assert!(fine.dq[0]>0.);assert_eq!(fine.start,[0.;21]);
    }
    #[test] fn native_step_pure_reference_preserves_velocity_and_rejects_without_commit() {
        let r=Reference::new([0.;21],[0.02;21],[0.005;21]);let (next,_)=r.proposal(0.00125,&[(-1.,1.);21]).unwrap();
        assert!((next.dq[0]-0.02).abs()<1e-5);assert_eq!(r.elapsed,0.);
        assert!(r.proposal(f32::NAN,&[(-1.,1.);21]).is_err());assert_eq!(r.q,[0.;21]);
    }
    #[test] fn native_step_pure_reference_brakes_before_stop_continuously() {
        let measured=(0.999,0.02);let (q,v,a)=brake_reference(measured.0,measured.1,(-1.,1.),0.00125).unwrap();
        assert!(q>measured.0 && q<1.);assert!(v>0. && v<measured.1);
        assert!((a+REFERENCE_ACCELERATION).abs()<1e-5);assert_eq!(measured,(0.999,0.02));
        let (q,v,_) = brake_reference(q,v,(-1.,1.),0.025).unwrap();assert!(q<1.);assert_eq!(v,0.);
        assert!(brake_reference(1.,0.1,(-1.,1.),0.00125).is_err());
    }
    #[test] fn native_step_pure_limit_braking_holds_only_the_stopped_coordinates() {
        let mut r=Reference::new([0.99997;21],[0.01;21],[1.;21]);
        for _ in 0..10 {let old=r.clone();r=r.proposal(0.00125,&[(-1.,1.);21]).unwrap().0;
            assert!(r.q[0]>=old.q[0] && r.q[0]<=1.);assert!(r.dq[0]<=old.dq[0]);}
        assert!(!r.braking);assert!(r.limit_braking.iter().all(|x|*x));assert_eq!(r.dq[0],0.);let held=r.q;
        r=r.proposal(0.025,&[(-1.,1.);21]).unwrap().0;assert_eq!(r.q,held);
    }
    #[test] fn native_step_pure_hysteresis_never_creates_absent_contact_force() {
        assert!(!contact_can_react(0.00001,-1.).unwrap());
        assert!(contact_can_react(0.,0.).unwrap()); // zero charge still may contact
        assert!(contact_can_react(0.,0.01).unwrap()); // desire does not remove geometry
        assert!(contact_can_react(0.,-0.01).unwrap()); // allocation must coordinate contact, not discard it
        let mut separated=false;let mut count=0;
        update_separation(&mut separated,&mut count,0.004,0.);
        assert!(!separated);assert!(!contact_can_react(0.004,0.).unwrap());
    }
    #[test] fn native_step_pure_implicit_passives_match_rapier_equation_once() {
        let (k,d,arm,q,v,a,h)=(5.,2.,0.1,0.2,-0.3,0.4,0.00125);
        let motor=passive_inverse(k,d,arm,q,v,a,h);
        let rhs=motor-k*q-(d+h*k)*v;
        assert!((rhs-(arm+h*d+h*h*k)*a).abs()<1e-6);
    }
    #[test] fn native_step_pure_retained_root_is_eliminated_not_dropped() {
        let s=Simulation::new(0.,crate::Profile::default(),true).unwrap();
        let (_,dq)=super::super::joint_state(&s).unwrap();
        let (linear,angular)=retained_accelerations(&s,&dq,&[0.2;21],feet(&s).unwrap()[0]).unwrap();
        let foot=feet(&s).unwrap()[0];assert!(linear[foot].length()<1e-4);assert!(angular[foot].length()<1e-4);
        assert!(linear[0].length()+angular[0].length()>1e-5);assert_eq!(s.substeps_completed,0);
    }
    #[test] fn native_step_pure_unloaded_geometry_is_not_liftoff() {
        let mut separated=false;let mut count=0;
        for _ in 0..10 {update_separation(&mut separated,&mut count,0.,0.);}
        assert!(!separated);
        for _ in 0..3 {update_separation(&mut separated,&mut count,0.004,0.);}
        assert!(separated);update_separation(&mut separated,&mut count,0.002,2.);assert!(separated);
        update_separation(&mut separated,&mut count,0.002,4.);assert!(!separated);
    }
}
