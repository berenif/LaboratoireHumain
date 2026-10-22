//! Explicit native pilot only. Read measured state, propose references, check
//! inverse dynamics, then return joint controls. Never advance or write the world.
use crate::*;
use rapier3d::na::{DMatrix, DVector};

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
}
impl Reference {
    fn new(q:[f32;21],dq:[f32;21],target:[f32;21])->Self {
        Self {q,dq,start:q,velocity:dq,target,elapsed:0.,braking:false}
    }
    /// Quintic Hermite in physical seconds, not one entire displacement per
    /// control evaluation. The measured state is used by feedback, not as a
    /// replacement for the persistent trajectory origin.
    fn proposal(&self,dt:f32,bounds:&[(f32,f32);21])->Result<(Self,[f32;21]),String> {
        if !dt.is_finite() || dt<=0. || dt>0.025 {return Err("Invalid Step reference interval".into());}
        if self.braking || (0..21).any(|i|needs_braking(self.q[i],self.dq[i],bounds[i],dt)) {
            let mut next=self.clone();next.braking=true;let mut acceleration=[0.;21];
            for i in 0..21 {
                let (q,v,a)=brake_reference(self.q[i],self.dq[i],bounds[i],dt)?;
                next.q[i]=q;next.dq[i]=v;acceleration[i]=a;
            }
            next.elapsed=(self.elapsed+dt).min(DURATION);
            return Ok((next,acceleration));
        }
        let t=(self.elapsed+dt).min(DURATION);let x=t/DURATION;
        let h=10.*x.powi(3)-15.*x.powi(4)+6.*x.powi(5);
        let dh=(30.*x.powi(2)-60.*x.powi(3)+30.*x.powi(4))/DURATION;
        let ddh=(60.*x-180.*x.powi(2)+120.*x.powi(3))/DURATION.powi(2);
        let v=x-6.*x.powi(3)+8.*x.powi(4)-3.*x.powi(5);
        let dv=1.-18.*x.powi(2)+32.*x.powi(3)-15.*x.powi(4);
        let ddv=(-36.*x+96.*x.powi(2)-60.*x.powi(3))/DURATION;
        let mut next=self.clone();next.elapsed=t;let mut ddq=[0.;21];
        for i in 0..21 {
            let delta=self.target[i]-self.start[i];
            next.q[i]=self.start[i]+delta*h+self.velocity[i]*DURATION*v;
            next.dq[i]=delta*dh+self.velocity[i]*dv;
            ddq[i]=delta*ddh+self.velocity[i]*ddv;
            if !next.q[i].is_finite() || !next.dq[i].is_finite() || !ddq[i].is_finite()
                || next.dq[i].abs()>0.5 || ddq[i].abs()>REFERENCE_ACCELERATION {
                return Err("Step reference exceeds pilot speed/acceleration envelope".into());
            }
        }
        if (0..21).any(|i|next.q[i]<bounds[i].0 || next.q[i]>bounds[i].1
            || needs_braking(next.q[i],next.dq[i],bounds[i],dt)) {
            let mut braking=self.clone();braking.braking=true;
            return braking.proposal(dt,bounds);
        }
        Ok((next,ddq))
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
}
impl State {
    pub fn active(&self)->bool { self.phase!=Phase::Cancelled }
    pub fn valid(&self,native:u64)->bool {
        self.format==FORMAT && self.side<2 && self.started_native<=self.last_native
            && self.last_native<=native && self.separation_samples<=3 && self.landed_samples<=20
            && self.com.is_finite() && self.forward.is_finite()
            && self.foot_goal.is_finite() && self.reference.elapsed.is_finite()
            && (0.0..=DURATION).contains(&self.reference.elapsed)
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
            foot_goal,separated:false,separation_samples:0,landed_samples:0})
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
        } else if elapsed>0 && !self.reference.braking && self.reference.elapsed>=DURATION {
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
        let (candidate,ddq)=if elapsed==0 {(self.reference.clone(),self.reference_acceleration)}
            else {self.reference.proposal(s.profile.dt_s,&joint_bounds(s))?};
        // Transaction: measured q/dq never overwritten; candidate is committed
        // only after bounds AND both dynamics evaluations pass.
        let controls=command_for_reference(s,&candidate,&ddq,self.side,self.separated)?;
        self.reference=candidate;self.reference_acceleration=ddq;self.last_native=s.substeps_completed;
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

/// Phase hysteresis is not a contact constraint. A positive geometric gap or a
/// separating desired normal acceleration carries ZERO intended reaction even
/// while phase remains Retraction. A penetrating intention is refused; it is
/// not repaired by inventing a root wrench or an impact solve.
fn contact_can_react(gap:f32,normal_acceleration:f32)->Result<bool,String> {
    if !gap.is_finite() || !normal_acceleration.is_finite() {return Err("Invalid contact intention".into());}
    if gap>0. {return Ok(false);}
    if normal_acceleration < -1e-4 {return Err("Step intention accelerates into a geometric contact".into());}
    Ok(normal_acceleration<=1e-4)
}

fn command_for_reference(s:&Simulation,reference:&Reference,ddq:&[f32;21],side:usize,exclusive:bool)->Result<[f32;21],String> {
    let (q,dq)=super::joint_state(s)?;let geometry=geometry(s)?;
    let retained=feet(s)?[1-side];
    let desired_acceleration=std::array::from_fn(|i|
        (ddq[i]+16.*(reference.q[i]-q[i])+8.*(reference.dq[i]-dq[i]))
            .clamp(-REFERENCE_ACCELERATION,REFERENCE_ACCELERATION));
    let ddq=&desired_acceleration;
    let (_,force,moment)=rnea(s,&q,&dq,ddq,&[],retained)?;
    let mut points=vec![];
    let (linear,angular)=retained_accelerations(s,&dq,ddq,retained)?;
    let twists=super::current_twists(s)?;
    let floor=s.world.colliders[s.floor].compute_aabb().maxs.y;
    for (n,g) in geometry.iter().enumerate() {
        if exclusive && n==side {continue;}
        for point in &g.points {
            let offset=*point-s.world.bodies[s.bodies[g.body]].translation();let omega=twists[g.body].1;
            let acceleration=linear[g.body]+angular[g.body].cross(offset)+omega.cross(omega.cross(offset));
            if contact_can_react(point.y-floor,acceleration.y)? {points.push((g.body,*point));}
        }
    }
    if points.is_empty(){return Err("Step has no geometric support candidates".into());}
    let root=s.world.bodies[s.bodies[0]].translation();let mut a=DMatrix::zeros(6,3*points.len());
    for (k,(_,point)) in points.iter().enumerate(){for j in 0..3 {
        let e=[Vector::X,Vector::Y,Vector::Z][j];let m=(*point-root).cross(e);a[(j,3*k+j)]=1.;
        for r in 0..3 {a[(r+3,3*k+j)]=m[r];}
    }}
    let desired=DVector::from_column_slice(&[force.x,force.y,force.z,moment.x,moment.y,moment.z]);
    let dual=(&a*a.transpose()+DMatrix::identity(6,6)*1e-6).lu().solve(&desired)
        .ok_or("Step minimum-norm contact allocation failed; no global infeasibility claim")?;
    let allocated=a.transpose()*dual;
    if (&a*&allocated-&desired).norm()>0.01 {return Err("Step contact allocation residual exceeds pilot tolerance".into());}
    let mut support=vec![];
    for (k,(body,point)) in points.into_iter().enumerate() {
        let f=Vector::new(allocated[3*k],allocated[3*k+1],allocated[3*k+2]);
        // Conservative intention cone (mu <= existing foot coefficient 1).
        if !f.is_finite() || f.y<0. || Vector::new(f.x,0.,f.z).length()>f.y {
            return Err("Step minimum-norm allocation violates unilateral/friction constraints; not global infeasibility".into());
        }support.push((body,point,f));
    }
    // Both evaluations use the SAME measured q/dq and SAME desired allocation.
    let (bias,_,_)=rnea(s,&q,&dq,&[0.;21],&support,retained)?;
    let (dynamic,residual_f,residual_m)=rnea(s,&q,&dq,ddq,&support,retained)?;
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
    }}Ok(controls)
}

#[cfg(test)]
mod tests {
    use super::*;
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
    #[test] fn native_step_pure_braking_latches_without_restarting_old_trajectory() {
        let mut r=Reference::new([0.99997;21],[0.01;21],[1.;21]);
        for _ in 0..10 {let old=r.clone();r=r.proposal(0.00125,&[(-1.,1.);21]).unwrap().0;
            assert!(r.q[0]>=old.q[0] && r.q[0]<=1.);assert!(r.dq[0]<=old.dq[0]);}
        assert!(r.braking);assert_eq!(r.dq[0],0.);let held=r.q;
        r=r.proposal(0.025,&[(-1.,1.);21]).unwrap().0;assert_eq!(r.q,held);
    }
    #[test] fn native_step_pure_hysteresis_never_creates_absent_contact_force() {
        assert!(!contact_can_react(0.00001,-1.).unwrap());
        assert!(contact_can_react(0.,0.).unwrap()); // zero charge still may contact
        assert!(!contact_can_react(0.,0.01).unwrap()); // separating, force must be zero
        assert!(contact_can_react(0.,-0.01).is_err());
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
