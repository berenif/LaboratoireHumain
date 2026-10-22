//! Explicit native pilot only. Read measured state, propose references, check
//! inverse dynamics, then return joint controls. Never advance or write the world.
use crate::*;
use rapier3d::na::{DMatrix, DVector, Vector3};

const FORMAT: &str = "native-step-pilot-v2";
const DURATION: f32 = 0.8;
const MAX_REFERENCE_DURATION: f32 = 3.; // pilot execution horizon, not a capture certificate
const SOLE_GUARD: f32 = 0.020;
const TRANSLATION_PRIORITY: f32 = 100.;
const POSTURE_PRIORITY: f32 = 1.;
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
    #[serde(default)]
    acceleration: [f32;21],
    #[serde(default)]
    acceleration_initialized: bool,
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
        Self {q,dq,acceleration:[0.;21],acceleration_initialized:true,start:q,velocity:dq,target,elapsed:0.,braking:false,duration:DURATION,limit_braking:[false;21]}
    }
    /// Replan only for a changed target, preserving accepted q/v/a. Estimate
    /// settling time using envelopes of this flow, not the removed quintic.
    /// Partial allocation can delay arrival: actual reference settling is also
    /// checked at phase boundaries. The original pilot deadline never changes.
    fn retimed(&self)->Result<Self,String> {
        let settled_by=|t:f64|->bool {
            let w=4./DURATION as f64;let decay=(-w*t).exp();
            (0..21).all(|i| {
                if self.limit_braking[i] {return true;}
                let c0=(self.q[i]-self.target[i]) as f64;
                let c1=self.dq[i] as f64+w*c0;
                let c2=0.5*(self.acceleration[i] as f64+2.*w*self.dq[i] as f64+w*w*c0);
                let p=c0.abs()+c1.abs()*t+c2.abs()*t*t;
                let v=(c1-w*c0).abs()+(2.*c2-w*c1).abs()*t+(w*c2).abs()*t*t;
                let a=(2.*c2-2.*w*c1+w*w*c0).abs()+(w*w*c1-4.*w*c2).abs()*t+(w*w*c2).abs()*t*t;
                p*decay<=0.001 && v*decay<=0.005 && a*decay<=0.02
            })
        };
        let mut lo=DURATION as f64;let mut hi=MAX_REFERENCE_DURATION as f64;
        if !settled_by(hi) {return Err("Step realizable reference exceeds pilot horizon: change preparation/placement; physical capture not certified".into());}
        // Past 0.8 s all degree <=2 exponential envelope terms decrease.
        for _ in 0..24 {let mid=(lo+hi)*0.5;if settled_by(mid) {hi=mid;} else {lo=mid;}}
        let mut r=self.clone();r.start=r.q;r.velocity=r.dq;r.elapsed=0.;
        r.duration=(hi as f32*(1.+16.*f32::EPSILON)).min(MAX_REFERENCE_DURATION);
        Ok(r)
    }
    fn settled(&self)->bool {
        (0..21).all(|i|self.limit_braking[i] || ((self.q[i]-self.target[i]).abs()<=0.001
            && self.dq[i].abs()<=0.005 && self.acceleration[i].abs()<=0.02))
    }
    /// Exact local third-order critically damped reference flow (5 / s).
    /// The accepted q/v/a is its initial condition, including after allocation.
    /// Target replacement changes jerk, never q/v/a; no polynomial clock restarts.
    /// This is a design trajectory, not a physical feasibility certificate.
    fn proposal(&self,dt:f32,bounds:&[(f32,f32);21])->Result<(Self,[f32;21]),String> {
        if !dt.is_finite() || dt<=0. || dt>0.025 {return Err("Invalid Step reference interval".into());}
        if self.braking && self.dq.iter().any(|v|*v!=0.) {return self.braking_proposal(dt,bounds);}
        let t=(self.elapsed+dt).min(self.duration);
        let mut next=self.clone();next.elapsed=t;let mut ddq=[0.;21];
        for i in 0..21 {
            let w=4. / DURATION as f64;let h=dt as f64;
            let c0=(self.q[i]-self.target[i]) as f64;
            let c1=self.dq[i] as f64+w*c0;
            let c2=0.5*(self.acceleration[i] as f64+2.*w*self.dq[i] as f64+w*w*c0);
            let p=c0+c1*h+c2*h*h;let dp=c1+2.*c2*h;let decay=(-w*h).exp();
            next.q[i]=(self.target[i] as f64+p*decay) as f32;
            next.dq[i]=((dp-w*p)*decay) as f32;
            ddq[i]=((2.*c2-2.*w*dp+w*w*p)*decay) as f32;
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
        next.braking=false;next.acceleration=ddq;
        for i in 0..21 {if next.limit_braking[i] && next.dq[i]==0. {next.acceleration[i]=0.;}}
        Ok((next,ddq))
    }
    /// Explicit alpha-zero fallback: brake current reference velocities and
    /// hold once stopped. Unlike nominal proposal this never resumes a target.
    fn braking_proposal(&self,dt:f32,bounds:&[(f32,f32);21])->Result<(Self,[f32;21]),String> {
        if !dt.is_finite() || dt<=0. || dt>0.025 {return Err("Invalid Step brake interval".into());}
        let mut next=self.clone();next.braking=true;let mut acceleration=[0.;21];
        for i in 0..21 {
            // A limited allocation must not inject a fresh +/-2 acceleration
            // every tick. Smooth ordinary braking from the accepted acceleration.
            // Joint-stop braking remains higher priority and bounded as before.
            let a0=self.acceleration[i];
            let goal=(-self.dq[i]/0.025).clamp(-REFERENCE_ACCELERATION,REFERENCE_ACCELERATION);
            let a1=a0+(goal-a0)*(1.-(-dt/0.025).exp());
            let mut v=self.dq[i]+(a0+a1)*dt*0.5;
            let mut q=self.q[i]+self.dq[i]*dt+(2.*a0+a1)*dt*dt/6.;
            let mut terminal=a1;
            if self.limit_braking[i] || needs_braking(self.q[i],self.dq[i],bounds[i],dt)
                || q<bounds[i].0 || q>bounds[i].1
                || (self.dq[i]!=0. && self.dq[i]*v<=0.) {
                let stopped=brake_reference(self.q[i],self.dq[i],bounds[i],dt)?;
                q=stopped.0;v=stopped.1;terminal=if v==0. {0.} else {stopped.2};
            }
            if !q.is_finite() || !v.is_finite() || v.abs()>0.5 || terminal.abs()>REFERENCE_ACCELERATION {
                return Err("Step brake exceeds pilot speed/acceleration envelope".into());
            }
            next.q[i]=q;next.dq[i]=v;next.acceleration[i]=terminal;
            acceleration[i]=((v-self.dq[i])/dt).clamp(-REFERENCE_ACCELERATION,REFERENCE_ACCELERATION);
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
    #[serde(default)]
    preparation:Option<Preparation>,
}
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Preparation {started:u64,updated:u64,com_origin:Vector,toward:Vector,clearance:f32,
    #[serde(default)] attempted:Option<u64>,
    #[serde(default)] retarget_rejection:Option<String>,
}
fn horizontal(v:Vector)->Vector {Vector::new(v.x,0.,v.z)}
/// Pendulum time is a directional heuristic from measured height and speed,
/// never a requirement to achieve a static COM projection before lifting.
fn transfer_goal(com:Vector,velocity:Vector,support_center:Vector)->Vector {
    let time=((com.y-support_center.y).max(0.01)/9.81).sqrt();
    let goal=support_center-horizontal(velocity)*time;
    Vector::new(goal.x,com.y,goal.z)
}
fn guarded_foot_goal(foot:Vector,clearance:f32,forward:Vector)->Vector {
    foot-forward*SOLE_GUARD+Vector::Y*(SOLE_GUARD-clearance)
}
fn measured_com_velocity(s:&Simulation)->Result<Vector,String> {
    let twists=super::current_twists(s)?;
    Ok(s.bodies.iter().zip(twists).map(|(h,t)|t.0*s.world.bodies[*h].mass()).sum::<Vector>()/s.total_mass())
}
fn support_center(g:&FootGeometry)->Vector {g.points.iter().copied().sum::<Vector>()/g.points.len() as f32}

impl State {
    pub fn active(&self)->bool { self.phase!=Phase::Cancelled }
    pub fn valid(&self,native:u64)->bool {
        self.format==FORMAT && self.side<2 && self.started_native<=self.last_native
            && self.last_native<=native && self.separation_samples<=3 && self.landed_samples<=20
            && self.com.is_finite() && self.forward.is_finite()
            && self.foot_goal.is_finite() && self.reference.elapsed.is_finite()
            && self.reference.duration.is_finite() && self.reference.duration>0. && self.reference.duration<=MAX_REFERENCE_DURATION
            && (0.0..=self.reference.duration).contains(&self.reference.elapsed)
            && self.preparation.as_ref().is_none_or(|p|p.started<=p.updated && p.updated<=native && p.attempted.is_none_or(|n|n>=p.updated && n<=native) && p.com_origin.is_finite() && p.toward.is_finite() && p.clearance.is_finite())
            && self.last_reactions.iter().all(|(_,p,f)|p.is_finite() && f.is_finite())
            && self.reference_acceleration.iter().all(|x|x.is_finite() && x.abs()<=REFERENCE_ACCELERATION)
            && self.reference.q.iter().chain(&self.reference.dq).chain(&self.reference.start)
                .chain(&self.reference.velocity).chain(&self.reference.acceleration).chain(&self.reference.target).all(|x|x.is_finite())
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
        let com=transfer_goal(s.center_of_mass(),measured_com_velocity(s)?,support_center(&support[1-side]));
        // Retreat and lift begin concurrently with dynamic support preparation.
        let foot_goal=guarded_foot_goal(foot,support[side].clearance,forward);
        let target=posture(s,side,com,foot_goal,SOLE_GUARD)?;
        Ok(Self {format:FORMAT.into(),phase:Phase::Retraction,side,last_native:s.substeps_completed,
            started_native:s.substeps_completed,reference:Reference::new(q,dq,target).retimed()?,reference_acceleration:[0.;21],com,forward,
            foot_goal,separated:false,separation_samples:0,landed_samples:0,last_reactions:vec![],preparation:None})
    }
    pub fn command(&mut self,s:&Simulation)->Result<[f32;21],String> {
        if s.grab.is_some() {return Err("Step pilot external grab dynamics are not qualified".into());}
        let elapsed=s.substeps_completed.checked_sub(self.last_native).ok_or("Step clock reversed")?;
        if elapsed>1 {return Err("Step missed a native command boundary".into());}
        if !pilot_budget_available(self.started_native,s.substeps_completed) {return Err("Step pilot exceeded 2400 native intervals".into());}
        // Older checkpoints store accepted acceleration on State only.
        if !self.reference.acceleration_initialized {
            self.reference.acceleration=std::array::from_fn(|i| {
                if self.reference.dq[i]==0. && (self.reference.braking || self.reference.limit_braking[i]) {0.}
                else {self.reference_acceleration[i]}
            });
            self.reference.acceleration_initialized=true;
        }
        let support=geometry(s)?;
        if support[1-self.side].load<3. || support[1-self.side].clearance>0. {return Err("Step lost measured retained-foot support".into());}
        let foot=&support[self.side];
        if elapsed>0 {update_separation(&mut self.separated,&mut self.separation_samples,foot.clearance,foot.load);}
        if self.phase==Phase::Retraction {
            if let Some(p)=&self.preparation {
                if s.substeps_completed.saturating_sub(p.started)>=640
                    && horizontal(s.center_of_mass()-p.com_origin).dot(p.toward)<0.001
                    && foot.clearance-p.clearance<0.001 {
                    return Err("Step preparation stayed stationary for 0.8 s: stop this candidate, no standing fallback".into());
                }
            }
            let refresh=self.preparation.as_ref().is_none_or(|p|s.substeps_completed.saturating_sub(p.attempted.unwrap_or(p.updated))>=40);
            if refresh {
                let center=support_center(&support[1-self.side]);
                let proposed_com=transfer_goal(s.center_of_mass(),measured_com_velocity(s)?,center);
                let origin=s.world.bodies[s.bodies[feet(s)?[self.side]]].translation();
                let mut proposed_foot=self.foot_goal;
                if self.preparation.is_none() {proposed_foot=guarded_foot_goal(origin,foot.clearance,self.forward);}
                else {proposed_foot.y=origin.y+SOLE_GUARD-foot.clearance;}
                let mut proposed_reference=self.reference.clone();
                proposed_reference.target=posture(s,self.side,proposed_com,proposed_foot,SOLE_GUARD)?;
                // retimed currently refuses only its planning horizon. Such a
                // refusal is optional too; actual command limits still propagate.
                let proposed_reference=proposed_reference.retimed();
                let remaining=(LIMIT_NATIVE-(s.substeps_completed-self.started_native)) as f32*s.profile.dt_s;
                let proposed_duration=proposed_reference.as_ref().ok().map(|r|r.duration);
                let retiming_reason=proposed_reference.as_ref().err().cloned();
                let adopt=admit_retarget(&self.reference,proposed_duration,remaining,s.profile.dt_s,&joint_bounds(s))?;
                if adopt {
                    self.com=proposed_com;self.foot_goal=proposed_foot;self.reference=proposed_reference.expect("admitted retiming");
                    if let Some(p)=&mut self.preparation {p.updated=s.substeps_completed;} else {
                        self.preparation=Some(Preparation {started:s.substeps_completed,updated:s.substeps_completed,com_origin:s.center_of_mass(),
                            toward:horizontal(center-s.center_of_mass()).try_normalize().unwrap_or(Vector::ZERO),clearance:foot.clearance,
                            attempted:None,retarget_rejection:None});
                    }
                }
                if let Some(p)=&mut self.preparation {
                    p.attempted=Some(s.substeps_completed);
                    p.retarget_rejection=if adopt {None} else {Some(format!("optional retarget duration {:?} s / {:?}; kept active reference with {} s left inside {} s pilot",proposed_duration,retiming_reason,self.reference.duration-self.reference.elapsed,remaining))};
                }
            }
        }
        if elapsed>0 && !self.reference.braking && self.phase==Phase::Retraction && self.separated {
            self.phase=Phase::Swing;
            let com=s.center_of_mass();let velocity=measured_com_velocity(s)?;
            let center=support_center(&support[1-self.side]);let time=((com.y-center.y).max(0.01)/9.81).sqrt();
            let origin=s.world.bodies[s.bodies[feet(s)?[self.side]]].translation();
            let lateral=Vector::Y.cross(self.forward);
            // Velocity-based catch placement retains the measured lateral side,
            // without a prescribed 134 mm transfer or a static-balance gate.
            let catch=com+horizontal(velocity)*time+lateral*horizontal(origin-com).dot(lateral);
            self.foot_goal=Vector::new(catch.x,origin.y+SOLE_GUARD-foot.clearance,catch.z);
            self.reference.target=posture(s,self.side,self.com,self.foot_goal,SOLE_GUARD)?;
            self.reference=self.reference.retimed()?;
        } else if elapsed>0 && !self.reference.braking && self.reference.elapsed>=self.reference.duration {
            match self.phase {
                Phase::Retraction=>return Err("Step retraction ended without whole-capsule separation and unloading".into()),
                Phase::Swing=>{
                    if !self.reference.settled() {return Err("Step reference deadline reached before swing target settled".into());}
                    self.phase=Phase::Landing;self.foot_goal.y-=SOLE_GUARD;
                    self.reference.target=posture(s,self.side,self.com,self.foot_goal,0.)?;
            self.reference=self.reference.retimed()?;
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
        let (controls,candidate,ddq,reactions)=command_for_family(s,&baseline,&nominal,self.side,self.separated,matches!(self.phase,Phase::Retraction|Phase::Swing),&self.last_reactions,self.com,if matches!(self.phase,Phase::Landing|Phase::Complete) {0.} else {SOLE_GUARD})?;
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

/// Timing refusal of an optional target does not invalidate an active path.
/// This only permits trying its next command: full RNEA/contact/motor admission
/// below remains mandatory. Neither path may extend the original pilot clock.
fn admit_retarget(active:&Reference,proposed_duration:Option<f32>,remaining:f32,dt:f32,bounds:&[(f32,f32);21])->Result<bool,String> {
    if proposed_duration.is_some_and(|duration|duration<=remaining) {return Ok(true);}
    let active_remaining=active.duration-active.elapsed;
    if !active.braking && active_remaining>0. && active_remaining<=remaining {
        let next=active.proposal(dt,bounds)?.0;
        let distance=|r:&Reference|->f64 {(0..21).map(|i|(r.target[i] as f64-r.q[i] as f64).powi(2)).sum()};
        if distance(&next)<distance(active) {return Ok(false);}
    }
    Err("Step preparation needs more time than this pilot reserve and no progressing active reference fits; change preparation/placement".into())
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
/// Unilateral lift objectives at CURRENT capsule extrema. Already-clear
/// points do not pull the foot down. Landing instead requests floor approach.
/// These DLS objectives never replace measured separation or Rapier contacts.
fn sole_guard_tasks(points:&[Vector],floor:f32,clearance_goal:f32)->Vec<(Vector,f32)> {
    points.iter().filter_map(|p| {
        let delta=clearance_goal-(p.y-floor);
        if clearance_goal>0. {if delta>0. {Some((*p,delta))} else {None}}
        else {Some((*p,delta))}
    }).collect()
}
/// Whole-body soft posture intention over all27 coordinates. Predicted base
/// motion is never actuated; there is no welded stance-foot elimination.
fn posture(s:&Simulation,side:usize,com_goal:Vector,foot_goal:Vector,clearance_goal:f32)->Result<[f32;21],String> {
    let direction=posture_direction(s,side,com_goal,foot_goal,clearance_goal)?;
    let (q,_)=super::joint_state(s)?;let mut target=q;
    for part in &s.model.segments {if let Some(h)=part.reference.as_ref().and_then(|r|r.hinge.as_ref()) {
        let a=&part.joint_profile.as_ref().unwrap().axes[0];let i=h.qpos_index;
        target[i]=(q[i]+direction[6+i]).clamp(a.min_radians,a.max_radians);
    }}Ok(target)
}
fn posture_direction(s:&Simulation,side:usize,com_goal:Vector,foot_goal:Vector,clearance_goal:f32)->Result<DVector<f32>,String> {
    let mut normal=DMatrix::identity(27,27)*0.001;let mut rhs=DVector::zeros(27);
    let mut add=|a:DMatrix<f32>,b:DVector<f32>,w:f32| {
        normal+=a.transpose()*&a*w;rhs+=a.transpose()*b*w;
    };
    for (n,i) in feet(s)?.into_iter().enumerate() {
        let point=s.world.bodies[s.bodies[i]].translation();let delta=if n==side {foot_goal-point}else{Vector::ZERO};
        let mut target=DVector::zeros(6);for r in 0..3 {target[r]=delta[r];}
        let jac=jacobian(s,i,point)?;
        if n==side {
            // Horizontal retreat/placement remains an origin task. Its stored
            // Y goal is legacy telemetry, not a clearance constraint.
            for row in [0,2] {add(jac.rows(row,1).into_owned(),target.rows(row,1).into_owned(),TRANSLATION_PRIORITY);}
            let sole=geometry(s)?;let floor=s.world.colliders[s.floor].compute_aabb().maxs.y;
            let tasks=sole_guard_tasks(&sole[side].points,floor,clearance_goal);
            let weight=TRANSLATION_PRIORITY/tasks.len().max(1) as f32;
            for (extremum,delta_y) in tasks {
                // Endpoint minus world-Y radius: the vertical Jacobian equals
                // that of the capsule centerline endpoint. Radius does not rotate.
                add(jacobian(s,i,extremum)?.rows(1,1).into_owned(),DVector::from_element(1,delta_y),weight);
            }
        } else {
            // Normal geometry and measured slip are soft tasks at LIVE contact
            // points, not three fixed coordinates or a welded foot orientation.
            for ct in super::standing::contacts_including_unloaded(s)?.iter().filter(|c|c.body==i && c.signed_gap_m<=0.) {
                let point_jac=jacobian(s,i,ct.point)?.rows(0,3).into_owned();
                let n=DVector::from_column_slice(&ct.normal.to_array());
                add((n.transpose()*&point_jac).rows(0,1).into_owned(),DVector::from_element(1,-ct.signed_gap_m),TRANSLATION_PRIORITY);
                let tangent=DMatrix::identity(3,3)-&n*n.transpose();
                add(&tangent*point_jac,-&tangent*DVector::from_column_slice(&ct.velocity.to_array())*0.025,POSTURE_PRIORITY);
            }
        }
        // Foot rotations are free. Only the torso is softly regularized below.
    }
    let root=jacobian(s,0,s.world.bodies[s.bodies[0]].translation())?;
    add(root.rows(3,3).into_owned(),DVector::zeros(3),POSTURE_PRIORITY);
    let mut com=DMatrix::zeros(3,27);
    for (i,h) in s.bodies.iter().enumerate(){let b=&s.world.bodies[*h];
        let p=b.translation()+b.rotation().normalize()*b.local_center_of_mass();
        com+=jacobian(s,i,p)?.rows(0,3)*(b.mass()/s.total_mass());}
    add(com,DVector::from_column_slice(&(com_goal-s.center_of_mass()).to_array()),TRANSLATION_PRIORITY);
    let delta=normal.lu().solve(&rhs).ok_or("Step posture allocation failed (not global infeasibility)")?;
    if delta.iter().any(|v|!v.is_finite()) {return Err("Nonfinite coordinated posture intention".into());}
    // One trust factor for the full coordinated direction; no physical pose
    // is written. Free base coordinates remain available in effort allocation.
    let scale=(0.15/delta.rows(6,21).amax().max(0.15)).min(1.);
    Ok(delta*scale)
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
/// Preferred free-base acceleration and a whitening basis for its soft cost.
/// No contact point or angular acceleration is fixed. Normal gap/velocity and
/// tangential slip use measured geometry; point centripetal terms are retained.
/// This frozen model is deliberately not an exact discrete Rapier constraint.
fn preferred_base(s:&Simulation,dq:&[f32;21],ddq:&[f32;21],side:usize,com_goal:Vector,clearance_goal:f32)
    ->Result<(DVector<f32>,DMatrix<f32>),String> {
    let twists=super::current_twists(s)?;
    let (linear,angular)=accelerations(s,dq,ddq,(Vector::ZERO,Vector::ZERO))?;
    let mut h=DMatrix::identity(6,6)*0.001;let mut rhs=DVector::zeros(6);
    let omega=4./DURATION;
    let mut add=|j:DMatrix<f32>,desired:DVector<f32>,weight:f32| {
        h+=j.transpose()*&j*weight;rhs+=j.transpose()*desired*weight;
    };
    let point_data=|body:usize,point:Vector|->Result<(DMatrix<f32>,Vector,Vector),String> {
        let b=&s.world.bodies[s.bodies[body]];let r=point-b.translation();let w=twists[body].1;
        let com=b.translation()+b.rotation().normalize()*b.local_center_of_mass();
        let v=twists[body].0+w.cross(point-com);
        let bias=linear[body]+angular[body].cross(r)+w.cross(w.cross(r));
        Ok((jacobian(s,body,point)?.view((0,0),(3,6)).into_owned(),v,bias))
    };
    let retained=feet(s)?[1-side];
    let contacts=super::standing::contacts_including_unloaded(s)?;
    let live:Vec<_>=contacts.iter().filter(|c|c.body==retained && c.signed_gap_m<=0.).collect();
    if live.is_empty() {return Err("No actual retained contact for soft base prediction".into());}
    for ct in &live {
        let (j,v,bias)=point_data(retained,ct.point)?;let n=DVector::from_column_slice(&ct.normal.to_array());
        let desired=-omega*omega*ct.signed_gap_m-2.*omega*v.dot(ct.normal);
        add((n.transpose()*&j).rows(0,1).into_owned(),DVector::from_element(1,desired-bias.dot(ct.normal)),TRANSLATION_PRIORITY/live.len() as f32);
        let tangent=DMatrix::identity(3,3)-&n*n.transpose();
        add(&tangent*j,&tangent*DVector::from_column_slice(&(-omega*v-bias).to_array()),POSTURE_PRIORITY/live.len() as f32);
    }
    let free=feet(s)?[side];let sole=geometry(s)?;
    let floor=s.world.colliders[s.floor].compute_aabb().maxs.y;
    let tasks=sole_guard_tasks(&sole[side].points,floor,clearance_goal);
    for (point,gap_error) in &tasks {
        let index=sole[side].points.iter().position(|p|p==point).ok_or("Missing capsule extremum")?;
        let radius=s.model.segments[free].reference.as_ref().ok_or("Missing capsule primitives")?.primitives[index/2].radius;
        // The radius is subtracted along WORLD Y, not attached to the rotating
        // material point. Differentiate the centerline endpoint for Jdot*v.
        let (j,v,bias)=point_data(free,*point+Vector::Y*radius)?;
        add(j.rows(1,1).into_owned(),DVector::from_element(1,omega*omega*gap_error-2.*omega*v.y-bias.y),TRANSLATION_PRIORITY/tasks.len() as f32);
    }
    let mut com_j=DMatrix::zeros(3,6);let mut com_bias=Vector::ZERO;
    for (i,handle) in s.bodies.iter().enumerate() {
        let b=&s.world.bodies[*handle];let point=b.translation()+b.rotation().normalize()*b.local_center_of_mass();
        let (j,_,bias)=point_data(i,point)?;let weight=b.mass()/s.total_mass();com_j+=j*weight;com_bias+=bias*weight;
    }
    let com_desired=(com_goal-s.center_of_mass())*(omega*omega)-measured_com_velocity(s)?*(2.*omega);
    add(com_j,DVector::from_column_slice(&(com_desired-com_bias).to_array()),TRANSLATION_PRIORITY);
    let mut torso=DMatrix::zeros(3,6);for i in 0..3 {torso[(i,3+i)]=1.;}
    add(torso,DVector::from_column_slice(&(-omega*twists[0].1).to_array()),POSTURE_PRIORITY);
    let factor=h.cholesky().ok_or("Soft base cost is not positive definite")?;
    let preferred=factor.solve(&rhs);
    // Coordinates have force-scale magnitude; all six base directions remain
    // available. This metric is a preference, not an extra feasibility bound.
    let basis=factor.l().transpose().try_inverse().ok_or("Soft base metric inverse")?/s.total_mass();
    if preferred.iter().chain(basis.iter()).any(|v|!v.is_finite()) {return Err("Nonfinite soft base prediction".into());}
    Ok((preferred,basis))
}
fn generalized_acceleration(root:&DVector<f32>,joints:&[f32;21])->DVector<f32> {
    let mut result=DVector::zeros(27);result.rows_mut(0,6).copy_from(root);
    result.rows_mut(6,21).copy_from(&DVector::from_column_slice(joints));result
}
#[cfg(test)]
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
#[cfg(test)]
fn rnea(s:&Simulation,q:&[f32;21],dq:&[f32;21],ddq:&[f32;21],support:&[Reaction],retained:usize)
    ->Result<([f32;21],Vector,Vector),String> {
    let (linear,angular)=retained_accelerations(s,dq,ddq,retained)?;
    rnea_accelerations(s,q,dq,ddq,support,&linear,&angular)
}
fn rnea_free(s:&Simulation,q:&[f32;21],dq:&[f32;21],a:&DVector<f32>,support:&[Reaction])
    ->Result<([f32;21],Vector,Vector),String> {
    let ddq=std::array::from_fn(|i|a[6+i]);
    let (linear,angular)=accelerations(s,dq,&ddq,(Vector::new(a[0],a[1],a[2]),Vector::new(a[3],a[4],a[5])))?;
    rnea_accelerations(s,q,dq,&ddq,support,&linear,&angular)
}
fn rnea_accelerations(s:&Simulation,q:&[f32;21],dq:&[f32;21],ddq:&[f32;21],support:&[Reaction],linear:&[Vector],angular:&[Vector])
    ->Result<([f32;21],Vector,Vector),String> {
    let n=s.bodies.len();let twists=super::current_twists(s)?;
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
    let extra=residual.rows(6,residual.len()-6).iter().fold(0f64,|m,x|m.max(x.abs()));
    record["allocatedForcesN"]=serde_json::json!(allocated.as_slice());
    record["residualN_Nm"]=serde_json::json!(residual.as_slice());
    if !allocated.iter().all(|x|x.is_finite()) || !force.is_finite() || !moment.is_finite() || force>0.01 || moment>0.01 || extra>0.0001 {
        return Err(format!("Step contact allocation residual: force={force} N moment={moment} Nm extra={extra}; no global infeasibility claim"));
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
    allocate_motor_wrench_base(a,w0,dw,normals,contact,t0,dt,caps,interval,warm,
        &DMatrix::zeros(a.nrows(),0),&DMatrix::zeros(caps.len(),0),record).map(|(f,a,_)|(f,a))
}
fn allocate_motor_wrench_base(a:&DMatrix<f32>,w0:&DVector<f32>,dw:&DVector<f32>,normals:&[Vector],
    contact:&DMatrix<f32>,t0:&DVector<f32>,dt:&DVector<f32>,caps:&DVector<f32>,
    interval:(f32,f32),warm:Option<&DVector<f32>>,base_w:&DMatrix<f32>,base_t:&DMatrix<f32>,record:&mut serde_json::Value)
    ->Result<(DVector<f32>,f32,DVector<f32>),String> {
    let nf=a.ncols();let ne=a.nrows();let nm=caps.len();let nb=base_w.ncols();let n=nf+1+nb;let scale=400.;
    if base_w.nrows()!=ne || base_t.shape()!=(nm,nb) || base_w.iter().chain(base_t.iter()).any(|x|!x.is_finite())
        || ne<6 || nf!=normals.len()*3 || normals.is_empty() || w0.len()!=ne || dw.len()!=ne
        || contact.shape()!=(nm,nf) || t0.len()!=nm || dt.len()!=nm
        || interval.0<0. || interval.1>1. || interval.0>interval.1
        || normals.iter().any(|n|!n.is_finite() || n.length()<0.5)
        || a.iter().chain(w0.iter()).chain(dw.iter()).chain(contact.iter()).chain(t0.iter()).chain(dt.iter()).chain(caps.iter()).any(|x|!x.is_finite()) {
        return Err("Invalid joint contact/motor allocation inputs".into());
    }
    let ns:Vec<_>=normals.iter().map(|v|Vector3::new(v.x as f64,v.y as f64,v.z as f64).normalize()).collect();
    let mut eq=DMatrix::<f64>::zeros(ne,n);eq.columns_mut(0,nf).copy_from(&a.map(|v|v as f64));
    eq.set_column(nf,&dw.map(|v|-(v as f64)/scale));eq.columns_mut(nf+1,nb).copy_from(&base_w.map(|v|-(v as f64)));let target=w0.map(|v|v as f64);
    let svd=eq.clone().svd(true,true);let cutoff=svd.singular_values.max()*1e-10;
    let inverse=svd.solve(&DMatrix::identity(ne,ne),cutoff).map_err(|e|format!("Joint affine projector: {e}"))?;
    let mut bands=DMatrix::<f64>::zeros(nm,n);bands.columns_mut(0,nf).copy_from(&contact.map(|v|v as f64));
    bands.set_column(nf,&dt.map(|v|v as f64/scale));
    bands.columns_mut(nf+1,nb).copy_from(&base_t.map(|v|v as f64));
    // The alpha=1 least-squares vector is only a seed. Its residual must not
    // reject a family that may be feasible at a smaller progression.
    let seed_svd=a.map(|v|v as f64).svd(true,true);
    let seed_cutoff=(seed_svd.singular_values.max()*1e-10).max(1e-12);
    let initial=seed_svd.solve(&(w0+dw*interval.1).map(|v|v as f64),seed_cutoff)
        .map_err(|e|format!("Joint allocation seed: {e}"))?.map(|v|v as f32);
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
        let beta=x.rows(nf+1,nb).map(|v|v as f32).into_owned();
        let wanted=w0+dw*alpha+base_w*&beta;let residual=a.map(|v|v as f64)*force.map(|v|v as f64)-wanted.map(|v|v as f64);
        let torque=t0+dt*alpha+contact*&force+base_t*&beta;
        #[cfg(test)]
        if matches!(iteration,0|1|2|4|8|16|32|64|128|192|256) {
            if record.get("pureIterationTrace").is_none() {record["pureIterationTrace"]=serde_json::json!([]);}
            let raw_force=x.rows(0,nf).into_owned();
            let raw_residual=a.map(|v|v as f64)*&raw_force-(w0.map(|v|v as f64)+dw.map(|v|v as f64)*(x[nf]/scale)+base_w.map(|v|v as f64)*x.rows(nf+1,nb));
            let raw_cone_deficit=ns.iter().enumerate().map(|(k,no)|{
                let f=Vector3::new(x[3*k],x[3*k+1],x[3*k+2]);let load=f.dot(no);
                ((f-no*load).norm()-load).max(-load)
            }).fold(0f64,f64::max);
            record["pureIterationTrace"].as_array_mut().unwrap().push(serde_json::json!({"iteration":iteration,"alpha":alpha,
                "rawForceResidualN":raw_residual.rows(0,3).norm(),"rawMomentResidualNm":raw_residual.rows(3,3).norm(),
                "rawConeDeficitN":raw_cone_deficit,"normalRepairN":(force.map(|v|v as f64)-raw_force).norm(),
                "finalForceResidualN":residual.rows(0,3).norm(),"finalMomentResidualNm":residual.rows(3,3).norm(),
                "finalTorqueExcessNm":torque.iter().zip(caps.iter()).map(|(v,c)|(v.abs()-c).max(0.)).fold(0f32,f32::max)}));
        }
        record["coneIterations"]=serde_json::json!(iteration);record["alpha"]=serde_json::json!(alpha);
        record["baseCorrectionCoordinates"]=serde_json::json!(beta.as_slice());
        record["allocatedForcesN"]=serde_json::json!(force.as_slice());record["desiredWrenchN_Nm"]=serde_json::json!(wanted.as_slice());
        record["residualN_Nm"]=serde_json::json!(residual.as_slice());record["allocatedMotorTorqueNm"]=serde_json::json!(torque.as_slice());
        if alpha>=interval.0 && alpha<=interval.1 && force.iter().all(|v|v.is_finite())
            && residual.rows(0,3).norm()<0.0001 && residual.rows(3,3).norm()<0.0001
            && residual.rows(6,ne-6).iter().all(|v|v.abs()<0.0001)
            && (0..normals.len()).all(|k|force_in_cone(Vector::new(force[3*k],force[3*k+1],force[3*k+2]),normals[k]))
            && torque.iter().zip(caps.iter()).all(|(t,c)|t.is_finite() && t.abs()<=*c) {return Ok((force,alpha,beta));}
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
fn blend_reference(baseline:&(Reference,[f32;21]),nominal:&(Reference,[f32;21]),alpha:f32,_dt:f32)->Result<(Reference,[f32;21]),String> {
    if alpha==1. {return Ok(nominal.clone());}
    if alpha==0. {return Ok(baseline.clone());}
    let mut r=nominal.0.clone();let a=std::array::from_fn(|i|baseline.1[i]+alpha*(nominal.1[i]-baseline.1[i]));
    for i in 0..21 {r.q[i]=baseline.0.q[i]+alpha*(nominal.0.q[i]-baseline.0.q[i]);r.dq[i]=baseline.0.dq[i]+alpha*(nominal.0.dq[i]-baseline.0.dq[i]);}
    // Limited progress is not a new target or a new planning horizon.
    // The accepted acceleration seeds the next flow, not an implicit zero.
    r.acceleration=std::array::from_fn(|i|baseline.0.acceleration[i]+alpha*(nominal.0.acceleration[i]-baseline.0.acceleration[i]));
    Ok((r,a))
}

/// Set a preferred load split for propulsion then capture/withdrawal. Contact cones/equalities/motor bands can move away from it.
/// Inputs are previous intentions only, never measured impulses.
fn prefer_unloading(seed:&mut DVector<f32>,points:&[(usize,Vector)],free:usize,total:Vector,free_share:f32)->Result<(),String> {
    let retained=points.iter().filter(|(body,_)|*body!=free).count();
    if retained==0 {return Err("No retained points for load preference".into());}
    // Hysteresis does not reserve support for an absent geometric group.
    let free_share=if points.iter().any(|(body,_)|*body==free) {free_share} else {0.};
    for is_free in [true,false] {
        let count=points.iter().filter(|(body,_)|(*body==free)==is_free).count();
        if count==0 {continue;}
        let desired=total*if is_free {free_share}else{1.-free_share};
        let sum=points.iter().enumerate().filter(|(_, (body,_))|(*body==free)==is_free)
            .map(|(k,_)|Vector::new(seed[3*k],seed[3*k+1],seed[3*k+2])).sum::<Vector>();
        let shift=(desired-sum)/count as f32;
        for (k,(body,_)) in points.iter().enumerate() {if (*body==free)==is_free {for j in 0..3 {seed[3*k+j]+=shift[j];}}}
    }
    Ok(())
}
/// Pendulum COP heuristic only: accelerate toward the retained support first,
/// brake the measured capture velocity as it approaches, then unload the free
/// foot. Neither this preference nor the rigid-base model proves feasibility.
fn preferred_free_share(com:Vector,velocity:Vector,free:Vector,retained:Vector)->f32 {
    let height=(com.y-retained.y).max(0.01);let omega=(9.81/height).sqrt();
    let acceleration=horizontal(retained-com)*(omega*omega)-horizontal(velocity)*(2.*omega);
    let cop=horizontal(com)-acceleration*(height/9.81);
    let span=horizontal(retained-free);
    if span.length_squared()<1e-8 {return 0.5;}
    (1.-(cop-horizontal(free)).dot(span)/span.length_squared()).clamp(0.,1.)
}
type CommandProposal=([f32;21],Reference,[f32;21],Vec<Reaction>);
fn command_for_family(s:&Simulation,baseline:&(Reference,[f32;21]),nominal:&(Reference,[f32;21]),side:usize,exclusive:bool,unload:bool,warm:&[Reaction],com_goal:Vector,clearance_goal:f32)->Result<CommandProposal,String> {
    let mut record=serde_json::json!({"native":s.substeps_completed,"nominalReference":nominal.0,
        "baselineReference":baseline.0,"side":side,"exclusive":exclusive,"units":"world N / Nm about torso origin"});
    let result=command_for_family_recorded(s,baseline,nominal,side,exclusive,unload,warm,com_goal,clearance_goal,&mut record);
    record["error"]=serde_json::json!(result.as_ref().err());
    record["controls"]=serde_json::json!(result.as_ref().ok().map(|x|x.0));
    #[cfg(feature="numeric-integration-budget")]
    crate::integration_budget::record_step_command(record);
    result
}
/// Rapier generic multibody biased normal projection, expressed at a frozen
/// measured configuration. This is a local prediction, not a replacement for
/// Rapier's four substeps and final unbiased stabilization pass.
#[cfg(test)]
fn contact_bias(gap:f32,h:f32,erp_inv_dt:f32,max_correction:f32)->f32 {
    gap.max(0.)/h+(erp_inv_dt*gap).clamp(-max_correction,0.)
}
fn command_for_family_recorded(s:&Simulation,baseline:&(Reference,[f32;21]),nominal:&(Reference,[f32;21]),side:usize,exclusive:bool,unload:bool,warm:&[Reaction],com_goal:Vector,clearance_goal:f32,record:&mut serde_json::Value)->Result<CommandProposal,String> {
    let (q,dq)=super::joint_state(s)?;let geometry=geometry(s)?;
    let raw=|r:&(Reference,[f32;21])|std::array::from_fn(|i|r.1[i]+16.*(r.0.q[i]-q[i])+8.*(r.0.dq[i]-dq[i]));
    let (a0,da,alpha_min)=acceleration_region(&raw(baseline),&raw(nominal));
    let desired_acceleration=std::array::from_fn(|i|a0[i]+da[i]);
    record["q"]=serde_json::json!(q);record["dq"]=serde_json::json!(dq);record["geometry"]=serde_json::json!(geometry);
    let ddq=&desired_acceleration;
    let (base0,base_basis)=preferred_base(s,&dq,&a0,side,com_goal,clearance_goal)?;
    let (base1,_)=preferred_base(s,&dq,ddq,side,com_goal,clearance_goal)?;
    let base_delta=&base1-&base0;
    let g0=generalized_acceleration(&base0,&a0);let g1=generalized_acceleration(&base1,ddq);
    let (torque0,force0,moment0)=rnea_free(s,&q,&dq,&g0,&[])?;
    let (torque1,force,moment)=rnea_free(s,&q,&dq,&g1,&[])?;
    let mut points=vec![];let mut candidates=vec![];let mut normals=vec![];
    let observed_contacts=super::standing::contacts_including_unloaded(s)?;
    if s.world.colliders[s.floor].friction()!=1. || geometry.iter().any(|g|s.world.colliders[s.colliders[g.body]].friction()!=1.) {
        return Err("Step cone allocator requires unchanged mu=1 foot/floor materials".into());
    }
    // Live solver points, including zero-load contacts. Neither a desired
    // upward acceleration nor a hysteresis flag invents/removes real contact.
    for ct in &observed_contacts {
        if exclusive && ct.body==feet(s)?[side] {continue;}
        let used=contact_can_react(ct.signed_gap_m,0.)?;
        candidates.push(serde_json::json!({"body":ct.body,"point":ct.point.to_array(),"gap":ct.signed_gap_m,"used":used}));
        if used {points.push((ct.body,ct.point));normals.push(ct.normal);}
    }
    record["predictionScope"]=serde_json::json!("free-base acceleration family with soft contact tasks; neither exact contact feasibility nor a full-native Rapier certificate");
    record["biasedContactFixedPointIsAdmissionGate"]=serde_json::json!(false);
    record["candidates"]=serde_json::json!(candidates);
    if points.is_empty(){return Err("Step has no geometric support candidates".into());}
    let root=s.world.bodies[s.bodies[0]].translation();let mut a=DMatrix::zeros(6,3*points.len());
    for (k,(_,point)) in points.iter().enumerate(){for j in 0..3 {
        let e=[Vector::X,Vector::Y,Vector::Z][j];let m=(*point-root).cross(e);a[(j,3*k+j)]=1.;
        for r in 0..3 {a[(r+3,3*k+j)]=m[r];}
    }}
    let vector6=|f:Vector,m:Vector|DVector::from_column_slice(&[f.x,f.y,f.z,m.x,m.y,m.z]);
    let w0=vector6(force0,moment0);let dw=vector6(force,moment)-&w0;
    let t0=DVector::from_column_slice(&torque0);let dt=DVector::from_column_slice(&torque1)-&t0;
    let mut physical_w=DMatrix::zeros(6,6);let mut physical_t=DMatrix::zeros(21,6);
    for j in 0..6 {
        let mut unit=g0.clone();unit[j]+=1.;let (t,f,m)=rnea_free(s,&q,&dq,&unit,&[])?;
        physical_w.set_column(j,&(vector6(f,m)-&w0));
        physical_t.set_column(j,&(DVector::from_column_slice(&t)-&t0));
    }
    let base_w=&physical_w*&base_basis;let base_t=&physical_t*&base_basis;
    record["basePreferredAcceleration"]=serde_json::json!(base0.as_slice());
    record["basePreferredDelta"]=serde_json::json!(base_delta.as_slice());
    record["baseBasisColumnMajor"]=serde_json::json!(base_basis.as_slice());
    record["wrenchBaseColumnMajor"]=serde_json::json!(base_w.as_slice());
    record["motorBaseColumnMajor"]=serde_json::json!(base_t.as_slice());
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
    if unload {
        let target_force=Vector::new(w0[0]+dw[0],w0[1]+dw[1],w0[2]+dw[2]);
        let feet_geometry=&geometry;
        let share=if exclusive {0.} else {preferred_free_share(s.center_of_mass(),measured_com_velocity(s)?,
            support_center(&feet_geometry[side]),support_center(&feet_geometry[1-side]))};
        prefer_unloading(&mut seed,&points,feet(s)?[side],target_force,share)?;
        record["preferredFreeLoadFraction"]=serde_json::json!(share);
        matched=points.len();
    }
    record["loadPreference"]=serde_json::json!({"captureDirectedPreference":unload,"hardZeroReactionConstraint":false,"monotonicLoadRequired":false});
    record["rootOrigin"]=serde_json::json!(root.to_array());
    let (allocated,alpha,beta)=allocate_motor_wrench_base(&a,&w0,&dw,&normals,&contact,&t0,&dt,&caps,(alpha_min,1.),
        if matched>0 {Some(&seed)} else {None},&base_w,&base_t,record)?;
    let base_acceleration=&base0+&base_delta*alpha+&base_basis*&beta;
    record["baseAcceleration"]=serde_json::json!(base_acceleration.as_slice());
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
    let (bias,_,_)=rnea_free(s,&q,&dq,&DVector::zeros(27),&support)?;
    let full=generalized_acceleration(&base_acceleration,ddq);
    let (dynamic,residual_f,residual_m)=rnea_free(s,&q,&dq,&full,&support)?;
    record["generalizedAcceleration"]=serde_json::json!(full.as_slice());
    let (lin,ang)=accelerations(s,&dq,ddq,(Vector::new(full[0],full[1],full[2]),Vector::new(full[3],full[4],full[5])))?;
    let twists=super::current_twists(s)?;let free=feet(s)?[side];
    record["predictedSoleVerticalAccelerations"]=serde_json::json!(geometry[side].points.iter().enumerate().map(|(index,p)| {
        let radius=s.model.segments[free].reference.as_ref().unwrap().primitives[index/2].radius;
        let r=*p+Vector::Y*radius-s.world.bodies[s.bodies[free]].translation();
        (lin[free]+ang[free].cross(r)+twists[free].1.cross(twists[free].1.cross(r))).y
    }).collect::<Vec<_>>());
    record["rneaResidualForceN"]=serde_json::json!(residual_f.to_array());
    record["rneaResidualMomentNm"]=serde_json::json!(residual_m.to_array());
    record["biasTorqueNm"]=serde_json::json!(bias);record["dynamicTorqueNm"]=serde_json::json!(dynamic);
    if residual_f.length()>0.01 || residual_m.length()>0.01 {return Err("Step predicted unactuated-root residual (approximate model)".into());}
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
pub(crate) fn rolling_posture_diagnostic(s:&Simulation)->Result<serde_json::Value,String> {
    let value=super::locomotion::observation(s)?;
    let state:State=serde_json::from_value(value["ownership"]["step"].clone()).map_err(|e|e.to_string())?;
    let geometry=geometry(s)?;let side=state.side;let free=feet(s)?[side];
    let d=posture_direction(s,side,state.com,state.foot_goal,SOLE_GUARD)?;
    let lowest=geometry[side].points.iter().min_by(|a,b|a.y.total_cmp(&b.y)).ok_or("No extrema")?;
    let lift=(jacobian(s,free,*lowest)?.row(1)*&d)[0];
    let retained=feet(s)?[1-side];let retained_displacement=jacobian(s,retained,s.world.bodies[s.bodies[retained]].translation())?*&d;
    Ok(serde_json::json!({"limitingPoint":lowest.to_array(),"predictedLimitingLiftM":lift,
        "predictedRetainedTwistDisplacement":retained_displacement.as_slice(),"jointMaxRad":d.rows(6,21).amax(),"baseDirection":d.rows(0,6).as_slice()}))
}

/// Diagnostic only: no allocation, reference update, admission or integration.
/// Compare a frozen-state inverse-dynamics approximation to an observed full
/// native transition. End-of-tick differences are NOT biased-pass samples.
#[cfg(test)]
pub(crate) fn identify_transition(before:&Simulation,after:&Simulation,controls:[f32;21])->Result<serde_json::Value,String> {
    let dt=before.profile.dt_s;let h=solver_h(before);
    let (q0,v0)=super::joint_state(before)?;let (q1,v1)=super::joint_state(after)?;
    let root_velocity=|s:&Simulation|->Result<DVector<f32>,String> {
        let id=s.world.multibody_joints.rigid_body_link(s.bodies[0]).ok_or("Missing root")?;
        let mb=s.world.multibody_joints.get_multibody(id.multibody).ok_or("Missing articulation")?;
        Ok(mb.joint_velocity(mb.link(id.id).ok_or("Missing root link")?).into_owned())
    };
    let rv0=root_velocity(before)?;let rv1=root_velocity(after)?;
    let mut acceleration=DVector::zeros(27);for i in 0..6 {acceleration[i]=(rv1[i]-rv0[i])/dt;}
    for i in 0..21 {acceleration[6+i]=(v1[i]-v0[i])/dt;}
    let c0=super::standing::contacts_including_unloaded(before)?;let c1=super::standing::contacts_including_unloaded(after)?;
    let twists1=super::current_twists(after)?;let mut support=vec![];let mut laws=vec![];
    let params=&before.world.integration_parameters;let soft=&params.static_contact_softness;
    let kappa=1./soft.cfm_factor(h)-1.;
    let root=before.world.multibody_joints.rigid_body_link(before.bodies[0]).ok_or("Missing root")?;
    let mb=before.world.multibody_joints.get_multibody(root.multibody).ok_or("Missing articulation")?;
    if mb.ndofs()!=27 || mb.links().any(|l|l.joint.kinematic) {return Err("Unexpected mass-coordinate reduction".into());}
    for old in &c0 {
        let b0=&before.world.bodies[before.bodies[old.body]];let b1=&after.world.bodies[after.bodies[old.body]];
        let local=b0.rotation().inverse()*(old.point-b0.translation());let material_point=b1.translation()+b1.rotation()*local;
        let com=b1.translation()+b1.rotation().normalize()*b1.local_center_of_mass();
        let vn0=old.velocity.dot(old.normal);let vn1=(twists1[old.body].0+twists1[old.body].1.cross(material_point-com)).dot(old.normal);
        let matching=c1.iter().filter(|c|c.body==old.body).min_by(|a,b|(a.point-material_point).length_squared().total_cmp(&(b.point-material_point).length_squared()));
        let matched=matching.filter(|c|(c.point-material_point).length()<0.001);
        let link=before.world.multibody_joints.rigid_body_link(before.bodies[old.body]).ok_or("Missing contact link")?;
        let jac=mb.body_jacobian(link.id);let torque=old.solver_lever_from_com.cross(old.normal);
        let row=DVector::from_fn(jac.ncols(),|col,_|(0..3).map(|axis|old.normal[axis]*jac[(axis,col)]+torque[axis]*jac[(3+axis,col)]).sum());
        let inv=mb.inv_augmented_mass().solve(&row).ok_or("Mass solve failed")?;let gamma=kappa*row.dot(&inv);
        let bias=contact_bias(old.signed_gap_m,h,soft.erp_inv_dt(h),params.max_corrective_velocity());
        let mean_force=matched.map(|c|c.impulse/dt);
        laws.push(serde_json::json!({"body":before.model.segments[old.body].id,"pointBefore":old.point.to_array(),"materialPointAfter":material_point.to_array(),
            "gapBeforeM":old.signed_gap_m,"gapAfterM":matched.map(|c|c.signed_gap_m),"vnBeforeMps":vn0,"vnAfterMps":vn1,"meanNormalAcceleration":(vn1-vn0)/dt,
            "matchingDistanceM":matched.map(|c|(c.point-material_point).length()),"normalImpulseNativeNs":matched.map(|c|c.impulse),"tangentImpulseNativeNs":matched.map(|c|c.tangent_impulse_world.to_array()),
            "gammaInverseKg":gamma,"biasedRhsMps":bias,"biasedFrozenModelResidualMps":mean_force.map(|f|vn0+h*(vn1-vn0)/dt+bias+h*gamma*f),
            "scope":"native mean acceleration/force substituted into biased substep law; not a measured biased-pass residual"}));
    }
    for ct in &c1 {support.push((ct.body,ct.point,(ct.normal*ct.impulse+ct.tangent_impulse_world)/dt));}
    let (required,rf,rm)=rnea_free(before,&q0,&v0,&acceleration,&support)?;
    let mut applied=[0.;21];let mut limits=vec![];
    for part in &before.model.segments {if let Some(h)=part.reference.as_ref().and_then(|r|r.hinge.as_ref()) {
        applied[h.actuator_index]=h.gear*controls[h.actuator_index];let axis=&part.joint_profile.as_ref().unwrap().axes[0];
        limits.push(serde_json::json!({"joint":h.name,"qpos":h.qpos_index,"q":q1[h.qpos_index],"min":axis.min_radians,"max":axis.max_radians,
            "meanAcceleration":acceleration[6+h.qpos_index],"design2Exceeded":acceleration[6+h.qpos_index].abs()>2.,"torqueNm":applied[h.actuator_index],"capNm":axis.max_motor_torque_nm.min(h.gear.abs())}));
    }}
    Ok(serde_json::json!({"nativeFrom":before.substeps_completed,"nativeTo":after.substeps_completed,"dt":dt,"solverH":h,
        "qBefore":q0,"qAfter":q1,"dqBefore":v0,"dqAfter":v1,"generalizedMeanAcceleration":acceleration.as_slice(),"controls":controls,"appliedJointTorqueNm":applied,
        "comBefore":before.center_of_mass().to_array(),"comAfter":after.center_of_mass().to_array(),"rootPositionBefore":before.world.bodies[before.bodies[0]].translation().to_array(),
        "rootPositionAfter":after.world.bodies[after.bodies[0]].translation().to_array(),"feetBefore":geometry(before)?,"feetAfter":geometry(after)?,"contacts":laws,"limits":limits,
        "frozenRneaRequiredTorqueNm":required,"frozenRneaTorqueResidualNm":std::array::from_fn::<_,21,_>(|i|required[i]-applied[i]),"frozenRneaRootForceResidualN":rf.to_array(),"frozenRneaRootMomentResidualNm":rm.to_array(),
        "limitations":"No internal pre/post-stabilization samples. Frozen pre-state RNEA uses full-native mean acceleration and contact force; discretization/contact-position error is not actuator failure. No withdrawal objective imposed."}))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    #[ignore="reads exact refusal6153; diagnostic witness only, no product gate or integration"]
    fn native_step_pure_refusal_6153_wrench_separator() {
        let data:serde_json::Value=serde_json::from_slice(&std::fs::read(std::env::var("LH_STEP_TRIAL").unwrap()).unwrap()).unwrap();
        let d=&data["result"]["commands"]["firstRefused"];assert_eq!(d["native"],6153);
        let root=Vector::from_array(serde_json::from_value(d["rootOrigin"].clone()).unwrap());
        let points:Vec<Vector>=d["candidates"].as_array().unwrap().iter().filter(|c|c["used"]==true)
            .map(|c|Vector::from_array(serde_json::from_value(c["point"].clone()).unwrap())).collect();
        assert_eq!(points.len(),4);
        let normals:Vec<[f32;3]>=serde_json::from_value(d["normals"].clone()).unwrap();
        // Fixed diagnostic witness derived from the recorded wrench. Force
        // coefficients have units m; moment coefficients are dimensionless.
        // Increasing the vertical coefficient by 1e-6 gives a strict reserve.
        let y=DVector::from_column_slice(&[0.3758930357959235,0.005506428698872378,1.,
            0.8052047337324387,-0.0007026634552326635,-0.30266698333638253]);
        let mut minimum_dual_margin=f64::INFINITY;
        for (point,normal) in points.iter().zip(normals) {
            let mut a=DMatrix::<f64>::zeros(6,3);
            for (j,e) in [Vector::X,Vector::Y,Vector::Z].into_iter().enumerate() {
                let m=(*point-root).cross(e);a[(j,j)]=1.;for k in 0..3 {a[(3+k,j)]=m[k] as f64;}
            }
            let dual=a.transpose()*&y;let n=Vector3::new(normal[0] as f64,normal[1] as f64,normal[2] as f64).normalize();
            let normal=dual.dot(&n);let tangent=(&dual-n*normal).norm();
            minimum_dual_margin=minimum_dual_margin.min(normal-tangent);
            assert!(normal-tangent>0.0000009);
        }
        let w0:Vec<f64>=serde_json::from_value(d["wrenchBaseline"].clone()).unwrap();
        let dw:Vec<f64>=serde_json::from_value(d["wrenchDelta"].clone()).unwrap();
        let left=y.dot(&DVector::from_vec(w0));let slope=y.dot(&DVector::from_vec(dw));
        assert!(left< -8.8 && left+slope< -7.7);
        // Every allowed reaction has y·A f >=0, but both affine endpoints
        // are strictly negative. Hence no alpha in[0,1] satisfies this frozen
        // wrench family. This says nothing about another acceleration/strategy.
        println!("PURE_SEPARATOR {}",serde_json::json!({"dualConeMarginM":minimum_dual_margin,
            "endpointWrenchPairingNm":[left,left+slope],"physicsIntegrations":0}));
    }
    #[test] fn native_step_pure_capsule_guard_uses_current_extrema_not_origin() {
        // Actual native6153 extrema. Origin has risen18.8mm but one end stays
        // below the floor. Reaching the old origin target would still not clear it.
        let points=[Vector::new(-0.66542566,0.004832344,0.3556114),Vector::new(-0.49647248,0.06636602,0.24530092),
            Vector::new(-0.6552332,-0.00009908527,0.3720976),Vector::new(-0.46589506,0.05157175,0.2947595)];
        let tasks=sole_guard_tasks(&points,0.,SOLE_GUARD);assert_eq!(tasks.len(),2);
        assert!((tasks[1].1-0.020099085).abs()<1e-8);
        let old_origin_remaining=0.047581717-0.045768335;
        assert!(points[2].y+old_origin_remaining<SOLE_GUARD);
        for (point,dy) in &tasks {assert!((point.y+dy-SOLE_GUARD).abs()<1e-7);}
        let raised:Vec<_>=points.iter().map(|p|*p+Vector::Y*0.03).collect();
        assert!(sole_guard_tasks(&raised,0.,SOLE_GUARD).is_empty());
        let shifted:Vec<_>=points.iter().map(|p|*p+Vector::Y).collect();
        let translated=sole_guard_tasks(&shifted,1.,SOLE_GUARD);
        assert_eq!(translated.len(),tasks.len());
        for (a,b) in tasks.iter().zip(translated) {assert!((a.1-b.1).abs()<1e-7);}
        let landing=sole_guard_tasks(&raised,0.,0.);assert_eq!(landing.len(),4);
        assert!(landing.iter().all(|(_,dy)|*dy<0.));
    }
    #[test] fn native_step_pure_optional_retarget_preserves_active_deadline() {
        let active=Reference::new([0.;21],[0.01;21],[0.02;21]);let before=serde_json::to_value(&active).unwrap();
        let mut proposed=active.clone();proposed.duration=2.;proposed.target=[0.2;21];
        assert!(!admit_retarget(&active,Some(proposed.duration),1.,0.00125,&[(-1.,1.);21]).unwrap());
        assert!(!admit_retarget(&active,None,1.,0.00125,&[(-1.,1.);21]).unwrap());
        assert_eq!(serde_json::to_value(&active).unwrap(),before);
        assert!(admit_retarget(&active,Some(proposed.duration),0.1,0.00125,&[(-1.,1.);21]).is_err());
        let mut stopped=active.clone();stopped.target=stopped.q;stopped.dq=[0.;21];
        assert!(admit_retarget(&stopped,Some(proposed.duration),1.,0.00125,&[(-1.,1.);21]).is_err());
        let mut expired=active;expired.elapsed=expired.duration;
        assert!(admit_retarget(&expired,Some(proposed.duration),1.,0.00125,&[(-1.,1.);21]).is_err());
    }
    #[test] fn native_step_pure_propulsion_precedes_unloading_from_measured_capture() {
        let free=Vector::new(-0.1,0.,0.);let support=Vector::new(0.1,0.,0.);
        let com=Vector::new(0.,0.8,0.);
        assert!(preferred_free_share(com,Vector::ZERO,free,support)>0.9);
        // Approaching the support while moving toward it calls for braking:
        // the preferred COP moves toward support, permitting free-foot unloading.
        assert!(preferred_free_share(Vector::new(0.09,0.8,0.),Vector::new(0.1,0.,0.),free,support)<0.1);
        // Same position, opposite velocity: feedback requests propulsion again.
        assert!(preferred_free_share(com,Vector::new(-0.1,0.,0.),free,support)>0.9);
    }
    #[test]
    #[ignore="reads exact recorded observation; reference updates only, no Rapier integration"]
    fn native_step_pure_recorded_5499_target_replacement_and_partial_progress() {
        let data:serde_json::Value=serde_json::from_slice(&std::fs::read(std::env::var("LH_STEP_TRIAL").unwrap()).unwrap()).unwrap();
        let state=&data["result"]["finalObservation"]["ownership"]["step"];
        let mut r:Reference=serde_json::from_value(state["reference"].clone()).unwrap();
        r.acceleration=serde_json::from_value(state["reference_acceleration"].clone()).unwrap();
        let initial=r.clone();let replanned=r.retimed().unwrap();
        assert_eq!(replanned.q,r.q);assert_eq!(replanned.dq,r.dq);assert_eq!(replanned.acceleration,r.acceleration);
        // Recorded left knee: the new goal extends, inherited velocity flexes.
        assert!(r.target[12]>r.q[12] && r.dq[12]<0.);
        let model=lh_model::Model::canonical().unwrap();let mut bounds=[(0.,0.);21];
        for p in &model.segments {if let Some(h)=p.reference.as_ref().and_then(|r|r.hinge.as_ref()) {
            let a=&p.joint_profile.as_ref().unwrap().axes[0];bounds[h.qpos_index]=(a.min_radians,a.max_radians);
        }}
        let mut first_positive=None;
        for update in 1..=320 {
            let nominal=r.proposal(0.00125,&bounds).unwrap();let baseline=r.braking_proposal(0.00125,&bounds).unwrap();
            let (next,a)=blend_reference(&baseline,&nominal,0.96,0.00125).unwrap();
            assert_eq!(next.acceleration[12],baseline.0.acceleration[12]+0.96*(nominal.0.acceleration[12]-baseline.0.acceleration[12]));assert!(next.elapsed>=r.elapsed);
            assert!((next.dq[12]-r.dq[12]).abs()<=2.*0.00125+1e-6);
            // Hold measured q/dq fixed to isolate reference tracking, not simulate motion.
            let tracking=a[12]+16.*(next.q[12]-(-0.384538621))+8.*(next.dq[12]-(-0.20407295));
            if tracking>0. && next.dq[12]>0. && first_positive.is_none() {first_positive=Some(update);}
            r=next;
            if update%40==0 {
                let before=r.clone();r=r.retimed().unwrap();
                assert_eq!(r.q,before.q);assert_eq!(r.dq,before.dq);assert_eq!(r.acceleration,before.acceleration);
            }
        }
        assert!(first_positive.is_some());assert!(r.q[12]>initial.q[12]+0.005);
        assert!(r.dq[12]>0.);assert_eq!(r.target,initial.target);
        println!("REFERENCE_ONLY native5499 direction recovered at update {:?}; knee reference delta {}; physics integrations 0",first_positive,r.q[12]-initial.q[12]);
        let first=&data["result"]["commands"]["firstAdmitted"];
        let mut r:Reference=serde_json::from_value(first["proposedReference"].clone()).unwrap();
        r.acceleration=serde_json::from_value(first["referenceAcceleration"].clone()).unwrap();
        let q=first["q"][12].as_f64().unwrap() as f32;let v=first["dq"][12].as_f64().unwrap() as f32;
        let effort=|r:&Reference,a:f32|a+16.*(r.q[12]-q)+8.*(r.dq[12]-v);
        let effort_before=effort(&r,r.acceleration[12]);assert!(effort_before<0.);
        for _ in 0..320 {
            let nominal=r.proposal(0.00125,&bounds).unwrap();let baseline=r.braking_proposal(0.00125,&bounds).unwrap();
            r=blend_reference(&baseline,&nominal,0.96,0.00125).unwrap().0;
        }
        assert!(effort(&r,r.acceleration[12])>0. && r.dq[12]>0.);
        println!("REFERENCE_ONLY native5339 tracking acceleration {} -> {}; reference speed {}; physics integrations 0",effort_before,effort(&r,r.acceleration[12]),r.dq[12]);
    }
    #[test] fn native_step_pure_geometry_and_velocity_drive_transfer_and_sole_guard() {
        let com=Vector::new(0.,0.8,0.);let support=Vector::new(0.2,0.,0.1);
        let still=transfer_goal(com,Vector::ZERO,support);
        assert_eq!(still,Vector::new(0.2,0.8,0.1));assert!((still-com).length()>0.1);
        let moving=transfer_goal(com,Vector::new(0.1,0.,0.),support);assert!(moving.x<still.x && moving.x>com.x);
        let foot=Vector::new(-0.2,0.03,0.);let gap=-0.0004;let goal=guarded_foot_goal(foot,gap,Vector::Z);
        assert!((gap+goal.y-foot.y-SOLE_GUARD).abs()<1e-7);assert_eq!(goal.z,-SOLE_GUARD);
        assert!(TRANSLATION_PRIORITY>POSTURE_PRIORITY);
    }
    #[test] fn native_step_pure_absent_free_contact_keeps_full_retained_preference() {
        let points=vec![(10,Vector::ZERO),(10,Vector::X),(10,Vector::Z)];
        let mut seed=DVector::from_column_slice(&[19.46621,370.20123,-64.63462,1.631024,15.114378,7.529111,3.862818,19.109674,13.427917]);
        let total=Vector::new(31.925089,403.81964,-40.066803);
        prefer_unloading(&mut seed,&points,16,total,0.15284866).unwrap();
        let sum=(0..3).map(|k|Vector::new(seed[3*k],seed[3*k+1],seed[3*k+2])).sum::<Vector>();
        assert!((sum-total).length()<0.0001);
        assert!((0..3).all(|k|seed[3*k+1]>0.));
    }
    #[test] fn native_step_pure_unloading_is_a_preference_not_contact_removal() {
        let points=vec![(0,Vector::ZERO),(1,Vector::X),(1,Vector::Z)];
        let mut seed=DVector::from_vec(vec![0.,200.,0., 0.,100.,0., 0.,100.,0.]);
        prefer_unloading(&mut seed,&points,0,Vector::new(0.,400.,0.),0.).unwrap();
        assert_eq!(seed[1],0.);assert_eq!(seed[4]+seed[7],400.);assert_eq!(points.len(),3);
        assert!(contact_can_react(-0.0004,0.1).unwrap());assert!(!contact_can_react(0.001,0.).unwrap());
        // The preferred free load is not an equality: retained force cap100
        // requires the allocator to admit a positive free load again.
        let mut a=DMatrix::zeros(6,6);a[(1,1)]=1.;a[(1,4)]=1.;
        let mut w=DVector::zeros(6);w[1]=200.;let mut motor=DMatrix::zeros(1,6);motor[(0,4)]=1.;
        let preferred=DVector::from_vec(vec![0.,0.,0.,0.,200.,0.]);
        let (f,_)=allocate_motor_wrench(&a,&w,&DVector::zeros(6),&[Vector::Y,Vector::Y],&motor,&DVector::zeros(1),&DVector::zeros(1),
            &DVector::from_vec(vec![100.]),(1.,1.),Some(&preferred),&mut serde_json::json!({})).unwrap();
        assert!(f[1]>=99.99);assert!(f[4]<=100.);
    }
    #[test] fn native_step_pure_infeasible_nominal_seed_can_reduce_progression() {
        let mut a=DMatrix::zeros(7,3);a[(1,1)]=1.;
        let mut w0=DVector::zeros(7);w0[1]=100.;w0[6]=-0.5;
        let mut dw=DVector::zeros(7);dw[6]=1.;
        let (f,alpha)=allocate_motor_wrench(&a,&w0,&dw,&[Vector::Y],&DMatrix::zeros(0,3),
            &DVector::zeros(0),&DVector::zeros(0),&DVector::zeros(0),(0.,1.),None,&mut serde_json::json!({})).unwrap();
        assert!((alpha-0.5).abs()<1e-5);assert!((f[1]-100.).abs()<1e-4);
    }
    #[test] fn native_step_pure_joint_allocation_preserves_dependent_contact_compatibility() {
        let mut a=DMatrix::zeros(7,6);a[(1,1)]=1.;a[(1,4)]=1.;a[(6,1)]=1.;a[(6,4)]=-1.;
        let mut desired=DVector::zeros(7);desired[1]=100.;desired[6]=20.;
        let (f,_)=allocate_motor_wrench(&a,&desired,&DVector::zeros(7),&[Vector::Y,Vector::Y],&DMatrix::zeros(0,6),
            &DVector::zeros(0),&DVector::zeros(0),&DVector::zeros(0),(1.,1.),None,&mut serde_json::json!({})).unwrap();
        assert!((f[1]-60.).abs()<1e-4);assert!((f[4]-40.).abs()<1e-4);
    }
    #[test] fn native_step_pure_retime_preserves_q_v_without_deadline_compression() {
        let mut r=Reference::new([0.;21],[-0.065298;21],[-0.01960036;21]);r.duration=0.10125059;r.elapsed=0.00125;
        let next=r.retimed().unwrap();assert_eq!(next.q,r.q);assert_eq!(next.dq,r.dq);assert!(next.duration>0.4);
        let count=(next.duration/0.00125).ceil() as usize;let mut p=next;for _ in 0..count {p=p.proposal(0.00125,&[(-1.,1.);21]).unwrap().0;}
        assert!(p.settled());
    }
    #[test] fn native_step_pure_discrete_contact_bias_has_no_slop_deadzone() {
        let h=0.0003125;let erp=18.739173;
        assert_eq!(contact_bias(0.,h,erp,3.),0.);
        assert!((contact_bias(-0.00001,h,erp,3.)+0.00018739173).abs()<1e-9);
        assert_eq!(contact_bias(0.001,h,erp,3.),0.001/h);
        let v=-0.01;let b=contact_bias(-0.00001,h,erp,3.);let gamma=0.4;let force=20.;
        let a=-(v+b)/h-gamma*force;
        assert!((v+h*a+b+h*gamma*force).abs()<1e-8);
        // Withdrawal requests zero force and the separating inequality, while
        // the actual contact remains available to Rapier.
        let release=(-(v+b)/h).max(0.01);assert!(v+h*release+b>=-1e-8);
        assert!(contact_can_react(-0.00001,release).unwrap());assert!(!contact_can_react(0.001,release).unwrap());
    }
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
        for _ in 0..200 {if r.dq.iter().all(|v|*v==0.) {break;}
            r=r.proposal(0.00125,&[(-1.,1.);21]).unwrap().0;}
        assert!(r.dq.iter().all(|v|*v==0.));let stopped=r.clone();
        let held=r.braking_proposal(0.00125,&[(-1.,1.);21]).unwrap().0;
        assert!(held.braking);assert_eq!(held.q,stopped.q);assert_eq!(held.dq,stopped.dq);
        let next=r.proposal(0.00125,&[(-1.,1.);21]).unwrap().0;
        assert!(!next.braking);assert!(next.q[0]>=stopped.q[0]);assert!(next.dq[0]>0.);
        assert_eq!(next.target,stopped.target);
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
        assert_eq!(half.start,r.start);assert_eq!(half.elapsed,0.00125);
        assert!(half.duration>0.00125 && half.duration==DURATION);
        let next=half.proposal(0.00125,&[(-1.,1.);21]).unwrap().0;
        assert!(next.q[0]>half.q[0]);assert!((next.dq[0]-half.dq[0]).abs()<=REFERENCE_ACCELERATION*0.00125);
    }
    #[test] fn native_step_pure_joint_partial_terminal_reference_cannot_jump_to_target() {
        let mut nominal=Reference::new([0.;21],[0.;21],[0.01;21]);nominal.elapsed=nominal.duration;
        nominal.q=nominal.target;let mut baseline=nominal.clone();baseline.q=[0.;21];baseline.braking=true;
        let (half,_)=blend_reference(&(baseline,[0.;21]),&(nominal,[0.;21]),0.5,0.00125).unwrap();
        assert_eq!(half.q,[0.005;21]);assert_eq!(half.dq,[0.;21]);assert_eq!(half.elapsed,DURATION);
        assert!(half.duration>0.00125);assert_eq!(half.target,[0.01;21]);
        let next=half.proposal(0.00125,&[(-1.,1.);21]).unwrap().0;assert!(next.q[0]<0.00501);
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
