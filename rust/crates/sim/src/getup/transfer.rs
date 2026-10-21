//! Default-off, gravity-only weight transfer. No swing, root actuation or pose writes.
//! Intentions are computed from the current 21-hinge body; only solved contacts
//! can qualify an unload. The deployed standing/recovery controller is unchanged.
use crate::*;
use rapier3d::na::{DMatrix, DVector};

pub const FORMAT: &str = "getup-supported-transfer-v1";
const LIMIT_NATIVE: u64 = 2000; // 2.5 physical seconds, including 20 qualified samples.
const RETURN_NATIVE: u64 = 3200; // bounded return plus a new 800-sample standing interval.
const MAX_SHIFT: f32 = 0.120;
const MAX_SPEED: f32 = 0.050;
const RAMP_S: f32 = 0.025;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Profile {
    #[serde(rename = "gravity-only-supported-transfer-v1")]
    GravityOnlyV1,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Phase { Transfer, Returning, Passed, Failed, Cancelled }
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct MaterialAnchor { body: usize, local: Vec3, world: Vec3 }
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct State {
    format: String,
    pub phase: Phase,
    pub retained_side: usize,
    start_native: u64,
    phase_native: u64,
    last_observed_native: u64,
    source_native: u64,
    last_fraction: f32,
    return_start_fraction: f32,
    pub observed_samples: u64,
    pub qualified_samples: u64,
    pub transfer_passed_native: Option<u64>,
    pub return_passed_native: Option<u64>,
    pub failure: Option<String>,
    start_com: Vec3,
    target_com: Vec3,
    start_q: [f32;21],
    target_q: [f32;21],
    forward: Vec3,
    lateral: Vec3,
    origins: [Vec3;2],
    foot_rotations: [Quat;2],
    material: Vec<MaterialAnchor>,
    pub maximum_origin_drift_m: f32,
    pub maximum_material_excursion_m: f32,
    pub maximum_control: f32,
    pub minimum_retained_load_n: f32,
    pub minimum_swing_load_n: f32,
    pub maximum_swing_clearance_m: f32,
    pub retained_com_inside: bool,
    pub retained_capture_inside: bool,
    pub last_loads_n: [f32;2],
}
impl State {
    pub fn active(&self) -> bool { matches!(self.phase,Phase::Transfer|Phase::Returning) }
    pub fn valid(&self, s: &Simulation, source: Option<u64>) -> bool {
        self.format == FORMAT && cfg!(feature="getup-supported-transfer-experiment")
            && s.profile.getup_supported_transfer == Some(Profile::GravityOnlyV1)
            && self.retained_side < 2 && self.start_native <= self.phase_native
            && self.phase_native <= self.last_observed_native
            && self.last_observed_native <= s.substeps_completed
            && (!self.active() || self.last_observed_native==s.substeps_completed)
            && (!self.active() || (source == Some(self.source_native)
                && s.substeps_completed-self.phase_native <= if self.phase==Phase::Transfer {LIMIT_NATIVE} else {RETURN_NATIVE}))
            && self.last_fraction.is_finite() && (0.0..=1.0).contains(&self.last_fraction)
            && self.return_start_fraction.is_finite() && (0.0..=1.0).contains(&self.return_start_fraction)
            && self.observed_samples==self.last_observed_native-self.start_native
            && self.qualified_samples <= 20 && self.material.len() <= 64 && !self.material.is_empty()
            && self.material.iter().all(|a| feet(s).is_ok_and(|f|f.contains(&a.body))
                && vector(a.local).is_finite() && vector(a.world).is_finite())
            && [self.start_com,self.target_com,self.forward,self.lateral].iter().all(|v|vector(*v).is_finite())
            && vector(self.start_com).distance(vector(self.target_com)) <= MAX_SHIFT+1e-6
            && self.start_q.iter().chain(&self.target_q).all(|x|x.is_finite())
            && self.origins.iter().all(|v|vector(*v).is_finite())
            && self.foot_rotations.iter().all(|q|rotation(*q).is_finite() && (rotation(*q).length_squared()-1.).abs()<1e-4)
            && [self.maximum_origin_drift_m,self.maximum_material_excursion_m,self.maximum_control,
                self.minimum_retained_load_n,self.minimum_swing_load_n,self.maximum_swing_clearance_m]
                .iter().all(|v|v.is_finite())
    }
    pub fn cancel(&mut self, reason: &str) { if self.active() { self.phase=Phase::Cancelled; self.failure=Some(reason.into()); } }
    fn fail(&mut self, message: &str) -> String {
        self.phase=Phase::Failed;self.failure=Some(message.into());message.into()
    }
    pub fn begin(s: &Simulation, retained_side: usize, source_native: u64,
        start_q: [f32;21], reference: super::supported_standing::StandingReference) -> Result<Self,String>
    {
        if !cfg!(feature="getup-supported-transfer-experiment") || s.profile.getup_supported_transfer.is_none()
            || retained_side>1 || s.grab.is_some() || s.environment.is_some()
            || !s.world.colliders[s.floor].is_enabled() || s.first_failure.is_some()
            || s.motion!=Motion::Upright || s.striker.as_ref().is_some_and(|p|!p.snapshot(&s.world).available)
        { return Err("Transfer requires its gravity-only, quiet flat-floor protocol".into()); }
        let ids=feet(s)?;let contacts=super::standing::contacts(s)?;
        let points:Vec<_>=contacts.iter().filter(|c|c.body==ids[retained_side]).map(|c|[c.point.x,c.point.z]).collect();
        let center=points.iter().fold(Vector::ZERO,|a,p|a+Vector::new(p[0],0.,p[1])) / points.len().max(1) as f32;
        if !crate::stepping::inside_support(points,[center.x,center.z],0.003) {
            return Err("Retained actual contact hull has no 3 mm interior at its centroid".into());
        }
        let start_com=reference.com;
        let target_com=Vector::new(center.x,start_com.y,center.z);
        if target_com.distance(start_com)<0.0001 || target_com.distance(start_com)>MAX_SHIFT {return Err("Required COM shift exceeds explicit 120 mm command bound".into());}
        let target_q=posture_target(s,target_com-s.center_of_mass())?;
        let material=contacts.iter().map(|c|MaterialAnchor{body:c.body,
            local:vec3(s.world.bodies[s.bodies[c.body]].position().inverse_transform_point(c.point)),world:vec3(c.point)}).collect::<Vec<_>>();
        if material.is_empty() || material.len()>64 {return Err("Invalid transfer material anchors".into());}
        let loads=ids.map(|i|s.integrated_support_loads_n[i]);
        Ok(Self {format:FORMAT.into(),phase:Phase::Transfer,retained_side,start_native:s.substeps_completed,
            phase_native:s.substeps_completed,last_observed_native:s.substeps_completed,source_native,last_fraction:0.,return_start_fraction:0.,
            observed_samples:0,qualified_samples:0,transfer_passed_native:None,return_passed_native:None,failure:None,
            start_com:vec3(start_com),target_com:vec3(target_com),start_q,target_q,forward:vec3(reference.forward),lateral:vec3(reference.lateral),
            origins:ids.map(|i|vec3(s.world.bodies[s.bodies[i]].translation())),
            foot_rotations:ids.map(|i|quat(s.world.bodies[s.bodies[i]].rotation().normalize())),material,
            maximum_origin_drift_m:0.,maximum_material_excursion_m:0.,maximum_control:0.,
            minimum_retained_load_n:loads[retained_side],minimum_swing_load_n:loads[1-retained_side],
            maximum_swing_clearance_m:0.,retained_com_inside:false,retained_capture_inside:false,last_loads_n:loads})
    }
    pub fn command(&mut self,s:&Simulation)->Result<[f32;21],String>{
        if !self.active(){return Err("Inactive transfer cannot own motors".into());}
        if !matches!(s.motion,Motion::Upright|Motion::Reacting) || s.first_failure.is_some() {
            return Err(self.fail("Transfer lost upright lifecycle"));
        }
        if s.grab.is_some() || s.striker.as_ref().is_some_and(|p|!p.snapshot(&s.world).available) {
            return Err(self.fail("Gravity-only transfer rejects active grab/striker wrench"));
        }
        let elapsed=s.substeps_completed.checked_sub(self.phase_native).ok_or("Transfer clock reversed")?;
        let limit=if self.phase==Phase::Transfer {LIMIT_NATIVE}else{RETURN_NATIVE};
        if elapsed>=limit {return Err(self.fail("Measured transfer/return deadline expired; no retry"));}
        let distance=vector(self.target_com).distance(vector(self.start_com));
        let returning=self.phase==Phase::Returning;
        let fraction=ramp_fraction(elapsed as f32*s.profile.dt_s,if returning {distance*self.return_start_fraction}else{distance});
        let f=if returning {self.return_start_fraction*(1.-fraction)}else{fraction};
        self.last_fraction=f;
        let reference=super::supported_standing::StandingReference{com:vector(self.start_com).lerp(vector(self.target_com),f),
            forward:vector(self.forward),lateral:vector(self.lateral)};
        let q_target=std::array::from_fn(|i|self.start_q[i]+f*(self.target_q[i]-self.start_q[i]));
        let controls=super::supported_standing::transfer_controls(s,&reference,&q_target,self.retained_side,f)
            .map_err(|e|self.fail(&e))?;
        let peak=controls.iter().map(|v|v.abs()).fold(0.,f32::max);self.maximum_control=self.maximum_control.max(peak);
        Ok(controls)
    }
    pub fn observe(&mut self,s:&Simulation,stable_native:u64)->Result<(),String>{
        if !self.active(){return Ok(());}
        if s.substeps_completed!=self.last_observed_native+1 {return Err(self.fail("Missing/duplicate native transfer observation"));}
        self.last_observed_native=s.substeps_completed;self.observed_samples+=1;
        let ids=feet(s)?;let loads=ids.map(|i|s.integrated_support_loads_n[i]);self.last_loads_n=loads;
        self.minimum_retained_load_n=self.minimum_retained_load_n.min(loads[self.retained_side]);
        self.minimum_swing_load_n=self.minimum_swing_load_n.min(loads[1-self.retained_side]);
        for (i,anchor) in ids.into_iter().zip(self.origins) {
            self.maximum_origin_drift_m=self.maximum_origin_drift_m.max(s.world.bodies[s.bodies[i]].translation().distance(vector(anchor)));
        }
        for a in &self.material {
            if s.integrated_support_loads_n[a.body]>=3. {
                let p=s.world.bodies[s.bodies[a.body]].position().transform_point(vector(a.local));
                self.maximum_material_excursion_m=self.maximum_material_excursion_m.max(p.distance(vector(a.world)));
            }
        }
        // Newly appearing solved material points are also anchored in the
        // original foot pose, not only the initial contact set.
        for c in super::standing::contacts(s)? {
            let side=ids.iter().position(|i|*i==c.body).ok_or("Unexpected transfer foot")?;
            let local=s.world.bodies[s.bodies[c.body]].position().inverse_transform_point(c.point);
            let original=vector(self.origins[side])+rotation(self.foot_rotations[side])*local;
            self.maximum_material_excursion_m=self.maximum_material_excursion_m.max(c.point.distance(original));
        }
        self.maximum_swing_clearance_m=self.maximum_swing_clearance_m.max(super::foot_floor_clearance(s)?[1-self.retained_side]);
        if self.maximum_origin_drift_m>0.01 || self.maximum_material_excursion_m>0.02 {
            return Err(self.fail("Original foot-origin/material-contact drift limit exceeded"));
        }
        if s.integrated_support_loads_n.iter().enumerate().any(|(i,l)|!ids.contains(&i)&&*l>=3.) {
            return Err(self.fail("Nonfoot support during transfer"));
        }
        let (q,_)=super::joint_state(s)?;
        for p in &s.model.segments {if let Some(h)=p.reference.as_ref().and_then(|r|r.hinge.as_ref()){
            let a=&p.joint_profile.as_ref().unwrap().axes[0];
            if q[h.qpos_index]<a.min_radians || q[h.qpos_index]>a.max_radians {
                return Err(self.fail("Actual transfer joint limit exceeded"));
            }
        }}
        let contacts=super::standing::contacts(s)?;
        let points=contacts.iter().filter(|c|c.body==ids[self.retained_side]).map(|c|[c.point.x,c.point.z]).collect::<Vec<_>>();
        let com=s.center_of_mass();let twists=super::current_twists(s)?;
        let velocity=s.bodies.iter().zip(twists).map(|(h,(v,_))|v*s.world.bodies[*h].mass()).sum::<Vector>()/s.total_mass();
        let height=com.y-s.world.colliders[s.floor].compute_aabb().maxs.y;
        let capture=com+velocity*(height.max(0.)/9.81).sqrt();
        self.retained_com_inside=crate::stepping::inside_support(points.clone(),[com.x,com.z],0.003);
        self.retained_capture_inside=crate::stepping::inside_support(points,[capture.x,capture.z],0.003);
        let good=loads[self.retained_side]>=0.8*s.total_mass()*9.81 && loads[1-self.retained_side]<3.
            && self.retained_com_inside && self.retained_capture_inside;
        if self.phase==Phase::Transfer {
            self.qualified_samples=if good {self.qualified_samples+1}else{0};
            if self.qualified_samples==20 {
                self.transfer_passed_native=Some(s.substeps_completed);self.phase=Phase::Returning;
                self.phase_native=s.substeps_completed;self.return_start_fraction=self.last_fraction;
            }
        }else if stable_native>=800 && ramp_fraction((s.substeps_completed-self.phase_native) as f32*s.profile.dt_s,
            vector(self.target_com).distance(vector(self.start_com))*self.return_start_fraction)>=1. {
            self.return_passed_native=Some(s.substeps_completed);self.phase=Phase::Passed;
        }
        Ok(())
    }
}
fn feet(s:&Simulation)->Result<[usize;2],String>{
    let find=|name|s.model.segments.iter().position(|p|p.id==name).ok_or_else(||format!("Missing {name}"));
    Ok([find("left_foot")?,find("right_foot")?])
}
/// Trapezoidal speed, never over 50 mm/s; 25 ms start/stop ramps. No lift target.
fn ramp_fraction(t:f32,d:f32)->f32{
    if d<1e-7{return 1.;}
    let r=RAMP_S.min((d/MAX_SPEED).sqrt()*RAMP_S.sqrt());
    let speed=MAX_SPEED.min(d/r);let duration=d/speed+r;
    let x=if t<=0.{0.}else if t<r{speed*t*t/(2.*r)}else if t<duration-r{speed*(t-r/2.)}
        else if t<duration{d-speed*(duration-t).powi(2)/(2.*r)}else{d};
    (x/d).clamp(0.,1.)
}
/// Analytic source-coordinate spatial Jacobian at an actual world point.
/// Virtual root columns aid a posture intention; they are NEVER motor outputs.
fn point_jacobian(s:&Simulation,body:usize,point:Vector)->Result<DMatrix<f32>,String>{
    let mut j=DMatrix::zeros(6,27);let root=s.world.bodies[s.bodies[0]].translation();
    for k in 0..3 {let axis=[Vector::X,Vector::Y,Vector::Z][k];j[(k,k)]=1.;
        let v=axis.cross(point-root);for r in 0..3{j[(r,k+3)]=v[r];j[(r+3,k+3)]=axis[r];}}
    let mut child=body;
    while child>0 {
        let part=&s.model.segments[child];let parent=s.model.segments.iter().position(|p|Some(&p.id)==part.parent.as_ref()).ok_or("Missing transfer parent")?;
        if parent>=child{return Err("Non-topological transfer model".into());}
        if let Some(h)=part.reference.as_ref().and_then(|r|r.hinge.as_ref()){
            let jp=part.joint_profile.as_ref().ok_or("Missing transfer hinge")?;let p=&s.world.bodies[s.bodies[parent]];
            let axis=(p.rotation().normalize()*rotation(jp.parent_frame.rotation)*Vector::X).normalize();
            let anchor=p.translation()+p.rotation().normalize()*vector(jp.parent_frame.anchor);let v=axis.cross(point-anchor);
            for r in 0..3{j[(r,6+h.qpos_index)]=v[r];j[(r+3,6+h.qpos_index)]=axis[r];}
        }child=parent;
    }Ok(j)
}
/// One bounded linear posture intention: preserve BOTH foot positions/orientations
/// and torso orientation while shifting COM. No simulation clone or world write.
fn posture_target(s:&Simulation,delta:Vector)->Result<[f32;21],String>{
    let mut normal=DMatrix::identity(27,27)*0.001;let mut rhs=DVector::zeros(27);
    let mut add=|a:DMatrix<f32>,b:DVector<f32>,weight:f32|{normal+=a.transpose()*&a*weight;rhs+=a.transpose()*b*weight;};
    for foot in feet(s)? {let b=&s.world.bodies[s.bodies[foot]];
        add(point_jacobian(s,foot,b.translation())?,DVector::zeros(6),100.);}
    let root=point_jacobian(s,0,s.world.bodies[s.bodies[0]].translation())?;
    add(root.rows(3,3).into_owned(),DVector::zeros(3),10.);
    let mut com=DMatrix::zeros(3,27);
    for (i,h) in s.bodies.iter().enumerate(){let b=&s.world.bodies[*h];let p=b.translation()+b.rotation().normalize()*b.local_center_of_mass();
        com+=point_jacobian(s,i,p)?.rows(0,3)*(b.mass()/s.total_mass());}
    add(com.clone(),DVector::from_column_slice(&delta.to_array()),10.);
    let solved=normal.lu().solve(&rhs).ok_or("Transfer posture solve failed")?;
    if solved.iter().any(|x|!x.is_finite()) || (com*&solved-DVector::from_column_slice(&delta.to_array())).norm()>0.005 {
        return Err("Transfer posture intention cannot reach bounded COM target".into());
    }
    for foot in feet(s)? {
        let p=s.world.bodies[s.bodies[foot]].translation();
        let residual=point_jacobian(s,foot,p)?*&solved;
        if residual.rows(0,3).norm()>0.002 || residual.rows(3,3).norm()>0.01 {
            return Err("Transfer posture would command excessive foot displacement".into());
        }
    }
    let (q,_)=super::joint_state(s)?;let target=std::array::from_fn(|i|q[i]+solved[6+i]);
    for p in &s.model.segments {if let Some(h)=p.reference.as_ref().and_then(|r|r.hinge.as_ref()){
        let a=&p.joint_profile.as_ref().unwrap().axes[0];let x=target[h.qpos_index];
        if solved[6+h.qpos_index].abs()>0.35 || x<a.min_radians+0.005 || x>a.max_radians-0.005 {
            return Err("Transfer posture exceeds unchanged joint limits or 0.35 rad intention bound".into());
        }
    }}
    Ok(target)
}

#[cfg(test)]
pub(crate) fn synthetic_state(s:&Simulation)->State {
    let ids=feet(s).unwrap();State{format:FORMAT.into(),phase:Phase::Transfer,retained_side:0,
        start_native:0,phase_native:0,last_observed_native:0,source_native:0,last_fraction:0.,return_start_fraction:0.,
        observed_samples:0,qualified_samples:0,transfer_passed_native:None,return_passed_native:None,failure:None,
        start_com:vec3(s.center_of_mass()),target_com:vec3(s.center_of_mass()+Vector::X*0.05),
        start_q:[0.;21],target_q:[0.;21],forward:vec3(Vector::Z),lateral:vec3(Vector::X),
        origins:ids.map(|i|vec3(s.world.bodies[s.bodies[i]].translation())),
        foot_rotations:ids.map(|i|quat(s.world.bodies[s.bodies[i]].rotation().normalize())),
        material:vec![MaterialAnchor{body:ids[0],local:vec3(Vector::ZERO),world:vec3(s.world.bodies[s.bodies[ids[0]]].translation())}],
        maximum_origin_drift_m:0.,maximum_material_excursion_m:0.,maximum_control:0.,minimum_retained_load_n:0.,
        minimum_swing_load_n:0.,maximum_swing_clearance_m:0.,retained_com_inside:false,retained_capture_inside:false,last_loads_n:[0.;2]}
}
#[cfg(test)]
mod transfer_tests {
    use super::*;
    #[test]
    fn ramp_is_bounded_and_finishes_inside_transfer_deadline(){
        for distance in [0.001,0.05,0.12] {
            let mut previous=0.;
            for n in 0..=2000 {let current=ramp_fraction(n as f32/800.,distance)*distance;
                assert!(current>=previous-1e-7 && current<=distance+1e-7);
                assert!((current-previous)*800.<=MAX_SPEED+0.00001);previous=current;
            }
            assert_eq!(ramp_fraction(2.475,distance),1.);
        }
    }
    #[test]
    fn source_hinge_jacobian_matches_generalized_twists_without_physics(){
        let mut s=Simulation::new_protocol(0.3,crate::Profile::default(),true).unwrap();
        let root=*s.world.multibody_joints.rigid_body_link(s.bodies[0]).unwrap();
        let mut v=[0.;27];for (i,x) in v.iter_mut().enumerate(){*x=(i as f32-13.)*0.01;}
        let mb=s.world.multibody_joints.get_multibody_mut(root.multibody).unwrap();
        for k in 0..6{mb.generalized_velocity_mut()[k]=v[k];}
        for (i,p) in s.model.segments.iter().enumerate(){if let Some(h)=p.reference.as_ref().and_then(|r|r.hinge.as_ref()){
            let link=mb.links().find(|l|l.rigid_body_handle()==s.bodies[i]).unwrap();let dof=link.assembly_id();
            mb.generalized_velocity_mut()[dof]=v[6+h.qpos_index];
        }}
        let before=bincode::serialize(&s.world).unwrap();let twists=super::super::current_twists(&s).unwrap();
        for (i,(velocity,angular)) in twists.into_iter().enumerate(){let b=&s.world.bodies[s.bodies[i]];
            let point=b.translation()+b.rotation().normalize()*b.local_center_of_mass();
            let j=point_jacobian(&s,i,point).unwrap()*DVector::from_column_slice(&v);
            assert!((Vector::new(j[0],j[1],j[2])-velocity).length()<1e-6);
            assert!((Vector::new(j[3],j[4],j[5])-angular).length()<1e-6);
        }
        assert_eq!(before,bincode::serialize(&s.world).unwrap());assert_eq!(s.substeps_completed,0);
    }
    #[test]
    fn default_profile_stays_off_and_wrong_or_lost_state_rejects(){
        let mut s=Simulation::new_protocol(0.,crate::Profile::default(),true).unwrap();
        assert!(s.profile.getup_supported_transfer.is_none());let mut state=synthetic_state(&s);
        assert!(!state.valid(&s,Some(0)));
        s.profile.getup_supported_transfer=Some(Profile::GravityOnlyV1);
        assert!(state.valid(&s,Some(0)));assert!(!state.valid(&s,Some(1)));
        state.format="other".into();assert!(!state.valid(&s,Some(0)));state.format=FORMAT.into();
        state.target_com.x=f32::NAN;assert!(!state.valid(&s,Some(0)));
    }
    #[test]
    fn active_state_roundtrip_and_unsupported_command_do_not_mutate_world(){
        let mut s=Simulation::new_protocol(0.,crate::Profile::default(),true).unwrap();
        s.profile.getup_supported_transfer=Some(Profile::GravityOnlyV1);s.motion=Motion::Upright;
        let state=synthetic_state(&s);let encoded=serde_json::to_vec(&state).unwrap();
        let mut restored:State=serde_json::from_slice(&encoded).unwrap();assert!(restored.valid(&s,Some(0)));
        let before=bincode::serialize(&s.world).unwrap();
        assert!(restored.command(&s).is_err());assert_eq!(restored.phase,Phase::Failed);
        assert_eq!(before,bincode::serialize(&s.world).unwrap());assert_eq!(s.substeps_completed,0);
    }
}
