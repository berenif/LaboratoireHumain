//! Explicit saved-case experiment. Desired joint curve only; no world edits.
//! The finite-FK preparation is conditional, not a future-contact certificate.
use super::native_step::Reference;
use crate::*;
#[derive(Clone,Serialize,Deserialize)]
#[serde(rename_all="camelCase")]
pub(crate) struct Path {
    format:String, entry_native:u64, duration_seconds:f64, descent_at_seconds:f64,
    entry_q:[f32;21], entry_dq:[f32;21], entry_acceleration:[f32;21],
    foot_goal:[f32;3], com_goal:[f32;3], segments:Vec<Segment>,
    #[serde(default)] pub started:bool,
    #[serde(default)] geometric_duration_seconds:Option<f64>,
    #[serde(default)] clock_polynomial:Option<[f64;6]>,
    #[serde(default)] measured_entry_q:Option<[f32;21]>,
    #[serde(default)] measured_entry_dq:Option<[f32;21]>,
    #[serde(default)] transition_source:Option<[f32;21]>,
}
#[derive(Clone,Serialize,Deserialize)]
struct Segment {start:f64,end:f64,bezier:[[f64;21];6]}
fn value<const N:usize>(mut p:[[f64;21];N],t:f64)->[f64;21] {
    for n in (1..N).rev() {for k in 0..n {for j in 0..21 {p[k][j]=(1.-t)*p[k][j]+t*p[k+1][j];}}}p[0]
}
impl Path {
    pub fn limited(&self)->bool {self.format=="post-separation-limited-curve-v2"}
    pub fn start_measured(&mut self,s:&Simulation,r:&mut Reference)->Result<(),String> {
        if !self.limited(){return self.start(s.substeps_completed,r);}
        if s.substeps_completed!=self.entry_native {return Err("Limited placement missed measured entry".into());}
        let (q,dq)=super::joint_state(s)?;
        for (actual,expected) in [(q,self.measured_entry_q.ok_or("Missing measured entry q")?),(dq,self.measured_entry_dq.ok_or("Missing measured entry dq")?)] {
            if actual.iter().zip(expected).any(|(a,b)|!a.is_finite() || (*a-b).abs()>2e-6) {return Err("Limited placement physical entry changed; no guessed restoration".into());}
        }
        self.transition_source=Some(s.native_locomotion.as_ref().and_then(|n|n.last_application.as_ref()).filter(|a|a.owner==super::locomotion::Owner::Step && s.substeps_completed.checked_sub(a.native_input)==Some(1)).ok_or("Missing previous actual Step command")?.controls);
        r.q=self.entry_q;r.dq=self.entry_dq;r.acceleration=self.entry_acceleration;
        self.started=true;Ok(())
    }
    pub fn transition(&self,native:u64,controls:[f32;21])->Result<[f32;21],String> {
        if !self.limited(){return Ok(controls);}
        let source=self.transition_source.ok_or("Missing placement command transition")?;
        let k=native.checked_sub(self.entry_native).ok_or("Placement clock reversed")?;
        let w=super::controller::terminal_bumpless_weight(k.min(20) as f64/20.);
        Ok(std::array::from_fn(|i|if k==0{source[i]}else{((1.-w)*source[i] as f64+w*controls[i] as f64) as f32}))
    }

    pub fn parse(json:&str,bounds:&[(f32,f32);21])->Result<Self,String> {
        let p:Self=serde_json::from_str(json).map_err(|e|e.to_string())?;p.validate(bounds)?;
        if p.started {return Err("Cannot arm an already started placement curve".into());}Ok(p)
    }
    pub fn validate(&self,bounds:&[(f32,f32);21])->Result<(),String> {
        if (!self.limited() && self.format!="post-separation-finite-curve-v1") || self.entry_native!=6309 || self.segments.is_empty() || self.segments.len()>128
            || !self.duration_seconds.is_finite() || self.duration_seconds<=0. || self.duration_seconds>30.
            || !(0.<self.descent_at_seconds && self.descent_at_seconds<self.duration_seconds)
            || self.foot_goal.iter().chain(&self.com_goal).any(|v|!v.is_finite()) {return Err("Invalid prospective placement curve".into());}
        if self.limited() && (self.duration_seconds>0.7 || self.geometric_duration_seconds.is_none_or(|t|!t.is_finite() || t<=0.) || self.clock_polynomial.is_none_or(|p|p.iter().any(|v|!v.is_finite())) || self.measured_entry_q.is_none() || self.measured_entry_dq.is_none()) {return Err("Invalid limited placement boundary".into());}
        let mut end=0.;let mut previous:Option<([f32;21],[f32;21],[f32;21])>=None;
        for s in &self.segments {
            let h=s.end-s.start;
            if !h.is_finite() || h<=0. || (s.start-end).abs()>1e-8 {return Err("Discontinuous placement clock".into());}
            for j in 0..21 {
                if s.bezier.iter().any(|p|!p[j].is_finite() || p[j]<bounds[j].0 as f64-1e-7 || p[j]>bounds[j].1 as f64+1e-7) {return Err("Placement Bernstein ROM bound".into());}
                let v:Vec<_>=(0..5).map(|k|5.*(s.bezier[k+1][j]-s.bezier[k][j])/h).collect();
                if !self.limited() && (v.iter().any(|v|v.abs()>0.5) || v.windows(2).any(|v|(4.*(v[1]-v[0])/h).abs()>2.)) {return Err("Placement derivative envelope".into());}
            }
            let initial=Self::on_segment(s,0.);let wanted=previous.unwrap_or_else(||if self.limited(){initial}else{(self.entry_q,self.entry_dq,self.entry_acceleration)});
            for (a,b) in [(initial.0,wanted.0),(initial.1,wanted.1),(initial.2,wanted.2)] {
                if a.iter().zip(b).any(|(a,b)|!a.is_finite() || !b.is_finite() || (*a-b).abs()>2e-6) {return Err("Placement q/v/a discontinuity".into());}
            }
            previous=Some(Self::on_segment(s,1.));end=s.end;
        }
        if (end-self.geometric_duration_seconds.filter(|_|self.limited()).unwrap_or(self.duration_seconds)).abs()>1e-8 {return Err("Placement duration differs from curve".into());}Ok(())
    }
    fn on_segment(s:&Segment,u:f64)->([f32;21],[f32;21],[f32;21]) {
        let h=s.end-s.start;let v:[[f64;21];5]=std::array::from_fn(|i|std::array::from_fn(|j|5.*(s.bezier[i+1][j]-s.bezier[i][j])/h));
        let a:[[f64;21];4]=std::array::from_fn(|i|std::array::from_fn(|j|4.*(v[i+1][j]-v[i][j])/h));
        (value(s.bezier,u).map(|x|x as f32),value(v,u).map(|x|x as f32),value(a,u).map(|x|x as f32))
    }
    pub fn can_arm(&self,native:u64)->bool {native<self.entry_native && !self.started}
    pub fn due(&self,native:u64)->bool {native>=self.entry_native}
    pub fn remaining(&self,native:u64,dt:f32)->f64 {self.duration_seconds-(native.saturating_sub(self.entry_native) as f64)*dt as f64}
    pub fn start(&mut self,native:u64,r:&Reference)->Result<(),String> {
        if native!=self.entry_native {return Err("Saved-case placement missed its exact separation boundary".into());}
        let actual=[r.q,r.dq,r.acceleration];let expected=[self.entry_q,self.entry_dq,self.entry_acceleration];
        if actual.iter().zip(expected).any(|(a,b)|a.iter().zip(b).any(|(a,b)|(*a-b).abs()>2e-6)) {return Err("Placement entry differs from preserved historical reference; no reset allowed".into());}
        self.started=true;Ok(())
    }
    pub fn sample(&self,native:u64,dt:f32,r:&Reference)->Result<(Reference,[f32;21],bool,Vector,Vector),String> {
        let t=native.checked_sub(self.entry_native).ok_or("Placement clock reversed")? as f64*dt as f64;
        if !self.started || t>self.duration_seconds {return Err("Prospective placement window ended without reception".into());}
        let (sample_t,vs,acs)=if self.limited(){
            let p=self.clock_polynomial.unwrap();let z=t/self.duration_seconds;
            let u=(0..6).map(|i|p[i]*z.powi(i as i32)).sum::<f64>();
            let du=(1..6).map(|i|i as f64*p[i]*z.powi(i as i32-1)).sum::<f64>();
            let ddu=(2..6).map(|i|(i*(i-1)) as f64*p[i]*z.powi(i as i32-2)).sum::<f64>();
            let g=self.geometric_duration_seconds.unwrap();
            if !u.is_finite() || !du.is_finite() || du<0. || u< -1e-9 || u>1.+1e-9 {return Err("Invalid limited placement clock".into());}
            (u.clamp(0.,1.)*g,g*du/self.duration_seconds,g*ddu/self.duration_seconds.powi(2))
        }else{(t,1.,0.)};
        let s=self.segments.iter().find(|s|sample_t<=s.end+1e-10).ok_or("Missing placement segment")?;
        let (q,mut v,mut a)=Self::on_segment(s,((sample_t-s.start)/(s.end-s.start)).clamp(0.,1.));
        for i in 0..21 {a[i]=(a[i] as f64*vs*vs+v[i] as f64*acs) as f32;v[i]=(v[i] as f64*vs) as f32;}
        if q.iter().chain(&v).chain(&a).any(|x|!x.is_finite()) {return Err("Nonfinite placement reference".into());}
        let mut next=r.clone();next.q=q;next.dq=v;next.acceleration=a;
        Ok((next,a,t>=self.descent_at_seconds,Vector::from_array(self.com_goal),Vector::from_array(self.foot_goal)))
    }
}
