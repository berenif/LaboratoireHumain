//! Process-wide opt-in reserve. Forecast clones use the same monotone ledger.
//! Not serialized into checkpoints: restoring a world cannot replenish it.
use serde::Serialize;
use std::sync::Mutex;

#[derive(Clone, Copy)]
pub enum Kind { Real, Forecast }
#[derive(Clone, Debug, Serialize)]
pub struct Ledger {
    pub remaining: u64,
    pub real: u64,
    pub forecast: u64,
    pub acquisition: u64,
    pub step: u64,
    pub in_step: bool,
    step_limit:u64,
    acquisition_limit:u64,
    pub first_stop: Option<String>,
}
impl Ledger {
    fn new() -> Self { Self {remaining:8800,real:0,forecast:0,acquisition:0,step:0,in_step:false,step_limit:2400,acquisition_limit:6400,first_stop:None} }
    fn available(&mut self) -> Result<(),String> {
        if let Some(reason)=&self.first_stop {return Err(reason.clone());}
        let reason=if self.remaining==0 {Some("total-integration-limit")}
            else if !self.in_step && self.acquisition>=self.acquisition_limit {Some("acquisition-integration-limit")}
            else if self.in_step && self.step>=self.step_limit {Some("step-integration-limit")} else {None};
        if let Some(reason)=reason {self.first_stop=Some(reason.into());return Err(reason.into());}
        Ok(())
    }
    fn debit(&mut self,kind:Kind)->Result<(),String> {
        self.available()?;
        self.remaining-=1;
        match kind {Kind::Real=>self.real+=1,Kind::Forecast=>self.forecast+=1}
        if self.in_step {self.step+=1} else {self.acquisition+=1}
        Ok(())
    }
    fn begin_step(&mut self)->Result<(),String> {
        if self.in_step || self.first_stop.is_some() {return Err("budget phase cannot restart".into());}
        self.in_step=true;Ok(())
    }
}
static RESERVE: Mutex<Option<Ledger>>=Mutex::new(None);
pub fn install()->Result<(),String> { install_limit(3064) }
pub fn install_continuation()->Result<(),String> { install_limit(2400) }
#[cfg(test)]
pub(crate) fn install_identification()->Result<(),String> { install_limit(4) }
pub fn install_getup_video_capture()->Result<(),String> {install_limits(8000,2400,8000)}
pub fn install_post_separation_025()->Result<(),String> { install_limit_with_step(12450,12450) }
pub fn install_reception010_replay()->Result<(),String> { install_limit(2000) }
pub fn install_resume_6330()->Result<(),String> { install_limit(1856) }
pub fn install_resume_6186()->Result<(),String> { install_limit(1000) }
pub fn install_resume_6182()->Result<(),String> { install_limit(1000) }
pub fn install_resume_6153()->Result<(),String> { install_limit(1200) }
pub fn install_resume_5579()->Result<(),String> { install_limit(1500) }
pub fn install_comparative_5339()->Result<(),String> { install_limit(2000) }
pub fn install_coordinated_resume()->Result<(),String> { install_limit(2200) }
pub fn install_short_resume()->Result<(),String> { install_limit(800) }
fn install_limit(limit:u64)->Result<(),String> {install_limit_with_step(limit,2400)}
fn install_limit_with_step(limit:u64,step_limit:u64)->Result<(),String> {install_limits(limit,step_limit,6400)}
fn install_limits(limit:u64,step_limit:u64,acquisition_limit:u64)->Result<(),String> {
    let mut slot=RESERVE.lock().map_err(|_|"budget lock poisoned")?;
    if slot.is_some() {return Err("global reserve already installed".into());}
    // Second trial consumed 5736; no new trial authorized by this reserve.
    let mut reserve=Ledger::new();reserve.remaining=limit;reserve.step_limit=step_limit;reserve.acquisition_limit=acquisition_limit;
    *slot=Some(reserve);Ok(())
}
pub fn snapshot()->Result<Ledger,String> {
    RESERVE.lock().map_err(|_|"budget lock poisoned")?.clone().ok_or("budget not installed".into())
}
pub fn begin_step()->Result<(),String> {
    RESERVE.lock().map_err(|_|"budget lock poisoned")?.as_mut().ok_or("budget not installed")?.begin_step()
}
pub fn available()->Result<(),String> {
    match RESERVE.lock().map_err(|_|"budget lock poisoned")?.as_mut() {Some(b)=>b.available(),None=>Ok(())}
}
pub fn debit(kind:Kind)->Result<(),String> {
    match RESERVE.lock().map_err(|_|"budget lock poisoned")?.as_mut() {Some(b)=>b.debit(kind),None=>Ok(())}
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test] fn budget_pure_global_reserve_cannot_reinstall() {
        install().unwrap();
        let a=std::thread::spawn(||for _ in 0..10 {debit(Kind::Real).unwrap();});
        let b=std::thread::spawn(||for _ in 0..10 {debit(Kind::Forecast).unwrap();});
        a.join().unwrap();b.join().unwrap();
        assert!(install().is_err());
        let b=snapshot().unwrap();assert_eq!((b.real,b.forecast,b.remaining),(10,10,3044));
    }
    #[test] fn budget_pure_shared_real_forecast_last_token() {
        let shared=Mutex::new(Ledger::new());
        for _ in 0..3200 {shared.lock().unwrap().debit(Kind::Real).unwrap();shared.lock().unwrap().debit(Kind::Forecast).unwrap();}
        let mut b=shared.lock().unwrap();assert_eq!((b.real,b.forecast,b.remaining),(3200,3200,2400));
        assert_eq!(b.debit(Kind::Forecast).unwrap_err(),"acquisition-integration-limit");
        assert_eq!(b.remaining,2400);assert!(b.begin_step().is_err());
    }
    #[test] fn budget_pure_phase_does_not_refill() {
        let mut b=Ledger::new();for _ in 0..6399 {b.debit(Kind::Forecast).unwrap();}
        b.begin_step().unwrap();assert!(b.begin_step().is_err());
        for _ in 0..2400 {b.debit(Kind::Real).unwrap();}
        assert_eq!(b.remaining,1);assert!(b.debit(Kind::Forecast).is_err());
        assert_eq!(b.real+b.forecast,8799);
    }
    #[test] fn budget_pure_total_boundary_and_sticky_reason() {
        let mut b=Ledger::new();for _ in 0..6400 {b.debit(Kind::Real).unwrap();}
        b.begin_step().unwrap();for _ in 0..2400 {b.debit(Kind::Forecast).unwrap();}
        assert_eq!(b.remaining,0);assert_eq!(b.debit(Kind::Real).unwrap_err(),"total-integration-limit");
        assert_eq!(b.debit(Kind::Forecast).unwrap_err(),"total-integration-limit");assert_eq!(b.real+b.forecast,8800);
    }
}

// Small process-local diagnostic, never fed back into the controller or forces.
static COMMANDS: Mutex<(Option<serde_json::Value>,Option<serde_json::Value>)>=Mutex::new((None,None));
static LAST_COMMAND: Mutex<Option<serde_json::Value>>=Mutex::new(None);
pub fn last_step_command()->Option<serde_json::Value> {LAST_COMMAND.lock().ok().and_then(|v|v.clone())}
pub fn record_step_command(record:serde_json::Value) {
    if let Ok(mut last)=LAST_COMMAND.lock() {*last=Some(record.clone());}
    if let Ok(mut saved)=COMMANDS.lock() {
        let slot=if record["error"].is_null() {&mut saved.0} else {&mut saved.1};
        if slot.is_none() {*slot=Some(record);}
    }
}
pub fn step_command_diagnostics()->serde_json::Value {
    match COMMANDS.lock() {
        Ok(saved)=>serde_json::json!({"firstAdmitted":saved.0,"firstRefused":saved.1}),
        Err(_)=>serde_json::json!({"error":"diagnostic lock poisoned"}),
    }
}

#[cfg(test)]
mod diagnostic_tests {
    use super::*;
    #[test] fn budget_pure_diagnostic_preserves_first_refusal_exactly() {
        let measured=f32::from_bits(0x3e9e0652);
        record_step_command(serde_json::json!({"error":null,"native":10}));
        record_step_command(serde_json::json!({"error":"first","native":11,"measured":measured}));
        record_step_command(serde_json::json!({"error":"later","native":12}));
        let saved=step_command_diagnostics();
        assert_eq!(saved["firstAdmitted"]["native"],10);
        assert_eq!(saved["firstRefused"]["native"],11);
        let recovered:f32=serde_json::from_value(saved["firstRefused"]["measured"].clone()).unwrap();
        assert_eq!(recovered.to_bits(),measured.to_bits());
    }
}

#[cfg(test)]
#[test]
fn placement_budget_pure_real_and_forecast_share_one_reserve() {
 let mut b=Ledger::new();b.remaining=12450;b.step_limit=12450;b.begin_step().unwrap();
 for i in 0..12450 {b.debit(if i%2==0 {Kind::Real}else{Kind::Forecast}).unwrap();}
 assert_eq!((b.real,b.forecast,b.step,b.remaining),(6225,6225,12450,0));
 assert!(b.debit(Kind::Real).is_err());assert!(b.begin_step().is_err());
}
