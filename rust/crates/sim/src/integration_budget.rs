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
    pub first_stop: Option<String>,
}
impl Ledger {
    fn new() -> Self { Self {remaining:8800,real:0,forecast:0,acquisition:0,step:0,in_step:false,first_stop:None} }
    fn available(&mut self) -> Result<(),String> {
        if let Some(reason)=&self.first_stop {return Err(reason.clone());}
        let reason=if self.remaining==0 {Some("total-integration-limit")}
            else if !self.in_step && self.acquisition>=6400 {Some("acquisition-integration-limit")}
            else if self.in_step && self.step>=2400 {Some("step-integration-limit")} else {None};
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
pub fn install()->Result<(),String> {
    let mut slot=RESERVE.lock().map_err(|_|"budget lock poisoned")?;
    if slot.is_some() {return Err("global reserve already installed".into());}
    *slot=Some(Ledger::new());Ok(())
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
        let b=snapshot().unwrap();assert_eq!((b.real,b.forecast,b.remaining),(10,10,8780));
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
