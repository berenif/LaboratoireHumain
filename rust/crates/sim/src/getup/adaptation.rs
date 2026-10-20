//! Local-only bounded SLOW correction. No default policy or dynamics selector.
use serde::{Deserialize, Serialize};

pub const FORMAT: &str = "frozen-slow-correction-v1";
pub const SCALE: f32 = 0.05;
pub const FEATURES: usize = 1024;

#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Head {
    pub weights: Vec<f32>,
    pub bias: [f32; 21],
}
impl Head {
    pub fn validate(&self) -> bool {
        self.weights.len() == FEATURES * 21
            && self.weights.iter().chain(&self.bias).all(|v| v.is_finite())
    }
    #[cfg(feature = "getup-policy-adaptation-experiment")]
    pub fn correction(&self, features: &[f32]) -> Result<[f32; 21], String> {
        if !self.validate() || features.len() != FEATURES || features.iter().any(|v| !v.is_finite()) {
            return Err("Invalid bounded correction head/features".into());
        }
        Ok(std::array::from_fn(|i| (self.weights[i * FEATURES..(i + 1) * FEATURES]
            .iter().zip(features).fold(self.bias[i] as f64,
                |sum, (w, x)| sum + *w as f64 * *x as f64) as f32).tanh()))
    }
}
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Pending {
    pub native_substep: u64,
    pub correction: [f32; 21],
}
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Receipt {
    pub native_substep: u64,
    pub reference_decision: u64,
    pub correction: [f32; 21],
    pub original_residual: [f32; 21],
    pub applied_residual: [f32; 21],
}
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct State {
    pub format: String,
    pub head: Option<Head>,
    pub pending: Option<Pending>,
    pub last: Option<Receipt>,
}
pub fn valid_correction(c: &[f32; 21]) -> bool {
    c.iter().all(|v| v.is_finite() && (-1.0..=1.0).contains(v))
}
impl State {
    pub fn valid(&self, native: u64) -> bool {
        self.format == FORMAT && self.head.as_ref().is_none_or(Head::validate)
            && self.pending.as_ref().is_none_or(|p| p.native_substep == native && valid_correction(&p.correction))
            && self.last.as_ref().is_none_or(|r| r.native_substep <= native
                && valid_correction(&r.correction) && valid_correction(&r.original_residual)
                && valid_correction(&r.applied_residual)
                && r.applied_residual.iter().zip(r.original_residual).all(|(a,b)| (*a-b).abs() <= SCALE + 2.0 * f32::EPSILON))
            && !(self.head.is_some() && self.pending.is_some())
    }
    #[cfg(feature = "getup-policy-adaptation-experiment")]
    pub fn apply(&mut self, native: u64, decision: u64, prediction: &super::actor::Prediction,
        features: &[f32]) -> Result<[f32; 21], String>
    {
        if !self.valid(native) { return Err("Invalid bounded adaptation state/clock".into()); }
        let correction = if let Some(head) = &self.head { head.correction(features)? }
            else { self.pending.take().map_or([0.0;21], |p|p.correction) };
        let applied = std::array::from_fn(|i| {
            // The frozen branch is exact even for signed zero and WASM libm.
            if correction[i] == 0.0 { prediction.action[i] }
            else { (prediction.raw[i] + SCALE * correction[i]).tanh() }
        });
        self.last = Some(Receipt { native_substep:native, reference_decision:decision,
            correction, original_residual:prediction.action, applied_residual:applied });
        Ok(applied)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn bounded_state_rejects_nonfinite_and_out_of_range_actions() {
        let mut c = [0.0;21]; assert!(valid_correction(&c));
        c[1]=1.001; assert!(!valid_correction(&c)); c[1]=f32::NAN; assert!(!valid_correction(&c));
        assert!(!Head {weights:vec![],bias:[0.0;21]}.validate());
    }
    #[test]
    #[cfg(feature = "getup-policy-adaptation-experiment")]
    fn zero_head_reuses_original_action_bits_and_correction_is_bounded() {
        let actor = super::super::actor::student();
        let mut input=[0.0;118]; input[117]=1.0;
        let (prediction,features)=actor.infer_with_features(&input).unwrap();
        let mut state=State {format:FORMAT.into(),head:Some(Head {weights:vec![0.0;FEATURES*21],bias:[0.0;21]}),pending:None,last:None};
        let zero=state.apply(500,1,&prediction,&features).unwrap();
        assert_eq!(zero.map(f32::to_bits),prediction.action.map(f32::to_bits));
        state.head=None; state.pending=Some(Pending {native_substep:520,correction:[1.0;21]});
        let corrected=state.apply(520,2,&prediction,&features).unwrap();
        assert!(corrected.iter().zip(prediction.action).all(|(a,b)|(*a-b).abs()<=SCALE));
    }
}
