//! Fixed, audited dense actor inference. No pickle, Python or dynamic model loading.
use std::sync::OnceLock;

struct Layer {
    width: usize,
    weights: Vec<f32>,
    bias: Vec<f32>,
}
pub struct Actor {
    input: usize,
    layers: Vec<Layer>,
}
#[derive(serde::Serialize)]
pub struct Prediction {
    pub raw: Vec<f32>,
    pub action: [f32; 21],
}
impl Actor {
    fn decode(bytes: &[u8], expected: usize) -> Result<Self, String> {
        if bytes.len() < 24 || &bytes[..8] != b"LHGPW001" {
            return Err("Invalid GetUp weight header".into());
        }
        let dims: Vec<_> = bytes[8..24].chunks_exact(4)
            .map(|b| u32::from_le_bytes(b.try_into().unwrap()) as usize).collect();
        if dims != [expected, 1024, 1024, 42] {
            return Err("Invalid GetUp actor dimensions".into());
        }
        let count: usize = dims.windows(2).map(|d| (d[0] + 1) * d[1]).sum();
        if bytes.len() != 24 + 4 * count { return Err("Invalid GetUp weight length".into()); }
        let values: Vec<_> = bytes[24..].chunks_exact(4)
            .map(|b| f32::from_le_bytes(b.try_into().unwrap())).collect();
        if values.iter().any(|x| !x.is_finite()) { return Err("Nonfinite GetUp weight".into()); }
        let mut offset = 0;
        let mut layers = Vec::new();
        for d in dims.windows(2) {
            let weights = values[offset..offset+d[0]*d[1]].to_vec(); offset += weights.len();
            let bias = values[offset..offset+d[1]].to_vec(); offset += bias.len();
            layers.push(Layer {width: d[0], weights, bias});
        }
        Ok(Self {input: expected, layers})
    }
    pub fn infer(&self, input: &[f32]) -> Result<Prediction, String> {
        if input.len() != self.input || input.iter().any(|v| !v.is_finite())
            || !(0.0..=1.0).contains(input.last().unwrap()) {
            return Err("Invalid GetUp actor observation".into());
        }
        let mut previous = input.to_vec();
        for (index, layer) in self.layers.iter().enumerate() {
            let mut next = Vec::with_capacity(layer.bias.len());
            for (weights, bias) in layer.weights.chunks_exact(layer.width).zip(&layer.bias) {
                // Wider accumulation is deterministic on native and WASM and limits drift
                // from BLAS reduction ordering. Layers retain source float32 precision.
                let sum = weights.iter().zip(&previous)
                    .fold(*bias as f64, |sum, (w, x)| sum + *w as f64 * *x as f64) as f32;
                next.push(if index < 2 { sum.max(0.0) } else { sum });
            }
            previous = next;
        }
        if previous.iter().any(|x| !x.is_finite()) { return Err("Nonfinite GetUp inference".into()); }
        Ok(Prediction {action: std::array::from_fn(|i| previous[i].tanh()*input[self.input-1]), raw: previous})
    }
}
pub fn teacher() -> &'static Actor {
    static VALUE: OnceLock<Actor> = OnceLock::new();
    VALUE.get_or_init(|| Actor::decode(include_bytes!("../../data/getup-policy/teacher.bin"), 68).expect("pinned teacher"))
}
pub fn student() -> &'static Actor {
    static VALUE: OnceLock<Actor> = OnceLock::new();
    VALUE.get_or_init(|| Actor::decode(include_bytes!("../../data/getup-policy/student.bin"), 118).expect("pinned student"))
}
pub fn infer(name: &str, input: &[f32]) -> Result<Prediction, String> {
    match name { "teacher" => teacher().infer(input), "student" => student().infer(input),
        _ => Err("Unknown GetUp actor".into()) }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn fixed_observations_match_pinned_pytorch_actor_without_physics() {
        let data: serde_json::Value = serde_json::from_str(include_str!("../../data/getup-policy/parity.json")).unwrap();
        let mut maximum_action = 0.0_f32; let mut maximum_raw = 0.0_f32;
        for fixture in data["actors"].as_array().unwrap() {
            let input: Vec<f32> = serde_json::from_value(fixture["input"].clone()).unwrap();
            let output = infer(fixture["actor"].as_str().unwrap(), &input).unwrap();
            for (a, b) in output.raw.iter().zip(fixture["raw"].as_array().unwrap()) {
                maximum_raw = maximum_raw.max((*a - b.as_f64().unwrap() as f32).abs());
            }
            for (a, b) in output.action.iter().zip(fixture["action"].as_array().unwrap()) {
                maximum_action = maximum_action.max((*a - b.as_f64().unwrap() as f32).abs());
            }
        }
        println!("actor parity: max action={maximum_action}, raw={maximum_raw}; 32 fixed observations, zero physics");
        assert!(maximum_action <= data["actionTolerance"].as_f64().unwrap() as f32);
        assert!(maximum_raw <= data["rawTolerance"].as_f64().unwrap() as f32);
        assert!(student().infer(&[0.0; 67]).is_err());
        let mut invalid = [0.0; 118]; invalid[4] = f32::NAN;
        assert!(student().infer(&invalid).is_err());
        assert!(Actor::decode(&[], 118).is_err());
    }
    #[test]
    fn source_pd_loop_and_motor_application_match_without_physics() {
        let data: serde_json::Value = serde_json::from_str(include_str!("../../data/getup-policy/parity.json")).unwrap();
        let model = lh_model::Model::canonical().unwrap();
        let mut maximum = 0.0_f32;
        for f in data["pd"].as_array().unwrap() {
            let read = |key: &str| -> [f32;21] { serde_json::from_value(f[key].clone()).unwrap() };
            let actual = crate::getup::slow_controls(&model,&read("reference"),&read("residual"),&read("q"),&read("dq"),f["holding"].as_bool().unwrap()).unwrap();
            for (a,b) in actual.into_iter().zip(read("controls")) { maximum = maximum.max((a-b).abs()); }
        }
        assert!(maximum < 1e-6);
        let mut s = crate::Simulation::new(0.2,crate::Profile::default(),true).unwrap();
        for h in &s.bodies { s.world.bodies[*h].reset_torques(false); }
        crate::getup::apply_controls(&mut s,&[1.0;21]).unwrap();
        let sum = s.bodies.iter().map(|h| s.world.bodies[*h].user_torque()).sum::<crate::Vector>();
        assert!(sum.length() < 1e-4);
        assert_eq!(s.diagnostics.requested_motor_effort_nm,920.0);
        assert_eq!(s.substeps_completed,0);
        println!("32 upstream PD-loop fixtures, maximum normalized control error {maximum}; torque sum conserved, zero physics");
    }
}
