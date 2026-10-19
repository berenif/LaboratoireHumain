//! One default-off right-flank preparation proposal. Only ordinary joint motor
//! references are returned here. Geometry, current generalized twists and solved
//! floor loads decide progress; elapsed time never establishes a contact mode.
use crate::*;

pub const CAP_NATIVE: u64 = 4000;
pub const QUIET_NATIVE: u32 = 120;
const REFERENCE_NATIVE: u64 = 800;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct Observation {
    pub native: u64,
    pub long_up: f32,
    pub face_up: f32,
    pub side_up: f32,
    pub head_m: f32,
    pub load_n: f32,
    pub right_flank_load_n: f32,
    pub maximum_linear_mps: f32,
    pub maximum_angular_radps: f32,
}

pub fn observation(s: &Simulation) -> Result<Observation, String> {
    let torso = s.model.segments.iter().position(|p| p.id == "torso").ok_or("Missing torso")?;
    let head = s.model.segments.iter().position(|p| p.id == "head").ok_or("Missing head")?;
    let r = s.world.bodies[s.bodies[torso]].rotation().normalize();
    let twists = super::current_twists(s)?;
    let o = Observation {
        native: s.substeps_completed,
        long_up: (r * Vector::Y).y,
        face_up: (r * Vector::Z).y,
        side_up: (r * Vector::X).y,
        head_m: s.world.bodies[s.bodies[head]].translation().y,
        load_n: s.integrated_support_loads_n.iter().sum(),
        right_flank_load_n: s.model.segments.iter().enumerate()
            .filter(|(_, p)| ["torso", "pelvis", "right_upper_arm", "right_lower_arm", "right_hand"].contains(&p.id.as_str()))
            .map(|(i, _)| s.integrated_support_loads_n[i]).sum(),
        maximum_linear_mps: twists.iter().map(|(v, _)| v.length()).fold(0.0, f32::max),
        maximum_angular_radps: twists.iter().map(|(_, w)| w.length()).fold(0.0, f32::max),
    };
    if [o.long_up, o.face_up, o.side_up, o.head_m, o.load_n, o.right_flank_load_n,
        o.maximum_linear_mps, o.maximum_angular_radps].iter().any(|x| !x.is_finite()) {
        return Err("Nonfinite physical roll observation".into());
    }
    Ok(o)
}

impl Observation {
    fn grounded(&self, weight: f32) -> bool {
        self.head_m <= 0.5 && self.load_n >= 0.8 * weight
    }
    pub fn settled_supine(&self, weight: f32) -> bool {
        self.long_up.abs() <= 0.35 && self.face_up >= 0.8 && self.grounded(weight)
            && self.maximum_linear_mps <= 0.22 && self.maximum_angular_radps <= 0.65
    }
    fn right_lateral(&self, weight: f32) -> bool {
        self.long_up.abs() <= 0.35 && self.face_up.abs() <= 0.65 && self.side_up >= 0.65
            && self.grounded(weight) && self.right_flank_load_n >= 3.0
    }
    fn prone(&self, weight: f32) -> bool {
        self.long_up.abs() <= 0.35 && self.face_up <= -0.8 && self.grounded(weight)
    }
    fn settled_prone(&self, weight: f32) -> bool {
        self.prone(weight) && self.maximum_linear_mps <= 0.22 && self.maximum_angular_radps <= 0.65
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Phase { TuckRollRight, LateralUncoil, PassiveProneSettle, Admitted, Rejected }

#[derive(Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct State {
    pub route: String,
    pub start_native: u64,
    pub cap_native: u64,
    pub phase: Phase,
    pub phase_start_native: u64,
    pub lateral_native: Option<u64>,
    pub prone_native: Option<u64>,
    pub admission_native: Option<u64>,
    pub rejection_native: Option<u64>,
    pub rejection: Option<String>,
    pub lateral_samples: u32,
    pub quiet_prone_samples: u32,
    pub maximum_quiet_prone_samples: u32,
    pub last_observed_native: u64,
    pub current: Observation,
    reference_start_q: [f32; 21],
    reference_start_dq: [f32; 21],
}

// Frozen proposal, not a copied fixture endpoint. Shoulder values come from
// limit-constrained exact-chain FK: right hand tucks over the chest, left hand
// reaches across it. The left flexed leg shifts mass toward the selected flank.
// All endpoints remain inside the original anatomical stops.
pub const TUCK: [f32; 21] = [0.72, -0.35, 0.0,
    -0.08, 0.05, -0.30, -0.60, 0.0, 0.0,
    0.06, 0.25, -1.25, -1.75, 0.0, 0.0,
    0.609, 0.242, 0.130, -1.027, -0.483, -0.213];
pub const UNCOIL: [f32; 21] = [-0.60, -0.25, 0.0,
    -0.08, 0.05, -0.35, -0.65, 0.0, 0.0,
    0.06, 0.10, -0.35, -0.65, 0.0, 0.0,
    0.609, 0.242, 0.130, -0.609, -0.242, 0.130];

/// Quintic Hermite reference: coherent q/dq, zero endpoint acceleration, zero
/// final velocity. The starting q/dq are reference values at the phase boundary.
fn interpolate(q0: f32, v0: f32, q1: f32, seconds: f32) -> (f32, f32) {
    let duration = REFERENCE_NATIVE as f32 / 800.0;
    let t = (seconds / duration).clamp(0.0, 1.0);
    if t >= 1.0 { return (q1, 0.0); }
    let d = q1 - q0;
    let b = v0 * duration;
    let a3 = 10.0 * d - 6.0 * b;
    let a4 = -15.0 * d + 8.0 * b;
    let a5 = 6.0 * d - 3.0 * b;
    (q0 + b*t + a3*t.powi(3) + a4*t.powi(4) + a5*t.powi(5),
        (b + 3.0*a3*t.powi(2) + 4.0*a4*t.powi(3) + 5.0*a5*t.powi(4)) / duration)
}

impl State {
    pub fn new(s: &Simulation) -> Result<Self, String> {
        let current = observation(s)?;
        if !current.settled_supine(s.total_mass() * 9.81) {
            return Err("Physical roll requires observed grounded quiet supine anatomy".into());
        }
        let (q, dq) = super::joint_state(s)?;
        Ok(Self { route: "right-flank-tuck-uncoil-v1".into(), start_native: s.substeps_completed,
            cap_native: CAP_NATIVE, phase: Phase::TuckRollRight, phase_start_native: s.substeps_completed,
            lateral_native: None, prone_native: None, admission_native: None, rejection_native: None,
            rejection: None, lateral_samples: 0, quiet_prone_samples: 0, maximum_quiet_prone_samples: 0,
            last_observed_native: s.substeps_completed, current,
            reference_start_q: q, reference_start_dq: dq })
    }
    fn reference(&self, native: u64) -> Result<([f32; 21], [f32; 21]), String> {
        let elapsed = native.checked_sub(self.phase_start_native).ok_or("Roll reference clock is ahead")?;
        let end = if self.phase == Phase::TuckRollRight { TUCK } else { UNCOIL };
        let mut q = [0.0; 21]; let mut dq = [0.0; 21];
        for i in 0..21 {
            (q[i], dq[i]) = interpolate(self.reference_start_q[i], self.reference_start_dq[i], end[i], elapsed as f32 / 800.0);
        }
        Ok((q, dq))
    }
    pub fn command(&self, s: &Simulation) -> Result<[f32; 21], String> {
        if self.phase == Phase::Rejected { return Err(self.rejection.clone().unwrap_or("Physical roll rejected".into())); }
        if matches!(self.phase, Phase::PassiveProneSettle | Phase::Admitted) { return Ok([0.0; 21]); }
        let (reference, velocity) = self.reference(s.substeps_completed)?;
        let (q, dq) = super::joint_state(s)?;
        let mut controls = [0.0; 21];
        for part in &s.model.segments {
            if let Some(h) = part.reference.as_ref().and_then(|r| r.hinge.as_ref()) {
                let i = h.qpos_index;
                let axis = &part.joint_profile.as_ref().unwrap().axes[0];
                // Feed forward only the known hinge passive spring/damper;
                // every contact reaction remains a native solver outcome.
                let feedforward = (axis.passive_stiffness_nm_per_rad * reference[i]
                    + axis.damping_nms_per_rad * velocity[i]) / h.gear;
                controls[h.actuator_index] = (feedforward + 1.2*(reference[i]-q[i])
                    + 0.18*(velocity[i]-dq[i])).clamp(-0.85, 0.85);
            }
        }
        Ok(controls)
    }
    /// Called once after each real native integration. No forecast invokes it.
    pub fn observe(&mut self, s: &Simulation) -> Result<(), String> {
        self.observe_sample(observation(s)?, s.total_mass() * 9.81)
    }
    fn observe_sample(&mut self, current: Observation, weight: f32) -> Result<(), String> {
        if matches!(self.phase, Phase::Admitted | Phase::Rejected) { return Ok(()); }
        if current.native != self.last_observed_native + 1 {
            return Err("Physical roll observation must cover every native sample exactly once".into());
        }
        let native = current.native;
        self.last_observed_native = native;
        self.current = current;
        if self.phase == Phase::TuckRollRight {
            self.lateral_samples = if self.current.right_lateral(weight) { self.lateral_samples + 1 } else { 0 };
            if self.lateral_samples >= 20 {
                (self.reference_start_q, self.reference_start_dq) = self.reference(native)?;
                self.phase = Phase::LateralUncoil;
                self.phase_start_native = native;
                self.lateral_native = Some(native);
            }
        }
        let entered_prone_settle = self.phase == Phase::LateralUncoil && self.current.prone(weight);
        if entered_prone_settle {
            self.phase = Phase::PassiveProneSettle;
            self.phase_start_native = native;
            self.prone_native = Some(native);
        }
        if self.phase == Phase::PassiveProneSettle && !entered_prone_settle {
            self.quiet_prone_samples = if self.current.settled_prone(weight) { self.quiet_prone_samples + 1 } else { 0 };
            self.maximum_quiet_prone_samples = self.maximum_quiet_prone_samples.max(self.quiet_prone_samples);
            if self.quiet_prone_samples >= QUIET_NATIVE {
                self.phase = Phase::Admitted;
                self.admission_native = Some(native);
            }
        }
        if native - self.start_native >= CAP_NATIVE && self.phase != Phase::Admitted {
            self.phase = Phase::Rejected;
            self.rejection_native = Some(native);
            self.rejection = Some("Physical roll did not earn grounded quiet prone admission within the fixed5-second cap; no retry".into());
        }
        Ok(())
    }
    pub fn valid_at(&self, native: u64) -> bool {
        self.route == "right-flank-tuck-uncoil-v1" && self.cap_native == CAP_NATIVE
            && self.start_native <= self.phase_start_native && self.phase_start_native <= self.last_observed_native
            && self.last_observed_native <= native && self.current.native == self.last_observed_native
            && self.quiet_prone_samples <= QUIET_NATIVE && self.maximum_quiet_prone_samples <= QUIET_NATIVE
            && self.quiet_prone_samples <= self.maximum_quiet_prone_samples
            && (matches!(self.phase, Phase::Admitted | Phase::Rejected)
                || self.last_observed_native == native && self.last_observed_native < self.start_native + CAP_NATIVE)
            && self.reference_start_q.iter().chain(&self.reference_start_dq).all(|x| x.is_finite())
            && self.lateral_native.is_none_or(|n| n >= self.start_native && n <= self.last_observed_native)
            && self.prone_native.is_none_or(|n| self.lateral_native.is_some_and(|l| n >= l) && n <= self.last_observed_native)
            && self.admission_native.is_none_or(|n| self.prone_native.is_some_and(|p| n >= p)
                && n <= self.start_native + CAP_NATIVE && n <= self.last_observed_native)
            && (self.phase != Phase::Admitted || self.admission_native.is_some() && self.quiet_prone_samples == QUIET_NATIVE)
            && (self.phase != Phase::Rejected || self.rejection_native.is_some_and(|n| n >= self.start_native && n <= native) && self.rejection.is_some())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn synthetic(phase: Phase) -> State {
        State { route: "right-flank-tuck-uncoil-v1".into(), start_native: 480, cap_native: CAP_NATIVE,
            phase, phase_start_native: 480, lateral_native: None, prone_native: None, admission_native: None,
            rejection_native: None, rejection: None, lateral_samples: 0, quiet_prone_samples: 0,
            maximum_quiet_prone_samples: 0, last_observed_native: 480,
            current: Observation { native: 480, long_up: 0.0, face_up: 1.0, side_up: 0.0, head_m: 0.1,
                load_n: 400.0, right_flank_load_n: 20.0, maximum_linear_mps: 0.0, maximum_angular_radps: 0.0 },
            reference_start_q: [0.0;21], reference_start_dq: [0.0;21] }
    }
    #[test]
    fn frozen_roll_endpoints_respect_all_original_anatomical_limits() {
        let s = Simulation::new(0.0, Profile::default(), false).unwrap();
        for part in &s.model.segments {
            if let Some(h) = part.reference.as_ref().and_then(|r| r.hinge.as_ref()) {
                let a = &part.joint_profile.as_ref().unwrap().axes[0];
                for target in [TUCK, UNCOIL] { assert!(target[h.qpos_index] > a.min_radians && target[h.qpos_index] < a.max_radians, "{}", part.id); }
            }
        }
    }
    #[test]
    fn reference_velocity_is_derivative_and_boundary_is_coherent() {
        for t in [0.05, 0.25, 0.5, 0.75, 0.95] {
            let (_, v) = interpolate(0.3, -0.02, -0.7, t);
            let numerical = (interpolate(0.3, -0.02, -0.7, t+0.0001).0 - interpolate(0.3, -0.02, -0.7, t-0.0001).0) / 0.0002;
            assert!((v-numerical).abs() < 0.006);
        }
        assert_eq!(interpolate(0.3, -0.02, -0.7, 0.0), (0.3, -0.02));
        assert_eq!(interpolate(0.3, -0.02, -0.7, 1.0), (-0.7, 0.0));
    }
    #[test]
    fn prone_requires_original_belly_motion_load_and_orientation_gates() {
        let mut o = Observation { native: 1, long_up: 0.0, face_up: -1.0, side_up: 0.0, head_m: 0.1,
            load_n: 400.0, right_flank_load_n: 20.0, maximum_linear_mps: 0.22, maximum_angular_radps: 0.65 };
        assert!(o.settled_prone(400.0));
        o.maximum_linear_mps = 0.2201; assert!(!o.settled_prone(400.0));
        o.maximum_linear_mps = 0.0; o.maximum_angular_radps = 0.6501; assert!(!o.settled_prone(400.0));
        o.maximum_angular_radps = 0.0; o.load_n = 319.99; assert!(!o.settled_prone(400.0));
        o.load_n = 400.0; o.face_up = -0.799; assert!(!o.settled_prone(400.0));
    }
    #[test]
    fn time_never_substitutes_for_lateral_or_prone_contact_observations() {
        let mut r = synthetic(Phase::TuckRollRight);
        for native in 481..=4480 {
            let mut o = r.current.clone(); o.native = native;
            r.observe_sample(o, 400.0).unwrap();
        }
        assert_eq!(r.phase, Phase::Rejected); assert_eq!(r.rejection_native, Some(4480));
        assert_eq!(r.lateral_native, None); assert_eq!(r.admission_native, None);
        assert!(r.valid_at(4480));
    }
    #[test]
    fn lateral_then_prone_requires120_consecutive_current_native_samples() {
        let mut r = synthetic(Phase::TuckRollRight);
        for native in 481..=500 {
            let mut o = r.current.clone(); o.native = native; o.face_up = 0.0; o.side_up = 1.0;
            r.observe_sample(o, 400.0).unwrap();
        }
        assert_eq!(r.lateral_native, Some(500)); assert_eq!(r.phase, Phase::LateralUncoil);
        for native in 501..=620 {
            let mut o = r.current.clone(); o.native = native; o.face_up = -1.0; o.side_up = 0.0;
            if native == 600 { o.maximum_linear_mps = 0.221; } else { o.maximum_linear_mps = 0.0; }
            r.observe_sample(o, 400.0).unwrap();
        }
        assert_eq!(r.phase, Phase::PassiveProneSettle); assert_eq!(r.quiet_prone_samples, 20);
        for native in 621..=720 {
            let mut o = r.current.clone(); o.native = native;
            r.observe_sample(o, 400.0).unwrap();
        }
        assert_eq!(r.phase, Phase::Admitted); assert_eq!(r.admission_native, Some(720));
        assert!(r.valid_at(720));
    }
    #[test]
    fn skipped_or_duplicate_sample_is_refused() {
        let mut r = synthetic(Phase::TuckRollRight);
        let mut o = r.current.clone(); assert!(r.observe_sample(o.clone(), 400.0).is_err());
        o.native += 2; assert!(r.observe_sample(o, 400.0).is_err());
        assert_eq!(r.last_observed_native, 480);
    }
}
