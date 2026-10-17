// Pure 46-coordinate command-rate problem. No dynamics or torque inversion.
// Hard task slabs are constraints, never weights in the posture objective.
// Bounded projected gradient/Dykstra; a failed numerical check returns no intent.

pub(crate) const AXES: usize = 46;
pub(crate) type Rates = [f64; AXES];
const FEASIBILITY: f64 = 1e-8;
const PROJECTION_CYCLES: usize = 512;
const SOLVE_ITERATIONS: usize = 512;

#[derive(Clone, Debug)]
pub(crate) struct Task {
    pub row: Rates,
    pub target: f64,
    pub weight: f64,
}

#[derive(Clone, Debug)]
pub(crate) struct HardTask {
    pub name: String,
    pub row: Rates,
    pub lower: f64,
    pub upper: f64,
}

#[derive(Clone, Debug)]
pub(crate) struct Problem {
    pub lower: Rates,
    pub upper: Rates,
    pub hard: Vec<HardTask>,
    pub soft: Vec<Task>,
    pub regularization: f64,
}

#[derive(Debug)]
pub(crate) struct Solution {
    pub rates: Rates,
    pub maximum_constraint_error: f64,
    pub projected_gradient_residual: f64,
    pub iterations: usize,
}

fn dot(a: &Rates, b: &Rates) -> f64 {
    a.iter().zip(b).map(|(a, b)| a * b).sum()
}
fn max_difference(a: &Rates, b: &Rates) -> f64 {
    a.iter()
        .zip(b)
        .map(|(a, b)| (a - b).abs())
        .fold(0., f64::max)
}

impl Problem {
    pub(crate) fn validate(&self) -> Result<(), String> {
        if self.hard.len() > 256
            || self.soft.len() > 256
            || !self.regularization.is_finite()
            || self.regularization <= 0.
        {
            return Err("Invalid bounded task dimensions/regularization".into());
        }
        for i in 0..AXES {
            if !self.lower[i].is_finite()
                || !self.upper[i].is_finite()
                || self.lower[i] > self.upper[i]
                || self.lower[i] < -1.
                || self.upper[i] > 1.
            {
                return Err(format!("Invalid total command-rate bounds at axis {i}"));
            }
        }
        for t in &self.hard {
            if t.row.iter().any(|v| !v.is_finite())
                || !t.lower.is_finite()
                || !t.upper.is_finite()
                || t.lower > t.upper
                || t.name.is_empty()
            {
                return Err("Invalid hard support task".into());
            }
        }
        for t in &self.soft {
            if t.row.iter().any(|v| !v.is_finite())
                || !t.target.is_finite()
                || !t.weight.is_finite()
                || t.weight < 0.
            {
                return Err("Invalid soft task".into());
            }
        }
        Ok(())
    }

    fn violation(&self, x: &Rates) -> f64 {
        let mut error: f64 = 0.;
        for i in 0..AXES {
            error = error.max(self.lower[i] - x[i]).max(x[i] - self.upper[i]);
        }
        for t in &self.hard {
            let v = dot(&t.row, x);
            error = error.max(t.lower - v).max(v - t.upper);
        }
        error
    }

    pub(crate) fn verify_rates(&self, rates: &Rates) -> Result<(), String> {
        self.validate()?;
        if rates.iter().any(|v| !v.is_finite()) || self.violation(rates) > FEASIBILITY {
            return Err("Emitted rates violate a hard task or total command bound".into());
        }
        Ok(())
    }

    // Projection onto intersection of box and hard slabs. Dykstra corrections
    // prevent repeated projections from accepting a feasible non-projection.
    fn project(&self, input: Rates) -> Result<Rates, String> {
        if input.iter().any(|v| !v.is_finite()) {
            return Err("Nonfinite projection input".into());
        }
        let mut x = input;
        let mut corrections = vec![[0.; AXES]; self.hard.len() + 1];
        for _ in 0..PROJECTION_CYCLES {
            let previous = x;
            let mut maximum_update: f64 = 0.;
            for (set, correction) in corrections.iter_mut().enumerate() {
                let old_correction = *correction;
                let mut y = x;
                for i in 0..AXES {
                    y[i] += correction[i];
                }
                if y.iter().any(|v| !v.is_finite()) {
                    return Err("Nonfinite projection intermediate".into());
                }
                let mut projected = y;
                if set == 0 {
                    for i in 0..AXES {
                        projected[i] = y[i].clamp(self.lower[i], self.upper[i]);
                    }
                } else {
                    let t = &self.hard[set - 1];
                    let norm = dot(&t.row, &t.row);
                    let value = dot(&t.row, &y);
                    if !norm.is_finite() || !value.is_finite() {
                        return Err("Nonfinite hard task evaluation".into());
                    }
                    let delta = value.clamp(t.lower, t.upper) - value;
                    if !delta.is_finite() {
                        return Err("Nonfinite hard task projection".into());
                    }
                    if norm < 1e-24 {
                        if delta.abs() > FEASIBILITY {
                            return Err(format!("Unrealizable hard task: {}", t.name));
                        }
                    } else {
                        for i in 0..AXES {
                            projected[i] += delta * t.row[i] / norm;
                        }
                    }
                }
                for i in 0..AXES {
                    correction[i] = y[i] - projected[i];
                }
                if correction
                    .iter()
                    .chain(projected.iter())
                    .any(|v| !v.is_finite())
                {
                    return Err("Nonfinite Dykstra correction".into());
                }
                maximum_update = maximum_update.max(max_difference(correction, &old_correction));
                x = projected;
            }
            if x.iter().any(|v| !v.is_finite()) {
                return Err("Nonfinite projection".into());
            }
            if max_difference(&x, &previous) <= FEASIBILITY
                && maximum_update <= FEASIBILITY
                && self.violation(&x) <= FEASIBILITY
            {
                return Ok(x);
            }
        }
        Err("Hard task projection infeasible or not converged within fixed budget".into())
    }

    pub(crate) fn solve(&self) -> Result<Solution, String> {
        self.validate()?;
        // Frobenius bound on Hessian spectral norm; never assumes diagonal tasks.
        let lipschitz = self.regularization
            + self
                .soft
                .iter()
                .map(|t| t.weight * dot(&t.row, &t.row))
                .sum::<f64>();
        if !lipschitz.is_finite() || lipschitz <= 0. {
            return Err("Nonfinite task curvature".into());
        }
        let mut x = self.project([0.; AXES])?;
        let mut last_residual = f64::INFINITY;
        for iteration in 1..=SOLVE_ITERATIONS {
            let mut gradient = x.map(|v| v * self.regularization);
            for t in &self.soft {
                let residual = t.weight * (dot(&t.row, &x) - t.target);
                for (g, a) in gradient.iter_mut().zip(t.row) {
                    *g += a * residual;
                }
            }
            if gradient.iter().any(|v| !v.is_finite()) {
                return Err("Nonfinite common task gradient".into());
            }
            let mut input = x;
            for i in 0..AXES {
                input[i] -= gradient[i] / lipschitz;
            }
            let next = self.project(input)?;
            let residual = lipschitz * max_difference(&x, &next);
            last_residual = residual;
            if !residual.is_finite() {
                return Err("Nonfinite common solve residual".into());
            }
            x = next;
            if residual <= 1e-6 {
                let violation = self.violation(&x);
                if violation > FEASIBILITY {
                    return Err("Hard task residual exceeds tolerance".into());
                }
                return Ok(Solution {
                    rates: x,
                    maximum_constraint_error: violation,
                    projected_gradient_residual: residual,
                    iterations: iteration,
                });
            }
        }
        Err(format!("Common task solve did not converge within fixed iteration budget: iterations={SOLVE_ITERATIONS}, projected_gradient_residual={last_residual:.17e}, lipschitz={lipschitz:.17e}"))
    }
}
