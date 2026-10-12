//! Pure 46-coordinate command-rate problem. No dynamics or torque inversion.
//! Hard task slabs are constraints, never weights in the posture objective.
//! Feasible active-set Newton, certified by the original projected-gradient test.
//! Dykstra projections and active-set pivots have separate fixed iteration caps.

pub(crate) const AXES: usize = 46;
pub(crate) type Rates = [f64; AXES];
const FEASIBILITY: f64 = 1e-8;
const CONVERGENCE: f64 = 1e-6;
const PROJECTION_CYCLES: usize = 512;
const SOLVE_ITERATIONS: usize = 512;
// Rank/direction tests choose a numerical search direction only. Acceptance
// always uses the unchanged original feasibility and projected-gradient tests.
const RANK_EPSILON: f64 = 128. * f64::EPSILON * AXES as f64;
type Matrix = [[f64; AXES]; AXES];

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

#[derive(Clone, Debug)]
struct Boundary {
    // Unit-normal constraints. Every original row stays in Problem and is
    // checked before acceptance; normalizing never changes its tolerance.
    row: Rates,
    bound: f64,
    equality: bool,
    name: String,
}

fn orthogonal_remainder(mut vector: Rates, basis: &[Rates]) -> Rates {
    // Reorthogonalization avoids spurious rank from duplicate support rows.
    for _ in 0..2 {
        for q in basis {
            let coefficient = dot(&vector, q);
            for i in 0..AXES {
                vector[i] -= coefficient * q[i];
            }
        }
    }
    vector
}

/// QR of independent active normals; A = T Q with lower-triangular T.
/// Equality multipliers are unrestricted, inequality multipliers are >= 0.
fn active_basis(
    boundaries: &[Boundary],
    working: &[usize],
) -> Result<(Vec<Rates>, Vec<Vec<f64>>), String> {
    let mut basis = Vec::new();
    let mut triangular = Vec::new();
    for &index in working {
        let row = boundaries[index].row;
        let remainder = orthogonal_remainder(row, &basis);
        let norm = dot(&remainder, &remainder).sqrt();
        if !norm.is_finite() || norm <= RANK_EPSILON {
            return Err(format!(
                "Dependent active normal: constraint={} rank={} remainder={norm:.17e}",
                boundaries[index].name,
                basis.len()
            ));
        }
        let mut coefficients: Vec<_> = basis.iter().map(|q| dot(&row, q)).collect();
        coefficients.push(norm);
        triangular.push(coefficients);
        basis.push(remainder.map(|v| v / norm));
    }
    Ok((basis, triangular))
}

fn independent(row: Rates, basis: &[Rates]) -> bool {
    let remainder = orthogonal_remainder(row, basis);
    dot(&remainder, &remainder).sqrt() > RANK_EPSILON
}

fn null_basis(active: &[Rates]) -> Result<Vec<Rates>, String> {
    let mut complete = active.to_vec();
    let mut null = Vec::new();
    for axis in 0..AXES {
        if complete.len() == AXES {
            break;
        }
        let mut unit = [0.; AXES];
        unit[axis] = 1.;
        let remainder = orthogonal_remainder(unit, &complete);
        let norm = dot(&remainder, &remainder).sqrt();
        if norm > RANK_EPSILON {
            let q = remainder.map(|v| v / norm);
            complete.push(q);
            null.push(q);
        }
    }
    if complete.len() != AXES {
        return Err(format!(
            "Incomplete active null space: rank={} null={}",
            active.len(),
            null.len()
        ));
    }
    Ok(null)
}

/// Cholesky on the reduced SPD Hessian. No ridge, pivot relaxation or objective
/// change is permitted; failure returns diagnostics rather than a command.
fn positive_definite_solve(matrix: &[Vec<f64>], rhs: &[f64]) -> Result<Vec<f64>, String> {
    let size = rhs.len();
    let mut factor = vec![vec![0.; size]; size];
    for i in 0..size {
        for j in 0..=i {
            let value = matrix[i][j] - (0..j).map(|k| factor[i][k] * factor[j][k]).sum::<f64>();
            if !value.is_finite() || (i == j && value <= 0.) {
                return Err(format!(
                    "Reduced Hessian Cholesky refused: row={i} column={j} pivot={value:.17e}"
                ));
            }
            factor[i][j] = if i == j {
                value.sqrt()
            } else {
                value / factor[j][j]
            };
        }
    }
    let mut answer = rhs.to_vec();
    for i in 0..size {
        answer[i] =
            (answer[i] - (0..i).map(|j| factor[i][j] * answer[j]).sum::<f64>()) / factor[i][i];
    }
    for i in (0..size).rev() {
        answer[i] = (answer[i] - (i + 1..size).map(|j| factor[j][i] * answer[j]).sum::<f64>())
            / factor[i][i];
    }
    if answer.iter().any(|v| !v.is_finite()) {
        return Err("Nonfinite reduced Newton solution".into());
    }
    Ok(answer)
}

fn newton_direction(
    hessian: &Matrix,
    gradient: &Rates,
    basis: &[Rates],
    correction: &Rates,
) -> Result<Rates, String> {
    let null = null_basis(basis)?;
    let transformed: Vec<Rates> = null
        .iter()
        .map(|q| std::array::from_fn(|i| dot(&hessian[i], q)))
        .collect();
    let matrix: Vec<Vec<f64>> = null
        .iter()
        .map(|q| transformed.iter().map(|hq| dot(q, hq)).collect())
        .collect();
    let corrected_gradient = std::array::from_fn(|i| gradient[i] + dot(&hessian[i], correction));
    let rhs: Vec<f64> = null.iter().map(|q| -dot(q, &corrected_gradient)).collect();
    let coefficients = positive_definite_solve(&matrix, &rhs)?;
    Ok(std::array::from_fn(|i| {
        correction[i]
            + null
                .iter()
                .zip(&coefficients)
                .map(|(q, a)| q[i] * a)
                .sum::<f64>()
    }))
}

/// Initial projection is only feasible to FEASIBILITY. Newton must restore
/// A_W (x+p)=b_W, not preserve an arbitrary initial equality residual via
/// A_W p=0; the latter can fail the unchanged optimality certificate at large H.
fn active_correction(
    boundaries: &[Boundary],
    working: &[usize],
    basis: &[Rates],
    triangular: &[Vec<f64>],
    x: &Rates,
) -> Result<Rates, String> {
    let mut coordinates = Vec::new();
    for (i, &index) in working.iter().enumerate() {
        let rhs = boundaries[index].bound - dot(&boundaries[index].row, x);
        let value = (rhs
            - (0..i)
                .map(|j| triangular[i][j] * coordinates[j])
                .sum::<f64>())
            / triangular[i][i];
        if !value.is_finite() {
            return Err(format!(
                "Nonfinite active affine correction: constraint={:?}",
                boundaries[index].name
            ));
        }
        coordinates.push(value);
    }
    Ok(std::array::from_fn(|i| {
        basis.iter().zip(&coordinates).map(|(q, a)| q[i] * a).sum()
    }))
}

fn active_multipliers(
    gradient: &Rates,
    basis: &[Rates],
    triangular: &[Vec<f64>],
) -> Result<Vec<f64>, String> {
    let mut multipliers: Vec<f64> = basis.iter().map(|q| -dot(q, gradient)).collect();
    for i in (0..basis.len()).rev() {
        multipliers[i] = (multipliers[i]
            - (i + 1..basis.len())
                .map(|j| triangular[j][i] * multipliers[j])
                .sum::<f64>())
            / triangular[i][i];
    }
    if multipliers.iter().any(|v| !v.is_finite()) {
        return Err("Nonfinite active multipliers".into());
    }
    Ok(multipliers)
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
        if x.iter().any(|v| !v.is_finite()) {
            return f64::INFINITY;
        }
        let mut error: f64 = 0.;
        for i in 0..AXES {
            error = error.max(self.lower[i] - x[i]).max(x[i] - self.upper[i]);
        }
        for t in &self.hard {
            let v = dot(&t.row, x);
            if !v.is_finite() {
                return f64::INFINITY;
            }
            error = error.max(t.lower - v).max(v - t.upper);
        }
        error
    }

    fn violation_detail(&self, x: &Rates) -> String {
        let mut worst = (0., "none".to_owned());
        for i in 0..AXES {
            for (side, error, bound) in [
                ("lower", self.lower[i] - x[i], self.lower[i]),
                ("upper", x[i] - self.upper[i], self.upper[i]),
            ] {
                if error > worst.0 {
                    worst = (
                        error,
                        format!(
                            "axis={i} side={side} value={:.17e} bound={bound:.17e}",
                            x[i]
                        ),
                    );
                }
            }
        }
        for (index, task) in self.hard.iter().enumerate() {
            let value = dot(&task.row, x);
            if !value.is_finite() {
                return format!(
                    "nonfinite hard_index={index} name={:?} value={value:?}",
                    task.name
                );
            }
            for (side, error, bound) in [
                ("lower", task.lower - value, task.lower),
                ("upper", value - task.upper, task.upper),
            ] {
                if error > worst.0 {
                    worst = (
                        error,
                        format!(
                            "hard_index={index} name={:?} side={side} value={value:.17e} bound={bound:.17e}",
                            task.name
                        ),
                    );
                }
            }
        }
        format!(
            "maximum_violation={:.17e} tolerance={FEASIBILITY:.17e} {}",
            worst.0, worst.1
        )
    }

    pub(crate) fn verify_rates(&self, rates: &Rates) -> Result<(), String> {
        self.validate()?;
        if let Some(axis) = rates.iter().position(|v| !v.is_finite()) {
            return Err(format!(
                "Emitted rates violate a hard task or total command bound: nonfinite axis={axis} value={:?}",
                rates[axis]
            ));
        }
        if self.violation(rates) > FEASIBILITY {
            return Err(format!(
                "Emitted rates violate a hard task or total command bound: {}",
                self.violation_detail(rates)
            ));
        }
        Ok(())
    }

    // Projection onto intersection of box and hard slabs. Dykstra corrections
    // prevent repeated projections from accepting a feasible non-projection.
    fn project(&self, input: Rates) -> Result<Rates, String> {
        self.project_with_precision(input, FEASIBILITY)
    }

    fn project_with_precision(&self, input: Rates, precision: f64) -> Result<Rates, String> {
        if !precision.is_finite() || precision <= 0. || precision > FEASIBILITY {
            return Err(format!("Invalid projection precision: {precision:.17e}"));
        }
        if input.iter().any(|v| !v.is_finite()) {
            return Err("Nonfinite projection input".into());
        }
        let mut x = input;
        let mut corrections = vec![[0.; AXES]; self.hard.len() + 1];
        let mut last_movement = f64::INFINITY;
        let mut last_correction_update = f64::INFINITY;
        for cycle in 0..PROJECTION_CYCLES {
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
                            return Err(format!(
                                "Unrealizable hard task: name={:?} cycle={cycle} squared_norm={norm:.17e} value={value:.17e} required_delta={delta:.17e}",
                                t.name
                            ));
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
            last_movement = max_difference(&x, &previous);
            last_correction_update = maximum_update;
            if last_movement <= precision
                && maximum_update <= precision
                && self.violation(&x) <= precision
            {
                return Ok(x);
            }
        }
        Err(format!(
            "Hard task projection infeasible or not converged within fixed budget: cycles={PROJECTION_CYCLES} precision={precision:.17e} movement={last_movement:.17e} correction_update={last_correction_update:.17e} {}",
            self.violation_detail(&x)
        ))
    }

    fn boundaries(&self) -> Result<Vec<Boundary>, String> {
        let mut result = Vec::new();
        let mut append = |row: Rates, lower: f64, upper: f64, name: String| -> Result<(), String> {
            let norm = dot(&row, &row).sqrt();
            if !norm.is_finite() {
                return Err(format!("Nonfinite constraint norm: {name}"));
            }
            if norm == 0. {
                if lower > FEASIBILITY || upper < -FEASIBILITY {
                    return Err(format!(
                        "Unrealizable zero hard task: {name} lower={lower:.17e} upper={upper:.17e}"
                    ));
                }
                return Ok(());
            }
            let normalized = row.map(|v| v / norm);
            if lower == upper {
                result.push(Boundary {
                    row: normalized,
                    bound: lower / norm,
                    equality: true,
                    name,
                });
            } else {
                result.push(Boundary {
                    row: normalized.map(|v| -v),
                    bound: -lower / norm,
                    equality: false,
                    name: format!("{name} lower"),
                });
                result.push(Boundary {
                    row: normalized,
                    bound: upper / norm,
                    equality: false,
                    name: format!("{name} upper"),
                });
            }
            Ok(())
        };
        for i in 0..AXES {
            let mut row = [0.; AXES];
            row[i] = 1.;
            append(row, self.lower[i], self.upper[i], format!("axis {i}"))?;
        }
        for (index, task) in self.hard.iter().enumerate() {
            append(
                task.row,
                task.lower,
                task.upper,
                format!("hard {index} {:?}", task.name),
            )?;
        }
        if result
            .iter()
            .any(|b| !b.bound.is_finite() || b.row.iter().any(|v| !v.is_finite()))
        {
            return Err("Nonfinite normalized constraints".into());
        }
        Ok(result)
    }

    fn gradient(&self, x: &Rates) -> Result<Rates, String> {
        let mut gradient = x.map(|v| v * self.regularization);
        for task in &self.soft {
            let residual = task.weight * (dot(&task.row, x) - task.target);
            for (g, a) in gradient.iter_mut().zip(task.row) {
                *g += a * residual;
            }
        }
        if gradient.iter().any(|v| !v.is_finite()) {
            return Err("Nonfinite common task gradient".into());
        }
        Ok(gradient)
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
        let mut hessian = [[0.; AXES]; AXES];
        for (i, row) in hessian.iter_mut().enumerate() {
            row[i] = self.regularization;
        }
        for task in &self.soft {
            for i in 0..AXES {
                for j in 0..=i {
                    let value = task.weight * task.row[i] * task.row[j];
                    hessian[i][j] += value;
                    if i != j {
                        hessian[j][i] += value;
                    }
                }
            }
        }
        if hessian.iter().flatten().any(|v| !v.is_finite()) {
            return Err("Nonfinite common task Hessian".into());
        }
        let boundaries = self.boundaries()?;
        let mut working = Vec::new();
        let mut basis = Vec::new();
        // Permanent equalities are represented once. Redundant rows stay in
        // the original feasibility/certificate problem, never get relaxed.
        for (index, boundary) in boundaries.iter().enumerate().filter(|(_, b)| b.equality) {
            if independent(boundary.row, &basis) {
                working.push(index);
                basis = active_basis(&boundaries, &working)?.0;
            }
        }
        let mut x = self
            .project([0.; AXES])
            .map_err(|error| format!("Initial feasibility: {error}"))?;
        let mut last_residual = f64::INFINITY;
        let mut last_step = f64::INFINITY;
        let mut last_blocker = None;
        for iteration in 1..=SOLVE_ITERATIONS {
            let gradient = self.gradient(&x)?;
            let mut input = x;
            for i in 0..AXES {
                input[i] -= gradient[i] / lipschitz;
            }
            // Certify THE RETURNED POINT, using the same gradient, L,
            // projection implementation, and acceptance tolerances as before.
            // L amplifies projection error in this certificate: compute the
            // projection more accurately, never loosen either acceptance gate.
            // Use 1/8 of the residual scale as a numerical margin for the inner
            // stopping tests; this is not a universal projection-error bound
            // for nearly parallel constraints. The 512-cycle cap is unchanged.
            let precision = FEASIBILITY.min(CONVERGENCE / (8. * lipschitz));
            let projected = self.project_with_precision(input,precision).map_err(|error| {
                format!(
                    "Optimality certificate at iteration={iteration} L={lipschitz:.17e}: {error}"
                )
            })?;
            let residual = lipschitz * max_difference(&x, &projected);
            if !residual.is_finite() {
                return Err("Nonfinite common solve residual".into());
            }
            last_residual = residual;
            if residual <= CONVERGENCE {
                let violation = self.violation(&x);
                if violation > FEASIBILITY {
                    return Err(format!(
                        "Hard task residual exceeds tolerance: {}",
                        self.violation_detail(&x)
                    ));
                }
                return Ok(Solution {
                    rates: x,
                    maximum_constraint_error: violation,
                    projected_gradient_residual: residual,
                    iterations: iteration,
                });
            }
            let (active, triangular) = active_basis(&boundaries, &working)
                .map_err(|error| format!("Active set iteration={iteration}: {error}"))?;
            let correction = active_correction(&boundaries, &working, &active, &triangular, &x)?;
            let direction = newton_direction(&hessian, &gradient, &active, &correction)
                .map_err(|error| format!("Newton iteration={iteration}: {error}"))?;
            let magnitude = direction.iter().map(|v| v.abs()).fold(0., f64::max);
            if !magnitude.is_finite() {
                return Err(format!(
                    "Nonfinite Newton direction at iteration={iteration}"
                ));
            }
            if magnitude <= RANK_EPSILON {
                let multipliers = active_multipliers(&gradient, &active, &triangular)?;
                let drop = working
                    .iter()
                    .enumerate()
                    .filter(|(_, index)| !boundaries[**index].equality)
                    .filter(|(i, _)| multipliers[*i] < 0.)
                    .min_by(|(a, _), (b, _)| multipliers[*a].total_cmp(&multipliers[*b]));
                if let Some((index, _)) = drop {
                    working.remove(index);
                    continue;
                }
                return Err(format!(
                    "Active stationary point failed projected-gradient certificate: iteration={iteration} residual={residual:.17e} tolerance={CONVERGENCE:.17e} direction={magnitude:.17e} active={} L={lipschitz:.17e} {}",
                    working.len(),
                    self.violation_detail(&x)
                ));
            }
            let mut alpha = 1.;
            let mut blocker = None;
            for (index, boundary) in boundaries.iter().enumerate() {
                if boundary.equality || working.contains(&index) {
                    continue;
                }
                let approach = dot(&boundary.row, &direction);
                // Dot-product roundoff for a direction orthogonal to an
                // already-active normal must not manufacture a blocking row.
                if approach <= RANK_EPSILON * magnitude.max(1.) {
                    continue;
                }
                let slack = boundary.bound - dot(&boundary.row, &x);
                let candidate = (slack / approach).max(0.);
                if candidate < alpha {
                    alpha = candidate;
                    blocker = Some(index);
                }
            }
            last_step = alpha;
            last_blocker = blocker;
            for i in 0..AXES {
                x[i] += alpha * direction[i];
            }
            if x.iter().any(|v| !v.is_finite()) || self.violation(&x) > FEASIBILITY {
                return Err(format!(
                    "Newton feasible-step check failed: iteration={iteration} alpha={alpha:.17e} blocker={:?} {}",
                    blocker.map(|i| &boundaries[i].name),
                    self.violation_detail(&x)
                ));
            }
            if let Some(index) = blocker {
                if !independent(boundaries[index].row, &active) {
                    return Err(format!(
                        "Dependent blocking constraint: iteration={iteration} alpha={alpha:.17e} constraint={:?} residual={residual:.17e}",
                        boundaries[index].name
                    ));
                }
                working.push(index);
            }
        }
        Err(format!(
            "Common task solve did not converge within fixed iteration budget: iterations={SOLVE_ITERATIONS} projected_gradient_residual={last_residual:.17e} tolerance={CONVERGENCE:.17e} L={lipschitz:.17e} active={} last_alpha={last_step:.17e} blocker={:?} {}",
            working.len(),
            last_blocker.map(|i| &boundaries[i].name),
            self.violation_detail(&x)
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn problem() -> Problem {
        Problem {
            lower: [-1.; AXES],
            upper: [1.; AXES],
            hard: vec![],
            soft: vec![],
            regularization: 0.01,
        }
    }
    #[test]
    fn all_46_axes_share_one_coupled_task_without_integration() {
        let mut p = problem();
        p.soft.push(Task {
            row: [1.; AXES],
            target: 23.,
            weight: 1.,
        });
        let solution = p.solve().unwrap();
        for v in solution.rates {
            assert!((v - 23. / 46.01).abs() < 1e-6);
        }
        assert!(solution.iterations <= SOLVE_ITERATIONS);
        assert!(solution.projected_gradient_residual <= 1e-6);
    }
    #[test]
    fn hard_support_cannot_be_bought_by_posture_without_integration() {
        let mut p = problem();
        let mut row = [0.; AXES];
        row[0] = 1.;
        row[45] = 1.;
        p.hard.push(HardTask {
            name: "retained heel".into(),
            row,
            lower: 0.,
            upper: 0.,
        });
        p.soft.push(Task {
            row,
            target: 1.,
            weight: 1000.,
        });
        let solution = p.solve().unwrap();
        assert!(dot(&row, &solution.rates).abs() < 1e-8);
        assert!(solution.maximum_constraint_error <= FEASIBILITY);
    }
    #[test]
    fn contradictory_support_returns_no_command_without_integration() {
        let mut p = problem();
        let mut row = [0.; AXES];
        row[2] = 1.;
        p.hard.push(HardTask {
            name: "A".into(),
            row,
            lower: 0.5,
            upper: 1.,
        });
        p.hard.push(HardTask {
            name: "B".into(),
            row,
            lower: -1.,
            upper: -0.5,
        });
        assert!(p.solve().is_err());
    }
    #[test]
    fn command_bounds_and_nonfinite_data_are_rejected_without_integration() {
        let mut p = problem();
        p.upper[45] = 1.01;
        assert!(p.solve().is_err());
        p.upper[45] = 1.;
        p.regularization = f64::NAN;
        assert!(p.solve().is_err());
        let mut p = problem();
        let mut row = [0.; AXES];
        row[0] = 1.;
        p.soft.push(Task {
            row,
            target: f64::MAX,
            weight: 2.,
        });
        assert!(p.solve().is_err());
    }

    #[test]
    fn repeated_equalities_fixed_axes_and_saturation_have_a_certified_solution_without_integration()
    {
        let mut p = problem();
        p.lower[0] = 0.25;
        p.upper[0] = 0.25;
        let mut row = [0.; AXES];
        row[0] = 1.;
        row[1] = 1.;
        p.hard.push(HardTask {
            name: "sum equality".into(),
            row,
            lower: 0.5,
            upper: 0.5,
        });
        p.hard.push(HardTask {
            name: "same equality rescaled".into(),
            row: row.map(|v| v * 2.),
            lower: 1.,
            upper: 1.,
        });
        p.hard.push(HardTask {
            name: "zero equality".into(),
            row: [0.; AXES],
            lower: 0.,
            upper: 0.,
        });
        for axis in 0..AXES {
            let mut row = [0.; AXES];
            row[axis] = 1.;
            p.soft.push(Task {
                row,
                target: if axis % 2 == 0 { 3. } else { -3. },
                weight: 1.,
            });
        }
        let s = p.solve().unwrap();
        assert!((s.rates[0] - 0.25).abs() < 1e-8);
        assert!((s.rates[1] - 0.25).abs() < 1e-8);
        for axis in 2..AXES {
            assert!((s.rates[axis] - if axis % 2 == 0 { 1. } else { -1. }).abs() < 1e-8);
        }
        assert!(s.projected_gradient_residual <= CONVERGENCE);
        p.verify_rates(&s.rates).unwrap();
    }

    #[test]
    fn dense_46_axis_quadratic_recovers_constructed_kkt_optimum_without_integration() {
        let mut p = problem();
        p.regularization = 0.05;
        let expected: Rates = std::array::from_fn(|i| (i as f64 - 22.5) / 80.);
        let mut normal = [0.; AXES];
        for k in 0..8 {
            let row: Rates = std::array::from_fn(|i| (((i + 3) * (k + 5) % 17) as f64 - 8.) / 40.);
            let target = dot(&row, &expected);
            if k < 3 {
                p.hard.push(HardTask {
                    name: format!("permanent {k}"),
                    row,
                    lower: target,
                    upper: target,
                });
                let multiplier = if k % 2 == 0 { -0.4 } else { 0.3 };
                for i in 0..AXES {
                    normal[i] += row[i] * multiplier;
                }
            } else if k < 6 {
                p.hard.push(HardTask {
                    name: format!("binding {k}"),
                    row,
                    lower: target - 0.5,
                    upper: target,
                });
                for i in 0..AXES {
                    normal[i] += row[i] * 0.2;
                }
            } else {
                p.hard.push(HardTask {
                    name: format!("slack {k}"),
                    row,
                    lower: target - 0.5,
                    upper: target + 0.5,
                });
            }
        }
        for k in 0..32 {
            let row: Rates = std::array::from_fn(|i| (((i + 1) * (k + 7) % 23) as f64 - 11.) / 30.);
            p.soft.push(Task {
                row,
                target: dot(&row, &expected),
                weight: 0.2 + (k % 5) as f64,
            });
        }
        // Coupled tasks have zero residual at expected; these 46 tasks make
        // gradient(expected)+A_active^T lambda exactly zero by construction.
        for i in 0..AXES {
            let mut row = [0.; AXES];
            row[i] = 1.;
            p.soft.push(Task {
                row,
                target: expected[i] + p.regularization * expected[i] + normal[i],
                weight: 1.,
            });
        }
        let s = p.solve().unwrap();
        assert!(
            max_difference(&s.rates, &expected) < 2e-6,
            "error={} iterations={}",
            max_difference(&s.rates, &expected),
            s.iterations
        );
        assert!(s.projected_gradient_residual <= CONVERGENCE);
        p.verify_rates(&s.rates).unwrap();
    }

    #[test]
    fn nearly_parallel_constraints_do_not_require_a_singular_kkt_inverse_without_integration() {
        let mut p = problem();
        let mut row = [0.; AXES];
        row[0] = 1.;
        p.hard.push(HardTask {
            name: "first".into(),
            row,
            lower: 0.2,
            upper: 0.2,
        });
        row[1] = 1e-8;
        p.hard.push(HardTask {
            name: "nearly parallel".into(),
            row,
            lower: 0.2,
            upper: 0.2,
        });
        let mut desire = [0.; AXES];
        desire[2] = 1.;
        p.soft.push(Task {
            row: desire,
            target: 0.4,
            weight: 1.,
        });
        let s = p.solve().unwrap();
        assert!((s.rates[0] - 0.2).abs() < 1e-8);
        assert!((s.rates[2] - 0.4 / 1.01).abs() < 1e-7);
        p.verify_rates(&s.rates).unwrap();
    }

    #[test]
    fn refusals_identify_the_original_constraint_and_residual_without_integration() {
        let mut p = problem();
        let mut row = [0.; AXES];
        row[3] = 1.;
        p.hard.push(HardTask {
            name: "retained material x".into(),
            row,
            lower: -0.01,
            upper: 0.01,
        });
        let mut rates = [0.; AXES];
        rates[3] = 0.01001;
        let error = p.verify_rates(&rates).unwrap_err();
        assert!(
            error.contains("retained material x")
                && error.contains("maximum_violation=")
                && error.contains("side=upper"),
            "{error}"
        );
        p.lower[3] = 0.5;
        let error = p.solve().unwrap_err();
        assert!(
            error.contains("cycles=512") && error.contains("maximum_violation="),
            "{error}"
        );
    }

    #[test]
    fn active_equalities_restore_initial_projection_residual_at_high_curvature_without_integration()
    {
        let mut p = problem();
        p.lower[0] = 0.25;
        p.upper[0] = 0.25;
        let mut row = [0.; AXES];
        row[0] = 1.;
        row[1] = 1.;
        p.hard.push(HardTask {
            name: "coupled equality".into(),
            row,
            lower: 0.5,
            upper: 0.5,
        });
        for i in 0..2 {
            let mut row = [0.; AXES];
            row[i] = 1.;
            p.soft.push(Task {
                row,
                target: 0.25,
                weight: 1e6,
            });
        }
        let initial = p.project([0.; AXES]).unwrap();
        assert!(
            (initial[0] - 0.25).abs() > 1e-10,
            "fixture must exercise approximate initial feasibility"
        );
        let solution = p.solve().unwrap();
        assert!((solution.rates[0] - 0.25).abs() < 1e-12);
        assert!((solution.rates[1] - 0.25).abs() < 1e-12);
        assert!(solution.projected_gradient_residual <= CONVERGENCE);
    }

    #[test]
    fn finite_rates_with_overflowing_constraint_evaluation_are_refused_without_integration() {
        let mut p = problem();
        let mut row = [0.; AXES];
        row[0] = f64::MAX;
        row[1] = f64::MAX;
        p.hard.push(HardTask {
            name: "overflow".into(),
            row,
            lower: -f64::MAX,
            upper: f64::MAX,
        });
        let error = p.verify_rates(&[1.; AXES]).unwrap_err();
        assert!(error.contains("nonfinite hard_index=0"), "{error}");
    }

    #[cfg(feature = "experimental-solver-step-observation")]
    #[test]
    fn all_four_archived_qps_have_pure_numeric_certificates_without_integration() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../docs/evidence/coordination-numeric-correction-2026-10-12/reconstructed-problems.json"
        );
        let archived: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
        assert_eq!(archived["nativeSubstep"], 12);
        assert_eq!(archived["nativeF32ComMatchesArchiveBitwise"], true);
        let rates = |v: &serde_json::Value| -> Rates {
            let values = v.as_array().unwrap();
            assert_eq!(values.len(), AXES);
            std::array::from_fn(|i| values[i].as_f64().unwrap())
        };
        let cases = archived["cases"].as_array().unwrap();
        assert_eq!(cases.len(), 4);
        for case in cases {
            let source = &case["problem"];
            let p = Problem {
                lower: rates(&source["lower"]),
                upper: rates(&source["upper"]),
                regularization: source["regularization"].as_f64().unwrap(),
                hard: source["hard"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|task| HardTask {
                        name: task["name"].as_str().unwrap().into(),
                        row: rates(&task["row"]),
                        lower: task["lower"].as_f64().unwrap(),
                        upper: task["upper"].as_f64().unwrap(),
                    })
                    .collect(),
                soft: source["soft"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|task| Task {
                        row: rates(&task["row"]),
                        target: task["target"].as_f64().unwrap(),
                        weight: task["weight"].as_f64().unwrap(),
                    })
                    .collect(),
            };
            let solution = p
                .solve()
                .unwrap_or_else(|error| panic!("scale={}: {error}", case["feedbackScale"]));
            p.verify_rates(&solution.rates).unwrap();
            let lipschitz = p.regularization
                + p.soft
                    .iter()
                    .map(|task| task.weight * dot(&task.row, &task.row))
                    .sum::<f64>();
            let gradient = p.gradient(&solution.rates).unwrap();
            let projected = p
                .project_with_precision(
                    std::array::from_fn(|i| solution.rates[i] - gradient[i] / lipschitz),
                    FEASIBILITY.min(CONVERGENCE / (8. * lipschitz)),
                )
                .unwrap();
            let residual = lipschitz * max_difference(&solution.rates, &projected);
            assert_eq!(residual, solution.projected_gradient_residual);
            assert!(residual <= CONVERGENCE && solution.maximum_constraint_error <= FEASIBILITY);
            assert!(solution.iterations <= SOLVE_ITERATIONS);
            eprintln!(
                "archived QP scale={} iterations={} projectedGradientResidual={:.17e} maximumConstraintError={:.17e}",
                case["feedbackScale"],
                solution.iterations,
                residual,
                solution.maximum_constraint_error
            );
        }
    }
}
