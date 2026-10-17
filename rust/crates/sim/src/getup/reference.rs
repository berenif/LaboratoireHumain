//! Source scipy.splrep(s=0, k=3) on uniform sample times: not-a-knot cubic.
//! Only joint angles, CoM height and torso vertical enter SLOW's target vector.
pub fn interpolate(knots: &[[f32; 25]], speed: f64) -> Result<Vec<[f32; 25]>, String> {
    let n = knots.len();
    if !(4..=250).contains(&n)
        || !(0.2..=0.8).contains(&speed)
        || knots.iter().flatten().any(|x| !x.is_finite())
    {
        return Err("Invalid source reference knots/speed".into());
    }
    let count = (n as f64 / speed).ceil() as usize;
    let mut out = vec![[0.0; 25]; count];
    for axis in 0..25 {
        let y: Vec<f64> = knots.iter().map(|p| p[axis] as f64).collect();
        // Eliminate endpoint second derivatives with the not-a-knot equations.
        let m = n - 2;
        let mut upper = vec![1.0; m];
        upper[0] = 0.0;
        upper[m - 1] = 0.0;
        let mut rhs = vec![0.0; m];
        let mut diagonal = vec![4.0; m];
        diagonal[0] = 6.0;
        diagonal[m - 1] = 6.0;
        for i in 0..m {
            rhs[i] = 6.0 * (y[i] - 2.0 * y[i + 1] + y[i + 2]);
        }
        for i in 1..m {
            let lower = if i == m - 1 { 0.0 } else { 1.0 };
            let factor = lower / diagonal[i - 1];
            diagonal[i] -= factor * upper[i - 1];
            rhs[i] -= factor * rhs[i - 1];
        }
        let mut second = vec![0.0; n];
        second[n - 2] = rhs[m - 1] / diagonal[m - 1];
        for i in (0..m - 1).rev() {
            second[i + 1] = (rhs[i] - upper[i] * second[i + 2]) / diagonal[i];
        }
        second[0] = 2.0 * second[1] - second[2];
        second[n - 1] = 2.0 * second[n - 2] - second[n - 3];
        for (j, pose) in out.iter_mut().enumerate() {
            let x = j as f64 * (n - 1) as f64 / (count - 1) as f64;
            let i = (x.floor() as usize).min(n - 2);
            let b = x - i as f64;
            let a = 1.0 - b;
            pose[axis] = (a * y[i]
                + b * y[i + 1]
                + ((a * a * a - a) * second[i] + (b * b * b - b) * second[i + 1]) / 6.0)
                as f32;
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    #[test]
    fn fixed_splines_match_scipy_without_physics() {
        let data: serde_json::Value =
            serde_json::from_str(include_str!("../../data/getup-policy/parity.json")).unwrap();
        let mut maximum = 0.0_f64;
        for fixture in data["splines"].as_array().unwrap() {
            let knots: Vec<[f32; 25]> = serde_json::from_value(fixture["knots"].clone()).unwrap();
            let actual = super::interpolate(&knots, fixture["speed"].as_f64().unwrap()).unwrap();
            let expected: Vec<Vec<f64>> =
                serde_json::from_value(fixture["expected"].clone()).unwrap();
            assert_eq!(actual.len(), expected.len());
            for (a, b) in actual.iter().flatten().zip(expected.iter().flatten()) {
                maximum = maximum.max((*a as f64 - b).abs());
            }
        }
        println!("12 source spline fixtures, max error {maximum}; zero physics");
        assert!(maximum < 1e-6);
        assert!(super::interpolate(&[[0.0; 25]; 3], 0.5).is_err());
    }
}
