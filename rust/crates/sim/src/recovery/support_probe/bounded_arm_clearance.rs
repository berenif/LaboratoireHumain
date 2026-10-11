//! A fixed measured root and a bounded feasibility solve; failure never commits q.
use super::*;

const MARGIN: f32 = 0.00001;
#[derive(Clone, Copy)]
struct Axis {
    ancestor: usize,
    body: usize,
    coordinate: usize,
    low: f32,
    high: f32,
}
struct Constraint {
    residual: f32,
    derivative: Vec<f64>,
}

fn derivative(k: &Kinematics, arm: &[usize], a: Axis, body: usize, point: Vector) -> Vector {
    if arm
        .iter()
        .position(|&i| i == body)
        .is_some_and(|n| a.ancestor <= n)
    {
        k.directions[a.body]
            .col(a.coordinate)
            .cross(point - k.anchors[a.body])
    } else {
        Vector::ZERO
    }
}

fn constraints(
    s: &Simulation,
    arm: &[usize],
    axes: &[Axis],
    k: &Kinematics,
    extra: f32,
) -> Result<Vec<Constraint>, String> {
    let mut out = Vec::new();
    for &i in arm {
        let required = s.world.colliders[s.floor].contact_skin()
            + s.world.colliders[s.colliders[i]].contact_skin()
            + extra;
        for p in vertices(s, i, k.poses[i]) {
            out.push(Constraint {
                residual: required + MARGIN - p.y,
                derivative: axes
                    .iter()
                    .map(|&a| derivative(k, arm, a, i, p).y as f64)
                    .collect(),
            });
        }
    }
    // Preserve the existing high-hand task (12 cm at full preparation) alongside floor clearance.
    let hand = arm[4];
    let p = k.poses[hand].translation;
    out.push(Constraint {
        residual: 0.12 * (extra / 0.005).clamp(0., 1.) + MARGIN - p.y,
        derivative: axes
            .iter()
            .map(|&a| derivative(k, arm, a, hand, p).y as f64)
            .collect(),
    });
    let torso = index(s, "torso");
    let (gap, n, p) = skin_plane(s, k, arm[1]);
    let skin = s.world.colliders[s.colliders[torso]].contact_skin()
        + s.world.colliders[s.colliders[arm[1]]].contact_skin();
    out.push(Constraint {
        residual: skin + MARGIN - gap,
        derivative: axes
            .iter()
            .map(|&a| n.dot(derivative(k, arm, a, arm[1], p)) as f64)
            .collect(),
    });
    for &i in arm {
        for j in 0..s.model.segments.len() {
            if i == j
                || (arm.contains(&j) && j < i)
                || s.model.segments[i]
                    .collision_exclusions
                    .contains(&s.model.segments[j].id)
                || s.model.segments[j]
                    .collision_exclusions
                    .contains(&s.model.segments[i].id)
            {
                continue;
            }
            let ca = &s.world.colliders[s.colliders[i]];
            let cb = &s.world.colliders[s.colliders[j]];
            let required = ca.contact_skin() + cb.contact_skin();
            let aa = ca
                .shape()
                .compute_aabb(&k.poses[i])
                .loosened(required + 0.002);
            if !aa.intersects(&cb.shape().compute_aabb(&k.poses[j])) {
                continue;
            }
            let c = rapier3d::parry::query::contact(
                &k.poses[i],
                ca.shape(),
                &k.poses[j],
                cb.shape(),
                required + 0.002,
            )
            .map_err(|_| {
                format!(
                    "unsupported collision query {} / {}",
                    s.model.segments[i].id, s.model.segments[j].id
                )
            })?;
            if let Some(c) = c {
                out.push(Constraint {
                    residual: required + MARGIN - c.dist,
                    derivative: axes
                        .iter()
                        .map(|&a| {
                            c.normal1.dot(
                                derivative(k, arm, a, j, c.point2)
                                    - derivative(k, arm, a, i, c.point1),
                            ) as f64
                        })
                        .collect(),
                });
            }
        }
    }
    Ok(out)
}
fn merit(c: &[Constraint]) -> f64 {
    c.iter().map(|x| (x.residual.max(0.) as f64).powi(2)).sum()
}

/// Strict convex box-constrained least squares, with KKT releases at active bounds.
fn bounded_step(c: &[Constraint], axes: &[Axis], q: &Coordinates) -> Option<Vec<f64>> {
    let n = axes.len();
    let mut h = na::DMatrix::<f64>::identity(n, n) * 1e-6;
    let mut g = na::DVector::<f64>::zeros(n);
    for r in c.iter().filter(|r| r.residual > 0.) {
        for i in 0..n {
            g[i] += r.derivative[i] * r.residual as f64;
            for j in 0..n {
                h[(i, j)] += r.derivative[i] * r.derivative[j];
            }
        }
    }
    let lo: Vec<_> = axes
        .iter()
        .map(|a| ((a.low - q[a.body][a.coordinate]) as f64).max(-0.12))
        .collect();
    let hi: Vec<_> = axes
        .iter()
        .map(|a| ((a.high - q[a.body][a.coordinate]) as f64).min(0.12))
        .collect();
    let mut x = na::DVector::<f64>::zeros(n);
    let mut active = vec![0_i8; n];
    for _ in 0..64 {
        let free: Vec<_> = (0..n).filter(|&i| active[i] == 0).collect();
        let mut z = x.clone();
        if !free.is_empty() {
            let hf = na::DMatrix::from_fn(free.len(), free.len(), |i, j| h[(free[i], free[j])]);
            let gf = na::DVector::from_fn(free.len(), |i, _| {
                let ii = free[i];
                g[ii]
                    - (0..n)
                        .filter(|&j| active[j] != 0)
                        .map(|j| h[(ii, j)] * x[j])
                        .sum::<f64>()
            });
            let solution = hf.cholesky()?.solve(&gf);
            for (i, &j) in free.iter().enumerate() {
                z[j] = solution[i];
            }
        }
        let mut alpha = 1.0_f64;
        let mut hit = None;
        for &i in &free {
            let d = z[i] - x[i];
            let t = if z[i] < lo[i] {
                (lo[i] - x[i]) / d
            } else if z[i] > hi[i] {
                (hi[i] - x[i]) / d
            } else {
                continue;
            };
            if t < alpha {
                alpha = t.max(0.);
                hit = Some((i, if d < 0. { -1 } else { 1 }));
            }
        }
        x += (z - &x) * alpha;
        if let Some((i, side)) = hit {
            x[i] = if side < 0 { lo[i] } else { hi[i] };
            active[i] = side;
            continue;
        }
        let gradient = &h * &x - &g;
        let release = (0..n)
            .filter(|&i| active[i] != 0)
            .map(|i| (i, gradient[i] * active[i] as f64))
            .filter(|&(_, v)| v > 1e-10)
            .max_by(|a, b| a.1.total_cmp(&b.1));
        if let Some((i, _)) = release {
            active[i] = 0;
        } else {
            return Some(x.iter().copied().collect());
        }
    }
    None
}

pub(super) fn project(
    s: &Simulation,
    q: &mut Coordinates,
    side: &str,
    extra: f32,
    warm: Option<&Coordinates>,
) -> Result<Value, Value> {
    let arm = arm_indices(s, side);
    let torso = *s.world.bodies[s.bodies[index(s, "torso")]].position();
    let axes: Vec<_> = arm
        .iter()
        .enumerate()
        .flat_map(|(ancestor, &i)| {
            s.model.segments[i]
                .joint_profile
                .as_ref()
                .unwrap()
                .axes
                .iter()
                .map(move |a| Axis {
                    ancestor,
                    body: i,
                    coordinate: match a.coordinate {
                        'x' => 0,
                        'y' => 1,
                        _ => 2,
                    },
                    low: a.min_radians,
                    high: a.max_radians,
                })
        })
        .collect();
    let desired = *q;
    let mut nominal = desired;
    for a in &axes {
        nominal[a.body][a.coordinate] = nominal[a.body][a.coordinate].clamp(a.low, a.high);
    }
    // Attempt the requested reference first: a warm feasible pose must not erase task progress.
    let mut seeds = vec![nominal];
    if let Some(w) = warm {
        let mut seed = nominal;
        for a in &axes {
            seed[a.body][a.coordinate] = w[a.body][a.coordinate].clamp(a.low, a.high);
        }
        seeds.push(seed);
    }
    for corner in 0..8 {
        let mut seed = nominal;
        for (n, a) in axes.iter().filter(|a| a.ancestor == 0).enumerate() {
            seed[a.body][a.coordinate] = if corner & (1 << n) == 0 {
                a.low
            } else {
                a.high
            };
        }
        seeds.push(seed);
    }
    let mut best = f32::INFINITY;
    let mut iterations = 0;
    for (start, mut candidate) in seeds.into_iter().enumerate() {
        for _ in 0..32 {
            iterations += 1;
            let k = fk(s, &candidate, &arm, torso);
            let c = constraints(s, &arm, &axes, &k, extra)
                .map_err(|e| json!({"status":"unsupported-query","detail":e}))?;
            let residual = c
                .iter()
                .map(|c| c.residual)
                .fold(f32::NEG_INFINITY, f32::max);
            best = best.min(residual);
            // Margin is solely numerical: all requested inequalities hold even at this tolerance.
            if residual <= MARGIN * 0.1
                && axes.iter().all(|a| {
                    candidate[a.body][a.coordinate] >= a.low
                        && candidate[a.body][a.coordinate] <= a.high
                })
            {
                *q = candidate;
                let links:Vec<_>=arm.iter().map(|&i|json!({"id":s.model.segments[i].id,
                    "projectedLowestY":vertices(s,i,k.poses[i]).iter().map(|p|p.y).fold(f32::INFINITY,f32::min),
                    "requiredLowestY":s.world.colliders[s.floor].contact_skin()+s.world.colliders[s.colliders[i]].contact_skin()+extra})).collect();
                return Ok(
                    json!({"status":"certified","iterations":iterations,"start":start,
                    "links":links,"maximumConstraintResidualM":residual,"constraintsChecked":c.len(),
                    "maximumCoordinateCorrectionRad":axes.iter().map(|a|(candidate[a.body][a.coordinate]-desired[a.body][a.coordinate]).abs()).fold(0_f32,f32::max),
                    "pelvisGlobalTarget":Value::Null,"root":"measured torso, fixed","extraClearanceM":extra}),
                );
            }
            let Some(delta) = bounded_step(&c, &axes, &candidate) else {
                break;
            };
            let old_merit = merit(&c);
            let mut accepted = None;
            for reduction in 0..10 {
                let scale = 0.5_f64.powi(reduction);
                let mut trial = candidate;
                for (a, &d) in axes.iter().zip(&delta) {
                    // Roundoff correction only: the subproblem itself is solved within bounds.
                    trial[a.body][a.coordinate] =
                        (candidate[a.body][a.coordinate] + (scale * d) as f32).clamp(a.low, a.high);
                }
                let ck = constraints(s, &arm, &axes, &fk(s, &trial, &arm, torso), extra)
                    .map_err(|e| json!({"status":"unsupported-query","detail":e}))?;
                if merit(&ck) < old_merit * (1. - 1e-5) {
                    accepted = Some(trial);
                    break;
                }
            }
            let Some(trial) = accepted else {
                break;
            };
            candidate = trial;
        }
    }
    Err(
        json!({"status":"not-converged","iterations":iterations,"bestMaximumConstraintResidualM":best,
        "commandCommitted":false,"globalImpossibilityProven":false,"pelvisGlobalTarget":Value::Null}),
    )
}

#[test]
fn active_bound_redistributes_the_step_instead_of_clipping_it() {
    let axes = [
        Axis {
            ancestor: 0,
            body: 1,
            coordinate: 0,
            low: -1.,
            high: 0.,
        },
        Axis {
            ancestor: 1,
            body: 2,
            coordinate: 0,
            low: -1.,
            high: 1.,
        },
    ];
    let c = [Constraint {
        residual: 0.1,
        derivative: vec![1., 1.],
    }];
    let q = [[0.; 3]; 25];
    let d = bounded_step(&c, &axes, &q).unwrap();
    assert_eq!(d[0], 0.);
    assert!((d[1] - 0.1).abs() < 1e-6);
}
