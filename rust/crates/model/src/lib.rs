//! Versioned canonical anatomy. No browser, rendering, or physics-engine types.
use nalgebra::{Matrix3, SymmetricEigen, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

pub const CANONICAL_JSON: &str = include_str!("../data/canonical-v1.json");
pub const PROTOCOL_JSON: &str = include_str!("../data/protocol-v1.json");
pub const FLAT_FLOOR_HALF_EXTENTS: [f32; 3] = [12.0, 0.08, 10.5];
pub const FLAT_FLOOR_CENTER: [f32; 3] = [0.0, -0.08, 0.0];

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
impl Vec3 {
    pub fn array(self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }
    pub fn finite(self) -> bool {
        self.array().iter().all(|v| v.is_finite())
    }
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct Quat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Geometry {
    pub vertices: Vec<f32>,
    pub indices: Vec<u32>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Frame {
    pub anchor: Vec3,
    pub rotation: Quat,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Axis {
    pub coordinate: char,
    pub min_radians: f32,
    pub max_radians: f32,
    pub passive_stiffness_nm_per_rad: f32,
    pub damping_nms_per_rad: f32,
    pub max_motor_torque_nm: f32,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Joint {
    pub parent_frame: Frame,
    pub child_frame: Frame,
    pub axes: Vec<Axis>,
    pub limit_soft_zone_fraction: f32,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Pose {
    pub id: String,
    pub position: Vec3,
    pub rotation: Quat,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
}
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CourseGeometry {
    pub vertices: Vec<Vec3>,
    pub triangles: Vec<[u32; 3]>,
    pub surface_triangles: Option<usize>,
}
#[derive(Debug, Clone, Deserialize)]
pub struct CourseMotion {
    pub pitch: f32,
    pub roll: f32,
    pub speed: f32,
}
#[derive(Debug, Clone, Deserialize)]
pub struct CoursePiece {
    pub id: String,
    pub geometry: CourseGeometry,
    pub position: Vec3,
    pub color: String,
    pub friction: f32,
    #[serde(default)]
    pub tiled: bool,
    pub motion: Option<CourseMotion>,
}
impl CoursePiece {
    /// Shared with the physical course: one second stationary, two second ramp.
    pub fn rotation_at(&self, time_s: f32) -> Quat {
        let Some(motion) = &self.motion else {
            return Quat {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            };
        };
        let ramp = ((time_s - 1.0) / 2.0).clamp(0.0, 1.0);
        let t = (time_s - 1.0).max(0.0) * motion.speed;
        let (sx, cx) = ((t * 1.3).sin() * motion.pitch * ramp * 0.5).sin_cos();
        let (sz, cz) = ((t * 0.93).sin() * motion.roll * ramp * 0.5).sin_cos();
        Quat {
            x: sx * cz,
            y: -sx * sz,
            z: cx * sz,
            w: cx * cz,
        }
    }
}
#[derive(Debug, Clone, Deserialize)]
pub struct CourseStation {
    pub id: String,
    pub position: Vec3,
}
#[derive(Debug, Clone, Deserialize)]
pub struct VisualPiece {
    pub id: String,
    pub geometry: CourseGeometry,
    pub position: Vec3,
    pub color: String,
}
#[derive(Debug, Clone, Deserialize)]
pub struct ProtocolRoom {
    pub width: f32,
    pub depth: f32,
    pub height: f32,
}
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Protocol {
    pub room: ProtocolRoom,
    pub striker_head: Vec3,
    pub stow_height: f32,
    pub pieces: Vec<VisualPiece>,
}
impl Protocol {
    pub fn canonical() -> Result<Self, String> {
        let p: Self = serde_json::from_str(PROTOCOL_JSON).map_err(|e| e.to_string())?;
        if p.pieces.len() != 4
            || !p.striker_head.finite()
            || [
                p.room.width,
                p.room.height,
                p.room.depth,
                p.stow_height,
                p.striker_head.x,
                p.striker_head.y,
                p.striker_head.z,
            ]
            .iter()
            .any(|v| !v.is_finite() || *v <= 0.0)
            || p.pieces.iter().any(|piece| {
                !piece.position.finite()
                    || piece.geometry.vertices.iter().any(|v| !v.finite())
                    || piece
                        .geometry
                        .triangles
                        .iter()
                        .flatten()
                        .any(|v| *v as usize >= piece.geometry.vertices.len())
            })
        {
            return Err("Invalid canonical protocol".into());
        }
        Ok(p)
    }
}
impl Model {
    pub fn course(&self, difficulty: &str) -> Result<Vec<CoursePiece>, String> {
        let pieces: Vec<CoursePiece> =
            serde_json::from_value(self.playground["courses"][difficulty].clone())
                .map_err(|e| format!("Course {difficulty}: {e}"))?;
        if pieces.len() != 26 {
            return Err("Canonical course must contain 26 pieces".into());
        }
        for piece in &pieces {
            if !piece.position.finite()
                || !piece.friction.is_finite()
                || piece.friction < 0.0
                || piece.geometry.vertices.iter().any(|v| !v.finite())
                || piece
                    .geometry
                    .triangles
                    .iter()
                    .flatten()
                    .any(|i| *i as usize >= piece.geometry.vertices.len())
                || piece.tiled
                    && piece
                        .geometry
                        .surface_triangles
                        .is_none_or(|n| n == 0 || n > piece.geometry.triangles.len())
            {
                return Err(format!("Invalid course solid {}", piece.id));
            }
        }
        Ok(pieces)
    }
    pub fn station(&self, id: &str) -> Result<Vec3, String> {
        let stations: Vec<CourseStation> =
            serde_json::from_value(self.playground["stations"].clone())
                .map_err(|e| e.to_string())?;
        stations
            .into_iter()
            .find(|s| s.id == id)
            .map(|s| s.position)
            .ok_or_else(|| format!("Unknown station {id}"))
    }
}
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct GoldenMass {
    pub volume: f64,
    pub center_of_mass: Vec3,
    pub inertia: [[f64; 3]; 3],
}
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Segment {
    pub id: String,
    pub region: Option<String>,
    pub parent: Option<String>,
    pub role: String,
    pub side: Option<String>,
    pub mass_kg: f32,
    pub geometry: Geometry,
    pub joint_profile: Option<Joint>,
    pub collision_exclusions: Vec<String>,
    pub initial: Pose,
    pub golden_mass: GoldenMass,
}
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Model {
    pub schema_version: u16,
    pub total_mass_kg: f32,
    pub stature_m: f32,
    pub segments: Vec<Segment>,
    pub playground: serde_json::Value,
    pub protocol: serde_json::Value,
}
impl Model {
    pub fn canonical() -> Result<Self, String> {
        let m: Self = serde_json::from_str(CANONICAL_JSON).map_err(|e| e.to_string())?;
        m.validate()?;
        Ok(m)
    }
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != 1 || self.segments.len() != 25 {
            return Err("Anatomy/schema mismatch".into());
        }
        let mass: f32 = self.segments.iter().map(|s| s.mass_kg).sum();
        if (mass - 72.2).abs() > 1e-4 || (self.stature_m - 1.84).abs() > 1e-6 {
            return Err("Mass/stature mismatch".into());
        }
        let mut seen = std::collections::BTreeSet::new();
        for s in &self.segments {
            if !seen.insert(&s.id)
                || !s.initial.position.finite()
                || s.parent.as_ref().is_some_and(|p| !seen.contains(p))
            {
                return Err(format!("Invalid topology: {}", s.id));
            }
            if s.parent.is_some() != s.joint_profile.is_some() {
                return Err(format!("Missing joint: {}", s.id));
            }
            integrate_mass(&s.geometry, s.mass_kg)?;
            if let Some(j) = &s.joint_profile {
                if !j.limit_soft_zone_fraction.is_finite()
                    || !(0.0..=0.5).contains(&j.limit_soft_zone_fraction)
                {
                    return Err(format!("Invalid soft joint limit: {}", s.id));
                }
                for a in &j.axes {
                    if !matches!(a.coordinate, 'x' | 'y' | 'z')
                        || !a.min_radians.is_finite()
                        || !a.max_radians.is_finite()
                        || a.min_radians > a.max_radians
                        || a.max_motor_torque_nm <= 0.0
                    {
                        return Err(format!("Invalid axis: {}", s.id));
                    }
                }
            }
            for excluded in &s.collision_exclusions {
                self.segments
                    .iter()
                    .find(|o| &o.id == excluded)
                    .ok_or("Unknown exclusion")?;
            }
        }
        Ok(())
    }
}
#[derive(Debug, Clone)]
pub struct IntegratedMass {
    pub volume: f64,
    pub center: Vec3,
    pub tensor: Matrix3<f64>,
    pub principal: Vec3,
    pub frame: Quat,
}
/// Signed tetrahedral integration of the closed canonical surface. f64 is used
/// only during initialization; the resulting production bodies use f32.
pub fn integrate_mass(g: &Geometry, mass: f32) -> Result<IntegratedMass, String> {
    if !mass.is_finite()
        || mass <= 0.0
        || !g.vertices.len().is_multiple_of(3)
        || !g.indices.len().is_multiple_of(3)
        || g.vertices.iter().any(|v| !v.is_finite())
    {
        return Err("Invalid mass mesh".into());
    }
    let mut volume = 0.0_f64;
    let mut first = Vector3::<f64>::zeros();
    let mut second = Matrix3::<f64>::zeros();
    for tri in g.indices.chunks_exact(3) {
        let mut p = [Vector3::<f64>::zeros(); 3];
        for (i, &index) in tri.iter().enumerate() {
            let o = index as usize * 3;
            let xyz = g.vertices.get(o..o + 3).ok_or("Mesh index out of range")?;
            p[i] = Vector3::new(xyz[0] as f64, xyz[1] as f64, xyz[2] as f64);
        }
        let v = p[0].dot(&p[1].cross(&p[2])) / 6.0;
        let sum = p[0] + p[1] + p[2];
        volume += v;
        first += sum * (v / 4.0);
        second += (sum * sum.transpose()
            + p.iter().map(|p| p * p.transpose()).sum::<Matrix3<f64>>())
            * (v / 20.0);
    }
    if !volume.is_finite() || volume.abs() < 1e-15 {
        return Err("Degenerate mesh".into());
    }
    let com = first / volume;
    let covariance = second * (mass as f64 / volume) - com * com.transpose() * (mass as f64);
    let tensor = Matrix3::identity() * covariance.trace() - covariance;
    let eigen = SymmetricEigen::new(tensor);
    let moments = eigen.eigenvalues;
    if moments.iter().any(|m| !m.is_finite() || *m <= 0.0)
        || (0..3).any(|i| moments[i] > moments[(i + 1) % 3] + moments[(i + 2) % 3] + 1e-10)
    {
        return Err("Invalid principal moments".into());
    }
    let mut axes = eigen.eigenvectors;
    if axes.determinant() < 0.0 {
        axes.set_column(2, &(-axes.column(2)));
    }
    let q = UnitQuaternion::from_matrix(&axes);
    Ok(IntegratedMass {
        volume: volume.abs(),
        center: Vec3 {
            x: com.x as f32,
            y: com.y as f32,
            z: com.z as f32,
        },
        tensor,
        principal: Vec3 {
            x: moments.x as f32,
            y: moments.y as f32,
            z: moments.z as f32,
        },
        frame: Quat {
            x: q.i as f32,
            y: q.j as f32,
            z: q.k as f32,
            w: q.w as f32,
        },
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn canonical_tensors_match_independent_legacy_integral() {
        let m = Model::canonical().expect("canonical");
        for s in m.segments {
            let p = integrate_mass(&s.geometry, s.mass_kg).expect("mass");
            assert!(
                (p.volume - s.golden_mass.volume.abs()).abs() < 1e-10,
                "{} volume",
                s.id
            );
            for i in 0..3 {
                for j in 0..3 {
                    assert!(
                        (p.tensor[(i, j)] - s.golden_mass.inertia[i][j]).abs() < 1e-8,
                        "{} tensor",
                        s.id
                    );
                }
            }
            assert!(
                p.center
                    .array()
                    .iter()
                    .zip(s.golden_mass.center_of_mass.array())
                    .all(|(a, b)| (*a - b).abs() < 1e-6)
            );
        }
    }
    #[test]
    fn rejects_invalid_geometry() {
        let mut g = Model::canonical()
            .expect("canonical")
            .segments
            .remove(0)
            .geometry;
        g.vertices[0] = f32::NAN;
        assert!(integrate_mass(&g, 1.0).is_err());
    }
    #[test]
    fn analytic_box_and_mirrored_mesh_tensors() {
        let g = Geometry {
            vertices: vec![
                -2.0, -1.0, -3.0, 2.0, -1.0, -3.0, 2.0, -1.0, 3.0, -2.0, -1.0, 3.0, -2.0, 1.0,
                -3.0, 2.0, 1.0, -3.0, 2.0, 1.0, 3.0, -2.0, 1.0, 3.0,
            ],
            indices: vec![
                0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 3, 7, 6, 3, 6, 2, 0, 4, 7, 0,
                7, 3, 1, 2, 6, 1, 6, 5,
            ],
        };
        let p = integrate_mass(&g, 5.0).expect("box");
        assert!((p.volume - 48.0).abs() < 1e-10);
        for (i, expected) in [50.0 / 3.0, 65.0 / 3.0, 25.0 / 3.0].iter().enumerate() {
            assert!((p.tensor[(i, i)] - expected).abs() < 1e-10);
        }
        for s in Model::canonical().expect("canonical").segments {
            let a = integrate_mass(&s.geometry, s.mass_kg).expect("original");
            let mut reflected = s.geometry;
            for p in reflected.vertices.chunks_exact_mut(3) {
                p[0] = -p[0];
            }
            for t in reflected.indices.chunks_exact_mut(3) {
                t.swap(1, 2);
            }
            let b = integrate_mass(&reflected, s.mass_kg).expect("reflected");
            assert!((a.volume - b.volume).abs() < 1e-10);
            for i in 0..3 {
                for j in 0..3 {
                    let sign = if (i == 0) != (j == 0) { -1.0 } else { 1.0 };
                    assert!((b.tensor[(i, j)] - sign * a.tensor[(i, j)]).abs() < 1e-10);
                }
            }
        }
    }
}
