//! Canonical presentation and exact surface picking. No simulation dependency.
use glam::{Mat4, Quat, Vec3};
use lh_contracts::Snapshot;
use lh_model::Model;
use serde::Serialize;

#[cfg(target_arch = "wasm32")]
mod gpu;
#[cfg(target_arch = "wasm32")]
pub use gpu::*;

pub const BODY_COUNT: usize = 25;
pub const DRAW_COUNT: usize = 64;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub segment: u32,
}

pub struct Mesh {
    pub model: Model,
    pub vertices: Vec<Vertex>,
    pub course: Vec<lh_model::CoursePiece>,
    pub protocol: Option<lh_model::Protocol>,
    body_vertex_count: usize,
}
impl Mesh {
    pub fn canonical() -> Result<Self, String> {
        let model = Model::canonical()?;
        let mut vertices = Vec::new();
        for (segment, part) in model.segments.iter().enumerate() {
            for face in part.geometry.indices.chunks_exact(3) {
                let points = face.map_array(|index| {
                    let start = index as usize * 3;
                    Vec3::from_slice(&part.geometry.vertices[start..start + 3])
                });
                let normal = (points[1] - points[0])
                    .cross(points[2] - points[0])
                    .normalize();
                if !normal.is_finite() {
                    return Err("Degenerate render triangle".into());
                }
                vertices.extend(points.map(|point| Vertex {
                    position: point.to_array(),
                    normal: normal.to_array(),
                    segment: segment as u32,
                }));
            }
        }
        let body_vertex_count = vertices.len();
        // The same primitive defines the current native and presented floor.
        let half = Vec3::from_array(lh_model::FLAT_FLOOR_HALF_EXTENTS);
        let center = Vec3::from_array(lh_model::FLAT_FLOOR_CENTER);
        let points = [
            [-1.0, -1.0, -1.0],
            [1.0, -1.0, -1.0],
            [1.0, -1.0, 1.0],
            [-1.0, -1.0, 1.0],
            [-1.0, 1.0, -1.0],
            [1.0, 1.0, -1.0],
            [1.0, 1.0, 1.0],
            [-1.0, 1.0, 1.0],
        ]
        .map(|sign| (center + half * Vec3::from_array(sign)).to_array());
        for face in [
            [4, 6, 5],
            [4, 7, 6],
            [0, 1, 2],
            [0, 2, 3],
            [0, 5, 1],
            [0, 4, 5],
            [3, 6, 7],
            [3, 2, 6],
            [0, 7, 4],
            [0, 3, 7],
            [1, 6, 2],
            [1, 5, 6],
        ] {
            let p = face.map(|i| Vec3::from_array(points[i]));
            let normal = (p[1] - p[0]).cross(p[2] - p[0]).normalize();
            vertices.extend(p.map(|point| Vertex {
                position: point.to_array(),
                normal: normal.to_array(),
                segment: BODY_COUNT as u32,
            }));
        }
        Ok(Self {
            model,
            vertices,
            course: Vec::new(),
            protocol: None,
            body_vertex_count,
        })
    }
    pub fn set_course(
        &mut self,
        settings: Option<lh_contracts::PlaygroundSettings>,
    ) -> Result<(), String> {
        let course = match settings {
            Some(settings) => self.model.course(match settings.difficulty {
                lh_contracts::Difficulty::Gentle => "gentle",
                lh_contracts::Difficulty::Challenging => "challenging",
                lh_contracts::Difficulty::Extreme => "extreme",
            })?,
            None => Vec::new(),
        };
        if course.len() + BODY_COUNT + 1 > DRAW_COUNT {
            return Err("Environment draw capacity".into());
        }
        let floor = self.vertices[self.vertices.len() - 36..].to_vec();
        let mut vertices = self.vertices[..self.body_vertex_count].to_vec();
        for (i, piece) in course.iter().enumerate() {
            for triangle in &piece.geometry.triangles {
                let points =
                    triangle.map(|v| Vec3::from_array(piece.geometry.vertices[v as usize].array()));
                let normal = (points[1] - points[0])
                    .cross(points[2] - points[0])
                    .normalize_or_zero();
                if normal.length_squared() == 0.0 {
                    continue;
                }
                vertices.extend(points.map(|point| Vertex {
                    position: point.to_array(),
                    normal: normal.to_array(),
                    segment: (BODY_COUNT + 1 + i) as u32,
                }));
            }
        }
        vertices.extend(floor);
        self.vertices = vertices;
        self.course = course;
        self.protocol = None;
        Ok(())
    }
    pub fn set_protocol(&mut self) -> Result<(), String> {
        self.set_course(None)?;
        let protocol = lh_model::Protocol::canonical()?;
        let floor = self.vertices.split_off(self.body_vertex_count);
        for (i, piece) in protocol.pieces.iter().enumerate() {
            for triangle in &piece.geometry.triangles {
                let points = triangle.map(|v| {
                    Vec3::from_array(piece.geometry.vertices[v as usize].array())
                        + Vec3::from_array(piece.position.array())
                });
                self.append_triangle(points, (BODY_COUNT + 1 + i) as u32);
            }
        }
        // Show the actual inner room bounds as narrow frame edges so front
        // walls and ceiling do not obscure the subject or the impact apparatus.
        let r = &protocol.room;
        let corners = [
            [-1.0, 0.0, -1.0],
            [1.0, 0.0, -1.0],
            [1.0, 0.0, 1.0],
            [-1.0, 0.0, 1.0],
            [-1.0, 1.0, -1.0],
            [1.0, 1.0, -1.0],
            [1.0, 1.0, 1.0],
            [-1.0, 1.0, 1.0],
        ]
        .map(|v| Vec3::new(v[0] * r.width / 2.0, v[1] * r.height, v[2] * r.depth / 2.0));
        for [a, b] in [
            [0, 1],
            [1, 2],
            [2, 3],
            [3, 0],
            [4, 5],
            [5, 6],
            [6, 7],
            [7, 4],
            [0, 4],
            [1, 5],
            [2, 6],
            [3, 7],
        ] {
            self.append_box(
                (corners[a] + corners[b]) * 0.5,
                (corners[b] - corners[a]).abs() * 0.5 + Vec3::splat(0.009),
                (BODY_COUNT + 5) as u32,
            );
        }
        self.vertices.extend(floor);
        self.protocol = Some(protocol);
        Ok(())
    }
    fn append_triangle(&mut self, points: [Vec3; 3], segment: u32) {
        let normal = (points[1] - points[0])
            .cross(points[2] - points[0])
            .normalize();
        self.vertices.extend(points.map(|p| Vertex {
            position: p.to_array(),
            normal: normal.to_array(),
            segment,
        }));
    }
    fn append_box(&mut self, center: Vec3, half: Vec3, segment: u32) {
        let points = [
            [-1.0, -1.0, -1.0],
            [1.0, -1.0, -1.0],
            [1.0, -1.0, 1.0],
            [-1.0, -1.0, 1.0],
            [-1.0, 1.0, -1.0],
            [1.0, 1.0, -1.0],
            [1.0, 1.0, 1.0],
            [-1.0, 1.0, 1.0],
        ]
        .map(|v| center + Vec3::from_array(v) * half);
        for triangle in [
            [4, 6, 5],
            [4, 7, 6],
            [0, 1, 2],
            [0, 2, 3],
            [0, 5, 1],
            [0, 4, 5],
            [3, 6, 7],
            [3, 2, 6],
            [0, 7, 4],
            [0, 3, 7],
            [1, 6, 2],
            [1, 5, 6],
        ] {
            self.append_triangle(triangle.map(|i| points[i]), segment);
        }
    }
}

pub fn striker_pose(snapshot: &Snapshot) -> Result<Option<Transform>, String> {
    snapshot
        .striker
        .as_ref()
        .map(|s| {
            let p = &s.body;
            let rotation = Quat::from_xyzw(p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.w);
            if p.id != "striker"
                || !p.position.finite()
                || !rotation.is_finite()
                || (rotation.length_squared() - 1.0).abs() > 0.001
            {
                return Err("Invalid striker pose".into());
            }
            Ok(Transform {
                position: Vec3::from_array(p.position.array()),
                rotation,
            })
        })
        .transpose()
}

pub fn environment_poses(
    snapshot: &Snapshot,
    course: &[lh_model::CoursePiece],
) -> Result<Vec<Transform>, String> {
    let poses = snapshot
        .environment
        .as_ref()
        .map(|e| e.pieces.as_slice())
        .unwrap_or(&[]);
    if poses.len() != course.len() {
        return Err("Environment geometry/pose count".into());
    }
    poses
        .iter()
        .zip(course)
        .map(|(pose, definition)| {
            let rotation = Quat::from_xyzw(
                pose.rotation.x,
                pose.rotation.y,
                pose.rotation.z,
                pose.rotation.w,
            );
            if pose.id != definition.id
                || !pose.position.finite()
                || !rotation.is_finite()
                || (rotation.length_squared() - 1.0).abs() > 0.001
            {
                return Err("Invalid environment pose".into());
            }
            Ok(Transform {
                position: Vec3::from_array(pose.position.array()),
                rotation,
            })
        })
        .collect()
}
// A checked three-element conversion avoids assuming the source slice length.
trait TriangleMap {
    fn map_array<T>(&self, f: impl Fn(u32) -> T) -> [T; 3];
}
impl TriangleMap for [u32] {
    fn map_array<T>(&self, f: impl Fn(u32) -> T) -> [T; 3] {
        [f(self[0]), f(self[1]), f(self[2])]
    }
}

#[derive(Clone, Copy)]
pub struct Transform {
    pub position: Vec3,
    pub rotation: Quat,
}
impl Transform {
    pub fn matrix(self) -> Mat4 {
        Mat4::from_rotation_translation(self.rotation, self.position)
    }
    pub fn interpolate(self, other: Self, alpha: f32) -> Self {
        Self {
            position: self.position.lerp(other.position, alpha),
            rotation: self.rotation.slerp(other.rotation, alpha).normalize(),
        }
    }
}
pub fn physical_poses(
    snapshot: &Snapshot,
    packed: &[f32],
    model: &Model,
) -> Result<[Transform; BODY_COUNT], String> {
    if snapshot.segments.len() != BODY_COUNT
        || packed.len() != BODY_COUNT * 13
        || packed.iter().any(|v| !v.is_finite())
    {
        return Err("Invalid pose buffer".into());
    }
    let mut result = [Transform {
        position: Vec3::ZERO,
        rotation: Quat::IDENTITY,
    }; BODY_COUNT];
    for (i, pose) in snapshot.segments.iter().enumerate() {
        if pose.id != model.segments[i].id {
            return Err("Pose/geometry identity mismatch".into());
        }
        let p = &packed[i * 13..(i + 1) * 13];
        let q = Quat::from_xyzw(p[3], p[4], p[5], p[6]);
        if (q.length_squared() - 1.0).abs() > 0.001 {
            return Err("Non-unit physical quaternion".into());
        }
        let expected = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.rotation.x,
            pose.rotation.y,
            pose.rotation.z,
            pose.rotation.w,
            pose.linear_velocity.x,
            pose.linear_velocity.y,
            pose.linear_velocity.z,
            pose.angular_velocity.x,
            pose.angular_velocity.y,
            pose.angular_velocity.z,
        ];
        if p.iter()
            .zip(expected)
            .any(|(a, b)| a.to_bits() != b.to_bits())
        {
            return Err("Packed/JSON physical pose disagreement".into());
        }
        result[i] = Transform {
            position: Vec3::from_slice(p),
            rotation: q,
        };
    }
    Ok(result)
}

pub struct Camera {
    pub target: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
}
impl Default for Camera {
    fn default() -> Self {
        Self {
            target: Vec3::new(0.0, 0.92, 0.0),
            yaw: 0.65,
            pitch: 0.22,
            distance: 3.6,
        }
    }
}
impl Camera {
    pub fn eye(&self) -> Vec3 {
        self.target
            + Vec3::new(
                self.yaw.sin() * self.pitch.cos(),
                self.pitch.sin(),
                self.yaw.cos() * self.pitch.cos(),
            ) * self.distance
    }
    pub fn matrix(&self, aspect: f32) -> Mat4 {
        // Preserve horizontal framing on narrow screens. Picking uses the
        // same matrix, so camera fitting cannot change surface correspondence.
        let fov = 2.0 * (24.0_f32.to_radians().tan() / aspect.min(1.0)).atan();
        Mat4::perspective_rh(fov, aspect, 0.05, 80.0)
            * Mat4::look_at_rh(self.eye(), self.target, Vec3::Y)
    }
    pub fn ray(&self, x: f32, y: f32, width: f32, height: f32) -> (Vec3, Vec3) {
        let inverse = self.matrix(width / height).inverse();
        let ndc = Vec3::new(x / width * 2.0 - 1.0, 1.0 - y / height * 2.0, 0.0);
        let near = inverse.project_point3(ndc);
        let far = inverse.project_point3(Vec3::new(ndc.x, ndc.y, 1.0));
        (near, (far - near).normalize())
    }
}
#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Pick {
    pub segment: usize,
    pub segment_id: String,
    pub region: Option<String>,
    pub local_anchor: [f32; 3],
    pub world_point: [f32; 3],
    pub plane_normal: [f32; 3],
    pub triangle: usize,
}
pub fn pick(
    mesh: &Mesh,
    poses: &[Transform; BODY_COUNT],
    origin: Vec3,
    direction: Vec3,
    selected: &str,
) -> Option<Pick> {
    let mut closest = f32::INFINITY;
    let mut found = None;
    for (i, part) in mesh.model.segments.iter().enumerate() {
        let inverse = poses[i].matrix().inverse();
        let local_origin = inverse.transform_point3(origin);
        let local_direction = inverse.transform_vector3(direction);
        for (triangle, face) in part.geometry.indices.chunks_exact(3).enumerate() {
            let [a, b, c] = face.map_array(|index| {
                Vec3::from_slice(
                    &part.geometry.vertices[index as usize * 3..index as usize * 3 + 3],
                )
            });
            let edge1 = b - a;
            let edge2 = c - a;
            let cross = local_direction.cross(edge2);
            let determinant = edge1.dot(cross);
            if determinant.abs() < 1e-8 {
                continue;
            }
            let relative = local_origin - a;
            let u = relative.dot(cross) / determinant;
            if !(0.0..=1.0).contains(&u) {
                continue;
            }
            let cross2 = relative.cross(edge1);
            let v = local_direction.dot(cross2) / determinant;
            if v < 0.0 || u + v > 1.0 {
                continue;
            }
            let distance = edge2.dot(cross2) / determinant;
            if distance > 0.0 && distance < closest {
                closest = distance;
                let point = origin + direction * distance;
                found = Some(Pick {
                    segment: i,
                    segment_id: part.id.clone(),
                    region: part.region.clone(),
                    local_anchor: (local_origin + local_direction * distance).to_array(),
                    world_point: point.to_array(),
                    plane_normal: (-direction).to_array(),
                    triangle,
                });
            }
        }
    }
    found.filter(|hit| selected.is_empty() || hit.region.as_deref() == Some(selected))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn portrait_and_landscape_protocol_views_contain_the_stowed_machine_at_every_yaw() {
        let protocol = lh_model::Protocol::canonical().unwrap();
        let camera = Camera {
            target: Vec3::new(0.0, 1.65, 1.1),
            distance: 6.8,
            ..Default::default()
        };
        for aspect in [0.6, 358.0 / 440.0, 1.3, 1.8] {
            for yaw in 0..16 {
                let rotation = Quat::from_rotation_y(yaw as f32 * std::f32::consts::TAU / 16.0);
                for piece in &protocol.pieces {
                    for vertex in &piece.geometry.vertices {
                        let world = rotation
                            * (Vec3::from_array(vertex.array())
                                + Vec3::from_array(piece.position.array()))
                            + Vec3::new(0.0, protocol.stow_height, 2.1);
                        let projected = camera.matrix(aspect).project_point3(world);
                        assert!(
                            projected.x.abs() < 0.95
                                && projected.y.abs() < 0.95
                                && (0.0..1.0).contains(&projected.z),
                            "{aspect}: {projected:?}"
                        );
                    }
                }
            }
        }
    }
    #[test]
    fn protocol_mesh_preserves_canonical_machine_surfaces_and_room_bounds() {
        let mut mesh = Mesh::canonical().unwrap();
        mesh.set_protocol().unwrap();
        let protocol = mesh.protocol.as_ref().unwrap();
        let mut index = mesh.body_vertex_count;
        assert!(mesh.course.is_empty());
        for (i, piece) in protocol.pieces.iter().enumerate() {
            for vertex in piece.geometry.triangles.iter().flatten() {
                let point = Vec3::from_array(piece.geometry.vertices[*vertex as usize].array())
                    + Vec3::from_array(piece.position.array());
                assert_eq!(mesh.vertices[index].position, point.to_array());
                assert_eq!(mesh.vertices[index].segment, (BODY_COUNT + 1 + i) as u32);
                index += 1;
            }
        }
        assert_eq!(mesh.vertices.len() - index, 12 * 36 + 36);
        assert!(
            mesh.vertices[index..mesh.vertices.len() - 36]
                .iter()
                .all(|v| v.segment == (BODY_COUNT + 5) as u32)
        );
        mesh.set_course(Some(Default::default())).unwrap();
        assert!(mesh.protocol.is_none());
        assert_eq!(mesh.course.len(), 26);
    }
    #[test]
    fn course_meshes_use_canonical_triangles_and_preserve_body_and_floor_slots() {
        use lh_contracts::{Difficulty, PlaygroundSettings, Station};
        let mut mesh = Mesh::canonical().unwrap();
        let original = bytemuck::cast_slice::<_, u8>(&mesh.vertices).to_vec();
        for difficulty in [
            Difficulty::Gentle,
            Difficulty::Challenging,
            Difficulty::Extreme,
        ] {
            mesh.set_course(Some(PlaygroundSettings {
                station: Station::Flat,
                difficulty,
            }))
            .unwrap();
            assert_eq!(mesh.course.len(), 26);
            let mut offset = mesh.body_vertex_count;
            for (i, piece) in mesh.course.iter().enumerate() {
                for triangle in &piece.geometry.triangles {
                    let points = triangle
                        .map(|v| Vec3::from_array(piece.geometry.vertices[v as usize].array()));
                    if (points[1] - points[0])
                        .cross(points[2] - points[0])
                        .normalize_or_zero()
                        .length_squared()
                        == 0.0
                    {
                        continue;
                    }
                    for point in points {
                        assert_eq!(mesh.vertices[offset].position, point.to_array());
                        assert_eq!(mesh.vertices[offset].segment, (BODY_COUNT + 1 + i) as u32);
                        offset += 1;
                    }
                }
            }
            assert_eq!(mesh.vertices.len() - offset, 36);
            assert!(
                mesh.vertices[offset..]
                    .iter()
                    .all(|v| v.segment == BODY_COUNT as u32)
            );
        }
        mesh.set_course(None).unwrap();
        assert_eq!(bytemuck::cast_slice::<_, u8>(&mesh.vertices), original);
    }
    #[test]
    fn uploaded_triangles_preserve_every_canonical_vertex_and_index() {
        let mesh = Mesh::canonical().expect("mesh");
        let mut offset = 0;
        for (i, s) in mesh.model.segments.iter().enumerate() {
            for index in &s.geometry.indices {
                assert_eq!(
                    mesh.vertices[offset].position,
                    s.geometry.vertices[*index as usize * 3..*index as usize * 3 + 3]
                );
                assert_eq!(mesh.vertices[offset].segment, i as u32);
                offset += 1;
            }
        }
        assert_eq!(mesh.vertices.len() - offset, 36);
    }
    #[test]
    fn camera_ray_picks_an_actual_surface_with_the_exact_segment_local_anchor() {
        let mesh = Mesh::canonical().expect("mesh");
        let poses = std::array::from_fn(|i| {
            let p = &mesh.model.segments[i].initial;
            Transform {
                position: Vec3::from_array(p.position.array()),
                rotation: Quat::from_xyzw(p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.w),
            }
        });
        let camera = Camera::default();
        let point = camera.matrix(1.5).project_point3(poses[4].position);
        let (origin, direction) = camera.ray(
            (point.x + 1.0) * 450.0,
            (1.0 - point.y) * 300.0,
            900.0,
            600.0,
        );
        let hit = pick(&mesh, &poses, origin, direction, "head").expect("head");
        assert_eq!(hit.segment_id, "head");
        assert!(
            (poses[hit.segment]
                .matrix()
                .transform_point3(Vec3::from_array(hit.local_anchor))
                - Vec3::from_array(hit.world_point))
            .length()
                < 1e-5
        );
    }
    #[test]
    fn selected_regions_do_not_pick_through_a_nearer_canonical_surface() {
        let mesh = Mesh::canonical().expect("mesh");
        let mut poses = [Transform {
            position: Vec3::new(100.0, 0.0, 0.0),
            rotation: Quat::IDENTITY,
        }; BODY_COUNT];
        let head = mesh
            .model
            .segments
            .iter()
            .position(|s| s.id == "head")
            .unwrap();
        let torso = mesh
            .model
            .segments
            .iter()
            .position(|s| s.region.as_deref() == Some("torso"))
            .unwrap();
        poses[head].position = Vec3::new(0.0, 0.0, 1.0);
        poses[torso].position = Vec3::ZERO;
        let origin = Vec3::new(0.0, 0.0, 3.0);
        assert_eq!(
            pick(&mesh, &poses, origin, -Vec3::Z, "").unwrap().segment,
            head
        );
        assert!(pick(&mesh, &poses, origin, -Vec3::Z, "torso").is_none());
        assert_eq!(
            pick(&mesh, &poses, origin, -Vec3::Z, "head")
                .unwrap()
                .segment,
            head
        );
    }
}
