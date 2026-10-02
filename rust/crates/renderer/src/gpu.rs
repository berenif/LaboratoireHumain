use crate::*;
use wasm_bindgen::prelude::*;
use web_sys::HtmlCanvasElement;
use wgpu::util::DeviceExt;

const SHADER: &str = r#"
struct Uniforms {
    view_projection: mat4x4<f32>,
    models: array<mat4x4<f32>, 64>,
    colors: array<vec4<f32>, 64>,
    light: vec4<f32>,
};
@group(0) @binding(0) var<uniform> scene: Uniforms;
struct Input { @location(0) position: vec3<f32>, @location(1) normal: vec3<f32>, @location(2) segment: u32 };
struct Output {
    @builtin(position) clip: vec4<f32>, @location(0) world: vec3<f32>,
    @location(1) normal: vec3<f32>, @location(2) color: vec4<f32>,
    @location(3) @interpolate(flat) segment: u32,
};
@vertex fn vertex(input: Input) -> Output {
    var output: Output;
    let model = scene.models[input.segment];
    let world = model * vec4(input.position, 1.0);
    output.clip = scene.view_projection * world;
    output.world = world.xyz;
    output.normal = (model * vec4(input.normal,0.0)).xyz;
    output.color = scene.colors[input.segment];
    output.segment = input.segment;
    return output;
}
@fragment fn fragment(input: Output) -> @location(0) vec4<f32> {
    let diffuse = max(dot(normalize(input.normal), normalize(scene.light.xyz)), 0.0);
    let coordinate = input.world.xz * 2.0;
    let coordinate_width = max(fwidth(coordinate), vec2(0.001));
    var color = input.color.rgb * (scene.light.w + 0.62 * diffuse);
    if input.segment == 25u && input.normal.y > 0.5 {
        let edge = abs(fract(coordinate - 0.5) - 0.5) / coordinate_width;
        let grid = 1.0 - min(min(edge.x,edge.y),1.0);
        color *= 1.0 - 0.075 * grid;
    }
    return vec4(color,1.0);
}
"#;
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Uniforms {
    view_projection: [f32; 16],
    models: [[f32; 16]; DRAW_COUNT],
    colors: [[f32; 4]; DRAW_COUNT],
    light: [f32; 4],
}
struct Gpu {
    canvas: HtmlCanvasElement,
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    render_format: wgpu::TextureFormat,
    pipeline: wgpu::RenderPipeline,
    vertices: wgpu::Buffer,
    uniform: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    depth: wgpu::TextureView,
    backend: String,
    adapter: serde_json::Value,
}
fn depth(device: &wgpu::Device, width: u32, height: u32) -> wgpu::TextureView {
    device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some("physical depth"),
            size: wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth24Plus,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        })
        .create_view(&Default::default())
}
fn error(message: impl std::fmt::Display) -> JsValue {
    JsValue::from_str(&message.to_string())
}

#[wasm_bindgen]
pub struct BrowserRenderer {
    gpu: Option<Gpu>,
    mesh: Mesh,
    camera: Camera,
    previous: [Transform; BODY_COUNT],
    current: [Transform; BODY_COUNT],
    displayed: [Transform; BODY_COUNT],
    environment_previous: Vec<Transform>,
    environment_current: Vec<Transform>,
    environment_displayed: Vec<Transform>,
    striker_previous: Option<Transform>,
    striker_current: Option<Transform>,
    striker_displayed: Option<Transform>,
    snapshot: Option<Snapshot>,
    arrival_ms: f64,
    interval_s: f64,
    selected: String,
    frames: u64,
    floor_enabled: bool,
}
impl BrowserRenderer {
    pub async fn create(canvas: HtmlCanvasElement, requested: &str) -> Result<Self, JsValue> {
        let backends = match requested {
            "webgpu" => wgpu::Backends::BROWSER_WEBGPU,
            "webgl2" => wgpu::Backends::GL,
            _ => return Err(error("Unknown graphics backend")),
        };
        let mesh = Mesh::canonical().map_err(error)?;
        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
            backends,
            ..Default::default()
        });
        let surface: wgpu::Surface<'static> = instance
            .create_surface(wgpu::SurfaceTarget::Canvas(canvas.clone()))
            .map_err(error)?;
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                force_fallback_adapter: false,
                compatible_surface: Some(&surface),
            })
            .await
            .map_err(error)?;
        let info = adapter.get_info();
        let adapter_details = serde_json::json!({"name":info.name,"vendor":info.vendor,"device":info.device,
            "deviceType":format!("{:?}",info.device_type),"driver":info.driver,"driverInfo":info.driver_info});
        let backend = match info.backend {
            wgpu::Backend::BrowserWebGpu => "WebGPU",
            wgpu::Backend::Gl => "WebGL2",
            other => return Err(error(format!("Unexpected graphics backend {other:?}"))),
        }
        .to_string();
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("presentation only"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::downlevel_webgl2_defaults(),
                ..Default::default()
            })
            .await
            .map_err(error)?;
        device.on_uncaptured_error(std::sync::Arc::new(|error: wgpu::Error| {
            web_sys::console::error_1(&JsValue::from_str(&error.to_string()));
            if let (Some(window), Ok(event)) = (
                web_sys::window(),
                web_sys::CustomEvent::new("lh-graphics-lost"),
            ) {
                let _ = window.dispatch_event(&event);
            }
        }));
        device.set_device_lost_callback(|reason, _| {
            if reason == wgpu::DeviceLostReason::Destroyed {
                return;
            }
            if let (Some(window), Ok(event)) = (
                web_sys::window(),
                web_sys::CustomEvent::new("lh-graphics-lost"),
            ) {
                let _ = window.dispatch_event(&event);
            }
        });
        device.push_error_scope(wgpu::ErrorFilter::Validation);
        let capabilities = surface.get_capabilities(&adapter);
        let format = capabilities
            .formats
            .iter()
            .copied()
            .find(|f| f.is_srgb())
            .unwrap_or(capabilities.formats[0]);
        // WebGPU canvas formats are linear UNORM; WebGL exposes sRGB formats.
        // An sRGB attachment view gives both the same linear-light shading and
        // output transfer, without backend-specific color math in the shader.
        let render_format = format.add_srgb_suffix();
        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format,
            width: 1,
            height: 1,
            present_mode: wgpu::PresentMode::Fifo,
            desired_maximum_frame_latency: 2,
            alpha_mode: capabilities.alpha_modes[0],
            view_formats: if render_format == format {
                vec![]
            } else {
                vec![render_format]
            },
        };
        surface.configure(&device, &config);
        let uniform = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("physical pose uniforms"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("pose layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("poses"),
            layout: &layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform.as_entire_binding(),
            }],
        });
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("canonical surfaces"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("mesh layout"),
            bind_group_layouts: &[&layout],
            push_constant_ranges: &[],
        });
        let attributes = wgpu::vertex_attr_array![0=>Float32x3,1=>Float32x3,2=>Uint32];
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("anatomical meshes"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vertex"),
                compilation_options: Default::default(),
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &attributes,
                }],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fragment"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: render_format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth24Plus,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: Default::default(),
            multiview: None,
            cache: None,
        });
        let vertices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("exact canonical triangles"),
            contents: bytemuck::cast_slice(&mesh.vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let depth = depth(&device, 1, 1);
        if let Some(validation_error) = device.pop_error_scope().await {
            vertices.destroy();
            uniform.destroy();
            device.destroy();
            return Err(error(validation_error));
        }
        let empty = [Transform {
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
        }; BODY_COUNT];
        Ok(Self {
            gpu: Some(Gpu {
                canvas,
                surface,
                device,
                queue,
                config,
                render_format,
                pipeline,
                vertices,
                uniform,
                bind_group,
                depth,
                backend,
                adapter: adapter_details,
            }),
            mesh,
            camera: Camera::default(),
            previous: empty,
            current: empty,
            displayed: empty,
            environment_previous: Vec::new(),
            environment_current: Vec::new(),
            environment_displayed: Vec::new(),
            striker_previous: None,
            striker_current: None,
            striker_displayed: None,
            snapshot: None,
            arrival_ms: 0.0,
            interval_s: 0.0,
            selected: String::new(),
            frames: 0,
            floor_enabled: true,
        })
    }
}
#[wasm_bindgen]
impl BrowserRenderer {
    pub fn accept_snapshot(
        &mut self,
        json: &str,
        packed: &[f32],
        paused: bool,
        now_ms: f64,
    ) -> Result<(), JsValue> {
        let snapshot: Snapshot = serde_json::from_str(json).map_err(error)?;
        let poses = physical_poses(&snapshot, packed, &self.mesh.model).map_err(error)?;
        if self.snapshot.as_ref().is_some_and(|old| {
            snapshot.stamp.generation < old.stamp.generation
                || snapshot.stamp.generation == old.stamp.generation
                    && snapshot.stamp.sequence <= old.stamp.sequence
        }) {
            return Err(error("Stale presentation observation"));
        }
        let settings = snapshot.environment.as_ref().map(|e| e.settings);
        let old_settings = self
            .snapshot
            .as_ref()
            .and_then(|s| s.environment.as_ref().map(|e| e.settings));
        let striker = striker_pose(&snapshot).map_err(error)?;
        if striker.is_some() && settings.is_some() {
            return Err(error("Mixed protocol/playground scene"));
        }
        let changed_geometry = settings.map(|s| s.difficulty) != old_settings.map(|s| s.difficulty)
            || striker.is_some() != self.mesh.protocol.is_some();
        if changed_geometry {
            let mut mesh = Mesh::canonical().map_err(error)?;
            mesh.set_course(settings).map_err(error)?;
            if striker.is_some() {
                mesh.set_protocol().map_err(error)?;
            }
            environment_poses(&snapshot, &mesh.course).map_err(error)?;
            let gpu = self
                .gpu
                .as_mut()
                .ok_or_else(|| error("Renderer disposed"))?;
            let vertices = gpu
                .device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("canonical body and environment"),
                    contents: bytemuck::cast_slice(&mesh.vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                });
            gpu.vertices.destroy();
            gpu.vertices = vertices;
            self.mesh = mesh;
        }
        let environment = environment_poses(&snapshot, &self.mesh.course).map_err(error)?;
        let new_trial = self
            .snapshot
            .as_ref()
            .is_none_or(|old| old.stamp.generation != snapshot.stamp.generation);
        if new_trial {
            self.camera.target = poses[0].position;
            if changed_geometry {
                self.camera.distance = if striker.is_some() { 6.8 } else { 3.6 };
            }
            if striker.is_some() {
                self.camera.target.y = 1.65;
                self.camera.target.z += 1.1;
            }
        }
        self.environment_previous =
            if new_trial || paused || snapshot.discontinuity || changed_geometry {
                environment.clone()
            } else {
                self.environment_displayed.clone()
            };
        self.environment_current = environment;
        self.environment_displayed = self.environment_previous.clone();
        self.striker_previous = if new_trial || paused || snapshot.discontinuity || changed_geometry
        {
            striker
        } else {
            self.striker_displayed
        };
        self.striker_current = striker;
        self.striker_displayed = self.striker_previous;
        if let Some(old) = &self.snapshot {
            if snapshot.stamp.generation < old.stamp.generation
                || (snapshot.stamp.generation == old.stamp.generation
                    && snapshot.stamp.sequence <= old.stamp.sequence)
            {
                return Err(error("Stale presentation observation"));
            }
            self.interval_s = (snapshot.simulation_time_s - old.simulation_time_s).max(0.0);
            let snap = paused
                || snapshot.discontinuity
                || snapshot.stamp.generation != old.stamp.generation
                || self.interval_s == 0.0;
            self.previous = if snap { poses } else { self.displayed };
        } else {
            self.previous = poses;
        }
        self.current = poses;
        self.displayed = self.previous;
        self.arrival_ms = now_ms;
        self.snapshot = Some(snapshot);
        Ok(())
    }
    pub fn resize(&mut self, width: f32, height: f32, pixel_ratio: f32) -> Result<(), JsValue> {
        if !width.is_finite()
            || !height.is_finite()
            || !pixel_ratio.is_finite()
            || width <= 0.0
            || height <= 0.0
            || pixel_ratio <= 0.0
        {
            return Err(error("Invalid viewport"));
        }
        let gpu = self
            .gpu
            .as_mut()
            .ok_or_else(|| error("Disposed graphics"))?;
        let limit = gpu.device.limits().max_texture_dimension_2d as f32;
        let factor = (limit / (width * pixel_ratio))
            .min(limit / (height * pixel_ratio))
            .min(1.0);
        let w = (width * pixel_ratio * factor).round().max(1.0) as u32;
        let h = (height * pixel_ratio * factor).round().max(1.0) as u32;
        if gpu.config.width != w || gpu.config.height != h {
            gpu.canvas.set_width(w);
            gpu.canvas.set_height(h);
            gpu.config.width = w;
            gpu.config.height = h;
            gpu.surface.configure(&gpu.device, &gpu.config);
            gpu.depth = depth(&gpu.device, w, h);
        }
        Ok(())
    }
    pub fn draw(&mut self, now_ms: f64) -> Result<bool, JsValue> {
        if self.snapshot.is_none() {
            return Ok(false);
        }
        let gpu = self
            .gpu
            .as_mut()
            .ok_or_else(|| error("Disposed graphics"))?;
        let alpha = if self.interval_s > 0.0 {
            ((now_ms - self.arrival_ms) / 1000.0 / self.interval_s).clamp(0.0, 1.0) as f32
        } else {
            1.0
        };
        self.displayed =
            std::array::from_fn(|i| self.previous[i].interpolate(self.current[i], alpha));
        let mut uniforms = Uniforms {
            view_projection: self
                .camera
                .matrix(gpu.config.width as f32 / gpu.config.height as f32)
                .to_cols_array(),
            models: [[0.0; 16]; DRAW_COUNT],
            colors: [[0.0; 4]; DRAW_COUNT],
            light: [-0.35, 0.8, 0.5, 0.48],
        };
        for (i, pose) in self.displayed.iter().enumerate() {
            uniforms.models[i] = pose.matrix().to_cols_array();
            let part = &self.mesh.model.segments[i];
            uniforms.colors[i] = match part.region.as_deref() {
                Some("head") => [0.92, 0.44, 0.22, 1.0],
                Some("pelvis") => [0.27, 0.51, 0.48, 1.0],
                Some("torso") => [0.39, 0.63, 0.57, 1.0],
                _ => {
                    if part.id.starts_with("left") {
                        [0.55, 0.72, 0.69, 1.0]
                    } else {
                        [0.28, 0.53, 0.54, 1.0]
                    }
                }
            };
            if !self.selected.is_empty() && part.region.as_deref() == Some(&self.selected) {
                uniforms.colors[i] = [0.96, 0.68, 0.28, 1.0];
            }
        }
        uniforms.models[BODY_COUNT] = Mat4::IDENTITY.to_cols_array();
        uniforms.colors[BODY_COUNT] = [0.80, 0.77, 0.69, 1.0];
        self.environment_displayed = self
            .environment_previous
            .iter()
            .zip(&self.environment_current)
            .map(|(a, b)| a.interpolate(*b, alpha))
            .collect();
        for (i, pose) in self.environment_displayed.iter().enumerate() {
            let slot = BODY_COUNT + 1 + i;
            uniforms.models[slot] = pose.matrix().to_cols_array();
            let rgb = u32::from_str_radix(self.mesh.course[i].color.trim_start_matches('#'), 16)
                .map_err(error)?;
            uniforms.colors[slot] = [
                ((rgb >> 16) & 255) as f32 / 255.0,
                ((rgb >> 8) & 255) as f32 / 255.0,
                (rgb & 255) as f32 / 255.0,
                1.0,
            ];
        }
        self.striker_displayed = self
            .striker_previous
            .zip(self.striker_current)
            .map(|(a, b)| a.interpolate(b, alpha));
        if let Some(pose) = self.striker_displayed {
            let protocol = self
                .mesh
                .protocol
                .as_ref()
                .ok_or_else(|| error("Missing striker geometry"))?;
            for (i, piece) in protocol.pieces.iter().enumerate() {
                let slot = BODY_COUNT + 1 + i;
                uniforms.models[slot] = pose.matrix().to_cols_array();
                let rgb =
                    u32::from_str_radix(piece.color.trim_start_matches('#'), 16).map_err(error)?;
                uniforms.colors[slot] = [
                    ((rgb >> 16) & 255) as f32 / 255.0,
                    ((rgb >> 8) & 255) as f32 / 255.0,
                    (rgb & 255) as f32 / 255.0,
                    1.0,
                ];
            }
            uniforms.models[BODY_COUNT + 5] = Mat4::IDENTITY.to_cols_array();
            uniforms.colors[BODY_COUNT + 5] = [0.48, 0.57, 0.52, 1.0];
        }
        gpu.queue
            .write_buffer(&gpu.uniform, 0, bytemuck::bytes_of(&uniforms));
        let frame = gpu.surface.get_current_texture().map_err(error)?;
        let view = frame.texture.create_view(&wgpu::TextureViewDescriptor {
            format: Some(gpu.render_format),
            ..Default::default()
        });
        let mut encoder = gpu
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("presentation frame"),
            });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("body and floor"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    depth_slice: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.82,
                            g: 0.80,
                            b: 0.75,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &gpu.depth,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            pass.set_pipeline(&gpu.pipeline);
            pass.set_bind_group(0, &gpu.bind_group, &[]);
            pass.set_vertex_buffer(0, gpu.vertices.slice(..));
            let count = self.mesh.vertices.len() - if self.floor_enabled { 0 } else { 36 };
            pass.draw(0..count as u32, 0..1);
        }
        gpu.queue.submit([encoder.finish()]);
        frame.present();
        self.frames += 1;
        Ok(true)
    }
    pub fn select_region(&mut self, region: &str) {
        self.selected = region.into();
    }
    pub fn set_floor_enabled(&mut self, enabled: bool) {
        self.floor_enabled = enabled;
    }
    pub fn orbit(&mut self, yaw: f32, pitch: f32) {
        self.camera.yaw += yaw;
        self.camera.pitch = (self.camera.pitch + pitch).clamp(-0.05, 1.3);
    }
    pub fn zoom(&mut self, factor: f32) {
        if factor.is_finite() && factor > 0.0 {
            self.camera.distance = (self.camera.distance * factor).clamp(1.5, 12.0);
        }
    }
    pub fn reset_camera(&mut self) {
        self.camera = Camera::default();
        if self.snapshot.is_some() {
            self.camera.target.x = self.displayed[0].position.x;
            self.camera.target.z = self.displayed[0].position.z;
            if self.mesh.protocol.is_some() {
                self.camera.distance = 6.8;
                self.camera.target.y = 1.65;
                self.camera.target.z += 1.1;
            }
        }
    }
    pub fn focus_body(&mut self) {
        if self.snapshot.is_some() {
            self.camera.target = self
                .displayed
                .iter()
                .zip(&self.mesh.model.segments)
                .map(|(pose, part)| pose.position * part.mass_kg)
                .sum::<Vec3>()
                / self.mesh.model.total_mass_kg;
            self.camera.distance = 3.0;
        }
    }
    pub fn pick(&self, x: f32, y: f32, width: f32, height: f32) -> Result<String, JsValue> {
        let (origin, direction) = self.camera.ray(x, y, width, height);
        serde_json::to_string(&crate::pick(
            &self.mesh,
            &self.displayed,
            origin,
            direction,
            &self.selected,
        ))
        .map_err(error)
    }
    pub fn drag_target(
        &self,
        x: f32,
        y: f32,
        width: f32,
        height: f32,
        point: &[f32],
        normal: &[f32],
    ) -> Result<Vec<f32>, JsValue> {
        if point.len() != 3 || normal.len() != 3 {
            return Err(error("Invalid drag plane"));
        }
        let (origin, direction) = self.camera.ray(x, y, width, height);
        let normal = Vec3::from_slice(normal);
        let denominator = direction.dot(normal);
        if denominator.abs() < 1e-5 {
            return Err(error("Parallel drag ray"));
        }
        let distance = (Vec3::from_slice(point) - origin).dot(normal) / denominator;
        if distance <= 0.0 {
            return Err(error("Drag plane behind camera"));
        }
        Ok((origin + direction * distance).to_array().to_vec())
    }
    pub fn project_segment(
        &self,
        segment: usize,
        width: f32,
        height: f32,
    ) -> Result<Vec<f32>, JsValue> {
        if segment >= BODY_COUNT {
            return Err(error("Unknown segment"));
        }
        let p = self
            .camera
            .matrix(width / height)
            .project_point3(self.displayed[segment].position);
        Ok(vec![(p.x + 1.0) * width * 0.5, (1.0 - p.y) * height * 0.5])
    }
    pub fn diagnostics(&self) -> Result<String, JsValue> {
        serde_json::to_string(&serde_json::json!({"backend":self.gpu.as_ref().map(|g| &g.backend),"renderedSegments":if self.snapshot.is_some(){BODY_COUNT}else{0},
            "renderedEnvironmentPieces":self.environment_displayed.len(),
            "renderedStrikerPieces":if self.striker_displayed.is_some(){4}else{0},
            "strikerPosition":self.striker_displayed.map(|p|p.position.to_array()),
            "vertices":self.mesh.vertices.len(),"drawCalls":1,"frames":self.frames,"generation":self.snapshot.as_ref().map(|s|s.stamp.generation),
            "tick":self.snapshot.as_ref().map(|s|s.stamp.tick),"floorEnabled":self.floor_enabled,
            "camera":{"yaw":self.camera.yaw,"pitch":self.camera.pitch,"distance":self.camera.distance},
            "width":self.gpu.as_ref().map(|g|g.config.width),"height":self.gpu.as_ref().map(|g|g.config.height),
            "renderFormat":self.gpu.as_ref().map(|g|format!("{:?}",g.render_format)),
            "adapter":self.gpu.as_ref().map(|g|&g.adapter)})).map_err(error)
    }
    pub fn dispose(&mut self) {
        if let Some(gpu) = self.gpu.take() {
            gpu.vertices.destroy();
            gpu.uniform.destroy();
            gpu.device.destroy();
        }
        self.snapshot = None;
    }
}
