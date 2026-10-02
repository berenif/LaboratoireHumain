//! Kinematic impact apparatus. Only current solver contacts create an impact;
//! positioning/retraction never write anatomical poses or inject body impulses.
use crate::{quat, vec3, vector};
use lh_contracts::{StrikePhase, StrikerSnapshot};
use lh_model::Protocol;
use rapier3d::{parry::query, prelude::*};
use std::collections::HashMap;

const SPEED: f32 = 4.0;
const STANDOFF: f32 = 1.28;
const MIN_STANDOFF: f32 = 0.72;
const TRAVEL: f32 = 1.60;
const OVERHEAD: f32 = 0.03;
const DESCEND: f32 = 0.03;
const BACK: f32 = 0.08;
const RISE: f32 = 0.06;
const HOME_TIME: f32 = 0.08;
const MARGIN: f32 = 0.02;

/// Actual whole-body impacts at the three standing headings. No recovery or
/// five-cycle admission is inferred from this short physical diagnostic.
pub fn trials(profile: &crate::Profile) -> Result<Vec<serde_json::Value>, String> {
    let mut reports = Vec::new();
    for heading in [
        0.0,
        std::f32::consts::FRAC_PI_3,
        -std::f32::consts::FRAC_PI_4,
    ] {
        let mut s = crate::Simulation::new_protocol(heading, profile.clone(), true)?;
        let handles = s.bodies.clone();
        let mut failure = None;
        for _ in 0..120 {
            if let Err(e) = s.advance_tick() {
                failure = Some(e);
                break;
            }
        }
        let initial = s.snapshot();
        let requested = failure.is_none() && s.request_strike()?;
        let mut min_pelvis_y = f32::INFINITY;
        let mut trace = Vec::new();
        if failure.is_none() {
            for _ in 0..180 {
                let result = s.advance_tick();
                let snapshot = s.snapshot();
                min_pelvis_y = min_pelvis_y.min(snapshot.segments[0].position.y);
                trace.push(snapshot);
                if let Err(e) = result {
                    failure = Some(e);
                    break;
                }
            }
        }
        let final_snapshot = s.snapshot();
        let ownership =
            s.bodies == handles && s.bodies.iter().all(|h| s.world.bodies[*h].is_dynamic());
        let passed = requested
            && failure.is_none()
            && ownership
            && final_snapshot.counters.strikes == 1
            && final_snapshot.counters.falls == 1
            && final_snapshot.striker.as_ref().is_some_and(|p| p.available)
            && min_pelvis_y < 0.56;
        reports.push(serde_json::json!({"heading":heading,"requested":requested,"initial":initial,
            "final":final_snapshot,"failure":failure,"minPelvisY":min_pelvis_y,"dynamicOwnership":ownership,
            "trace":trace,"passed":passed}));
    }
    Ok(reports)
}

pub(crate) struct Striker {
    definition: Protocol,
    body: RigidBodyHandle,
    pub collider: ColliderHandle,
    pub room_colliders: Vec<ColliderHandle>,
    vertices: Vec<Vector>,
    parts: Vec<(Vector, Vector)>,
    phase: StrikePhase,
    phase_time: f32,
    position: Vector,
    rotation: Rotation,
    start: Vector,
    origin: Vector,
    direction: Vector,
    travel: f32,
    travel_limit: f32,
    obstructed: bool,
    impact_id: u32,
    impacted: bool,
    last_impact_impulse: f32,
}
impl Striker {
    pub fn new(world: &mut PhysicsWorld) -> Result<Self, String> {
        let definition = Protocol::canonical()?;
        let room = &definition.room;
        let mut room_colliders = Vec::new();
        let thickness = 0.16;
        let groups =
            InteractionGroups::new(Group::GROUP_2, Group::GROUP_1, InteractionTestMode::And);
        for (position, half) in [
            (
                Vector::new(-room.width / 2.0 - thickness, room.height / 2.0, 0.0),
                Vector::new(thickness, room.height / 2.0, room.depth / 2.0 + thickness),
            ),
            (
                Vector::new(room.width / 2.0 + thickness, room.height / 2.0, 0.0),
                Vector::new(thickness, room.height / 2.0, room.depth / 2.0 + thickness),
            ),
            (
                Vector::new(0.0, room.height / 2.0, -room.depth / 2.0 - thickness),
                Vector::new(room.width / 2.0 + thickness, room.height / 2.0, thickness),
            ),
            (
                Vector::new(0.0, room.height / 2.0, room.depth / 2.0 + thickness),
                Vector::new(room.width / 2.0 + thickness, room.height / 2.0, thickness),
            ),
            (
                Vector::new(0.0, room.height + thickness, 0.0),
                Vector::new(
                    room.width / 2.0 + thickness,
                    thickness,
                    room.depth / 2.0 + thickness,
                ),
            ),
        ] {
            room_colliders.push(
                world.insert_collider(
                    ColliderBuilder::cuboid(half.x, half.y, half.z)
                        .translation(position)
                        .friction(1.1)
                        .restitution(0.03)
                        .collision_groups(groups),
                    None,
                ),
            );
        }
        let home = Vector::new(0.0, definition.stow_height, 2.1);
        let body = world
            .bodies
            .insert(RigidBodyBuilder::kinematic_position_based().translation(home));
        let head = definition.striker_head;
        let collider = world.colliders.insert_with_parent(
            ColliderBuilder::cuboid(head.x, head.y, head.z)
                .friction(0.7)
                .restitution(0.02)
                .collision_groups(groups)
                .enabled(false),
            body,
            &mut world.bodies,
        );
        let vertices = definition
            .pieces
            .iter()
            .flat_map(|p| {
                p.geometry
                    .vertices
                    .iter()
                    .map(|v| vector(p.position) + vector(*v))
            })
            .collect();
        let parts = definition
            .pieces
            .iter()
            .map(|p| {
                (
                    vector(p.position),
                    p.geometry
                        .vertices
                        .iter()
                        .fold(Vector::ZERO, |half, v| half.max(vector(*v).abs())),
                )
            })
            .collect();
        Ok(Self {
            definition,
            body,
            collider,
            room_colliders,
            vertices,
            parts,
            phase: StrikePhase::Idle,
            phase_time: 0.0,
            position: home,
            rotation: Rotation::IDENTITY,
            start: home,
            origin: home,
            direction: -Vector::Z,
            travel: 0.0,
            travel_limit: TRAVEL,
            obstructed: false,
            impact_id: 0,
            impacted: false,
            last_impact_impulse: 0.0,
        })
    }
    fn home(&self) -> Vector {
        Vector::new(0.0, self.definition.stow_height, 2.1)
    }
    pub fn available(&self) -> bool {
        self.phase == StrikePhase::Idle
    }
    fn inside(&self, position: Vector, rotation: Rotation) -> bool {
        self.vertices.iter().all(|v| {
            let p = position + rotation * *v;
            p.x.abs() <= self.definition.room.width / 2.0 - MARGIN + 1e-6
                && p.z.abs() <= self.definition.room.depth / 2.0 - MARGIN + 1e-6
                && p.y >= MARGIN - 1e-6
                && p.y <= self.definition.room.height - MARGIN + 1e-6
        })
    }
    fn contact_distance(
        world: &PhysicsWorld,
        handle: ColliderHandle,
        pose: &Pose,
        half: Vector,
        prediction: f32,
    ) -> Result<Option<f32>, String> {
        let c = &world.colliders[handle];
        if !c.is_enabled() {
            return Ok(None);
        }
        let mut predicted = *c.position();
        if prediction > 0.0
            && let Some(body) = c.parent().and_then(|h| world.bodies.get(h))
        {
            predicted.translation += body.linvel() * prediction;
            predicted.rotation =
                Rotation::from_scaled_axis(body.angvel() * prediction) * predicted.rotation;
        }
        query::contact(pose, &Cuboid::new(half), &predicted, c.shape(), 0.0)
            .map(|contact| contact.map(|c| c.dist))
            .map_err(|e| format!("Striker clearance: {e:?}"))
    }
    fn path_clear(
        &self,
        world: &PhysicsWorld,
        anatomy: &[ColliderHandle],
        from: Vector,
        to: Vector,
        rotation: Rotation,
        assembly: bool,
    ) -> Result<bool, String> {
        let samples = ((to - from).length() / 0.06).ceil().max(1.0) as usize;
        let parts = if assembly {
            self.parts.clone()
        } else {
            vec![(Vector::ZERO, vector(self.definition.striker_head))]
        };
        for i in 0..=samples {
            let origin = from.lerp(to, i as f32 / samples as f32);
            for (offset, half) in &parts {
                let pose = Pose::from_parts(origin + rotation * *offset, rotation);
                for handle in anatomy {
                    if Self::contact_distance(
                        world,
                        *handle,
                        &pose,
                        *half + Vector::splat(0.012),
                        0.0,
                    )?
                    .is_some()
                    {
                        return Ok(false);
                    }
                }
            }
        }
        Ok(true)
    }
    fn predicted_clear(
        &self,
        world: &PhysicsWorld,
        anatomy: &[ColliderHandle],
        parked: Vector,
        start: Vector,
        rotation: Rotation,
    ) -> Result<bool, String> {
        let duration = OVERHEAD + DESCEND;
        let samples = (duration * 120.0).ceil() as usize;
        for i in 0..=samples {
            let time = duration * i as f32 / samples as f32;
            let mut position = self
                .position
                .lerp(parked, (time / OVERHEAD).clamp(0.0, 1.0));
            position.y = self.definition.stow_height
                + (start.y - self.definition.stow_height)
                    * ((time - OVERHEAD) / DESCEND).clamp(0.0, 1.0);
            for (offset, half) in &self.parts {
                let pose = Pose::from_parts(position + rotation * *offset, rotation);
                for handle in anatomy {
                    if Self::contact_distance(
                        world,
                        *handle,
                        &pose,
                        *half + Vector::splat(0.09),
                        time,
                    )?
                    .is_some()
                    {
                        return Ok(false);
                    }
                }
            }
        }
        Ok(true)
    }
    fn leaving_overlap(
        &self,
        world: &PhysicsWorld,
        anatomy: &[ColliderHandle],
        from: Vector,
        to: Vector,
    ) -> Result<bool, String> {
        // Retraction may start in measured contact. No new obstacle or deeper
        // overlap may be crossed by any drawn part during disabled withdrawal.
        let mut overlaps = HashMap::new();
        for (part, (offset, half)) in self.parts.iter().enumerate() {
            for h in anatomy {
                if let Some(depth) = Self::contact_distance(
                    world,
                    *h,
                    &Pose::from_parts(from + self.rotation * *offset, self.rotation),
                    *half + Vector::splat(0.012),
                    0.0,
                )? {
                    overlaps.insert((part, *h), depth);
                }
            }
        }
        let samples = ((to - from).length() / 0.06).ceil().max(1.0) as usize;
        for i in 1..=samples {
            for (part, (offset, half)) in self.parts.iter().enumerate() {
                let pose = Pose::from_parts(
                    from.lerp(to, i as f32 / samples as f32) + self.rotation * *offset,
                    self.rotation,
                );
                for h in anatomy {
                    if let Some(depth) =
                        Self::contact_distance(world, *h, &pose, *half + Vector::splat(0.012), 0.0)?
                    {
                        if overlaps
                            .get(&(part, *h))
                            .is_none_or(|previous| depth < *previous - 0.001)
                        {
                            return Ok(false);
                        }
                        overlaps.insert((part, *h), depth);
                    } else {
                        overlaps.remove(&(part, *h));
                    }
                }
            }
        }
        Ok(true)
    }
    pub fn request(
        &mut self,
        world: &mut PhysicsWorld,
        anatomy: &[ColliderHandle],
        torso: Vector,
    ) -> Result<bool, String> {
        if !self.available() || !torso.is_finite() {
            return Ok(false);
        }
        let safe_x = self.definition.room.width / 2.0 - MARGIN;
        let safe_z = self.definition.room.depth / 2.0 - MARGIN;
        if torso.x.abs() >= safe_x || torso.z.abs() >= safe_z {
            return Ok(false);
        }
        let inward = Vector::new(-torso.x, 0.0, -torso.z)
            .try_normalize()
            .unwrap_or(-Vector::Z);
        let mut candidates = Vec::new();
        for i in 0..2048 {
            let angle = i as f32 * std::f32::consts::TAU / 2048.0;
            let direction = Vector::new(angle.sin(), 0.0, angle.cos());
            let score = direction.dot(inward);
            if score <= 0.001 {
                continue;
            }
            let rotation = Rotation::from_rotation_arc(Vector::Z, direction);
            if !self.inside(self.position, rotation) {
                continue;
            }
            let (mut low, mut high) = (MIN_STANDOFF, STANDOFF);
            for vertex in &self.vertices {
                let p = torso + rotation * *vertex;
                for (value, component, bound) in
                    [(p.x, direction.x, safe_x), (p.z, direction.z, safe_z)]
                {
                    if component.abs() < 1e-9 {
                        if value.abs() > bound {
                            low = f32::INFINITY;
                        }
                    } else {
                        let a = (value - bound) / component;
                        let b = (value + bound) / component;
                        low = low.max(a.min(b));
                        high = high.min(a.max(b));
                    }
                }
                if low > high {
                    break;
                }
            }
            if low <= high {
                candidates.push((score, high, direction, rotation));
            }
        }
        candidates.sort_by(|a, b| b.0.total_cmp(&a.0).then(b.1.total_cmp(&a.1)));
        let min_y = self
            .vertices
            .iter()
            .map(|v| v.y)
            .fold(f32::INFINITY, f32::min);
        let max_y = self
            .vertices
            .iter()
            .map(|v| v.y)
            .fold(f32::NEG_INFINITY, f32::max);
        for (_, standoff, direction, rotation) in candidates {
            let mut start = torso - direction * standoff;
            start.y = start.y.clamp(
                -min_y + MARGIN,
                self.definition.room.height - max_y - MARGIN,
            );
            let parked = Vector::new(start.x, self.definition.stow_height, start.z);
            if !self.inside(start, rotation)
                || !self.inside(parked, rotation)
                || !self.path_clear(world, anatomy, self.position, parked, rotation, true)?
                || !self.path_clear(world, anatomy, parked, start, rotation, true)?
                || !self.predicted_clear(world, anatomy, parked, start, rotation)?
            {
                continue;
            }
            self.start = start;
            self.direction = direction;
            self.rotation = rotation;
            self.travel_limit = TRAVEL;
            for vertex in &self.vertices {
                let p = start + rotation * *vertex;
                for (value, component, bound) in
                    [(p.x, direction.x, safe_x), (p.z, direction.z, safe_z)]
                {
                    if component > 1e-9 {
                        self.travel_limit = self.travel_limit.min((bound - value) / component);
                    } else if component < -1e-9 {
                        self.travel_limit = self.travel_limit.min((bound + value) / -component);
                    }
                }
            }
            self.origin = self.position;
            self.phase = StrikePhase::Positioning;
            self.phase_time = 0.0;
            self.travel = 0.0;
            self.obstructed = false;
            self.impacted = false;
            world.colliders[self.collider].set_enabled(false);
            return Ok(true);
        }
        Ok(false)
    }
    fn retract(&mut self, world: &mut PhysicsWorld, obstructed: bool) {
        self.origin = self.position;
        self.obstructed = obstructed;
        self.phase = StrikePhase::Retracting;
        self.phase_time = 0.0;
        if obstructed {
            world.colliders[self.collider].set_enabled(false);
        }
    }
    pub fn before_step(
        &mut self,
        world: &mut PhysicsWorld,
        anatomy: &[ColliderHandle],
        dt: f32,
    ) -> Result<(), String> {
        if self.available() {
            return Ok(());
        }
        self.phase_time += dt;
        match self.phase {
            StrikePhase::Positioning => {
                let parked = Vector::new(self.start.x, self.definition.stow_height, self.start.z);
                let mut next = self
                    .origin
                    .lerp(parked, (self.phase_time / OVERHEAD).clamp(0.0, 1.0));
                next.y = self.definition.stow_height
                    + (self.start.y - self.definition.stow_height)
                        * ((self.phase_time - OVERHEAD) / DESCEND).clamp(0.0, 1.0);
                if !self.path_clear(world, anatomy, self.position, next, self.rotation, true)? {
                    self.retract(world, true);
                } else if self.phase_time >= OVERHEAD + DESCEND
                    && world.bodies[self.body].translation().distance(self.start) < 1e-6
                {
                    // The disabled descent was integrated already. Hold this
                    // position for an enabled zero-velocity solver step.
                    self.position = self.start;
                    self.phase = StrikePhase::Striking;
                    self.phase_time = 0.0;
                    world.colliders[self.collider].set_enabled(true);
                } else {
                    self.position = next;
                }
            }
            StrikePhase::Striking => {
                self.travel = (self.travel + SPEED * dt).min(self.travel_limit);
                self.position = self.start + self.direction * self.travel;
            }
            StrikePhase::Retracting => {
                if self.obstructed {
                    let rise = (self.phase_time / RISE).clamp(0.0, 1.0);
                    let home = ((self.phase_time - RISE) / HOME_TIME).clamp(0.0, 1.0);
                    let mut next = self.origin.lerp(self.home(), home);
                    next.y = self.origin.y + (self.definition.stow_height - self.origin.y) * rise;
                    if self.leaving_overlap(world, anatomy, self.position, next)? {
                        self.position = next;
                    } else {
                        self.phase_time = (self.phase_time - dt).max(0.0);
                    }
                } else {
                    let back = (self.phase_time / BACK).clamp(0.0, 1.0);
                    let rise = ((self.phase_time - BACK) / RISE).clamp(0.0, 1.0);
                    let home = ((self.phase_time - BACK - RISE) / HOME_TIME).clamp(0.0, 1.0);
                    let mut stowed = self.origin.lerp(self.start, back);
                    stowed.y += (self.definition.stow_height - stowed.y) * rise;
                    let next = stowed.lerp(self.home(), home);
                    if back < 1.0
                        || self.path_clear(
                            world,
                            anatomy,
                            self.position,
                            next,
                            self.rotation,
                            true,
                        )?
                    {
                        self.position = next;
                    } else {
                        self.retract(world, true);
                    }
                    if back >= 1.0 {
                        world.colliders[self.collider].set_enabled(false);
                    }
                }
                if self.phase_time >= RISE + HOME_TIME + if self.obstructed { 0.0 } else { BACK } {
                    self.phase = StrikePhase::Idle;
                    self.phase_time = 0.0;
                }
            }
            StrikePhase::Idle => {}
        }
        world.bodies[self.body]
            .set_next_kinematic_position(Pose::from_parts(self.position, self.rotation));
        Ok(())
    }
    pub fn after_step(
        &mut self,
        world: &mut PhysicsWorld,
        anatomy: &[ColliderHandle],
    ) -> Result<bool, String> {
        if self.phase != StrikePhase::Striking {
            return Ok(false);
        }
        let mut impulse = 0.0;
        for h in anatomy {
            if let Some(pair) = world.narrow_phase.contact_pair(self.collider, *h) {
                impulse += crate::measurement::solver_normal_impulse(pair)?.length();
            }
        }
        let impact = !self.impacted && impulse > 0.25;
        if impact {
            self.impacted = true;
            self.impact_id += 1;
            self.last_impact_impulse = impulse;
        }
        if impact || self.travel >= self.travel_limit {
            self.retract(world, false);
        }
        Ok(impact)
    }
    pub fn snapshot(&self, world: &PhysicsWorld) -> StrikerSnapshot {
        let body = &world.bodies[self.body];
        StrikerSnapshot {
            phase: self.phase,
            available: self.available(),
            impact_id: self.impact_id,
            last_impact_impulse_ns: self.last_impact_impulse,
            body: lh_model::Pose {
                id: "striker".into(),
                position: vec3(body.translation()),
                rotation: quat(*body.rotation()),
                linear_velocity: vec3(body.linvel()),
                angular_velocity: vec3(body.angvel()),
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Profile, runtime::Runtime};
    use lh_contracts::{Action, Command, Mode, Reject, Stamp};
    #[test]
    fn misses_remain_misses_and_clear_room_paths_retract_at_walls_and_corners() {
        for target in [
            Vector::new(0.0, 1.2, 0.0),
            Vector::new(4.4, 1.2, 0.0),
            Vector::new(-4.4, 1.2, 0.0),
            Vector::new(0.0, 1.2, 4.4),
            Vector::new(0.0, 1.2, -4.4),
            Vector::new(3.3, 1.2, 3.3),
            Vector::new(-3.3, 1.2, -3.3),
        ] {
            let mut world = PhysicsWorld::new();
            world.integration_parameters.dt = 1.0 / 240.0;
            let mut striker = Striker::new(&mut world).unwrap();
            assert!(
                striker.request(&mut world, &[], target).unwrap(),
                "{target:?}"
            );
            assert!(!striker.request(&mut world, &[], target).unwrap());
            for _ in 0..300 {
                striker.before_step(&mut world, &[], 1.0 / 240.0).unwrap();
                world.step();
                assert!(!striker.after_step(&mut world, &[]).unwrap());
                let pose = world.bodies[striker.body].position();
                assert!(
                    striker.inside(pose.translation, pose.rotation),
                    "{target:?}: {pose:?}"
                );
            }
            let result = striker.snapshot(&world);
            assert!(result.available);
            assert_eq!(result.impact_id, 0);
            assert!(
                world.bodies[striker.body]
                    .translation()
                    .distance(striker.home())
                    < 1e-5
            );
        }
    }
    #[test]
    fn paused_hidden_busy_and_future_strikes_are_rejected_without_queueing() {
        let mut r = Runtime::new(Mode::Protocol, 0.0, Profile::default()).unwrap();
        let command = |sequence, tick, action| Command {
            stamp: Stamp::new(1, sequence, tick),
            action,
        };
        r.enqueue(command(1, 0, Action::Pause)).unwrap();
        r.advance(0).unwrap();
        assert_eq!(
            r.enqueue(command(2, 0, Action::Strike)),
            Err(Reject::Cancelled)
        );
        r.enqueue(command(3, 0, Action::Resume)).unwrap();
        r.advance(0).unwrap();
        assert_eq!(r.enqueue(command(4, 10, Action::Strike)), Err(Reject::Late));
        r.enqueue(command(5, 0, Action::Strike)).unwrap();
        let reply = r.advance(0).unwrap();
        assert!(reply.acknowledgements[0].rejected.is_none());
        assert_eq!(reply.snapshot.counters.strikes, 0);
        assert_eq!(
            r.enqueue(command(6, 0, Action::Strike)),
            Err(Reject::Cancelled)
        );
        r.enqueue(command(7, 0, Action::Reset)).unwrap();
        let reset = r.advance(0).unwrap();
        assert!(reset.snapshot.striker.unwrap().available);
        assert_eq!(reset.snapshot.counters.strikes, 0);
        r.set_visible(false).unwrap();
        assert_eq!(
            r.enqueue(Command {
                stamp: Stamp::new(2, 1, 0),
                action: Action::Strike
            }),
            Err(Reject::Cancelled)
        );
        r.set_visible(true).unwrap();
        assert_eq!(
            r.advance(4).unwrap().snapshot.striker.unwrap().phase,
            StrikePhase::Idle
        );
    }
    #[test]
    fn obstructed_positioning_is_never_enabled_through_a_body() {
        let mut world = PhysicsWorld::new();
        let mut striker = Striker::new(&mut world).unwrap();
        let obstruction = world.insert_collider(
            ColliderBuilder::cuboid(4.8, 0.3, 4.8).translation(Vector::new(
                0.0,
                striker.definition.stow_height,
                0.0,
            )),
            None,
        );
        assert!(
            !striker
                .request(&mut world, &[obstruction], Vector::new(0.0, 1.2, 0.0))
                .unwrap()
        );
        assert!(striker.available());
        assert!(!world.colliders[striker.collider].is_enabled());
        assert_eq!(striker.impact_id, 0);
    }
    #[test]
    fn impossible_corner_and_outside_room_requests_remain_idle() {
        let mut world = PhysicsWorld::new();
        let mut s = Striker::new(&mut world).unwrap();
        for target in [
            Vector::new(4.3, 1.2, 4.3),
            Vector::new(5.0, 1.2, 0.0),
            Vector::splat(f32::NAN),
        ] {
            assert!(!s.request(&mut world, &[], target).unwrap());
            assert!(s.available());
            assert_eq!(s.impact_id, 0);
        }
    }
    #[test]
    fn disabled_withdrawal_checks_the_actuator_as_well_as_the_head() {
        let mut world = PhysicsWorld::new();
        let mut s = Striker::new(&mut world).unwrap();
        s.rotation = Rotation::IDENTITY;
        let obstacle = world.insert_collider(
            ColliderBuilder::cuboid(0.1, 0.1, 0.1).translation(Vector::new(0.0, 1.7, -1.19)),
            None,
        );
        let from = Vector::new(0.0, 1.0, 0.0);
        let to = Vector::new(0.0, 2.0, 0.0);
        assert!(
            s.path_clear(&world, &[obstacle], from, to, s.rotation, false)
                .unwrap()
        );
        assert!(!s.leaving_overlap(&world, &[obstacle], from, to).unwrap());
    }
}
