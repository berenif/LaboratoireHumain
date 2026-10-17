//! Test-only continuation of C in its original living world. Motor targets only.
use super::*;

#[derive(Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
struct Stage {
    label: String,
    duration_s: f32,
    coordinates: Coordinates,
    hand: String,
    target_hand_center_m: lh_model::Vec3,
    acquire_contact: bool,
}
#[derive(Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
struct RiseRow {
    progress: f32,
    coordinates: Coordinates,
}
#[derive(Clone, Deserialize)]
struct ReferenceRow {
    poses: Vec<lh_model::Pose>,
}
#[derive(Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
struct Program {
    initial_coordinates: Coordinates,
    preparation_stages: Vec<Stage>,
    rise_rows: Vec<RiseRow>,
    rise_duration_s: f32,
    construction_errors: Vec<Value>,
    rows: Vec<ReferenceRow>,
}
struct Running {
    entry_time: f32,
    stage_time: f32,
    stage: usize,
    from: Coordinates,
    offset: Coordinates,
    held: Coordinates,
    holding_bias: Coordinates,
    entry_torso_height: f32,
    best_progress: f32,
    last_progress_time: f32,
    milestone_steps: u64,
    max_milestone_steps: u64,
    missing_hands_steps: u64,
}
pub(super) struct Controller {
    program: Program,
    running: Option<Running>,
    last_observation: Value,
    hold_first_preparation: bool,
    contact_tracking: bool,
    land_tracking: RefCell<super::land_arm_tracking::State>,
}
impl Controller {
    pub(super) fn load(path: &str) -> Result<Self, String> {
        let p: Program = serde_json::from_slice(&std::fs::read(path).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
        if !p.construction_errors.is_empty()
            || p.preparation_stages.len() != 6
            || p.rise_rows.len() < 2
            || p.rise_rows[0].progress != 0.0
            || p.rise_rows.last().unwrap().progress != 1.0
            || !p
                .rise_rows
                .windows(2)
                .all(|r| r[0].progress < r[1].progress)
            || !p.rise_duration_s.is_finite()
            || p.rise_duration_s <= 0.0
            || !p.preparation_stages.iter().all(|s| {
                s.duration_s.is_finite()
                    && s.duration_s > 0.0
                    && s.coordinates.iter().flatten().all(|q| q.is_finite())
            })
            || !p
                .rise_rows
                .iter()
                .all(|r| r.coordinates.iter().flatten().all(|q| q.is_finite()))
        {
            return Err("invalid or refused prone motor reference".into());
        }
        Ok(Self {
            program: p,
            running: None,
            last_observation: Value::Null,
            hold_first_preparation: super::left_hand_command_audit::enabled(),
            contact_tracking: super::prone_chain_execution::contact_tracking_enabled(),
            land_tracking: RefCell::new(super::land_arm_tracking::State::default()),
        })
    }
    pub(super) fn gravity_sides(&self) -> &'static [&'static str] {
        if self.running.as_ref().is_some_and(|s| s.stage >= 3) {
            &["left", "right"]
        } else {
            &["left"]
        }
    }
    pub(super) fn active(&self) -> bool {
        self.running.is_some()
    }
    pub(super) fn start(&mut self, s: &Simulation, held: &Coordinates) -> Result<(), String> {
        let measured = offline_trajectory::coordinates(s)?;
        let mut offset = [[0.0; 3]; 25];
        for i in 0..25 {
            for k in 0..3 {
                offset[i][k] = measured[i][k] - self.program.initial_coordinates[i][k];
            }
        }
        let mut holding_bias = [[0.0; 3]; 25];
        for i in 0..25 {
            for k in 0..3 {
                if !arm(&s.model.segments[i].id) {
                    holding_bias[i][k] = held[i][k] - measured[i][k];
                }
            }
        }
        let mut from = measured;
        for i in 0..25 {
            if !arm(&s.model.segments[i].id) {
                from[i] = held[i];
            }
        }
        let now = s.substeps_completed as f32 * s.profile.dt_s;
        let torso = s
            .model
            .segments
            .iter()
            .position(|p| p.id == "torso")
            .unwrap();
        self.running = Some(Running {
            entry_time: now,
            stage_time: now,
            stage: 0,
            from,
            held: *held,
            holding_bias,
            offset,
            entry_torso_height: s.world.bodies[s.bodies[torso]].translation().y,
            best_progress: f32::NEG_INFINITY,
            last_progress_time: now,
            milestone_steps: 0,
            max_milestone_steps: 0,
            missing_hands_steps: 0,
        });
        Ok(())
    }
    pub(super) fn command(&self, s: &Simulation) -> Option<(Coordinates, Coordinates, Value)> {
        let state = self.running.as_ref()?;
        let now = s.substeps_completed as f32 * s.profile.dt_s;
        let rise = state.stage >= self.program.preparation_stages.len();
        let duration = if rise {
            self.program.rise_duration_s
        } else {
            self.program.preparation_stages[state.stage].duration_s
        };
        let raw = ((now - state.stage_time) / duration).clamp(0.0, 1.0);
        let u = raw * raw * (3.0 - 2.0 * raw);
        let du = 6.0 * raw * (1.0 - raw) / duration;
        let mut q = [[0.0; 3]; 25];
        let mut dq = [[0.0; 3]; 25];
        if rise {
            let j = self
                .program
                .rise_rows
                .partition_point(|r| r.progress <= u)
                .saturating_sub(1)
                .min(self.program.rise_rows.len() - 2);
            let a = &self.program.rise_rows[j];
            let b = &self.program.rise_rows[j + 1];
            let span = b.progress - a.progress;
            let t = ((u - a.progress) / span).clamp(0.0, 1.0);
            for i in 0..25 {
                for k in 0..3 {
                    q[i][k] = a.coordinates[i][k]
                        + (b.coordinates[i][k] - a.coordinates[i][k]) * t
                        + state.offset[i][k];
                    dq[i][k] = (b.coordinates[i][k] - a.coordinates[i][k]) / span * du;
                }
            }
        } else {
            let target = &self.program.preparation_stages[state.stage].coordinates;
            for i in 0..25 {
                for k in 0..3 {
                    let end = target[i][k] + state.offset[i][k];
                    q[i][k] = state.from[i][k] + (end - state.from[i][k]) * u;
                    dq[i][k] = (end - state.from[i][k]) * du;
                }
            }
        }
        // Preserve C's settled non-arm motor efforts through preparation.
        // Blend their existing spring bias away only during the coordinated rise.
        for (i, p) in s.model.segments.iter().enumerate() {
            if !arm(&p.id) {
                for k in 0..3 {
                    if rise {
                        q[i][k] += state.holding_bias[i][k] * (1.0 - u);
                        dq[i][k] -= state.holding_bias[i][k] * du;
                    } else {
                        q[i][k] = state.held[i][k];
                        dq[i][k] = 0.0;
                    }
                }
            }
        }
        // Keep canonical hard bounds; only tiny captured-coordinate offsets can hit them.
        for (i, p) in s.model.segments.iter().enumerate().skip(1) {
            for a in &p.joint_profile.as_ref().unwrap().axes {
                let k = match a.coordinate {
                    'x' => 0,
                    'y' => 1,
                    _ => 2,
                };
                q[i][k] = q[i][k].clamp(a.min_radians, a.max_radians);
            }
        }
        let label = if rise {
            "loaded-trunk-rise"
        } else {
            &self.program.preparation_stages[state.stage].label
        };
        let tracking = if self.contact_tracking {
            let reference = self.program.rows[0]
                .poses
                .iter()
                .find(|p| p.id == "torso")
                .unwrap();
            let task = super::land_arm_tracking::Task {
                stage: state.stage,
                progress: u,
                reference_torso: Pose::from_parts(
                    s.heading * vector(reference.position),
                    s.heading * rotation(reference.rotation),
                ),
                land_goals: [
                    s.heading * vector(self.program.preparation_stages[2].target_hand_center_m),
                    s.heading * vector(self.program.preparation_stages[5].target_hand_center_m),
                ],
            };
            self.land_tracking
                .borrow_mut()
                .adjust(s, &mut q, &mut dq, task)
        } else {
            Value::Null
        };
        Some((
            q,
            dq,
            json!({"phase":label,"stage":state.stage,"progress":u,"entryTimeS":state.entry_time,"stageTimeS":state.stage_time,"landTracking":tracking}),
        ))
    }
    pub(super) fn observe(
        &mut self,
        s: &Simulation,
        substep: u32,
        contact: &[Value],
        all_still: bool,
    ) -> Result<(), measurement::Failure> {
        let Some(state) = self.running.as_mut() else {
            return Ok(());
        };
        let c = &crate::recovery_contract::contract().recovery_acceptance;
        let now = s.substeps_completed as f32 * s.profile.dt_s;
        let index = |id: &str| s.model.segments.iter().position(|p| p.id == id).unwrap();
        let actual = |id: &str| contact_sum(contact, id);
        let loaded = |id: &str| {
            actual(id) >= c.minimum_load_n && super::super::super::load(s, id) >= c.minimum_load_n
        };
        let hands = loaded("leftHand") && loaded("rightHand");
        let torso = &s.world.bodies[s.bodies[index("torso")]];
        let up = (*torso.rotation() * Vector::Y).y;
        let rise_m = torso.translation().y - state.entry_torso_height;
        let limb = |id: &str| {
            id.ends_with("Hand")
                || id.ends_with("Thigh")
                || id.ends_with("Shin")
                || id.ends_with("Foot")
                || id.ends_with("Forefoot")
        };
        let mut points = Vec::new();
        let mut limb_load = 0.;
        let mut lower = false;
        for p in &s.model.segments {
            if limb(&p.id) {
                limb_load += actual(&p.id);
                if loaded(&p.id) {
                    lower |= !p.id.ends_with("Hand");
                    for cp in contact.iter().filter(|v| v["id"] == p.id) {
                        points.push([
                            cp["point"]["x"].as_f64().unwrap() as f32,
                            cp["point"]["z"].as_f64().unwrap() as f32,
                        ]);
                    }
                }
            }
        }
        let com = s.heading.inverse() * s.center_of_mass();
        let vel = super::velocity(s);
        let capture = s.heading.inverse()
            * (s.center_of_mass()
                + vel * ((s.center_of_mass().y - s.support_height()).max(0.2) / 9.81).sqrt());
        let inside = crate::stepping::inside_support(points.clone(), [com.x, com.z], 0.003)
            && crate::stepping::inside_support(points, [capture.x, capture.z], 0.003);
        let unloaded = ["head", "neck", "torso"].iter().all(|id| {
            actual(id) < c.minimum_load_n && super::super::super::load(s, id) < c.minimum_load_n
        });
        let milestone = unloaded
            && up >= 0.40
            && rise_m >= 0.12
            && s.world.bodies[s.bodies[0]].translation().y > 0.28
            && hands
            && lower
            && limb_load >= 0.8 * s.total_mass() * 9.81
            && inside
            && all_still;
        state.milestone_steps = if milestone {
            state.milestone_steps + 1
        } else {
            0
        };
        state.max_milestone_steps = state.max_milestone_steps.max(state.milestone_steps);
        let rise = state.stage >= self.program.preparation_stages.len();
        let mut error = None;
        let progress;
        let mut advanced = false;
        if !rise {
            let task = &self.program.preparation_stages[state.stage];
            let hand =
                s.heading.inverse() * s.world.bodies[s.bodies[index(&task.hand)]].translation();
            let distance = (hand - vector(task.target_hand_center_m)).length();
            progress = -distance;
            if !self.hold_first_preparation
                && now - state.stage_time >= task.duration_s
                && distance <= 0.05
                && (!task.acquire_contact || loaded(&task.hand))
            {
                state.from = task.coordinates;
                for i in 0..25 {
                    for k in 0..3 {
                        state.from[i][k] += state.offset[i][k];
                    }
                }
                state.stage += 1;
                advanced = true;
                state.stage_time = now;
                state.best_progress = f32::NEG_INFINITY;
                state.last_progress_time = now;
            }
            for (i, p) in s.model.segments.iter().enumerate() {
                if !arm(&p.id) {
                    state.from[i] = state.held[i];
                }
            }
            self.last_observation = json!({"active":true,"stage":state.stage,"targetHand":task.hand,"handDistanceM":distance,"handActuallyPersistent":loaded(&task.hand),"entryTimeS":state.entry_time});
        } else {
            progress = rise_m;
            state.missing_hands_steps = if hands {
                0
            } else {
                state.missing_hands_steps + 1
            };
            if state.missing_hands_steps >= crate::recovery_contract::substeps(c.support_loss_s) {
                error = Some(measurement::Failure::new(
                    s.tick,
                    substep,
                    "prone-hand-support-loss",
                    "bilateral real hands required for trunk rise".into(),
                    state.missing_hands_steps as f32 * s.profile.dt_s,
                    c.support_loss_s as f32,
                ));
            }
            self.last_observation =
                json!({"active":true,"stage":state.stage,"entryTimeS":state.entry_time});
        }
        if !advanced && progress > state.best_progress + 0.005 {
            state.best_progress = progress;
            state.last_progress_time = now;
        }
        let passed =
            state.max_milestone_steps >= crate::recovery_contract::substeps(c.stable_persistence_s);
        if error.is_none() && now - state.last_progress_time >= c.stall_s as f32 && !passed {
            error = Some(measurement::Failure::new(
                s.tick,
                substep,
                "prone-stage-stall",
                "no measured improvement under original stall duration".into(),
                now - state.last_progress_time,
                c.stall_s as f32,
            ));
        }
        self.last_observation["milestone"] = json!({"headNeckTorsoUnloaded":unloaded,"torsoUpY":up,"torsoCenterRiseM":rise_m,"handsActuallyPersistent":hands,"lowerLimbActuallyPersistent":lower,"limbLoadN":limb_load,"comAndCaptureInsideActualLimbHull":inside,"all25Still":all_still,"consecutiveSteps":state.milestone_steps,"maxConsecutiveSteps":state.max_milestone_steps,"passed":passed});
        if let Some(e) = error { Err(e) } else { Ok(()) }
    }
    pub(super) fn passed(&self) -> bool {
        self.running.as_ref().is_some_and(|s| {
            s.max_milestone_steps
                >= crate::recovery_contract::substeps(
                    crate::recovery_contract::contract()
                        .recovery_acceptance
                        .stable_persistence_s,
                )
        })
    }
    pub(super) fn observation(&self) -> Value {
        self.last_observation.clone()
    }
}

#[test]
fn a_new_task_measures_progress_from_its_own_goal() {
    let fixture = serde_json::from_str::<FixtureSet>(FIXTURES)
        .unwrap()
        .cases
        .into_iter()
        .find(|x| x.id == "landed-crouch-left")
        .unwrap();
    let mut s = Simulation::build(
        fixture.heading,
        crate::recovery_contract::pinned_profile(),
        true,
        None,
        Some(&fixture.poses),
    )
    .unwrap();
    let q = offline_trajectory::coordinates(&s).unwrap();
    let hand = s
        .model
        .segments
        .iter()
        .position(|p| p.id == "leftHand")
        .unwrap();
    let first_goal = s.heading.inverse() * s.world.bodies[s.bodies[hand]].translation();
    let first = Stage {
        label: "first".into(),
        duration_s: 1.0,
        coordinates: q,
        hand: "leftHand".into(),
        target_hand_center_m: crate::vec3(first_goal),
        acquire_contact: false,
    };
    let second = Stage {
        label: "second".into(),
        target_hand_center_m: crate::vec3(first_goal + Vector::X * 0.3),
        ..first.clone()
    };
    let mut controller = Controller {
        program: Program {
            initial_coordinates: q,
            preparation_stages: vec![first, second],
            rise_rows: vec![
                RiseRow {
                    progress: 0.0,
                    coordinates: q,
                },
                RiseRow {
                    progress: 1.0,
                    coordinates: q,
                },
            ],
            rise_duration_s: 1.0,
            construction_errors: vec![],
            rows: vec![],
        },
        running: None,
        last_observation: Value::Null,
        hold_first_preparation: false,
        contact_tracking: false,
        land_tracking: RefCell::new(super::land_arm_tracking::State::default()),
    };
    let bodies = serde_json::to_value(&s.world.bodies).unwrap();
    controller.start(&s, &q).unwrap();
    s.substeps_completed = 240;
    controller.observe(&s, 0, &[], false).unwrap();
    let state = controller.running.as_ref().unwrap();
    assert_eq!(state.stage, 1);
    assert_eq!(
        state.best_progress,
        f32::NEG_INFINITY,
        "completed goal must not seed the new goal"
    );
    s.substeps_completed = 241;
    controller.observe(&s, 0, &[], false).unwrap();
    let state = controller.running.as_ref().unwrap();
    assert!((state.best_progress + 0.3).abs() < 1e-5);
    assert_eq!(
        state.last_progress_time,
        s.substeps_completed as f32 * s.profile.dt_s
    );
    assert_eq!(serde_json::to_value(&s.world.bodies).unwrap(), bodies);
}
