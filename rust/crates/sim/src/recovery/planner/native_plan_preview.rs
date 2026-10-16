//! Complete static-scene snapshots and a fail-closed preview orchestration shell.
//!
//! The native driver reuses Simulation::advance_tick_observed unchanged, with
//! all four steps reserved before each tick. It never duplicates the integration
//! kernel. Native admission remains closed until a shared experimental owner,
//! frozen prerequisites and measured actual/copy parity have been established.

use crate::Simulation;
use rapier3d::prelude::{PhysicsPipeline, PhysicsWorld};
use serde::Serialize;

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum PreviewError {
    Snapshot(String),
    InvalidHorizon,
    ClockOverflow,
    Reservation(String),
    Kernel(String),
    ClockMismatch { expected: u64, actual: u64 },
    NativeKernelParityUnavailable,
    ParityMismatch(String),
}

/// Constructed only by comparing a complete native four-substep parity trace.
/// This is evidence for the measured kernel/state, not a blanket physical
/// success certificate. The batch ledger separately admits each frozen run.
#[derive(Debug, Serialize)]
pub(crate) struct NativeKernelAdmission {
    initial_substep: u64,
    verified_substeps: u32,
    transfer_scope: Option<Vec<u8>>,
}
impl NativeKernelAdmission {
    fn require_scope(&self, scope: Option<&[u8]>) -> Result<(), PreviewError> {
        if self.transfer_scope.as_deref() != scope {
            return Err(PreviewError::ParityMismatch(
                "Native kernel certificate belongs to a different candidate scope".into(),
            ));
        }
        Ok(())
    }
}

/// Non-deserializable native admission capability. Its only constructor is
/// StaticPreviewBranch::certify_transfer_horizon below; a pure adapter payload
/// cannot be passed directly to the real command owner.
pub(crate) struct NativeWindowReceipt {
    data: super::transfer_adapter::NativeWindowData,
}
impl NativeWindowReceipt {
    pub(super) fn into_data(self) -> super::transfer_adapter::NativeWindowData {
        self.data
    }
}

/// Four-step verification capability, never a material/history admission.
/// Only a completed ParityVerification branch can construct it below.
pub(crate) struct NativeTechnicalReceipt {
    data: super::transfer_adapter::NativeTechnicalData,
}
impl NativeTechnicalReceipt {
    pub(super) fn into_data(self) -> super::transfer_adapter::NativeTechnicalData {
        self.data
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
struct ParityFrame {
    native_substep: u64,
    world: Vec<u8>,
    state: Vec<u8>,
    owner_memory: Vec<u8>,
    /// These numeric fields are compared bitwise rather than through JSON,
    /// which does not distinguish all nonfinite values or signed zeros.
    body_and_motor_bits: Vec<u32>,
}

fn coordinates_bits(values: &[[f32; 3]]) -> Vec<[u32; 3]> {
    values.iter().map(|value| value.map(f32::to_bits)).collect()
}

/// Persistent recovery state is part of parity even when the caller has no
/// separate memory. Exhaustive destructuring makes new fields a compile error.
fn native_memory(s: &Simulation) -> serde_json::Value {
    // Every Simulation field is named. Added persistent state must be assigned
    // to the world, snapshot, explicit decision memory, or refused static scope.
    let Simulation {
        world: _,
        model,
        bodies,
        colliders,
        joints,
        floor,
        filter,
        profile,
        tick,
        generation,
        motion,
        counters,
        targets,
        rest_targets,
        rest_com,
        rest_com_roundoff,
        heading,
        grab,
        last_press,
        applied_sequence,
        contacts,
        persistence,
        integrated_support_substeps,
        integrated_support_loads_n,
        diagnostics,
        first_failure,
        motor_ceiling_sum,
        substeps_completed,
        stepping,
        motion_observation,
        environment,
        striker,
        recovery,
        coordination,
    } = s;
    let crate::Filter { exclusions } = filter;
    let recovery = recovery.as_ref().map(|r| {
        let crate::recovery::Controller {
            status, targets, hands, lead, stand_feet, stand_root, stand_goal,
            best_progress, velocities, foot_anchors, hand_anchors, knee_anchors,
            handoff_from, stable_substeps, handoff_substeps, settle_substeps, roll_arm,
        } = r;
        let anchors = |values: &[Option<crate::recovery::PlantedAnchor>; 2]| {
            values.map(|a| a.map(|a| serde_json::json!({
                "segment":a.segment,
                "localBits":a.local_point.to_array().map(f32::to_bits),
                "worldBits":a.world_point.to_array().map(f32::to_bits)
            })))
        };
        let crate::recovery::Status {phase,phase_time_s,qualified_time_s,stable_time_s,
            loaded_feet,elapsed_s,stalled_s,unsupported_s,retries,last_retry,progress}=status;
        let status=serde_json::json!({"phase":phase,"phaseTimeBits":phase_time_s.to_bits(),
            "qualifiedTimeBits":qualified_time_s.to_bits(),"stableTimeBits":stable_time_s.to_bits(),
            "loadedFeet":loaded_feet,"elapsedBits":elapsed_s.to_bits(),"stalledBits":stalled_s.to_bits(),
            "unsupportedBits":unsupported_s.to_bits(),"retries":retries,"lastRetry":last_retry,
            "progressBits":progress.to_bits()});
        serde_json::json!({
            "status":status,"rollArm":roll_arm,"targetsBits":coordinates_bits(targets),
            "velocityBits":coordinates_bits(velocities),
            "handsBits":hands.map(|v|v.to_array().map(f32::to_bits)),
            "lead":lead,"standFeetBits":stand_feet.map(|v|v.map(|x|x.to_array().map(f32::to_bits))),
            "standRootBits":stand_root.map(pose_bits),"standGoalBits":stand_goal.map(|q|q.to_array().map(f32::to_bits)),
            "bestProgressBits":best_progress.to_bits(),"footAnchors":anchors(foot_anchors),
            "handAnchors":anchors(hand_anchors),"kneeAnchors":anchors(knee_anchors),
            "handoffFromBits":handoff_from.as_ref().map(|q|coordinates_bits(q)),
            "stableSubsteps":stable_substeps,"handoffSubsteps":handoff_substeps,
            "settleSubsteps":settle_substeps
        })
    });
    serde_json::json!({"recovery":recovery,
        "modeSignature":coordination.as_ref().map(|o|o.memory.mode_signature),
        "stepping":stepping.as_ref().map(|step|step.parity_memory()),
        "motionObservationBits":motion_observation.parity_memory(),
        "restComBits":rest_com.to_array().map(f32::to_bits),
        "restComRoundoffBits":rest_com_roundoff.to_array().map(f32::to_bits),
        "headingBits":heading.to_array().map(f32::to_bits),
        "motorCeilingBits":motor_ceiling_sum.to_bits(),
        "bodies":bodies.iter().map(|h|h.into_raw_parts()).collect::<Vec<_>>(),
        "colliders":colliders.iter().map(|h|h.into_raw_parts()).collect::<Vec<_>>(),
        "joints":joints.iter().map(|h|h.map(|h|h.into_raw_parts())).collect::<Vec<_>>(),
        "floor":floor.into_raw_parts(),"filter":exclusions.iter().map(|(a,b)|(a.to_string(),b.to_string())).collect::<Vec<_>>(),
        "model":model_scope(model),"profile":profile_scope(profile),
        "tick":tick,"generation":generation,"motion":motion,"counters":counters,
        "targetsBits":coordinates_bits(targets),"restTargetsBits":coordinates_bits(rest_targets),
        "lastPress":last_press,"appliedSequence":applied_sequence,
        "contacts":contacts.iter().map(|c| {let lh_contracts::Contact {segment,normal_load_n,persistence_s}=c;
            (segment,normal_load_n.to_bits(),persistence_s.to_bits())}).collect::<Vec<_>>(),
        "persistenceBits":persistence.iter().map(|v|v.to_bits()).collect::<Vec<_>>(),
        "integratedSupportSubsteps":integrated_support_substeps,
        "integratedSupportLoadBits":integrated_support_loads_n.map(f32::to_bits),
        "diagnostics":diagnostics,"firstFailure":first_failure,"substeps":substeps_completed,
        "staticScope":{"grabPresent":grab.is_some(),"environmentPresent":environment.is_some(),"strikerPresent":striker.is_some()}})
}

/// Proof bookkeeping differs between a copied forecast and its real lease.
/// Remove exactly those two fields, preserving every other owner/adapter field
/// (including future additions) in both the kernel parity and lease identities.
fn coordination_decision_evidence(mut evidence: serde_json::Value) -> serde_json::Value {
    if let Some(transfer) = evidence
        .get_mut("transfer")
        .and_then(serde_json::Value::as_object_mut)
    {
        transfer.remove("authorization");
        transfer.remove("nativeRecords");
    }
    evidence
}

fn parity_frame(s: &Simulation, memory: &impl Serialize) -> Result<ParityFrame, PreviewError> {
    // Dynamic interactions have private owners outside this static protocol.
    // Refuse them instead of projecting only a presence flag as full memory.
    if s.grab.is_some() || s.environment.is_some() || s.striker.is_some() {
        return Err(PreviewError::Snapshot(
            "Native static parity excludes active interaction owners".into(),
        ));
    }
    let serial = |v| {
        serde_json::to_vec(v)
            .map_err(|e| PreviewError::Snapshot(format!("Parity serialization: {e}")))
    };
    let mut bits = Vec::new();
    for handle in &s.bodies {
        let body = &s.world.bodies[*handle];
        bits.extend(body.translation().to_array().map(f32::to_bits));
        let q = body.rotation();
        bits.extend([q.x, q.y, q.z, q.w].map(f32::to_bits));
        bits.extend(body.linvel().to_array().map(f32::to_bits));
        bits.extend(body.angvel().to_array().map(f32::to_bits));
    }
    for handle in s.joints.iter().flatten() {
        let joint = s
            .world
            .impulse_joints
            .get(*handle)
            .ok_or_else(|| PreviewError::Snapshot("Missing native parity joint".into()))?;
        for motor in &joint.data.motors {
            bits.extend(
                [
                    motor.target_pos,
                    motor.target_vel,
                    motor.stiffness,
                    motor.damping,
                    motor.max_force,
                    motor.impulse,
                    motor.delivered_abs_impulse,
                    motor.delivered_impulse,
                ]
                .map(f32::to_bits),
            );
            bits.extend(
                motor
                    .delivered_angular_impulse_1
                    .to_array()
                    .map(f32::to_bits),
            );
            bits.extend(
                motor
                    .delivered_angular_impulse_2
                    .to_array()
                    .map(f32::to_bits),
            );
        }
    }
    let state = serde_json::json!({
        "snapshot": s.snapshot(), "substeps": s.substeps_completed, "tick": s.tick,
        "targets": s.targets, "restTargets": s.rest_targets,
        "targetsBits":coordinates_bits(&s.targets),
        "restTargetsBits":coordinates_bits(&s.rest_targets),
        "contacts": s.contacts, "nativeMemory": native_memory(s),
        "persistence": s.persistence,
        "integratedSupportSubsteps": s.integrated_support_substeps,
        "integratedSupportLoadsN": s.integrated_support_loads_n,
        "recovery": s.recovery_status(),
        "coordination": s.coordination.as_ref().map(|owner| coordination_decision_evidence(owner.evidence())),
    });
    Ok(ParityFrame {
        native_substep: s.substeps_completed,
        world: serde_json::to_vec(&s.world).map_err(|e| PreviewError::Snapshot(e.to_string()))?,
        state: serial(&state)?,
        owner_memory: serde_json::to_vec(memory)
            .map_err(|e| PreviewError::Snapshot(e.to_string()))?,
        body_and_motor_bits: bits,
    })
}

/// Exact before-state for the transfer's short real lease. The owner is passed
/// explicitly because configure_motors temporarily takes it out of Simulation.
/// Only authorization/history bookkeeping is excluded; all decision memory,
/// native caches, recovery observer, targets and numeric body/motor bits remain.
pub(super) fn transfer_identity(
    s: &Simulation,
    owner: &super::whole_body_controller::Controller,
) -> Result<Vec<u8>, String> {
    let mut frame = parity_frame(s, &()).map_err(|e| format!("Transfer identity: {e:?}"))?;
    let mut state: serde_json::Value =
        serde_json::from_slice(&frame.state).map_err(|e| e.to_string())?;
    state["coordination"] = coordination_decision_evidence(owner.evidence());
    state["nativeMemory"]["modeSignature"] = serde_json::json!(owner.memory.mode_signature);
    frame.state = serde_json::to_vec(&state).map_err(|e| e.to_string())?;
    Ok(frame.exact_identity())
}

impl ParityFrame {
    /// Canonical tagged length-delimited bytes, not decimal JSON arrays of
    /// bytes. All compared contents remain exact; no hash collisions or decoder
    /// ambiguity are introduced by concatenation.
    fn exact_identity(&self) -> Vec<u8> {
        let mut out = b"LH-native-transfer-identity-v2\0".to_vec();
        out.extend_from_slice(&self.native_substep.to_le_bytes());
        for part in [&self.world, &self.state, &self.owner_memory] {
            out.extend_from_slice(&(part.len() as u64).to_le_bytes());
            out.extend_from_slice(part);
        }
        out.extend_from_slice(&(self.body_and_motor_bits.len() as u64).to_le_bytes());
        for bits in &self.body_and_motor_bits {
            out.extend_from_slice(&bits.to_le_bytes());
        }
        out
    }
}

fn pose_bits(p: rapier3d::prelude::Pose) -> serde_json::Value {
    serde_json::json!({"translationBits":p.translation.to_array().map(f32::to_bits),
        "rotationBits":p.rotation.to_array().map(f32::to_bits)})
}

fn profile_scope(profile: &crate::Profile) -> serde_json::Value {
    let crate::Profile {
        id,
        dt_s,
        tick_substeps,
        contact_friction_model,
        solver_iterations,
        internal_pgs_iterations,
        fall_internal_pgs_iterations,
        additional_body_iterations,
        allowed_linear_error_m,
        prediction_distance_m,
        motor_scale,
        aggregate_motor_cap_nm,
        angular_damping,
        contact_skin_m,
        max_ccd_substeps,
        disable_solver_speed_cap,
        posture_stiffness_multiplier,
        posture_damping_multiplier,
        supported_tilt_gain,
        supported_com_gain_rad_per_m,
        supported_com_velocity_gain_s_rad_per_m,
        supported_angular_velocity_gain_s,
        grab_limits,
        reaching_iterations,
        reaching_coordinate_increment_rad,
        reaching_damping_m2,
        corrective_steps_enabled,
        recovery_enabled,
    } = profile;
    let crate::grab::Limits {
        angular_frequency_rad_s,
        target_speed_m_s,
        target_acceleration_m_s2,
        force_n,
        torque_nm,
        positive_power_w,
    } = grab_limits;
    serde_json::json!({"id":id,"dtBits":dt_s.to_bits(),"tickSubsteps":tick_substeps,
        "contactFrictionModel":contact_friction_model,"solverIterations":solver_iterations,
        "internalPgsIterations":internal_pgs_iterations,"fallInternalPgsIterations":fall_internal_pgs_iterations,
        "additionalBodyIterations":additional_body_iterations,"allowedLinearErrorBits":allowed_linear_error_m.to_bits(),
        "predictionDistanceBits":prediction_distance_m.to_bits(),"motorScaleBits":motor_scale.to_bits(),
        "aggregateMotorCapBits":aggregate_motor_cap_nm.to_bits(),"angularDampingBits":angular_damping.to_bits(),
        "contactSkinBits":contact_skin_m.to_bits(),"maxCcdSubsteps":max_ccd_substeps,
        "disableSolverSpeedCap":disable_solver_speed_cap,"postureStiffnessBits":posture_stiffness_multiplier.to_bits(),
        "postureDampingBits":posture_damping_multiplier.to_bits(),"supportedTiltBits":supported_tilt_gain.to_bits(),
        "supportedComGainBits":supported_com_gain_rad_per_m.to_bits(),
        "supportedComVelocityGainBits":supported_com_velocity_gain_s_rad_per_m.to_bits(),
        "supportedAngularVelocityGainBits":supported_angular_velocity_gain_s.to_bits(),
        "grabLimitsBits":[angular_frequency_rad_s.to_bits(),target_speed_m_s.to_bits(),target_acceleration_m_s2.to_bits(),
            force_n.to_bits(),torque_nm.to_bits(),positive_power_w.to_bits()],
        "reachingIterations":reaching_iterations,"reachingCoordinateIncrementBits":reaching_coordinate_increment_rad.to_bits(),
        "reachingDampingBits":reaching_damping_m2.to_bits(),"correctiveStepsEnabled":corrective_steps_enabled,
        "recoveryEnabled":recovery_enabled})
}

/// Metadata projection only; no canonical construction, mass integration or
/// geometry query. Exhaustive patterns make added model fields a compile error.
fn model_scope(model: &lh_model::Model) -> serde_json::Value {
    let lh_model::Model {
        schema_version,
        total_mass_kg,
        stature_m,
        segments,
        playground,
        protocol,
    } = model;
    let vector = |v: lh_model::Vec3| {
        let lh_model::Vec3 { x, y, z } = v;
        [x.to_bits(), y.to_bits(), z.to_bits()]
    };
    let rotation = |q: lh_model::Quat| {
        let lh_model::Quat { x, y, z, w } = q;
        [x.to_bits(), y.to_bits(), z.to_bits(), w.to_bits()]
    };
    let frame = |f: &lh_model::Frame| {
        let lh_model::Frame {
            anchor,
            rotation: q,
        } = f;
        serde_json::json!({"anchorBits":vector(*anchor),"rotationBits":rotation(*q)})
    };
    let segments:Vec<_>=segments.iter().map(|segment| {
        let lh_model::Segment {id,region,parent,role,side,mass_kg,geometry,joint_profile,collision_exclusions,initial,golden_mass}=segment;
        let lh_model::GoldenMass {volume,center_of_mass,inertia}=golden_mass;
        let lh_model::Geometry {vertices,indices}=geometry;
        let lh_model::Pose{id:pose_id,position,rotation:q,linear_velocity,angular_velocity}=initial;
        let joint=joint_profile.as_ref().map(|j|{
            let lh_model::Joint{parent_frame,child_frame,axes,limit_soft_zone_fraction}=j;
            let axes:Vec<_>=axes.iter().map(|a|{
                let lh_model::Axis{coordinate,min_radians,max_radians,passive_stiffness_nm_per_rad,damping_nms_per_rad,max_motor_torque_nm}=a;
                serde_json::json!({"coordinate":coordinate,"minBits":min_radians.to_bits(),"maxBits":max_radians.to_bits(),
                    "passiveStiffnessBits":passive_stiffness_nm_per_rad.to_bits(),"dampingBits":damping_nms_per_rad.to_bits(),
                    "maxMotorTorqueBits":max_motor_torque_nm.to_bits()})
            }).collect();
            serde_json::json!({"parentFrame":frame(parent_frame),"childFrame":frame(child_frame),"axes":axes,
                "limitSoftZoneFractionBits":limit_soft_zone_fraction.to_bits()})
        });
        serde_json::json!({"id":id,"region":region,"parent":parent,"role":role,"side":side,
            "massBits":mass_kg.to_bits(),"geometry":{"vertexBits":vertices.iter().map(|v|v.to_bits()).collect::<Vec<_>>(),"indices":indices},
            "jointProfile":joint,"collisionExclusions":collision_exclusions,
            "initial":{"id":pose_id,"positionBits":vector(*position),"rotationBits":rotation(*q),
                "linearVelocityBits":vector(*linear_velocity),"angularVelocityBits":vector(*angular_velocity)},
            "goldenMass":{"volumeBits":volume.to_bits(),"centerOfMassBits":vector(*center_of_mass),
                "inertiaBits":inertia.map(|row|row.map(f64::to_bits))}})
    }).collect();
    serde_json::json!({"schemaVersion":schema_version,"totalMassBits":total_mass_kg.to_bits(),"statureBits":stature_m.to_bits(),
        "segments":segments,"playground":playground,"protocol":protocol})
}

pub(super) fn transfer_scope(
    s: &Simulation,
    owner: &super::whole_body_controller::Controller,
) -> Result<Vec<u8>, String> {
    serde_json::to_vec(&serde_json::json!({"schema":"transfer-native-kernel-v1",
        "candidate":owner.immutable_transfer_scope(),"model":model_scope(&s.model),"profile":profile_scope(&s.profile),
        "headingBits":s.heading.to_array().map(f32::to_bits)}))
    .map_err(|e| e.to_string())
}
fn simulation_transfer_scope(s: &Simulation) -> Result<Option<Vec<u8>>, PreviewError> {
    s.coordination
        .as_ref()
        .filter(|o| o.transfer.is_some())
        .map(|o| transfer_scope(s, o).map_err(PreviewError::Snapshot))
        .transpose()
}

pub(super) fn require_transfer_native_state(s: &Simulation) -> Result<(), String> {
    let active = s.motion == lh_contracts::Motion::Recovering
        && s.recovery.is_some()
        && s.counters.recoveries == 0;
    let complete = s.motion == lh_contracts::Motion::Upright
        && s.counters.recoveries == 1
        && s.recovery.is_none();
    if s.first_failure.is_some() || (!active && !complete) {
        return Err("Original native recovery observer failed or terminal state is invalid".into());
    }
    Ok(())
}

/// Frames can only be appended from actual Simulation states at consecutive
/// integrated substeps. The caller archives both traces with the frozen receipt.
#[derive(Debug, Serialize)]
pub(crate) struct NativeParityTrace {
    #[serde(skip)]
    simulation_identity: usize,
    initial: ParityFrame,
    frames: Vec<ParityFrame>,
    post_tick: Option<ParityFrame>,
    transfer_scope: Option<Vec<u8>>,
}
impl NativeParityTrace {
    pub(crate) fn begin(s: &Simulation, memory: &impl Serialize) -> Result<Self, PreviewError> {
        if s.tick.checked_mul(4) != Some(s.substeps_completed) {
            return Err(PreviewError::Snapshot(
                "Parity starts at a completed tick".into(),
            ));
        }
        Ok(Self {
            simulation_identity: std::ptr::from_ref(s) as usize,
            initial: parity_frame(s, memory)?,
            frames: Vec::new(),
            post_tick: None,
            transfer_scope: simulation_transfer_scope(s)?,
        })
    }
    pub(crate) fn record(
        &mut self,
        s: &Simulation,
        memory: &impl Serialize,
    ) -> Result<(), PreviewError> {
        if self.simulation_identity != std::ptr::from_ref(s) as usize {
            return Err(PreviewError::ParityMismatch(
                "Parity trace changed Simulation instance".into(),
            ));
        }
        let expected = self
            .initial
            .native_substep
            .checked_add(self.frames.len() as u64 + 1)
            .ok_or(PreviewError::ClockOverflow)?;
        if self.frames.len() >= 4 || s.substeps_completed != expected {
            return Err(PreviewError::ClockMismatch {
                expected,
                actual: s.substeps_completed,
            });
        }
        if self.transfer_scope != simulation_transfer_scope(s)? {
            return Err(PreviewError::ParityMismatch(
                "Candidate scope changed during native parity".into(),
            ));
        }
        self.frames.push(parity_frame(s, memory)?);
        Ok(())
    }

    /// Must run after advance_tick_observed returns successfully, since its
    /// fourth callback precedes final persistence/recovery/tick bookkeeping.
    pub(crate) fn record_post_tick(
        &mut self,
        s: &Simulation,
        memory: &impl Serialize,
    ) -> Result<(), PreviewError> {
        if self.simulation_identity != std::ptr::from_ref(s) as usize
            || self.frames.len() != 4
            || self.post_tick.is_some()
            || self.initial.native_substep.checked_add(4) != Some(s.substeps_completed)
            || s.tick.checked_mul(4) != Some(s.substeps_completed)
        {
            return Err(PreviewError::ParityMismatch(
                "Post-tick parity requires the same completed four-step instance".into(),
            ));
        }
        if self.transfer_scope != simulation_transfer_scope(s)? {
            return Err(PreviewError::ParityMismatch(
                "Candidate scope changed at native parity endpoint".into(),
            ));
        }
        self.post_tick = Some(parity_frame(s, memory)?);
        Ok(())
    }
}

/// Compare every available actual frame immediately. The predicted trace may
/// already contain four frames, but the real run must stop on its first mismatch.
pub(crate) fn compare_prefix(
    predicted: &NativeParityTrace,
    actual: &NativeParityTrace,
) -> Result<(), PreviewError> {
    if predicted.simulation_identity == actual.simulation_identity {
        return Err(PreviewError::ParityMismatch(
            "Parity requires two distinct Simulation instances".into(),
        ));
    }
    if predicted.transfer_scope != actual.transfer_scope
        || actual.frames.len() > predicted.frames.len()
        || predicted.initial != actual.initial
        || predicted.frames[..actual.frames.len()] != actual.frames
        || actual.post_tick.is_some() && predicted.post_tick != actual.post_tick
    {
        return Err(PreviewError::ParityMismatch(
            "Native state or memory differs at the current real prefix".into(),
        ));
    }
    Ok(())
}

pub(crate) fn native_kernel_admission(
    predicted: &NativeParityTrace,
    actual: &NativeParityTrace,
) -> Result<NativeKernelAdmission, PreviewError> {
    if predicted.frames.len() != 4 || actual.frames.len() != 4 {
        return Err(PreviewError::NativeKernelParityUnavailable);
    }
    compare_prefix(predicted, actual)?;
    if predicted.post_tick != actual.post_tick
        || (actual.transfer_scope.is_some() && actual.post_tick.is_none())
    {
        return Err(PreviewError::ParityMismatch(
            "Post-tick parity boundary differs".into(),
        ));
    }
    Ok(NativeKernelAdmission {
        initial_substep: actual.initial.native_substep,
        verified_substeps: 4,
        transfer_scope: actual.transfer_scope.clone(),
    })
}

/// Long-run admission requires the final tick bookkeeping on both instances.
/// The old four-callback API remains available to archived numeric-worker code.
pub(crate) fn native_kernel_admission_complete(
    predicted: &NativeParityTrace,
    actual: &NativeParityTrace,
) -> Result<NativeKernelAdmission, PreviewError> {
    if predicted.post_tick.is_none() || actual.post_tick.is_none() {
        return Err(PreviewError::NativeKernelParityUnavailable);
    }
    native_kernel_admission(predicted, actual)
}

/// Every field of Simulation is copied explicitly. New fields require a compile
/// error here instead of silently disappearing from the forecast's state.
pub(crate) fn copy_static_snapshot(source: &Simulation) -> Result<Simulation, PreviewError> {
    let refuse = |s: &str| PreviewError::Snapshot(s.into());
    super::contact_observation::require_inactive().map_err(PreviewError::Snapshot)?;
    if source.grab.is_some() || source.striker.is_some() || source.environment.is_some() {
        return Err(refuse(
            "Preview admits only a static scene without interactions",
        ));
    }
    if source.first_failure.is_some() || source.motion == lh_contracts::Motion::Halted {
        return Err(refuse(
            "Interrupted/failed histories are not preview snapshots",
        ));
    }
    if source.profile.dt_s != lh_contracts::SUBSTEP_S || source.profile.tick_substeps != 4 {
        return Err(refuse(
            "Preview requires the original 240 Hz / four-substep clock",
        ));
    }
    if source.tick.checked_mul(4) != Some(source.substeps_completed) {
        return Err(refuse(
            "Snapshot requires a completed tick; intra-tick accumulators are local",
        ));
    }
    if source.model.segments.len() != 25
        || source.bodies.len() != 25
        || source.colliders.len() != 25
        || source.joints.len() != 25
        || source.world.bodies.len() != 25
        || source.world.colliders.len() != 26
        || source.world.multibody_joints.iter().next().is_some()
        || source
            .bodies
            .iter()
            .any(|h| source.world.bodies.get(*h).is_none())
        || source
            .colliders
            .iter()
            .any(|h| source.world.colliders.get(*h).is_none())
        || !source
            .world
            .colliders
            .get(source.floor)
            .is_some_and(|floor| floor.parent().is_none() && floor.is_enabled())
    {
        return Err(refuse(
            "Preview requires the complete native body and one enabled static floor",
        ));
    }
    // The Rapier pipeline contains transient workspace, not persistent physical
    // state. This follows the previously audited copy_world boundary, preserving
    // all contact, joint, CCD, broad-phase and island caches without serde loss.
    let world = PhysicsWorld {
        gravity: source.world.gravity,
        integration_parameters: source.world.integration_parameters,
        physics_pipeline: PhysicsPipeline::new(),
        islands: source.world.islands.clone(),
        broad_phase: source.world.broad_phase.clone(),
        narrow_phase: source.world.narrow_phase.clone(),
        bodies: source.world.bodies.clone(),
        colliders: source.world.colliders.clone(),
        impulse_joints: source.world.impulse_joints.clone(),
        multibody_joints: source.world.multibody_joints.clone(),
        ccd_solver: source.world.ccd_solver.clone(),
    };
    Ok(Simulation {
        world,
        model: source.model.clone(),
        bodies: source.bodies.clone(),
        colliders: source.colliders.clone(),
        joints: source.joints.clone(),
        floor: source.floor,
        filter: source.filter.clone(),
        profile: source.profile.clone(),
        tick: source.tick,
        generation: source.generation,
        motion: source.motion,
        counters: source.counters.clone(),
        targets: source.targets.clone(),
        rest_targets: source.rest_targets.clone(),
        rest_com: source.rest_com,
        rest_com_roundoff: source.rest_com_roundoff,
        heading: source.heading,
        grab: None,
        last_press: source.last_press,
        applied_sequence: source.applied_sequence,
        contacts: source.contacts.clone(),
        persistence: source.persistence.clone(),
        integrated_support_substeps: source.integrated_support_substeps,
        integrated_support_loads_n: source.integrated_support_loads_n,
        diagnostics: source.diagnostics.clone(),
        first_failure: source.first_failure.clone(),
        motor_ceiling_sum: source.motor_ceiling_sum,
        substeps_completed: source.substeps_completed,
        stepping: source.stepping.clone(),
        motion_observation: source.motion_observation.clone(),
        environment: None,
        striker: None,
        recovery: source.recovery.clone(),
        coordination: source.coordination.clone(),
    })
}

/// M must have value semantics: cloning it must not share mutable controller
/// state with the real history. The common runtime Memory is a value-only type;
/// Clone alone cannot guarantee this for an arbitrary Arc<Mutex<_>> supplied
/// by a future caller. Immutable shared plan data is allowed.
pub(crate) struct StaticPreviewBranch<M> {
    simulation: Simulation,
    memory: M,
    completed_preview: Option<(u64, u32)>,
    requested_preview: Option<(u64, u32)>,
    preview_purpose: Option<PreviewPurpose>,
}

impl<M: Clone> StaticPreviewBranch<M> {
    pub(crate) fn capture(source: &Simulation, memory: &M) -> Result<Self, PreviewError> {
        if let Some(owner) = &source.coordination {
            if let Some(adapter) = &owner.transfer {
                require_transfer_native_state(source).map_err(PreviewError::Snapshot)?;
                adapter
                    .check_native_terminal(
                        source.substeps_completed,
                        source.counters.recoveries == 1,
                    )
                    .map_err(PreviewError::Snapshot)?;
                let identity = transfer_identity(source, owner).map_err(PreviewError::Snapshot)?;
                adapter
                    .require_window_checkpoint(source.substeps_completed, &identity)
                    .map_err(PreviewError::Snapshot)?;
            }
        }
        Ok(Self {
            simulation: copy_static_snapshot(source)?,
            memory: memory.clone(),
            completed_preview: None,
            requested_preview: None,
            preview_purpose: None,
        })
    }

    pub(crate) fn simulation(&self) -> &Simulation {
        &self.simulation
    }
    pub(crate) fn memory(&self) -> &M {
        &self.memory
    }

    /// A receipt can only be made from this branch's completed native state,
    /// never from JSON fixtures or a caller-supplied success boolean.
    pub(crate) fn certify_transfer_horizon(
        &self,
        kernel: &NativeKernelAdmission,
        start: u64,
        steps: u32,
    ) -> Result<NativeWindowReceipt, PreviewError> {
        kernel.require_scope(simulation_transfer_scope(&self.simulation)?.as_deref())?;
        if self.preview_purpose != Some(PreviewPurpose::Candidate)
            || self.requested_preview != Some((start, steps))
        {
            return Err(PreviewError::Snapshot(
                "Native receipt request differs from its reserved horizon".into(),
            ));
        }
        let (completed_start, completed_steps) = self.completed_preview.ok_or_else(|| {
            PreviewError::Snapshot("No successful complete native horizon for this receipt".into())
        })?;
        if completed_start != start || completed_steps > steps {
            return Err(PreviewError::Snapshot(
                "Native completed horizon differs from request".into(),
            ));
        }
        require_transfer_native_state(&self.simulation).map_err(PreviewError::Snapshot)?;
        if self.simulation.first_failure.is_some()
            || self.simulation.tick.checked_mul(4) != Some(self.simulation.substeps_completed)
        {
            return Err(PreviewError::Snapshot(
                "Transfer horizon did not complete cleanly".into(),
            ));
        }
        let owner = self.simulation.coordination.as_ref().ok_or_else(|| {
            PreviewError::Snapshot("Transfer owner absent at forecast end".into())
        })?;
        let observed = super::native_observation::observe(&self.simulation, owner.start, &[])
            .map_err(PreviewError::Snapshot)?;
        let adapter = owner.transfer.as_ref().ok_or_else(|| {
            PreviewError::Snapshot("No transfer adapter in native forecast".into())
        })?;
        adapter
            .check_native_terminal(
                self.simulation.substeps_completed,
                self.simulation.counters.recoveries == 1,
            )
            .map_err(PreviewError::Snapshot)?;
        let data = adapter
            .complete_native_preview(start, completed_steps, &observed)
            .map_err(PreviewError::Snapshot)?;
        Ok(NativeWindowReceipt { data })
    }

    /// Bind a completed copied verification tick to one real counterpart.
    /// Its durable four-step reserve is enforced separately by the native Gate;
    /// creating this receipt neither integrates nor grants budget credit.
    pub(crate) fn certify_technical_transfer_tick(
        &self,
        actual: &Simulation,
    ) -> Result<NativeTechnicalReceipt, PreviewError> {
        let (start, steps) = self
            .completed_preview
            .ok_or(PreviewError::NativeKernelParityUnavailable)?;
        if steps != 4
            || self.preview_purpose != Some(PreviewPurpose::ParityVerification)
            || self.requested_preview != Some((start, 4))
            || actual.substeps_completed != start
            || actual.tick.checked_mul(4) != Some(start)
            || std::ptr::from_ref(actual) == std::ptr::from_ref(&self.simulation)
        {
            return Err(PreviewError::NativeKernelParityUnavailable);
        }
        require_transfer_native_state(actual).map_err(PreviewError::Snapshot)?;
        require_transfer_native_state(&self.simulation).map_err(PreviewError::Snapshot)?;
        let predicted_scope = simulation_transfer_scope(&self.simulation)?
            .ok_or(PreviewError::NativeKernelParityUnavailable)?;
        if simulation_transfer_scope(actual)?.as_ref() != Some(&predicted_scope) {
            return Err(PreviewError::ParityMismatch(
                "Technical counterpart candidate scope differs".into(),
            ));
        }
        let owner = self
            .simulation
            .coordination
            .as_ref()
            .ok_or(PreviewError::NativeKernelParityUnavailable)?;
        let data = owner
            .transfer
            .as_ref()
            .ok_or(PreviewError::NativeKernelParityUnavailable)?
            .technical_data(start, std::ptr::from_ref(actual) as usize, predicted_scope)
            .map_err(PreviewError::Snapshot)?;
        // The adapter also checks this immutable external entry during admission.
        Ok(NativeTechnicalReceipt { data })
    }

    /// Requested reservation and actual integration remain distinct when a
    /// terminal tick ends the forecast early. No unused reservation is refunded.
    pub(crate) fn transfer_horizon_evidence(&self) -> serde_json::Value {
        serde_json::json!({"requested":self.requested_preview,
            "successfulCompletedHorizon":self.completed_preview,
            "actualNativeSteps":self.requested_preview.and_then(|(start,_)|self.simulation.substeps_completed.checked_sub(start))})
    }

    pub(crate) fn set_feedback_scale(&mut self, scale: f64) -> Result<(), PreviewError> {
        if ![0.0, 0.5, 1.0, 1.5].contains(&scale) {
            return Err(PreviewError::Snapshot(
                "Feedback proposal outside the frozen family".into(),
            ));
        }
        let owner =
            self.simulation.coordination.as_mut().ok_or_else(|| {
                PreviewError::Snapshot("Missing copied coordination owner".into())
            })?;
        owner.feedback_scale = scale;
        Ok(())
    }

    /// Reuse the exact existing native tick kernel, including all original
    /// observers, integrity checks and clocks. The callback only observes the
    /// copied simulation; decision/installation must belong to the same explicit
    /// Simulation owner used in the actual history. No recursive plan search.
    /// This method remains inaccessible without measured native admission.
    pub(crate) fn preview<O>(
        &mut self,
        admission: &NativeKernelAdmission,
        branch_id: u64,
        steps: u32,
        budget: &mut impl PreviewBudget,
        observe: impl FnMut(&Simulation, u32, &mut M) -> Result<O, String>,
    ) -> PreviewRun<O> {
        let scope = match simulation_transfer_scope(&self.simulation) {
            Ok(scope) => scope,
            Err(error) => return PreviewRun::refused(error),
        };
        if let Err(error) = admission.require_scope(scope.as_deref()) {
            return PreviewRun::refused(error);
        }
        if !(120..=480).contains(&steps) || steps % 4 != 0 {
            return PreviewRun::refused(PreviewError::InvalidHorizon);
        }
        self.preview_ticks(
            PreviewPurpose::Candidate,
            branch_id,
            steps,
            budget,
            observe,
            |_, _| Ok(()),
        )
    }

    pub(crate) fn preview_with_post_tick<O>(
        &mut self,
        admission: &NativeKernelAdmission,
        branch_id: u64,
        steps: u32,
        budget: &mut impl PreviewBudget,
        observe: impl FnMut(&Simulation, u32, &mut M) -> Result<O, String>,
        after_tick: impl FnMut(&Simulation, &mut M) -> Result<(), String>,
    ) -> PreviewRun<O> {
        let scope = match simulation_transfer_scope(&self.simulation) {
            Ok(scope) => scope,
            Err(error) => return PreviewRun::refused(error),
        };
        if let Err(error) = admission.require_scope(scope.as_deref()) {
            return PreviewRun::refused(error);
        }
        if !(120..=480).contains(&steps) || steps % 4 != 0 {
            return PreviewRun::refused(PreviewError::InvalidHorizon);
        }
        self.preview_ticks(
            PreviewPurpose::Candidate,
            branch_id,
            steps,
            budget,
            observe,
            after_tick,
        )
    }

    /// Only the ledger's frozen verification phase may use this entrypoint.
    /// Its four steps are charged to technical verification, not free trials.
    /// Histories/candidate selection still require a measured parity receipt.
    pub(crate) fn verify_tick<O>(
        &mut self,
        branch_id: u64,
        budget: &mut impl PreviewBudget,
        observe: impl FnMut(&Simulation, u32, &mut M) -> Result<O, String>,
    ) -> PreviewRun<O> {
        self.preview_ticks(
            PreviewPurpose::ParityVerification,
            branch_id,
            4,
            budget,
            observe,
            |_, _| Ok(()),
        )
    }

    pub(crate) fn verify_tick_with_post_tick<O>(
        &mut self,
        branch_id: u64,
        budget: &mut impl PreviewBudget,
        observe: impl FnMut(&Simulation, u32, &mut M) -> Result<O, String>,
        after_tick: impl FnMut(&Simulation, &mut M) -> Result<(), String>,
    ) -> PreviewRun<O> {
        self.preview_ticks(
            PreviewPurpose::ParityVerification,
            branch_id,
            4,
            budget,
            observe,
            after_tick,
        )
    }

    fn preview_ticks<O>(
        &mut self,
        purpose: PreviewPurpose,
        branch_id: u64,
        steps: u32,
        budget: &mut impl PreviewBudget,
        mut observe: impl FnMut(&Simulation, u32, &mut M) -> Result<O, String>,
        mut after_tick: impl FnMut(&Simulation, &mut M) -> Result<(), String>,
    ) -> PreviewRun<O> {
        if let Err(error) = budget.admit_preview(purpose, steps) {
            return PreviewRun::refused(PreviewError::Reservation(error));
        }
        self.completed_preview = None;
        let run_start = self.simulation.substeps_completed;
        self.requested_preview = Some((run_start, steps));
        self.preview_purpose = Some(purpose);
        if let Some(owner) = &self.simulation.coordination {
            if owner.transfer.is_some() {
                let identity = match transfer_identity(&self.simulation, owner) {
                    Ok(identity) => identity,
                    Err(error) => return PreviewRun::refused(PreviewError::Snapshot(error)),
                };
                self.simulation
                    .coordination
                    .as_mut()
                    .unwrap()
                    .transfer
                    .as_mut()
                    .unwrap()
                    .begin_native_preview(run_start, identity);
            }
        }
        let mut run = PreviewRun {
            records: Vec::new(),
            refusal: None,
        };
        for preview_index in (0..steps).step_by(4) {
            if let Err(error) = self.simulation.require_step_context() {
                run.refusal = Some(PreviewError::Snapshot(error));
                break;
            }
            let before_substep = self.simulation.substeps_completed;
            if self.simulation.tick.checked_mul(4) != Some(before_substep) {
                run.refusal = Some(PreviewError::Snapshot(
                    "Native preview requires a complete tick boundary".into(),
                ));
                break;
            }
            let Some(expected) = before_substep.checked_add(4) else {
                run.refusal = Some(PreviewError::ClockOverflow);
                break;
            };
            if let Err(error) = budget.reserve_four_before_tick(StepReservation {
                branch_id,
                before_substep,
                preview_index,
                purpose,
            }) {
                run.refusal = Some(PreviewError::Reservation(error));
                break;
            }
            // The common adapter executes only after the durable reservation.
            // Bind to THIS private copied instance, never the source history.
            let instance = std::ptr::from_ref(&self.simulation) as usize;
            if let Some(adapter) = self
                .simulation
                .coordination
                .as_mut()
                .and_then(|o| o.transfer.as_mut())
            {
                if let Err(error) = adapter.preview_window(instance, before_substep) {
                    run.refusal = Some(PreviewError::Snapshot(error));
                    break;
                }
            }
            let memory = &mut self.memory;
            let result = self.simulation.advance_tick_observed(|state, substep| {
                let fail = |check, reason| {
                    crate::measurement::Failure::new(state.tick, substep, check, reason, 1.0, 0.0)
                };
                if state.substeps_completed != before_substep + u64::from(substep) + 1 {
                    return Err(fail(
                        "preview-substep-clock",
                        "Native observer clock mismatch".into(),
                    ));
                }
                state
                    .require_step_context()
                    .map_err(|e| fail("preview-context", e))?;
                let record =
                    observe(state, substep, memory).map_err(|e| fail("preview-observer", e))?;
                run.records.push(record);
                // The first deadline check belongs to the reservation. Later
                // checks occur after each original observer, before continuing
                // to another integration. An in-flight native step cannot be
                // interrupted, and its full reservation is already consumed.
                budget
                    .check_wall_deadline()
                    .map_err(|e| fail("preview-wall-budget", e))?;
                Ok(())
            });
            if let Err(failure) = result {
                run.refusal = Some(PreviewError::Kernel(format!("{failure:?}")));
                break;
            }
            if self.simulation.substeps_completed != expected {
                run.refusal = Some(PreviewError::ClockMismatch {
                    expected,
                    actual: self.simulation.substeps_completed,
                });
                break;
            }
            let mut terminal_complete = false;
            if let Some(owner) = &self.simulation.coordination {
                if let Some(adapter) = &owner.transfer {
                    terminal_complete = match adapter.terminal_monitoring_complete(
                        expected,
                        self.simulation.counters.recoveries == 1,
                    ) {
                        Ok(complete) => complete,
                        Err(error) => {
                            run.refusal = Some(PreviewError::Snapshot(error));
                            break;
                        }
                    };
                    let identity = match transfer_identity(&self.simulation, owner) {
                        Ok(identity) => identity,
                        Err(error) => {
                            run.refusal = Some(PreviewError::Snapshot(error));
                            break;
                        }
                    };
                    if let Err(error) = self
                        .simulation
                        .coordination
                        .as_mut()
                        .unwrap()
                        .transfer
                        .as_mut()
                        .unwrap()
                        .record_preview_post_tick(
                            expected,
                            identity,
                            terminal_complete || preview_index + 4 == steps,
                        )
                    {
                        run.refusal = Some(PreviewError::Snapshot(error));
                        break;
                    }
                }
            }
            if let Err(error) = after_tick(&self.simulation, &mut self.memory) {
                run.refusal = Some(PreviewError::Kernel(format!("Post-tick observer: {error}")));
                break;
            }
            if let Err(error) = budget.check_wall_deadline() {
                run.refusal = Some(PreviewError::Reservation(error));
                break;
            }
            if terminal_complete {
                break;
            }
        }
        if run.refusal.is_none() && !run.records.is_empty() && run.records.len() % 4 == 0 {
            self.completed_preview = Some((run_start, run.records.len() as u32));
        }
        run
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PreviewPurpose {
    ParityVerification,
    Candidate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct StepReservation {
    pub branch_id: u64,
    /// Absolute physical clock of the copied history, before the next step.
    pub before_substep: u64,
    pub preview_index: u32,
    pub purpose: PreviewPurpose,
}

/// Implement with the ONE durable batch ledger, not a fresh per-copy counter.
/// Check the global wall deadline and reserve one step durably BEFORE returning
/// Ok. No cancellation/refund is provided: failures retain their reservation.
pub(crate) trait PreviewBudget {
    /// Must verify the frozen prerequisites and the batch phase. Verification
    /// requires its dedicated technical allowance; Candidate requires that the
    /// ledger has accepted the measured parity receipt for this same frozen run.
    fn admit_preview(&mut self, purpose: PreviewPurpose, steps: u32) -> Result<(), String>;
    fn reserve_one_before_step(&mut self, request: StepReservation) -> Result<(), String>;
    /// Check only: never reserves again for an already-reserved substep.
    fn check_wall_deadline(&mut self) -> Result<(), String>;
    /// A ledger adapter may override with one atomic four-step reservation.
    /// Default reservations all precede integration; partial reservation failure
    /// executes no tick and intentionally does not refund earlier reservations.
    fn reserve_four_before_tick(&mut self, request: StepReservation) -> Result<(), String> {
        request
            .before_substep
            .checked_add(4)
            .ok_or("Preview clock overflow")?;
        request
            .preview_index
            .checked_add(3)
            .ok_or("Preview index overflow")?;
        self.check_wall_deadline()?;
        for offset in 0..4_u32 {
            self.reserve_one_before_step(StepReservation {
                branch_id: request.branch_id,
                before_substep: request.before_substep + u64::from(offset),
                preview_index: request.preview_index + offset,
                purpose: request.purpose,
            })?;
        }
        Ok(())
    }
}

trait NativeStepClock {
    fn completed_native_substeps(&self) -> u64;
    fn require_step_context(&self) -> Result<(), String> {
        Ok(())
    }
}
impl NativeStepClock for Simulation {
    fn completed_native_substeps(&self) -> u64 {
        self.substeps_completed
    }
    fn require_step_context(&self) -> Result<(), String> {
        super::contact_observation::require_inactive()?;
        if self.grab.is_some()
            || self.striker.is_some()
            || self.environment.is_some()
            || self.first_failure.is_some()
            || self.motion == lh_contracts::Motion::Halted
        {
            return Err("Static preview context changed or history stopped".into());
        }
        Ok(())
    }
}

pub(crate) struct PreviewRun<O> {
    pub records: Vec<O>,
    /// Completed records remain available on refusal; the branch retains the
    /// last state for failure archiving. Nothing is transplanted into reality.
    pub refusal: Option<PreviewError>,
}
impl<O> PreviewRun<O> {
    fn refused(error: PreviewError) -> Self {
        Self {
            records: Vec::new(),
            refusal: Some(error),
        }
    }
}

/// Private generic shell is exercised only with fake clocks in this version.
/// Counter equality detects a broken callback; it cannot undo an over-integrating
/// callback. The closed native gate is essential until an exactly-one-step shared
/// kernel exists and is certified, rather than trusting an arbitrary closure.
fn run_guarded_steps<S: NativeStepClock, M, O>(
    state: &mut S,
    memory: &mut M,
    branch_id: u64,
    steps: u32,
    budget: &mut impl PreviewBudget,
    mut kernel: impl FnMut(&mut S, &mut M) -> Result<O, String>,
) -> PreviewRun<O> {
    if steps == 0 || steps > 480 {
        return PreviewRun::refused(PreviewError::InvalidHorizon);
    }
    let mut run = PreviewRun {
        records: Vec::new(),
        refusal: None,
    };
    for preview_index in 0..steps {
        if let Err(error) = state.require_step_context() {
            run.refusal = Some(PreviewError::Snapshot(error));
            break;
        }
        let before_substep = state.completed_native_substeps();
        let Some(expected) = before_substep.checked_add(1) else {
            run.refusal = Some(PreviewError::ClockOverflow);
            break;
        };
        if let Err(error) = budget.reserve_one_before_step(StepReservation {
            branch_id,
            before_substep,
            preview_index,
            purpose: PreviewPurpose::ParityVerification,
        }) {
            run.refusal = Some(PreviewError::Reservation(error));
            break;
        }
        let result = kernel(state, memory);
        let actual = state.completed_native_substeps();
        if actual != expected && !(actual == before_substep && result.is_err()) {
            run.refusal = Some(PreviewError::ClockMismatch { expected, actual });
            break;
        }
        match result {
            Ok(record) => run.records.push(record),
            Err(error) => {
                run.refusal = Some(PreviewError::Kernel(error));
                break;
            }
        }
    }
    run
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pure_compact_identity_is_length_delimited_and_bit_exact() {
        let frame = ParityFrame {
            native_substep: 36,
            world: b"a".to_vec(),
            state: b"bc".to_vec(),
            owner_memory: vec![],
            body_and_motor_bits: vec![(-0.0_f32).to_bits(), 0x7fc00001],
        };
        let before = frame.exact_identity();
        assert_eq!(
            before.len(),
            b"LH-native-transfer-identity-v2\0".len() + 8 + 4 * 8 + 3 + 2 * 4
        );
        let mut changed = frame.clone();
        changed.world = b"ab".to_vec();
        changed.state = b"c".to_vec();
        assert_ne!(
            before,
            changed.exact_identity(),
            "Concatenation boundaries must be bound"
        );
        let mut changed = frame.clone();
        changed.native_substep += 1;
        assert_ne!(before, changed.exact_identity());
        let mut changed = frame.clone();
        changed.owner_memory.push(0);
        assert_ne!(before, changed.exact_identity());
        let mut changed = frame.clone();
        changed.body_and_motor_bits[0] = 0.0_f32.to_bits();
        assert_ne!(before, changed.exact_identity());
        let mut changed = frame.clone();
        changed.body_and_motor_bits[1] = 0x7fc00002;
        assert_ne!(before, changed.exact_identity());
        let mut changed = frame.clone();
        changed.world[0] ^= 1;
        assert_ne!(before, changed.exact_identity());
        let mut changed = frame.clone();
        changed.state[0] ^= 1;
        assert_ne!(before, changed.exact_identity());
        assert_eq!(before, frame.clone().exact_identity());
    }

    #[test]
    fn pure_profile_projection_is_exhaustive_and_preserves_float_bits() {
        let original = crate::Profile::default();
        let before = profile_scope(&original);
        macro_rules! changed {
            ($c:ident,$edit:expr) => {{
                let mut $c = original.clone();
                $edit;
                assert_ne!(before, profile_scope(&$c), stringify!($edit));
            }};
        }
        changed!(c, c.id.push('x'));
        changed!(c, c.dt_s = 0.);
        changed!(c, c.tick_substeps += 1);
        changed!(
            c,
            c.contact_friction_model = crate::ContactFriction::Simplified
        );
        changed!(c, c.solver_iterations += 1);
        changed!(c, c.internal_pgs_iterations += 1);
        changed!(c, c.fall_internal_pgs_iterations += 1);
        changed!(c, c.additional_body_iterations += 1);
        changed!(c, c.allowed_linear_error_m = 0.);
        changed!(c, c.prediction_distance_m = 0.);
        changed!(c, c.motor_scale = 0.);
        changed!(c, c.aggregate_motor_cap_nm = 0.);
        changed!(c, c.angular_damping = 0.);
        changed!(c, c.contact_skin_m = 0.);
        changed!(c, c.max_ccd_substeps += 1);
        changed!(c, c.disable_solver_speed_cap = false);
        changed!(c, c.posture_stiffness_multiplier = 0.);
        changed!(c, c.posture_damping_multiplier = 0.);
        changed!(c, c.supported_tilt_gain = 0.);
        changed!(c, c.supported_com_gain_rad_per_m = 0.);
        changed!(c, c.supported_com_velocity_gain_s_rad_per_m = 0.);
        changed!(c, c.supported_angular_velocity_gain_s = 0.);
        changed!(c, c.grab_limits.force_n = 0.);
        changed!(c, c.grab_limits.torque_nm = 0.);
        changed!(c, c.grab_limits.positive_power_w = 0.);
        changed!(c, c.grab_limits.angular_frequency_rad_s = 0.);
        changed!(c, c.grab_limits.target_speed_m_s = 0.);
        changed!(c, c.grab_limits.target_acceleration_m_s2 = 0.);
        changed!(c, c.reaching_iterations += 1);
        changed!(c, c.reaching_coordinate_increment_rad = 0.);
        changed!(c, c.reaching_damping_m2 = 0.);
        changed!(c, c.corrective_steps_enabled = true);
        changed!(c, c.recovery_enabled = true);
        let mut zero = original.clone();
        zero.motor_scale = 0.;
        let mut negative = zero.clone();
        negative.motor_scale = -0.;
        assert_ne!(profile_scope(&zero), profile_scope(&negative));
        for bits in [0x7fc00001, 0x7fc00002] {
            let mut c = original.clone();
            c.motor_scale = f32::from_bits(bits);
            assert_eq!(profile_scope(&c)["motorScaleBits"], bits);
        }
    }

    #[test]
    fn pure_model_projection_binds_all_metadata_without_constructing_a_world() {
        // Metadata deserialization only: canonical()/validate() would integrate
        // mass properties and are deliberately not called by this test.
        let original: lh_model::Model = serde_json::from_str(lh_model::CANONICAL_JSON).unwrap();
        let before = model_scope(&original);
        macro_rules! changed {
            ($c:ident,$edit:expr) => {{
                let mut $c = original.clone();
                $edit;
                assert_ne!(before, model_scope(&$c), stringify!($edit));
            }};
        }
        changed!(c, c.schema_version += 1);
        changed!(c, c.total_mass_kg += 1.);
        changed!(c, c.stature_m += 1.);
        changed!(c, c.playground = serde_json::Value::Null);
        changed!(c, c.protocol = serde_json::Value::Null);
        changed!(c, c.segments[1].id.push('x'));
        changed!(c, c.segments[1].region = Some("changed".into()));
        changed!(c, c.segments[1].parent = None);
        changed!(c, c.segments[1].role.push('x'));
        changed!(c, c.segments[1].side = Some("changed".into()));
        changed!(c, c.segments[1].mass_kg += 1.);
        changed!(c, c.segments[1].geometry.vertices[0] += 1.);
        changed!(c, c.segments[1].geometry.indices[0] += 1);
        changed!(c, c.segments[1].collision_exclusions.push("changed".into()));
        changed!(c, c.segments[1].initial.id.push('x'));
        changed!(c, c.segments[1].initial.position.x += 1.);
        changed!(c, c.segments[1].initial.rotation.w += 1.);
        changed!(c, c.segments[1].initial.linear_velocity.x += 1.);
        changed!(c, c.segments[1].initial.angular_velocity.x += 1.);
        changed!(c, c.segments[1].golden_mass.volume += 1.);
        changed!(c, c.segments[1].golden_mass.center_of_mass.x += 1.);
        changed!(c, c.segments[1].golden_mass.inertia[0][0] += 1.);
        changed!(c, c.segments[1].joint_profile = None);
        changed!(
            c,
            c.segments[1]
                .joint_profile
                .as_mut()
                .unwrap()
                .parent_frame
                .anchor
                .x += 1.
        );
        changed!(
            c,
            c.segments[1]
                .joint_profile
                .as_mut()
                .unwrap()
                .parent_frame
                .rotation
                .w += 1.
        );
        changed!(
            c,
            c.segments[1]
                .joint_profile
                .as_mut()
                .unwrap()
                .child_frame
                .anchor
                .x += 1.
        );
        changed!(
            c,
            c.segments[1]
                .joint_profile
                .as_mut()
                .unwrap()
                .child_frame
                .rotation
                .w += 1.
        );
        changed!(
            c,
            c.segments[1]
                .joint_profile
                .as_mut()
                .unwrap()
                .limit_soft_zone_fraction += 1.
        );
        changed!(
            c,
            c.segments[1].joint_profile.as_mut().unwrap().axes[0].coordinate = 'z'
        );
        changed!(
            c,
            c.segments[1].joint_profile.as_mut().unwrap().axes[0].min_radians += 1.
        );
        changed!(
            c,
            c.segments[1].joint_profile.as_mut().unwrap().axes[0].max_radians += 1.
        );
        changed!(
            c,
            c.segments[1].joint_profile.as_mut().unwrap().axes[0].passive_stiffness_nm_per_rad +=
                1.
        );
        changed!(
            c,
            c.segments[1].joint_profile.as_mut().unwrap().axes[0].damping_nms_per_rad += 1.
        );
        changed!(
            c,
            c.segments[1].joint_profile.as_mut().unwrap().axes[0].max_motor_torque_nm += 1.
        );
        let mut c = original.clone();
        c.segments[1].golden_mass.volume = f64::from_bits(0x7ff8000000000001);
        assert_eq!(
            model_scope(&c)["segments"][1]["goldenMass"]["volumeBits"],
            0x7ff8000000000001_u64
        );
        let mut d = c.clone();
        d.segments[1].golden_mass.volume = f64::from_bits(0x7ff8000000000002);
        assert_ne!(model_scope(&c), model_scope(&d));
        c.segments[1].mass_kg = 0.;
        d = c.clone();
        d.segments[1].mass_kg = -0.;
        assert_ne!(model_scope(&c), model_scope(&d));
    }

    fn synthetic_parity(identity: usize, count: usize, post_tick: bool) -> NativeParityTrace {
        let frame = |step| ParityFrame {
            native_substep: step,
            world: serde_json::to_vec(&u128::MAX).unwrap(),
            state: vec![step as u8],
            owner_memory: vec![42],
            body_and_motor_bits: vec![(-0.0_f32).to_bits()],
        };
        NativeParityTrace {
            simulation_identity: identity,
            initial: frame(0),
            frames: (1..=count).map(|step| frame(step as u64)).collect(),
            post_tick: post_tick.then(|| frame(4)),
            transfer_scope: None,
        }
    }

    #[test]
    fn pure_transfer_kernel_scope_requires_new_complete_matching_candidate_parity() {
        let legacy = native_kernel_admission_complete(
            &synthetic_parity(1, 4, true),
            &synthetic_parity(2, 4, true),
        )
        .unwrap();
        assert!(
            legacy
                .require_scope(Some(b"candidate-entry-source-A"))
                .is_err()
        );
        legacy.require_scope(None).unwrap();
        let mut predicted = synthetic_parity(1, 4, true);
        let mut actual = synthetic_parity(2, 4, true);
        predicted.transfer_scope = Some(b"candidate-entry-source-A".to_vec());
        actual.transfer_scope = Some(b"candidate-entry-source-B".to_vec());
        assert!(native_kernel_admission_complete(&predicted, &actual).is_err());
        actual.transfer_scope = predicted.transfer_scope.clone();
        let token = native_kernel_admission_complete(&predicted, &actual).unwrap();
        token
            .require_scope(Some(b"candidate-entry-source-A"))
            .unwrap();
        assert!(
            token
                .require_scope(Some(b"candidate-entry-source-B"))
                .is_err()
        );
        assert!(token.require_scope(None).is_err());
        predicted.post_tick = None;
        actual.post_tick = None;
        assert!(native_kernel_admission(&predicted, &actual).is_err());
    }

    #[test]
    fn pure_parity_prefix_refuses_first_differing_frame() {
        let predicted = synthetic_parity(1, 4, true);
        let mut actual = synthetic_parity(2, 1, false);
        compare_prefix(&predicted, &actual).unwrap();
        actual.frames[0].owner_memory[0] ^= 1;
        assert!(compare_prefix(&predicted, &actual).is_err());
        assert_eq!(actual.frames.len(), 1);
    }

    #[test]
    fn pure_complete_admission_requires_and_compares_post_tick() {
        let predicted = synthetic_parity(1, 4, true);
        let mut actual = synthetic_parity(2, 4, false);
        assert!(native_kernel_admission_complete(&predicted, &actual).is_err());
        actual.post_tick = predicted.post_tick.clone();
        native_kernel_admission_complete(&predicted, &actual).unwrap();
        actual.post_tick.as_mut().unwrap().owner_memory[0] ^= 1;
        assert!(native_kernel_admission_complete(&predicted, &actual).is_err());
    }

    #[test]
    fn pure_parity_preserves_u128_and_signed_motor_zero() {
        let predicted = synthetic_parity(1, 4, true);
        assert_eq!(
            std::str::from_utf8(&predicted.initial.world).unwrap(),
            u128::MAX.to_string()
        );
        let mut actual = synthetic_parity(2, 1, false);
        actual.frames[0].body_and_motor_bits[0] = 0.0_f32.to_bits();
        assert!(compare_prefix(&predicted, &actual).is_err());
        let mut actual = synthetic_parity(2, 1, false);
        actual.frames[0].world = serde_json::to_vec(&(u128::MAX - 1)).unwrap();
        assert!(compare_prefix(&predicted, &actual).is_err());
    }

    #[test]
    fn pure_parity_rejects_same_instance_and_longer_actual_prefix() {
        let predicted = synthetic_parity(1, 1, false);
        assert!(compare_prefix(&predicted, &synthetic_parity(1, 1, false)).is_err());
        assert!(compare_prefix(&predicted, &synthetic_parity(2, 2, false)).is_err());
        let old_predicted = synthetic_parity(1, 4, false);
        let old_actual = synthetic_parity(2, 4, false);
        native_kernel_admission(&old_predicted, &old_actual).unwrap();
        assert!(native_kernel_admission_complete(&old_predicted, &old_actual).is_err());
    }

    #[test]
    fn pure_parity_excludes_only_transfer_authorization_and_record_history() {
        let owner = serde_json::json!({
            "rates":[0.125,-0.0],"nextSubstep":40,
            "transfer": {
                "decision":{"terminalClock":{"first_success_at":null},"lastObserved":39,"queue":[1,2,3,4]},
                "authorization":{"kind":"preview","instance":123,"end":40},
                "nativeRecords":[{"at":36}],
                "futureDecisionField":17
            }
        });
        let encoded = |value| serde_json::to_vec(&coordination_decision_evidence(value)).unwrap();
        let expected = encoded(owner.clone());
        let mut actual_owner = owner.clone();
        actual_owner["transfer"]["authorization"] =
            serde_json::json!({"kind":"native-window","next":4});
        actual_owner["transfer"]["nativeRecords"] = serde_json::json!([]);
        assert_eq!(expected, encoded(actual_owner.clone()));
        let mut predicted = synthetic_parity(1, 4, true);
        let install = |trace: &mut NativeParityTrace, state: &[u8]| {
            trace.initial.state = state.to_vec();
            for frame in &mut trace.frames {
                frame.state = state.to_vec();
            }
            trace.post_tick.as_mut().unwrap().state = state.to_vec();
        };
        install(&mut predicted, &expected);
        let mut actual = synthetic_parity(2, 4, true);
        install(&mut actual, &encoded(actual_owner.clone()));
        native_kernel_admission_complete(&predicted, &actual).unwrap();
        for changed in [
            {
                let mut v = actual_owner.clone();
                v["transfer"]["decision"]["lastObserved"] = 38.into();
                v
            },
            {
                let mut v = actual_owner.clone();
                v["transfer"]["futureDecisionField"] = 18.into();
                v
            },
            {
                let mut v = actual_owner.clone();
                v["rates"][0] = 0.25.into();
                v
            },
        ] {
            let changed = encoded(changed);
            assert_ne!(expected, changed);
            actual.frames[0].state = changed;
            assert!(compare_prefix(&predicted, &actual).is_err());
        }
        let no_transfer =
            serde_json::json!({"rates":[0.125],"transfer":null,"authorization":"unrelated"});
        assert_eq!(
            coordination_decision_evidence(no_transfer.clone()),
            no_transfer
        );
    }

    #[test]
    fn pure_parity_rejects_diagnostic_sample_or_score_memory_divergence() {
        let memory = serde_json::json!({
            "diagnostic":{"heel_loaded_substeps":[12,12],"consecutive_stable_substeps":1,"passed":false},
            "sample":{"heel_loads_n":[3.0,4.0],"com_inside":true,"passed":false},
            "center_of_mass_speed_mps":0.01
        });
        let baseline = serde_json::to_vec(&memory).unwrap();
        let install = |trace: &mut NativeParityTrace| {
            trace.initial.owner_memory = baseline.clone();
            for frame in &mut trace.frames {
                frame.owner_memory = baseline.clone();
            }
            trace.post_tick.as_mut().unwrap().owner_memory = baseline.clone();
        };
        let mut predicted = synthetic_parity(1, 4, true);
        install(&mut predicted);
        let mut changed_diagnostic = memory.clone();
        changed_diagnostic["diagnostic"]["consecutive_stable_substeps"] = 2.into();
        let mut changed_sample = memory.clone();
        changed_sample["sample"]["heel_loads_n"][0] = 3.5.into();
        let mut changed_speed = memory.clone();
        changed_speed["center_of_mass_speed_mps"] = 0.02.into();
        for changed in [changed_diagnostic, changed_sample, changed_speed] {
            let mut actual = synthetic_parity(2, 4, true);
            install(&mut actual);
            native_kernel_admission_complete(&predicted, &actual).unwrap();
            actual.frames[0].owner_memory = serde_json::to_vec(&changed).unwrap();
            assert_eq!(predicted.frames[0].world, actual.frames[0].world);
            assert_eq!(
                predicted.frames[0].body_and_motor_bits,
                actual.frames[0].body_and_motor_bits
            );
            assert!(compare_prefix(&predicted, &actual).is_err());
            actual.frames[0].owner_memory = baseline.clone();
            actual.post_tick.as_mut().unwrap().owner_memory = serde_json::to_vec(&changed).unwrap();
            assert!(native_kernel_admission_complete(&predicted, &actual).is_err());
        }
    }

    struct FakeClock(u64);
    impl NativeStepClock for FakeClock {
        fn completed_native_substeps(&self) -> u64 {
            self.0
        }
    }
    #[derive(Default)]
    struct FakeBudget {
        admitted: usize,
        maximum: usize,
        requests: Vec<StepReservation>,
    }
    impl PreviewBudget for FakeBudget {
        fn admit_preview(&mut self, _: PreviewPurpose, _: u32) -> Result<(), String> {
            Ok(())
        }
        fn check_wall_deadline(&mut self) -> Result<(), String> {
            Ok(())
        }
        fn reserve_one_before_step(&mut self, request: StepReservation) -> Result<(), String> {
            if self.admitted >= self.maximum {
                return Err("global ceiling/deadline".into());
            }
            self.admitted += 1;
            self.requests.push(request);
            Ok(())
        }
    }

    #[test]
    fn admission_requires_four_measured_native_frames() {
        let source = Simulation::new(0.0, crate::Profile::default(), true).unwrap();
        let trace = NativeParityTrace::begin(&source, &()).unwrap();
        assert!(matches!(
            native_kernel_admission(&trace, &trace),
            Err(PreviewError::NativeKernelParityUnavailable)
        ));
    }

    #[test]
    fn static_snapshot_copies_world_and_controller_memory_without_integration() {
        // Construction/configuration only. No step, advance or predictor call.
        let mut source = Simulation::new(0.0, crate::Profile::default(), true).unwrap();
        source.recovery = Some(crate::recovery::Controller::new(&source).unwrap());
        source.recovery.as_mut().unwrap().status.phase_time_s = 0.37;
        source.recovery.as_mut().unwrap().stable_substeps = 17;
        source.recovery.as_mut().unwrap().handoff_substeps = 23;
        source.recovery.as_mut().unwrap().settle_substeps = 31;
        source.integrated_support_substeps[3] = 7;
        source.integrated_support_loads_n[3] = 2.5;
        source.persistence[3] = 0.125;
        source.counters.falls = 2;
        let memory = vec![2_u64, 7, 11];
        let before_world = serde_json::to_value(&source.world).unwrap();
        let before_status =
            serde_json::to_value(&source.recovery.as_ref().unwrap().status).unwrap();
        let mut branch = StaticPreviewBranch::capture(&source, &memory).unwrap();
        assert_eq!(
            serde_json::to_value(&branch.simulation().world).unwrap(),
            before_world
        );
        assert_eq!(
            serde_json::to_value(&branch.simulation().recovery.as_ref().unwrap().status).unwrap(),
            before_status
        );
        let copied_controller = branch.simulation().recovery.as_ref().unwrap();
        let source_controller = source.recovery.as_ref().unwrap();
        assert_eq!(copied_controller.targets, source_controller.targets);
        assert_eq!(copied_controller.velocities, source_controller.velocities);
        assert_eq!(
            copied_controller.best_progress.to_bits(),
            source_controller.best_progress.to_bits()
        );
        assert_eq!(
            (
                copied_controller.stable_substeps,
                copied_controller.handoff_substeps,
                copied_controller.settle_substeps
            ),
            (17, 23, 31)
        );
        assert_eq!(
            branch.simulation().filter.exclusions,
            source.filter.exclusions
        );
        assert_eq!(branch.simulation().targets, source.targets);
        assert_eq!(branch.simulation().rest_targets, source.rest_targets);
        assert_eq!(
            branch.simulation().integrated_support_substeps,
            source.integrated_support_substeps
        );
        assert_eq!(
            branch.simulation().integrated_support_loads_n,
            source.integrated_support_loads_n
        );
        assert_eq!(branch.simulation().persistence, source.persistence);
        assert_eq!(branch.memory(), &memory);
        // Changes to copied memory and world data cannot change the source.
        // These are synthetic state edits, still with zero native integrations.
        branch.simulation.world.gravity.x = 1.0;
        branch
            .simulation
            .recovery
            .as_mut()
            .unwrap()
            .status
            .phase_time_s = 9.0;
        branch.simulation.persistence[3] = 0.0;
        branch.memory[0] = 99;
        assert_eq!(serde_json::to_value(&source.world).unwrap(), before_world);
        assert_eq!(
            serde_json::to_value(&source.recovery.as_ref().unwrap().status).unwrap(),
            before_status
        );
        assert_eq!(source.persistence[3], 0.125);
        assert_eq!(memory, [2, 7, 11]);
        assert_eq!(
            (
                source.substeps_completed,
                branch.simulation.substeps_completed
            ),
            (0, 0)
        );
        source.substeps_completed = 1;
        assert!(matches!(
            copy_static_snapshot(&source),
            Err(PreviewError::Snapshot(_))
        ));
    }

    #[test]
    fn fake_kernel_consumes_one_reservation_per_step_and_preserves_partial_trace() {
        let mut state = FakeClock(24);
        let mut memory = 0;
        let mut budget = FakeBudget {
            maximum: 2,
            ..Default::default()
        };
        let run = run_guarded_steps(&mut state, &mut memory, 9, 4, &mut budget, |s, m| {
            s.0 += 1;
            *m += 1;
            Ok(s.0)
        });
        assert_eq!(run.records, [25, 26]);
        assert!(matches!(run.refusal, Some(PreviewError::Reservation(_))));
        assert_eq!((state.0, memory, budget.admitted), (26, 2, 2));
        assert_eq!(
            budget.requests[1],
            StepReservation {
                branch_id: 9,
                before_substep: 25,
                preview_index: 1,
                purpose: PreviewPurpose::ParityVerification
            }
        );
    }

    #[test]
    fn refused_reservation_does_not_invoke_kernel_or_change_memory() {
        let mut state = FakeClock(0);
        let mut memory = 0;
        let mut budget = FakeBudget::default();
        let run: PreviewRun<()> =
            run_guarded_steps(&mut state, &mut memory, 0, 1, &mut budget, |_, _| {
                panic!("unreserved callback")
            });
        assert!(matches!(run.refusal, Some(PreviewError::Reservation(_))));
        assert_eq!((state.0, memory), (0, 0));
    }

    #[test]
    fn partial_four_step_reservation_is_retained_without_starting_a_tick() {
        let mut budget = FakeBudget {
            maximum: 3,
            ..Default::default()
        };
        let request = StepReservation {
            branch_id: 4,
            before_substep: 24,
            preview_index: 0,
            purpose: PreviewPurpose::ParityVerification,
        };
        assert!(budget.reserve_four_before_tick(request).is_err());
        assert_eq!(budget.admitted, 3);
        assert_eq!(
            budget
                .requests
                .iter()
                .map(|r| r.before_substep)
                .collect::<Vec<_>>(),
            [24, 25, 26]
        );
        // No native state or kernel is needed to reject an incomplete batch.
        assert!(budget.reserve_four_before_tick(request).is_err());
        assert_eq!(budget.admitted, 3);
    }

    #[test]
    fn clock_violation_and_kernel_error_never_refund_a_reservation() {
        let mut state = FakeClock(0);
        let mut budget = FakeBudget {
            maximum: 4,
            ..Default::default()
        };
        let run = run_guarded_steps(&mut state, &mut (), 0, 2, &mut budget, |s, _| {
            s.0 += 2;
            Ok(())
        });
        assert_eq!(
            run.refusal,
            Some(PreviewError::ClockMismatch {
                expected: 1,
                actual: 2
            })
        );
        assert_eq!(budget.admitted, 1);
        let run: PreviewRun<()> =
            run_guarded_steps(&mut state, &mut (), 1, 2, &mut budget, |s, _| {
                s.0 += 1;
                Err("observer refuses integrated state".into())
            });
        assert!(matches!(run.refusal, Some(PreviewError::Kernel(_))));
        assert_eq!(budget.admitted, 2);
    }

    #[test]
    fn bad_horizon_and_clock_overflow_refuse_before_reservation() {
        let mut state = FakeClock(u64::MAX);
        let mut budget = FakeBudget {
            maximum: 1,
            ..Default::default()
        };
        for steps in [0, 481, 1] {
            let run: PreviewRun<()> =
                run_guarded_steps(&mut state, &mut (), 0, steps, &mut budget, |_, _| {
                    panic!("invalid clock/horizon callback")
                });
            assert!(run.refusal.is_some());
        }
        assert_eq!(budget.admitted, 0);
    }
}
