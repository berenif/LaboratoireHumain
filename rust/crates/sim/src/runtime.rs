//! Worker-ready lifecycle owner. No clocks, DOM, renderer, or pose setters.
//! Station changes construct fresh physical trials before acknowledging them.
use crate::{Profile, Simulation, measurement::Failure};
use lh_contracts::{
    Acknowledgement, Action, Command, MAX_ADVANCE_TICKS, Mode, Reject, SCHEMA_VERSION, Snapshot,
    Stamp, Timeline,
};
use serde::{Deserialize, Serialize};
use bincode::Options;

const GETUP_CHECKPOINT_LIMIT: u64 = 64 * 1024 * 1024;

#[cfg(feature = "getup-terminal-base-ready-experiment")]
const TERMINAL_BASE_READY_CHECKPOINT_FORMAT: &str = "terminal-base-ready-v1";

#[cfg(feature = "getup-terminal-base-ready-experiment")]
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct TerminalBaseReadyCheckpoint {
    format: String,
    inner_schema: u32,
    // Preserve either existing typed auxiliary JSON string verbatim, including
    // schema 2's exact-bit root-translation sidecar.
    auxiliary: String,
}

#[cfg(feature = "root-translation-carry-experiment")]
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct RootTranslationCheckpointEntry {
    /// Stable rigid-body identity, including the arena generation.
    root: [u32; 2],
    #[serde(deserialize_with = "required_root_translation_state")]
    state: Option<rapier3d::dynamics::RootTranslationIntegratorState>,
}

#[cfg(feature = "root-translation-carry-experiment")]
fn required_root_translation_state<'de, D: serde::Deserializer<'de>>(deserializer: D)
    -> Result<Option<rapier3d::dynamics::RootTranslationIntegratorState>, D::Error>
{
    Option::deserialize(deserializer)
}

#[cfg(feature = "root-translation-carry-experiment")]
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct RootTranslationCheckpoint {
    format: String,
    // Keep the original typed Runtime serialization intact. Converting it to
    // JSON Value first changes the decimal serialization of f32 fields.
    runtime: String,
    roots: Vec<RootTranslationCheckpointEntry>,
}

#[cfg(feature = "getup-policy-adaptation-experiment")]
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct AdaptationCheckpoint {
    format: String,
    inner_schema: u32,
    auxiliary: String,
}

#[derive(Serialize)]
pub struct Reply {
    pub completed_ticks: u32,
    pub paused: bool,
    pub visible: bool,
    pub mode: Mode,
    pub acknowledgements: Vec<Acknowledgement>,
    pub snapshot: Snapshot,
    pub failure: Option<Failure>,
}
#[derive(Serialize, Deserialize)]
pub struct Runtime {
    simulation: Simulation,
    #[serde(skip)]
    quiet: Option<crate::quiet::QuietObservation>,
    #[serde(skip)]
    recovery_measurement: Option<crate::cycles::QualificationObservation>,
    timeline: Timeline,
    heading: f32,
    mode: Mode,
    paused: bool,
    visible: bool,
    disposed: bool,
    highest_press: u64,
    observation_sequence: u64,
    playground_settings: lh_contracts::PlaygroundSettings,
    acknowledgements: Vec<Acknowledgement>,
}
fn acknowledgement(
    command: &Command,
    tick: Option<u64>,
    rejected: Option<Reject>,
) -> Acknowledgement {
    Acknowledgement {
        stamp: command.stamp,
        applied_tick: tick,
        rejected,
    }
}
impl Runtime {
    #[cfg(feature = "stationary-balance-experiment")]
    pub(crate) fn balance_simulation_mut(&mut self) -> &mut Simulation {
        &mut self.simulation
    }
    pub fn new_getup_fixture(kind:&str) -> Result<Self,String> {
        let mut profile=Profile::default();profile.getup_policy_enabled=true;
        Self::new_getup_fixture_with_profile(kind, profile)
    }
    /// Construct a fresh fixture with an explicit complete numerical profile.
    /// No checkpoint or already-integrated world is reconfigured.
    pub fn new_getup_fixture_with_profile(kind: &str, profile: Profile) -> Result<Self, String> {
        profile.validate()?;
        if !profile.getup_policy_enabled {
            return Err("GetUp fixture profile must explicitly enable the policy".into());
        }
        let mut runtime=Self::new(Mode::Protocol,0.0,profile)?;
        crate::getup::fixtures::initialize(&mut runtime.simulation,kind)?;
        Ok(runtime)
    }
    pub fn getup_fixture_report(&self) -> Result<String,String> {
        let s=&self.simulation;
        #[cfg(feature = "unit-root-rotation-experiment")]
        let unit_root_rotation = {
            let root = *s.bodies.first().ok_or("Missing root")?;
            let link = s.world.multibody_joints.rigid_body_link(root).ok_or("Missing root articulation")?;
            let mb = s.world.multibody_joints.get_multibody(link.multibody).ok_or("Missing root articulation")?;
            (s.profile.root_rotation_normalization.is_some() || mb.unit_root_rotation_enabled()).then(|| serde_json::json!({
                "algorithm":"unit-free-root-v1", "enabled":mb.unit_root_rotation_enabled(),
                "root":root.into_raw_parts(),
                "validationError":mb.validate_unit_root_rotation().err(),
                "rootNormSquared":s.world.bodies[root].rotation().length_squared(),
            }))
        };
        let bounds:Vec<_>=s.colliders.iter().enumerate().filter(|(_,h)|s.world.colliders[**h].is_enabled()).map(|(i,h)|{
            let a=s.world.colliders[*h].compute_aabb();serde_json::json!({"body":s.model.segments[i].id,
                "min":[a.mins.x,a.mins.y,a.mins.z],"max":[a.maxs.x,a.maxs.y,a.maxs.z]})
        }).collect();
        let (q,dq)=crate::getup::joint_state(s)?;
        #[derive(Serialize)]
        struct Report<'a> {
            #[serde(flatten)]
            observations: serde_json::Value,
            profile: &'a Profile,
            #[cfg(feature = "unit-root-rotation-experiment")]
            #[serde(rename = "rootRotationNormalization", skip_serializing_if = "Option::is_none")]
            unit_root_rotation: Option<serde_json::Value>,
            #[cfg(feature = "root-translation-carry-experiment")]
            #[serde(rename = "rootTranslationCompensation", skip_serializing_if = "Option::is_none")]
            root_translation_compensation: Option<serde_json::Value>,
            #[serde(rename = "physicalRoll", skip_serializing_if = "Option::is_none")]
            physical_roll: Option<serde_json::Value>,
        }
        #[cfg(feature = "root-translation-carry-experiment")]
        let active_roots: Vec<_> = self.root_translation_checkpoint_entries()?.into_iter()
            .filter(|entry| entry.state.is_some()).map(|entry| entry.root).collect();
        serde_json::to_string(&Report {
            observations: serde_json::json!({"state":s.getup_fixture,"nativeSubsteps":s.substeps_completed,
                "jointQ":q,"jointDq":dq,"colliderBounds":bounds,"massKg":s.total_mass(),
                "solverBudget":{"currentInternalPgsIterations":s.world.integration_parameters.num_internal_pgs_iterations,
                    "selectedNextInternalPgsIterations":s.selected_internal_pgs_iterations(),
                    "sourceRecoveryActive":s.getup_controller.as_ref().is_some_and(|c|c.source_recovery_active())}}),
            profile: &s.profile,
            physical_roll: s.getup_controller.as_ref().and_then(|c| c.physical_roll_report()),
            #[cfg(feature = "unit-root-rotation-experiment")]
            unit_root_rotation,
            #[cfg(feature = "root-translation-carry-experiment")]
            root_translation_compensation: (!active_roots.is_empty()).then(|| serde_json::json!({
                "algorithm": rapier3d::dynamics::RootTranslationIntegratorState::ALGORITHM,
                "activeRoots": active_roots,
            })),
        }).map_err(|e|e.to_string())
    }
    pub fn trigger_getup_fixture(&mut self) -> Result<(),String> {
        crate::getup::fixtures::trigger(&mut self.simulation)
    }
    pub fn getup_standing_diagnostic(&self, anchors_json: &str) -> Result<String,String> {
        let anchors: Option<[lh_model::Vec3;2]> = serde_json::from_str(anchors_json).map_err(|e|e.to_string())?;
        crate::getup::controller::standing_diagnostic(&self.simulation,anchors).map(|v|v.to_string())
    }
    pub fn getup_native_observation(&self)->Result<String,String> {
        crate::getup::locomotion::observation(&self.simulation).map(|v|v.to_string())
    }
    #[cfg(feature="getup-native-locomotion-experiment")]
    pub fn set_getup_native_mode(&mut self, mode:&str)->Result<(),String> {
        if self.disposed || self.timeline.depth()!=0 { return Err("Native mode requires a live, empty command boundary".into()); }
        let mode=match mode {"passive"=>crate::getup::locomotion::Mode::Passive,
            "active"=>crate::getup::locomotion::Mode::Active,_=>return Err("Unknown native motor mode".into())};
        crate::getup::locomotion::set_mode(&mut self.simulation,mode)
    }
    #[cfg(feature="getup-native-locomotion-experiment")]
    pub fn begin_getup_native_step(&mut self, side:usize)->Result<(),String> {
        if self.disposed || self.timeline.depth()!=0 {return Err("Step requires a live empty command boundary".into());}
        crate::getup::locomotion::begin_step(&mut self.simulation,side)
    }
    fn checkpoint_compatible(&self) -> Result<(), String> {
        let s = &self.simulation;
        s.profile.validate()?;
        crate::getup::locomotion::validate(s)?;
        s.validate_unit_root_rotation_policy()?;
        if let Some(controller) = &s.getup_controller {
            if !controller.transfer_valid(s) {return Err("Invalid/unsupported transfer checkpoint state".into());}
            if !controller.adaptation_valid(s.substeps_completed) { return Err("Invalid checkpoint adaptation state/clock".into()); }
            if !controller.physical_roll_clock_valid(s.substeps_completed) {
                return Err("Invalid checkpoint physical roll native clock or source admission".into());
            }
            if s.profile.getup_repeated_recovery.is_some()
                && !controller.repeated_recovery_clock_valid(s.substeps_completed)
            {
                return Err("Invalid checkpoint repeated GetUp recovery clock".into());
            }
            if self.simulation.profile.getup_repeated_recovery.is_some()
                && controller.terminal_base_ready_variant() && !controller.balance_acquire_variant()
            { return Err("Repeated early transfer requires explicit balance acquisition variant".into()); }
            if controller.terminal_bumpless_variant()
                && !cfg!(feature = "getup-terminal-bumpless-experiment")
            {
                return Err("Checkpoint requires getup-terminal-bumpless-experiment".into());
            }
            if controller.terminal_base_ready_variant()
                && !cfg!(feature = "getup-terminal-base-ready-experiment")
            {
                return Err("Checkpoint requires getup-terminal-base-ready-experiment".into());
            }
            if !controller.terminal_base_ready_variant_valid() {
                return Err("Invalid checkpoint terminal base-ready controller state".into());
            }
            if !controller.terminal_bumpless_clock_valid(s.substeps_completed) {
                return Err("Invalid checkpoint terminal bumpless activation clock".into());
            }
        }
        if s.getup_controller.as_ref().is_some_and(|c| !c.checkpoint_variant_supported()) {
            return Err("Checkpoint requires the experimental terminal-balance controller build".into());
        }
        if self.mode != Mode::Protocol || self.timeline.depth() != 0 || self.quiet.is_some()
            || self.recovery_measurement.is_some() || s.getup_controller.is_none()
            || !s.profile.getup_policy_enabled || s.stepping.is_some()
            || s.environment.is_some() || s.recovery.is_some() || s.grab.is_some()
        {
            return Err("Checkpoint requires the isolated GetUp protocol".into());
        }
        #[cfg(all(test, feature = "experimental-solver-step-observation"))]
        if s.coordination.is_some() { return Err("Unsupported experimental controller".into()); }
        Ok(())
    }

    /// Default-OFF numerical experiment. This changes only integrator history;
    /// it never changes coordinates, velocities, forces or the native clock.
    #[cfg(feature = "root-translation-carry-experiment")]
    pub fn set_root_translation_compensation_enabled(&mut self, enabled: bool) -> Result<(), String> {
        self.checkpoint_compatible()?;
        // Preflight every root before changing history, so a later invalid
        // root cannot leave this operation partly applied. Disabling also
        // remains available if an active state has become unexportable.
        let mut roots = Vec::new();
        for multibody in self.simulation.world.multibody_joints.multibodies() {
            multibody.clone().set_root_translation_compensation_enabled(enabled).map_err(str::to_string)?;
            roots.push(multibody.root().rigid_body_handle());
        }
        for root in roots {
            let index = self.simulation.world.multibody_joints.rigid_body_link(root)
                .ok_or("Missing checkpoint root")?.multibody;
            self.simulation.world.multibody_joints.get_multibody_mut(index)
                .ok_or("Missing checkpoint multibody")?
                .set_root_translation_compensation_enabled(enabled).map_err(str::to_string)?;
        }
        Ok(())
    }

    #[cfg(feature = "root-translation-carry-experiment")]
    fn root_translation_checkpoint_entries(&self) -> Result<Vec<RootTranslationCheckpointEntry>, String> {
        let world = &self.simulation.world;
        let mut entries = Vec::new();
        for multibody in world.multibody_joints.multibodies() {
            let root = multibody.root().rigid_body_handle();
            let link = world.multibody_joints.rigid_body_link(root).ok_or("Missing checkpoint root link")?;
            if world.bodies.get(root).is_none() || link.id != 0
                || world.multibody_joints.get_multibody(link.multibody)
                    .is_none_or(|mapped| mapped.root().rigid_body_handle() != root)
            {
                return Err("Invalid checkpoint root identity".into());
            }
            let state = multibody.root_translation_integrator_state();
            if let Some(state) = state {
                // Validation is immutable on the live world; no stale or
                // invalid active state may silently become an inactive export.
                multibody.clone().restore_root_translation_integrator_state(state).map_err(str::to_string)?;
            }
            let (index, generation) = root.into_raw_parts();
            entries.push(RootTranslationCheckpointEntry { root: [index, generation], state });
        }
        entries.sort_unstable_by_key(|entry| entry.root);
        if entries.windows(2).any(|pair| pair[0].root == pair[1].root) {
            return Err("Duplicate checkpoint root identity".into());
        }
        Ok(entries)
    }

    #[cfg(feature = "root-translation-carry-experiment")]
    fn restore_root_translation_checkpoint_entries(&mut self, entries: Vec<RootTranslationCheckpointEntry>) -> Result<(), String> {
        let expected = self.root_translation_checkpoint_entries()?;
        if entries.len() != expected.len() || entries.iter().zip(&expected).any(|(entry, expected)| entry.root != expected.root) {
            return Err("Checkpoint root identities differ, are missing, duplicated or unordered".into());
        }
        if !entries.iter().any(|entry| entry.state.is_some()) {
            return Err("Compensated checkpoint has no active root state".into());
        }
        for entry in entries {
            let root = rapier3d::prelude::RigidBodyHandle::from_raw_parts(entry.root[0], entry.root[1]);
            let index = self.simulation.world.multibody_joints.rigid_body_link(root)
                .ok_or("Missing checkpoint root")?.multibody;
            let multibody = self.simulation.world.multibody_joints.get_multibody_mut(index)
                .ok_or("Missing checkpoint multibody")?;
            match entry.state {
                Some(state) => multibody.restore_root_translation_integrator_state(state).map_err(str::to_string)?,
                None => multibody.set_root_translation_compensation_enabled(false).map_err(str::to_string)?,
            }
        }
        Ok(())
    }

    /// Isolated GetUp world, native/controller memory and command timeline.
    /// Rapier serde skips pipeline/CCD workspaces and the initial multibody
    /// topology-refresh queue. First-step continuity is tested explicitly;
    /// this does not promise generic raw-world checkpoint completeness.
    /// Active compensated root integration additionally requires schema 2's
    /// exact-bit sidecar: generic Rapier world serde does not preserve it.
    pub fn getup_checkpoint(&self) -> Result<Vec<u8>, String> {
        self.checkpoint_compatible()?;
        let world = bincode::DefaultOptions::new().with_fixint_encoding().with_limit(64 * 1024 * 1024)
            .serialize(&self.simulation.world).map_err(|e| e.to_string())?;
        // Self-describing auxiliary state preserves serde's optional fields;
        // binary Rapier state preserves nonfinite workspace sentinels exactly.
        let auxiliary = serde_json::to_string(self).map_err(|e| e.to_string())?;
        let schema = 1_u32;
        #[cfg(feature = "root-translation-carry-experiment")]
        let (schema, auxiliary) = {
            let roots = self.root_translation_checkpoint_entries()?;
            if roots.iter().any(|entry| entry.state.is_some()) {
                (2_u32, serde_json::to_string(&RootTranslationCheckpoint {
                    format: rapier3d::dynamics::RootTranslationIntegratorState::ALGORITHM.into(),
                    runtime: auxiliary,
                    roots,
                }).map_err(|e| e.to_string())?)
            } else { (schema, auxiliary) }
        };
        #[cfg(feature = "getup-terminal-base-ready-experiment")]
        let (schema, auxiliary) = if self.simulation.getup_controller.as_ref()
            .is_some_and(|controller| controller.terminal_base_ready_variant())
        {
            (3_u32, serde_json::to_string(&TerminalBaseReadyCheckpoint {
                format: TERMINAL_BASE_READY_CHECKPOINT_FORMAT.into(),
                inner_schema: schema,
                auxiliary,
            }).map_err(|e| e.to_string())?)
        } else { (schema, auxiliary) };
        #[cfg(feature = "getup-policy-adaptation-experiment")]
        let (schema, auxiliary) = if self.simulation.getup_controller.as_ref().is_some_and(|c|c.adaptation_active()) {
            (4_u32, serde_json::to_string(&AdaptationCheckpoint {format:crate::getup::adaptation::FORMAT.into(),
                inner_schema:schema, auxiliary}).map_err(|e|e.to_string())?)
        } else { (schema,auxiliary) };
        bincode::DefaultOptions::new().with_fixint_encoding().with_limit(64 * 1024 * 1024)
            .serialize(&(schema, lh_model::CANONICAL_JSON, world, auxiliary)).map_err(|e| e.to_string())
    }

    pub fn from_getup_checkpoint(bytes: &[u8]) -> Result<Self, String> {
        if bytes.len() as u64 > GETUP_CHECKPOINT_LIMIT { return Err("Checkpoint exceeds size limit".into()); }
        let (schema, model, world, auxiliary): (u32, String, Vec<u8>, String) =
            bincode::DefaultOptions::new().with_fixint_encoding().with_limit(64 * 1024 * 1024)
                .reject_trailing_bytes().deserialize(bytes).map_err(|e| e.to_string())?;
        if model != lh_model::CANONICAL_JSON {
            return Err("Checkpoint schema/model differs".into());
        }
        let adaptation_checkpoint = schema == 4;
        #[cfg(not(feature = "getup-policy-adaptation-experiment"))]
        if adaptation_checkpoint { return Err("Checkpoint requires getup-policy-adaptation-experiment".into()); }
        #[cfg(feature = "getup-policy-adaptation-experiment")]
        let (schema, auxiliary) = if adaptation_checkpoint {
            let wrapper: AdaptationCheckpoint = serde_json::from_str(&auxiliary).map_err(|e|e.to_string())?;
            if wrapper.format != crate::getup::adaptation::FORMAT || !matches!(wrapper.inner_schema,1|2|3) {
                return Err("Invalid bounded adaptation checkpoint envelope".into());
            }
            (wrapper.inner_schema,wrapper.auxiliary)
        } else { (schema,auxiliary) };
        #[cfg(not(feature = "getup-terminal-base-ready-experiment"))]
        if schema == 3 {
            return Err("Checkpoint requires getup-terminal-base-ready-experiment".into());
        }
        let terminal_base_ready_checkpoint = schema == 3;
        #[cfg(feature = "getup-terminal-base-ready-experiment")]
        let (schema, auxiliary) = if terminal_base_ready_checkpoint {
            let wrapper: TerminalBaseReadyCheckpoint = serde_json::from_str(&auxiliary).map_err(|e| e.to_string())?;
            if wrapper.format != TERMINAL_BASE_READY_CHECKPOINT_FORMAT {
                return Err("Checkpoint terminal base-ready format differs".into());
            }
            if !matches!(wrapper.inner_schema, 1 | 2) {
                return Err("Checkpoint terminal base-ready inner schema differs".into());
            }
            (wrapper.inner_schema, wrapper.auxiliary)
        } else { (schema, auxiliary) };
        #[cfg(not(feature = "root-translation-carry-experiment"))]
        if schema == 2 { return Err("Compensated checkpoint requires root-translation-carry-experiment".into()); }
        #[cfg(feature = "root-translation-carry-experiment")]
        let (auxiliary, roots) = match schema {
            1 => (auxiliary, None),
            2 => {
                let wrapper: RootTranslationCheckpoint = serde_json::from_str(&auxiliary).map_err(|e| e.to_string())?;
                if wrapper.format != rapier3d::dynamics::RootTranslationIntegratorState::ALGORITHM {
                    return Err("Checkpoint root integration format differs".into());
                }
                (wrapper.runtime, Some(wrapper.roots))
            },
            _ => return Err("Checkpoint schema/model differs".into()),
        };
        #[cfg(not(feature = "root-translation-carry-experiment"))]
        if schema != 1 { return Err("Checkpoint schema/model differs".into()); }
        let mut restored: Self = serde_json::from_str(&auxiliary).map_err(|e| e.to_string())?;
        restored.simulation.profile.validate()?;
        if restored.simulation.getup_controller.as_ref().is_some_and(|c|!c.transfer_restore_allowed()) {
            return Err("Active supported-transfer restoration is not admitted; exact state is diagnostic only".into());
        }
        if restored.simulation.getup_controller.as_ref().is_some_and(|c|c.adaptation_active()) != adaptation_checkpoint {
            return Err("Checkpoint adaptation marker/controller differs".into());
        }
        let terminal_base_ready_variant = restored.simulation.getup_controller.as_ref()
            .is_some_and(|controller| controller.terminal_base_ready_variant());
        #[cfg(not(feature = "getup-terminal-base-ready-experiment"))]
        if terminal_base_ready_variant {
            return Err("Checkpoint requires getup-terminal-base-ready-experiment".into());
        }
        if terminal_base_ready_variant != terminal_base_ready_checkpoint {
            return Err("Checkpoint terminal base-ready marker/controller differs".into());
        }
        restored.simulation.world = bincode::DefaultOptions::new().with_fixint_encoding()
            .with_limit(64 * 1024 * 1024).reject_trailing_bytes().deserialize(&world).map_err(|e| e.to_string())?;
        restored.simulation.restore_unit_root_rotation_policy()?;
        restored.checkpoint_compatible()?;
        #[cfg(feature = "root-translation-carry-experiment")]
        if let Some(roots) = roots {
            restored.restore_root_translation_checkpoint_entries(roots)?;
        }
        // Schema 1 is a deliberate inactive baseline boundary: Rapier serde
        // constructs every skipped root integrator state as None.
        Ok(restored)
    }

    pub fn verify_getup_checkpoint(&self, bytes: &[u8]) -> Result<String, String> {
        let restored = Self::from_getup_checkpoint(bytes)?;
        // The binary world includes maps with compound keys; JSON cannot
        // represent those maps. Compare generalized motion and observations
        // here, and validate an actual next tick separately in the admitted run.
        let same_joints = crate::getup::joint_state(&self.simulation)?
            == crate::getup::joint_state(&restored.simulation)?;
        let same_handles = self.simulation.bodies == restored.simulation.bodies
            && self.simulation.colliders == restored.simulation.colliders
            && self.simulation.joints == restored.simulation.joints;
        let same_snapshot = serde_json::to_value(self.simulation.snapshot()).map_err(|e| e.to_string())?
            == serde_json::to_value(restored.simulation.snapshot()).map_err(|e| e.to_string())?;
        let same_controller = self.getup_report()? == restored.getup_report()?;
        let same_native_owner = serde_json::to_string(&self.simulation.native_locomotion).map_err(|e|e.to_string())?
            == serde_json::to_string(&restored.simulation.native_locomotion).map_err(|e|e.to_string())?;
        #[cfg(feature = "root-translation-carry-experiment")]
        let same_integrator = self.root_translation_checkpoint_entries()? == restored.root_translation_checkpoint_entries()?;
        #[cfg(feature = "root-translation-carry-experiment")]
        if !same_integrator { return Err("Checkpoint round-trip changed root integrator state".into()); }
        if !same_joints || !same_handles || !same_snapshot || !same_controller || !same_native_owner {
            return Err(format!("Checkpoint round-trip changed data: joints={same_joints}, handles={same_handles}, snapshot={same_snapshot}, controller={same_controller}, nativeOwner={same_native_owner}"));
        }
        let report = serde_json::json!({"passed":true,"physicsIntegrations":0,"bytes":bytes.len(),
            "nativeSubsteps":restored.simulation.substeps_completed,
            "forecastSubsteps":restored.simulation.getup_controller.as_ref().unwrap().status.forecast_substeps,
            "binaryWorldDeserialized":true,"jointStateEqual":same_joints,"handlesEqual":same_handles,
            "snapshotEqual":same_snapshot,"controllerEqual":same_controller,"nativeOwnerEqual":same_native_owner});
        #[cfg(feature = "root-translation-carry-experiment")]
        let report = {
            let mut report = report;
            report["rootIntegratorStateEqual"] = serde_json::json!(same_integrator);
            report
        };
        Ok(report.to_string())
    }

    #[cfg(feature = "getup-policy-adaptation-experiment")]
    pub fn getup_adaptation_state(&self) -> Result<String, String> {
        self.simulation.getup_controller.as_ref().ok_or_else(|| "Missing source controller".to_string())
            .map(|c|c.adaptation_diagnostic().to_string())
    }
    #[cfg(feature = "getup-policy-adaptation-experiment")]
    pub fn set_getup_adaptation_head(&mut self, json: &str) -> Result<(), String> {
        if json.len() > 1024 * 1024 { return Err("Correction head exceeds bound".into()); }
        let head: Option<crate::getup::adaptation::Head> = serde_json::from_str(json).map_err(|e| e.to_string())?;
        self.simulation.getup_controller.as_mut().ok_or("Missing source controller")?.set_adaptation_head(head)
    }
    #[cfg(feature = "getup-policy-adaptation-experiment")]
    pub fn getup_adaptation_preview(&self) -> Result<String, String> {
        self.simulation.getup_controller.as_ref().ok_or("Missing source controller")?
            .adaptation_preview(&self.simulation).map(|v|v.to_string())
    }
    #[cfg(feature = "getup-policy-adaptation-experiment")]
    pub fn queue_getup_correction(&mut self, json: &str) -> Result<(), String> {
        if json.len() > 1024 { return Err("Correction input exceeds bound".into()); }
        let correction: [f32;21] = serde_json::from_str(json).map_err(|e|e.to_string())?;
        let native = self.simulation.substeps_completed;
        self.simulation.getup_controller.as_mut().ok_or("Missing source controller")?.queue_adaptation(native,correction)
    }
    /// Recorded teacher branch/reference only; this never advances either world.
    #[cfg(feature="getup-supported-transfer-experiment")]
    pub fn begin_getup_transfer(&mut self,side:usize)->Result<(),String>{
        self.checkpoint_compatible()?;
        let s=&mut self.simulation;
        let mut controller=s.getup_controller.take().ok_or("Missing GetUp controller")?;
        let result=controller.begin_transfer(s,side);
        s.getup_controller=Some(controller);result
    }
    #[cfg(feature="getup-supported-transfer-experiment")]
    pub fn getup_transfer_report(&self)->String {
        serde_json::json!({"native":self.simulation.substeps_completed,"motion":self.simulation.motion,
            "transfer":self.simulation.getup_controller.as_ref().map(|c|c.transfer_report())}).to_string()
    }
    pub fn getup_report(&self) -> Result<String, String> {
        serde_json::to_string(&self.simulation.getup_controller).map_err(|e| e.to_string())
    }
    /// Read-only qualification evidence, including ownership before integration.
    pub(crate) fn simulation(&self) -> &Simulation {
        &self.simulation
    }
    pub fn new(mode: Mode, heading: f32, profile: Profile) -> Result<Self, String> {
        Self::new_trial(mode, heading, profile, 1, true)
    }
    /// A replacement worker starts an empty trial in a fresh generation. This
    /// initializes bodies once; it cannot transfer active input or running poses.
    pub fn new_trial(
        mode: Mode,
        heading: f32,
        profile: Profile,
        generation: u64,
        floor_enabled: bool,
    ) -> Result<Self, String> {
        Self::new_trial_with_settings(
            mode,
            heading,
            profile,
            generation,
            floor_enabled,
            Default::default(),
        )
    }
    pub fn new_trial_with_settings(
        mode: Mode,
        heading: f32,
        profile: Profile,
        generation: u64,
        floor_enabled: bool,
        playground_settings: lh_contracts::PlaygroundSettings,
    ) -> Result<Self, String> {
        if generation == 0 || generation > 9_007_199_254_740_991 {
            return Err("Invalid initial generation".into());
        }
        let mut simulation = match mode {
            Mode::Protocol => Simulation::new_protocol(heading, profile, floor_enabled)?,
            Mode::Playground => {
                Simulation::new_playground(heading, profile, floor_enabled, playground_settings)?
            }
        };
        simulation.generation = generation;
        if let Some(state)=&mut simulation.native_locomotion {state.set_initial_generation(generation);}
        Ok(Self {
            timeline: Timeline::new(simulation.generation),
            simulation,
            quiet: None,
            recovery_measurement: None,
            heading,
            mode,
            paused: false,
            visible: true,
            disposed: false,
            highest_press: 0,
            observation_sequence: 0,
            playground_settings,
            acknowledgements: Vec::new(),
        })
    }
    /// None means queued, not applied. Every command receives a later application
    /// or rejection acknowledgement at its exact requested tick boundary.
    pub fn enqueue(&mut self, command: Command) -> Result<(), Reject> {
        if self.disposed {
            return Err(Reject::Invalid);
        }
        if self.quiet.is_some() {
            return Err(Reject::Unsupported);
        }
        if command.stamp.schema != SCHEMA_VERSION {
            return Err(Reject::Schema);
        }
        if command.stamp.generation != self.timeline.generation {
            return Err(Reject::Generation);
        }
        if matches!(command.action, Action::Strike | Action::StrikeFromBehind)
            && self.mode == Mode::Protocol
            && (self.paused
                || !self.visible
                || self.simulation.first_failure.is_some()
                || self
                    .simulation
                    .striker
                    .as_ref()
                    .is_none_or(|s| !s.available()))
        {
            return Err(Reject::Cancelled);
        }
        if (!self.visible
            || self.paused
            || !matches!(
                self.simulation.motion,
                lh_contracts::Motion::Upright | lh_contracts::Motion::Reacting
            ))
            && matches!(
                command.action,
                Action::GrabBegin { .. } | Action::GrabMove { .. }
            )
        {
            return Err(Reject::Cancelled);
        }
        if matches!(
            command.action,
            Action::Pause
                | Action::Resume
                | Action::Reset
                | Action::Initialize { .. }
                | Action::SelectStation { .. }
                | Action::SetDifficulty { .. }
                | Action::Strike
                | Action::StrikeFromBehind
        ) && command.stamp.tick != self.timeline.tick
        {
            return Err(Reject::Late);
        }
        let press = match &command.action {
            Action::GrabBegin { press, .. } => Some(*press),
            _ => None,
        };
        self.timeline.enqueue(command)?;
        if let Some(press) = press {
            self.highest_press = self.highest_press.max(press);
        }
        Ok(())
    }
    fn cancel_pointer_intent(&mut self) {
        self.simulation.cancel_grab();
        self.simulation.last_press = self.simulation.last_press.max(self.highest_press);
        for command in self.timeline.cancel_grabs() {
            self.acknowledgements
                .push(acknowledgement(&command, None, Some(Reject::Cancelled)));
        }
    }
    /// Enqueue is atomic across a worker packet. Rejection cannot leave hidden
    /// intent waiting to execute after the caller receives a failed packet.
    pub fn enqueue_batch(&mut self, commands: Vec<Command>) -> Result<(), Reject> {
        if self.disposed {
            return Err(Reject::Invalid);
        }
        if commands.len() > lh_contracts::MAX_QUEUED_COMMANDS {
            return Err(Reject::QueueFull);
        }
        let before = self.timeline.clone();
        let highest_press = self.highest_press;
        for command in commands {
            if let Err(reject) = self.enqueue(command) {
                self.timeline = before;
                self.highest_press = highest_press;
                return Err(reject);
            }
        }
        Ok(())
    }
    pub fn set_visible(&mut self, visible: bool) -> Result<(), Reject> {
        if self.disposed {
            return Err(Reject::Invalid);
        }
        if self.quiet.is_some() && !visible {
            return Err(Reject::Unsupported);
        }
        if !visible {
            self.cancel_pointer_intent();
        }
        self.visible = visible;
        Ok(())
    }
    pub fn shutdown(&mut self) {
        self.cancel_pointer_intent();
        for command in self.timeline.clear() {
            self.acknowledgements
                .push(acknowledgement(&command, None, Some(Reject::Cancelled)));
        }
        self.disposed = true;
    }
    fn replace(
        &mut self,
        mode: Mode,
        heading: f32,
        settings: lh_contracts::PlaygroundSettings,
    ) -> Result<(), Reject> {
        let generation = self
            .timeline
            .generation
            .checked_add(1)
            .ok_or(Reject::Invalid)?;
        let floor = self.simulation.world.colliders[self.simulation.floor].is_enabled();
        let profile = self.simulation.profile.clone();
        let mut next = match mode {
            Mode::Protocol => Simulation::new_protocol(heading, profile, floor),
            Mode::Playground => Simulation::new_playground(heading, profile, floor, settings),
        }
        .map_err(|_| Reject::Invalid)?;
        next.generation = generation;
        if let Some(state)=&mut next.native_locomotion {state.set_initial_generation(generation);}
        next.last_press = self.highest_press;
        // Construct first so a failed initialization preserves the old world.
        self.cancel_pointer_intent();
        for command in self.timeline.clear() {
            self.acknowledgements
                .push(acknowledgement(&command, None, Some(Reject::Generation)));
        }
        self.simulation = next;
        self.timeline = Timeline::new(generation);
        self.heading = heading;
        self.mode = mode;
        self.playground_settings = settings;
        self.observation_sequence = 0;
        Ok(())
    }
    fn apply(&mut self, command: &Command) -> Result<(), Reject> {
        if command.stamp.generation != self.timeline.generation {
            return Err(Reject::Generation);
        }
        match command.action {
            Action::Pause => {
                self.paused = true;
                self.cancel_pointer_intent();
            }
            Action::Resume => {
                self.paused = false;
            }
            Action::CancelGrab => self.cancel_pointer_intent(),
            Action::Reset => self.replace(self.mode, self.heading, self.playground_settings)?,
            Action::Initialize { mode, heading } => {
                self.replace(mode, heading, self.playground_settings)?
            }
            Action::SetFloorEnabled { enabled } => self
                .simulation
                .set_floor_enabled(enabled)
                .map_err(|_| Reject::Invalid)?,
            Action::SelectStation { station } if self.mode == Mode::Playground => {
                self.replace(
                    self.mode,
                    self.heading,
                    lh_contracts::PlaygroundSettings {
                        station,
                        ..self.playground_settings
                    },
                )?;
            }
            Action::SetDifficulty { difficulty } if self.mode == Mode::Playground => {
                self.replace(
                    self.mode,
                    self.heading,
                    lh_contracts::PlaygroundSettings {
                        difficulty,
                        ..self.playground_settings
                    },
                )?;
            }
            ref action @ (Action::Strike | Action::StrikeFromBehind) if self.mode == Mode::Protocol => {
                if self.paused || !self.visible { return Err(Reject::Cancelled); }
                let requested = if matches!(action, Action::StrikeFromBehind) {
                    self.simulation.request_strike_from_behind()
                } else { self.simulation.request_strike() };
                if !requested.map_err(|_| Reject::Invalid)?
                {
                    return Err(Reject::Cancelled);
                }
            }
            Action::Strike | Action::StrikeFromBehind | Action::SelectStation { .. } | Action::SetDifficulty { .. } => {
                return Err(Reject::Unsupported);
            }
            _ => {
                if (!self.visible
                    || self.paused
                    || !matches!(
                        self.simulation.motion,
                        lh_contracts::Motion::Upright | lh_contracts::Motion::Reacting
                    ))
                    && matches!(
                        command.action,
                        Action::GrabBegin { .. } | Action::GrabMove { .. }
                    )
                {
                    return Err(Reject::Cancelled);
                }
                self.simulation.apply_grab_action(&command.action)?;
            }
        }
        if command.stamp.generation == self.timeline.generation {
            self.timeline.last_applied = self.timeline.last_applied.max(command.stamp.sequence);
            self.simulation.applied_sequence = self.timeline.last_applied;
        }
        if let Some(observer) = self.recovery_measurement.as_mut() {
            observer.applied(&self.simulation, &command.action);
        }
        Ok(())
    }
    fn boundary(&mut self) {
        for command in self.timeline.boundary() {
            let tick = self.timeline.tick;
            let result = self.apply(&command);
            self.acknowledgements.push(acknowledgement(
                &command,
                result.as_ref().ok().map(|_| tick),
                result.err(),
            ));
        }
    }
    pub fn advance(&mut self, ticks: u32) -> Result<Reply, Reject> {
        self.advance_traced(ticks, |_, _| {})
    }
    #[cfg(feature = "getup-terminal-tail-observer-experiment")]
    pub fn terminal_tail_build_identity() -> serde_json::Value {
        serde_json::json!({"experiment":"native-motor-observer-v1",
            "variant":"source-normal-motors",
            "checkpointResumeSupported":true,"checkpointExportSupported":true,
            "observerFeature":"getup-terminal-tail-observer-experiment",
            "footFloorClearance":{"order":["left_foot","right_foot"],"units":"metres",
                "source":"minimum transformed source primitive endpoint Y minus radius minus actual floor collider AABB maximum Y",
                "separationCorroboration":"strictly positive; independent of the existing 3 N unload criterion"},
            "nativeTrace":"read-only-post-native-observer-v1","maxTicksPerCall":1,
            "poseWordLayout":"per body: positionXYZ,quaternionXYZW,cachedLinearVelocityXYZ,cachedAngularVelocityXYZ",
            "currentMotionSource":"fresh physicalObservationWords and current.motion; pose velocity words are cache receipts"})
    }
    /// Forecast-only diagnostic. No command, boundary, or real-world step.
    #[cfg(feature = "getup-terminal-tail-observer-experiment")]
    pub fn getup_teacher_feasibility(&self) -> Result<serde_json::Value, String> {
        self.checkpoint_compatible()?;
        crate::getup::controller::teacher_feasibility_probe(&self.simulation)
    }
    /// Read-only contact/mass inspection of an exact restored physical world.
    /// All matrix refreshes are confined to a detached multibody clone.
    #[cfg(feature = "getup-terminal-tail-observer-experiment")]
    pub fn getup_roll_checkpoint_diagnostic(&self) -> Result<serde_json::Value, String> {
        self.checkpoint_compatible()?;
        crate::getup::roll_inspection::diagnostic(&self.simulation)
    }
    /// Test-only, bounded immutable observation of the original advance path.
    /// Trace rows are local to this call and never enter simulation/checkpoints.
    #[cfg(feature = "getup-terminal-tail-observer-experiment")]
    pub fn advance_terminal_tail_observed(&mut self, ticks: u32) -> Result<serde_json::Value, Reject> {
        if ticks > 1 { return Err(Reject::Invalid); }
        let mut rows = Vec::with_capacity(20);
        let mut trace_error = None;
        let reply = self.advance_observed(ticks, |s, _| {
            let observation = (|| -> Result<serde_json::Value, String> {
                let snapshot = s.snapshot();
                let pose_words: Vec<_> = snapshot.segments.iter().flat_map(|p| [
                    p.position.x,p.position.y,p.position.z,
                    p.rotation.x,p.rotation.y,p.rotation.z,p.rotation.w,
                    p.linear_velocity.x,p.linear_velocity.y,p.linear_velocity.z,
                    p.angular_velocity.x,p.angular_velocity.y,p.angular_velocity.z,
                ].map(f32::to_bits)).collect();
                let physical = crate::getup::physical_observation(s)?;
                let original_anchors = s.getup_controller.as_ref().and_then(|c|c.stable_foot_anchors());
                let diagnostic = crate::getup::controller::standing_diagnostic(s, original_anchors)?;
                let motor_application = s.last_motor_application.as_ref().ok_or("Missing applied motor receipt")?;
                let motor_json = serde_json::to_value(motor_application).map_err(|error|error.to_string())?;
                if motor_json["commandInputNativeSubstep"].as_u64() != s.substeps_completed.checked_sub(1) {
                    return Err("Stale applied motor receipt".into());
                }
                let foot_clearance = crate::getup::foot_floor_clearance(s)?;
                #[cfg(feature = "root-translation-carry-experiment")]
                let carry = {
                    let mut entries: Vec<_> = s.world.multibody_joints.multibodies().map(|mb| {
                        let (index, generation) = mb.root().rigid_body_handle().into_raw_parts();
                        RootTranslationCheckpointEntry { root:[index,generation], state:mb.root_translation_integrator_state() }
                    }).collect();
                    entries.sort_unstable_by_key(|entry|entry.root);
                    serde_json::to_string(&entries).map_err(|error|error.to_string())?
                };
                #[cfg(not(feature = "root-translation-carry-experiment"))]
                let carry = "[]";
                Ok(serde_json::json!({"nativeSubsteps":s.substeps_completed,
                    "commandInputNativeSubstep":s.substeps_completed.saturating_sub(1),
                    "physicalSeconds":s.substeps_completed as f64 / 800.0,
                    "massKg":s.total_mass(),"weightN":s.total_mass()*9.81,
                    "originalStableFeet":original_anchors,"motion":snapshot.motion,"counters":snapshot.counters,
                    "rootTranslationIntegratorStateJson":carry,
                    "poseWords":pose_words,"physicalObservationWords":physical.map(f32::to_bits).to_vec(),
                    "current":diagnostic,"getup":snapshot.diagnostics.getup,
                    "motorApplication":motor_json,"footFloorClearanceM":foot_clearance,
                    "integrityLimits":{"maxAnchorSeparationM":snapshot.diagnostics.max_anchor_separation_m,
                        "maxFloorPenetrationM":snapshot.diagnostics.max_floor_penetration_m,
                        "maxSelfPenetrationM":snapshot.diagnostics.max_self_penetration_m,
                        "maxLimitErrorRad":snapshot.diagnostics.max_limit_error_rad},
                    "requestedMotorEffortNm":snapshot.diagnostics.requested_motor_effort_nm,
                    "directPelvisForceN":snapshot.diagnostics.direct_pelvis_force_n,
                    "directPelvisTorqueNm":snapshot.diagnostics.direct_pelvis_torque_nm}))
            })();
            match observation {
                Ok(row) => rows.push(row),
                Err(error) => { if trace_error.is_none() { trace_error = Some(error); } },
            }
            Ok(())
        })?;
        let native0_state = if ticks == 0 && self.simulation.substeps_completed == 0 {
            let s = &self.simulation;
            let pose_words: Vec<_> = reply.snapshot.segments.iter().flat_map(|p| [
                p.position.x,p.position.y,p.position.z,p.rotation.x,p.rotation.y,p.rotation.z,p.rotation.w,
                p.linear_velocity.x,p.linear_velocity.y,p.linear_velocity.z,
                p.angular_velocity.x,p.angular_velocity.y,p.angular_velocity.z,
            ].map(f32::to_bits)).collect();
            let physical_words = crate::getup::physical_observation(s).map(|p|p.map(f32::to_bits).to_vec());
            #[cfg(feature = "root-translation-carry-experiment")]
            let carry = self.root_translation_checkpoint_entries().and_then(|entries|
                serde_json::to_string(&entries).map_err(|error|error.to_string()));
            #[cfg(not(feature = "root-translation-carry-experiment"))]
            let carry: Result<String,String> = Ok("[]".into());
            Some(serde_json::json!({"nativeSubsteps":0,"physicsIntegrations":0,"poseWords":pose_words,
                "physicalObservationWords":physical_words.as_ref().ok(),
                "rootTranslationIntegratorStateJson":carry.as_ref().ok(),
                "physicalObservationError":physical_words.err(),"integratorReadError":carry.err()}))
        } else { None };
        Ok(serde_json::json!({"reply":reply,"nativeTrace":rows,
            "native0State":native0_state,"buildIdentity":Self::terminal_tail_build_identity(),"traceError":trace_error}))
    }
    /// Enable independent ownership/contact/posture evidence before integration.
    /// The normal public command path and physical controllers are preserved.
    pub fn enable_recovery_measurement(&mut self) -> Result<(), Reject> {
        if self.disposed
            || self.recovery_measurement.is_some()
            || self.quiet.is_some()
            || self.paused
            || !self.visible
            || self.mode != Mode::Protocol
            || !self.simulation.profile.recovery_enabled
            || !crate::recovery_contract::profile_matches_pinned(&self.simulation.profile)
            || self.simulation.substeps_completed != 0
            || self.timeline.depth() != 0
            || self.timeline.last_applied != 0
            || self.simulation.first_failure.is_some()
            || !self.simulation.world.colliders[self.simulation.floor].is_enabled()
        {
            return Err(Reject::Invalid);
        }
        self.recovery_measurement = Some(crate::cycles::QualificationObservation::new(
            &self.simulation,
        ));
        Ok(())
    }
    pub fn recovery_progress(&self) -> Option<serde_json::Value> {
        self.recovery_measurement
            .as_ref()
            .map(|observer| observer.report(&self.simulation))
    }
    /// Only an untouched, visible trial on the flat floor can be measured.
    /// No body state or controller setting is changed by this observer.
    pub fn enable_quiet_measurement(&mut self) -> Result<(), Reject> {
        if self.disposed
            || self.quiet.is_some()
            || self.paused
            || !self.visible
            || self.simulation.tick != 0
            || self.timeline.depth() != 0
            || self.timeline.last_applied != 0
            || self.simulation.first_failure.is_some()
            || self.mode == Mode::Playground
                && self.playground_settings.station != lh_contracts::Station::Flat
            || !self.simulation.world.colliders[self.simulation.floor].is_enabled()
        {
            return Err(Reject::Invalid);
        }
        self.quiet = Some(crate::quiet::QuietObservation::new(
            &self.simulation,
            self.heading,
        ));
        Ok(())
    }
    pub fn quiet_progress(&self) -> Option<serde_json::Value> {
        self.quiet
            .as_ref()
            .map(|observer| observer.progress(&self.simulation))
    }
    pub fn enable_quiet_drift_trace(&mut self) -> Result<(), Reject> {
        if self.simulation.tick != 0 || self.disposed {
            return Err(Reject::Invalid);
        }
        if !self
            .quiet
            .as_mut()
            .ok_or(Reject::Invalid)?
            .enable_drift_trace(&self.simulation)
        {
            return Err(Reject::Invalid);
        }
        Ok(())
    }
    pub fn quiet_report(&self) -> Option<serde_json::Value> {
        self.quiet.as_ref().map(|observer| {
            let mut report = observer.report(&self.simulation);
            report["mode"] = serde_json::json!(self.mode);
            report
        })
    }
    pub fn advance_observed(
        &mut self,
        ticks: u32,
        observer: impl FnMut(&Simulation, u32) -> Result<(), Failure>,
    ) -> Result<Reply, Reject> {
        self.advance_with(ticks, observer, |_, _| {})
    }
    pub fn advance_traced(
        &mut self,
        ticks: u32,
        trace: impl FnMut(&Simulation, crate::profiling::Event),
    ) -> Result<Reply, Reject> {
        self.advance_with(ticks, |_, _| Ok(()), trace)
    }
    fn advance_with(
        &mut self,
        ticks: u32,
        mut observer: impl FnMut(&Simulation, u32) -> Result<(), Failure>,
        mut trace: impl FnMut(&Simulation, crate::profiling::Event),
    ) -> Result<Reply, Reject> {
        if self.disposed || ticks > MAX_ADVANCE_TICKS {
            return Err(Reject::Invalid);
        }
        let mut completed_ticks = 0;
        // Tick zero is also a lifecycle boundary while hidden or paused.
        if ticks == 0 {
            self.boundary();
        }
        for _ in 0..ticks {
            let event = |s: &Simulation, begin| crate::profiling::Event {
                generation: s.generation,
                tick: s.tick,
                substep: 0,
                phase: crate::profiling::Phase::Tick,
                begin,
            };
            trace(&self.simulation, event(&self.simulation, true));
            self.boundary();
            if self.paused
                || !self.visible
                || self.simulation.first_failure.is_some()
                || (self.quiet.is_some() && self.simulation.tick == crate::quiet::END_TICK)
            {
                trace(&self.simulation, event(&self.simulation, false));
                break;
            }
            let quiet = &mut self.quiet;
            let recovery_measurement = &mut self.recovery_measurement;
            if self
                .simulation
                .advance_tick_traced(
                    |simulation, substep| {
                        if let Some(quiet) = quiet.as_mut() {
                            quiet.observe(simulation, substep)?;
                        }
                        if let Some(observer) = recovery_measurement.as_mut() {
                            observer.observe(simulation, substep)?;
                        }
                        observer(simulation, substep)
                    },
                    &mut trace,
                )
                .is_err()
            {
                self.cancel_pointer_intent();
                trace(&self.simulation, event(&self.simulation, false));
                break;
            }
            self.timeline.tick = self.simulation.tick;
            if matches!(
                self.simulation.motion,
                lh_contracts::Motion::Falling | lh_contracts::Motion::Fallen
            ) {
                self.cancel_pointer_intent();
            }
            if let Some(quiet) = self.quiet.as_mut() {
                quiet.completed_tick(&self.simulation);
            }
            completed_ticks += 1;
            trace(&self.simulation, event(&self.simulation, false));
        }
        self.observation_sequence = self
            .observation_sequence
            .checked_add(1)
            .ok_or(Reject::Invalid)?;
        let mut snapshot = self.simulation.snapshot();
        snapshot.stamp = Stamp::new(
            self.timeline.generation,
            self.observation_sequence,
            self.timeline.tick,
        );
        snapshot.diagnostics.queue_depth = self.timeline.depth() as u32;
        Ok(Reply {
            completed_ticks,
            paused: self.paused,
            visible: self.visible,
            mode: self.mode,
            acknowledgements: std::mem::take(&mut self.acknowledgements),
            snapshot,
            failure: self.simulation.first_failure.clone(),
        })
    }
}

#[cfg(test)]
#[path = "getup_fixture_profile_tests.rs"]
mod getup_fixture_profile_tests;

#[cfg(test)]
#[path = "getup_root_translation_checkpoint_tests.rs"]
mod getup_root_translation_checkpoint_tests;

#[cfg(test)]
#[path = "getup_terminal_base_ready_checkpoint_tests.rs"]
mod getup_terminal_base_ready_checkpoint_tests;

#[cfg(test)]
#[path = "getup_terminal_bumpless_checkpoint_tests.rs"]
mod getup_terminal_bumpless_checkpoint_tests;

#[cfg(test)]
mod tests {
    use super::*;
    use lh_model::Vec3;

    #[test]
    fn getup_checkpoint_round_trip_is_complete_and_never_steps() {
        let mut profile = Profile::default();
        profile.getup_policy_enabled = true;
        let mut live = Runtime::new(Mode::Protocol, 0.0, profile).unwrap();
        let root = live.simulation.bodies[0];
        live.simulation.world.bodies[root].add_torque(rapier3d::prelude::Vector::X * 0.125, false);
        let checkpoint = live.getup_checkpoint().unwrap();
        let restored = Runtime::from_getup_checkpoint(&checkpoint).unwrap();
        let verification: serde_json::Value = serde_json::from_str(&live.verify_getup_checkpoint(&checkpoint).unwrap()).unwrap();
        assert_eq!(verification["passed"], true);
        assert_eq!(verification["nativeSubsteps"], 0);
        assert_eq!(restored.simulation.substeps_completed, 0);
        assert_eq!(restored.timeline.tick, live.timeline.tick);
        assert_eq!(restored.getup_report().unwrap(), live.getup_report().unwrap());
        assert!(Runtime::from_getup_checkpoint(&checkpoint[..checkpoint.len()/2]).is_err());
        let mut trailing = checkpoint.clone(); trailing.push(0);
        assert!(Runtime::from_getup_checkpoint(&trailing).is_err());
        assert!(Runtime::new(Mode::Protocol, 0.0, Profile::default()).unwrap().getup_checkpoint().is_err());
    }
    #[test]
    fn read_only_observer_keeps_every_physical_reply_and_visits_each_substep() {
        let mut plain = runtime();
        let mut observed = runtime();
        let mut visits = Vec::new();
        for _ in 0..6 {
            let expected = plain.advance(4).unwrap();
            let actual = observed
                .advance_observed(4, |simulation, substep| {
                    visits.push((simulation.tick, substep));
                    Ok(())
                })
                .unwrap();
            assert_eq!(
                serde_json::to_value(actual).unwrap(),
                serde_json::to_value(expected).unwrap()
            );
        }
        assert_eq!(
            visits,
            (0..24)
                .flat_map(|tick| (0..lh_contracts::SUBSTEPS).map(move |substep| (tick, substep)))
                .collect::<Vec<_>>()
        );
    }
    #[test]
    fn observer_failure_halts_at_its_exact_substep_and_cancels_future_grabs() {
        let mut owner = runtime();
        submit(&mut owner, 1, 0, begin(41));
        submit(&mut owner, 2, 2, begin(42));
        let reply = owner
            .advance_observed(4, |simulation, substep| {
                if simulation.tick == 1 && substep == 2 {
                    Err(Failure::new(
                        1,
                        2,
                        "observer-sentinel",
                        "read-only test".into(),
                        1.0,
                        0.0,
                    ))
                } else {
                    Ok(())
                }
            })
            .unwrap();
        assert_eq!(reply.completed_ticks, 1);
        assert_eq!(reply.failure.as_ref().unwrap().check, "observer-sentinel");
        assert_eq!(reply.failure.as_ref().unwrap().substep, 2);
        assert!(owner.simulation.grab.is_none());
        assert_eq!(owner.timeline.depth(), 0);
        assert!(
            reply
                .acknowledgements
                .iter()
                .any(|ack| ack.stamp.sequence == 2 && ack.rejected == Some(Reject::Cancelled))
        );
        let frozen = serde_json::to_value(&reply.snapshot.segments).unwrap();
        let again = owner.advance(4).unwrap();
        assert_eq!(again.completed_ticks, 0);
        assert_eq!(
            serde_json::to_value(&again.snapshot.segments).unwrap(),
            frozen
        );
    }
    #[test]
    fn quiet_trial_rejects_contaminated_start_and_commands() {
        let mut pending = runtime();
        submit(&mut pending, 1, 0, begin(1));
        assert_eq!(pending.enable_quiet_measurement(), Err(Reject::Invalid));
        let mut hidden = runtime();
        hidden.set_visible(false).unwrap();
        assert_eq!(hidden.enable_quiet_measurement(), Err(Reject::Invalid));
        let mut advanced = runtime();
        advanced.advance(1).unwrap();
        assert_eq!(advanced.enable_quiet_measurement(), Err(Reject::Invalid));
        let mut no_floor =
            Runtime::new_trial(Mode::Protocol, 0.0, Profile::default(), 1, false).unwrap();
        assert_eq!(no_floor.enable_quiet_measurement(), Err(Reject::Invalid));
        let mut quiet = runtime();
        quiet.enable_quiet_measurement().unwrap();
        assert_eq!(quiet.enable_quiet_measurement(), Err(Reject::Invalid));
        assert_eq!(
            quiet.enqueue(Command {
                stamp: Stamp::new(1, 1, 0),
                action: begin(1)
            }),
            Err(Reject::Unsupported)
        );
        assert_eq!(quiet.set_visible(false), Err(Reject::Unsupported));
        assert!(quiet.advance(5).is_err());
        assert_eq!(quiet.quiet_progress().unwrap()["quietSubsteps"], 0);
    }
    #[test]
    fn full_quiet_trial_matches_the_independent_native_observer_and_stops_at_1920() {
        let profile = Profile::default();
        let mut direct = Simulation::new_protocol(0.0, profile.clone(), true).unwrap();
        let mut owner = Runtime::new(Mode::Protocol, 0.0, profile).unwrap();
        assert_eq!(owner.enable_quiet_drift_trace(), Err(Reject::Invalid));
        owner.enable_quiet_measurement().unwrap();
        owner.enable_quiet_drift_trace().unwrap();
        assert_eq!(owner.enable_quiet_drift_trace(), Err(Reject::Invalid));
        let initial = direct.snapshot();
        for _ in 0..120 {
            direct.advance_tick().unwrap();
        }
        let settled = direct.snapshot();
        let mut metrics = crate::measurement::QuietMetrics::default();
        for _ in 0..1800 {
            direct
                .advance_tick_observed(|simulation, substep| {
                    metrics.observe(simulation, &settled, substep)
                })
                .unwrap();
        }
        for _ in 0..639 {
            owner.advance(3).unwrap();
        }
        assert_eq!(owner.advance(2).unwrap().completed_ticks, 2);
        assert_eq!(owner.advance(4).unwrap().completed_ticks, 1);
        let report = owner.quiet_report().unwrap();
        assert_eq!(report["progress"]["passed"], true);
        assert_eq!(
            report["progress"]["settlingSubsteps"],
            crate::quiet::SETTLE_TICKS * u64::from(lh_contracts::SUBSTEPS)
        );
        assert_eq!(
            report["progress"]["quietSubsteps"],
            (crate::quiet::END_TICK - crate::quiet::SETTLE_TICKS)
                * u64::from(lh_contracts::SUBSTEPS)
        );
        assert_eq!(report["settled"]["simulation_time_s"], 3.0);
        assert_eq!(report["final"]["simulation_time_s"], 48.0);
        assert_eq!(report["initial"], serde_json::to_value(initial).unwrap());
        assert_eq!(report["settled"], serde_json::to_value(settled).unwrap());
        assert_eq!(
            report["final"],
            serde_json::to_value(direct.snapshot()).unwrap()
        );
        assert_eq!(
            report["measurements"]["quiet"],
            serde_json::to_value(metrics).unwrap()
        );
        let samples = report["driftTrace"]["samples"].as_array().unwrap();
        assert_eq!(
            samples.len() as u64,
            1 + crate::quiet::END_TICK / u64::from(lh_contracts::TICK_HZ)
        );
        assert_eq!(samples[0]["completedSubsteps"], 0);
        for (second, sample) in samples.iter().enumerate() {
            assert_eq!(
                sample["completedSubsteps"],
                second as u64 * u64::from(lh_contracts::TICK_HZ * lh_contracts::SUBSTEPS)
            );
            assert_eq!(sample["simulationTimeS"], second as f64);
        }
        let last = samples.last().unwrap();
        assert_eq!(
            last["completedSubsteps"],
            crate::quiet::END_TICK * u64::from(lh_contracts::SUBSTEPS)
        );
        assert_eq!(
            last["simulationTimeS"],
            report["final"]["simulation_time_s"]
        );
        // Leg coordinates in this trace currently cover impulse joints only.
        // Empty arrays do not qualify the current model's multibody legs.
        let leg_indices: Vec<_> = direct
            .model
            .segments
            .iter()
            .enumerate()
            .filter(|(_, segment)| {
                matches!(
                    segment.role.as_str(),
                    "thigh" | "shin" | "ankle" | "hindfoot" | "forefoot"
                ) && segment.joint_profile.is_some()
            })
            .map(|(i, _)| i)
            .collect();
        let impulse_joint_leg_count = leg_indices
            .iter()
            .filter(|&&i| direct.joints[i].is_some())
            .count();
        let unsupported_multibody_leg_count = leg_indices
            .iter()
            .filter(|&&i| {
                direct
                    .world
                    .multibody_joints
                    .rigid_body_link(direct.bodies[i])
                    .is_some()
            })
            .count();
        assert!(
            unsupported_multibody_leg_count > 0,
            "current GetUp fixture must expose the unsupported multibody-leg diagnostic gap"
        );
        assert!(
            samples.iter().all(|sample| {
                sample["legs"].as_array().unwrap().len() == impulse_joint_leg_count
            })
        );
        assert_eq!(owner.enable_quiet_drift_trace(), Err(Reject::Invalid));
        assert_eq!(owner.advance(4).unwrap().completed_ticks, 0);
        assert_eq!(owner.quiet_report().unwrap(), report);
    }
    fn runtime() -> Runtime {
        Runtime::new(Mode::Playground, 0.0, Profile::default()).expect("runtime")
    }
    fn submit(r: &mut Runtime, sequence: u64, tick: u64, action: Action) {
        r.enqueue(Command {
            stamp: Stamp::new(r.timeline.generation, sequence, tick),
            action,
        })
        .expect("enqueue");
    }
    fn begin(press: u64) -> Action {
        Action::GrabBegin {
            press,
            segment: 14,
            local_anchor: Vec3::default(),
            target: Vec3 {
                x: 0.2,
                y: 0.9,
                z: 0.0,
            },
        }
    }
    #[test]
    fn replacement_trial_rejects_old_generation_and_starts_with_fresh_intent() {
        let mut baseline = runtime();
        let initial = baseline.advance(0).expect("initial");
        let mut replacement =
            Runtime::new_trial(Mode::Playground, 0.0, Profile::default(), 42, true)
                .expect("replacement");
        let restarted = replacement.advance(0).expect("restarted");
        assert_eq!(restarted.snapshot.stamp.generation, 42);
        assert_eq!(restarted.snapshot.stamp.tick, 0);
        assert_eq!(
            serde_json::to_value(&initial.snapshot.segments).unwrap(),
            serde_json::to_value(&restarted.snapshot.segments).unwrap()
        );
        assert_eq!(
            replacement.enqueue(Command {
                stamp: Stamp::new(1, 1, 0),
                action: begin(41),
            }),
            Err(Reject::Generation)
        );
        let stepped = replacement.advance(1).expect("step");
        assert!(stepped.failure.is_none());
        assert!(replacement.simulation.grab.is_none());
        assert!(Runtime::new_trial(Mode::Playground, 0.0, Profile::default(), 0, true).is_err());
        let mut unsupported =
            Runtime::new_trial(Mode::Playground, 0.0, Profile::default(), 43, false).unwrap();
        assert!(unsupported.advance(4).unwrap().failure.is_none());
        assert!(!unsupported.simulation.world.colliders[unsupported.simulation.floor].is_enabled());
    }
    #[test]
    fn pause_and_visibility_cancel_active_and_future_grabs_without_advancing_bodies() {
        for hidden in [false, true] {
            let mut r = runtime();
            submit(&mut r, 1, 0, begin(41));
            r.advance(1).expect("start");
            submit(&mut r, 2, 3, begin(42));
            if hidden {
                r.set_visible(false).expect("hide");
            } else {
                submit(&mut r, 3, 1, Action::Pause);
            }
            let frozen = r.advance(4).expect("suspend");
            assert_eq!(frozen.completed_ticks, 0);
            assert!(r.simulation.grab.is_none());
            assert_eq!(r.timeline.depth(), 0);
            assert!(
                frozen
                    .acknowledgements
                    .iter()
                    .any(|a| a.rejected == Some(Reject::Cancelled))
            );
            let again = r.advance(4).expect("still suspended");
            assert_eq!(
                serde_json::to_value(&frozen.snapshot.segments).expect("poses"),
                serde_json::to_value(&again.snapshot.segments).expect("poses")
            );
            assert_eq!(
                frozen.snapshot.simulation_time_s,
                again.snapshot.simulation_time_s
            );
            assert!(lh_contracts::accepts_observation(
                frozen.snapshot.stamp,
                again.snapshot.stamp
            ));
            if hidden {
                r.set_visible(true).expect("show");
            } else {
                submit(&mut r, 4, 1, Action::Resume);
            }
            r.advance(0).expect("resume boundary");
            submit(&mut r, 5, 1, begin(42));
            assert_eq!(
                r.advance(0).expect("old press").acknowledgements[0].rejected,
                Some(Reject::Invalid)
            );
            submit(&mut r, 6, 1, begin(43));
            assert!(
                r.advance(1).expect("fresh press").acknowledgements[0]
                    .rejected
                    .is_none()
            );
        }
    }
    #[test]
    fn reset_isolates_queued_commands_and_stale_generations() {
        let mut r = runtime();
        let before = r.advance(1).expect("first").snapshot;
        submit(&mut r, 1, 3, Action::SetFloorEnabled { enabled: false });
        submit(&mut r, 2, 1, Action::Reset);
        let reset = r.advance(0).expect("reset");
        assert_eq!(reset.snapshot.stamp.generation, 2);
        assert_eq!(reset.snapshot.simulation_time_s, 0.0);
        assert_eq!(r.timeline.depth(), 0);
        assert!(
            reset
                .acknowledgements
                .iter()
                .any(|a| a.rejected == Some(Reject::Generation))
        );
        assert_eq!(
            r.enqueue(Command {
                stamp: Stamp::new(1, 3, 0),
                action: Action::Pause
            }),
            Err(Reject::Generation)
        );
        assert_eq!(
            r.enqueue(Command {
                stamp: Stamp::new(1, 4, 99),
                action: Action::Pause
            }),
            Err(Reject::Generation)
        );
        assert!(!lh_contracts::accepts_observation(
            reset.snapshot.stamp,
            before.stamp
        ));
        assert!(r.simulation.world.colliders[r.simulation.floor].is_enabled());
    }
    #[test]
    fn floor_disable_clears_support_at_the_boundary_and_preserves_body_identity() {
        let mut r = runtime();
        r.advance(4).expect("ground");
        let handles = r.simulation.bodies.clone();
        submit(&mut r, 1, 4, Action::SetFloorEnabled { enabled: false });
        let cleared = r.advance(0).expect("disable");
        assert!(cleared.snapshot.contacts.is_empty());
        let com = r.simulation.center_of_mass();
        let falling = r.advance(4).expect("free fall");
        assert!(falling.snapshot.contacts.is_empty());
        assert!(r.simulation.center_of_mass().y < com.y);
        assert_eq!(handles, r.simulation.bodies);
    }
    #[test]
    fn observed_fall_cancels_future_input_and_reset_opens_a_fresh_trial() {
        let mut r = runtime();
        submit(&mut r, 1, 0, Action::SetFloorEnabled { enabled: false });
        submit(&mut r, 2, 20, begin(41));
        let mut acknowledgements = Vec::new();
        for _ in 0..3 {
            acknowledgements.extend(r.advance(4).unwrap().acknowledgements);
        }
        assert_eq!(r.simulation.motion, lh_contracts::Motion::Falling);
        assert_eq!(r.simulation.counters.falls, 1);
        assert_eq!(r.timeline.depth(), 0);
        assert!(acknowledgements.iter().any(|ack| ack.stamp.sequence == 2
            && ack.rejected == Some(Reject::Cancelled)
            && ack.applied_tick.is_none()));
        assert_eq!(
            r.enqueue(Command {
                stamp: Stamp::new(1, 3, 12),
                action: begin(42)
            }),
            Err(Reject::Cancelled)
        );
        submit(&mut r, 4, 12, Action::Reset);
        let reset = r.advance(0).unwrap();
        assert_eq!(reset.snapshot.stamp.generation, 2);
        assert_eq!(reset.snapshot.motion, lh_contracts::Motion::Upright);
        assert_eq!(reset.snapshot.counters.falls, 0);
        submit(&mut r, 5, 0, begin(42));
        assert!(
            r.advance(0)
                .unwrap()
                .acknowledgements
                .iter()
                .all(|ack| ack.rejected.is_none())
        );
    }
    #[test]
    fn preaccepted_future_commands_apply_once_at_their_requested_tick() {
        let mut r = runtime();
        submit(&mut r, 1, 2, begin(41));
        submit(&mut r, 2, 0, Action::SetFloorEnabled { enabled: false });
        let result = r.advance(3).expect("scheduled advances");
        assert_eq!(result.acknowledgements.len(), 2);
        assert_eq!(result.acknowledgements[0].applied_tick, Some(0));
        assert_eq!(result.acknowledgements[1].applied_tick, Some(2));
        assert!(result.acknowledgements.iter().all(|a| a.rejected.is_none()));
        assert!(r.simulation.grab.is_some());
        assert_eq!(result.snapshot.acknowledged_sequence, 2);
        assert!(r.advance(0).expect("no replay").acknowledgements.is_empty());
    }
    #[test]
    fn invalid_batches_teardown_and_unported_actions_cannot_claim_execution() {
        let mut r = runtime();
        assert!(matches!(r.advance(5), Err(Reject::Invalid)));
        assert_eq!(r.simulation.tick, 0);
        submit(&mut r, 1, 0, Action::Strike);
        let reply = r.advance(0).expect("acknowledge");
        assert_eq!(
            reply.acknowledgements[0].rejected,
            Some(Reject::Unsupported)
        );
        assert_eq!(reply.acknowledgements[0].applied_tick, None);
        assert_eq!(reply.snapshot.counters.strikes, 0);
        r.shutdown();
        assert_eq!(r.enqueue_batch(Vec::new()), Err(Reject::Invalid));
        assert!(matches!(r.advance(0), Err(Reject::Invalid)));
        assert_eq!(
            r.enqueue(Command {
                stamp: Stamp::new(1, 2, 0),
                action: Action::Resume
            }),
            Err(Reject::Invalid)
        );
    }
    #[test]
    fn rejected_command_packet_cannot_leave_intent_or_consume_a_press() {
        let mut r = runtime();
        let commands = vec![
            Command {
                stamp: Stamp::new(1, 1, 0),
                action: begin(99),
            },
            Command {
                stamp: Stamp::new(0, 2, 0),
                action: Action::Pause,
            },
        ];
        assert_eq!(r.enqueue_batch(commands), Err(Reject::Generation));
        assert_eq!(r.timeline.depth(), 0);
        assert_eq!(r.highest_press, 0);
        submit(&mut r, 1, 0, begin(1));
        let reply = r.advance(0).expect("fresh packet");
        assert!(reply.acknowledgements[0].rejected.is_none());
        assert!(!reply.paused);
    }
}

#[cfg(all(test,feature="getup-supported-transfer-experiment"))]
mod supported_transfer_restore_tests {
    use super::*;
    #[test]
    fn active_transfer_checkpoint_is_rejected_before_any_physical_advance(){
        let profile=Profile{getup_policy_enabled:true,
            getup_repeated_recovery:Some(crate::GetUpRepeatedRecovery::CompletedCycleConfirmedFallV1),
            getup_supported_transfer:Some(crate::getup::transfer::Profile::GravityOnlyV1),..Profile::default()};
        let runtime=Runtime::new_getup_fixture_with_profile("belly",profile).unwrap();
        let bytes=runtime.getup_checkpoint().unwrap();
        let (schema,model,world,aux):(u32,String,Vec<u8>,String)=bincode::DefaultOptions::new().with_fixint_encoding().deserialize(&bytes).unwrap();
        assert_eq!(schema,3);
        let mut wrapper:TerminalBaseReadyCheckpoint=serde_json::from_str(&aux).unwrap();
        let mut value:serde_json::Value=serde_json::from_str(&wrapper.auxiliary).unwrap();
        value["simulation"]["getup_controller"]["supported_transfer"]=serde_json::to_value(crate::getup::transfer::synthetic_state(&runtime.simulation)).unwrap();
        wrapper.auxiliary=serde_json::to_string(&value).unwrap();
        let changed=bincode::DefaultOptions::new().with_fixint_encoding().serialize(&(schema,model,world,serde_json::to_string(&wrapper).unwrap())).unwrap();
        let error=Runtime::from_getup_checkpoint(&changed).err().expect("active restore must reject");
        assert!(error.contains("Active supported-transfer restoration"),"{error}");
        assert_eq!(runtime.simulation.substeps_completed,0);
    }
}

#[cfg(all(test,feature="getup-native-locomotion-experiment"))]
mod native_owner_roundtrip_tests {
    use super::*;
    #[test]
    fn native_contract_owner_mode_and_activation_are_part_of_exact_verification() {
        let p=Profile{getup_policy_enabled:true,
            getup_repeated_recovery:Some(crate::GetUpRepeatedRecovery::CompletedCycleConfirmedFallV1),
            getup_native_locomotion:Some(crate::getup::locomotion::Profile::NativeJointOwnerV1),..Profile::default()};
        let mut r=Runtime::new(Mode::Protocol,0.0,p).unwrap();
        let active=r.getup_checkpoint().unwrap();
        assert!(r.verify_getup_checkpoint(&active).is_ok());
        r.set_getup_native_mode("passive").unwrap();
        assert!(r.verify_getup_checkpoint(&active).unwrap_err().contains("nativeOwner=false"));
        let passive=r.getup_checkpoint().unwrap();
        assert!(r.verify_getup_checkpoint(&passive).is_ok());
        r.set_getup_native_mode("active").unwrap();
        assert!(r.verify_getup_checkpoint(&active).unwrap_err().contains("nativeOwner=false"));
        let ramp=r.getup_checkpoint().unwrap();
        assert!(r.verify_getup_checkpoint(&ramp).is_ok());
        assert_eq!(r.simulation.substeps_completed,0);
    }
}
