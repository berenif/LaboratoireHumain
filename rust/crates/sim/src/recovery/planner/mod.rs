//! Experimental coordination preparation. No production controller is activated.
pub(crate) mod contact_observation;
pub(crate) mod continuation;
pub(crate) mod coordinated_runtime;
pub(crate) mod native_plan_preview;
#[cfg(feature = "numeric-parity-worker")]
pub(crate) mod numeric_parity_worker;
pub(crate) mod native_observation;
pub(crate) mod geometric_tasks;
pub(crate) mod whole_body_controller;
pub(crate) mod support_state;
pub(crate) mod whole_body_tasks;
#[cfg(feature = "coordination-worker")]
pub(crate) mod transition_worker;
