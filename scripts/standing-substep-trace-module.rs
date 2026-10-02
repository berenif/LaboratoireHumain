//! Output-neutral diagnostic sink used only by the isolated standing trace build.
//!
//! The production crate never contains this module.  The diagnostic builder
//! copies the pinned source, adds this file, and inserts read-only snapshots at
//! solver barriers.  A mutex is sufficient because tracing is deliberately
//! single-process and bounded to one physical step.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Mutex, OnceLock};
use std::string::String;
use std::vec::Vec;

static ENABLED: AtomicBool = AtomicBool::new(false);
static EVENTS: OnceLock<Mutex<Vec<String>>> = OnceLock::new();

fn events() -> &'static Mutex<Vec<String>> {
    EVENTS.get_or_init(|| Mutex::new(Vec::new()))
}

/// Clear the sink and enable one diagnostic step.
pub fn begin() {
    events().lock().unwrap().clear();
    ENABLED.store(true, Ordering::SeqCst);
}

/// Disable tracing and return every event in solver order.
pub fn finish() -> Vec<String> {
    ENABLED.store(false, Ordering::SeqCst);
    std::mem::take(&mut *events().lock().unwrap())
}

pub(crate) fn enabled() -> bool {
    ENABLED.load(Ordering::Relaxed)
}

pub(crate) fn push(event: String) {
    if enabled() {
        events().lock().unwrap().push(event);
    }
}
