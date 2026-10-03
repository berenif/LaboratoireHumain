#![cfg(not(target_arch = "wasm32"))]

use std::{
    fs,
    path::PathBuf,
    process::Command,
    sync::atomic::{AtomicU64, Ordering},
};

static NEXT_DIRECTORY: AtomicU64 = AtomicU64::new(0);

struct Evidence(PathBuf);
impl Evidence {
    fn new() -> Self {
        let path = std::env::temp_dir().join(format!(
            "lh-acceptance-{}-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos(),
            NEXT_DIRECTORY.fetch_add(1, Ordering::Relaxed)
        ));
        assert!(!path.exists());
        Self(path)
    }
    fn run(&self, args: &[&str]) -> std::process::Output {
        Command::new(env!("CARGO_BIN_EXE_lh-acceptance"))
            .arg(&self.0)
            .args(args)
            .output()
            .unwrap()
    }
}
impl Drop for Evidence {
    fn drop(&mut self) {
        if self.0.is_dir() {
            fs::remove_dir_all(&self.0).unwrap();
        } else if self.0.is_file() {
            fs::remove_file(&self.0).unwrap();
        }
    }
}

#[test]
fn invalid_mode_or_profile_does_not_create_evidence() {
    let out = Evidence::new();
    assert_eq!(
        out.run(&["--diagnose-worker-quiet", "--mode", "unknown"])
            .status
            .code(),
        Some(2)
    );
    assert!(!out.0.exists());
    let profile = Evidence::new();
    let mut value = serde_json::to_value(lh_sim::Profile::default()).unwrap();
    value["dt_s"] = serde_json::json!(0.02);
    fs::write(&profile.0, serde_json::to_vec(&value).unwrap()).unwrap();
    assert_eq!(
        out.run(&["--profile", profile.0.to_str().unwrap()])
            .status
            .code(),
        Some(2)
    );
    assert!(!out.0.exists());
}

#[test]
fn disturbance_exit_matches_the_scoped_result_without_release_admission() {
    for (require_steps, expected) in [(false, 0), (true, 1)] {
        let out = Evidence::new();
        let mut args = vec!["--diagnose-disturbances", "--fixture", "slow-hand-forward"];
        if require_steps {
            args.push("--require-steps");
        }
        let result = out.run(&args);
        assert_eq!(
            result.status.code(),
            Some(expected),
            "{}",
            String::from_utf8_lossy(&result.stderr)
        );
        let summary: serde_json::Value = serde_json::from_slice(&result.stdout).unwrap();
        assert_eq!(summary["passed"], expected == 0);
        assert_eq!(summary["cases"], 1);
        assert_eq!(summary["releaseAccepted"], false);
        let reports: serde_json::Value =
            serde_json::from_slice(&fs::read(out.0.join("disturbances.json")).unwrap()).unwrap();
        assert_eq!(reports[0]["passed"], expected == 0);
    }
}
