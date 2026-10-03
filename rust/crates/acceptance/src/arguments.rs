//! Reject ambiguous diagnostic requests before creating evidence or integrating.
use std::collections::BTreeSet;

const DIAGNOSTICS: &[&str] = &[
    "--diagnose-articulation",
    "--diagnose-hybrid-articulation",
    "--diagnose-recovery",
    "--diagnose-terrain",
    "--diagnose-striker",
    "--diagnose-contact-roundoff",
    "--diagnose-tangent-contacts",
    "--diagnose-bounded-tangent-contacts",
    "--diagnose-worker-quiet",
    "--diagnose-worker-quiet-drift",
    "--diagnose-disturbances",
    "--diagnose-foot-load",
];

pub fn validate(args: &[String]) -> Result<(), String> {
    if args
        .first()
        .is_none_or(|s| s.is_empty() || s.starts_with("--"))
    {
        return Err("Usage: lh-acceptance fresh-output-directory [diagnostic] [options]".into());
    }
    let mode = args
        .get(1)
        .map(String::as_str)
        .filter(|s| DIAGNOSTICS.contains(s));
    let mut index = if mode.is_some() { 2 } else { 1 };
    let mut seen = BTreeSet::new();
    while let Some(flag) = args.get(index) {
        if !seen.insert(flag.as_str()) {
            return Err(format!("Duplicate option: {flag}"));
        }
        match flag.as_str() {
            "--profile" => {
                if matches!(
                    mode,
                    Some(
                        "--diagnose-articulation"
                            | "--diagnose-hybrid-articulation"
                            | "--diagnose-contact-roundoff"
                            | "--diagnose-tangent-contacts"
                            | "--diagnose-bounded-tangent-contacts"
                    )
                ) {
                    return Err("This diagnostic requires the unchanged default profile".into());
                }
            }
            "--fixture"
                if matches!(
                    mode,
                    Some("--diagnose-recovery" | "--diagnose-disturbances")
                ) => {}
            "--heading"
                if matches!(
                    mode,
                    Some("--diagnose-worker-quiet" | "--diagnose-worker-quiet-drift")
                ) => {}
            "--mode"
                if matches!(
                    mode,
                    Some("--diagnose-worker-quiet" | "--diagnose-worker-quiet-drift")
                ) => {}
            "--require-steps" if mode == Some("--diagnose-disturbances") => {
                index += 1;
                continue;
            }
            _ => return Err(format!("Unknown, misplaced or inapplicable option: {flag}")),
        }
        let value = args
            .get(index + 1)
            .filter(|s| !s.is_empty() && !s.starts_with("--"))
            .ok_or_else(|| format!("Missing value for {flag}"))?;
        if flag == "--heading" && !value.parse::<f32>().is_ok_and(f32::is_finite) {
            return Err("Heading must be a finite number".into());
        }
        if flag == "--mode" && !matches!(value.as_str(), "protocol" | "playground" | "all") {
            return Err("Mode must be protocol, playground or all".into());
        }
        index += 2;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn check(args: &[&str]) -> Result<(), String> {
        validate(&args.iter().map(|s| (*s).into()).collect::<Vec<_>>())
    }

    #[test]
    fn admits_existing_diagnostics_and_scoped_options() {
        assert!(check(&["evidence/fresh"]).is_ok());
        assert!(check(&["evidence/fresh", "--profile", "profile.json"]).is_ok());
        for mode in DIAGNOSTICS {
            assert!(check(&["evidence/fresh", mode]).is_ok(), "{mode}");
        }
        assert!(
            check(&[
                "out",
                "--diagnose-disturbances",
                "--require-steps",
                "--fixture",
                "pull",
                "--profile",
                "profile.json"
            ])
            .is_ok()
        );
        assert!(check(&["out", "--diagnose-recovery", "--fixture", "crouch"]).is_ok());
        assert!(check(&["out", "--diagnose-worker-quiet-drift", "--heading", "-1.2"]).is_ok());
        for mode in ["protocol", "playground", "all"] {
            assert!(check(&["out", "--diagnose-worker-quiet", "--mode", mode]).is_ok());
            assert!(check(&["out", "--diagnose-worker-quiet-drift", "--mode", mode]).is_ok());
        }
    }

    #[test]
    fn typos_or_misplaced_modes_cannot_run_a_different_gate() {
        for args in [
            vec![],
            vec!["--diagnose-terrain"],
            vec!["out", "--diagnose-terain"],
            vec!["out", "--require-steps"],
            vec!["out", "--fixture", "pull"],
            vec!["out", "--profile", "p.json", "--diagnose-terrain"],
            vec!["out", "--diagnose-terrain", "--diagnose-striker"],
            vec!["out", "--diagnose-striker", "--heading", "0"],
            vec!["out", "--diagnose-terrain", "--mode", "all"],
            vec!["out", "unexpected"],
        ] {
            assert!(check(&args).is_err(), "{args:?}");
        }
    }

    #[test]
    fn rejects_missing_duplicate_and_nonfinite_values() {
        for args in [
            vec!["out", "--profile"],
            vec!["out", "--profile", "--heading"],
            vec!["out", "--profile", "a", "--profile", "b"],
            vec![
                "out",
                "--diagnose-disturbances",
                "--require-steps",
                "--require-steps",
            ],
            vec!["out", "--diagnose-recovery", "--fixture", ""],
            vec!["out", "--diagnose-worker-quiet", "--mode"],
            vec!["out", "--diagnose-worker-quiet", "--mode", "unknown"],
            vec![
                "out",
                "--diagnose-worker-quiet",
                "--mode",
                "all",
                "--mode",
                "all",
            ],
        ] {
            assert!(check(&args).is_err(), "{args:?}");
        }
        for heading in ["NaN", "inf", "-inf", "1e100", "north"] {
            assert!(check(&["out", "--diagnose-worker-quiet", "--heading", heading]).is_err());
        }
    }

    #[test]
    fn fixed_calibrations_reject_profile_overrides() {
        for mode in [
            "--diagnose-articulation",
            "--diagnose-hybrid-articulation",
            "--diagnose-contact-roundoff",
            "--diagnose-tangent-contacts",
            "--diagnose-bounded-tangent-contacts",
        ] {
            assert!(check(&["out", mode, "--profile", "p.json"]).is_err());
        }
    }
}
