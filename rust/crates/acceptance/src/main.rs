use lh_sim::{Profile, Simulation};
use serde_json::json;
use std::fs;
mod arguments;

fn run() -> Result<bool, Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    arguments::validate(&args)?;
    let out = args
        .first()
        .ok_or("Usage: lh-acceptance fresh-output-directory")?;
    if std::path::Path::new(out).exists() {
        return Err("Refuse existing evidence directory".into());
    }
    let profile = if let Some(index) = args.iter().position(|a| a == "--profile") {
        serde_json::from_slice(&fs::read(
            args.get(index + 1).ok_or("Missing profile path")?,
        )?)?
    } else {
        Profile::default()
    };
    profile.validate()?;
    fs::create_dir_all(out)?;
    fs::write(
        format!("{out}/profile.json"),
        serde_json::to_vec_pretty(&profile)?,
    )?;
    #[cfg(not(target_arch = "wasm32"))]
    if args.get(1).is_some_and(|a| {
        matches!(
            a.as_str(),
            "--diagnose-articulation" | "--diagnose-hybrid-articulation"
        )
    }) {
        if args.iter().any(|a| a == "--profile") {
            return Err(
                "Articulation feasibility uses the unchanged default profile; omit --profile"
                    .into(),
            );
        }
        let hybrid = args[1] == "--diagnose-hybrid-articulation";
        let report = if hybrid {
            lh_sim::articulation::diagnose_hybrid(&profile)?
        } else {
            lh_sim::articulation::diagnose(&profile)?
        };
        let name = if hybrid {
            "hybrid-articulation"
        } else {
            "articulation"
        };
        fs::write(
            format!("{out}/{name}.json"),
            serde_json::to_vec_pretty(&report)?,
        )?;
        let passive_summary = report["passiveTrials"].as_array().map(|trials| trials.iter().map(|trial|
            json!({"variant": trial["variant"], "floorEnabled": trial["floorEnabled"],
                "completedSubsteps": trial["completedSubsteps"], "passed": trial["passed"],
                "generalizedChecksPassed": trial["generalizedChecksPassed"],
                "firstFailure": trial["firstFailure"], "firstGeneralizedFailure": trial["firstGeneralizedFailure"],
                "firstIntegrityFailure": trial["firstIntegrityFailure"]})).collect::<Vec<_>>());
        println!(
            "{}",
            json!({"scope": report["scope"], "passed": report["passed"],
            "nativeFirstStep": report["nativeFirstStep"], "passiveTrials": passive_summary, "releaseAccepted": false})
        );
        return Ok(report["passed"] == true);
    }
    if args.get(1).is_some_and(|a| a == "--diagnose-recovery") {
        let selected = args
            .iter()
            .position(|a| a == "--fixture")
            .map(|i| {
                args.get(i + 1)
                    .map(String::as_str)
                    .ok_or("Missing recovery fixture")
            })
            .transpose()?;
        let reports = lh_sim::recovery::fixture_trials(&profile, selected)?;
        let passed = !reports.is_empty() && reports.iter().all(|r| r["passed"] == true);
        fs::write(
            format!("{out}/recovery.json"),
            serde_json::to_vec_pretty(&reports)?,
        )?;
        println!(
            "{}",
            json!({"scope":"Landed-pose recovery diagnostics; no strike-cycle admission", "cases":reports.len(),"passed":passed,"releaseAccepted":false})
        );
        return Ok(passed);
    }
    if args.get(1).is_some_and(|a| a == "--diagnose-terrain") {
        let reports = lh_sim::environment::trials(&profile)?;
        let passed = reports.len() == 21 && reports.iter().all(|r| r["passed"] == true);
        fs::write(
            format!("{out}/terrain.json"),
            serde_json::to_vec_pretty(&reports)?,
        )?;
        println!(
            "{}",
            json!({"scope":"Two-second structural/contact checks across seven stations and three difficulties",
            "cases":reports.len(),"passed":passed,"releaseAccepted":false})
        );
        return Ok(passed);
    }
    if args.get(1).is_some_and(|a| a == "--diagnose-striker") {
        let reports = lh_sim::striker::trials(&profile)?;
        let passed = reports.len() == 3 && reports.iter().all(|r| r["passed"] == true);
        fs::write(
            format!("{out}/striker.json"),
            serde_json::to_vec_pretty(&reports)?,
        )?;
        println!(
            "{}",
            json!({"scope":"Three whole-body impacts with measured contact and physical descent; no recovery admission", "cases":reports.len(),"passed":passed,"releaseAccepted":false})
        );
        return Ok(passed);
    }
    if args
        .get(1)
        .is_some_and(|a| a == "--diagnose-contact-roundoff")
    {
        if args.iter().any(|a| a == "--profile") {
            return Err("Roundoff diagnostic uses fixed default settings; omit --profile".into());
        }
        let contact = lh_sim::calibration::tangent::matrix()?;
        let force_only = lh_sim::calibration::tangent::force_only_controls()?;
        fs::write(
            format!("{out}/contact-roundoff.json"),
            serde_json::to_vec_pretty(
                &json!({"scope":"Unchanged 216 contact fixtures plus 36 floor-disabled causal controls; diagnostic only", "contact":contact,"forceOnly":force_only,"releaseAccepted":false}),
            )?,
        )?;
        println!(
            "{}",
            json!({"scope":"Contact roundoff causal diagnostic only", "contactFailures":contact.iter().filter(|r| !r.passed).count(),"forceOnlyFailures":force_only.iter().filter(|r| !r.passed).count(),"releaseAccepted":false})
        );
        return Ok(false);
    }
    if args.get(1).is_some_and(|a| {
        matches!(
            a.as_str(),
            "--diagnose-tangent-contacts" | "--diagnose-bounded-tangent-contacts"
        )
    }) {
        if args.iter().any(|a| a == "--profile") {
            return Err("Tangent calibration uses fixed default settings; omit --profile".into());
        }
        let reports = if args[1] == "--diagnose-bounded-tangent-contacts" {
            lh_sim::calibration::tangent::bounded_matrix()?
        } else {
            lh_sim::calibration::tangent::matrix()?
        };
        let passed = reports.len() == 216 && reports.iter().all(|r| r.passed);
        fs::write(
            format!("{out}/tangent-calibration.json"),
            serde_json::to_vec_pretty(&reports)?,
        )?;
        println!(
            "{}",
            json!({"scope": "Isolated flat-box linear contact calibration only", "cases": reports.len(), "passed": passed, "releaseAccepted": false})
        );
        return Ok(passed);
    }
    if args.get(1).is_some_and(|a| {
        matches!(
            a.as_str(),
            "--diagnose-worker-quiet" | "--diagnose-worker-quiet-drift"
        )
    }) {
        let headings = if let Some(index) = args.iter().position(|a| a == "--heading") {
            vec![
                args.get(index + 1)
                    .ok_or("Missing heading")?
                    .parse::<f32>()?,
            ]
        } else {
            vec![0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0]
        };
        let selected_mode = args
            .iter()
            .position(|a| a == "--mode")
            .map_or("protocol", |index| args[index + 1].as_str());
        let modes = match selected_mode {
            "playground" => vec![lh_contracts::Mode::Playground],
            "all" => vec![lh_contracts::Mode::Protocol, lh_contracts::Mode::Playground],
            _ => vec![lh_contracts::Mode::Protocol],
        };
        let mut reports = Vec::new();
        for (mode, heading) in modes
            .into_iter()
            .flat_map(|mode| headings.iter().copied().map(move |heading| (mode, heading)))
        {
            let mut runtime = lh_sim::runtime::Runtime::new(mode, heading, profile.clone())?;
            runtime
                .enable_quiet_measurement()
                .map_err(|reject| format!("{reject:?}"))?;
            if args[1] == "--diagnose-worker-quiet-drift" {
                runtime
                    .enable_quiet_drift_trace()
                    .map_err(|reject| format!("{reject:?}"))?;
            }
            for _ in 0..480 {
                let reply = runtime.advance(4).map_err(|reject| format!("{reject:?}"))?;
                if reply.failure.is_some() {
                    break;
                }
            }
            reports.push(runtime.quiet_report().ok_or("Missing quiet report")?);
        }
        let passed = reports.iter().all(|r| r["progress"]["passed"] == true);
        fs::write(
            format!("{out}/worker-quiet.json"),
            serde_json::to_vec_pretty(&reports)?,
        )?;
        println!(
            "{}",
            json!({"scope": "Worker-runtime quiet standing only", "passed": passed, "releaseAccepted": false})
        );
        return Ok(passed);
    }
    if args.get(1).is_some_and(|a| a == "--diagnose-disturbances") {
        let selected = args
            .iter()
            .position(|a| a == "--fixture")
            .map(|index| {
                args.get(index + 1)
                    .map(String::as_str)
                    .ok_or("Missing fixture id")
            })
            .transpose()?;
        let reports = if args.iter().any(|a| a == "--require-steps") {
            lh_sim::scenarios::steps_with_profile(&profile, selected)?
        } else {
            lh_sim::scenarios::disturbances_with_profile(&profile, selected)?
        };
        fs::write(
            format!("{out}/disturbances.json"),
            serde_json::to_vec_pretty(&reports)?,
        )?;
        let passed = !reports.is_empty() && reports.iter().all(|r| r["passed"] == true);
        println!(
            "{}",
            serde_json::to_string(
                &json!({"scope":"Selected disturbance diagnostic only; no gate promotion", "out":out, "cases":reports.len(), "passed":passed, "failed": reports.iter().filter(|r| r["passed"] != true).count(), "releaseAccepted":false})
            )?
        );
        return Ok(passed);
    }
    if args.get(1).is_some_and(|a| a == "--diagnose-foot-load") {
        let enabled = lh_sim::calibration::individual_feet_with_ccd(true)?;
        let disabled = lh_sim::calibration::individual_feet_with_ccd(false)?;
        fs::write(
            format!("{out}/foot-load-causal-probe.json"),
            serde_json::to_vec_pretty(
                &json!({"scope":"CCD on/off diagnostic only; no acceptance scenario removed or configuration promoted", "ccdEnabled":enabled,"ccdDisabled":disabled}),
            )?,
        )?;
        return Ok(false);
    }
    let mut calibrations = Vec::new();
    for warm in [0.0, 0.5, 1.0] {
        for scenario in ["rest", "changing", "unloading", "contact-loss"] {
            calibrations.push(lh_sim::calibration::contact_load_with_profile(
                warm, scenario, &profile,
            )?);
        }
    }
    fs::write(
        format!("{out}/contact-calibration.json"),
        serde_json::to_vec_pretty(&calibrations)?,
    )?;
    let calibration_pass = calibrations.iter().all(|r| r.passed);
    let canonical_calibrations = if calibration_pass {
        lh_sim::calibration::canonical_contact_load_with_profile(&profile)?
    } else {
        Vec::new()
    };
    fs::write(
        format!("{out}/canonical-contact-calibration.json"),
        serde_json::to_vec_pretty(&canonical_calibrations)?,
    )?;
    let canonical_load_pass = calibration_pass
        && !canonical_calibrations.is_empty()
        && canonical_calibrations.iter().all(|r| r["passed"] == true);
    let joints = if canonical_load_pass {
        lh_sim::calibration::isolated_joints_with_profile(&profile)?
    } else {
        Vec::new()
    };
    fs::write(
        format!("{out}/isolated-joints.json"),
        serde_json::to_vec_pretty(&joints)?,
    )?;
    let isolated_pass =
        canonical_load_pass && !joints.is_empty() && joints.iter().all(|r| r.passed);
    let feet = if isolated_pass {
        lh_sim::calibration::individual_feet_with_profile(true, &profile)?
    } else {
        Vec::new()
    };
    fs::write(
        format!("{out}/individual-feet.json"),
        serde_json::to_vec_pretty(&feet)?,
    )?;
    let feet_pass = isolated_pass && !feet.is_empty() && feet.iter().all(|r| r.passed);
    // A primitive-box pass is insufficient to admit contact-based control.
    // Canonical loaded foot surfaces must also agree with momentum balance.
    let gate_a = feet_pass;
    let chains = if feet_pass {
        lh_sim::calibration::loaded_leg_chains_with_profile(&profile)?
    } else {
        Vec::new()
    };
    fs::write(
        format!("{out}/loaded-chains.json"),
        serde_json::to_vec_pretty(&chains)?,
    )?;
    let gate_b = feet_pass && !chains.is_empty() && chains.iter().all(|r| r["passed"] == true);
    let mut runs = Vec::new();
    if gate_b {
        for heading in [0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0] {
            let mut s = Simulation::new(heading, profile.clone(), true)?;
            let initial = s.snapshot();
            let mut first_failure = None;
            for _ in 0..120 {
                if let Err(f) = s.advance_tick() {
                    first_failure = Some(f);
                    break;
                }
            }
            let settled = s.snapshot();
            let mut metrics = lh_sim::measurement::QuietMetrics::default();
            if first_failure.is_none() {
                for _ in 0..1800 {
                    if let Err(f) =
                        s.advance_tick_observed(|s, substep| metrics.observe(s, &settled, substep))
                    {
                        first_failure = Some(f);
                        break;
                    }
                }
            }
            let final_snapshot = s.snapshot();
            let passed = first_failure.is_none() && s.tick == 1920;
            runs.push(json!({"heading":heading,"passed":passed,"completedTicks":s.tick,"initial":initial,"settled":settled,"final":final_snapshot,"firstFailure":first_failure,
                "measurements":{"massKg":s.total_mass(),"quiet":metrics}}));
        }
    }
    let passed = gate_b && !runs.is_empty() && runs.iter().all(|r| r["passed"] == true);
    let disturbances = if passed {
        lh_sim::scenarios::disturbances_with_profile(&profile, None)?
    } else {
        Vec::new()
    };
    fs::write(
        format!("{out}/disturbances.json"),
        serde_json::to_vec_pretty(&disturbances)?,
    )?;
    let disturbed_failure = disturbances.iter().any(|r| r["passed"] != true);
    let report = json!({"schema":2,"status":if passed&&!disturbed_failure {"partial-gates-pass"} else {"fail"},"releaseAccepted":false,
        "engine":"Rapier 0.35.0 + experimental contact reporting and angular-coordinate patches", "profile":profile,"primitiveContactCalibrationPassed":calibration_pass,
        "canonicalContactCalibrationPassed":canonical_load_pass,
        "isolatedJointsPassed":isolated_pass,"contactMeasurementAdmitted":feet_pass,"standingDiagnostic":runs,
        "gateA":if gate_a {"pass"} else {"fail"}, "gateB":if gate_b {"pass"} else if gate_a {"fail"} else {"incomplete"},
        "gateC":if passed {"pass"} else if gate_b {"fail"} else {"incomplete"},
        "gateD":if disturbed_failure {"fail"}else{"incomplete"},"gatesEThroughI":"incomplete",
        "scope":"Gate A calibration and isolated joints; Gate B individual feet and loaded chains; Gate C quiet standing; Gate D existing recoverable pull probes. Every physical check observes 240 Hz integrations. Gate D remains incomplete even if selected probes pass; later scenarios, browser physics and release acceptance are not established."});
    fs::write(
        format!("{out}/report.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    println!(
        "{}",
        serde_json::to_string(&json!({"out":out,"passed":passed,"releaseAccepted":false}))?
    );
    Ok(passed && !disturbed_failure)
}
fn main() {
    match run() {
        Ok(true) => {}
        Ok(false) => std::process::exit(1),
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    }
}
