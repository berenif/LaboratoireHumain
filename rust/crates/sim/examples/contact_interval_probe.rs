//! Read-only replay of the existing foot momentum fixture, at CCD boundaries.
fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().collect();
    let profile = if let Some(path) = args.get(1) {
        serde_json::from_str(&std::fs::read_to_string(path).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?
    } else {
        lh_sim::Profile::default()
    };
    profile.validate()?;
    let foot = args.get(2).map(String::as_str).unwrap_or("left_foot");
    let warmstart = args
        .get(3)
        .map(|x| x.parse::<f32>())
        .transpose()
        .map_err(|e| e.to_string())?
        .unwrap_or(0.0);
    let scenario = args.get(4).map(String::as_str).unwrap_or("rest");
    println!(
        "{}",
        lh_sim::calibration::contact_interval_probe::replay(&profile, foot, warmstart, scenario)?
    );
    Ok(())
}
