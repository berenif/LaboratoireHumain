//! Inspect an exact saved world without advancing it or changing its caches.
use lh_sim::runtime::Runtime;
use std::{fs, io::Write, path::Path};
fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 3, "getup_roll_inspect CHECKPOINT OUTPUT_JSON");
    let bytes = fs::read(&args[1]).unwrap();
    let runtime = Runtime::from_getup_checkpoint(&bytes).unwrap();
    let before = runtime.getup_checkpoint().unwrap();
    let report = runtime.getup_roll_checkpoint_diagnostic().unwrap();
    let after = runtime.getup_checkpoint().unwrap();
    assert_eq!(before, after, "Inspection must not change the complete runtime/checkpoint");
    let path = Path::new(&args[2]); fs::create_dir_all(path.parent().unwrap()).unwrap();
    let mut file = fs::OpenOptions::new().write(true).create_new(true).open(path).unwrap();
    serde_json::to_writer_pretty(&mut file, &report).unwrap(); writeln!(file).unwrap(); file.sync_all().unwrap();
    println!("native={} integrations=0 exact-runtime-checkpoint-unchanged=true", report["nativeSubsteps"]);
}
