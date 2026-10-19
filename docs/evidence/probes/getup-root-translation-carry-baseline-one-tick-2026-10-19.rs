use lh_sim::runtime::Runtime;
use std::{
    fs::{self, OpenOptions},
    path::Path,
};
fn main() {
    let a: Vec<_> = std::env::args().collect();
    assert_eq!(a.len(), 3);
    let out = Path::new(&a[2]);
    fs::create_dir_all(out).unwrap();
    OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(out.join("started-once"))
        .unwrap();
    let input = fs::read(&a[1]).unwrap();
    let mut r = Runtime::from_getup_checkpoint(&input).unwrap();
    r.advance(0).unwrap();
    let reply = r.advance(1).unwrap();
    assert!(reply.failure.is_none());
    assert_eq!(reply.completed_ticks, 1);
    fs::write(
        out.join("reply.json"),
        serde_json::to_vec_pretty(&reply).unwrap(),
    )
    .unwrap();
    fs::write(out.join("checkpoint.bin"), r.getup_checkpoint().unwrap()).unwrap();
    fs::write(out.join("controller.json"), r.getup_report().unwrap()).unwrap();
}
