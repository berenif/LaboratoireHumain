//! Restores the declared mixed f32/f64 Snapshot wire types, without tolerances.
use serde_json::Value;
use std::{fs, path::Path};
fn main() {
    let a: Vec<_> = std::env::args().collect();
    assert_eq!(a.len(), 3);
    let mut v: Value = serde_json::from_slice(&fs::read(&a[1]).unwrap()).unwrap();
    let snapshot: lh_contracts::Snapshot = serde_json::from_value(v["snapshot"].take()).unwrap();
    v["snapshot"] = serde_json::to_value(snapshot).unwrap();
    fs::write(Path::new(&a[2]), serde_json::to_vec_pretty(&v).unwrap()).unwrap();
}
