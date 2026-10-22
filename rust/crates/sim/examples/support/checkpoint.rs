use lh_sim::runtime::Runtime;
use serde_json::{Value,json};
use std::{fs::OpenOptions,io::Write,path::Path};

pub fn save_checkpoint(runtime:&Runtime,path:&Path)->Result<Value,String> {
    let bytes=runtime.getup_checkpoint()?;
    // Two 7 MiB checkpoints + 1 MiB result stay below 16 MiB total.
    if bytes.len()>7*1024*1024 {return Err("checkpoint exceeds 7 MiB reserve".into());}
    let mut file=OpenOptions::new().write(true).create_new(true).open(path).map_err(|e|format!("checkpoint {}: {e}",path.display()))?;
    file.write_all(&bytes).and_then(|_|file.sync_all()).map_err(|e|format!("checkpoint write {}: {e}",path.display()))?;
    let verification=runtime.verify_getup_checkpoint(&bytes);
    Ok(json!({"path":path,"bytes":bytes.len(),"restored":verification.is_ok(),
        "verification":verification.as_ref().ok().and_then(|s|serde_json::from_str::<Value>(s).ok()),
        "restoreError":verification.as_ref().err(),"scope":"exact serialized state; next-step continuity not tested"}))
}
