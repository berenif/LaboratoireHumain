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

/// One compact stop state. Compression is the installed system gzip, not a
/// project dependency; no intermediate uncompressed file or input copy.
pub fn save_compressed_checkpoint(runtime:&Runtime,path:&Path)->Result<Value,String> {
    fn gzip(args:&[&str],bytes:Vec<u8>)->Result<Vec<u8>,String> {
        use std::process::{Command,Stdio};
        let mut child=Command::new("gzip").args(args).stdin(Stdio::piped()).stdout(Stdio::piped()).stderr(Stdio::piped()).spawn().map_err(|e|e.to_string())?;
        let mut input=child.stdin.take().ok_or("gzip stdin unavailable")?;
        let writer=std::thread::spawn(move || input.write_all(&bytes));
        let output=child.wait_with_output().map_err(|e|e.to_string())?;
        writer.join().map_err(|_|"gzip writer panicked")?.map_err(|e|e.to_string())?;
        if !output.status.success() {return Err(format!("gzip failed: {}",String::from_utf8_lossy(&output.stderr)));}
        Ok(output.stdout)
    }
    let bytes=runtime.getup_checkpoint()?;let raw_size=bytes.len();
    let compressed=gzip(&["-n","-6","-c"],bytes.clone())?;
    if compressed.len()>1_300_000 {return Err(format!("compressed checkpoint {} exceeds 1300000-byte reserved proof space",compressed.len()));}
    let decoded=gzip(&["-d","-c"],compressed.clone())?;
    if decoded!=bytes {return Err("compressed checkpoint roundtrip mismatch".into());}
    let verification:Value=serde_json::from_str(&runtime.verify_getup_checkpoint(&decoded)?).map_err(|e|e.to_string())?;
    if verification["passed"]!=true {return Err("compressed checkpoint restoration failed".into());}
    let mut file=OpenOptions::new().write(true).create_new(true).open(path).map_err(|e|e.to_string())?;
    file.write_all(&compressed).and_then(|_|file.sync_all()).map_err(|e|e.to_string())?;
    Ok(json!({"path":path,"bytes":compressed.len(),"uncompressedBytes":raw_size,"compression":"gzip -n -6","restored":true,"verification":verification,
        "scope":"exact compressed serialized stop; next-step continuity not tested"}))
}
