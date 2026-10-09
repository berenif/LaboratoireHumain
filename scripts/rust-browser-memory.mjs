import { spawn } from 'node:child_process';
import { createInterface } from 'node:readline';
import { resolve } from 'node:path';

export async function memorySampler() {
  if (process.platform !== 'win32') return { hardware:null, sample:async()=>({rssMiB:null}), close(){} };
  const child = spawn('powershell.exe', ['-NoProfile', '-File', resolve('scripts/rust-browser-memory.ps1')], { windowsHide:true, stdio:['pipe','pipe','pipe'] });
  const lines = createInterface({ input:child.stdout });
  const pending = [];
  let hardwareResolve;
  const hardware = new Promise(resolve => { hardwareResolve=resolve; });
  lines.on('line', line => {
    try { const value=JSON.parse(line); if(value.type==='hardware') hardwareResolve(value); else pending.shift()?.(value); }
    catch { pending.shift()?.({rssMiB:null,error:'Invalid memory response'}); }
  });
  child.on('error', error=>{hardwareResolve(null);pending.splice(0).forEach(resolve=>resolve({rssMiB:null,error:String(error)}));});
  child.on('exit', ()=>{hardwareResolve(null);pending.splice(0).forEach(resolve=>resolve({rssMiB:null,error:'Memory sampler exited'}));});
  const watchdog=setTimeout(()=>hardwareResolve(null),15000);
  const identity=await hardware;clearTimeout(watchdog);
  return { hardware:identity, async sample(ids) {
    if (!identity || child.exitCode!==null) return {rssMiB:null};
    return await new Promise(resolve=>{
      const timer=setTimeout(()=>resolve({rssMiB:null,error:'Memory sample timed out'}),10000);
      pending.push(value=>{clearTimeout(timer);resolve(value);});
      child.stdin.write(JSON.stringify(ids)+'\n');
    });
  }, close(){child.stdin.end();lines.close();child.kill();} };
}
