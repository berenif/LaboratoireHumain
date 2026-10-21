// Full preserved UI + separately identified Recovery demo. No simulation/test run.
import {spawnSync} from 'node:child_process';
import {readFileSync,writeFileSync,copyFileSync,mkdirSync,existsSync,readdirSync,statSync} from 'node:fs';
import {resolve,join} from 'node:path';
import {createHash} from 'node:crypto';
import {browserEnvironment,checkBrowserTool,workspaceOutput} from './rust-browser-tools.mjs';
const recovery=resolve(process.argv[2]??''),out=workspaceOutput(process.argv[3]??'rust/dist/restored-app');
if(!process.argv[2]||existsSync(out))throw Error('Provide verified recovery assets and a new output directory');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const pins={'lh_worker_bg.wasm':'f9b06192c1d9b265dd179e80d8aa6c4ce72ad5cc0fa2dd2bddb9ee8fdb87fa3a','lh_worker.js':'402d5904187bf71d511dc70cad171d87502598b31695fd67aaea5024b333de8a','getup-800hz-3plus1-repeat-cycle-v1.json':'aaa415a19767c218a26b05bad277193cccba2a726b52dcd5aac0a5444afc39d6'};
for(const [p,h]of Object.entries(pins))if(hash(join(recovery,p))!==h)throw Error('Recovery artifact mismatch: '+p);
const env=browserEnvironment(),trunk=checkBrowserTool('trunk',undefined,env);checkBrowserTool('wasm-bindgen',undefined,env);
function run(exe,args,cwd=process.cwd()){const r=spawnSync(exe,args,{env,cwd,stdio:'inherit'});if(r.status!==0)throw Error('Build failed: '+exe);}
// This default build is intentional and isolated. It never replaces the pinned recovery artifacts.
run(process.execPath,['scripts/build-rust-worker.mjs','rust/dist/worker']);
run(trunk,['build','--config',resolve('rust/Trunk.toml'),'--release','--offline','true','--locked','true','--skip-version-check','--dist',out,'--public-url','/'],resolve('rust'));
mkdirSync(join(out,'recovery'));mkdirSync(join(out,'minimal'));
for(const p of ['lh_worker.js','lh_worker_bg.wasm'])copyFileSync(join(recovery,p),join(out,'recovery',p));
copyFileSync(join(recovery,'getup-800hz-3plus1-repeat-cycle-v1.json'),join(out,'recovery/profile.json'));
copyFileSync('rust/browser/minimal/capabilities.json',join(out,'recovery/capabilities.json'));
copyFileSync('rust/browser/sim-worker.js',join(out,'recovery/sim-worker.js'));
for(const p of ['index.html','app.js','worker.js'])copyFileSync('rust/browser/minimal/'+p,join(out,'minimal',p));
copyFileSync('rust/crates/model/data/getup-v1.json',join(out,'minimal/model.json'));
const files={};function visit(dir){for(const p of readdirSync(dir)){const f=join(dir,p);if(statSync(f).isDirectory())visit(f);else files[f.slice(out.length+1)]={bytes:statSync(f).size,sha256:hash(f)};}}visit(out);
const manifest={schema:'lh-restored-browser/v1',sourceCommit:spawnSync('git',['rev-parse','HEAD'],{encoding:'utf8'}).stdout.trim(),defaultWorker:files['lh_worker_bg.wasm'],recoveryWorker:files['recovery/lh_worker_bg.wasm'],modelSha256:hash('rust/crates/model/data/getup-v1.json'),files,physicsIntegrations:0,browserValidated:false};
writeFileSync(join(out,'artifact-manifest.json'),JSON.stringify(manifest,null,2)+'\n');console.log(JSON.stringify({output:out,bytes:Object.values(files).reduce((n,f)=>n+f.bytes,0),files:Object.keys(files).length}));
