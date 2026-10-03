import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { copyFileSync, existsSync, lstatSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { dirname, join, resolve, sep } from 'node:path';
import { rustCheckCommands, rustEnvironment, rustToolchain } from './rust-toolchain.mjs';

const args = process.argv.slice(2);
const foundationOnly = args.includes('--foundation');
const positional = args.filter(arg => !arg.startsWith('--'));
assert.ok(args.every(arg => !arg.startsWith('--') || arg === '--foundation') && positional.length <= 1,
  'Usage: node scripts/run-rust-gates.mjs [--foundation] [fresh-output-directory]');
const out = resolve(positional[0] ?? `evidence/rust-verification-${new Date().toISOString().replaceAll(/[:.]/g, '-')}`);
assert.ok(!existsSync(out) && out.startsWith(resolve('.') + sep), 'Use a fresh workspace output directory');
mkdirSync(out, {recursive:true});
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
function sources() {
  const entries = {};
  function visit(directory) {
    for (const e of readdirSync(directory,{withFileTypes:true}).sort((a,b)=>a.name.localeCompare(b.name))) {
      if (['target','dist','.git'].includes(e.name)) continue;
      const path=join(directory,e.name);assert.ok(!lstatSync(path).isSymbolicLink(), `Refuse linked source ${path}`);
      if(e.isDirectory()) visit(path);else entries[path.replaceAll('\\','/')]=hash(readFileSync(path));
    }
  }
  visit('rust');
  for(const path of ['scripts/run-rust-gates.mjs','scripts/rust-toolchain.mjs','docs/rust-rework-performance-v1.json']) entries[path]=hash(readFileSync(path));
  return entries;
}
const before=sources();
for(const path of Object.keys(before)) {const target=join(out,'source',path);mkdirSync(dirname(target),{recursive:true});copyFileSync(path,target);}
const env=rustEnvironment();
const records=[];
const nativeChecks = [
  { name: 'native', args: [], report: 'report.json' },
  { name: 'worker-quiet', args: ['--diagnose-worker-quiet', '--mode', 'all'], report: 'worker-quiet.json' },
  { name: 'terrain', args: ['--diagnose-terrain'], report: 'terrain.json' },
  { name: 'striker', args: ['--diagnose-striker'], report: 'striker.json' },
];
const manifest={schema:1,started:new Date().toISOString(),command:[process.execPath,...process.argv.slice(1)],node:process.version,
  requestedScope:foundationOnly?'implemented-native-foundation':'release',
  sourceBefore:before,sourceFingerprint:hash(Buffer.from(JSON.stringify(before))),runs:records,complete:false,releaseAccepted:false};
const save=()=>writeFileSync(join(out,'execution.json'),JSON.stringify(manifest,null,2)+'\n');
function run(name,args,timeout=300000) {
  const started=performance.now();
  const r=spawnSync('cargo',args,{env,encoding:'utf8',windowsHide:true,timeout,maxBuffer:16*1024*1024});
  writeFileSync(join(out,`${name}.log`),(r.stdout??'')+(r.stderr??''));
  records.push({name,command:['cargo',...args],exit:r.status,signal:r.signal,error:r.error?.message??null,wallMs:performance.now()-started});save();
  console.log(JSON.stringify(records.at(-1)));return r.status===0;
}
try {
  save();
  const tc=`+${rustToolchain}`,compiler=tc,workspace=['--manifest-path','rust/Cargo.toml'];
  const version=spawnSync('rustc',[compiler,'--version','--verbose'],{encoding:'utf8',windowsHide:true});
  assert.match(version.stdout,/rustc 1\.89\.0/,'Installed formatting/lint/WASM compiler must match the pinned version');
  manifest.toolchainVersion=version.stdout;manifest.nativeToolchain=tc;manifest.auxiliaryToolchainAlias=compiler;
  const [fmtCommand, lintCommand, wasmLintCommand]=rustCheckCommands('lint');
  const fmt=run('fmt',fmtCommand);
  const lint=run('clippy',lintCommand);
  const wasmLint=run('wasm-clippy',wasmLintCommand);
  const tests=run('tests',[tc,'test','--offline','--locked','--release',...workspace,'--workspace']);
  const build=run('native-build',[tc,'build','--offline','--locked','--release',...workspace,'-p','lh-acceptance']);
  const wasm=run('wasm-core-build',[compiler,'build','--offline','--locked','--release',...workspace,'--target','wasm32-unknown-unknown','-p','lh-sim']);
  const jacobian=run('engine-jacobian',[tc,'test','--offline','--locked','--release','--manifest-path','rust/vendor/rapier3d/Cargo.toml','--lib','coordinate_jacobian','--no-default-features','--features','dim3,f32,std']);
  if(build&&tests&&jacobian) {
    const executable=resolve(`rust/target/release/lh-acceptance${process.platform==='win32'?'.exe':''}`);
    manifest.executableSha256=hash(readFileSync(executable));
    manifest.resultFiles={};
    const repeats=[];
    for (const check of nativeChecks) {
      for(const index of [1,2]) {
        const name=`${check.name}-${index}`,destination=join(out,name),started=performance.now();
        const args=[destination,...check.args];
        const r=spawnSync(executable,args,{env,encoding:'utf8',windowsHide:true,timeout:300000,maxBuffer:16*1024*1024});
        writeFileSync(join(out,`${name}.log`),(r.stdout??'')+(r.stderr??''));
        records.push({name,command:[executable,...args],exit:r.status,signal:r.signal,error:r.error?.message??null,wallMs:performance.now()-started});save();
        console.log(JSON.stringify(records.at(-1)));
        // Repeat even a failed gate to establish within-build reproducibility.
      }
      const a=join(out,`${check.name}-1`),b=join(out,`${check.name}-2`);
      if(existsSync(join(a,check.report))&&existsSync(join(b,check.report))) {
        const left=readdirSync(a).sort(),right=readdirSync(b).sort();
        manifest.resultFiles[check.name]=left;
        repeats.push(JSON.stringify(left)===JSON.stringify(right)
          &&left.every(file=>readFileSync(join(a,file)).equals(readFileSync(join(b,file)))));
      } else repeats.push(false);
    }
    manifest.repeatable=repeats.every(Boolean);
  }
  manifest.completedRequestedChecks=fmt&&lint&&wasmLint&&tests&&build&&wasm&&jacobian
    &&nativeChecks.every(check=>[1,2].every(index=>records.some(r=>r.name===`${check.name}-${index}`&&r.exit===0&&!r.error&&!r.signal)));
  manifest.complete=false;
  // Publication requires the complete physical/browser/performance matrix.
  manifest.requiredReleaseStages=['A','B','C','D','E','F','G','H','I','native-repeat','browser-physics','public-modes','worker-lifecycle','input','WebGPU','WebGL2','graphics-recovery','root','subpath','performance','soak','Pages-artifact','final-artifact'];
  manifest.releaseAccepted=false;
} finally {
  manifest.finished=new Date().toISOString();manifest.sourceAfter=sources();manifest.sourceUnchanged=JSON.stringify(manifest.sourceAfter)===JSON.stringify(before);
  manifest.implementedChecksPassed=manifest.completedRequestedChecks===true&&manifest.repeatable===true&&manifest.sourceUnchanged;
  save();
}
console.log(JSON.stringify({out,requestedScope:manifest.requestedScope,implementedChecksPassed:manifest.implementedChecksPassed,complete:manifest.complete,repeatable:manifest.repeatable,sourceUnchanged:manifest.sourceUnchanged,releaseAccepted:false}));
process.exitCode=(foundationOnly?manifest.implementedChecksPassed:manifest.releaseAccepted)?0:1;
