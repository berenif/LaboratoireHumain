import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { copyFileSync, existsSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { dirname, join, resolve, sep } from 'node:path';
import { prepareRustBuild, rustCheckCommands, rustEnvironment, rustToolchain } from './rust-toolchain.mjs';
import { sourceHashes, fingerprint, releaseDecision, requiredReleaseStages, nativeTestCount, persistReceipt } from './rust-release-tools.mjs';

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
  return sourceHashes();
}
const before=sources();
for(const path of Object.keys(before)) {const target=join(out,'source',path);mkdirSync(dirname(target),{recursive:true});copyFileSync(path,target);}
const env=rustEnvironment();
const records=[];
const nativeChecks = [
  { name: 'native', args: [], report: 'report.json' },
  { name: 'worker-quiet', args: ['--diagnose-worker-quiet', '--mode', 'all'], report: 'worker-quiet.json' },
  { name: 'striker', args: ['--diagnose-striker'], report: 'striker.json' },
];
const releaseChecks = [
  { name:'actuators', args:['--diagnose-actuators'], report:'actuators.json' },
  { name:'contacts', args:['--diagnose-contact-roundoff'], report:'contact-roundoff.json' },
  { name:'disturbances', args:['--diagnose-disturbances'], report:'disturbances.json', candidate:true },
  { name:'steps', args:['--diagnose-disturbances','--require-steps'], report:'disturbances.json', candidate:true },
  { name:'legacy', args:['--diagnose-legacy'], report:'legacy.json', candidate:true },
  { name:'recovery', args:['--diagnose-recovery'], report:'recovery.json', candidate:true },
  { name:'cycles', args:['--diagnose-cycles'], report:'cycles.json', candidate:true },
  { name:'terrain', args:['--diagnose-terrain'], report:'terrain.json', candidate:true },
];
const checks = foundationOnly ? nativeChecks : [...nativeChecks, ...releaseChecks];
const stages = [];
const manifest={schema:1,started:new Date().toISOString(),command:[process.execPath,...process.argv.slice(1)],node:process.version,
  requestedScope:foundationOnly?'implemented-native-foundation':'release',
  sourceBefore:before,sourceFingerprint:fingerprint(before),runs:records,stages,complete:false,releaseAccepted:false};
const save=()=>persistReceipt(join(out,'execution.json'),manifest);
function run(name,args,timeout=300000,executable='cargo',minimumTests=0) {
  if (executable === 'cargo' && ['build','test','clippy'].includes(args[1])) {
    (manifest.buildFreshness ??= []).push({name, ...prepareRustBuild()});
    save();
  }
  const started=performance.now();
  const r=spawnSync(executable,args,{env,encoding:'utf8',windowsHide:true,timeout,maxBuffer:16*1024*1024});
  writeFileSync(join(out,`${name}.log`),(r.stdout??'')+(r.stderr??''));
  const testCount = minimumTests ? nativeTestCount(r.stdout??'') : null;
  records.push({name,command:[executable,...args],exit:r.status,signal:r.signal,error:r.error?.message??null,
    testCount,minimumTests,wallMs:performance.now()-started});save();
  console.log(JSON.stringify(records.at(-1)));
  return r.status===0&&!r.signal&&!r.error&&(!minimumTests||testCount>=minimumTests);
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
  const tests=run('tests',[tc,'test','--offline','--locked','--release',...workspace,'--workspace'],300000,'cargo',76);
  const build=run('native-build',[tc,'build','--offline','--locked','--release',...workspace,'-p','lh-acceptance']);
  const wasm=run('wasm-core-build',[compiler,'build','--offline','--locked','--release',...workspace,'--target','wasm32-unknown-unknown','-p','lh-sim']);
  const jacobian=run('engine-jacobian',[tc,'test','--offline','--locked','--release','--manifest-path','rust/vendor/rapier3d/Cargo.toml','--lib','coordinate_jacobian','--no-default-features','--features','dim3,f32,std'],300000,'cargo',1);
  const cached=run('engine-cached-equivalence',[tc,'test','--offline','--locked','--release','--manifest-path','rust/vendor/rapier3d/Cargo.toml','--lib','velocity_cache_tests','--no-default-features','--features','dim3,f32,std,experimental-joint-velocity-cache'],300000,'cargo',3);
  const tooling=run('tooling',['--test','tests/rust-tooling.test.mjs','tests/rust-release.test.mjs','tests/rust-build-freshness.integration.mjs'],300000,process.execPath);
  stages.push(...[['format',fmt],['native-clippy',lint],['wasm-clippy',wasmLint],['native-tests',tests],['tooling',tooling],['scalar-cached-equivalence',cached]].map(([name,passed])=>({name,passed})));
  if(build) {
    const executable=resolve(`rust/target/release/lh-acceptance${process.platform==='win32'?'.exe':''}`);
    manifest.executableSha256=hash(readFileSync(executable));
    manifest.resultFiles={};
    const repeats=[];
    const profilePath=join(out,'candidate-profile.json');
    for (const check of checks) {
      if(check.candidate) {
        const profile=JSON.parse(readFileSync(join(out,'native-1','profile.json')));
        profile.id += '-release-fixtures'; profile.corrective_steps_enabled=true; profile.recovery_enabled=true;
        writeFileSync(profilePath,JSON.stringify(profile,null,2)+'\n');
      }
      for(const index of [1,2]) {
        const name=`${check.name}-${index}`,destination=join(out,name),started=performance.now();
        const args=[destination,...check.args,...(check.candidate?['--profile',profilePath]:[])];
        const r=spawnSync(executable,args,{env,encoding:'utf8',windowsHide:true,timeout:1800000,maxBuffer:16*1024*1024});
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
    const success=name=>[1,2].every(index=>records.some(r=>r.name===`${name}-${index}`&&r.exit===0&&!r.error&&!r.signal));
    const read=(name,file,fallback=null)=>{
      try {return JSON.parse(readFileSync(join(out,name+'-1',file)));}
      catch(error) {
        (manifest.coverageErrors??=[]).push({name,file,error:String(error)});save();
        return fallback;
      }
    };
    stages.push({name:'native-repeat',passed:manifest.repeatable});
    stages.push({name:'anatomy-structure-collision-ownership',passed:success('native')&&jacobian&&success('worker-quiet')&&success('striker')});
    if(!foundationOnly) {
      const contact=read('contacts','contact-roundoff.json',{});
      stages.push({name:'contact-stress',passed:success('contacts')&&contact.contact?.length===216&&contact.contact.every(r=>r.passed)});
      stages.push({name:'force-controls',passed:success('contacts')&&contact.forceOnly?.length===36&&contact.forceOnly.every(r=>r.passed)});
      for(const [name,stage,count] of [['disturbances','disturbances',14],['steps','strict-steps',9],['legacy','legacy-four',4],['recovery','recovery',24],['terrain','terrain-21',21]]) {
        const check=releaseChecks.find(c=>c.name===name),path=join(out,name+'-1',check.report);
        const parsed=read(name,check.report), rows=Array.isArray(parsed)?parsed:[];
        stages.push({name:stage,passed:success(name)&&rows.length===count&&rows.every(r=>r.passed),expectedCases:count,actualCases:rows.length,passedCases:rows.filter(r=>r.passed).length});
      }
      stages.push({name:'cycles-five',passed:success('cycles')&&read('cycles','cycles.json')?.completedCycles===5});
      const production=read('native','profile.json',{});
      stages.push({name:'production-profile',passed:production.corrective_steps_enabled===true&&production.recovery_enabled===true});
      const actuatorParsed = read('actuators','actuators.json'), actuatorRows = Array.isArray(actuatorParsed)?actuatorParsed:[];
      stages.push({name:'actuators',passed:success('actuators')&&actuatorRows.length===432&&actuatorRows.every(r=>r.passed),
        expectedCases:432,actualCases:actuatorRows.length,passedCases:actuatorRows.filter(r=>r.passed).length});
      const appBuild=run('retained-build',['scripts/run-tool.mjs','vinext','build'],600000,process.execPath);
      const appTests=run('retained-tests',['--test',...readdirSync('tests').filter(file=>file.endsWith('.test.mjs')).map(file=>join('tests',file))],1800000,process.execPath);
      const typecheck=run('retained-typecheck',['node_modules/typescript/bin/tsc','--noEmit','--pretty','false'],300000,process.execPath);
      const application=appBuild&&appTests&&typecheck;
      stages.push({name:'retained-application',passed:application});
      const physical=stages.filter(stage=>['contact-stress','force-controls','anatomy-structure-collision-ownership','actuators','disturbances','strict-steps','legacy-four','recovery','cycles-five','terrain-21'].includes(stage.name)).every(stage=>stage.passed);
      if(physical) {
        const browser=run('candidate-browser',['scripts/qualify-rust-candidate.mjs',join(out,'candidate'),profilePath],3600000,process.execPath);
        const path=join(out,'candidate','qualification.json');
        if(existsSync(path)) {
          const qualification=JSON.parse(readFileSync(path));
          stages.push(...qualification.stages);
          manifest.candidateId=qualification.candidateId;manifest.assetsFingerprint=qualification.assetsFingerprint;
          manifest.candidateDirectory=join(out,'candidate');
          manifest.assetsUnchanged=qualification.assetsUnchanged;
          manifest.browserEvidence=qualification;
        }
        manifest.browserProcessPassed=browser;
      } else {
        manifest.browserBlockedBy=stages.filter(stage=>!stage.passed).map(stage=>stage.name);
      }
    }
  }
  manifest.completedRequestedChecks=fmt&&lint&&wasmLint&&tests&&build&&wasm&&jacobian
    &&checks.every(check=>[1,2].every(index=>records.some(r=>r.name===`${check.name}-${index}`&&r.exit===0&&!r.error&&!r.signal)));
  manifest.requiredReleaseStages=requiredReleaseStages;
} finally {
  manifest.finished=new Date().toISOString();manifest.sourceAfter=sources();manifest.sourceUnchanged=JSON.stringify(manifest.sourceAfter)===JSON.stringify(before);
  stages.push({name:'source-unchanged',passed:manifest.sourceUnchanged});
  Object.assign(manifest,releaseDecision(stages));
  manifest.implementedChecksPassed=manifest.completedRequestedChecks===true&&manifest.repeatable===true&&manifest.sourceUnchanged;
  save();
}
console.log(JSON.stringify({out,requestedScope:manifest.requestedScope,implementedChecksPassed:manifest.implementedChecksPassed,complete:manifest.complete,repeatable:manifest.repeatable,sourceUnchanged:manifest.sourceUnchanged,releaseAccepted:manifest.releaseAccepted,missing:manifest.missing,failed:manifest.failed}));
if(manifest.releaseAccepted) {
  if(!run('accepted-bundle',['scripts/export-rust-candidate.mjs',out,join(out,'accepted-rust-pages')],300000,process.execPath)) {
    manifest.releaseAccepted=false;manifest.exportError='Accepted artifact export failed';save();
  }
}
process.exitCode=(foundationOnly?manifest.implementedChecksPassed:manifest.releaseAccepted)?0:1;
