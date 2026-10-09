import assert from 'node:assert/strict';
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';
import { sha256, treeHashes, validateCandidate } from './rust-release-tools.mjs';

const [outArg,url,bundleArg]=process.argv.slice(2);
assert.ok(outArg&&url&&bundleArg,'Supply fresh output, published URL and accepted bundle');
const out=resolve(outArg),bundle=resolve(bundleArg);assert.ok(!existsSync(out));
const candidate=JSON.parse(readFileSync(join(bundle,'candidate.json'))),receipt=JSON.parse(readFileSync(join(bundle,'receipt.json')));
validateCandidate(candidate,receipt,treeHashes(join(bundle,'site')));
mkdirSync(out,{recursive:true});
const report={candidateId:candidate.candidateId,assets:{},cases:[],passed:false};
let browser;
try {
  const base=new URL(url);assert.equal(base.pathname,candidate.mount);
  for(const [name,digest] of Object.entries(candidate.assets)) {
    const asset=new URL(name,base);asset.searchParams.set('qualified',candidate.candidateId);
    const response=await fetch(asset,{cache:'no-store',signal:AbortSignal.timeout(30000)});
    assert.ok(response.ok,`Published asset missing: ${name}`);
    report.assets[name]=sha256(Buffer.from(await response.arrayBuffer()));
    assert.equal(report.assets[name],digest,`Published asset mismatch: ${name}`);
  }
  browser=await chromium.launch(browserLaunchOptions(true));
  for(const mode of ['protocol','playground'])for(const backend of ['webgl2','webgpu']) {
    const page=await browser.newPage(),errors=[];page.on('pageerror',error=>errors.push(String(error)));
    const location=new URL(base);location.searchParams.set('mode',mode);location.searchParams.set('renderer',backend);location.searchParams.set('qualified',candidate.candidateId);
    await page.goto(location.href);await page.waitForFunction(()=>window.__lhRust?.diagnostics.ready,{},{timeout:45000});
    const first=await page.evaluate(()=>window.__lhRust.diagnostics);
    await page.waitForFunction(tick=>window.__lhRust.diagnostics.tick>tick,first.tick,{timeout:15000});
    const final=await page.evaluate(()=>window.__lhRust.diagnostics);
    assert.equal(final.mode,mode);assert.equal(final.backend.toLowerCase(),backend);
    assert.equal(final.failure,null);assert.equal(final.graphics.renderedSegments,25);assert.equal(final.workerInstances,1);assert.deepEqual(errors,[]);
    report.cases.push({mode,backend,passed:true,workerStarted:true,firstTick:first.tick,finalTick:final.tick});
    await page.screenshot({path:join(out,mode+'-'+backend+'.png')});await page.close();
  }
  report.passed=report.cases.length===4;
}catch(error){report.error=String(error);}
finally{await browser?.close();writeFileSync(join(out,'report.json'),JSON.stringify(report,null,2)+'\n');}
console.log(JSON.stringify({passed:report.passed,candidateId:report.candidateId,error:report.error}));
process.exitCode=report.passed?0:1;
