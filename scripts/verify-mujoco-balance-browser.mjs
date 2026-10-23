import assert from 'node:assert/strict';
import {readFile,writeFile,mkdir} from 'node:fs/promises';
import {spawn} from 'node:child_process';
const {chromium}=await import('playwright').catch(()=>import('/opt/codex/cua_node/lib/node_modules/playwright/index.mjs'));
const out='web/proofs/balance-browser';await mkdir(out,{recursive:true});
const server=spawn(process.execPath,['scripts/serve-rust-preview.mjs','web/dist','5197'],{stdio:['ignore','pipe','inherit']});await new Promise(r=>server.stdout.once('data',r));
const browser=await chromium.launch({headless:true,executablePath:'/usr/bin/chromium',args:['--no-sandbox','--disable-dev-shm-usage']});
const page=await browser.newPage({viewport:{width:1280,height:900}}),errors=[],report={checks:[]};page.on('pageerror',e=>errors.push(String(e)));
const state=()=>page.evaluate(()=>window.__lhMujoco?.state);
try{
 await page.goto('http://127.0.0.1:5197');await page.waitForFunction(()=>window.__lhMujoco?.state);
 const initial=await state();assert.equal(initial.mode,'passive');
 await page.selectOption('#mode','balance');await page.click('#pause');
 await page.waitForFunction(()=>window.__lhMujoco.state.time>=10);await page.click('#pause');
 const stood=await state();assert(!stood.fallen);assert(stood.balance.tilt<20*Math.PI/180);assert(stood.reactions.supportedBodies.every(b=>[7,10].includes(b)));report.stood=stood.balance;report.checks.push('10 seconds standing in Chromium');
 const ticks=stood.steps;await page.waitForTimeout(100);assert.equal((await state()).steps,ticks);report.checks.push('pause preserves integration counter');
 await page.screenshot({path:out+'/standing.png'});
 await page.selectOption('#push-direction','lateral');await page.click('#small-push');await page.waitForFunction(t=>window.__lhMujoco.state.time>t+3,stood.time);await page.click('#pause');assert(!(await state()).fallen);report.checks.push('fixed lateral push via UI');
 const box=await page.locator('canvas').boundingBox(),hand=(await state()).screen.find(p=>p.name==='left_hand');await page.mouse.move(box.x+hand.x,box.y+hand.y);await page.mouse.down();assert((await state()).grab);await page.mouse.move(box.x+hand.x+20,box.y+hand.y-15);await page.mouse.up();assert.equal((await state()).grab,null);report.checks.push('grab/move/release in balance');
 for(let i=0;i<10;i++){await page.click('#reset');const s=await state();assert.deepEqual(s.qpos,initial.qpos);assert(s.qvel.every(v=>v===0));assert(s.paused&&!s.fallen);assert.equal(s.mode,'balance');}report.checks.push('10 exact resets preserve selected mode');
 await page.selectOption('#mode','passive');assert.equal((await state()).mode,'passive');
 await page.setViewportSize({width:390,height:844});await page.waitForTimeout(100);assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));report.checks.push('mobile layout, cloud browser only');
 assert.deepEqual(errors,[]);report.passed=true;
}finally{
 const s=await state().catch(()=>null);report.ticks=s?.steps??null;report.errors=errors;
 await writeFile(out+'/result.json',JSON.stringify(report,null,2));
 if(s){const path='web/proofs/balance-ledger.json',ledger=JSON.parse(await readFile(path));ledger.runs.push({type:'browser',ticks:s.steps});ledger.newTicks+=s.steps;await writeFile(path,JSON.stringify(ledger));}
 console.log(JSON.stringify(report));await browser.close();server.kill();
}
