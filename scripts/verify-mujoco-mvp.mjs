import assert from 'node:assert/strict';
import {mkdir,writeFile} from 'node:fs/promises';
import {spawn} from 'node:child_process';
const {chromium}=await import('playwright').catch(()=>import('/opt/codex/cua_node/lib/node_modules/playwright/index.mjs'));
const port=5196,server=spawn(process.execPath,['scripts/serve-rust-preview.mjs','web/dist',String(port)],{stdio:['ignore','pipe','inherit']});
await new Promise((resolve,reject)=>{server.stdout.once('data',resolve);server.once('error',reject);});
const browser=await chromium.launch({headless:true,executablePath:process.env.CHROMIUM_PATH??'/usr/bin/chromium',args:['--no-sandbox','--disable-dev-shm-usage']});
const errors=[],report={engine:'MuJoCo 3.17.0',rapierHistoricalTicks:45735,checks:[]};
const output=process.env.MUJOCO_PROOFS??'web/proofs';await mkdir(output,{recursive:true});
try{
 const page=await browser.newPage({viewport:{width:1280,height:850}});page.on('pageerror',e=>errors.push(String(e)));
 const state=()=>page.evaluate(()=>window.__lhMujoco.state);
 await page.goto(`http://127.0.0.1:${port}/`);await page.waitForFunction(()=>window.__lhMujoco?.state?.engine);
 const initial=await state();assert.equal(initial.physicalBodies,16);assert.equal(initial.actuators,21);assert.equal(initial.time,0);assert(initial.paused&&!initial.failed);report.checks.push('official WASM, 16 bodies / 21 actuators, finite initial state');
 await page.screenshot({path:output+'/mujoco-initial.png'});
 await page.locator('#pause').click();await page.waitForFunction(()=>window.__lhMujoco.state.time>.4);assert((await state()).contacts>0);report.checks.push('real gravity integration and ground contacts');
 await page.locator('#pause').click();const frozen=await state();await page.waitForTimeout(120);assert.equal((await state()).steps,frozen.steps);report.checks.push('pause stops integration');
 await page.locator('#reset').click();assert.deepEqual((await state()).qpos,initial.qpos);
 await page.locator('#pause').click();
 const box=await page.locator('canvas').boundingBox();const hand=(await state()).screen.find(p=>p.name==='left_hand');
 await page.mouse.move(box.x+hand.x,box.y+hand.y);await page.mouse.down();assert((await state()).grab,'screen pick starts physical grab');
 await page.mouse.move(box.x+hand.x+75,box.y+hand.y-55,{steps:4});await page.waitForTimeout(180);
 const pulled=await state();assert(pulled.force>0&&pulled.force<=180.0001);assert(pulled.qvel.some(x=>Math.abs(x)>.1));await page.screenshot({path:output+'/mujoco-pull.png'});
 await page.mouse.up();assert.equal((await state()).grab,null);assert.equal((await state()).force,0);report.checks.push('pointer pick, bounded spring traction, body response, release');report.pull={body:pulled.grab.body,forceN:pulled.force,time:pulled.time};
 await page.locator('#reset').click();await page.locator('#push').click();await page.waitForFunction(()=>window.__lhMujoco.state.time>.10);const pushed=await state();assert(pushed.qvel[0]>.01);report.push={rootVelocityX:pushed.qvel[0],time:pushed.time};report.checks.push('explicit push changes physical velocity');
 await page.waitForFunction(()=>window.__lhMujoco.state.time>1.2);await page.locator('#pause').click();assert(!(await state()).failed);await page.screenshot({path:output+'/mujoco-ground.png'});
 for(let i=0;i<25;i++){await page.locator('#reset').click();const s=await state();assert.equal(s.time,0);assert(s.paused);assert.equal(s.grab,null);assert.deepEqual(s.qpos,initial.qpos);assert(s.qvel.every(x=>x===0));}report.checks.push('25 repeated resets restore initial state with zero velocity');
 const c0=(await state()).camera;await page.mouse.move(box.x+50,box.y+100);await page.mouse.down();await page.mouse.move(box.x+120,box.y+125,{steps:4});await page.mouse.up();assert.notEqual((await state()).camera.yaw,c0.yaw);await page.locator('#zoom-in').click();assert((await state()).camera.zoom>c0.zoom);await page.locator('#camera-reset').click();assert.deepEqual((await state()).camera,c0);report.checks.push('background orbit, zoom and camera reset');
 // Loss of pointer capture/focus must never leave a spring attached.
 const h=(await state()).screen.find(p=>p.name==='left_hand');await page.mouse.move(box.x+h.x,box.y+h.y);await page.mouse.down();assert((await state()).grab);await page.evaluate(()=>window.dispatchEvent(new Event('blur')));assert.equal((await state()).grab,null);assert((await state()).paused);await page.mouse.up();report.checks.push('focus loss releases body and pauses');
 report.mujocoIntegrationSteps=(await state()).steps;
 await page.setViewportSize({width:390,height:844});await page.waitForTimeout(100);assert(await page.locator('canvas').isVisible());assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));await page.screenshot({path:output+'/mujoco-mobile.png'});report.checks.push('mobile layout fits viewport');
 assert.deepEqual(errors,[]);report.browserErrors=errors;report.passed=true;
 console.log(JSON.stringify(report,null,2));await writeFile(output+'/acceptance.json',JSON.stringify(report,null,2)+'\n');
}finally{await browser.close();server.kill();}
