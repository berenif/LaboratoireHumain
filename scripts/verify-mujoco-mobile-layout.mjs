// Layout acceptance only. Never clicks a control that starts physics.
// Recorded report text is a labelled DOM fixture, not a restored physical state.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import {spawn} from 'node:child_process';
import {pathToFileURL} from 'node:url';
const {chromium}=await import(process.env.LH_PLAYWRIGHT_MODULE?pathToFileURL(process.env.LH_PLAYWRIGHT_MODULE).href:'playwright');
const output=process.argv[2]||'web/proofs/mobile-layout-8efb3d0';
assert(!fs.existsSync(output),'Use a new proof directory; never overwrite evidence');
const server=spawn(process.execPath,['scripts/serve-rust-preview.mjs','web/dist','5204'],{stdio:['ignore','pipe','pipe']});
let browser;
try{
 await new Promise((resolve,reject)=>{const timer=setTimeout(()=>reject(Error('Preview readiness timeout')),5000);server.stdout.on('data',b=>{if(String(b).includes('5204')){clearTimeout(timer);resolve()}});server.once('exit',c=>{clearTimeout(timer);reject(Error('Preview exited '+c))})});
 browser=await chromium.launch({executablePath:process.env.CHROMIUM_PATH||'/usr/bin/chromium',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disk-cache-size=0']});
 const page=await browser.newPage({viewport:{width:390,height:844},isMobile:true,hasTouch:true,deviceScaleFactor:1});
 const errors=[];page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)errors.push(r.status()+' '+r.url())});
 await page.goto('http://127.0.0.1:5204/',{waitUntil:'networkidle'});await page.waitForFunction(()=>window.__lhMujoco?.state);
 const physical=()=>page.evaluate(()=>{const s=window.__lhMujoco.state;return {steps:s.steps,time:s.time,paused:s.paused,qpos:s.qpos,qvel:s.qvel,force:s.force,resets:s.resets}});
 const initial=await physical();assert.equal(initial.steps,0);assert.equal(initial.paused,true);
 await page.evaluate(()=>window.__layoutNodes=['pause','reset','camera-reset','zoom-in','zoom-out','getup-start','getup-continue'].map(id=>[id,document.getElementById(id),document.getElementById(id).onclick]));
 const reports=[];let portrait;
 for(const viewport of [{width:360,height:800},{width:390,height:844},{width:844,height:390},{width:1366,height:900}]){
  await page.setViewportSize(viewport);await page.evaluate(()=>scrollTo(0,0));await page.evaluate(()=>new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r))));
  const report=await page.evaluate(()=>{
   const rect=e=>{const r=e.getBoundingClientRect();return {x:r.x,y:r.y,w:r.width,h:r.height,right:r.right,bottom:r.bottom}};
   const scene=rect(document.querySelector('.scene')),toolbar=rect(document.querySelector('.playback-controls'));
   const controls=[...document.querySelectorAll('button,select,summary')].map(e=>({id:e.id||e.tagName,...rect(e)}));
   const canvas=document.querySelector('canvas'),pixels=canvas.getContext('2d').getImageData(0,0,canvas.width,canvas.height).data;
   const colors=new Set(['160,200,187','216,203,177','109,148,129','49,85,78']);let left=canvas.width,top=canvas.height,right=0,bottom=0,count=0;
   for(let y=0;y<canvas.height;y++)for(let x=0;x<canvas.width;x++){const i=4*(y*canvas.width+x);if(colors.has([pixels[i],pixels[i+1],pixels[i+2]].join(','))){left=Math.min(left,x);right=Math.max(right,x);top=Math.min(top,y);bottom=Math.max(bottom,y);count++}}
   return {viewport:[innerWidth,innerHeight],width:document.documentElement.scrollWidth,scene,toolbar,controls,body:{left,top,right,bottom,count,width:canvas.width,height:canvas.height},sameNodes:window.__layoutNodes.every(([id,e,handler])=>document.querySelectorAll('#'+id).length===1&&document.getElementById(id)===e&&e.onclick===handler)};
  });
  assert.equal(report.width,viewport.width,'Horizontal overflow');assert(report.sameNodes,'Control duplicated/replaced');
  for(const c of report.controls){assert(c.w>=44&&c.h>=44,'Small touch target: '+c.id)}
  assert(report.body.count>100&&report.body.left>1&&report.body.top>1&&report.body.right<report.body.width-2&&report.body.bottom<report.body.height-2,'Body clipped by canvas');
  if(viewport.width<1000){
   for(const id of ['pause','camera-reset','zoom-in','zoom-out','getup-start','getup-continue']){const c=report.controls.find(c=>c.id===id);assert(c.y>=0&&c.bottom<=viewport.height,'Primary control below first viewport: '+id)}
   assert(report.scene.y>=0&&report.scene.bottom<=viewport.height,'Scene outside initial viewport');
   const a=report.scene,b=report.toolbar;assert(Math.min(a.right,b.right)<=Math.max(a.x,b.x)||Math.min(a.bottom,b.bottom)<=Math.max(a.y,b.y),'Toolbar covers scene');
  }
  reports.push(report);if(viewport.width===390)portrait=await page.screenshot();
  await page.locator('#getup-details summary').scrollIntoViewIfNeeded();await page.locator('#getup-details summary').tap();assert(await page.locator('#getup-details').evaluate(e=>e.open));
  assert(await page.locator('.getup-warning').isVisible());await page.locator('#getup-details summary').tap();
  await page.locator('#zoom-in').scrollIntoViewIfNeeded();await page.locator('#zoom-in').tap();assert((await page.evaluate(()=>window.__lhMujoco.state.camera.zoom))>1);
  await page.locator('#camera-reset').tap();assert.equal(await page.evaluate(()=>window.__lhMujoco.state.camera.zoom),1);
  await page.locator('#reset').scrollIntoViewIfNeeded();assert(await page.locator('#reset').evaluate(e=>{const r=e.getBoundingClientRect();return document.elementFromPoint(r.x+r.width/2,r.y+r.height/2)===e}),'Reset inaccessible');
  assert.deepEqual(await physical(),initial);
 }
 // This text is a layout fixture based on the preserved complete-journey report.
 // It does not change GetUp ownership, enable Continue or restore any world state.
 await page.setViewportSize({width:844,height:390});
 await page.evaluate(()=>{
  const e=document.getElementById('getup-result');e.hidden=false;e.textContent='FIXTURE D’AFFICHAGE — BILAN ENREGISTRÉ, ZÉRO NOUVEAU PAS\nEN PAUSE · Fenêtre de 5 s terminée — en pause\nDepuis état final vivant du cycle · 4000 / 4 000 pas · t=27.500 s\nDeux pieds chargés · calme continu : 3.300 s\nPoussée torse +X : 15 N · 120 / 120 pas (150 ms prévus après 1 s calme)\nPoussée traversée et dernière seconde calme observée sur ce cas.\nFin en pause explicite. Aucun relais vers Équilibre.';
  const p=document.getElementById('getup-previous');p.hidden=false;p.textContent='Fixture de mise en page : bilan du cycle conservé, 10 200 pas enregistrés. Le monde affiché reste à zéro pas.';
 });
 assert(!(await page.locator('#getup-details').evaluate(e=>e.open)),'Details should remain collapsed');
 assert(await page.locator('#getup-result').isVisible());assert(await page.locator('#getup-previous').isVisible());assert(await page.locator('#getup-continue').isDisabled());
 await page.locator('#getup-result').evaluate(e=>scrollBy(0,e.getBoundingClientRect().top-138));
 await page.evaluate(()=>new Promise(r=>requestAnimationFrame(r)));
 const fixture=await page.locator('#getup-result').evaluate(e=>{const r=e.getBoundingClientRect();return {labelled:e.textContent.startsWith('FIXTURE'),width:e.clientWidth,scrollWidth:e.scrollWidth,top:r.top,toolbarBottom:document.querySelector('.playback-controls').getBoundingClientRect().bottom}});
 assert(fixture.labelled&&fixture.scrollWidth<=fixture.width&&fixture.top>=fixture.toolbarBottom,'Report hidden by toolbar or overflowing');
 const landscape=await page.screenshot();
 await page.locator('#getup-result').evaluate(e=>scrollBy(0,e.getBoundingClientRect().bottom-(innerHeight-16)));
 assert(await page.locator('#getup-result').evaluate(e=>e.getBoundingClientRect().bottom<=innerHeight),'Report end unreachable');
 assert.deepEqual(await physical(),initial);assert.deepEqual(errors,[]);
 const audit={passed:true,integrations:0,physicalUnchanged:true,controlsRetained:true,phoneTested:false,viewportReports:reports,fixture,errors};
 const auditBytes=Buffer.from(JSON.stringify(audit));assert(portrait.length+landscape.length+auditBytes.length<=500000,'Proof budget');
 fs.mkdirSync(output,{recursive:true});fs.writeFileSync(path.join(output,'portrait.png'),portrait);fs.writeFileSync(path.join(output,'landscape-fixture.png'),landscape);fs.writeFileSync(path.join(output,'audit.json'),auditBytes);
 console.log(JSON.stringify({passed:true,integrations:0,viewports:reports.map(r=>r.viewport),proofBytes:portrait.length+landscape.length+auditBytes.length,output,phoneTested:false}));
}finally{await browser?.close();server.kill('SIGTERM')}
