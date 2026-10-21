// Minimal local preview: serve only explicit source and verified compiled inputs.
import {createServer} from 'node:http';
import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
if(!process.argv[2])throw Error('Usage: node scripts/serve-minimal-demo.mjs <verified-worker-assets-directory> [port]');
const assets=resolve(process.argv[2]);
const expected={
 'lh_worker_bg.wasm':'f9b06192c1d9b265dd179e80d8aa6c4ce72ad5cc0fa2dd2bddb9ee8fdb87fa3a',
 'lh_worker.js':'402d5904187bf71d511dc70cad171d87502598b31695fd67aaea5024b333de8a',
 'getup-800hz-3plus1-repeat-cycle-v1.json':'aaa415a19767c218a26b05bad277193cccba2a726b52dcd5aac0a5444afc39d6'};
for(const [name,digest] of Object.entries(expected))if(createHash('sha256').update(readFileSync(resolve(assets,name))).digest('hex')!==digest)throw Error('Unverified demo input: '+name);
const routes=new Map([
 ['/', ['rust/browser/minimal/index.html','text/html']],
 ['/app.js',['rust/browser/minimal/app.js','text/javascript']],
 ['/worker.js',['rust/browser/minimal/worker.js','text/javascript']],
 ['/model.json',['rust/crates/model/data/getup-v1.json','application/json']],
 ['/capabilities.json',['rust/browser/minimal/capabilities.json','application/json']],
 ['/profile.json',[resolve(assets,'getup-800hz-3plus1-repeat-cycle-v1.json'),'application/json']],
 ...['lh_worker.js','lh_worker_bg.wasm'].map(x=>['/'+x,[resolve(assets,x),x.endsWith('.wasm')?'application/wasm':'text/javascript']])]);
const server=createServer((req,res)=>{const item=routes.get(new URL(req.url,'http://localhost').pathname);if(!item){res.writeHead(404).end();return;}res.writeHead(200,{'Content-Type':item[1],'Cache-Control':'no-store'}).end(readFileSync(item[0]));});
server.listen(Number(process.argv[3]??5186),'127.0.0.1',()=>console.log('Minimal demo: http://127.0.0.1:'+server.address().port));
