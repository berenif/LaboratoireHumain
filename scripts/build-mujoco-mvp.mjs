import {mkdir,copyFile,readFile,writeFile,link} from 'node:fs/promises';
const out='web/dist';await mkdir(out+'/vendor',{recursive:true});
for(const f of ['index.html','app.js','engine.js','mvp.css','LICENSE-MuJoCo.txt'])await copyFile('web/mujoco/'+f,out+'/'+f);
await copyFile('rust/browser/app.css',out+'/app.css');
for(const f of ['mujoco.js','mujoco.wasm'])await copyFile('node_modules/@mujoco/mujoco/'+f,out+'/vendor/'+f);
// Only presentation includes/materials are removed. Body/joint/actuator,
// collision and passive parameters remain those of the preserved MJCF.
let xml=await readFile('rust/crates/model/data/getup-humanoid.xml','utf8');
xml=xml.replace(/<include\b[^>]*\/>/g,'').replace(/\s+material="[^"]*"/g,'');
await writeFile(out+'/humanoid.xml',xml);
await mkdir(out+'/local-getup',{recursive:true});
for(const f of ['cycle.js','continuation.js','slow.js'])await copyFile('web/mujoco/local-getup/'+f,out+'/local-getup/'+f);
await mkdir(out+'/getup-assets',{recursive:true});
// Immutable runtime inputs: hard links avoid duplicating the existing weights
// under the workspace disk cap. A normal tar/package includes their file bytes.
// Never overwrite a changed linked asset, which could mutate its source inode.
for(const [source,name] of [
 ['web/proofs/getup-weak-seed10/weak-final.json','weak-final.json'],
 ['web/proofs/getup-continuous-slow/reference.json','reference.json'],
 ['rust/crates/sim/data/getup-policy/student.bin','student.bin'],
 ['rust/crates/model/data/getup-v1.json','getup-v1.json'],
]){
 const destination=out+'/getup-assets/'+name;
 try{await link(source,destination);}catch(e){
  if(e.code!=='EEXIST')throw e;
  if(!(await readFile(source)).equals(await readFile(destination)))throw Error('Stale build asset; preserve and replace explicitly: '+destination);
 }
}
console.log('MuJoCo + known-entry GetUp built: web/dist (same-origin assets, no runtime downloads)');
