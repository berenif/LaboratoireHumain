import {mkdir,copyFile,readFile,writeFile} from 'node:fs/promises';
const out='web/dist';await mkdir(out+'/vendor',{recursive:true});
for(const f of ['index.html','app.js','engine.js','mvp.css','LICENSE-MuJoCo.txt'])await copyFile('web/mujoco/'+f,out+'/'+f);
await copyFile('rust/browser/app.css',out+'/app.css');
for(const f of ['mujoco.js','mujoco.wasm'])await copyFile('node_modules/@mujoco/mujoco/'+f,out+'/vendor/'+f);
// Only presentation includes/materials are removed. Body/joint/actuator,
// collision and passive parameters remain those of the preserved MJCF.
let xml=await readFile('rust/crates/model/data/getup-humanoid.xml','utf8');
xml=xml.replace(/<include\b[^>]*\/>/g,'').replace(/\s+material="[^"]*"/g,'');
await writeFile(out+'/humanoid.xml',xml);
console.log('MuJoCo MVP built: web/dist (official local WASM, no network at runtime)');
