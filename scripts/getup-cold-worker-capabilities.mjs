// No Runtime constructor, checkpoint, or physical call.
import {readFileSync} from 'node:fs';
import {resolve,join} from 'node:path';
import {pathToFileURL} from 'node:url';
import {checkColdWorkerCapabilities} from './getup-cold-worker-contract.mjs';
const worker=resolve(process.argv[2]);
const m=await import(pathToFileURL(join(worker,'lh_worker.js')));
m.initSync({module:readFileSync(join(worker,'lh_worker_bg.wasm'))});
console.log(JSON.stringify(checkColdWorkerCapabilities(JSON.parse(m.getup_build_capabilities()))));
