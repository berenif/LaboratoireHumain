// Promotion validates existing bytes; it never invokes a compiler or builder.
import assert from 'node:assert/strict';
import { cpSync, existsSync, mkdirSync, readFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { fingerprint, treeHashes, validateCandidate } from './rust-release-tools.mjs';
import { workspaceOutput } from './rust-browser-tools.mjs';

const bundle=resolve(process.argv[2]??''),destination=process.argv[3]?workspaceOutput(process.argv[3]):null;
assert.ok(process.argv[2],'Supply an accepted candidate bundle');
const candidate=JSON.parse(readFileSync(join(bundle,'candidate.json')));
const receipt=JSON.parse(readFileSync(join(bundle,'receipt.json')));
validateCandidate(candidate,receipt,treeHashes(join(bundle,'site')));
assert.equal(fingerprint(treeHashes(join(bundle,'source'))),candidate.sourceFingerprint,'Frozen source bundle changed');
assert.equal(candidate.mount,'/LaboratoireHumain/');
if(destination){
  assert.ok(!existsSync(destination),'Refuse to overwrite a promotion destination');
  mkdirSync(destination,{recursive:true});cpSync(join(bundle,'site'),destination,{recursive:true,errorOnExist:true});
  assert.deepEqual(treeHashes(destination),candidate.assets,'Promotion changed the accepted artifact');
}
console.log(JSON.stringify({candidateId:candidate.candidateId,receiptId:fingerprint(receipt),assetsFingerprint:candidate.assetsFingerprint,promoted:destination}));
