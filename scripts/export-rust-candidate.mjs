// Export only an already accepted build, including its frozen sources and receipt.
import assert from 'node:assert/strict';
import { cpSync, existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { fingerprint, sourceHashes, treeHashes, validateCandidate } from './rust-release-tools.mjs';
import { workspaceOutput } from './rust-browser-tools.mjs';

const run=resolve(process.argv[2]??''),out=workspaceOutput(process.argv[3]);
const receipt=JSON.parse(readFileSync(join(run,'execution.json')));
const build=join(run,'candidate');
const candidate=JSON.parse(readFileSync(join(build,'candidate.json')));
validateCandidate(candidate,receipt,treeHashes(join(build,'site')));
assert.equal(fingerprint(sourceHashes()),candidate.sourceFingerprint,'Live sources changed after acceptance');
assert.equal(fingerprint(treeHashes(join(run,'source'))),candidate.sourceFingerprint,'Frozen source snapshot changed');
assert.ok(!existsSync(out),'Use a fresh accepted bundle directory');
mkdirSync(out,{recursive:true});
cpSync(join(build,'site'),join(out,'site'),{recursive:true,errorOnExist:true});
cpSync(join(run,'source'),join(out,'source'),{recursive:true,errorOnExist:true});
cpSync(build,join(out,'qualification'),{recursive:true,errorOnExist:true});
writeFileSync(join(out,'candidate.json'),JSON.stringify(candidate,null,2)+'\n');
writeFileSync(join(out,'receipt.json'),JSON.stringify(receipt,null,2)+'\n');
validateCandidate(candidate,receipt,treeHashes(join(out,'site')));
console.log(JSON.stringify({candidateId:candidate.candidateId,receiptId:fingerprint(receipt),directory:out}));
