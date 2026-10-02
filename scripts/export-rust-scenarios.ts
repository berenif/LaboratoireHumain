import { createHash } from "node:crypto";
import { existsSync, mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { ACCEPTANCE, RECOVERY_ACCEPTANCE, PULL_FIXTURES } from "./physics-fixtures";
const path="rust/crates/sim/data/scenarios-v1.json";
if(existsSync(path)) throw new Error("Refuse replacement of versioned inputs");
mkdirSync("rust/crates/sim/data",{recursive:true});
const sources=["scripts/physics-fixtures.ts","src/character/GrabAnchorController.ts"];
const fingerprints=Object.fromEntries(sources.map(p=>[p,createHash("sha256").update(readFileSync(p)).digest("hex")]));
writeFileSync(path,JSON.stringify({schema:1,commandTickHz:60,sourceFingerprints:fingerprints,
  contract:ACCEPTANCE,recoveryContract:RECOVERY_ACCEPTANCE,pulls:PULL_FIXTURES},null,2)+"\n",{flag:"wx"});
console.log(JSON.stringify({path,pulls:PULL_FIXTURES.length,fingerprints}));
