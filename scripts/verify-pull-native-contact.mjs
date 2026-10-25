// Independent scalar/contact counterexamples, no engine or physical integration.
import assert from'node:assert/strict';import{contactLaw,errorBound}from'./pull-native-contact.mjs';
let tests=0;
for(const smooth of[-3,0,1,2,4])for(const a of[-2,0,.5,1,3]){const exact=smooth<2?(smooth+3)/5:smooth/2,b=errorBound([2],[[1]],[3],[1],[a],[smooth],r=>[r[0]/2]);assert(Math.sqrt(2)*Math.abs(a-exact)<=b.eta+1e-14);assert(b.forces[0]>=0);tests++;}
assert.deepEqual(contactLaw([[1],[-1]],[2,3],[0,0],[0]),[0,0]);tests++;
assert.deepEqual(contactLaw([[1],[-1]],[2,3],[0,0],[1]),[0,3]);tests++;
// A frozen-limit reaction cannot satisfy the opposing sign's actual constitutive law.
assert.notDeepEqual(contactLaw([[-1]],[3],[1],[1]),[1]);tests++;
// Native four nonnegative pyramid edges guarantee the anisotropic L1 cone.
for(const e of[[0,0,0,0],[3,0,2,1],[1,2,3,4]]){const n=e.reduce((a,b)=>a+b,0),t1=.7*(e[0]-e[1]),t2=.3*(e[2]-e[3]);assert(Math.abs(t1)/.7+Math.abs(t2)/.3<=n+1e-14);tests++;}
assert.throws(()=>errorBound([2],[[1]],[3],[1],[NaN],[0],r=>[r[0]/2]));tests++;
console.log(JSON.stringify({passed:true,tests,ticks:0,nativeForwards:0,branchCrossingBound:true}));
