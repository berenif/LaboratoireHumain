import test from 'node:test';
import assert from 'node:assert/strict';
import {appendInterval,intervalSummary} from '../scripts/getup-wasm-adaptation-math.mjs';
const row=pass=>({current:{predicates:Object.fromEntries(['torso','bothFeetLoadedAndPersistent','footLoadFraction','onlyFeet','linearMotion','angularMotion'].map(k=>[k,pass]))}});
test('native interval joins prefix before an interior failure',()=>{
 const next=appendInterval({longest:79,tail:79},[...Array.from({length:19},()=>row(true)),row(false)]);
 assert.deepEqual(next,{longest:98,tail:0});
});
test('native interval joins full packets and preserves later tail',()=>{
 assert.deepEqual(appendInterval({longest:20,tail:20},Array.from({length:20},()=>row(true))),{longest:40,tail:40});
 assert.deepEqual(intervalSummary([row(false),row(true),row(true)]),{longest:2,tail:2,prefix:0,count:3});
});
