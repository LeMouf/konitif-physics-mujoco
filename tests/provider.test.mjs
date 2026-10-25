import assert from 'node:assert/strict';
import {test} from 'node:test';
import {readFileSync} from 'node:fs';
import {ExperimentSimulationSession,scheduleMujocoSubsteps,resolveMujocoExperimentTiming,createMujocoBackend} from '../dist/index.js';
import {readMujocoGroundContacts} from '../dist/mujocoGroundContacts.js';
test('import is inert and backend creation rejects an absent engine resource',()=>{
 assert.throws(()=>createMujocoBackend({moduleUrl:' '}),/ressource/);
 const code=readFileSync(new URL('../dist/createMujocoBackend.js',import.meta.url),'utf8');
 assert.match(code,/mujocoPhysicsWorker\.js/);assert.doesNotMatch(code,/mujocoPhysicsWorker\.ts/);
});
test('scheduler preserves fractional time and reports dropped whole steps',()=>{
 const a=scheduleMujocoSubsteps({requestedDeltaSeconds:1/120,accumulatorSeconds:0,stepDtSeconds:1/300,maxSubsteps:10});
 const b=scheduleMujocoSubsteps({requestedDeltaSeconds:1/120,accumulatorSeconds:a.accumulatorSeconds,stepDtSeconds:1/300,maxSubsteps:10});
 assert.equal(a.steps,2);assert.equal(b.steps,3);
 const c=scheduleMujocoSubsteps({requestedDeltaSeconds:.1,accumulatorSeconds:0,stepDtSeconds:.01,maxSubsteps:4});
 assert.equal(c.steps,4);assert.ok(Math.abs(c.droppedDeltaSeconds-.06)<1e-10);
});
test('one requested observation owns an integral number of solver steps',()=>{
 assert.equal(resolveMujocoExperimentTiming(1/60,1/240).maxSubsteps,4);
 assert.throws(()=>resolveMujocoExperimentTiming(1/60,1/100));
});
test('paused experiments do no work; explicit steps detach observed state',async()=>{
 let time=0;const state={angle:0};let targets=0;
 const port={async stepAndWait(dt){time+=dt;state.angle++;},setJointTarget(){targets++;},readObservation(){return {simulatedTimeSeconds:time,droppedTimeSeconds:0,state};}};
 const session=new ExperimentSimulationSession(port,.01,{joint:{minRadians:-1,maxRadians:1}});
 assert.equal(await session.tick(),null);assert.equal(time,0);
 session.setMotorTarget('joint',.5);assert.equal(targets,1);assert.throws(()=>session.setMotorTarget('joint',2));
 const observed=await session.advanceOneStep();state.angle=5;assert.equal(observed.state.angle,1);assert.equal(observed.simulatedTimeSeconds,.01);
});
test('an incomplete physical step faults the session rather than inventing time',async()=>{
 const session=new ExperimentSimulationSession({async stepAndWait(){},setJointTarget(){},readObservation(){return {simulatedTimeSeconds:0,droppedTimeSeconds:0,state:{}};}},.01,{});
 await assert.rejects(session.advanceOneStep(),/exactly one step/);
 assert.equal(session.getStatus().faulted,true);assert.throws(()=>session.resume(),/faulted/);
});
test('observed empty contacts differ from unavailable contact data',()=>{
 const model={geom_type:[0,6],geom_bodyid:[0,1]};
 assert.equal(readMujocoGroundContacts(model,{}).status,'unavailable');
 assert.deepEqual(readMujocoGroundContacts(model,{ncon:0,contact:{get(){}}}),{status:'observed',points:[]});
});
test('compiled worker loads as ESM and refuses initialization without a supplied engine',async()=>{
 let receive;let resolveReply;
 globalThis.self={addEventListener(type,listener){assert.equal(type,'message');receive=listener;},location:{href:'http://127.0.0.1/worker.js'}};
 globalThis.postMessage=message=>resolveReply(message);
 try{
  await import('../dist/mujocoPhysicsWorker.js');
  const reply=new Promise(resolve=>{resolveReply=resolve;});
  receive({data:{id:'init',command:'init',payload:{}}});
  const result=await reply;assert.equal(result.ok,false);assert.match(result.error,/moduleUrl/);
 }finally{delete globalThis.self;delete globalThis.postMessage;}
});
