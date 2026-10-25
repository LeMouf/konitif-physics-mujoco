import {PhysicsWorkerBackendAdapter} from '@konitif/physics/worker';
import type {MujocoPhysicsInitOptions} from './mujocoInitOptions.js';
/** Explicit runtime resource and worker factory; no product paths or network discovery. */
export function createMujocoBackend(options:{moduleUrl:string;workerFactory?:()=>Worker}){
 if(!options.moduleUrl.trim())throw Error('Une ressource de moteur MuJoCo doit être fournie.');
 const initPayload:MujocoPhysicsInitOptions={moduleUrl:options.moduleUrl};
 return new PhysicsWorkerBackendAdapter({engine:'mujoco',initPayload,workerSource:'./mujocoPhysicsWorker.ts',worker:options.workerFactory?.()??new Worker(new URL('./mujocoPhysicsWorker.ts',import.meta.url),{type:'module',name:'konitif-mujoco-physics'}),deadlineHost:{schedule(callback,delay){const timer=setTimeout(callback,delay);return()=>clearTimeout(timer);}}});
}
