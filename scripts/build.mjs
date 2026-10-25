import {execFileSync} from 'node:child_process';
import {readFileSync,writeFileSync} from 'node:fs';
execFileSync(process.execPath,['node_modules/typescript/bin/tsc','-p','tsconfig.json'],{stdio:'inherit'});
const file=new URL('../dist/createMujocoBackend.js',import.meta.url);
writeFileSync(file,readFileSync(file,'utf8').replaceAll('./mujocoPhysicsWorker.ts','./mujocoPhysicsWorker.js'));
