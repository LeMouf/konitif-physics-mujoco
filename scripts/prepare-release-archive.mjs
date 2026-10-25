import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { constants, copyFileSync, mkdirSync, readFileSync, realpathSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, join, relative, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const authorityRoot = fileURLToPath(new URL('..', import.meta.url));
const releaseRoot = realpathSync(resolve(authorityRoot, process.env.MUJOCO_RELEASE_ROOT ?? '.'));
const output = execFileSync(process.execPath, ['scripts/verify-package.mjs'], {
  cwd: releaseRoot,
  encoding: 'utf8',
});
process.stdout.write(output);
const evidence = JSON.parse(output);
assert.equal(typeof evidence.evidence, 'string');
assert.equal(typeof evidence.archive, 'string');
const directory = realpathSync(evidence.evidence);
const archive = realpathSync(evidence.archive);
const archiveRelative = relative(directory, archive);
assert.ok(archiveRelative !== '' && !archiveRelative.startsWith(`..${process.platform === 'win32' ? '\\' : '/'}`));
assert.equal(dirname(archive), directory);
const bytes = readFileSync(archive);
assert.equal(`sha512-${createHash('sha512').update(bytes).digest('base64')}`, evidence.integrity);
assert.equal(bytes.length, evidence.bytes);
mkdirSync(join(authorityRoot, '.release'), { recursive: true });
copyFileSync(archive, join(authorityRoot, '.release/package.tgz'), constants.COPYFILE_EXCL);
console.log('Verified archive retained at .release/package.tgz');
