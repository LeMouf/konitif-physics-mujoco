import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { readFileSync, realpathSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { resolve } from 'node:path';

const authorityRoot = fileURLToPath(new URL('..', import.meta.url));

export function assertPublishingTools(nodeVersion, npmVersion) {
  const meets = (value, minimum) => {
    if (!/^\d+\.\d+\.\d+$/.test(value)) return false;
    const parts = value.split('.').map(Number);
    for (let index = 0; index < 3; index += 1) {
      if (parts[index] !== minimum[index]) return parts[index] > minimum[index];
    }
    return true;
  };
  assert.ok(meets(nodeVersion, [22, 14, 0]), 'Node >=22.14.0 required');
  assert.ok(meets(npmVersion, [11, 5, 1]), 'npm >=11.5.1 required; no automatic upgrade');
}

export function assertReleaseInputs(policy, manifest, lock, env) {
  assert.equal(env.GITHUB_REPOSITORY, policy.repository);
  assert.ok(env.GITHUB_EVENT_NAME === 'push' || env.GITHUB_EVENT_NAME === 'workflow_dispatch');
  assert.match(manifest.version, /^0\.(0|[1-9]\d*)\.(0|[1-9]\d*)$/);
  const expectedTag = `v${manifest.version}`;
  assert.equal(env.GITHUB_REF, `refs/tags/${expectedTag}`);
  if (env.GITHUB_EVENT_NAME === 'workflow_dispatch') {
    assert.equal(env.MUJOCO_RELEASE_TAG, expectedTag);
  }
  assert.equal(manifest.name, policy.packageName);
  assert.equal(manifest.private, false);
  assert.equal(manifest.license, 'PolyForm-Noncommercial-1.0.0');
  assert.equal(manifest.publishConfig?.access, 'public');
  assert.equal(manifest.publishConfig?.registry, 'https://registry.npmjs.org/');
  assert.equal(manifest.repository?.url, `git+https://github.com/${policy.repository}.git`);
  assert.equal(lock.name, manifest.name);
  assert.equal(lock.version, manifest.version);
  assert.equal(lock.packages[''].name, manifest.name);
  assert.equal(lock.packages[''].version, manifest.version);
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const releaseRoot = realpathSync(resolve(authorityRoot, process.env.MUJOCO_RELEASE_ROOT ?? '.'));
  const read = (root, file) => JSON.parse(readFileSync(resolve(root, file), 'utf8'));
  assertReleaseInputs(
    read(authorityRoot, 'release-policy.json'),
    read(releaseRoot, 'package.json'),
    read(releaseRoot, 'package-lock.json'),
    process.env,
  );
  assertPublishingTools(
    process.versions.node,
    execFileSync('npm', ['--version'], { encoding: 'utf8' }).trim(),
  );
}
