import assert from 'node:assert/strict';
import { mkdtempSync, mkdirSync, rmSync, symlinkSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { spawnSync } from 'node:child_process';
import test from 'node:test';
import { fileURLToPath } from 'node:url';

const script = fileURLToPath(new URL('../scripts/select-ci-runtime.sh', import.meta.url)).replaceAll('\\', '/');
const bash = process.platform === 'win32' ? 'C:/Program Files/Git/bin/bash.exe' : 'bash';
function run(npmVersion) {
  const directory = mkdtempSync(join(tmpdir(), 'konitif-cached-runtime-'));
  const bin = join(directory, 'node', process.versions.node, 'x64', 'bin');
  mkdirSync(bin, { recursive: true });
  symlinkSync(process.execPath, join(bin, 'node'));
  if (npmVersion) writeFileSync(join(bin, 'npm'), `#!/bin/sh\nprintf '%s\\n' '${npmVersion}'\n`, { mode: 0o755 });
  try {
    return spawnSync(bash, [script], { encoding: 'utf8', env: {
      ...process.env, RUNNER_TOOL_CACHE: directory.replaceAll('\\', '/'), GITHUB_PATH: join(directory, 'github-path').replaceAll('\\', '/')
    } });
  } finally {
    rmSync(directory, { recursive: true, force: true });
  }
}
test('selects a compatible cached runtime even without the hardcoded patch release', () => {
  const result = run('11.6.0');
  assert.equal(result.status, 0, result.stderr);
  assert.match(result.stdout, /Using cached Node/);
});
test('fails closed for incompatible or missing cached npm without installing anything', () => {
  for (const version of ['10.9.4', '11.5.0', 'invalid', null]) {
    const result = run(version);
    assert.equal(result.status, 1);
    assert.match(result.stderr, /no installation attempted/);
  }
});
