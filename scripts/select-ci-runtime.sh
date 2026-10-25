#!/usr/bin/env bash
set -euo pipefail
: "${RUNNER_TOOL_CACHE:?Runner cache required}"
: "${GITHUB_PATH:?GitHub path required}"
# Runner images do not all contain the same patch release. Use only cached
# runtimes satisfying the publishing contract; never download or upgrade one.
for candidate_bin in "${RUNNER_TOOL_CACHE}/node/24.20.0/x64/bin" "${RUNNER_TOOL_CACHE}"/node/*/x64/bin; do
  [[ -x "${candidate_bin}/node" && -x "${candidate_bin}/npm" ]] || continue
  if ! npm_version=$(PATH="${candidate_bin}:${PATH}" "${candidate_bin}/npm" --version); then continue; fi
  if "${candidate_bin}/node" -e '
    const meets = (value, minimum) => {
      if (!/^\d+\.\d+\.\d+$/.test(value)) return false;
      const parts = value.split(".").map(Number);
      for (let i = 0; i < 3; i++) if (parts[i] !== minimum[i]) return parts[i] > minimum[i];
      return true;
    };
    process.exit(meets(process.versions.node, [22,14,0]) && meets(process.argv[1], [11,5,1]) ? 0 : 1);
  ' "${npm_version}"; then
    printf '%s\n' "${candidate_bin}" >> "${GITHUB_PATH}"
    printf 'Using cached Node %s / npm %s\n' "$("${candidate_bin}/node" --version)" "${npm_version}"
    exit 0
  fi
done
echo 'No cached Node >=22.14.0 with npm >=11.5.1; no installation attempted.' >&2
exit 1
