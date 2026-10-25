export interface MujocoStepScheduleInput {
  requestedDeltaSeconds: number;
  accumulatorSeconds: number;
  stepDtSeconds: number;
  maxSubsteps: number;
}

export interface MujocoStepSchedule {
  steps: number;
  requestedDeltaSeconds: number;
  advancedDeltaSeconds: number;
  droppedDeltaSeconds: number;
  accumulatorSeconds: number;
}

export function scheduleMujocoSubsteps(input: MujocoStepScheduleInput): MujocoStepSchedule {
  const requestedDeltaSeconds = finiteNonNegative(input.requestedDeltaSeconds);
  const previousAccumulatorSeconds = finiteNonNegative(input.accumulatorSeconds);
  const stepDtSeconds = Math.max(Number.EPSILON, finitePositive(input.stepDtSeconds, 1 / 240));
  const maxSubsteps = Math.max(1, Math.floor(finitePositive(input.maxSubsteps, 1)));
  const availableSeconds = previousAccumulatorSeconds + requestedDeltaSeconds;
  const availableSteps = Math.floor((availableSeconds + stepDtSeconds * 1e-9) / stepDtSeconds);
  const steps = Math.min(maxSubsteps, availableSteps);
  const droppedSteps = Math.max(0, availableSteps - steps);
  const advancedDeltaSeconds = steps * stepDtSeconds;
  const droppedDeltaSeconds = droppedSteps * stepDtSeconds;
  const accumulatorSeconds = Math.max(
    0,
    availableSeconds - advancedDeltaSeconds - droppedDeltaSeconds
  );

  return {
    steps,
    requestedDeltaSeconds,
    advancedDeltaSeconds,
    droppedDeltaSeconds,
    accumulatorSeconds
  };
}

export function applyMujocoTimestepToXml(source: string, stepDtSeconds: number): string {
  const timestep = formatMujocoNumber(finitePositive(stepDtSeconds, 1 / 240));
  const optionPattern = /<option\b[^>]*>/i;
  const optionMatch = source.match(optionPattern)?.[0];

  if (optionMatch) {
    const nextOption = /\btimestep\s*=\s*"[^"]*"/i.test(optionMatch)
      ? optionMatch.replace(/\btimestep\s*=\s*"[^"]*"/i, `timestep="${timestep}"`)
      : optionMatch.replace(/<option\b/i, `<option timestep="${timestep}"`);

    return source.replace(optionMatch, nextOption);
  }

  return source.replace(/(<mujoco\b[^>]*>)/i, `$1\n  <option timestep="${timestep}" />`);
}

function finiteNonNegative(value: number): number {
  return Number.isFinite(value) && value > 0 ? value : 0;
}

function finitePositive(value: number, fallback: number): number {
  return Number.isFinite(value) && value > 0 ? value : fallback;
}

function formatMujocoNumber(value: number): string {
  return Number(value.toFixed(9)).toString();
}
