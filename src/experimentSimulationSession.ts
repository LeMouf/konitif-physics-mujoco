/** An experiment owns its physics port exclusively; renderers only consume observations. */
export interface ExperimentPhysicsPort<T> {
  stepAndWait(dtSeconds: number): Promise<void>;
  setJointTarget(jointName: string, target: { mode: 'position'; value: number }): void;
  readObservation(): { simulatedTimeSeconds: number; droppedTimeSeconds: number; state: T };
}

export interface ExperimentObservation<T> {
  sequence: number;
  simulatedTimeSeconds: number;
  state: T;
}

export interface ExperimentMotor {
  minRadians: number;
  maxRadians: number;
}

/** No wall clock or rendering dependency. A host explicitly supplies each requested tick. */
export class ExperimentSimulationSession<T> {
  private paused = true;
  private busy = false;
  private faulted = false;
  private sequence = 0;

  constructor(
    private readonly physics: ExperimentPhysicsPort<T>,
    readonly stepDtSeconds: number,
    private readonly motors: Readonly<Record<string, ExperimentMotor>>
  ) {
    if (!Number.isFinite(stepDtSeconds) || stepDtSeconds <= 0) {
      throw new Error('The physical timestep must be finite and positive.');
    }
    for (const motor of Object.values(motors)) {
      if (!Number.isFinite(motor.minRadians) || !Number.isFinite(motor.maxRadians)
        || motor.minRadians > motor.maxRadians) {
        throw new Error('Motor limits must be finite and ordered.');
      }
    }
    // The configuration cannot change underneath an admitted experiment.
    this.motors = structuredClone(motors);
  }

  pause(): void { this.paused = true; }

  resume(): void {
    this.assertHealthy();
    this.paused = false;
  }

  getStatus(): { paused: boolean; busy: boolean; faulted: boolean; sequence: number } {
    return { paused: this.paused, busy: this.busy, faulted: this.faulted, sequence: this.sequence };
  }

  setMotorTarget(jointName: string, radians: number): void {
    this.assertHealthy();
    if (this.busy) throw new Error('Wait for the current physical step before changing targets.');
    const motor = Object.prototype.hasOwnProperty.call(this.motors, jointName) ? this.motors[jointName] : undefined;
    if (!motor) throw new Error(`No physical motor is configured for ${jointName}.`);
    if (!Number.isFinite(radians) || radians < motor.minRadians || radians > motor.maxRadians) {
      throw new Error(`Target is outside the motor limits for ${jointName}.`);
    }
    this.physics.setJointTarget(jointName, { mode: 'position', value: radians });
  }

  /** A paused host does no work; explicit single-step remains available. */
  async tick(): Promise<ExperimentObservation<T> | null> {
    this.assertHealthy();
    return this.paused ? null : this.advanceOneStep();
  }

  async advanceOneStep(): Promise<ExperimentObservation<T>> {
    this.assertHealthy();
    if (this.busy) throw new Error('A physical step is already pending.');
    this.busy = true;
    try {
      const before = this.physics.readObservation();
      this.validateClock(before);
      await this.physics.stepAndWait(this.stepDtSeconds);
      const after = this.physics.readObservation();
      this.validateClock(after);
      const tolerance = Math.max(1e-9, this.stepDtSeconds * 1e-6);
      if (Math.abs(after.simulatedTimeSeconds - before.simulatedTimeSeconds - this.stepDtSeconds) > tolerance
        || after.droppedTimeSeconds !== before.droppedTimeSeconds) {
        throw new Error('Physics did not advance exactly one step without dropping time.');
      }
      const state = structuredClone(after.state);
      this.sequence += 1;
      return { sequence: this.sequence, simulatedTimeSeconds: after.simulatedTimeSeconds, state };
    } catch (error) {
      // An interrupted step may already have mutated physics. Recreate the experiment before retrying.
      this.faulted = true;
      this.paused = true;
      throw error;
    } finally {
      this.busy = false;
    }
  }

  private assertHealthy(): void {
    if (this.faulted) throw new Error('This experiment is faulted; recreate it from a known state.');
  }

  private validateClock(observation: { simulatedTimeSeconds: number; droppedTimeSeconds: number }): void {
    if (!Number.isFinite(observation.simulatedTimeSeconds) || observation.simulatedTimeSeconds < 0
      || !Number.isFinite(observation.droppedTimeSeconds) || observation.droppedTimeSeconds < 0) {
      throw new Error('Physics must provide a finite simulation clock and dropped-time counter.');
    }
  }
}
