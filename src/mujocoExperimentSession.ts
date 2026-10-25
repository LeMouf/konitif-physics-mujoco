import { PhysicsService } from '@konitif/physics';
import type {
  BodyTransform, JointState, PhysicsBackendFactory, PhysicsSubjectSource, PhysicsVector3
} from '@konitif/physics';
import { ExperimentSimulationSession, type ExperimentMotor } from './experimentSimulationSession.js';

export interface MujocoExperimentState {
  sourceId: string;
  jointStates: JointState[];
  bodyTransforms: BodyTransform[];
  centerOfMass: PhysicsVector3 | null;
}

export interface MujocoExperimentOptions {
  source: PhysicsSubjectSource;
  /** Interval between controller updates and observable snapshots. */
  stepDtSeconds: number;
  /** Fixed MuJoCo solver timestep. Defaults to the observable step interval. */
  physicsStepDtSeconds?: number;
  motors: Readonly<Record<string, ExperimentMotor>>;
  initialJointPositions?:Readonly<Record<string,number>>;
  backendFactory: PhysicsBackendFactory;
}

export interface MujocoExperimentTiming {
  stepDtSeconds: number;
  physicsStepDtSeconds: number;
  maxSubsteps: number;
}

export function resolveMujocoExperimentTiming(
  stepDtSeconds: number,
  physicsStepDtSeconds = stepDtSeconds
): MujocoExperimentTiming {
  if (!Number.isFinite(stepDtSeconds) || stepDtSeconds < 1 / 1000 || stepDtSeconds > 1 / 30) {
    throw new Error('The experiment step interval must be within the MuJoCo worker range.');
  }
  if (!Number.isFinite(physicsStepDtSeconds) || physicsStepDtSeconds < 1 / 1000
    || physicsStepDtSeconds > 1 / 30) {
    throw new Error('The experiment physics timestep must be within the MuJoCo worker range.');
  }
  const ratio = stepDtSeconds / physicsStepDtSeconds;
  const maxSubsteps = Math.round(ratio);
  if (maxSubsteps < 1 || maxSubsteps > 24 || Math.abs(ratio - maxSubsteps) > 1e-9) {
    throw new Error('The experiment step interval must contain between 1 and 24 whole physics substeps.');
  }
  return { stepDtSeconds, physicsStepDtSeconds, maxSubsteps };
}

/** Own a dedicated service, so a viewer cannot also advance the experiment's clock. */
export async function createMujocoExperimentSession(options: MujocoExperimentOptions) {
  const source = structuredClone(options.source);
  const motors = structuredClone(options.motors);
  if(options.initialJointPositions) {
    for(const [name,value] of Object.entries(options.initialJointPositions)) {
      const motor=motors[name];
      if(!motor||!Number.isFinite(value)||value<motor.minRadians||value>motor.maxRadians)
        throw new Error(`Invalid experiment initial joint position for ${name}.`);
    }
    source.metadata={...source.metadata,experimentInitialJointPositions:structuredClone(options.initialJointPositions)};
  }
  const timing = resolveMujocoExperimentTiming(options.stepDtSeconds, options.physicsStepDtSeconds);
  const { stepDtSeconds, physicsStepDtSeconds, maxSubsteps } = timing;
  if (source.kind !== 'mjcf') throw new Error('A MuJoCo experiment requires an MJCF source.');
  const indices = source.metadata?.actuatorCtrlIndex;
  if (!indices || typeof indices !== 'object') throw new Error('The source must declare its physical actuators.');
  const actuatorIndices = indices as Record<string, unknown>;
  const jointIndices = source.metadata?.jointQposIndex;
  const observedJointNames = jointIndices && typeof jointIndices === 'object'
    ? Object.keys(jointIndices) : Object.keys(motors);
  for (const jointName of Object.keys(motors)) {
    const index = Object.prototype.hasOwnProperty.call(actuatorIndices, jointName) ? actuatorIndices[jointName] : undefined;
    if (typeof index !== 'number' || !Number.isInteger(index) || index < 0) {
      throw new Error(`No physical actuator is mapped for ${jointName}.`);
    }
  }
  const service = new PhysicsService({ enabled: true, backendFactories: { mujoco: options.backendFactory } });
  let disposed = false;
  const readObservation = () => {
    if (disposed) throw new Error('The MuJoCo experiment has been disposed.');
    const snapshot = service.snapshot();
    const backend = snapshot.backendStatus;
    const metadata = backend?.metadata;
    if (!snapshot.enabled || !snapshot.initialized || snapshot.engine !== 'mujoco'
      || snapshot.loadedSourceId !== source.id || backend?.engine !== 'mujoco'
      || backend.loadedSourceId !== source.id || !backend.initialized) {
      throw new Error('The experiment physics source is not ready.');
    }
    if (metadata?.stepDtSeconds !== physicsStepDtSeconds) {
      throw new Error('The backend physical timestep differs from the experiment.');
    }
    if (typeof metadata.simulatedTime !== 'number' || typeof metadata.droppedSimulationTime !== 'number') {
      throw new Error('The MuJoCo backend must expose its simulation clock.');
    }
    return {
      simulatedTimeSeconds: metadata.simulatedTime,
      droppedTimeSeconds: metadata.droppedSimulationTime,
      state: {
        sourceId: source.id,
        jointStates: observedJointNames.map(name => service.getJointState(name)),
        bodyTransforms: service.getBodyTransforms(),
        centerOfMass: service.getCenterOfMass()
      } satisfies MujocoExperimentState
    };
  };
  const session = new ExperimentSimulationSession({
    async stepAndWait(dt) { await service.stepAndWait(dt); },
    setJointTarget(name, target) {
      readObservation();
      service.setJointTarget(name, target);
    },
    readObservation
  }, stepDtSeconds, motors);
  try {
    await service.setEngine('mujoco');
    service.setRuntimeConfig({ stepDtSeconds: physicsStepDtSeconds, maxSubsteps });
    await service.loadSubject(source);
    await service.synchronize();
    const initial = readObservation();
    if (initial.simulatedTimeSeconds !== 0 || initial.droppedTimeSeconds !== 0) {
      throw new Error('A new experiment must start from a fresh simulation clock.');
    }
    return {
      session,
      initialState: structuredClone(initial.state),
      dispose() {
        if (session.getStatus().busy) throw new Error('Wait for the pending physical step before disposing.');
        session.pause();
        disposed = true;
        service.dispose();
      }
    };
  } catch (error) {
    service.dispose();
    throw error;
  }
}
