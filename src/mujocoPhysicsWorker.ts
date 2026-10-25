import type {
  BodyTransform,
  JointState,
  JointTarget,
  PhysicsBackendStatus,
  PhysicsBodyImpulse,
  PhysicsMetadata,
  PhysicsQuaternion,
  PhysicsRootPose,
  PhysicsRuntimeConfig,
  PhysicsVector3,
  PhysicsSubjectSource
} from '@konitif/physics';
import type { MujocoPhysicsInitOptions } from './mujocoInitOptions.js';
import { resolveWorkerSourceMetadata, type MujocoWorkerSourceMetadata } from './mujocoWorkerSourceMetadata.js';
import type {
  PhysicsWorkerRequest,
  PhysicsWorkerResponse,
  PhysicsWorkerStateMessage,
  PhysicsWorkerStatePayload
} from '@konitif/physics/worker';
import { applyMujocoTimestepToXml, scheduleMujocoSubsteps } from './mujocoStepScheduler.js';
import { readMujocoGroundContacts } from './mujocoGroundContacts.js';
import {
  MUJOCO_SUPPORT_GROUND_GAP_METERS,
  resolveMujocoSupportFloorHeight,
  type MujocoSupportBodyObservation
} from './mujocoSupportFloor.js';

type MujocoModule = {
  MjModel?: {
    from_xml_string?: (xml: string) => MujocoModel;
    fromXMLString?: (xml: string) => MujocoModel;
    loadFromXML?: (path: string) => MujocoModel;
  };
  MjData?: new (model: MujocoModel) => MujocoData;
  FS?: {
    mkdir?: (path: string) => void;
    writeFile?: (path: string, data: string | Uint8Array) => void;
  };
  mj_step?: (model: MujocoModel, data: MujocoData) => void;
  mj_forward?: (model: MujocoModel, data: MujocoData) => void;
};

type MujocoModuleFactoryOptions = {
  locateFile?: (path: string, scriptDirectory: string) => string;
};

type MujocoModel = Record<string, unknown> & {
  nbody?: number;
  body_mass?: ArrayLike<number>;
  delete?: () => void;
};

type MujocoData = Record<string, unknown> & {
  xpos?: ArrayLike<number>;
  xipos?: ArrayLike<number>;
  xquat?: ArrayLike<number>;
  qpos?: ArrayLike<number>;
  qvel?: ArrayLike<number>;
  qacc?: ArrayLike<number>;
  qacc_warmstart?: ArrayLike<number>;
  ctrl?: ArrayLike<number>;
  act?: ArrayLike<number>;
  qfrc_applied?: ArrayLike<number>;
  xfrc_applied?: ArrayLike<number>;
  subtree_com?: ArrayLike<number>;
  delete?: () => void;
};

let mujoco: MujocoModule | null = null;
let mujocoModuleUrl: string | null = null;
let mujocoModuleUrlCandidates: string[] = [];
let model: MujocoModel | null = null;
let data: MujocoData | null = null;
let sourceMetadata: MujocoWorkerSourceMetadata = {};
let loadedSourceId: string | null = null;
let stepCount = 0;
let simulatedTime = 0;
let simulationStepAccumulator = 0;
let droppedSimulationTime = 0;
let lastRequestedStepDelta = 0;
let lastAdvancedStepDelta = 0;
let runtimeConfig = resolveRuntimeConfig({});
const jointTargets = new Map<string, JointTarget>();
const pendingBodyImpulses: PhysicsBodyImpulse[] = [];

self.addEventListener('message', (event: MessageEvent<PhysicsWorkerRequest>) => {
  const request = event.data;

  void handleRequest(request).catch((error: unknown) => {
    postResponse({
      id: request.id,
      ok: false,
      error: formatWorkerError(error)
    });
  });
});

function formatWorkerError(error: unknown): string {
  const message = error instanceof Error && error.message ? error.message : String(error || '');
  const name = error instanceof Error && error.name ? error.name : isRecord(error) && typeof error.constructor?.name === 'string' ? error.constructor.name : '';
  const excPtr = isRecord(error) && typeof error.excPtr === 'number' ? ` excPtr=${error.excPtr}` : '';

  return [name, message].filter((part) => part.length > 0).join(': ') + excPtr || 'Unknown worker error.';
}

async function handleRequest(request: PhysicsWorkerRequest): Promise<void> {
  switch (request.command) {
    case 'init':
      configureMujocoModule(request.payload);
      await loadMujocoModule();
      postResponse({ id: request.id, ok: true, payload: createStatePayload() });
      break;
    case 'loadSubject':
      await loadPhysicsSubjectSource(request.payload as PhysicsSubjectSource);
      postResponse({ id: request.id, ok: true, payload: createStatePayload() });
      break;
    case 'step':
      stepSimulation(readNumberPayload(request.payload, 'dt'));
      publishState();
      break;
    case 'stepAndWait':
      stepSimulation(readNumberPayload(request.payload, 'dt'));
      postResponse({ id: request.id, ok: true, payload: createStatePayload() });
      break;
    case 'synchronize':
      // Joint targets are deliberately queued without running mj_forward for
      // every individual motor command. A barrier must still expose their
      // resulting kinematic state when no simulation step follows them.
      applyJointTargets();
      postResponse({ id: request.id, ok: true, payload: createStatePayload() });
      break;
    case 'resetSimulation':
      resetSimulation();
      postResponse({ id: request.id, ok: true, payload: createStatePayload() });
      break;
    case 'setJointTarget':
      applyJointTargetPayload(request.payload);
      break;
    case 'applyBodyImpulse':
      queueBodyImpulse(request.payload);
      break;
    case 'clearJointTargets':
      clearJointTargets();
      break;
    case 'releaseJointTargets':
      releaseJointTargets(request.payload);
      break;
    case 'syncKinematicPose':
      syncKinematicPose(request.payload);
      break;
    case 'setRuntimeConfig':
      setRuntimeConfig(request.payload);
      break;
    case 'dispose':
      disposeSimulation();
      postResponse({ id: request.id, ok: true });
      break;
  }
}

async function loadMujocoModule(): Promise<void> {
  if (mujoco) {
    return;
  }

  const moduleUrlCandidates = resolveMujocoModuleUrlCandidates();
  const errors: string[] = [];
  let imported: Record<string, unknown> | null = null;

  for (const moduleUrl of moduleUrlCandidates) {
    try {
      const readiness = await inspectMujocoModuleUrl(moduleUrl);

      if (!readiness.available) {
        errors.push(`${moduleUrl}: ${readiness.reason}`);
        continue;
      }

      imported = (await importBrowserModule(moduleUrl)) as Record<string, unknown>;
      mujocoModuleUrl = moduleUrl;
      break;
    } catch (error) {
      errors.push(`${moduleUrl}: ${error instanceof Error ? error.message : String(error)}`);
    }
  }

  if (!imported) {
    const configurationHint =
      'Supply moduleUrl in the provider init options with a compatible MuJoCo JavaScript module.';
    throw new Error(
      errors.length > 0
        ? `MuJoCo WASM module is unavailable. ${configurationHint} Tried: ${errors.join(' | ')}`
        : `MuJoCo WASM module URL is not configured. ${configurationHint}`
    );
  }

  const resolvedModule = (typeof imported.default === 'function'
    ? await imported.default(createMujocoModuleFactoryOptions(mujocoModuleUrl))
    : imported.default ?? imported) as MujocoModule;

  if (!resolvedModule.MjModel || !resolvedModule.MjData || typeof resolvedModule.mj_step !== 'function') {
    throw new Error('MuJoCo WASM module loaded, but its JavaScript API is not compatible with this adapter.');
  }

  mujoco = resolvedModule;
}

function configureMujocoModule(payload: unknown): void {
  if (!isRecord(payload)) {
    return;
  }

  const options = payload as MujocoPhysicsInitOptions;
  const nextModuleUrl = typeof options.moduleUrl === 'string' ? options.moduleUrl.trim() : '';
  const nextModuleUrlCandidates = Array.isArray(options.moduleUrlCandidates)
    ? options.moduleUrlCandidates.filter((candidate): candidate is string => typeof candidate === 'string').map((candidate) => candidate.trim())
    : [];

  if (nextModuleUrl.length > 0) {
    mujocoModuleUrl = nextModuleUrl;
  }

  mujocoModuleUrlCandidates = uniqueStrings([
    ...nextModuleUrlCandidates.filter((candidate) => candidate.length > 0),
    ...(nextModuleUrl.length > 0 ? [nextModuleUrl] : [])
  ]);
}

function resolveMujocoModuleUrlCandidates(): string[] {
  return uniqueStrings([
    ...mujocoModuleUrlCandidates,
    ...(mujocoModuleUrl ? [mujocoModuleUrl] : [])
  ]);
}

function uniqueStrings(values: string[]): string[] {
  return Array.from(new Set(values));
}

async function inspectMujocoModuleUrl(moduleUrl: string): Promise<{ available: boolean; reason: string }> {
  if (!isBrowserFetchableModuleUrl(moduleUrl)) {
    return { available: true, reason: 'not preflighted' };
  }

  try {
    const response = await fetch(moduleUrl, {
      method: 'HEAD',
      cache: 'no-store'
    });

    if (!response.ok) {
      return { available: false, reason: `HTTP ${response.status}` };
    }

    const contentType = response.headers.get('content-type')?.toLowerCase() ?? '';

    if (contentType && !isJavaScriptContentType(contentType)) {
      return { available: false, reason: `unexpected content-type "${contentType}"` };
    }

    return { available: true, reason: 'available' };
  } catch (error) {
    return {
      available: false,
      reason: error instanceof Error ? error.message : String(error)
    };
  }
}

function isBrowserFetchableModuleUrl(moduleUrl: string): boolean {
  return (
    moduleUrl.startsWith('/') ||
    moduleUrl.startsWith('./') ||
    moduleUrl.startsWith('../') ||
    moduleUrl.startsWith('http://') ||
    moduleUrl.startsWith('https://')
  );
}

function isJavaScriptContentType(contentType: string): boolean {
  return (
    contentType.includes('javascript') ||
    contentType.includes('ecmascript') ||
    contentType.includes('text/plain') ||
    contentType.includes('application/octet-stream')
  );
}

function importBrowserModule(moduleUrl: string): Promise<unknown> {
  const dynamicImport = new Function('moduleUrl', 'return import(moduleUrl)') as (moduleUrl: string) => Promise<unknown>;

  return dynamicImport(moduleUrl);
}

function createMujocoModuleFactoryOptions(moduleUrl: string | null): MujocoModuleFactoryOptions {
  return {
    locateFile: (path: string, scriptDirectory: string) => {
      if (!moduleUrl) {
        return `${scriptDirectory}${path}`;
      }

      return new URL(path, new URL(moduleUrl, self.location.href)).toString();
    }
  };
}

async function loadPhysicsSubjectSource(source: PhysicsSubjectSource): Promise<void> {
  await loadMujocoModule();
  disposeSimulation();

  const xml = source.sourceText ?? (source.url ? await fetchText(source.url) : null);

  if (!xml) {
    throw new Error(`MuJoCo source "${source.id}" does not provide sourceText or url.`);
  }

  await loadSubjectAssets(source);
  const nextModel = createModelFromXml(applyMujocoTimestepToXml(xml, runtimeConfig.stepDtSeconds));
  const DataConstructor = mujoco?.MjData;

  if (!DataConstructor) {
    throw new Error('MuJoCo MjData constructor is unavailable.');
  }

  model = nextModel;
  data = new DataConstructor(nextModel);
  sourceMetadata = resolveWorkerSourceMetadata(source.metadata);
  loadedSourceId = source.id;
  resetFreeJointPose();
  // An experiment may author its initial state once, before any physical timestep.
  const initialPositions=source.metadata?.experimentInitialJointPositions;
  if(isRecord(initialPositions)) {
    const targets:Record<string,JointTarget>={};
    for(const [name,value] of Object.entries(initialPositions)) {
      if(typeof value!=='number'||!Number.isFinite(value)||sourceMetadata.jointQposIndex?.[name]===undefined)
        throw new Error('Invalid experiment initial joint configuration.');
      targets[name]={mode:'position',value};
    }
    applyJointPoseTargets(targets);restoreJointTargets(targets);
  }
  settleRootHeightToSupportFloor();
  clearSimulationDynamics();
  forwardSimulation();
}

async function loadSubjectAssets(source:PhysicsSubjectSource):Promise<void>{
 const entries=source.metadata?.assets;if(entries===undefined)return;
 if(!Array.isArray(entries)||entries.length>128)throw Error('Invalid MuJoCo asset manifest.');
 const fs=mujoco?.FS;if(!fs?.writeFile)throw Error('MuJoCo filesystem is unavailable.');
 let total=0;const seen=new Set<string>();
 for(const entry of entries){
  if(!isRecord(entry)||typeof entry.path!=='string'||typeof entry.url!=='string'||!entry.path||entry.path.startsWith('/')||entry.path.includes('\\')||entry.path.split('/').some(p=>!p||p==='.'||p==='..')||seen.has(entry.path))throw Error('Invalid MuJoCo asset path.');
  seen.add(entry.path);const response=await fetch(entry.url);if(!response.ok)throw Error(`MuJoCo asset unavailable: ${entry.path} (${response.status}).`);
  const bytes=new Uint8Array(await response.arrayBuffer());total+=bytes.byteLength;if(total>32*1024*1024)throw Error('MuJoCo asset budget exceeded.');
  const parts=('/workbench/'+entry.path).split('/');for(let i=2;i<parts.length;i++){try{fs.mkdir?.(parts.slice(0,i).join('/'));}catch{}}
  fs.writeFile('/workbench/'+entry.path,bytes);
 }
}

function createModelFromXml(xml: string): MujocoModel {
  const modelApi = mujoco?.MjModel;
  const fromXml = modelApi?.from_xml_string ?? modelApi?.fromXMLString;

  if (!fromXml) {
    const loadFromXML = modelApi?.loadFromXML;

    if (!loadFromXML) {
      throw new Error('MuJoCo MjModel XML loader API is unavailable.');
    }

    const xmlPath = `/workbench/model-${Date.now()}.xml`;
    writeMujocoXmlFile(xmlPath, xml);

    return loadFromXML(xmlPath);
  }

  return fromXml(xml);
}

function writeMujocoXmlFile(path: string, xml: string): void {
  const fileSystem = mujoco?.FS;

  if (!fileSystem?.writeFile) {
    throw new Error('MuJoCo FS.writeFile API is unavailable.');
  }

  try {
    fileSystem.mkdir?.('/workbench');
  } catch {
    // Emscripten throws when the directory already exists.
  }

  fileSystem.writeFile(path, xml);
}

function stepSimulation(dt: number): void {
  if (!mujoco || !model || !data) {
    return;
  }

  const schedule = scheduleMujocoSubsteps({
    requestedDeltaSeconds: dt,
    accumulatorSeconds: simulationStepAccumulator,
    stepDtSeconds: runtimeConfig.stepDtSeconds,
    maxSubsteps: runtimeConfig.maxSubsteps
  });
  simulationStepAccumulator = schedule.accumulatorSeconds;
  droppedSimulationTime += schedule.droppedDeltaSeconds;
  lastRequestedStepDelta = schedule.requestedDeltaSeconds;
  lastAdvancedStepDelta = schedule.advancedDeltaSeconds;

  if (schedule.steps === 0) {
    return;
  }

  applyJointTargets();

  for (let index = 0; index < schedule.steps; index += 1) {
    const appliedBodyForceBaselines = applyPendingBodyImpulses(runtimeConfig.stepDtSeconds);
    mujoco.mj_step?.(model, data);
    restoreAppliedBodyForces(appliedBodyForceBaselines);
  }

  applyJointTargets();

  stepCount += schedule.steps;
  simulatedTime += schedule.advancedDeltaSeconds;
}

function queueBodyImpulse(payload: unknown): void {
  if (!isRecord(payload) || !isRecord(payload.impulse)) {
    return;
  }

  const candidate = payload.impulse;
  if (typeof candidate.bodyName !== 'string' || !isFiniteVector(candidate.impulse)) {
    return;
  }

  pendingBodyImpulses.push({
    bodyName: candidate.bodyName,
    impulse: { ...candidate.impulse },
    ...(isFiniteVector(candidate.point) ? { point: { ...candidate.point } } : {})
  });
}

function applyPendingBodyImpulses(
  stepDtSeconds: number
): Array<{ offset: number; values: number[] }> {
  if (!data || pendingBodyImpulses.length === 0 || stepDtSeconds <= 0) {
    return [];
  }

  const forces = toMutableNumberArray(data.xfrc_applied);
  const positions = toReadonlyNumberArray(data.xpos);
  const bodyNames = sourceMetadata.bodyNames;
  if (!forces || !bodyNames) {
    pendingBodyImpulses.length = 0;
    return [];
  }

  const baselineByOffset = new Map<number, number[]>();

  for (const bodyImpulse of pendingBodyImpulses.splice(0)) {
    const bodyIndex = bodyNames.indexOf(bodyImpulse.bodyName);
    const offset = bodyIndex * 6;
    if (bodyIndex < 0 || forces.length < offset + 6) {
      continue;
    }
    if (!baselineByOffset.has(offset)) {
      baselineByOffset.set(
        offset,
        Array.from({ length: 6 }, (_, index) => forces[offset + index] ?? 0)
      );
    }

    const force = {
      x: bodyImpulse.impulse.x / stepDtSeconds,
      y: bodyImpulse.impulse.y / stepDtSeconds,
      z: bodyImpulse.impulse.z / stepDtSeconds
    };
    forces[offset] += force.x;
    forces[offset + 1] += force.y;
    forces[offset + 2] += force.z;

    if (bodyImpulse.point && positions && positions.length >= bodyIndex * 3 + 3) {
      const arm = {
        x: bodyImpulse.point.x - positions[bodyIndex * 3],
        y: bodyImpulse.point.y - positions[bodyIndex * 3 + 1],
        z: bodyImpulse.point.z - positions[bodyIndex * 3 + 2]
      };
      forces[offset + 3] += arm.y * force.z - arm.z * force.y;
      forces[offset + 4] += arm.z * force.x - arm.x * force.z;
      forces[offset + 5] += arm.x * force.y - arm.y * force.x;
    }
  }
  return [...baselineByOffset].map(([offset, values]) => ({ offset, values }));
}

function restoreAppliedBodyForces(
  baselines: readonly { offset: number; values: readonly number[] }[]
): void {
  const forces = toMutableNumberArray(data?.xfrc_applied);
  if (!forces) return;
  for (const baseline of baselines) {
    baseline.values.forEach((value, index) => {
      forces[baseline.offset + index] = value;
    });
  }
}

function isFiniteVector(value: unknown): value is PhysicsVector3 {
  return (
    isRecord(value) &&
    typeof value.x === 'number' &&
    Number.isFinite(value.x) &&
    typeof value.y === 'number' &&
    Number.isFinite(value.y) &&
    typeof value.z === 'number' &&
    Number.isFinite(value.z)
  );
}

function applyJointTargets(): void {
  if (!data || jointTargets.size === 0) {
    return;
  }

  const qpos = toMutableNumberArray(data.qpos);
  const qvel = toMutableNumberArray(data.qvel);
  const ctrl = toMutableNumberArray(data.ctrl);

  let requiresForward = false;

  for (const [jointName, target] of jointTargets) {
    const mode = target.mode ?? 'position';
    const ctrlIndex = sourceMetadata.actuatorCtrlIndex?.[jointName];
    const qposIndex = sourceMetadata.jointQposIndex?.[jointName];
    const qvelIndex = sourceMetadata.jointQvelIndex?.[jointName];

    if (mode === 'position' && typeof target.value === 'number' && ctrl && ctrlIndex !== undefined) {
      ctrl[ctrlIndex] = target.value;
      // mj_step consumes actuator controls directly. Running mj_forward here
      // would solve the same model twice around every buffered frame without
      // changing the observable qpos/qvel state.
      continue;
    }

    if (mode === 'position' && typeof target.value === 'number' && qpos && qposIndex !== undefined) {
      qpos[qposIndex] = target.value;
      requiresForward = true;
    }

    if (qvel && qvelIndex !== undefined && (typeof target.velocity === 'number' || typeof target.value === 'number')) {
      qvel[qvelIndex] = target.velocity ?? 0;
      requiresForward = true;
    }
  }

  if (requiresForward) {
    forwardSimulation();
  }
}

function applyJointTargetPayload(payload: unknown): void {
  if (!isRecord(payload) || typeof payload.jointName !== 'string' || !isRecord(payload.target)) {
    return;
  }

  jointTargets.set(payload.jointName, payload.target as JointTarget);
  // A Viewer frame sends one command per animated joint before its explicit
  // step/barrier. Applying and publishing here used to run a progressively
  // larger target set (and serialize the complete MuJoCo state) once per
  // joint. Keep the commands ordered in the worker and collapse that work at
  // the next stepSimulation()/synchronize() boundary.
}

function clearJointTargets(): void {
  if (jointTargets.size === 0) {
    return;
  }

  jointTargets.clear();
  publishState();
}

function releaseJointTargets(payload: unknown): void {
  const releaseTargets = resolveJointTargetRecord(isRecord(payload) ? payload.targets : undefined);

  jointTargets.clear();
  resetFreeJointPose();
  applyJointPoseTargets(releaseTargets);
  settleRootHeightToSupportFloor();
  clearSimulationDynamics();
  retainReleaseLockedJointTargets(releaseTargets);
  forwardSimulation();
  publishState();
}

function syncKinematicPose(payload: unknown): void {
  const targets = resolveJointTargetRecord(isRecord(payload) ? payload.targets : undefined);
  const options = isRecord(payload) && isRecord(payload.options) ? payload.options : {};
  const rootPose = resolvePhysicsRootPose(options.rootPose);

  if (rootPose) {
    applyFreeJointPose(rootPose);
  } else if (options.resetRoot !== false) {
    resetFreeJointPose();
  }

  applyJointPoseTargets(targets);
  if (!rootPose) {
    settleRootHeightToSupportFloor();
  }

  if (options.clearDynamics !== false) {
    clearSimulationDynamics();
  }

  if (options.retainTargets !== false) {
    restoreJointTargets(targets);
  }

  forwardSimulation();

  if (options.clearDynamics !== false) {
    clearSimulationDynamics();
  }

  publishState();
}

function resolvePhysicsRootPose(value: unknown): PhysicsRootPose | null {
  if (!isRecord(value) || !isRecord(value.position) || !isRecord(value.rotation)) {
    return null;
  }

  const position = value.position;
  const rotation = value.rotation;
  const components = [
    position.x,
    position.y,
    position.z,
    rotation.x,
    rotation.y,
    rotation.z,
    rotation.w
  ];

  if (!components.every((component) => typeof component === 'number' && Number.isFinite(component))) {
    return null;
  }

  return {
    position: { x: position.x as number, y: position.y as number, z: position.z as number },
    rotation: {
      x: rotation.x as number,
      y: rotation.y as number,
      z: rotation.z as number,
      w: rotation.w as number
    }
  };
}

function resolveJointTargetRecord(value: unknown): Record<string, JointTarget> {
  if (!isRecord(value)) {
    return {};
  }

  const targets: Record<string, JointTarget> = {};

  for (const [jointName, target] of Object.entries(value)) {
    if (!isRecord(target)) {
      continue;
    }

    targets[jointName] = target as JointTarget;
  }

  return targets;
}

function applyJointPoseTargets(targets: Record<string, JointTarget>): void {
  if (!data) {
    return;
  }

  const qpos = toMutableNumberArray(data.qpos);
  const qvel = toMutableNumberArray(data.qvel);
  let didMutate = false;

  for (const [jointName, target] of Object.entries(targets)) {
    const qposIndex = sourceMetadata.jointQposIndex?.[jointName];
    const qvelIndex = sourceMetadata.jointQvelIndex?.[jointName];

    if (qpos && qposIndex !== undefined && typeof target.value === 'number' && Number.isFinite(target.value)) {
      qpos[qposIndex] = target.value;
      didMutate = true;
    }

    if (qvel && qvelIndex !== undefined) {
      qvel[qvelIndex] = typeof target.velocity === 'number' && Number.isFinite(target.velocity) ? target.velocity : 0;
      didMutate = true;
    }
  }

  if (didMutate) {
    forwardSimulation();
  }
}

function retainReleaseLockedJointTargets(targets: Record<string, JointTarget>): void {
  jointTargets.clear();

  if (!sourceMetadata.releaseLockedJointNames || sourceMetadata.releaseLockedJointNames.size === 0) {
    return;
  }

  for (const jointName of sourceMetadata.releaseLockedJointNames) {
    const target = targets[jointName];

    if (!target) {
      continue;
    }

    jointTargets.set(jointName, { ...target, velocity: 0 });
  }
}

function resetSimulation(): void {
  if (!model) {
    return;
  }

  const DataConstructor = mujoco?.MjData;

  if (!DataConstructor) {
    return;
  }

  const retainedTargets = Object.fromEntries(jointTargets);
  pendingBodyImpulses.length = 0;

  data?.delete?.();
  data = new DataConstructor(model);
  jointTargets.clear();
  stepCount = 0;
  simulatedTime = 0;
  simulationStepAccumulator = 0;
  droppedSimulationTime = 0;
  lastRequestedStepDelta = 0;
  lastAdvancedStepDelta = 0;
  resetFreeJointPose();
  applyJointPoseTargets(retainedTargets);
  settleRootHeightToSupportFloor();
  restoreJointTargets(retainedTargets);
  clearSimulationDynamics();
  forwardSimulation();
  clearSimulationDynamics();
  publishState();
}

function forwardSimulation(): void {
  if (!mujoco || !model || !data) {
    return;
  }

  mujoco.mj_forward?.(model, data);
}

function settleRootHeightToSupportFloor(): void {
  if (!data || !sourceMetadata.supportBodyNames || sourceMetadata.supportBodyNames.size === 0) {
    return;
  }

  const qpos = data.qpos as { [index: number]: number; length: number } | undefined;

  if (!qpos || qpos.length < 3) {
    return;
  }

  forwardSimulation();

  const positions = toReadonlyNumberArray(data.xpos);
  const rotations = toReadonlyNumberArray(data.xquat);
  const bodyNames = sourceMetadata.bodyNames;

  if (!positions || !rotations || !bodyNames) {
    return;
  }

  const observations: MujocoSupportBodyObservation[] = [];

  for (const supportBodyName of sourceMetadata.supportBodyNames) {
    const bodyIndex = bodyNames.indexOf(supportBodyName);

    if (bodyIndex < 0) {
      continue;
    }

    const position = resolveBodyPosition(positions, bodyIndex);
    const rotation = resolveBodyRotation(rotations, bodyIndex);

    if (!position || !rotation) {
      continue;
    }

    observations.push({
      bodyName: supportBodyName,
      position,
      rotation,
      colliders: sourceMetadata.bodyColliderProxies?.[supportBodyName],
      fallbackGroundClearance: sourceMetadata.supportBodyGroundClearance?.[supportBodyName]
    });
  }

  const supportFloorHeight = resolveMujocoSupportFloorHeight(observations);

  if (supportFloorHeight === null) {
    return;
  }

  qpos[2] += MUJOCO_SUPPORT_GROUND_GAP_METERS - supportFloorHeight;
  forwardSimulation();
}

function resolveBodyPosition(positions: ArrayLike<number>, bodyIndex: number): PhysicsVector3 | null {
  const position = {
    x: positions[bodyIndex * 3],
    y: positions[bodyIndex * 3 + 1],
    z: positions[bodyIndex * 3 + 2]
  };

  return [position.x, position.y, position.z].every(Number.isFinite) ? position : null;
}

function resolveBodyRotation(rotations: ArrayLike<number>, bodyIndex: number): PhysicsQuaternion | null {
  const rotation = {
    w: rotations[bodyIndex * 4],
    x: rotations[bodyIndex * 4 + 1],
    y: rotations[bodyIndex * 4 + 2],
    z: rotations[bodyIndex * 4 + 3]
  };

  return [rotation.x, rotation.y, rotation.z, rotation.w].every(Number.isFinite) ? rotation : null;
}

function resetFreeJointPose(): void {
  const qpos = data?.qpos as { [index: number]: number; length: number } | undefined;

  if (!qpos || qpos.length < 7) {
    return;
  }

  qpos[0] = 0;
  qpos[1] = 0;
  qpos[3] = 1;
  qpos[4] = 0;
  qpos[5] = 0;
  qpos[6] = 0;
  forwardSimulation();
}

function applyFreeJointPose(rootPose: PhysicsRootPose): void {
  const qpos = data?.qpos as { [index: number]: number; length: number } | undefined;

  if (!qpos || qpos.length < 7) {
    return;
  }

  qpos[0] = rootPose.position.x;
  qpos[1] = rootPose.position.y;
  qpos[2] = rootPose.position.z;
  qpos[3] = rootPose.rotation.w;
  qpos[4] = rootPose.rotation.x;
  qpos[5] = rootPose.rotation.y;
  qpos[6] = rootPose.rotation.z;
  forwardSimulation();
}

function clearSimulationDynamics(): void {
  clearNumberArray(data?.qvel);
  clearNumberArray(data?.qacc);
  clearNumberArray(data?.qacc_warmstart);
  clearNumberArray(data?.ctrl);
  clearNumberArray(data?.act);
  clearNumberArray(data?.qfrc_applied);
  clearNumberArray(data?.xfrc_applied);
}

function restoreJointTargets(targets: Record<string, JointTarget>): void {
  jointTargets.clear();

  for (const [jointName, target] of Object.entries(targets)) {
    jointTargets.set(jointName, { ...target });
  }
}

function setRuntimeConfig(payload: unknown): void {
  runtimeConfig = resolveRuntimeConfig(isRecord(payload) ? payload.config : payload);
}

function resolveRuntimeConfig(value: unknown): Required<Pick<PhysicsRuntimeConfig, 'stepDtSeconds' | 'maxSubsteps'>> {
  const source = isRecord(value) ? value : {};
  const stepDtSeconds =
    typeof source.stepDtSeconds === 'number' && Number.isFinite(source.stepDtSeconds)
      ? Math.min(1 / 30, Math.max(1 / 1000, source.stepDtSeconds))
      : 1 / 240;
  const maxSubsteps =
    typeof source.maxSubsteps === 'number' && Number.isFinite(source.maxSubsteps)
      ? Math.round(Math.min(24, Math.max(1, source.maxSubsteps)))
      : 8;

  return { stepDtSeconds, maxSubsteps };
}

function clearNumberArray(value: unknown): void {
  const array = toMutableNumberArray(value);

  if (!array) {
    return;
  }

  for (let index = 0; index < array.length; index += 1) {
    array[index] = 0;
  }
}

function createStatePayload(): PhysicsWorkerStatePayload {
  const jointStates = createJointStates();
  const bodyTransforms = createBodyTransforms();
  const centerOfMassObservation = resolveCenterOfMassObservation();
  const centerOfMass = centerOfMassObservation.value;

  return {
    jointStates,
    bodyTransforms,
    centerOfMass,
    status: createBackendStatus({
      jointStates,
      bodyTransforms,
      centerOfMass,
      centerOfMassSource: centerOfMassObservation.source,
      centerOfMassCrossCheckErrorMeters: centerOfMassObservation.crossCheckErrorMeters
    })
  };
}

function createBackendStatus(payload: {
  jointStates: JointState[];
  bodyTransforms: BodyTransform[];
  centerOfMass: PhysicsVector3 | null;
  centerOfMassSource: 'subtree_com' | 'mass_weighted_xipos' | 'unavailable';
  centerOfMassCrossCheckErrorMeters: number | null;
}): PhysicsBackendStatus {
  const centerOfMassDiagnostics = createCenterOfMassDiagnostics(payload);

  return {
    engine: 'mujoco',
    initialized: mujoco !== null,
    loadedSourceId,
    diagnostics: centerOfMassDiagnostics,
    metadata: {
      moduleUrl: mujocoModuleUrl,
      bodyTransformCount: payload.bodyTransforms.length,
      jointStateCount: payload.jointStates.length,
      centerOfMassAvailable: payload.centerOfMass !== null,
      centerOfMassSource: payload.centerOfMassSource,
      centerOfMassCrossCheckErrorMeters: payload.centerOfMassCrossCheckErrorMeters,
      stepDtSeconds: runtimeConfig.stepDtSeconds,
      maxSubsteps: runtimeConfig.maxSubsteps,
      stepCount,
      simulatedTime,
      simulationStepAccumulator,
      droppedSimulationTime,
      lastRequestedStepDelta,
      lastAdvancedStepDelta
    }
  };
}

function createCenterOfMassDiagnostics(payload: {
  centerOfMass: PhysicsVector3 | null;
  centerOfMassSource: 'subtree_com' | 'mass_weighted_xipos' | 'unavailable';
  centerOfMassCrossCheckErrorMeters: number | null;
}): NonNullable<PhysicsBackendStatus['diagnostics']> {
  const diagnostics: NonNullable<PhysicsBackendStatus['diagnostics']> = [];

  if (payload.centerOfMassSource === 'mass_weighted_xipos') {
    diagnostics.push({
      id: 'mujoco.center-of-mass.fallback',
      severity: 'warning',
      message: 'MuJoCo subtree center of mass is unavailable; using the mass-weighted body fallback.'
    });
  } else if (payload.centerOfMassSource === 'unavailable') {
    diagnostics.push({
      id: 'mujoco.center-of-mass.unavailable',
      severity: 'warning',
      message: 'MuJoCo did not provide a usable center-of-mass observation.'
    });
  }

  if (
    payload.centerOfMassCrossCheckErrorMeters !== null &&
    payload.centerOfMassCrossCheckErrorMeters > 0.002
  ) {
    diagnostics.push({
      id: 'mujoco.center-of-mass.cross-check-mismatch',
      severity: 'warning',
      message: 'MuJoCo subtree and mass-weighted center-of-mass observations differ by more than 2 mm.',
      metadata: {
        errorMeters: payload.centerOfMassCrossCheckErrorMeters
      }
    });
  }

  return diagnostics;
}

function createJointStates(): JointState[] {
  if (!data) {
    return [];
  }

  const qpos = toReadonlyNumberArray(data.qpos);
  const qvel = toReadonlyNumberArray(data.qvel);
  const indexedJointNames = new Set([
    ...Object.keys(sourceMetadata.jointQposIndex ?? {}),
    ...Object.keys(sourceMetadata.jointQvelIndex ?? {})
  ]);
  const jointNames = uniqueStrings([
    ...indexedJointNames,
    ...Array.from(jointTargets.keys()).filter((jointName) => indexedJointNames.has(jointName))
  ]);

  return jointNames.map((jointName) => {
    const qposIndex = sourceMetadata.jointQposIndex?.[jointName];
    const qvelIndex = sourceMetadata.jointQvelIndex?.[jointName];

    return {
      name: jointName,
      value: qposIndex !== undefined ? qpos?.[qposIndex] ?? 0 : 0,
      velocity: qvelIndex !== undefined ? qvel?.[qvelIndex] : undefined
    };
  });
}

function createBodyTransforms(): BodyTransform[] {
  if (!data) {
    return [];
  }

  const positions = toReadonlyNumberArray(data.xpos);
  const rotations = toReadonlyNumberArray(data.xquat);
  const bodyCount = resolveBodyCount(positions);

  if (!positions || !rotations || bodyCount <= 0) {
    return [];
  }

  const transforms: BodyTransform[] = [];

  for (let index = 0; index < bodyCount; index += 1) {
    transforms.push({
      bodyName: sourceMetadata.bodyNames?.[index] ?? `body-${index}`,
      position: {
        x: positions[index * 3] ?? 0,
        y: positions[index * 3 + 1] ?? 0,
        z: positions[index * 3 + 2] ?? 0
      },
      rotation: {
        w: rotations[index * 4] ?? 1,
        x: rotations[index * 4 + 1] ?? 0,
        y: rotations[index * 4 + 2] ?? 0,
        z: rotations[index * 4 + 3] ?? 0
      },
      metadata: createBodyTransformMetadata(sourceMetadata.bodyNames?.[index] ?? `body-${index}`, index)
    });
  }

  return transforms;
}

function createBodyTransformMetadata(bodyName: string, bodyIndex: number): BodyTransform['metadata'] | undefined {
  const parentBodyName = sourceMetadata.bodyParentNames?.[bodyName];
  const metadata: NonNullable<BodyTransform['metadata']> = {};

  if (parentBodyName) {
    metadata.parentBodyName = parentBodyName;
  }

  if (sourceMetadata.coordinateFrame) {
    metadata.coordinateFrame = sourceMetadata.coordinateFrame;
  }

  if (typeof sourceMetadata.sceneYawRadians === 'number') {
    metadata.sceneYawRadians = sourceMetadata.sceneYawRadians;
  }

  if (sourceMetadata.visualRootBodyName === bodyName) {
    metadata.visualRoot = true;
    metadata.groundContacts = readMujocoGroundContacts(model, data);
  }

  if (sourceMetadata.auxiliaryBodyNames?.has(bodyName)) {
    metadata.role = 'auxiliary';
  }

  if (sourceMetadata.supportBodyNames?.has(bodyName)) {
    metadata.support = true;
  }

  if (sourceMetadata.contactBodyNames?.has(bodyName)) {
    metadata.contact = true;
  }

  if (sourceMetadata.debugHiddenBodyNames?.has(bodyName)) {
    metadata.debugVisible = false;
  }

  const colliderProxies = sourceMetadata.bodyColliderProxies?.[bodyName];

  if (colliderProxies && colliderProxies.length > 0) {
    metadata.colliderProxies = colliderProxies as unknown as PhysicsMetadata[];
  }

  const mass = resolveBodyMass(bodyIndex);

  if (mass !== null) {
    metadata.mass = mass;
  }

  const centerOfMass = resolveBodyCenterOfMass(bodyIndex);

  if (centerOfMass) {
    metadata.centerOfMass = centerOfMass as unknown as PhysicsMetadata;
  }

  return Object.keys(metadata).length > 0 ? metadata : undefined;
}

function resolveBodyMass(bodyIndex: number): number | null {
  const bodyMasses = toReadonlyNumberArray(model?.body_mass);

  if (!bodyMasses || bodyIndex < 0 || bodyIndex >= bodyMasses.length) {
    return null;
  }

  const mass = bodyMasses[bodyIndex];

  return typeof mass === 'number' && Number.isFinite(mass) && mass > 0 ? mass : null;
}

function resolveBodyCenterOfMass(bodyIndex: number): PhysicsVector3 | null {
  const inertialPositions = toReadonlyNumberArray(data?.xipos);

  if (!inertialPositions || bodyIndex < 0 || inertialPositions.length < bodyIndex * 3 + 3) {
    return null;
  }

  const offset = bodyIndex * 3;
  const x = inertialPositions[offset];
  const y = inertialPositions[offset + 1];
  const z = inertialPositions[offset + 2];

  if (
    typeof x !== 'number' ||
    typeof y !== 'number' ||
    typeof z !== 'number' ||
    !Number.isFinite(x) ||
    !Number.isFinite(y) ||
    !Number.isFinite(z)
  ) {
    return null;
  }

  return { x, y, z };
}

function resolveCenterOfMassObservation(): {
  value: PhysicsVector3 | null;
  source: 'subtree_com' | 'mass_weighted_xipos' | 'unavailable';
  crossCheckErrorMeters: number | null;
} {
  const subtreeCenterOfMass = resolveSubtreeCenterOfMass();
  const massWeightedCenterOfMass = resolveMassWeightedCenterOfMass();
  const crossCheckErrorMeters = subtreeCenterOfMass && massWeightedCenterOfMass
    ? Math.hypot(
        subtreeCenterOfMass.x - massWeightedCenterOfMass.x,
        subtreeCenterOfMass.y - massWeightedCenterOfMass.y,
        subtreeCenterOfMass.z - massWeightedCenterOfMass.z
      )
    : null;

  if (subtreeCenterOfMass) {
    return {
      value: subtreeCenterOfMass,
      source: 'subtree_com',
      crossCheckErrorMeters
    };
  }

  if (massWeightedCenterOfMass) {
    return {
      value: massWeightedCenterOfMass,
      source: 'mass_weighted_xipos',
      crossCheckErrorMeters
    };
  }

  return {
    value: null,
    source: 'unavailable',
    crossCheckErrorMeters: null
  };
}

function resolveMassWeightedCenterOfMass(): PhysicsVector3 | null {
  const inertialPositions = toReadonlyNumberArray(data?.xipos);
  const bodyMasses = toReadonlyNumberArray(model?.body_mass);

  if (!inertialPositions || !bodyMasses || inertialPositions.length < 6 || bodyMasses.length < 2) {
    return null;
  }

  const bodyCount = Math.min(resolveBodyCount(inertialPositions), bodyMasses.length);
  const bodyIndices = resolveCenterOfMassBodyIndices(bodyCount);
  let totalMass = 0;
  let weightedX = 0;
  let weightedY = 0;
  let weightedZ = 0;

  for (const bodyIndex of bodyIndices) {
    const mass = bodyMasses[bodyIndex];

    if (typeof mass !== 'number' || !Number.isFinite(mass) || mass <= 0) {
      continue;
    }

    const positionOffset = bodyIndex * 3;
    const x = inertialPositions[positionOffset];
    const y = inertialPositions[positionOffset + 1];
    const z = inertialPositions[positionOffset + 2];

    if (
      typeof x !== 'number' ||
      typeof y !== 'number' ||
      typeof z !== 'number' ||
      !Number.isFinite(x) ||
      !Number.isFinite(y) ||
      !Number.isFinite(z)
    ) {
      continue;
    }

    totalMass += mass;
    weightedX += x * mass;
    weightedY += y * mass;
    weightedZ += z * mass;
  }

  if (totalMass <= 0) {
    return null;
  }

  return {
    x: weightedX / totalMass,
    y: weightedY / totalMass,
    z: weightedZ / totalMass
  };
}

function resolveCenterOfMassBodyIndices(bodyCount: number): number[] {
  const bodyNames = sourceMetadata.bodyNames;
  const centerOfMassBodyName = sourceMetadata.centerOfMassBodyName ?? sourceMetadata.visualRootBodyName;

  if (!centerOfMassBodyName) {
    return createBodyIndexRange(bodyCount);
  }

  if (!bodyNames || bodyNames.length < bodyCount) {
    return [];
  }

  const selectedIndices: number[] = [];

  for (let bodyIndex = 1; bodyIndex < bodyCount; bodyIndex += 1) {
    const bodyName = bodyNames[bodyIndex];

    if (!bodyName) {
      continue;
    }

    if (bodyName === centerOfMassBodyName || isBodyDescendantOf(bodyName, centerOfMassBodyName)) {
      selectedIndices.push(bodyIndex);
    }
  }

  return selectedIndices;
}

function createBodyIndexRange(bodyCount: number): number[] {
  const indices: number[] = [];

  for (let bodyIndex = 1; bodyIndex < bodyCount; bodyIndex += 1) {
    indices.push(bodyIndex);
  }

  return indices;
}

function isBodyDescendantOf(bodyName: string, ancestorBodyName: string): boolean {
  let parentBodyName = sourceMetadata.bodyParentNames?.[bodyName];
  const visited = new Set<string>();

  while (parentBodyName && !visited.has(parentBodyName)) {
    if (parentBodyName === ancestorBodyName) {
      return true;
    }

    visited.add(parentBodyName);
    parentBodyName = sourceMetadata.bodyParentNames?.[parentBodyName];
  }

  return false;
}

function resolveSubtreeCenterOfMass(): PhysicsVector3 | null {
  const subtreeCom = toReadonlyNumberArray(data?.subtree_com);

  if (!subtreeCom || subtreeCom.length < 3) {
    return null;
  }

  const centerOfMassBodyName = sourceMetadata.centerOfMassBodyName ?? sourceMetadata.visualRootBodyName;
  const bodyIndex = centerOfMassBodyName
    ? sourceMetadata.bodyNames?.findIndex((bodyName) => bodyName === centerOfMassBodyName) ?? -1
    : 0;

  if (bodyIndex < 0) {
    return null;
  }

  const offset = bodyIndex * 3;

  if (subtreeCom.length < offset + 3) {
    return null;
  }

  return {
    x: subtreeCom[offset] ?? 0,
    y: subtreeCom[offset + 1] ?? 0,
    z: subtreeCom[offset + 2] ?? 0
  };
}

function resolveBodyCount(positions: ArrayLike<number> | null): number {
  if (typeof model?.nbody === 'number' && model.nbody > 0) {
    return model.nbody;
  }

  return positions ? Math.floor(positions.length / 3) : 0;
}

function publishState(): void {
  postMessage({ type: 'state', payload: createStatePayload() } satisfies PhysicsWorkerStateMessage);
}

function postResponse(response: PhysicsWorkerResponse): void {
  postMessage(response);
}

function disposeSimulation(): void {
  data?.delete?.();
  model?.delete?.();
  data = null;
  model = null;
  sourceMetadata = {};
  loadedSourceId = null;
  stepCount = 0;
  simulatedTime = 0;
  simulationStepAccumulator = 0;
  droppedSimulationTime = 0;
  lastRequestedStepDelta = 0;
  lastAdvancedStepDelta = 0;
  jointTargets.clear();
  pendingBodyImpulses.length = 0;
}

async function fetchText(url: string): Promise<string> {
  const response = await fetch(url);

  if (!response.ok) {
    throw new Error(`Failed to fetch MuJoCo source "${url}" (${response.status}).`);
  }

  return response.text();
}

function readNumberPayload(payload: unknown, key: string): number {
  return isRecord(payload) && typeof payload[key] === 'number' ? payload[key] : 0;
}

function toReadonlyNumberArray(value: unknown): ArrayLike<number> | null {
  return value && typeof value === 'object' && typeof (value as ArrayLike<number>).length === 'number'
    ? (value as ArrayLike<number>)
    : null;
}

function toMutableNumberArray(value: unknown): { [index: number]: number; length: number } | null {
  return value && typeof value === 'object' && typeof (value as ArrayLike<number>).length === 'number'
    ? (value as { [index: number]: number; length: number })
    : null;
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return !!value && typeof value === 'object';
}
