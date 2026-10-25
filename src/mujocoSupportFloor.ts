import type {
  PhysicsQuaternion,
  PhysicsColliderProxy,
  PhysicsVector3
} from '@konitif/physics';

/**
 * Deliberate clearance kept between the lowest support collider and z=0.
 *
 * This is the single physical authority for both the authored neutral pose and
 * runtime support-floor settlement. It is intentionally sub-millimetric: high
 * enough to avoid an initial interpenetration, but too small to create a
 * visible or dynamically meaningful fall before the first simulation step.
 */
export const MUJOCO_SUPPORT_GROUND_GAP_METERS = 0.0002;

export interface MujocoSupportBodyObservation {
  bodyName: string;
  position: PhysicsVector3;
  rotation: PhysicsQuaternion;
  colliders?: readonly PhysicsColliderProxy[];
  fallbackGroundClearance?: number;
}

export function resolveMujocoSupportFloorHeight(
  observations: readonly MujocoSupportBodyObservation[]
): number | null {
  let lowestHeight = Number.POSITIVE_INFINITY;

  for (const observation of observations) {
    const colliderHeights = (observation.colliders ?? [])
      .map((collider) => resolveColliderFloorHeight(observation, collider))
      .filter((height): height is number => height !== null);

    if (colliderHeights.length > 0) {
      lowestHeight = Math.min(lowestHeight, ...colliderHeights);
      continue;
    }

    const clearance = finiteNonNegative(observation.fallbackGroundClearance);

    if (isFiniteVector(observation.position) && clearance !== null) {
      lowestHeight = Math.min(lowestHeight, observation.position.z - clearance);
    }
  }

  return Number.isFinite(lowestHeight) ? lowestHeight : null;
}

function resolveColliderFloorHeight(
  body: MujocoSupportBodyObservation,
  collider: PhysicsColliderProxy
): number | null {
  if (!isFiniteVector(body.position)) {
    return null;
  }

  const bodyRotation = normalizeQuaternion(body.rotation);
  const colliderRotation = normalizeQuaternion(collider.rotation);
  const colliderPosition = collider.position && isFiniteVector(collider.position)
    ? collider.position
    : ZERO_VECTOR;
  const worldCenter = addVectors(body.position, rotateVector(colliderPosition, bodyRotation));

  switch (collider.shape) {
    case 'box':
      return resolveBoxFloorHeight(worldCenter, bodyRotation, colliderRotation, collider.size);
    case 'sphere':
      return resolveSphereFloorHeight(worldCenter, collider.size);
    case 'ellipsoid':
      return resolveEllipsoidFloorHeight(worldCenter, bodyRotation, colliderRotation, collider.size);
    case 'capsule':
      return resolveCapsuleFloorHeight(body, bodyRotation, colliderRotation, collider);
    default:
      return null;
  }
}

function resolveBoxFloorHeight(
  center: PhysicsVector3,
  bodyRotation: PhysicsQuaternion,
  colliderRotation: PhysicsQuaternion,
  size: readonly number[] | undefined
): number | null {
  const halfExtents = resolveSize(size, 3);

  if (!halfExtents) {
    return null;
  }

  const axes = WORLD_AXES.map((axis) => rotateVector(rotateVector(axis, colliderRotation), bodyRotation));
  const verticalRadius = axes.reduce(
    (radius, axis, index) => radius + Math.abs(axis.z) * halfExtents[index],
    0
  );

  return center.z - verticalRadius;
}

function resolveSphereFloorHeight(center: PhysicsVector3, size: readonly number[] | undefined): number | null {
  const radius = finitePositive(size?.[0]);
  return radius === null ? null : center.z - radius;
}

function resolveEllipsoidFloorHeight(
  center: PhysicsVector3,
  bodyRotation: PhysicsQuaternion,
  colliderRotation: PhysicsQuaternion,
  size: readonly number[] | undefined
): number | null {
  const radii = resolveSize(size, 3);

  if (!radii) {
    return null;
  }

  const axes = WORLD_AXES.map((axis) => rotateVector(rotateVector(axis, colliderRotation), bodyRotation));
  const verticalRadius = Math.hypot(
    axes[0].z * radii[0],
    axes[1].z * radii[1],
    axes[2].z * radii[2]
  );

  return center.z - verticalRadius;
}

function resolveCapsuleFloorHeight(
  body: MujocoSupportBodyObservation,
  bodyRotation: PhysicsQuaternion,
  colliderRotation: PhysicsQuaternion,
  collider: PhysicsColliderProxy
): number | null {
  const radius = finitePositive(collider.size?.[0]);

  if (radius === null) {
    return null;
  }

  const colliderPosition = collider.position && isFiniteVector(collider.position)
    ? collider.position
    : ZERO_VECTOR;
  const fromTo = collider.fromTo;

  if (fromTo && fromTo.length >= 6 && fromTo.slice(0, 6).every(Number.isFinite)) {
    const start = transformLocalPoint(
      { x: fromTo[0], y: fromTo[1], z: fromTo[2] },
      colliderPosition,
      colliderRotation,
      body.position,
      bodyRotation
    );
    const end = transformLocalPoint(
      { x: fromTo[3], y: fromTo[4], z: fromTo[5] },
      colliderPosition,
      colliderRotation,
      body.position,
      bodyRotation
    );

    return Math.min(start.z, end.z) - radius;
  }

  const halfLength = finiteNonNegative(collider.size?.[1]) ?? 0;
  const axis = rotateVector(rotateVector(WORLD_AXES[2], colliderRotation), bodyRotation);
  const center = addVectors(body.position, rotateVector(colliderPosition, bodyRotation));

  return center.z - Math.abs(axis.z) * halfLength - radius;
}

function transformLocalPoint(
  point: PhysicsVector3,
  colliderPosition: PhysicsVector3,
  colliderRotation: PhysicsQuaternion,
  bodyPosition: PhysicsVector3,
  bodyRotation: PhysicsQuaternion
): PhysicsVector3 {
  const bodyLocalPoint = addVectors(colliderPosition, rotateVector(point, colliderRotation));
  return addVectors(bodyPosition, rotateVector(bodyLocalPoint, bodyRotation));
}

function rotateVector(vector: PhysicsVector3, quaternion: PhysicsQuaternion): PhysicsVector3 {
  const { x, y, z, w } = quaternion;
  const tx = 2 * (y * vector.z - z * vector.y);
  const ty = 2 * (z * vector.x - x * vector.z);
  const tz = 2 * (x * vector.y - y * vector.x);

  return {
    x: vector.x + w * tx + (y * tz - z * ty),
    y: vector.y + w * ty + (z * tx - x * tz),
    z: vector.z + w * tz + (x * ty - y * tx)
  };
}

function normalizeQuaternion(value: PhysicsQuaternion | undefined): PhysicsQuaternion {
  if (!value || ![value.x, value.y, value.z, value.w].every(Number.isFinite)) {
    return IDENTITY_QUATERNION;
  }

  const magnitude = Math.hypot(value.x, value.y, value.z, value.w);

  if (magnitude <= Number.EPSILON) {
    return IDENTITY_QUATERNION;
  }

  return {
    x: value.x / magnitude,
    y: value.y / magnitude,
    z: value.z / magnitude,
    w: value.w / magnitude
  };
}

function resolveSize(size: readonly number[] | undefined, length: number): number[] | null {
  if (!size || size.length < length) {
    return null;
  }

  const values = size.slice(0, length).map(finitePositive);
  return values.every((value): value is number => value !== null) ? values : null;
}

function finitePositive(value: number | undefined): number | null {
  return typeof value === 'number' && Number.isFinite(value) && value > 0 ? value : null;
}

function finiteNonNegative(value: number | undefined): number | null {
  return typeof value === 'number' && Number.isFinite(value) && value >= 0 ? value : null;
}

function isFiniteVector(value: PhysicsVector3): boolean {
  return [value.x, value.y, value.z].every(Number.isFinite);
}

function addVectors(left: PhysicsVector3, right: PhysicsVector3): PhysicsVector3 {
  return {
    x: left.x + right.x,
    y: left.y + right.y,
    z: left.z + right.z
  };
}

const ZERO_VECTOR: PhysicsVector3 = { x: 0, y: 0, z: 0 };
const IDENTITY_QUATERNION: PhysicsQuaternion = { x: 0, y: 0, z: 0, w: 1 };
const WORLD_AXES: readonly PhysicsVector3[] = [
  { x: 1, y: 0, z: 0 },
  { x: 0, y: 1, z: 0 },
  { x: 0, y: 0, z: 1 }
];
