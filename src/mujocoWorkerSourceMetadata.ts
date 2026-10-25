import type { PhysicsColliderProxy } from '@konitif/physics';

// Execution metadata only. Legacy field precedence and permissive validation are retained.
export type MujocoWorkerSourceMetadata = {
  coordinateFrame?: string;
  sceneYawRadians?: number;
  bodyNames?: string[];
  bodyParentNames?: Record<string, string>;
  jointQposIndex?: Record<string, number>;
  jointQvelIndex?: Record<string, number>;
  actuatorCtrlIndex?: Record<string, number>;
  releaseLockedJointNames?: Set<string>;
  auxiliaryBodyNames?: Set<string>;
  supportBodyNames?: Set<string>;
  supportBodyGroundClearance?: Record<string, number>;
  contactBodyNames?: Set<string>;
  debugHiddenBodyNames?: Set<string>;
  visualRootBodyName?: string;
  centerOfMassBodyName?: string;
  bodyColliderProxies?: Record<string, PhysicsColliderProxy[]>;
};

export function resolveWorkerSourceMetadata(metadata: unknown): MujocoWorkerSourceMetadata {
  if (!isRecord(metadata)) return {};
  return resolveWorkerFields(metadata, isRecord(metadata.visualAlignment) ? metadata.visualAlignment : undefined);
}

/** Shared useful fields only; execution never interprets the joint mapping table. */
export function resolveWorkerFields(
  metadata: Record<string, unknown>,
  visualAlignment: Partial<Record<keyof MujocoWorkerSourceMetadata, unknown>> | undefined
): MujocoWorkerSourceMetadata {
  return {
    coordinateFrame:
      typeof visualAlignment?.coordinateFrame === 'string'
        ? visualAlignment.coordinateFrame
        : typeof metadata.coordinateFrame === 'string'
          ? metadata.coordinateFrame
          : undefined,
    sceneYawRadians:
      typeof visualAlignment?.sceneYawRadians === 'number' && Number.isFinite(visualAlignment.sceneYawRadians)
        ? visualAlignment.sceneYawRadians
        : undefined,
    bodyNames: Array.isArray(metadata.bodyNames)
      ? metadata.bodyNames.filter((entry): entry is string => typeof entry === 'string')
      : undefined,
    bodyParentNames: resolveStringRecord(metadata.bodyParentNames),
    jointQposIndex: resolveNumberRecord(metadata.jointQposIndex),
    jointQvelIndex: resolveNumberRecord(metadata.jointQvelIndex),
    actuatorCtrlIndex: resolveNumberRecord(metadata.actuatorCtrlIndex),
    releaseLockedJointNames: resolveStringSet(visualAlignment?.releaseLockedJointNames ?? metadata.releaseLockedJointNames),
    bodyColliderProxies: resolveColliderProxyRecord(visualAlignment?.bodyColliderProxies ?? metadata.bodyColliderProxies),
    auxiliaryBodyNames: resolveStringSet(visualAlignment?.auxiliaryBodyNames ?? metadata.auxiliaryBodyNames),
    supportBodyNames: resolveStringSet(visualAlignment?.supportBodyNames ?? metadata.supportBodyNames),
    supportBodyGroundClearance: resolveNumberRecord(
      visualAlignment?.supportBodyGroundClearance ?? metadata.supportBodyGroundClearance
    ),
    contactBodyNames: resolveStringSet(visualAlignment?.contactBodyNames ?? metadata.contactBodyNames),
    debugHiddenBodyNames: resolveStringSet(visualAlignment?.debugHiddenBodyNames ?? metadata.debugHiddenBodyNames),
    visualRootBodyName:
      typeof visualAlignment?.visualRootBodyName === 'string'
        ? visualAlignment.visualRootBodyName
        : typeof metadata.visualRootBodyName === 'string'
          ? metadata.visualRootBodyName
          : undefined,
    centerOfMassBodyName:
      typeof visualAlignment?.centerOfMassBodyName === 'string'
        ? visualAlignment.centerOfMassBodyName
        : typeof metadata.centerOfMassBodyName === 'string'
          ? metadata.centerOfMassBodyName
          : undefined
  };
}

function resolveColliderProxyRecord(value: unknown): Record<string, PhysicsColliderProxy[]> | undefined {
  if (!isRecord(value)) {
    return undefined;
  }

  const record: Record<string, PhysicsColliderProxy[]> = {};

  for (const [bodyName, entries] of Object.entries(value)) {
    if (!Array.isArray(entries)) {
      continue;
    }

    const colliders = entries.filter((entry): entry is PhysicsColliderProxy => {
      return (
        isRecord(entry) &&
        typeof entry.shape === 'string' &&
        ['sphere', 'box', 'capsule', 'ellipsoid'].includes(entry.shape)
      );
    });

    if (colliders.length > 0) {
      record[bodyName] = colliders;
    }
  }

  return Object.keys(record).length > 0 ? record : undefined;
}

function resolveStringSet(value: unknown): Set<string> | undefined {
  if (!Array.isArray(value)) {
    return undefined;
  }

  const entries = value.filter((entry): entry is string => typeof entry === 'string' && entry.length > 0);

  return entries.length > 0 ? new Set(entries) : undefined;
}

function resolveStringRecord(value: unknown): Record<string, string> | undefined {
  if (!isRecord(value)) {
    return undefined;
  }

  const entries = Object.entries(value).filter((entry): entry is [string, string] => typeof entry[1] === 'string');

  return entries.length > 0 ? Object.fromEntries(entries) : undefined;
}

function resolveNumberRecord(value: unknown): Record<string, number> | undefined {
  if (!isRecord(value)) {
    return undefined;
  }

  const entries = Object.entries(value).filter((entry): entry is [string, number] => typeof entry[1] === 'number');

  return entries.length > 0 ? Object.fromEntries(entries) : undefined;
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return !!value && typeof value === 'object';
}
