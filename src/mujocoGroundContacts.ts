/** Solver contact geometry, not a force/adhesion or dynamic stability estimate.
 * Only contacts against a world-owned plane are currently supported.
 */
export function readMujocoGroundContacts(model: any, data: any) {
  const unavailable = { status: 'unavailable' as const, points: [] as Array<{ x: number; y: number; z: number }> };
  try {
    if (!model?.geom_type || !model?.geom_bodyid || !Number.isInteger(data?.ncon) || data.ncon < 0 ||
      typeof data.contact?.get !== 'function') return unavailable;
    const isGround = (g: number) => model.geom_type[g] === 0 && model.geom_bodyid[g] === 0;
    if (!Array.from(model.geom_type as ArrayLike<number>).some((_, g) => isGround(g))) return unavailable;
    const points: Array<{ x: number; y: number; z: number }> = [];
    for (let i = 0; i < data.ncon; i++) {
      const c = data.contact.get(i);
      if (!c || !Number.isFinite(c.dist) || !Number.isInteger(c.efc_address)) return unavailable;
      const a = c.geom1, b = c.geom2;
      if (![a, b].every(g => Number.isInteger(g) && g >= 0 && g < model.geom_type.length)) return unavailable;
      if (c.efc_address < 0 || c.dist > 0 || isGround(a) === isGround(b)) continue;
      const p = c.pos;
      if (!p || ![p[0], p[1], p[2]].every(Number.isFinite)) return unavailable;
      points.push({ x: p[0], y: p[1], z: p[2] });
    }
    return { status: 'observed' as const, points };
  } catch { return unavailable; }
}
