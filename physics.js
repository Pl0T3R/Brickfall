// physics.js - a small rigid-body physics engine.
//
// Shapes:   sphere, box, infinite plane (static only)
// Bodies:   static, dynamic, kinematic (moved by you, pushes dynamics, never pushed back)
// Solver:   sequential impulses with soft contacts and substepping (the "soft step"
//           approach from Box2D v3), warm starting, speculative contacts,
//           Coulomb friction, restitution, rolling resistance, island sleeping.
// Units:    meters, kilograms, seconds. No dependencies.

export const STATIC = 0;
export const DYNAMIC = 1;
export const KINEMATIC = 2;

// ---------------------------------------------------------------------------
// Math
// ---------------------------------------------------------------------------

export class Vec3 {
  constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; }
  set(x, y, z) { this.x = x; this.y = y; this.z = z; return this; }
  copy(v) { this.x = v.x; this.y = v.y; this.z = v.z; return this; }
  clone() { return new Vec3(this.x, this.y, this.z); }
  add(v) { this.x += v.x; this.y += v.y; this.z += v.z; return this; }
  sub(v) { this.x -= v.x; this.y -= v.y; this.z -= v.z; return this; }
  scale(s) { this.x *= s; this.y *= s; this.z *= s; return this; }
  addScaled(v, s) { this.x += v.x * s; this.y += v.y * s; this.z += v.z * s; return this; }
  subVectors(a, b) { this.x = a.x - b.x; this.y = a.y - b.y; this.z = a.z - b.z; return this; }
  dot(v) { return this.x * v.x + this.y * v.y + this.z * v.z; }
  crossVectors(a, b) {
    const x = a.y * b.z - a.z * b.y, y = a.z * b.x - a.x * b.z, z = a.x * b.y - a.y * b.x;
    this.x = x; this.y = y; this.z = z; return this;
  }
  lengthSq() { return this.x * this.x + this.y * this.y + this.z * this.z; }
  length() { return Math.sqrt(this.lengthSq()); }
  normalize() { const l = this.length(); if (l > 1e-12) this.scale(1 / l); return this; }
  distanceTo(v) { const dx = this.x - v.x, dy = this.y - v.y, dz = this.z - v.z; return Math.sqrt(dx * dx + dy * dy + dz * dz); }
}

export class Quat {
  constructor(x = 0, y = 0, z = 0, w = 1) { this.x = x; this.y = y; this.z = z; this.w = w; }
  set(x, y, z, w) { this.x = x; this.y = y; this.z = z; this.w = w; return this; }
  copy(q) { this.x = q.x; this.y = q.y; this.z = q.z; this.w = q.w; return this; }
  clone() { return new Quat(this.x, this.y, this.z, this.w); }
  normalize() {
    let l = Math.hypot(this.x, this.y, this.z, this.w);
    if (l < 1e-12) { this.set(0, 0, 0, 1); return this; }
    l = 1 / l; this.x *= l; this.y *= l; this.z *= l; this.w *= l; return this;
  }
  setFromAxisAngle(axis, angle) {
    const s = Math.sin(angle / 2);
    this.x = axis.x * s; this.y = axis.y * s; this.z = axis.z * s; this.w = Math.cos(angle / 2); return this;
  }
  multiplyQuats(a, b) {
    const x = a.x * b.w + a.w * b.x + a.y * b.z - a.z * b.y;
    const y = a.y * b.w + a.w * b.y + a.z * b.x - a.x * b.z;
    const z = a.z * b.w + a.w * b.z + a.x * b.y - a.y * b.x;
    const w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    this.x = x; this.y = y; this.z = z; this.w = w; return this;
  }
  conjugate() { this.x = -this.x; this.y = -this.y; this.z = -this.z; return this; }
}

// Rotation matrices are row-major Float64Array(9). Column i is the body's local axis i in world space.
function quatToMat(q, m) {
  const x2 = q.x + q.x, y2 = q.y + q.y, z2 = q.z + q.z;
  const xx = q.x * x2, xy = q.x * y2, xz = q.x * z2;
  const yy = q.y * y2, yz = q.y * z2, zz = q.z * z2;
  const wx = q.w * x2, wy = q.w * y2, wz = q.w * z2;
  m[0] = 1 - (yy + zz); m[1] = xy - wz; m[2] = xz + wy;
  m[3] = xy + wz; m[4] = 1 - (xx + zz); m[5] = yz - wx;
  m[6] = xz - wy; m[7] = yz + wx; m[8] = 1 - (xx + yy);
}
// out = M v
function mulMV(m, v, out) {
  const x = m[0] * v.x + m[1] * v.y + m[2] * v.z;
  const y = m[3] * v.x + m[4] * v.y + m[5] * v.z;
  const z = m[6] * v.x + m[7] * v.y + m[8] * v.z;
  out.x = x; out.y = y; out.z = z; return out;
}
// out = M^T v
function mulMTV(m, v, out) {
  const x = m[0] * v.x + m[3] * v.y + m[6] * v.z;
  const y = m[1] * v.x + m[4] * v.y + m[7] * v.z;
  const z = m[2] * v.x + m[5] * v.y + m[8] * v.z;
  out.x = x; out.y = y; out.z = z; return out;
}
function axisOf(m, i, out) { return out.set(m[i], m[3 + i], m[6 + i]); }

// ---------------------------------------------------------------------------
// Bodies
// ---------------------------------------------------------------------------

let nextBodyId = 1;
const toVec3 = (v, out) => (Array.isArray(v) ? out.set(v[0], v[1], v[2]) : out.copy(v));

export class Body {
  /**
   * @param {object} o
   *   shape: 'sphere' | 'box' | 'plane'
   *   radius (sphere), halfExtents [x,y,z] (box), normal [x,y,z] (plane; position is a point on it)
   *   type: STATIC | DYNAMIC | KINEMATIC (default DYNAMIC if mass > 0, else STATIC)
   *   mass, position, quaternion, friction, restitution, rollingResistance,
   *   linearDamping, angularDamping, gravityScale, userData
   */
  constructor(o = {}) {
    this.id = nextBodyId++;
    this.world = null;
    this.shape = o.shape || 'sphere';
    this.radius = o.radius ?? 0.05;
    this.halfExtents = toVec3(o.halfExtents ?? [0.05, 0.05, 0.05], new Vec3());
    this.normal = toVec3(o.normal ?? [0, 1, 0], new Vec3()).normalize();
    this.position = o.position ? toVec3(o.position, new Vec3()) : new Vec3();
    this.quaternion = o.quaternion ? new Quat().copy(o.quaternion).normalize() : new Quat();
    this.velocity = new Vec3();
    this.angularVelocity = new Vec3();
    this.mass = o.mass ?? 0;
    this.type = o.type ?? (this.mass > 0 ? DYNAMIC : STATIC);
    if (this.shape === 'plane') this.type = STATIC;
    this.friction = o.friction ?? 0.5;
    this.restitution = o.restitution ?? 0.2;
    this.rollingResistance = o.rollingResistance ?? 0;
    this.linearDamping = o.linearDamping ?? 0.01;
    this.angularDamping = o.angularDamping ?? 0.05;
    this.gravityScale = o.gravityScale ?? 1;
    this.collisionGroup = o.collisionGroup ?? 1;
    this.collisionMask = o.collisionMask ?? 0xffffffff;
    this.canSleep = o.canSleep ?? true;
    this.userData = o.userData ?? null;

    this.sleeping = false;
    this.sleepTime = 0;
    this._sleepIsland = null;
    this._kinTarget = null;   // kinematic target pose for the current step
    this._index = -1;
    this.invMass = 0;
    this.invInertiaLocal = new Vec3();
    this.R = new Float64Array(9);
    this.invI = new Float64Array(9);   // world-space inverse inertia (row major)
    this.aabb = new Float64Array(6);   // minX minY minZ maxX maxY maxZ
    this.offset = 0;                   // plane offset: dot(normal, x) = offset
    this.boundingRadius = 0;
    this.setType(this.type);
    this.updateTransform();
  }

  setType(type) {
    this.type = this.shape === 'plane' ? STATIC : type;
    if (this.type === DYNAMIC && this.mass > 0) {
      this.invMass = 1 / this.mass;
      const m = this.mass;
      if (this.shape === 'sphere') {
        const i = 0.4 * m * this.radius * this.radius;
        this.invInertiaLocal.set(1 / i, 1 / i, 1 / i);
      } else {
        const h = this.halfExtents;
        this.invInertiaLocal.set(
          3 / (m * (h.y * h.y + h.z * h.z)),
          3 / (m * (h.x * h.x + h.z * h.z)),
          3 / (m * (h.x * h.x + h.y * h.y)));
      }
    } else {
      this.invMass = 0;
      this.invInertiaLocal.set(0, 0, 0);
      if (this.type !== KINEMATIC) { this.velocity.set(0, 0, 0); this.angularVelocity.set(0, 0, 0); }
    }
    this._kinTarget = null;
    this.wake();
    return this;
  }

  setMass(mass) { this.mass = mass; return this.setType(this.type); }

  updateTransform() {
    quatToMat(this.quaternion, this.R);
    if (this.shape === 'plane') {
      this.offset = this.normal.dot(this.position);
      this.boundingRadius = Infinity;
    } else if (this.shape === 'sphere') {
      this.boundingRadius = this.radius;
    } else {
      this.boundingRadius = this.halfExtents.length();
    }
    const R = this.R, d = this.invInertiaLocal, I = this.invI;
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        I[i * 3 + j] = R[i * 3] * d.x * R[j * 3] + R[i * 3 + 1] * d.y * R[j * 3 + 1] + R[i * 3 + 2] * d.z * R[j * 3 + 2];
      }
    }
  }

  /** Teleport. Clears velocity unless keepVelocity. */
  setPose(position, quaternion, keepVelocity = false) {
    if (position) this.position.copy(position);
    if (quaternion) this.quaternion.copy(quaternion).normalize();
    if (!keepVelocity) { this.velocity.set(0, 0, 0); this.angularVelocity.set(0, 0, 0); }
    this._kinTarget = null;
    this.updateTransform();
    this.wake();
  }

  /** Kinematic bodies: move to this pose over the next step of length dt (sets velocities). */
  moveKinematic(position, quaternion, dt) {
    const inv = 1 / dt;
    this.velocity.set((position.x - this.position.x) * inv, (position.y - this.position.y) * inv, (position.z - this.position.z) * inv);
    if (quaternion) {
      angularVelocityBetween(this.quaternion, quaternion, dt, this.angularVelocity);
    } else {
      this.angularVelocity.set(0, 0, 0);
    }
    if (!this._kinTarget) this._kinTarget = { p: new Vec3(), q: new Quat() };
    this._kinTarget.p.copy(position);
    this._kinTarget.q.copy(quaternion || this.quaternion);
  }

  wake() {
    if (!this.sleeping) { this.sleepTime = 0; return; }
    const island = this._sleepIsland || [this];
    for (const b of island) { b.sleeping = false; b.sleepTime = 0; b._sleepIsland = null; }
  }

  applyImpulse(impulse, worldPoint) {
    if (this.type !== DYNAMIC) return;
    this.wake();
    this.velocity.addScaled(impulse, this.invMass);
    if (worldPoint) {
      const r = _t0.subVectors(worldPoint, this.position);
      const c = _t1.crossVectors(r, impulse);
      mulMV(this.invI, c, c);
      this.angularVelocity.add(c);
    }
  }
}

/** Angular velocity that rotates q0 into q1 over dt (shortest path). */
export function angularVelocityBetween(q0, q1, dt, out) {
  // dq = q1 * conj(q0)
  const cx = -q0.x, cy = -q0.y, cz = -q0.z, cw = q0.w;
  let x = q1.x * cw + q1.w * cx + q1.y * cz - q1.z * cy;
  let y = q1.y * cw + q1.w * cy + q1.z * cx - q1.x * cz;
  let z = q1.z * cw + q1.w * cz + q1.x * cy - q1.y * cx;
  let w = q1.w * cw - q1.x * cx - q1.y * cy - q1.z * cz;
  if (w < 0) { x = -x; y = -y; z = -z; w = -w; }
  const s = Math.sqrt(x * x + y * y + z * z);
  if (s < 1e-9) return out.set(0, 0, 0);
  const angle = 2 * Math.atan2(s, w);
  const k = angle / (s * dt);
  return out.set(x * k, y * k, z * k);
}

// ---------------------------------------------------------------------------
// Contacts
// ---------------------------------------------------------------------------

class ContactPoint {
  constructor() {
    this.position = new Vec3();   // world point (between the two surfaces)
    this.separation = 0;          // negative = penetrating
    this.localA = new Vec3();     // anchor in A's local frame
    this.localB = new Vec3();
    this.rA = new Vec3();         // anchor relative to A's center (world), fixed during a step
    this.rB = new Vec3();
    this.normalImpulse = 0;
    this.friction = new Vec3();   // accumulated friction impulse (world vector, kept between steps)
    this.f1 = 0;                  // friction impulse along the manifold tangents during a step
    this.f2 = 0;
    this.maxNormalImpulse = 0;
    this.normalMass = 0;
    this.tangentMass1 = 0;
    this.tangentMass2 = 0;
    this.relativeVelocity = 0;
    // Jacobian terms for the normal, t1 and t2 axes (12 numbers each):
    // rA x d, invIA (rA x d), rB x d, invIB (rB x d)
    this.J = new Float64Array(36);
  }
}

class Manifold {
  constructor() {
    this.a = null; this.b = null; this.key = 0;
    this.normal = new Vec3();     // from a to b
    this.t1 = new Vec3(); this.t2 = new Vec3();
    this.count = 0;
    this.points = [new ContactPoint(), new ContactPoint(), new ContactPoint(), new ContactPoint()];
    this.friction = 0; this.restitution = 0; this.rolling = 0; this.rollingMass = 0;
    this.rollingImpulse = new Vec3();
    this.touching = false; this.wasTouching = false;
    this.softness = null;
    this.keepStamp = -1;
  }
}

// Scratch space for narrowphase output before reduction to 4 points.
const MAXP = 16;
const candP = Array.from({ length: MAXP }, () => new Vec3());
const candS = new Float64Array(MAXP);
let candCount = 0;
function pushCandidate(x, y, z, s) {
  if (candCount >= MAXP) return;
  candP[candCount].set(x, y, z); candS[candCount] = s; candCount++;
}

// Pick up to 4 candidates that cover the largest area, keeping the deepest.
const pick = [0, 0, 0, 0];
function reduceCandidates(man) {
  const n = man.normal;
  if (candCount <= 4) {
    for (let i = 0; i < candCount; i++) { man.points[i].position.copy(candP[i]); man.points[i].separation = candS[i]; }
    man.count = candCount;
    return;
  }
  let i0 = 0;
  for (let i = 1; i < candCount; i++) if (candS[i] < candS[i0]) i0 = i;
  let i1 = -1, best = -1;
  for (let i = 0; i < candCount; i++) {
    if (i === i0) continue;
    const d = candP[i].distanceTo(candP[i0]);
    if (d > best) { best = d; i1 = i; }
  }
  const sArea = (a, b, c) => {
    const ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
    const vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
    return (uy * vz - uz * vy) * n.x + (uz * vx - ux * vz) * n.y + (ux * vy - uy * vx) * n.z;
  };
  let i2 = -1; best = -1; let sign = 1;
  for (let i = 0; i < candCount; i++) {
    if (i === i0 || i === i1) continue;
    const a = sArea(candP[i0], candP[i1], candP[i]);
    if (Math.abs(a) > best) { best = Math.abs(a); i2 = i; sign = a >= 0 ? 1 : -1; }
  }
  let i3 = -1; best = 0;
  const P0 = candP[i0], P1 = candP[i1], P2 = candP[i2];
  for (let i = 0; i < candCount; i++) {
    if (i === i0 || i === i1 || i === i2) continue;
    const q = candP[i];
    const out = Math.max(-sign * sArea(P0, P1, q), -sign * sArea(P1, P2, q), -sign * sArea(P2, P0, q));
    if (out > best) { best = out; i3 = i; }
  }
  pick[0] = i0; pick[1] = i1; pick[2] = i2; pick[3] = i3;
  let c = 0;
  for (let k = 0; k < 4; k++) {
    const i = pick[k]; if (i < 0) continue;
    man.points[c].position.copy(candP[i]); man.points[c].separation = candS[i]; c++;
  }
  man.count = c;
}

// --- narrowphase -----------------------------------------------------------

const _t0 = new Vec3(), _t1 = new Vec3(), _t2 = new Vec3(), _t3 = new Vec3(), _t4 = new Vec3();
const _ax = [new Vec3(), new Vec3(), new Vec3()];
const _bx = [new Vec3(), new Vec3(), new Vec3()];

function collideSphereSphere(A, B, margin, man) {
  const d = _t0.subVectors(B.position, A.position);
  const dist = d.length();
  const s = dist - A.radius - B.radius;
  if (s > margin) return;
  if (dist > 1e-9) man.normal.copy(d).scale(1 / dist); else man.normal.set(0, 1, 0);
  const n = man.normal, k = A.radius + s * 0.5;
  candCount = 0;
  pushCandidate(A.position.x + n.x * k, A.position.y + n.y * k, A.position.z + n.z * k, s);
  reduceCandidates(man);
}

// A = box, B = sphere; normal from box to sphere.
function collideBoxSphere(A, B, margin, man) {
  const h = A.halfExtents, r = B.radius;
  const local = mulMTV(A.R, _t0.subVectors(B.position, A.position), _t1);
  const qx = Math.max(-h.x, Math.min(h.x, local.x));
  const qy = Math.max(-h.y, Math.min(h.y, local.y));
  const qz = Math.max(-h.z, Math.min(h.z, local.z));
  const dx = local.x - qx, dy = local.y - qy, dz = local.z - qz;
  const d2 = dx * dx + dy * dy + dz * dz;
  const nLocal = _t2, surf = _t3;
  let s;
  if (d2 > 1e-12) {
    const dist = Math.sqrt(d2);
    s = dist - r;
    if (s > margin) return;
    nLocal.set(dx / dist, dy / dist, dz / dist);
    surf.set(qx, qy, qz);
  } else {
    // center inside the box: push out through the nearest face
    const px = h.x - Math.abs(local.x), py = h.y - Math.abs(local.y), pz = h.z - Math.abs(local.z);
    surf.copy(local);
    if (px <= py && px <= pz) { nLocal.set(Math.sign(local.x) || 1, 0, 0); s = -px - r; surf.x = h.x * nLocal.x; }
    else if (py <= pz) { nLocal.set(0, Math.sign(local.y) || 1, 0); s = -py - r; surf.y = h.y * nLocal.y; }
    else { nLocal.set(0, 0, Math.sign(local.z) || 1); s = -pz - r; surf.z = h.z * nLocal.z; }
  }
  mulMV(A.R, nLocal, man.normal);
  mulMV(A.R, surf, surf).add(A.position);
  const n = man.normal, k = s * 0.5;   // midpoint between box surface and sphere surface
  candCount = 0;
  pushCandidate(surf.x + n.x * k, surf.y + n.y * k, surf.z + n.z * k, s);
  reduceCandidates(man);
}

// A = plane, B = sphere
function collidePlaneSphere(A, B, margin, man) {
  const n = A.normal;
  const s = n.dot(B.position) - A.offset - B.radius;
  if (s > margin) return;
  man.normal.copy(n);
  const k = B.radius + s * 0.5;
  candCount = 0;
  pushCandidate(B.position.x - n.x * k, B.position.y - n.y * k, B.position.z - n.z * k, s);
  reduceCandidates(man);
}

// A = plane, B = box
function collidePlaneBox(A, B, margin, man) {
  const n = A.normal, h = B.halfExtents, R = B.R;
  axisOf(R, 0, _bx[0]); axisOf(R, 1, _bx[1]); axisOf(R, 2, _bx[2]);
  // quick reject using the support point
  const ext = h.x * Math.abs(_bx[0].dot(n)) + h.y * Math.abs(_bx[1].dot(n)) + h.z * Math.abs(_bx[2].dot(n));
  if (n.dot(B.position) - A.offset - ext > margin) return;
  man.normal.copy(n);
  candCount = 0;
  for (let i = 0; i < 8; i++) {
    const sx = i & 1 ? 1 : -1, sy = i & 2 ? 1 : -1, sz = i & 4 ? 1 : -1;
    const vx = B.position.x + _bx[0].x * h.x * sx + _bx[1].x * h.y * sy + _bx[2].x * h.z * sz;
    const vy = B.position.y + _bx[0].y * h.x * sx + _bx[1].y * h.y * sy + _bx[2].y * h.z * sz;
    const vz = B.position.z + _bx[0].z * h.x * sx + _bx[1].z * h.y * sy + _bx[2].z * h.z * sz;
    const s = n.x * vx + n.y * vy + n.z * vz - A.offset;
    if (s <= margin) pushCandidate(vx - n.x * s * 0.5, vy - n.y * s * 0.5, vz - n.z * s * 0.5, s);
  }
  reduceCandidates(man);
}

// Box vs box: separating axis test, then face clipping or edge-edge.
const polyA = Array.from({ length: 16 }, () => new Vec3());
const polyB = Array.from({ length: 16 }, () => new Vec3());
const hv = (v, i) => (i === 0 ? v.x : i === 1 ? v.y : v.z);

function clipPolygon(inPoly, inCount, outPoly, axis, center, limit, sign) {
  // keep points where sign * dot(p - center, axis) <= limit
  let outCount = 0;
  if (inCount === 0) return 0;
  let prev = inPoly[inCount - 1];
  let dPrev = sign * (axis.x * (prev.x - center.x) + axis.y * (prev.y - center.y) + axis.z * (prev.z - center.z)) - limit;
  for (let i = 0; i < inCount; i++) {
    const cur = inPoly[i];
    const dCur = sign * (axis.x * (cur.x - center.x) + axis.y * (cur.y - center.y) + axis.z * (cur.z - center.z)) - limit;
    if (dPrev <= 0) {
      if (dCur <= 0) { outPoly[outCount++].copy(cur); }
      else { const t = dPrev / (dPrev - dCur); outPoly[outCount++].set(prev.x + (cur.x - prev.x) * t, prev.y + (cur.y - prev.y) * t, prev.z + (cur.z - prev.z) * t); }
    } else if (dCur <= 0) {
      const t = dPrev / (dPrev - dCur);
      outPoly[outCount++].set(prev.x + (cur.x - prev.x) * t, prev.y + (cur.y - prev.y) * t, prev.z + (cur.z - prev.z) * t);
      outPoly[outCount++].copy(cur);
    }
    if (outCount >= 15) break;
    prev = cur; dPrev = dCur;
  }
  return outCount;
}

function collideBoxBox(A, B, margin, man) {
  const hA = A.halfExtents, hB = B.halfExtents;
  for (let i = 0; i < 3; i++) { axisOf(A.R, i, _ax[i]); axisOf(B.R, i, _bx[i]); }
  const d = _t0.subVectors(B.position, A.position);
  const projA = (L) => hA.x * Math.abs(_ax[0].dot(L)) + hA.y * Math.abs(_ax[1].dot(L)) + hA.z * Math.abs(_ax[2].dot(L));
  const projB = (L) => hB.x * Math.abs(_bx[0].dot(L)) + hB.y * Math.abs(_bx[1].dot(L)) + hB.z * Math.abs(_bx[2].dot(L));

  let sA = -Infinity, iA = -1, sB = -Infinity, iB = -1;
  for (let i = 0; i < 3; i++) {
    const L = _ax[i];
    const s = Math.abs(d.dot(L)) - hv(hA, i) - projB(L);
    if (s > margin) return;
    if (s > sA) { sA = s; iA = i; }
  }
  for (let j = 0; j < 3; j++) {
    const L = _bx[j];
    const s = Math.abs(d.dot(L)) - hv(hB, j) - projA(L);
    if (s > margin) return;
    if (s > sB) { sB = s; iB = j; }
  }
  let sE = -Infinity, eI = -1, eJ = -1;
  const L = _t1, bestL = _t4;
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      L.crossVectors(_ax[i], _bx[j]);
      const len = L.length();
      if (len < 1e-5) continue;
      L.scale(1 / len);
      const s = Math.abs(d.dot(L)) - projA(L) - projB(L);
      if (s > margin) return;
      if (s > sE) { sE = s; eI = i; eJ = j; bestL.copy(L); }
    }
  }

  const faceTol = 0.0005, edgeTol = 0.001;
  const useB = sB > sA + faceTol;
  const sFace = useB ? sB : sA;

  if (eI >= 0 && sE > sFace + edgeTol) {
    // edge-edge: one contact at the closest points of the two edges
    const n = man.normal.copy(bestL);
    if (d.dot(n) < 0) n.scale(-1);
    const pA = _t2.copy(A.position), pB = _t3.copy(B.position);
    for (let k = 0; k < 3; k++) {
      if (k !== eI) pA.addScaled(_ax[k], hv(hA, k) * (_ax[k].dot(n) > 0 ? 1 : -1));
      if (k !== eJ) pB.addScaled(_bx[k], -hv(hB, k) * (_bx[k].dot(n) > 0 ? 1 : -1));
    }
    const u = _ax[eI], v = _bx[eJ];
    const rx = pA.x - pB.x, ry = pA.y - pB.y, rz = pA.z - pB.z;
    const b = u.dot(v), c = u.x * rx + u.y * ry + u.z * rz, f = v.x * rx + v.y * ry + v.z * rz;
    const denom = 1 - b * b;
    let sParam = denom > 1e-9 ? (b * f - c) / denom : 0;
    const eA = hv(hA, eI), eB = hv(hB, eJ);
    sParam = Math.max(-eA, Math.min(eA, sParam));
    let tParam = b * sParam + f;
    tParam = Math.max(-eB, Math.min(eB, tParam));
    pA.addScaled(u, sParam); pB.addScaled(v, tParam);
    const s = (pB.x - pA.x) * n.x + (pB.y - pA.y) * n.y + (pB.z - pA.z) * n.z;
    candCount = 0;
    pushCandidate((pA.x + pB.x) * 0.5, (pA.y + pB.y) * 0.5, (pA.z + pB.z) * 0.5, s);
    reduceCandidates(man);
    return;
  }

  // face contact
  const ref = useB ? B : A, inc = useB ? A : B;
  const rAx = useB ? _bx : _ax, iAx = useB ? _ax : _bx;
  const rH = ref.halfExtents, iH = inc.halfExtents;
  const k = useB ? iB : iA;
  const nRef = _t1.copy(rAx[k]);
  const dRI = _t2.subVectors(inc.position, ref.position);
  if (dRI.dot(nRef) < 0) nRef.scale(-1);

  // incident face: the face of inc most anti-parallel to nRef
  let ki = 0, bestDot = -1;
  for (let j = 0; j < 3; j++) { const a = Math.abs(iAx[j].dot(nRef)); if (a > bestDot) { bestDot = a; ki = j; } }
  const incSign = iAx[ki].dot(nRef) > 0 ? -1 : 1;
  const k1 = (ki + 1) % 3, k2 = (ki + 2) % 3;
  const fc = _t3.copy(inc.position).addScaled(iAx[ki], incSign * hv(iH, ki));
  const e1 = hv(iH, k1), e2 = hv(iH, k2);
  const u1 = iAx[k1], u2 = iAx[k2];
  polyA[0].copy(fc).addScaled(u1, e1).addScaled(u2, e2);
  polyA[1].copy(fc).addScaled(u1, -e1).addScaled(u2, e2);
  polyA[2].copy(fc).addScaled(u1, -e1).addScaled(u2, -e2);
  polyA[3].copy(fc).addScaled(u1, e1).addScaled(u2, -e2);

  const r1 = (k + 1) % 3, r2 = (k + 2) % 3;
  let cnt = 4;
  cnt = clipPolygon(polyA, cnt, polyB, rAx[r1], ref.position, hv(rH, r1), 1);
  cnt = clipPolygon(polyB, cnt, polyA, rAx[r1], ref.position, hv(rH, r1), -1);
  cnt = clipPolygon(polyA, cnt, polyB, rAx[r2], ref.position, hv(rH, r2), 1);
  cnt = clipPolygon(polyB, cnt, polyA, rAx[r2], ref.position, hv(rH, r2), -1);
  if (cnt === 0) return;

  const rf = _t4.copy(ref.position).addScaled(nRef, hv(rH, k));  // a point on the reference face
  man.normal.copy(nRef);
  if (useB) man.normal.scale(-1);
  candCount = 0;
  for (let i = 0; i < cnt; i++) {
    const p = polyA[i];
    const s = (p.x - rf.x) * nRef.x + (p.y - rf.y) * nRef.y + (p.z - rf.z) * nRef.z;
    if (s <= margin) pushCandidate(p.x - nRef.x * s * 0.5, p.y - nRef.y * s * 0.5, p.z - nRef.z * s * 0.5, s);
  }
  reduceCandidates(man);
}

const SHAPE_ORDER = { plane: 0, box: 1, sphere: 2 };

// Fills man (a, b, normal a->b, points). Returns false if not touching.
function collide(x, y, margin, man) {
  let a = x, b = y;
  const oa = SHAPE_ORDER[a.shape], ob = SHAPE_ORDER[b.shape];
  if (oa > ob || (oa === ob && a.id > b.id)) { a = y; b = x; }
  man.a = a; man.b = b; man.count = 0; candCount = 0;
  if (a.shape === 'plane') {
    if (b.shape === 'sphere') collidePlaneSphere(a, b, margin, man);
    else if (b.shape === 'box') collidePlaneBox(a, b, margin, man);
  } else if (a.shape === 'box') {
    if (b.shape === 'box') collideBoxBox(a, b, margin, man);
    else collideBoxSphere(a, b, margin, man);
  } else {
    collideSphereSphere(a, b, margin, man);
  }
  return man.count > 0;
}

// ---------------------------------------------------------------------------
// World
// ---------------------------------------------------------------------------

function makeSoft(hertz, zeta, h) {
  if (hertz === 0) return { biasRate: 0, massScale: 1, impulseScale: 0 };
  const omega = 2 * Math.PI * hertz;
  const a1 = 2 * zeta + h * omega;
  const a2 = h * omega * a1;
  const a3 = 1 / (1 + a2);
  return { biasRate: omega / a1, massScale: a2 * a3, impulseScale: a3 };
}

function tangentBasis(n, t1, t2) {
  if (Math.abs(n.x) > 0.57735) t1.set(n.y, -n.x, 0); else t1.set(0, n.z, -n.y);
  t1.normalize();
  t2.crossVectors(n, t1);
}

export class World {
  constructor(opts = {}) {
    this.gravity = new Vec3(0, -9.81, 0);
    this.substeps = opts.substeps ?? 6;
    this.contactHertz = 120;
    this.contactHertzLimit = 0.25;        // contactHertz is clamped to this fraction of the substep rate
    this.contactDampingRatio = 10;
    this.contactPushMaxVelocity = 2;    // m/s, how fast overlaps get pushed apart
    this.restitutionThreshold = 0.4;    // m/s, slower impacts don't bounce
    this.linearSlop = 0;
    this.speculativeDistance = 0.01;
    this.sleepThreshold = 0.03;         // m/s
    this.timeToSleep = 0.5;             // s
    this.maxLinearSpeed = 40;
    this.maxAngularSpeed = 80;
    this.bodies = [];
    this.pairFilter = null;             // (a, b) => false to disable a pair
    this.onImpact = null;               // (a, b, point, speed) on the first touch of a pair

    this._planes = [];
    this._solids = [];                  // non-plane bodies, sorted by aabb.minX
    this._manifolds = [];               // active manifolds this step
    this._map = new Map();
    this._pool = [];
    this._pairs = [];
    this._stamp = 0;
    this._parent = [];
    this._lastH = 0;
    this.time = 0;
    this.stepTimeMs = 0;
  }

  addBody(b) {
    if (b.world === this) return b;
    b.world = this;
    this.bodies.push(b);
    if (b.shape === 'plane') this._planes.push(b); else this._solids.push(b);
    b.updateTransform();
    return b;
  }

  removeBody(b) {
    if (b.world !== this) return;
    b.world = null;
    const rm = (arr) => { const i = arr.indexOf(b); if (i >= 0) arr.splice(i, 1); };
    rm(this.bodies); rm(this._planes); rm(this._solids);
    if (b._sleepIsland) {
      for (const o of b._sleepIsland) if (o !== b) o.wake();
    }
  }

  wakeAll() { for (const b of this.bodies) b.wake(); }

  _acquire() { return this._pool.pop() || new Manifold(); }

  _canCollide(a, b) {
    if (a.type !== DYNAMIC && b.type !== DYNAMIC) return false;
    if (!(a.collisionGroup & b.collisionMask) || !(b.collisionGroup & a.collisionMask)) return false;
    if (this.pairFilter && !this.pairFilter(a, b)) return false;
    return true;
  }

  _isMoving(b) {
    if (b.type === STATIC) return false;
    const speed = b.velocity.length() + b.angularVelocity.length() * b.boundingRadius;
    return speed > this.sleepThreshold;
  }

  _updateAabb(b, dt) {
    const p = b.position, A = b.aabb, m = this.speculativeDistance * 0.5;
    let ex, ey, ez;
    if (b.shape === 'sphere') { ex = ey = ez = b.radius; }
    else {
      const R = b.R, h = b.halfExtents;
      ex = Math.abs(R[0]) * h.x + Math.abs(R[1]) * h.y + Math.abs(R[2]) * h.z;
      ey = Math.abs(R[3]) * h.x + Math.abs(R[4]) * h.y + Math.abs(R[5]) * h.z;
      ez = Math.abs(R[6]) * h.x + Math.abs(R[7]) * h.y + Math.abs(R[8]) * h.z;
    }
    let vx = 0, vy = 0, vz = 0, spin = 0;
    if (b.type !== STATIC && !b.sleeping) {
      vx = b.velocity.x * dt; vy = b.velocity.y * dt; vz = b.velocity.z * dt;
      spin = b.angularVelocity.length() * b.boundingRadius * dt;
    }
    A[0] = p.x - ex - m - spin + Math.min(0, vx); A[3] = p.x + ex + m + spin + Math.max(0, vx);
    A[1] = p.y - ey - m - spin + Math.min(0, vy); A[4] = p.y + ey + m + spin + Math.max(0, vy);
    A[2] = p.z - ez - m - spin + Math.min(0, vz); A[5] = p.z + ez + m + spin + Math.max(0, vz);
  }

  _pairMargin(a, b, dt) {
    let rel = 0;
    const va = a.type === STATIC || a.sleeping ? null : a.velocity;
    const vb = b.type === STATIC || b.sleeping ? null : b.velocity;
    if (va && vb) rel = Math.hypot(va.x - vb.x, va.y - vb.y, va.z - vb.z);
    else if (va) rel = va.length(); else if (vb) rel = vb.length();
    let spin = 0;
    if (va) spin += a.angularVelocity.length() * Math.min(a.boundingRadius, 1);
    if (vb) spin += b.angularVelocity.length() * Math.min(b.boundingRadius, 1);
    return Math.min(0.5, this.speculativeDistance + (rel + spin) * dt);
  }

  _findPairs(dt) {
    const pairs = this._pairs; pairs.length = 0;
    const solids = this._solids;
    for (const b of solids) this._updateAabb(b, dt);
    // insertion sort by minX (nearly sorted from the previous step)
    for (let i = 1; i < solids.length; i++) {
      const b = solids[i], key = b.aabb[0];
      let j = i - 1;
      while (j >= 0 && solids[j].aabb[0] > key) { solids[j + 1] = solids[j]; j--; }
      solids[j + 1] = b;
    }
    for (let i = 0; i < solids.length; i++) {
      const a = solids[i], A = a.aabb;
      for (let j = i + 1; j < solids.length; j++) {
        const b = solids[j], B = b.aabb;
        if (B[0] > A[3]) break;
        if (B[1] > A[4] || B[4] < A[1] || B[2] > A[5] || B[5] < A[2]) continue;
        if (!this._canCollide(a, b)) continue;
        pairs.push(a, b);
      }
    }
    for (const p of this._planes) {
      for (const b of solids) {
        if (b.type !== DYNAMIC) continue;
        const A = b.aabb, n = p.normal;
        // AABB vs plane: lowest corner along the normal
        const lo = (n.x > 0 ? A[0] : A[3]) * n.x + (n.y > 0 ? A[1] : A[4]) * n.y + (n.z > 0 ? A[2] : A[5]) * n.z;
        if (lo - p.offset > this.speculativeDistance) continue;
        if (!this._canCollide(p, b)) continue;
        pairs.push(p, b);
      }
    }
  }

  step(dt) {
    if (!(dt > 0)) return;
    const t0 = (typeof performance !== 'undefined' ? performance.now() : Date.now());
    const bodies = this.bodies;
    this._stamp++;

    for (const b of bodies) if (b.type !== STATIC || b.shape === 'plane') b.updateTransform();

    this._findPairs(dt);
    const pairs = this._pairs;
    const tmp = this._acquire();

    // Pass 1: wake sleeping bodies that are being touched by something moving.
    for (let i = 0; i < pairs.length; i += 2) {
      const a = pairs[i], b = pairs[i + 1];
      if (a.sleeping === b.sleeping) continue;
      const sleeper = a.sleeping ? a : b, other = a.sleeping ? b : a;
      if (other.type === STATIC) continue;
      const margin = this._pairMargin(a, b, dt);
      if (!collide(a, b, margin, tmp)) continue;
      let minS = Infinity;
      for (let k = 0; k < tmp.count; k++) minS = Math.min(minS, tmp.points[k].separation);
      if (minS < 0.001 || (this._isMoving(other) && minS < margin)) sleeper.wake();
    }

    // Pass 2: narrowphase + warm start matching.
    const oldMap = this._map, newMap = new Map(), active = this._manifolds;
    active.length = 0;
    for (let i = 0; i < pairs.length; i += 2) {
      const a = pairs[i], b = pairs[i + 1];
      const key = Math.min(a.id, b.id) * 1048576 + Math.max(a.id, b.id);
      const old = oldMap.get(key);
      const aIdle = a.sleeping || a.type === STATIC, bIdle = b.sleeping || b.type === STATIC;
      if (aIdle && bIdle) {
        if (old) { old.keepStamp = this._stamp; newMap.set(key, old); }
        continue;
      }
      const man = this._acquire();
      if (!collide(a, b, this._pairMargin(a, b, dt), man)) { this._pool.push(man); continue; }
      man.key = key;
      man.wasTouching = false;
      man.rollingImpulse.set(0, 0, 0);
      for (let k = 0; k < man.count; k++) {
        const cp = man.points[k];
        cp.localA.subVectors(cp.position, man.a.position); mulMTV(man.a.R, cp.localA, cp.localA);
        cp.localB.subVectors(cp.position, man.b.position); mulMTV(man.b.R, cp.localB, cp.localB);
        cp.normalImpulse = 0; cp.friction.set(0, 0, 0); cp.maxNormalImpulse = 0;
      }
      if (old && old.a === man.a) {
        man.wasTouching = old.touching;
        man.rollingImpulse.copy(old.rollingImpulse);
        const tol2 = 0.015 * 0.015;
        let used = 0;
        for (let k = 0; k < man.count; k++) {
          const cp = man.points[k];
          let best = -1, bestD = tol2;
          for (let m = 0; m < old.count; m++) {
            if (used & (1 << m)) continue;
            const d2 = _t0.subVectors(cp.localA, old.points[m].localA).lengthSq();
            if (d2 < bestD) { bestD = d2; best = m; }
          }
          if (best >= 0) {
            used |= 1 << best;
            cp.normalImpulse = old.points[best].normalImpulse;
            cp.friction.copy(old.points[best].friction);
          }
        }
      }
      newMap.set(key, man);
      active.push(man);
    }
    this._pool.push(tmp);
    for (const [key, m] of oldMap) if (newMap.get(key) !== m) this._pool.push(m);
    this._map = newMap;

    this._solve(dt);

    // Kinematic bodies end exactly on their targets.
    const moved = [];
    for (const b of this._solids) {
      if (b.type === KINEMATIC && b._kinTarget) {
        b.position.copy(b._kinTarget.p); b.quaternion.copy(b._kinTarget.q); b.updateTransform();
        b._kinTarget = null;
        moved.push(b);
      }
    }

    this._updateSleep(dt);
    this._emitImpacts();
    // A kinematic body only moves during steps it was given a target for.
    for (const b of moved) { b.velocity.set(0, 0, 0); b.angularVelocity.set(0, 0, 0); }
    this.time += dt;
    this.stepTimeMs = (typeof performance !== 'undefined' ? performance.now() : Date.now()) - t0;
  }

  _solve(dt) {
    const active = this._manifolds;
    const n = this.substeps, h = dt / n, invH = 1 / h;
    if (this._lastH > 0 && Math.abs(h - this._lastH) > 1e-9) {
      // stored impulses are per substep: rescale them to the new substep length
      const k = h / this._lastH;
      for (const man of active) {
        man.rollingImpulse.scale(k);
        for (let i = 0; i < man.count; i++) { man.points[i].normalImpulse *= k; man.points[i].friction.scale(k); }
      }
    }
    this._lastH = h;
    const contactHertz = Math.min(this.contactHertz, this.contactHertzLimit / h);
    const soft = makeSoft(contactHertz, this.contactDampingRatio, h);
    const staticSoft = makeSoft(2 * contactHertz, this.contactDampingRatio, h);
    const moving = [];
    for (const b of this._solids) {
      if (b.type === DYNAMIC && !b.sleeping) moving.push(b);
      else if (b.type === KINEMATIC) moving.push(b);
    }

    // prepare: anchors, Jacobians, effective masses
    for (const man of active) {
      const a = man.a, b = man.b, nrm = man.normal, t1 = man.t1, t2 = man.t2;
      const mA = a.invMass, mB = b.invMass, IA = a.invI, IB = b.invI;
      tangentBasis(nrm, t1, t2);
      man.friction = Math.sqrt(a.friction * b.friction);
      man.restitution = Math.max(a.restitution, b.restitution);
      man.softness = (mA === 0 || mB === 0) ? staticSoft : soft;
      const rr = (a.shape === 'sphere' ? a.rollingResistance * a.radius : 0) + (b.shape === 'sphere' ? b.rollingResistance * b.radius : 0);
      man.rolling = rr;
      if (rr > 0) {
        const k = (IA[0] + IA[4] + IA[8] + IB[0] + IB[4] + IB[8]) / 3;
        man.rollingMass = k > 0 ? 1 / k : 0;
      }
      for (let i = 0; i < man.count; i++) {
        const cp = man.points[i], J = cp.J;
        cp.rA.subVectors(cp.position, a.position);
        cp.rB.subVectors(cp.position, b.position);
        cp.normalMass = prepareAxis(mA, mB, IA, IB, cp.rA, cp.rB, nrm, J, 0);
        cp.tangentMass1 = prepareAxis(mA, mB, IA, IB, cp.rA, cp.rB, t1, J, 12);
        cp.tangentMass2 = prepareAxis(mA, mB, IA, IB, cp.rA, cp.rB, t2, J, 24);
        cp.relativeVelocity = axisVelocity(a, b, nrm, J, 0);
        cp.f1 = cp.friction.dot(t1);
        cp.f2 = cp.friction.dot(t2);
        cp.maxNormalImpulse = 0;
      }
    }

    const g = this.gravity, maxV = this.maxLinearSpeed, maxW = this.maxAngularSpeed;
    for (let sub = 0; sub < n; sub++) {
      // integrate velocities
      for (const b of moving) {
        if (b.type !== DYNAMIC) continue;
        b.velocity.addScaled(g, h * b.gravityScale);
        b.velocity.scale(1 / (1 + h * b.linearDamping));
        b.angularVelocity.scale(1 / (1 + h * b.angularDamping));
        const v2 = b.velocity.lengthSq(); if (v2 > maxV * maxV) b.velocity.scale(maxV / Math.sqrt(v2));
        const w2 = b.angularVelocity.lengthSq(); if (w2 > maxW * maxW) b.angularVelocity.scale(maxW / Math.sqrt(w2));
      }
      for (const man of active) warmStart(man);
      this._solveContacts(active, invH, true);
      for (const b of moving) integratePose(b, h);
      // relax: solve again without the position bias so it doesn't turn into bounce
      this._solveContacts(active, invH, false);
    }

    // restitution
    const thr = this.restitutionThreshold;
    for (const man of active) {
      if (man.restitution === 0) continue;
      const a = man.a, b = man.b, nrm = man.normal;
      for (let i = 0; i < man.count; i++) {
        const cp = man.points[i];
        if (cp.relativeVelocity > -thr || cp.maxNormalImpulse === 0) continue;
        const vn = axisVelocity(a, b, nrm, cp.J, 0);
        let imp = -cp.normalMass * (vn + man.restitution * cp.relativeVelocity);
        const newImp = Math.max(cp.normalImpulse + imp, 0);
        imp = newImp - cp.normalImpulse;
        cp.normalImpulse = newImp;
        cp.maxNormalImpulse = Math.max(cp.maxNormalImpulse, imp);
        applyAxisImpulse(a, b, nrm, cp.J, 0, imp);
      }
    }

    for (const man of active) {
      let touching = false;
      const t1 = man.t1, t2 = man.t2;
      for (let i = 0; i < man.count; i++) {
        const cp = man.points[i];
        if (cp.maxNormalImpulse > 0) touching = true;
        cp.friction.set(t1.x * cp.f1 + t2.x * cp.f2, t1.y * cp.f1 + t2.y * cp.f2, t1.z * cp.f1 + t2.z * cp.f2);
      }
      man.touching = touching;
    }
  }

  // One Gauss-Seidel pass over all contacts. Body velocities live in locals per manifold.
  _solveContacts(active, invH, useBias) {
    const slop = this.linearSlop, maxPush = this.contactPushMaxVelocity;
    for (let m = 0; m < active.length; m++) {
      const man = active[m];
      const a = man.a, b = man.b, sft = man.softness, mu = man.friction;
      const mA = a.invMass, mB = b.invMass;
      const va = a.velocity, wa = a.angularVelocity, vb = b.velocity, wb = b.angularVelocity;
      let vax = va.x, vay = va.y, vaz = va.z, wax = wa.x, way = wa.y, waz = wa.z;
      let vbx = vb.x, vby = vb.y, vbz = vb.z, wbx = wb.x, wby = wb.y, wbz = wb.z;
      const pa = a.position, pb = b.position, RA = a.R, RB = b.R;
      const nx = man.normal.x, ny = man.normal.y, nz = man.normal.z;
      const count = man.count, pts = man.points;
      let totalNormal = 0;

      for (let i = 0; i < count; i++) {
        const cp = pts[i], J = cp.J, la = cp.localA, lb = cp.localB;
        // current separation from the anchors' current positions
        const dx = pb.x + RB[0] * lb.x + RB[1] * lb.y + RB[2] * lb.z - (pa.x + RA[0] * la.x + RA[1] * la.y + RA[2] * la.z);
        const dy = pb.y + RB[3] * lb.x + RB[4] * lb.y + RB[5] * lb.z - (pa.y + RA[3] * la.x + RA[4] * la.y + RA[5] * la.z);
        const dz = pb.z + RB[6] * lb.x + RB[7] * lb.y + RB[8] * lb.z - (pa.z + RA[6] * la.x + RA[7] * la.y + RA[8] * la.z);
        const s = cp.separation + dx * nx + dy * ny + dz * nz;
        let bias = 0, massScale = 1, impulseScale = 0;
        if (s > 0) {
          bias = s * invH;   // speculative: allow closing the gap, no more
        } else if (useBias) {
          bias = Math.max(sft.biasRate * Math.min(0, s + slop), -maxPush);
          massScale = sft.massScale;
          impulseScale = sft.impulseScale;
        }
        const vn = (vbx - vax) * nx + (vby - vay) * ny + (vbz - vaz) * nz
          + J[6] * wbx + J[7] * wby + J[8] * wbz - J[0] * wax - J[1] * way - J[2] * waz;
        let imp = -cp.normalMass * massScale * (vn + bias) - impulseScale * cp.normalImpulse;
        const newImp = Math.max(cp.normalImpulse + imp, 0);
        imp = newImp - cp.normalImpulse;
        cp.normalImpulse = newImp;
        if (imp > cp.maxNormalImpulse) cp.maxNormalImpulse = imp;
        totalNormal += newImp;
        const ia = mA * imp, ib = mB * imp;
        vax -= nx * ia; vay -= ny * ia; vaz -= nz * ia;
        wax -= J[3] * imp; way -= J[4] * imp; waz -= J[5] * imp;
        vbx += nx * ib; vby += ny * ib; vbz += nz * ib;
        wbx += J[9] * imp; wby += J[10] * imp; wbz += J[11] * imp;
      }

      // friction (circular cone)
      const t1x = man.t1.x, t1y = man.t1.y, t1z = man.t1.z, t2x = man.t2.x, t2y = man.t2.y, t2z = man.t2.z;
      for (let i = 0; i < count; i++) {
        const cp = pts[i], J = cp.J;
        const dvx = vbx - vax, dvy = vby - vay, dvz = vbz - vaz;
        const v1 = dvx * t1x + dvy * t1y + dvz * t1z + J[18] * wbx + J[19] * wby + J[20] * wbz - J[12] * wax - J[13] * way - J[14] * waz;
        const v2 = dvx * t2x + dvy * t2y + dvz * t2z + J[30] * wbx + J[31] * wby + J[32] * wbz - J[24] * wax - J[25] * way - J[26] * waz;
        let n1 = cp.f1 - v1 * cp.tangentMass1, n2 = cp.f2 - v2 * cp.tangentMass2;
        const maxF = mu * cp.normalImpulse;
        const len2 = n1 * n1 + n2 * n2;
        if (len2 > maxF * maxF) { const k = maxF / Math.sqrt(len2); n1 *= k; n2 *= k; }
        const d1 = n1 - cp.f1, d2 = n2 - cp.f2;
        cp.f1 = n1; cp.f2 = n2;
        const px = t1x * d1 + t2x * d2, py = t1y * d1 + t2y * d2, pz = t1z * d1 + t2z * d2;
        vax -= px * mA; vay -= py * mA; vaz -= pz * mA;
        wax -= J[15] * d1 + J[27] * d2; way -= J[16] * d1 + J[28] * d2; waz -= J[17] * d1 + J[29] * d2;
        vbx += px * mB; vby += py * mB; vbz += pz * mB;
        wbx += J[21] * d1 + J[33] * d2; wby += J[22] * d1 + J[34] * d2; wbz += J[23] * d1 + J[35] * d2;
      }

      // rolling resistance (spheres)
      if (man.rolling > 0 && man.rollingMass > 0) {
        const R = man.rollingImpulse, IA = a.invI, IB = b.invI;
        const ox = R.x, oy = R.y, oz = R.z;
        let rx = ox - (wbx - wax) * man.rollingMass;
        let ry = oy - (wby - way) * man.rollingMass;
        let rz = oz - (wbz - waz) * man.rollingMass;
        const maxR = man.rolling * totalNormal;
        const len = Math.hypot(rx, ry, rz);
        if (len > maxR) { const k = maxR / len; rx *= k; ry *= k; rz *= k; }
        R.set(rx, ry, rz);
        const x = rx - ox, y = ry - oy, z = rz - oz;
        wax -= IA[0] * x + IA[1] * y + IA[2] * z; way -= IA[3] * x + IA[4] * y + IA[5] * z; waz -= IA[6] * x + IA[7] * y + IA[8] * z;
        wbx += IB[0] * x + IB[1] * y + IB[2] * z; wby += IB[3] * x + IB[4] * y + IB[5] * z; wbz += IB[6] * x + IB[7] * y + IB[8] * z;
      }

      if (mA > 0) { va.x = vax; va.y = vay; va.z = vaz; wa.x = wax; wa.y = way; wa.z = waz; }
      if (mB > 0) { vb.x = vbx; vb.y = vby; vb.z = vbz; wb.x = wbx; wb.y = wby; wb.z = wbz; }
    }
  }

  _updateSleep(dt) {
    const solids = this._solids, parent = this._parent;
    const dyn = [];
    for (const b of solids) {
      if (b.type !== DYNAMIC || b.sleeping) { b._index = -1; continue; }
      b._index = dyn.length; dyn.push(b);
      if (!b.canSleep || this._isMoving(b)) b.sleepTime = 0; else b.sleepTime += dt;
    }
    parent.length = dyn.length;
    for (let i = 0; i < dyn.length; i++) parent[i] = i;
    const find = (i) => { while (parent[i] !== i) { parent[i] = parent[parent[i]]; i = parent[i]; } return i; };
    for (const man of this._manifolds) {
      const a = man.a, b = man.b;
      let minS = Infinity;
      for (let i = 0; i < man.count; i++) minS = Math.min(minS, man.points[i].separation);
      if (minS > this.speculativeDistance * 0.5) continue;
      if (a._index >= 0 && b._index >= 0) {
        const ra = find(a._index), rb = find(b._index);
        if (ra !== rb) parent[ra] = rb;
      } else if (a.type === KINEMATIC || b.type === KINEMATIC) {
        const k = a.type === KINEMATIC ? a : b, d = k === a ? b : a;
        if (d._index >= 0 && this._isMoving(k)) d.sleepTime = 0;
      }
    }
    const islands = new Map();
    for (let i = 0; i < dyn.length; i++) {
      const r = find(i);
      let isl = islands.get(r);
      if (!isl) { isl = []; islands.set(r, isl); }
      isl.push(dyn[i]);
    }
    for (const isl of islands.values()) {
      let minT = Infinity;
      for (const b of isl) minT = Math.min(minT, b.sleepTime);
      if (minT < this.timeToSleep) continue;
      for (const b of isl) {
        b.sleeping = true; b._sleepIsland = isl;
        b.velocity.set(0, 0, 0); b.angularVelocity.set(0, 0, 0);
      }
    }
  }

  _emitImpacts() {
    if (!this.onImpact) return;
    for (const man of this._manifolds) {
      if (!man.touching || man.wasTouching) continue;
      let speed = 0, best = null;
      for (let i = 0; i < man.count; i++) {
        const cp = man.points[i];
        if (cp.maxNormalImpulse > 0 && -cp.relativeVelocity > speed) { speed = -cp.relativeVelocity; best = cp; }
      }
      if (best && speed > 0.15) this.onImpact(man.a, man.b, best.position, speed);
    }
  }

  /** Closest hit along a ray. filter(body) => false to skip. Returns {body, distance, point, normal} or null. */
  raycast(origin, dir, maxDist = 100, filter = null) {
    let best = null, bestT = maxDist;
    const d = _t0.copy(dir).normalize();
    const n = new Vec3();
    for (const b of this.bodies) {
      if (filter && !filter(b)) continue;
      const t = rayBody(b, origin, d, bestT, n);
      if (t >= 0 && t < bestT) {
        bestT = t;
        best = { body: b, distance: t, point: new Vec3().copy(origin).addScaled(d, t), normal: n.clone() };
      }
    }
    return best;
  }
}

// --- solver helpers ---------------------------------------------------------

// Fills J[o..o+11] for direction d and returns the effective mass along d.
function prepareAxis(mA, mB, IA, IB, rA, rB, d, J, o) {
  const ax = rA.y * d.z - rA.z * d.y, ay = rA.z * d.x - rA.x * d.z, az = rA.x * d.y - rA.y * d.x;
  const bx = rB.y * d.z - rB.z * d.y, by = rB.z * d.x - rB.x * d.z, bz = rB.x * d.y - rB.y * d.x;
  const iax = IA[0] * ax + IA[1] * ay + IA[2] * az, iay = IA[3] * ax + IA[4] * ay + IA[5] * az, iaz = IA[6] * ax + IA[7] * ay + IA[8] * az;
  const ibx = IB[0] * bx + IB[1] * by + IB[2] * bz, iby = IB[3] * bx + IB[4] * by + IB[5] * bz, ibz = IB[6] * bx + IB[7] * by + IB[8] * bz;
  J[o] = ax; J[o + 1] = ay; J[o + 2] = az; J[o + 3] = iax; J[o + 4] = iay; J[o + 5] = iaz;
  J[o + 6] = bx; J[o + 7] = by; J[o + 8] = bz; J[o + 9] = ibx; J[o + 10] = iby; J[o + 11] = ibz;
  const k = mA + mB + ax * iax + ay * iay + az * iaz + bx * ibx + by * iby + bz * ibz;
  return k > 0 ? 1 / k : 0;
}
function axisVelocity(a, b, d, J, o) {
  const va = a.velocity, vb = b.velocity, wa = a.angularVelocity, wb = b.angularVelocity;
  return (vb.x - va.x) * d.x + (vb.y - va.y) * d.y + (vb.z - va.z) * d.z
    + J[o + 6] * wb.x + J[o + 7] * wb.y + J[o + 8] * wb.z - J[o] * wa.x - J[o + 1] * wa.y - J[o + 2] * wa.z;
}
function applyAxisImpulse(a, b, d, J, o, imp) {
  if (a.invMass > 0) {
    const m = a.invMass * imp;
    a.velocity.x -= d.x * m; a.velocity.y -= d.y * m; a.velocity.z -= d.z * m;
    a.angularVelocity.x -= J[o + 3] * imp; a.angularVelocity.y -= J[o + 4] * imp; a.angularVelocity.z -= J[o + 5] * imp;
  }
  if (b.invMass > 0) {
    const m = b.invMass * imp;
    b.velocity.x += d.x * m; b.velocity.y += d.y * m; b.velocity.z += d.z * m;
    b.angularVelocity.x += J[o + 9] * imp; b.angularVelocity.y += J[o + 10] * imp; b.angularVelocity.z += J[o + 11] * imp;
  }
}
function warmStart(man) {
  const a = man.a, b = man.b, mA = a.invMass, mB = b.invMass;
  const va = a.velocity, wa = a.angularVelocity, vb = b.velocity, wb = b.angularVelocity;
  const n = man.normal, t1 = man.t1, t2 = man.t2;
  for (let i = 0; i < man.count; i++) {
    const cp = man.points[i], J = cp.J, s = cp.normalImpulse, f1 = cp.f1, f2 = cp.f2;
    const px = n.x * s + t1.x * f1 + t2.x * f2, py = n.y * s + t1.y * f1 + t2.y * f2, pz = n.z * s + t1.z * f1 + t2.z * f2;
    if (mA > 0) {
      va.x -= px * mA; va.y -= py * mA; va.z -= pz * mA;
      wa.x -= J[3] * s + J[15] * f1 + J[27] * f2; wa.y -= J[4] * s + J[16] * f1 + J[28] * f2; wa.z -= J[5] * s + J[17] * f1 + J[29] * f2;
    }
    if (mB > 0) {
      vb.x += px * mB; vb.y += py * mB; vb.z += pz * mB;
      wb.x += J[9] * s + J[21] * f1 + J[33] * f2; wb.y += J[10] * s + J[22] * f1 + J[34] * f2; wb.z += J[11] * s + J[23] * f1 + J[35] * f2;
    }
  }
  if (man.rolling > 0) {
    const R = man.rollingImpulse;
    if (mA > 0) { const I = a.invI; wa.x -= I[0] * R.x + I[1] * R.y + I[2] * R.z; wa.y -= I[3] * R.x + I[4] * R.y + I[5] * R.z; wa.z -= I[6] * R.x + I[7] * R.y + I[8] * R.z; }
    if (mB > 0) { const I = b.invI; wb.x += I[0] * R.x + I[1] * R.y + I[2] * R.z; wb.y += I[3] * R.x + I[4] * R.y + I[5] * R.z; wb.z += I[6] * R.x + I[7] * R.y + I[8] * R.z; }
  }
}
function integratePose(b, h) {
  const v = b.velocity, w = b.angularVelocity, q = b.quaternion;
  b.position.x += v.x * h; b.position.y += v.y * h; b.position.z += v.z * h;
  const hh = 0.5 * h;
  const qx = q.x, qy = q.y, qz = q.z, qw = q.w;
  q.x += hh * (w.x * qw + w.y * qz - w.z * qy);
  q.y += hh * (-w.x * qz + w.y * qw + w.z * qx);
  q.z += hh * (w.x * qy - w.y * qx + w.z * qw);
  q.w += hh * (-w.x * qx - w.y * qy - w.z * qz);
  q.normalize();
  quatToMat(q, b.R);
}

// --- raycasts ---------------------------------------------------------------

const _ro = new Vec3(), _rd = new Vec3();
function rayBody(b, o, d, maxT, outN) {
  if (b.shape === 'plane') {
    const denom = b.normal.dot(d);
    if (denom >= -1e-9) return -1;
    const t = (b.offset - b.normal.dot(o)) / denom;
    if (t < 0 || t > maxT) return -1;
    outN.copy(b.normal);
    return t;
  }
  if (b.shape === 'sphere') {
    const ox = o.x - b.position.x, oy = o.y - b.position.y, oz = o.z - b.position.z;
    const bq = ox * d.x + oy * d.y + oz * d.z;
    const c = ox * ox + oy * oy + oz * oz - b.radius * b.radius;
    const disc = bq * bq - c;
    if (disc < 0) return -1;
    const t = -bq - Math.sqrt(disc);
    if (t < 0 || t > maxT) return -1;
    outN.set(ox + d.x * t, oy + d.y * t, oz + d.z * t).normalize();
    return t;
  }
  // box: slab test in local space
  mulMTV(b.R, _ro.subVectors(o, b.position), _ro);
  mulMTV(b.R, d, _rd);
  const h = b.halfExtents;
  let tmin = -Infinity, tmax = Infinity, axis = -1, sgn = 0;
  for (let i = 0; i < 3; i++) {
    const oi = hv(_ro, i), di = hv(_rd, i), hi = hv(h, i);
    if (Math.abs(di) < 1e-12) { if (Math.abs(oi) > hi) return -1; continue; }
    let t1 = (-hi - oi) / di, t2 = (hi - oi) / di, s = -1;
    if (t1 > t2) { const tt = t1; t1 = t2; t2 = tt; s = 1; }
    if (t1 > tmin) { tmin = t1; axis = i; sgn = s; }
    if (t2 < tmax) tmax = t2;
    if (tmin > tmax) return -1;
  }
  if (tmin < 0 || tmin > maxT || axis < 0) return -1;
  outN.set(axis === 0 ? sgn : 0, axis === 1 ? sgn : 0, axis === 2 ? sgn : 0);
  mulMV(b.R, outN, outN);
  return tmin;
}
