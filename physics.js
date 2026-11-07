// physics.js: rigid bodies that collide using nothing but distance functions.
//
// There are no meshes and no convex pieces. A body is an SDF, and the questions
// a physics engine asks are answered by evaluating SDFs:
//
//   mass and inertia   integrate over the points where d < 0
//   surface samples    project random points onto d = 0
//   is this touching?  evaluate the other SDF at a surface point
//   contact normal     the gradient of that SDF
//   penetration depth  the (negative) distance itself
//   tunnelling         a point that moved further than the distances at both
//                      ends of its path allow must have crossed a surface
//   thin or sharp      walking downhill on max(dA, dB) finds the deepest point
//   overlaps           shared by two solids, wherever the samples happen to be
//
// The world is treated like any other body, except that it never moves.
//
// The solver is substepped position-based dynamics, after Müller et al.,
// "Detailed Rigid Body Simulation with Extended Position Based Dynamics" (2020).

const PHYSICS = {
  gravity: 9.8,
  substeps: 10,
  iterations: 3,
  searchEvery: 3,      // substeps between full contact searches
  fastSpeed: 10,       // a body this fast is searched for on every substep
  margin: 0.012,       // how close a sample must be before it is refined
  thinSpacing: 1.4,    // more samples once they are this many half-thicknesses apart
  friction: 0.6,
  restitution: 0.2,
  rollingResistance: 0.012, // a torque, as a length: how far the load sits off-centre
  slop: 0.002,         // overlap allowed at rest, so resting contacts stay in touch
  // Overlap left over is repaired gently when shallow and faster when deep. Fast
  // repairs of shallow overlaps fed energy into anything also held by the pointer.
  repairSpeed: 0.5,    // units per second, for overlap just past the allowance
  repairPerDepth: 4,   // extra speed per unit of overlap
  maxPushSpeed: 6,     // never faster than this
  maxSpeed: 40,
  sleepSpeed: 0.12,
  sleepAfter: 0.6,
  stepsPerSecond: 60,
  grab: {
    stiffness: 0.12,   // fraction of the gap to the pointer closed per substep
    maxForce: 8,       // strongest pull, in multiples of the body's weight in normal gravity
    damping: 3,        // per second, while held
    spinDamping: 6,    // per second, while held
    smoothing: 0.35,   // fraction of the way to the pointer the target moves per step
  },
};

// Gradient of a distance function, by tetrahedral differences.
const gradientOf = (f, x, y, z, e = 0.0015) => {
  const a = f(x + e, y - e, z - e);
  const b = f(x - e, y - e, z + e);
  const c = f(x - e, y + e, z - e);
  const d = f(x + e, y + e, z + e);
  const nx = a - b - c + d;
  const ny = -a - b + c + d;
  const nz = -a + b - c + d;
  const len = Math.hypot(nx, ny, nz) || 1;
  return [nx / len, ny / len, nz / len];
};

const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross3 = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const mulMat3 = (m, v) => [
  m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
  m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
  m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
];
const mulMat3T = (m, v) => [
  m[0] * v[0] + m[3] * v[1] + m[6] * v[2],
  m[1] * v[0] + m[4] * v[1] + m[7] * v[2],
  m[2] * v[0] + m[5] * v[1] + m[8] * v[2],
];
const invertMat3 = (m) => {
  const [a, b, c, d, e, f, g, h, i] = m;
  const A = e * i - f * h;
  const B = f * g - d * i;
  const C = d * h - e * g;
  const det = a * A + b * B + c * C;
  const s = 1 / det;
  return [A * s, (c * h - b * i) * s, (b * f - c * e) * s,
    B * s, (a * i - c * g) * s, (c * d - a * f) * s,
    C * s, (b * g - a * h) * s, (a * e - b * d) * s];
};
const matFromQuat = ([x, y, z, w]) => [
  1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
  2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
  2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
];
const normalizeQuat = (q) => {
  const len = Math.hypot(q[0], q[1], q[2], q[3]);
  return [q[0] / len, q[1] / len, q[2] / len, q[3] / len];
};
// Adds a small rotation, given as a vector, to a quaternion.
const rotateQuat = (q, t) => {
  const [x, y, z, w] = q;
  return normalizeQuat([
    x + 0.5 * (t[0] * w + t[1] * z - t[2] * y),
    y + 0.5 * (t[1] * w + t[2] * x - t[0] * z),
    z + 0.5 * (t[2] * w + t[0] * y - t[1] * x),
    w + 0.5 * (-t[0] * x - t[1] * y - t[2] * z),
  ]);
};
// Degrees, in the same x, then y, then z order as shape.rotate().
const quatFromEuler = (rx = 0, ry = 0, rz = 0) => {
  const half = (d) => (d * Math.PI) / 360;
  const [cx, sx, cy, sy, cz, sz] = [half(rx), half(ry), half(rz)].flatMap((a) => [Math.cos(a), Math.sin(a)]);
  return [
    sx * cy * cz - cx * sy * sz,
    cx * sy * cz + sx * cy * sz,
    cx * cy * sz - sx * sy * cz,
    cx * cy * cz + sx * sy * sz,
  ];
};

// A small repeatable random generator, so that a simulation can be replayed.
const makeRandom = (seed) => () => {
  seed = (seed + 0x6d2b79f5) | 0;
  let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
  t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
};

const analysisCache = new WeakMap();

// Everything physics needs to know about a shape, measured from its SDF alone.
// Samples are spread over the surface, so a thin, curved part such as a ring's tube
// can fall between them. When the gap between samples is large compared with how
// thick the shape is, it gets twice as many.
const analyseShape = (shape, sampleCount = null) => {
  if (analysisCache.has(shape)) return analysisCache.get(shape);
  const d = shape.d;
  const random = makeRandom(12345);

  // Pulling a point onto d = 0 works from anywhere, which also finds how far a
  // shape reaches. Marching in along fixed directions can miss thin shapes.
  const project = (px, py, pz) => {
    let x = px;
    let y = py;
    let z = pz;
    let dist = 1;
    for (let it = 0; it < 16 && Math.abs(dist) > 2e-4; it++) {
      dist = d(x, y, z);
      const g = gradientOf(d, x, y, z);
      x -= g[0] * dist;
      y -= g[1] * dist;
      z -= g[2] * dist;
    }
    return Math.abs(d(x, y, z)) < 1e-3 ? [x, y, z] : null;
  };
  const candidates = [];
  const far = 32;
  for (let n = 0; n < 500; n++) {
    const p = project((random() * 2 - 1) * far, (random() * 2 - 1) * far, (random() * 2 - 1) * far);
    if (p) candidates.push(p);
  }
  const bounds = (list) => [0, 1, 2].map((i) => [Math.min(...list.map((p) => p[i])), Math.max(...list.map((p) => p[i]))]);

  // Then again from inside those bounds, which reaches concave parts and interiors.
  for (let round = 0; round < 2; round++) {
    const b = bounds(candidates);
    for (let n = 0; n < 900; n++) {
      const p = project(
        b[0][0] + random() * (b[0][1] - b[0][0]),
        b[1][0] + random() * (b[1][1] - b[1][0]),
        b[2][0] + random() * (b[2][1] - b[2][0]),
      );
      if (p) candidates.push(p);
    }
  }

  // Mass properties: add up small cells over the shape's tight bounds, with the
  // same number along each axis so a thin bar is measured as well as a cube. A cell
  // cut by the surface counts in part, which the distance at its centre tells us.
  const [bx, by, bz] = bounds(candidates);
  const cells = 40;
  const cx = (bx[1] - bx[0]) / cells;
  const cy = (by[1] - by[0]) / cells;
  const cz = (bz[1] - bz[0]) / cells;
  const blur = Math.min(cx, cy, cz);
  const com = [0, 0, 0];
  const inside = [];
  let volume = 0;
  let deepest = 0; // the deepest interior distance is half the shape's thickness
  for (let i = -1; i <= cells; i++) {
    for (let j = -1; j <= cells; j++) {
      for (let k = -1; k <= cells; k++) {
        const x = bx[0] + (i + 0.5) * cx;
        const y = by[0] + (j + 0.5) * cy;
        const z = bz[0] + (k + 0.5) * cz;
        const dist = d(x, y, z);
        if (dist < deepest) deepest = dist;
        const fill = Math.max(0, Math.min(1, 0.5 - dist / blur));
        if (fill <= 0) continue;
        inside.push([x, y, z, fill]);
        volume += fill;
        com[0] += x * fill;
        com[1] += y * fill;
        com[2] += z * fill;
      }
    }
  }
  com[0] /= volume;
  com[1] /= volume;
  com[2] /= volume;
  const unit = cx * cy * cz;
  const inertia = [0, 0, 0, 0, 0, 0, 0, 0, 0];
  // Each cell's own inertia, about its centre.
  const selfX = (cy * cy + cz * cz) / 12;
  const selfY = (cx * cx + cz * cz) / 12;
  const selfZ = (cx * cx + cy * cy) / 12;
  for (const [px, py, pz, fill] of inside) {
    const x = px - com[0];
    const y = py - com[1];
    const z = pz - com[2];
    inertia[0] += (y * y + z * z + selfX) * fill;
    inertia[4] += (x * x + z * z + selfY) * fill;
    inertia[8] += (x * x + y * y + selfZ) * fill;
    inertia[1] -= x * y * fill;
    inertia[2] -= x * z * fill;
    inertia[5] -= y * z * fill;
  }
  inertia[3] = inertia[1];
  inertia[6] = inertia[2];
  inertia[7] = inertia[5];
  for (let i = 0; i < 9; i++) inertia[i] *= unit;

  // Keep a well-spread set of samples. Choosing each point furthest from those
  // already chosen finds corners without being told what a corner is.
  const nearest = candidates.map((p) => Math.hypot(p[0] - com[0], p[1] - com[1], p[2] - com[2]) * -1);
  let next = nearest.indexOf(Math.min(...nearest));
  const chosen = [];
  let spacing = Math.hypot(bx[1] - bx[0], by[1] - by[0], bz[1] - bz[0]);
  nearest.fill(Infinity);
  const halfThickness = -deepest;
  const most = sampleCount || 64;
  const spacings = [];
  for (let n = 0; n < most && candidates.length; n++) {
    const pick = candidates[next];
    chosen.push(pick);
    let best = -1;
    let bestIndex = 0;
    for (let i = 0; i < candidates.length; i++) {
      const c = candidates[i];
      nearest[i] = Math.min(nearest[i], Math.hypot(c[0] - pick[0], c[1] - pick[1], c[2] - pick[2]));
      if (nearest[i] > best) { best = nearest[i]; bestIndex = i; }
    }
    spacing = best;
    spacings.push(best);
    next = bestIndex;
  }
  let count = sampleCount || 32;
  if (!sampleCount && spacings[31] > PHYSICS.thinSpacing * halfThickness) count = 64;
  chosen.length = Math.min(chosen.length, count);
  spacing = spacings[Math.min(count, spacings.length) - 1];
  const points = chosen.map((p) => [p[0] - com[0], p[1] - com[1], p[2] - com[2]]);
  const radius = Math.max(...candidates.map((p) => Math.hypot(p[0] - com[0], p[1] - com[1], p[2] - com[2])));
  const renderRadius = Math.max(...candidates.map((p) => Math.hypot(p[0], p[1], p[2])));

  const result = {
    d, volume: volume * unit, com, inertia, halfThickness, invInertia: invertMat3(inertia), points, spacing, radius, renderRadius,
  };
  analysisCache.set(shape, result);
  return result;
};

// The deepest point inside both solids, or null if they do not overlap. First
// project onto whichever solid the point is outside of, until it is inside both.
// Then walk downhill on max(dA, dB). Needs nothing but the two distance functions.
const deepestOverlap = (fa, fb, sx, sy, sz) => {
  let x = sx;
  let y = sy;
  let z = sz;
  let a = fa(x, y, z);
  let b = fb(x, y, z);
  for (let i = 0; i < 10 && (a >= 0 || b >= 0); i++) {
    const outside = a >= b ? fa : fb;
    const d = Math.max(a, b);
    const n = gradientOf(outside, x, y, z);
    x -= n[0] * (d + 1e-4);
    y -= n[1] * (d + 1e-4);
    z -= n[2] * (d + 1e-4);
    a = fa(x, y, z);
    b = fb(x, y, z);
  }
  if (a >= 0 || b >= 0) return null;

  let f = Math.max(a, b);
  let step = Math.max(-f, 0.004);
  for (let i = 0; i < 12 && step > 2e-4; i++) {
    const na = gradientOf(fa, x, y, z);
    const nb = gradientOf(fb, x, y, z);
    const close = step * 0.5;
    const dx = a > b + close ? na[0] : b > a + close ? nb[0] : na[0] + nb[0];
    const dy = a > b + close ? na[1] : b > a + close ? nb[1] : na[1] + nb[1];
    const dz = a > b + close ? na[2] : b > a + close ? nb[2] : na[2] + nb[2];
    const len = Math.hypot(dx, dy, dz);
    if (len < 0.05) break; // the two surfaces face each other, so this is the bottom
    const tx = x - (dx / len) * step;
    const ty = y - (dy / len) * step;
    const tz = z - (dz / len) * step;
    const ta = fa(tx, ty, tz);
    const tb = fb(tx, ty, tz);
    const trial = Math.max(ta, tb);
    if (trial < f) {
      x = tx;
      y = ty;
      z = tz;
      a = ta;
      b = tb;
      f = trial;
      step *= 1.3;
    } else {
      step *= 0.5;
    }
  }
  return { x, y, z, deepest: -f };
};

class RigidBody {
  constructor(shape, { at = [0, 0, 0], rotate = [0, 0, 0], density = 1, friction, restitution, velocity = [0, 0, 0], spin = [0, 0, 0] } = {}) {
    this.shape = shape;
    this.info = analyseShape(shape);
    this.mass = density * this.info.volume;
    this.invMass = 1 / this.mass;
    this.invInertiaLocal = this.info.invInertia.map((v) => v / density);
    this.friction = friction ?? PHYSICS.friction;
    this.restitution = restitution ?? PHYSICS.restitution;
    this.start = { at, rotate, velocity, spin };
    // Distance to this body from a world position.
    this.dist = (x, y, z) => {
      const R = this.R;
      const px = x - this.x[0];
      const py = y - this.x[1];
      const pz = z - this.x[2];
      const c = this.info.com;
      return this.info.d(
        R[0] * px + R[3] * py + R[6] * pz + c[0],
        R[1] * px + R[4] * py + R[7] * pz + c[1],
        R[2] * px + R[5] * py + R[8] * pz + c[2],
      );
    };
    this.reset();
  }

  reset() {
    const { at, rotate, velocity, spin } = this.start;
    this.q = quatFromEuler(...rotate);
    this.R = matFromQuat(this.q);
    // `at` places the shape's own origin, like shape.move() would.
    const c = mulMat3(this.R, this.info.com);
    this.x = [at[0] + c[0], at[1] + c[1], at[2] + c[2]];
    this.v = [...velocity];
    this.w = [...spin];
    this.xPrev = [...this.x];
    this.qPrev = [...this.q];
    this.RPrev = [...this.R];
    this.xSearch = null;
    this.RSearch = null;
    this.sleeping = false;
    this.still = 0;
    this.prepare();
  }

  // Refreshes what the solver reads during a substep. Sleeping bodies act immovable.
  prepare() {
    this.dx = [0, 0, 0];
    this.dth = [0, 0, 0];
    this.repairDx = [0, 0, 0];
    this.repairDth = [0, 0, 0];
    if (this.sleeping) {
      this.im = 0;
      this.invI = [0, 0, 0, 0, 0, 0, 0, 0, 0];
      return;
    }
    this.im = this.invMass;
    // R * invILocal * R^T
    const R = this.R;
    const L = this.invInertiaLocal;
    const c0 = mulMat3(R, [L[0], L[3], L[6]]);
    const c1 = mulMat3(R, [L[1], L[4], L[7]]);
    const c2 = mulMat3(R, [L[2], L[5], L[8]]);
    this.invI = [
      c0[0] * R[0] + c1[0] * R[1] + c2[0] * R[2], c0[0] * R[3] + c1[0] * R[4] + c2[0] * R[5], c0[0] * R[6] + c1[0] * R[7] + c2[0] * R[8],
      c0[1] * R[0] + c1[1] * R[1] + c2[1] * R[2], c0[1] * R[3] + c1[1] * R[4] + c2[1] * R[5], c0[1] * R[6] + c1[1] * R[7] + c2[1] * R[8],
      c0[2] * R[0] + c1[2] * R[1] + c2[2] * R[2], c0[2] * R[3] + c1[2] * R[4] + c2[2] * R[5], c0[2] * R[6] + c1[2] * R[7] + c2[2] * R[8],
    ];
  }

  // How hard it is to move a point at r along direction n, as an inverse mass.
  resistance(r, n) {
    const c = cross3(r, n);
    return this.im + dot3(c, mulMat3(this.invI, c));
  }

  applyPush(r, p, sign) {
    const P = [p[0] * sign, p[1] * sign, p[2] * sign];
    this.dx[0] += P[0] * this.im;
    this.dx[1] += P[1] * this.im;
    this.dx[2] += P[2] * this.im;
    const t = mulMat3(this.invI, cross3(r, P));
    this.dth[0] += t[0];
    this.dth[1] += t[1];
    this.dth[2] += t[2];
  }

  // The same movement, but only tidying up an overlap. It changes where the body
  // is, and must not count towards how fast it is going.
  applyRepair(r, p, sign) {
    const P = [p[0] * sign, p[1] * sign, p[2] * sign];
    const t = mulMat3(this.invI, cross3(r, P));
    for (let i = 0; i < 3; i++) {
      this.dx[i] += P[i] * this.im;
      this.dth[i] += t[i];
      this.repairDx[i] += P[i] * this.im;
      this.repairDth[i] += t[i];
    }
  }

  // Movement of a point at r so far during this substep's solve.
  pushed(r) {
    const t = cross3(this.dth, r);
    return [this.dx[0] + t[0], this.dx[1] + t[1], this.dx[2] + t[2]];
  }

  // The fastest any point on the body can be moving.
  get pointSpeed() { return Math.hypot(...this.v) + Math.hypot(...this.w) * this.info.radius; }

  // Like pushed(), without the repairs: the movement that counts as real motion.
  movedForReal(r) {
    const t = cross3([
      this.dth[0] - this.repairDth[0], this.dth[1] - this.repairDth[1], this.dth[2] - this.repairDth[2],
    ], r);
    return [this.dx[0] - this.repairDx[0] + t[0], this.dx[1] - this.repairDx[1] + t[1], this.dx[2] - this.repairDx[2] + t[2]];
  }

  velocityAt(r) {
    const t = cross3(this.w, r);
    return [this.v[0] + t[0], this.v[1] + t[1], this.v[2] + t[2]];
  }

  applyImpulse(r, p, sign) {
    const P = [p[0] * sign, p[1] * sign, p[2] * sign];
    this.v[0] += P[0] * this.im;
    this.v[1] += P[1] * this.im;
    this.v[2] += P[2] * this.im;
    const t = mulMat3(this.invI, cross3(r, P));
    this.w[0] += t[0];
    this.w[1] += t[1];
    this.w[2] += t[2];
  }

  // The transform that draws this body: world = R * (local - com) + x.
  renderTranslation() {
    const c = mulMat3(this.R, this.info.com);
    return [this.x[0] - c[0], this.x[1] - c[1], this.x[2] - c[2]];
  }
}

class PhysicsWorld {
  constructor({ world = () => Infinity, floor = true, substeps = PHYSICS.substeps, gravity = PHYSICS.gravity } = {}) {
    this.tree = world;
    this.floor = floor;
    this.substeps = substeps;
    this.gravity = gravity;
    this.bodies = [];
    this.edits = [];
    this.grab = null;
    this.manifolds = [];
    this.overlapMemory = new Map();
    this.accumulator = 0;
    this.stats = { stepMs: 0, contacts: 0, queries: 0, sweeps: 0 };
    // What the world's surface is right now, including live edits.
    this.worldDist = (x, y, z) => {
      let d = this.tree(x, y, z);
      for (const e of this.edits) {
        const px = x - e.pos[0];
        const py = y - e.pos[1];
        const pz = z - e.pos[2];
        let de;
        if (e.kind === 0) {
          de = Math.hypot(px, py, pz) - e.size[0];
        } else {
          const qx = Math.abs(px) - (e.size[0] - 0.02);
          const qy = Math.abs(py) - (e.size[1] - 0.02);
          const qz = Math.abs(pz) - (e.size[2] - 0.02);
          de = Math.hypot(Math.max(qx, 0), Math.max(qy, 0), Math.max(qz, 0))
            + Math.min(Math.max(qx, qy, qz), 0) - 0.02;
        }
        d = e.op > 0.5 ? (e.k > 0 ? SDF.smin(d, de, e.k) : Math.min(d, de)) : SDF.smax(d, -de, e.k);
      }
      return this.floor ? Math.min(d, y) : d;
    };
  }

  add(shape, options) {
    const body = new RigidBody(shape, options);
    body.id = this.bodies.length;
    this.bodies.push(body);
    return body;
  }

  reset() {
    for (const b of this.bodies) b.reset();
    this.grab = null;
    this.manifolds = [];
    this.overlapMemory.clear();
    this.accumulator = 0;
  }

  get awakeCount() { return this.bodies.reduce((n, b) => n + (b.sleeping ? 0 : 1), 0); }

  wake(body) {
    body.sleeping = false;
    body.still = 0;
  }

  wakeAll() { for (const b of this.bodies) this.wake(b); }

  // The ground changed, so anything resting on it may need to move.
  setEdits(edits) {
    this.edits = edits;
    this.wakeAll();
  }

  bodyAt(x, y, z) {
    let best = null;
    let bestD = 0.03;
    for (const b of this.bodies) {
      const d = b.dist(x, y, z);
      if (d < bestD) { best = b; bestD = d; }
    }
    return best;
  }

  grabBody(body, point) {
    const r = [point[0] - body.x[0], point[1] - body.x[1], point[2] - body.x[2]];
    this.wake(body);
    this.grab = { body, local: mulMat3T(body.R, r), target: [...point], goal: [...point] };
  }

  moveGrab(target) {
    if (!this.grab) return;
    this.grab.goal = target;
    this.wake(this.grab.body);
  }

  releaseGrab() { this.grab = null; }

  // Advances by real time in fixed steps, so results do not depend on frame rate.
  advance(dt) {
    const fixed = 1 / PHYSICS.stepsPerSecond;
    this.accumulator = Math.min(this.accumulator + dt, fixed * 3);
    const start = performance.now();
    let steps = 0;
    while (this.accumulator >= fixed) {
      this.step(fixed);
      this.accumulator -= fixed;
      steps++;
    }
    if (steps) this.stats.stepMs += ((performance.now() - start) / steps - this.stats.stepMs) * 0.1;
  }

  step(dt) {
    const h = dt / this.substeps;
    this.stats.contacts = 0;
    if (this.grab) {
      // Pointers move in jumps. Ease towards them, so a jump is not a kick.
      const { target, goal } = this.grab;
      for (let i = 0; i < 3; i++) target[i] += (goal[i] - target[i]) * PHYSICS.grab.smoothing;
    }
    for (let i = 0; i < this.substeps; i++) {
      const fast = this.bodies.some((b) => !b.sleeping && b.pointSpeed > PHYSICS.fastSpeed);
      this.substep(h, fast || i % PHYSICS.searchEvery === 0);
    }
    this.updateSleep(dt);
  }

  substep(h, search) {
    const { bodies } = this;
    for (const b of bodies) {
      b.prepare();
      b.xPrev = [...b.x];
      b.qPrev = [...b.q];
      b.RPrev = b.R;
      if (b.sleeping) continue;
      b.v[1] -= this.gravity * h;
      const speed = Math.hypot(b.v[0], b.v[1], b.v[2]);
      if (speed > PHYSICS.maxSpeed) {
        const s = PHYSICS.maxSpeed / speed;
        b.v = b.v.map((c) => c * s);
      }
      b.x = [b.x[0] + b.v[0] * h, b.x[1] + b.v[1] * h, b.x[2] + b.v[2] * h];
      b.q = rotateQuat(b.q, [b.w[0] * h, b.w[1] * h, b.w[2] * h]);
      b.R = matFromQuat(b.q);
    }

    let contacts;
    if (search) {
      contacts = this.findContacts(h);
      for (const b of bodies) {
        b.xSearch = b.x;
        b.RSearch = b.R;
      }
      this.manifolds = contacts;
    } else {
      // Whatever is being dragged moves unpredictably, so it is always searched.
      const held = this.grab && !this.grab.body.sleeping ? this.grab.body : null;
      const others = held ? this.manifolds.filter((m) => m.A !== held && m.B !== held) : this.manifolds;
      contacts = this.refreshContacts(others);
      if (held) {
        contacts.push(...this.findContacts(h, held));
        held.xSearch = held.x;
        held.RSearch = held.R;
      }
      this.manifolds = contacts;
    }
    this.stats.contacts = Math.max(this.stats.contacts, contacts.reduce((n, m) => n + m.contacts.length, 0));

    // Solve positions. Contacts share the load on the early passes, so that no
    // single point takes it all, and the last pass removes what overlap remains.
    this.solveGrab(h);
    for (let pass = 0; pass < PHYSICS.iterations; pass++) {
      for (const m of contacts) {
        const share = pass === PHYSICS.iterations - 1 ? 1 : 1 / Math.max(1, m.contacts.length - pass);
        for (const c of m.contacts) this.solveNormal(c, h, share);
      }
    }
    for (const m of contacts) this.solvePatchFriction(m, h);
    for (const m of contacts) for (const c of m.contacts) this.solveNormal(c, h, 1);
    for (const b of bodies) {
      if (b.sleeping) continue;
      b.x = [b.x[0] + b.dx[0], b.x[1] + b.dx[1], b.x[2] + b.dx[2]];
      b.q = rotateQuat(b.q, b.dth);
      b.R = matFromQuat(b.q);

      // Velocity is whatever motion the positions ended up describing.
      // Except for repairs: speed that came from tidying an overlap would be energy
      // from nowhere, and would feed the next collision.
      b.v = [
        (b.x[0] - b.xPrev[0] - b.repairDx[0]) / h,
        (b.x[1] - b.xPrev[1] - b.repairDx[1]) / h,
        (b.x[2] - b.xPrev[2] - b.repairDx[2]) / h,
      ];
      const [px, py, pz, pw] = b.qPrev;
      const [x, y, z, w] = b.q;
      const rw = w * pw + x * px + y * py + z * pz;
      const sign = (rw < 0 ? -2 : 2) / h;
      b.w = [
        sign * (x * pw - w * px - y * pz + z * py) - b.repairDth[0] / h,
        sign * (y * pw - w * py - z * px + x * pz) - b.repairDth[1] / h,
        sign * (z * pw - w * pz - x * py + y * px) - b.repairDth[2] / h,
      ];
    }
    for (const m of contacts) this.solveVelocity(m, h);

    // Velocity is rebuilt from positions above, so damping has to come after it.
    if (this.grab && !this.grab.body.sleeping) {
      const { body } = this.grab;
      const slow = Math.exp(-PHYSICS.grab.damping * h);
      const slowSpin = Math.exp(-PHYSICS.grab.spinDamping * h);
      body.v = body.v.map((c) => c * slow);
      body.w = body.w.map((c) => c * slowSpin);
    }
  }

  // Every awake body, or only the pairs involving `only`.
  findContacts(h, only = null) {
    const { bodies } = this;
    const contacts = [];
    const horizon = only ? h : h * PHYSICS.searchEvery;
    for (const b of bodies) {
      if (b.sleeping || (only && b !== only)) continue;
      // The world's distance at the centre tells us if the whole body is clear.
      if (this.worldDist(b.x[0], b.x[1], b.x[2]) < b.info.radius + PHYSICS.margin + b.pointSpeed * horizon) {
        const seeds = this.collide(b, null, this.worldDist, contacts, horizon);
        this.collideOverlap(b, null, this.worldDist, seeds, contacts);
      }
    }
    for (let i = 0; i < bodies.length; i++) {
      for (let j = i + 1; j < bodies.length; j++) {
        const a = bodies[i];
        const b = bodies[j];
        if ((a.sleeping && b.sleeping) || (only && a !== only && b !== only)) continue;
        const reach = a.info.radius + b.info.radius + PHYSICS.margin;
        const dx = a.x[0] - b.x[0];
        const dy = a.x[1] - b.x[1];
        const dz = a.x[2] - b.x[2];
        if (dx * dx + dy * dy + dz * dz > reach * reach) continue;
        // Each body's samples are tested against the other's distance function.
        const seeds = [
          ...this.collide(a, b, b.dist, contacts, horizon),
          ...this.collide(b, a, a.dist, contacts, horizon),
        ];
        this.collideOverlap(a, b, b.dist, seeds, contacts);
      }
    }
    return contacts;
  }

  // Between searches, a contact stays on the same material point of its body.
  // Reading the distance there again is far cheaper than searching.
  refreshContacts(manifolds) {
    const live = [];
    for (const m of manifolds) {
      const { A, B, field } = m;
      if (m.overlap) {
        // The deepest point moves as the bodies do, so follow it from where it was.
        const r = mulMat3(A.R, m.contacts[0].local);
        const hit = deepestOverlap(A.dist, field, A.x[0] + r[0], A.x[1] + r[1], A.x[2] + r[2]);
        const contact = hit && this.overlapContact(A, B, field, hit);
        if (contact) live.push({ A, B, field, overlap: true, contacts: [contact] });
        continue;
      }
      const contacts = [];
      for (const c of m.contacts) {
        const rA = mulMat3(A.R, c.local);
        const x = A.x[0] + rA[0];
        const y = A.x[1] + rA[1];
        const z = A.x[2] + rA[2];
        const d = field(x, y, z);
        if (d >= 0) continue;
        const n = gradientOf(field, x, y, z);
        const rB = B ? [x - B.x[0], y - B.x[1], z - B.x[2]] : null;
        contacts.push({
          A, B, n, depth: -d, swept: false, rA, rB, local: c.local, lambda: 0,
          vnBefore: dot3(this.relativeVelocity(A, B, rA, rB), n),
          prevA: this.previousOffset(A, rA),
          prevB: B ? this.previousOffset(B, rB) : null,
        });
      }
      if (contacts.length) live.push({ A, B, field, contacts });
    }
    return live;
  }

  // Finds where body A's surface meets the distance function `field`, which
  // belongs to body B, or to the world if B is null.
  collide(A, B, field, contacts, h) {
    const { points, spacing } = A.info;
    const found = [];
    const R = A.R;
    const moving = !B && !!A.xSearch && A.pointSpeed * h > 0.01;
    let low1 = null;
    let low2 = null;
    let lowD1 = Infinity;
    let lowD2 = Infinity;
    for (const p of points) {
      const r = mulMat3(R, p);
      let x = A.x[0] + r[0];
      let y = A.x[1] + r[1];
      let z = A.x[2] + r[2];
      let d = field(x, y, z);
      if (d < lowD1) {
        low2 = low1;
        lowD2 = lowD1;
        low1 = [x, y, z];
        lowD1 = d;
      } else if (d < lowD2) {
        low2 = [x, y, z];
        lowD2 = d;
      }

      if (moving && d > 0) {
        // A point that travelled further than the distances at both ends of its
        // path could have passed through a surface. Only then do we look closer.
        const rp = mulMat3(A.RSearch, p);
        const sx = A.xSearch[0] + rp[0];
        const sy = A.xSearch[1] + rp[1];
        const sz = A.xSearch[2] + rp[2];
        const length = Math.hypot(x - sx, y - sy, z - sz);
        const dStart = field(sx, sy, sz);
        if (length > dStart + d && dStart > 0) {
          this.stats.sweeps++;
          const hit = this.sweep(field, [sx, sy, sz], [(x - sx) / length, (y - sy) / length, (z - sz) / length], length);
          if (hit) {
            const n = gradientOf(field, hit.point[0], hit.point[1], hit.point[2]);
            const dir = [(x - sx) / length, (y - sy) / length, (z - sz) / length];
            const depth = (length - hit.t) * Math.abs(dot3(dir, n));
            found.push({ x, y, z, n, depth, swept: true });
            continue;
          }
        }
      }
      if (d > PHYSICS.margin) continue;

      // A sample rarely lands on the true contact. Slide it along A's surface,
      // downhill on `field`, so curved bodies roll instead of rocking on samples.
      let step = spacing * 0.5;
      let misses = 0;
      for (let it = 0; it < 12 && misses < 4; it++) {
        const n = gradientOf(field, x, y, z);
        const nA = gradientOf(A.dist, x, y, z);
        const along = dot3(n, nA);
        const gx = n[0] - along * nA[0];
        const gy = n[1] - along * nA[1];
        const gz = n[2] - along * nA[2];
        // The surfaces are parallel here, so this is the true contact.
        if (Math.hypot(gx, gy, gz) < 0.004) break;
        let tx = x - gx * step;
        let ty = y - gy * step;
        let tz = z - gz * step;
        const off = A.dist(tx, ty, tz);
        const back = gradientOf(A.dist, tx, ty, tz);
        tx -= back[0] * off;
        ty -= back[1] * off;
        tz -= back[2] * off;
        const trial = field(tx, ty, tz);
        if (trial < d) {
          x = tx;
          y = ty;
          z = tz;
          d = trial;
          step *= 1.3;
          misses = 0;
        } else {
          step *= 0.4;
          misses++;
        }
      }
      if (d >= 0) continue;
      found.push({ x, y, z, n: null, depth: -d, swept: false });
    }

    // Samples that slid to the same place are one contact. Keep the deepest.
    found.sort((a, b) => b.depth - a.depth);
    const merge = spacing * 0.25;
    const kept = [];
    for (const f of found) {
      if (!kept.some((k) => Math.hypot(k.x - f.x, k.y - f.y, k.z - f.z) < merge)) kept.push(f);
    }
    const seeds = low2 ? [low1, low2] : low1 ? [low1] : [];
    if (!kept.length) return seeds;

    // Four well-spread points hold any patch. More only rock against each other,
    // which reads as extra load and makes friction too strong.
    while (kept.length > 4) {
      let closest = Infinity;
      let drop = -1;
      for (let i = 1; i < kept.length; i++) {
        for (let j = 0; j < kept.length; j++) {
          if (i === j) continue;
          const gap = Math.hypot(kept[i].x - kept[j].x, kept[i].y - kept[j].y, kept[i].z - kept[j].z);
          if (gap < closest) {
            closest = gap;
            drop = i;
          }
        }
      }
      kept.splice(drop, 1);
    }
    const manifold = { A, B, field, contacts: [] };
    contacts.push(manifold);
    for (const f of kept) {
      const n = f.n || gradientOf(field, f.x, f.y, f.z);
      const rA = [f.x - A.x[0], f.y - A.x[1], f.z - A.x[2]];
      const rB = B ? [f.x - B.x[0], f.y - B.x[1], f.z - B.x[2]] : null;
      const vRel = this.relativeVelocity(A, B, rA, rB);
      manifold.contacts.push({
        A, B, n, depth: f.depth, swept: f.swept, rA, rB, local: mulMat3T(A.R, rA), lambda: 0, vnBefore: dot3(vRel, n),
        prevA: this.previousOffset(A, rA),
        prevB: B ? this.previousOffset(B, rB) : null,
      });
    }
    return seeds;
  }

  // Samples can miss an overlap between thin bodies entirely. This asks the
  // distance functions directly, from a few places likely to find it. Concave
  // bodies, such as two linked rings, can overlap in several separate places, so
  // every distinct overlap found becomes its own contact.
  collideOverlap(A, B, field, seeds, contacts) {
    const key = B ? `${A.id}:${B.id}` : `${A.id}`;
    const memory = this.overlapMemory.get(key) || [];
    const starts = memory.map((known) => {
      const r = mulMat3(A.R, known);
      return [A.x[0] + r[0], A.x[1] + r[1], A.x[2] + r[2]];
    });
    starts.push(B
      ? [(A.x[0] + B.x[0]) / 2, (A.x[1] + B.x[1]) / 2, (A.x[2] + B.x[2]) / 2]
      : [...A.x]);
    starts.push(...seeds);

    const apart = Math.min(A.info.spacing, B ? B.info.spacing : Infinity) * 0.5;
    const hits = [];
    let misses = 0;
    for (const s of starts) {
      // Most pairs are not touching at all, so do not keep looking for long.
      if (!hits.length && misses >= 2) break;
      const hit = deepestOverlap(A.dist, field, s[0], s[1], s[2]);
      if (!hit) {
        misses++;
        continue;
      }
      const same = hits.find((h) => Math.hypot(h.x - hit.x, h.y - hit.y, h.z - hit.z) < apart);
      if (!same) hits.push(hit);
      else if (hit.deepest > same.deepest) Object.assign(same, hit);
    }
    if (!hits.length) {
      this.overlapMemory.delete(key);
      return;
    }
    this.overlapMemory.set(key, hits.map((h) => mulMat3T(A.R, [h.x - A.x[0], h.y - A.x[1], h.z - A.x[2]])));

    // These contacts exist to catch what the samples miss. A sample already holding
    // the same spot about as deeply has it covered, and more load there would only
    // distort friction.
    const sampled = [];
    for (const m of contacts) {
      if (!((m.A === A && m.B === B) || (m.A === B && m.B === A))) continue;
      for (const c of m.contacts) sampled.push({ x: c.rA[0] + m.A.x[0], y: c.rA[1] + m.A.x[1], z: c.rA[2] + m.A.x[2], depth: c.depth });
    }
    for (const hit of hits) {
      const contact = this.overlapContact(A, B, field, hit);
      if (!contact) continue;
      // Faces resting on each other have a whole plateau of near-equal points. Those
      // are not separate overlaps, and samples are already holding them up.
      if (sampled.length && contact.depth <= Math.max(...sampled.map((c) => c.depth)) * 1.25 + PHYSICS.slop) continue;
      const covered = sampled.some((c) => Math.hypot(c.x - hit.x, c.y - hit.y, c.z - hit.z) < apart * 1.5
        && contact.depth <= c.depth * 1.25 + PHYSICS.slop);
      if (!covered) contacts.push({ A, B, field, overlap: true, contacts: [contact] });
    }
  }

  overlapContact(A, B, field, hit) {
    // Half the overlap is on each side of the deepest point. Too little to matter
    // at rest, where samples are already holding the body up.
    if (hit.deepest * 2 < PHYSICS.slop * 1.5) return null;
    // Sharp edges have a gradient that flips between faces, so measure across a
    // width that averages over the ridge.
    const e = Math.max(hit.deepest, 0.003);
    const difference = (x, y, z) => field(x, y, z) - A.dist(x, y, z);
    const n = gradientOf(difference, hit.x, hit.y, hit.z, e);
    // Faces meeting head-on separate by twice the deepest distance. Faces at an
    // angle to the push separate more slowly, so they need further to go.
    const na = gradientOf(A.dist, hit.x, hit.y, hit.z);
    const nb = gradientOf(field, hit.x, hit.y, hit.z);
    const square = Math.max((Math.abs(dot3(n, na)) + Math.abs(dot3(n, nb))) / 2, 0.5);
    const depth = (hit.deepest * 2) / square;
    const rA = [hit.x - A.x[0], hit.y - A.x[1], hit.z - A.x[2]];
    const rB = B ? [hit.x - B.x[0], hit.y - B.x[1], hit.z - B.x[2]] : null;
    return {
      A, B, n, depth, swept: false, soft: true, rA, rB, local: mulMat3T(A.R, rA), lambda: 0,
      vnBefore: dot3(this.relativeVelocity(A, B, rA, rB), n),
      prevA: this.previousOffset(A, rA),
      prevB: B ? this.previousOffset(B, rB) : null,
    };
  }

  // March along a path using distances; returns the first time it meets a surface.
  sweep(field, start, dir, length) {
    let t = 0;
    for (let i = 0; i < 64 && t < length; i++) {
      const point = [start[0] + dir[0] * t, start[1] + dir[1] * t, start[2] + dir[2] * t];
      const d = field(point[0], point[1], point[2]);
      if (d < 0.0005) return { t, point };
      t += d;
    }
    return null;
  }

  relativeVelocity(A, B, rA, rB) {
    const va = A.velocityAt(rA);
    if (!B) return va;
    const vb = B.velocityAt(rB);
    return [va[0] - vb[0], va[1] - vb[1], va[2] - vb[2]];
  }

  // Where this material point on the body was at the start of the substep.
  previousOffset(body, r) {
    const local = mulMat3T(body.R, r);
    const rp = mulMat3(body.RPrev, local);
    return [body.xPrev[0] + rp[0] - body.x[0], body.xPrev[1] + rp[1] - body.x[1], body.xPrev[2] + rp[2] - body.x[2]];
  }

  // A pull like a hand's: it has a strongest force, in proportion to the body's
  // weight. Pressed against something, it presses. It cannot overpower the contact
  // and send the body spinning.
  solveGrab(h) {
    if (!this.grab) return;
    const { body, local, target } = this.grab;
    if (body.sleeping) return;
    const r = mulMat3(body.R, local);
    const e = [target[0] - body.x[0] - r[0], target[1] - body.x[1] - r[1], target[2] - body.x[2] - r[2]];
    const len = Math.hypot(...e);
    if (len < 1e-5) return;
    const n = [e[0] / len, e[1] / len, e[2] / len];
    const { stiffness, maxForce } = PHYSICS.grab;
    const lambda = Math.min((len * stiffness) / body.resistance(r, n), maxForce * body.mass * PHYSICS.gravity * h * h);
    body.applyPush(r, [n[0] * lambda, n[1] * lambda, n[2] * lambda], 1);
  }

  solveNormal(c, h, share) {
    const { A, B, n, rA, rB } = c;
    const moved = A.pushed(rA);
    if (B) {
      const mb = B.pushed(rB);
      moved[0] -= mb[0];
      moved[1] -= mb[1];
      moved[2] -= mb[2];
    }
    // Deep overlaps separate gradually. Swept hits must stop now.
    // A contact always stops the approach, however shallow it is, or a resting body
    // would sink, get caught and bounce forever. The allowance only decides whether
    // existing overlap is worth repairing.
    const closing = Math.max(-c.vnBefore * h, 0);
    let allowed;
    if (c.swept) {
      allowed = c.depth;
    } else if (c.soft) {
      // A single point on a long body is a lever, so ease out of what is left.
      allowed = closing + Math.min(Math.max(c.depth - PHYSICS.slop - closing, 0) * 0.2, this.repairLimit(c.depth) * h);
    } else {
      allowed = Math.max(closing, Math.min(c.depth - PHYSICS.slop, this.repairLimit(c.depth) * h));
    }
    const remaining = allowed - dot3(moved, n);
    if (remaining <= 0) return;
    const wn = A.resistance(rA, n) + (B ? B.resistance(rB, n) : 0);
    if (wn <= 0) return;

    // A push has two jobs. Stopping real motion into the contact is a real impulse:
    // that is the bodies' own approach, plus anything neighbours have pressed them
    // with, which is how load travels down a stack. Anything beyond that only removes
    // overlap left from before, so it moves the body without giving it any speed.
    const real = A.movedForReal(rA);
    if (B) {
      const rb = B.movedForReal(rB);
      real[0] -= rb[0];
      real[1] -= rb[1];
      real[2] -= rb[2];
    }
    const approach = Math.max(closing - dot3(real, n), 0);
    const stopping = Math.min(remaining, approach) * share;
    const repairing = (remaining - Math.min(remaining, approach)) * share;
    if (stopping > 0) {
      const lambda = stopping / wn;
      c.lambda += lambda;
      const push = [n[0] * lambda, n[1] * lambda, n[2] * lambda];
      A.applyPush(rA, push, 1);
      if (B) B.applyPush(rB, push, -1);
    }
    if (repairing > 0) {
      const lambda = repairing / wn;
      const push = [n[0] * lambda, n[1] * lambda, n[2] * lambda];
      A.applyRepair(rA, push, 1);
      if (B) B.applyRepair(rB, push, -1);
    }
  }

  repairLimit(depth) {
    return Math.min(PHYSICS.repairSpeed + PHYSICS.repairPerDepth * depth, PHYSICS.maxPushSpeed);
  }

  // All contacts between two bodies act as one patch. Friction applied at the
  // patch's load-weighted centre cannot twist a block the way friction at one
  // off-centre point would, and the patch's size gives real torsional friction.
  summarisePatch(m, h) {
    const { A, B, contacts } = m;
    let total = 0;
    const rA = [0, 0, 0];
    const rB = [0, 0, 0];
    const n = [0, 0, 0];
    for (const c of contacts) {
      total += c.lambda;
      for (let i = 0; i < 3; i++) {
        rA[i] += c.rA[i] * c.lambda;
        if (B) rB[i] += c.rB[i] * c.lambda;
        n[i] += c.n[i] * c.lambda;
      }
    }
    if (total <= 0) return null;
    for (let i = 0; i < 3; i++) {
      rA[i] /= total;
      rB[i] /= total;
    }
    const len = Math.hypot(...n) || 1;
    n[0] /= len;
    n[1] /= len;
    n[2] /= len;
    let spread = 0;
    let closing = 0;
    for (const c of contacts) {
      spread += c.lambda * Math.hypot(c.rA[0] - rA[0], c.rA[1] - rA[1], c.rA[2] - rA[2]);
      closing += c.lambda * c.vnBefore;
    }
    const friction = B ? Math.sqrt(A.friction * B.friction) : A.friction;

    // Friction depends on the load being carried. Pushes that only ease old overlap
    // out carry nothing, so the load is what stopping the approach takes, plus
    // whatever other contacts are pressing these two bodies together with.
    const linear = A.im + (B ? B.im : 0);
    let load = total;
    if (linear > 0) {
      let carried = (Math.max(-closing / total, 0) * h) / linear;
      if (A.im > 0) carried += Math.max(total - dot3(A.dx, n) / A.im, 0);
      if (B && B.im > 0) carried += Math.max(dot3(B.dx, n) / B.im + total, 0);
      load = Math.min(total, carried);
    }
    return { A, B, total: load, rA, rB, n, spread: Math.max(spread / total, 0.01), friction };
  }

  // Static friction: undo sideways slip, up to what the normal load allows.
  solvePatchFriction(m, h) {
    const patch = this.summarisePatch(m, h);
    if (!patch) return;
    const { A, B, rA, rB, n, total, friction } = patch;
    // Repairs do not count as slip. Resisting one would turn it back into real speed.
    const prevA = this.previousOffset(A, rA);
    const a = A.movedForReal(rA);
    const slip = [a[0] + rA[0] - prevA[0], a[1] + rA[1] - prevA[1], a[2] + rA[2] - prevA[2]];
    if (B) {
      const prevB = this.previousOffset(B, rB);
      const b = B.movedForReal(rB);
      slip[0] -= b[0] + rB[0] - prevB[0];
      slip[1] -= b[1] + rB[1] - prevB[1];
      slip[2] -= b[2] + rB[2] - prevB[2];
    }
    const along = dot3(slip, n);
    const t = [slip[0] - along * n[0], slip[1] - along * n[1], slip[2] - along * n[2]];
    const slipLength = Math.hypot(...t);
    if (slipLength < 1e-9) return;
    const dir = [t[0] / slipLength, t[1] / slipLength, t[2] / slipLength];
    const wt = A.resistance(rA, dir) + (B ? B.resistance(rB, dir) : 0);
    if (wt <= 0) return;
    const lambda = slipLength / wt;
    if (lambda <= friction * total) {
      const push = [-dir[0] * lambda, -dir[1] * lambda, -dir[2] * lambda];
      A.applyPush(rA, push, 1);
      if (B) B.applyPush(rB, push, -1);
    }
  }

  solveVelocity(m, h) {
    const patch = this.summarisePatch(m, h);
    if (!patch) return;
    const { A, B, rA, rB, n, total, friction, spread } = patch;

    // Sliding friction.
    const v = this.relativeVelocity(A, B, rA, rB);
    const vn = dot3(v, n);
    const t = [v[0] - vn * n[0], v[1] - vn * n[1], v[2] - vn * n[2]];
    const vt = Math.hypot(...t);
    if (vt > 1e-9) {
      const dir = [t[0] / vt, t[1] / vt, t[2] / vt];
      const w = A.resistance(rA, dir) + (B ? B.resistance(rB, dir) : 0);
      if (w > 0) {
        const impulse = Math.min((friction * total) / h, vt / w);
        const p = [-dir[0] * impulse, -dir[1] * impulse, -dir[2] * impulse];
        A.applyImpulse(rA, p, 1);
        if (B) B.applyImpulse(rB, p, -1);
      }
    }

    // Twisting and rolling lose a little too, so that bodies do eventually stop.
    this.resistSpin(patch, (friction * total * spread) / h, PHYSICS.rollingResistance * A.mass * this.gravity * h);

    // Each contact stops its own approach, with a bounce if it was fast enough.
    for (const c of m.contacts) {
      if (c.lambda <= 0) continue;
      const u = this.relativeVelocity(A, B, c.rA, c.rB);
      const un = dot3(u, c.n);
      const e = Math.abs(un) > 4 * this.gravity * h ? (B ? Math.min(A.restitution, B.restitution) : A.restitution) : 0;
      const change = -un + Math.max(-e * c.vnBefore, 0);
      if (Math.abs(change) < 1e-9) continue;
      const w = A.resistance(c.rA, c.n) + (B ? B.resistance(c.rB, c.n) : 0);
      if (w <= 0) continue;
      const impulse = change / w;
      const p = [c.n[0] * impulse, c.n[1] * impulse, c.n[2] * impulse];
      A.applyImpulse(c.rA, p, 1);
      if (B) B.applyImpulse(c.rB, p, -1);
    }
  }

  resistSpin({ A, B, n }, twistLimit, rollLimit) {
    const relative = B ? [A.w[0] - B.w[0], A.w[1] - B.w[1], A.w[2] - B.w[2]] : A.w;
    const about = dot3(relative, n);
    const roll = [relative[0] - about * n[0], relative[1] - about * n[1], relative[2] - about * n[2]];
    const reduce = (axisVector, speed, limit) => {
      if (speed < 1e-6) return;
      const axis = axisVector.map((c) => c / speed);
      const weight = dot3(axis, mulMat3(A.invI, axis)) + (B ? dot3(axis, mulMat3(B.invI, axis)) : 0);
      if (weight <= 0) return;
      const L = Math.min(limit, speed / weight);
      const ta = mulMat3(A.invI, axis);
      A.w[0] -= ta[0] * L;
      A.w[1] -= ta[1] * L;
      A.w[2] -= ta[2] * L;
      if (B) {
        const tb = mulMat3(B.invI, axis);
        B.w[0] += tb[0] * L;
        B.w[1] += tb[1] * L;
        B.w[2] += tb[2] * L;
      }
    };
    reduce(n.map((c) => c * about), Math.abs(about), twistLimit);
    reduce(roll, Math.hypot(...roll), rollLimit);
  }

  updateSleep(dt) {
    const { bodies } = this;
    const held = this.grab && this.grab.body;
    for (const b of bodies) {
      if (b.sleeping) continue;
      const r = b.info.radius;
      const speed = Math.hypot(...b.v) + Math.hypot(...b.w) * r;
      if (speed < PHYSICS.sleepSpeed && b !== held) {
        b.still += dt;
        if (b.still > PHYSICS.sleepAfter) {
          b.sleeping = true;
          b.v = [0, 0, 0];
          b.w = [0, 0, 0];
        }
      } else {
        b.still = 0;
      }
    }
    // A moving body wakes what it is actually touching. Waking everything within its
    // bounding sphere let one tremor set off bodies across the scene, each of which
    // set off more.
    for (const m of this.manifolds) {
      if (!m.B || m.A.sleeping === m.B.sleeping) continue;
      const moving = m.A.sleeping ? m.B : m.A;
      const resting = m.A.sleeping ? m.A : m.B;
      if (moving.pointSpeed > 0.3) this.wake(resting);
    }
  }

  totalEnergy() {
    let e = 0;
    for (const b of this.bodies) {
      e += b.mass * this.gravity * b.x[1] + 0.5 * b.mass * dot3(b.v, b.v);
      const L = mulMat3T(b.R, b.w);
      const inertia = b.info.inertia.map((v) => (v * b.mass) / b.info.volume);
      e += 0.5 * dot3(L, mulMat3(inertia, L));
    }
    return e;
  }
}
