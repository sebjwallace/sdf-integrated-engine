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
  friction: 0.6,
  restitution: 0.2,
  rollingResistance: 0.012, // a torque, as a length: how far the load sits off-centre
  slop: 0.002,         // overlap allowed at rest, so resting contacts stay in touch
  maxPushSpeed: 6,     // deep overlaps separate at this speed, not explosively
  maxSpeed: 40,
  sleepSpeed: 0.12,
  sleepAfter: 0.6,
  stepsPerSecond: 60,
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
const analyseShape = (shape, sampleCount = 32) => {
  if (analysisCache.has(shape)) return analysisCache.get(shape);
  const d = shape.d;
  const random = makeRandom(12345);

  // How far does it reach? March inwards from far away in many directions.
  let reach = 0;
  const far = 16;
  for (let i = 0; i < 128; i++) {
    const y = 1 - (2 * (i + 0.5)) / 128;
    const r = Math.sqrt(1 - y * y);
    const a = i * 2.39996323;
    const dir = [Math.cos(a) * r, y, Math.sin(a) * r];
    for (let t = far, n = 0; t > 0 && n < 96; n++) {
      const dist = d(dir[0] * t, dir[1] * t, dir[2] * t);
      if (dist < 0.0005) { reach = Math.max(reach, t); break; }
      t -= dist;
    }
  }
  const box = reach * 1.05;

  // Surface candidates: pull random points onto d = 0.
  const candidates = [];
  for (let n = 0; n < 1400; n++) {
    let x = (random() * 2 - 1) * box;
    let y = (random() * 2 - 1) * box;
    let z = (random() * 2 - 1) * box;
    let dist = 1;
    for (let it = 0; it < 12 && Math.abs(dist) > 2e-4; it++) {
      dist = d(x, y, z);
      const g = gradientOf(d, x, y, z);
      x -= g[0] * dist;
      y -= g[1] * dist;
      z -= g[2] * dist;
    }
    if (Math.abs(d(x, y, z)) < 1e-3) candidates.push([x, y, z]);
  }

  // Mass properties: add up small cells over the shape's tight bounds. A cell cut
  // by the surface counts in part, which the distance at its centre tells us.
  const lo = [0, 1, 2].map((i) => Math.min(...candidates.map((p) => p[i])));
  const hi = [0, 1, 2].map((i) => Math.max(...candidates.map((p) => p[i])));
  const cell = Math.max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]) / 48;
  const counts = [0, 1, 2].map((i) => Math.ceil((hi[i] - lo[i]) / cell) + 4);
  const com = [0, 0, 0];
  const inside = [];
  let volume = 0;
  for (let i = 0; i < counts[0]; i++) {
    for (let j = 0; j < counts[1]; j++) {
      for (let k = 0; k < counts[2]; k++) {
        const x = lo[0] + (i - 1.5) * cell;
        const y = lo[1] + (j - 1.5) * cell;
        const z = lo[2] + (k - 1.5) * cell;
        const fill = Math.max(0, Math.min(1, 0.5 - d(x, y, z) / cell));
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
  const unit = cell * cell * cell;
  const inertia = [0, 0, 0, 0, 0, 0, 0, 0, 0];
  const selfTerm = (cell * cell) / 6; // each cell's own inertia, about its centre
  for (const [px, py, pz, fill] of inside) {
    const x = px - com[0];
    const y = py - com[1];
    const z = pz - com[2];
    inertia[0] += (y * y + z * z + selfTerm) * fill;
    inertia[4] += (x * x + z * z + selfTerm) * fill;
    inertia[8] += (x * x + y * y + selfTerm) * fill;
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
  let spacing = reach;
  nearest.fill(Infinity);
  for (let n = 0; n < sampleCount && candidates.length; n++) {
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
    next = bestIndex;
  }
  const points = chosen.map((p) => [p[0] - com[0], p[1] - com[1], p[2] - com[2]]);
  const radius = Math.max(...candidates.map((p) => Math.hypot(p[0] - com[0], p[1] - com[1], p[2] - com[2])));
  const renderRadius = Math.max(...candidates.map((p) => Math.hypot(p[0], p[1], p[2])));

  const result = {
    d, volume: volume * unit, com, inertia, invInertia: invertMat3(inertia), points, spacing, radius, renderRadius,
  };
  analysisCache.set(shape, result);
  return result;
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

  // Movement of a point at r so far during this substep's solve.
  pushed(r) {
    const t = cross3(this.dth, r);
    return [this.dx[0] + t[0], this.dx[1] + t[1], this.dx[2] + t[2]];
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
    this.bodies.push(body);
    return body;
  }

  reset() {
    for (const b of this.bodies) b.reset();
    this.grab = null;
    this.manifolds = [];
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
    this.grab = { body, local: mulMat3T(body.R, r), target: [...point] };
  }

  moveGrab(target) {
    if (!this.grab) return;
    this.grab.target = target;
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
    for (let i = 0; i < this.substeps; i++) {
      const fast = this.bodies.some((b) => !b.sleeping && b.v[0] * b.v[0] + b.v[1] * b.v[1] + b.v[2] * b.v[2] > PHYSICS.fastSpeed ** 2);
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
      contacts = this.manifolds = this.refreshContacts(this.manifolds);
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
    for (const m of contacts) this.solvePatchFriction(m);
    for (const m of contacts) for (const c of m.contacts) this.solveNormal(c, h, 1);
    for (const b of bodies) {
      if (b.sleeping) continue;
      b.x = [b.x[0] + b.dx[0], b.x[1] + b.dx[1], b.x[2] + b.dx[2]];
      b.q = rotateQuat(b.q, b.dth);
      b.R = matFromQuat(b.q);

      // Velocity is whatever motion the positions ended up describing.
      b.v = [(b.x[0] - b.xPrev[0]) / h, (b.x[1] - b.xPrev[1]) / h, (b.x[2] - b.xPrev[2]) / h];
      const [px, py, pz, pw] = b.qPrev;
      const [x, y, z, w] = b.q;
      const rw = w * pw + x * px + y * py + z * pz;
      const sign = (rw < 0 ? -2 : 2) / h;
      b.w = [
        sign * (x * pw - w * px - y * pz + z * py),
        sign * (y * pw - w * py - z * px + x * pz),
        sign * (z * pw - w * pz - x * py + y * px),
      ];
    }
    for (const m of contacts) this.solveVelocity(m, h);
  }

  findContacts(h) {
    const { bodies } = this;
    const contacts = [];
    const horizon = h * PHYSICS.searchEvery;
    for (const b of bodies) {
      if (b.sleeping) continue;
      // The world's distance at the centre tells us if the whole body is clear.
      if (this.worldDist(b.x[0], b.x[1], b.x[2]) < b.info.radius + PHYSICS.margin + Math.hypot(...b.v) * horizon) {
        this.collide(b, null, this.worldDist, contacts, horizon);
      }
    }
    for (let i = 0; i < bodies.length; i++) {
      for (let j = i + 1; j < bodies.length; j++) {
        const a = bodies[i];
        const b = bodies[j];
        if (a.sleeping && b.sleeping) continue;
        const reach = a.info.radius + b.info.radius + PHYSICS.margin;
        const dx = a.x[0] - b.x[0];
        const dy = a.x[1] - b.x[1];
        const dz = a.x[2] - b.x[2];
        if (dx * dx + dy * dy + dz * dz > reach * reach) continue;
        // Each body's samples are tested against the other's distance function.
        this.collide(a, b, b.dist, contacts, horizon);
        this.collide(b, a, a.dist, contacts, horizon);
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
    const moving = !B && !!A.xSearch && Math.hypot(...A.v) * h > 0.01;
    for (const p of points) {
      const r = mulMat3(R, p);
      let x = A.x[0] + r[0];
      let y = A.x[1] + r[1];
      let z = A.x[2] + r[2];
      let d = field(x, y, z);

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
    if (!kept.length) return;
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

  solveGrab(h) {
    if (!this.grab) return;
    const { body, local, target } = this.grab;
    if (body.sleeping) return;
    const r = mulMat3(body.R, local);
    const e = [target[0] - body.x[0] - r[0], target[1] - body.x[1] - r[1], target[2] - body.x[2] - r[2]];
    const len = Math.hypot(...e);
    if (len < 1e-5) return;
    const n = [e[0] / len, e[1] / len, e[2] / len];
    // A soft spring, limited so a fast pointer cannot fling a body through walls.
    const move = Math.min(len * 0.12, 18 * h);
    const lambda = move / body.resistance(r, n);
    body.applyPush(r, [n[0] * lambda, n[1] * lambda, n[2] * lambda], 1);
    body.v = body.v.map((c) => c * 0.985);
    body.w = body.w.map((c) => c * 0.97);
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
    const allowed = c.swept ? c.depth : Math.min(Math.max(c.depth - PHYSICS.slop, 0), PHYSICS.maxPushSpeed * h);
    const remaining = allowed - dot3(moved, n);
    if (remaining <= 0) return;
    const wn = A.resistance(rA, n) + (B ? B.resistance(rB, n) : 0);
    if (wn <= 0) return;
    const lambda = (remaining / wn) * share;
    c.lambda += lambda;
    const push = [n[0] * lambda, n[1] * lambda, n[2] * lambda];
    A.applyPush(rA, push, 1);
    if (B) B.applyPush(rB, push, -1);
  }

  // All contacts between two bodies act as one patch. Friction applied at the
  // patch's load-weighted centre cannot twist a block the way friction at one
  // off-centre point would, and the patch's size gives real torsional friction.
  summarisePatch(m) {
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
    for (const c of contacts) spread += c.lambda * Math.hypot(c.rA[0] - rA[0], c.rA[1] - rA[1], c.rA[2] - rA[2]);
    const friction = B ? Math.sqrt(A.friction * B.friction) : A.friction;
    return { A, B, total, rA, rB, n, spread: Math.max(spread / total, 0.01), friction };
  }

  // Static friction: undo sideways slip, up to what the normal load allows.
  solvePatchFriction(m) {
    const patch = this.summarisePatch(m);
    if (!patch) return;
    const { A, B, rA, rB, n, total, friction } = patch;
    const prevA = this.previousOffset(A, rA);
    const a = A.pushed(rA);
    const slip = [a[0] + rA[0] - prevA[0], a[1] + rA[1] - prevA[1], a[2] + rA[2] - prevA[2]];
    if (B) {
      const prevB = this.previousOffset(B, rB);
      const b = B.pushed(rB);
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
    const patch = this.summarisePatch(m);
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
    // A moving body wakes anything it could be touching.
    for (const a of bodies) {
      if (a.sleeping) continue;
      if (Math.hypot(...a.v) + Math.hypot(...a.w) * a.info.radius < 0.3) continue;
      for (const b of bodies) {
        if (!b.sleeping || a === b) continue;
        const reach = a.info.radius + b.info.radius + 0.05;
        if (Math.hypot(a.x[0] - b.x[0], a.x[1] - b.x[1], a.x[2] - b.x[2]) < reach) this.wake(b);
      }
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
