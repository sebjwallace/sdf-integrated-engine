// sdf.js: a tiny signed-distance-function modelling language.
//
// Shapes are plain JS objects that know how to write themselves out as GLSL.
// Everything chains and composes:
//
//   union(
//     box(2, 1, 1).round(0.1).paint('#c33'),
//     sphere(0.5).move(0, 1, 0),
//   ).smooth(0.2).mirror('x')
//
// compile(shape) produces `float treeMap(vec3)` for raymarching and
// `Hit treeHit(vec3)`, which also carries the material, for shading.
//
// shape.named('Wheels') marks an object: something the outliner lists, which
// can be selected and hidden. Every surface knows which object it belongs to,
// and parts given the same name are one object. A hidden object's distance is
// pushed far away by a uniform, so hiding needs no recompiling.
//
// Each object can also be moved, turned and scaled after compiling, by the
// uObjM, uObjO and uObjS uniforms (see Viewer.setObjectTransform). The change is
// made in the space the object is named in, so name objects outside any
// mirror(), polar() or grid(), or a move would happen in every copy at once.
// shape.d sees the same change, through the object's `holder`.
//
// shape.pose('elbow') marks a joint: a rotation and position supplied from
// JavaScript every frame, so a scene can animate without recompiling.
//
// Every shape also carries shape.d(x, y, z): the same distance function as plain
// JavaScript, which is what the physics engine queries on the CPU.

const SDF = (() => {
  const f = (n) => (+n).toFixed(5);
  const vec = (a) => `vec${a.length}(${a.map(f).join(',')})`;

  // ---- materials ---------------------------------------------------------

  const hexToLinear = (hex) => {
    const n = parseInt(hex.replace('#', '').replace(/^(.)(.)(.)$/, '$1$1$2$2$3$3'), 16);
    return [n >> 16, (n >> 8) & 255, n & 255].map((c) => Math.pow(c / 255, 2.2));
  };

  // rough: 0 is a mirror, 1 is chalk. metal: 0 to 1. glow: emissive strength.
  const material = (color, rough = 0.5, metal = 0, glow = 0) => ({
    color: hexToLinear(color), rough, metal, glow,
  });
  const toMaterial = (m, ...rest) => (typeof m === 'string' ? material(m, ...rest) : m);

  const DEFAULT_MATERIAL = material('#b8b8b8', 0.5);

  // Rows of R = Rz*Ry*Rx, from degrees. Passed as GLSL columns they form R's
  // inverse, which is what carries a point into a shape's local space.
  const rotationRows = (x, y, z) => {
    const r = (d) => (d * Math.PI) / 180;
    const [cx, sx, cy, sy, cz, sz] = [r(x), r(y), r(z)].flatMap((a) => [Math.cos(a), Math.sin(a)]);
    return [
      cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx,
      sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx,
      -sy, cy * sx, cy * cx,
    ];
  };

  // ---- distance helpers for the CPU side ------------------------------------

  const smin = (a, b, k) => {
    const h = Math.max(k - Math.abs(a - b), 0) / k;
    return Math.min(a, b) - h * h * k * 0.25;
  };
  const smax = (a, b, k) => (k > 0 ? -smin(-a, -b, k) : Math.max(a, b));
  const extrudeDist = (d, z, h) => {
    const wy = Math.abs(z) - h;
    return Math.min(Math.max(d, wy), 0) + Math.hypot(Math.max(d, 0), Math.max(wy, 0));
  };

  // ---- code generation ---------------------------------------------------

  // One Ctx writes one GLSL function body. `full` selects Hit values with
  // materials rather than bare floats; every shape below works in both modes.
  class Ctx {
    constructor(full, funcs, joints, objects, frames, holders) {
      this.full = full;
      this.funcs = funcs;
      this.joints = joints;
      this.objects = objects;
      this.frames = frames;
      this.holders = holders;
      this.lines = [];
      this.n = 0;
      this.cache = new Map();
      this.materials = [];
      this.tagged = new Map();
      // The space operations between the root and the shape being written, so
      // an object can say which frame its own transform is applied in.
      this.path = [];
    }
    // Writes a child inside a change of space, remembering the change.
    inside(step, fn) {
      this.path.push(step);
      try { return fn(); } finally { this.path.pop(); }
    }
    object(name) {
      if (!this.objects.includes(name)) this.objects.push(name);
      return this.objects.indexOf(name);
    }
    // The material m, belonging to object id. One copy per pair, so memo() can
    // still tell when a shape is reused with the same material.
    tag(m, id) {
      if (id === undefined || m.id === id) return m;
      if (!this.tagged.has(m)) this.tagged.set(m, new Map());
      const byId = this.tagged.get(m);
      if (!byId.has(id)) byId.set(id, { ...m, id });
      return byId.get(id);
    }
    // Pushes a hidden object's distance out of reach.
    hide(v, id) {
      return this.remap(v, (d) => `(uHidden[${id}]>.5?1e5:${d})`);
    }
    // A shape used twice at the same point is only computed once.
    memo(shape, p, m, emit) {
      if (!this.materials.includes(m)) this.materials.push(m);
      const key = `${p}/${this.full ? this.materials.indexOf(m) : ''}`;
      if (!this.cache.has(shape)) this.cache.set(shape, new Map());
      const seen = this.cache.get(shape);
      if (!seen.has(key)) seen.set(key, emit(this, p, m));
      return seen.get(key);
    }
    // Remembers the frame an object is named in, and who holds its transform.
    named(id, holder) {
      if (!this.frames[id]) this.frames[id] = [...this.path];
      if (!this.holders[id]) this.holders[id] = [];
      if (!this.holders[id].includes(holder)) this.holders[id].push(holder);
      this.namedCount = Math.max(this.namedCount || 0, id + 1);
    }
    joint(name) {
      if (!this.joints.includes(name)) this.joints.push(name);
      return this.joints.indexOf(name);
    }
    let(type, expr) {
      const name = `v${this.n++}`;
      this.lines.push(`${type} ${name}=${expr};`);
      return name;
    }
    leaf(expr, m) {
      const d = this.let('float', expr);
      if (!this.full) return d;
      return this.let('Hit', `Hit(${d},${vec(m.color)},${f(m.rough)},${f(m.metal)},${f(m.glow)},${f(m.id ?? -1)})`);
    }
    // Apply a distance-only change, such as rounding, to a result.
    remap(v, fn) {
      if (!this.full) return this.let('float', fn(v));
      const h = this.let('Hit', v);
      this.lines.push(`${h}.d=${fn(`${h}.d`)};`);
      return h;
    }
  }

  class Shape {
    constructor(emit, d) {
      this.emit = (c, p, m) => c.memo(this, p, m, emit);
      this.d = d;
    }

    // --- transforms (these move space in the opposite direction) ---
    move(x = 0, y = 0, z = 0) {
      const { d } = this;
      return new Shape(
        (c, p, m) => c.inside({ move: [x, y, z] }, () => this.emit(c, c.let('vec3', `${p}-${vec([x, y, z])}`), m)),
        (px, py, pz) => d(px - x, py - y, pz - z),
      );
    }
    // Degrees, applied in x, then y, then z order.
    rotate(x = 0, y = 0, z = 0) {
      const rows = rotationRows(x, y, z);
      const { d } = this;
      const [a, b, e, g, h, i, j, k, l] = rows;
      return new Shape(
        (c, p, m) => c.inside({ rotate: rows }, () => this.emit(c, c.let('vec3', `mat3(${rows.map(f).join(',')})*${p}`), m)),
        // GLSL's constructor takes columns, so this multiplies by R's transpose.
        (px, py, pz) => d(a * px + g * py + j * pz, b * px + h * py + k * pz, e * px + i * py + l * pz),
      );
    }
    // An animated rotate-then-move. The scene's animate() sets it by name. Its
    // value lives on the GPU side, so shape.d ignores it and sees the rest pose.
    pose(name) {
      return new Shape((c, p, m) => {
        const i = c.joint(name);
        return c.inside({ joint: i }, () => this.emit(c, c.let('vec3', `uJointR[${i}]*(${p}-uJointT[${i}])`), m));
      }, this.d);
    }
    scale(s) {
      const { d } = this;
      return new Shape(
        (c, p, m) => c.inside({ scale: s }, () => c.remap(this.emit(c, c.let('vec3', `${p}/${f(s)}`), m), (v) => `${v}*${f(s)}`)),
        (x, y, z) => d(x / s, y / s, z / s) * s,
      );
    }

    // --- surface operations ---
    round(r) {
      const { d } = this;
      return new Shape((c, p, m) => c.remap(this.emit(c, p, m), (v) => `${v}-${f(r)}`), (x, y, z) => d(x, y, z) - r);
    }
    shell(t) {
      const { d } = this;
      return new Shape((c, p, m) => c.remap(this.emit(c, p, m), (v) => `abs(${v})-${f(t)}`), (x, y, z) => Math.abs(d(x, y, z)) - t);
    }

    // --- space folding: one shape's cost, many copies on screen ---
    // Model the positive side only, and it appears on both. e.g. mirror('xz')
    mirror(axes) {
      const { d } = this;
      const [mx, my, mz] = ['x', 'y', 'z'].map((a) => axes.includes(a));
      return new Shape((c, p, m) => {
        const q = c.let('vec3', p);
        for (const a of axes) c.lines.push(`${q}.${a}=abs(${q}.${a});`);
        return c.inside({ fold: 'mirror' }, () => this.emit(c, q, m));
      }, (x, y, z) => d(mx ? Math.abs(x) : x, my ? Math.abs(y) : y, mz ? Math.abs(z) : z));
    }
    // n copies around an axis. Model the shape on the first listed axis:
    // +x for 'y' and 'z', +y for 'x'.
    polar(n, axis = 'y') {
      const plane = { x: 'yz', y: 'xz', z: 'xy' }[axis];
      const { d } = this;
      const sector = (2 * Math.PI) / n;
      const fold = (u, v) => {
        const raw = Math.atan2(v, u) + sector / 2;
        const a = raw - sector * Math.floor(raw / sector) - sector / 2;
        const r = Math.hypot(u, v);
        return [r * Math.cos(a), r * Math.sin(a)];
      };
      const js = {
        x: (x, y, z) => { const [u, v] = fold(y, z); return d(x, u, v); },
        y: (x, y, z) => { const [u, v] = fold(x, z); return d(u, y, v); },
        z: (x, y, z) => { const [u, v] = fold(x, y); return d(u, v, z); },
      }[axis];
      return new Shape((c, p, m) => {
        const q = c.let('vec3', p);
        c.lines.push(`${q}.${plane}=polarFold(${q}.${plane},${f(n)});`);
        return c.inside({ fold: 'polar' }, () => this.emit(c, q, m));
      }, js);
    }
    // Finite grid: spacing [x,y,z], and copies either side of centre [x,y,z].
    grid(spacing, count) {
      const s = vec(spacing.map((v) => v || 1));
      const { d } = this;
      const [sx, sy, sz] = spacing.map((v) => v || 1);
      const wrap = (v, size, n) => v - size * Math.max(-n, Math.min(n, Math.round(v / size)));
      return new Shape(
        (c, p, m) => c.inside({ fold: 'grid' }, () => this.emit(c, c.let('vec3', `${p}-${s}*clamp(round(${p}/${s}),-${vec(count)},${vec(count)})`), m)),
        (x, y, z) => d(wrap(x, sx, count[0]), wrap(y, sy, count[1]), wrap(z, sz, count[2])),
      );
    }
    // Twist around the y axis, in radians per unit of height.
    twist(k) {
      return new Shape((c, p, m) => {
        const a = c.let('float', `${p}.y*${f(k)}`);
        const q = c.let('vec3', `vec3(cos(${a})*${p}.x-sin(${a})*${p}.z,${p}.y,sin(${a})*${p}.x+cos(${a})*${p}.z)`);
        // Twisting stretches space, so shrink the distance to stay safe.
        return c.inside({ fold: 'twist' }, () => c.remap(this.emit(c, q, m), (v) => `${v}/sqrt(1.+${f(k * k)}*dot(${p}.xz,${p}.xz))`));
      }, (x, y, z) => {
        const a = y * k;
        const co = Math.cos(a);
        const si = Math.sin(a);
        return this.d(co * x - si * z, y, si * x + co * z) / Math.sqrt(1 + k * k * (x * x + z * z));
      });
    }

    // Inner paints win over outer ones, so a group can set a default colour.
    // Painting keeps the object the surface belongs to.
    paint(mat, ...rest) {
      const own = toMaterial(mat, ...rest);
      return new Shape((c, p, m) => this.emit(c, p, c.tag(own, m.id)), this.d);
    }

    // Stretches by [sx, sy, sz] about the point `about`. Uneven stretching
    // makes the distance an underestimate, which is safe: the surface is exact.
    stretch([sx, sy, sz], about = [0, 0, 0]) {
      const { d } = this;
      const [ax, ay, az] = about;
      const k = Math.min(sx, sy, sz);
      return new Shape(
        (c, p, m) => c.inside({ fold: 'stretch' }, () => c.remap(
          this.emit(c, c.let('vec3', `${vec(about)}+(${p}-${vec(about)})/${vec([sx, sy, sz])}`), m),
          (v) => `${v}*${f(k)}`,
        )),
        (x, y, z) => d(ax + (x - ax) / sx, ay + (y - ay) / sy, az + (z - az) / sz) * k,
      );
    }

    // An object for the outliner. Inner names win over outer ones, as paint does.
    // Its transform maps a point to local = M * point + O, then scales the
    // distance by s. The holder carries the same numbers for shape.d.
    named(name) {
      const { d } = this;
      const holder = { M: null, O: [0, 0, 0], s: 1 };
      const shape = new Shape((c, p, m) => {
        const id = c.object(name);
        c.named(id, holder);
        const q = c.let('vec3', `uObjM[${id}]*${p}+uObjO[${id}]`);
        return c.hide(c.remap(this.emit(c, q, c.tag(m, id)), (v) => `${v}*uObjS[${id}]`), id);
      }, (x, y, z) => {
        const { M, O, s } = holder;
        if (!M) return d(x, y, z);
        return d(
          M[0] * x + M[1] * y + M[2] * z + O[0],
          M[3] * x + M[4] * y + M[5] * z + O[1],
          M[6] * x + M[7] * y + M[8] * z + O[2],
        ) * s;
      });
      shape.holder = holder;
      return shape;
    }
  }

  // ---- primitives (all centred on the origin; sizes are full sizes) --------

  // Each primitive is written twice: once for the GPU, once for the CPU.
  const prim = (expr, d) => new Shape((c, p, m) => c.leaf(expr(p), m), d);

  const sphere = (r) => prim((p) => `length(${p})-${f(r)}`, (x, y, z) => Math.hypot(x, y, z) - r);
  const box = (x, y = x, z = x) => {
    const [bx, by, bz] = [x / 2, y / 2, z / 2];
    return prim((p) => `sdBox(${p},${vec([bx, by, bz])})`, (px, py, pz) => {
      const qx = Math.abs(px) - bx;
      const qy = Math.abs(py) - by;
      const qz = Math.abs(pz) - bz;
      return Math.hypot(Math.max(qx, 0), Math.max(qy, 0), Math.max(qz, 0)) + Math.min(Math.max(qx, qy, qz), 0);
    });
  };
  const ellipsoid = (x, y, z) => prim((p) => `sdEllipsoid(${p},${vec([x, y, z])})`, (px, py, pz) => {
    const a = Math.hypot(px / x, py / y, pz / z);
    const b = Math.hypot(px / (x * x), py / (y * y), pz / (z * z));
    return (a * (a - 1)) / Math.max(b, 1e-5);
  });
  const cylinder = (r, h) => prim((p) => `sdCylinder(${p},${f(r)},${f(h / 2)})`, (x, y, z) => {
    const dx = Math.hypot(x, z) - r;
    const dy = Math.abs(y) - h / 2;
    return Math.min(Math.max(dx, dy), 0) + Math.hypot(Math.max(dx, 0), Math.max(dy, 0));
  });
  const cone = (r1, r2, h) => prim((p) => `sdCone(${p},${f(r1)},${f(r2)},${f(h / 2)})`, (x, y, z) => {
    const half = h / 2;
    const qx = Math.hypot(x, z);
    const qy = y;
    const k2x = r2 - r1;
    const k2y = 2 * half;
    const caX = qx - Math.min(qx, qy < 0 ? r1 : r2);
    const caY = Math.abs(qy) - half;
    const t = Math.max(0, Math.min(1, ((r2 - qx) * k2x + (half - qy) * k2y) / (k2x * k2x + k2y * k2y)));
    const cbX = qx - r2 + k2x * t;
    const cbY = qy - half + k2y * t;
    const s = cbX < 0 && caY < 0 ? -1 : 1;
    return s * Math.sqrt(Math.min(caX * caX + caY * caY, cbX * cbX + cbY * cbY));
  });
  const torus = (R, r) => prim(
    (p) => `length(vec2(length(${p}.xz)-${f(R)},${p}.y))-${f(r)}`,
    (x, y, z) => Math.hypot(Math.hypot(x, z) - R, y) - r,
  );
  const capsule = (a, b, r) => {
    const ba = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    const baba = ba[0] * ba[0] + ba[1] * ba[1] + ba[2] * ba[2];
    return prim((p) => `sdCapsule(${p},${vec(a)},${vec(b)})-${f(r)}`, (x, y, z) => {
      const px = x - a[0];
      const py = y - a[1];
      const pz = z - a[2];
      const t = Math.max(0, Math.min(1, (px * ba[0] + py * ba[1] + pz * ba[2]) / baba));
      return Math.hypot(px - ba[0] * t, py - ba[1] * t, pz - ba[2] * t) - r;
    });
  };

  // Corner cutting turns a handful of control points into a smooth curve.
  const chaikin = (pts, iterations) => {
    for (let k = 0; k < iterations; k++) {
      pts = pts.flatMap((a, i) => {
        const b = pts[(i + 1) % pts.length];
        return [
          [a[0] * 0.75 + b[0] * 0.25, a[1] * 0.75 + b[1] * 0.25],
          [a[0] * 0.25 + b[0] * 0.75, a[1] * 0.25 + b[1] * 0.75],
        ];
      });
    }
    return pts;
  };

  let polygonCount = 0;
  const polygonFunc = (points, smooth) => {
    const pts = chaikin(points, smooth);
    const name = `sdPoly${polygonCount++}`;
    const xs = pts.map((q) => q[0]);
    const ys = pts.map((q) => q[1]);
    const lo = [Math.min(...xs), Math.min(...ys)];
    const hi = [Math.max(...xs), Math.max(...ys)];
    // Far from the bounding box, its distance is a good enough answer, and it
    // skips the loop over every segment.
    const code = `float ${name}(vec2 p){
      float far=length(max(abs(p-${vec([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2])})-${vec([(hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2])},0.));
      if(far>.45)return far;
      const int N=${pts.length};
      const vec2 P[N]=vec2[N](${pts.map(vec).join(',')});
      float d=dot(p-P[0],p-P[0]),s=1.;
      for(int i=0,j=N-1;i<N;j=i,i++){
        vec2 e=P[j]-P[i],w=p-P[i],b=w-e*clamp(dot(w,e)/dot(e,e),0.,1.);
        d=min(d,dot(b,b));
        bvec3 c=bvec3(p.y>=P[i].y,p.y<P[j].y,e.x*w.y>e.y*w.x);
        if(all(c)||all(not(c)))s=-s;
      }
      return s*sqrt(d);
    }`;
    // The same calculation for the CPU.
    const [cx, cy] = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2];
    const [hx, hy] = [(hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2];
    const d = (u, v) => {
      const far = Math.hypot(Math.max(Math.abs(u - cx) - hx, 0), Math.max(Math.abs(v - cy) - hy, 0));
      if (far > 0.45) return far;
      let best = (u - pts[0][0]) ** 2 + (v - pts[0][1]) ** 2;
      let sign = 1;
      for (let i = 0, j = pts.length - 1; i < pts.length; j = i, i++) {
        const [ax, ay] = pts[i];
        const ex = pts[j][0] - ax;
        const ey = pts[j][1] - ay;
        const wx = u - ax;
        const wy = v - ay;
        const t = Math.max(0, Math.min(1, (wx * ex + wy * ey) / (ex * ex + ey * ey)));
        const bx = wx - ex * t;
        const by = wy - ey * t;
        best = Math.min(best, bx * bx + by * by);
        const c1 = v >= ay;
        const c2 = v < pts[j][1];
        const c3 = ex * wy > ey * wx;
        if ((c1 && c2 && c3) || (!c1 && !c2 && !c3)) sign = -sign;
      }
      return sign * Math.sqrt(best);
    };
    return { name, code, d };
  };
  const usePolygon = (ctx, fn) => { if (!ctx.funcs.includes(fn.code)) ctx.funcs.push(fn.code); return fn.name; };

  // Draw an outline in the XY plane and push it `depth` along Z.
  const extrude = (points, depth, { smooth = 0 } = {}) => {
    const fn = polygonFunc(points, smooth);
    return new Shape(
      (c, p, m) => c.leaf(`sdExtrude(${usePolygon(c, fn)}(${p}.xy),${p}.z,${f(depth / 2)})`, m),
      (x, y, z) => extrudeDist(fn.d(x, y), z, depth / 2),
    );
  };
  // Draw a half-profile as [radius, height] and spin it around Y.
  const lathe = (points, { smooth = 0 } = {}) => {
    const fn = polygonFunc(points, smooth);
    return new Shape(
      (c, p, m) => c.leaf(`${usePolygon(c, fn)}(vec2(length(${p}.xz),${p}.y))`, m),
      (x, y, z) => fn.d(Math.hypot(x, z), y),
    );
  };

  // ---- booleans ------------------------------------------------------------

  class Boolean3D extends Shape {
    constructor(kind, shapes, k = 0) {
      super((c, p, m) => {
        const ops = {
          union: [k ? (a, b) => `smin(${a},${b},${f(k)})` : (a, b) => `min(${a},${b})`,
                  k ? (a, b) => `hBlend(${a},${b},${f(k)})` : (a, b) => `hUnion(${a},${b})`],
          subtract: [(a, b) => `smax(${a},-${b},${f(k)})`, (a, b) => `hSubtract(${a},${b},${f(k)})`],
          intersect: [(a, b) => `smax(${a},${b},${f(k)})`, (a, b) => `hIntersect(${a},${b},${f(k)})`],
        };
        const op = ops[kind][c.full ? 1 : 0];
        return shapes
          .map((s) => s.emit(c, p, m))
          .reduce((a, b) => c.let(c.full ? 'Hit' : 'float', op(a, b)));
      }, Boolean3D.distance(kind, shapes, k));
      Object.assign(this, { kind, shapes });
    }
    static distance(kind, shapes, k) {
      const fns = shapes.map((s) => s.d);
      const combine = {
        union: k ? (a, b) => smin(a, b, k) : Math.min,
        subtract: (a, b) => smax(a, -b, k),
        intersect: (a, b) => smax(a, b, k),
      }[kind];
      return (x, y, z) => {
        let d = fns[0](x, y, z);
        for (let i = 1; i < fns.length; i++) d = combine(d, fns[i](x, y, z));
        return d;
      };
    }
    // Melt the joins together: union(a, b).smooth(0.1)
    smooth(k) { return new Boolean3D(this.kind, this.shapes, k); }
  }

  const union = (...s) => new Boolean3D('union', s);
  const subtract = (...s) => new Boolean3D('subtract', s); // first shape minus the rest
  const intersect = (...s) => new Boolean3D('intersect', s);

  // A fixed world plus bodies that physics moves. Rays skip a body whose bounding
  // sphere is further away than the nearest surface found so far. That can never
  // change the answer. Even so, checking every body at every ray step is slow, so
  // one lookup in a small texture of lower bounds usually skips all of them.
  // Each body is an object of its own; a hidden one is skipped entirely.
  const withBodies = (world, bodies) => new Shape((c, p, m) => {
    // Edits change the world, not the bodies, so a pit cannot erase what falls in.
    const first = world.emit(c, p, m);
    const out = c.let(c.full ? 'Hit' : 'float', c.full ? `applyEditsHit(${first},${p})` : `applyEdits(${first},${p})`);
    const nearest = c.full ? `${out}.d` : out;
    c.lines.push(`if(uHideBodies<.5&&bodyLowerBound(${p})<${nearest}){`);
    // Each body can be stretched about its centre of mass: xyz holds 1 / the
    // stretch, w the distance scale. Its bounding radius changes with it.
    const decl = `uniform vec4 uBodyScale[${bodies.length}];\nuniform float uBodyRadius[${bodies.length}];`;
    if (!c.funcs.includes(decl)) c.funcs.unshift(decl);
    bodies.forEach((body, i) => {
      const j = c.joint(`body${i}`);
      const id = c.object(body.name || `Body ${i + 1}`);
      c.lines.push(`if(uHidden[${id}]<.5&&length(${p}-uJointT[${j}])-uBodyRadius[${i}]<${nearest}){`);
      const q = c.let('vec3', `uJointR[${j}]*(${p}-uJointT[${j}])`);
      const com = vec(body.com || [0, 0, 0]);
      const qs = c.let('vec3', `${com}+(${q}-${com})*uBodyScale[${i}].xyz`);
      const v = c.remap(body.shape.emit(c, qs, c.tag(m, id)), (d) => `${d}*uBodyScale[${i}].w`);
      c.lines.push(c.full ? `${out}=hUnion(${out},${v});}` : `${out}=min(${out},${v});}`);
    });
    c.lines.push('}');
    return out;
  }, world.d);

  // ---- compiler --------------------------------------------------------------

  const compile = (shape) => {
    const funcs = [];
    const joints = [];
    const objects = [];
    const frames = [];
    const holders = [];
    let namedCount = 0;
    const body = (full) => {
      const ctx = new Ctx(full, funcs, joints, objects, frames, holders);
      const result = shape.emit(ctx, 'p', DEFAULT_MATERIAL);
      namedCount = Math.max(namedCount, ctx.namedCount || 0);
      return `${ctx.lines.join('\n  ')}\n  return ${result};`;
    };
    const dist = body(false);
    const full = body(true);
    const n = joints.length;
    let uniforms = n ? `uniform mat3 uJointR[${n}];\nuniform vec3 uJointT[${n}];\n` : '';
    if (objects.length) uniforms += `uniform float uHidden[${objects.length}];\n`;
    if (namedCount) uniforms += `uniform mat3 uObjM[${namedCount}];\nuniform vec3 uObjO[${namedCount}];\nuniform float uObjS[${namedCount}];\n`;
    return {
      joints,
      objects,
      // For each named object: the space operations above it, outermost first,
      // and the holders its CPU distance reads. Bodies have neither.
      frames,
      holders,
      namedCount,
      code: `${uniforms}${funcs.join('\n')}
float treeMap(vec3 p){\n  ${dist}\n}
Hit treeHit(vec3 p){\n  ${full}\n}`,
    };
  };

  return {
    material, sphere, box, ellipsoid, cylinder, cone, torus, capsule, extrude, lathe,
    union, subtract, intersect, withBodies, compile, rotationRows, smin, smax, scenes: {},
  };
})();
