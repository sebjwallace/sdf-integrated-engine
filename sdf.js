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
// shape.pose('elbow') marks a joint: a rotation and position supplied from
// JavaScript every frame, so a scene can animate without recompiling.

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

  // ---- code generation ---------------------------------------------------

  // One Ctx writes one GLSL function body. `full` selects Hit values with
  // materials rather than bare floats; every shape below works in both modes.
  class Ctx {
    constructor(full, funcs, joints) {
      this.full = full;
      this.funcs = funcs;
      this.joints = joints;
      this.lines = [];
      this.n = 0;
      this.cache = new Map();
      this.materials = [];
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
      return this.let('Hit', `Hit(${d},${vec(m.color)},${f(m.rough)},${f(m.metal)},${f(m.glow)})`);
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
    constructor(emit) { this.emit = (c, p, m) => c.memo(this, p, m, emit); }

    // --- transforms (these move space in the opposite direction) ---
    move(x = 0, y = 0, z = 0) {
      return new Shape((c, p, m) => this.emit(c, c.let('vec3', `${p}-${vec([x, y, z])}`), m));
    }
    // Degrees, applied in x, then y, then z order.
    rotate(x = 0, y = 0, z = 0) {
      const rows = rotationRows(x, y, z);
      return new Shape((c, p, m) => this.emit(c, c.let('vec3', `mat3(${rows.map(f).join(',')})*${p}`), m));
    }
    // An animated rotate-then-move. The scene's animate() sets it by name.
    pose(name) {
      return new Shape((c, p, m) => {
        const i = c.joint(name);
        return this.emit(c, c.let('vec3', `uJointR[${i}]*(${p}-uJointT[${i}])`), m);
      });
    }
    scale(s) {
      return new Shape((c, p, m) => c.remap(this.emit(c, c.let('vec3', `${p}/${f(s)}`), m), (d) => `${d}*${f(s)}`));
    }

    // --- surface operations ---
    round(r) { return new Shape((c, p, m) => c.remap(this.emit(c, p, m), (d) => `${d}-${f(r)}`)); }
    shell(t) { return new Shape((c, p, m) => c.remap(this.emit(c, p, m), (d) => `abs(${d})-${f(t)}`)); }

    // --- space folding: one shape's cost, many copies on screen ---
    // Model the positive side only, and it appears on both. e.g. mirror('xz')
    mirror(axes) {
      return new Shape((c, p, m) => {
        const q = c.let('vec3', p);
        for (const a of axes) c.lines.push(`${q}.${a}=abs(${q}.${a});`);
        return this.emit(c, q, m);
      });
    }
    // n copies around an axis. Model the shape on the first listed axis:
    // +x for 'y' and 'z', +y for 'x'.
    polar(n, axis = 'y') {
      const plane = { x: 'yz', y: 'xz', z: 'xy' }[axis];
      return new Shape((c, p, m) => {
        const q = c.let('vec3', p);
        c.lines.push(`${q}.${plane}=polarFold(${q}.${plane},${f(n)});`);
        return this.emit(c, q, m);
      });
    }
    // Finite grid: spacing [x,y,z], and copies either side of centre [x,y,z].
    grid(spacing, count) {
      const s = vec(spacing.map((v) => v || 1));
      return new Shape((c, p, m) => this.emit(c, c.let('vec3', `${p}-${s}*clamp(round(${p}/${s}),-${vec(count)},${vec(count)})`), m));
    }
    // Twist around the y axis, in radians per unit of height.
    twist(k) {
      return new Shape((c, p, m) => {
        const a = c.let('float', `${p}.y*${f(k)}`);
        const q = c.let('vec3', `vec3(cos(${a})*${p}.x-sin(${a})*${p}.z,${p}.y,sin(${a})*${p}.x+cos(${a})*${p}.z)`);
        // Twisting stretches space, so shrink the distance to stay safe.
        return c.remap(this.emit(c, q, m), (d) => `${d}/sqrt(1.+${f(k * k)}*dot(${p}.xz,${p}.xz))`);
      });
    }

    // Inner paints win over outer ones, so a group can set a default colour.
    paint(mat, ...rest) {
      const own = toMaterial(mat, ...rest);
      return new Shape((c, p) => this.emit(c, p, own));
    }
  }

  // ---- primitives (all centred on the origin; sizes are full sizes) --------

  const prim = (expr) => new Shape((c, p, m) => c.leaf(expr(p), m));

  const sphere = (r) => prim((p) => `length(${p})-${f(r)}`);
  const box = (x, y = x, z = x) => prim((p) => `sdBox(${p},${vec([x / 2, y / 2, z / 2])})`);
  const ellipsoid = (x, y, z) => prim((p) => `sdEllipsoid(${p},${vec([x, y, z])})`);
  const cylinder = (r, h) => prim((p) => `sdCylinder(${p},${f(r)},${f(h / 2)})`);
  const cone = (r1, r2, h) => prim((p) => `sdCone(${p},${f(r1)},${f(r2)},${f(h / 2)})`);
  const torus = (R, r) => prim((p) => `length(vec2(length(${p}.xz)-${f(R)},${p}.y))-${f(r)}`);
  const capsule = (a, b, r) => prim((p) => `sdCapsule(${p},${vec(a)},${vec(b)})-${f(r)}`);

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
    return { name, code };
  };
  const usePolygon = (ctx, fn) => { if (!ctx.funcs.includes(fn.code)) ctx.funcs.push(fn.code); return fn.name; };

  // Draw an outline in the XY plane and push it `depth` along Z.
  const extrude = (points, depth, { smooth = 0 } = {}) => {
    const fn = polygonFunc(points, smooth);
    return new Shape((c, p, m) => c.leaf(`sdExtrude(${usePolygon(c, fn)}(${p}.xy),${p}.z,${f(depth / 2)})`, m));
  };
  // Draw a half-profile as [radius, height] and spin it around Y.
  const lathe = (points, { smooth = 0 } = {}) => {
    const fn = polygonFunc(points, smooth);
    return new Shape((c, p, m) => c.leaf(`${usePolygon(c, fn)}(vec2(length(${p}.xz),${p}.y))`, m));
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
      });
      Object.assign(this, { kind, shapes });
    }
    // Melt the joins together: union(a, b).smooth(0.1)
    smooth(k) { return new Boolean3D(this.kind, this.shapes, k); }
  }

  const union = (...s) => new Boolean3D('union', s);
  const subtract = (...s) => new Boolean3D('subtract', s); // first shape minus the rest
  const intersect = (...s) => new Boolean3D('intersect', s);

  // ---- compiler --------------------------------------------------------------

  const compile = (shape) => {
    const funcs = [];
    const joints = [];
    const body = (full) => {
      const ctx = new Ctx(full, funcs, joints);
      const result = shape.emit(ctx, 'p', DEFAULT_MATERIAL);
      return `${ctx.lines.join('\n  ')}\n  return ${result};`;
    };
    const dist = body(false);
    const full = body(true);
    const n = joints.length;
    const uniforms = n ? `uniform mat3 uJointR[${n}];\nuniform vec3 uJointT[${n}];\n` : '';
    return {
      joints,
      code: `${uniforms}${funcs.join('\n')}
float treeMap(vec3 p){\n  ${dist}\n}
Hit treeHit(vec3 p){\n  ${full}\n}`,
    };
  };

  return {
    material, sphere, box, ellipsoid, cylinder, cone, torus, capsule, extrude, lathe,
    union, subtract, intersect, compile, rotationRows, scenes: {},
  };
})();
