// gizmo.js: the handles drawn over the selected object, for moving, turning and
// scaling it, as in Blender.
//
//   Move    an arrow per axis, a square per plane, and a ring in the centre that
//           moves freely across the screen.
//   Rotate  a ring per axis, and an outer ring that turns about the view.
//   Scale   a line per axis of the object, a square per pair of axes, and an
//           outer ring that scales evenly.
//
// Move and rotate use the world's axes; scale uses the object's own. Hold Ctrl
// (or Cmd) to snap. Escape or the right button cancels a drag.
//
// The gizmo is flat 2D drawing on a canvas laid over the 3D view. It knows nothing
// about what it moves: the viewer supplies where the object is
// (transformFrame) and applies each change (begin/update/endTransform).

const GIZMO = {
  size: 90,       // CSS pixels from the centre to the end of an arrow
  reach: 8,       // how close the pointer must be to a handle, in CSS pixels
  colors: ['#ff3653', '#8adb00', '#2c8fff'],
  active: '#ffe27a',
  snap: { move: 0.1, rotate: 15, scale: 0.1 },
};

const v3 = {
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  mul: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  norm: (a) => { const l = Math.hypot(...a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; },
};

// Row-major 3x3 matrices.
const m3 = {
  identity: () => [1, 0, 0, 0, 1, 0, 0, 0, 1],
  mul: (a, b) => {
    const out = new Array(9);
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) {
      out[r * 3 + c] = a[r * 3] * b[c] + a[r * 3 + 1] * b[3 + c] + a[r * 3 + 2] * b[6 + c];
    }
    return out;
  },
  column: (m, i) => [m[i], m[3 + i], m[6 + i]],
  // Rotation by `angle` radians about the unit vector `a`, right-handed.
  axisAngle: ([x, y, z], angle) => {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    const t = 1 - c;
    return [
      t * x * x + c, t * x * y - s * z, t * x * z + s * y,
      t * x * y + s * z, t * y * y + c, t * y * z - s * x,
      t * x * z - s * y, t * y * z + s * x, t * z * z + c,
    ];
  },
};

// Quaternions as [x, y, z, w], as in physics.js.
const quatAxisAngle = ([x, y, z], angle) => {
  const s = Math.sin(angle / 2);
  return [x * s, y * s, z * s, Math.cos(angle / 2)];
};
const quatMul = ([ax, ay, az, aw], [bx, by, bz, bw]) => [
  aw * bx + ax * bw + ay * bz - az * by,
  aw * by - ax * bz + ay * bw + az * bx,
  aw * bz + ax * by - ay * bx + az * bw,
  aw * bw - ax * bx - ay * by - az * bz,
];

const AXIS_NAMES = ['X', 'Y', 'Z'];

// Distance from point p to the segment a-b, all [x, y] in CSS pixels.
const toSegment = (p, a, b) => {
  const ex = b[0] - a[0];
  const ey = b[1] - a[1];
  const t = Math.max(0, Math.min(1, ((p[0] - a[0]) * ex + (p[1] - a[1]) * ey) / (ex * ex + ey * ey || 1)));
  return Math.hypot(p[0] - a[0] - ex * t, p[1] - a[1] - ey * t);
};

const insidePolygon = (p, pts) => {
  let inside = false;
  for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
    const [xi, yi] = pts[i];
    const [xj, yj] = pts[j];
    if ((yi > p[1]) !== (yj > p[1]) && p[0] < ((xj - xi) * (p[1] - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
};

class Gizmo {
  constructor(viewer, canvas) {
    this.viewer = viewer;
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.mode = 'move';  // 'move', 'rotate', 'scale', or null for no gizmo
    this.handles = [];
    this.hover = null;
    this.drag = null;
    this.drawn = '';
  }

  setMode(mode) {
    if (this.drag) this.cancel();
    this.mode = mode;
    this.hover = null;
  }

  // ---- layout -------------------------------------------------------------------

  // Works out where every handle is on screen for this frame. Returns false when
  // there is nothing to show.
  layout() {
    const v = this.viewer;
    this.handles = [];
    this.frame = null;
    if (!this.mode || v.selected < 0 || !v.canTransform) return false;
    // While dragging, the handles stay where the drag began, except for moving,
    // where they follow the object as Blender's do.
    const frame = v.transformFrame(v.selected);
    if (!frame) return false;
    const pivot = this.drag && this.mode !== 'move' ? this.drag.pivot : frame.pivot;
    const centre = v.project(pivot);
    if (!centre || centre[2] <= 0) return false;
    const { clientHeight: h } = this.canvas;
    const k = (GIZMO.size * 2 * centre[2]) / (3.2 * h);  // world units per gizmo size
    const { eye } = v.basis(v.cam);
    const toCam = v3.norm(v3.sub(eye, pivot));
    const world = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    const axes = this.mode === 'scale' ? (this.drag ? this.drag.axes : frame.local) : world;
    const at = (p) => v.project(p);
    const c = [centre[0], centre[1]];
    this.frame = { pivot, centre: c, k, toCam, axes };
    const add = (handle) => this.handles.push(handle);

    // Axis handles fade out as they turn to point at the camera, where they
    // would be too short to grab.
    const visibility = (a) => {
      const side = Math.sqrt(Math.max(0, 1 - v3.dot(a, toCam) ** 2));
      return Math.max(0, Math.min(1, (side - 0.12) / 0.15));
    };

    if (this.mode === 'move' || this.mode === 'scale') {
      const length = this.mode === 'move' ? 1 : 0.85;
      axes.forEach((a, i) => {
        const alpha = visibility(a);
        if (!alpha) return;
        const from = at(v3.add(pivot, v3.mul(a, k * 0.2)));
        const to = at(v3.add(pivot, v3.mul(a, k * length)));
        if (!from || !to) return;
        add({ kind: 'axis', axis: i, alpha, from, to, hit: (p) => toSegment(p, from, to) < GIZMO.reach + 2 });
      });
      // A square in each plane, between its two axes.
      axes.forEach((n, i) => {
        const facing = Math.abs(v3.dot(n, toCam));
        const alpha = Math.max(0, Math.min(1, (facing - 0.2) / 0.15));
        if (!alpha) return;
        const [a, b] = axes.filter((_, j) => j !== i);
        const corner = (s, t) => at(v3.add(pivot, v3.add(v3.mul(a, k * s), v3.mul(b, k * t))));
        const pts = [corner(0.26, 0.26), corner(0.42, 0.26), corner(0.42, 0.42), corner(0.26, 0.42)];
        if (pts.some((p) => !p)) return;
        add({ kind: 'plane', axis: i, alpha, pts, hit: (p) => insidePolygon(p, pts) });
      });
    }

    if (this.mode === 'move') {
      add({ kind: 'free', centre: c, hit: (p) => Math.hypot(p[0] - c[0], p[1] - c[1]) < 13 });
    }

    if (this.mode === 'rotate') {
      // The near half of each ring is solid and can be grabbed; the far half is faint.
      axes.forEach((n, i) => {
        const [a, b] = axes.filter((_, j) => j !== i);
        const pts = [];
        for (let s = 0; s <= 64; s++) {
          const t = (s / 64) * Math.PI * 2;
          const offset = v3.add(v3.mul(a, Math.cos(t) * k * 0.92), v3.mul(b, Math.sin(t) * k * 0.92));
          const p = at(v3.add(pivot, offset));
          if (!p) return;
          pts.push({ p, front: v3.dot(offset, toCam) > -k * 0.05 });
        }
        add({
          kind: 'ring', axis: i, pts,
          hit: (q) => pts.some((s, j) => j > 0 && s.front && pts[j - 1].front && toSegment(q, pts[j - 1].p, s.p) < GIZMO.reach),
        });
      });
    }

    if (this.mode === 'rotate' || this.mode === 'scale') {
      const r = GIZMO.size * 1.1;
      add({ kind: this.mode === 'rotate' ? 'turn' : 'even', centre: c, r, hit: (p) => Math.abs(Math.hypot(p[0] - c[0], p[1] - c[1]) - r) < GIZMO.reach });
    }
    if (this.mode === 'scale') {
      add({ kind: 'even', centre: c, r: 0, hit: (p) => Math.hypot(p[0] - c[0], p[1] - c[1]) < 11 });
    }
    return true;
  }

  // The handle under the pointer. Smaller targets are checked first.
  handleAt(x, y) {
    if (!this.layout()) return null;
    const order = ['free', 'even', 'plane', 'axis', 'ring', 'turn'];
    const sorted = [...this.handles].sort((a, b) => order.indexOf(a.kind) - order.indexOf(b.kind) || (b.r ?? 0) - (a.r ?? 0));
    return sorted.find((hd) => hd.hit([x, y])) || null;
  }

  same(a, b) { return !!a && !!b && a.kind === b.kind && a.axis === b.axis && (a.r ?? 0) === (b.r ?? 0); }

  // ---- drawing --------------------------------------------------------------------

  // Redraws when anything on screen would change. Called every frame.
  update() {
    const shown = this.layout();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.max(1, Math.round(this.canvas.clientWidth * dpr));
    const h = Math.max(1, Math.round(this.canvas.clientHeight * dpr));
    const resized = w !== this.canvas.width || h !== this.canvas.height;
    if (resized) { this.canvas.width = w; this.canvas.height = h; }
    const key = shown ? JSON.stringify([
      this.mode, this.frame.centre, this.handles.map((hd) => hd.from || hd.pts?.[0] || hd.centre),
      this.hover && [this.hover.kind, this.hover.axis, this.hover.r], this.drag && this.drag.label,
    ]) : '';
    if (key === this.drawn && !resized) return;
    this.drawn = key;
    const g = this.ctx;
    g.setTransform(1, 0, 0, 1, 0, 0);
    g.clearRect(0, 0, w, h);
    if (!shown) return;
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.lineCap = 'round';
    g.lineJoin = 'round';
    const active = this.drag ? this.drag.handle : this.hover;
    const colour = (hd, base) => (this.same(hd, active) ? GIZMO.active : base);

    for (const hd of this.handles) {
      g.globalAlpha = (hd.alpha ?? 1) * (this.drag && !this.same(hd, this.drag.handle) ? 0.35 : 1);
      if (hd.kind === 'axis') {
        const col = colour(hd, GIZMO.colors[hd.axis]);
        g.strokeStyle = col;
        g.fillStyle = col;
        g.lineWidth = 2.5;
        g.beginPath();
        g.moveTo(...hd.from);
        g.lineTo(...hd.to);
        g.stroke();
        const dx = hd.to[0] - hd.from[0];
        const dy = hd.to[1] - hd.from[1];
        const l = Math.hypot(dx, dy) || 1;
        const [ux, uy] = [dx / l, dy / l];
        g.beginPath();
        if (this.mode === 'move') {
          // An arrowhead.
          g.moveTo(hd.to[0] + ux * 14, hd.to[1] + uy * 14);
          g.lineTo(hd.to[0] - uy * 5.5, hd.to[1] + ux * 5.5);
          g.lineTo(hd.to[0] + uy * 5.5, hd.to[1] - ux * 5.5);
        } else {
          // A small square, as Blender's scale handles have.
          const s = 5;
          g.moveTo(hd.to[0] + (ux - uy) * s, hd.to[1] + (uy + ux) * s);
          g.lineTo(hd.to[0] + (ux + uy) * s, hd.to[1] + (uy - ux) * s);
          g.lineTo(hd.to[0] + (-ux + uy) * s, hd.to[1] + (-uy - ux) * s);
          g.lineTo(hd.to[0] + (-ux - uy) * s, hd.to[1] + (-uy + ux) * s);
        }
        g.closePath();
        g.fill();
      } else if (hd.kind === 'plane') {
        const col = colour(hd, GIZMO.colors[hd.axis]);
        g.fillStyle = col;
        g.strokeStyle = col;
        g.lineWidth = 1.5;
        g.beginPath();
        hd.pts.forEach((p, i) => (i ? g.lineTo(...p) : g.moveTo(...p)));
        g.closePath();
        g.globalAlpha *= 0.45;
        g.fill();
        g.globalAlpha /= 0.45;
        g.stroke();
      } else if (hd.kind === 'ring') {
        const col = colour(hd, GIZMO.colors[hd.axis]);
        g.strokeStyle = col;
        for (const front of [false, true]) {
          g.lineWidth = front ? 2.5 : 1.5;
          const alpha = g.globalAlpha;
          if (!front) g.globalAlpha *= 0.25;
          g.beginPath();
          hd.pts.forEach((s, i) => {
            if (i && s.front === front && hd.pts[i - 1].front === front) g.lineTo(...s.p);
            else g.moveTo(...s.p);
          });
          g.stroke();
          g.globalAlpha = alpha;
        }
      } else if (hd.kind === 'free' || hd.kind === 'turn' || hd.kind === 'even') {
        const col = colour(hd, '#ffffff');
        g.strokeStyle = col;
        g.fillStyle = col;
        g.lineWidth = hd.kind === 'free' ? 2 : 1.5;
        if (hd.kind !== 'free' && !this.same(hd, active)) g.globalAlpha *= 0.6;
        g.beginPath();
        g.arc(hd.centre[0], hd.centre[1], hd.r || 9, 0, Math.PI * 2);
        if (hd.kind === 'even' && !hd.r) g.fill();
        else g.stroke();
      }
    }
    g.globalAlpha = 1;

    if (this.drag && this.drag.label) {
      // What the drag has done so far, beside the pointer.
      const [x, y] = this.drag.at;
      g.font = '600 12px system-ui, -apple-system, "Segoe UI", sans-serif';
      const text = this.drag.label;
      const w2 = g.measureText(text).width + 12;
      g.fillStyle = 'rgba(14, 15, 18, 0.82)';
      g.beginPath();
      g.roundRect(x + 16, y + 10, w2, 22, 4);
      g.fill();
      g.fillStyle = '#e8eaee';
      g.fillText(text, x + 22, y + 25);
    }
  }

  // ---- dragging -------------------------------------------------------------------

  hoverAt(x, y) {
    const hd = this.handleAt(x, y);
    if (!this.same(hd, this.hover)) this.hover = hd;
    return !!hd;
  }

  // Starts a drag if the pointer is on a handle. Returns whether it did.
  press(x, y) {
    const handle = this.handleAt(x, y);
    if (!handle) return false;
    const v = this.viewer;
    const { pivot, centre, k, toCam, axes } = this.frame;
    const start = v.beginTransform(v.selected);
    if (!start) return false;
    this.drag = {
      handle, pivot, centre, k, toCam, axes, x, y, at: [x, y], label: '',
      angle: Math.atan2(-(y - centre[1]), x - centre[0]), turned: 0,
    };
    const ray = v.pointerRay(x, y);
    if (handle.kind === 'axis' && this.mode === 'move') this.drag.along = this.alongAxis(ray, axes[handle.axis]);
    if (handle.kind === 'plane' && this.mode === 'move') this.drag.hit = this.onPlane(ray, axes[handle.axis]);
    if (handle.kind === 'free') this.drag.hit = v.onDragPlane(x, y, pivot);
    if (handle.kind === 'axis' && this.mode === 'scale') {
      const tip = v.project(v3.add(pivot, v3.mul(axes[handle.axis], k)));
      const dx = tip[0] - centre[0];
      const dy = tip[1] - centre[1];
      const l = Math.hypot(dx, dy) || 1;
      this.drag.screenAxis = [dx / l, dy / l];
    }
    this.hover = handle;
    return true;
  }

  // Where along the axis through the pivot the pointer's ray passes closest.
  alongAxis({ eye, dir }, a) {
    const w = v3.sub(this.drag ? this.drag.pivot : this.frame.pivot, eye);
    const b = v3.dot(a, dir);
    const denom = 1 - b * b;
    if (denom < 1e-6) return null;
    return (b * v3.dot(dir, w) - v3.dot(a, w)) / denom;
  }

  // Where the pointer's ray meets the plane through the pivot with normal n.
  onPlane({ eye, dir }, n) {
    const pivot = this.drag ? this.drag.pivot : this.frame.pivot;
    const den = v3.dot(n, dir);
    if (Math.abs(den) < 1e-6) return null;
    const t = v3.dot(n, v3.sub(pivot, eye)) / den;
    return t > 0 ? v3.add(eye, v3.mul(dir, t)) : null;
  }

  move(x, y, snap) {
    const d = this.drag;
    if (!d) return;
    const v = this.viewer;
    d.at = [x, y];
    const { handle, axes, centre } = d;
    const ray = v.pointerRay(x, y);
    const step = (value, size) => (snap ? Math.round(value / size) * size : value);
    const fmt = (n) => (n >= 0 ? '+' : '') + n.toFixed(snap ? 1 : 3);

    if (this.mode === 'move') {
      let delta = null;
      if (handle.kind === 'axis') {
        const s = this.alongAxis(ray, axes[handle.axis]);
        if (s === null || d.along === null) return;
        const amount = step(s - d.along, GIZMO.snap.move);
        delta = v3.mul(axes[handle.axis], amount);
        d.label = `${AXIS_NAMES[handle.axis]} ${fmt(amount)}`;
      } else {
        const hit = handle.kind === 'plane' ? this.onPlane(ray, axes[handle.axis]) : v.onDragPlane(x, y, d.pivot);
        if (!hit || !d.hit) return;
        delta = v3.sub(hit, d.hit).map((c) => step(c, GIZMO.snap.move));
        if (handle.kind === 'plane') delta[handle.axis] = 0;
        d.label = delta.map((c, i) => `${AXIS_NAMES[i]} ${fmt(c)}`).filter((_, i) => handle.kind !== 'plane' || i !== handle.axis).join('  ');
      }
      v.updateTransform({ move: delta });
    } else if (this.mode === 'rotate') {
      // The angle swept around the gizmo's centre on screen. Anticlockwise on
      // screen is a positive turn about an axis that points at the viewer.
      const angle = Math.atan2(-(y - centre[1]), x - centre[0]);
      let change = angle - d.angle;
      if (change > Math.PI) change -= 2 * Math.PI;
      if (change < -Math.PI) change += 2 * Math.PI;
      d.angle = angle;
      d.turned += change;
      const axis = handle.kind === 'turn' ? d.toCam : axes[handle.axis];
      const sign = v3.dot(axis, d.toCam) >= 0 ? 1 : -1;
      const degrees = step((d.turned * sign * 180) / Math.PI, GIZMO.snap.rotate);
      d.label = `${handle.kind === 'turn' ? 'View' : AXIS_NAMES[handle.axis]} ${degrees.toFixed(snap ? 0 : 1)}°`;
      v.updateTransform({ rotate: { axis, angle: (degrees * Math.PI) / 180 } });
    } else if (this.mode === 'scale') {
      let factor;
      if (handle.kind === 'axis') {
        const [ux, uy] = d.screenAxis;
        const from = (d.x - centre[0]) * ux + (d.y - centre[1]) * uy;
        const to = (x - centre[0]) * ux + (y - centre[1]) * uy;
        factor = Math.abs(from) > 6 ? to / from : 1 + (to - from) / GIZMO.size;
      } else {
        const from = Math.hypot(d.x - centre[0], d.y - centre[1]);
        const to = Math.hypot(x - centre[0], y - centre[1]);
        factor = from > 6 ? to / from : 1 + (to - from) / GIZMO.size;
      }
      factor = Math.max(snap ? GIZMO.snap.scale : 0.02, step(factor, GIZMO.snap.scale));
      const factors = [0, 1, 2].map((i) => {
        if (handle.kind === 'axis') return i === handle.axis ? factor : 1;
        if (handle.kind === 'plane') return i === handle.axis ? 1 : factor;
        return factor;
      });
      const which = handle.kind === 'axis' ? AXIS_NAMES[handle.axis]
        : handle.kind === 'plane' ? AXIS_NAMES.filter((_, i) => i !== handle.axis).join('') : 'All';
      d.label = `${which} ×${factor.toFixed(snap ? 1 : 3)}`;
      v.updateTransform({ scale: factors });
    }
  }

  release() {
    if (!this.drag) return;
    this.drag = null;
    this.viewer.endTransform(false);
  }

  cancel() {
    if (!this.drag) return;
    this.drag = null;
    this.viewer.endTransform(true);
  }
}
