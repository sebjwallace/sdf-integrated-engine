// Measures how bodies actually behave: what the eye would see, frame by frame.
window.TW = (() => {
  const { box, sphere, capsule, torus, cylinder } = SDF;
  const shapes = {
    ball: sphere(.3), block: box(.44, .44, .44).round(.04), pill: capsule([0, -.26, 0], [0, .26, 0], .17),
    ring: torus(.36, .09), beam: box(.14, .14, 2.6),
  };
  const restY = { ball: .3, block: .26, pill: .17, ring: .09, beam: .07 };
  const restRot = { ball: [0, 0, 0], block: [0, 0, 0], pill: [90, 0, 0], ring: [0, 0, 0], beam: [0, 0, 0] };

  // Tracks what the eye sees: fixed points on the body, sampled once per 60 Hz frame.
  const tracker = (b) => {
    const R = b.info.radius;
    const local = [[R, 0, 0], [0, R, 0], [0, 0, R], [-R, 0, 0], [0, -R, 0], [0, 0, -R]].map(p => p.map(v => v * .8));
    const pts = () => local.map(l => { const r = mulMat3(b.R, l); return [b.x[0] + r[0], b.x[1] + r[1], b.x[2] + r[2]]; });
    let p1 = null, p2 = null, n = 0, sum = 0, peak = 0, peakSpin = 0, peakSpeed = 0, flips = 0, lastW = null;
    return {
      sample(expected = 0) {
        const p = pts();
        if (p1 && p2) {
          for (let i = 0; i < p.length; i++) {
            const ax = p[i][0] - 2 * p1[i][0] + p2[i][0], ay = p[i][1] - 2 * p1[i][1] + p2[i][1], az = p[i][2] - 2 * p1[i][2] + p2[i][2];
            const a = Math.hypot(ax, ay, az) * 3600; // units / s^2
            sum += a * a; n++; if (a > peak) peak = a;
          }
        }
        p2 = p1; p1 = p;
        peakSpin = Math.max(peakSpin, Math.hypot(...b.w));
        peakSpeed = Math.max(peakSpeed, b.pointSpeed);
        if (lastW && Math.hypot(...b.w) > .5 && dot3(lastW, b.w) < 0) flips++;
        lastW = [...b.w];
      },
      result(seconds) { return { jitterRms: +Math.sqrt(sum / Math.max(n, 1)).toFixed(1), jitterPeak: +peak.toFixed(0), peakSpin: +peakSpin.toFixed(1), peakPointSpeed: +peakSpeed.toFixed(1), spinFlipsPerSec: +(flips / seconds).toFixed(1) }; },
    };
  };
  const contactCount = (w) => w.manifolds.reduce((n, m) => n + m.contacts.length, 0);

  const noSleep = (fn) => { const keep = PHYSICS.sleepSpeed; PHYSICS.sleepSpeed = -1; try { return fn(); } finally { PHYSICS.sleepSpeed = keep; } };
  const grabPoint = (b) => { // the highest surface sample: where a user tends to click
    let best = null, top = -Infinity;
    for (const p of b.info.points) { const r = mulMat3(b.R, p); const y = b.x[1] + r[1]; if (y > top) { top = y; best = [b.x[0] + r[0], y, b.x[2] + r[2]]; } }
    return best;
  };

  const scenarios = {
    // Left alone on the floor. Anything measured here is motion from nothing.
    rest(name) {
      return noSleep(() => {
        const w = new PhysicsWorld(); const b = w.add(shapes[name], { at: [0, restY[name] + .001, 0], rotate: restRot[name] });
        for (let f = 0; f < 120; f++) w.step(1 / 60);
        const t = tracker(b); let inj = 0, e = w.totalEnergy(); let changes = 0, last = contactCount(w);
        for (let f = 0; f < 240; f++) { w.step(1 / 60); t.sample(); const ne = w.totalEnergy(); if (ne > e) inj += ne - e; e = ne; const c = contactCount(w); if (c !== last) changes++; last = c; }
        return { ...t.result(4), energyInjectedPerSec: +(inj / 4 / b.mass).toExponential(1), contactChangesPerSec: +(changes / 4).toFixed(1), driftMm: +(Math.hypot(b.x[0], b.x[2]) * 1000).toFixed(1) };
      });
    },
    // Held in the air and moved smoothly, then stopped.
    air(name, { rate = 1 } = {}) {
      const w = new PhysicsWorld(); const b = w.add(shapes[name], { at: [1, 2, 0], rotate: restRot[name] });
      const g = grabPoint(b); w.grabBody(b, g); const off = g.map((v, i) => v - [1, 2, 0][i]);
      for (let f = 0; f < 90; f++) w.step(1 / 60);
      const t = tracker(b); let err = 0, n = 0; const R = 1, speed = 1.2;
      for (let f = 0; f < 240; f++) {
        const a = (f / 60) * speed / R; const tgt = [R * Math.cos(a) + off[0], 2 + off[1], R * Math.sin(a) + off[2]];
        if (f % rate === 0) w.moveGrab(tgt);
        w.step(1 / 60); t.sample();
        const r = mulMat3(b.R, w.grab.local); const e = Math.hypot(b.x[0] + r[0] - tgt[0], b.x[1] + r[1] - tgt[1], b.x[2] + r[2] - tgt[2]); err += e; n++;
      }
      // Stop the pointer. How long until it calms down?
      let calm = -1; for (let f = 0; f < 600; f++) { w.step(1 / 60); if (b.pointSpeed < .3) { if (calm < 0) calm = f / 60; } else calm = -1; }
      return { ...t.result(4), lag: +(err / n).toFixed(3), secondsToCalmAfterStop: calm < 0 ? '>10' : +calm.toFixed(1) };
    },
    // Pulled along the floor, pressed slightly into it.
    floor(name, { rate = 1, speed = 1 } = {}) {
      const w = new PhysicsWorld(); const b = w.add(shapes[name], { at: [-2, restY[name] + .001, 0], rotate: restRot[name] });
      for (let f = 0; f < 30; f++) w.step(1 / 60);
      const g = grabPoint(b); w.grabBody(b, g); const y = g[1] - .03;
      const t = tracker(b); let changes = 0, last = contactCount(w), inj = 0, sum = 0, cnt = 0, maxY = 0;
      for (let f = 0; f < 240; f++) {
        if (f % rate === 0) w.moveGrab([g[0] + (f / 60) * speed, y, g[2]]);
        w.step(1 / 60); t.sample(); const c = contactCount(w); if (c !== last) changes++; last = c; sum += c; cnt++; maxY = Math.max(maxY, b.x[1]);
      }
      return { ...t.result(4), contactChangesPerSec: +(changes / 4).toFixed(1), avgContacts: +(sum / cnt).toFixed(1), hop: +(maxY - restY[name]).toFixed(3) };
    },
    // Pulled against something that will not move, and held there.
    wall(name, { rate = 1 } = {}) {
      const wall = box(.4, 6, 6).move(1.2, 3, 0);
      const w = new PhysicsWorld({ world: wall.d }); const b = w.add(shapes[name], { at: [0, restY[name] + .001, 0], rotate: restRot[name] });
      for (let f = 0; f < 30; f++) w.step(1 / 60);
      const g = grabPoint(b); w.grabBody(b, g);
      const t = tracker(b);
      for (let f = 0; f < 240; f++) { if (f % rate === 0) w.moveGrab([Math.min(g[0] + (f / 60) * 1.5, 2), g[1], g[2]]); w.step(1 / 60); if (f > 90) t.sample(); }
      let d = Infinity; for (const p of b.info.points) { const r = mulMat3(b.R, p); d = Math.min(d, wall.d(b.x[0] + r[0], b.x[1] + r[1], b.x[2] + r[2])); }
      return { ...t.result(2.5), deepestIntoWall: +(-Math.min(d, 0)).toFixed(3) };
    },
    // A ring threaded on a peg, pulled sideways against it.
    hooked() {
      const peg = cylinder(.11, 2).move(0, 1, 0);
      const w = new PhysicsWorld({ world: peg.d }); const b = w.add(shapes.ring, { at: [.1, .6, 0] });
      for (let f = 0; f < 60; f++) w.step(1 / 60);
      const g = [b.x[0] + .36, b.x[1], b.x[2]]; w.grabBody(b, g);
      const t = tracker(b);
      for (let f = 0; f < 240; f++) { const a = f / 60; w.moveGrab([g[0] + .5 * Math.min(a, 1) , g[1] + .25 * Math.sin(a * 1.5), g[2] + .3 * Math.sin(a)]); w.step(1 / 60); t.sample(); }
      return { ...t.result(4), stillThreaded: Math.hypot(b.x[0], b.x[2]) < .45 };
    },
  };

  // A setting is a change, applied for the length of a run, then undone.
  const withSetting = (setting, run) => {
    const keep = { ...PHYSICS }; const proto = PhysicsWorld.prototype; const saved = {};
    const patch = (name, fn) => { saved[name] = proto[name]; proto[name] = fn(saved[name]); };
    try { setting && setting(patch); return run(); }
    finally { Object.assign(PHYSICS, keep); for (const k of Object.keys(saved)) proto[k] = saved[k]; }
  };
  return { shapes, scenarios, withSetting };
})();

window.IM = (() => {
  const { box, sphere, capsule, torus } = SDF;
  const shapes = TW.shapes;
  const rest = { ball: .3, block: .26, pill: .17, ring: .09, beam: .07 };
  const landing = (name, rotate) => {
    const w = new PhysicsWorld(); const b = w.add(shapes[name], { at: [0, 1.5, 0], rotate });
    const e0 = w.totalEnergy(); let touched = false, rebound = 0, lowest = 9, spin = 0, gain = 0, jolt = 0, prevV = [...b.v], prevW = [...b.w], sleepAt = -1, e = e0, energyUp = 0;
    for (let f = 0; f < 900; f++) {
      w.step(1 / 60);
      if (w.manifolds.length && !touched) touched = true;
      lowest = Math.min(lowest, b.x[1]);
      if (touched) { rebound = Math.max(rebound, b.x[1] - lowest); spin = Math.max(spin, Math.hypot(...b.w)); }
      const dv = Math.hypot(b.v[0] - prevV[0], b.v[1] - prevV[1], b.v[2] - prevV[2]) + Math.hypot(b.w[0] - prevW[0], b.w[1] - prevW[1], b.w[2] - prevW[2]) * b.info.radius;
      jolt = Math.max(jolt, dv); prevV = [...b.v]; prevW = [...b.w];
      const ne = w.totalEnergy(); if (ne > e + 1e-9) energyUp += ne - e; e = ne; gain = Math.max(gain, (ne - e0) / Math.abs(e0));
      if (b.sleeping && sleepAt < 0) { sleepAt = f / 60; break; }
    }
    return { spin: +spin.toFixed(1), rebound: +rebound.toFixed(2), jolt: +jolt.toFixed(1), injected: +(energyUp / Math.abs(e0) * 100).toFixed(1), sleep: sleepAt < 0 ? '>15' : +sleepAt.toFixed(1) };
  };
  const hit = (na, nb, speed) => {
    const w = new PhysicsWorld(); const b = w.add(shapes[nb], { at: [0, rest[nb] + .001, 0] }); const a = w.add(shapes[na], { at: [-1.2, rest[nb] + .25, 0], velocity: [speed, 0, 0] });
    const e0 = w.totalEnergy(); let gain = 0, sb = 0, sa = 0, va = 0; for (let f = 0; f < 240; f++) { w.step(1 / 60); gain = Math.max(gain, (w.totalEnergy() - e0) / Math.abs(e0)); sa = Math.max(sa, Math.hypot(...a.w)); sb = Math.max(sb, Math.hypot(...b.w)); va = Math.max(va, Math.hypot(...b.v)); }
    return { gainPct: +(gain * 100).toFixed(1), spinA: +sa.toFixed(1), spinB: +sb.toFixed(1), targetSpeed: +va.toFixed(2) };
  };
  // Move the pointer at a steady speed, let go, and see what the throw carries.
  const throwTest = (name, speed) => {
    const w = new PhysicsWorld({ floor: false, gravity: 0 }); const b = w.add(shapes[name], { at: [0, 2, 0] });
    const g = b.x.map((v, i) => v + b.info.points[0][i]); w.grabBody(b, g);
    for (let f = 0; f < 60; f++) { w.moveGrab([g[0] + (f / 60) * speed, g[1], g[2]]); w.step(1 / 60); }
    const atRelease = Math.hypot(...b.v), spin = Math.hypot(...b.w); w.releaseGrab(); w.step(1 / 60);
    return { pointerSpeed: speed, speedAtRelease: +atRelease.toFixed(2), spin: +spin.toFixed(1), keeps: +(Math.hypot(...b.v) / speed).toFixed(2) };
  };
  const quickLift = (name) => { // how hard can the pointer yank? Time to rise one unit.
    const w = new PhysicsWorld(); const b = w.add(shapes[name], { at: [0, rest[name] + .001, 0] }); for (let f = 0; f < 20; f++) w.step(1 / 60);
    const g = b.x.map((v, i) => v + b.info.points[0][i]); w.grabBody(b, g); w.moveGrab([g[0], g[1] + 1, g[2]]); let t = -1; for (let f = 0; f < 240; f++) { w.step(1 / 60); const r = mulMat3(b.R, w.grab.local); if (b.x[1] + r[1] > g[1] + .9) { t = f / 60; break; } } return t < 0 ? '>4' : +t.toFixed(2);
  };
  const pile = () => {
    const sc = SDF.scenes.physics.physics(); const w = new PhysicsWorld({ world: sc.world.d }); for (const { shape, ...o } of sc.bodies) w.add(shape, o);
    const peak = {}; let tot = 0, worst = 0, sleepAt = -1, e = w.totalEnergy(), up = 0;
    const kinds = w.bodies.map((b) => { const v = b.info.volume; return Math.abs(v - analyseShape(shapes.ball).volume) < 1e-3 ? 'ball' : b.info.radius > 1 ? 'beam' : Math.abs(b.info.radius - .45) < .02 ? 'ring' : b.info.radius < .36 ? 'block' : 'pill'; });
    for (let f = 0; f < 1500; f++) { const t = performance.now(); w.step(1 / 60); const m = performance.now() - t; tot += m; worst = Math.max(worst, m); w.bodies.forEach((b, i) => { peak[kinds[i]] = Math.max(peak[kinds[i]] || 0, Math.hypot(...b.w)); }); const ne = w.totalEnergy(); if (ne > e) up += ne - e; e = ne; if (w.awakeCount === 0) { sleepAt = f / 60; break; } }
    const outside = w.bodies.filter(b => Math.abs(b.x[0]) > 5 || Math.abs(b.x[2]) > 5 || b.x[1] < .03).length;
    return { avgMs: +(tot / Math.max(1, (sleepAt < 0 ? 1500 : sleepAt * 60))).toFixed(1), worstMs: +worst.toFixed(1), settle: sleepAt < 0 ? '>25' : +sleepAt.toFixed(1), peakSpin: Object.fromEntries(Object.entries(peak).map(([k, v]) => [k, +v.toFixed(1)])), outside };
  };
  const run = () => ({
    landing: { 'ball': landing('ball', [0, 0, 0]), 'block tilted': landing('block', [25, 10, 30]), 'block flat': landing('block', [0, 0, 0]), 'pill tilted': landing('pill', [50, 0, 20]), 'ring flat': landing('ring', [0, 0, 0]), 'ring tilted': landing('ring', [35, 0, 10]), 'ring on edge': landing('ring', [90, 0, 0]), 'beam tilted': landing('beam', [20, 30, 8]) },
    hits: { 'ball>block': hit('ball', 'block', 4), 'block>ring': hit('block', 'ring', 4), 'ring>block': hit('ring', 'block', 4), 'ring>ring': hit('ring', 'ring', 4) },
    throws: { ring3: throwTest('ring', 3), ring8: throwTest('ring', 8), block8: throwTest('block', 8), ball15: throwTest('ball', 15) },
    lift: { ring: quickLift('ring'), block: quickLift('block'), beam: quickLift('beam') },
    pile: pile(),
  });
  return { run };
})();

// Evaluated in the page. Ground truth comes from a brute-force grid search, which
// shares no code with the engine.
window.trueOverlap = (fa, fb, lo, hi, cells = 36) => {
  let best = Infinity, at = null;
  const s = hi.map((v, i) => (v - lo[i]) / cells);
  for (let i = 0; i <= cells; i++) for (let j = 0; j <= cells; j++) for (let k = 0; k <= cells; k++) {
    const x = lo[0] + i * s[0], y = lo[1] + j * s[1], z = lo[2] + k * s[2];
    const f = Math.max(fa(x, y, z), fb(x, y, z));
    if (f < best) { best = f; at = [x, y, z]; }
  }
  // Refine around the best cell with a finer local grid.
  let span = Math.max(...s);
  for (let round = 0; round < 4; round++) {
    const c = at; const n = 6; const h = span / n;
    for (let i = -n; i <= n; i++) for (let j = -n; j <= n; j++) for (let k = -n; k <= n; k++) {
      const x = c[0] + i * h, y = c[1] + j * h, z = c[2] + k * h;
      const f = Math.max(fa(x, y, z), fb(x, y, z));
      if (f < best) { best = f; at = [x, y, z]; }
    }
    span = h * 2;
  }
  return { depth: Math.max(0, -best), at };
};
const box2 = (b, m = .1) => { const r = b.info.radius + m; return { lo: b.x.map(v => v - r), hi: b.x.map(v => v + r) }; };
// Runs one body against a fixed world shape, tracking the worst true overlap.
window.againstWorld = (worldShape, shape, opts, frames = 240, region = null) => {
  const w = new PhysicsWorld({ world: worldShape.d, floor: false });
  const b = w.add(shape, opts);
  let worst = 0, worstFrame = -1, minY = b.x[1], tiltAtLowest = 0, energy0 = w.totalEnergy(), energyGain = 0;
  const t0 = performance.now(); let simMs = 0;
  for (let f = 0; f < frames; f++) {
    const t = performance.now(); w.step(1 / 60); simMs += performance.now() - t;
    energyGain = Math.max(energyGain, (w.totalEnergy() - energy0) / Math.max(Math.abs(energy0), 1e-6));
    if (b.x[1] < minY) { minY = b.x[1]; tiltAtLowest = Math.acos(Math.min(1, Math.abs(b.R[4]))) * 180 / Math.PI; }
    if (frames <= 60 || f % 3 === 0) {
      let { lo, hi } = box2(b);
      if (region) { lo = lo.map((v, i) => Math.max(v, region[0][i])); hi = hi.map((v, i) => Math.min(v, region[1][i])); }
      const o = lo.every((v, i) => v < hi[i]) ? trueOverlap(b.dist, worldShape.d, lo, hi, 22) : { depth: 0 };
      if (o.depth > worst) { worst = o.depth; worstFrame = f; }
    }
  }
  return { world: w, body: b, worstOverlap: +worst.toFixed(4), worstFrame, energyGainPct: +(energyGain * 100).toFixed(1), finalY: +b.x[1].toFixed(3), minY: +minY.toFixed(3), tilt: +tiltAtLowest.toFixed(1), asleep: b.sleeping, msPerStep: +(simMs / frames).toFixed(2) };
};
window.pairTest = (sa, oa, sb, ob, frames = 240, floor = true) => {
  const w = new PhysicsWorld({ floor });
  const a = w.add(sa, oa), b = w.add(sb, ob);
  let worst = 0, worstFrame = -1; let simMs = 0; const e0 = w.totalEnergy(); let gain = 0;
  for (let f = 0; f < frames; f++) {
    const t = performance.now(); w.step(1 / 60); simMs += performance.now() - t;
    gain = Math.max(gain, (w.totalEnergy() - e0) / Math.max(Math.abs(e0), 1e-6));
    if (f % 3 === 0) {
      // Search inside the smaller body's bounds, around the other's position.
      const small = a.info.radius < b.info.radius ? a : b;
      const r = small.info.radius + .05;
      const o = trueOverlap(a.dist, b.dist, small.x.map(v => v - r), small.x.map(v => v + r), 26);
      if (o.depth > worst) { worst = o.depth; worstFrame = f; }
    }
  }
  return { a, b, energyGainPct: +(gain * 100).toFixed(1), worstOverlap: +worst.toFixed(4), worstFrame, msPerStep: +(simMs / frames).toFixed(2), asleep: w.awakeCount === 0 };
};
window.edgeCases = () => {
  const { box, capsule, cylinder, sphere, union } = SDF;
  const out = {};
  // 1. A bar dropped across a fixed bar, at right angles. Should come to rest on it.
  // Top of the fixed bar is y = 1.05, so a centre below 1.05 + half height means it went in.
  {
    const r = againstWorld(box(4, .1, .1).move(0, 1, 0), box(.1, .1, 4), { at: [0, 1.6, 0] }, 90, [[-2.1, .9, -.1], [2.1, 1.1, .1]]);
    out.barOnBar = { centreY: r.finalY, restsAbove: 1.1, lowestCentreY: r.minY, worstOverlap: r.worstOverlap, gainPct: r.energyGainPct, passedThrough: r.minY < .9, msPerStep: r.msPerStep };
  }
  // 2. Same, off-centre along both bars, so no sample is anywhere near the crossing.
  {
    const r = againstWorld(box(4, .1, .1).move(0, 1, 0), box(.1, .1, 4), { at: [.73, 1.6, .41] }, 45, [[-2.1, .9, -.1], [2.1, 1.1, .1]]);
    out.barOnBarOffCentre = { lowestCentreY: r.minY, worstOverlap: r.worstOverlap, gainPct: r.energyGainPct, tiltDeg: r.tilt, stoppedOnBar: r.minY > 1.05 || r.tilt > 8, ms: r.msPerStep };
  }
  // 3. True sharp edge on sharp edge: both bars turned 45 degrees about their length.
  {
    const r = againstWorld(box(4, .3, .3).rotate(45, 0, 0).move(0, 1, 0), box(.3, .3, 4).rotate(0, 0, 45), { at: [.31, 2, .23], rotate: [0, 0, 45] }, 45, [[-2.1, .7, -.3], [2.1, 1.3, .3]]);
    const top = 1 + .15 * Math.SQRT2, under = .15 * Math.SQRT2;
    out.edgeOnEdge = { lowestCentreY: r.minY, firstTouchAt: +(top + under).toFixed(3), worstOverlap: r.worstOverlap, gainPct: r.energyGainPct, tiltDeg: r.tilt, ms: r.msPerStep };
  }
  // 4. A thin plate's edge on a thin fixed plate's edge.
  {
    const r = againstWorld(box(.04, 1.2, 3).move(0, 1, 0), box(3, 1.2, .04), { at: [.2, 2.6, .3] }, 45, [[-.1, .3, -1.6], [.1, 1.7, 1.6]]);
    out.plateEdgeOnPlateEdge = { lowestCentreY: r.minY, firstTouchAt: 2.2, worstOverlap: r.worstOverlap, gainPct: r.energyGainPct, tiltDeg: r.tilt, ms: r.msPerStep };
  }
  // 5. Round bars, so the edge case is not about sharpness.
  {
    const r = againstWorld(capsule([-2, 1, 0], [2, 1, 0], .05), capsule([0, 0, -2], [0, 0, 2], .05), { at: [.5, 1.6, .9] }, 60, [[-2.1, .9, -.1], [2.1, 1.1, .1]]);
    out.roundBars = { lowestCentreY: r.minY, firstTouchAt: 1.1, worstOverlap: r.worstOverlap, gainPct: r.energyGainPct, tiltDeg: r.tilt, ms: r.msPerStep };
  }
  // 6. Two moving bodies: a bar dropped across another bar resting on the floor.
  {
    const r = pairTest(box(.1, .1, 3), { at: [.4, .55, .3] }, box(3, .1, .1), { at: [0, .05, 0] }, 120);
    out.movingBars = { upperY: +r.a.x[1].toFixed(3), restsAtAtLeast: .15, worstOverlap: r.worstOverlap, gainPct: r.energyGainPct, passedThrough: r.a.x[1] < .1, msPerStep: r.msPerStep };
  }
  // 7. A corner driven into the middle of a large flat face. Samples on the face are irrelevant here.
  {
    const r = pairTest(box(.3, .3, .3), { at: [.37, .95, .22], rotate: [35, 0, 35] }, box(3, .1, 3), { at: [0, .05, 0] }, 120);
    out.cornerOnFace = { worstOverlap: r.worstOverlap, gainPct: r.energyGainPct, y: +r.a.x[1].toFixed(3) };
  }
  // 8. A small sphere landing on a thin rod, between samples.
  {
    const r = againstWorld(box(4, .04, .04).move(0, 1, 0), sphere(.08), { at: [.67, 1.5, 0] }, 45, [[-2.1, .9, -.1], [2.1, 1.1, .1]]);
    out.ballOnThinRod = { lowestCentreY: r.minY, firstTouchAt: 1.1, worstOverlap: r.worstOverlap, gainPct: r.energyGainPct, passedThrough: r.minY < .9 };
  }
  // 9. The analysis of a thin body must match the formulas.
  {
    const bar = analyseShape(box(.1, .1, 4));
    const v = .04;
    out.thinBarAnalysis = {
      radius: +bar.radius.toFixed(3), expectedRadius: +Math.hypot(.05, .05, 2).toFixed(3),
      volumeErrPct: +(100 * Math.abs(bar.volume - v) / v).toFixed(2),
      IxxErrPct: +(100 * Math.abs(bar.inertia[0] - v * (16 + .01) / 12) / (v * (16 + .01) / 12)).toFixed(2),
      IzzErrPct: +(100 * Math.abs(bar.inertia[8] - v * .02 / 12) / (v * .02 / 12)).toFixed(2),
    };
  }
  return out;
};

// Does energy come from nowhere? Settle a cluster, nudge one body, and compare what
// follows with what the nudge put in.
window.EN = (() => {
  const { box, sphere, capsule, torus, cylinder } = SDF;
  const ring = torus(.36, .09), cube = box(.44, .44, .44).round(.04), beam = box(.14, .14, 2.6), ball = sphere(.3), pill = capsule([0, -.26, 0], [0, .26, 0], .17);
  const kinetic = (w) => { let k = 0; for (const b of w.bodies) { k += .5 * b.mass * dot3(b.v, b.v); const L = mulMat3T(b.R, b.w); const I = b.info.inertia.map(v => v * b.mass / b.info.volume); k += .5 * dot3(L, mulMat3(I, L)); } return k; };
  const scenes = {
    rings(w) { for (let i = 0; i < 6; i++) w.add(ring, { at: [(i % 2) * .12 - .06, .4 + i * .3, (i % 3) * .1 - .1], rotate: [i * 23, i * 40, i * 11] }); },
    ringsCubes(w) { for (let i = 0; i < 8; i++) w.add(i % 2 ? cube : ring, { at: [Math.sin(i * 2.3) * .3, .5 + i * .42, Math.cos(i * 2.3) * .3], rotate: [i * 31, i * 17, i * 47] }); },
    beams(w) { for (let i = 0; i < 5; i++) w.add(beam, { at: [Math.sin(i) * .3, .3 + i * .28, Math.cos(i) * .3], rotate: [0, i * 67, i % 2 ? 4 : -4] }); },
    mixed(w) { const l = [ring, cube, beam, ring, ball, pill, beam, cube, ring, beam]; l.forEach((s, i) => w.add(s, { at: [Math.sin(i * 1.9) * .5, .5 + i * .42, Math.cos(i * 1.9) * .5], rotate: [i * 29, i * 53, i * 13] })); },
  };
  const totalMass = (w) => w.bodies.reduce((m, b) => m + b.mass, 0);
  // Drop a cluster, let it settle, nudge one body, and see what happens to the energy.
  const test = (name, { sleep = false, nudge = .5 } = {}) => {
    const keep = PHYSICS.sleepSpeed; if (!sleep) PHYSICS.sleepSpeed = -1;
    try {
      const w = new PhysicsWorld(); scenes[name](w);
      let e = w.totalEnergy(), createdFall = 0; const start = performance.now();
      for (let f = 0; f < 300; f++) { w.step(1 / 60); const ne = w.totalEnergy(); if (ne > e) createdFall += ne - e; e = ne; }
      const mass = totalMass(w);
      // Is it actually still, five seconds later, with nothing touching it?
      let settledKe = 0, restCreated = 0; for (let f = 0; f < 120; f++) { w.step(1 / 60); settledKe = Math.max(settledKe, kinetic(w)); const ne = w.totalEnergy(); if (ne > e) restCreated += ne - e; e = ne; }
      // The nudge: set one body moving gently. It is the only energy we add.
      const target = w.bodies[w.bodies.length - 1]; for (const b of w.bodies) w.wake(b);
      target.v = [target.v[0] + nudge, target.v[1], target.v[2]];
      const nudgeKe = .5 * target.mass * nudge * nudge;
      e = w.totalEnergy(); const e0 = e; let peakKe = 0, created = 0, peakPE = 0, last = 0, calm = -1; const pe0 = w.bodies.reduce((s, b) => s + b.mass * w.gravity * b.x[1], 0);
      let peakOthers = 0;
      for (let f = 0; f < 600; f++) {
        w.step(1 / 60); const k = kinetic(w); peakKe = Math.max(peakKe, k); const ne = w.totalEnergy(); if (ne > e) created += ne - e; e = ne;
        peakPE = Math.max(peakPE, w.bodies.reduce((s, b) => s + b.mass * w.gravity * b.x[1], 0) - pe0);
        for (const b of w.bodies) if (b !== target) peakOthers = Math.max(peakOthers, b.pointSpeed);
        last = k; if (k < nudgeKe * .001) { if (calm < 0) calm = f / 60; } else calm = -1;
      }
      const g = (v) => +(v / nudgeKe).toFixed(2);
      return { fallCreatedPerMass: +(createdFall / mass).toFixed(3), stillKE: +(settledKe / mass).toExponential(1), restCreated: +(restCreated / mass).toExponential(1), peakKE_vsNudge: g(peakKe), createdAfterNudge_vsNudge: g(created), risePE_vsNudge: g(peakPE), lastKE_vsNudge: g(last), calmAt: calm < 0 ? '>10' : +calm.toFixed(1), othersPeakSpeed: +peakOthers.toFixed(2) };
    } finally { PHYSICS.sleepSpeed = keep; }
  };
  const all = (opts) => Object.fromEntries(Object.keys(scenes).map(n => [n, test(n, opts)]));
  return { test, all, scenes };
})();
