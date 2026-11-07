// The proofs. Each check states what should be true and how close is close enough.
window.runChecks = () => {
  const results = [];
  const check = (group, name, value, pass, want = '') => results.push({ group, name, value, pass: !!pass, want });
  const near = (a, b, tol) => Math.abs(a - b) <= tol;
  const pct = (a, b) => (100 * Math.abs(a - b)) / Math.abs(b);
  const { sphere, box, torus, union, cylinder } = SDF;

  // --- Mass, measured from the SDF alone, against textbook formulas.
  const s = analyseShape(sphere(0.5)); const sV = (4 / 3) * Math.PI * 0.125;
  check('mass', 'sphere volume % error', +pct(s.volume, sV).toFixed(2), pct(s.volume, sV) < 1, '< 1');
  check('mass', 'sphere inertia % error', +pct(s.inertia[0], 0.4 * sV * 0.25).toFixed(2), pct(s.inertia[0], 0.4 * sV * 0.25) < 1, '< 1');
  const bx = analyseShape(box(1, 0.4, 0.6));
  check('mass', 'box inertia % error', +pct(bx.inertia[0], (0.24 * 0.52) / 12).toFixed(2), pct(bx.inertia[0], (0.24 * 0.52) / 12) < 1, '< 1');
  const t = analyseShape(torus(0.5, 0.15)); const tV = 2 * Math.PI * Math.PI * 0.5 * 0.0225;
  check('mass', 'torus volume % error', +pct(t.volume, tV).toFixed(2), pct(t.volume, tV) < 1, '< 1');
  const bar = analyseShape(box(0.1, 0.1, 4));
  check('mass', 'thin bar radius', +bar.radius.toFixed(3), near(bar.radius, 2.001, 0.01), '2.001');
  check('mass', 'thin bar inertia % error', +pct(bar.inertia[0], (0.04 * 16.01) / 12).toFixed(2), pct(bar.inertia[0], (0.04 * 16.01) / 12) < 1, '< 1');
  const off = analyseShape(union(sphere(0.3), sphere(0.3).move(1, 0, 0)));
  check('mass', 'off-centre shape, centre of mass', +off.com[0].toFixed(3), near(off.com[0], 0.5, 0.005), '0.5');

  // --- Rest.
  {
    const w = new PhysicsWorld(); const b = w.add(sphere(0.3), { at: [0, 1.5, 0] });
    let gain = 0; let last = w.totalEnergy();
    for (let i = 0; i < 300; i++) { w.step(1 / 60); const e = w.totalEnergy(); if (e > last + 1e-3) gain = Math.max(gain, e - last); last = e; }
    check('rest', 'sphere rests at its radius', +b.x[1].toFixed(4), near(b.x[1], 0.3, 0.004), '0.3');
    check('rest', 'sphere falls asleep', b.sleeping, b.sleeping, 'true');
    check('rest', 'dropped straight, stays put (units)', +Math.hypot(b.x[0], b.x[2]).toFixed(4), Math.hypot(b.x[0], b.x[2]) < 0.02, '< 0.02');
    check('rest', 'no energy from nowhere', gain, gain === 0, '0');
  }
  {
    const w = new PhysicsWorld(); const b = w.add(box(0.5, 0.5, 0.5), { at: [0, 1.5, 0], rotate: [37, 20, 14], spin: [3, 1, 2] });
    for (let i = 0; i < 500; i++) w.step(1 / 60);
    check('rest', 'tumbling box lands flat', +Math.max(Math.abs(b.R[3]), Math.abs(b.R[4]), Math.abs(b.R[5])).toFixed(3), Math.max(Math.abs(b.R[3]), Math.abs(b.R[4]), Math.abs(b.R[5])) > 0.999, '1');
    check('rest', 'at its half height', +b.x[1].toFixed(3), near(b.x[1], 0.25, 0.005), '0.25');
  }

  // --- Rolling, against 5/7 g sin a. Rolling resistance is off, to match the formula.
  {
    const keep = PHYSICS.rollingResistance; PHYSICS.rollingResistance = 0;
    for (const deg of [10, 20]) {
      const a = (deg * Math.PI) / 180; const ramp = box(60, 1, 8).rotate(0, 0, -deg);
      const w = new PhysicsWorld({ world: ramp.d, floor: false }); const r = 0.3; const u = 0.5 + r + 0.001;
      const b = w.add(sphere(r), { at: [-14 * Math.cos(a) + Math.sin(a) * u, 14 * Math.sin(a) + Math.cos(a) * u, 0], friction: 0.9 });
      for (let i = 0; i < 30; i++) w.step(1 / 60);
      const x0 = [...b.x]; const v0 = Math.hypot(...b.v); const T = 1.5;
      for (let i = 0; i < T * 60; i++) w.step(1 / 60);
      const measured = (2 * (Math.hypot(b.x[0] - x0[0], b.x[1] - x0[1]) - v0 * T)) / (T * T);
      const expected = (5 / 7) * 9.8 * Math.sin(a);
      check('rolling', `${deg} degree slope, acceleration % error`, +pct(measured, expected).toFixed(2), pct(measured, expected) < 6, '< 6');
      const ratio = (Math.hypot(...b.w) * r) / Math.hypot(...b.v);
      check('rolling', `${deg} degree slope, no slipping (ratio)`, +ratio.toFixed(3), near(ratio, 1, 0.02), '1');
    }
    PHYSICS.rollingResistance = keep;
  }

  // --- Friction: mu = 0.5, so a block should hold below about 26.6 degrees.
  {
    const slide = (deg) => {
      const a = (deg * Math.PI) / 180; const ramp = box(60, 1, 8).rotate(0, 0, -deg);
      const w = new PhysicsWorld({ world: ramp.d, floor: false }); const u = 0.5 + 0.2 + 0.001;
      const b = w.add(box(0.4, 0.4, 0.4), { at: [Math.sin(a) * u - 4 * Math.cos(a), Math.cos(a) * u + 4 * Math.sin(a), 0], rotate: [0, 0, -deg], friction: 0.5 });
      for (let i = 0; i < 60; i++) w.step(1 / 60);
      const x0 = [...b.x]; for (let i = 0; i < 90; i++) w.step(1 / 60);
      return Math.hypot(b.x[0] - x0[0], b.x[1] - x0[1]);
    };
    check('friction', 'holds at 15 degrees (distance moved)', +slide(15).toFixed(3), slide(15) < 0.02, '< 0.02');
    check('friction', 'almost holds at 22 degrees', +slide(22).toFixed(3), slide(22) < 0.2, '< 0.2');
    check('friction', 'slides at 30 degrees', +slide(30).toFixed(3), slide(30) > 0.5, '> 0.5');
    check('friction', 'slides further at 40 degrees', +slide(40).toFixed(3), slide(40) > 4, '> 4');
  }

  // --- Thin walls, including the limit we know about.
  {
    const shoot = (r, speed, wall) => {
      const w = new PhysicsWorld({ world: box(wall, 10, 10).move(3, 0, 0).d, floor: false, gravity: 0 });
      const b = w.add(sphere(r), { at: [0, 0, 0], velocity: [speed, 0, 0], restitution: 0 });
      for (let i = 0; i < 60; i++) w.step(1 / 60);
      return b.x[0];
    };
    check('tunnelling', 'ball, speed 40, 5 cm wall (final x)', +shoot(0.2, 40, 0.05).toFixed(2), shoot(0.2, 40, 0.05) < 3, '< 3');
    check('tunnelling', 'small ball, speed 40, 5 mm wall', +shoot(0.05, 40, 0.005).toFixed(2), shoot(0.05, 40, 0.005) < 3, '< 3');
    check('tunnelling', 'tiny ball is at least stopped at a 2 mm wall', +shoot(0.03, 40, 0.002).toFixed(2), shoot(0.03, 40, 0.002) < 3.5, '< 3.5');
  }

  // --- Concave shapes.
  {
    const peg = cylinder(0.1, 3).move(0, 1.5, 0); const w = new PhysicsWorld({ world: peg.d }); const ring = torus(0.36, 0.09);
    const a = w.add(ring, { at: [0.03, 2.6, 0], rotate: [6, 0, 3] }); const c = w.add(ring, { at: [-0.03, 3.5, 0.02], rotate: [-5, 0, 4] });
    for (let i = 0; i < 900; i++) w.step(1 / 60);
    check('concave', 'first ring threads the peg', +Math.hypot(a.x[0], a.x[2]).toFixed(3), Math.hypot(a.x[0], a.x[2]) < 0.27 && a.x[1] < 0.2, '< 0.27');
    check('concave', 'second ring rests on the first', +c.x[1].toFixed(3), c.x[1] > 0.17 && c.x[1] < 0.35 && Math.hypot(c.x[0], c.x[2]) < 0.27, '0.17 to 0.35');
  }

  // --- Stacks.
  for (const n of [3, 5, 8]) {
    const w = new PhysicsWorld(); const s5 = box(0.5, 0.5, 0.5); const l = [];
    for (let i = 0; i < n; i++) l.push(w.add(s5, { at: [0, 0.26 + i * 0.52, 0] }));
    for (let i = 0; i < 600; i++) w.step(1 / 60);
    const top = l[n - 1]; const h = 0.25 + (n - 1) * 0.5;
    check('stacks', `${n} boxes stand`, +top.x[1].toFixed(3), near(top.x[1], h, 0.05) && Math.hypot(top.x[0], top.x[2]) < 0.1, h.toFixed(2));
  }
  {
    const w = new PhysicsWorld(); const s5 = box(0.5, 0.5, 0.5); const l = [];
    for (let i = 0; i < 6; i++) l.push(w.add(s5, { at: [i % 2 ? 0.06 : -0.06, 0.26 + i * 0.52, ((i % 3) - 1) * 0.05], rotate: [0, i * 9, 0] }));
    for (let i = 0; i < 600; i++) w.step(1 / 60);
    check('stacks', 'misaligned six stand', +l[5].x[1].toFixed(3), near(l[5].x[1], 2.75, 0.05), '2.75');
  }

  // --- Live edits.
  {
    const slab = box(10, 2, 10).move(0, 1, 0); const w = new PhysicsWorld({ world: slab.d }); const b = w.add(sphere(0.3), { at: [0, 2.5, 0] });
    for (let i = 0; i < 200; i++) w.step(1 / 60);
    w.setEdits([{ kind: 0, op: 0, pos: [0, 2, 0], size: [0.6, 0.6, 0.6], k: 0 }]);
    for (let i = 0; i < 300; i++) w.step(1 / 60);
    check('edits', 'falls into a new pit', +b.x[1].toFixed(3), near(b.x[1], 1.7, 0.01), '1.7');
    w.setEdits([]); for (let i = 0; i < 300; i++) w.step(1 / 60);
    check('edits', 'closing the pit pushes it back out', +b.x[1].toFixed(3), near(b.x[1], 2.3, 0.01), '2.3');
  }

  // --- Edges: thin and sharp bodies that surface samples can miss.
  const e = edgeCases();
  check('edges', 'bar on bar, off-centre, stops', e.barOnBarOffCentre.worstOverlap, e.barOnBarOffCentre.worstOverlap < 0.02, '< 0.02');
  check('edges', 'sharp edge on sharp edge, deepest', e.edgeOnEdge.worstOverlap, e.edgeOnEdge.worstOverlap < 0.03, '< 0.03');
  check('edges', 'plate edge on plate edge, deepest', e.plateEdgeOnPlateEdge.worstOverlap, e.plateEdgeOnPlateEdge.worstOverlap < 0.02, '< 0.02');
  check('edges', 'round bars, deepest', e.roundBars.worstOverlap, e.roundBars.worstOverlap < 0.02, '< 0.02');
  check('edges', 'moving bars, deepest', e.movingBars.worstOverlap, e.movingBars.worstOverlap < 0.02, '< 0.02');
  check('edges', 'corner on face, deepest', e.cornerOnFace.worstOverlap, e.cornerOnFace.worstOverlap < 0.01, '< 0.01');
  for (const [k, v] of Object.entries(e)) if (v.gainPct !== undefined) check('edges', `${k}: energy gained %`, v.gainPct, v.gainPct < 0.5, '< 0.5');

  // --- Dragging: the twitch measurements. These limits catch a return to wild
  // motion, and leave room for the run-to-run variation these scenes have.
  const S = TW.scenarios;
  for (const n of ['ball', 'block', 'pill', 'ring', 'beam']) {
    const r = S.rest(n);
    check('calm', `${n} at rest, jitter`, r.jitterRms, r.jitterRms < 2, '< 2');
    const a = S.air(n);
    check('calm', `${n} held in air, jitter`, a.jitterRms, a.jitterRms < 15, '< 15');
    check('calm', `${n} calms after the pointer stops (s)`, a.secondsToCalmAfterStop, typeof a.secondsToCalmAfterStop === 'number' && a.secondsToCalmAfterStop < 3, '< 3');
    const f = S.floor(n);
    check('calm', `${n} dragged along floor, jitter`, f.jitterRms, f.jitterRms < 50, '< 50');
    const wl = S.wall(n);
    check('calm', `${n} pressed against a wall, jitter`, wl.jitterRms, wl.jitterRms < 150, '< 150');
    check('calm', `${n} pressed against a wall, spin`, wl.peakSpin, wl.peakSpin < 25, '< 25');
  }
  check('calm', 'ring pulled along floor stays down (hop)', S.floor('ring').hop, S.floor('ring').hop < 0.05, '< 0.05');
  {
    // Known and open: a beam held by one end with its other end on the floor, like a
    // ladder, keeps wobbling. A cube held the same way is still.
    const lean = (shape, local, height) => {
      const w = new PhysicsWorld(); const b = w.add(shape, { at: [1, height, 0] });
      w.grabBody(b, b.x.map((v, i) => v + local[i]));
      for (let f = 0; f < 300; f++) w.step(1 / 60);
      let sum = 0; for (let f = 0; f < 600; f++) { w.step(1 / 60); sum += b.pointSpeed ** 2; }
      return +Math.sqrt(sum / 600).toFixed(3);
    };
    const cube = lean(box(0.44, 0.44, 0.44).round(0.04), [0.25, 0.25, 0], 0.4);
    check('calm', 'cube held leaning on its edge, wobble', cube, cube < 0.05, '< 0.05');
    const beam = lean(box(0.14, 0.14, 2.6), [0, 0.07, 1.25], 2);
    results.push({ group: 'calm', name: 'beam held leaning on the floor, wobble', value: beam, pass: beam < 0.05, want: '< 0.05', open: true });
  }
  const hk = S.hooked();
  check('calm', 'hooked ring stays threaded', hk.stillThreaded, hk.stillThreaded, 'true');
  check('calm', 'hooked ring, jitter', hk.jitterRms, hk.jitterRms < 40, '< 40');

  // --- Throws keep the pointer's speed, roughly, with no wild spin.
  const m = IM.run();
  for (const [k, v] of Object.entries(m.throws)) {
    check('throws', `${k}: fraction of pointer speed kept`, v.keeps, v.keeps > 0.6 && v.keeps < 1.3, '0.6 to 1.3');
    check('throws', `${k}: spin at release`, v.spin, v.spin < 30, '< 30');
  }
  for (const [k, v] of Object.entries(m.lift)) check('throws', `${k} lifts one unit promptly (s)`, v, typeof v === 'number' && v < 1.5, '< 1.5');
  for (const [k, v] of Object.entries(m.landing)) {
    check('landing', `${k}: energy injected %`, v.injected, v.injected < 8, '< 8');
    check('landing', `${k}: settles (s)`, v.sleep, typeof v.sleep === 'number' && v.sleep < 8, '< 8');
  }
  for (const [k, v] of Object.entries(m.hits)) check('landing', `${k}: energy gained %`, v.gainPct, v.gainPct < 0.5, '< 0.5');
  check('pile', 'nothing escapes the tray', m.pile.outside, m.pile.outside === 0, '0');
  // Known and open: a few rings and capsules keep trembling at a very low level.
  results.push({ group: 'pile', name: 'every body falls asleep within 25 s', value: m.pile.settle, pass: typeof m.pile.settle === 'number', want: 'a number', open: true });

  // --- Energy: a gentle nudge must not set off a pile. The limits on created energy
  // leave room for the small rises from tidying overlaps; a return of the feedback
  // loop reads in the hundreds or thousands.
  for (const [k, v] of Object.entries(EN.all({ sleep: true }))) {
    // Total energy, including height, must never rise above what the nudge added.
    // Peak kinetic energy alone cannot tell this: a nudge may tip something over, and
    // then gravity rightly turns height into speed.
    check('energy', `${k}: total energy rise, times the nudge`, v.peakTotalRise_vsNudge, v.peakTotalRise_vsNudge < 1, '< 1');
    results.push({ group: 'energy', name: `${k}: peak kinetic energy, times the nudge (for reference)`, value: v.peakKE_vsNudge, pass: true, want: '' });
    check('energy', `${k}: energy created, times the nudge`, v.createdAfterNudge_vsNudge, v.createdAfterNudge_vsNudge < 100, '< 100');
  }
  {
    // Left alone, a pile of rings must lose energy, not sustain it.
    const keep = PHYSICS.sleepSpeed; PHYSICS.sleepSpeed = -1;
    const w = new PhysicsWorld(); EN.scenes.rings(w);
    const ke = () => w.bodies.reduce((s, b) => s + 0.5 * b.mass * dot3(b.v, b.v), 0);
    const mass = w.bodies.reduce((s, b) => s + b.mass, 0);
    for (let i = 0; i < 600; i++) w.step(1 / 60);
    const early = ke() / mass; for (let i = 0; i < 1800; i++) w.step(1 / 60); const late = ke() / mass;
    PHYSICS.sleepSpeed = keep;
    check('energy', 'ring pile, kinetic energy after 40 s', +late.toExponential(1), late < 0.01, '< 0.01');
  }

  // --- Liveliness: the opposite of calm. These catch over-damping, which the checks
  // above cannot. Ideals come from conservation of momentum.
  {
    const ball = sphere(0.25); const cube = box(0.44, 0.44, 0.44).round(0.04);
    const head = (asleep, speed) => {
      const w = new PhysicsWorld({ floor: false, gravity: 0 });
      const t = w.add(cube, { at: [0, 0, 0] }); const a = w.add(ball, { at: [-1.2, 0, 0], velocity: [speed, 0, 0] });
      if (asleep) { t.sleeping = true; }
      for (let f = 0; f < 150; f++) w.step(1 / 60);
      const ideal = (1 + PHYSICS.restitution) * a.mass / (a.mass + t.mass) * speed;
      return { got: t.v[0], ideal, kept: (a.mass * a.v[0] + t.mass * t.v[0]) / (a.mass * speed) };
    };
    for (const [asleep, speed] of [[false, 4], [true, 4], [false, 1], [true, 1]]) {
      const r = head(asleep, speed); const label = `${asleep ? 'sleeping' : 'awake'} cube, ball at ${speed}`;
      check('lively', `${label}: cube speed vs ideal`, `${r.got.toFixed(2)} / ${r.ideal.toFixed(2)}`, Math.abs(r.got - r.ideal) < 0.1 * r.ideal, 'within 10%');
      check('lively', `${label}: momentum kept %`, Math.round(r.kept * 100), Math.abs(r.kept - 1) < 0.05, '95 to 105');
    }
  }
  {
    // A bouncy ball should rebound to about e squared of the height it fell.
    const w = new PhysicsWorld(); const b = w.add(sphere(0.2), { at: [0, 1.2, 0], restitution: 0.8 });
    let hit = false, top = 0, lowest = 9;
    for (let f = 0; f < 120; f++) { w.step(1 / 60); lowest = Math.min(lowest, b.x[1]); if (b.v[1] > 0.1) hit = true; if (hit) top = Math.max(top, b.x[1]); }
    const ratio = (top - 0.2) / (1.2 - 0.2);
    check('lively', 'bouncy ball rebounds (height ratio)', +ratio.toFixed(2), Math.abs(ratio - 0.64) < 0.12, 'about 0.64');
  }
  {
    // A row of touching balls, struck at one end: the far end should carry it on.
    const w = new PhysicsWorld({ floor: false, gravity: 0 }); const s = sphere(0.2); const row = [];
    for (let i = 0; i < 5; i++) row.push(w.add(s, { at: [i * 0.4, 0, 0], friction: 0, restitution: 1 }));
    const striker = w.add(s, { at: [-1, 0, 0], velocity: [2, 0, 0], friction: 0, restitution: 1 });
    for (let f = 0; f < 120; f++) w.step(1 / 60);
    const last = row[row.length - 1].v[0];
    check('lively', 'row of balls: far end moves off (speed, of 2)', +last.toFixed(2), last > 1, '> 1');
    const total = striker.v[0] + row.reduce((s2, b) => s2 + b.v[0], 0);
    check('lively', 'row of balls: momentum kept (of 2)', +total.toFixed(2), Math.abs(total - 2) < 0.1, '2');
  }
  {
    // A break: a ball fired into a settled, sleeping group of cubes should scatter it.
    const w = new PhysicsWorld(); const cube = box(0.4, 0.4, 0.4).round(0.03); const group = [];
    for (let i = 0; i < 3; i++) for (let j = 0; j < 2; j++) group.push(w.add(cube, { at: [i * 0.42, 0.201, (j - 0.5) * 0.42] }));
    for (let f = 0; f < 120; f++) w.step(1 / 60);
    const asleep = group.every((b) => b.sleeping);
    const start = group.map((b) => [...b.x]);
    w.add(sphere(0.2), { at: [-1.5, 0.2, 0.21], velocity: [7, 0, 0], density: 3 });
    for (let f = 0; f < 120; f++) w.step(1 / 60);
    const moved = group.filter((b, i) => Math.hypot(b.x[0] - start[i][0], b.x[2] - start[i][2]) > 0.15).length;
    check('lively', `break shot scatters a ${asleep ? 'sleeping' : 'resting'} group (cubes moved, of 6)`, moved, moved >= 3, '3 or more');
  }

  // --- An awkward shape: the playground's anchor. Thin eye, concave arms, flat flukes,
  // weight low down, and smooth blends where its parts meet.
  {
    const com = analyseShape(ANCHOR).com;
    const eyeOf = (b) => mulMat3(b.R, [-com[0], 0.58 - com[1], -com[2]]).map((v, i) => v + b.x[i]);
    {
      const w = new PhysicsWorld(); const b = w.add(ANCHOR, { at: [0, 1.5, 0], rotate: [0, 0, 10] });
      const e0 = w.totalEnergy(); let gain = 0; let slept = -1;
      for (let f = 0; f < 1200; f++) { w.step(1 / 60); gain = Math.max(gain, w.totalEnergy() - e0); if (b.sleeping) { slept = f / 60; break; } }
      const tilt = Math.acos(Math.abs(b.R[4])) * 180 / Math.PI;
      check('anchor', 'dropped on the floor, ends on its side (shaft tilt)', +tilt.toFixed(0), tilt > 45, '> 45');
      check('anchor', 'dropped on the floor, energy gained', +(gain / b.mass).toFixed(4), gain <= 0, '0');
      check('anchor', 'dropped on the floor, settles (s)', slept < 0 ? '>20' : +slept.toFixed(1), slept >= 0, 'within 20');
    }
    {
      const peg = cylinder(0.05, 2).move(0, 1, 0); const w = new PhysicsWorld({ world: peg.d });
      const b = w.add(ANCHOR, { at: [0, 2.4, -0.58], rotate: [90, 0, 0] });
      for (let f = 0; f < 600; f++) w.step(1 / 60);
      const e = eyeOf(b);
      check('anchor', 'eye threads a thin peg (distance from axis)', +Math.hypot(e[0], e[2]).toFixed(3), Math.hypot(e[0], e[2]) < 0.1, '< 0.1');
    }
    {
      // A thin peg's tip meeting the anchor end-on. It must start clear of the peg:
      // an earlier version of this test spawned the anchor already speared on it.
      const peg = cylinder(0.05, 2).move(0, 1, 0); const w = new PhysicsWorld({ world: peg.d });
      const b = w.add(ANCHOR, { at: [0, 3, 0], rotate: [0, 40, 180] });
      let deep = 0;
      for (let f = 0; f < 300; f++) {
        w.step(1 / 60);
        // Both ways round: the anchor's samples inside the peg, and the peg's surface
        // inside the anchor, so a thin overlap cannot hide between samples.
        for (const p of b.info.points) { const r = mulMat3(b.R, p); deep = Math.max(deep, -Math.min(0, peg.d(b.x[0] + r[0], b.x[1] + r[1], b.x[2] + r[2]))); }
        for (let y = 0; y <= 2; y += 0.02) for (let k = 0; k < 8; k++) { const a = k * Math.PI / 4; deep = Math.max(deep, -Math.min(0, b.dist(0.05 * Math.cos(a), y, 0.05 * Math.sin(a)))); }
      }
      check('anchor', 'thin peg tip meets the anchor end-on, deepest (m)', +deep.toFixed(4), deep < 0.005, '< 0.005');
    }
    {
      // Stress: eight anchors and four rings dropped together, spawned clear of each
      // other. Tangling thin parts can briefly overlap; this catches it getting worse.
      const base = SDF.scenes.physics.physics(); const ring = torus(0.36, 0.09);
      const w = new PhysicsWorld({ world: base.world.d });
      for (let i = 0; i < 8; i++) w.add(ANCHOR, { at: [Math.sin(i * 2.4) * 0.9, 2.5 + i * 1.6, Math.cos(i * 2.4) * 0.9], rotate: [i * 47, i * 83, i * 29] });
      for (let i = 0; i < 4; i++) w.add(ring, { at: [Math.sin(i * 1.7 + 1) * 0.6, 1.2 + i * 0.25, Math.cos(i * 1.7 + 1) * 0.6], rotate: [i * 31, 0, i * 17] });
      const e0 = w.totalEnergy(); let gain = 0, worst = 0;
      for (let f = 1; f <= 900; f++) {
        w.step(1 / 60); gain = Math.max(gain, w.totalEnergy() - e0);
        if (f % 12 === 0) for (let i = 0; i < 8; i++) for (let j = i + 1; j < 12; j++) {
          const a = w.bodies[i], b = w.bodies[j];
          if (Math.hypot(a.x[0] - b.x[0], a.x[1] - b.x[1], a.x[2] - b.x[2]) > a.info.radius + b.info.radius) continue;
          const s = a.info.radius < b.info.radius ? a : b; const r = s.info.radius;
          worst = Math.max(worst, trueOverlap(a.dist, b.dist, s.x.map((v) => v - r), s.x.map((v) => v + r), 12).depth);
        }
      }
      const outside = w.bodies.filter((b) => Math.abs(b.x[0]) > 5 || Math.abs(b.x[2]) > 5 || b.x[1] < 0.02).length;
      check('anchor', 'eight anchors and four rings: worst brief overlap (m)', +worst.toFixed(4), worst < 0.03, '< 0.03');
      check('anchor', 'eight anchors and four rings: energy never rises', +gain.toFixed(4), gain <= 0, '0');
      check('anchor', 'eight anchors and four rings: nothing escapes', outside, outside === 0, '0');
    }
  }
  return { results, measurements: m };
};
