// scenes.js: everything here is made purely from the shapes in sdf.js.
// The floor sits at y = 0. `center` and `extent` (half-sizes) give a box that
// must enclose the model, because rays only march inside it. Tighter is faster.
// An animated scene adds animate(t, set), and may add movingExtent: a smaller box
// that follows the model, positioned by the centre animate() returns.

SDF.scenes.car = {
  label: 'Car',
  center: [0, 0.65, 0],
  extent: [2.35, 0.68, 1.12],
  camera: { theta: 52, phi: 9, dist: 7.8 },
  build() {
    const { material, box, cylinder, ellipsoid, capsule, extrude, union, subtract, intersect } = SDF;
    const paintwork = material('#8e0b14', 0.16);
    const glass = material('#3d4a54', 0.04, 1);   // dark, so all you see is reflection
    const chrome = material('#e4e6ea', 0.14, 1);
    const rubber = material('#0a0a0b', 0.82);
    const trim = material('#0e0f11', 0.5);
    const headlamp = material('#fff3d6', 0.15, 0, 0.85);
    const taillamp = material('#ff1206', 0.2, 0, 1.6);

    // Long objects are drawn as outlines and pushed through the car. The body
    // is what remains where the side view and the top view overlap.
    const sideView = extrude([
      [2.14, 0.3], [2.2, 0.47], [2.06, 0.62], [1.25, 0.77], [0.72, 0.86], [-1.4, 0.92],
      [-2.08, 0.93], [-2.18, 0.72], [-2.14, 0.36], [-1.6, 0.21], [1.6, 0.21],
    ], 4, { smooth: 3 });
    const topView = extrude([
      [2.25, -1], [2.25, 0.5], [1.65, 0.85], [0.9, 0.93], [-1.0, 0.93], [-1.75, 0.9], [-2.25, 0.66], [-2.25, -1],
    ], 3, { smooth: 3 }).rotate(90);

    const wheelAt = [[1.36, 0.32, 0.8], [-1.36, 0.32, 0.8]];
    const arch = (x) => cylinder(0.385, 0.9).rotate(90).move(x, 0.32, 0.95);
    const seam = (x) => box(0.014, 0.58, 0.5).move(x, 0.56, 0.95);

    const hull = intersect(sideView, topView).smooth(0.12);
    const body = subtract(
      hull,
      arch(1.36), arch(-1.36), seam(0.48), seam(-0.62),
    ).smooth(0.012).paint(paintwork);

    // The cabin is shaped from three views, so its sides lean inwards.
    const cabin = intersect(
      extrude([[0.92, 0.7], [0.14, 1.19], [-0.6, 1.23], [-1.68, 0.84], [-1.75, 0.7]], 3, { smooth: 3 }),
      extrude([[-1, 0.6], [0.84, 0.6], [0.8, 0.85], [0.57, 1.3], [-1, 1.3]], 5).rotate(0, -90),
    ).smooth(0.06);

    // Glass is the cabin's skin, kept only inside window-shaped regions.
    const skin = cabin.round(0.005);
    const panes = union(
      intersect(skin, box(1.1, 0.6, 1.08).rotate(0, 0, 32).move(0.48, 1, 0)).smooth(0.008),
      intersect(skin, extrude([[0.68, 0.88], [0.15, 1.13], [-0.27, 1.16], [-0.27, 0.88]], 0.7, { smooth: 2 }).move(0, 0, 0.7)),
      intersect(skin, extrude([[-0.35, 0.88], [-0.35, 1.16], [-0.62, 1.16], [-1.1, 0.95], [-1.15, 0.88]], 0.7, { smooth: 2 }).move(0, 0, 0.7)),
      intersect(skin, box(1.1, 0.5, 1.02).rotate(0, 0, -20).move(-1.05, 1.05, 0)).smooth(0.008),
    ).paint(glass);

    const lamps = union(
      intersect(hull.round(0.004), ellipsoid(0.2, 0.038, 0.2).rotate(0, 0, -12).move(1.98, 0.59, 0.6)).paint(headlamp),
      intersect(hull.round(0.004), box(0.5, 0.075, 0.5).round(0.02).move(-2.1, 0.76, 0.55)).paint(taillamp),
    );

    const details = union(
      box(0.12, 0.13, 0.62).round(0.02).move(2.13, 0.36, 0).paint(trim),            // grille
      box(0.1, 0.12, 0.7).round(0.02).move(-2.12, 0.34, 0).paint(trim),             // rear diffuser
      box(1.85, 0.09, 0.1).round(0.012).move(0, 0.25, 0.85).paint(trim),            // side skirt
      capsule([0.06, 0.74, 0.94], [-0.14, 0.74, 0.94], 0.018).paint(chrome),        // door handle
      capsule([0.6, 0.91, 0.74], [0.61, 0.94, 0.84], 0.014).paint(trim),            // mirror arm
      ellipsoid(0.05, 0.036, 0.07).move(0.61, 0.95, 0.88).paint(paintwork),         // mirror
      cylinder(0.05, 0.2).rotate(0, 0, 90).move(-2.09, 0.3, 0.55).paint(chrome),    // exhaust
    );

    // Wheels are modelled with the axle on y, then turned to face outwards.
    const spokes = box(0.21, 0.035, 0.05).move(0.11, 0.1, 0).polar(5, 'y');
    const wheel = union(
      subtract(cylinder(0.27, 0.2).round(0.05), cylinder(0.21, 0.1).move(0, 0.13, 0)).smooth(0.02).paint(rubber),
      spokes.paint('#d5d7db', 0.35, 0.5),
      cylinder(0.055, 0.08).move(0, 0.1, 0).paint(chrome),
      cylinder(0.2, 0.02).move(0, 0.07, 0).paint('#5c5f66', 0.35, 1),
    ).rotate(90);
    const wheels = union(...wheelAt.map(([x, y, z]) => wheel.move(x, y, z)));

    // Dark wheel wells, so the cut-outs read as openings instead of red caves.
    const well = (x) => cylinder(0.385, 0.45).rotate(90).move(x, 0.32, 0.45).paint(trim);

    // Everything is modelled once, on the +z side, and mirrored.
    return union(body, well(1.36), well(-1.36), cabin.paint(paintwork), panes, lamps, details, wheels).mirror('z');
  },
};

// The walk is planned from the feet up. Each foot follows a path in the robot's
// own frame: backwards along the floor while planted, then an arc forwards. The
// knee angles are solved from that path, so planted feet do not slide.
const WALK = {
  period: 1.1,     // seconds for a full cycle of two steps
  planted: 0.6,    // fraction of the cycle a foot is on the floor
  stride: 0.22,    // how far a foot reaches forwards or backwards of the hip
  lift: 0.13,
  circle: 2,       // radius of the path the robot walks around
  thigh: 0.38,
  shin: 0.38,
  ankle: 0.14,     // ankle height above the sole
  hipHeight: 0.84,
  hipWidth: 0.3,
  toe: 0.29,       // sole length in front of and behind the ankle
  heel: 0.13,
};

const smooth01 = (x) => x * x * (3 - 2 * x);
const degrees = (radians) => (radians * 180) / Math.PI;

// Where a foot is at phase u (0 to 1): forward offset, ankle height, toe pitch.
const footPath = (u, reach) => {
  if (u < WALK.planted) {
    return { forward: reach * (1 - (2 * u) / WALK.planted), height: WALK.ankle, pitch: 0 };
  }
  const w = (u - WALK.planted) / (1 - WALK.planted);
  // Toe down as it leaves the floor, toe up as it reaches forward. The ankle
  // rises by however far the pitch would otherwise push the sole into the floor.
  const pitch = 18 * Math.sin(2 * Math.PI * w);
  const tilt = Math.sin((Math.abs(pitch) * Math.PI) / 180);
  return {
    forward: reach * (2 * smooth01(w) - 1),
    height: WALK.ankle + WALK.lift * Math.sin(Math.PI * w) + tilt * (pitch > 0 ? WALK.toe : WALK.heel),
    pitch,
  };
};

// Two-bone IK in the side view. Angles are measured from straight down, towards
// the front, with the knee always bending forwards.
const solveLeg = (hipY, foot) => {
  const dy = foot.height - hipY;
  const reach = WALK.thigh + WALK.shin - 1e-3;
  const dist = Math.min(reach, Math.hypot(foot.forward, dy));
  const atHip = Math.acos((WALK.thigh ** 2 + dist ** 2 - WALK.shin ** 2) / (2 * WALK.thigh * dist));
  const thigh = Math.atan2(foot.forward, -dy) + atHip;
  const kneeForward = WALK.thigh * Math.sin(thigh);
  const kneeY = hipY - WALK.thigh * Math.cos(thigh);
  const shin = Math.atan2(foot.forward - kneeForward, kneeY - foot.height);
  return { thigh, shin };
};

SDF.scenes.robot = {
  label: 'Walking robot',
  center: [0, 1.5, 0],
  extent: [3.9, 1.9, 3.9],
  movingExtent: [1.7, 1.85, 1.7],
  camera: { theta: 30, phi: 15, dist: 13 },
  build() {
    const { material, box, sphere, cylinder, capsule, torus, union, subtract } = SDF;
    const shell = material('#e8e2d4', 0.3);
    const joint = material('#2a2d33', 0.45, 0.6);
    const eye = material('#37e0ff', 0.2, 0, 3);
    const SIDES = [['L', 1], ['R', -1]];

    const torso = subtract(
      box(1.1, 1.2, 0.8).round(0.16),
      cylinder(0.24, 1).rotate(90).move(0, 0.1, 0.8),
    ).smooth(0.03).paint(shell);
    const dial = torus(0.2, 0.035).rotate(90).move(0, 0.1, 0.56).paint(joint);
    const antenna = union(
      capsule([0, 0, 0], [0.1, 0.32, 0], 0.016).paint(joint),
      sphere(0.05).move(0.1, 0.34, 0).paint(eye),
    ).pose('antenna');
    const head = union(
      subtract(box(0.8, 0.55, 0.65).round(0.14), box(0.64, 0.15, 0.3).move(0, 0.03, 0.42)).smooth(0.04).paint(shell),
      sphere(0.075).move(0.16, 0.03, 0.36).mirror('x').paint(eye),
      antenna,
    ).pose('head');

    const arm = (name, side) => union(
      sphere(0.17).paint(joint),
      capsule([0, 0, 0], [side * 0.26, -0.5, 0.05], 0.1).paint(shell),
      union(
        sphere(0.11).paint(joint),
        capsule([0, 0, 0], [side * 0.04, -0.42, 0.17], 0.085).paint(shell),
        torus(0.08, 0.03).rotate(0, 0, 90).move(side * 0.05, -0.54, 0.21).paint(joint),
      ).pose(`elbow${name}`),
    ).pose(`shoulder${name}`);

    const body = union(
      torso.move(0, 0.68, 0), dial.move(0, 0.68, 0),
      cylinder(0.11, 0.2).move(0, 1.4, 0).paint(joint),
      head,
      ...SIDES.map(([name, side]) => arm(name, side)),
    ).pose('body');

    const leg = (name) => union(
      capsule([0, 0, 0], [0, -WALK.thigh, 0], 0.13).paint(joint),
      union(
        capsule([0, 0, 0], [0, -WALK.shin, 0], 0.105).paint(joint),
        box(0.24, 0.08, 0.32).round(0.05).move(0, -0.05, 0.08).paint(shell).pose(`ankle${name}`),
      ).pose(`knee${name}`),
    ).pose(`hip${name}`);

    return union(body, ...SIDES.map(([name]) => leg(name))).pose('root');
  },

  animate(t, set) {
    const u = (t / WALK.period) % 1;
    const wave = (phase, cycles = 1) => Math.cos(2 * Math.PI * cycles * (u - phase));
    const mid = WALK.planted / 2; // when the first foot is directly under the hip

    // Highest over a planted foot, lowest as weight changes feet.
    const hipY = WALK.hipHeight + 0.018 * wave(mid, 2);

    // Walk speed follows from the stride, so the planted foot keeps pace with
    // the floor. On a circle the outer foot covers more ground than the inner.
    const speed = (2 * WALK.stride) / (WALK.period * WALK.planted);
    const turnRate = speed / WALK.circle;
    const angle = t * turnRate;
    const position = [WALK.circle * Math.cos(angle), 0, WALK.circle * Math.sin(angle)];
    set('root', { rotate: [0, -degrees(angle), 0], move: position });

    for (const [name, side, offset] of [['L', 1, 0], ['R', -1, 0.5]]) {
      const reach = (WALK.stride * (WALK.circle + side * WALK.hipWidth)) / WALK.circle;
      const foot = footPath((u + offset) % 1, reach);
      const { thigh, shin } = solveLeg(hipY, foot);
      // A positive x rotation swings a limb backwards, so forward angles negate.
      set(`hip${name}`, { rotate: [-degrees(thigh), 0, 0], move: [side * WALK.hipWidth, hipY, 0] });
      set(`knee${name}`, { rotate: [degrees(thigh - shin), 0, 0], move: [0, -WALK.thigh, 0] });
      set(`ankle${name}`, { rotate: [foot.pitch + degrees(shin), 0, 0], move: [0, -WALK.shin, 0] });

      // Each arm swings with the opposite leg.
      const forward = -wave(offset);
      set(`shoulder${name}`, { rotate: [-22 * forward, 0, side * 4], move: [side * 0.74, 1.02, 0] });
      set(`elbow${name}`, { rotate: [-18 - 12 * Math.max(forward, 0), 0, 0], move: [side * 0.26, -0.5, 0.05] });
    }

    // The body shifts over the planted foot and twists against the stride.
    set('body', {
      rotate: [3, 3 * wave(0), 2 * wave(mid)],
      move: [0.035 * wave(mid), hipY, 0],
    });
    set('head', { rotate: [-3 + 1.5 * wave(mid, 2), -5 * wave(0), -2 * wave(mid)], move: [0, 1.8, 0] });
    // The antenna lags behind each step, like a spring.
    set('antenna', { rotate: [5 * wave(mid + 0.1, 2), 0, 7 * wave(mid + 0.12)], move: [0, 0.4, 0] });

    return [position[0], 1.75, position[2]];
  },
};

// A model with real insides, for the cutaway tool. Every part is its own solid,
// so a cut shows how they fit together. The flow axis is x and the stem is y.
SDF.scenes.valve = {
  label: 'Ball valve',
  center: [0, 1.25, 0],
  extent: [1.65, 1.3, 1.1],
  camera: { theta: 38, phi: 20, dist: 11 },
  cutAxis: 'z',
  build() {
    const { material, box, sphere, cylinder, torus, lathe, union, subtract } = SDF;
    const casting = material('#4f6d7a', 0.45, 0.3);
    const brass = material('#c9a14a', 0.22, 1);
    const seal = material('#ece8dc', 0.6);
    const steel = material('#9ea3aa', 0.3, 1);
    const handle = material('#b4352a', 0.35);
    const toFlow = (shape) => shape.rotate(0, 0, 90); // lathe axis y becomes flow axis x

    const stemBore = cylinder(0.11, 1.4).move(0, 0.8, 0);
    const housing = subtract(
      union(
        toFlow(lathe([
          [0, -1.5], [0.75, -1.5], [0.75, -1.32], [0.46, -1.32], [0.46, -0.85], [0.82, -0.55],
          [0.82, 0.55], [0.46, 0.85], [0.46, 1.32], [0.75, 1.32], [0.75, 1.5], [0, 1.5],
        ])),
        cylinder(0.25, 0.6).move(0, 0.92, 0),
      ).smooth(0.04),
      toFlow(cylinder(0.3, 3.4)),
      sphere(0.62),
      stemBore,
    ).paint(casting);

    // Modelled once at +y of the flow axis, then mirrored to the other flange.
    const bolts = toFlow(cylinder(0.04, 0.26).move(0.6, 1.41, 0).polar(6, 'y').mirror('y')).paint(steel);
    const seats = toFlow(torus(0.42, 0.06).move(0, 0.47, 0).mirror('y')).paint(seal);
    const ball = subtract(sphere(0.59), toFlow(cylinder(0.3, 1.6))).paint(brass);
    const stem = cylinder(0.1, 0.92).move(0, 0.92, 0).paint(steel);
    const packing = torus(0.14, 0.035).move(0, 0.98, 0).paint(seal);
    const lever = union(
      cylinder(0.17, 0.12).move(0, 1.3, 0).paint(steel),
      box(1.3, 0.07, 0.17).round(0.02).move(0.56, 1.36, 0).paint(handle),
    );

    const stand = box(0.36, 0.34, 0.6).round(0.02).move(1.3, -0.75, 0).mirror('x').paint(casting);
    const plate = box(3.2, 0.06, 1).round(0.02).move(0, -0.92, 0).paint('#26292e', 0.5);

    return union(housing, bolts, seats, ball, stem, packing, lever, stand, plate).move(0, 1, 0);
  },
};

SDF.scenes.mug = {
  label: 'Coffee mug',
  center: [0.25, 0.75, 0],
  extent: [1.1, 0.78, 0.85],
  camera: { theta: 35, phi: 26, dist: 6.5 },
  build() {
    const { cylinder, torus, lathe, union, subtract } = SDF;
    const profile = lathe(
      // Starts past the axis, so smoothing the corners cannot dimple the centre.
      [[-0.3, 0], [0.74, 0], [0.8, 0.08], [0.8, 1.42], [0.73, 1.42], [0.71, 0.14], [-0.3, 0.14]],
      { smooth: 1 },
    );
    const handle = torus(0.4, 0.075).rotate(90).move(0.85, 0.74, 0);
    return union(
      subtract(union(profile, handle).smooth(0.06), cylinder(0.71, 1.3).move(0, 0.8, 0)).paint('#2f7d78', 0.18),
      cylinder(0.72, 1.06).move(0, 0.66, 0).paint('#2b1609', 0.08),
    );
  },
};

SDF.scenes.sculpture = {
  label: 'Twisted sculpture',
  center: [0, 1.6, 0],
  extent: [1.12, 1.6, 1.12],
  camera: { theta: 25, phi: 12, dist: 10 },
  build() {
    const { box, cylinder, sphere, torus, union, subtract } = SDF;
    const column = subtract(
      box(0.9, 2.9, 0.9).round(0.05),
      sphere(0.56).grid([0, 0.9, 0], [0, 2, 0]),
    ).smooth(0.04).twist(0.9).paint('#c79a3a', 0.28, 1);
    const rings = torus(0.86, 0.035).grid([0, 0.9, 0], [0, 1, 0]).paint('#e8e4dc', 0.35);
    return union(
      column.move(0, 1.57, 0),
      rings.move(0, 1.57, 0),
      cylinder(1.05, 0.1).round(0.02).move(0, 0.07, 0).paint('#17181b', 0.3),
    );
  },
};
