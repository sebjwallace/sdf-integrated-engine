// scenes.js: everything here is made purely from the shapes in sdf.js.
// The floor sits at y = 0. `center` and `extent` (half-sizes) give a box that
// must enclose the model, because rays only march inside it. Tighter is faster.

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

SDF.scenes.robot = {
  label: 'Robot',
  center: [0, 1.8, 0],
  extent: [1.3, 1.8, 0.8],
  camera: { theta: 28, phi: 10, dist: 9 },
  build() {
    const { material, box, sphere, cylinder, capsule, torus, union, subtract } = SDF;
    const shell = material('#e8e2d4', 0.3);
    const joint = material('#2a2d33', 0.45, 0.6);
    const eye = material('#37e0ff', 0.2, 0, 3);

    const torso = subtract(
      box(1.1, 1.2, 0.8).round(0.16),
      cylinder(0.24, 1).rotate(90).move(0, 0.1, 0.8),
    ).smooth(0.03).paint(shell);
    const dial = torus(0.2, 0.035).rotate(90).move(0, 0.1, 0.56).paint(joint);
    const head = union(
      subtract(box(0.8, 0.55, 0.65).round(0.14), box(0.64, 0.15, 0.3).move(0, 0.03, 0.42)).smooth(0.04).paint(shell),
      sphere(0.075).move(0.16, 0.03, 0.36).mirror('x').paint(eye),
      capsule([0, 0.4, 0], [0.1, 0.72, 0], 0.016).paint(joint),
      sphere(0.05).move(0.1, 0.74, 0).paint(eye),
    ).rotate(0, -12, 3);
    const arm = union(
      sphere(0.17).paint(joint),
      capsule([0, 0, 0], [0.26, -0.5, 0.05], 0.1).paint(shell),
      sphere(0.11).move(0.26, -0.5, 0.05).paint(joint),
      capsule([0.26, -0.5, 0.05], [0.3, -0.92, 0.22], 0.085).paint(shell),
      torus(0.08, 0.03).rotate(0, 0, 90).move(0.31, -1.04, 0.26).paint(joint),
    );
    const leg = union(
      capsule([0.3, 0, 0], [0.32, -0.62, 0], 0.13).paint(joint),
      box(0.3, 0.1, 0.5).round(0.05).move(0.32, -0.74, 0.08).paint(shell),
    );

    return union(
      torso, dial,
      cylinder(0.11, 0.2).move(0, 0.72, 0).paint(joint),
      head.move(0, 1.12, 0),
      arm.move(0.74, 0.34, 0).mirror('x'),
      leg.move(0, -0.68, 0).mirror('x'),
    ).move(0, 1.57, 0);
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
      [[0, 0], [0.74, 0], [0.8, 0.08], [0.8, 1.42], [0.73, 1.42], [0.71, 0.14], [0, 0.14]],
      { smooth: 1 },
    );
    const handle = torus(0.4, 0.075).rotate(90).move(0.85, 0.74, 0);
    return union(
      subtract(union(profile, handle).smooth(0.06), cylinder(0.71, 1.3).move(0, 0.8, 0)).paint('#2f7d78', 0.18),
      cylinder(0.72, 0.02).move(0, 1.18, 0).paint('#2b1609', 0.08),
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
