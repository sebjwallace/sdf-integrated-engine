# How the physics works

A short orientation. For the model and the reasoning behind each part, read
[The physics engine, explained](physics-theory.md) before changing anything.

## The files

| File | What it does |
|---|---|
| `sdf.js` | The modelling language. Every shape compiles to GLSL for the GPU, and also has `shape.d(x, y, z)`, the same distance function in JavaScript for the CPU. |
| `renderer.js` | Raymarches the scene in WebGL2. Handles the camera, picking, dragging, edits and the cutaway, and applies the gizmo's moves to objects and bodies. |
| `gizmo.js` | The move, rotate and scale handles drawn over the selected object. Works only while the simulation is paused. |
| `field.js` | Bakes a scene's distances into a 3D texture, and keeps live edits (holes, blobs) applied on top. |
| `physics.js` | The rigid body engine. Everything below is in here. |
| `scenes.js` | The scenes, including the physics playground and the `ANCHOR` shape. |
| `tests/` | The test suite. `checks.js` holds the checks, `metrics.js` the measurements behind them, `run.mjs` runs everything in real Chrome. |

All settings are in the `PHYSICS` object at the top of `physics.js`.

## The core idea

Every question a physics engine asks is answered by evaluating SDFs:

| Question | Answer |
|---|---|
| Mass and inertia | Add up small cells where `d < 0`. A cell cut by the surface counts in part, using the distance at its centre. |
| Where is the surface? | Project random points onto `d = 0`, then keep a well-spread set (farthest-point sampling, which finds corners on its own). |
| Is this point touching? How deep? | Evaluate the other body's distance function there. |
| Contact normal | The gradient of that distance function. |
| Did a fast point pass through something? | If it moved further than the distances at both ends of its path allow, it may have crossed a surface, so march along the path to check. |
| Overlaps that samples miss | `deepestOverlap()` walks downhill on `max(dA, dB)` to the deepest point inside both bodies. |

The world is just another body that never moves. Bodies collide with the world and
with each other through the same code.

## The solver

A substepped position-based solver, after Müller et al., *Detailed Rigid Body
Simulation with Extended Position Based Dynamics* (2020). Each frame is 10 substeps.
In each substep:

1. Apply gravity and move bodies by their velocity.
2. Find contacts. A full search runs every 3 substeps; in between, existing contacts
   are re-read at the same material points (`refreshContacts`). Fast bodies, and
   whatever the pointer is holding, are searched every substep.
3. Solve contacts as position corrections, then derive velocities from how far bodies
   actually moved.
4. Apply friction, rolling resistance and bounce at the velocity level.

Things that matter and are easy to break:

- **Pushes come in two kinds.** Stopping a body's approach into a contact is a real
  impulse and becomes velocity. Removing overlap left over from before is a *repair*:
  it moves the body but must not give it any speed. Mixing these up made energy appear
  from nowhere, and piles kept themselves moving. See `applyPush` and `applyRepair`.
- **Load from neighbours is real.** A box on a stack is pressed by the box above.
  Treating that as a repair made stacks collapse. `movedForReal` is what lets a contact
  count it.
- **Contacts between two bodies form a patch.** Friction acts at the patch's
  load-weighted centre, so blocks do not spin when they should grip.
- **A body sinks slightly into what it touches.** The 2 mm allowance (`slop`) keeps
  resting contacts in touch. Stacks depend on it.
- **Sleeping bodies are immovable** inside the solver. They must be woken *before* a
  hit is solved (`wakeOnImpact`), or hits into them lose all their momentum.
- **Moving bodies by hand happens while paused.** The gizmo calls `place()` (a new
  pose, at rest), `setShape()` (a stretched shape, measured again), and then
  `forgetContacts()`, since old contacts no longer describe the scene. Moving a world
  part changes `worldDist` directly: named shapes read their transform from a `holder`.
- **The pointer's pull is force-limited** (`PHYSICS.grab`), with damping applied after
  velocities are derived, so it cannot overpower contacts.

## Traps that have caught people before

These cost real time. Check them before trusting a surprising result.

1. **Check the starting position.** Three times, an apparent physics bug was a test
   that created bodies already inside each other. Measure overlap at frame 0 first.
2. **Settling is chaotic.** Moving starting positions by a few centimetres can turn
   "settles in 7 seconds" into "never settles". Never judge a change on one run; use
   several starting variations.
3. **`pointSpeed` exaggerates for long bodies.** It is speed plus spin times the
   body's radius. A beam's radius is 1.3, so slow spin about its own length reads as
   fast.
4. **Peak kinetic energy is not an energy-conservation test.** Something knocked over
   rightly turns height into speed. Compare total energy, including height, instead.
5. **Measure what the eye sees.** Jitter must be sampled at the screen's frame rate.
   Sampling faster than the screen once made a harmless effect look 40 times worse.
6. **"In the air" must be in the air.** A 2.6 unit beam held by one end at height 2
   reaches the floor.
7. **The tests' limits were mostly set after seeing results.** They catch things
   getting worse; passing is not proof of realism.

## Measuring

`tests/metrics.js` has reusable measurements, all callable in the browser console once
loaded:

- `TW.scenarios.rest | air | floor | wall | hooked`: jitter while resting or dragged.
- `IM.run()`: landings, collisions, throws, the full pile.
- `EN.test(name)`: settle a cluster, nudge one body, and see whether energy grows.
- `edgeCases()`: thin and sharp bodies, against a brute-force overlap measure.
- `trueOverlap(fa, fb, lo, hi)`: brute-force deepest overlap of two distance
  functions, independent of the engine.

To try an idea, change a value in `PHYSICS`, or replace a method on
`PhysicsWorld.prototype`, run the measurement, and restore it. That is how every
change so far was chosen.
