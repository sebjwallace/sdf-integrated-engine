# The physics engine, explained

Read this before changing `physics.js`. The code shows *what* happens; this explains
*why*: the model each part rests on, the reasoning behind each rule, and the
mistakes that were made before a rule was found. Most rules here were learned by
breaking something, so a change that looks like a harmless simplification often
undoes one of them.

Each section names the functions it describes. Rules that are easy to break are
marked **Rule**, with the test in `tests/checks.js` that catches breaking them.

## Contents

1. [What a distance field can and cannot tell you](#1-what-a-distance-field-can-and-cannot-tell-you)
2. [A body, measured from its distance function](#2-a-body-measured-from-its-distance-function)
3. [Position-based dynamics, and why it shapes everything](#3-position-based-dynamics-and-why-it-shapes-everything)
4. [Finding contacts](#4-finding-contacts)
5. [Solving a contact: stopping versus repairing](#5-solving-a-contact-stopping-versus-repairing)
6. [Friction](#6-friction)
7. [Bounce, and the velocity pass](#7-bounce-and-the-velocity-pass)
8. [Sleeping and waking](#8-sleeping-and-waking)
9. [The pointer's pull](#9-the-pointers-pull)
10. [The world, edits, and how rendering reads physics](#10-the-world-edits-and-how-rendering-reads-physics)
11. [Energy: what should happen to it](#11-energy-what-should-happen-to-it)
12. [Approximations, all in one place](#12-approximations-all-in-one-place)
13. [Before you change anything](#13-before-you-change-anything)

---

## 1. What a distance field can and cannot tell you

A signed distance function `d(p)` gives, for any point `p`, the distance to the
nearest surface: positive outside, negative inside, zero on the surface. Everything
in this engine is built on four properties of `d`, and on their limits.

**The sign says inside or outside.** That is how contacts, mass and edits work.

**The value bounds how far away the surface is.** Nothing is closer than `|d|`. For
an exact SDF it is exactly the distance; for many useful shapes it is only a lower
bound (see below). This "nothing is closer than d" property is what makes three
things safe:
- raymarching, which steps by `d`;
- the tunnelling check (section 4), which asks whether a point moved further than
  the distances at both ends of its path allow;
- culling, which skips any body whose centre is further from the world than the
  body's radius.

**The gradient is the direction away from the nearest surface.** Outside a shape and
near its surface, `∇d` is the outward normal. That is where contact normals come
from. It is computed numerically by `gradientOf()`, from four samples at the corners
of a small tetrahedron, 1.5 mm across by default.

**Inside a shape, the gradient points to the *nearest* surface, not the one you came
in through.** Deep inside, the nearest surface changes abruptly along the shape's
medial axis: halfway through a slab, the gradient flips from one face to the other.
This one fact explains several rules:
- A deeply overlapping point can be pushed out of the wrong side (see the thin walls
  investigation).
- Normals at sharp edges flip between faces, so overlap contacts measure the gradient
  across a wider span to average over the ridge (section 4).
- Deep overlaps are repaired gently, never in one jump (section 5).

### Exact and inexact distances

Primitives (sphere, box, capsule, torus) are exact. Some operations are not:
- `union` with `min` is exact outside and only a bound inside;
- `intersect` and `subtract` give bounds;
- `.smooth()` blends, `twist`, and `scale` with uneven factors give bounds.

A bound underestimates the distance. Raymarching and the tunnelling check stay safe,
because they only need "nothing is closer". Physics is a little less accurate: a
contact's measured depth is approximate, and a point may need a few small steps
instead of one. Nothing in the engine assumes exact distances, and nothing should.

### One shape, two languages

Every shape in `sdf.js` compiles to GLSL for rendering, and also carries `shape.d`,
the same function in JavaScript for physics. They are written separately, so they can
drift apart.

**Rule.** The JavaScript and GLSL distances must agree. A shape that renders in one
place and collides in another is very confusing to debug. *Guarded by:* `cpu vs gpu`,
which compares them on every baked scene.

---

## 2. A body, measured from its distance function

`analyseShape()` measures a shape once, and caches the result by shape object.

### Mass properties by integration

Mass is the volume of `d < 0`. It is found by filling the shape's bounds with a grid,
40 cells along each axis, and adding up the cells inside. A cell cut by the surface
counts in part, using the distance at its centre:

```
fill = clamp(0.5 − d(centre) / cellSize, 0, 1)
```

Fractional fill is what makes the results accurate: within 0.4% of textbook values
for spheres, boxes and tori. Counting whole cells instead was far worse: a box's
inertia came out 9.7% wrong, against 0.07% now. The centre of
mass and the inertia tensor come from the same cells, each with its own small inertia
added.

Bodies have density 1 unless told otherwise, so mass is volume.

### Where is the surface?

Physics needs points on each body's surface. They are found without a mesh:

1. **Project random points onto the surface.** From any point, `p ← p − d(p)·∇d(p)`,
   repeated, lands on `d = 0`. This works from anywhere, including far away.
2. **Find how far the shape reaches** from those projected points. An earlier version
   marched inwards along fixed directions instead, missed the ends of thin bars, and
   decided a 4 unit bar had a radius of 0.42. Bodies were then culled as "too far
   away to touch" and passed straight through each other.
3. **Keep a well-spread set** by farthest-point sampling: each new sample is the
   candidate furthest from all those already chosen. This finds corners and ends on
   its own, without being told what a corner is.

### Thin shapes get more samples

A contact can fall between samples. That matters most where a shape is thin and
curved, such as a ring's tube: a resting ring could touch the floor between samples
and be held up only by the backup overlap search, which dropped out every few frames.
The ring then sagged and was pushed back up, over and over: a visible tremble.

So the analysis also measures how thin the shape is. The deepest point inside is
half its thickness. If samples end up more than `thinSpacing` (1.4) half-thicknesses
apart, the shape gets 64 samples instead of 32. Rings, beams and plates qualify;
balls, blocks and capsules do not.

### Two origins

A shape has its own origin, wherever it was modelled. A body's position `x` is its
centre of mass. `RigidBody.dist()` converts between them for queries, and
`renderTranslation()` converts back for drawing. Mixing the two up shows as bodies
that rotate around the wrong point.

---

## 3. Position-based dynamics, and why it shapes everything

The solver follows Müller et al., *Detailed Rigid Body Simulation with Extended
Position Based Dynamics* (2020). Its central idea explains most of the rules in this
document, so it is worth understanding properly.

### The idea

A classic engine computes forces, turns them into velocities, then moves bodies.
Position-based dynamics works the other way around:

1. Move every body freely, by its velocity, as if nothing were in the way.
2. **Correct positions** so that constraints hold: no overlaps, the grip on the
   pointer, and so on.
3. **Derive the velocity from how far each body actually moved:**
   `v = (x − xPrev) / h`.

Step 3 is the whole point. Collisions never need forces or stiff springs, which is
why the method is stable. But it has a consequence that is easy to forget:

> **Any change to a body's position becomes velocity, unless something deliberately
> excludes it.**

Nearly every serious bug in this engine's history was a position change turning into
speed that it should not have had. Section 5 explains the main one.

### Substeps

Each frame of 1/60 s is split into 10 substeps, after the paper's finding that many
small steps with a single correction each beat few steps with many corrections.
Substeps are what make stacks of eight to twelve boxes stand.

`advance()` runs fixed steps of 1/60 s from real time, at most three per frame, so
results do not depend on frame rate.

### What happens in one substep

In order, from `substep()`:

1. **Integrate.** Add gravity to `v`, clamp to `maxSpeed`, move `x` by `v·h`, rotate
   `q` by `ω·h`.
2. **Find contacts** (section 4). A full search every `searchEvery` substeps; in
   between, re-read existing contacts at the same material points.
3. **Pointer pull** (section 9), as a position correction.
4. **Normal passes.** Solve every contact, three times over, sharing load on the
   first passes (section 5).
5. **Static friction** for each patch (section 6), then one more normal pass.
6. **Apply** the accumulated corrections, then **derive velocities** from the motion,
   excluding repairs (section 5).
7. **Velocity pass** (section 7): sliding friction, rolling and twisting resistance,
   and bounce.
8. **Damping** of the held body (section 9).

Sleep is updated once per step, after the substeps.

### How hard is it to move a point?

Every correction uses the same quantity: how much a point at offset `r` from the
centre of mass moves along a direction `n` per unit of push. That is the generalised
inverse mass:

```
w = 1/m + (r × n) · I⁻¹ (r × n)
```

The first term is the body sliding; the second is it turning. To move a point by `C`
along `n`, push with `λ = C / w`, and the body moves by:

```
Δx = λ n / m        Δθ = I⁻¹ (r × λ n)
```

Between two bodies, `w` is the sum of both, and they are pushed in opposite
directions. In the code, `resistance()` is `w`, and `applyPush()` adds `Δx` and `Δθ`
to running totals (`dx`, `dth`), which are applied together after all the passes.

The world-space inverse inertia `I⁻¹` is computed once per substep, in `prepare()`.
A sleeping body gets zero inverse mass and inertia there: to the solver, it is
immovable (section 8).

---

## 4. Finding contacts

A contact is a point, a normal `n` (pointing from the other body towards this one), a
depth, and the offsets `rA`, `rB` from each body's centre of mass. Contacts between
the same two bodies are grouped into a **manifold**, or patch.

### Search and refresh

A full search is the expensive part. It runs every 3 substeps (`searchEvery`). In
between, `refreshContacts()` keeps each contact attached to the same point *of the
body* and simply re-reads the distance there, which is far cheaper.

Two cases always get a full search on every substep: fast bodies (`fastSpeed`, by the
fastest point on the body, not its centre), and whatever the pointer is holding.

"Fastest point" matters. A long beam spinning quickly can have its centre almost
still while its ends sweep through things. Using the centre's speed alone missed this.

### Culling

Before testing anything, the search rejects pairs that cannot touch. A body is
tested against the world only if the world's distance at its centre is less than the
body's radius plus a margin plus how far it can move before the next search. Two
bodies are tested only if their bounding spheres meet.

This is only safe because the body's radius is right: see the thin bar story in
section 2.

### Samples against a field

`collide(A, B, field)` tests body A's samples against a distance function: either
the world, or body B. Every pair is tested both ways round: A's samples against B,
then B's against A.

For each sample:

1. **Read the distance** to the field. Far away, skip it.
2. **Tunnelling check**, for moving bodies. If the point's path since the last search
   is longer than the distances at both its ends added together, the straight path
   could have crossed a surface. Only then, march along the path with `sweep()` to
   find where. This costs almost nothing when nothing is wrong, which is the usual
   case.
3. **Slide to the true contact.** A sample rarely lands exactly on the contact. Near
   the field, it is moved along A's own surface, downhill on the field, until the two
   surfaces' normals are parallel: that point is the true contact. Each step moves
   along the surface, then projects back onto it. Without this, a sphere rests on its
   samples like a polyhedron and rocks instead of rolling. With it, a sphere rolls
   without slipping, and rolling down a slope is within about 2% of the textbook
   acceleration.
4. **Keep it if it is inside** the field.

Then contacts that slid to the same place are merged, keeping the deepest, and the
patch is cut to **at most four well-spread points**. More points add nothing to
stability, and they rock against each other, which reads as extra load and made
friction too strong.

### The overlap search, for what samples miss

Two thin bodies can overlap with no sample anywhere near. Round bars crossing at
right angles fell straight through each other this way. `collideOverlap()` asks the
two distance functions directly, using `deepestOverlap()`:

1. **Get inside both bodies.** From a starting point, if it is outside either body,
   project onto the one it is further outside, and repeat. For two overlapping convex
   shapes this finds a point inside both; for concave ones it usually does.
2. **Walk to the deepest point.** Inside both, walk downhill on `max(dA, dB)`: towards
   whichever body's surface is nearer, or both when they are about equal. The minimum
   is the point deepest inside both. Its depth is `δ = −max(dA, dB)`.

It starts from several places: where an overlap was found last time, the midpoint of
the two bodies, and the samples closest to touching. It gives up after two misses,
because most pairs are not touching at all. **Every distinct overlap is kept**, since
two linked rings can touch in two places.

Turning the deepest point into a contact:

- **Normal:** the gradient of `dB − dA`, measured across a wider span (at least 3 mm)
  so that it averages over sharp ridges instead of flipping between faces.
- **Depth:** for two faces meeting head-on, they separate when moved apart by `2δ`.
  Faces meeting at an angle to the push separate more slowly, so the depth is
  `2δ / s`, where `s` is how squarely the faces meet the normal, never less than 0.5.
- These contacts are marked **soft**: a single point on a long body is a lever, so
  they ease out of overlap rather than push hard (section 5).

An overlap contact is only added if the samples have not already covered it. Two
faces resting flat on each other have a whole plateau of nearly equal deepest
points, and treating each as a separate contact made stacks fall.

---

## 5. Solving a contact: stopping versus repairing

This is the heart of the engine, and the part most worth understanding before
changing anything. It lives in `solveNormal()`.

### A push has two jobs

When two bodies overlap, the correction does two different things:

1. **Stopping:** the bodies were moving into each other, and must stop. That is a
   real impulse. It should change their velocity, and it is how momentum passes from
   one body to another.
2. **Repairing:** some overlap is left over from before, perhaps because a contact
   was found late, or a body was placed overlapping. Removing it is tidying an error.
   It should move the body, but must **not** give it any speed.

Remember section 3: any change in position becomes velocity unless deliberately
excluded. Before this split existed, every repair turned into speed. A body pushed
out of an overlap flew into its neighbour, created a new overlap, was pushed out
again, and so on. A gentle nudge to a settled pile of rings produced **700 times** the
nudge's energy, and piles kept moving on their own.

So repairs are accumulated separately (`applyRepair()`, into `repairDx` and
`repairDth`), and subtracted when velocities are derived.

**Rule.** Repairs must never become velocity. *Guarded by:* every `energy` check, and
`ring pile, kinetic energy after 40 s`.

### What counts as real motion: load travels down a stack

Here is the subtlety that made the first version of that fix collapse stacks. Picture
a stack. The top box presses on the one below it. At the lower contact, that pressing
is not the lower box's own approach, but it is not leftover error either: it is
**load**, passed down from above, and supporting it is a real impulse.

So the amount of a push that counts as stopping is:

```
approach = closing − (real motion of the contact point so far this substep)·n
```

where `closing` is the contact's own approach this substep (`−vnBefore·h`), and the
real motion is everything other contacts have already done to these two bodies,
excluding repairs (`movedForReal()`). Neighbours pressing the bodies together make
that motion negative along `n`, which raises `approach`.

**Rule.** Load from neighbours is real. *Guarded by:* `8 boxes stand`,
`misaligned six stand`.

### How much to push: the allowance

```
allowed = max(closing, min(depth − slop, repairLimit(depth) · h))
remaining = allowed − (motion so far)·n
```

Three ideas are packed into this:

- **Always stop the approach**, however shallow the contact. The `closing` term does
  this. An earlier rule let bodies sink freely until they were 2 mm deep. They would
  fall, be caught, and repeat: a constant tiny bounce.
- **Leave a little overlap at rest** (`slop`, 2 mm). Resting contacts stay slightly
  inside each other, so a body that rocks by a hair keeps its contacts. Without it,
  contacts flickered between search and refresh, and stacks of five collapsed. With
  it, twelve stand. Bodies rest about 2 mm low; that is intended.
- **Repair gently when shallow, faster when deep.** `repairLimit()` is 0.5 units/s
  plus 4 per unit of depth, at most 6. Fast repair of shallow overlaps stretched the
  pointer's pull on held bodies, which released it as motion (investigation 1). Slow
  repair of deep overlaps left a ball buried for seconds when a pit closed over it.
  Scaling with depth handles both.

Two kinds of contact are treated differently:
- **Swept** contacts (from the tunnelling check) are solved fully at once, because
  the body has already gone too far.
- **Soft** contacts (from the overlap search) stop the approach exactly, then ease out
  only a fifth of the remaining overlap per substep.

### Sharing the load

Contacts are solved one at a time, and the first one solved would take the whole load,
leaving the others doing nothing. Its off-centre push would then twist the body.

So on the early passes, each contact solves only a share: `1 / (number of contacts −
pass)`. The last pass solves fully. With three passes and four contacts, the shares
are a quarter, a third, then everything that remains.

**Rule.** Do not let the first contact take everything. *Guarded by:* the `friction`
checks, which fail when blocks twist instead of gripping.

---

## 6. Friction

### Contacts form a patch

Friction is applied once per manifold, at the patch's centre: the average of its
contact points, weighted by how much load each carried. Applying friction at each
contact separately meant the first, off-centre contact did all the gripping, and a
block on a slope spun about its normal instead of holding. That made a block slide
on a 15° slope with friction 0.5, where it should hold up to about 26.6°.

`summarisePatch()` computes the centre, the average normal, the load, and the patch's
spread (how far its contacts are from the centre, used for twisting friction).

### The load that friction sees

Coulomb friction can resist up to `μ × load`. The obvious measure of load is the total
of the contacts' pushes. But that over-counts, for the same reason as section 5: a
patch that rocks has contacts pushing against each other, and each push adds to the
total without supporting anything. Measured, this inflated friction by about 30%.

So the load used for friction is the smaller of the total, and what was actually
carried: stopping the patch's approach, plus whatever other contacts are pressing
these two bodies together with.

### Static friction, as a position correction

`solvePatchFriction()` measures how far the patch's centre slipped sideways this
substep, and undoes it, if that takes no more than `μ × load`. If it would take more,
the patch is sliding, and static friction does nothing; sliding friction handles it in
the velocity pass.

**Rule.** Slip excludes repairs. If a repair shifts a body sideways and friction
resists it, friction turns the repair back into real speed, which is the same energy
leak as section 5 by another route.

### Sliding, twisting and rolling

In the velocity pass, `solveVelocity()`:

- **Sliding:** removes tangential velocity at the patch centre, up to `μ × load / h`.
- **Twisting:** resists spin about the normal, up to `μ × load × spread / h`. A flat
  patch resists twisting; a single point barely does.
- **Rolling:** resists rolling with a small constant torque (`rollingResistance`,
  0.012, as a lever arm, times the body's weight). Without it, balls rolled forever and
  piles never settled. It is constant, not proportional to load, because the load
  measured per contact was too noisy.

---

## 7. Bounce, and the velocity pass

After velocities are derived from positions, each contact that carried load sets its
normal velocity to bounce, or to zero:

```
new normal velocity = e × (arrival speed)   if the arrival was fast enough
                    = 0                     otherwise
```

`e` is the restitution: 0.2 by default, the lower of the two bodies' values.
"Fast enough" means faster than `4·g·h`, so that resting contacts, which arrive at
about `g·h` every substep, never bounce and can settle.

**Rule.** Judge the bounce by the speed the contact *arrived* with (`vnBefore`), not
the speed now. By the time this pass runs, the contact has already stopped the body,
so its current speed is about zero. Judging by that meant nothing ever bounced, a bug
present from the engine's first version until it was caught by a test. *Guarded by:*
`bouncy ball rebounds`.

This pass also removes any velocity separating the two bodies at a contact that
carried load. That follows the paper, and is part of why resting bodies stay still.

---

## 8. Sleeping and waking

A body falls asleep when its fastest point has moved slower than `sleepSpeed` (0.12)
for `sleepAfter` (0.6 s). Sleeping bodies skip integration, and **the solver treats
them as immovable**: `prepare()` gives them zero inverse mass. That makes them
excellent supports, and it is why the next rule matters.

**Rule.** Wake a sleeping body *before* a hit on it is solved. Bodies used to be woken
only at the end of a step, after the hit had already been solved against something
immovable. The hit lost all its momentum and the hitter bounced off, as if it had hit
a wall. Since most of a settled scene is asleep, nearly every hit felt dead.
`wakeOnImpact()` now wakes a sleeping body as soon as something approaches it faster
than `wakeImpact` (0.15). *Guarded by:* the `lively` checks with sleeping targets.

**Rule.** Wake only what a body actually touches. The old rule woke everything within
a moving body's bounding sphere. A long beam's sphere reaches across much of the tray,
so one tremor woke bodies it was not touching; they trembled as they woke, and woke
others. That was a feedback loop of its own.

Gentle approaches below `wakeImpact` do not wake anything. Without that limit,
resting neighbours would keep each other awake forever.

---

## 9. The pointer's pull

Dragging is a position correction towards a target, in `solveGrab()`, designed to
behave like a hand rather than like a rigid rod.

- **Force-limited.** Each substep, the grab point is pulled a fraction (`stiffness`,
  0.12) of the way to the target, but never with more force than `maxForce` (8) times
  the body's weight. A pull with no limit is effectively an infinite force: pressed
  into a wall, it overpowered the contact and set bodies spinning at over 60 rad/s.
  Capping it was the biggest single improvement to how dragging feels.
- **The limit uses normal gravity**, `PHYSICS.gravity`, not the world's. With zero
  gravity the hand would otherwise have no strength at all.
- **Damping happens after velocities are derived.** Damping applied before that is
  simply overwritten, which is exactly what an earlier version did: held bodies
  swung for over ten seconds after the pointer stopped.
- **The target is smoothed.** Pointers move in jumps; easing the target
  (`smoothing`, 0.35 per step) stops each jump becoming a kick.
- **The held body is searched every substep**, because it moves unpredictably.

A known weak spot: a contact's repair can stretch the pull, which then releases it as
motion. See investigation 1, the leaning beam.

---

## 10. The world, edits, and how rendering reads physics

The world is a body that never moves. Its distance function is `worldDist`:

```
worldDist = min(floor, edits applied to the scene's world shape)
```

`setEdits()` updates the edits and wakes every body, since anything may have lost its
support. Edits only change the world, never bodies, both here and in rendering. An
earlier version applied a pit to the whole rendered scene, including bodies, so a
ring that fell into a pit vanished on screen while physics had it exactly where it
should be.

### Rendering only reads

The renderer never changes physics. Each frame, `applySimulation()` copies every
body's rotation and position into shader joints, and the shader draws each body's
shape at its joint, via `SDF.withBodies()`. Two details matter:

- The shader needs the inverse rotation, to carry a point into the body's space. A
  rotation matrix's rows, passed where GLSL expects columns, form that inverse, so the
  rows of `R` are uploaded as they are.
- A small 3D texture of lower bounds on the distance to any body lets most ray steps
  skip checking bodies at all (`updateBodyBounds()`). Like raymarching, it relies on
  "nothing is closer than d".

---

## 11. Energy: what should happen to it

Most of the hardest bugs were energy appearing from nowhere. It helps to know what
should and should not happen.

**Should conserve:** a body falling or swinging freely, ignoring the small losses
from stepping. A held pendulum loses energy slowly, and that is correct.

**Should lose energy:** every collision (restitution below 1), friction, rolling and
twisting resistance, damping of a held body.

**May gain:** only from outside. The pointer pulling, or a pit opening or closing.

**Ways energy has leaked in before, all now fixed:**

| Leak | What it looked like |
|---|---|
| Repairs becoming speed | Piles never settled; a nudge set off hundreds of times its own energy |
| Friction resisting repairs | The same, more faintly |
| Waking everything within a radius | A tremor spreading across the scene |
| Fast repairs stretching the pointer's pull | A held beam wobbling forever |

**How to check.** Total energy, including height, must never rise above where it
started plus what was added from outside. Peak kinetic energy alone is not a test: a
nudge may rightly tip something over, and gravity then turns height into speed. Use
`EN.test()` in `tests/metrics.js`: it settles a cluster, nudges one body, and
measures both.

---

## 12. Approximations, all in one place

| Approximation | Effect |
|---|---|
| Surface samples, 32 or 64 per body | Thin parts can be missed; the overlap search catches most of it |
| Contacts re-read, not re-searched, between searches | Small fast changes can be found a little late |
| Patches cut to four points | Deliberate; more points destabilise |
| 2 mm allowance at rest | Bodies rest slightly sunk in |
| Inexact distances from blends and operations | Depths and normals are approximate |
| Gradient by finite differences | Normals blur slightly near sharp edges |
| Spin held fixed while bodies rotate | Lopsided bodies tumble slightly wrongly (investigation 3) |
| Constant rolling resistance | Rolling balls slow down a little, by design, so they stop |
| Rolling contact, overall | Down a 10° slope, about 2% off the textbook acceleration, measured with rolling resistance off; the cause is not known |
| Friction load estimate | Within a few percent of the true threshold |
| Sleeping bodies are immovable | Correct only because they are woken before hits |

---

## 13. Before you change anything

1. **Read the rule** in this document for the part you are changing, and the history
   behind it.
2. **Run the tests first**, so you know the starting point: `node tests/run.mjs`.
3. **Measure, then change one thing.** The fastest way to evaluate an idea is to patch
   it in the browser (change `PHYSICS`, or replace a method on
   `PhysicsWorld.prototype`), measure with `tests/metrics.js`, then restore. Every
   change in this engine was chosen that way, and several confident guesses turned out
   to be wrong.
4. **Check starting positions** before believing a bad result. Tests that start with
   bodies already overlapping look exactly like physics bugs.
5. **Use several starting variations** for anything involving settling. Settling is
   chaotic.
6. **Run the whole suite after.** Fixes in this engine have repeatedly broken
   something elsewhere, and only the full suite showed it: the energy fix broke
   stacks, extra overlap contacts broke stacks, and slow repairs broke pits.
