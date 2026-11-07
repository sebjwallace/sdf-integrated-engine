# 4. Very thin walls

## The problem

Fast, small bodies against walls of **1 mm or less** are stopped at the wall but end
up slightly past its middle, on the wrong side. Walls of 2 mm and more hold. Measured
with the current engine; the wall's centre is at x = 3:

| Ball | Speed | Wall | Ends at | Right side? |
|---|---|---|---|---|
| r = 0.2 | 40 | 5 cm | 2.73 | Yes |
| r = 0.05 | 40 | 5 mm | 2.95 | Yes |
| r = 0.03 | 40 | 2 mm | 2.97 | Yes |
| r = 0.02 | 40 | 1 mm | 3.01 | **No** |
| r = 0.05 | 40 | 0.5 mm | 3.00 | **No**, roughly at the middle |

Earlier versions of the engine also let 2 mm walls through. Later changes to repairs
fixed that, which suggests repairs are involved.

Without the sweep described below, small fast balls go straight through to x = 40, so
the sweep is clearly doing its job. The problem is only what happens right at the
wall.

## Reproduce

```js
const w = new PhysicsWorld({ world: SDF.box(0.001, 10, 10).move(3, 0, 0).d, floor: false, gravity: 0 });
const b = w.add(SDF.sphere(0.02), { at: [0, 0, 0], velocity: [40, 0, 0], restitution: 0 });
for (let i = 0; i < 60; i++) w.step(1 / 60);
b.x[0]; // about 3.01: just past the middle, the wrong side
```

## What is known

- Fast points are swept: if a point moved further than the distances at both ends of
  its path allow, the path is marched to find the crossing (`collide` and `sweep` in
  `physics.js`).
- The test `tiny ball is at least stopped at a 2 mm wall` passes with a deliberately
  loose limit. It checks the ball is stopped near the wall, not that it stays on the
  correct side.
- A likely cause: once a point is inside a wall thinner than the allowance (2 mm), the
  gradient points to whichever face is nearer, which can be the far one. The repair
  then pushes it out the wrong side.

## Ideas worth trying

1. Remember which side of a surface a swept point came from, and push it back out that
   side rather than following the gradient.
2. For contacts found by the sweep, use the approach direction instead of the gradient
   at the point.
3. For very thin walls only, shrink the allowance.

## Done when

- Small, fast balls end on the correct side of walls of 0.5 mm and 1 mm, and still
  do for 2 mm.
- Tighten the existing test to check the side, not just closeness.
- Stacks still stand, since they rely on the allowance.
