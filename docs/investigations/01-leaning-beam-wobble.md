# 1. A beam held leaning on the floor wobbles

## The problem

Hold a long beam by one end so that its other end rests on the floor, like a ladder
held at the top. It keeps wobbling and never settles. A cube held the same way, on one
edge, is perfectly still.

The test suite measures this as `beam held leaning on the floor, wobble`. It is
currently about **0.15**, against a target of **below 0.05**, and is marked `OPEN`.

## Reproduce

In the browser console on `index.html`:

```js
const w = new PhysicsWorld();
const b = w.add(SDF.box(0.14, 0.14, 2.6), { at: [1, 2, 0] });
w.grabBody(b, b.x.map((v, i) => v + [0, 0.07, 1.25][i]));
for (let f = 0; f < 300; f++) w.step(1 / 60);
let sum = 0;
for (let f = 0; f < 600; f++) { w.step(1 / 60); sum += b.pointSpeed ** 2; }
Math.sqrt(sum / 600); // about 0.15. A cube in the same setup gives about 0.003.
```

## What is known

The mechanism is understood, and measured:

1. A contact repairs a small overlap at the beam's foot. A repair moves the body
   without giving it speed, which is correct on its own.
2. But the beam is also held by the pointer, so the repair stretches the pointer's
   pull.
3. The pull releases that stretch as real motion, swinging the beam so its foot digs
   in again.
4. Repeat.

Evidence:

- Turning repairs off entirely stops the wobble: **0.95 to 0.004** (measured before the
  current gentle repairs).
- The wobble scales with repair speed. That is why repairs are now gentle for shallow
  overlaps (`repairSpeed: 0.5`, `repairPerDepth: 4`), which cut it from 0.95 to about
  0.15.
- In this setup, the biggest energy gains happen in substeps with **no contacts at
  all**, which is the stretched pull releasing.

## Ruled out

Each of these was changed on its own and made little or no difference:

- Pull stiffness, pull force limit, and drag damping.
- Friction (off, and high), and bounce.
- Contact sample density, searching every substep, and the overlap search.
- The allowance (`slop`) of 1 mm or 4 mm.
- A held pendulum **in the air** loses energy, as it should. The pull alone is fine.

## Ideas worth trying

1. **Stop repairs from stretching the pull.** For example, when a repair moves the
   held body, move the pull's target point by the same amount, or measure the pull's
   stretch from the body's position before repairs.
2. **Treat the pull as a proper constraint** in the same solver pass as the contacts
   (a soft distance constraint with compliance, XPBD-style), so the two are solved
   together instead of fighting.
3. **Damp the held body's motion relative to the pointer**, not in absolute terms.

## Done when

- The `OPEN` check passes: wobble below 0.05.
- These still pass: `closing the pit pushes it back out` (deep repairs still work),
  the `calm` drag checks, and the `throws` checks (dragging must not get sluggish).
