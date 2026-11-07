# 3. Lopsided bodies tumble slightly wrongly

## The problem

A body spinning freely in the air should keep its angular momentum exactly. For
lopsided bodies it does not. Over 5 seconds of free spin with no gravity:

| Body | Momentum change | Energy change |
|---|---|---|
| Beam | 31% | −1.9% |
| Ring | 21% | −1.1% |
| Block | 0.5% | −1.0% |

Energy goes down, not up, so this does not cause twitching. It means long, thin and
ring-shaped bodies tumble a little wrongly in the air. A thrown beam will not
precess the way a real one would.

## Reproduce

```js
const momentum = (b) => {
  const I = b.info.inertia.map((v) => (v * b.mass) / b.info.volume);
  return mulMat3(b.R, mulMat3(I, mulMat3T(b.R, b.w)));
};
const w = new PhysicsWorld({ floor: false, gravity: 0 });
const b = w.add(SDF.box(0.14, 0.14, 2.6), { at: [0, 0, 0], spin: [3, 0, 0.5] });
const L0 = momentum(b);
for (let f = 0; f < 300; f++) w.step(1 / 60);
const L = momentum(b);
Math.hypot(L[0] - L0[0], L[1] - L0[1], L[2] - L0[2]) / Math.hypot(...L0); // about 0.31
```

## Why it happens

Bodies rotate with their spin held fixed in the world, while their inertia turns with
them. For a lopsided body, the same angular momentum means a different spin as it
turns. The solver also rebuilds spin from the change in orientation at the end of each
substep.

## Something already tried

Carrying angular momentum through the rotation in the integration step was tried, and
had **no effect**, because the end of the substep rebuilds spin from orientation and
overwrites it. A fix has to happen at or after that point.

## Ideas worth trying

1. After spin is rebuilt from orientation, convert it to momentum using the inertia
   from the start of the substep, and back to spin using the inertia at the end.
2. Add the gyroscopic term implicitly, as described by Erin Catto (2015, *Numerical
   Methods*, GDC). Explicit versions are unstable for very lopsided bodies such as the
   beam.

## Done when

- Add a check to `tests/checks.js`: free spin keeps momentum within about 2% over
  5 seconds, for the beam, ring and block.
- Everything else still passes. Watch the `calm` and `energy` checks in particular,
  since this changes how every spinning body moves.
