# 2. Piles do not always fall fully asleep

## The problem

After the physics playground settles, one body sometimes never falls asleep. It
usually sits almost still, rocking or trembling very slightly, and keeps itself (and
sometimes a neighbour) awake. The test `every body falls asleep within 25 s` is marked
`OPEN`.

This is the faint remnant of a much larger problem that has been fixed. Read
[How the physics works](../how-the-physics-works.md) on repairs and sample density
before starting.

## It is chaotic

This matters more than anything else here. Moving every body's starting position by
5 to 10 cm flips the same settings between settling in about 7 seconds and never
settling. Results from the last investigation, for four starting shifts:

| Setting | Shift 0 | +0.05 | −0.07 | +0.11 |
|---|---|---|---|---|
| With the anchor | Never (anchor at 0.08 units/s) | Never (a ring, a beam) | 10.2 s | 8.9 s |
| Without the anchor | 6.9 s | 7.2 s | 8.5 s | 22.7 s |

**Never judge a change on one run.** Use several starting shifts and count how many
settle.

## Reproduce

```js
const run = (shift) => {
  const sc = SDF.scenes.physics.physics();
  const w = new PhysicsWorld({ world: sc.world.d });
  for (const { shape, ...o } of sc.bodies) w.add(shape, { ...o, at: [o.at[0] + shift, o.at[1], o.at[2]] });
  for (let f = 1; f <= 1800; f++) { w.step(1 / 60); if (w.awakeCount === 0) return f / 60; }
  return w.bodies.filter((b) => !b.sleeping).map((b) => b.pointSpeed.toFixed(2));
};
[0, 0.05, -0.07, 0.11].map(run);
```

## What is known

- Bodies stay awake because their speed occasionally spikes above `sleepSpeed` (0.12),
  which resets their sleep timer.
- Rings trembled much more before thin bodies got denser surface samples
  (`thinSpacing`). Rings are now usually fine.
- The anchor can keep rocking very slowly on its curved arms.
- Earlier, a body would sometimes lose contact for about one frame, drop roughly a
  millimetre, and be caught. Keeping contacts across tiny gaps ("speculative
  contacts") was tried, did not help the playground, and was removed.

## Ideas worth trying

1. **Sleep on a smoothed speed** instead of the instant speed, so one brief spike does
   not reset the timer. Check it does not make bodies freeze mid-fall.
2. **Sleep groups of touching bodies together**, as many engines do. A pile sleeps or
   wakes as one unit.
3. **Find what causes the spikes** on the bodies that stay awake. Log contacts and
   speeds per substep for the stuck body. The tools in `tests/metrics.js` help.

## Done when

- The `OPEN` check passes, and the pile settles in at least four of four starting
  shifts.
- The `energy` and `lively` checks still pass: bodies must still react properly when
  hit, and must not freeze too early.
