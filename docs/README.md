# Notes for whoever works on this next

This project is a signed distance field (SDF) engine written from scratch in plain
HTML, CSS and JavaScript, with no dependencies. It renders scenes by raymarching on
the GPU, and simulates rigid bodies on the CPU using nothing but the same distance
functions. There are no meshes anywhere.

Read these first, in this order:

1. [How the physics works](how-the-physics-works.md): a short orientation. The files,
   the core idea, and the traps that have caught people before.
2. [The physics engine, explained](physics-theory.md): the model and the reasoning
   behind every part of the solver, including the rules that are easy to break and
   the tests that guard them. **Read this before changing `physics.js`.**

Then pick an open problem from the list below.

## Open problems

Each document says what is known, how to reproduce the problem, what has already been
ruled out, ideas worth trying, and how to tell when it is fixed.

| # | Problem | Impact | Status in the tests |
|---|---|---|---|
| 1 | [A beam held leaning on the floor wobbles](investigations/01-leaning-beam-wobble.md) | Noticeable when dragging long bodies | `OPEN` check |
| 2 | [Piles do not always fall fully asleep](investigations/02-piles-not-always-sleeping.md) | Faint, occasional | `OPEN` check |
| 3 | [Lopsided bodies tumble slightly wrongly](investigations/03-tumbling-angular-momentum.md) | Subtle; accuracy, not stability | Not tested yet |
| 4 | [Very thin walls](investigations/04-very-thin-walls.md) | Rare: fast, small bodies and walls of 1 mm or less | Partly tested |
| 5 | [Thin parts tangling, and sharp edges](investigations/05-thin-and-sharp-contact.md) | Brief overlaps of a centimetre or so | Tested, passing |
| 6 | [The physics scene renders slowly while moving](investigations/06-physics-scene-render-speed.md) | Visible: the image goes soft | Not tested |

## Running things

- **The app:** open `index.html` in Chrome. There is no build step and no server.
- **The tests:** `node tests/run.mjs` runs about 160 checks in real headless Chrome
  and takes several minutes. It needs Node 22 or later, and Chrome. Set the `CHROME`
  environment variable if Chrome is not at the default macOS path.

Test results are marked `ok`, `FAIL` or `OPEN`. `OPEN` means a known problem, listed
above, that is shown rather than hidden. The run exits with an error only on `FAIL`.
