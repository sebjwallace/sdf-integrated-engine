# 6. The physics scene renders slowly while moving

## The problem

While bodies move in the physics playground, the renderer drops to about **20 to 25%
resolution** on a 2880×1626 screen to keep its frame rate, so the image looks soft.
It sharpens once everything stops. Measured on an Apple M1. Faster GPUs will do
better, but the scene is still the slowest in the project.

Physics itself is not the bottleneck: it takes about 4 to 12 ms per step on average.

## What is known

- The physics scene is rendered in Live mode: the full scene is evaluated at every ray
  step. Static scenes can use a baked 3D texture instead, which is many times faster
  (see `field.js`), but moving bodies cannot be baked into it every frame.
- Every ray step used to check every body's bounding sphere. A small 3D texture of
  lower bounds (`updateBodyBounds` in `renderer.js`) now lets most steps skip them all.
  That made frames about 2.6 times faster when it was added.
- Currently, with 32 bodies settled, one preview frame takes about **78 ms** at
  1440×813 and **17 ms** at 576×325 (median of six). The very first frame after the
  scene loads is far slower, over a second, while the GPU prepares the shader, so
  never measure just one.
- The world itself (tray, hills, peg, arch) is evaluated live too, even though it never
  moves unless the user digs a pit.

## Ideas worth trying

1. **Bake the static world, keep bodies live.** Ray steps would read the world from the
   baked texture (as the other scenes do), and evaluate only nearby bodies. Edits
   already update the baked texture in small regions, so pits still work.
2. **Bake each body's shape once, in its own space.** Every ring is the same ring. A
   small 3D texture per shape, sampled in the body's local frame, would replace
   evaluating the shape's tree. Thin parts may need a fine texture.
3. **Render at a sharper resolution when only part of the screen changes**, for
   example when a single body is being dragged.

## How to measure

The tests report frame times while dragging (`Frames while dragging` at the end of
`node tests/run.mjs`). For a direct measurement, time a draw in the page and force it
to finish by reading one pixel back:

```js
const v = viewer, gl = v.gl, px = new Float32Array(4), times = [];
gl.bindFramebuffer(gl.FRAMEBUFFER, v.fbo);
gl.viewport(0, 0, 1440, 813);
for (let i = 0; i < 6; i++) {
  const t = performance.now();
  v.draw(false, 1440, 813, [0, 0]);
  gl.readPixels(0, 0, 1, 1, gl.RGBA, gl.FLOAT, px);
  times.push(performance.now() - t);
}
times.sort((a, b) => a - b)[3]; // milliseconds for a typical preview frame
```

## Done when

- The playground stays at 50% resolution or more while moving, on the same machine.
- The image is unchanged once everything stops: compare screenshots before and after.
