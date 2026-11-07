// Checks for moving, turning and scaling objects with the gizmo, using real
// mouse events. Runs from run.mjs, or alone: node tests/transform.mjs
import { pathToFileURL } from 'node:url';
import { openPage, sleep } from './browser.mjs';

export async function transformChecks({ ev, send, until }) {
  const results = [];
  const check = (name, value, pass, want) => results.push({ group: 'transform', name, value, pass: !!pass, want });
  const mouse = (type, x, y, o = {}) => send('Input.dispatchMouseEvent', { type, x, y, button: 'left', buttons: type === 'mouseReleased' ? 0 : 1, clickCount: 1, ...o });
  // Drags the middle of a gizmo handle by `px` pixels along it on screen.
  const dragAxis = async (axis, px, { release = true } = {}) => {
    const h = await ev(`(viewer.gizmo.layout(), viewer.gizmo.handles.find(h=>h.kind==='axis'&&h.axis===${axis}))`);
    if (!h) return false;
    const l = Math.hypot(h.to[0] - h.from[0], h.to[1] - h.from[1]);
    const u = [(h.to[0] - h.from[0]) / l, (h.to[1] - h.from[1]) / l];
    const m = [(h.from[0] + h.to[0]) / 2, (h.from[1] + h.to[1]) / 2];
    await mouse('mouseMoved', m[0], m[1], { buttons: 0 });
    await mouse('mousePressed', m[0], m[1]);
    for (let i = 1; i <= 10; i++) { await mouse('mouseMoved', m[0] + (u[0] * px * i) / 10, m[1] + (u[1] * px * i) / 10); await sleep(16); }
    if (release) await mouse('mouseReleased', m[0] + u[0] * px, m[1] + u[1] * px);
    return [m[0] + u[0] * px, m[1] + u[1] * px];
  };
  // Largest gap between the baked grid and the CPU distance, as run.mjs measures it.
  const gridError = () => ev(`(()=>{const f=viewer.field,gl=f.gl,tree=viewer.tree,[w,h,dz]=f.dims;gl.bindFramebuffer(gl.FRAMEBUFFER,f.fbo);let max=0;
    for(let z=0;z<dz;z+=Math.ceil(dz/8)){gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,f.base,0,z);const a=new Float32Array(w*h*4);gl.readPixels(0,0,w,h,gl.RGBA,gl.FLOAT,a);
    for(let j=0;j<h;j+=7)for(let i=0;i<w;i+=7){const g=a[(j*w+i)*4];const d=tree.d(f.lo[0]+(i+.5)*f.voxel,f.lo[1]+(j+.5)*f.voxel,f.lo[2]+(z+.5)*f.voxel);max=Math.max(max,Math.abs(g-d))}}viewer.restart(false);return max})()`);

  // A baked scene: move, rotate and scale the mug, then compare the grid with the CPU.
  await ev(`location.hash='mug';1`);
  await sleep(300);
  await until('viewer.usingField', 60000);
  const mug = await ev(`viewer.objects.indexOf('Mug')`);
  await ev(`viewer.select(${mug});viewer.setGizmo('move');1`);
  await sleep(100);
  await dragAxis(0, 60);
  const moved = await ev(`viewer.states[${mug}].t[0]`);
  check('mug: dragging the X arrow moves it along x', +moved.toFixed(3), moved > 0.2, '> 0.2');
  check('mug: and only along x', await ev(`Math.hypot(viewer.states[${mug}].t[1],viewer.states[${mug}].t[2])`), await ev(`Math.hypot(viewer.states[${mug}].t[1],viewer.states[${mug}].t[2])<1e-9`), '0');
  await ev(`viewer.setGizmo('rotate');1`);
  await ev(`viewer.beginTransform(${mug});viewer.updateTransform({rotate:{axis:[0,0,1],angle:.6}});viewer.endTransform(false);1`);
  await ev(`viewer.setGizmo('scale');1`);
  await sleep(100);
  await dragAxis(1, 40);
  const stretch = await ev(`viewer.states[${mug}].S[1]`);
  check('mug: dragging a scale handle stretches it', +stretch.toFixed(3), stretch > 1.1, '> 1.1');
  const ready = await until('viewer.usingField', 60000);
  check('mug: the grid is baked again', ready >= 0, ready >= 0, 'true');
  const error = await gridError();
  check('mug: moved grid matches the CPU', +error.toFixed(5), error < 0.002, '< 0.002');

  // Escape puts things back while dragging.
  await ev(`viewer.setGizmo('move');1`);
  await sleep(100);
  const before = await ev(`JSON.stringify(viewer.states[${mug}].t)`);
  const end = await dragAxis(2, 50, { release: false });
  await send('Input.dispatchKeyEvent', { type: 'keyDown', key: 'Escape', code: 'Escape', windowsVirtualKeyCode: 27 });
  await mouse('mouseReleased', end[0], end[1]);
  const restored = before === await ev(`JSON.stringify(viewer.states[${mug}].t)`);
  check('escape cancels a drag', restored, restored && await ev(`viewer.selected===${mug}`), 'true');

  // The car's parts are mirrored: both sides must move together.
  await ev(`location.hash='car';1`);
  await sleep(300);
  await until('viewer.usingField', 60000);
  const wheels = await ev(`viewer.objects.indexOf('Wheels')`);
  await ev(`viewer.select(${wheels});viewer.beginTransform(${wheels});viewer.updateTransform({move:[0,0,.5]});viewer.endTransform(false);1`);
  const box = await ev(`viewer.objectBounds(${wheels})`);
  const shift = (box.min[2] + box.max[2]) / 2;
  check('car: wheels move as one, both sides', +shift.toFixed(2), Math.abs(shift - 0.5) < 0.1, '0.5');

  // Physics: no gizmo while running. Paused, a body can be moved, and plays on from there.
  await ev(`location.hash='physics';1`);
  await until('!!viewer.sim', 30000);
  await sleep(500);
  const block = await ev(`viewer.objects.indexOf('Block 1')`);
  await ev(`viewer.select(${block});viewer.setPaused(false);1`);
  check('physics: no gizmo while running', await ev('viewer.gizmo.layout()'), !(await ev('viewer.gizmo.layout()')), 'false');
  await ev(`viewer.setPaused(true);1`);
  const body = `viewer.sim.bodies[viewer.bodyOfObject.get(${block})]`;
  const y0 = await ev(`${body}.x[1]`);
  await dragAxis(1, 120);
  const lift = (await ev(`${body}.x[1]`)) - y0;
  check('physics: paused, the Y arrow lifts a body', +lift.toFixed(2), lift > 0.5, '> 0.5');
  await ev(`${body}.v=[0,0,0];1`);
  const vol0 = await ev(`${body}.info.volume`);
  await ev(`viewer.beginTransform(${block});viewer.updateTransform({scale:[2,1,1]});viewer.endTransform(false);1`);
  const ratio = (await ev(`${body}.info.volume`)) / vol0;
  check('physics: stretching a body doubles its volume', +ratio.toFixed(3), Math.abs(ratio - 2) < 0.05, '2');
  const yLifted = await ev(`${body}.x[1]`);
  await sleep(500);
  check('physics: nothing moves while paused', +((await ev(`${body}.x[1]`)) - yLifted).toFixed(4), (await ev(`${body}.x[1]`)) === yLifted, '0');
  await ev(`viewer.setPaused(false);1`);
  // Headless Chrome draws slowly, and physics takes at most 3 steps a frame, so wait for it.
  await until(`${yLifted}-${body}.x[1]>0.3`, 15000);
  const fell = yLifted - (await ev(`${body}.x[1]`));
  check('physics: playing again, it falls', +fell.toFixed(2), fell > 0.3, '> 0.3');
  // A world part: moving it changes what bodies collide with.
  await ev(`viewer.setPaused(true);1`);
  const ramp = await ev(`viewer.objects.indexOf('Ramp')`);
  const inside = await ev(`(viewer.objectState(${ramp}), viewer.sim.worldDist(...viewer.states[${ramp}].pivot))`);
  await ev(`viewer.beginTransform(${ramp});viewer.updateTransform({move:[0,2,0]});viewer.endTransform(false);1`);
  const after = await ev(`viewer.sim.worldDist(...viewer.states[${ramp}].pivot)`);
  check('physics: a moved ramp moves for the physics too', +after.toFixed(2), inside < 0 && after > 0.3, '> 0.3');
  await ev(`viewer.setPaused(false);1`);
  return results;
}

if (import.meta.url === pathToFileURL(process.argv[1]).href) {
  const page = await openPage({ port: 9412 });
  const results = await transformChecks(page);
  for (const r of results) console.log(`  ${r.pass ? 'ok  ' : 'FAIL'} ${r.name.padEnd(48)} ${String(r.value).padEnd(10)} ${r.pass ? '' : `want ${r.want}`}`);
  if (page.errors.length) console.log(page.errors.join('\n'));
  await page.close();
  process.exit(results.every((r) => r.pass) && !page.errors.length ? 0 : 1);
}
