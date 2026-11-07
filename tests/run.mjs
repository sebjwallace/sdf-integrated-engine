// Runs every proof in real Chrome: node tests/run.mjs
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { openPage, sleep } from './browser.mjs';
import { transformChecks } from './transform.mjs';

const here = dirname(fileURLToPath(import.meta.url));
const page = await openPage();
const { ev, send, until, errors } = page;
for (const file of ['metrics.js', 'checks.js']) await ev(`${readFileSync(join(here, file), 'utf8')}\n;1`);
const { results, measurements } = await ev('runChecks()');

// CPU and GPU distances must agree, on every scene that has a baked grid.
for (const scene of ['car', 'valve', 'mug', 'sculpture']) {
  await ev(`location.hash='${scene}';1`);
  await sleep(200);
  await until('viewer.usingField', 60000);
  const max = await ev(`(()=>{const f=viewer.field,gl=f.gl,tree=SDF.scenes.${scene}.build(),[w,h,dz]=f.dims;gl.bindFramebuffer(gl.FRAMEBUFFER,f.fbo);let max=0;
    for(let z=0;z<dz;z+=Math.ceil(dz/8)){gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,f.base,0,z);const a=new Float32Array(w*h*4);gl.readPixels(0,0,w,h,gl.RGBA,gl.FLOAT,a);
    for(let j=0;j<h;j+=7)for(let i=0;i<w;i+=7){const g=a[(j*w+i)*4];const d=tree.d(f.lo[0]+(i+.5)*f.voxel,f.lo[1]+(j+.5)*f.voxel,f.lo[2]+(z+.5)*f.voxel);max=Math.max(max,Math.abs(g-d))}}viewer.restart(false);return max})()`);
  results.push({ group: 'cpu vs gpu', name: `${scene}: largest difference`, value: +max.toFixed(5), pass: max < 0.002, want: '< 0.002' });
}

// The real page, with real mouse events.
await ev(`location.hash='physics';1`);
await until('!!viewer.sim', 30000);
await sleep(300);
const mouse = (type, x, y, o = {}) => send('Input.dispatchMouseEvent', { type, x, y, button: 'left', buttons: type === 'mouseReleased' ? 0 : 1, clickCount: 1, ...o });
const toScreen = (p) => ev(`(()=>{const v=viewer;const {eye,forward,right,up}=v.basis(v.cam);const d=[${p}].map((c,i)=>c-eye[i]);const z=d[0]*forward[0]+d[1]*forward[1]+d[2]*forward[2];const x=(d[0]*right[0]+d[1]*right[1]+d[2]*right[2])/z*3.2,y=(d[0]*up[0]+d[1]*up[1]+d[2]*up[2])/z*3.2;const h=v.canvas.clientHeight,w=v.canvas.clientWidth;return [(x*h+w)/2,(h-y*h)/2]})()`);
const calmed = await until('viewer.sim.awakeCount<=8', 45000);
results.push({ group: 'page', name: 'playground mostly settles', value: await ev('viewer.sim.awakeCount'), pass: calmed >= 0, want: '8 or fewer awake' });
// Choose a sleeping body, so that the one we aim at is still there when we press.
const index = await ev('viewer.sim.bodies.findIndex(b=>b.sleeping&&b.x[1]>.4&&b.x[1]<1.6&&b.info.radius<1&&Math.abs(b.x[0])<3&&Math.abs(b.x[2])<3)');
const start = await ev(`viewer.sim.bodies[${index}].x.slice()`);
const [sx, sy] = await toScreen(start);
await mouse('mouseMoved', sx, sy, { buttons: 0 });
await mouse('mousePressed', sx, sy);
results.push({ group: 'page', name: 'mouse picks up a body', value: await ev('!!viewer.bodyGrab'), pass: await ev('!!viewer.bodyGrab'), want: 'true' });
await ev('window.__d=[];window.__on=1;(()=>{let l=performance.now();const f=t=>{if(!__on)return;__d.push(t-l);l=t;requestAnimationFrame(f)};requestAnimationFrame(f)})();1');
for (let i = 1; i <= 40; i++) { await mouse('mouseMoved', sx + i * 3, sy - i * 6); await sleep(16); }
const lifted = await ev(`viewer.sim.grab ? viewer.sim.grab.body.x[1]-(${start[1]}) : 0`);
results.push({ group: 'page', name: 'dragging lifts it', value: +lifted.toFixed(2), pass: lifted > 1, want: '> 1' });
const frames = await ev('(()=>{__on=0;const a=__d.slice(3).sort((x,y)=>x-y);return {median:+a[a.length>>1].toFixed(1),p95:+a[Math.floor(a.length*.95)].toFixed(1)}})()');
await mouse('mouseReleased', sx + 120, sy - 240);
results.push({ group: 'page', name: 'letting go releases it', value: await ev('viewer.sim.grab===null'), pass: await ev('viewer.sim.grab===null'), want: 'true' });
for (const scene of ['car', 'robot', 'mug']) {
  await ev(`location.hash='${scene}';1`);
  await sleep(300);
  const ok = await ev('document.getElementById("error").hidden');
  results.push({ group: 'page', name: `${scene} loads`, value: ok, pass: ok, want: 'true' });
}
results.push(...await transformChecks(page));
results.push({ group: 'page', name: 'no console errors', value: errors.length, pass: errors.length === 0, want: '0' });

let group = '';
for (const r of results) {
  if (r.group !== group) { group = r.group; console.log(`\n${group}`); }
  const mark = r.pass ? 'ok  ' : r.open ? 'OPEN' : 'FAIL';
  console.log(`  ${mark} ${r.name.padEnd(48)} ${String(r.value).padEnd(10)} ${r.pass ? '' : `want ${r.want}`}`);
}
const failed = results.filter((r) => !r.pass && !r.open);
const open = results.filter((r) => !r.pass && r.open);
console.log(`\n${results.length - failed.length - open.length} of ${results.length} passed, ${failed.length} failed, ${open.length} known and open. Frames while dragging: median ${frames.median} ms, 95th percentile ${frames.p95} ms.`);
console.log(`Measurements: ${JSON.stringify({ throws: measurements.throws, lift: measurements.lift, pile: measurements.pile })}`);
if (errors.length) console.log(errors.join('\n'));
await page.close();
process.exit(failed.length ? 1 : 0);
