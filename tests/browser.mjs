// Opens index.html in real headless Chrome, so tests run on the same GPU, shaders
// and JavaScript as the page. No dependencies beyond Node 22 and Chrome.
import { spawn } from 'node:child_process';
import { mkdtempSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, dirname } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const CHROME = process.env.CHROME || '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome';
const root = dirname(dirname(fileURLToPath(import.meta.url)));
export const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

export async function openPage({ port = 9400, timeout = 600000 } = {}) {
  const profile = mkdtempSync(join(tmpdir(), 'sdf-tests-'));
  const chrome = spawn(CHROME, ['--headless=new', `--remote-debugging-port=${port}`, `--user-data-dir=${profile}`,
    '--use-angle=metal', '--ignore-gpu-blocklist', '--window-size=1440,900', '--force-device-scale-factor=2', 'about:blank'], { stdio: 'ignore' });
  // Resolves once Chrome has exited and its profile is deleted. Await it before
  // process.exit(), or the profile (tens of MB) is left in the temp folder.
  const exited = new Promise((r) => chrome.once('exit', r));
  const close = async () => {
    chrome.kill();
    await Promise.race([exited, sleep(3000)]);
    rmSync(profile, { recursive: true, force: true, maxRetries: 10, retryDelay: 200 });
  };
  const watchdog = setTimeout(async () => { console.error('Timed out.'); await close(); process.exit(3); }, timeout);
  let tabs;
  for (let i = 0; i < 60 && !tabs; i++) {
    try { tabs = await (await fetch(`http://127.0.0.1:${port}/json`)).json(); } catch { await sleep(250); }
  }
  if (!tabs) throw new Error(`Could not start Chrome at ${CHROME}. Set the CHROME environment variable.`);
  const ws = new WebSocket(tabs.find((t) => t.type === 'page').webSocketDebuggerUrl);
  await new Promise((r) => { ws.onopen = r; });
  let id = 0;
  const pending = new Map();
  const errors = [];
  ws.onmessage = (e) => {
    const m = JSON.parse(e.data);
    if (m.id && pending.has(m.id)) { pending.get(m.id)(m); pending.delete(m.id); }
    if (m.method === 'Runtime.consoleAPICalled' && m.params.type === 'error') errors.push(m.params.args.map((a) => a.value).join(' ').slice(0, 500));
    if (m.method === 'Runtime.exceptionThrown') errors.push(JSON.stringify(m.params.exceptionDetails).slice(0, 500));
  };
  const send = (method, params = {}) => new Promise((r) => { const i = ++id; pending.set(i, r); ws.send(JSON.stringify({ id: i, method, params })); });
  const ev = async (code) => {
    const r = await send('Runtime.evaluate', { expression: code, returnByValue: true, awaitPromise: true });
    if (r.result.exceptionDetails) throw new Error(r.result.exceptionDetails.exception?.description || JSON.stringify(r.result.exceptionDetails).slice(0, 500));
    return r.result.result.value;
  };
  const until = async (code, ms = 60000) => { const t = Date.now(); while (Date.now() - t < ms) { if (await ev(code)) return Date.now() - t; await sleep(100); } return -1; };
  await send('Runtime.enable');
  await send('Page.enable');
  await send('Page.navigate', { url: pathToFileURL(join(root, 'index.html')).href });
  await until('!!window.viewer && !!viewer.live', 30000);
  return { ev, send, until, errors, close: () => { clearTimeout(watchdog); ws.close(); return close(); } };
}
