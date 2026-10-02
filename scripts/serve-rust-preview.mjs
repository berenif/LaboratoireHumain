import { createServer } from 'node:http';
import { readFile, stat } from 'node:fs/promises';
import { extname, resolve, sep } from 'node:path';

const root = resolve(process.argv[2] ?? 'rust/dist/worker');
const port = Number(process.argv[3] ?? 5184);
if (!Number.isInteger(port) || (port !== 0 && port < 1024) || port > 65535) throw new Error('Invalid preview port');
const mount = process.argv[4] ?? '/';
if (!/^\/(?:[A-Za-z0-9_-]+\/)*$/.test(mount)) throw new Error('Preview mount must be an absolute directory path ending with /');
const types = { '.html': 'text/html', '.js': 'text/javascript', '.wasm': 'application/wasm', '.json': 'application/json', '.css': 'text/css', '.png': 'image/png', '.svg': 'image/svg+xml' };
const server = createServer(async (request, response) => {
  try {
    const pathname = decodeURIComponent(new URL(request.url, `http://127.0.0.1:${port}`).pathname);
    if (mount !== '/' && pathname === mount.slice(0, -1)) { response.writeHead(308, { Location: mount }).end(); return; }
    if (!pathname.startsWith(mount)) { response.writeHead(404).end(); return; }
    const local = pathname.slice(mount.length);
    const file = resolve(root, `./${local || 'index.html'}`);
    if (!file.startsWith(root + sep)) { response.writeHead(403).end(); return; }
    if (!local) {
      try { await stat(file); }
      catch { response.writeHead(200, { 'Content-Type': 'text/html' }).end('<!doctype html><title>Rust worker verification</title>'); return; }
    }
    if (!(await stat(file)).isFile()) { response.writeHead(404).end(); return; }
    response.writeHead(200, { 'Content-Type': types[extname(file)] ?? 'application/octet-stream', 'Cache-Control': 'no-store' });
    response.end(await readFile(file));
  } catch { response.writeHead(404).end(); }
});
server.listen(port, '127.0.0.1', () => console.log(`Rust preview: http://127.0.0.1:${server.address().port}${mount}`));
