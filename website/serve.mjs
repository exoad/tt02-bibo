/*
 * A static server for the generated docs, with no dependencies. Nuxt writes
 * absolute asset URLs - `/_nuxt/entry.css` - which under file:// resolve to the
 * root of C:, so double-clicking index.html gets unstyled text and a dead
 * sidebar. Flattening the routes so relative URLs work costs the sidebar its
 * grouping, which Docus builds from the directory structure.
 */
import { createServer } from 'node:http'
import { readFile, stat } from 'node:fs/promises'
import { join, extname, normalize } from 'node:path'
import { fileURLToPath } from 'node:url'
import { dirname } from 'node:path'

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '.output', 'public')

const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.txt': 'text/plain; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.webp': 'image/webp',
  '.ico': 'image/x-icon',
  '.woff': 'font/woff',
  '.woff2': 'font/woff2',
}

async function resolve(urlPath) {
  /* normalize() first, so a request for /../../secrets cannot leave ROOT */
  const rel = normalize(decodeURIComponent(urlPath.split('?')[0])).replace(/^([/\\])+/, '')
  const base = join(ROOT, rel)
  if (!base.startsWith(ROOT)) return null

  for (const candidate of [base, `${base}.html`, join(base, 'index.html')]) {
    try {
      if ((await stat(candidate)).isFile()) return candidate
    }
    catch { /* try the next spelling */ }
  }
  return null
}

const server = createServer(async (req, res) => {
  const file = (await resolve(req.url || '/')) || join(ROOT, '404.html')
  try {
    const body = await readFile(file)
    res.writeHead(file.endsWith('404.html') ? 404 : 200, {
      'content-type': TYPES[extname(file)] || 'application/octet-stream',
      'cache-control': 'no-cache',
    })
    res.end(body)
  }
  catch {
    res.writeHead(404, { 'content-type': 'text/plain' })
    res.end('not found')
  }
})

try {
  await stat(join(ROOT, 'index.html'))
}
catch {
  console.error('No build found. Run `npm run generate` in website/ first.')
  process.exit(1)
}

/* A busy port means a previous copy already serves this folder - the desired end
 * state, so pressing the hub's Docs button twice must not raise an error. */
server.on('error', (err) => {
  if (err.code === 'EADDRINUSE') {
    console.log(`already serving on ${port}`)
    process.exit(0)
  }
  throw err
})

const port = Number(process.argv[2] || 4173)
server.listen(port, '127.0.0.1', () => {
  console.log(`http://127.0.0.1:${server.address().port}/`)
})
