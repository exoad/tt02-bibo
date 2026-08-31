import { readdirSync, statSync, existsSync } from 'node:fs'
import { join } from 'node:path'

/* Every content route, enumerated from the files sync.mjs just wrote.
 *
 * NOT crawlLinks alone. The first build of this site used the crawler, said
 * "Prerendered 6 routes", exited 0 and produced a .output/public holding
 * 200.html and 404.html - no index, no API pages, nothing. A crawler finds
 * what something links to, and the landing page's links are components the
 * crawler does not follow, so the whole site was invisible to it while the
 * build reported success.
 *
 * The generator knows exactly which pages exist because it wrote them. Reading
 * the directory cannot silently find fewer than there are. */
function contentRoutes(dir = join(__dirname, 'content'), prefix = '') {
  if (!existsSync(dir)) return []
  const routes = []
  for (const name of readdirSync(dir)) {
    const p = join(dir, name)
    if (statSync(p).isDirectory()) {
      routes.push(...contentRoutes(p, `${prefix}/${name.replace(/^\d+\./, '')}`))
    }
    else if (name.endsWith('.md')) {
      const slug = name.replace(/^\d+\./, '').replace(/\.md$/, '')
      routes.push(slug === 'index' ? (prefix || '/') : `${prefix}/${slug}`)
    }
  }
  return routes
}

export default defineNuxtConfig({
  extends: ['docus'],

  app: {
    head: { title: 'bibo firmware' },
  },

  // Straight into the documentation. There is no content/index.md, so without
  // this `/` has nothing behind it - Docus's landing collection sources that
  // one file and it no longer exists.
  routeRules: {
    '/': { redirect: { to: '/getting-started', statusCode: 302 } },
  },

  nitro: {
    prerender: {
      crawlLinks: true,
      routes: contentRoutes(),
    },
  },

  compatibilityDate: '2026-08-31',
})
