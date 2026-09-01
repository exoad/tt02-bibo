/*
 * Is the built site older than the firmware it documents?
 *
 * Exits 0 when a rebuild is needed, 1 when the build is current - batch-file
 * convention, where `if errorlevel 1` reads as "not stale".
 *
 * WHY THIS EXISTS. docs.bat used to rebuild only when .output was MISSING, so
 * the first press after editing a header served the previous version of the
 * reference and gave no sign of it. A documentation site that silently shows
 * yesterday's signatures is worse than one that is missing, because the
 * missing one sends you to the header.
 *
 * Compares modification times rather than hashing: the generator reads these
 * files and nothing else, so a newer mtime on any of them is exactly the
 * condition under which its output could differ.
 */
import { statSync, readdirSync, existsSync } from 'node:fs'
import { join, dirname } from 'node:path'
import { fileURLToPath } from 'node:url'

const HERE = dirname(fileURLToPath(import.meta.url))
const REPO = join(HERE, '..')
const BUILT = join(HERE, '.output', 'public', 'index.html')

if (!existsSync(BUILT)) {
  console.log('no build')
  process.exit(0)
}

const builtAt = statSync(BUILT).mtimeMs

/* Everything sync.mjs reads: the library headers, and the two files it copies
 * in whole. Miss one and the site goes stale without saying so. */
function newest(dir) {
  let t = 0
  if (!existsSync(dir)) return t
  for (const name of readdirSync(dir)) {
    const p = join(dir, name)
    const s = statSync(p)
    t = Math.max(t, s.isDirectory() ? newest(p) : s.mtimeMs)
  }
  return t
}

const sources = Math.max(
  newest(join(REPO, 'firmware', 'lib')),
  statSync(join(REPO, 'firmware', 'README.md')).mtimeMs,
  statSync(join(HERE, 'sync.mjs')).mtimeMs,
)

if (sources > builtAt) {
  console.log('stale')
  process.exit(0)
}

console.log('current')
process.exit(1)
