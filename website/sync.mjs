/*
 * Generate the documentation site's content from firmware/ itself.
 *
 * WHY GENERATED RATHER THAN WRITTEN. A hand-written API page is correct on the
 * day it is written and wrong from the next commit onwards, silently - which is
 * this project's characteristic failure. Every signature and every prose block
 * on an API page here is read out of the header at build time, so a page cannot
 * describe a function that no longer exists.
 *
 * THE INPUT IS THE COMMENT STYLE THE FIRMWARE ALREADY USES. There are no
 * Doxygen tags in this codebase and none are being introduced: a declaration is
 * documented by the block comment directly above it, which is how every header
 * here is already written. The OUTPUT is Doxygen-shaped - signature, then prose,
 * grouped by kind - the input is just prose.
 *
 * Run by `npm run dev`, `build` and `generate`, so the content cannot be stale
 * relative to the firmware in the same tree.
 */
import { readFileSync, writeFileSync, mkdirSync, rmSync, readdirSync, statSync } from 'node:fs'
import { join, relative, basename, dirname } from 'node:path'
import { fileURLToPath } from 'node:url'

const HERE = dirname(fileURLToPath(import.meta.url))
const REPO = join(HERE, '..')
const LIB = join(REPO, 'firmware', 'lib')
const OUT = join(HERE, 'content')

/* ---- reading the headers ------------------------------------------------- */

function headers(dir) {
  const found = []
  for (const name of readdirSync(dir)) {
    const p = join(dir, name)
    if (statSync(p).isDirectory()) found.push(...headers(p))
    else if (name.endsWith('.hxx')) found.push(p)
  }
  return found
}

/* A block comment, stripped of its frame, kept as prose.
 *
 * The frame varies across these headers - some blocks are ruled off with
 * dashes or equals signs, some are plain - so the ruler lines go and the rest
 * is kept verbatim. Nothing here tries to interpret the prose. */
function unframe(lines) {
  return lines
    .map(l => l.replace(/^\s*\/\*+/, '').replace(/\*+\/\s*$/, '').replace(/^\s*\*\s?/, ''))
    .filter(l => !/^\s*[-=]{4,}\s*$/.test(l))
    .join('\n')
    .replace(/\n{3,}/g, '\n\n')
    .trim()
}

const DECL = [
  /* Order matters: a function whose return type is `struct X` must not be read
   * as a struct declaration, so the more specific patterns come first. */
  { kind: 'function',
    re: /^\s*(?:static\s+|inline\s+|constexpr\s+)*(?:\[\[nodiscard\]\]\s*)?([A-Za-z_][\w:]*\s*[*&]?)\s+([A-Za-z_]\w*)\s*\(([^;{]*)\)\s*(?:noexcept)?\s*[{;]?\s*$/,
    take: m => ({ name: m[2], sig: `${m[1].trim()} ${m[2]}(${m[3].trim()})` }) },
  { kind: 'enum',
    re: /^\s*enum(?:\s+class)?\s+([A-Za-z_]\w*)/,
    take: m => ({ name: m[1], sig: `enum ${m[1]}` }) },
  { kind: 'struct',
    re: /^\s*(?:struct|class)\s+([A-Za-z_]\w*)\s*(?:$|\{|:)/,
    take: m => ({ name: m[1], sig: `struct ${m[1]}` }) },
  { kind: 'constant',
    re: /^\s*(?:static\s+)?constexpr\s+([A-Za-z_][\w:]*)\s+([A-Za-z_]\w*)\s*(?:=|\{)/,
    take: m => ({ name: m[2], sig: `constexpr ${m[1]} ${m[2]}` }) },
  { kind: 'type',
    re: /^\s*using\s+([A-Za-z_]\w*)\s*=\s*([^;]+);/,
    take: m => ({ name: m[1], sig: `using ${m[1]} = ${m[2].trim()};` }) },
]

/* #define is handled apart from the list above because it is NOT SCOPED.
 *
 * The others are only legal inside a namespace, so they are looked for only
 * there. A preprocessor directive is conventionally written at column 0 even
 * when it sits among namespaced code, which put every constant in cal.hxx -
 * the whole calibration table - outside the depth guard and reported the file
 * as having nothing in it. */
const DEFINE = {
  kind: 'constant',
  re: /^\s*#define\s+([A-Z][A-Z0-9_]*)\s+(.+?)\s*(?:\/\*.*)?$/,
  take: m => ({ name: m[1], sig: `#define ${m[1]} ${m[2].trim()}` }),
}

function parse(path) {
  const src = readFileSync(path, 'utf8').replace(/\r\n/g, '\n').split('\n')
  const out = { banner: '', ns: [], decls: [] }

  let pending = null      // the comment block most recently closed
  let buf = null          // the comment block being read
  let ns = []             // namespace stack, by name
  let depth = 0
  let seenPragma = false

  for (let i = 0; i < src.length; i++) {
    const line = src[i]

    /* comments first - a brace inside one is not structure */
    if (buf !== null) {
      buf.push(line)
      if (line.includes('*/')) { pending = unframe(buf); buf = null }
      continue
    }
    if (/^\s*\/\*/.test(line) && !line.includes('*/')) { buf = [line]; continue }
    if (/^\s*\/\*.*\*\/\s*$/.test(line)) { pending = unframe([line]); continue }

    if (/^\s*#pragma\s+once/.test(line)) {
      /* everything before the guard is the file's own banner */
      if (pending && !out.banner) { out.banner = pending; pending = null }
      seenPragma = true
      continue
    }

    const nsm = line.match(/^\s*namespace\s+([A-Za-z_]\w*)/)
    if (nsm) { ns.push({ name: nsm[1], at: depth }); pending = null; continue }

    /* declarations, only where a declaration can be: inside a namespace, and
     * not nested inside a struct or function body. depth is 1 for the outer
     * `bibo` brace and 2 inside a sub-namespace. */
    const dm = seenPragma ? line.match(DEFINE.re) : null
    if (dm) {
      out.decls.push({ ...DEFINE.take(dm), kind: DEFINE.kind, doc: pending || '',
                       ns: ns.map(x => x.name).join('::'), line: i + 1 })
      pending = null
    }
    /* `depth <= ns.length` alone is the guard. Requiring depth > 0 as well
     * assumed every declaration lives in a namespace, and types.hxx - the file
     * defining Int32, Bool and Void, the vocabulary the whole firmware is
     * written in - declares them at FILE SCOPE. It reported as empty. */
    else if (seenPragma && depth <= ns.length) {
      for (const d of DECL) {
        const m = line.match(d.re)
        if (m) {
          const got = d.take(m)
          if (!/^(if|for|while|switch|return|else|do|sizeof)$/.test(got.name)) {
            out.decls.push({ ...got, kind: d.kind, doc: pending || '',
                             ns: ns.map(x => x.name).join('::'), line: i + 1 })
          }
          break
        }
      }
      pending = null
    }

    for (const ch of line) {
      if (ch === '{') depth++
      else if (ch === '}') {
        depth--
        while (ns.length && ns[ns.length - 1].at >= depth) ns.pop()
      }
    }
    if (/^\s*$/.test(line)) pending = pending   // blank keeps the comment attached
    else if (!/^\s*[#}]/.test(line) && buf === null && !nsm) { /* consumed above */ }
  }

  out.ns = [...new Set(out.decls.map(d => d.ns))].filter(Boolean)
  return out
}

/* ---- writing the pages --------------------------------------------------- */

const esc = s => s.replace(/\{\{/g, '&#123;&#123;')   // Vue would eat these

function apiPage(path, mod, order) {
  const p = parse(path)
  const rel = relative(REPO, path).replace(/\\/g, '/')
  const groups = [
    ['Types', p.decls.filter(d => ['struct', 'enum', 'type'].includes(d.kind))],
    ['Constants', p.decls.filter(d => d.kind === 'constant')],
    ['Functions', p.decls.filter(d => d.kind === 'function')],
  ]

  let md = `---\ntitle: ${mod}\ndescription: ${rel}\nnavigation:\n  title: ${mod}\n---\n\n`
  md += `# ${mod}\n\n`
  md += `::note\nGenerated from \`${rel}\`. Edit the header, not this page.\n::\n\n`
  if (p.banner) md += `${esc(p.banner)}\n\n`

  for (const [label, items] of groups) {
    if (!items.length) continue
    md += `## ${label}\n\n`
    for (const d of items) {
      md += `### ${d.name}\n\n`
      md += '```cpp\n' + d.sig + '\n```\n\n'
      if (d.ns) md += `*Namespace* \`${d.ns}\` · *line ${d.line}*\n\n`
      if (d.doc) md += `${esc(d.doc)}\n\n`
    }
  }
  return { md, count: p.decls.length, banner: p.banner, mod }
}

/* ---- run ----------------------------------------------------------------- */

rmSync(join(OUT, '3.api'), { recursive: true, force: true })
mkdirSync(join(OUT, '3.api'), { recursive: true })

const files = headers(LIB).sort()
let total = 0
const summaries = []

files.forEach((f, i) => {
  const mod = basename(f, '.hxx')
  const page = apiPage(f, mod, i)
  writeFileSync(join(OUT, '3.api', `${String(i + 1)}.${mod}.md`), page.md)
  total += page.count
  summaries.push(page)
})

/* the subsystems page is the headers' own banners, which is where the
 * explanation of each subsystem already lives */
let sub = `---\ntitle: Subsystems\ndescription: What each part of the firmware is responsible for\n---\n\n# Subsystems\n\n`
sub += `The firmware is ${files.length} headers under \`firmware/lib\`. Each one states what it owns and, more usefully, what it deliberately does not.\n\n`
for (const s of summaries) {
  const first = (s.banner || '').split('\n\n')[0].trim()
  if (!first) continue
  sub += `## ${s.mod}\n\n${esc(first)}\n\n[Reference &rarr;](/api/${s.mod})\n\n`
}
writeFileSync(join(OUT, '2.subsystems.md'), sub)

/* Getting started is firmware/README.md, not a retelling of it.
 *
 * Two copies of a toolchain instruction is one copy and one lie - and the
 * README is the one a person finds first when they are standing in the repo,
 * so it stays the original. Its H1 goes because the frontmatter supplies it. */
const readme = readFileSync(join(REPO, 'firmware', 'README.md'), 'utf8')
  .replace(/\r\n/g, '\n')
  .replace(/^#\s+.*\n/, '')
  .trim()
writeFileSync(join(OUT, '1.getting-started.md'),
  `---\ntitle: Getting started\ndescription: Toolchain, build and flash for the Pico 2 W\n---\n\n`
  + `# Getting started\n\n`
  + `::note\nGenerated from \`firmware/README.md\`.\n::\n\n${esc(readme)}\n`)

writeFileSync(join(OUT, 'index.md'),
  `---\ntitle: bibo firmware\ndescription: API reference and subsystem guide for the TT-02 firmware\nnavigation: false\n---\n\n`
  + `# bibo firmware\n\n`
  + `The firmware for a self-driving 1/10 RC car on a Tamiya TT-02, running on an RP2350 (Pico 2 W).\n\n`
  + `Every page under **Reference** is generated from the headers in \`firmware/lib\` at build time, so a signature here cannot drift from the one that compiles. ${files.length} headers, ${total} declarations.\n\n`
  + `::card-group\n`
  + `  ::card\n  ---\n  title: Getting started\n  to: /getting-started\n  icon: i-lucide-rocket\n  ---\n  Toolchain, SDK, build and flash.\n  ::\n`
  + `  ::card\n  ---\n  title: Subsystems\n  to: /subsystems\n  icon: i-lucide-layers\n  ---\n  What each part owns, and what it deliberately does not.\n  ::\n`
  + `  ::card\n  ---\n  title: Reference\n  to: /api/hal\n  icon: i-lucide-book-open\n  ---\n  Generated API for every header.\n  ::\n`
  + `::\n`)

console.log(`${files.length} header(s), ${total} declaration(s)`)
for (const s of summaries) console.log(`  ${String(s.count).padStart(4)}  ${s.mod}`)
