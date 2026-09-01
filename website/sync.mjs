/*
 * Generate the documentation site's content from firmware/ itself.
 *
 * A hand-written API page is correct the day it is written and silently wrong
 * from the next commit on, so every signature and prose block here is read out of
 * the header at build time - from the block comment above a declaration. Run by
 * `npm run dev`, `build` and `generate`, so the content cannot go stale.
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

/* A block comment, stripped of its frame, kept as prose. Ruler lines of dashes
 * or equals signs go; the rest is verbatim and nothing here interprets it. */
function unframe(lines) {
  return lines
    .map(l => l.replace(/^\s*\/\*+/, '').replace(/\*+\/\s*$/, '').replace(/^\s*\*\s?/, ''))
    .filter(l => !/^\s*[-=]{4,}\s*$/.test(l))
    .join('\n')
    .replace(/\n{3,}/g, '\n\n')
    .trim()
}

/* Split a doc comment into its Doxygen tags and the prose around them. BOTH
 * SHAPES HAVE TO WORK: the firmware is converted to `@brief`/`@param` a header
 * at a time, and untagged text becomes the description, so a file nobody has
 * touched still renders instead of blanking until the conversion finishes. */
function doxygen(text) {
  const out = { brief: '', body: [], params: [], returns: '', notes: [], warns: [] }
  if (!text) return out

  let cur = null                    // the tag currently collecting continuation lines

  for (const raw of text.split('\n')) {
    const m = raw.match(/^\s*[@\\](brief|param|return|returns|note|warning|see)\b\s*(.*)$/)
    if (m) {
      const [, tag, rest] = m
      if (tag === 'brief') { out.brief = rest; cur = { push: s => (out.brief += ' ' + s) } }
      else if (tag === 'param') {
        /* `@param name description`; the name may carry an [in]/[out] prefix. */
        const pm = rest.match(/^(\[[^\]]*\]\s*)?(\S+)\s*(.*)$/)
        const p = { name: pm ? pm[2] : rest, dir: (pm && pm[1] || '').trim(), text: pm ? pm[3] : '' }
        out.params.push(p)
        cur = { push: s => (p.text += ' ' + s) }
      }
      else if (tag === 'return' || tag === 'returns') {
        out.returns = rest
        cur = { push: s => (out.returns += ' ' + s) }
      }
      else if (tag === 'note') { out.notes.push(rest); cur = { push: s => (out.notes[out.notes.length - 1] += ' ' + s) } }
      else if (tag === 'warning') { out.warns.push(rest); cur = { push: s => (out.warns[out.warns.length - 1] += ' ' + s) } }
      else { cur = null }
      continue
    }

    /* A blank line ends a tag; anything else continues the one in progress. */
    if (!raw.trim()) { cur = null; out.body.push(''); continue }
    if (cur) { cur.push(raw.trim()); continue }
    out.body.push(raw)
  }

  out.body = out.body.join('\n').replace(/\n{3,}/g, '\n\n').trim()
  return out
}

const DECL = [
  /* Order matters: `struct X f()` must not be read as a struct declaration. */
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

/* #define is handled apart from the list above because it is NOT SCOPED: a
 * directive sits at column 0 even among namespaced code, which put cal.hxx's
 * whole calibration table outside the depth guard and reported it as empty. */
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

    /* `[\w:]*` so a CONCATENATED namespace keeps all of itself. Matching only
     * the first segment of `namespace bibo::lights` made every reference page
     * claim its declarations lived in plain `bibo` - generated, and wrong. */
    const nsm = line.match(/^\s*namespace\s+([A-Za-z_][\w:]*)/)
    if (nsm) { ns.push({ name: nsm[1], at: depth }); pending = null; continue }

    /* declarations only where one can be: inside a namespace, not nested in a
     * struct or function body. depth is 1 for the outer `bibo` brace. */
    const dm = seenPragma ? line.match(DEFINE.re) : null
    if (dm) {
      out.decls.push({ ...DEFINE.take(dm), kind: DEFINE.kind, doc: pending || '',
                       ns: ns.map(x => x.name).join('::'), line: i + 1 })
      pending = null
    }
    /* `depth <= ns.length` alone is the guard. Also requiring depth > 0 assumed
     * every declaration lives in a namespace, and shared.hxx - Int32, Bool, Void
     * - declares them at FILE SCOPE, so it reported as empty. */
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

/* Vue would eat a `{{`. Coerces first: `@param name` with no description is legal. */
const esc = s => String(s ?? '').replace(/\{\{/g, '&#123;&#123;')

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
      const dx = doxygen(d.doc)

      md += `### ${d.name}\n\n`
      if (dx.brief) md += `${esc(dx.brief)}\n\n`
      md += '```cpp\n' + d.sig + '\n```\n\n'
      if (d.ns) md += `*Namespace* \`${d.ns}\` · *line ${d.line}*\n\n`

      if (dx.body) md += `${esc(dx.body)}\n\n`

      if (dx.params.length) {
        md += '| Parameter | Description |\n|---|---|\n'
        for (const p of dx.params) {
          const nm = p.dir ? `\`${p.name}\` *${p.dir}*` : `\`${p.name}\``
          md += `| ${nm} | ${esc(p.text.trim()).replace(/\n/g, ' ')} |\n`
        }
        md += '\n'
      }

      if (dx.returns) md += `**Returns** — ${esc(dx.returns.trim())}\n\n`

      /* Docus renders these as coloured callouts, which is the point. */
      for (const n of dx.notes) md += `::note\n${esc(n.trim())}\n::\n\n`
      for (const w of dx.warns) md += `::warning\n${esc(w.trim())}\n::\n\n`
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

/* the subsystems page is the headers' own banners, where each already explains itself */
let sub = `---\ntitle: Subsystems\ndescription: What each part of the firmware is responsible for\n---\n\n# Subsystems\n\n`
sub += `The firmware is ${files.length} headers under \`firmware/lib\`. Each one states what it owns and, more usefully, what it deliberately does not.\n\n`
for (const s of summaries) {
  const first = (s.banner || '').split('\n\n')[0].trim()
  if (!first) continue
  sub += `## ${s.mod}\n\n${esc(first)}\n\n[Reference &rarr;](/api/${s.mod})\n\n`
}
writeFileSync(join(OUT, '2.subsystems.md'), sub)

/* Getting started is firmware/README.md itself, not a retelling - two copies of a
 * toolchain instruction is one copy and one lie. Its H1 goes; frontmatter has it. */
const readme = readFileSync(join(REPO, 'firmware', 'README.md'), 'utf8')
  .replace(/\r\n/g, '\n')
  .replace(/^#\s+.*\n/, '')
  .trim()
writeFileSync(join(OUT, '1.getting-started.md'),
  `---\ntitle: Getting started\ndescription: Toolchain, build and flash for the Pico 2 W\n---\n\n`
  + `# Getting started\n\n`
  + `::note\nGenerated from \`firmware/README.md\`.\n::\n\n${esc(readme)}\n`)

/* NO LANDING PAGE. `/` redirects to the first page of the docs - see the route
 * rule in nuxt.config.ts. */

console.log(`${files.length} header(s), ${total} declaration(s)`)
for (const s of summaries) console.log(`  ${String(s.count).padStart(4)}  ${s.mod}`)
