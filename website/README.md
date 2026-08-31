# website — the firmware documentation

A [Docus](https://docus.dev) site. Getting started, the subsystems, and an API
reference for every header in `firmware/lib`.

```bat
npm install
npm run generate
node serve.mjs
```

Or press **Docs** in the hub, which runs `docs.bat` and does all three.

## The API pages are generated, not written

`sync.mjs` reads `firmware/lib/**/*.hxx` and writes `content/` before every
build. A page cannot describe a function that no longer exists, because the
signature on it was read out of the header seconds earlier.

There are no Doxygen tags in the firmware and none are being added. A
declaration is documented by the block comment above it, which is how these
headers were already written; the generator turns that into signature-plus-prose
grouped by kind. The *output* is Doxygen-shaped, the *input* is prose.

`content/` is gitignored for the same reason — committing it would commit a
second copy of the headers, and the second copy is the one that goes stale.

Getting started is `firmware/README.md`, copied in at build time rather than
retold. Two copies of a toolchain instruction is one copy and one lie.

## Why it is served rather than opened

Nuxt writes absolute asset URLs (`/_nuxt/entry.css`). Opened as `file://` those
resolve against the root of `C:` and load nothing, so a double-clicked
`index.html` is unstyled text with a dead sidebar. `serve.mjs` is a
dependency-free static server; `docs.bat` starts it and opens a browser, and
exits quietly if one is already listening.

## Notes

`.npmrc` sets `legacy-peer-deps`. npm 10.9.0 fails on Nuxt's dependency graph
with `Cannot read properties of null (reading 'edgesOut')` — a resolver bug, not
a network problem: `npm i is-odd` succeeds in the same shell and `npm i nuxt`
does not.

Docus is pinned to `~5.12.3`. 5.13.0 depends on `ai@7.0.87`, which was never
published — the registry stops at 7.0.86, so that release cannot install.
