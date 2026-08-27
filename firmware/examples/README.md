# Example sketches

The teaching sequence, in order. Each one adds a single idea, and
`00-start-here.c` explains the order and what hardware each needs.

## Why these are in the repo

The hub keeps its working sketch library in
`%LOCALAPPDATA%\tt02-auto\sketches`, which is the right place for it — it is
per-user scratch space, it does not want to be in git, and it should not need a
checkout to exist.

But that meant these files existed in exactly one place, on one machine, with no
backup and no history. They are hours of written explanation, not scratch, and
losing them to a reinstall would be losing real work.

So the canonical copies live here and the library is the working copy.

## The thing that made this necessary

`firmware/src/sketch.c` is a **scratch slot**, not a home. The hub mirrors
whichever library sketch is open into it, because that is the file `CMakeLists.txt`
compiles for the `sketch` target — so opening `00-start-here.c` and pressing
Build & Flash replaces whatever was in the slot, by design.

That is fine for scratch and fatal for anything you meant to keep. Work that
matters goes in a named file here; the slot is only ever the thing currently
being flashed.

## Editing

Open one in the hub's Code view and press Build & Flash. Changing a number and
watching what breaks teaches more than reading does, which is what the whole
sequence is arranged around.

If you change one here rather than in the library, copy it across — nothing
syncs these two directories automatically, and a silent sync would be worse
than none, because it would overwrite edits without asking which copy won.
