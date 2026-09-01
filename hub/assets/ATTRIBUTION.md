# Third-party assets in `hub/`

## Kenney Car Kit 3.1 - `assets/models/car.obj`

Created and distributed by [Kenney](https://kenney.nl/assets/car-kit).

Licensed **[CC0 1.0](http://creativecommons.org/publicdomain/zero/1.0/)** - public
domain. Personal, educational and commercial use are all permitted and
**attribution is not required**. It is here anyway, because knowing where a file
came from is worth more than the license obliges.

The pack's own license text ships verbatim as `assets/models/LICENSE-kenney.txt`.

### What is used

One model of the fifty in the kit: `sedan-sports.obj`, renamed to `car.obj`
(2,088 triangles), plus the kit's shared `Textures/colormap.png`. Nothing else is
vendored - no other vehicles - so the whole 3D car is one 122 KB text file and
one 12 KB PNG.

**The texture is MODIFIED.** CC0 permits it, and this records it. `colormap.png`
is a 512x512 palette atlas: each triangle's UV points at a solid swatch rather
than at a painted panel, so there is no "repaint the bonnet" - there is only
"change the swatch the bonnet samples". `tools/livery.py` does that: it reads
`car.obj`, samples the atlas at the `body` group's UVs, picks the most common
**chromatic** swatch (the grays that dominate are the underbody, which is never
seen), and remaps it to Subaru World Rally Team blue, preserving each texel's
luminance so the palette's baked shading survives. 38,624 texels changed from
`#EE6445` to `#143C96`.

Re-runnable and idempotent: it refuses to repaint a swatch that is already the
livery color.

**It is scaled per axis to the TT-02's real 430 x 190 x 135 mm, not uniformly.**
That is a deliberate ~15% distortion of the width. The corridor, the Fit erosion
and the flat map's footprint all derive from those measured numbers, and a car
drawn wider than the corridor beside it would be a picture contradicting the
measurement it sits next to. See `loadCarObj()` in `src/scene3d.cpp`.

### What it is not

A generic sports saloon in rally blue. It is **not** an Impreza mesh and not a
TT-02 - a stand-in with the right footprint and the right color, nothing more.

The closest properly-licensed low-poly Impreza WRC found was **"Subaru Impreza
WRC - Super Rally 3D" by Aeroux on Sketchfab** - 368 triangles, CC Attribution,
marked downloadable:

  https://sketchfab.com/3d-models/subaru-impreza-wrc-super-rally-3d-cc754e3b1a42456a82ba8dbb9251c843

Sketchfab returns **401** on its download endpoint without a logged-in account,
so it could not be fetched here. Dropping its OBJ + texture into this directory
as `car.obj` / `colormap.png` is all the loader needs; nothing in the code is
specific to the Kenney mesh beyond the `body` / `wheel-*` / `spoiler` group names
used for the untextured fallback.

Note also that a model of a real, trademarked car carries design and trademark
questions that are separate from whatever copyright license the uploader chose.
For a personal project that is not a practical concern; it would be one if this
were ever distributed.

## Fugue Icons 3.5.6 — `assets/icons/*.png`

© 2013 [Yusuke Kamiyamane](https://p.yusukekamiyamane.com/). All rights reserved.

Licensed under a [Creative Commons Attribution 3.0 License](http://creativecommons.org/licenses/by/3.0/).

**Attribution is a condition of the license, not a courtesy.** It has to appear
somewhere a user of the built application can find it. It is currently in:

- this file
- `assets/icons/LICENSE.txt` (the pack's own README, shipped verbatim)
- the repo `README.md`

If the app ever grows an About box, it belongs there too. The alternative the
author offers is buying a royalty-free license, which removes the requirement.

### What is used

**68 of the pack's 3,570 icons are vendored** in `assets/icons/`, at their
native 16x16 and unmodified. 49 of those are wired to a name in
`src/icons.cpp`; the remaining 19 are kept because they have been used before
or are the obvious candidate for something not built yet.

A few files back more than one name - `car` is both the Vehicle section and the
3D Fit overlay, `dashboard` is both Full overlays - which is why the two counts
differ by more than the unused list.

They are drawn at **integer** multiples of 16 physical pixels. See
`src/icons.hpp` for why: this is pixel art hinted at one size, and a fractional
DPI scale turns it to mush.

The full pack is not vendored. `vendor/fugue-icons-3.5.6.zip` is gitignored;
re-download it from the link above if more icons are needed and extract into
`assets/icons/` with the upstream file names kept as they are, so the source art
stays findable.

Vendored, in Fugue's own spelling:

`application-wave`, `arrow-circle`, `arrow-circle-135-left`, `arrow-transition`, `asterisk`, `battery`, `binocular`, `blue-document`, `broom`, `bug`, `burn-small`, `car`, `card`, `chart-pie`, `chart-up`, `clock`, `clock-history`, `compass`, `control`, `control-record`, `control-stop`, `counter`, `cross`, `dashboard`, `disk`, `document`, `door-open`, `drive-download`, `equalizer`, `eraser`, `exclamation`, `eye`, `gear`, `grid-dot`, `hammer`, `information`, `layer-shape-polyline`, `light-bulb`, `lightning`, `map-pin`, `memory`, `monitor`, `network`, `node-design`, `plug`, `plug-connect`, `plug-disconnect`, `processor`, `radar`, `ruler`, `server`, `shield`, `spectrum`, `status`, `status-away`, `status-busy`, `status-offline`, `switch`, `system-monitor`, `table`, `target`, `terminal`, `thermometer`, `tick`, `usb-flash-drive`, `wall`, `wrench`, `wrench-screwdriver`

### A note on the bonus icons

The pack also ships 24 px and 32 px variants, but only for ~260 and ~79 icons
respectively, and none of the ones this app uses. That is why everything here is
16 px: the larger sizes were checked for and are not available for these names.

---

## The application icon — `assets/bibo.ico`

No Fugue pixels are in it. Generated by `assets/make_icon.ps1` from primitives —
rounded rectangles, two gradients and a bevel — in the app's own visual
language. Fugue is 16×16 only; an application icon needs 256.

**Its composition, though, follows Fugue's `car.png`**, which is the icon the
hub already shows for anything to do with the vehicle: a silver cabin over a
blue body, a dark grille with a badge, amber lamps either side, tyres beneath.
That was deliberate — the app icon and the car icon in the UI should be the same
car. Said plainly here rather than left implied, because "drawn from primitives"
describes how the file was made and not what it is a picture of. The pack it
echoes is already attributed above and in `THIRD_PARTY.md`, and the built
binary carries that attribution in `LegalCopyright`.

It was a radar sweep until 2026-09-01. The radar is the sensor; the car is the
thing.

Re-run the script after a theme change so the icon and the UI do not drift:

```powershell
powershell -ExecutionPolicy Bypass -File hub\assets\make_icon.ps1
```

It writes DIB entries up to 64 px and PNG at 128/256. That split is deliberate:
PNG entries are legal at every size since Vista, but GDI+ cannot decode them, so
an all-PNG `.ico` is unreadable to a lot of tooling even though Explorer shows it.
