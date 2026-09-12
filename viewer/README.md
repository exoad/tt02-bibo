# viewer

The bibo viewer: one 3D view of the car and what the lidar can see, with a few
plain Dear ImGui panels floating over it. Win32 + D3D11, vanilla ImGui v1.92.9
from `third_party/imgui`. No editor, no language server, no reference browser —
the code is written elsewhere. It does not link the RPLIDAR SDK and never will:
the board owns the lidar and the viewer only ever receives scans.

## Build

    viewer\build.bat            incremental; `build.bat clean` wipes build\

MSVC x64, `/W4`, warning-clean. Produces `viewer\build\bibo.exe`.

The build also compiles `firmware/pilot/src/bibowire.cxx` — **the same source
the board's pilot compiles**, not a copy. `docs/bibowire.md` §11 requires it:
one object file means the encoder and the decoder cannot drift into disagreeing
about a field's offset while both still compile.

## Run

    viewer\build\bibo.exe

Left-drag orbits, right-drag pans, the wheel zooms, `R` resets the camera.

Type a host in the Connection panel and press Connect. The default is
`bibobox.local:8020` — a **name**, because the field network is a phone hotspot
whose DHCP hands out a different address every outing, and the client resolves
it again on every attempt for the same reason.

## The link

`src/link.hxx` / `link.cxx` is a bibowire client on its own thread. The frame
loop never waits on a socket; it takes a copy of the newest decoded state under
one lock, newest-wins.

- **Handshake.** `HELLO` is the first bytes on the connection and nothing else
  is sent until `WELCOME`.
- **Reconnect.** 250 ms, 500 ms, 1 s, 2 s, 4 s, then 4 s forever, each with
  ±20 % jitter, never giving up. It redials when no frame of *any* type has
  arrived in 3000 ms.
- **`bootId`.** A different one means the pilot restarted, and the viewer
  clears scan, decision, board state and the `revIndex` baseline before drawing
  a single point.
- **Staleness.** `Session::revolution()` and its siblings take the clock and
  return an empty `Opt` when the answer is too old — so there is no value to
  draw rather than a flag somebody has to remember to check. Past the feed's
  **stale band** the cloud is drawn desaturated with its age in the panel; past
  1500 ms (`GONE_MS`) it is not drawn at all.
- **The stale band is measured, not a constant.** `FRESH_MS = 400` is right for
  a feed arriving *faster* than it and wrong for one arriving slower, which is
  how a 2 fps camera came to read STALE on **57 frames out of 57** while
  delivering exactly what it promised. Each feed now keeps its own `Cadence` —
  the widest gap in a sliding window of its last 64 arrivals — and is stale past
  1.5× that, floored at `FRESH_MS` so measuring can never *tighten* §7's number
  and capped at `STALE_CEIL_MS = 1200` so stale stays a band every feed passes
  **through** on its way to `GONE_MS` rather than one it can skip. The scan had
  the same disease from the other end: its mean interval is ~103 ms, but this
  board goes quiet for ~400 ms every few seconds, which put the old threshold
  exactly on the feed's own jitter.
- **Latency.** The viewer sends its own `PING` at 1 Hz and times the `PONG` by
  its echoed token, so the round trip in the Connection panel is measured from
  *this* machine. It keeps the last 16 and shows the **minimum** as well as the
  current one — on a hotspot the mean measures the worst moment of the last
  sixteen seconds rather than the path — and half that minimum is the one-way
  delay used to age the picture through the board's clock.
- **Two different lies, two mechanisms.** The round trip is the network's
  number. The board's *own* ages — `scanAgeMs`, `picoSilentMs`, `controlAgeMs` —
  sit beside the values they describe, because a link with an 8 ms round trip
  can be carrying a two-second-old picture, and a healthy link must never make
  a dead sensor look alive.

## Tests

    viewer\tests\build_link_test.bat run

Exercises the decode path against hand-built frames with no board attached:
the scan-to-scene conversion, byte-at-a-time reassembly, resync, an unknown
type, the `bootId` reset, the `DECIDE`-to-`SCAN` tie, the staleness bands and
the backoff schedule. What it does **not** cover — sockets, the connect
deadline, the reconnect loop itself — is listed at the top of `test_link.cxx`
rather than left to be assumed.

## The camera

Its own floating window, opened from the **camera** checkbox in the View panel
and closed by its own title-bar X. `CAMERA` (0x20) carries a JPEG; `stb_image`
decodes it and it is uploaded to a D3D11 texture, recreated only when the
dimensions change rather than every frame.

- **The window being open *is* the subscription.** Opening it sends
  `SUBSCRIBE` with the camera bit set; closing it sends one without, and
  releases the texture. The board only opens `/dev/video0` while somebody is
  subscribed, and the stream is roughly **1 MB/s at 640×480** — against
  §10's assumption of ~200 KB/s and its verdict that even *that* does not fit
  alongside the scan on this hotspot. A camera window nobody is looking at
  costs nothing, exactly as the scan already does.
- **The bit is `tag - 0x10`, so `CAMERA` is bit 16.** That convention is
  settled in `firmware/pilot/src/viewfeed.cxx`, not here. The naive
  `1u << (tag & 0x1F)` is a known bug — `DECIDE` and `SCHEMA` collide on it.
- **An unsubscribe is never a zero mask**, because a zero mask means
  *everything* to the board. Turning the camera off names every other type.
- **Staleness follows the rate the camera is actually delivering.** Past the
  band it earned (see *The stale band is measured* above) the picture is drawn
  desaturated with its age; past 1500 ms it is **not drawn at all** and the
  window says why on an empty background. A photograph looks equally
  convincing whether it was taken now or forty seconds ago. The window shows
  the band and the cadence it came from, because "STALE" with no number beside
  it is precisely the claim that used to flicker twice a second.
- **The viewer asks for the frame rate.** The board's own default is 2 fps —
  honest for a phone hotspot, a slideshow on a LAN — and only the viewer knows
  which it is on. The **rate** slider sends frames-per-second in what §5 of
  `docs/bibowire.md` reserved as `SUBSCRIBE.reserved0`: the body is still 12
  bytes, an older board ignores it exactly as it always did, and 0 means "did
  not ask" so the board's default stands. The board clamps to
  `bibowire::CAM_FPS_MAX = 15`. Measured on the bench: **0 → 1.8 fps, 10 → 7.1,
  15 → 12.2**.
- **Asking for more costs the scan.** Measured against the real board, the
  camera at 10 fps stretched the worst SCAN gap from 402 ms to ~1420 ms. CAMERA
  is `CLASS_BULK` so pictures are *dropped* before scans, but the board-side
  capture and encode still compete with the lidar. 10 is a reasonable default
  on a LAN; on the hotspot, turn it down.
- **Rotate and flip.** 0/90/180/270 and the two mirrors, for a camera bolted to
  the car sideways. `ImGui::Image` **cannot** express a quarter turn — `uv0`/
  `uv1` are opposite corners, which can mirror an axis but never transpose one
  onto the other — so the picture is drawn with `AddImageQuad` and four
  independent corners, and the fit swaps width for height at 90 and 270. The
  corner arithmetic lives in `src/orient.cxx`, which names no ImGui or D3D type
  precisely so `viewer/tests` can hold it to an answer.
- **The overlays do not follow rotate and flip.** Crosshair, reversing guides,
  centre box and thirds are placed against the picture rectangle as drawn:
  right is right on the screen and the bottom edge is the guides' near end,
  whichever way the camera is mounted. They used to turn and mirror with the
  picture's contents, which was correct arithmetic and meant re-placing every
  guide each time a mount was corrected.
- **It says why there is no picture**: not connected, handshaking, not
  subscribed yet, subscribed with nothing yet arrived, or too old — and any
  `EVENT` the board sent mentioning the camera is shown **verbatim** beside
  it, so "a second pilot has the camera" reaches the person instead of an
  empty rectangle.
- **Gaps are counted, never smoothed** — `frameIndex` is monotonic, so the
  window reports `frames 91-94 missing` rather than showing the next picture
  as though nothing were dropped.

## Driving

`src/drive.cxx` is the Drive window. Once this viewer holds the control slot,
WASD goes out as `CONTROL` at 20 Hz and the deliberate acts - **ARM**,
**DISARM**, **ESTOP**, **CLEAR ESTOP** - as `COMMAND`s, each answered by a
`CMDACK` whose sentence is shown.

- **Connecting never arms the car.** Let the stream run for half a second, then
  press **ARM**. The board refuses it, and says which reason, while the estop is
  latched, while the Pico is not answering, when the pilot is not in manual, or
  when the ARM carries a stale arm epoch.
- **Anything that moves the arm epoch disarms**: an estop, the deadman tripping
  after 300 ms of silence, the Pico link dropping, leaving the slot, or a
  **DISARM** from any viewer. A stream that resumes does not bring the arm back
  - press ARM again. Until 2026-09-12 the board refused ARM outright, and the
  only way to get throttle was starting the pilot with `--arm`.

## Not wired yet

- **`CTLSTATE` on UDP.** The socket is bound and its port is carried in
  `HELLO`, and a datagram that arrives is decoded and shown — but a board that
  only sends it to a control holder will never send it here.
