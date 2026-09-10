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
  draw rather than a flag somebody has to remember to check. Past 400 ms the
  cloud is drawn desaturated with its age in the panel; past 1500 ms it is not
  drawn at all.
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

## Not wired yet

- **Driving.** The viewer connects as an *observer*: `HELLO` carries
  `wantControl = 0` and no `CONTROL` or `COMMAND` is ever sent. The Pico is not
  connected to the board, so that path cannot be tested end to end at all yet.
  The seam for it is at the bottom of `src/link.hxx`.
- **`CTLSTATE` on UDP.** The socket is bound and its port is carried in
  `HELLO`, and a datagram that arrives is decoded and shown — but a board that
  only sends it to a control holder will never send it here.
