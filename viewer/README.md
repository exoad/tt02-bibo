# viewer

The bibo viewer: one 3D view of the car and what the lidar can see, with a few
plain Dear ImGui panels floating over it. Win32 + D3D11, vanilla ImGui v1.92.9
from `third_party/imgui`. No editor, no language server, no reference browser —
the code is written elsewhere. It does not link the RPLIDAR SDK and never will:
the board owns the lidar and the viewer only ever receives scans.

## Build

    viewer\build.bat            incremental; `build.bat clean` wipes build\

MSVC x64, `/W4`, warning-clean. Produces `viewer\build\bibo.exe`.

## Run

    viewer\build\bibo.exe

Left-drag orbits, right-drag pans, the wheel zooms, `R` resets the camera.

## Not wired yet

- **The network.** Connection's host, port and buttons set local state only.
  The protocol client drops into `Link` in `src/main.cxx`, at the two `// SEAM`
  comments.
- **The scan.** The cloud is a synthetic room from `scene::fillSyntheticCloud`;
  the real scan replaces that one call in the frame loop.
- **The car readouts.** All `--`, because nothing has told this program
  otherwise. They come from the protocol's DECIDE and CTLSTATE messages.
