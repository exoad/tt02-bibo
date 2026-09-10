// The camera window: its own floating Dear ImGui window, showing the board's
// JPEG stream, and nothing else on screen changes when it is closed.
//
// ---------------------------------------------------------------------------
// `camview`, NOT `camera`
//
// Every other "camera" in this program is the 3D orbit camera - scene::Camera,
// scene::resetCamera, the R key. This module is the one that shows a picture
// from a lens on the car, and giving it the obvious name would make half the
// uses of the word in the viewer mean the other thing.
//
// ---------------------------------------------------------------------------
// THE WINDOW BEING OPEN IS THE SUBSCRIPTION
//
// This is the load-bearing behaviour of the whole module, not a nicety. The
// board only opens /dev/video0 while somebody is subscribed, and the stream is
// roughly 1 MB/s at 640x480 - measured, against docs/bibowire.md section 10's
// assumption of ~200 KB/s and its verdict that even THAT does not fit
// alongside the scan on this hotspot. So a camera window nobody is looking at
// must cost nothing: closing it sends SUBSCRIBE without the camera bit and
// releases the texture, exactly as the scan already costs nothing when there
// is no viewer.
//
// The consequence worth stating: opening this window takes bandwidth away from
// the scan, on a link that does not have it spare. CAMERA is CLASS_BULK and is
// discarded before any scan or state frame (section 7), so what degrades under
// pressure is this window and not the car's picture of the world - which is
// the reason a camera is safe to add to this link at all.
//
// ---------------------------------------------------------------------------
// THE D3D TYPES ARE FORWARD-DECLARED
//
// Same reason link.hxx never names a SOCKET: <d3d11.h> drags in <windows.h>,
// which defines `small`, `near`, `far` and - the one that actually breaks a
// build here - SEVERITY_ERROR, which is a member of bibowire::Severity. A
// header that made every file including it deal with that would be spreading a
// platform problem rather than containing it.
#pragma once

#include "shared.hxx"

#include "link.hxx"

struct ID3D11Device;
struct ID3D11DeviceContext;
struct ID3D11Texture2D;
struct ID3D11ShaderResourceView;

namespace camview
{

  // Past this the last frame is not drawn at all, only described. The scan's
  // numbers, deliberately: docs/bibowire.md section 7 fixes them for the
  // picture the operator reads, and a camera that stayed on screen under the
  // same conditions that blank the cloud would be the one surface still
  // claiming the car can see.
  //
  // They live in link.hxx as FRESH_MS and GONE_MS and are used from there
  // rather than restated, so there is one pair of numbers in this program.

  struct View
  {
      // The window, and therefore the subscription. Bound to ImGui::Begin's
      // close button, so the X in the corner is what unsubscribes.
      Bool open = false;

      // Recreated only when the dimensions change - not per frame. At 640x480
      // that is 1.2 MB of device memory and a full pipeline flush every time,
      // to display a picture whose size has not moved since the camera was
      // switched on.
      ID3D11Texture2D* tex = nullptr;
      ID3D11ShaderResourceView* srv = nullptr;
      Int32 texW = 0;
      Int32 texH = 0;

      // WHICH frame is currently in that texture. A JPEG is decoded once, when
      // its frameIndex is new, rather than once per redraw at 60 Hz.
      Bool haveShown = false;
      UInt32 shownIndex = 0;
      Size shownBytes = 0;

      // Counted, never smoothed, and shown. A frame that would not decode is a
      // fact about the link or the camera, and a viewer that silently drew the
      // previous picture instead would be reporting success while measuring
      // nothing.
      UInt32 decodeFailures = 0;
      Str decodeWhy;
  };

  // The device the textures are created on, and the DPI multiplier the layout
  // uses. Called once, after the D3D11 device and ImGui both exist.
  Void init(ID3D11Device* device, ID3D11DeviceContext* context, Float32 scale);

  // Releases the texture. Safe on a View that never had one, and safe to call
  // twice - it must run BEFORE the device is destroyed.
  Void shutdown(View& v);

  // One frame. Sets the subscription from `v.open`, draws the window when it
  // is open, and says why there is no picture when there is not one.
  Void drawWindow(View& v, link::Client& lk, const link::Snapshot& snap, Int64 nowMs);

}
