// The Camera window: a floating ImGui window showing the board's JPEG stream.
//
// `camview`, not `camera`: scene::Camera and resetCamera are the 3D orbit camera.
//
// THE WINDOW BEING OPEN IS THE SUBSCRIPTION. The board opens the camera only
// while somebody is subscribed, and the stream is about 1 MB/s at 640x480, more
// than scan and state together (docs/bibowire.md section 10). Closing the window
// sends SUBSCRIBE without the camera bit and releases the texture. CAMERA is
// CLASS_BULK and is discarded before any scan or state frame (section 7), so
// under pressure this window degrades, not the car's picture of the world. A
// frame is aged against link.hxx's FRESH_MS and GONE_MS, the scan's own bands.
//
// The D3D types are forward-declared: <d3d11.h> drags in <windows.h>, whose
// `small`, `near`, `far` and SEVERITY_ERROR macros break bibowire::Severity in
// every file that would include this header.
#pragma once

#include "shared.hxx"

#include "link.hxx"

struct ID3D11Device;
struct ID3D11DeviceContext;
struct ID3D11Texture2D;
struct ID3D11ShaderResourceView;

namespace camview
{
  // The rate asked for before the slider is touched. Not higher: at ten the
  // board's capture and encode starved the lidar loop until the worst scan gap
  // came within 80 ms of GONE_MS. The slider still goes to CAM_FPS_MAX.
  constexpr Int32 CAM_FPS_DEFAULT_ASK = 6;

  // Guide geometry is inline so the suite can check its SIGN: positive steer is
  // RIGHT (chassis.hxx steerToUs sends it toward servoMax, cal.hxx
  // STEER_CAL_RIGHT), and overlays are drawn in window space, where +x is right
  // whatever rotate and flip say. Invert either and it still looks plausible.
  // Clamped here rather than trusted: Ctrl+click turns a slider into a text box.
  [[nodiscard]] inline Float32 pctToUnit(Int32 pct, Int32 lo, Int32 hi)
  {
      Int32 n = pct;
      if(n < lo)
      {
          n = lo;
      }
      if(n > hi)
      {
          n = hi;
      }
      return static_cast<Float32>(n) * 0.01f;
  }

  // Fraction of the frame's width the far end may swing at full lock and full
  // bend. Well under half, so the guides stay on the picture.
  constexpr Float32 GUIDE_BEND_SPAN = 0.45f;

  // How far the guide at `t` (0 at the bumper, 1 at the far end) is pushed
  // sideways, as a fraction of the drawn picture's width. t*t, not t: a steering
  // angle has had more distance to act at the far end.
  [[nodiscard]] inline Float32 bendAt(Float32 steer, Int32 bendPct, Float32 t)
  {
      return steer * pctToUnit(bendPct, 0, 100) * GUIDE_BEND_SPAN * t * t;
  }

  struct View
  {
      // The window, and therefore the subscription. Bound to ImGui::Begin's
      // close button, so the X in the corner is what unsubscribes.
      Bool open = false;

      // Recreated only when the dimensions change, not per frame.
      ID3D11Texture2D* tex = nullptr;
      ID3D11ShaderResourceView* srv = nullptr;
      Int32 texW = 0;
      Int32 texH = 0;

      // Which frame is in the texture, so a JPEG is decoded once per frame and
      // not once per redraw.
      Bool haveShown = false;
      UInt32 shownIndex = 0;
      Size shownBytes = 0;

      // Counted and shown. A frame that would not decode is never silently
      // replaced by the previous picture.
      UInt32 decodeFailures = 0;
      Str decodeWhy;

      // Quarter turns CLOCKWISE (0..3 for 0..270 degrees) and flips, for a
      // camera mounted on its side. They describe the mount, so they outlive the
      // window being closed and releaseTexture does NOT reset them.
      Int32 turns = 0;
      Bool flipX = false;
      Bool flipY = false;

      // Alignment overlays, OFF BY DEFAULT. NOTHING HERE IS CALIBRATED: there is
      // no camera calibration or camera-to-car transform (docs/hardware.md), so
      // these are marks placed by eye and never labelled with a distance.
      Bool showCross = false;
      Bool showGuides = false;
      Bool showBox = false;
      Bool showThirds = false;

      // Whether the overlay settings show under the controls row. A panel, NOT A
      // POPUP: camera.cxx's drawOverlayToggle says why.
      Bool overlayPanel = false;

      // The guide trapezoid in whole PERCENT of the frame: a Float32 slider would
      // print through a locale-aware "%.2f". They outlive the window like `turns`.
      // centre and spread are the near end; converge is the half-width at the far
      // end; near and far are heights down the frame, 0 at the top.
      Int32 guideCentrePct = 50;
      Int32 guideSpreadPct = 42;
      Int32 guideConvergePct = 12;
      Int32 guideNearPct = 100;
      Int32 guideFarPct = 45;

      // Guides that bend with the wheels, off by default. THE BEND IS A FEEL
      // NUMBER, NOT GEOMETRY: there is no measured wheelbase, steering-angle map
      // or lens calibration, so it is tuned by eye and never labelled with a
      // radius. It follows CTLSTATE's steerNowMilli, where the wheels ARE, since
      // the slew limiter makes a command lag the real angle by about a second.
      Bool guideBend = false;
      Int32 guideBendPct = 45;

      // Half-width of the centred box, percent of the frame.
      Int32 boxPct = 20;

      // Frames per second asked of the board; 0 leaves the board's default of
      // two, a slideshow on a LAN. Held here, not in link::Client, so it
      // survives a disconnect and is re-sent on the next connection like `open`.
      Int32 fps = CAM_FPS_DEFAULT_ASK;
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
