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

  // What the camera window asks the board for when nobody has touched the
  // slider.
  //
  // SIX, not ten. Ten was chosen for smoothness alone and measured against the
  // scan it competes with: asking for ten pushed the worst SCAN gap from 402 ms
  // to about 1420 ms, and GONE_MS is 1500 - eighty milliseconds before the point
  // cloud stops being drawn at all rather than merely greying. CLASS_BULK drops
  // pictures before scans on the wire, but the board still spends capture and
  // encode on frames the ring will discard, and that cost lands on the loop that
  // owns the lidar. Six is visibly moving rather than a slideshow, which is what
  // was actually asked for, and it leaves the scan its margin. The slider still
  // goes to CAM_FPS_MAX for anyone on a link that can afford it.
  constexpr Int32 CAM_FPS_DEFAULT_ASK = 6;

  // ---------------------------------------------------------------------------
  // GUIDE GEOMETRY, inline so the suite can reach it
  //
  // trim.hxx's reason arrived at from a third side, and orient.cxx's before
  // that: a quarter turn is a transpose and nobody can eyeball a transpose -
  // and nobody can eyeball a SIGN either. Which way the guides sweep when the
  // wheels turn right rests on two facts written down elsewhere, neither of
  // them inferred: chassis.hxx's steerToUs sends a positive fraction toward
  // servoMax, which cal.hxx names STEER_CAL_RIGHT, so positive steer is RIGHT;
  // and link.cxx states the frame where it builds the cloud, so image u grows
  // to the right. Invert either and this compiles perfectly, looks entirely
  // plausible, and is discovered while driving.
  //
  // So it lives here rather than in camera.cxx's anonymous namespace, where the
  // suite cannot reach it - camera.hxx names no ImGui and no D3D type, only
  // forward declarations, which is what makes that possible.

  // Clamped HERE rather than trusted from the slider. Ctrl+click on an ImGui
  // slider is a text box, and a guide built from a number outside its own range
  // is a line drawn somewhere off the picture.
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
  // bend. Capped well under half for a reason: guides that leave the picture
  // are marking something the camera cannot see.
  constexpr Float32 GUIDE_BEND_SPAN = 0.45f;

  // How far the guide at `t` - 0 at the bumper, 1 at the far end - is pushed
  // sideways in IMAGE space. t*t and not t: a steering angle has had more
  // distance to act at the far end, so the guides barely move at the bumper and
  // sweep hardest where they matter.
  [[nodiscard]] inline Float32 bendAt(Float32 steer, Int32 bendPct, Float32 t)
  {
      return steer * pctToUnit(bendPct, 0, 100) * GUIDE_BEND_SPAN * t * t;
  }

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

      // ---- how the picture is oriented -----------------------------------
      //
      // Quarter turns CLOCKWISE: 0, 1, 2, 3 for 0, 90, 180, 270 degrees. The
      // camera can be bolted to the car on its side, and a picture that is
      // only readable with the operator's head tilted is a picture nobody
      // reads in a hurry.
      //
      // These live on the View, which outlives the window being closed and
      // reopened, so a sideways mount is set up once and stays set for the
      // session rather than being re-entered every time the checkbox is
      // ticked. They are deliberately NOT reset by releaseTexture: the
      // orientation describes how the camera is MOUNTED, which does not change
      // because a frame was late.
      Int32 turns = 0;
      Bool flipX = false;
      Bool flipY = false;

      // ---- alignment overlays ---------------------------------------------
      //
      // OFF BY DEFAULT, all four of them. An alignment aid nobody asked for,
      // drawn over a live picture, is clutter on the one surface that is
      // supposed to show the room.
      //
      // NOTHING HERE IS CALIBRATED, and the UI says so rather than leaving it
      // to be inferred. There is no camera calibration in this project and no
      // measured camera-to-car transform - docs/conventions.md records even the
      // lidar-to-vehicle transform as assumed rather than measured - so these
      // lines carry no distance and are never labelled with one. They are marks
      // the operator places by eye and then reads the same way every time,
      // which is a real aid; a band labelled "1 m" would be an invented number
      // somebody judges clearance against.
      Bool showCross = false;
      Bool showGuides = false;
      Bool showBox = false;
      Bool showThirds = false;

      // The guide trapezoid, in PERCENT of the frame - whole numbers on
      // purpose. A Float32 slider would be printed by ImGui through "%.2f",
      // whose decimal point honours the locale, and a machine set to a comma
      // decimal writes "0,42" - the bug this project has already met three
      // times. Percent has no decimal point in it.
      //
      // Live here beside `turns` and for the same reason: they describe how the
      // camera is MOUNTED and how the operator reads it, so closing the window
      // and reopening it must not throw the setup away.
      //
      // centre and spread are the near end; converge is the half-width at the
      // far end, which is what makes it a trapezoid rather than a corridor.
      // near and far are heights down the frame, 0 at the top.
      Int32 guideCentrePct = 50;
      Int32 guideSpreadPct = 42;
      Int32 guideConvergePct = 12;
      Int32 guideNearPct = 100;
      Int32 guideFarPct = 45;

      // ---- guides that follow the steering --------------------------------
      //
      // A reversing camera's guides bend with the wheels, and that is what makes
      // them readable while turning rather than only while straight. Off by
      // default like every other overlay here.
      //
      // THE BEND IS A FEEL NUMBER, NOT A GEOMETRY. A real backup camera derives
      // its curve from a measured wheelbase, a steering-angle map and a lens
      // calibration. This project has none of the three - conventions.md records
      // even the lidar-to-vehicle transform as assumed - so this is a sweep the
      // operator tunes until it matches what the car does, exactly like the
      // spread and converge sliders above it. It carries no radius and is never
      // labelled with one.
      //
      // It follows where the wheels ARE (CTLSTATE's steerNowMilli), not what was
      // asked for: the slew limiter means a command takes about a second to
      // become an angle, so bending on the request would show a turn the car has
      // not made yet.
      Bool guideBend = false;
      Int32 guideBendPct = 45;

      // Half-width of the centred box, percent of the frame. It is a scaled
      // copy of the picture's own outline rather than a square - see drawBox.
      Int32 boxPct = 20;

      // ---- what rate this viewer asks the board for -----------------------
      //
      // Frames per second, 0 meaning "do not ask" - the board then keeps its
      // own conservative default. Held here rather than in link::Client so it
      // survives a disconnect and is re-sent on the next connection, the same
      // way `open` is.
      //
      // The default asks for a rate rather than sitting at 0: the board's
      // default is two frames a second, which is an honest number for a phone
      // hotspot and reads as a slideshow on a LAN. A camera window is already
      // opt-in and already costs bandwidth, so the useful default is the one
      // that shows moving pictures to the person who just asked for a camera.
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
