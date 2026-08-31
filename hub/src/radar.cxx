// Interactive 2D map of recent scan revolutions, drawn with ImDrawList.
//
// World space is millimeters with the sensor at the origin, x right, y down on
// screen; a measurement at (angle, dist) maps to (dist*sin a, -dist*cos a), so
// 0 degrees points up and angle increases clockwise. Screen space follows from
//     screen = center_px + (world_mm - view_center_mm) * px_per_mm
// which is the only place the view model enters the renderer: pan moves
// view_center_mm, zoom scales px_per_mm.
//
// The hot path is ~5 revolutions x ~500 points x 1 disc per frame at 60 fps,
// so the points are emitted as untessellated 8-gons written straight into the
// draw list through PrimReserve/PrimWriteVtx/PrimWriteIdx. Every point in a
// revolution shares one flat color, so the color is hoisted out of the inner
// loop entirely: distance is read off the rings, not off a color ramp.
//
// RENDER MODES (MapMode, radar.h). `mode` reaches exactly one switch, in
// draw(), around the block that emits the returns. Everything else - the view
// model, the gestures, the grid, the rings, the compass, the blind zone, the
// heading arrow, the scale bar, the nearest-return highlight, the measurement
// and every published readout - is mode-blind, so switching changes the marks
// and never the measurement.
//
//   Points     flat neutral dots, current revolution brightest. The default.
//   Rays       one clipped line per return, from the sensor. A direction that
//              came back empty simply has no ray, which is what makes a
//              dropout visible as a wedge instead of a gap you have to infer.
//   Distance   the same dots, colored off the ui::plot ramp, indexed against
//              the device's IN-SPEC WINDOW rather than the zoom - so a wall
//              keeps its color while you scale the view.
//   Density    hit counts accumulated into a fixed world-space grid over a
//              rolling window of revolutions, shaded by count.
//   Occupancy  the same grid, but returns persist and decay in wall-clock
//              time (io.DeltaTime, never per frame), with the live revolution
//              drawn over the memory so "now" stays distinct from "recalled".
//
// Density and Occupancy need state across frames and radar.h has nowhere to put
// it, so it lives in a file-static keyed on the owning RadarView - the same
// single-threaded-ImGui argument the scratch buffers rest on - and clear()
// resets it so a reconnect never draws the previous room.
//
// NOTE: the draw list caches _VtxWritePtr/_IdxWritePtr/_VtxCurrentIdx into its
// buffers. Growing VtxBuffer/IdxBuffer behind its back leaves those pointers
// dangling into freed memory, so all reservation goes through PrimReserve(),
// and _VtxCurrentIdx is re-read *after* every call (a large-mesh vertex-offset
// split resets it to 0).

#include "shared.hxx"
#include "radar.hxx"
#include "lights.hxx"
#include "map_geometry.hxx"
#include "vehicle.hxx"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <utility>

#include "theme.hxx"

namespace
{

  constexpr Size MAX_TRAIL = 5;      // revolutions kept, including the current one
  constexpr Int32    SEGS     = 8;      // segments per point disc (2-5 px across)
  constexpr Float32  PI       = 3.14159265358979323846f;

  // Shortest return treated as real. The C1 is specified from 0.05 m; anything
  // closer is a reflection off its own housing and must not drive the auto-fit
  // or win the "nearest obstacle" readout.
  constexpr Float32 MIN_VALID_MM = 50.0f;

  // Longest return treated as real. The C1 is specified to 12 m; beyond that the
  // device still occasionally reports something, but it is unreliable. Drawing
  // those while the readouts discarded them is the same contradiction the blind
  // zone exists to prevent, so the window is applied in both places.
  constexpr Float32 MAX_VALID_MM = 12000.0f;

  // The C1M1 datasheet rev 1.1, Figure 2-1, gives TWO ranges and we only ever
  // used one: 0.05-12 m against a 70%-reflective target, and 0.05-6 m against a
  // 10%-reflective one. So 12 m is the WHITE-WALL figure. A dark target at 8 m is
  // outside what the sensor is specified to see, and the "in-spec" percentage is
  // correspondingly optimistic on a dark scene.
  //
  // Not enforced as a second ceiling, because nothing here knows the reflectivity
  // of what it is looking at - it is recorded so the number is not mistaken for a
  // guarantee it never was.
  constexpr Float32 DARK_TARGET_MAX_MM = 6000.0f;

  // Figure 2-1: Resolution 15 mm, Accuracy +/-30 mm.
  //
  // This is what the sensor can DISTINGUISH, and it is why the readouts below do
  // not print a millimeter. Two surfaces 10 mm apart are one surface to a C1, and
  // a display that says "437 mm" is offering three significant figures the device
  // cannot support - which reads as precision and is decoration.
  constexpr Float32 RESOLUTION_MM = 15.0f;

  // Below this radius the blind disc is too small to render as anything but a
  // smudge, so it is skipped and the sensor hub stands in for it.
  //
  // It is deliberately NOT a floor on the drawn radius. Inflating the disc to a
  // legible minimum marks area the sensor can see perfectly well as blind: at a
  // typical fit zoom a 16 dp floor covered ~200 mm against a real blind radius of
  // 50 mm, so a hand held near the unit produced returns that landed inside the
  // disc labeled "cannot see here". The disc is drawn true to scale; zoom in to
  // inspect it.
  constexpr Float32 BLIND_MIN_PX = 4.0f;

  // Every range-ring label sits on this bearing (deg, 0 = up, clockwise), so the
  // labels form one legible radial column instead of landing wherever they fit.
  // Deliberately off both the cardinals (0/90/180/270) and the intercardinals
  // (45/135/...) that carry the compass numbers.
  constexpr Float32 RING_LABEL_BEARING = 25.0f;

  // Zoom limits, expressed as the visible radius (center -> nearer edge) in mm.
  constexpr Float32 MIN_VISIBLE_MM = 50.0f;      //  5 cm across the short half-axis
  constexpr Float32 MAX_VISIBLE_MM = 40000.0f;   // 40 m

  // Auto-fit easing. Growth is much faster than shrink: a return that lands
  // outside the current range has to appear now, while a range that has become
  // too large may settle back slowly without anything popping.
  constexpr Float32 FIT_RISE = 0.35f;
  constexpr Float32 FIT_FALL = 0.06f;

  // Discs written per PrimReserve() call. Keeps any single reservation far below
  // the 64K vertex ceiling of a 16-bit ImDrawIdx.
  constexpr Int32 DISC_BATCH = 2048;

  // ---------------------------------------------------------------- palette ---

  // Map chrome. These are plot colors, not UI chrome, so they are explicit
  // rather than following ImGui's theme - see theme.h.
  constexpr ImU32 RING_COL     = (ui::ansi::GRID       & 0x00FFFFFFu) | (0x9Au << IM_COL32_A_SHIFT);
  constexpr ImU32 RING_MAJOR   = (ui::ansi::GRID_MAJOR & 0x00FFFFFFu) | (0xA6u << IM_COL32_A_SHIFT);
  // Rings beyond the fitted range only ever clip a corner of the widget. Drawn
  // faint so those slivers stop reading as stray diagonal strokes.
  constexpr ImU32 RING_FAINT   = (ui::ansi::GRID       & 0x00FFFFFFu) | (0x60u << IM_COL32_A_SHIFT);
  constexpr ImU32 AXIS_COL     = ui::ansi::AXIS;
  constexpr ImU32 HEADING_COL  = ui::ansi::HEADING;
  constexpr ImU32 HUB_COL      = ui::ansi::BRCYAN;
  constexpr ImU32 HUB_CORE_COL  = ui::ansi::BRWHITE;
  constexpr ImU32 EMPTY_COL    = ui::ansi::GRAY;
  constexpr ImU32 NEAREST_COL  = ui::ansi::NEAREST;
  constexpr ImU32 MEASURE_COL  = ui::ansi::MEASURE;

  // Label plate: dark enough to punch a hole in a dense point cluster, but not so
  // opaque that the map reads as a grid of boxes.
  constexpr ImU32 PLATE_BG     = IM_COL32(0x00, 0x00, 0x00, 0xE0);

  // Ring / compass label inks. Distances read brighter than bearings so the two
  // families of number never get confused for one another.
  constexpr ImU32 RING_TEXT_COL = ui::ansi::GRAY;
  constexpr ImU32 BEARING_COL  = ui::ansi::GRAY;
  constexpr ImU32 CARDINAL_COL = ui::ansi::WHITE;
  constexpr ImU32 TICK_COL     = IM_COL32(0x4A, 0x4A, 0x4A, 0xFF);
  constexpr ImU32 TICK_MAJOR_COL= ui::ansi::GRAY;
  constexpr ImU32 SCALE_COL    = ui::ansi::WHITE;

  // Blind zone: the C1 returns nothing inside its 0.05 m spec floor. Drawn as a
  // hatched red-ish disc so it reads as "cannot see here", not "nothing here".
  // Deliberately a chalky, desaturated rose rather than a vivid warning orange:
  // at the smallest drawn size it sits directly under the nearest-return ring,
  // and the two must not read as one blob.
  constexpr ImU32 BLIND_FILL   = IM_COL32(0x00, 0x00, 0x00, 0xFF);
  constexpr ImU32 BLIND_HATCH  = IM_COL32(0xCD, 0x00, 0x00, 0x5A);
  constexpr ImU32 BLIND_EDGE   = IM_COL32(0xCD, 0x00, 0x00, 0xFF);
  constexpr ImU32 BLIND_TEXT   = IM_COL32(0xCD, 0x00, 0x00, 0xFF);

  // The far end of the same envelope: the 12 m spec limit. Same chalky family as
  // the blind zone so the two read as a matched pair rather than two unrelated
  // annotations, but dimmer - it is a boundary, not a hazard.
  constexpr ImU32 RANGE_LIMIT_COL = IM_COL32(0xCD, 0x00, 0x00, 0x9A);

  // Scan points. One flat neutral for every return at every distance: range is
  // what the rings and the scale bar are for, so tinting the points by it only
  // made the map look like a heatmap of something it was not measuring. Alpha is
  // supplied per revolution, so the RGB is kept separate from it here.
  //
  // That reasoning is why Points is the default and why it is untinted. It is NOT
  // an argument that a color ramp is never legitimate: the objection was to
  // color arriving unasked-for. MapMode::MAP_MODE_DISTANCE is the same ramp, chosen
  // deliberately, and it says so in the toggle.
  constexpr ImU32 POINT_RGB    = IM_COL32(0xFF, 0xFF, 0xFF, 0x00);

  // Density heat. A single dark-to-hot family, deliberately NOT the distance ramp
  // above: the two modes measure different quantities and must not share hues, or
  // a bright cell in one mode reads as a range in the other.
  constexpr ImU32 DENS_LOW_RGB  = IM_COL32(0x00, 0x00, 0xEE, 0x00);   // 1 hit
  constexpr ImU32 DENS_MID_RGB  = IM_COL32(0xFF, 0x00, 0xFF, 0x00);
  constexpr ImU32 DENS_HIGH_RGB = IM_COL32(0xFF, 0xFF, 0xFF, 0x00);   // saturated

  // Occupancy memory. Cool neutral so the decayed map reads as "remembered", and
  // the live revolution drawn over it in POINT_RGB reads as "now".
  constexpr ImU32 OCC_RGB      = IM_COL32(0x5C, 0x5C, 0xFF, 0x00);

  // Motion. The one alarming color in the file, and deliberately: a cell here
  // means something is where nothing was, which is the only thing the map draws
  // that a driving robot would need to react to.
  constexpr ImU32 MOTION_RGB   = IM_COL32(0xFF, 0x00, 0x00, 0x00);

  // Free space. Cool and desaturated so a large filled area does not dominate the
  // view - the fill is the subject but it is also most of the screen.
  constexpr ImU32 CLEAR_RGB    = IM_COL32(0x00, 0xCD, 0xCD, 0x00);

  // A drivable opening. The one place the status green is used as DATA rather than
  // as a UI state, and it means the same thing there as it does everywhere else -
  // this one is fine, go.
  constexpr ImU32 GAP_RGB      = IM_COL32(0x00, 0xFF, 0x00, 0x00);

  // A bearing that returned nothing at all. Steel, not red: no return is the
  // sensor working correctly against an absent or absorbing surface, and it must
  // not read as an error the way an out-of-range return does.
  constexpr ImU32 NORETURN_RGB = IM_COL32(0x7F, 0x7F, 0x7F, 0x00);

  // A fitted wall. Distinct from the raw returns it was fitted to, because the
  // difference between "measured" and "inferred" is the whole point of the mode.
  constexpr ImU32 WALL_RGB     = IM_COL32(0x00, 0xFF, 0xFF, 0x00);

  // Sweep ramp: position within one revolution. Deliberately a THIRD hue family,
  // sharing stops with neither the distance ramp nor the density heat, because a
  // point's color here says nothing about its range or how often it was seen.
  constexpr ImU32 SWEEP_START_RGB = IM_COL32(0x00, 0xFF, 0xFF, 0x00);   // aqua
  constexpr ImU32 SWEEP_MID_RGB   = IM_COL32(0xFF, 0x00, 0xFF, 0x00);   // violet
  constexpr ImU32 SWEEP_END_RGB   = IM_COL32(0xFF, 0xFF, 0x00, 0x00);   // yellow

  // Unit 8-gon, reused for every point so the inner loop needs no trig for the
  // disc itself. Radius is nudged out slightly so the polygon covers about as
  // much area as the circle it stands in for.
  struct UnitNgon
  {
      ImVec2 v[SEGS];

      UnitNgon()
      {
          for(Int32 i = 0; i < SEGS; ++i)
          {
              const Float32 a = static_cast<Float32>(i) * (2.0f * PI / static_cast<Float32>(SEGS));
              v[i] = ImVec2(std::cos(a) * 1.045f, std::sin(a) * 1.045f);
          }
      }
  };

  const UnitNgon& ngon()
  {
      static const UnitNgon n;
      return n;
  }

  // ------------------------------------------------------------- color ramp ---

  // Channel-wise blend of two packed colors. Alpha is dropped: every ramp entry
  // is stored with alpha 0 so the caller ORs in the per-revolution alpha, exactly
  // as POINT_RGB is used.
  inline ImU32 lerpRgb(ImU32 a, ImU32 b, Float32 t)
  {
      const Float32 u = 1.0f - t;
      const Int32 r = static_cast<Int32>((static_cast<Float32>(((a >> IM_COL32_R_SHIFT) & 0xFFu)) * u +
                          static_cast<Float32>(((b >> IM_COL32_R_SHIFT) & 0xFFu)) * t + 0.5f));
      const Int32 g = static_cast<Int32>((static_cast<Float32>(((a >> IM_COL32_G_SHIFT) & 0xFFu)) * u +
                          static_cast<Float32>(((b >> IM_COL32_G_SHIFT) & 0xFFu)) * t + 0.5f));
      const Int32 b2 = static_cast<Int32>((static_cast<Float32>(((a >> IM_COL32_B_SHIFT) & 0xFFu)) * u +
                           static_cast<Float32>(((b >> IM_COL32_B_SHIFT) & 0xFFu)) * t + 0.5f));
      return IM_COL32(r, g, b2, 0);
  }

  constexpr Int32 RAMP_N = 128;

  // A three-stop ramp baked once. Two of these exist: range (Distance mode) and
  // heat (Density mode). Both are indexed by a normalized quantity, never by a
  // screen-space one, so the same physical thing keeps its color as you zoom.
  struct Ramp
  {
      ImU32 c[RAMP_N];

      Ramp(ImU32 lo, ImU32 mid, ImU32 hi)
      {
          for(Int32 i = 0; i < RAMP_N; ++i)
          {
              const Float32 t = static_cast<Float32>(i) / static_cast<Float32>((RAMP_N - 1));
              c[i] = (t < 0.5f) ? lerpRgb(lo,  mid, t * 2.0f)
                                : lerpRgb(mid, hi,  (t - 0.5f) * 2.0f);
          }
      }

      ImU32 at(Float32 t) const
      {
          Int32 i = static_cast<Int32>((t * static_cast<Float32>((RAMP_N - 1)) + 0.5f));
          if(i < 0)
          {
              i = 0;
          }
          if(i >= RAMP_N)
          {
              i = RAMP_N - 1;
          }
          return c[i];
      }
  };

  // Range ramp: red near, yellow mid, green far - the ui::ansi::RAMP_* stops.
  const Ramp& rangeRamp()
  {
      static const Ramp r(ui::ansi::RAMP_NEAR, ui::ansi::RAMP_MID, ui::ansi::RAMP_FAR);
      return r;
  }

  const Ramp& heatRamp()
  {
      static const Ramp r(DENS_LOW_RGB, DENS_MID_RGB, DENS_HIGH_RGB);
      return r;
  }

  // Position within a revolution, 0 at the first sample and 1 at the last.
  const Ramp& sweepRamp()
  {
      static const Ramp r(SWEEP_START_RGB, SWEEP_MID_RGB, SWEEP_END_RGB);
      return r;
  }

  // Where a range sits in the device's in-spec window. Deliberately the SPEC
  // window and not the visible range: indexing against the zoom would repaint the
  // whole scene every time the view scaled, so the same wall would change color
  // while its distance had not changed at all.
  inline Float32 rangeT(Float32 mm)
  {
      const Float32 t = (mm - MIN_VALID_MM) / (MAX_VALID_MM - MIN_VALID_MM);
      return t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
  }

  // ------------------------------------------------------------------ utils ---

  inline Float32 clampf(Float32 v, Float32 lo, Float32 hi)
  {
      return v < lo ? lo : (v > hi ? hi : v);
  }

  Float32 currentDpi()
  {
      const Float32 s = ui::dpiScale();
      return (s > 0.0f && s < 16.0f) ? s : 1.0f;
  }

  // Map labels ride the app's "small" type role rather than a hardcoded pixel
  // size. LegacySize already has DPI baked in by LoadFonts, so it is never
  // multiplied by currentDpi() again.
  ImFont* labelFont()
  {
      return ui::fonts.small ? ui::fonts.small : ImGui::GetFont();
  }

  Float32 labelPx()
  {
      ImFont* f = ui::fonts.small;
      return (f && f->LegacySize > 0.0f) ? f->LegacySize : 15.0f * currentDpi();
  }

  // Rounds up to the next "nice" 1 / 2 / 5 x 10^n value, so ring spacing reads
  // as a round number at every zoom level.
  Float32 niceStep(Float32 raw)
  {
      if(!(raw > 0.0f))
      {
          return 1.0f;
      }

      const Float32 e = std::floor(std::log10(raw));
      const Float32 p = std::pow(10.0f, e);
      const Float32 m = raw / p;                    // 1 .. 10

      Float32 s;
      if(m < 1.5f)
      {
          s = 1.0f;
      }
      else if(m < 3.5f)
      {
          s = 2.0f;
      }
      else if(m < 7.5f)
      {
          s = 5.0f;
      }
      else
      {
          s = 10.0f;
      }
      return s * p;
  }

  // Rounds *down* to the next "nice" 1 / 2 / 5 x 10^n value. The scale bar needs
  // this direction: its drawn length must never exceed the budget it was given.
  Float32 niceStepDown(Float32 raw)
  {
      if(!(raw > 0.0f))
      {
          return 1.0f;
      }

      const Float32 e = std::floor(std::log10(raw));
      const Float32 p = std::pow(10.0f, e);
      const Float32 m = raw / p;                    // 1 .. 10

      Float32 s;
      if(m < 2.0f)
      {
          s = 1.0f;
      }
      else if(m < 5.0f)
      {
          s = 2.0f;
      }
      else
      {
          s = 5.0f;
      }
      return s * p;
  }

  // Ring labels. The rings are chosen radii, not measurements - a ring drawn at
  // exactly 500 mm IS at 500 mm - so these are printed as asked.
  Void formatRing(Char* buf, Size n, Float32 mm)
  {
      if(mm >= 1000.0f)
      {
          std::snprintf(buf, n, "%.1f m", static_cast<Float64>(mm) / 1000.0);
      }
      else
      {
          std::snprintf(buf, n, "%.0f mm", static_cast<Float64>(mm));
      }
  }

  // Readout labels: as much precision as the SENSOR deserves, which is a
  // different question from what the magnitude deserves.
  //
  // This used to print "%.0f mm" below a meter - "437 mm", three significant
  // figures. The C1 resolves 15 mm and is accurate to +/-30 mm, so the last digit
  // of that was never measured, and the digit before it was decoration. A readout
  // that jitters between 437 and 441 while nothing moves teaches you to distrust
  // the display; one that reads 440 and stays there is telling the truth.
  //
  // So: snapped to the resolution below a meter, and two decimals of a meter
  // above it - 10 mm granularity, which is still finer than the accuracy but
  // coarse enough not to claim anything absurd.
  Void formatDist(Char* buf, Size n, Float32 mm)
  {
      if(mm < 1000.0f)
      {
          const Float32 snapped =
              std::round(mm / RESOLUTION_MM) * RESOLUTION_MM;
          std::snprintf(buf, n, "%.0f mm", static_cast<Float64>(snapped));
      }
      else if(mm < 10000.0f)
      {
          std::snprintf(buf, n, "%.2f m", static_cast<Float64>(mm) / 1000.0);
      }
      else
      {
          std::snprintf(buf, n, "%.1f m", static_cast<Float64>(mm) / 1000.0);
      }
  }

  // ------------------------------------------------------------- primitives ---

  // A screen rectangle, top-left and bottom-right.
  //
  // This travelled as `const ImVec2& p0, const ImVec2& p1` through a dozen
  // signatures, which is how drawGridLabels came to take eight parameters and
  // run to 159 columns - it needs TWO rectangles, the plot area and the label
  // area, and spelling each as a pair meant four parameters for two things.
  struct Rect
  {
      ImVec2 p0{ 0.0f, 0.0f };
      ImVec2 p1{ 0.0f, 0.0f };
  };

  // How the world maps onto that rectangle: where the sensor sits in screen
  // space, how many pixels a millimeter is worth, and the display scale.
  //
  // These three are constant across every overlay drawn in a frame, while the
  // RECTANGLE changes - the grid draws into the plot area and its labels into a
  // slightly larger one. Separating them is what lets the varying part vary
  // without dragging the fixed part along as three more parameters.
  struct MapScale
  {
      ImVec2  s0{ 0.0f, 0.0f };   // sensor origin, screen space
      Float32 pxPerMm = 0.0f;
      Float32 dpi     = 1.0f;
  };

  // One screen-space point disc. Color is uniform across a revolution and so is
  // passed to emitDiscs() once rather than stored per point.
  struct Dot { Float32 x, y; };

  // Reused between revolutions and frames so the per-frame cost is a memcpy-free
  // refill rather than an allocation. ImGui is single-threaded, so a file-local
  // buffer is safe here.
  Vec<Dot>& scratch()
  {
      static Vec<Dot> s;
      return s;
  }

  // Separate buffer for the nearest-object highlight: it is emitted while the
  // main scratch still holds the current revolution's dots.
  Vec<Dot>& nearestScratch()
  {
      static Vec<Dot> s;
      return s;
  }

  // A screen-space point that carries its own color, for the modes where the

  // A screen-space axis-aligned cell, for Density and Occupancy.
  struct Cell { Float32 x0, y0, x1, y1; ImU32 c; };

  Vec<Cell>& cellScratch()
  {
      static Vec<Cell> s;
      return s;
  }

  // --------------------------------------------------------- persistent maps ---
  //
  // Density and Occupancy both accumulate into the SAME fixed world-space grid,
  // anchored on the sensor origin in millimeters. World-space and not screen-space
  // is the whole point: a cell must mean the same patch of floor at every zoom, or
  // panning would redraw the map into a different shape.
  //
  // radar.h is the contract and carries no room for this, so it lives here, keyed
  // on the RadarView that owns it. ImGui is single-threaded and the app holds one
  // map, so a single-slot file-static is safe; a second view simply takes the slot
  // over and starts from empty rather than inheriting the first one's room.

  constexpr Float32 CELL_MM    = 60.0f;                       // ~6 cm per cell
  constexpr Int32   GRID_HALF  = static_cast<Int32>((MAX_VALID_MM / CELL_MM)); // 200 cells each way
  constexpr Int32   GRID_N     = GRID_HALF * 2 + 1;            // 401
  constexpr Int32   GRID_CELLS = GRID_N * GRID_N;              // 160,801

  // Density's rolling window, in revolutions. ~4 s at the C1's 9.8 Hz: long
  // enough that a wall accumulates a solid count, short enough that the map still
  // tracks a scene that changes.
  constexpr Int32 DENS_WINDOW = 40;

  // Hit count at which a density cell is fully saturated. Fixed, NOT the current
  // frame's maximum: normalizing against a running maximum would make the same
  // count mean a different brightness from one frame to the next, which is the
  // one thing a heatmap may not do.
  constexpr Float32 DENS_FULL = 36.0f;

  // Occupancy decay. Time constant in seconds, and the value below which a cell
  // is dropped from the active list and considered forgotten.
  constexpr Float32 OCC_TAU   = 9.0f;
  constexpr Float32 OCC_FLOOR = 0.02f;

  // --- The vehicle -----------------------------------------------------------
  //
  // Up here rather than beside the mode that first needed it, because three
  // modes now measure against the car - Fit erodes free space by it, Full draws
  // it and projects a corridor from it - and a chassis dimension that lives
  // inside one renderer is a fact the other two have to guess at.
  // The car, from vehicle.hxx - one definition for the whole app. These aliases
  // keep the drawing code readable; the numbers and their sources are there.
  constexpr Float32 EGO_LEN_MM       = vehicle::CAR_LEN_MM;
  constexpr Float32 EGO_WID_MM       = vehicle::CAR_WID_MM;
  constexpr Float32 EGO_HEIGHT_MM    = vehicle::CAR_HEIGHT_MM;
  constexpr Float32 EGO_WHEELBASE_MM = vehicle::CAR_WHEELBASE_MM;
  constexpr Float32 EGO_TREAD_MM     = vehicle::CAR_TREAD_MM;
  constexpr Float32 EGO_WHEEL_D_MM   = vehicle::CAR_TIRE_DIA_MM;
  constexpr Float32 EGO_WHEEL_W_MM   = vehicle::CAR_TIRE_WID_MM;
  constexpr Float32 EGO_SENSOR_AHEAD_MM = vehicle::C1_MOUNT_AHEAD_MM;

  // The sensor is drawn at the middle of the chassis because that is the only
  // position that is currently TRUE: the C1 is not mounted yet. When it is, this
  // becomes a measured offset and the footprint shifts around the origin - the
  // drawing already works in terms of it rather than assuming the center.

  // --- Clearance -------------------------------------------------------------
  //
  // Nearest in-spec return per bearing bin, smoothed in time. 3 deg bins: at ~505
  // points a revolution that is four samples per bin, and four is the smallest
  // number that makes an EMPTY bin mean something. At 1.5 deg it was two, a
  // quarter of the bins came up empty by chance, and every one of them drew a
  // spike out to the range ceiling.
  constexpr Int32   CLR_BINS   = 120;
  constexpr Float32 CLR_BIN_DEG = 360.0f / static_cast<Float32>(CLR_BINS);

  // Consecutive empty revolutions before a bin is allowed to open toward the
  // ceiling. An empty bin is genuinely ambiguous - open space and a surface too
  // dark to return look identical from here - so a single one is treated as no
  // information and the bin holds. Sustained emptiness is evidence; one
  // revolution of it is not.
  constexpr Int32 CLR_MISS_OPEN = 4;

  // Asymmetric smoothing, and the asymmetry is the point. A bin CLOSES instantly
  // - something that just appeared two meters ahead is reported at two meters on
  // the frame it appears, because a free-space map that lags an obstacle is worse
  // than no free-space map. A bin OPENS slowly, so a single dropped return cannot
  // briefly declare a wall gone. Per-second rate, applied via a dt-corrected step.
  constexpr Float32 CLR_OPEN_TAU = 0.55f;

  // --- Motion ----------------------------------------------------------------
  //
  // A cell counts as motion when it is hit and the neighborhood around it was
  // cold in the occupancy map.
  //
  // THE NEIGHBORHOOD IS A DISTANCE, NOT A CELL COUNT, and getting that wrong is
  // what made this mode useless. It was a fixed 3x3 - plus or minus one 60 mm cell
  // - which is smaller than the gap between adjacent samples at anything past
  // about 6 m: the C1 steps 0.72 deg, so neighboring returns on the SAME wall are
  // 25 mm apart at 2 m but 151 mm apart at 12 m. Beyond that crossover every
  // return on a distant stationary surface landed further from the last one than
  // the test could reach, found nothing warm, and was reported as new. Forever.
  // The symptom was exactly what you would expect and exactly what was reported:
  // near things behaved, far things strobed.
  //
  // So the radius is derived from the sampling geometry instead of picked:
  //
  //     radius = 1.4 * (arc between samples at this range) + 45 mm
  //
  // where the 45 mm covers the +/-30 mm range spec and the phase drift between
  // revolutions (the motor is not locked to the sample clock, so a wall's returns
  // walk along it). That is 1 cell close in and 5 at the ceiling.
  //
  // This deliberately makes the mode LESS sensitive with range, and it should be:
  // at 12 m the device cannot place a return to better than ~150 mm, so declaring
  // motion at 60 mm resolution out there was reporting precision that does not
  // exist.
  constexpr Float32 MOT_COLD_BELOW = 0.35f;
  constexpr Float32 MOT_ARC_RAD    = 0.72f * PI / 180.0f;   // one sample step
  constexpr Float32 MOT_SLACK_MM   = 45.0f;                 // range spec + phase drift
  constexpr Int32   MOT_MAX_CELLS  = 5;                     // bounds the worst case

  // Revolutions of evidence before motion is reported at all. On the first sweep
  // nothing has been seen before, so every return is legitimately new and the
  // whole map flashes - which is true, useless, and looks like a fault.
  constexpr Int32 MOT_WARMUP_REVS = 8;

  // Motion fades much faster than occupancy - it is an event, not a map.
  constexpr Float32 MOT_TAU   = 1.6f;
  constexpr Float32 MOT_FLOOR = 0.04f;

  // Cell index for a world position, or -1 when it falls outside the grid. The
  // grid covers exactly the in-spec window, so anything the rest of this file
  // already discards is outside it too.
  inline Int32 cellIndex(Float32 wx, Float32 wy)
  {
      const Int32 ix = static_cast<Int32>(std::floor(wx / CELL_MM)) + GRID_HALF;
      const Int32 iy = static_cast<Int32>(std::floor(wy / CELL_MM)) + GRID_HALF;
      if(ix < 0 || ix >= GRID_N || iy < 0 || iy >= GRID_N)
      {
          return -1;
      }
      return iy * GRID_N + ix;
  }

  // Grid axis index for a world coordinate, unclamped, for computing the visible
  // index window from the widget rect.
  inline Int32 cellAxis(Float32 w)
  {
      return static_cast<Int32>(std::floor(w / CELL_MM)) + GRID_HALF;
  }

  struct MapState
  {
      const RadarView* owner = nullptr;
      Bool             ready = false;      // buffers allocated

      // Density: a count per cell, plus a ring of the cell indices each revolution
      // contributed so the oldest revolution can be decremented back out again.
      // Exact, and bounded: the ring holds at most DENS_WINDOW revolutions.
      Vec<UInt16> dens;
      Vec<Int32>  ring[DENS_WINDOW];
      Int32                   ringHead = 0;

      // Occupancy: an intensity per cell, plus the list of cells that are above
      // the floor. Decaying only the active list keeps the per-frame cost
      // proportional to what has actually been seen, not to the whole grid.
      Vec<Float32>   occ;
      Vec<Int32> occActive;

      // Motion: same shape as occupancy, and read against it.
      Vec<Float32>   mot;
      Vec<Int32> motActive;

      // Clearance: the smoothed nearest range per bearing bin, in mm. 0 means the
      // bin has never had a return; MAX_VALID_MM means "clear as far as the sensor
      // can see", and those are different facts, so they are drawn differently.
      Array<Float32, CLR_BINS> clr= {};
      Array<Bool, CLR_BINS> clrSeen= {};
      Array<Int32, CLR_BINS> clrMiss= {};   // consecutive revolutions with no return

      // Revolutions folded in. Motion needs it: on the first sweep nothing has
      // been seen before, so everything is legitimately new.
      Int32 revs = 0;

      // The world frame's reference profile - the room as it looked when the
      // frame was zeroed - plus the current estimate against it. See
      // mapgeo::estimateHeading.
      Array<Float32, CLR_BINS> refClr= {};
      Array<Bool, CLR_BINS> refSeen= {};
      Bool    refValid   = false;
      Float32 headingDeg = 0.0f;
      Float32 headingOk  = 0.0f;   // 0..1 confidence

      Void ensure()
      {
          if(ready)
          {
              return;
          }
          dens.assign(static_cast<Size>(GRID_CELLS), static_cast<UInt16>(0));
          occ.assign(static_cast<Size>(GRID_CELLS), 0.0f);
          mot.assign(static_cast<Size>(GRID_CELLS), 0.0f);
          occActive.clear();
          motActive.clear();
          occActive.reserve(4096);
          motActive.reserve(1024);
          for(Int32 i = 0; i < DENS_WINDOW; ++i)
          {
              ring[i].clear();
          }
          ringHead = 0;
          ready = true;
      }

      // Back to an empty room. Buffers are kept allocated - this runs on every
      // reconnect, and the grid is the same size every time.
      Void reset()
      {
          if(ready)
          {
              std::fill(dens.begin(), dens.end(), static_cast<UInt16>(0));
              std::fill(occ.begin(), occ.end(), 0.0f);
              std::fill(mot.begin(), mot.end(), 0.0f);
          }
          occActive.clear();
          motActive.clear();
          for(Int32 i = 0; i < DENS_WINDOW; ++i)
          {
              ring[i].clear();
          }
          ringHead = 0;
          for(Int32 i = 0; i < CLR_BINS; ++i)
          {
              clr[i] = 0.0f; clrSeen[i] = false; clrMiss[i] = 0;
          }
      }
  };

  // One accumulator per VIEW, not one shared and reset on every switch.
  //
  // It was a single static that wiped itself whenever the owner changed, which
  // was correct while there was exactly one map on screen. There are two now -
  // the live map and the recorder's playback view - and with the old scheme
  // switching tabs threw away the density and occupancy history each time, which
  // is precisely the history those modes exist to accumulate.
  //
  // A fixed slot table rather than a Map: the number of views is a property
  // of the UI and is small, so the bound belongs in the code where it can be seen.
  MapState& mapStateFor(const RadarView* owner)
  {
      constexpr Int32 SLOTS = 4;
      static MapState pool[SLOTS];

      for(Int32 i = 0; i < SLOTS; ++i)
      {
          if(pool[i].owner == owner)
          {
              return pool[i];
          }
      }

      for(Int32 i = 0; i < SLOTS; ++i)
      {
          if(pool[i].owner == nullptr)
          {
              pool[i].reset();
              pool[i].owner = owner;
              return pool[i];
          }
      }

      // More views than slots. Recycling the last one keeps the app working and
      // degrades one view's history rather than failing; if this ever fires,
      // SLOTS is the number to raise.
      pool[SLOTS - 1].reset();
      pool[SLOTS - 1].owner = owner;
      return pool[SLOTS - 1];
  }

  // World position of a return, in mm, sensor at the origin. Same convention as
  // everywhere else in this file: 0 deg is up, angle increases clockwise.
  inline Void returnWorld(const LidarPoint& p, Float32& wx, Float32& wy)
  {
      const Float32 a = (p.angleDeg - 90.0f) * (PI / 180.0f);
      wx = p.distMm * std::cos(a);
      wy = p.distMm * std::sin(a);
  }

  inline Bool inWindow(Float32 d)
  {
      return d >= MIN_VALID_MM && d <= MAX_VALID_MM;
  }

  // Folds one revolution into both accumulators. Called from push(), once per
  // revolution, in EVERY mode - the maps are what the sensor has seen, and that
  // must not depend on which mode happened to be on screen while it saw it.
  // True when every cell of the 3x3 around `ci` is below the cold threshold, i.e.
  // nothing has been seen anywhere near here recently. Rows are contiguous, so the
  // neighborhood is nine indexed reads with no bounds maths beyond the edges.
  // How far either side of a cell counts as "the same place", given how far away
  // it is. See MOT_ARC_RAD for why this cannot be a constant.
  Int32 motionRadiusCells(Float32 rangeMm)
  {
      const Float32 mm = 1.4f * rangeMm * MOT_ARC_RAD + MOT_SLACK_MM;
      Int32 n = static_cast<Int32>(std::ceil(mm / CELL_MM));
      if(n < 1)
      {
          n = 1;
      }
      if(n > MOT_MAX_CELLS)
      {
          n = MOT_MAX_CELLS;
      }
      return n;
  }

  Bool neighborhoodCold(const MapState& st, Int32 ci, Int32 rad)
  {
      const Int32 ix = ci % GRID_N;
      const Int32 iy = ci / GRID_N;

      const Int32 x0 = (ix > rad) ? ix - rad : 0;
      const Int32 x1 = (ix < GRID_N - 1 - rad) ? ix + rad : GRID_N - 1;
      const Int32 y0 = (iy > rad) ? iy - rad : 0;
      const Int32 y1 = (iy < GRID_N - 1 - rad) ? iy + rad : GRID_N - 1;

      for(Int32 y = y0; y <= y1; ++y)
      {
          const Size row = static_cast<Size>(y) * static_cast<Size>(GRID_N);
          for(Int32 x = x0; x <= x1; ++x)
          {
              if(st.occ[row + static_cast<Size>(x)] >= MOT_COLD_BELOW)
              {
                  return false;
              }
          }
      }
      return true;
  }

  Void accumulateRevolution(MapState& st, const Vec<LidarPoint>& pts)
  {
      st.ensure();

      if(st.revs < 1000000)
      {
          ++st.revs;
      }

      // Retire the revolution that has just rolled out of the window.
      Vec<Int32>& slot = st.ring[st.ringHead];
      for(Int32 ci : slot)
      {
          if(st.dens[static_cast<Size>(ci)] > 0)
          {
              --st.dens[static_cast<Size>(ci)];
          }
      }
      slot.clear();

      // Nearest in-spec return per bearing bin for THIS revolution. Seeded above
      // the ceiling so "no return in this bin" is distinguishable from "a return
      // at the ceiling", which the smoothing below treats differently.
      Array<Float32, CLR_BINS> bin;
      for(Int32 i = 0; i < CLR_BINS; ++i)
      {
          bin[i] = FLT_MAX;
      }

      for(const LidarPoint& p : pts)
      {
          if(!inWindow(p.distMm))
          {
              continue;
          }

          // Clearance is per-bearing and needs no world position.
          {
              Float32 a = p.angleDeg;
              a -= std::floor(a / 360.0f) * 360.0f;          // into [0, 360)
              Int32   b = static_cast<Int32>((a / CLR_BIN_DEG));
              if(b < 0)
              {
                  b = 0;
              }
              if(b >= CLR_BINS)
              {
                  b = CLR_BINS - 1;
              }
              if(p.distMm < bin[b])
              {
                  bin[b] = p.distMm;
              }
          }

          Float32 wx, wy;
          returnWorld(p, wx, wy);

          const Int32 ci = cellIndex(wx, wy);
          if(ci < 0)
          {
              continue;
          }

          if(st.dens[static_cast<Size>(ci)] < 0xFFFFu)
          {
              ++st.dens[static_cast<Size>(ci)];
              slot.push_back(ci);
          }

          // Motion is decided BEFORE occupancy is written, or every return would
          // find the cell it just asserted and nothing would ever look new.
          //
          // The radius comes from THIS return's range: a far return is allowed to
          // have wandered further and still be the same surface.
          if(st.revs >= MOT_WARMUP_REVS
             && neighborhoodCold(st, ci, motionRadiusCells(p.distMm)))
          {
              if(!(st.mot[static_cast<Size>(ci)] > 0.0f))
              {
                  st.motActive.push_back(ci);
              }
              st.mot[static_cast<Size>(ci)] = 1.0f;
          }

          // Occupancy: a fresh return re-asserts the cell completely. The cell
          // then fades on its own from here, in wall-clock time.
          if(!(st.occ[static_cast<Size>(ci)] > 0.0f))
          {
              st.occActive.push_back(ci);
          }
          st.occ[static_cast<Size>(ci)] = 1.0f;
      }

      // Fold this revolution's profile into the smoothed one. Closing is
      // immediate; opening eases; an isolated empty bin holds where it is.
      const Float32 rise = 1.0f - std::exp(-(1.0f / 10.0f) / CLR_OPEN_TAU);  // ~1 rev
      for(Int32 i = 0; i < CLR_BINS; ++i)
      {
          const Bool empty = (bin[i] == FLT_MAX);

          if(empty)
          {
              if(++st.clrMiss[i] < CLR_MISS_OPEN)
              {
                  continue;                        // no information: hold
              }
          }
          else
          {
              st.clrMiss[i] = 0;
          }

          const Float32 target = empty ? MAX_VALID_MM : bin[i];

          if(!st.clrSeen[i])
          {
              st.clr[i]      = target;
              st.clrSeen[i] = true;
          }
          else if(target < st.clr[i])
          {
              st.clr[i] = target;                                   // closes now
          }
          else
          {
              st.clr[i] += (target - st.clr[i]) * rise;             // opens slowly
          }
      }

      st.ringHead = (st.ringHead + 1) % DENS_WINDOW;
  }

  // Time-based, never frame-based: at 30 fps and at 144 fps a cell must reach the
  // same intensity after the same number of SECONDS, or the map means something
  // different on a slower machine. dt is clamped so a stall (a resize, a debugger
  // break) cannot wipe the map in one step.
  Void decayField(Vec<Float32>& v, Vec<Int32>& active, Float32 tau, Float32 floorV, Float32 dt)
  {
      if(active.empty() || !(dt > 0.0f))
      {
          return;
      }
      if(dt > 0.25f)
      {
          dt = 0.25f;
      }

      const Float32 k = std::exp(-dt / tau);

      Size w = 0;
      for(Size r = 0; r < active.size(); ++r)
      {
          const Int32 ci = active[r];
          const Float32   nv = v[static_cast<Size>(ci)] * k;

          if(nv < floorV)
          {
              v[static_cast<Size>(ci)] = 0.0f;      // forgotten; drop from the list
              continue;
          }

          v[static_cast<Size>(ci)] = nv;
          active[w++]   = ci;
      }
      active.resize(w);
  }

  Void decayOccupancy(MapState& st, Float32 dt)
  {
      if(!st.ready)
      {
          return;
      }

      decayField(st.occ, st.occActive, OCC_TAU, OCC_FLOOR, dt);
      decayField(st.mot, st.motActive, MOT_TAU, MOT_FLOOR, dt);
  }

  // Filled convex 8-gons written directly into the draw list. Skips ImGui's arc
  // tessellation and anti-aliased fringe, which together cost roughly 3x the
  // vertices and a pair of trig calls per segment.
  //
  // Exactly SEGS vertices and (SEGS-2)*3 == 18 indices are written per disc,
  // which is precisely what is reserved for it.
  // Grows outward from the closest sample to cover the whole nearest surface.
  //
  // Returns arrive angle-sorted, so a physical object is a contiguous run of
  // samples at a similar range. Two samples belong to the same object when the
  // angular gap between them is small (no missed returns in between) and the
  // radial step is small (not a jump to something behind it). The run is capped
  // so a smooth wall cannot drag the highlight around the entire room.
  Void gatherNearestCluster(const Vec<LidarPoint>& pts, Int32 bestI, const ImVec2& s0, Float32 ppm, Vec<Dot>& out)
  {
      out.clear();

      const Int32 n = static_cast<Int32>(pts.size());
      if(n <= 0 || bestI < 0 || bestI >= n)
      {
          return;
      }

      constexpr Float32 MAX_ANGLE_GAP_DEG = 3.0f;   // samples sit ~0.7 deg apart
      constexpr Int32   MAX_SAMPLES     = 140;    // ~100 deg of a dense revolution

      const Float32 deg2rad = PI / 180.0f;

      auto emit = [&](Int32 i)
      {
          const LidarPoint& p = pts[static_cast<Size>(i)];
          const Float32 rr  = p.distMm * ppm;
          const Float32 ang = (p.angleDeg - 90.0f) * deg2rad;
          out.push_back(Dot{ s0.x + rr * std::cos(ang), s0.y + rr * std::sin(ang) });
      };

      // A step is "smooth" relative to the range it is at: 60 mm of slop at the
      // sensor, proportionally more further out where samples are further apart.
      auto joins = [](Float32 aMm, Float32 bMm)
      {
          const Float32 tol = std::max(60.0f, 0.12f * std::min(aMm, bMm));
          return std::fabs(aMm - bMm) <= tol;
      };

      auto inWindow = [](const LidarPoint& p)
      {
          return p.distMm >= MIN_VALID_MM && p.distMm <= MAX_VALID_MM;
      };

      auto angleGap = [](Float32 fromDeg, Float32 toDeg)
      {
          Float32 d = std::fabs(toDeg - fromDeg);
          // the wrap at 0/360
          if(d > 180.0f)
          {
              d = 360.0f - d;
          }
          return d;
      };

      emit(bestI);

      // Walk both ways round the revolution, wrapping modulo n.
      for(Int32 dir = -1; dir <= 1; dir += 2)
      {
          Int32 prev = bestI;
          for(Int32 step = 1; step < MAX_SAMPLES; ++step)
          {
              const Int32 i = ((bestI + dir * step) % n + n) % n;
              if(i == bestI)
              {
                  break;                          // wrapped all the way round
              }

              const LidarPoint& p = pts[static_cast<Size>(i)];
              const LidarPoint& q = pts[static_cast<Size>(prev)];

              if(!inWindow(p))
              {
                  break;
              }
              if(angleGap(q.angleDeg, p.angleDeg) > MAX_ANGLE_GAP_DEG)
              {
                  break;
              }
              if(!joins(q.distMm, p.distMm))
              {
                  break;
              }

              emit(i);
              prev = i;
          }
      }
  }

  Void emitDiscs(ImDrawList* dl, const Dot* dots, Int32 count, Float32 r, ImU32 col, const ImVec2& uv)
  {
      if(count <= 0 || r < 0.35f || (col & IM_COL32_A_MASK) == 0)
      {
          return;
      }

      const ImVec2* u = ngon().v;

      for(Int32 start = 0; start < count; start += DISC_BATCH)
      {
          const Int32 n = std::min(DISC_BATCH, count - start);

          dl->PrimReserve(n * (SEGS - 2) * 3, n * SEGS);

          // Must be read after PrimReserve(): a large-mesh split resets it to 0.
          UInt32 base = dl->_VtxCurrentIdx;

          for(Int32 k = 0; k < n; ++k)
          {
              const Dot& d = dots[start + k];

              for(Int32 i = 0; i < SEGS; ++i)
              {
                  dl->PrimWriteVtx(ImVec2(d.x + u[i].x * r, d.y + u[i].y * r), uv, col);
              }

              for(UInt32 i = 2; i < static_cast<UInt32>(SEGS); ++i)
              {
                  dl->PrimWriteIdx(static_cast<ImDrawIdx>(base));
                  dl->PrimWriteIdx(static_cast<ImDrawIdx>((base + i - 1u)));
                  dl->PrimWriteIdx(static_cast<ImDrawIdx>((base + i)));
              }
              base += static_cast<UInt32>(SEGS);
          }
      }
  }

  // Quads written per PrimReserve() call. 8192 quads is 32768 vertices, half the
  // 64K ceiling of a 16-bit ImDrawIdx, so a batch can never overflow one.
  constexpr Int32 QUAD_BATCH = 8192;

  // A closed triangle fan around `hub`, for the clearance polygon.
  //
  // AddConvexPolyFilled cannot be used for this: a room's free-space profile is
  // emphatically not convex, and that call assumes it is - it would fill straight
  // across every doorway and alcove. The profile IS star-shaped about the sensor
  // though, by construction, since it holds exactly one radius per bearing. So a
  // fan from the sensor is not an approximation, it is the exact shape, and it
  // costs one triangle per bin with no seams between them.
  //
  // Reserved: `count` triangles and count+1 vertices.
  Void emitFan(ImDrawList* dl, const ImVec2& hub, const ImVec2* ring, Int32 count, ImU32 col, const ImVec2& uv)
  {
      if(count < 3 || (col & IM_COL32_A_MASK) == 0)
      {
          return;
      }

      dl->PrimReserve(count * 3, count + 1);
      const UInt32 base = dl->_VtxCurrentIdx;

      dl->PrimWriteVtx(hub, uv, col);
      for(Int32 i = 0; i < count; ++i)
      {
          dl->PrimWriteVtx(ring[i], uv, col);
      }

      for(Int32 i = 0; i < count; ++i)
      {
          const UInt32 a = base + 1u + static_cast<UInt32>(i);
          const UInt32 b = base + 1u + static_cast<UInt32>(((i + 1) % count));
          dl->PrimWriteIdx(static_cast<ImDrawIdx>(base));
          dl->PrimWriteIdx(static_cast<ImDrawIdx>(a));
          dl->PrimWriteIdx(static_cast<ImDrawIdx>(b));
      }
  }

  // Axis-aligned filled cells, one color each.
  //
  // Reserved per cell: 4 vertices and 6 indices.
  // Written  per cell: 4 PrimWriteVtx and 2 triangles == 6 PrimWriteIdx.
  Void emitCells(ImDrawList* dl, const Cell* cells, Int32 count, const ImVec2& uv)
  {
      if(count <= 0)
      {
          return;
      }

      for(Int32 start = 0; start < count; start += QUAD_BATCH)
      {
          const Int32 n = std::min(QUAD_BATCH, count - start);

          dl->PrimReserve(n * 6, n * 4);
          UInt32 base = dl->_VtxCurrentIdx;

          for(Int32 k = 0; k < n; ++k)
          {
              const Cell& c = cells[start + k];

              dl->PrimWriteVtx(ImVec2(c.x0, c.y0), uv, c.c);
              dl->PrimWriteVtx(ImVec2(c.x1, c.y0), uv, c.c);
              dl->PrimWriteVtx(ImVec2(c.x1, c.y1), uv, c.c);
              dl->PrimWriteVtx(ImVec2(c.x0, c.y1), uv, c.c);

              dl->PrimWriteIdx(static_cast<ImDrawIdx>(base));
              dl->PrimWriteIdx(static_cast<ImDrawIdx>((base + 1u)));
              dl->PrimWriteIdx(static_cast<ImDrawIdx>((base + 2u)));
              dl->PrimWriteIdx(static_cast<ImDrawIdx>(base));
              dl->PrimWriteIdx(static_cast<ImDrawIdx>((base + 2u)));
              dl->PrimWriteIdx(static_cast<ImDrawIdx>((base + 3u)));

              base += 4u;
          }
      }
  }

  // Strokes only the part of a circle that can fall inside the widget. Range
  // rings are centered on the sensor, which may sit far outside the rect once the
  // view is panned; tessellating the whole circle would then cost thousands of
  // segments for a few visible pixels.

  // Only the visible slice of a dashed circle. Same visibility reasoning as
  Void strokeRing(ImDrawList* dl, const ImVec2& c, Float32 r, const Rect& area, ImU32 col, Float32 th)
  {
      const ImVec2& p0 = area.p0;
      const ImVec2& p1 = area.p1;
      if(r <= 0.75f)
      {
          return;
      }

      const Bool inside = (c.x >= p0.x && c.x <= p1.x && c.y >= p0.y && c.y <= p1.y);
      if(inside)
      {
          Int32 seg = static_cast<Int32>((2.0f * PI * r / 12.0f));
          seg = static_cast<Int32>(clampf(static_cast<Float32>(seg), 16.0f, 512.0f));
          dl->AddCircle(c, r, col, seg, th);
          return;
      }

      // The sensor is outside the rect, so the rect subtends less than 180 deg
      // from it: the corner bearings, taken relative to the rect center, bracket
      // the only arc worth drawing.
      const ImVec2 rc((p0.x + p1.x) * 0.5f, (p0.y + p1.y) * 0.5f);
      const Float32  ac = std::atan2(rc.y - c.y, rc.x - c.x);

      const ImVec2 corner[4] = { p0, ImVec2(p1.x, p0.y), p1, ImVec2(p0.x, p1.y) };
      Float32 lo = 0.0f, hi = 0.0f;
      for(Int32 i = 0; i < 4; ++i)
      {
          Float32 a = std::atan2(corner[i].y - c.y, corner[i].x - c.x) - ac;
          while(a >  PI)
          {
              a -= 2.0f * PI;
          }
          while(a < -PI)
          {
              a += 2.0f * PI;
          }
          lo = std::min(lo, a);
          hi = std::max(hi, a);
      }

      const Float32 span = hi - lo;
      if(span <= 0.0f)
      {
          return;
      }

      Int32 seg = static_cast<Int32>((span * r / 12.0f));          // ~12 px chords
      seg = static_cast<Int32>(clampf(static_cast<Float32>(seg), 8.0f, 512.0f));

      dl->PathArcTo(c, r, ac + lo, ac + hi, seg);
      dl->PathStroke(col, th);
  }

  // strokeRing, but the blind disc is small so the whole ring is cheap either way.
  Void dashedRing(ImDrawList* dl, const ImVec2& c, Float32 r, ImU32 col, Float32 th, Float32 dashPx)
  {
      if(r <= 1.5f)
      {
          return;
      }

      Int32 dashes = static_cast<Int32>((2.0f * PI * r / (dashPx * 2.0f)));
      dashes = static_cast<Int32>(clampf(static_cast<Float32>(dashes), 8.0f, 72.0f));

      const Float32 sweep = 2.0f * PI / static_cast<Float32>(dashes);
      for(Int32 i = 0; i < dashes; ++i)
      {
          const Float32 a0 = static_cast<Float32>(i) * sweep;
          dl->PathArcTo(c, r, a0, a0 + sweep * 0.55f, 3);
          dl->PathStroke(col, th);
      }
  }

  // 45-degree hatching clipped to a disc, by chord. Line count is capped so this
  // stays a fixed small cost however far the disc is zoomed in.
  Void hatchDisc(ImDrawList* dl, const ImVec2& c, Float32 r, ImU32 col, Float32 th, Float32 spacing)
  {
      if(r <= 3.0f)
      {
          return;
      }

      Float32 h = std::max(spacing, 2.0f * r / 22.0f);
      const Float32 k = 0.70710678f;                 // cos/sin 45 deg

      for(Float32 d = -r + h * 0.5f; d < r; d += h)
      {
          const Float32 half = std::sqrt(std::max(r * r - d * d, 0.0f));
          if(half < 1.0f)
          {
              continue;
          }

          // n = (-k, k) is the offset direction, u = (k, k) runs along the line.
          const Float32 mx = c.x - k * d;
          const Float32 my = c.y + k * d;
          dl->AddLine(ImVec2(mx - k * half, my - k * half),
                      ImVec2(mx + k * half, my + k * half), col, th);
      }
  }

  // plate behind a label, so numbers survive being drawn over a dense point
  // cluster. `tl` is the text's top-left.
  //
  // Radius tracks the UI's frame rounding rather than being its own number: the
  // map sits inside the app, and a 4px plate against 2px chrome reads as a
  // different toolkit showing through.
  Void plate(ImDrawList* dl, const ImVec2& tl, const ImVec2& ts, Float32 dpi)
  {
      const Float32 px = 5.0f * dpi;
      const Float32 py = 2.5f * dpi;
      const ImVec2  a(tl.x - px, tl.y - py);
      const ImVec2  b(tl.x + ts.x + px, tl.y + ts.y + py);
      const Float32 r = ImGui::GetStyle().FrameRounding;

      static_cast<Void>(r);

      // A hole punched in the map, not a tag screwed to it. Square corners, no
      // bevel: the plate exists so a number stays readable over a dense cluster,
      // and the moment it has highlights of its own it is competing with the data
      // it was added to protect.
      dl->AddRectFilled(a, b, PLATE_BG);
  }

  // Text with a plate behind it, anchored by its top-left corner.
  Void plateText(ImDrawList* dl, const ImVec2& pos, Float32 dpi, ImU32 col, const Char* txt)
  {
      ImFont*      font = labelFont();
      const Float32  fs   = labelPx();
      const ImVec2 ts   = font->CalcTextSizeA(fs, FLT_MAX, 0.0f, txt);

      plate(dl, pos, ts, dpi);
      dl->AddText(font, fs, pos, col, txt);
  }

  // Text with a plate behind it, anchored by its center. Returns false without
  // drawing when the plate would not fit inside [p0,p1].
  Bool plateTextAt(ImDrawList* dl, const ImVec2& mid, const Rect& area, Float32 dpi, ImU32 col, const Char* txt)
  {
      const ImVec2& p0 = area.p0;
      const ImVec2& p1 = area.p1;
      ImFont*      font = labelFont();
      const Float32  fs   = labelPx();
      const ImVec2 ts   = font->CalcTextSizeA(fs, FLT_MAX, 0.0f, txt);
      const ImVec2 tl(mid.x - ts.x * 0.5f, mid.y - ts.y * 0.5f);

      const Float32 px = 5.0f * dpi;
      const Float32 py = 2.5f * dpi;
      if(tl.x - px < p0.x || tl.y - py < p0.y ||
          tl.x + ts.x + px > p1.x || tl.y + ts.y + py > p1.y)
      {
          return false;
      }

      plate(dl, tl, ts, dpi);
      dl->AddText(font, fs, tl, col, txt);
      return true;
  }

  // ----------------------------------------------------------------- pieces ---

  // Which rings exist this frame, and which one carries the compass. Computed
  // once, then consumed by the lines pass (under the points) and the labels pass
  // (over them) so numbers are never buried by a dense cluster.
  struct GridSpec
  {
      Bool  on        = false;
      Float32 stepMm   = 0.0f;
      Float32 stepPx   = 0.0f;
      Int32   i0        = 1;
      Int32   i1        = 0;
      Int32   compassI = 1;
      Float32 compassR = 0.0f;     // px, from the sensor
      Float32 ppm       = 0.0f;     // px per mm, for range-limit drawing
      Bool  centered   = false;    // is the sensor itself inside the widget?
  };

  // `s0` is where the sensor lands on screen; it may be well outside [p0,p1].
  GridSpec computeGrid(const Rect& area, const MapScale& sc, Float32 visibleMm, Float32 radiusPx)
  {
      const ImVec2& p0 = area.p0;
      const ImVec2& p1 = area.p1;
      const ImVec2& s0 = sc.s0;
      const Float32 ppm = sc.pxPerMm;
      GridSpec g;

      g.ppm     = ppm;
      g.stepMm = niceStep(visibleMm / 4.0f);
      g.stepPx = g.stepMm * ppm;
      if(!(g.stepPx > 0.5f))
      {
          return g;
      }

      // Ring index window: from the nearest point of the rect to its farthest
      // corner. Those differ by at most the rect diagonal, so the count is
      // naturally bounded by the widget size, not by how far the view is panned.
      const Float32 nx = clampf(s0.x, p0.x, p1.x);
      const Float32 ny = clampf(s0.y, p0.y, p1.y);
      const Float32 nearD = std::sqrt((nx - s0.x) * (nx - s0.x) + (ny - s0.y) * (ny - s0.y));

      g.centered = (nearD <= 0.0f);

      Float32 farD = 0.0f;
      const ImVec2 corner[4] = { p0, ImVec2(p1.x, p0.y), p1, ImVec2(p0.x, p1.y) };
      for(Int32 i = 0; i < 4; ++i)
      {
          const Float32 dx = corner[i].x - s0.x;
          const Float32 dy = corner[i].y - s0.y;
          farD = std::max(farD, std::sqrt(dx * dx + dy * dy));
      }

      g.i0 = static_cast<Int32>(std::floor(nearD / g.stepPx));
      g.i1 = static_cast<Int32>(std::ceil(farD / g.stepPx));
      if(g.i0 < 1)
      {
          g.i0 = 1;
      }
      if(g.i1 > g.i0 + 32)
      {
          g.i1 = g.i0 + 32;
      }

      // The outermost ring that still fits inside the fitted radius carries the
      // compass rose. Rounding to nearest instead would let it poke past the top
      // and bottom of the widget, taking its ticks and its 0 / 180 labels with it.
      g.compassI = static_cast<Int32>(std::floor(radiusPx / g.stepPx));
      if(g.compassI < 1)
      {
          g.compassI = 1;
      }
      g.compassR = static_cast<Float32>(g.compassI) * g.stepPx;

      g.on = true;
      return g;
  }

  // Screen-space unit vector for a bearing in degrees: 0 is up, clockwise.
  inline ImVec2 bearingDir(Float32 deg)
  {
      const Float32 a = deg * (PI / 180.0f);
      return ImVec2(std::sin(a), -std::cos(a));
  }

  // Meter squares, axis-aligned on the sensor, with the same 1/2/5 step ladder
  // the rings use so the two grids never disagree about scale.
  //
  // Drawn from the sensor outward rather than from the widget corner, so the
  // origin always lands on a line: a grid whose lines are at arbitrary offsets
  // from the thing everything is measured from is decoration.
  Void drawGridCartesian(ImDrawList* dl, const GridSpec& g, const Rect& area, const MapScale& sc)
  {
      const ImVec2& p0 = area.p0;
      const ImVec2& p1 = area.p1;
      const ImVec2& s0 = sc.s0;
      const Float32 dpi = sc.dpi;
      if(!g.on || g.stepPx < 4.0f)
      {
          return;
      }

      const Int32 nx0 = static_cast<Int32>(std::floor((p0.x - s0.x) / g.stepPx));
      const Int32 nx1 = static_cast<Int32>(std::ceil((p1.x - s0.x) / g.stepPx));
      const Int32 ny0 = static_cast<Int32>(std::floor((p0.y - s0.y) / g.stepPx));
      const Int32 ny1 = static_cast<Int32>(std::ceil((p1.y - s0.y) / g.stepPx));

      // A sane bound: a step of a few pixels over a wide widget could otherwise
      // ask for thousands of lines.
      if((nx1 - nx0) > 400 || (ny1 - ny0) > 400)
      {
          return;
      }

      for(Int32 i = nx0; i <= nx1; ++i)
      {
          const Float32 x = s0.x + static_cast<Float32>(i) * g.stepPx;
          if(x < p0.x || x > p1.x)
          {
              continue;
          }
          const Bool major = (i % 5) == 0;
          dl->AddLine(ImVec2(x, p0.y), ImVec2(x, p1.y),
                      (i == 0) ? RING_MAJOR : (major ? RING_MAJOR : RING_COL),
                      ((i == 0) ? 1.6f : (major ? 1.3f : 1.0f)) * dpi);
      }
      for(Int32 j = ny0; j <= ny1; ++j)
      {
          const Float32 y = s0.y + static_cast<Float32>(j) * g.stepPx;
          if(y < p0.y || y > p1.y)
          {
              continue;
          }
          const Bool major = (j % 5) == 0;
          dl->AddLine(ImVec2(p0.x, y), ImVec2(p1.x, y),
                      (j == 0) ? RING_MAJOR : (major ? RING_MAJOR : RING_COL),
                      ((j == 0) ? 1.6f : (major ? 1.3f : 1.0f)) * dpi);
      }
  }

  // Distance labels for the Cartesian grid, along the two axes through the
  // sensor. Only the major lines get one - a number on every meter line is a wall
  // of text on a grid whose whole point is to be read at a glance.
  Void drawGridCartesianLabels(ImDrawList* dl, const GridSpec& g, const Rect& area, const MapScale& sc)
  {
      const ImVec2& p0 = area.p0;
      const ImVec2& p1 = area.p1;
      const ImVec2& s0 = sc.s0;
      const Float32 dpi = sc.dpi;
      if(!g.on || g.stepPx < 18.0f)
      {
          return;
      }

      ImFont*       f  = labelFont();
      const Float32 fs = labelPx();

      for(Int32 i = -40; i <= 40; ++i)
      {
          if(i == 0 || (i % 5) != 0)
          {
              continue;
          }
          const Float32 x = s0.x + static_cast<Float32>(i) * g.stepPx;
          if(x < p0.x + 8.0f * dpi || x > p1.x - 8.0f * dpi)
          {
              continue;
          }

          Array<Char, 24> buf;
          formatRing(buf.data(), buf.size(), std::fabs(static_cast<Float32>(i)) * g.stepMm);
          const ImVec2 ts = f->CalcTextSizeA(fs, FLT_MAX, 0.0f, buf.data());

          const Float32 y = clampf(s0.y, p0.y + 4.0f * dpi, p1.y - ts.y - 4.0f * dpi);
          plateText(dl, ImVec2(x - ts.x * 0.5f, y + 3.0f * dpi), dpi, RING_TEXT_COL, buf.data());
      }

      for(Int32 j = -40; j <= 40; ++j)
      {
          if(j == 0 || (j % 5) != 0)
          {
              continue;
          }
          const Float32 y = s0.y + static_cast<Float32>(j) * g.stepPx;
          if(y < p0.y + 8.0f * dpi || y > p1.y - 8.0f * dpi)
          {
              continue;
          }

          Array<Char, 24> buf;
          formatRing(buf.data(), buf.size(), std::fabs(static_cast<Float32>(j)) * g.stepMm);
          const ImVec2 ts = f->CalcTextSizeA(fs, FLT_MAX, 0.0f, buf.data());

          const Float32 x = clampf(s0.x, p0.x + 4.0f * dpi, p1.x - ts.x - 4.0f * dpi);
          plateText(dl, ImVec2(x + 4.0f * dpi, y - ts.y * 0.5f), dpi, RING_TEXT_COL, buf.data());
      }
  }

  // Rings, axes and compass ticks. Drawn beneath the point cloud.
  Void drawGridLines(ImDrawList* dl, const GridSpec& g, const Rect& area, const MapScale& sc)
  {
      const ImVec2& p0 = area.p0;
      const ImVec2& p1 = area.p1;
      const ImVec2& s0 = sc.s0;
      const Float32 dpi = sc.dpi;
      if(!g.on)
      {
          return;
      }

      // Axes through the sensor, spanning the widget.
      if(s0.y >= p0.y && s0.y <= p1.y)
      {
          dl->AddLine(ImVec2(p0.x, s0.y), ImVec2(p1.x, s0.y), AXIS_COL, 1.0f * dpi);
      }
      if(s0.x >= p0.x && s0.x <= p1.x)
      {
          dl->AddLine(ImVec2(s0.x, p0.y), ImVec2(s0.x, p1.y), AXIS_COL, 1.0f * dpi);
      }

      // The device's 12 m spec limit. Dashed, matching the blind zone's treatment
      // at the other end of the range, so the pair reads as one envelope: nothing
      // is drawn inside the inner disc or outside this ring.
      {
          const Float32 r = MAX_VALID_MM * g.ppm;
          if(r > 8.0f * dpi &&
              !(s0.x + r < p0.x || s0.x - r > p1.x || s0.y + r < p0.y || s0.y - r > p1.y))
          {
              dashedRing(dl, s0, r, RANGE_LIMIT_COL, 1.4f * dpi, 7.0f * dpi);
          }
      }

      // Emphasis, brightest first: the compass ring (the fitted range), then every
      // fifth ring inside it, then the ordinary rings, then the slivers beyond the
      // fitted range that only ever clip a corner.
      for(Int32 i = g.i0; i <= g.i1; ++i)
      {
          // The faint tier only applies while the sensor is on screen and the
          // compass ring is therefore meaningful; once it has been panned or
          // zoomed away, every visible ring is carrying the reading.
          ImU32 col; Float32 th;
          if(!g.centered)       { col = ((i % 5) == 0) ? RING_MAJOR : RING_COL;
                                       th  = ((i % 5) == 0) ? 1.3f : 1.0f; }
          else if(i == g.compassI)
          {
              col = RING_MAJOR;
              th = 1.7f;
          }
          else if(i >  g.compassI)
          {
              col = RING_FAINT;
              th = 1.0f;
          }
          else if((i % 5) == 0)
          {
              col = RING_MAJOR;
              th = 1.3f;
          }
          else
          {
              col = RING_COL;
              th = 1.0f;
          }

          strokeRing(dl, s0, static_cast<Float32>(i) * g.stepPx, area, col, th * dpi);
      }

      // Bearing ticks around the compass ring: every 15 deg, longer every 45.
      // They point outwards so they never add clutter inside the map.
      if(g.compassR > 6.0f)
      {
          for(Int32 b = 0; b < 360; b += 15)
          {
              const ImVec2 d = bearingDir(static_cast<Float32>(b));
              const ImVec2 a(s0.x + d.x * g.compassR, s0.y + d.y * g.compassR);
              if(a.x < p0.x || a.x > p1.x || a.y < p0.y || a.y > p1.y)
              {
                  continue;
              }

              const Bool  cardinal = (b % 90) == 0;
              const Bool  major    = (b % 45) == 0;
              const Float32 len      = (major ? 9.0f : 5.0f) * dpi;
              const ImU32 col      = (b == 0) ? HEADING_COL
                                              : (major ? TICK_MAJOR_COL : TICK_COL);

              dl->AddLine(a, ImVec2(a.x + d.x * len, a.y + d.y * len),
                          col, (cardinal ? 2.0f : 1.2f) * dpi);
          }
      }
  }

  // Range-ring distances and bearing numbers. Drawn over the point cloud, each on
  // its own plate.
  //
  // Two rects: ring labels hunt for somewhere to land and so are confined to
  // [p0,p1], an inset kept clear of the app's own status text. Bearing numbers sit
  // at fixed points on the compass ring - the widget's vertical midline, where
  // nothing else is drawn - so they may use the full widget rect [f0,f1] and stay
  // visible when the compass ring reaches the top and bottom edges.
  Void drawGridLabels(ImDrawList* dl, const GridSpec& g, const Rect& area, const Rect& fit, const MapScale& sc)
  {
      const ImVec2& s0 = sc.s0;
      const Float32 dpi = sc.dpi;
      if(!g.on)
      {
          return;
      }

      // Ring distances, all on one bearing so they read as a column. When a ring
      // is too big for that bearing to still be on screen, the search sweeps
      // *clockwise* down the right-hand side rather than jumping about, so the
      // numbers stay in reading order however far out they go; only if the whole
      // right side is off-widget does it mirror to the left.
      ImFont*      font   = labelFont();
      const Float32  fs     = labelPx();
      const ImVec2 probe  = font->CalcTextSizeA(fs, FLT_MAX, 0.0f, "0.0 m");
      const Bool   room   = g.stepPx > probe.y * 1.9f;   // else the column collides

      if(room)
      {
          for(Int32 i = g.i0; i <= g.i1; ++i)
          {
              const Float32 r = static_cast<Float32>(i) * g.stepPx;
              if(r < 10.0f * dpi)
              {
                  continue;
              }

              Array<Char, 24> buf;
              formatRing(buf.data(), buf.size(), static_cast<Float32>(i) * g.stepMm);

              for(Int32 k = 0; k < 22; ++k)
              {
                  const Float32 sweep = static_cast<Float32>((k % 11)) * 15.0f;
                  const Float32 sign  = (k < 11) ? 1.0f : -1.0f;
                  const ImVec2 d = bearingDir(sign * (RING_LABEL_BEARING + sweep));

                  if(plateTextAt(dl, ImVec2(s0.x + d.x * r, s0.y + d.y * r),
                                  area, dpi, RING_TEXT_COL, buf.data()))
                  {
                      break;
                  }
              }
          }
      }

      // Bearing numbers, inside the compass ring so they stay clear of the
      // widget edge at every zoom.
      if(g.compassR > 34.0f * dpi)
      {
          const Float32 rl = g.compassR - 21.0f * dpi;

          for(Int32 b = 0; b < 360; b += 45)
          {
              Array<Char, 8> buf;
              std::snprintf(buf.data(), buf.size(), "%d", b);

              const ImVec2 d = bearingDir(static_cast<Float32>(b));
              const ImU32  col = (b == 0)        ? HEADING_COL
                               : ((b % 90) == 0) ? CARDINAL_COL
                                                 : BEARING_COL;

              plateTextAt(dl, ImVec2(s0.x + d.x * rl, s0.y + d.y * rl),
                          fit, dpi, col, buf.data());
          }
      }
  }

  // The sensor's dead zone: nothing inside MIN_VALID_MM is real. Hatched rather
  // than merely empty, so it reads as "cannot see here". Drawn true to scale - see
  // BLIND_MIN_PX for why it is never inflated to a legible minimum.
  Void drawBlindZone(ImDrawList* dl, const Rect& area, const MapScale& sc)
  {
      const ImVec2& p0 = area.p0;
      const ImVec2& p1 = area.p1;
      const ImVec2& s0 = sc.s0;
      const Float32 ppm = sc.pxPerMm;
      const Float32 dpi = sc.dpi;
      const Float32 r = MIN_VALID_MM * ppm;

      // Too small to read as a region; the hub already occupies this area.
      if(r < BLIND_MIN_PX * dpi)
      {
          return;
      }

      if(s0.x + r < p0.x || s0.x - r > p1.x || s0.y + r < p0.y || s0.y - r > p1.y)
      {
          return;
      }

      // The transmitter's own shadow: the region the C1 cannot see inside, which
      // is not a hazard and not a surface. It stays black like everything else and
      // the only marks on it are red - the color this palette already spends on
      // "there is nothing here".
      hatchDisc(dl, s0, r, BLIND_HATCH, 1.0f * dpi, 7.0f * dpi);
      dashedRing(dl, s0, r, BLIND_EDGE, 1.4f * dpi, 5.0f * dpi);
  }

  // Its caption, drawn with the rest of the labels so it lands over the points.
  //
  // Only ever drawn *inside* the disc, and only once the disc is genuinely wide
  // enough to hold it. Hanging it off the outside instead would put it exactly
  // where the nearest-return readout lands whenever the closest obstacle is near
  // the sensor - which, at the zoom levels where this caption shows at all, is
  // most of the time.
  Void drawBlindLabel(ImDrawList* dl, const Rect& area, const MapScale& sc)
  {
      const ImVec2& s0 = sc.s0;
      const Float32 ppm = sc.pxPerMm;
      const Float32 dpi = sc.dpi;
      const Float32 r = MIN_VALID_MM * ppm;

      ImFont*      font = labelFont();
      const Float32  fs   = labelPx();
      const Char*  txt  = "blind zone  < 50 mm";
      const ImVec2 ts   = font->CalcTextSizeA(fs, FLT_MAX, 0.0f, txt);

      if(ts.x + 14.0f * dpi > r * 1.55f)
      {
          return;
      }

      plateTextAt(dl, ImVec2(s0.x, s0.y + r * 0.42f), area, dpi, BLIND_TEXT, txt);
  }

  // Which way the unit is physically pointing. The C1 has an arrow molded on its
  // housing marking the front, the SDK reports that direction as angle 0, and this
  // map puts 0 up - so this arrow lines up with the molded one when the device is
  // oriented the same way as the map.
  //
  // Deliberately a thin debug-overlay stroke: a long shaft plus an open head, no
  // fill, no plate and no caption - the bearing numbers around the compass ring
  // already say where 0 is. Anchored on the sensor's world origin, so it tracks
  // correctly once the sensor is panned off-center.
  Void drawHeadingArrow(ImDrawList* dl, const Rect& area, const MapScale& sc, Float32 radiusPx)
  {
      const ImVec2& p0 = area.p0;
      const ImVec2& p1 = area.p1;
      const ImVec2& s0 = sc.s0;
      const Float32 ppm = sc.pxPerMm;
      const Float32 dpi = sc.dpi;
      const ImVec2 d = bearingDir(0.0f);           // straight up on screen

      // A fixed fraction of the fitted radius: the arrow is an orientation cue,
      // not a measurement, so it should keep the same commanding size at every
      // zoom rather than growing and shrinking with the range.
      const Float32 len = clampf(radiusPx * 0.58f, 64.0f * dpi, 280.0f * dpi);

      // Start clear of the blind disc so the two do not overlap into a blob.
      const Float32 r0 = std::max(MIN_VALID_MM * ppm, BLIND_MIN_PX * dpi) + 5.0f * dpi;
      if(len <= r0 + 12.0f * dpi)
      {
          return;
      }

      const ImVec2 a(s0.x + d.x * r0,  s0.y + d.y * r0);
      const ImVec2 t(s0.x + d.x * len, s0.y + d.y * len);

      // Cheap reject when the whole thing is off-widget.
      const Float32 pad = 40.0f * dpi;
      if(std::max(a.x, t.x) < p0.x - pad || std::min(a.x, t.x) > p1.x + pad ||
          std::max(a.y, t.y) < p0.y - pad || std::min(a.y, t.y) > p1.y + pad)
      {
          return;
      }

      const Float32 th   = 2.0f * dpi;
      const Float32 head = std::min(30.0f * dpi, (len - r0) * 0.30f);

      // Emissive rather than drawn: the arrow is an indicator, so it gets the
      // same treatment as a lit lamp - a wide dim pass under a narrow bright one -
      // instead of being a flat stroke like the grid it sits on.
      const ImU32 glow = (HEADING_COL & 0x00FFFFFFu)
                       | (static_cast<ImU32>(34u) << IM_COL32_A_SHIFT);
      dl->AddLine(a, t, glow, th * 3.2f);
      dl->AddLine(a, t, HEADING_COL, th);

      // Open head: two strokes swept back from the tip at +/-26 deg.
      for(Int32 s = -1; s <= 1; s += 2)
      {
          const ImVec2 b = bearingDir(180.0f + static_cast<Float32>(s) * 26.0f);
          const ImVec2 e(t.x + b.x * head, t.y + b.y * head);
          dl->AddLine(t, e, glow, th * 3.2f);
          dl->AddLine(t, e, HEADING_COL, th);
      }
  }

  // Labeled bar in the bottom-left corner. Unlike the rings this stays useful
  // when the sensor has been panned right out of the widget.
  Void drawScaleBar(ImDrawList* dl, const Rect& area, const MapScale& sc)
  {
      const ImVec2& p0 = area.p0;
      const ImVec2& p1 = area.p1;
      const Float32 ppm = sc.pxPerMm;
      const Float32 dpi = sc.dpi;
      if(!(ppm > 0.0f))
      {
          return;
      }

      const Float32 budget = std::min((p1.x - p0.x) * 0.24f, 200.0f * dpi);
      if(budget < 30.0f * dpi)
      {
          return;
      }

      const Float32 lenMm = niceStepDown(budget / ppm);
      const Float32 lenPx = lenMm * ppm;
      if(!(lenPx > 8.0f))
      {
          return;
      }

      const Float32 x0  = p0.x + 4.0f * dpi;
      const Float32 y   = p1.y - 12.0f * dpi;
      const Float32 x1  = x0 + lenPx;
      const Float32 cap = 5.0f * dpi;
      const Float32 th  = 1.8f * dpi;

      // Half-filled, like a map scale: the midpoint tick is the free half-value.
      const ImVec2 barA(x0, y - cap * 0.5f);
      const ImVec2 barB(x1, y + cap * 0.5f);

      dl->AddRectFilled(barA, ImVec2(x0 + lenPx * 0.5f, barB.y), SCALE_COL);
      dl->AddRect(barA, barB, SCALE_COL, 0.0f, 0, th);
      dl->AddLine(ImVec2(x0, y - cap * 1.6f), ImVec2(x0, y + cap * 1.6f), SCALE_COL, th);
      dl->AddLine(ImVec2(x1, y - cap * 1.6f), ImVec2(x1, y + cap * 1.6f), SCALE_COL, th);

      Array<Char, 24> buf;
      formatRing(buf.data(), buf.size(), lenMm);

      ImFont*      font = labelFont();
      const ImVec2 ts   = font->CalcTextSizeA(labelPx(), FLT_MAX, 0.0f, buf.data());
      plateText(dl, ImVec2(x0, y - cap * 1.8f - ts.y), dpi, SCALE_COL, buf.data());
  }

  // Projects one revolution into screen space, dropping "no return" samples and
  // anything outside the widget. At high zoom this is what keeps the submitted
  // geometry proportional to what is actually visible.
  Void collectDots(const Vec<LidarPoint>& pts, const MapScale& sc, const Rect& cull, Vec<Dot>& out)
  {
      const ImVec2& s0  = sc.s0;
      const Float32 ppm = sc.pxPerMm;
      const ImVec2& lo  = cull.p0;
      const ImVec2& hi  = cull.p1;
      out.clear();
      if(pts.empty())
      {
          return;
      }
      out.reserve(pts.size());

      const Float32 deg2rad = PI / 180.0f;

      for(const LidarPoint& p : pts)
      {
          const Float32 d = p.distMm;

          // Below MIN_VALID_MM the device is inside its own spec floor: those are
          // housing reflections, and every readout already discards them. Drawing
          // them anyway put dots inside the disc marked "cannot see here", which
          // is exactly the contradiction the blind zone exists to prevent.
          // (0 mm means "no return" and is caught by the same test.) The upper
          // bound is the device's 12 m spec limit - see MAX_VALID_MM.
          if(!(d >= MIN_VALID_MM) || d > MAX_VALID_MM)
          {
              continue;
          }

          const Float32 rr  = d * ppm;
          const Float32 ang = (p.angleDeg - 90.0f) * deg2rad;   // == (sin a, -cos a)
          const Float32 x   = s0.x + rr * std::cos(ang);
          const Float32 y   = s0.y + rr * std::sin(ang);

          if(x < lo.x || x > hi.x || y < lo.y || y > hi.y)
          {
              continue;
          }

          out.push_back(Dot{ x, y });
      }
  }

  // ------------------------------------------------------------ render modes ---

  // Everything the return-rendering paths share, so five of them do not each take
  // nine arguments. Purely display state: nothing here feeds a readout.
  struct MarkCtx
  {
      ImDrawList* dl      = nullptr;
      ImVec2      p0, p1;             // widget rect
      ImVec2      cullLo, cullHi;   // rect grown by the dot radius
      ImVec2      s0;                 // sensor, in screen space
      ImVec2      uv;
      Float32       ppm   = 0.0f;
      Float32       dpi   = 1.0f;
      Float32       dotR = 2.0f;

      // What to draw at the origin. See RadarView::ego.
      scene3d::EgoView ego = scene3d::EgoView::EGO_VIEW_CAR;

      // Where the active mode writes its one-line reading. Never null.
      Char*       diag    = nullptr;
      Size        diagCap = 0;
  };

  // Small helper so a mode can report its number without repeating the guard.
  template<typename... Args>
  Void say(const MarkCtx& c, const Char* fmt, Args... args)
  {
      if(c.diag != nullptr && c.diagCap > 0)
      {
          std::snprintf(c.diag, c.diagCap, fmt, args...);
      }
  }

  // MapMode::MAP_MODE_POINTS. Unchanged from the flat-dot renderer this file has always
  // had, and deliberately so: it is the verified default, and the other modes are
  // alternatives to it rather than revisions of it.
  Void drawMarksPoints(const MarkCtx& c, const Deque<Vec<LidarPoint>>& trail, Bool showTrail)
  {
      const Int32 last = static_cast<Int32>(trail.size()) - 1;
      Vec<Dot>& dots = scratch();

      // Older revolutions fade out behind the current one. Same flat color,
      // lower alpha and a slightly smaller dot: the trail is context, the
      // latest revolution is the reading.
      if(showTrail && last > 0)
      {
          for(Int32 i = 0; i < last; ++i)
          {
              const Float32 a  = 0.07f + 0.13f * (static_cast<Float32>(i) / static_cast<Float32>(std::max(1, last)));
              const Int32   a8 = static_cast<Int32>((clampf(a, 0.0f, 1.0f) * 255.0f + 0.5f));
              if(a8 <= 0)
              {
                  continue;
              }

              collectDots(trail[static_cast<Size>(i)], MapScale{ c.s0, c.ppm, c.dpi },
                  Rect{ c.cullLo, c.cullHi }, dots);
              emitDiscs(c.dl, dots.data(), static_cast<Int32>(dots.size()), 1.6f * c.dpi,
                        POINT_RGB | (static_cast<ImU32>(a8) << IM_COL32_A_SHIFT), c.uv);
          }
      }

      collectDots(trail[static_cast<Size>(last)], MapScale{ c.s0, c.ppm, c.dpi },
                  Rect{ c.cullLo, c.cullHi }, dots);
      emitDiscs(c.dl, dots.data(), static_cast<Int32>(dots.size()), c.dotR,
                POINT_RGB | (static_cast<ImU32>(255u) << IM_COL32_A_SHIFT), c.uv);
  }
  // Grid index window covering the widget rect. Returns false when the grid does
  // not reach the view at all - which it will not once you pan past 12 m.
  Bool visibleCellRange(const MarkCtx& c, Int32& ix0, Int32& iy0, Int32& ix1, Int32& iy1)
  {
      if(!(c.ppm > 0.0f))
      {
          return false;
      }

      ix0 = cellAxis((c.p0.x - c.s0.x) / c.ppm);
      ix1 = cellAxis((c.p1.x - c.s0.x) / c.ppm);
      iy0 = cellAxis((c.p0.y - c.s0.y) / c.ppm);
      iy1 = cellAxis((c.p1.y - c.s0.y) / c.ppm);

      if(ix1 < 0 || iy1 < 0 || ix0 >= GRID_N || iy0 >= GRID_N)
      {
          return false;
      }

      ix0 = std::max(ix0, 0);
      iy0 = std::max(iy0, 0);
      ix1 = std::min(ix1, GRID_N - 1);
      iy1 = std::min(iy1, GRID_N - 1);
      return ix0 <= ix1 && iy0 <= iy1;
  }

  // MapMode::MAP_MODE_DENSITY. Hit count per fixed world cell over the rolling window,
  // shaded by count. A wall is hit by two or three samples of every revolution
  // and saturates; a speckle reflection is hit once and stays at the dim end.
  Void drawMarksDensity(const MarkCtx& c, const MapState& st)
  {
      if(!st.ready)
      {
          return;
      }

      Int32 ix0, iy0, ix1, iy1;
      if(!visibleCellRange(c, ix0, iy0, ix1, iy1))
      {
          return;
      }

      const Float32 cellPx = CELL_MM * c.ppm;

      // Zoomed right out a cell is under a pixel and would disappear entirely.
      // Widening it to one pixel is a sub-pixel lie about area, and the only one
      // taken anywhere in this mode; the alternative is a mode that looks empty.
      const Float32 pad = std::max(0.0f, (1.0f - cellPx) * 0.5f);
      const Float32 ext = cellPx + pad * 2.0f;

      const Ramp& ramp = heatRamp();

      Vec<Cell>& out = cellScratch();
      out.clear();

      for(Int32 iy = iy0; iy <= iy1; ++iy)
      {
          const Float32  y0  = c.s0.y + static_cast<Float32>((iy - GRID_HALF)) * cellPx - pad;
          const Size row = static_cast<Size>(iy) * static_cast<Size>(GRID_N);

          for(Int32 ix = ix0; ix <= ix1; ++ix)
          {
              const UInt16 n = st.dens[row + static_cast<Size>(ix)];
              if(n == 0)
              {
                  continue;
              }

              Float32 t = static_cast<Float32>(n) / DENS_FULL;
              if(t > 1.0f)
              {
                  t = 1.0f;
              }
              t = std::sqrt(t);       // the low counts are where the detail is

              const UInt32 a8 =
                  static_cast<UInt32>((clampf(0.18f + 0.72f * t, 0.0f, 1.0f) * 255.0f + 0.5f));

              const Float32 x0 = c.s0.x + static_cast<Float32>((ix - GRID_HALF)) * cellPx - pad;

              out.push_back(Cell{ x0, y0, x0 + ext, y0 + ext,
                                  ramp.at(t) | (static_cast<ImU32>(a8) << IM_COL32_A_SHIFT) });
          }
      }

      emitCells(c.dl, out.data(), static_cast<Int32>(out.size()), c.uv);
  }
  // MapMode::MAP_MODE_CONTOUR. Adjacent returns joined into polylines, so a wall is drawn
  // as a wall rather than as the dots a wall happens to produce.
  //
  // A break is declared when consecutive returns are further apart than a gap that
  // GROWS WITH RANGE. It has to: at 0.72 deg the arc between neighboring samples
  // is ~13 mm at 1 m and ~150 mm at 12 m, so a fixed threshold either shatters
  // every distant surface or bridges across doorways up close.
  Void drawMarksContour(const MarkCtx& c, const Deque<Vec<LidarPoint>>& trail)
  {
      const Vec<LidarPoint>& pts = trail.back();
      const Int32 n = static_cast<Int32>(pts.size());
      if(n < 2)
      {
          return;
      }

      // Two samples of arc plus a fixed allowance for the C1's +/-30 mm accuracy,
      // so noise along a flat surface does not read as a break in it.
      const Float32 ARC = 2.2f * (0.72f * PI / 180.0f);

      Vec<ImVec2> run;
      run.reserve(64);

      const ImU32 col = POINT_RGB | (static_cast<ImU32>(220u) << IM_COL32_A_SHIFT);
      const Float32 th  = 1.6f * c.dpi;

      const auto flush = [&]() {
          if(run.size() >= 2)
          {
              c.dl->AddPolyline(run.data(), static_cast<Int32>(run.size()), col, 0, th);
          }
          run.clear();
      };

      Float32 prevD = 0.0f;
      Float32 prevA = 0.0f;
      Bool  havePrev = false;

      for(Int32 i = 0; i < n; ++i)
      {
          const LidarPoint& p = pts[static_cast<Size>(i)];
          if(!inWindow(p.distMm))
          {
              flush();                    // a dropout ends the surface
              havePrev = false;
              continue;
          }

          if(havePrev)
          {
              Float32 da = p.angleDeg - prevA;
              da -= std::floor(da / 360.0f + 0.5f) * 360.0f;      // to [-180, 180)

              // Law of cosines on the two polar samples, and a hard angular gate:
              // a 30 deg jump between "adjacent" samples means the revolution
              // skipped, not that the surface continued.
              const Float32 rad = da * (PI / 180.0f);
              const Float32 gap = std::sqrt(prevD * prevD + p.distMm * p.distMm
                                          - 2.0f * prevD * p.distMm * std::cos(rad));
              const Float32 lim = ARC * std::max(prevD, p.distMm) + 90.0f;

              if(gap > lim || std::fabs(da) > 30.0f)
              {
                  flush();
              }
          }

          const Float32 ang = (p.angleDeg - 90.0f) * (PI / 180.0f);
          const Float32 rr  = p.distMm * c.ppm;
          run.push_back(ImVec2(c.s0.x + rr * std::cos(ang), c.s0.y + rr * std::sin(ang)));

          prevD    = p.distMm;
          prevA    = p.angleDeg;
          havePrev = true;
      }

      flush();
  }

  // MapMode::MAP_MODE_CLEARANCE. The smoothed free-space profile as a filled polygon: how
  // far it is safe to drive on each bearing. The one mode that answers a question
  // about the EMPTY space rather than about the returns.
  //
  // The boundary is ramped by range and the fill is not, so near danger reads off
  // the outline while the drivable area stays a single flat shape.
  Void drawMarksClearance(const MarkCtx& c, const MapState& st, const Deque<Vec<LidarPoint>>& trail)
  {
      if(!st.ready)
      {
          return;
      }

      ImVec2 pt[CLR_BINS];
      Bool   any = false;
      for(Int32 i = 0; i < CLR_BINS; ++i)
      {
          if(st.clrSeen[i])
          {
              any = true;
          }

          const Float32 d   = st.clrSeen[i] ? st.clr[i] : 0.0f;
          const Float32 deg = (static_cast<Float32>(i) + 0.5f) * CLR_BIN_DEG - 90.0f;
          const Float32 a   = deg * (PI / 180.0f);
          const Float32 rr  = d * c.ppm;
          pt[i] = ImVec2(c.s0.x + rr * std::cos(a), c.s0.y + rr * std::sin(a));
      }
      if(!any)
      {
          return;
      }

      emitFan(c.dl, c.s0, pt, CLR_BINS, CLEAR_RGB | (static_cast<ImU32>(44u) << IM_COL32_A_SHIFT), c.uv);

      const Ramp& ramp = rangeRamp();
      for(Int32 i = 0; i < CLR_BINS; ++i)
      {
          const Int32 j = (i + 1) % CLR_BINS;
          const Float32 t = clampf(st.clr[i] / MAX_VALID_MM, 0.0f, 1.0f);
          c.dl->AddLine(pt[i], pt[j], ramp.at(t) | (static_cast<ImU32>(210u) << IM_COL32_A_SHIFT),
                        1.8f * c.dpi);
      }

      // The live revolution over the top, dim: the polygon is a derived, smoothed
      // thing and should never be the only evidence on screen.
      Vec<Dot>& dots = scratch();
      collectDots(trail.back(), MapScale{ c.s0, c.ppm, c.dpi },
                  Rect{ c.cullLo, c.cullHi }, dots);
      emitDiscs(c.dl, dots.data(), static_cast<Int32>(dots.size()), 1.5f * c.dpi,
                POINT_RGB | (static_cast<ImU32>(90u) << IM_COL32_A_SHIFT), c.uv);
  }

  // MapMode::MAP_MODE_MOTION. Cells that were hit while the memory map said their whole
  // neighborhood was empty - so a static room is very nearly blank and anything
  // that moves through it draws a bright trail.
  //
  // This is the complement of Density: that one shows what has stayed put, this
  // one shows what has not.
  Void drawMarksMotion(const MarkCtx& c, const MapState& st, const Deque<Vec<LidarPoint>>& trail)
  {
      // Context first, and dim - an empty motion map is the correct reading for a
      // still room, but on its own it is indistinguishable from a broken one.
      Vec<Dot>& dots = scratch();
      collectDots(trail.back(), MapScale{ c.s0, c.ppm, c.dpi },
                  Rect{ c.cullLo, c.cullHi }, dots);
      emitDiscs(c.dl, dots.data(), static_cast<Int32>(dots.size()), 1.5f * c.dpi,
                POINT_RGB | (static_cast<ImU32>(70u) << IM_COL32_A_SHIFT), c.uv);

      if(!st.ready || st.motActive.empty() || !(c.ppm > 0.0f))
      {
          return;
      }

      const Float32 cellPx = CELL_MM * c.ppm;
      const Float32 pad     = std::max(0.0f, (1.0f - cellPx) * 0.5f);
      const Float32 ext     = cellPx + pad * 2.0f;

      Vec<Cell>& out = cellScratch();
      out.clear();
      out.reserve(st.motActive.size());

      for(Int32 ci : st.motActive)
      {
          const Float32 v = st.mot[static_cast<Size>(ci)];
          if(v < MOT_FLOOR)
          {
              continue;
          }

          const Int32 ix = ci % GRID_N;
          const Int32 iy = ci / GRID_N;

          const Float32 x0 = c.s0.x + static_cast<Float32>((ix - GRID_HALF)) * cellPx - pad;
          const Float32 y0 = c.s0.y + static_cast<Float32>((iy - GRID_HALF)) * cellPx - pad;

          if(x0 + ext < c.p0.x || x0 > c.p1.x || y0 + ext < c.p0.y || y0 > c.p1.y)
          {
              continue;
          }

          const UInt32 a8 =
              static_cast<UInt32>((clampf(0.15f + 0.85f * v, 0.0f, 1.0f) * 255.0f + 0.5f));

          out.push_back(Cell{ x0, y0, x0 + ext, y0 + ext,
                              MOTION_RGB | (static_cast<ImU32>(a8) << IM_COL32_A_SHIFT) });
      }

      emitCells(c.dl, out.data(), static_cast<Int32>(out.size()), c.uv);
  }
  // MapMode::MAP_MODE_GAPS. Openings wide enough to drive through.
  //
  // This is follow-the-gap, drawn. A run of adjacent bearing bins whose clearance
  // is beyond a threshold is an opening; its usable width is the CHORD across the
  // run taken at the run's NEAREST edge, because that is the width that would
  // actually have to fit - measuring at the far edge would flatter every doorway.
  Void drawMarksGaps(const MarkCtx& c, const MapState& st, const Deque<Vec<LidarPoint>>& trail)
  {
      // The live revolution underneath, dim: the gaps are derived, and a derived
      // thing should never be the only evidence on screen.
      Vec<Dot>& dots = scratch();
      collectDots(trail.back(), MapScale{ c.s0, c.ppm, c.dpi },
                  Rect{ c.cullLo, c.cullHi }, dots);
      emitDiscs(c.dl, dots.data(), static_cast<Int32>(dots.size()), 1.5f * c.dpi,
                POINT_RGB | (static_cast<ImU32>(90u) << IM_COL32_A_SHIFT), c.uv);

      if(!st.ready)
      {
          return;
      }

      // Openings start where the clearance passes this. Absolute rather than
      // relative to the scene: a gap is drivable or it is not, and that does not
      // depend on how big the room happens to be.
      constexpr Float32 GAP_OPEN_MM = 1200.0f;

      // The car is ~190 mm across; this is that plus margin for steering error.
      constexpr Float32 GAP_MIN_WIDTH_MM = 350.0f;

      Float32 bestWidth = 0.0f;
      Float32 bestDeg   = 0.0f;
      Int32   count     = 0;

      Int32 b = 0;
      while(b < CLR_BINS)
      {
          if(!st.clrSeen[b] || st.clr[b] < GAP_OPEN_MM)
          {
              ++b;
              continue;
          }

          const Int32 start = b;
          Float32 nearestMm = st.clr[b];
          while(b < CLR_BINS && st.clrSeen[b] && st.clr[b] >= GAP_OPEN_MM)
          {
              if(st.clr[b] < nearestMm)
              {
                  nearestMm = st.clr[b];
              }
              ++b;
          }
          const Int32 span = b - start;

          const Float32 spanDeg  = static_cast<Float32>(span) * CLR_BIN_DEG;
          const Float32 halfRad = spanDeg * 0.5f * (PI / 180.0f);
          const Float32 width   = 2.0f * nearestMm * std::sin(halfRad);
          if(width < GAP_MIN_WIDTH_MM)
          {
              continue;
          }

          ++count;

          const Float32 midDeg = (static_cast<Float32>(start) + span * 0.5f) * CLR_BIN_DEG;
          if(width > bestWidth)
          {
              bestWidth = width;
              bestDeg = midDeg;
          }

          // The wedge of the opening, and the chord that has to fit.
          const Float32 a0 = (static_cast<Float32>(start) * CLR_BIN_DEG - 90.0f) * (PI / 180.0f);
          const Float32 a1 = a0 + spanDeg * (PI / 180.0f);
          const Float32 rr = nearestMm * c.ppm;

          const ImVec2 e0(c.s0.x + rr * std::cos(a0), c.s0.y + rr * std::sin(a0));
          const ImVec2 e1(c.s0.x + rr * std::cos(a1), c.s0.y + rr * std::sin(a1));

          const ImU32 fill = GAP_RGB | (static_cast<ImU32>(0x26u) << IM_COL32_A_SHIFT);
          constexpr Int32 STEPS = 12;
          ImVec2 prev = e0;
          for(Int32 k = 1; k <= STEPS; ++k)
          {
              const Float32 a = a0 + (a1 - a0) * (static_cast<Float32>(k) / STEPS);
              const ImVec2 cur(c.s0.x + rr * std::cos(a), c.s0.y + rr * std::sin(a));
              c.dl->AddTriangleFilled(c.s0, prev, cur, fill);
              prev = cur;
          }

          c.dl->AddLine(e0, e1,
                        GAP_RGB | (static_cast<ImU32>(0xF0u) << IM_COL32_A_SHIFT),
                        2.2f * c.dpi);

          // Width, on the chord.
          Array<Char, 32> lab;
          std::snprintf(lab.data(), lab.size(), "%.2f m", static_cast<Float64>(width / 1000.0f));
          ImFont* f = labelFont();
          const Float32 fs = labelPx();
          const ImVec2 ts = f->CalcTextSizeA(fs, FLT_MAX, 0.0f, lab.data());
          const ImVec2 mid((e0.x + e1.x) * 0.5f, (e0.y + e1.y) * 0.5f);
          plateText(c.dl, ImVec2(mid.x - ts.x * 0.5f, mid.y - ts.y * 0.5f), c.dpi,
                    GAP_RGB | (static_cast<ImU32>(0xFFu) << IM_COL32_A_SHIFT), lab.data());
      }

      if(count > 0)
      {
          say(c, "%d gap%s, widest %.2f m at %.0f deg", count, count == 1 ? "" : "s",
              static_cast<Float64>(bestWidth / 1000.0f), static_cast<Float64>(bestDeg));
      }
      else
      {
          say(c, "no gap wider than %.2f m", static_cast<Float64>(GAP_MIN_WIDTH_MM / 1000.0f));
      }
  }
  // ---------------------------------------------------------------------------
  // MapMode::MAP_MODE_WALLS
  //
  // Straight segments FITTED to the returns, by iterative end-point fit (the
  // split-and-merge that a SLAM front end runs before it does anything else).
  //
  // One step past Contour, and the difference matters: Contour joins adjacent dots
  // and will happily trace a curve or a cloud. This asks a harder question - is
  // this run of dots actually a straight surface? - and only draws what answers
  // yes. What comes out is a landmark list, not a picture.
  //
  // The tolerance is set from the SENSOR, not from taste: the C1 is specified to
  // +/-30 mm, so anything inside ~45 mm of a straight line is a straight line as
  // far as this device can tell, and splitting there would be fitting noise.
  constexpr Float32 WALL_TOL_MM = 45.0f;
  constexpr Float32 WALL_MIN_MM = 250.0f;   // shorter than this is furniture
  constexpr Int32   WALL_MIN_PTS = 6;

  // The geometry itself lives in map_geometry.hxx, which has no ImGui in it and
  // is therefore testable - see tests/test_map_geometry.cxx. These are the same
  // types, not parallel ones: a second copy of a fitted-wall struct is a second
  // place for the definition of "corner" to drift.
  using mapgeo::WorldPt;
  using mapgeo::WallSeg;
  using mapgeo::Corner;
  using mapgeo::perpDist;

  // ---------------------------------------------------------------------------
  // Wall fitting, separated from wall DRAWING.
  //
  // Two modes need the segments now - Walls draws them, Corners intersects them -
  // and running the split-and-merge twice, or copying it, would be two ways for
  // the same scene to produce two different answers.
  // ---------------------------------------------------------------------------
  // drawEgo lives with the Full renderer below; Fit needs it too, and moving it
  // up would drag the whole vehicle-drawing block away from the mode it belongs
  // to. One declaration is the smaller cost.
  struct MarkCtx;
  Void drawEgo(const MarkCtx& c);
  inline scene3d::EgoView egoView(const MarkCtx& c)
  {
      return c.ego;
  }

  constexpr Int32 WALL_MAX = 64;

  Void fitWalls(const Vec<LidarPoint>& pts, Vec<WallSeg>& out)
  {
      out.clear();

      const Int32 n = static_cast<Int32>(pts.size());
      if(n < WALL_MIN_PTS)
      {
          return;
      }

      // Same break rule as the contour helper: a gap that grows with range,
      // because the arc between adjacent samples does.
      const Float32 arc = 2.2f * (0.72f * PI / 180.0f);

      static Vec<WorldPt> run;
      run.clear();
      run.reserve(64);

      // Explicit stack rather than recursion: a pathological run should cost
      // memory that is visible here, not stack that is not.
      static Vec<Pair<Int32, Int32>> todo;

      const auto flushRun = [&]() {
          const Int32 m = static_cast<Int32>(run.size());
          if(m >= WALL_MIN_PTS)
          {
              todo.clear();
              todo.push_back(std::make_pair(0, m - 1));

              while(!todo.empty())
              {
                  const Pair<Int32, Int32> seg = todo.back();
                  todo.pop_back();

                  const Int32 b = seg.first;
                  const Int32 e = seg.second;
                  if(e - b < 1)
                  {
                      continue;
                  }

                  Int32   worst  = -1;
                  Float32 worstD = 0.0f;
                  for(Int32 i = b + 1; i < e; ++i)
                  {
                      const Float32 d = perpDist(run[static_cast<Size>(i)],
                                                 run[static_cast<Size>(b)],
                                                 run[static_cast<Size>(e)]);
                      if(d > worstD)
                      {
                          worstD = d;
                          worst = i;
                      }
                  }

                  if(worst > 0 && worstD > WALL_TOL_MM)
                  {
                      todo.push_back(std::make_pair(b, worst));
                      todo.push_back(std::make_pair(worst, e));
                      continue;
                  }

                  // Straight enough for this device. Emit it if it is long enough
                  // to be a surface rather than an object.
                  const WorldPt& pa = run[static_cast<Size>(b)];
                  const WorldPt& pb = run[static_cast<Size>(e)];
                  const Float32 dx = pb.x - pa.x;
                  const Float32 dy = pb.y - pa.y;
                  const Float32 lenMm = std::sqrt(dx * dx + dy * dy);
                  if(lenMm < WALL_MIN_MM || (e - b + 1) < WALL_MIN_PTS)
                  {
                      continue;
                  }
                  if(static_cast<Int32>(out.size()) >= WALL_MAX)
                  {
                      continue;
                  }

                  Float32 deg = std::atan2(dx, -dy) * 180.0f / PI;   // 0 = up
                  while(deg <    0.0f)
                  {
                      deg += 180.0f;  // a wall has
                  }
                  while(deg >= 180.0f)
                  {
                      deg -= 180.0f;  // no direction
                  }

                  out.push_back(WallSeg{ pa, pb, lenMm, deg });
              }
          }
          run.clear();
      };

      Float32 prevD = 0.0f, prevA = 0.0f;
      Bool havePrev = false;

      for(Int32 i = 0; i < n; ++i)
      {
          const LidarPoint& p = pts[static_cast<Size>(i)];
          if(!inWindow(p.distMm))
          {
              flushRun();
              havePrev = false;
              continue;
          }

          if(havePrev)
          {
              Float32 da = p.angleDeg - prevA;
              da -= std::floor(da / 360.0f + 0.5f) * 360.0f;
              const Float32 rad = da * (PI / 180.0f);
              const Float32 gap = std::sqrt(prevD * prevD + p.distMm * p.distMm
                                            - 2.0f * prevD * p.distMm * std::cos(rad));
              const Float32 lim = arc * std::max(prevD, p.distMm) + 90.0f;
              if(gap > lim || std::fabs(da) > 30.0f)
              {
                  flushRun();
              }
          }

          Float32 wx, wy;
          returnWorld(p, wx, wy);
          run.push_back(WorldPt{ wx, wy });

          prevD = p.distMm;
          prevA = p.angleDeg;
          havePrev = true;
      }
      flushRun();
  }

  // MapMode::MAP_MODE_WALLS
  //
  // Straight segments FITTED to the returns, by iterative end-point fit (the
  // split-and-merge that a SLAM front end runs before it does anything else).
  //
  // The question it asks is harder than "where are the returns": is this run of
  // dots actually a straight surface? Only what answers yes is drawn. What comes
  // out is a landmark list, not a picture.
  //
  // The tolerance is set from the SENSOR, not from taste: the C1 is specified to
  // +/-30 mm, so anything inside ~45 mm of a straight line is a straight line as
  // far as this device can tell, and splitting there would be fitting noise.
  Void drawMarksWalls(const MarkCtx& c, const Deque<Vec<LidarPoint>>& trail)
  {
      const Vec<LidarPoint>& pts = trail.back();

      // The returns underneath, dim: a fitted wall is inferred, and the evidence
      // it was inferred from has to stay visible beside it.
      Vec<Dot>& dots = scratch();
      collectDots(pts, MapScale{ c.s0, c.ppm, c.dpi },
                  Rect{ c.cullLo, c.cullHi }, dots);
      emitDiscs(c.dl, dots.data(), static_cast<Int32>(dots.size()), 1.4f * c.dpi,
                POINT_RGB | (static_cast<ImU32>(70u) << IM_COL32_A_SHIFT), c.uv);

      static Vec<WallSeg> walls;
      fitWalls(pts, walls);

      Float32 longestMm = 0.0f, longestDeg = 0.0f;

      for(const WallSeg& w : walls)
      {
          if(w.lenMm > longestMm)
          {
              longestMm = w.lenMm;
              longestDeg = w.deg;
          }

          const ImVec2 sa(c.s0.x + w.a.x * c.ppm, c.s0.y + w.a.y * c.ppm);
          const ImVec2 sb(c.s0.x + w.b.x * c.ppm, c.s0.y + w.b.y * c.ppm);

          c.dl->AddLine(sa, sb,
                        WALL_RGB | (static_cast<ImU32>(0x2Cu) << IM_COL32_A_SHIFT),
                        5.0f * c.dpi);
          c.dl->AddLine(sa, sb,
                        WALL_RGB | (static_cast<ImU32>(0xF0u) << IM_COL32_A_SHIFT),
                        2.0f * c.dpi);

          // End caps, so a wall reads as a bounded segment and not as a line that
          // happens to stop.
          const Float32 ux = (w.b.x - w.a.x) / w.lenMm;
          const Float32 uy = (w.b.y - w.a.y) / w.lenMm;
          const Float32 cap = 4.0f * c.dpi;
          c.dl->AddLine(ImVec2(sa.x - uy * cap, sa.y + ux * cap),
                        ImVec2(sa.x + uy * cap, sa.y - ux * cap),
                        WALL_RGB | (static_cast<ImU32>(0xF0u) << IM_COL32_A_SHIFT),
                        2.0f * c.dpi);
          c.dl->AddLine(ImVec2(sb.x - uy * cap, sb.y + ux * cap),
                        ImVec2(sb.x + uy * cap, sb.y - ux * cap),
                        WALL_RGB | (static_cast<ImU32>(0xF0u) << IM_COL32_A_SHIFT),
                        2.0f * c.dpi);

          if(w.lenMm >= 700.0f)
          {
              Array<Char, 24> lab;
              std::snprintf(lab.data(), lab.size(), "%.2f m",
                            static_cast<Float64>(w.lenMm / 1000.0f));
              ImFont* f = labelFont();
              const Float32 fs = labelPx();
              const ImVec2 ts = f->CalcTextSizeA(fs, FLT_MAX, 0.0f, lab.data());
              plateText(c.dl,
                        ImVec2((sa.x + sb.x) * 0.5f - ts.x * 0.5f,
                               (sa.y + sb.y) * 0.5f - ts.y * 0.5f),
                        c.dpi,
                        WALL_RGB | (static_cast<ImU32>(0xFFu) << IM_COL32_A_SHIFT),
                        lab.data());
          }
      }

      const Int32 count = static_cast<Int32>(walls.size());
      if(count > 0)
      {
          say(c, "%d wall%s fitted, longest %.2f m at %.0f deg", count,
              count == 1 ? "" : "s",
              static_cast<Float64>(longestMm / 1000.0f), static_cast<Float64>(longestDeg));
      }
      else
      {
          say(c, "no straight run longer than %.2f m",
              static_cast<Float64>(WALL_MIN_MM / 1000.0f));
      }
  }

  // ---------------------------------------------------------------------------
  // MapMode::MAP_MODE_CORNERS
  //
  // Where two fitted walls meet. One step past Walls, the way Walls is one step
  // past joining the dots.
  //
  // A corner is worth its own mode because of what it is FOR. A wall constrains
  // two of the three numbers a robot needs - it fixes your distance from it and
  // your heading against it, and tells you nothing about where you are ALONG it,
  // because sliding a wall along itself leaves it looking identical. A corner does
  // not slide. It is a point landmark, and a scan-matcher keys on it for exactly
  // that reason.
  //
  // Only real intersections count, and the two tests are what make it honest:
  // the walls have to actually turn (a 5 deg join is one wall the fitter split in
  // two, not a corner), and the crossing point has to be near an END of both, not
  // somewhere off in space where two extended lines happen to meet.
  // ---------------------------------------------------------------------------

  Void drawMarksCorners(const MarkCtx& c, const Deque<Vec<LidarPoint>>& trail)
  {
      const Vec<LidarPoint>& pts = trail.back();

      Vec<Dot>& dots = scratch();
      collectDots(pts, MapScale{ c.s0, c.ppm, c.dpi },
                  Rect{ c.cullLo, c.cullHi }, dots);
      emitDiscs(c.dl, dots.data(), static_cast<Int32>(dots.size()), 1.4f * c.dpi,
                POINT_RGB | (static_cast<ImU32>(50u) << IM_COL32_A_SHIFT), c.uv);

      static Vec<WallSeg> walls;
      fitWalls(pts, walls);

      // The walls stay on screen, dim. A corner without the two surfaces that
      // produced it is a claim with its evidence deleted.
      for(const WallSeg& w : walls)
      {
          c.dl->AddLine(ImVec2(c.s0.x + w.a.x * c.ppm, c.s0.y + w.a.y * c.ppm),
                        ImVec2(c.s0.x + w.b.x * c.ppm, c.s0.y + w.b.y * c.ppm),
                        WALL_RGB | (static_cast<ImU32>(0x60u) << IM_COL32_A_SHIFT),
                        1.4f * c.dpi);
      }

      static Vec<Corner> corners;
      mapgeo::findCorners(walls, corners, MIN_VALID_MM, MAX_VALID_MM);

      const ImU32 col = ui::ansi::BRMAGENTA;

      Float32 nearestMm  = 0.0f;
      Float32 nearestDeg = 0.0f;

      for(const Corner& k : corners)
      {
          if(nearestMm <= 0.0f || k.rangeMm < nearestMm)
          {
              nearestMm  = k.rangeMm;
              nearestDeg = std::atan2(k.x, -k.y) * 180.0f / PI;
              if(nearestDeg < 0.0f)
              {
                  nearestDeg += 360.0f;
              }
          }

          const ImVec2 at(c.s0.x + k.x * c.ppm, c.s0.y + k.y * c.ppm);
          const Float32 arm = 15.0f * c.dpi;

          // The two arms, then the vertex. Drawn as the angle it IS rather than as
          // a generic marker, so a 90 deg room corner and a 40 deg pillar edge do
          // not look like the same landmark.
          c.dl->AddLine(at, ImVec2(at.x + std::cos(k.a0) * arm, at.y + std::sin(k.a0) * arm),
                        col, 2.2f * c.dpi);
          c.dl->AddLine(at, ImVec2(at.x + std::cos(k.a1) * arm, at.y + std::sin(k.a1) * arm),
                        col, 2.2f * c.dpi);

          c.dl->AddCircleFilled(at, 3.2f * c.dpi, col, 12);
          c.dl->AddCircle(at, 6.5f * c.dpi,
                          (col & 0x00FFFFFFu) | (static_cast<ImU32>(0x70u) << IM_COL32_A_SHIFT),
                          16, 1.2f * c.dpi);

          Array<Char, 16> lab;
          std::snprintf(lab.data(), lab.size(), "%.0f deg", static_cast<Float64>(k.angDeg));
          plateTextAt(c.dl, ImVec2(at.x, at.y - 16.0f * c.dpi), Rect{ c.p0, c.p1 },
                      c.dpi, col, lab.data());
      }

      const Int32 n = static_cast<Int32>(corners.size());
      if(n > 0)
      {
          say(c, "%d corner%s from %d wall%s, nearest %.2f m at %.0f deg",
              n, n == 1 ? "" : "s",
              static_cast<Int32>(walls.size()), walls.size() == 1u ? "" : "s",
              static_cast<Float64>(nearestMm / 1000.0f), static_cast<Float64>(nearestDeg));
      }
      else
      {
          say(c, "no corners: %d wall%s, none meeting at more than %.0f deg",
              static_cast<Int32>(walls.size()), walls.size() == 1u ? "" : "s",
              static_cast<Float64>(mapgeo::CORNER_MIN_DEG));
      }
  }

  // ---------------------------------------------------------------------------
  // MapMode::MAP_MODE_FIT
  //
  // Where the CAR fits, as opposed to where the beam reaches.
  //
  // Clearance answers "how far away is the nearest thing on this bearing", which
  // is a question about the sensor. This answers "how far along this bearing could
  // the car actually go", which is a question about the car, and the two give
  // different answers constantly: a 150 mm slot between a chair leg and a wall is
  // free space and is not a route. Every gap narrower than the chassis disappears
  // here, which is the entire point.
  //
  // This is obstacle inflation - configuration space - done in polar form. Instead
  // of growing every obstacle by the vehicle's half-width and then measuring, each
  // obstacle is asked directly where along a bearing it would first come within
  // half a car of the centerline. For an obstacle at (R, dth) off the bearing, the
  // center first touches it at
  //
  //     r = R cos(dth) - sqrt(halfW^2 - (R sin(dth))^2)
  //
  // and it cannot touch at all when |R sin(dth)| >= halfW - its perpendicular
  // offset from the bearing already clears the car. That second condition is what
  // keeps this cheap: it bounds how many bins can possibly block a given bearing,
  // and beyond about 60 deg nothing in spec can.
  // ---------------------------------------------------------------------------
  constexpr Int32 FIT_WINDOW_BINS = 24;    // +/-72 deg; see the note above

  // Half the width the car sweeps, plus a margin. Clearing an obstacle by the
  // width of the paint is not clearing it.
  constexpr Float32 FIT_MARGIN_MM = 30.0f;

  Void drawMarksFit(const MarkCtx& c, const MapState& st, const Deque<Vec<LidarPoint>>& trail)
  {
      if(!st.ready)
      {
          say(c, "waiting for a full revolution");
          return;
      }

      const Float32 halfW = EGO_WID_MM * 0.5f + FIT_MARGIN_MM;

      // Flattened out of MapState first: the geometry takes plain arrays so it can
      // be tested without one.
      static Array<Float32, CLR_BINS> free;
      static Array<Bool, CLR_BINS> seen0;
      for(Int32 i = 0; i < CLR_BINS; ++i)
      {
          free[i]  = st.clrSeen[i] ? st.clr[i] : 0.0f;
          seen0[i] = st.clrSeen[i];
      }

      static Array<Float32, CLR_BINS> reach;
      mapgeo::computeReach(mapgeo::PolarScan{ free.data(), seen0.data(), CLR_BINS, CLR_BIN_DEG },
                           halfW, reach.data());

      ImVec2 freePoly[CLR_BINS];
      ImVec2 fitPoly[CLR_BINS];
      for(Int32 i = 0; i < CLR_BINS; ++i)
      {
          const Float32 deg = (static_cast<Float32>(i) + 0.5f) * CLR_BIN_DEG - 90.0f;
          const Float32 a   = deg * (PI / 180.0f);
          const Float32 cs  = std::cos(a), sn = std::sin(a);
          freePoly[i] = ImVec2(c.s0.x + free[i]  * c.ppm * cs, c.s0.y + free[i]  * c.ppm * sn);
          fitPoly[i]  = ImVec2(c.s0.x + reach[i] * c.ppm * cs, c.s0.y + reach[i] * c.ppm * sn);
      }

      // What the beam reaches: an outline only, and dim. It is the thing being
      // subtracted FROM, so it must not compete with the answer.
      for(Int32 i = 0; i < CLR_BINS; ++i)
      {
          c.dl->AddLine(freePoly[i], freePoly[(i + 1) % CLR_BINS],
                        CLEAR_RGB | (static_cast<ImU32>(0x66u) << IM_COL32_A_SHIFT),
                        1.0f * c.dpi);
      }

      // Where the car fits: filled, because this is the mode's actual output.
      // Star-shaped about the sensor by construction, so the fan is exact.
      emitFan(c.dl, c.s0, fitPoly, CLR_BINS,
              GAP_RGB | (static_cast<ImU32>(38u) << IM_COL32_A_SHIFT), c.uv);
      for(Int32 i = 0; i < CLR_BINS; ++i)
      {
          c.dl->AddLine(fitPoly[i], fitPoly[(i + 1) % CLR_BINS],
                        GAP_RGB | (static_cast<ImU32>(0xE0u) << IM_COL32_A_SHIFT),
                        1.8f * c.dpi);
      }

      // The returns, so the surfaces doing the blocking are visible.
      const Vec<LidarPoint>& pts = trail.back();
      Vec<Dot>& dots = scratch();
      collectDots(pts, MapScale{ c.s0, c.ppm, c.dpi },
                  Rect{ c.cullLo, c.cullHi }, dots);
      emitDiscs(c.dl, dots.data(), static_cast<Int32>(dots.size()), 1.4f * c.dpi,
                POINT_RGB | (static_cast<ImU32>(90u) << IM_COL32_A_SHIFT), c.uv);

      // The car, so the width being subtracted is on screen next to its effect.
      drawEgo(c);

      // Bearings the car cannot enter at all. Marked, because "the polygon is
      // pinched here" is easy to miss and "you cannot go that way" is the single
      // most useful thing this mode knows.
      Int32 blocked = 0;
      for(Int32 i = 0; i < CLR_BINS; ++i)
      {
          if(reach[i] > EGO_LEN_MM * 0.5f)
          {
              continue;
          }
          ++blocked;

          const Float32 deg = (static_cast<Float32>(i) + 0.5f) * CLR_BIN_DEG - 90.0f;
          const Float32 a   = deg * (PI / 180.0f);
          // Outside the car, always. At a wide zoom a fixed screen radius sat on
          // top of the footprint and the ring of blocked bearings hid the very
          // thing whose width produced them.
          const Float32 r0  = std::max(26.0f * c.dpi,
                                       EGO_LEN_MM * 0.5f * c.ppm + 8.0f * c.dpi);
          const Float32 r1  = r0 + 9.0f * c.dpi;
          c.dl->AddLine(ImVec2(c.s0.x + std::cos(a) * r0, c.s0.y + std::sin(a) * r0),
                        ImVec2(c.s0.x + std::cos(a) * r1, c.s0.y + std::sin(a) * r1),
                        IM_COL32(0xCD, 0x00, 0x00, 0xC0), 2.0f * c.dpi);
      }

      const Float32 freeArea = mapgeo::polarArea(free.data(), CLR_BINS, CLR_BIN_DEG);
      const Float32 fitArea  = mapgeo::polarArea(reach.data(), CLR_BINS, CLR_BIN_DEG);

      // Forward reach, on the bearing the car actually points.
      const Int32 fwd = 0;    // bin 0 is centered on bearing 0 + half a bin
      say(c, "car fits in %.1f m2 of %.1f m2 free (%.0f%%)  |  %.2f m ahead  |  "
             "%d of %d bearings blocked",
          static_cast<Float64>(fitArea), static_cast<Float64>(freeArea),
          freeArea > 0.01f ? 100.0 * static_cast<Float64>(fitArea / freeArea) : 0.0,
          static_cast<Float64>(reach[fwd] / 1000.0f),
          blocked, CLR_BINS);
  }

  // ===========================================================================
  // MapMode::MAP_MODE_FULL - the field display.
  //
  // The one you point at somebody. Everything the other thirteen modes work out,
  // composited, plus three things only this mode computes: the car itself to
  // scale, the objects around it as fitted boxes, and the corridor it would drive
  // into if it went straight.
  //
  // The reference is the display an autonomous car shows its passengers - a
  // self-view, tracked objects as boxes, and a driveable path - because that is
  // the honest picture of what a scan MEANS as opposed to what it contains. A
  // point cloud tells you there are returns at these bearings; this tells you
  // there is a thing 1.2 m to your left and you have 3 m in front of you.
  //
  // Everything here is derived from the CURRENT revolution. There is no tracking
  // across frames, so a "box" is a cluster this instant and not an object with an
  // identity - which is why nothing here is numbered or given a velocity. Both
  // would be inventions, and an inventing display is worse than a plain one.
  // ===========================================================================

  // Object extraction. A cluster is a run of consecutive returns with no large
  // gap between them; the break threshold has to GROW with range because the arc
  // between two samples does - at 0.72 deg, neighbors are 63 mm apart at 5 m and
  // 151 mm apart at 12 m, so one fixed number would either shred distant objects
  // or weld together near ones.
  constexpr Int32   OBJ_MIN_PTS     = 4;
  constexpr Float32 OBJ_GAP_BASE_MM = 140.0f;
  constexpr Float32 OBJ_GAP_SLOPE   = 0.045f;    // ~3.5 sample arcs
  constexpr Float32 OBJ_MIN_EXT_MM  = 35.0f;     // smaller than this is one return
  constexpr Float32 OBJ_MAX_LEN_MM  = 2400.0f;   // longer than this is a wall
  constexpr Int32   OBJ_MAX         = 48;

  // The corridor: how far straight ahead the car could go before something enters
  // the width it sweeps. Capped rather than unbounded - past 6 m the number stops
  // being a driving decision and starts being a room measurement.
  constexpr Float32 CORRIDOR_MAX_MM  = 6000.0f;
  constexpr Float32 CORRIDOR_WARN_MM = 1500.0f;
  constexpr Float32 CORRIDOR_STOP_MM = 700.0f;

  struct Obstacle
  {
      Float32 cx = 0.0f, cy = 0.0f;      // center, mm, sensor frame (y is DOWN)
      Float32 ux = 1.0f, uy = 0.0f;      // principal axis, unit
      Float32 halfL = 0.0f, halfW = 0.0f;
      Float32 nearMm = 0.0f;             // closest return in the cluster
      Bool    inPath = false;
      Int32   n = 0;
  };

  // Oriented bounding box of a run of points, by principal axis.
  //
  // PCA rather than a min-area rotating-calipers fit: the returns come off ONE
  // side of an object, so the true minimum-area box is fitted to a partial
  // outline and is not more correct - it is just more expensive and less stable
  // frame to frame, which on a live display is the thing that matters.
  Bool fitObstacle(const WorldPt* p, Int32 m, Obstacle& out)
  {
      if(m < OBJ_MIN_PTS)
      {
          return false;
      }

      const Float32 fm = static_cast<Float32>(m);
      Float32 mx = 0.0f, my = 0.0f;
      for(Int32 i = 0; i < m; ++i)
      {
          mx += p[i].x;
          my += p[i].y;
      }
      mx /= fm; my /= fm;

      Float32 sxx = 0.0f, sxy = 0.0f, syy = 0.0f;
      for(Int32 i = 0; i < m; ++i)
      {
          const Float32 dx = p[i].x - mx, dy = p[i].y - my;
          sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
      }

      const Float32 th = 0.5f * std::atan2(2.0f * sxy, sxx - syy);
      const Float32 ux = std::cos(th), uy = std::sin(th);

      Float32 tLo = FLT_MAX, tHi = -FLT_MAX, sLo = FLT_MAX, sHi = -FLT_MAX;
      Float32 nearMm = FLT_MAX;
      for(Int32 i = 0; i < m; ++i)
      {
          const Float32 dx = p[i].x - mx, dy = p[i].y - my;
          const Float32 t  =  dx * ux + dy * uy;
          const Float32 s  = -dx * uy + dy * ux;
          if(t < tLo)
          {
              tLo = t;
          }
          if(t > tHi)
          {
              tHi = t;
          }
          if(s < sLo)
          {
              sLo = s;
          }
          if(s > sHi)
          {
              sHi = s;
          }

          const Float32 d = std::sqrt(p[i].x * p[i].x + p[i].y * p[i].y);
          if(d < nearMm)
          {
              nearMm = d;
          }
      }

      const Float32 halfL = (tHi - tLo) * 0.5f;
      const Float32 halfW = (sHi - sLo) * 0.5f;

      // A box with no extent is one return with a rounding error, and a box
      // longer than a room's worth of furniture is a wall - neither is an object.
      if(halfL * 2.0f < OBJ_MIN_EXT_MM || halfL * 2.0f > OBJ_MAX_LEN_MM)
      {
          return false;
      }

      const Float32 mid = (tLo + tHi) * 0.5f;
      const Float32 off = (sLo + sHi) * 0.5f;

      out.cx = mx + ux * mid - uy * off;
      out.cy = my + uy * mid + ux * off;
      out.ux = ux;
      out.uy = uy;
      out.halfL = halfL;
      // Floored, so a flat-on surface still reads as a box rather than a line.
      out.halfW = std::max(halfW, OBJ_MIN_EXT_MM * 0.5f);
      out.nearMm = nearMm;
      out.n = m;
      return true;
  }

  Void findObstacles(const Vec<LidarPoint>& pts, Vec<Obstacle>& out)
  {
      out.clear();

      static Vec<WorldPt> w;
      w.clear();
      w.reserve(pts.size());

      const Float32 deg2rad = PI / 180.0f;
      for(const LidarPoint& p : pts)
      {
          if(!(p.distMm >= MIN_VALID_MM) || p.distMm > MAX_VALID_MM)
          {
              continue;
          }
          const Float32 a = (p.angleDeg - 90.0f) * deg2rad;
          w.push_back(WorldPt{ p.distMm * std::cos(a), p.distMm * std::sin(a) });
      }

      const Int32 n = static_cast<Int32>(w.size());
      if(n < OBJ_MIN_PTS)
      {
          return;
      }

      const auto gapAt = [&](Int32 i, Int32 j) {
          const Float32 dx = w[static_cast<Size>(j)].x - w[static_cast<Size>(i)].x;
          const Float32 dy = w[static_cast<Size>(j)].y - w[static_cast<Size>(i)].y;
          return std::sqrt(dx * dx + dy * dy);
      };

      // The scan is a CIRCLE, so a cluster straddling 0 deg would otherwise be cut
      // in half and fitted twice. Rotating the start to the largest gap in the
      // ring removes the wrap case entirely rather than special-casing it.
      Int32   startAt = 0;
      Float32 widest  = -1.0f;
      for(Int32 i = 0; i < n; ++i)
      {
          const Float32 g = gapAt(i, (i + 1) % n);
          if(g > widest)
          {
              widest = g;
              startAt = (i + 1) % n;
          }
      }

      static Vec<WorldPt> run;
      run.clear();

      const auto flush = [&]() {
          Obstacle o;
          if(static_cast<Int32>(out.size()) < OBJ_MAX
             && fitObstacle(run.data(), static_cast<Int32>(run.size()), o))
          {
              out.push_back(o);
          }
          run.clear();
      };

      for(Int32 k = 0; k < n; ++k)
      {
          const Int32 i = (startAt + k) % n;
          const WorldPt& cur = w[static_cast<Size>(i)];

          if(!run.empty())
          {
              const WorldPt& prev = run.back();
              const Float32 dx = cur.x - prev.x, dy = cur.y - prev.y;
              const Float32 gap = std::sqrt(dx * dx + dy * dy);
              const Float32 r   = std::sqrt(prev.x * prev.x + prev.y * prev.y);
              if(gap > OBJ_GAP_BASE_MM + r * OBJ_GAP_SLOPE)
              {
                  flush();
              }
          }
          run.push_back(cur);
      }
      flush();
  }

  // How far the car could go straight ahead before something enters the width it
  // sweeps. Forward is -y; the width is the chassis plus a small margin, because
  // clearing an obstacle by 5 mm is not clearing it.
  Float32 corridorFree(const Vec<LidarPoint>& pts, Float32 halfWidthMm)
  {
      Float32 best = CORRIDOR_MAX_MM;
      const Float32 deg2rad = PI / 180.0f;

      // Anything closer than the bumper is the car, not an obstacle.
      //
      // This is not a fudge to make the number look better - it is what the
      // geometry says. A return at 80 mm dead ahead is inside the chassis, and
      // once the C1 is actually mounted the bodywork will occlude that bearing
      // entirely. Without this the corridor reads "0.02 m" whenever the sensor is
      // sitting on a desk among clutter, which is true of the returns and false
      // of the car.
      const Float32 nose = EGO_LEN_MM * 0.5f + EGO_SENSOR_AHEAD_MM;

      for(const LidarPoint& p : pts)
      {
          if(!(p.distMm >= MIN_VALID_MM) || p.distMm > MAX_VALID_MM)
          {
              continue;
          }
          const Float32 a = (p.angleDeg - 90.0f) * deg2rad;
          const Float32 x = p.distMm * std::cos(a);
          const Float32 y = p.distMm * std::sin(a);

          if(y >= 0.0f)                       // behind or beside; not in the way
          {
              continue;
          }
          if(std::fabs(x) > halfWidthMm)
          {
              continue;
          }

          const Float32 ahead = -y;
          if(ahead <= nose)                   // inside the footprint
          {
              continue;
          }
          if(ahead < best)
          {
              best = ahead;
          }
      }
      return best;
  }

  ImU32 clearanceColor(Float32 mm)
  {
      if(mm >= CORRIDOR_WARN_MM)
      {
          return ui::ansi::BRGREEN;
      }
      if(mm >= CORRIDOR_STOP_MM)
      {
          return ui::ansi::BRYELLOW;
      }
      return ui::ansi::BRRED;
  }

  // The car. Drawn to scale, which means it VANISHES at 12 m across - correct, and
  // the reason there is no minimum size: a vehicle footprint that stays legible
  // while the world zooms out is lying about how big the car is.
  // The C1's own footprint, to scale. 55.6 mm square - see lidar/README.md.
  //
  // Drawn instead of the car when the ego view says sensor, and it is a genuinely
  // different claim: this is what is on the desk right now, where the car
  // footprint is what will be there once the thing is built.
  Void drawSensorFootprint(const MarkCtx& c)
  {
      constexpr Float32 BASE_MM = vehicle::C1_BASE_MM;

      const Float32 h = BASE_MM * 0.5f * c.ppm;
      if(h < 1.0f)
      {
          return;                       // smaller than a pixel; the hub says it
      }

      const ImVec2 a(c.s0.x - h, c.s0.y - h);
      const ImVec2 b(c.s0.x + h, c.s0.y + h);

      c.dl->AddRectFilled(a, b, (ui::ansi::BLACK & 0x00FFFFFFu) | (0xC0u << IM_COL32_A_SHIFT));
      c.dl->AddRect(a, b, (ui::ansi::WHITE & 0x00FFFFFFu) | (0xD0u << IM_COL32_A_SHIFT), 0.0f, 0,
                    std::max(1.0f, 1.2f * c.dpi));
  }

  Void drawEgo(const MarkCtx& c)
  {
      if(egoView(c) == scene3d::EgoView::EGO_VIEW_SENSOR)
      {
          drawSensorFootprint(c);
          return;
      }

      const Float32 hw = EGO_WID_MM * 0.5f * c.ppm;
      const Float32 hl = EGO_LEN_MM * 0.5f * c.ppm;
      if(hw < 1.5f)
      {
          return;
      }

      const Float32 oy = -EGO_SENSOR_AHEAD_MM * c.ppm;   // screen y is down
      const Float32 cx = c.s0.x;
      const Float32 cy = c.s0.y + oy;

      const ImU32 body = IM_COL32(0xE5, 0xE5, 0xE5, 0x40);
      const ImU32 edge = ui::ansi::BRWHITE;
      const Float32 th = std::max(1.0f, 1.4f * c.dpi);

      // Wheels first, so the body sits over them. Placed on the real TREAD rather
      // than tucked inside the body edge - on a touring car the tires sit inboard
      // of the arches, and the difference is 14 mm a side.
      const Float32 wr = EGO_WHEEL_D_MM * 0.5f * c.ppm;
      const Float32 ww = EGO_WHEEL_W_MM * 0.5f * c.ppm;
      const Float32 ax = EGO_WHEELBASE_MM * 0.5f * c.ppm;
      const Float32 tx = EGO_TREAD_MM * 0.5f * c.ppm;
      if(wr > 1.0f)
      {
          for(Int32 sx = -1; sx <= 1; sx += 2)
          {
              for(Int32 sy = -1; sy <= 1; sy += 2)
              {
                  const Float32 wx = cx + static_cast<Float32>(sx) * tx;
                  const Float32 wy = cy + static_cast<Float32>(sy) * ax;
                  c.dl->AddRectFilled(ImVec2(wx - ww, wy - wr), ImVec2(wx + ww, wy + wr),
                                      IM_COL32(0x7F, 0x7F, 0x7F, 0xC0));
              }
          }
      }

      // The shell in plan view. A touring body is not a hexagon: it is widest over
      // the rear arches, waists slightly at the doors, and tapers to a nose about
      // 40% of its width. Fourteen points, mirrored, from the body's own
      // proportions - the previous six-point wedge read as an arrowhead, which is
      // not what is bolted to this chassis.
      //
      // Front is -y. Fractions are of the half-width and half-length.
      struct Rib { Float32 y, w; };
      constexpr Rib RIBS[7] = {
          { -1.00f, 0.34f },   // nose
          { -0.90f, 0.62f },
          { -0.72f, 0.88f },
          { -0.42f, 0.99f },   // front arches
          {  0.10f, 0.94f },   // door waist
          {  0.58f, 1.00f },   // rear arches, the widest point
          {  1.00f, 0.80f },   // tail
      };

      ImVec2 poly[14];
      for(Int32 i = 0; i < 7; ++i)                       // down the right side
      {
          poly[i] = ImVec2(cx + hw * RIBS[i].w, cy + hl * RIBS[i].y);
      }
      for(Int32 i = 0; i < 7; ++i)                       // back up the left
      {
          poly[7 + i] = ImVec2(cx - hw * RIBS[6 - i].w, cy + hl * RIBS[6 - i].y);
      }

      c.dl->AddConvexPolyFilled(poly, 14, body);
      c.dl->AddPolyline(poly, 14, edge, ImDrawFlags_Closed, th);

      // The greenhouse, so it reads as a car rather than as a blob, and so the
      // front half is distinguishable from the back at a glance.
      if(hl > 10.0f)
      {
          const ImVec2 cab[4] = {
              ImVec2(cx - hw * 0.50f, cy - hl * 0.26f),
              ImVec2(cx + hw * 0.50f, cy - hl * 0.26f),
              ImVec2(cx + hw * 0.62f, cy + hl * 0.40f),
              ImVec2(cx - hw * 0.62f, cy + hl * 0.40f),
          };
          c.dl->AddPolyline(cab, 4, IM_COL32(0xE5, 0xE5, 0xE5, 0x66),
                            ImDrawFlags_Closed, th);
      }

      // Center line: the axis the corridor is measured along, so the two cannot
      // appear to disagree.
      if(hl > 6.0f)
      {
          c.dl->AddLine(ImVec2(cx, cy - hl * 0.95f), ImVec2(cx, cy + hl * 0.90f),
                        IM_COL32(0xE5, 0xE5, 0xE5, 0x40), th);
      }
  }

  // Label placement, first come first served.
  //
  // The near field around the sensor is where the objects are smallest, most
  // numerous and closest together, which is exactly where a label per box turns
  // into an unreadable pile - and the first version of this did precisely that.
  // Callers place in priority order and a label that would land on one already
  // down is dropped; the box is still drawn, still counted.
  struct LabelRect { Float32 x0, y0, x1, y1; };

  Bool claimLabel(Vec<LabelRect>& taken, const ImVec2& mid, const ImVec2& ts, Float32 dpi)
  {
      const Float32 pad = 4.0f * dpi;
      const LabelRect r{ mid.x - ts.x * 0.5f - pad, mid.y - ts.y * 0.5f - pad,
                         mid.x + ts.x * 0.5f + pad, mid.y + ts.y * 0.5f + pad };

      for(const LabelRect& q : taken)
      {
          if(!(r.x1 < q.x0 || r.x0 > q.x1 || r.y1 < q.y0 || r.y0 > q.y1))
          {
              return false;
          }
      }

      taken.push_back(r);
      return true;
  }

  // A tracked-object box, drawn as CORNER BRACKETS rather than a closed outline.
  //
  // Brackets because a full rectangle around a cluster reads as "this rectangle is
  // the object", which is a stronger claim than the data supports - the lidar sees
  // one face and the box is a bound, not a shape. Open corners say bound.
  Void drawObstacle(const MarkCtx& c, const Obstacle& o, Bool label, Vec<LabelRect>& taken)
  {
      const Float32 px = -o.uy, py = o.ux;

      const Float32 wx = o.ux * o.halfL, wy = o.uy * o.halfL;
      const Float32 vx = px   * o.halfW, vy = py   * o.halfW;

      const ImVec2 k[4] = {
          ImVec2(c.s0.x + (o.cx - wx - vx) * c.ppm, c.s0.y + (o.cy - wy - vy) * c.ppm),
          ImVec2(c.s0.x + (o.cx + wx - vx) * c.ppm, c.s0.y + (o.cy + wy - vy) * c.ppm),
          ImVec2(c.s0.x + (o.cx + wx + vx) * c.ppm, c.s0.y + (o.cy + wy + vy) * c.ppm),
          ImVec2(c.s0.x + (o.cx - wx + vx) * c.ppm, c.s0.y + (o.cy - wy + vy) * c.ppm),
      };

      const ImU32 base = o.inPath ? ui::ansi::BRRED : ui::ansi::BRWHITE;
      const ImU32 fill = (base & 0x00FFFFFFu)
                       | (static_cast<ImU32>(o.inPath ? 34u : 20u) << IM_COL32_A_SHIFT);
      const ImU32 line = (base & 0x00FFFFFFu)
                       | (static_cast<ImU32>(o.inPath ? 0xFFu : 0xD0u) << IM_COL32_A_SHIFT);

      // Genuinely convex - it is a rectangle - so the fast path is exact here, in
      // a way it was not for the room profile that once used it.
      c.dl->AddConvexPolyFilled(k, 4, fill);

      const Float32 th = std::max(1.0f, (o.inPath ? 2.0f : 1.5f) * c.dpi);

      for(Int32 i = 0; i < 4; ++i)
      {
          const ImVec2& a = k[i];
          const ImVec2& b = k[(i + 1) % 4];
          const ImVec2& z = k[(i + 3) % 4];

          // An arm along each edge leaving this corner, a third of the edge or
          // 11 px, whichever is shorter - so a small box gets small brackets
          // instead of two arms meeting in the middle and closing the outline.
          for(Int32 s = 0; s < 2; ++s)
          {
              const ImVec2& e = (s == 0) ? b : z;
              const Float32 dx = e.x - a.x, dy = e.y - a.y;
              const Float32 len = std::sqrt(dx * dx + dy * dy);
              if(len < 1.0f)
              {
                  continue;
              }
              const Float32 arm = std::min(len * 0.33f, 11.0f * c.dpi);
              c.dl->AddLine(a, ImVec2(a.x + dx / len * arm, a.y + dy / len * arm),
                            line, th);
          }
      }

      if(!label)
      {
          return;
      }

      // Range, and size for anything big enough that its size is a fact rather
      // than a quantisation artifact.
      Array<Char, 40> lab;
      const Float32 across = o.halfW * 2.0f, along = o.halfL * 2.0f;
      if(std::max(across, along) >= 250.0f)
      {
          std::snprintf(lab.data(), lab.size(), "%.2f m  %.0fx%.0f",
                        static_cast<Float64>(o.nearMm / 1000.0f),
                        static_cast<Float64>(along), static_cast<Float64>(across));
      }
      else
      {
          std::snprintf(lab.data(), lab.size(), "%.2f m",
                        static_cast<Float64>(o.nearMm / 1000.0f));
      }

      ImFont*       f  = labelFont();
      const Float32 fs = labelPx();
      const ImVec2  ts = f->CalcTextSizeA(fs, FLT_MAX, 0.0f, lab.data());

      // Pushed OUTWARD along the bearing from the sensor, so labels on opposite
      // sides of the map lean away from each other instead of all piling toward
      // the middle - which is also where the sensor readout already is.
      const Float32 d = std::sqrt(o.cx * o.cx + o.cy * o.cy);
      const Float32 nx = (d > 1.0f) ? o.cx / d : 0.0f;
      const Float32 ny = (d > 1.0f) ? o.cy / d : -1.0f;
      const Float32 push = (std::max(o.halfL, o.halfW) * c.ppm) + 13.0f * c.dpi;

      const ImVec2 mid(c.s0.x + o.cx * c.ppm + nx * push,
                       c.s0.y + o.cy * c.ppm + ny * push);

      if(!claimLabel(taken, mid, ts, c.dpi))
      {
          return;
      }

      plateTextAt(c.dl, mid, Rect{ c.p0, c.p1 }, c.dpi, line, lab.data());
  }

  // The corridor the car would drive into. A ribbon of the chassis' own width,
  // colored by how much of it is free.
  Void drawCorridor(const MarkCtx& c, Float32 halfWidthMm, Float32 freeMm, Vec<LabelRect>& taken)
  {
      const Float32 hw = halfWidthMm * c.ppm;
      if(hw < 1.5f)
      {
          return;
      }

      const Float32 y0 = c.s0.y - (EGO_LEN_MM * 0.5f + EGO_SENSOR_AHEAD_MM) * c.ppm;
      const Float32 y1 = c.s0.y - freeMm * c.ppm;
      if(y1 >= y0 - 2.0f)
      {
          return;                       // nothing between the bumper and the stop
      }

      const ImU32 col  = clearanceColor(freeMm);
      const ImU32 fill = (col & 0x00FFFFFFu) | (static_cast<ImU32>(24u) << IM_COL32_A_SHIFT);
      const ImU32 line = (col & 0x00FFFFFFu) | (static_cast<ImU32>(0xB0u) << IM_COL32_A_SHIFT);

      c.dl->AddRectFilled(ImVec2(c.s0.x - hw, y1), ImVec2(c.s0.x + hw, y0), fill);
      c.dl->AddLine(ImVec2(c.s0.x - hw, y0), ImVec2(c.s0.x - hw, y1), line, 1.4f * c.dpi);
      c.dl->AddLine(ImVec2(c.s0.x + hw, y0), ImVec2(c.s0.x + hw, y1), line, 1.4f * c.dpi);

      // The stop line, and what it is.
      c.dl->AddLine(ImVec2(c.s0.x - hw, y1), ImVec2(c.s0.x + hw, y1), line, 2.2f * c.dpi);

      Array<Char, 32> lab;
      if(freeMm >= CORRIDOR_MAX_MM)
      {
          std::snprintf(lab.data(), lab.size(), "clear >%.0f m",
                        static_cast<Float64>(CORRIDOR_MAX_MM / 1000.0f));
      }
      else
      {
          std::snprintf(lab.data(), lab.size(), "ahead %.2f m",
                        static_cast<Float64>(freeMm / 1000.0f));
      }

      // Claims its space FIRST, before any object label. It is the one number on
      // this display that is a driving decision rather than an observation, so if
      // something has to lose a label it is not this.
      ImFont*       lf = labelFont();
      const ImVec2  ts = lf->CalcTextSizeA(labelPx(), FLT_MAX, 0.0f, lab.data());
      const ImVec2  at(c.s0.x, y1 - 11.0f * c.dpi);

      if(claimLabel(taken, at, ts, c.dpi))
      {
          plateTextAt(c.dl, at, Rect{ c.p0, c.p1 }, c.dpi,
                      (col & 0x00FFFFFFu) | (static_cast<ImU32>(0xFFu) << IM_COL32_A_SHIFT),
                      lab.data());
      }
  }

  // there may not be a sidebar in view.
  Void drawMarksFull(const MarkCtx& c, const MapState& st, const Deque<Vec<LidarPoint>>& trail, Float32 hz)
  {
      const Vec<LidarPoint>& pts = trail.back();

      // ---- what this mode adds, worked out first so the panel can report it --
      //
      // The corridor is measured against the chassis plus 30 mm a side: clearing
      // an obstacle by the width of the paint is not clearing it.
      const Float32 halfWidth = EGO_WID_MM * 0.5f + 30.0f;
      const Float32 freeAhead = corridorFree(pts, halfWidth);

      static Vec<Obstacle> obstacles;
      findObstacles(pts, obstacles);

      Int32 inPathCount = 0;
      for(Obstacle& o : obstacles)
      {
          // The OBB's own extent along x, which is the axis the corridor is
          // bounded on - exact for a rotated rectangle, unlike a bounding radius.
          const Float32 ex = std::fabs(o.ux) * o.halfL + std::fabs(o.uy) * o.halfW;
          o.inPath = (o.cy < 0.0f) && (std::fabs(o.cx) - ex <= halfWidth);
          if(o.inPath)
          {
              ++inPathCount;
          }
      }

      // ---- free space, underneath everything -------------------------------
      if(st.ready)
      {
          ImVec2 poly[CLR_BINS];
          Bool any = false;
          for(Int32 i = 0; i < CLR_BINS; ++i)
          {
              if(st.clrSeen[i])
              {
                  any = true;
              }
              const Float32 d   = st.clrSeen[i] ? st.clr[i] : 0.0f;
              const Float32 deg = (static_cast<Float32>(i) + 0.5f) * CLR_BIN_DEG - 90.0f;
              const Float32 a   = deg * (PI / 180.0f);
              const Float32 rr  = d * c.ppm;
              poly[i] = ImVec2(c.s0.x + rr * std::cos(a), c.s0.y + rr * std::sin(a));
          }
          if(any)
          {
              emitFan(c.dl, c.s0, poly, CLR_BINS,
                      CLEAR_RGB | (static_cast<ImU32>(28u) << IM_COL32_A_SHIFT), c.uv);
              for(Int32 i = 0; i < CLR_BINS; ++i)
              {
                  c.dl->AddLine(poly[i], poly[(i + 1) % CLR_BINS],
                                CLEAR_RGB | (static_cast<ImU32>(120u) << IM_COL32_A_SHIFT),
                                1.2f * c.dpi);
              }
          }
      }

      // ---- surfaces --------------------------------------------------------
      drawMarksContour(c, trail);

      // ---- the corridor, under the evidence --------------------------------
      //
      // The label bookkeeping starts here rather than with the objects, because
      // the corridor's own label is the highest-priority one on the map.
      static Vec<LabelRect> taken;
      taken.clear();

      // The sensor's own readouts are already on the map and are not negotiable,
      // so they claim their space before anything else does.
      taken.push_back(LabelRect{ c.s0.x - 52.0f * c.dpi, c.s0.y - 26.0f * c.dpi,
                                 c.s0.x + 96.0f * c.dpi, c.s0.y + 26.0f * c.dpi });

      drawCorridor(c, halfWidth, freeAhead, taken);

      // ---- the returns themselves, brightest -------------------------------
      Vec<Dot>& dots = scratch();
      collectDots(pts, MapScale{ c.s0, c.ppm, c.dpi },
                  Rect{ c.cullLo, c.cullHi }, dots);
      emitDiscs(c.dl, dots.data(), static_cast<Int32>(dots.size()), c.dotR,
                POINT_RGB | (static_cast<ImU32>(0xFFu) << IM_COL32_A_SHIFT), c.uv);

      // ---- objects, over the returns they were fitted to --------------------
      //
      // Drawn in PRIORITY ORDER - in-path first, then nearest - because labels are
      // first come first served and the order therefore decides which object gets
      // to keep its number when two would collide. In-path and near is exactly the
      // order a driver would want them in.
      static Vec<Int32> order;
      order.clear();
      for(Int32 i = 0; i < static_cast<Int32>(obstacles.size()); ++i)
      {
          order.push_back(i);
      }

      std::sort(order.begin(), order.end(), [&](Int32 a2, Int32 b2) {
          const Obstacle& x = obstacles[static_cast<Size>(a2)];
          const Obstacle& y = obstacles[static_cast<Size>(b2)];
          if(x.inPath != y.inPath)
          {
              return x.inPath;
          }
          return x.nearMm < y.nearMm;
      });

      for(Int32 idx : order)
      {
          const Obstacle& o = obstacles[static_cast<Size>(idx)];
          drawObstacle(c, o, o.inPath || o.nearMm < 2500.0f, taken);
      }

      // ---- the car ---------------------------------------------------------
      drawEgo(c);

      // ---- the widest drivable gap ------------------------------------------
      Float32 bestWidth = 0.0f, bestDeg = 0.0f;
      if(st.ready)
      {
          Int32 b = 0;
          while(b < CLR_BINS)
          {
              if(!st.clrSeen[b] || st.clr[b] < 1200.0f)
              {
                  ++b;
                  continue;
              }
              const Int32 start = b;
              Float32 nearestMm = st.clr[b];
              while(b < CLR_BINS && st.clrSeen[b] && st.clr[b] >= 1200.0f)
              {
                  if(st.clr[b] < nearestMm)
                  {
                      nearestMm = st.clr[b];
                  }
                  ++b;
              }
              const Int32 span = b - start;
              const Float32 spanDeg = static_cast<Float32>(span) * CLR_BIN_DEG;
              const Float32 width = 2.0f * nearestMm
                                  * std::sin(spanDeg * 0.5f * (PI / 180.0f));
              if(width > bestWidth)
              {
                  bestWidth = width;
                  bestDeg = (static_cast<Float32>(start) + span * 0.5f) * CLR_BIN_DEG;
              }
          }

          if(bestWidth > 350.0f)
          {
              const Float32 a = (bestDeg - 90.0f) * (PI / 180.0f);
              const Float32 rr = 0.90f * std::min(c.p1.x - c.p0.x, c.p1.y - c.p0.y) * 0.5f;
              const ImVec2 tip(c.s0.x + rr * std::cos(a), c.s0.y + rr * std::sin(a));
              c.dl->AddLine(c.s0, tip,
                            GAP_RGB | (static_cast<ImU32>(0x30u) << IM_COL32_A_SHIFT),
                            6.0f * c.dpi);
              c.dl->AddLine(c.s0, tip,
                            GAP_RGB | (static_cast<ImU32>(0xC0u) << IM_COL32_A_SHIFT),
                            1.6f * c.dpi);

              Array<Char, 40> lab;
              std::snprintf(lab.data(), lab.size(), "widest gap %.2f m",
                            static_cast<Float64>(bestWidth / 1000.0f));
              ImFont* f = labelFont();
              const ImVec2 ts = f->CalcTextSizeA(labelPx(), FLT_MAX, 0.0f, lab.data());
              plateText(c.dl, ImVec2(tip.x - ts.x * 0.5f, tip.y - ts.y * 0.5f), c.dpi,
                        GAP_RGB | (static_cast<ImU32>(0xFFu) << IM_COL32_A_SHIFT), lab.data());
          }
      }

      // ---- the instrument panel, on the map ---------------------------------
      Int32   inSpec = 0, noReturn = 0, outOfRange = 0;
      Float32 nearMm = MAX_VALID_MM, maxMm = 0.0f;
      Float64 sumMm  = 0.0;

      for(const LidarPoint& p : pts)
      {
          if(p.distMm <= 0.0f)
          {
              ++noReturn;
              continue;
          }
          if(!inWindow(p.distMm))
          {
              ++outOfRange;
              continue;
          }
          ++inSpec;
          sumMm += static_cast<Float64>(p.distMm);
          if(p.distMm < nearMm)
          {
              nearMm = p.distMm;
          }
          if(p.distMm > maxMm)
          {
              maxMm  = p.distMm;
          }
      }
      const Int32 total = static_cast<Int32>(pts.size());
      if(inSpec == 0)
      {
          nearMm = 0.0f;
      }

      Float32 minClr = MAX_VALID_MM, minClrDeg = 0.0f;
      if(st.ready)
      {
          for(Int32 i = 0; i < CLR_BINS; ++i)
          {
              if(st.clrSeen[i] && st.clr[i] < minClr)
              {
                  minClr = st.clr[i];
                  minClrDeg = (static_cast<Float32>(i) + 0.5f) * CLR_BIN_DEG;
              }
          }
      }

      struct Row { const Char* k; Char v[32]; };
      Row rows[10];
      Int32 nr = 0;

      const auto addRow = [&](const Char* k, const Char* fmt, Float64 a) {
          if(nr >= 10)
          {
              return;
          }
          rows[nr].k = k;
          std::snprintf(rows[nr].v, sizeof(rows[nr].v), fmt, a);
          ++nr;
      };

      addRow("rate",      "%.1f Hz",  static_cast<Float64>(hz));
      addRow("returns",   "%.0f /rev", static_cast<Float64>(total));
      addRow("in spec",   "%.0f %%",
             total > 0 ? 100.0 * static_cast<Float64>(inSpec) / total : 0.0);
      addRow("nearest",   "%.2f m",   static_cast<Float64>(nearMm / 1000.0f));
      addRow("mean",      "%.2f m",
             inSpec > 0 ? sumMm / inSpec / 1000.0 : 0.0);
      addRow("furthest",  "%.2f m",   static_cast<Float64>(maxMm / 1000.0f));
      addRow("clearance", "%.2f m",   static_cast<Float64>(minClr / 1000.0f));
      addRow("gap",       "%.2f m",   static_cast<Float64>(bestWidth / 1000.0f));
      addRow("objects",   "%.0f",     static_cast<Float64>(obstacles.size()));
      addRow("ahead",     "%.2f m",   static_cast<Float64>(freeAhead / 1000.0f));

      ImFont*       f  = labelFont();
      const Float32 fs = labelPx();
      const Float32 lh = fs * 1.45f;
      const Float32 pad = 9.0f * c.dpi;

      Float32 kw = 0.0f, vw = 0.0f;
      for(Int32 i = 0; i < nr; ++i)
      {
          kw = std::max(kw, f->CalcTextSizeA(fs, FLT_MAX, 0.0f, rows[i].k).x);
          vw = std::max(vw, f->CalcTextSizeA(fs, FLT_MAX, 0.0f, rows[i].v).x);
      }
      const Float32 gapW = fs * 0.9f;
      const Float32 boxW = kw + gapW + vw + pad * 2.0f;
      const Float32 boxH = lh * static_cast<Float32>(nr) + pad * 2.0f;

      // Right edge, vertically centered: the only quarter of the map the HUD does
      // not already use.
      const ImVec2 a(c.p1.x - boxW - 14.0f * c.dpi,
                     (c.p0.y + c.p1.y) * 0.5f - boxH * 0.5f);
      const ImVec2 b(a.x + boxW, a.y + boxH);

      ui::plate(a, b, (ui::ansi::BLACK & 0x00FFFFFFu) | (0xE8u << IM_COL32_A_SHIFT), 0.0f);

      for(Int32 i = 0; i < nr; ++i)
      {
          const Float32 y = a.y + pad + lh * static_cast<Float32>(i);
          c.dl->AddText(f, fs, ImVec2(a.x + pad, y), ui::ansi::GRAY, rows[i].k);

          // One row is a decision rather than a measurement, and it is colored
          // like one. The rest stay white: tinting a number to make it stand out,
          // when its value carries no state, is the habit this palette exists to
          // stop.
          const ImU32 col = (i == nr - 1) ? clearanceColor(freeAhead)
                                          : ui::ansi::WHITE;
          const Float32 w = f->CalcTextSizeA(fs, FLT_MAX, 0.0f, rows[i].v).x;
          c.dl->AddText(f, fs, ImVec2(b.x - pad - w, y), col, rows[i].v);
      }

      say(c, "%d objects (%d in path)  |  %.2f m ahead  |  %d in spec of %d  |  "
             "clearance %.2f m at %.0f deg  |  widest gap %.2f m",
          static_cast<Int32>(obstacles.size()), inPathCount,
          static_cast<Float64>(freeAhead / 1000.0f),
          inSpec, total,
          static_cast<Float64>(minClr / 1000.0f), static_cast<Float64>(minClrDeg),
          static_cast<Float64>(bestWidth / 1000.0f));
  }

  // ---------------------------------------------------------------------------
  // MapMode::MAP_MODE_MINIMAL
  //
  // The one to show somebody who does not care how it works.
  //
  // Every other mode is an instrument: it has a grid to measure against, a number
  // in the corner, and a reason for each color. This one has none of those, and
  // removing them IS the design - a display for an audience is not a debug view
  // with the labels turned off, it is a different question. "What can it see?"
  // rather than "is bin 47 stale?".
  //
  // So: the room as one soft shape, the returns as a scatter of light on its edge,
  // the car in the middle. No rings, no bearings, no readout, no legend. It should
  // be legible from two meters away by somebody who has never seen a lidar.
  // ---------------------------------------------------------------------------
  Void drawMarksMinimal(const MarkCtx& c, const MapState& st, const Deque<Vec<LidarPoint>>& trail)
  {
      // The room. One filled shape, no outline heavier than it needs - the free
      // space is the subject here, not the boundary.
      if(st.ready)
      {
          ImVec2 poly[CLR_BINS];
          Bool any = false;
          for(Int32 i = 0; i < CLR_BINS; ++i)
          {
              if(st.clrSeen[i])
              {
                  any = true;
              }
              const Float32 d   = st.clrSeen[i] ? st.clr[i] : 0.0f;
              const Float32 deg = (static_cast<Float32>(i) + 0.5f) * CLR_BIN_DEG - 90.0f;
              const Float32 a   = deg * (PI / 180.0f);
              const Float32 rr  = d * c.ppm;
              poly[i] = ImVec2(c.s0.x + rr * std::cos(a), c.s0.y + rr * std::sin(a));
          }

          if(any)
          {
              // Star-shaped about the sensor by construction, so the fan is exact.
              emitFan(c.dl, c.s0, poly, CLR_BINS,
                      (ui::ansi::BLUE  & 0x00FFFFFFu) | (0x60u << IM_COL32_A_SHIFT), c.uv);
              for(Int32 i = 0; i < CLR_BINS; ++i)
              {
                  c.dl->AddLine(poly[i], poly[(i + 1) % CLR_BINS],
                                (ui::ansi::BRCYAN & 0x00FFFFFFu) | (0xC0u << IM_COL32_A_SHIFT), 1.6f * c.dpi);
              }
          }
      }

      // The returns, as light rather than as data. Bigger and softer than the
      // debug modes' 2 px dots, because at a glance a scatter of small dots reads
      // as noise and a scatter of larger ones reads as an edge.
      Vec<Dot>& dots = scratch();
      collectDots(trail.back(), MapScale{ c.s0, c.ppm, c.dpi },
                  Rect{ c.cullLo, c.cullHi }, dots);
      emitDiscs(c.dl, dots.data(), static_cast<Int32>(dots.size()), c.dotR * 1.5f,
                IM_COL32(0xFF, 0xFF, 0xFF, 0xE0), c.uv);

      drawEgo(c);

      // Nothing in the corner. Deliberately: `diag` stays empty, so the HUD line
      // above the map has nothing to print and the view is what it says it is.
      say(c, "%s", "");
  }

  // The sensor itself. Deliberately smaller than BLIND_MIN_PX so it reads as the
  // core of the blind disc rather than competing with its hatched edge.
  Void drawHub(ImDrawList* dl, const ImVec2& c, Float32 dpi)
  {
      // The origin. A ring and a cross - the crosshair every plotting tool has
      // drawn at 0,0 since they were vector displays, because it marks a point
      // exactly without covering it.
      const Float32 r = 5.0f * dpi;
      const Float32 t = 1.5f * dpi;

      dl->AddCircle(c, r, HUB_COL, 16, t);
      dl->AddLine(ImVec2(c.x - r * 1.9f, c.y), ImVec2(c.x - r * 0.55f, c.y), HUB_COL, t);
      dl->AddLine(ImVec2(c.x + r * 0.55f, c.y), ImVec2(c.x + r * 1.9f, c.y), HUB_COL, t);
      dl->AddLine(ImVec2(c.x, c.y - r * 1.9f), ImVec2(c.x, c.y - r * 0.55f), HUB_COL, t);
      dl->AddLine(ImVec2(c.x, c.y + r * 0.55f), ImVec2(c.x, c.y + r * 1.9f), HUB_COL, t);
      dl->AddCircleFilled(c, 1.6f * dpi, HUB_CORE_COL, 8);
  }

  Void drawPlaceholder(ImDrawList* dl, const ImVec2& c)
  {
      ImFont*      font = labelFont();
      const Float32  fs   = labelPx() * 1.25f;
      const Char*  txt  = "No scan data";
      const ImVec2 ts   = font->CalcTextSizeA(fs, FLT_MAX, 0.0f, txt);

      dl->AddText(font, fs, ImVec2(c.x - ts.x * 0.5f, c.y - ts.y * 0.5f), EMPTY_COL, txt);
  }

}

// ------------------------------------------------------------------ state ---

namespace
{

  // Distance the auto-fit should try to contain. Deliberately the 95th percentile
  // rather than the maximum: a single return down a corridor or through a doorway
  // would otherwise stretch the view to 16 m and shrink the room to a smudge.
  // Returns below the C1's 0.05 m spec floor are housing reflections, not data.
  Float32 fitDistanceMm(const LidarFrame& frame)
  {
      static Vec<Float32> d;
      d.clear();
      d.reserve(frame.points.size());

      for(const LidarPoint& p : frame.points)
      {
          if(p.distMm >= MIN_VALID_MM && p.distMm <= MAX_VALID_MM)
          {
              d.push_back(p.distMm);
          }
      }

      if(d.empty())
      {
          return 0.0f;
      }

      Size k = static_cast<Size>((d.size() * 0.95f));
      if(k >= d.size())
      {
          k = d.size() - 1;
      }

      std::nth_element(d.begin(), d.begin() + k, d.end());
      return d[k];
  }

}

// One row per mode. Kept as a table rather than a switch because every entry
// has to answer the same three questions, and a table makes a missing answer
// obvious.
constexpr MapModeInfo MODE_INFO[static_cast<Size>(MapMode::MAP_MODE_COUNT)] = {
    { "Points",
      "One flat dot per in-spec return, current revolution brightest.",
      "The honest default, and the only mode here that infers nothing. "
      "Everything else is a claim about this." },

    { "Density",
      "Hit counts in a fixed 6 cm world grid over the last ~4 s.",
      "What is STABLE. A wall is hit every revolution and saturates to white; "
      "anything that moved leaves a low-count blue smear. Least jittery mode." },

    { "Motion",
      "Cells hit now whose whole neighborhood the memory map says was empty.",
      "What changed. The complement of Density: that shows what stayed put, "
      "this shows what did not. A still room is correctly almost blank." },

    { "Clearance",
      "Free-space polygon: one smoothed radius per 3 deg of bearing.",
      "How far the BEAM reaches on each bearing. Closes instantly on a new "
      "obstacle, opens slowly. Compare with Fit, which asks about the car." },

    { "Gaps",
      "Openings wide enough to drive through, with their width in meters.",
      "The follow-the-gap primitive, drawn. Width is the chord across the "
      "opening at its nearest edge, so it is what would actually have to fit." },

    { "Walls",
      "Straight segments FITTED to the returns, with their lengths.",
      "Not the dots joined up - a decision about which runs of dots are "
      "actually a straight surface, and how long and which way each one is. "
      "The landmarks a SLAM front end would key on." },

    { "Corners",
      "Where two fitted walls meet at more than 35 deg, with the angle.",
      "A wall fixes your distance and heading but not where you are ALONG it - "
      "slide it and it looks the same. A corner does not slide. These are the "
      "point landmarks, which is what a scan-matcher actually keys on." },

    { "Fit",
      "Free space eroded by the chassis width: where the CAR can go.",
      "Clearance is about the sensor; this is about the car. Every gap "
      "narrower than the chassis vanishes, which is the point - a 15 cm slot "
      "is free space and is not a route. Red ticks mark blocked bearings." },

    { "Full",
      "The field display: car, objects as boxes, the corridor ahead, numbers.",
      "For standing outside with the laptop as the only instrument. Nothing "
      "hidden behind a panel or a hover - it is all on one screen." },

    { "Minimal",
      "The room as one shape, the returns on its edge, the car. Nothing else.",
      "For showing people. No grid, no numbers, no legend - not Full with the "
      "labels off, but a different question: what can it see, rather than is "
      "bin 47 stale. Legible across a room by someone who has never seen a "
      "lidar." },
};

const MapModeInfo& mapModeInfo(MapMode m) noexcept
{
    const Size i = static_cast<Size>(m);
    if(i >= static_cast<Size>(MapMode::MAP_MODE_COUNT))
    {
        return MODE_INFO[0];
    }
    return MODE_INFO[i];
}

const Char* mapModeName(MapMode m) noexcept
{
    return mapModeInfo(m).name;
}

GridStyle mapModeGrid(MapMode m) noexcept
{
    switch(m)
    {
    // Lengths and right angles. Squares.
    case MapMode::MAP_MODE_WALLS:
    case MapMode::MAP_MODE_CORNERS:
        return GridStyle::GRID_STYLE_CARTESIAN;

    // These two ARE a fixed world grid - 60 mm cells accumulated in world
    // space. Drawing meter squares over them shows the thing they are actually
    // built on; rings would be a second, unrelated coordinate system laid on
    // top of the one the data lives in.
    case MapMode::MAP_MODE_DENSITY:
    case MapMode::MAP_MODE_MOTION:
        return GridStyle::GRID_STYLE_CARTESIAN;

    // Nothing to measure against. See the mode's own note.
    case MapMode::MAP_MODE_MINIMAL:
        return GridStyle::GRID_STYLE_NONE;

    // Range and bearing are the quantities; rings and spokes are the ruler.
    default:
        return GridStyle::GRID_STYLE_RADIAL;
    }
}

ImU32 mapModeBackground(MapMode m) noexcept
{
    // Black. All of them, and a real one - 0x000000, not a dark blue standing in
    // for it.
    //
    // The per-mode tint is gone on purpose. It was a way of telling you which
    // mode was active, but the mode toggle already does that in words, and the
    // cost was that every data color had to survive being laid over fourteen
    // different grounds - so none of them could be fully itself. On a terminal
    // palette the ground has exactly one job: to be the thing every other color
    // is maximally far from. Anything other than black is worse at that job.
    static_cast<Void>(m);
    return ui::ansi::BLACK;
}

Void RadarView::push(const LidarFrame& frame)
{
    trail.push_back(frame.points);
    lastHz = frame.hz;
    while(trail.size() > MAX_TRAIL)
    {
        trail.pop_front();
    }

    hasData = true;

    // The persistent maps are fed here rather than in draw(), in every mode:
    // they record what the sensor has SEEN, and that cannot depend on which
    // mode happened to be on screen while it saw it. Switching to Density or
    // Occupancy therefore shows a map that is already correct.
    accumulateRevolution(mapStateFor(this), frame.points);

    const Float32 fitMm = fitDistanceMm(frame);
    if(fitMm > 0.0f)
    {
        // Keep a short history and fit to the LARGEST recent revolution, not the
        // newest one. The 95th percentile moves by meters between consecutive
        // revolutions - returns flicker in and out at the edges of the room - and
        // chasing it directly made the view visibly bounce. Taking the window
        // maximum means a single sparse revolution cannot pull the view in, so
        // it only shrinks once the scene has actually stayed small.
        fitHist[fitN % FIT_HISTORY] = fitMm;
        ++fitN;

        const Int32 n = std::min(fitN, FIT_HISTORY);
        Float32 windowed = 0.0f;
        for(Int32 i = 0; i < n; ++i)
        {
            windowed = std::max(windowed, fitHist[i]);
        }

        const Float32 desired = std::min(std::max(windowed * 1.15f, 750.0f), 16000.0f);

        // Snap the target to a 1/2/5 x 10^n ladder and hold it there. Easing
        // toward a continuous target is what made this jitter: `windowed` moves
        // a little every revolution as values roll out of the ring buffer, so
        // the target moved every revolution, so the view never arrived. A
        // deadband did not fix it either, because it was measured against the
        // eased value, which is itself in motion.
        //
        // Now the target is one of a handful of discrete radii. Between
        // thresholds it is *exactly* constant, so the ease converges and stops.
        if(fitStepMm <= 0.0f)
        {
            fitStepMm = niceStep(desired);
        }
        else if(desired > fitStepMm)
        {
            // Grow immediately: a return outside the current radius is invisible
            // until the view contains it.
            fitStepMm = niceStep(desired);
        }
        else if(desired < fitStepMm * 0.55f)
        {
            // Shrink only when the scene has become MUCH smaller. The wide gap
            // is deliberate hysteresis - without it the view oscillates between
            // two neighboring rungs whenever the scene sits near a boundary.
            fitStepMm = niceStep(desired);
        }

        const Float32 target = std::min(std::max(fitStepMm, 750.0f), 16000.0f);
        const Float32 k = (target > autoRangeMm) ? FIT_RISE : FIT_FALL;
        autoRangeMm += (target - autoRangeMm) * k;
    }
}

Void RadarView::clear()
{
    trail.clear();
    hasData           = false;
    hasNearest        = false;
    nearestMm         = 0.0f;
    nearestBearingDeg = 0.0f;
    measureActive     = false;
    measureMm         = 0.0f;

    // Otherwise a reconnect fits to the previous room for the next 2.4 s.
    fitStepMm = 0.0f;
    fitN = 0;
    for(Int32 i = 0; i < FIT_HISTORY; ++i)
    {
        fitHist[i] = 0.0f;
    }

    // Same reasoning, one step further: without this a reconnect in a different
    // room would draw the PREVIOUS room's walls, decaying, for a minute.
    mapStateFor(this).reset();
}

// ------------------------------------------------------------- view model ---

Void RadarView::fit()
{
    autoFit       = true;
    viewCenterMm = ImVec2(0.0f, 0.0f);
    measureActive = false;
    measureMm     = 0.0f;

    if(radiusPx > 0.0f)
    {
        pxPerMm = radiusPx / std::max(autoRangeMm, 1.0f);
    }
}

Void RadarView::setRangeMm(Float32 mm)
{
    const Float32 r = clampf(mm, MIN_VISIBLE_MM, MAX_VISIBLE_MM);

    autoFit       = false;
    viewCenterMm = ImVec2(0.0f, 0.0f);
    autoRangeMm  = r;
    pxPerMm      = (radiusPx > 0.0f) ? (radiusPx / r) : 0.0f;
}

Float32 RadarView::visibleRangeMm() const noexcept
{
    if(pxPerMm > 0.0f && radiusPx > 0.0f)
    {
        return radiusPx / pxPerMm;
    }
    return autoRangeMm;
}

ImVec2 RadarView::toScreen(const ImVec2& worldMm) const noexcept
{
    return ImVec2(centerPx.x + (worldMm.x - viewCenterMm.x) * pxPerMm,
                  centerPx.y + (worldMm.y - viewCenterMm.y) * pxPerMm);
}

ImVec2 RadarView::toWorld(const ImVec2& screenPx) const noexcept
{
    if(!(pxPerMm > 0.0f))
    {
        return viewCenterMm;
    }

    return ImVec2(viewCenterMm.x + (screenPx.x - centerPx.x) / pxPerMm,
                  viewCenterMm.y + (screenPx.y - centerPx.y) / pxPerMm);
}

// ------------------------------------------------------------------- draw ---

// ---------------------------------------------------------------------------
// The 3D branch of RadarView::draw.
//
// Lives here rather than in scene3d.cxx because it is the part that needs
// radar.cxx's private state - the wall fitter, the clearance map, the trail -
// and scene3d is deliberately kept free of all of it so it stays a renderer.
// ---------------------------------------------------------------------------
Void drawScene3D(RadarView& rv, const MapState& st, ImDrawList* dl, const Rect& area, Float32 dpi, Bool hovered, Bool active)
{
    const ImVec2& p0 = area.p0;
    const ImVec2& p1 = area.p1;
    ImGuiIO& io = ImGui::GetIO();

    // ---- camera control ---------------------------------------------------
    //
    // Left drag orbits, right or middle drag pans, wheel zooms - the same three
    // gestures the flat map already uses for pan and zoom, so the two views do
    // not need separate muscle memory.
    if(active)
    {
        const ImVec2 d = io.MouseDelta;
        if(ImGui::IsMouseDown(ImGuiMouseButton_Left) && !io.KeyShift)
        {
            rv.cam.orbit(-d.x * 0.006f, d.y * 0.006f);
        }
        else if(ImGui::IsMouseDown(ImGuiMouseButton_Right)
             || ImGui::IsMouseDown(ImGuiMouseButton_Middle)
             || (ImGui::IsMouseDown(ImGuiMouseButton_Left) && io.KeyShift))
        {
            // Panning in meters-per-pixel at the target's depth, so the scene
            // tracks the cursor rather than sliding faster when zoomed out.
            const Float32 k = rv.cam.dist / std::max(1.0f, (p1.y - p0.y));
            rv.cam.pan(-d.x * k, d.y * k);
        }
    }

    if(hovered && io.MouseWheel != 0.0f)
    {
        rv.cam.zoom(std::pow(0.88f, io.MouseWheel));
    }

    // Locked to the car: the target is re-pinned every frame rather than only
    // when the lock is switched on, so a target left over from a world-locked
    // pan cannot survive the switch.
    if(rv.cam.lockToCar)
    {
        rv.cam.target = scene3d::Vec3{ 0.0f, 0.0f, 120.0f };
    }

    // ---- the world frame --------------------------------------------------
    //
    // Car lock is the sensor's frame: the car is fixed and the room turns
    // around it. World lock is the room's: the room holds still and the car
    // turns. The difference is one rotation, and the rotation has to be
    // MEASURED, because nothing on this machine reports heading.
    MapState& mst = const_cast<MapState&>(st);
    if(!rv.cam.lockToCar && st.ready)
    {
        // Zeroed on entry: the moment you ask for a world frame is the moment
        // that defines which way "world zero" points.
        if(!mst.refValid)
        {
            for(Int32 i = 0; i < CLR_BINS; ++i)
            {
                mst.refClr[i]  = st.clr[i];
                mst.refSeen[i] = st.clrSeen[i];
            }
            mst.refValid   = true;
            mst.headingDeg = 0.0f;
            mst.headingOk  = 1.0f;
        }
        else
        {
            Float32 deg = 0.0f, score = 0.0f;
            const mapgeo::PolarScan refScan{ mst.refClr.data(), mst.refSeen.data(),
                                             CLR_BINS, CLR_BIN_DEG };
            const mapgeo::PolarScan curScan{ st.clr.data(), st.clrSeen.data(),
                                             CLR_BINS, CLR_BIN_DEG };
            if(mapgeo::estimateHeading(refScan, curScan, deg, score))
            {
                // Eased, not snapped. The estimate is per-revolution and jitters
                // by a fraction of a bin; easing it costs a little lag and buys
                // a world that does not shiver.
                Float32 d = deg - mst.headingDeg;
                while(d >  180.0f)
                {
                    d -= 360.0f;
                }
                while(d < -180.0f)
                {
                    d += 360.0f;
                }
                mst.headingDeg += d * 0.25f;

                mst.headingOk = score;
            }
        }
    }
    else if(rv.cam.lockToCar)
    {
        // Dropped on the way out, so re-entering World lock zeroes on the room
        // as it is THEN rather than on a reference from minutes ago.
        mst.refValid = false;
    }

    const Float32 worldYaw = rv.cam.lockToCar ? 0.0f : mst.headingDeg;

    // ---- what this mode needs ---------------------------------------------
    static Vec<WallSeg> walls;
    static Array<Float32, CLR_BINS> reach;
    static Array<Float32, CLR_BINS> freeR;
    static Array<Bool, CLR_BINS> seenR;

    const Bool haveData = !rv.trailEmpty();

    if(haveData && (rv.scene == scene3d::SceneMode::SCENE_MODE_WALLS
                 || rv.scene == scene3d::SceneMode::SCENE_MODE_FULL))
    {
        fitWalls(rv.lastRevolution(), walls);
    }
    else
    {
        walls.clear();
    }

    Bool haveReach = false;
    if(st.ready && (rv.scene == scene3d::SceneMode::SCENE_MODE_FIT
                 || rv.scene == scene3d::SceneMode::SCENE_MODE_FULL))
    {
        for(Int32 i = 0; i < CLR_BINS; ++i)
        {
            freeR[i] = st.clrSeen[i] ? st.clr[i] : 0.0f;
            seenR[i] = st.clrSeen[i];
        }
        mapgeo::computeReach(mapgeo::PolarScan{ freeR.data(), seenR.data(),
                                                CLR_BINS, CLR_BIN_DEG },
                             EGO_WID_MM * 0.5f + 30.0f, reach.data());
        haveReach = true;
    }

    scene3d::DrawArgs a;
    a.dl      = dl;
    a.p0      = p0;
    a.p1      = p1;
    a.dpi     = dpi;
    a.mode    = rv.scene;
    a.ego     = rv.ego;
    a.worldYawDeg = worldYaw;
    a.worldHeadingOk = rv.cam.lockToCar ? -1.0f : mst.headingOk;

    // Solved from ONE clock, here, once a frame - see lights.hxx. ImGui's time
    // is monotonic and frame-rate independent, which is what the flasher needs.
    a.lamps   = lights::solve(rv.lighting, ImGui::GetTime());

    // Handed in rather than recomputed: these come from the same accumulators
    // the flat map reads, so the two dimensions cannot disagree about them.
    // Detections and the corridor for the ride view, from the SAME fitter the
    // flat map's Full uses. Recomputing them here with a second set of
    // thresholds would let the two dimensions disagree about what is out there.
    static Vec<scene3d::Detection> dets;
    dets.clear();
    Float32 corridorAhead = 0.0f;
    const Float32 corridorHalfW = EGO_WID_MM * 0.5f + 30.0f;

    if(haveData && rv.scene == scene3d::SceneMode::SCENE_MODE_FULL)
    {
        const Vec<LidarPoint>& pts = rv.lastRevolution();
        corridorAhead = corridorFree(pts, corridorHalfW);

        static Vec<Obstacle> obs;
        findObstacles(pts, obs);
        for(Obstacle& o2 : obs)
        {
            const Float32 ex = std::fabs(o2.ux) * o2.halfL + std::fabs(o2.uy) * o2.halfW;
            o2.inPath = (o2.cy < 0.0f) && (std::fabs(o2.cx) - ex <= corridorHalfW);

            scene3d::Detection d;
            d.cx = o2.cx; d.cy = o2.cy;
            d.ux = o2.ux; d.uy = o2.uy;
            d.halfL = o2.halfL; d.halfW = o2.halfW;
            d.nearMm = o2.nearMm; d.inPath = o2.inPath;
            dets.push_back(d);
        }
    }

    a.objects       = dets.empty() ? nullptr : dets.data();
    a.objectN       = static_cast<Int32>(dets.size());
    a.corridorHalfW = corridorHalfW;
    a.corridorFree  = corridorAhead;

    a.hz        = rv.hz();
    a.returns   = static_cast<Int32>(rv.lastRevolution().size());
    a.nearestMm = rv.hasNearest ? rv.nearestMm : 0.0f;
    a.aheadMm   = (corridorAhead > 0.0f) ? corridorAhead
                : (haveReach ? reach[0] : 0.0f);
    a.points  = haveData ? &rv.lastRevolution() : nullptr;
    a.walls   = &walls;
    a.reach   = haveReach ? reach.data() : nullptr;
    a.reachN  = haveReach ? CLR_BINS : 0;
    a.reachBinDeg = CLR_BIN_DEG;
    a.diag    = rv.diag.data();
    a.diagCap = rv.diag.size();

    rv.diag[0] = 0;
    scene3d::draw(rv.cam, a);

    if(!haveData)
    {
        drawPlaceholder(dl, ImVec2((p0.x + p1.x) * 0.5f, (p0.y + p1.y) * 0.5f));
    }
}

Void RadarView::draw(const ImVec2& size)
{
    ImGuiIO&    io  = ImGui::GetIO();
    const Float32 dpi = currentDpi();

    // Occupancy fades in wall-clock time, so its clock ticks whatever is on
    // screen and before any early return: a cell's age must not depend on which
    // mode you were looking at while it aged, nor on the widget being big
    // enough to draw. DeltaTime, never a per-frame constant - see decayOccupancy.
    MapState& map = mapStateFor(this);
    decayOccupancy(map, io.DeltaTime);

    ImVec2 sz = size;
    if(sz.x <= 0.0f)
    {
        sz.x = ImGui::GetContentRegionAvail().x + sz.x;
    }
    if(sz.y <= 0.0f)
    {
        sz.y = ImGui::GetContentRegionAvail().y + sz.y;
    }
    sz.x = std::max(sz.x, 1.0f);
    sz.y = std::max(sz.y, 1.0f);

    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 p1 = ImVec2(p0.x + sz.x, p0.y + sz.y);

    // One hit area owns all three drag buttons, so nothing else in the UI can
    // steal a pan or a measurement half-way through.
    ImGui::InvisibleButton("##radar_view", sz,
                           ImGuiButtonFlags_MouseButtonLeft |
                           ImGuiButtonFlags_MouseButtonRight |
                           ImGuiButtonFlags_MouseButtonMiddle);

    const Bool hovered = ImGui::IsItemHovered();
    const Bool active  = ImGui::IsItemActive();

    // Claim the wheel only while this widget is under the cursor; the rest of
    // the UI keeps its scrolling.
    ImGui::SetItemKeyOwner(ImGuiKey_MouseWheelY);

    centerPx = ImVec2(p0.x + sz.x * 0.5f, p0.y + sz.y * 0.5f);

    const Float32 minSide = std::min(sz.x, sz.y);
    const Float32 margin   = std::min(18.0f * dpi, minSide * 0.08f);
    radiusPx = std::max(minSide * 0.5f - margin, 0.0f);

    cursorValid = false;

    ImDrawList* dl = ImGui::GetWindowDrawList();
    if(dl == nullptr)
    {
        return;
    }

    // ---- the 3D branch ---------------------------------------------------
    //
    // Taken here, after the hit area and the occupancy tick and before any of
    // the flat map's geometry, because everything below this point is about a
    // top-down projection that the scene does not have.
    if(is3D)
    {
        // The nearest return is a property of the DATA, not of the projection,
        // and the telemetry panel reads it. Computing it only on the flat path
        // meant switching to 3D blanked a sensor readout that had nothing to do
        // with the camera.
        hasNearest = false;
        if(!trail.empty())
        {
            const Vec<LidarPoint>& cur = trail.back();
            const LidarPoint* best = nullptr;
            for(const LidarPoint& p : cur)
            {
                if(p.distMm < MIN_VALID_MM || p.distMm > MAX_VALID_MM)
                {
                    continue;
                }
                if(best == nullptr || p.distMm < best->distMm)
                {
                    best = &p;
                }
            }
            if(best != nullptr)
            {
                hasNearest        = true;
                nearestMm         = best->distMm;
                nearestBearingDeg = best->angleDeg;
            }
        }

        drawScene3D(*this, map, dl, Rect{ p0, p1 }, dpi, hovered, active);
        return;
    }

    if(radiusPx < 12.0f * dpi)   // too small to say anything useful
    {
        dl->PushClipRect(p0, p1, true);
        dl->PopClipRect();
        return;
    }

    // ---- scale ------------------------------------------------------------
    const Float32 minPpm = radiusPx / MAX_VISIBLE_MM;
    const Float32 maxPpm = radiusPx / MIN_VISIBLE_MM;

    if(autoFit)
    {
        viewCenterMm = ImVec2(0.0f, 0.0f);
        pxPerMm      = radiusPx / std::max(autoRangeMm, 1.0f);
    }
    else if(!(pxPerMm > 0.0f))
    {
        // set_range_mm() before the first draw, or a resize from nothing.
        pxPerMm = radiusPx / std::max(autoRangeMm, 1.0f);
    }
    pxPerMm = clampf(pxPerMm, minPpm, maxPpm);

    // ---- input ------------------------------------------------------------
    Bool resetNow = false;
    if(hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
    {
        fit();
        pxPerMm = clampf(radiusPx / std::max(autoRangeMm, 1.0f), minPpm, maxPpm);
        resetNow  = true;
    }

    // Pan: left or middle drag, 1:1 with the cursor.
    Bool panning = false;
    if(active && !resetNow &&
        (ImGui::IsMouseDown(ImGuiMouseButton_Left) || ImGui::IsMouseDown(ImGuiMouseButton_Middle)))
    {
        panning = true;
        const ImVec2 d = io.MouseDelta;
        if(d.x != 0.0f || d.y != 0.0f)
        {
            viewCenterMm.x -= d.x / pxPerMm;
            viewCenterMm.y -= d.y / pxPerMm;
            autoFit = false;
        }
    }

    // Zoom about the cursor: the world point under the mouse stays put.
    Float32 wheel = io.MouseWheel;
    if(wheel == 0.0f && io.KeyShift)
    {
        wheel = io.MouseWheelH;      // some backends swap the axis under shift
    }

    // `active` covers a drag that has wandered outside the rect, where
    // IsItemHovered() goes false but this widget still owns the mouse.
    if((hovered || active) && wheel != 0.0f && !resetNow)
    {
        const Float32 step   = io.KeyCtrl ? 1.03f : (io.KeyShift ? 1.30f : 1.10f);
        const Float32 factor = std::pow(step, wheel);
        const ImVec2 anchorWorld = toWorld(io.MousePos);
        const Float32  ppm          = clampf(pxPerMm * factor, minPpm, maxPpm);

        if(ppm != pxPerMm)
        {
            pxPerMm = ppm;
            viewCenterMm.x = anchorWorld.x - (io.MousePos.x - centerPx.x) / ppm;
            viewCenterMm.y = anchorWorld.y - (io.MousePos.y - centerPx.y) / ppm;
            autoFit = false;
        }
    }

    // Measure: right drag, anchored on press.
    if(active && !panning && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
    {
        measureActive  = true;
        measureFromMm = toWorld(io.MousePos);
        measureToMm   = measureFromMm;
        measureMm      = 0.0f;
    }
    if(measureActive)
    {
        if(ImGui::IsMouseDown(ImGuiMouseButton_Right))
        {
            measureToMm = toWorld(io.MousePos);
            const Float32 dx = measureToMm.x - measureFromMm.x;
            const Float32 dy = measureToMm.y - measureFromMm.y;
            measureMm    = std::sqrt(dx * dx + dy * dy);
        }
        else
        {
            measureActive = false;
            measureMm     = 0.0f;
        }
    }

    if(panning)
    {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
    }
    else if(measureActive)
    {
        ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
    }

    // ---- readouts ---------------------------------------------------------
    if(hovered || active)
    {
        const ImVec2 w = toWorld(io.MousePos);
        cursorValid    = true;
        cursorRangeMm = std::sqrt(w.x * w.x + w.y * w.y);

        Float32 b = std::atan2(w.x, -w.y) * (180.0f / PI);   // 0 = up, clockwise
        if(b < 0.0f)
        {
            b += 360.0f;
        }
        cursorBearingDeg = b;
    }

    const Float32 visibleMm = std::max(radiusPx / pxPerMm, 1.0f);
    const ImVec2 s0        = toScreen(ImVec2(0.0f, 0.0f));   // sensor on screen

    // ---- draw -------------------------------------------------------------
    // Order matters: chrome that has to stay legible (every number, the scale
    // bar) goes on *after* the point cloud, everything else underneath it.
    dl->PushClipRect(p0, p1, true);

    // The viewer's own ground. Modes that encode a quantity by hue tint it
    // toward their own palette, so which one is active is legible from the map
    // rather than only from the toggle below it.
    // One flat fill and no light on it. A gradient here would be a claim that
    // the display is a physical panel with a lamp behind it, and every pixel it
    // touches is a pixel whose color is no longer purely its own.
    dl->AddRectFilled(p0, p1, mapModeBackground(mode));

    // Cleared every frame: a mode that has nothing to report must not leave the
    // previous mode's reading on screen.
    diag[0] = 0;

    const Bool bare = (mapModeGrid(mode) == GridStyle::GRID_STYLE_NONE);

    // The rectangle varies between the plot area and the label area; the
    // mapping does not, so it is built once here.
    const Rect     plot{ p0, p1 };
    const MapScale scale{ s0, pxPerMm, dpi };

    GridSpec grid;
    if(showGrid)
    {
        grid = computeGrid(plot, scale, visibleMm, radiusPx);

        // The grid belongs to the MODE now - see mapModeGrid().
        // `bare` is the same decision applied to everything else the map draws
        // around the data.
        switch(mapModeGrid(mode))
        {
        case GridStyle::GRID_STYLE_CARTESIAN:
            drawGridCartesian(dl, grid, plot, scale);
            break;
        case GridStyle::GRID_STYLE_NONE:
            break;
        case GridStyle::GRID_STYLE_RADIAL:
        default:
            drawGridLines(dl, grid, plot, scale);
            break;
        }
        drawBlindZone(dl, plot, scale);
    }

    const Bool havePoints = hasData && !trail.empty();
    if(!havePoints)
    {
        hasNearest = false;
        drawPlaceholder(dl, centerPx);
    }

    if(havePoints)
    {
        const ImVec2 uv   = ImGui::GetFontTexUvWhitePixel();
        const Int32    last = static_cast<Int32>(trail.size()) - 1;

        const Float32  dotR = 2.0f * dpi;
        const ImVec2 cullLo(p0.x - dotR, p0.y - dotR);
        const ImVec2 cullHi(p1.x + dotR, p1.y + dotR);

        MarkCtx mc;
        mc.dl      = dl;
        mc.p0      = p0;
        mc.p1      = p1;
        mc.cullLo = cullLo;
        mc.cullHi = cullHi;
        mc.s0      = s0;
        mc.uv      = uv;
        mc.ppm     = pxPerMm;
        mc.dpi     = dpi;
        mc.dotR   = dotR;
        mc.ego     = ego;
        mc.diag    = diag.data();
        mc.diagCap = diag.size();

        // The ONLY place `mode` is consulted. Everything above it (geometry,
        // gestures, the whole view model) and everything below it (the nearest
        // return, the measurement, every label and readout) is mode-blind.
        switch(mode)
        {
        case MapMode::MAP_MODE_DENSITY:   drawMarksDensity(mc, map); break;
        case MapMode::MAP_MODE_MOTION:    drawMarksMotion(mc, map, trail); break;
        case MapMode::MAP_MODE_CLEARANCE: drawMarksClearance(mc, map, trail); break;
        case MapMode::MAP_MODE_GAPS:      drawMarksGaps(mc, map, trail); break;
        case MapMode::MAP_MODE_WALLS:     drawMarksWalls(mc, trail); break;
        case MapMode::MAP_MODE_CORNERS:   drawMarksCorners(mc, trail); break;
        case MapMode::MAP_MODE_FIT:       drawMarksFit(mc, map, trail); break;
        case MapMode::MAP_MODE_FULL:      drawMarksFull(mc, map, trail, lastHz); break;
        case MapMode::MAP_MODE_MINIMAL:   drawMarksMinimal(mc, map, trail); break;
        case MapMode::MAP_MODE_POINTS:
        case MapMode::MAP_MODE_COUNT:
        default:                          drawMarksPoints(mc, trail, showTrail); break;
        }

        // ---- nearest return -----------------------------------------------
        const Vec<LidarPoint>& cur = trail[static_cast<Size>(last)];

        const Int32 nCur = static_cast<Int32>(cur.size());

        Int32 bestI = -1;
        for(Int32 i = 0; i < nCur; ++i)
        {
            // MIN_VALID_MM, not 0: the C1 is specified from 0.05 m, and the
            // sub-50 mm returns off its own housing would otherwise win this
            // comparison every single frame.
            const LidarPoint& p = cur[static_cast<Size>(i)];
            if(p.distMm < MIN_VALID_MM || p.distMm > MAX_VALID_MM)
            {
                continue;
            }
            if(bestI < 0 || p.distMm < cur[static_cast<Size>(bestI)].distMm)
            {
                bestI = i;
            }
        }

        hasNearest = (bestI >= 0);
        if(bestI >= 0)
        {
            const LidarPoint* best = &cur[static_cast<Size>(bestI)];

            nearestMm = best->distMm;

            Float32 b = std::fmod(best->angleDeg, 360.0f);
            if(b < 0.0f)
            {
                b += 360.0f;
            }
            nearestBearingDeg = b;

            // Minimal overrides the checkboxes rather than reading them. The
            // mode IS the statement "no instrument chrome"; leaving a red
            // nearest-return marker on a display meant for an audience because
            // a debug toggle happened to be ticked would defeat the mode.
            if(showNearest && !bare)
            {
                const Float32  ang = (best->angleDeg - 90.0f) * (PI / 180.0f);
                const Float32  rr  = best->distMm * pxPerMm;
                const ImVec2 np(s0.x + rr * std::cos(ang), s0.y + rr * std::sin(ang));

                // Highlight the whole nearest *object*, not just the single
                // closest sample. Returns are angle-sorted, so the object is the
                // contiguous run of samples around `best` that stays at a
                // similar range - a wall or a hand spans many samples, and
                // ringing one of them says nothing about its extent.
                Vec<Dot>& hot = nearestScratch();
                gatherNearestCluster(cur, bestI, s0, pxPerMm, hot);
                emitDiscs(dl, hot.data(), static_cast<Int32>(hot.size()),
                          dotR * 1.35f, NEAREST_COL, uv);

                // The one return the map is calling out, so it is lit rather
                // than outlined: a halo, the ring, then the core.
                dl->AddCircleFilled(np, 13.0f * dpi,
                                    (NEAREST_COL & 0x00FFFFFFu)
                                    | (static_cast<ImU32>(30u) << IM_COL32_A_SHIFT), 20);
                dl->AddCircle(np, 10.0f * dpi, NEAREST_COL, 20, 1.6f * dpi);
                dl->AddCircleFilled(np, 2.6f * dpi, NEAREST_COL, 12);

                if(showLabels)
                {
                    Array<Char, 24> buf;
                    formatDist(buf.data(), buf.size(), best->distMm);
                    plateText(dl, ImVec2(np.x + 14.0f * dpi, np.y - 6.0f * dpi),
                              dpi, NEAREST_COL, buf.data());
                }
            }
        }
    }

    // ---- measurement ------------------------------------------------------
    if(measureActive)
    {
        const ImVec2 a = toScreen(measureFromMm);
        const ImVec2 b = toScreen(measureToMm);

        dl->AddLine(a, b, MEASURE_COL, 1.6f * dpi);
        dl->AddCircleFilled(a, 3.5f * dpi, MEASURE_COL, 12);
        dl->AddCircleFilled(b, 3.5f * dpi, MEASURE_COL, 12);

        Array<Char, 24> buf;
        formatDist(buf.data(), buf.size(), measureMm);

        ImFont*      font = labelFont();
        const ImVec2 ts   = font->CalcTextSizeA(labelPx(), FLT_MAX, 0.0f, buf.data());
        plateText(dl, ImVec2((a.x + b.x) * 0.5f - ts.x * 0.5f,
                             (a.y + b.y) * 0.5f - ts.y - 6.0f * dpi),
                  dpi, MEASURE_COL, buf.data());
    }

    drawHub(dl, s0, dpi);

    // ---- chrome that must stay readable over the points --------------------
    // Orientation matters enough to survive either overlay toggle.
    if(showGrid || showLabels)
    {
        drawHeadingArrow(dl, plot, scale, radiusPx);
    }

    // The app draws its own status text in the top and bottom gutters of this
    // rect, so every map label is placed inside an inset copy of it.
    const ImVec2 lab0(p0.x + 12.0f * dpi, p0.y + 30.0f * dpi);
    const ImVec2 lab1(p1.x - 12.0f * dpi, p1.y - 30.0f * dpi);

    if(lab1.x > lab0.x && lab1.y > lab0.y)
    {
        if(showGrid && showLabels)
        {
            // Labels follow the same style choice the lines did.
            switch(mapModeGrid(mode))
            {
            case GridStyle::GRID_STYLE_CARTESIAN:
                drawGridCartesianLabels(dl, grid, Rect{ lab0, lab1 }, scale);
                break;
            case GridStyle::GRID_STYLE_NONE:
                break;
            case GridStyle::GRID_STYLE_RADIAL:
            default:
                drawGridLabels(dl, grid, Rect{ lab0, lab1 },
                               Rect{ ImVec2(p0.x + 4.0f * dpi, p0.y + 4.0f * dpi),
                                     ImVec2(p1.x - 4.0f * dpi, p1.y - 4.0f * dpi) },
                               scale);
                break;
            }

            if(mapModeGrid(mode) != GridStyle::GRID_STYLE_NONE)
            {
                drawBlindLabel(dl, Rect{ lab0, lab1 }, scale);
            }
        }

        if(showLabels)
        {
            if(!bare)
            {
                drawScaleBar(dl, Rect{ lab0, lab1 }, scale);
            }
        }
    }

    dl->PopClipRect();
}
