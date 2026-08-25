// Interactive 2D map of recent scan revolutions, drawn with ImDrawList.
//
// World space is millimetres with the sensor at the origin, x right, y down on
// screen; a measurement at (angle, dist) maps to (dist*sin a, -dist*cos a), so
// 0 degrees points up and angle increases clockwise. Screen space follows from
//     screen = center_px + (world_mm - view_center_mm) * px_per_mm
// which is the only place the view model enters the renderer: pan moves
// view_center_mm, zoom scales px_per_mm.
//
// The hot path is ~5 revolutions x ~500 points x 1 disc per frame at 60 fps,
// so the points are emitted as untessellated 8-gons written straight into the
// draw list through PrimReserve/PrimWriteVtx/PrimWriteIdx. Every point in a
// revolution shares one flat colour, so the colour is hoisted out of the inner
// loop entirely: distance is read off the rings, not off a colour ramp.
//
// NOTE: the draw list caches _VtxWritePtr/_IdxWritePtr/_VtxCurrentIdx into its
// buffers. Growing VtxBuffer/IdxBuffer behind its back leaves those pointers
// dangling into freed memory, so all reservation goes through PrimReserve(),
// and _VtxCurrentIdx is re-read *after* every call (a large-mesh vertex-offset
// split resets it to 0).

#include "radar.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <vector>

#include "theme.h"

namespace {

constexpr size_t kMaxTrail = 5;      // revolutions kept, including the current one
constexpr int    kSegs     = 8;      // segments per point disc (2-5 px across)
constexpr float  kPi       = 3.14159265358979323846f;

// Shortest return treated as real. The C1 is specified from 0.05 m; anything
// closer is a reflection off its own housing and must not drive the auto-fit
// or win the "nearest obstacle" readout.
constexpr float kMinValidMm = 50.0f;

// Longest return treated as real. The C1 is specified to 12 m; beyond that the
// device still occasionally reports something, but it is unreliable. Drawing
// those while the readouts discarded them is the same contradiction the blind
// zone exists to prevent, so the window is applied in both places.
constexpr float kMaxValidMm = 12000.0f;

// Below this radius the blind disc is too small to render as anything but a
// smudge, so it is skipped and the sensor hub stands in for it.
//
// It is deliberately NOT a floor on the drawn radius. Inflating the disc to a
// legible minimum marks area the sensor can see perfectly well as blind: at a
// typical fit zoom a 16 dp floor covered ~200 mm against a real blind radius of
// 50 mm, so a hand held near the unit produced returns that landed inside the
// disc labelled "cannot see here". The disc is drawn true to scale; zoom in to
// inspect it.
constexpr float kBlindMinPx = 4.0f;

// Every range-ring label sits on this bearing (deg, 0 = up, clockwise), so the
// labels form one legible radial column instead of landing wherever they fit.
// Deliberately off both the cardinals (0/90/180/270) and the intercardinals
// (45/135/...) that carry the compass numbers.
constexpr float kRingLabelBearing = 25.0f;

// Zoom limits, expressed as the visible radius (centre -> nearer edge) in mm.
constexpr float kMinVisibleMm = 50.0f;      //  5 cm across the short half-axis
constexpr float kMaxVisibleMm = 40000.0f;   // 40 m

// Auto-fit easing. Growth is much faster than shrink: a return that lands
// outside the current range has to appear now, while a range that has become
// too large may settle back slowly without anything popping.
constexpr float kFitRise = 0.35f;
constexpr float kFitFall = 0.06f;

// Discs written per PrimReserve() call. Keeps any single reservation far below
// the 64K vertex ceiling of a 16-bit ImDrawIdx.
constexpr int kDiscBatch = 2048;

// ---------------------------------------------------------------- palette ---

// Map chrome. These are plot colours, not UI chrome, so they are explicit
// rather than following ImGui's theme - see theme.h.
constexpr ImU32 kRingCol     = (ui::plot::grid       & 0x00FFFFFFu) | (0x8Cu << IM_COL32_A_SHIFT);
constexpr ImU32 kRingMajor   = (ui::plot::grid_major & 0x00FFFFFFu) | (0xDCu << IM_COL32_A_SHIFT);
// Rings beyond the fitted range only ever clip a corner of the widget. Drawn
// faint so those slivers stop reading as stray diagonal strokes.
constexpr ImU32 kRingFaint   = (ui::plot::grid       & 0x00FFFFFFu) | (0x44u << IM_COL32_A_SHIFT);
constexpr ImU32 kAxisCol     = ui::plot::axis;
constexpr ImU32 kHeadingCol  = ui::plot::heading;
constexpr ImU32 kHubCol      = ui::plot::hub;
constexpr ImU32 kHubCoreCol  = ui::plot::hub_core;
constexpr ImU32 kEmptyCol    = ui::plot::label;
constexpr ImU32 kNearestCol  = ui::plot::nearest;
constexpr ImU32 kMeasureCol  = ui::plot::measure;

// Label plate: dark enough to punch a hole in a dense point cluster, but not so
// opaque that the map reads as a grid of boxes.
constexpr ImU32 kPlateBg     = IM_COL32(0x0C, 0x0D, 0x11, 0xCC);

// Ring / compass label inks. Distances read brighter than bearings so the two
// families of number never get confused for one another.
constexpr ImU32 kRingTextCol = IM_COL32(0xC6, 0xCC, 0xD8, 0xFF);
constexpr ImU32 kBearingCol  = IM_COL32(0x94, 0x9A, 0xA6, 0xFF);
constexpr ImU32 kCardinalCol = IM_COL32(0xC0, 0xC6, 0xD2, 0xFF);
constexpr ImU32 kTickCol     = IM_COL32(0x76, 0x7C, 0x88, 0xFF);
constexpr ImU32 kTickMajorCol= IM_COL32(0xA6, 0xAC, 0xB8, 0xFF);
constexpr ImU32 kScaleCol    = IM_COL32(0xD4, 0xD8, 0xE2, 0xFF);

// Blind zone: the C1 returns nothing inside its 0.05 m spec floor. Drawn as a
// hatched red-ish disc so it reads as "cannot see here", not "nothing here".
// Deliberately a chalky, desaturated rose rather than a vivid warning orange:
// at the smallest drawn size it sits directly under the nearest-return ring,
// and the two must not read as one blob.
constexpr ImU32 kBlindFill   = IM_COL32(0xC0, 0x92, 0x8C, 0x2A);
constexpr ImU32 kBlindHatch  = IM_COL32(0xD2, 0xA6, 0xA0, 0x59);
constexpr ImU32 kBlindEdge   = IM_COL32(0xE2, 0xBC, 0xB4, 0xDC);
constexpr ImU32 kBlindText   = IM_COL32(0xE6, 0xC6, 0xBE, 0xFF);

// The far end of the same envelope: the 12 m spec limit. Same chalky family as
// the blind zone so the two read as a matched pair rather than two unrelated
// annotations, but dimmer - it is a boundary, not a hazard.
constexpr ImU32 kRangeLimitCol = IM_COL32(0xC0, 0x92, 0x8C, 0x8C);

// Scan points. One flat neutral for every return at every distance: range is
// what the rings and the scale bar are for, so tinting the points by it only
// made the map look like a heatmap of something it was not measuring. Alpha is
// supplied per revolution, so the RGB is kept separate from it here.
constexpr ImU32 kPointRgb    = IM_COL32(0xE6, 0xEA, 0xF2, 0x00);

// Unit 8-gon, reused for every point so the inner loop needs no trig for the
// disc itself. Radius is nudged out slightly so the polygon covers about as
// much area as the circle it stands in for.
struct UnitNgon
{
    ImVec2 v[kSegs];

    UnitNgon()
    {
        for (int i = 0; i < kSegs; ++i)
        {
            const float a = (float)i * (2.0f * kPi / (float)kSegs);
            v[i] = ImVec2(std::cos(a) * 1.045f, std::sin(a) * 1.045f);
        }
    }
};

const UnitNgon& Ngon()
{
    static const UnitNgon n;
    return n;
}

// ------------------------------------------------------------------ utils ---

inline float Clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

float Dpi()
{
    const float s = ui::DpiScale();
    return (s > 0.0f && s < 16.0f) ? s : 1.0f;
}

// Map labels ride the app's "small" type role rather than a hardcoded pixel
// size. LegacySize already has DPI baked in by LoadFonts, so it is never
// multiplied by Dpi() again.
ImFont* LabelFont()
{
    return ui::fonts.small ? ui::fonts.small : ImGui::GetFont();
}

float LabelPx()
{
    ImFont* f = ui::fonts.small;
    return (f && f->LegacySize > 0.0f) ? f->LegacySize : 15.0f * Dpi();
}

// Rounds up to the next "nice" 1 / 2 / 5 x 10^n value, so ring spacing reads
// as a round number at every zoom level.
float NiceStep(float raw)
{
    if (!(raw > 0.0f))
        return 1.0f;

    const float e = std::floor(std::log10(raw));
    const float p = std::pow(10.0f, e);
    const float m = raw / p;                    // 1 .. 10

    float s;
    if      (m < 1.5f) s = 1.0f;
    else if (m < 3.5f) s = 2.0f;
    else if (m < 7.5f) s = 5.0f;
    else               s = 10.0f;
    return s * p;
}

// Rounds *down* to the next "nice" 1 / 2 / 5 x 10^n value. The scale bar needs
// this direction: its drawn length must never exceed the budget it was given.
float NiceStepDown(float raw)
{
    if (!(raw > 0.0f))
        return 1.0f;

    const float e = std::floor(std::log10(raw));
    const float p = std::pow(10.0f, e);
    const float m = raw / p;                    // 1 .. 10

    float s;
    if      (m < 2.0f) s = 1.0f;
    else if (m < 5.0f) s = 2.0f;
    else               s = 5.0f;
    return s * p;
}

// Ring labels: round metres above a metre, millimetres below.
void FormatRing(char* buf, size_t n, float mm)
{
    if (mm >= 1000.0f) std::snprintf(buf, n, "%.1f m", (double)mm / 1000.0);
    else               std::snprintf(buf, n, "%.0f mm", (double)mm);
}

// Readout labels: as much precision as the magnitude deserves.
void FormatDist(char* buf, size_t n, float mm)
{
    if      (mm < 1000.0f)  std::snprintf(buf, n, "%.0f mm", (double)mm);
    else if (mm < 10000.0f) std::snprintf(buf, n, "%.2f m", (double)mm / 1000.0);
    else                    std::snprintf(buf, n, "%.1f m", (double)mm / 1000.0);
}

// ------------------------------------------------------------- primitives ---

// One screen-space point disc. Colour is uniform across a revolution and so is
// passed to EmitDiscs() once rather than stored per point.
struct Dot { float x, y; };

// Reused between revolutions and frames so the per-frame cost is a memcpy-free
// refill rather than an allocation. ImGui is single-threaded, so a file-local
// buffer is safe here.
std::vector<Dot>& Scratch()
{
    static std::vector<Dot> s;
    return s;
}

// Separate buffer for the nearest-object highlight: it is emitted while the
// main scratch still holds the current revolution's dots.
std::vector<Dot>& NearestScratch()
{
    static std::vector<Dot> s;
    return s;
}

// Filled convex 8-gons written directly into the draw list. Skips ImGui's arc
// tessellation and anti-aliased fringe, which together cost roughly 3x the
// vertices and a pair of trig calls per segment.
//
// Exactly kSegs vertices and (kSegs-2)*3 == 18 indices are written per disc,
// which is precisely what is reserved for it.
// Grows outward from the closest sample to cover the whole nearest surface.
//
// Returns arrive angle-sorted, so a physical object is a contiguous run of
// samples at a similar range. Two samples belong to the same object when the
// angular gap between them is small (no missed returns in between) and the
// radial step is small (not a jump to something behind it). The run is capped
// so a smooth wall cannot drag the highlight around the entire room.
void GatherNearestCluster(const std::vector<LidarPoint>& pts, int best_i,
                          const ImVec2& s0, float ppm, std::vector<Dot>& out)
{
    out.clear();

    const int n = (int)pts.size();
    if (n <= 0 || best_i < 0 || best_i >= n)
        return;

    constexpr float kMaxAngleGapDeg = 3.0f;   // samples sit ~0.7 deg apart
    constexpr int   kMaxSamples     = 140;    // ~100 deg of a dense revolution

    const float deg2rad = kPi / 180.0f;

    auto emit = [&](int i)
    {
        const LidarPoint& p = pts[(size_t)i];
        const float rr  = p.dist_mm * ppm;
        const float ang = (p.angle_deg - 90.0f) * deg2rad;
        out.push_back(Dot{ s0.x + rr * std::cos(ang), s0.y + rr * std::sin(ang) });
    };

    // A step is "smooth" relative to the range it is at: 60 mm of slop at the
    // sensor, proportionally more further out where samples are further apart.
    auto joins = [](float a_mm, float b_mm)
    {
        const float tol = std::max(60.0f, 0.12f * std::min(a_mm, b_mm));
        return std::fabs(a_mm - b_mm) <= tol;
    };

    auto in_window = [](const LidarPoint& p)
    {
        return p.dist_mm >= kMinValidMm && p.dist_mm <= kMaxValidMm;
    };

    auto angle_gap = [](float from_deg, float to_deg)
    {
        float d = std::fabs(to_deg - from_deg);
        if (d > 180.0f) d = 360.0f - d;         // the wrap at 0/360
        return d;
    };

    emit(best_i);

    // Walk both ways round the revolution, wrapping modulo n.
    for (int dir = -1; dir <= 1; dir += 2)
    {
        int prev = best_i;
        for (int step = 1; step < kMaxSamples; ++step)
        {
            const int i = ((best_i + dir * step) % n + n) % n;
            if (i == best_i)
                break;                          // wrapped all the way round

            const LidarPoint& p = pts[(size_t)i];
            const LidarPoint& q = pts[(size_t)prev];

            if (!in_window(p)) break;
            if (angle_gap(q.angle_deg, p.angle_deg) > kMaxAngleGapDeg) break;
            if (!joins(q.dist_mm, p.dist_mm)) break;

            emit(i);
            prev = i;
        }
    }
}

void EmitDiscs(ImDrawList* dl, const Dot* dots, int count,
               float r, ImU32 col, const ImVec2& uv)
{
    if (count <= 0 || r < 0.35f || (col & IM_COL32_A_MASK) == 0)
        return;

    const ImVec2* u = Ngon().v;

    for (int start = 0; start < count; start += kDiscBatch)
    {
        const int n = std::min(kDiscBatch, count - start);

        dl->PrimReserve(n * (kSegs - 2) * 3, n * kSegs);

        // Must be read after PrimReserve(): a large-mesh split resets it to 0.
        unsigned int base = dl->_VtxCurrentIdx;

        for (int k = 0; k < n; ++k)
        {
            const Dot& d = dots[start + k];

            for (int i = 0; i < kSegs; ++i)
                dl->PrimWriteVtx(ImVec2(d.x + u[i].x * r, d.y + u[i].y * r), uv, col);

            for (unsigned int i = 2; i < (unsigned int)kSegs; ++i)
            {
                dl->PrimWriteIdx((ImDrawIdx)base);
                dl->PrimWriteIdx((ImDrawIdx)(base + i - 1u));
                dl->PrimWriteIdx((ImDrawIdx)(base + i));
            }
            base += (unsigned int)kSegs;
        }
    }
}

// Strokes only the part of a circle that can fall inside the widget. Range
// rings are centred on the sensor, which may sit far outside the rect once the
// view is panned; tessellating the whole circle would then cost thousands of
// segments for a few visible pixels.
void StrokeRing(ImDrawList* dl, const ImVec2& c, float r,
                const ImVec2& p0, const ImVec2& p1, ImU32 col, float th)
{
    if (r <= 0.75f)
        return;

    const bool inside = (c.x >= p0.x && c.x <= p1.x && c.y >= p0.y && c.y <= p1.y);
    if (inside)
    {
        int seg = (int)(2.0f * kPi * r / 12.0f);
        seg = (int)Clampf((float)seg, 16.0f, 512.0f);
        dl->AddCircle(c, r, col, seg, th);
        return;
    }

    // The sensor is outside the rect, so the rect subtends less than 180 deg
    // from it: the corner bearings, taken relative to the rect centre, bracket
    // the only arc worth drawing.
    const ImVec2 rc((p0.x + p1.x) * 0.5f, (p0.y + p1.y) * 0.5f);
    const float  ac = std::atan2(rc.y - c.y, rc.x - c.x);

    const ImVec2 corner[4] = { p0, ImVec2(p1.x, p0.y), p1, ImVec2(p0.x, p1.y) };
    float lo = 0.0f, hi = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
        float a = std::atan2(corner[i].y - c.y, corner[i].x - c.x) - ac;
        while (a >  kPi) a -= 2.0f * kPi;
        while (a < -kPi) a += 2.0f * kPi;
        lo = std::min(lo, a);
        hi = std::max(hi, a);
    }

    const float span = hi - lo;
    if (span <= 0.0f)
        return;

    int seg = (int)(span * r / 12.0f);          // ~12 px chords
    seg = (int)Clampf((float)seg, 8.0f, 512.0f);

    dl->PathArcTo(c, r, ac + lo, ac + hi, seg);
    dl->PathStroke(col, th);
}

// Only the visible slice of a dashed circle. Same visibility reasoning as
// StrokeRing, but the blind disc is small so the whole ring is cheap either way.
void DashedRing(ImDrawList* dl, const ImVec2& c, float r, ImU32 col,
                float th, float dash_px)
{
    if (r <= 1.5f)
        return;

    int dashes = (int)(2.0f * kPi * r / (dash_px * 2.0f));
    dashes = (int)Clampf((float)dashes, 8.0f, 72.0f);

    const float sweep = 2.0f * kPi / (float)dashes;
    for (int i = 0; i < dashes; ++i)
    {
        const float a0 = (float)i * sweep;
        dl->PathArcTo(c, r, a0, a0 + sweep * 0.55f, 3);
        dl->PathStroke(col, th);
    }
}

// 45-degree hatching clipped to a disc, by chord. Line count is capped so this
// stays a fixed small cost however far the disc is zoomed in.
void HatchDisc(ImDrawList* dl, const ImVec2& c, float r, ImU32 col,
               float th, float spacing)
{
    if (r <= 3.0f)
        return;

    float h = std::max(spacing, 2.0f * r / 22.0f);
    const float k = 0.70710678f;                 // cos/sin 45 deg

    for (float d = -r + h * 0.5f; d < r; d += h)
    {
        const float half = std::sqrt(std::max(r * r - d * d, 0.0f));
        if (half < 1.0f)
            continue;

        // n = (-k, k) is the offset direction, u = (k, k) runs along the line.
        const float mx = c.x - k * d;
        const float my = c.y + k * d;
        dl->AddLine(ImVec2(mx - k * half, my - k * half),
                    ImVec2(mx + k * half, my + k * half), col, th);
    }
}

// Rounded plate behind a label, so numbers survive being drawn over a dense
// point cluster. `tl` is the text's top-left.
void Plate(ImDrawList* dl, const ImVec2& tl, const ImVec2& ts, float dpi)
{
    const float px = 5.0f * dpi;
    const float py = 2.5f * dpi;
    dl->AddRectFilled(ImVec2(tl.x - px, tl.y - py),
                      ImVec2(tl.x + ts.x + px, tl.y + ts.y + py),
                      kPlateBg, 4.0f * dpi);
}

// Text with a plate behind it, anchored by its top-left corner.
void PlateText(ImDrawList* dl, const ImVec2& pos, float dpi, ImU32 col, const char* txt)
{
    ImFont*      font = LabelFont();
    const float  fs   = LabelPx();
    const ImVec2 ts   = font->CalcTextSizeA(fs, FLT_MAX, 0.0f, txt);

    Plate(dl, pos, ts, dpi);
    dl->AddText(font, fs, pos, col, txt);
}

// Text with a plate behind it, anchored by its centre. Returns false without
// drawing when the plate would not fit inside [p0,p1].
bool PlateTextAt(ImDrawList* dl, const ImVec2& mid, const ImVec2& p0, const ImVec2& p1,
                 float dpi, ImU32 col, const char* txt)
{
    ImFont*      font = LabelFont();
    const float  fs   = LabelPx();
    const ImVec2 ts   = font->CalcTextSizeA(fs, FLT_MAX, 0.0f, txt);
    const ImVec2 tl(mid.x - ts.x * 0.5f, mid.y - ts.y * 0.5f);

    const float px = 5.0f * dpi;
    const float py = 2.5f * dpi;
    if (tl.x - px < p0.x || tl.y - py < p0.y ||
        tl.x + ts.x + px > p1.x || tl.y + ts.y + py > p1.y)
        return false;

    Plate(dl, tl, ts, dpi);
    dl->AddText(font, fs, tl, col, txt);
    return true;
}

// ----------------------------------------------------------------- pieces ---

// Which rings exist this frame, and which one carries the compass. Computed
// once, then consumed by the lines pass (under the points) and the labels pass
// (over them) so numbers are never buried by a dense cluster.
struct GridSpec
{
    bool  on        = false;
    float step_mm   = 0.0f;
    float step_px   = 0.0f;
    int   i0        = 1;
    int   i1        = 0;
    int   compass_i = 1;
    float compass_r = 0.0f;     // px, from the sensor
    float ppm       = 0.0f;     // px per mm, for range-limit drawing
    bool  centred   = false;    // is the sensor itself inside the widget?
};

// `s0` is where the sensor lands on screen; it may be well outside [p0,p1].
GridSpec ComputeGrid(const ImVec2& p0, const ImVec2& p1, const ImVec2& s0,
                     float ppm, float visible_mm, float radius_px)
{
    GridSpec g;

    g.ppm     = ppm;
    g.step_mm = NiceStep(visible_mm / 4.0f);
    g.step_px = g.step_mm * ppm;
    if (!(g.step_px > 0.5f))
        return g;

    // Ring index window: from the nearest point of the rect to its farthest
    // corner. Those differ by at most the rect diagonal, so the count is
    // naturally bounded by the widget size, not by how far the view is panned.
    const float nx = Clampf(s0.x, p0.x, p1.x);
    const float ny = Clampf(s0.y, p0.y, p1.y);
    const float near_d = std::sqrt((nx - s0.x) * (nx - s0.x) + (ny - s0.y) * (ny - s0.y));

    g.centred = (near_d <= 0.0f);

    float far_d = 0.0f;
    const ImVec2 corner[4] = { p0, ImVec2(p1.x, p0.y), p1, ImVec2(p0.x, p1.y) };
    for (int i = 0; i < 4; ++i)
    {
        const float dx = corner[i].x - s0.x;
        const float dy = corner[i].y - s0.y;
        far_d = std::max(far_d, std::sqrt(dx * dx + dy * dy));
    }

    g.i0 = (int)std::floor(near_d / g.step_px);
    g.i1 = (int)std::ceil(far_d / g.step_px);
    if (g.i0 < 1) g.i0 = 1;
    if (g.i1 > g.i0 + 32) g.i1 = g.i0 + 32;

    // The outermost ring that still fits inside the fitted radius carries the
    // compass rose. Rounding to nearest instead would let it poke past the top
    // and bottom of the widget, taking its ticks and its 0 / 180 labels with it.
    g.compass_i = (int)std::floor(radius_px / g.step_px);
    if (g.compass_i < 1) g.compass_i = 1;
    g.compass_r = (float)g.compass_i * g.step_px;

    g.on = true;
    return g;
}

// Screen-space unit vector for a bearing in degrees: 0 is up, clockwise.
inline ImVec2 BearingDir(float deg)
{
    const float a = deg * (kPi / 180.0f);
    return ImVec2(std::sin(a), -std::cos(a));
}

// Rings, axes and compass ticks. Drawn beneath the point cloud.
void DrawGridLines(ImDrawList* dl, const GridSpec& g, const ImVec2& p0, const ImVec2& p1,
                   const ImVec2& s0, float dpi)
{
    if (!g.on)
        return;

    // Axes through the sensor, spanning the widget.
    if (s0.y >= p0.y && s0.y <= p1.y)
        dl->AddLine(ImVec2(p0.x, s0.y), ImVec2(p1.x, s0.y), kAxisCol, 1.0f * dpi);
    if (s0.x >= p0.x && s0.x <= p1.x)
        dl->AddLine(ImVec2(s0.x, p0.y), ImVec2(s0.x, p1.y), kAxisCol, 1.0f * dpi);

    // The device's 12 m spec limit. Dashed, matching the blind zone's treatment
    // at the other end of the range, so the pair reads as one envelope: nothing
    // is drawn inside the inner disc or outside this ring.
    {
        const float r = kMaxValidMm * g.ppm;
        if (r > 8.0f * dpi &&
            !(s0.x + r < p0.x || s0.x - r > p1.x || s0.y + r < p0.y || s0.y - r > p1.y))
            DashedRing(dl, s0, r, kRangeLimitCol, 1.4f * dpi, 7.0f * dpi);
    }

    // Emphasis, brightest first: the compass ring (the fitted range), then every
    // fifth ring inside it, then the ordinary rings, then the slivers beyond the
    // fitted range that only ever clip a corner.
    for (int i = g.i0; i <= g.i1; ++i)
    {
        // The faint tier only applies while the sensor is on screen and the
        // compass ring is therefore meaningful; once it has been panned or
        // zoomed away, every visible ring is carrying the reading.
        ImU32 col; float th;
        if      (!g.centred)       { col = ((i % 5) == 0) ? kRingMajor : kRingCol;
                                     th  = ((i % 5) == 0) ? 1.3f : 1.0f; }
        else if (i == g.compass_i) { col = kRingMajor; th = 1.7f; }
        else if (i >  g.compass_i) { col = kRingFaint; th = 1.0f; }
        else if ((i % 5) == 0)     { col = kRingMajor; th = 1.3f; }
        else                       { col = kRingCol;   th = 1.0f; }

        StrokeRing(dl, s0, (float)i * g.step_px, p0, p1, col, th * dpi);
    }

    // Bearing ticks around the compass ring: every 15 deg, longer every 45.
    // They point outwards so they never add clutter inside the map.
    if (g.compass_r > 6.0f)
    {
        for (int b = 0; b < 360; b += 15)
        {
            const ImVec2 d = BearingDir((float)b);
            const ImVec2 a(s0.x + d.x * g.compass_r, s0.y + d.y * g.compass_r);
            if (a.x < p0.x || a.x > p1.x || a.y < p0.y || a.y > p1.y)
                continue;

            const bool  cardinal = (b % 90) == 0;
            const bool  major    = (b % 45) == 0;
            const float len      = (major ? 9.0f : 5.0f) * dpi;
            const ImU32 col      = (b == 0) ? kHeadingCol
                                            : (major ? kTickMajorCol : kTickCol);

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
void DrawGridLabels(ImDrawList* dl, const GridSpec& g, const ImVec2& p0, const ImVec2& p1,
                    const ImVec2& f0, const ImVec2& f1, const ImVec2& s0, float dpi)
{
    if (!g.on)
        return;

    // Ring distances, all on one bearing so they read as a column. When a ring
    // is too big for that bearing to still be on screen, the search sweeps
    // *clockwise* down the right-hand side rather than jumping about, so the
    // numbers stay in reading order however far out they go; only if the whole
    // right side is off-widget does it mirror to the left.
    ImFont*      font   = LabelFont();
    const float  fs     = LabelPx();
    const ImVec2 probe  = font->CalcTextSizeA(fs, FLT_MAX, 0.0f, "0.0 m");
    const bool   room   = g.step_px > probe.y * 1.9f;   // else the column collides

    if (room)
    {
        for (int i = g.i0; i <= g.i1; ++i)
        {
            const float r = (float)i * g.step_px;
            if (r < 10.0f * dpi)
                continue;

            char buf[24];
            FormatRing(buf, sizeof(buf), (float)i * g.step_mm);

            for (int k = 0; k < 22; ++k)
            {
                const float sweep = (float)(k % 11) * 15.0f;
                const float sign  = (k < 11) ? 1.0f : -1.0f;
                const ImVec2 d = BearingDir(sign * (kRingLabelBearing + sweep));

                if (PlateTextAt(dl, ImVec2(s0.x + d.x * r, s0.y + d.y * r),
                                p0, p1, dpi, kRingTextCol, buf))
                    break;
            }
        }
    }

    // Bearing numbers, inside the compass ring so they stay clear of the
    // widget edge at every zoom.
    if (g.compass_r > 34.0f * dpi)
    {
        const float rl = g.compass_r - 21.0f * dpi;

        for (int b = 0; b < 360; b += 45)
        {
            char buf[8];
            std::snprintf(buf, sizeof(buf), "%d", b);

            const ImVec2 d = BearingDir((float)b);
            const ImU32  col = (b == 0)        ? kHeadingCol
                             : ((b % 90) == 0) ? kCardinalCol
                                               : kBearingCol;

            PlateTextAt(dl, ImVec2(s0.x + d.x * rl, s0.y + d.y * rl),
                        f0, f1, dpi, col, buf);
        }
    }
}

// The sensor's dead zone: nothing inside kMinValidMm is real. Hatched rather
// than merely empty, so it reads as "cannot see here". Drawn true to scale - see
// kBlindMinPx for why it is never inflated to a legible minimum.
void DrawBlindZone(ImDrawList* dl, const ImVec2& p0, const ImVec2& p1, const ImVec2& s0,
                   float ppm, float dpi)
{
    const float r = kMinValidMm * ppm;

    // Too small to read as a region; the hub already occupies this area.
    if (r < kBlindMinPx * dpi)
        return;

    if (s0.x + r < p0.x || s0.x - r > p1.x || s0.y + r < p0.y || s0.y - r > p1.y)
        return;

    const int seg = (int)Clampf(r * 0.6f, 20.0f, 96.0f);
    dl->AddCircleFilled(s0, r, kBlindFill, seg);
    HatchDisc(dl, s0, r, kBlindHatch, 1.0f * dpi, 6.0f * dpi);
    DashedRing(dl, s0, r, kBlindEdge, 1.7f * dpi, 4.5f * dpi);
}

// Its caption, drawn with the rest of the labels so it lands over the points.
//
// Only ever drawn *inside* the disc, and only once the disc is genuinely wide
// enough to hold it. Hanging it off the outside instead would put it exactly
// where the nearest-return readout lands whenever the closest obstacle is near
// the sensor - which, at the zoom levels where this caption shows at all, is
// most of the time.
void DrawBlindLabel(ImDrawList* dl, const ImVec2& p0, const ImVec2& p1, const ImVec2& s0,
                    float ppm, float dpi)
{
    const float r = kMinValidMm * ppm;

    ImFont*      font = LabelFont();
    const float  fs   = LabelPx();
    const char*  txt  = "blind zone  < 50 mm";
    const ImVec2 ts   = font->CalcTextSizeA(fs, FLT_MAX, 0.0f, txt);

    if (ts.x + 14.0f * dpi > r * 1.55f)
        return;

    PlateTextAt(dl, ImVec2(s0.x, s0.y + r * 0.42f), p0, p1, dpi, kBlindText, txt);
}

// Which way the unit is physically pointing. The C1 has an arrow moulded on its
// housing marking the front, the SDK reports that direction as angle 0, and this
// map puts 0 up - so this arrow lines up with the moulded one when the device is
// oriented the same way as the map.
//
// Deliberately a thin debug-overlay stroke: a long shaft plus an open head, no
// fill, no plate and no caption - the bearing numbers around the compass ring
// already say where 0 is. Anchored on the sensor's world origin, so it tracks
// correctly once the sensor is panned off-centre.
void DrawHeadingArrow(ImDrawList* dl, const ImVec2& p0, const ImVec2& p1, const ImVec2& s0,
                      float ppm, float radius_px, float dpi)
{
    const ImVec2 d = BearingDir(0.0f);           // straight up on screen

    // A fixed fraction of the fitted radius: the arrow is an orientation cue,
    // not a measurement, so it should keep the same commanding size at every
    // zoom rather than growing and shrinking with the range.
    const float len = Clampf(radius_px * 0.58f, 64.0f * dpi, 280.0f * dpi);

    // Start clear of the blind disc so the two do not overlap into a blob.
    const float r0 = std::max(kMinValidMm * ppm, kBlindMinPx * dpi) + 5.0f * dpi;
    if (len <= r0 + 12.0f * dpi)
        return;

    const ImVec2 a(s0.x + d.x * r0,  s0.y + d.y * r0);
    const ImVec2 t(s0.x + d.x * len, s0.y + d.y * len);

    // Cheap reject when the whole thing is off-widget.
    const float pad = 40.0f * dpi;
    if (std::max(a.x, t.x) < p0.x - pad || std::min(a.x, t.x) > p1.x + pad ||
        std::max(a.y, t.y) < p0.y - pad || std::min(a.y, t.y) > p1.y + pad)
        return;

    const float th   = 2.0f * dpi;
    const float head = std::min(30.0f * dpi, (len - r0) * 0.30f);

    dl->AddLine(a, t, kHeadingCol, th);

    // Open head: two strokes swept back from the tip at +/-26 deg.
    for (int s = -1; s <= 1; s += 2)
    {
        const ImVec2 b = BearingDir(180.0f + (float)s * 26.0f);
        dl->AddLine(t, ImVec2(t.x + b.x * head, t.y + b.y * head), kHeadingCol, th);
    }
}

// Labelled bar in the bottom-left corner. Unlike the rings this stays useful
// when the sensor has been panned right out of the widget.
void DrawScaleBar(ImDrawList* dl, const ImVec2& p0, const ImVec2& p1, float ppm, float dpi)
{
    if (!(ppm > 0.0f))
        return;

    const float budget = std::min((p1.x - p0.x) * 0.24f, 200.0f * dpi);
    if (budget < 30.0f * dpi)
        return;

    const float len_mm = NiceStepDown(budget / ppm);
    const float len_px = len_mm * ppm;
    if (!(len_px > 8.0f))
        return;

    const float x0  = p0.x + 4.0f * dpi;
    const float y   = p1.y - 12.0f * dpi;
    const float x1  = x0 + len_px;
    const float cap = 5.0f * dpi;
    const float th  = 1.8f * dpi;

    // Half-filled, like a map scale: the midpoint tick is the free half-value.
    dl->AddRectFilled(ImVec2(x0, y - cap * 0.5f),
                      ImVec2(x0 + len_px * 0.5f, y + cap * 0.5f), kScaleCol);
    dl->AddRect(ImVec2(x0, y - cap * 0.5f), ImVec2(x1, y + cap * 0.5f),
                kScaleCol, 0.0f, 0, th);
    dl->AddLine(ImVec2(x0, y - cap * 1.6f), ImVec2(x0, y + cap * 1.6f), kScaleCol, th);
    dl->AddLine(ImVec2(x1, y - cap * 1.6f), ImVec2(x1, y + cap * 1.6f), kScaleCol, th);

    char buf[24];
    FormatRing(buf, sizeof(buf), len_mm);

    ImFont*      font = LabelFont();
    const ImVec2 ts   = font->CalcTextSizeA(LabelPx(), FLT_MAX, 0.0f, buf);
    PlateText(dl, ImVec2(x0, y - cap * 1.8f - ts.y), dpi, kScaleCol, buf);
}

// Projects one revolution into screen space, dropping "no return" samples and
// anything outside the widget. At high zoom this is what keeps the submitted
// geometry proportional to what is actually visible.
void CollectDots(const std::vector<LidarPoint>& pts, const ImVec2& s0, float ppm,
                 const ImVec2& lo, const ImVec2& hi, std::vector<Dot>& out)
{
    out.clear();
    if (pts.empty())
        return;
    out.reserve(pts.size());

    const float deg2rad = kPi / 180.0f;

    for (const LidarPoint& p : pts)
    {
        const float d = p.dist_mm;

        // Below kMinValidMm the device is inside its own spec floor: those are
        // housing reflections, and every readout already discards them. Drawing
        // them anyway put dots inside the disc marked "cannot see here", which
        // is exactly the contradiction the blind zone exists to prevent.
        // (0 mm means "no return" and is caught by the same test.) The upper
        // bound is the device's 12 m spec limit - see kMaxValidMm.
        if (!(d >= kMinValidMm) || d > kMaxValidMm)
            continue;

        const float rr  = d * ppm;
        const float ang = (p.angle_deg - 90.0f) * deg2rad;   // == (sin a, -cos a)
        const float x   = s0.x + rr * std::cos(ang);
        const float y   = s0.y + rr * std::sin(ang);

        if (x < lo.x || x > hi.x || y < lo.y || y > hi.y)
            continue;

        out.push_back(Dot{ x, y });
    }
}

// The sensor itself. Deliberately smaller than kBlindMinPx so it reads as the
// core of the blind disc rather than competing with its hatched edge.
void DrawHub(ImDrawList* dl, const ImVec2& c, float dpi)
{
    dl->AddCircleFilled(c, 6.5f * dpi, kHubCol, 20);
    dl->AddCircleFilled(c, 2.6f * dpi, kHubCoreCol, 12);
}

void DrawPlaceholder(ImDrawList* dl, const ImVec2& c, float dpi)
{
    ImFont*      font = LabelFont();
    const float  fs   = LabelPx() * 1.25f;
    const char*  txt  = "No scan data";
    const ImVec2 ts   = font->CalcTextSizeA(fs, FLT_MAX, 0.0f, txt);

    dl->AddText(font, fs, ImVec2(c.x - ts.x * 0.5f, c.y - ts.y * 0.5f), kEmptyCol, txt);
}

} // namespace

// ------------------------------------------------------------------ state ---

namespace {

// Distance the auto-fit should try to contain. Deliberately the 95th percentile
// rather than the maximum: a single return down a corridor or through a doorway
// would otherwise stretch the view to 16 m and shrink the room to a smudge.
// Returns below the C1's 0.05 m spec floor are housing reflections, not data.
float FitDistanceMm(const LidarFrame& frame)
{
    static std::vector<float> d;
    d.clear();
    d.reserve(frame.points.size());

    for (const LidarPoint& p : frame.points)
        if (p.dist_mm >= kMinValidMm && p.dist_mm <= kMaxValidMm)
            d.push_back(p.dist_mm);

    if (d.empty()) return 0.0f;

    size_t k = (size_t)(d.size() * 0.95f);
    if (k >= d.size()) k = d.size() - 1;

    std::nth_element(d.begin(), d.begin() + k, d.end());
    return d[k];
}

} // namespace

void RadarView::push(const LidarFrame& frame)
{
    trail_.push_back(frame.points);
    while (trail_.size() > kMaxTrail)
        trail_.pop_front();

    has_data_ = true;

    const float fit_mm = FitDistanceMm(frame);
    if (fit_mm > 0.0f)
    {
        // Keep a short history and fit to the LARGEST recent revolution, not the
        // newest one. The 95th percentile moves by metres between consecutive
        // revolutions - returns flicker in and out at the edges of the room - and
        // chasing it directly made the view visibly bounce. Taking the window
        // maximum means a single sparse revolution cannot pull the view in, so
        // it only shrinks once the scene has actually stayed small.
        fit_hist_[fit_n_ % kFitHistory] = fit_mm;
        ++fit_n_;

        const int n = std::min(fit_n_, kFitHistory);
        float windowed = 0.0f;
        for (int i = 0; i < n; ++i) windowed = std::max(windowed, fit_hist_[i]);

        float target = std::min(std::max(windowed * 1.15f, 750.0f), 16000.0f);

        // Deadband. Without it the range creeps every revolution by a percent or
        // two, which reads as jitter even though each step is tiny. Inside the
        // band the view is held perfectly still.
        constexpr float kHold = 0.08f;
        if (std::fabs(target - auto_range_mm_) <= kHold * auto_range_mm_)
            target = auto_range_mm_;

        const float k = (target > auto_range_mm_) ? kFitRise : kFitFall;
        auto_range_mm_ += (target - auto_range_mm_) * k;
    }
}

void RadarView::clear()
{
    trail_.clear();
    has_data_           = false;
    has_nearest_        = false;
    nearest_mm_         = 0.0f;
    nearest_bearing_deg_ = 0.0f;
    measure_active_     = false;
    measure_mm_         = 0.0f;

    // Otherwise a reconnect fits to the previous room for the next 2.4 s.
    fit_n_ = 0;
    for (int i = 0; i < kFitHistory; ++i) fit_hist_[i] = 0.0f;
}

// ------------------------------------------------------------- view model ---

void RadarView::fit()
{
    auto_fit_       = true;
    view_center_mm_ = ImVec2(0.0f, 0.0f);
    measure_active_ = false;
    measure_mm_     = 0.0f;

    if (radius_px_ > 0.0f)
        px_per_mm_ = radius_px_ / std::max(auto_range_mm_, 1.0f);
}

void RadarView::set_range_mm(float mm)
{
    const float r = Clampf(mm, kMinVisibleMm, kMaxVisibleMm);

    auto_fit_       = false;
    view_center_mm_ = ImVec2(0.0f, 0.0f);
    auto_range_mm_  = r;
    px_per_mm_      = (radius_px_ > 0.0f) ? (radius_px_ / r) : 0.0f;
}

float RadarView::visible_range_mm() const
{
    if (px_per_mm_ > 0.0f && radius_px_ > 0.0f)
        return radius_px_ / px_per_mm_;
    return auto_range_mm_;
}

ImVec2 RadarView::to_screen(const ImVec2& world_mm) const
{
    return ImVec2(center_px_.x + (world_mm.x - view_center_mm_.x) * px_per_mm_,
                  center_px_.y + (world_mm.y - view_center_mm_.y) * px_per_mm_);
}

ImVec2 RadarView::to_world(const ImVec2& screen_px) const
{
    if (!(px_per_mm_ > 0.0f))
        return view_center_mm_;

    return ImVec2(view_center_mm_.x + (screen_px.x - center_px_.x) / px_per_mm_,
                  view_center_mm_.y + (screen_px.y - center_px_.y) / px_per_mm_);
}

// ------------------------------------------------------------------- draw ---

void RadarView::draw(const ImVec2& size)
{
    ImGuiIO&    io  = ImGui::GetIO();
    const float dpi = Dpi();

    ImVec2 sz = size;
    if (sz.x <= 0.0f) sz.x = ImGui::GetContentRegionAvail().x + sz.x;
    if (sz.y <= 0.0f) sz.y = ImGui::GetContentRegionAvail().y + sz.y;
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

    const bool hovered = ImGui::IsItemHovered();
    const bool active  = ImGui::IsItemActive();

    // Claim the wheel only while this widget is under the cursor; the rest of
    // the UI keeps its scrolling.
    ImGui::SetItemKeyOwner(ImGuiKey_MouseWheelY);

    center_px_ = ImVec2(p0.x + sz.x * 0.5f, p0.y + sz.y * 0.5f);

    const float min_side = std::min(sz.x, sz.y);
    const float margin   = std::min(18.0f * dpi, min_side * 0.08f);
    radius_px_ = std::max(min_side * 0.5f - margin, 0.0f);

    cursor_valid_ = false;

    ImDrawList* dl = ImGui::GetWindowDrawList();
    if (dl == nullptr)
        return;

    if (radius_px_ < 12.0f * dpi)   // too small to say anything useful
    {
        dl->PushClipRect(p0, p1, true);
        dl->PopClipRect();
        return;
    }

    // ---- scale ------------------------------------------------------------
    const float min_ppm = radius_px_ / kMaxVisibleMm;
    const float max_ppm = radius_px_ / kMinVisibleMm;

    if (auto_fit_)
    {
        view_center_mm_ = ImVec2(0.0f, 0.0f);
        px_per_mm_      = radius_px_ / std::max(auto_range_mm_, 1.0f);
    }
    else if (!(px_per_mm_ > 0.0f))
    {
        // set_range_mm() before the first draw, or a resize from nothing.
        px_per_mm_ = radius_px_ / std::max(auto_range_mm_, 1.0f);
    }
    px_per_mm_ = Clampf(px_per_mm_, min_ppm, max_ppm);

    // ---- input ------------------------------------------------------------
    bool reset_now = false;
    if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
    {
        fit();
        px_per_mm_ = Clampf(radius_px_ / std::max(auto_range_mm_, 1.0f), min_ppm, max_ppm);
        reset_now  = true;
    }

    // Pan: left or middle drag, 1:1 with the cursor.
    bool panning = false;
    if (active && !reset_now &&
        (ImGui::IsMouseDown(ImGuiMouseButton_Left) || ImGui::IsMouseDown(ImGuiMouseButton_Middle)))
    {
        panning = true;
        const ImVec2 d = io.MouseDelta;
        if (d.x != 0.0f || d.y != 0.0f)
        {
            view_center_mm_.x -= d.x / px_per_mm_;
            view_center_mm_.y -= d.y / px_per_mm_;
            auto_fit_ = false;
        }
    }

    // Zoom about the cursor: the world point under the mouse stays put.
    float wheel = io.MouseWheel;
    if (wheel == 0.0f && io.KeyShift)
        wheel = io.MouseWheelH;      // some backends swap the axis under shift

    // `active` covers a drag that has wandered outside the rect, where
    // IsItemHovered() goes false but this widget still owns the mouse.
    if ((hovered || active) && wheel != 0.0f && !reset_now)
    {
        const float step   = io.KeyCtrl ? 1.03f : (io.KeyShift ? 1.30f : 1.10f);
        const float factor = std::pow(step, wheel);
        const ImVec2 anchor_world = to_world(io.MousePos);
        const float  ppm          = Clampf(px_per_mm_ * factor, min_ppm, max_ppm);

        if (ppm != px_per_mm_)
        {
            px_per_mm_ = ppm;
            view_center_mm_.x = anchor_world.x - (io.MousePos.x - center_px_.x) / ppm;
            view_center_mm_.y = anchor_world.y - (io.MousePos.y - center_px_.y) / ppm;
            auto_fit_ = false;
        }
    }

    // Measure: right drag, anchored on press.
    if (active && !panning && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
    {
        measure_active_  = true;
        measure_from_mm_ = to_world(io.MousePos);
        measure_to_mm_   = measure_from_mm_;
        measure_mm_      = 0.0f;
    }
    if (measure_active_)
    {
        if (ImGui::IsMouseDown(ImGuiMouseButton_Right))
        {
            measure_to_mm_ = to_world(io.MousePos);
            const float dx = measure_to_mm_.x - measure_from_mm_.x;
            const float dy = measure_to_mm_.y - measure_from_mm_.y;
            measure_mm_    = std::sqrt(dx * dx + dy * dy);
        }
        else
        {
            measure_active_ = false;
            measure_mm_     = 0.0f;
        }
    }

    if (panning)                 ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
    else if (measure_active_)    ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);

    // ---- readouts ---------------------------------------------------------
    if (hovered || active)
    {
        const ImVec2 w = to_world(io.MousePos);
        cursor_valid_    = true;
        cursor_range_mm_ = std::sqrt(w.x * w.x + w.y * w.y);

        float b = std::atan2(w.x, -w.y) * (180.0f / kPi);   // 0 = up, clockwise
        if (b < 0.0f) b += 360.0f;
        cursor_bearing_deg_ = b;
    }

    const float visible_mm = std::max(radius_px_ / px_per_mm_, 1.0f);
    const ImVec2 s0        = to_screen(ImVec2(0.0f, 0.0f));   // sensor on screen

    // ---- draw -------------------------------------------------------------
    // Order matters: chrome that has to stay legible (every number, the scale
    // bar) goes on *after* the point cloud, everything else underneath it.
    dl->PushClipRect(p0, p1, true);

    GridSpec grid;
    if (show_grid)
    {
        grid = ComputeGrid(p0, p1, s0, px_per_mm_, visible_mm, radius_px_);
        DrawGridLines(dl, grid, p0, p1, s0, dpi);
        DrawBlindZone(dl, p0, p1, s0, px_per_mm_, dpi);
    }

    const bool have_points = has_data_ && !trail_.empty();
    if (!have_points)
    {
        has_nearest_ = false;
        DrawPlaceholder(dl, center_px_, dpi);
    }

    if (have_points)
    {
        const ImVec2 uv   = ImGui::GetFontTexUvWhitePixel();
        const int    last = (int)trail_.size() - 1;

        const float  dot_r = 2.0f * dpi;
        const ImVec2 cull_lo(p0.x - dot_r, p0.y - dot_r);
        const ImVec2 cull_hi(p1.x + dot_r, p1.y + dot_r);

        std::vector<Dot>& dots = Scratch();

        // Older revolutions fade out behind the current one. Same flat colour,
        // lower alpha and a slightly smaller dot: the trail is context, the
        // latest revolution is the reading.
        if (show_trail && last > 0)
        {
            for (int i = 0; i < last; ++i)
            {
                const float a  = 0.07f + 0.13f * ((float)i / (float)std::max(1, last));
                const int   a8 = (int)(Clampf(a, 0.0f, 1.0f) * 255.0f + 0.5f);
                if (a8 <= 0)
                    continue;

                CollectDots(trail_[(size_t)i], s0, px_per_mm_, cull_lo, cull_hi, dots);
                EmitDiscs(dl, dots.data(), (int)dots.size(), 1.6f * dpi,
                          kPointRgb | ((ImU32)a8 << IM_COL32_A_SHIFT), uv);
            }
        }

        CollectDots(trail_[(size_t)last], s0, px_per_mm_, cull_lo, cull_hi, dots);
        EmitDiscs(dl, dots.data(), (int)dots.size(), dot_r,
                  kPointRgb | ((ImU32)255u << IM_COL32_A_SHIFT), uv);

        // ---- nearest return -----------------------------------------------
        const std::vector<LidarPoint>& cur = trail_[(size_t)last];

        const int n_cur = (int)cur.size();

        int best_i = -1;
        for (int i = 0; i < n_cur; ++i)
        {
            // kMinValidMm, not 0: the C1 is specified from 0.05 m, and the
            // sub-50 mm returns off its own housing would otherwise win this
            // comparison every single frame.
            const LidarPoint& p = cur[(size_t)i];
            if (p.dist_mm < kMinValidMm || p.dist_mm > kMaxValidMm)
                continue;
            if (best_i < 0 || p.dist_mm < cur[(size_t)best_i].dist_mm)
                best_i = i;
        }

        has_nearest_ = (best_i >= 0);
        if (best_i >= 0)
        {
            const LidarPoint* best = &cur[(size_t)best_i];

            nearest_mm_ = best->dist_mm;

            float b = std::fmod(best->angle_deg, 360.0f);
            if (b < 0.0f) b += 360.0f;
            nearest_bearing_deg_ = b;

            if (show_nearest)
            {
                const float  ang = (best->angle_deg - 90.0f) * (kPi / 180.0f);
                const float  rr  = best->dist_mm * px_per_mm_;
                const ImVec2 np(s0.x + rr * std::cos(ang), s0.y + rr * std::sin(ang));

                // Highlight the whole nearest *object*, not just the single
                // closest sample. Returns are angle-sorted, so the object is the
                // contiguous run of samples around `best` that stays at a
                // similar range - a wall or a hand spans many samples, and
                // ringing one of them says nothing about its extent.
                std::vector<Dot>& hot = NearestScratch();
                GatherNearestCluster(cur, best_i, s0, px_per_mm_, hot);
                EmitDiscs(dl, hot.data(), (int)hot.size(),
                          dot_r * 1.35f, kNearestCol, uv);

                dl->AddCircle(np, 10.0f * dpi, kNearestCol, 20, 1.6f * dpi);
                dl->AddCircle(np, 4.0f * dpi, kNearestCol, 12, 1.0f * dpi);

                if (show_labels)
                {
                    char buf[24];
                    FormatDist(buf, sizeof(buf), best->dist_mm);
                    PlateText(dl, ImVec2(np.x + 14.0f * dpi, np.y - 6.0f * dpi),
                              dpi, kNearestCol, buf);
                }
            }
        }
    }

    // ---- measurement ------------------------------------------------------
    if (measure_active_)
    {
        const ImVec2 a = to_screen(measure_from_mm_);
        const ImVec2 b = to_screen(measure_to_mm_);

        dl->AddLine(a, b, kMeasureCol, 1.6f * dpi);
        dl->AddCircleFilled(a, 3.5f * dpi, kMeasureCol, 12);
        dl->AddCircleFilled(b, 3.5f * dpi, kMeasureCol, 12);

        char buf[24];
        FormatDist(buf, sizeof(buf), measure_mm_);

        ImFont*      font = LabelFont();
        const ImVec2 ts   = font->CalcTextSizeA(LabelPx(), FLT_MAX, 0.0f, buf);
        PlateText(dl, ImVec2((a.x + b.x) * 0.5f - ts.x * 0.5f,
                             (a.y + b.y) * 0.5f - ts.y - 6.0f * dpi),
                  dpi, kMeasureCol, buf);
    }

    DrawHub(dl, s0, dpi);

    // ---- chrome that must stay readable over the points --------------------
    // Orientation matters enough to survive either overlay toggle.
    if (show_grid || show_labels)
        DrawHeadingArrow(dl, p0, p1, s0, px_per_mm_, radius_px_, dpi);

    // The app draws its own status text in the top and bottom gutters of this
    // rect, so every map label is placed inside an inset copy of it.
    const ImVec2 lab0(p0.x + 12.0f * dpi, p0.y + 30.0f * dpi);
    const ImVec2 lab1(p1.x - 12.0f * dpi, p1.y - 30.0f * dpi);

    if (lab1.x > lab0.x && lab1.y > lab0.y)
    {
        if (show_grid && show_labels)
        {
            DrawGridLabels(dl, grid, lab0, lab1,
                           ImVec2(p0.x + 4.0f * dpi, p0.y + 4.0f * dpi),
                           ImVec2(p1.x - 4.0f * dpi, p1.y - 4.0f * dpi), s0, dpi);
            DrawBlindLabel(dl, lab0, lab1, s0, px_per_mm_, dpi);
        }

        if (show_labels)
            DrawScaleBar(dl, lab0, lab1, px_per_mm_, dpi);
    }

    dl->PopClipRect();
}
