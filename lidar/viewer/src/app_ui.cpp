// Application layout: interactive map, a control bar beneath it, and a rail of
// readouts. Plain Dear ImGui widgets throughout.
//
// The rail is a two-level tab bar: Lidar | Pico | Debug at the top, with the
// four lidar readouts nested under Lidar. It is the command hub for the whole
// car, not just the scanner, so the top level names subsystems.
//
// Deliberately fits one viewport with no scrolling anywhere. The rail carries a
// lot of telemetry, so it is split across tabs rather than made scrollable. The
// one exception is the Debug console, which is a log and scrolls by nature.
#include "app_ui.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <cstring>
#include <string>
#include <vector>

#include "imgui.h"
#include "lidar_source.h"
#include "pico_link.h"
#include "radar.h"
#include "theme.h"

namespace {

LidarSource g_lidar;
RadarView   g_radar;
PicoLink    g_pico;

std::vector<std::string> g_ports;
std::vector<const char*> g_port_items;
int   g_port_index  = -1;
int   g_baud_index  = 2;     // 460800
int   g_range_index = 0;     // Fit
float g_dpi         = 1.0f;

LidarFrame g_frame;
bool       g_have_frame = false;

// The C1 is specified over 0.05 - 12 m. Returns outside that window are the
// housing (below) or unreliable long-range noise (above). The same window
// governs what the map draws, so nothing is ever shown that is not also
// counted.
constexpr float kMinValidMm = 50.0f;
constexpr float kMaxValidMm = 12000.0f;

// ---------------------------------------------------------------- derived ---
// Recomputed once per revolution, not per UI frame. LidarFrame's own
// valid_count / max_dist_mm are deliberately raw (every return the device
// sent), so everything spec-bounded is derived here.

float g_mean_mm   = 0.0f;
float g_max_mm    = 0.0f;
float g_points_ps = 0.0f;

// Return classification. These four sum to the revolution's sample count.
int g_n_inspec   = 0;
int g_n_noreturn = 0;   // dist == 0, the device saw nothing that way
int g_n_toonear  = 0;   // 0 < dist < 50 mm, housing reflection
int g_n_toofar   = 0;   // dist > 12 m, beyond the rated range

// Signal quality, over in-spec returns only.
float g_q_mean = 0.0f;
int   g_q_min  = 0;
int   g_q_max  = 0;

constexpr int kQualityBuckets = 16;         // 0..63 folded into 16 bins
float g_q_hist[kQualityBuckets] = {};
float g_q_hist_max = 1.0f;

constexpr int kDistBuckets = 24;            // 0..12 m in 0.5 m bins
float g_dist_hist[kDistBuckets] = {};
float g_dist_hist_max = 1.0f;

// Angular coverage: fraction of 1-degree bins with at least one in-spec return.
float g_coverage = 0.0f;

// Clearance: distance to the nearest return in each 30 degree sector, in metres.
constexpr int kSectors = 12;
float g_sector_m[kSectors] = {};
constexpr float kClearanceCapM = 2.5f;   // beyond this a direction is just "clear"

// ------------------------------------------------------------- pico link ---
// The debug/bring-up channel to the Pico 2 W over USB CDC. Ports, the drained
// line log, and the console's own view state.
//
// If a connected board ever appears to swallow writes: TinyUSB CDC refuses OUT
// data until the host asserts DTR, and a write without it blocks until the
// driver gives up with "the semaphore timeout period has expired". That reads
// exactly like dead hardware and is not. pico_link.cpp asserts DTR; do not
// remove it.

std::vector<std::string> g_pico_ports;
std::vector<const char*> g_pico_items;
int  g_pico_index = -1;

// The console is a debug aid, not a record: this app runs for hours, so the log
// is bounded and the oldest lines fall off the front.
constexpr size_t kLogMax = 4000;
std::vector<PicoLine> g_log;
std::vector<int>      g_log_shown;    // indices passing the filter, rebuilt per frame

char g_cmd_buf[192]   = {};
char g_filter_buf[64] = {};
bool g_log_autoscroll = true;

// Result of the last BOOTSEL touch, so a failure is not silent.
bool g_bootsel_done = false;
bool g_bootsel_ok   = false;

// --tab <name> preselects a rail tab at startup, for screenshots and for
// launching straight into the readout you care about. Two levels now: a top
// tab, and optionally one of the lidar sub-tabs.
int g_force_top        = -1;
int g_force_sub        = -1;
int g_force_tab_frames = 0;

// Rolling rotation-rate history for the sparkline.
constexpr int kHistory = 240;
float g_hz_hist[kHistory] = {};
int   g_hz_count = 0;

const int   kBauds[]      = { 115200, 256000, 460800 };
const char* kBaudItems[]  = { "115200", "256000", "460800" };

struct RangeOpt { const char* label; float mm; };   // mm <= 0 means auto-fit
const RangeOpt kRanges[] = {
    { "Fit", 0.0f }, { "0.5 m", 500.0f }, { "1 m", 1000.0f }, { "2 m", 2000.0f },
    { "4 m", 4000.0f }, { "8 m", 8000.0f }, { "12 m", 12000.0f },
};
constexpr int kRangeCount = (int)(sizeof(kRanges) / sizeof(kRanges[0]));
const char* kRangeItems[kRangeCount] = {};

// PushFont in 1.92 takes a pre-scale base size; LegacySize already has the DPI
// baked in by LoadFonts, so it is never multiplied by g_dpi again.
struct ScopedFont
{
    explicit ScopedFont(ImFont* f) { ImGui::PushFont(f, f ? f->LegacySize : 0.0f); }
    ~ScopedFont() { ImGui::PopFont(); }
};

void RefreshPorts()
{
    g_ports = LidarSource::list_ports();

    g_port_items.clear();
    for (const auto& s : g_ports) g_port_items.push_back(s.c_str());

    if (g_ports.empty()) { g_port_index = -1; return; }

    // Identify the CP210x bridge outright where we can - the remaining ports on
    // a typical machine are Bluetooth links, and connecting to one of those just
    // produces a confusing timeout.
    const std::string preferred = LidarSource::preferred_port();
    if (!preferred.empty())
    {
        for (int i = 0; i < (int)g_ports.size(); ++i)
        {
            if (_stricmp(g_ports[i].c_str(), preferred.c_str()) == 0)
            {
                g_port_index = i;
                return;
            }
        }
    }

    // Fallback: a USB bridge normally enumerates above the built-in ports.
    if (g_port_index < 0 || g_port_index >= (int)g_ports.size())
        g_port_index = (int)g_ports.size() - 1;
}

bool Busy()
{
    const LidarState s = g_lidar.state();
    return s == LidarState::Scanning || s == LidarState::Connecting;
}

// ---------------------------------------------------------------- pico ----

void RefreshPicoPorts()
{
    g_pico_ports = PicoLink::list_pico_ports();

    g_pico_items.clear();
    for (const auto& s : g_pico_ports) g_pico_items.push_back(s.c_str());

    if (g_pico_ports.empty()) { g_pico_index = -1; return; }
    if (g_pico_index < 0 || g_pico_index >= (int)g_pico_ports.size()) g_pico_index = 0;
}

void ConnectPico()
{
    if (g_pico_index < 0 || g_pico_index >= (int)g_pico_ports.size()) return;
    g_pico.connect(g_pico_ports[g_pico_index]);
}

void SendPico(const char* line)
{
    if (!line || !line[0]) return;
    g_pico.send(line);            // the link logs it; drain() gives it back to us
}

// Drains once per frame, which is what PicoLink asks for, and keeps the log
// bounded.
void PumpPico()
{
    g_pico.drain(g_log);
    if (g_log.size() > kLogMax)
        g_log.erase(g_log.begin(), g_log.begin() + (g_log.size() - kLogMax));
}

const char* PicoStateText(PicoState s)
{
    switch (s)
    {
    case PicoState::Connecting: return "Connecting";
    case PicoState::Connected:  return "Connected";
    case PicoState::Error:      return "Error";
    default:                    return "Not connected";
    }
}

ImU32 PicoStateColor(PicoState s)
{
    switch (s)
    {
    case PicoState::Connecting: return ui::plot::warn;
    case PicoState::Connected:  return ui::plot::ok;
    case PicoState::Error:      return ui::plot::bad;
    default:                    return ui::plot::idle;
    }
}

// A silent board is the expected state right now - no firmware speaks yet - so
// this says so in words rather than showing an empty readout.
void PicoAgeText(char* buf, size_t n, double age_s)
{
    if (age_s < 0.0)        std::snprintf(buf, n, "nothing received yet");
    else if (age_s < 2.0)   std::snprintf(buf, n, "%.1f s ago", age_s);
    else if (age_s < 600.0) std::snprintf(buf, n, "silent for %.1f s", age_s);
    else                    std::snprintf(buf, n, "silent for %.0f min", age_s / 60.0);
}

bool LogMatches(const PicoLine& ln)
{
    if (g_filter_buf[0] == '\0') return true;

    const char* hay = ln.text.c_str();
    for (; *hay; ++hay)
    {
        const char* h = hay;
        const char* n = g_filter_buf;
        while (*n && *h &&
               std::tolower((unsigned char)*h) == std::tolower((unsigned char)*n)) { ++h; ++n; }
        if (*n == '\0') return true;
    }
    return false;
}

void RecomputeDerived()
{
    g_points_ps = g_frame.hz * (float)g_frame.points.size();

    float sector_mm[kSectors] = {};
    for (int i = 0; i < kQualityBuckets; ++i) g_q_hist[i] = 0.0f;
    for (int i = 0; i < kDistBuckets; ++i)    g_dist_hist[i] = 0.0f;

    static bool bin_seen[360];
    std::memset(bin_seen, 0, sizeof(bin_seen));

    double sum   = 0.0;
    double q_sum = 0.0;
    int    n     = 0;
    float  max_mm = 0.0f;
    int    q_lo = 255, q_hi = 0;

    g_n_noreturn = g_n_toonear = g_n_toofar = 0;

    for (const LidarPoint& p : g_frame.points)
    {
        if (p.dist_mm <= 0.0f)          { ++g_n_noreturn; continue; }
        if (p.dist_mm < kMinValidMm)    { ++g_n_toonear;  continue; }
        if (p.dist_mm > kMaxValidMm)    { ++g_n_toofar;   continue; }

        sum += p.dist_mm;
        ++n;
        if (p.dist_mm > max_mm) max_mm = p.dist_mm;

        q_sum += p.quality;
        if (p.quality < q_lo) q_lo = p.quality;
        if (p.quality > q_hi) q_hi = p.quality;

        int qb = (int)p.quality * kQualityBuckets / 64;
        qb = std::min(std::max(qb, 0), kQualityBuckets - 1);
        g_q_hist[qb] += 1.0f;

        int db = (int)(p.dist_mm / (kMaxValidMm / kDistBuckets));
        db = std::min(std::max(db, 0), kDistBuckets - 1);
        g_dist_hist[db] += 1.0f;

        int ab = (int)p.angle_deg;
        if (ab >= 0 && ab < 360) bin_seen[ab] = true;

        int s = (int)(p.angle_deg / (360.0f / kSectors));
        s = std::min(std::max(s, 0), kSectors - 1);

        // Nearest return wins the sector - that is the obstacle that matters.
        if (sector_mm[s] == 0.0f || p.dist_mm < sector_mm[s])
            sector_mm[s] = p.dist_mm;
    }

    g_n_inspec = n;
    g_mean_mm  = n ? (float)(sum / n) : 0.0f;
    g_max_mm   = max_mm;
    g_q_mean   = n ? (float)(q_sum / n) : 0.0f;
    g_q_min    = n ? q_lo : 0;
    g_q_max    = n ? q_hi : 0;

    int covered = 0;
    for (int i = 0; i < 360; ++i) if (bin_seen[i]) ++covered;
    g_coverage = covered / 360.0f;

    g_q_hist_max = 1.0f;
    for (int i = 0; i < kQualityBuckets; ++i) g_q_hist_max = std::max(g_q_hist_max, g_q_hist[i]);
    g_dist_hist_max = 1.0f;
    for (int i = 0; i < kDistBuckets; ++i) g_dist_hist_max = std::max(g_dist_hist_max, g_dist_hist[i]);

    // Capped, not scaled to the maximum: one open doorway at 8 m would
    // otherwise crush every near-field bar to invisibility.
    for (int i = 0; i < kSectors; ++i)
        g_sector_m[i] = std::min(sector_mm[i] / 1000.0f, kClearanceCapM);

    if (g_hz_count < kHistory)
    {
        g_hz_hist[g_hz_count++] = g_frame.hz;
    }
    else
    {
        std::memmove(g_hz_hist, g_hz_hist + 1, sizeof(float) * (kHistory - 1));
        g_hz_hist[kHistory - 1] = g_frame.hz;
    }
}

void PumpData()
{
    if (g_lidar.poll(g_frame))
    {
        g_radar.push(g_frame);
        g_have_frame = true;
        RecomputeDerived();
    }
    PumpPico();
}

void ApplyRange()
{
    const float mm = kRanges[g_range_index].mm;
    if (mm <= 0.0f) g_radar.fit();
    else            g_radar.set_range_mm(mm);
}

void Connect()
{
    if (g_port_index < 0 || g_port_index >= (int)g_ports.size()) return;

    g_radar.clear();
    g_have_frame = false;
    g_hz_count   = 0;
    g_lidar.start(g_ports[g_port_index], kBauds[g_baud_index]);
}

// ------------------------------------------------------------- HUD on map

void DrawMapHud(const ImVec2& p0, const ImVec2& size)
{
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImFont* f = ui::fonts.small ? ui::fonts.small : ImGui::GetFont();
    const float px  = f->LegacySize;
    const float pad = 14.0f * g_dpi;

    const char* state_text = "Not connected";
    ImU32       accent     = ui::plot::idle;
    switch (g_lidar.state())
    {
    case LidarState::Connecting: state_text = "Connecting";  accent = ui::plot::warn; break;
    case LidarState::Scanning:   state_text = "Scanning";    accent = ui::plot::ok;   break;
    case LidarState::Error:      state_text = "Error";       accent = ui::plot::bad;  break;
    default: break;
    }

    // ---- top left: state + connection -----------------------------------
    float x = p0.x + pad;
    const float y = p0.y + pad;

    dl->AddCircleFilled(ImVec2(x + px * 0.25f, y + px * 0.55f), px * 0.25f, accent, 12);
    x += px * 0.75f;
    dl->AddText(f, px, ImVec2(x, y), accent, state_text);
    x += f->CalcTextSizeA(px, FLT_MAX, 0.0f, state_text).x + 12.0f * g_dpi;

    if (g_port_index >= 0 && g_port_index < (int)g_ports.size())
    {
        char conn[64];
        std::snprintf(conn, sizeof(conn), "%s  -  %d baud",
                      g_ports[g_port_index].c_str(), kBauds[g_baud_index]);
        dl->AddText(f, px, ImVec2(x, y), ui::plot::label, conn);
    }

    // ---- top right: throughput ------------------------------------------
    char thru[96];
    std::snprintf(thru, sizeof(thru), "%.0f pts/s   %.0f fps",
                  g_points_ps, ImGui::GetIO().Framerate);
    const float tw = f->CalcTextSizeA(px, FLT_MAX, 0.0f, thru).x;
    dl->AddText(f, px, ImVec2(p0.x + size.x - pad - tw, y), ui::plot::label, thru);

    // ---- bottom left: cursor / measurement -------------------------------
    char read[128];
    read[0] = '\0';

    if (g_radar.measuring())
        std::snprintf(read, sizeof(read), "measure   %.2f m", g_radar.measure_mm() / 1000.0f);
    else if (g_radar.cursor_valid())
        std::snprintf(read, sizeof(read), "%.1f deg   %.2f m",
                      g_radar.cursor_bearing_deg(), g_radar.cursor_range_mm() / 1000.0f);

    if (read[0])
    {
        const float rw = f->CalcTextSizeA(px, FLT_MAX, 0.0f, read).x;
        const ImVec2 bp(p0.x + pad, p0.y + size.y - pad - px - 10.0f * g_dpi);
        dl->AddRectFilled(bp, ImVec2(bp.x + rw + 18.0f * g_dpi, bp.y + px + 12.0f * g_dpi),
                          IM_COL32(28, 28, 32, 235));
        dl->AddText(f, px, ImVec2(bp.x + 9.0f * g_dpi, bp.y + 6.0f * g_dpi),
                    IM_COL32(235, 235, 240, 255), read);
    }

    // ---- bottom right: zoom state ----------------------------------------
    char zoom[96];
    std::snprintf(zoom, sizeof(zoom), "%s   %.1f m across",
                  g_radar.is_auto_fit() ? "fit" : "manual",
                  g_radar.visible_range_mm() * 2.0f / 1000.0f);
    const float zw = f->CalcTextSizeA(px, FLT_MAX, 0.0f, zoom).x;
    dl->AddText(f, px, ImVec2(p0.x + size.x - pad - zw, p0.y + size.y - pad - px),
                ui::plot::label, zoom);
}

// ------------------------------------------------------------- rail pieces

void StatCell(const char* value, const char* caption, ImU32 color)
{
    {
        ScopedFont sf(ui::fonts.stat);
        ImGui::PushStyleColor(ImGuiCol_Text, color);
        ImGui::TextUnformatted(value);
        ImGui::PopStyleColor();
    }
    ScopedFont sf(ui::fonts.small);
    ImGui::TextDisabled("%s", caption);
}

void KeyValue(const char* k, const char* fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    ImGui::TableNextRow();
    ImGui::TableNextColumn(); ImGui::TextDisabled("%s", k);
    ImGui::TableNextColumn(); ImGui::TextUnformatted(buf);
}

void DrawConnection()
{
    const bool busy = Busy();

    ImGui::BeginDisabled(busy);
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (g_port_items.empty())
        ImGui::TextDisabled("No serial ports found");
    else
        ImGui::Combo("##port", &g_port_index, g_port_items.data(), (int)g_port_items.size());

    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::Combo("##baud", &g_baud_index, kBaudItems, 3);
    ImGui::EndDisabled();

    const float bh = ImGui::GetFrameHeight() * 1.2f;
    if (busy)
    {
        if (ImGui::Button("Disconnect", ImVec2(-FLT_MIN, bh))) g_lidar.stop();
    }
    else
    {
        ImGui::BeginDisabled(g_ports.empty());
        if (ImGui::Button("Connect", ImVec2(-FLT_MIN, bh))) Connect();
        ImGui::EndDisabled();
    }

    const std::string err = g_lidar.error();
    if (!err.empty() && g_lidar.state() == LidarState::Error)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ui::plot::bad);
        ImGui::TextWrapped("%s", err.c_str());
        ImGui::PopStyleColor();
    }
}

void TabLive()
{
    char hz[24] = "--", pts[24] = "--", valid[24] = "--";
    char near_s[24] = "--", mean_s[24] = "--", max_s[24] = "--";

    if (g_have_frame)
    {
        std::snprintf(hz,  sizeof(hz),  "%.1f", g_frame.hz);
        std::snprintf(pts, sizeof(pts), "%d",   (int)g_frame.points.size());

        const double frac = g_frame.points.empty()
                          ? 0.0 : (double)g_n_inspec / (double)g_frame.points.size();
        std::snprintf(valid, sizeof(valid), "%d%%", (int)(frac * 100.0 + 0.5));

        if (g_radar.has_nearest())
            std::snprintf(near_s, sizeof(near_s), "%.2f", g_radar.nearest_mm() / 1000.0f);
        std::snprintf(mean_s, sizeof(mean_s), "%.2f", g_mean_mm / 1000.0f);
        std::snprintf(max_s,  sizeof(max_s),  "%.2f", g_max_mm / 1000.0f);
    }

    if (ImGui::BeginTable("stats", 3, ImGuiTableFlags_SizingStretchSame))
    {
        ImGui::TableNextRow();
        ImGui::TableNextColumn(); StatCell(hz,     "Hz",       ui::plot::ok);
        ImGui::TableNextColumn(); StatCell(pts,    "pts/rev",  ui::plot::ramp_far);
        ImGui::TableNextColumn(); StatCell(valid,  "in-spec",  ui::plot::ramp_mid);

        ImGui::TableNextRow();
        ImGui::TableNextColumn(); StatCell(near_s, "near (m)", ui::plot::nearest);
        ImGui::TableNextColumn(); StatCell(mean_s, "mean (m)", ui::plot::label);
        ImGui::TableNextColumn(); StatCell(max_s,  "max (m)",  ui::plot::label);
        ImGui::EndTable();
    }

    char overlay[48];
    std::snprintf(overlay, sizeof(overlay), "rotation  %.1f Hz", g_have_frame ? g_frame.hz : 0.0f);
    ImGui::PlotLines("##hz", g_hz_hist, g_hz_count, 0, overlay,
                     0.0f, 15.0f, ImVec2(-FLT_MIN, 46.0f * g_dpi));

    ImGui::TextDisabled("Clearance by sector (m, capped %.1f)", kClearanceCapM);
    ImGui::PlotHistogram("##sectors", g_sector_m, kSectors, 0, nullptr,
                         0.0f, kClearanceCapM, ImVec2(-FLT_MIN, 58.0f * g_dpi));

    ScopedFont sf(ui::fonts.small);
    ImGui::TextDisabled("0 deg");
    ImGui::SameLine(ImGui::GetContentRegionAvail().x * 0.45f);
    ImGui::TextDisabled("180");
    ImGui::SameLine(ImGui::GetContentRegionAvail().x * 0.90f);
    ImGui::TextDisabled("360");
}

void TabSignal()
{
    // Return classification. These four sum to the revolution's sample count,
    // which is what makes the in-spec percentage interpretable rather than
    // just low.
    const int total = g_have_frame ? (int)g_frame.points.size() : 0;

    ImGui::TextDisabled("Returns this revolution (%d samples)", total);

    if (ImGui::BeginTable("returns", 3, ImGuiTableFlags_SizingStretchSame |
                                        ImGuiTableFlags_RowBg))
    {
        auto cell = [&](const char* label, int n, ImU32 col)
        {
            ImGui::TableNextColumn();
            ImGui::PushStyleColor(ImGuiCol_Text, col);
            ImGui::Text("%d", n);
            ImGui::PopStyleColor();
            ScopedFont sf(ui::fonts.small);
            ImGui::TextDisabled("%s", label);
            if (total > 0) ImGui::TextDisabled("%.0f%%", 100.0 * n / total);
        };

        ImGui::TableNextRow();
        cell("in spec",  g_n_inspec,   ui::plot::ok);
        cell("no return", g_n_noreturn, ui::plot::idle);
        cell("< 50 mm",  g_n_toonear,  ui::plot::warn);

        ImGui::EndTable();
    }

    if (g_n_toofar > 0)
        ImGui::TextDisabled("beyond 12 m: %d", g_n_toofar);
    else
        ImGui::TextDisabled("beyond 12 m: none");

    ImGui::Spacing();

    // Signal quality is reported per measurement by the device and is otherwise
    // completely invisible - it is the main clue when returns start dropping.
    ImGui::TextDisabled("Signal quality  (mean %.1f, range %d-%d of 63)",
                        g_q_mean, g_q_min, g_q_max);
    ImGui::PlotHistogram("##qhist", g_q_hist, kQualityBuckets, 0, nullptr,
                         0.0f, g_q_hist_max, ImVec2(-FLT_MIN, 62.0f * g_dpi));

    ScopedFont sf(ui::fonts.small);
    ImGui::TextDisabled("weak");
    ImGui::SameLine(ImGui::GetContentRegionAvail().x * 0.82f);
    ImGui::TextDisabled("strong");
}

void TabScan()
{
    const LidarScanInfo si = g_lidar.scan_info();

    const float ang_res = (g_have_frame && !g_frame.points.empty())
                        ? 360.0f / (float)g_frame.points.size() : 0.0f;

    if (ImGui::BeginTable("scan", 2, ImGuiTableFlags_SizingStretchProp))
    {
        KeyValue("Mode", "%s", si.mode.empty() ? "--" : si.mode.c_str());
        KeyValue("Mode id", "%d", si.mode_id);
        KeyValue("Sample period", si.us_per_sample > 0 ? "%.2f us" : "--",
                 si.us_per_sample);
        KeyValue("Sample rate", si.us_per_sample > 0 ? "%.2f kHz" : "--",
                 si.us_per_sample > 0 ? 1000.0f / si.us_per_sample : 0.0f);
        KeyValue("Mode max range", si.max_distance_m > 0 ? "%.1f m" : "--",
                 si.max_distance_m);
        KeyValue("Angular res", ang_res > 0 ? "%.2f deg" : "--", ang_res);
        KeyValue("Coverage", "%.0f%% of 360 deg", g_coverage * 100.0f);
        ImGui::EndTable();
    }

    ImGui::Spacing();
    ImGui::TextDisabled("Range distribution (0 - 12 m, 0.5 m bins)");
    ImGui::PlotHistogram("##dhist", g_dist_hist, kDistBuckets, 0, nullptr,
                         0.0f, g_dist_hist_max, ImVec2(-FLT_MIN, 70.0f * g_dpi));

    ScopedFont sf(ui::fonts.small);
    ImGui::TextDisabled("0");
    ImGui::SameLine(ImGui::GetContentRegionAvail().x * 0.45f);
    ImGui::TextDisabled("6 m");
    ImGui::SameLine(ImGui::GetContentRegionAvail().x * 0.88f);
    ImGui::TextDisabled("12 m");
}

void TabDevice()
{
    const LidarDeviceInfo info = g_lidar.info();
    const LidarStats      st   = g_lidar.stats();
    const bool known = !info.serial.empty();

    if (ImGui::BeginTable("dev", 2, ImGuiTableFlags_SizingStretchProp))
    {
        KeyValue("Model",    known ? "0x%02X" : "--", info.model);
        KeyValue("Firmware", known ? "%d.%02d" : "--", info.fw_major, info.fw_minor);
        KeyValue("Hardware", known ? "rev %d" : "--", info.hw_rev);
        KeyValue("Health",   "%s", known ? (info.health == 0 ? "OK" : "check") : "--");
        ImGui::EndTable();
    }

    if (known)
    {
        ScopedFont sf(ui::fonts.small);
        ImGui::TextDisabled("%s", info.serial.c_str());
    }

    ImGui::Spacing();
    ImGui::SeparatorText("Session");

    if (ImGui::BeginTable("sess", 2, ImGuiTableFlags_SizingStretchProp))
    {
        const int mins = (int)(st.uptime_s / 60.0);
        const int secs = (int)st.uptime_s % 60;

        KeyValue("Uptime", "%dm %02ds", mins, secs);
        KeyValue("Revolutions", "%llu", st.frames);
        KeyValue("Measurements", "%llu", st.points);
        KeyValue("Dropped revs", "%u", st.timeouts);
        KeyValue("Avg rate", st.uptime_s > 1.0 ? "%.2f Hz" : "--",
                 st.uptime_s > 1.0 ? (double)st.frames / st.uptime_s : 0.0);
        ImGui::EndTable();
    }

    // Pre-heat matters: the dToF core drifts with die temperature and cold
    // readings sit outside the calibrated point. See docs/calibration.md.
    ScopedFont sf(ui::fonts.small);
    if (st.uptime_s > 0.0 && st.uptime_s < 120.0)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ui::plot::warn);
        ImGui::TextWrapped("Warming up - %.0fs of 120s. Ranges drift until the "
                           "scan core reaches temperature.", st.uptime_s);
        ImGui::PopStyleColor();
    }
    else if (st.uptime_s >= 120.0)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ui::plot::ok);
        ImGui::TextUnformatted("Warmed up - measurements are trustworthy.");
        ImGui::PopStyleColor();
    }
}

// ------------------------------------------------------------------- pico

void TabPico()
{
    const ImGuiStyle& sty  = ImGui::GetStyle();
    const PicoState   st   = g_pico.state();
    const bool        live = (st == PicoState::Connected);
    const bool        busy = live || st == PicoState::Connecting;
    const float       bh   = ImGui::GetFrameHeight() * 1.2f;

    ImGui::SeparatorText("Link");

    // ---- port + refresh --------------------------------------------------
    const float refresh_w = ImGui::CalcTextSize("Refresh").x + sty.FramePadding.x * 2.0f;

    ImGui::BeginDisabled(busy);
    if (g_pico_items.empty())
    {
        ImGui::AlignTextToFramePadding();
        ImGui::PushStyleColor(ImGuiCol_Text, ui::plot::warn);
        ImGui::TextUnformatted("No Pico found");
        ImGui::PopStyleColor();
    }
    else
    {
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - refresh_w - sty.ItemSpacing.x);
        ImGui::Combo("##picoport", &g_pico_index, g_pico_items.data(), (int)g_pico_items.size());
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    if (ImGui::Button("Refresh")) RefreshPicoPorts();

    if (g_pico_ports.empty())
    {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("Nothing with USB VID 2E8A is enumerated. The board only "
                            "appears as a serial port once its firmware presents USB "
                            "CDC; a board sitting in BOOTSEL mounts as the RPI-RP2 "
                            "drive instead.");
        ImGui::PopTextWrapPos();
    }

    // ---- connect ---------------------------------------------------------
    if (busy)
    {
        if (ImGui::Button("Disconnect", ImVec2(-FLT_MIN, bh))) g_pico.disconnect();
    }
    else
    {
        ImGui::BeginDisabled(g_pico_index < 0);
        if (ImGui::Button("Connect", ImVec2(-FLT_MIN, bh))) ConnectPico();
        ImGui::EndDisabled();
    }

    const std::string err = g_pico.error();
    if (!err.empty() && st == PicoState::Error)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ui::plot::bad);
        ImGui::TextWrapped("%s", err.c_str());
        ImGui::PopStyleColor();
    }

    // ---- status ----------------------------------------------------------
    if (ImGui::BeginTable("picostat", 2, ImGuiTableFlags_SizingStretchProp))
    {
        ImGui::TableNextRow();
        ImGui::TableNextColumn(); ImGui::TextDisabled("State");
        ImGui::TableNextColumn();
        ImGui::PushStyleColor(ImGuiCol_Text, PicoStateColor(st));
        ImGui::TextUnformatted(PicoStateText(st));
        ImGui::PopStyleColor();

        const std::string p = g_pico.port();
        KeyValue("Port",    "%s",   p.empty() ? "--" : p.c_str());
        KeyValue("Sent",    "%llu", g_pico.tx_lines());
        KeyValue("Received","%llu", g_pico.rx_lines());
        KeyValue("Dropped", "%llu", g_pico.dropped());
        ImGui::EndTable();
    }

    // The whole point of this readout: a board that never answers must look
    // deliberately silent, not broken or unpopulated.
    const double age = g_pico.last_rx_age_s();
    char age_s[64];
    PicoAgeText(age_s, sizeof(age_s), age);

    const ImU32 age_col = (age >= 0.0 && age < 2.0) ? ui::plot::ok
                        : (live ? ui::plot::warn : ui::plot::idle);
    ImGui::PushStyleColor(ImGuiCol_Text, age_col);
    ImGui::Text("Last line: %s", age_s);
    ImGui::PopStyleColor();

    // ---- commands --------------------------------------------------------
    ImGui::Spacing();
    ImGui::SeparatorText("Commands");

    ImGui::BeginDisabled(!live);
    if (ImGui::BeginTable("picocmd", 3, ImGuiTableFlags_SizingStretchSame))
    {
        auto cmd = [](const char* label, const char* line)
        {
            ImGui::TableNextColumn();
            if (ImGui::Button(label, ImVec2(-FLT_MIN, 0.0f))) SendPico(line);
        };

        // "?" first because it is the only command the board currently on the
        // bench understands - it answers "S <uptime_ms> ... 1500 1500 ..." then
        // "OK". The rest are the vocabulary of firmware/ and go live once that
        // is flashed; until then they come back "ERR bad command", which is the
        // board being correct rather than anything here being broken.
        ImGui::TableNextRow();
        cmd("?  status", "?");
        cmd("PING",      "PING");
        cmd("ID",        "ID");

        ImGui::TableNextRow();
        cmd("HELP",    "HELP");
        cmd("LED ON",  "LED ON");
        cmd("LED OFF", "LED OFF");

        ImGui::TableNextRow();
        cmd("Blink 2",   "LED BLINK 2");
        cmd("Blink off", "LED BLINK 0");
        ImGui::EndTable();
    }

    {
        ScopedFont sf(ui::fonts.small);
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("The board on the bench answers only ?. Everything else "
                            "returns ERR bad command until firmware/ is flashed.");
        ImGui::PopTextWrapPos();
    }

    // Free-text line. Enter sends and keeps the cursor here, which is what you
    // want when you are poking at a fresh command vocabulary.
    const float send_w = ImGui::CalcTextSize("Send").x + sty.FramePadding.x * 2.0f;
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - send_w - sty.ItemSpacing.x);

    bool fire = ImGui::InputTextWithHint("##picocmdline", "type a command",
                                         g_cmd_buf, sizeof(g_cmd_buf),
                                         ImGuiInputTextFlags_EnterReturnsTrue);
    if (fire) ImGui::SetKeyboardFocusHere(-1);

    ImGui::SameLine();
    if (ImGui::Button("Send")) fire = true;

    if (fire)
    {
        SendPico(g_cmd_buf);
        g_cmd_buf[0] = '\0';
    }
    ImGui::EndDisabled();

    // ---- flashing --------------------------------------------------------
    ImGui::Spacing();
    ImGui::SeparatorText("Flashing");

    const std::string bport = live ? g_pico.port()
                            : (g_pico_index >= 0 && g_pico_index < (int)g_pico_ports.size()
                               ? g_pico_ports[g_pico_index] : std::string());

    ImGui::BeginDisabled(bport.empty());
    if (ImGui::Button("Reboot to BOOTSEL...", ImVec2(-FLT_MIN, bh)))
        ImGui::OpenPopup("Reboot to BOOTSEL?");
    ImGui::EndDisabled();

    {
        ScopedFont sf(ui::fonts.small);
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("Stops the board and remounts it as the RPI-RP2 drive so a "
                            ".uf2 can be copied over. Confirmation required.");
        ImGui::PopTextWrapPos();
    }

    if (g_bootsel_done)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, g_bootsel_ok ? ui::plot::ok : ui::plot::bad);
        ImGui::TextWrapped("%s", g_bootsel_ok
            ? "BOOTSEL touch sent. The port is gone until a .uf2 is copied or the board is power-cycled."
            : "BOOTSEL touch failed - the port could not be opened.");
        ImGui::PopStyleColor();
    }

    // Kept last so nothing above it moves when the link state changes - the
    // command buttons stay where your hand expects them.
    ImGui::Spacing();
    {
        ScopedFont sf(ui::fonts.small);
        ImGui::PushTextWrapPos(0.0f);
        if (live && age < 0.0)
            ImGui::TextDisabled("Port open, board silent. It only speaks when spoken to - "
                                "send ? and it should answer.");
        else if (live)
            ImGui::TextDisabled("Silence between commands is normal: this board answers "
                                "only when asked.");
        ImGui::PopTextWrapPos();
    }

    const ImVec2 centre = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(centre, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Reboot to BOOTSEL?", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::PushTextWrapPos(420.0f * g_dpi);
        ImGui::TextWrapped("This reboots %s into the RP2350 USB bootloader.",
                           bport.empty() ? "the board" : bport.c_str());
        ImGui::Spacing();
        ImGui::BulletText("Whatever the board is running stops immediately.");
        ImGui::BulletText("The serial link drops and the port disappears.");
        ImGui::BulletText("It remounts as the RPI-RP2 mass-storage drive.");
        ImGui::BulletText("It does not come back until a .uf2 is copied onto it,\n"
                          "or the board is power-cycled.");
        ImGui::Spacing();
        ImGui::TextWrapped("Only do this when you are about to flash.");
        ImGui::PopTextWrapPos();

        ImGui::Separator();

        if (ImGui::Button("Cancel", ImVec2(150.0f * g_dpi, bh)))
            ImGui::CloseCurrentPopup();

        ImGui::SameLine();
        if (ImGui::Button("Reboot to BOOTSEL", ImVec2(260.0f * g_dpi, bh)))
        {
            g_pico.disconnect();
            g_bootsel_ok   = PicoLink::bootsel_touch(bport);
            g_bootsel_done = true;
            RefreshPicoPorts();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

// ------------------------------------------------------------------ debug
// The one scrolling pane in the app. It is a log; a log that cannot scroll is
// not a log.

void TabDebug()
{
    if (ImGui::Button("Clear")) g_log.clear();
    ImGui::SameLine();
    ImGui::Checkbox("Auto-scroll", &g_log_autoscroll);

    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::InputTextWithHint("##logfilter", "filter lines", g_filter_buf, sizeof(g_filter_buf));

    g_log_shown.clear();
    for (int i = 0; i < (int)g_log.size(); ++i)
        if (LogMatches(g_log[i])) g_log_shown.push_back(i);

    {
        ScopedFont sf(ui::fonts.small);
        if (g_filter_buf[0])
            ImGui::TextDisabled("%d of %d lines   -   %llu sent / %llu received",
                                (int)g_log_shown.size(), (int)g_log.size(),
                                g_pico.tx_lines(), g_pico.rx_lines());
        else
            ImGui::TextDisabled("%d lines   -   %llu sent / %llu received",
                                (int)g_log.size(), g_pico.tx_lines(), g_pico.rx_lines());
    }

    {
        ScopedFont sf(ui::fonts.small);
        ImGui::PushStyleColor(ImGuiCol_Text, ui::plot::ramp_near);
        ImGui::TextUnformatted(">  host to Pico");
        ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Text, ui::plot::ramp_far);
        ImGui::TextUnformatted("<  Pico to host");
        ImGui::PopStyleColor();
    }

    ImGui::BeginChild("##console", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders,
                      ImGuiWindowFlags_HorizontalScrollbar);

    if (g_log_shown.empty())
    {
        ImGui::PushTextWrapPos(0.0f);
        if (g_log.empty())
            ImGui::TextDisabled("No traffic. Connect on the Pico tab, then send PING - "
                                "host lines appear here even if the board never answers.");
        else
            ImGui::TextDisabled("No lines match \"%s\".", g_filter_buf);
        ImGui::PopTextWrapPos();
    }
    else
    {
        ImGuiListClipper clipper;
        clipper.Begin((int)g_log_shown.size());
        while (clipper.Step())
        {
            for (int r = clipper.DisplayStart; r < clipper.DisplayEnd; ++r)
            {
                const PicoLine& ln = g_log[g_log_shown[r]];

                char buf[512];
                std::snprintf(buf, sizeof(buf), "%8.2f  %c  %s",
                              ln.t_s, ln.outgoing ? '>' : '<', ln.text.c_str());

                ImGui::PushStyleColor(ImGuiCol_Text,
                                      ln.outgoing ? ui::plot::ramp_near : ui::plot::ramp_far);
                ImGui::TextUnformatted(buf);
                ImGui::PopStyleColor();
            }
        }
    }

    // Sticks to the bottom only while the view already is at the bottom, so
    // scrolling up to read something does not yank you back.
    if (g_log_autoscroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
        ImGui::SetScrollHereY(1.0f);

    ImGui::EndChild();
}

// --------------------------------------------------------- control bar

void DrawControlBar()
{
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Range");
    ImGui::SameLine();

    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6.0f);
    if (ImGui::Combo("##range", &g_range_index, kRangeItems, kRangeCount))
        ApplyRange();

    ImGui::SameLine();
    ImGui::TextUnformatted("|");
    ImGui::SameLine();

    ImGui::Checkbox("Grid", &g_radar.show_grid);          ImGui::SameLine();
    ImGui::Checkbox("Trail", &g_radar.show_trail);        ImGui::SameLine();
    ImGui::Checkbox("Labels", &g_radar.show_labels);      ImGui::SameLine();
    ImGui::Checkbox("Nearest", &g_radar.show_nearest);

    ImGui::SameLine();
    ImGui::TextUnformatted("|");
    ImGui::SameLine();

    if (ImGui::Button("Reset view"))
    {
        g_range_index = 0;
        g_radar.fit();
    }

    ImGui::SameLine();
    ScopedFont sf(ui::fonts.small);
    ImGui::TextDisabled("drag pans  -  wheel zooms  -  double-click fits  -  right-drag measures");
}

} // namespace

// ------------------------------------------------------------------- public

void app::Init(float dpi_scale)
{
    g_dpi = dpi_scale > 0.0f ? dpi_scale : 1.0f;

    for (int i = 0; i < kRangeCount; ++i) kRangeItems[i] = kRanges[i].label;

    RefreshPorts();
    RefreshPicoPorts();
    ApplyRange();

    // --connect [port] [baud] pins a specific port; --no-connect suppresses the
    // automatic attempt. With neither, we just connect: this is a single-purpose
    // tool and the common case is "plug it in and look at it".
    bool suppress = false;

    for (int i = 1; i < __argc; ++i)
    {
        if (std::strcmp(__argv[i], "--no-connect") == 0)
        {
            suppress = true;
            continue;
        }

        // --tab names both levels: a top tab, or one of the lidar sub-tabs by
        // its own name (which also selects Lidar above it).
        if (std::strcmp(__argv[i], "--tab") == 0 && i + 1 < __argc)
        {
            struct TabName { const char* name; int top; int sub; };
            static const TabName kTabNames[] = {
                { "lidar",  0, -1 }, { "live",   0,  0 }, { "signal", 0, 1 },
                { "scan",   0,  2 }, { "device", 0,  3 },
                { "pico",   1, -1 }, { "debug",  2, -1 },
            };
            for (const TabName& t : kTabNames)
                if (_stricmp(__argv[i + 1], t.name) == 0)
                {
                    g_force_top        = t.top;
                    g_force_sub        = t.sub;
                    g_force_tab_frames = 4;
                }
            continue;
        }

        // --range <metres> pins the view instead of auto-fitting.
        if (std::strcmp(__argv[i], "--range") == 0 && i + 1 < __argc)
        {
            const float m = (float)std::atof(__argv[i + 1]);
            for (int k = 0; k < kRangeCount; ++k)
                if (kRanges[k].mm > 0.0f && std::fabs(kRanges[k].mm - m * 1000.0f) < 1.0f)
                    g_range_index = k;
            ApplyRange();
            continue;
        }

        if (std::strcmp(__argv[i], "--connect") != 0) continue;

        if (i + 1 < __argc && __argv[i + 1][0] != '-')
        {
            for (int p = 0; p < (int)g_ports.size(); ++p)
                if (_stricmp(g_ports[p].c_str(), __argv[i + 1]) == 0) g_port_index = p;
        }
        if (i + 2 < __argc && __argv[i + 2][0] != '-')
        {
            const int b = std::atoi(__argv[i + 2]);
            for (int k = 0; k < 3; ++k)
                if (kBauds[k] == b) g_baud_index = k;
        }
        break;
    }

    if (!suppress) Connect();
}

void app::SetDpiScale(float dpi_scale)
{
    g_dpi = dpi_scale > 0.0f ? dpi_scale : 1.0f;
}

void app::Frame()
{
    PumpData();

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);

    ImGui::Begin("##root", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                 ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus |
                 ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    const ImVec2 avail  = ImGui::GetContentRegionAvail();
    const float  rail_w = std::min(380.0f * g_dpi, avail.x * 0.36f);
    const float  gap    = ImGui::GetStyle().ItemSpacing.x;
    const float  ctrl_h = ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y * 2.0f;
    const float  map_w  = avail.x - rail_w - gap;
    const float  map_h  = avail.y - ctrl_h - gap;

    if (map_w > 80.0f * g_dpi && map_h > 80.0f * g_dpi)
    {
        const ImVec2 p0 = ImGui::GetCursorScreenPos();

        ImGui::BeginChild("##map", ImVec2(map_w, map_h), ImGuiChildFlags_Borders,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        g_radar.draw(ImGui::GetContentRegionAvail());
        DrawMapHud(p0, ImVec2(map_w, map_h));
        ImGui::EndChild();

        ImGui::BeginChild("##controls", ImVec2(map_w, ctrl_h), ImGuiChildFlags_Borders,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        DrawControlBar();
        ImGui::EndChild();

        ImGui::SetCursorScreenPos(ImVec2(p0.x + map_w + gap, p0.y));
    }

    ImGui::BeginChild("##rail", ImVec2(rail_w, avail.y), ImGuiChildFlags_Borders,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    {
        // Two levels: subsystems on top, readouts underneath. Tabs rather than
        // a scrollbar - there is more here than fits at once, and the no-scroll
        // rule is deliberate.
        if (ImGui::BeginTabBar("##toptabs"))
        {
            // --tab selects at startup. The flag has to persist for a few
            // frames: the tab bar only honours SetSelected once it has laid the
            // items out, which is not on frame one.
            auto top = [](int which)
            {
                return (g_force_top == which && g_force_tab_frames > 0)
                     ? ImGuiTabItemFlags_SetSelected : 0;
            };
            auto sub = [](int which)
            {
                return (g_force_sub == which && g_force_tab_frames > 0)
                     ? ImGuiTabItemFlags_SetSelected : 0;
            };
            if (g_force_tab_frames > 0) --g_force_tab_frames;

            if (ImGui::BeginTabItem("Lidar", nullptr, top(0)))
            {
                // Lidar-specific, so it lives inside the lidar tab rather than
                // above the whole rail.
                DrawConnection();

                if (ImGui::BeginTabBar("##lidartabs"))
                {
                    if (ImGui::BeginTabItem("Live",   nullptr, sub(0))) { TabLive();   ImGui::EndTabItem(); }
                    if (ImGui::BeginTabItem("Signal", nullptr, sub(1))) { TabSignal(); ImGui::EndTabItem(); }
                    if (ImGui::BeginTabItem("Scan",   nullptr, sub(2))) { TabScan();   ImGui::EndTabItem(); }
                    if (ImGui::BeginTabItem("Device", nullptr, sub(3))) { TabDevice(); ImGui::EndTabItem(); }
                    ImGui::EndTabBar();
                }
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Pico",  nullptr, top(1))) { TabPico();  ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Debug", nullptr, top(2))) { TabDebug(); ImGui::EndTabItem(); }
            ImGui::EndTabBar();
        }
    }
    ImGui::EndChild();

    ImGui::End();
}

void app::Shutdown()
{
    g_lidar.stop();
    g_pico.disconnect();
}
