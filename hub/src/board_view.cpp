// Interactive 2D board diagrams. See board_view.h.
//
// The drawing is laid out in MILLIMETRES of real board and scaled to whatever
// layout space it is given, so it stays to scale at any window size. Nothing is
// hard-coded in pixels except text, which has a floor to stay legible.
//
// The pinout and the project's own assignments live in ONE table (PICO2_W).
// docs/wiring.md is its source; changing a wire means changing a row here.

#include "shared.hpp"
#include "board_view.hpp"
#include "theme.hpp"

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace board {
namespace {

// imgui.h does not export ImMin/ImMax/ImClamp - they live in imgui_internal.h,
// which nothing else here includes.
inline Float32 minf(Float32 a, Float32 b) { return a < b ? a : b; }
inline Float32 maxf(Float32 a, Float32 b) { return a > b ? a : b; }
inline Float32 clampf(Float32 v, Float32 lo, Float32 hi) { return v < lo ? lo : (v > hi ? hi : v); }

// =========================================================== the pin table ==

enum class Kind : UInt8 { KIND_GPIO, KIND_GROUND, KIND_POWER, KIND_SYS };

struct Pin
{
    Int32         phys;   // physical pad number, 1..40
    const Char* name;   // silkscreen name
    Kind        kind;
    const Char* use;    // what the project wires it to, or nullptr
};

// Repeated verbatim on every ground pad because it is an invariant, not a note
// about one particular pin.
constexpr const Char* GND = "Common ground with ESC - mandatory";

// Physical order: 1 is top-left, 1..20 run DOWN the left edge, 21 is
// bottom-right, 21..40 run UP the right edge, so 40 is top-right.
constexpr Pin PICO2_W[40] = {
    // left edge, top -> bottom
    {  1, "GP0",         Kind::KIND_GPIO,   "Servo signal" },
    {  2, "GP1",         Kind::KIND_GPIO,   "ESC signal" },
    {  3, "GND",         Kind::KIND_GROUND, GND },
    {  4, "GP2",         Kind::KIND_GPIO,   nullptr },
    {  5, "GP3",         Kind::KIND_GPIO,   nullptr },
    {  6, "GP4",         Kind::KIND_GPIO,   "I2C SDA - ToF, IMU, display" },
    {  7, "GP5",         Kind::KIND_GPIO,   "I2C SCL" },
    {  8, "GND",         Kind::KIND_GROUND, GND },
    {  9, "GP6",         Kind::KIND_GPIO,   nullptr },
    { 10, "GP7",         Kind::KIND_GPIO,   nullptr },
    { 11, "GP8",         Kind::KIND_GPIO,   nullptr },
    { 12, "GP9",         Kind::KIND_GPIO,   "UART RX - lidar, if moved to the Pico" },
    { 13, "GND",         Kind::KIND_GROUND, GND },
    { 14, "GP10",        Kind::KIND_GPIO,   "ToF #1 XSHUT - front level" },
    { 15, "GP11",        Kind::KIND_GPIO,   "ToF #2 XSHUT - front ~20 deg down, curb" },
    { 16, "GP12",        Kind::KIND_GPIO,   "ToF #3 XSHUT" },
    { 17, "GP13",        Kind::KIND_GPIO,   "ToF #4 XSHUT" },
    { 18, "GND",         Kind::KIND_GROUND, GND },
    { 19, "GP14",        Kind::KIND_GPIO,   nullptr },
    { 20, "GP15",        Kind::KIND_GPIO,   "Encoder signal" },
    // right edge, bottom -> top
    { 21, "GP16",        Kind::KIND_GPIO,   "SPI MISO - SD" },
    { 22, "GP17",        Kind::KIND_GPIO,   "SPI CS" },
    { 23, "GND",         Kind::KIND_GROUND, GND },
    { 24, "GP18",        Kind::KIND_GPIO,   "SPI SCK" },
    { 25, "GP19",        Kind::KIND_GPIO,   "SPI MOSI" },
    { 26, "GP20",        Kind::KIND_GPIO,   nullptr },
    { 27, "GP21",        Kind::KIND_GPIO,   nullptr },
    { 28, "GND",         Kind::KIND_GROUND, GND },
    { 29, "GP22",        Kind::KIND_GPIO,   nullptr },
    { 30, "RUN",         Kind::KIND_SYS,    nullptr },
    { 31, "GP26 (ADC0)", Kind::KIND_GPIO,   nullptr },
    { 32, "GP27 (ADC1)", Kind::KIND_GPIO,   nullptr },
    { 33, "AGND",        Kind::KIND_GROUND, nullptr },
    { 34, "GP28 (ADC2)", Kind::KIND_GPIO,   nullptr },
    { 35, "ADC_VREF",    Kind::KIND_POWER,  nullptr },
    { 36, "3V3(OUT)",    Kind::KIND_POWER,  nullptr },
    { 37, "3V3_EN",      Kind::KIND_SYS,    nullptr },
    { 38, "GND",         Kind::KIND_GROUND, GND },
    { 39, "VSYS",        Kind::KIND_POWER,  nullptr },
    { 40, "VBUS",        Kind::KIND_POWER,  nullptr },
};

constexpr Int32 PIN_COUNT = static_cast<Int32>((sizeof(PICO2_W) / sizeof(PICO2_W[0])));

// Three categories, and only three: what the project uses, what carries power,
// and what is still free.
enum class Cat : UInt8 { CAT_FREE, CAT_ASSIGNED, CAT_POWER_GND };

constexpr Cat catOf(const Pin& p)
{
    return (p.kind != Kind::KIND_GPIO) ? Cat::CAT_POWER_GND
         : (p.use ? Cat::CAT_ASSIGNED : Cat::CAT_FREE);
}

const Char* kindWord(Kind k)
{
    switch(k)
    {
    case Kind::KIND_GROUND: return "Ground";
    case Kind::KIND_POWER:  return "Power";
    case Kind::KIND_SYS:    return "System";
    case Kind::KIND_GPIO:   break;
    }
    return "GPIO";
}

// ======================================================= board geometry, mm ==
//
// Raspberry Pi Pico 2 W: 51.0 x 21.0 mm, 40 castellations on a 2.54 mm pitch,
// 20 per long edge, the row centred along the length.

constexpr Float32 BOARD_W = 21.0f;
constexpr Float32 BOARD_H = 51.0f;
constexpr Float32 PITCH  = 2.54f;
constexpr Float32 PIN_Y0  = (BOARD_H - 19.0f * PITCH) * 0.5f;   // 1.37
constexpr Float32 PAD_W   = 3.4f;
constexpr Float32 PAD_H   = 1.9f;

constexpr Float32 LABEL_W = 17.5f;   // name + number column, each side
constexpr Float32 TOP_PAD = 2.6f;    // the USB shell overhangs the top edge
constexpr Float32 BOT_PAD = 1.4f;

inline Bool  onLeft(Int32 phys)  { return phys <= 20; }
inline Float32 pinY(Int32 phys)
{
    const Int32 idx = onLeft(phys) ? (phys - 1) : (40 - phys);
    return PIN_Y0 + static_cast<Float32>(idx) * PITCH;
}

// ================================================================= colours ==
//
// The board is drawn as the object looks: green solder mask, white silkscreen,
// gold-plated castellations, black packages, a bright shield can over the radio.
// The tab exists to be recognised as the thing on the desk, and a grey
// abstraction of it was not being recognised as anything.
//
// The pin CATEGORY therefore cannot live in the pad fill any more - every pad is
// gold on the real board. It moves to a short mask stripe just inboard of each
// pad, which reads as board decoration rather than as a recolouring of the
// hardware, and to the label outside the board where there is black to sit on.

constexpr ImU32 BOARD_FILL = IM_COL32(0x14, 0x50, 0x31, 0xFF);   // solder mask
constexpr ImU32 BOARD_EDGE = IM_COL32(0x0A, 0x2E, 0x1C, 0xFF);   // routed edge
constexpr ImU32 PART_FILL  = IM_COL32(0x14, 0x14, 0x17, 0xFF);   // black package
constexpr ImU32 PART_EDGE  = IM_COL32(0x33, 0x33, 0x39, 0xFF);
constexpr ImU32 METAL     = IM_COL32(0x8A, 0x91, 0x9A, 0xFF);   // can / USB shell
constexpr ImU32 METAL_EDGE = IM_COL32(0xBC, 0xC4, 0xCC, 0xFF);
constexpr ImU32 HOLE      = IM_COL32(0x07, 0x09, 0x08, 0xFF);
constexpr ImU32 SILK      = IM_COL32(0xEC, 0xF0, 0xEC, 0xFF);   // white legend, ON the board

// The board view lives inside the DARK viewport, so text drawn off the board
// still sits on a dark ground and stays light. Status here comes from ui::plot,
// not ui::sem - see the note on the two grounds in theme.hpp.
constexpr ImU32 GOLD      = IM_COL32(0xC6, 0xA0, 0x4A, 0xFF);   // ENIG plating
constexpr ImU32 GOLD_EDGE  = IM_COL32(0xE8, 0xC8, 0x7C, 0xFF);
constexpr ImU32 FREE_PAD   = IM_COL32(0x6E, 0x7A, 0x72, 0xFF);   // free-GPIO stripe

// The user LED. Green on the real board, and off is a dark green lens rather
// than a hole - an unlit LED is still visibly an LED.
constexpr ImU32 LED_OFF    = IM_COL32(0x24, 0x3A, 0x2A, 0xFF);
constexpr ImU32 LED_ON     = IM_COL32(0x7C, 0xF7, 0x86, 0xFF);
constexpr ImU32 LED_RIM    = IM_COL32(0x0E, 0x1A, 0x12, 0xFF);

ImU32 withAlpha(ImU32 c, Float32 a)
{
    const ImU32 v = static_cast<ImU32>((clampf(a, 0.0f, 1.0f) * 255.0f + 0.5f));
    return (c & ~IM_COL32_A_MASK) | (v << IM_COL32_A_SHIFT);
}

// A pin category is not a status, and these used to be drawn as though it were:
// assigned took sem::GOOD and power/ground took sem::WARN, so a whole column of
// GND / VBUS / VSYS / 3V3 labels sat in warning orange announcing a problem that
// did not exist. Power is not a warning. An assigned pin is not "healthy".
//
// The three categories encode ONE axis - how much this pin has to do with the
// project - so ui::pin is one hue at three weights rather than three unrelated
// hues borrowed from the status palette.
ImU32 catColor(Cat c)
{
    switch(c)
    {
    case Cat::CAT_ASSIGNED:  return ui::pin::ASSIGNED;
    case Cat::CAT_POWER_GND: return ui::pin::POWER;
    case Cat::CAT_FREE:      break;
    }
    return ui::pin::FREE;
}

ImU32 labelColor(Cat c)
{
    return catColor(c);
}

// ==================================================================== paint ==

struct Paint
{
    ImDrawList* dl = nullptr;
    ImVec2      org{};      // screen position of model (0,0)
    Float32       s = 1.0f;   // pixels per millimetre

    ImVec2 toScreen(Float32 x, Float32 y) const { return ImVec2(org.x + x * s, org.y + y * s); }
    Float32  toPx(Float32 mm)         const { return mm * s; }

    Void rect(Float32 x0, Float32 y0, Float32 x1, Float32 y1, ImU32 fill, ImU32 edge,
              Float32 roundMm = 0.0f) const
    {
        const Float32 r = roundMm * s;
        if(fill & IM_COL32_A_MASK) dl->AddRectFilled(toScreen(x0, y0), toScreen(x1, y1), fill, r);
        if(edge & IM_COL32_A_MASK) dl->AddRect(toScreen(x0, y0), toScreen(x1, y1), edge, r, 0, maxf(1.0f, s * 0.09f));
    }
};

// The pin name may carry an ADC alias - "GP26 (ADC0)". The alias is drawn dimmer
// than the name so the column still scans as a list of GPIO numbers.
struct SplitName
{
    const Char* mainB = nullptr;
    const Char* mainE = nullptr;
    const Char* altB  = nullptr;   // "(ADC0)", or nullptr
};

SplitName split(const Char* name)
{
    SplitName sn;
    sn.mainB = name;
    const Char* p = std::strstr(name, " (");
    sn.mainE = p ? p : (name + std::strlen(name));
    sn.altB  = p ? (p + 1) : nullptr;
    return sn;
}

// =============================================================== the drawing ==

// Is the LED lit on THIS frame?
//
// Blink is animated from the host clock at the rate the board reported, and the
// phase is deliberately not claimed to match: nothing tells us where in its
// cycle the board is, and pretending otherwise would be a lie that looks like a
// measurement. What this shows is "it is blinking, at about this rate", which is
// what the reported state actually supports.
Bool ledLit(const Live& lv)
{
    switch(lv.led)
    {
    case Live::Led::LED_ON:    return true;
    case Live::Led::LED_BLINK:
        if(!(lv.ledHz > 0.0f)) return false;
        return std::fmod(ImGui::GetTime() * static_cast<Float64>(lv.ledHz), 1.0) < 0.5;
    case Live::Led::LED_OFF:
    case Live::Led::LED_UNKNOWN:
        break;
    }
    return false;
}

Void drawPico2W(ImDrawList* dl, const ImVec2& rmin, const ImVec2& rmax, Bool interactive,
                const Live& lv)
{
    const Float32 aw = rmax.x - rmin.x;
    const Float32 ah = rmax.y - rmin.y;
    if(aw < 8.0f || ah < 8.0f) return;

    const Float32 mw = BOARD_W + 2.0f * LABEL_W;          // 56 mm of model width
    const Float32 mh = BOARD_H + TOP_PAD + BOT_PAD;       // 55 mm of model height

    Paint g;
    g.dl = dl;
    g.s  = minf(aw / mw, ah / mh);
    if(g.s <= 0.0f) return;
    g.org.x = rmin.x + (aw - mw * g.s) * 0.5f + LABEL_W * g.s;
    g.org.y = rmin.y + (ah - mh * g.s) * 0.5f + TOP_PAD * g.s;

    // ---- type sizes ---------------------------------------------------------
    // LegacySize already has the DPI baked in; it is a ceiling, never a factor.
    // The only thing that shrinks text is a cramped pin pitch.
    ImFont* font = ui::fonts.small ? ui::fonts.small : ImGui::GetFont();
    const Float32 base = (font && font->LegacySize > 0.0f) ? font->LegacySize
                                                         : ImGui::GetFontSize();
    const Float32 pitchPx = PITCH * g.s;
    const Float32 fs       = minf(base, pitchPx * 0.78f);
    const Bool  labels   = fs >= 7.0f;
    const Float32 numColW = fs * 1.6f;
    const Float32 silkFs  = minf(base * 0.85f, g.s * 1.5f);
    const Bool  silk     = silkFs >= 8.0f;

    // How far the labels actually reach outboard of the pads, per side. The hit
    // column below is deliberately wider than any name so the rows are easy to
    // point at, but the hover highlight has to hug the text instead — sized to
    // the hit column it hangs out in empty space beside the short names.
    Float32 labelExt[2] = { 0.0f, 0.0f };   // [0] left side, [1] right side
    if(labels)
        for(Int32 i = 0; i < PIN_COUNT; ++i)
        {
            const SplitName sn = split(PICO2_W[i].name);
            const Float32 w = numColW
                          + font->CalcTextSizeA(fs, FLT_MAX, 0.0f, sn.mainB, sn.mainE).x
                          + (sn.altB ? fs * 0.28f
                                      + font->CalcTextSizeA(fs, FLT_MAX, 0.0f, sn.altB).x
                                      : 0.0f);
            Float32& e = labelExt[onLeft(PICO2_W[i].phys) ? 0 : 1];
            if(w > e) e = w;
        }

    // ---- hover --------------------------------------------------------------
    const ImVec2 mouse = ImGui::GetMousePos();
    Int32 hot = -1;
    if(interactive)
    {
        for(Int32 i = 0; i < PIN_COUNT; ++i)
        {
            const Float32 y  = pinY(PICO2_W[i].phys);
            const ImVec2 a = g.toScreen(onLeft(PICO2_W[i].phys) ? -LABEL_W : BOARD_W - PAD_W,
                                 y - PITCH * 0.5f);
            const ImVec2 b = g.toScreen(onLeft(PICO2_W[i].phys) ? PAD_W : BOARD_W + LABEL_W,
                                 y + PITCH * 0.5f);
            if(mouse.x >= a.x && mouse.x <= b.x && mouse.y >= a.y && mouse.y <= b.y)
            {
                hot = i;
                break;
            }
        }
    }

    // ---- board slab ---------------------------------------------------------
    g.rect(0.0f, 0.0f, BOARD_W, BOARD_H, BOARD_FILL, BOARD_EDGE, 1.0f);

    // USB micro-B shell, overhanging the top edge. Ringed when the CDC link is
    // actually open, which is the one thing about the connector that can change.
    g.rect(6.75f, -1.9f, 14.25f, 4.6f, METAL, METAL_EDGE, 0.4f);
    g.rect(7.85f, -1.2f, 13.15f, 1.4f, IM_COL32(0x18, 0x18, 0x1C, 0xFF), METAL_EDGE, 0.3f);
    if(lv.link)
        g.rect(6.45f, -2.2f, 14.55f, 4.9f, 0, withAlpha(ui::plot::OK, 0.85f), 0.5f);

    // Silkscreen title, in the gap the RPi logo occupies on the real board.
    if(silk)
    {
        const Char* t = "Pico 2 W";
        const Float32 tw = font->CalcTextSizeA(silkFs, FLT_MAX, 0.0f, t).x;
        dl->AddText(font, silkFs, ImVec2(g.toScreen(BOARD_W * 0.5f, 6.2f).x - tw * 0.5f,
                                          g.toScreen(0.0f, 6.2f).y), withAlpha(SILK, 0.55f), t);
    }

    // User LED.
    //
    // On a Pico 2 W this is NOT an RP2350 GPIO - it hangs off the CYW43439, so
    // it cannot light at all unless the wireless chip came up. That is why the
    // lens is forced dark when cyw43 is known to have failed: a board reporting
    // "led on" with a dead radio has an LED that is not on, and drawing it lit
    // would contradict the hardware. See firmware/src/main.c.
    //
    // Position is approximate. It sits in the top-left region of the real board;
    // this drawing does not need the exact offset to be useful, and inventing
    // precision it does not have would be worse than being visibly schematic.
    {
        const Bool  usable = (lv.cyw43 != Live::Tri::TRI_NO);
        const Bool  lit    = usable && ledLit(lv);
        const Bool  known  = (lv.led != Live::Led::LED_UNKNOWN);

        // Clear of the pad column (which runs to x = PAD_W), of the "Pico 2 W"
        // legend above, and of the BOOTSEL body to the right at x >= 8.
        const Float32 x0 = 4.1f, y0 = 8.8f, x1 = 6.1f, y1 = 10.1f;

        if(lit)
        {
            // The same lamp the chrome uses, so the board's LED and the console's
            // indicators glow identically - it is the one real LED on screen and
            // it should not be the odd one out.
            ui::led(dl, g.toScreen((x0 + x1) * 0.5f, (y0 + y1) * 0.5f),
                    maxf(1.5f, g.toPx(0.75f)), LED_ON, true);
        }

        g.rect(x0, y0, x1, y1, lit ? LED_ON : LED_OFF, LED_RIM, 0.2f);

        // Unknown is drawn hollow: nothing has told us what this LED is doing.
        if(!known)
            g.rect(x0, y0, x1, y1, IM_COL32(0, 0, 0, 0),
                   withAlpha(ui::plot::IDLE, 0.9f), 0.2f);

        if(silk)
            dl->AddText(font, silkFs,
                        ImVec2(g.toScreen(x1 + 0.45f, 0.0f).x,
                               g.toScreen(0.0f, (y0 + y1) * 0.5f).y - silkFs * 0.5f),
                        withAlpha(SILK, 0.6f), "LED");
    }

    // BOOTSEL. Lit amber when the board is actually sitting in the bootloader,
    // because in that state it is not running anything and every other reading
    // on this drawing is stale.
    g.rect(8.0f, 9.0f, 13.0f, 12.6f, PART_FILL, PART_EDGE, 0.3f);
    dl->AddCircleFilled(g.toScreen(10.5f, 10.8f), maxf(1.0f, g.toPx(1.05f)),
                        lv.bootsel ? ui::plot::WARN : METAL);
    dl->AddCircle(g.toScreen(10.5f, 10.8f), maxf(1.0f, g.toPx(1.05f)),
                  lv.bootsel ? IM_COL32_WHITE : METAL_EDGE, 0,
                  maxf(1.0f, g.s * 0.09f));
    if(lv.bootsel)
        g.rect(7.7f, 8.7f, 13.3f, 12.9f, 0, withAlpha(ui::plot::WARN, 0.9f), 0.4f);
    if(silk)
    {
        const Char* t = "BOOTSEL";
        const Float32 tw = font->CalcTextSizeA(silkFs, FLT_MAX, 0.0f, t).x;
        dl->AddText(font, silkFs, ImVec2(g.toScreen(10.5f, 12.9f).x - tw * 0.5f,
                                          g.toScreen(0.0f, 12.9f).y), withAlpha(SILK, 0.45f), t);
    }

    // RP2350, QFN-60 7x7.
    g.rect(7.0f, 17.0f, 14.0f, 24.0f, IM_COL32(0x1F, 0x1F, 0x24, 0xFF), PART_EDGE, 0.3f);
    dl->AddCircleFilled(g.toScreen(7.9f, 17.9f), maxf(1.0f, g.toPx(0.45f)), PART_EDGE);
    if(silk)
    {
        const Char* t = "RP2350";
        const Float32 tw = font->CalcTextSizeA(silkFs, FLT_MAX, 0.0f, t).x;
        dl->AddText(font, silkFs, ImVec2(g.toScreen(10.5f, 20.0f).x - tw * 0.5f,
                                          g.toScreen(0.0f, 20.0f).y), withAlpha(SILK, 0.5f), t);
    }

    // QSPI flash.
    g.rect(8.3f, 26.5f, 12.7f, 30.3f, PART_FILL, PART_EDGE, 0.2f);

    // CYW43439 radio, under its shield can. The can is bright metal, so its
    // legend is drawn dark rather than white.
    g.rect(5.0f, 32.6f, 16.0f, 40.4f, METAL, METAL_EDGE, 0.4f);
    if(silk)
    {
        const Char* t = "CYW43439";
        const Float32 tw = font->CalcTextSizeA(silkFs, FLT_MAX, 0.0f, t).x;
        dl->AddText(font, silkFs, ImVec2(g.toScreen(10.5f, 35.6f).x - tw * 0.5f,
                                          g.toScreen(0.0f, 35.6f).y),
                    IM_COL32(0x22, 0x26, 0x2A, 0xE0), t);
    }

    // Did it come up? This is the difference between "the LED is off" and "the
    // LED cannot be turned on", and the two look identical without it.
    if(lv.cyw43 != Live::Tri::TRI_UNKNOWN)
    {
        const ImU32 pip = (lv.cyw43 == Live::Tri::TRI_YES) ? ui::plot::OK : ui::plot::BAD;
        ui::led(dl, g.toScreen(15.0f, 33.6f), maxf(1.5f, g.toPx(0.62f)), pip, true);
    }

    // 3-pin debug connector.
    g.rect(8.2f, 41.8f, 12.8f, 44.2f, PART_FILL, PART_EDGE, 0.2f);
    for(Int32 i = 0; i < 3; ++i)
        dl->AddCircleFilled(g.toScreen(9.3f + 1.2f * static_cast<Float32>(i), 43.0f),
                            maxf(1.0f, g.toPx(0.32f)), METAL_EDGE);
    if(silk)
    {
        const Char* t = "DEBUG";
        const Float32 tw = font->CalcTextSizeA(silkFs, FLT_MAX, 0.0f, t).x;
        dl->AddText(font, silkFs, ImVec2(g.toScreen(7.9f, 43.0f).x - tw,
                                          g.toScreen(0.0f, 43.0f).y - silkFs * 0.5f),
                    withAlpha(SILK, 0.45f), t);
    }

    // Antenna keep-out at the bottom edge, with the meandered trace inside it.
    // Kept clear of the two lower mounting holes.
    {
        const ImU32 ko = withAlpha(ui::plot::WARN, 0.35f);
        g.rect(5.8f, 45.4f, 15.2f, 50.6f, IM_COL32(0, 0, 0, 0), ko, 0.3f);

        ImVec2 mp[12];
        for(Int32 k = 0; k < 12; ++k)
        {
            const Float32 col = static_cast<Float32>((k / 2));
            const Bool  top = (((k + 1) / 2) % 2) == 1;
            mp[k] = g.toScreen(7.5f + 1.2f * col, top ? 47.0f : 49.8f);
        }
        dl->AddPolyline(mp, 12, withAlpha(ui::plot::WARN, 0.75f),
                        maxf(1.0f, g.s * 0.22f), 0);

        if(silk)
            dl->AddText(font, silkFs, g.toScreen(6.2f, 45.6f), withAlpha(ui::plot::WARN, 0.7f), "ANT");
    }

    // Mounting holes: 47.0 x 11.4 mm centres.
    {
        const Float32 r = maxf(1.0f, g.toPx(1.05f));
        const Float32 hx[2] = { 4.8f, 16.2f };
        const Float32 hy[2] = { 2.0f, 49.0f };
        for(Int32 a = 0; a < 2; ++a)
            for(Int32 b = 0; b < 2; ++b)
            {
                dl->AddCircleFilled(g.toScreen(hx[a], hy[b]), r, HOLE);
                dl->AddCircle(g.toScreen(hx[a], hy[b]), r, PART_EDGE, 0, maxf(1.0f, g.s * 0.09f));
            }
    }

    // ---- pads and labels ----------------------------------------------------
    for(Int32 i = 0; i < PIN_COUNT; ++i)
    {
        const Pin&  p    = PICO2_W[i];
        const Bool  left = onLeft(p.phys);
        const Cat   cat  = catOf(p);
        const Bool  on   = (i == hot);
        const Float32 y    = pinY(p.phys);

        // Row highlight, so the eye can follow the pad out to its name.
        if(on)
        {
            const Float32 pad = fs * 0.35f;
            const Float32 ext = labelExt[left ? 0 : 1];
            const ImVec2 a(left ? g.toScreen(-0.9f, y).x - ext - pad : g.toScreen(BOARD_W - PAD_W, y).x,
                           g.toScreen(0.0f, y - PITCH * 0.5f).y);
            const ImVec2 b(left ? g.toScreen(PAD_W, y).x : g.toScreen(BOARD_W + 0.9f, y).x + ext + pad,
                           g.toScreen(0.0f, y + PITCH * 0.5f).y);
            dl->AddRectFilled(a, b, IM_COL32(0xFF, 0xFF, 0xFF, 0x14), g.toPx(0.5f));
        }

        // The pad itself, flush with the board edge as a castellation is. Gold,
        // like every other pad on the board.
        const Float32 px0 = left ? 0.0f : BOARD_W - PAD_W;
        const Float32 px1 = left ? PAD_W : BOARD_W;
        g.rect(px0, y - PAD_H * 0.5f, px1, y + PAD_H * 0.5f,
               withAlpha(GOLD, on ? 1.0f : 0.92f),
               on ? IM_COL32_WHITE : withAlpha(GOLD_EDGE, 0.75f), 0.35f);

        // Category stripe, inboard of the pad. This is what the legend refers
        // to now that the pads themselves are all one colour.
        {
            const ImU32 sc = (cat == Cat::CAT_FREE) ? FREE_PAD : catColor(cat);
            const Float32 sx0 = left ? PAD_W + 0.30f : BOARD_W - PAD_W - 0.95f;
            g.rect(sx0, y - PAD_H * 0.34f, sx0 + 0.65f, y + PAD_H * 0.34f,
                   withAlpha(sc, on ? 1.0f : 0.85f), 0, 0.15f);
        }

        // Plated hole.
        const Float32 hx = left ? 1.55f : BOARD_W - 1.55f;
        dl->AddCircleFilled(g.toScreen(hx, y), maxf(1.0f, g.toPx(0.62f)), HOLE);

        // A pin that is being DRIVEN right now, as opposed to merely assigned.
        // Only the two the control firmware reports on: claiming any of the
        // others were live would be decoration, not state.
        if(lv.havePwm && (p.phys == 1 || p.phys == 2))
            g.rect(px0 - 0.3f, y - PAD_H * 0.5f - 0.3f, px1 + 0.3f, y + PAD_H * 0.5f + 0.3f,
                   0, withAlpha(ui::plot::OK, 0.95f), 0.45f);

        if(!labels) continue;

        const ImU32 nameCol = on ? IM_COL32_WHITE : labelColor(cat);
        const ImU32 numCol  = on ? IM_COL32_WHITE : withAlpha(ui::plot::IDLE, 0.85f);

        Char num[8];
        std::snprintf(num, sizeof(num), "%d", p.phys);
        const Float32 numW = font->CalcTextSizeA(fs, FLT_MAX, 0.0f, num).x;

        const SplitName sn   = split(p.name);
        const Float32 mainW   = font->CalcTextSizeA(fs, FLT_MAX, 0.0f, sn.mainB, sn.mainE).x;
        const Float32 altW    = sn.altB ? font->CalcTextSizeA(fs, FLT_MAX, 0.0f, sn.altB).x : 0.0f;
        const Float32 altGap  = sn.altB ? fs * 0.28f : 0.0f;

        const Float32 ty = g.toScreen(0.0f, y).y - fs * 0.5f;

        // Two columns per side: the physical number nearest the pad, the name
        // outboard of it.
        const Float32 numEdge  = left ? g.toScreen(-0.9f, y).x : g.toScreen(BOARD_W + 0.9f, y).x;
        const Float32 nameEdge = left ? (numEdge - numColW) : (numEdge + numColW);

        if(left)
        {
            dl->AddText(font, fs, ImVec2(numEdge - numW, ty), numCol, num);
            const Float32 x0 = nameEdge - (mainW + altGap + altW);
            dl->AddText(font, fs, ImVec2(x0, ty), nameCol, sn.mainB, sn.mainE);
            if(sn.altB)
                dl->AddText(font, fs, ImVec2(x0 + mainW + altGap, ty),
                            withAlpha(nameCol, 0.55f), sn.altB);
        }
        else
        {
            dl->AddText(font, fs, ImVec2(numEdge, ty), numCol, num);
            dl->AddText(font, fs, ImVec2(nameEdge, ty), nameCol, sn.mainB, sn.mainE);
            if(sn.altB)
                dl->AddText(font, fs, ImVec2(nameEdge + mainW + altGap, ty),
                            withAlpha(nameCol, 0.55f), sn.altB);
        }
    }

    // ---- tooltip ------------------------------------------------------------
    if(hot >= 0)
    {
        const Pin& p = PICO2_W[hot];
        ImGui::BeginTooltip();
        ImGui::Text("Pin %d", p.phys);
        ImGui::SameLine();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(labelColor(catOf(p))), "%s", p.name);
        if(ui::fonts.small) ImGui::PushFont(ui::fonts.small, ui::fonts.small->LegacySize);
        if(p.kind != Kind::KIND_GPIO)   ImGui::TextDisabled("%s", kindWord(p.kind));
        else if(!p.use)            ImGui::TextDisabled("Free");
        if(p.use)
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::plot::OK), "%s", p.use);

        // What it is doing now, when that is actually known.
        if(lv.havePwm && (p.phys == 1 || p.phys == 2))
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::plot::OK),
                               "driving %d us", (p.phys == 1) ? lv.servoUs : lv.escUs);

        if(ui::fonts.small) ImGui::PopFont();
        ImGui::EndTooltip();
    }
}

} // namespace

const Char* name(Which w)
{
    switch(w)
    {
    case Which::WHICH_PICO2_W: return "Pico 2 W";
    case Which::WHICH_COUNT:  break;
    }
    return "";
}

// One "label value" chip in the live strip, drawn right to left. Returns the new
// right edge so the caller can pack the next one beside it.
Float32 liveChip(ImDrawList* dl, ImFont* f, Float32 fs, Float32 right, Float32 y,
                 const Char* label, const Char* value, ImU32 col, Bool dot,
                 Bool lit = true)
{
    const Float32 gap  = fs * 0.35f;
    const Float32 vw   = f->CalcTextSizeA(fs, FLT_MAX, 0.0f, value).x;
    const Float32 lw   = f->CalcTextSizeA(fs, FLT_MAX, 0.0f, label).x;
    const Float32 dw   = dot ? fs * 0.85f : 0.0f;

    Float32 x = right - vw;
    dl->AddText(f, fs, ImVec2(x, y), col, value);
    x -= gap + lw;
    dl->AddText(f, fs, ImVec2(x, y), withAlpha(ui::plot::IDLE, 0.9f), label);

    if(dot)
    {
        x -= dw;
        ui::led(dl, ImVec2(x + fs * 0.30f, y + fs * 0.5f), fs * 0.22f, col, lit);
    }
    return x - fs * 1.1f;
}

Void draw(Which w, const ImVec2& size, const Live& lv)
{
    ImVec2 avail = size;
    if(avail.x <= 0.0f) avail.x = ImGui::GetContentRegionAvail().x;
    if(avail.y <= 0.0f) avail.y = ImGui::GetContentRegionAvail().y;
    avail.x = maxf(avail.x, 1.0f);
    avail.y = maxf(avail.y, 1.0f);

    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("##board_view", avail);
    const Bool interactive = ImGui::IsItemHovered();

    if(w != Which::WHICH_PICO2_W) return;

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->PushClipRect(p0, ImVec2(p0.x + avail.x, p0.y + avail.y), true);

    // Legend strip. Three swatches, no explanation.
    const Float32 dpi = ui::dpiScale();
    ImFont*     lf  = ui::fonts.small ? ui::fonts.small : ImGui::GetFont();
    const Float32 lfs = (lf && lf->LegacySize > 0.0f) ? lf->LegacySize : ImGui::GetFontSize();
    const Float32 strip = lfs + 10.0f * dpi;
    const Bool  showLegend = avail.y > strip * 4.0f && avail.x > 260.0f * dpi;

    if(showLegend)
    {
        struct Item { Cat cat; const Char* label; };
        const Item items[3] = {
            { Cat::CAT_ASSIGNED, "Assigned" },
            { Cat::CAT_POWER_GND, "Power / GND" },
            { Cat::CAT_FREE,     "Free GPIO" },
        };

        Float32 x = p0.x + 4.0f * dpi;
        const Float32 y = p0.y + 4.0f * dpi;
        const Float32 sw = lfs * 0.72f;
        for(Int32 i = 0; i < 3; ++i)
        {
            const ImU32 c = (items[i].cat == Cat::CAT_FREE) ? FREE_PAD : catColor(items[i].cat);
            ui::plate(ImVec2(x, y + (lfs - sw) * 0.5f),
                      ImVec2(x + sw, y + (lfs + sw) * 0.5f),
                      c, ImGui::GetStyle().FrameRounding);
            x += sw + 5.0f * dpi;
            dl->AddText(lf, lfs, ImVec2(x, y), SILK, items[i].label);
            x += lf->CalcTextSizeA(lfs, FLT_MAX, 0.0f, items[i].label).x + 16.0f * dpi;
        }

        const Char* dim = "51 x 21 mm";
        const Float32 dw  = lf->CalcTextSizeA(lfs, FLT_MAX, 0.0f, dim).x;
        const Float32 dx  = p0.x + avail.x - 4.0f * dpi - dw;
        if(dx > x) dl->AddText(lf, lfs, ImVec2(dx, y), withAlpha(ui::plot::IDLE, 0.8f), dim);
    }

    // Live strip. What the board is doing, in words, under the legend that says
    // what the colours mean. Only drawn when there is something to report -
    // a row of "unknown" for a board nobody has connected to is noise.
    Float32 top = p0.y + (showLegend ? strip : 0.0f);

    const Bool haveLive = lv.link || lv.bootsel
                        || lv.cyw43 != Live::Tri::TRI_UNKNOWN
                        || lv.led   != Live::Led::LED_UNKNOWN
                        || lv.havePwm;

    if(showLegend && haveLive)
    {
        const Float32 y = top + 2.0f * dpi;
        Float32 right = p0.x + avail.x - 4.0f * dpi;

        if(lv.havePwm)
        {
            Char buf[48];
            std::snprintf(buf, sizeof(buf), "%d / %d us", lv.servoUs, lv.escUs);
            right = liveChip(dl, lf, lfs, right, y, "servo / esc", buf,
                             ui::plot::OK, false);
        }

        if(lv.cyw43 != Live::Tri::TRI_UNKNOWN)
        {
            const Bool up = (lv.cyw43 == Live::Tri::TRI_YES);
            right = liveChip(dl, lf, lfs, right, y, "cyw43", up ? "up" : "FAILED",
                             up ? ui::plot::OK : ui::plot::BAD, false);
        }

        if(lv.led != Live::Led::LED_UNKNOWN)
        {
            Char buf[32];
            const Char* v = "off";
            if(lv.led == Live::Led::LED_ON) v = "on";
            else if(lv.led == Live::Led::LED_BLINK)
            {
                std::snprintf(buf, sizeof(buf), "blink %.1f Hz", static_cast<Float64>(lv.ledHz));
                v = buf;
            }
            const Bool lit = ledLit(lv) && lv.cyw43 != Live::Tri::TRI_NO;
            right = liveChip(dl, lf, lfs, right, y, "LED", v,
                             lit ? LED_ON : ui::plot::IDLE, true, lit);
        }

        if(lv.bootsel)
            right = liveChip(dl, lf, lfs, right, y, "board", "BOOTSEL",
                             ui::plot::WARN, true);
        else if(lv.link)
            right = liveChip(dl, lf, lfs, right, y, "link", "open",
                             ui::plot::OK, true);

        static_cast<Void>(right);
        top += strip;
    }

    drawPico2W(dl, ImVec2(p0.x, top), ImVec2(p0.x + avail.x, p0.y + avail.y),
               interactive, lv);

    dl->PopClipRect();
}

} // namespace board
