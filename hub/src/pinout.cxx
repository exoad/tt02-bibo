// The Pico's pin table - the forty pads, what each one is, and what this
// project has wired to it.
//
// This used to be the bottom of board_view.cxx, under a drawing of the physical
// board. The drawing is gone - there are two boards in this project now and a
// picture of one of them, shown whichever is plugged in, was a diagram that
// could be wrong. The TABLE is not: every pin this project uses is identical on
// the Pico 2 and the Pico 2 W, which is the fact that makes them
// interchangeable, so one table serves both.
//
// The reference library renders it as a flat chart. That is now the only
// rendering, but the split is kept - data here, drawing there - because it is
// what stopped the forty rows from being copied and going stale against
// docs/wiring.md.
#include "shared.hxx"
#include "pinout.hxx"

namespace board {
namespace {

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

inline Bool onLeft(Int32 phys)
{
    return phys <= 20;
}

} // namespace

// ---- the pin table, for other views -------------------------------------
// A read-only window onto the table above. Callers get a PinRef rather than the
// internal Pin so the storage can change without touching them.

Int32 pinCount()
{
    return PIN_COUNT;
}

PinRef pinAt(Int32 i)
{
    PinRef out;
    if(i < 0 || i >= PIN_COUNT)
    {
        return out;
    }

    const Pin& p = PICO2_W[i];
    out.phys = p.phys;
    out.name = p.name;

    // The ground note is an invariant repeated on every ground pad, which is
    // right on a board drawing where you hover one pad at a time and wrong on a
    // chart where it would print eight times down one column.
    out.use  = (p.kind == Kind::KIND_GROUND) ? nullptr : p.use;

    switch(p.kind)
    {
    case Kind::KIND_GPIO:   out.role = PinRole::PIN_ROLE_GPIO;   break;
    case Kind::KIND_GROUND: out.role = PinRole::PIN_ROLE_GROUND; break;
    case Kind::KIND_POWER:  out.role = PinRole::PIN_ROLE_POWER;  break;
    case Kind::KIND_SYS:    out.role = PinRole::PIN_ROLE_SYS;    break;
    }
    return out;
}

Bool pinOnLeft(Int32 phys)
{
    return onLeft(phys);
}

} // namespace board
