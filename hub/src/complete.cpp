#include "complete.hpp"

#include <algorithm>
#include <cstring>

namespace cmpl {
namespace {

Bool identChar(Char c) noexcept
{
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
        || (c >= '0' && c <= '9') || c == '_';
}

Char lower(Char c) noexcept
{
    return (c >= 'A' && c <= 'Z') ? static_cast<Char>(c - 'A' + 'a') : c;
}

Bool startsWith(const Char* s, const Str& p, Bool caseSensitive) noexcept
{
    for(Size i = 0; i < p.size(); ++i)
    {
        const Char a = s[i];
        if(a == '\0')
            return false;
        if(caseSensitive ? (a != p[i]) : (lower(a) != lower(p[i])))
            return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// The table. Kept in the order a person would want to read it, not sorted:
// suggest() sorts what it returns, and grouping by subsystem here is what makes
// this maintainable against pico2w.h.
// ---------------------------------------------------------------------------
const Item TABLE[] = {
    // ---- pico2w.h: GPIO ----
    { "gpioOpen",       "Void gpioOpen(Pin pin, PinDir dir)",
      "Claim a pin and set its direction. Both, in one call.", Kind::KIND_FUNCTION },
    { "gpioWrite",      "Void gpioWrite(Pin pin, Bool high)",
      "Drive an output pin high or low.", Kind::KIND_FUNCTION },
    { "gpioRead",       "Bool gpioRead(Pin pin)",
      "Read an input pin.", Kind::KIND_FUNCTION },
    { "gpioToggle",     "Void gpioToggle(Pin pin)",
      "Flip an output pin.", Kind::KIND_FUNCTION },
    { "gpioPull",       "Void gpioPull(Pin pin, PinPull pull)",
      "Enable the internal pull-up or pull-down.", Kind::KIND_FUNCTION },

    // ---- pico2w.h: time ----
    { "sleepMs",        "Void sleepMs(UInt32 ms)",
      "Block for milliseconds.", Kind::KIND_FUNCTION },
    { "sleepUs",        "Void sleepUs(UInt64 us)",
      "Block for microseconds.", Kind::KIND_FUNCTION },
    { "nowMs",          "UInt32 nowMs(Void)",
      "Milliseconds since boot. Wraps after ~49 days.", Kind::KIND_FUNCTION },
    { "nowUs",          "UInt64 nowUs(Void)",
      "Microseconds since boot.", Kind::KIND_FUNCTION },

    // ---- pico2w.h: serial ----
    { "serialOpen",     "Void serialOpen(Void)",
      "Bring up stdio over USB. Nothing prints before this.", Kind::KIND_FUNCTION },
    { "serialPrint",    "Void serialPrint(CharSeq text)",
      "Write text with no newline.", Kind::KIND_FUNCTION },
    { "serialPrintLine","Void serialPrintLine(CharSeq text)",
      "Write text followed by a newline.", Kind::KIND_FUNCTION },
    { "serialPrintf",   "serialPrintf(fmt, ...)",
      "printf to the USB console.", Kind::KIND_MACRO },

    // ---- pico2w.h: PWM, servo, ESC ----
    { "pwmOpen",        "Void pwmOpen(Pin pin, UInt32 freqHz)",
      "Configure a pin for PWM at a frequency.", Kind::KIND_FUNCTION },
    { "pwmWrite",       "Void pwmWrite(Pin pin, Float32 duty)",
      "Set duty cycle, 0.0 to 1.0. Clamped.", Kind::KIND_FUNCTION },
    { "servoOpen",      "Void servoOpen(Pin pin)",
      "Configure a pin for a servo or an ESC (50 Hz).", Kind::KIND_FUNCTION },
    { "servoWriteUs",   "Void servoWriteUs(Pin pin, UInt32 us)",
      "Hold a pulse width, 1000-2000 us. Clamped.", Kind::KIND_FUNCTION },
    { "servoCenter",    "Void servoCenter(Pin pin)",
      "1500 us: centred servo, neutral ESC.", Kind::KIND_FUNCTION },

    // ---- pico2w.h: ADC ----
    { "adcOpen",        "Void adcOpen(Pin pin)",
      "Enable the ADC on GP26-GP29.", Kind::KIND_FUNCTION },
    { "adcRead",        "UInt16 adcRead(Pin pin)",
      "Raw 12-bit reading, 0-4095.", Kind::KIND_FUNCTION },
    { "adcReadVolts",   "Float32 adcReadVolts(Pin pin)",
      "Reading scaled to volts against the 3.3 V reference.", Kind::KIND_FUNCTION },

    // ---- pico2w.h: types ----
    { "Pin",            "typedef Int32 Pin",
      "A GPIO number - GP28, not physical pin 34.", Kind::KIND_TYPE },
    { "PinDir",         "enum PinDir",
      "PIN_DIR_IN or PIN_DIR_OUT.", Kind::KIND_TYPE },
    { "PinPull",        "enum PinPull",
      "PIN_PULL_NONE, PIN_PULL_UP or PIN_PULL_DOWN.", Kind::KIND_TYPE },

    { "PIN_DIR_IN",     "PinDir", "", Kind::KIND_MACRO },
    { "PIN_DIR_OUT",    "PinDir", "", Kind::KIND_MACRO },
    { "PIN_PULL_NONE",  "PinPull", "", Kind::KIND_MACRO },
    { "PIN_PULL_UP",    "PinPull", "", Kind::KIND_MACRO },
    { "PIN_PULL_DOWN",  "PinPull", "", Kind::KIND_MACRO },

    { "SERVO_MIN_US",   "1000", "Full one way.", Kind::KIND_MACRO },
    { "SERVO_MID_US",   "1500", "Centre / neutral.", Kind::KIND_MACRO },
    { "SERVO_MAX_US",   "2000", "Full the other way.", Kind::KIND_MACRO },
    { "SERVO_HZ",       "50",   "Servo and ESC frame rate.", Kind::KIND_MACRO },
    { "SERVO_PERIOD_US","20000","One frame at 50 Hz.", Kind::KIND_MACRO },
    { "PWM_WRAP",       "65535","16-bit PWM counter top.", Kind::KIND_MACRO },

    // ---- shared.h ----
    { "Int8",    "int8_t",   "", Kind::KIND_TYPE },
    { "Int16",   "int16_t",  "", Kind::KIND_TYPE },
    { "Int32",   "int32_t",  "", Kind::KIND_TYPE },
    { "Int64",   "int64_t",  "", Kind::KIND_TYPE },
    { "UInt8",   "uint8_t",  "", Kind::KIND_TYPE },
    { "UInt16",  "uint16_t", "", Kind::KIND_TYPE },
    { "UInt32",  "uint32_t", "", Kind::KIND_TYPE },
    { "UInt64",  "uint64_t", "", Kind::KIND_TYPE },
    { "Float32", "float",    "", Kind::KIND_TYPE },
    { "Float64", "double",   "", Kind::KIND_TYPE },
    { "Bool",    "bool",     "", Kind::KIND_TYPE },
    { "Void",    "void",     "", Kind::KIND_TYPE },
    { "Utf8",    "char",     "", Kind::KIND_TYPE },
    { "Size",    "size_t",   "", Kind::KIND_TYPE },
    { "UPtr",    "uintptr_t","", Kind::KIND_TYPE },
    { "CFile",   "FILE",     "", Kind::KIND_TYPE },
    { "Any",     "void*",    "", Kind::KIND_TYPE },
    { "CharSeq", "const Utf8*", "A borrowed, NUL-terminated string.", Kind::KIND_TYPE },

    // ---- the language ----
    { "break",    "", "", Kind::KIND_KEYWORD },
    { "case",     "", "", Kind::KIND_KEYWORD },
    { "const",    "", "", Kind::KIND_KEYWORD },
    { "continue", "", "", Kind::KIND_KEYWORD },
    { "default",  "", "", Kind::KIND_KEYWORD },
    { "do",       "", "", Kind::KIND_KEYWORD },
    { "else",     "", "", Kind::KIND_KEYWORD },
    { "enum",     "", "", Kind::KIND_KEYWORD },
    { "extern",   "", "", Kind::KIND_KEYWORD },
    { "false",    "", "", Kind::KIND_KEYWORD },
    { "for",      "", "", Kind::KIND_KEYWORD },
    { "if",       "", "", Kind::KIND_KEYWORD },
    { "inline",   "", "", Kind::KIND_KEYWORD },
    { "return",   "", "", Kind::KIND_KEYWORD },
    { "sizeof",   "", "", Kind::KIND_KEYWORD },
    { "static",   "", "", Kind::KIND_KEYWORD },
    { "struct",   "", "", Kind::KIND_KEYWORD },
    { "switch",   "", "", Kind::KIND_KEYWORD },
    { "true",     "", "", Kind::KIND_KEYWORD },
    { "typedef",  "", "", Kind::KIND_KEYWORD },
    { "while",    "", "", Kind::KIND_KEYWORD },
};

Vec<Item> buildAll()
{
    return Vec<Item>(TABLE, TABLE + sizeof(TABLE) / sizeof(TABLE[0]));
}

} // namespace

const Vec<Item>& all()
{
    static const Vec<Item> v = buildAll();
    return v;
}

Str wordAtEnd(const Str& line)
{
    Size end = line.size();
    Size i   = end;
    while(i > 0 && identChar(line[i - 1]))
        --i;

    // A run that starts with a digit is a number, not an identifier being
    // typed - completing `42` against the table would be nonsense.
    if(i < end && line[i] >= '0' && line[i] <= '9')
        return Str();

    return line.substr(i, end - i);
}

Size suggest(const Str& prefix, Vec<const Item*>& out, Size max)
{
    if(prefix.empty() || max == 0)
        return 0;

    const Vec<Item>& items = all();

    Vec<const Item*> hits;
    for(const Item& it : items)
        if(startsWith(it.name, prefix, false))
            hits.push_back(&it);

    std::sort(hits.begin(), hits.end(), [&prefix](const Item* a, const Item* b)
    {
        // Case-exact prefix first: typing `Int` should offer Int32 before it
        // offers anything that merely matches when folded.
        const Bool ea = startsWith(a->name, prefix, true);
        const Bool eb = startsWith(b->name, prefix, true);
        if(ea != eb)
            return ea;

        const Size la = std::strlen(a->name);
        const Size lb = std::strlen(b->name);
        if(la != lb)
            return la < lb;

        return std::strcmp(a->name, b->name) < 0;
    });

    const Size n = std::min(max, hits.size());
    for(Size i = 0; i < n; ++i)
        out.push_back(hits[i]);
    return n;
}

} // namespace cmpl
