#include "complete.hxx"

#include <algorithm>
#include <cstring>

namespace cmpl
{
  namespace
  {

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
            {
                return false;
            }
            if(caseSensitive ? (a != p[i]) : (lower(a) != lower(p[i])))
            {
                return false;
            }
        }
        return true;
    }

    // ---------------------------------------------------------------------------
    // The table. Kept in the order a person would want to read it, not sorted:
    // suggest() sorts what it returns, and grouping by subsystem here is what makes
    // this maintainable against pico2w.h.
    // ---------------------------------------------------------------------------
    constexpr Item TABLE[] = {
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
        { "serialWaitForHost", "Bool serialWaitForHost(UInt32 timeoutMs)",
          "Wait for the host to open the port. 0 waits forever.", Kind::KIND_FUNCTION },

        // ---- pico2w.h: the onboard LED ----
        { "ledOpen",        "Bool ledOpen(Void)",
          "Bring up the CYW43439. Required before any led* call; can fail.",
          Kind::KIND_FUNCTION },
        { "ledWrite",       "Void ledWrite(Bool on)",
          "The onboard LED. Not a GPIO - it is on the wireless chip.",
          Kind::KIND_FUNCTION },
        { "ledRead",        "Bool ledRead(Void)",
          "Current state of the onboard LED.", Kind::KIND_FUNCTION },
        { "ledToggle",      "Void ledToggle(Void)",
          "Flip the onboard LED.", Kind::KIND_FUNCTION },

        // ---- pico2w.h: SPI ----
        { "spiOpen",        "Bool spiOpen(Pin sck, Pin mosi, Pin cs, UInt32 hz)",
          "Bring up an SPI bus. cs may be -1 to drive it yourself. "
          "False if the pins are not one controller.", Kind::KIND_FUNCTION },
        { "spiWrite",       "Size spiWrite(Pin sck, const UInt8* data, Size n)",
          "Blocking write. Returns bytes sent.", Kind::KIND_FUNCTION },
        { "spiWriteByte",   "Size spiWriteByte(Pin sck, UInt8 b)",
          "One byte, blocking.", Kind::KIND_FUNCTION },
        { "spiTransfer",    "Size spiTransfer(Pin sck, const UInt8* tx, UInt8* rx, Size n)",
          "Full duplex: send and capture the same count.", Kind::KIND_FUNCTION },
        { "spiBaud",        "UInt32 spiBaud(Pin sck, UInt32 hz)",
          "What the hardware actually settled on - the divider is an integer.",
          Kind::KIND_FUNCTION },
        { "spiForSck",      "spi_inst_t* spiForSck(Pin sck)",
          "Which controller a SCK pin belongs to, or NULL.", Kind::KIND_FUNCTION },

        // ---- st77xx.h: the SPI color display ----
        { "tftInit",        "Bool tftInit(Void)",
          "Reset and configure the panel. False only if the SPI pins are wrong.",
          Kind::KIND_FUNCTION },
        { "tftFill",        "Void tftFill(UInt16 color)",
          "Whole screen to one color.", Kind::KIND_FUNCTION },
        { "tftRect",        "Void tftRect(Int32 x, Int32 y, Int32 w, Int32 h, UInt16 color)",
          "Filled rectangle, clipped to the panel.", Kind::KIND_FUNCTION },
        { "tftPixel",       "Void tftPixel(Int32 x, Int32 y, UInt16 color)",
          "One pixel.", Kind::KIND_FUNCTION },
        { "tftText",        "Void tftText(Int32 x, Int32 y, const Utf8* s, UInt16 fg, UInt16 bg, Int32 scale)",
          "5x7 text. Lowercase folds to uppercase.", Kind::KIND_FUNCTION },
        { "tftChar",        "Void tftChar(Int32 x, Int32 y, Utf8 ch, UInt16 fg, UInt16 bg, Int32 scale)",
          "One character, streamed in a single transfer.", Kind::KIND_FUNCTION },
        { "TFT_RGB",        "TFT_RGB(r, g, b)",
          "Pack 8-8-8 into the panel's 5-6-5.", Kind::KIND_MACRO },
        { "PANEL_W",        "PANEL_W",
          "Panel width. Set by the panel choice at the top of st77xx.h.",
          Kind::KIND_MACRO },
        { "PANEL_H",        "PANEL_H",
          "Panel height.", Kind::KIND_MACRO },

        // ---- gfx.h: the 2D canvas over the panel ----
        { "gfxInit",        "Bool gfxInit(Void)",
          "Bring up the panel and clear it.", Kind::KIND_FUNCTION },
        { "gfxPresent",     "Void gfxPresent(Void)",
          "Push the back buffer. NOTHING is visible until this runs.",
          Kind::KIND_FUNCTION },
        { "gfxClear",       "Void gfxClear(UInt16 color)",
          "Whole canvas to one color.", Kind::KIND_FUNCTION },
        { "gfxPixel",       "Void gfxPixel(Int32 x, Int32 y, UInt16 color)",
          "One pixel, clipped.", Kind::KIND_FUNCTION },
        { "gfxLine",        "Void gfxLine(Int32 x0, Int32 y0, Int32 x1, Int32 y1, UInt16 c)",
          "Bresenham line.", Kind::KIND_FUNCTION },
        { "gfxHLine",       "Void gfxHLine(Int32 x, Int32 y, Int32 w, UInt16 c)",
          "Horizontal run - the fast path.", Kind::KIND_FUNCTION },
        { "gfxVLine",       "Void gfxVLine(Int32 x, Int32 y, Int32 h, UInt16 c)",
          "Vertical run.", Kind::KIND_FUNCTION },
        { "gfxRect",        "Void gfxRect(Int32 x, Int32 y, Int32 w, Int32 h, UInt16 c)",
          "Rectangle outline.", Kind::KIND_FUNCTION },
        { "gfxRectFill",    "Void gfxRectFill(Int32 x, Int32 y, Int32 w, Int32 h, UInt16 c)",
          "Filled rectangle.", Kind::KIND_FUNCTION },
        { "gfxRoundRect",   "Void gfxRoundRect(Int32 x, Int32 y, Int32 w, Int32 h, Int32 r, UInt16 c)",
          "Rounded outline.", Kind::KIND_FUNCTION },
        { "gfxRoundRectFill", "Void gfxRoundRectFill(Int32 x, Int32 y, Int32 w, Int32 h, Int32 r, UInt16 c)",
          "Filled rounded rectangle.", Kind::KIND_FUNCTION },
        { "gfxCircle",      "Void gfxCircle(Int32 cx, Int32 cy, Int32 r, UInt16 c)",
          "Circle outline, midpoint algorithm.", Kind::KIND_FUNCTION },
        { "gfxCircleFill",  "Void gfxCircleFill(Int32 cx, Int32 cy, Int32 r, UInt16 c)",
          "Filled circle, drawn as spans.", Kind::KIND_FUNCTION },
        { "gfxTriangle",    "Void gfxTriangle(Int32 x0, Int32 y0, Int32 x1, Int32 y1, Int32 x2, Int32 y2, UInt16 c)",
          "Triangle outline.", Kind::KIND_FUNCTION },
        { "gfxTriangleFill","Void gfxTriangleFill(Int32 x0, Int32 y0, Int32 x1, Int32 y1, Int32 x2, Int32 y2, UInt16 c)",
          "Filled triangle, scanline.", Kind::KIND_FUNCTION },
        { "gfxClip",        "Void gfxClip(Int32 x, Int32 y, Int32 w, Int32 h)",
          "Restrict drawing to a rectangle.", Kind::KIND_FUNCTION },
        { "gfxClipReset",   "Void gfxClipReset(Void)",
          "Back to the whole screen.", Kind::KIND_FUNCTION },
        { "gfxTextAt",      "Void gfxTextAt(Int32 x, Int32 y, const Utf8* s)",
          "Text at an absolute position.", Kind::KIND_FUNCTION },
        { "gfxTextAligned", "Void gfxTextAligned(Int32 x, Int32 y, const Utf8* s, GfxAlign a)",
          "Left, centered or right about x - so a changing value stays put.",
          Kind::KIND_FUNCTION },
        { "gfxTextColor",  "Void gfxTextColor(UInt16 fg)",
          "Set the ink.", Kind::KIND_FUNCTION },
        { "gfxTextBackground", "Void gfxTextBackground(UInt16 bg)",
          "Opaque background - new text erases the old as it draws.",
          Kind::KIND_FUNCTION },
        { "gfxTextTransparent", "Void gfxTextTransparent(Void)",
          "Leave what is behind the glyph alone.", Kind::KIND_FUNCTION },
        { "gfxTextSize",    "Void gfxTextSize(Int32 scale)",
          "1 is 6x8, 2 is 12x16, up to 8.", Kind::KIND_FUNCTION },
        { "gfxTextWidth",   "Int32 gfxTextWidth(const Utf8* s)",
          "Width in pixels at the current size, for layout.", Kind::KIND_FUNCTION },
        { "gfxCursor",      "Void gfxCursor(Int32 x, Int32 y)",
          "Where gfxPrint starts.", Kind::KIND_FUNCTION },
        { "gfxPrint",       "Void gfxPrint(const Utf8* s)",
          "Draw at the cursor and advance it.", Kind::KIND_FUNCTION },
        { "gfxPrintf",      "Void gfxPrintf(const Utf8* fmt, ...)",
          "printf into the cursor.", Kind::KIND_FUNCTION },
        { "gfxBlend",       "UInt16 gfxBlend(UInt16 a, UInt16 b, UInt8 t)",
          "Mix two colors; t is 0 for all a, 255 for all b.", Kind::KIND_FUNCTION },
        { "gfxDim",         "UInt16 gfxDim(UInt16 c, UInt8 amount)",
          "Toward black.", Kind::KIND_FUNCTION },
        { "gfxLighten",     "UInt16 gfxLighten(UInt16 c, UInt8 amount)",
          "Toward white.", Kind::KIND_FUNCTION },
        { "gfxHsv",         "UInt16 gfxHsv(Int32 hue, UInt8 sat, UInt8 val)",
          "Hue 0-359, sat and val 0-255. Integer throughout.",
          Kind::KIND_FUNCTION },
        { "gfxPeek",        "UInt16 gfxPeek(Int32 x, Int32 y)",
          "Read a pixel back. Needs the back buffer; the panel cannot be read.",
          Kind::KIND_FUNCTION },
        { "gfxPixelBlend",  "Void gfxPixelBlend(Int32 x, Int32 y, UInt16 c, UInt8 alpha)",
          "Alpha over what is already there.", Kind::KIND_FUNCTION },

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
          "1500 us: centered servo, neutral ESC.", Kind::KIND_FUNCTION },

        // ---- pico2w.h: ADC ----
        { "adcOpen",        "Void adcOpen(Pin pin)",
          "Enable the ADC on GP26-GP29.", Kind::KIND_FUNCTION },
        { "adcRead",        "UInt16 adcRead(Pin pin)",
          "Raw 12-bit reading, 0-4095.", Kind::KIND_FUNCTION },
        { "adcReadVolts",   "Float32 adcReadVolts(Pin pin)",
          "Reading scaled to volts against the 3.3 V reference.", Kind::KIND_FUNCTION },
        { "tempC",          "Float32 tempC(Void)",
          "Die temperature. Reads the chip, not the room.", Kind::KIND_FUNCTION },

        // ---- pico2w.h: watchdog and reboot ----
        { "watchdogStart",  "Void watchdogStart(UInt32 ms)",
          "Reset the board if watchdogFeed() is not called within ms.",
          Kind::KIND_FUNCTION },
        { "watchdogFeed",   "Void watchdogFeed(Void)",
          "Tell the watchdog the program is still alive.", Kind::KIND_FUNCTION },
        { "watchdogCausedReboot", "Bool watchdogCausedReboot(Void)",
          "True if THIS boot was a watchdog reset. Print it at startup.",
          Kind::KIND_FUNCTION },
        { "rebootToBootsel","Void rebootToBootsel(Void)",
          "Drop into the UF2 bootloader without the BOOTSEL button.",
          Kind::KIND_FUNCTION },

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
        { "SERVO_MID_US",   "1500", "Center / neutral.", Kind::KIND_MACRO },
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

  }

  const Vec<Item>& all()
  {
      static const Vec<Item> v = buildAll();
      return v;
  }

  Str wordAtEnd(const Str& line)
  {
      Size end = line.size();
      Size i = end;
      while(i > 0 && identChar(line[i - 1]))
      {
          --i;
      }

      // A run that starts with a digit is a number, not an identifier being
      // typed - completing `42` against the table would be nonsense.
      if(i < end && line[i] >= '0' && line[i] <= '9')
      {
          return Str();
      }

      return line.substr(i, end - i);
  }

  Size suggest(const Str& prefix, Vec<const Item*>& out, Size max)
  {
      if(prefix.empty() || max == 0)
      {
          return 0;
      }

      const Vec<Item>& items = all();

      Vec<const Item*> hits;
      for(const Item& it : items)
      {
          if(startsWith(it.name, prefix, false))
          {
              hits.push_back(&it);
          }
      }

      std::sort(hits.begin(), hits.end(), [&prefix](const Item* a, const Item* b)
      {
          // Case-exact prefix first: typing `Int` should offer Int32 before it
          // offers anything that merely matches when folded.
          const Bool ea = startsWith(a->name, prefix, true);
          const Bool eb = startsWith(b->name, prefix, true);
          if(ea != eb)
          {
              return ea;
          }

          const Size la = std::strlen(a->name);
          const Size lb = std::strlen(b->name);
          if(la != lb)
          {
              return la < lb;
          }

          return std::strcmp(a->name, b->name) < 0;
      });

      const Size n = std::min(max, hits.size());
      for(Size i = 0; i < n; ++i)
      {
          out.push_back(hits[i]);
      }
      return n;
  }

}
