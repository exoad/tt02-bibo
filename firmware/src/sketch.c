/*
 * Scan the I2C bus and show what answered, on the LCD.
 *
 * The first thing to run whenever anything is added to I2C, before a driver for
 * it exists. It needs no knowledge of the device: it walks every address, asks
 * "is anyone there", and lists the ones that say yes. That turns "is my wiring
 * right" into a yes or no, which is otherwise a question you cannot answer.
 *
 * ---------------------------------------------------------------------------
 * WIRING
 *
 *   Display          Pico            Sensor (VL53L1X)   Pico
 *   GND              GND             GND                GND
 *   VCC              3V3             VIN                3V3
 *   SCL/SCK          GP18            SDA                GP4
 *   SDA/MOSI         GP19            SCL                GP5
 *   RES              GP20            INT                not connected
 *   DC               GP21            XSHUT              not connected
 *   CS               GP17
 *   BLK              3V3
 *
 * XSHUT and INT are genuinely optional for ONE sensor. XSHUT holds a sensor in
 * reset so its neighbour can be re-addressed, which only matters once there are
 * several - they all ship at 0x29 and cannot share a bus until they have been
 * given different addresses. INT signals "a reading is ready"; polling works
 * without it.
 *
 * ---------------------------------------------------------------------------
 * THE SAFE AREA
 *
 * This panel has rounded corners cut into the glass. The controller addresses
 * the full 240x280 rectangle and accepts pixels for the corners quite happily -
 * you simply cannot see them, and anything near an edge disappears into the
 * curve. That is why the bottom line of the first version of this sketch was
 * missing: it was there, behind the bezel.
 *
 * So everything is laid out against gfxSafe*() rather than against 0 and
 * width/height. The background still fills the whole rectangle - it should, or
 * there would be a dark border - but everything that has to be READ sits
 * inside.
 *
 * SAFE_INSET is the number to tune. The outline below is drawn on purpose:
 * adjust until the frame is fully visible with a little to spare, then set
 * SHOW_SAFE_FRAME to 0.
 */

#include "pico2w.h"
#include "gfx.h"

/* ---- the screen ---------------------------------------------------------- */
#define SCREEN_W        240
#define SCREEN_H        280
#define SCREEN_XOFF     0
#define SCREEN_YOFF     20

/* How far in from each edge is actually visible past the rounded corners. */
#define SAFE_INSET      14
#define SHOW_SAFE_FRAME 1

/* ---- the bus ------------------------------------------------------------- */
#define PIN_SDA         4
#define PIN_SCL         5
#define I2C_HZ          100000u     /* 100 kHz - the slow, always-works speed */

/* Below 0x08 and above 0x77 are reserved by the I2C specification. Poking them
 * is how you confuse a bus rather than discover one. */
#define ADDR_FIRST      0x08
#define ADDR_LAST       0x77

Int32 main(Void)
{
    serialOpen();

    Screen screen;
    gfxOpen(&screen, SCREEN_W, SCREEN_H, SCREEN_XOFF, SCREEN_YOFF);
    gfxSafeInset(&screen, SAFE_INSET);

    const Bool haveBus = i2cOpen(PIN_SDA, PIN_SCL, I2C_HZ);

    const Int32 left = gfxSafeLeft(&screen);
    const Int32 top  = gfxSafeTop(&screen);

    while(true)
    {
        /* The BACKGROUND fills everything, corners included - a safe area is
         * about what can be read, not about where colour may go. Insetting the
         * fill too would draw a dark frame around the picture. */
        gfxClear(&screen, GFX_NAVY);

#if SHOW_SAFE_FRAME
        gfxSafeOutline(&screen, gfxDim(GFX_CYAN, 180));
#endif

        gfxTextTransparent(&screen);
        gfxTextColour(&screen, GFX_ORANGE);
        gfxTextSize(&screen, 2);
        gfxTextAt(&screen, left, top, "I2C SCAN");

        gfxTextColour(&screen, GFX_GREY);
        gfxTextSize(&screen, 1);
        gfxCursor(&screen, left, top + 22);
        gfxPrintf(&screen, "SDA GP%d   SCL GP%d", PIN_SDA, PIN_SCL);

        if(!haveBus)
        {
            gfxTextColour(&screen, GFX_RED);
            gfxTextAt(&screen, left, top + 42, "BAD PINS - NOT AN I2C PAIR");
            gfxPresent(&screen);
            sleepMs(2000);
            continue;
        }

        Int32 found = 0;
        Int32 y     = top + 46;

        for(Int32 a = ADDR_FIRST; a <= ADDR_LAST; ++a)
        {
            if(!i2cPresent(PIN_SDA, (UInt8) a))
            {
                continue;
            }

            ++found;

            /* Named rather than left as a number to go and look up. */
            const Utf8* who = (a == 0x29) ? "VL53L1X"
                            : (a == 0x3C || a == 0x3D) ? "OLED"
                            : (a == 0x68) ? "IMU"
                            : "";

            gfxTextColour(&screen, GFX_GREEN);
            gfxCursor(&screen, left, y);
            gfxPrintf(&screen, "0X%02X %s", a, who);

            serialPrintf("found 0x%02x %s\n", a, who);
            y += 14;
        }

        if(found == 0)
        {
            gfxTextColour(&screen, GFX_RED);
            gfxTextAt(&screen, left, top + 46, "NOTHING ON THE BUS");

            gfxTextColour(&screen, GFX_GREY);
            gfxTextAt(&screen, left, top + 66, "SDA/SCL SWAPPED?");
            gfxTextAt(&screen, left, top + 80, "VIN AND GND?");
            gfxTextAt(&screen, left, top + 94, "PULL-UPS?");
        }

        /* Bottom-left of the SAFE area, not of the panel - which is where the
         * previous version put it, and why it was never seen. */
        gfxTextColour(&screen, GFX_CYAN);
        gfxCursor(&screen, left, gfxSafeBottom(&screen) - 8);
        gfxPrintf(&screen, "%d DEVICE%s", found, (found == 1) ? "" : "S");

        gfxPresent(&screen);

        serialPrintf("scan: %d device(s)\n", found);
        sleepMs(1000);
    }

    return 0;
}
