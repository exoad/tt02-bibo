/*
 * Scan the I2C bus and show what answered, on the LCD.
 *
 * This is the first thing to run whenever anything is added to I2C, before any
 * driver for it exists. It needs no knowledge of the device: it walks every
 * address, asks "is anyone there", and lists the ones that say yes. That turns
 * "is my wiring right" into a yes or no, which is the question you otherwise
 * cannot answer.
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
 * given different addresses. INT is an interrupt line for "a reading is ready",
 * and polling works without it.
 *
 * ---------------------------------------------------------------------------
 * WHAT YOU SHOULD SEE
 *
 *   0x29                    the VL53L1X, at its factory address. Wiring good.
 *   nothing found           SDA and SCL swapped, or the sensor has no power,
 *                           or no pull-ups on the bus
 *   an address you expect   whatever else is attached
 *
 * 0x29 is the VL53L1X's default and it is worth recognising on sight.
 */

#include "pico2w.h"
#include "gfx.h"

/* ---- the screen ---------------------------------------------------------- */
#define SCREEN_W     240
#define SCREEN_H     280
#define SCREEN_XOFF  0
#define SCREEN_YOFF  20

/* ---- the bus ------------------------------------------------------------- */
#define PIN_SDA      4
#define PIN_SCL      5
#define I2C_HZ       100000u        /* 100 kHz - the slow, always-works speed */

/* The 7-bit addresses that are legal to probe. Below 0x08 and above 0x77 are
 * reserved by the I2C specification, and poking them is how you confuse a bus
 * rather than discover one. */
#define ADDR_FIRST   0x08
#define ADDR_LAST    0x77

Int32 main(Void)
{
    serialOpen();

    Screen screen;
    gfxOpen(&screen, SCREEN_W, SCREEN_H, SCREEN_XOFF, SCREEN_YOFF);

    const Bool haveBus = i2cOpen(PIN_SDA, PIN_SCL, I2C_HZ);

    while(true)
    {
        gfxClear(&screen, GFX_NAVY);

        gfxTextTransparent(&screen);
        gfxTextColour(&screen, GFX_ORANGE);
        gfxTextSize(&screen, 2);
        gfxTextAt(&screen, 8, 8, "I2C SCAN");

        gfxTextColour(&screen, GFX_GREY);
        gfxTextSize(&screen, 1);
        gfxCursor(&screen, 8, 30);
        gfxPrintf(&screen, "SDA GP%d   SCL GP%d", PIN_SDA, PIN_SCL);

        if(!haveBus)
        {
            gfxTextColour(&screen, GFX_RED);
            gfxTextAt(&screen, 8, 50, "BAD PINS - NOT AN I2C PAIR");
            gfxPresent(&screen);
            sleepMs(2000);
            continue;
        }

        Int32 found = 0;
        Int32 y     = 56;

        for(Int32 a = ADDR_FIRST; a <= ADDR_LAST; ++a)
        {
            if(!i2cPresent(PIN_SDA, (UInt8) a))
            {
                continue;
            }

            ++found;

            /* 0x29 is the VL53L1X's factory address, so it is named rather than
             * left as a number to look up. */
            const Utf8* who = (a == 0x29) ? "VL53L1X"
                            : (a == 0x3C || a == 0x3D) ? "OLED"
                            : (a == 0x68) ? "IMU"
                            : "";

            gfxTextColour(&screen, GFX_GREEN);
            gfxCursor(&screen, 8, y);
            gfxPrintf(&screen, "0X%02X %s", a, who);

            serialPrintf("found 0x%02x %s\n", a, who);
            y += 14;
        }

        if(found == 0)
        {
            gfxTextColour(&screen, GFX_RED);
            gfxTextAt(&screen, 8, 56, "NOTHING ON THE BUS");

            gfxTextColour(&screen, GFX_GREY);
            gfxTextAt(&screen, 8, 76, "CHECK SDA/SCL NOT SWAPPED");
            gfxTextAt(&screen, 8, 90, "CHECK VIN AND GND");
            gfxTextAt(&screen, 8, 104, "CHECK PULL-UPS");
        }

        gfxTextColour(&screen, GFX_CYAN);
        gfxCursor(&screen, 8, SCREEN_H - 16);
        gfxPrintf(&screen, "%d DEVICE%s", found, (found == 1) ? "" : "S");

        gfxPresent(&screen);

        serialPrintf("scan: %d device(s)\n", found);
        sleepMs(1000);
    }

    return 0;
}
