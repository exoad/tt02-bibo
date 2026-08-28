/*
 * A live range view for the VL53L1X, on the LCD.
 *
 * ---------------------------------------------------------------------------
 * WIRING
 *
 *   Sensor   Pico            Display   Pico
 *   VIN      3V3             VCC/BLK   3V3
 *   GND      GND             GND       GND
 *   SDA      GP4  (I2C0)     SCL/SCK   GP18 (SPI0)
 *   SCL      GP5  (I2C0)     SDA/MOSI  GP19
 *   XSHUT    -               RES/DC/CS GP20/21/17
 *   INT      -
 *
 * ---------------------------------------------------------------------------
 * WHAT THE VIEW SHOWS, AND WHY EACH PART IS THERE
 *
 *   the number      millimetres, and metres beneath it. Millimetres because
 *                   that is what the sensor reports and rounding at the source
 *                   loses information; metres because that is what a person
 *                   thinks in.
 *
 *   the bar         the same reading as a proportion of full scale. A number
 *                   alone is hard to watch - moving your hand and seeing a bar
 *                   follow tells you instantly whether the sensor is tracking,
 *                   which a digit flickering between 812 and 809 does not.
 *
 *   the status      0 is the only good one. A distance that came with a bad
 *                   status is not a shorter distance, it is not a distance at
 *                   all, so it is shown greyed rather than as a number to
 *                   believe.
 *
 *   min and max     the range seen since power-on. Sweep the sensor across a
 *                   room and these tell you what it actually managed, which is
 *                   the honest answer to "how far does it reach" for THIS
 *                   sensor in THIS light.
 *
 * The mode matters more than it looks. SHORT reaches about 1.3 m and rejects
 * ambient infrared well; LONG reaches about 4 m indoors and is easily blinded
 * by daylight. Start on LONG and switch if readings collapse near a window.
 */

#include "../lib/tt02.h"

/* ---- the screen ---------------------------------------------------------- */
#define SCREEN_W     240
#define SCREEN_H     280
#define SCREEN_XOFF  0
#define SCREEN_YOFF  20
#define SAFE_INSET   14

/* ---- the bus ------------------------------------------------------------- */
#define PIN_SDA      4
#define PIN_SCL      5
#define I2C_HZ       400000u     /* the sensor is happy at 400 kHz */

/* Full scale for the bar, in millimetres. 2 m is a useful indoor span: far
 * enough to be interesting, near enough that the bar moves when you do. */
#define BAR_FULL_MM  2000

Int32 main(Void)
{
    serialOpen();

    Screen screen;
    gfxOpen(&screen, SCREEN_W, SCREEN_H, SCREEN_XOFF, SCREEN_YOFF);
    gfxSafeInset(&screen, SAFE_INSET);

    const Int32 left  = gfxSafeLeft(&screen);
    const Int32 right = gfxSafeRight(&screen);
    const Int32 wide  = gfxSafeWidth(&screen);

    const Bool haveBus = i2cOpen(PIN_SDA, PIN_SCL, I2C_HZ);

    Vl53 tof;
    const Bool haveTof = haveBus && vl53Open(&tof, PIN_SDA, VL53_ADDR_DEFAULT);

    if(haveTof)
    {
        vl53StartRanging(&tof);
    }

    UInt16 mm     = 0;
    UInt8  status = 255;
    UInt32 reads  = 0;

    /* Seeded so the FIRST good reading replaces them, rather than starting at
     * zero and reporting a minimum of 0 mm forever. */
    UInt16 seenMin = 0xFFFF;
    UInt16 seenMax = 0;

    while(true)
    {
        if(haveTof && vl53Ready(&tof))
        {
            mm     = vl53Distance(&tof);
            status = vl53Status(&tof);
            vl53Clear(&tof);
            ++reads;

            if(status == 0)
            {
                if(mm < seenMin)
                {
                    seenMin = mm;
                }
                if(mm > seenMax)
                {
                    seenMax = mm;
                }
            }

            if((reads % 20u) == 0u)
            {
                serialPrintf("range %u mm status %u\n", mm, status);
            }
        }

        gfxClear(&screen, GFX_NAVY);

        Int32 y = gfxSafeTop(&screen);

        gfxTextTransparent(&screen);
        gfxTextColour(&screen, GFX_ORANGE);
        gfxTextSize(&screen, 2);
        gfxTextAt(&screen, left, y, "RANGE");
        y += 26;

        if(!haveBus)
        {
            gfxTextSize(&screen, 1);
            gfxTextColour(&screen, GFX_RED);
            gfxTextAt(&screen, left, y, "I2C PINS NOT A PAIR");
            gfxPresent(&screen);
            sleepMs(1000);
            continue;
        }

        if(!haveTof)
        {
            gfxTextSize(&screen, 1);
            gfxTextColour(&screen, GFX_RED);
            gfxTextAt(&screen, left, y, "NO VL53L1X AT 0X29");
            y += 20;
            gfxTextColour(&screen, GFX_GREY);
            gfxTextAt(&screen, left, y, "SDA GP4  SCL GP5");
            y += 14;
            gfxTextAt(&screen, left, y, "VIN TO 3V3, GND TO GND");
            y += 14;
            gfxTextAt(&screen, left, y, "RUN THE I2C SCAN FIRST");
            gfxPresent(&screen);
            sleepMs(1000);
            continue;
        }

        /* ---- the number -------------------------------------------------- */
        const Bool good = (status == 0);

        gfxTextColour(&screen, good ? GFX_WHITE : GFX_DARKGREY);
        gfxTextSize(&screen, 4);
        gfxCursor(&screen, left, y);
        if(good)
        {
            gfxPrintf(&screen, "%u", mm);
        }
        else
        {
            gfxPrint(&screen, "----");
        }
        y += 36;

        gfxTextSize(&screen, 1);
        gfxTextColour(&screen, GFX_GREY);
        gfxTextAt(&screen, left, y, "MM");
        y += 18;

        gfxTextSize(&screen, 2);
        gfxTextColour(&screen, good ? GFX_CYAN : GFX_DARKGREY);
        gfxCursor(&screen, left, y);
        if(good)
        {
            /* One decimal, done in integers - a float here would pull in the
             * whole soft-float formatting path for one number. */
            gfxPrintf(&screen, "%u.%02u M", mm / 1000u, (mm % 1000u) / 10u);
        }
        else
        {
            gfxPrint(&screen, "--.-- M");
        }
        y += 30;

        /* ---- the bar ----------------------------------------------------- */
        const Int32 barH = 16;
        gfxRect(&screen, left, y, wide, barH, GFX_DARKGREY);

        if(good)
        {
            Int32 fill = (Int32) (((UInt32) mm * (UInt32) (wide - 2))
                                  / (UInt32) BAR_FULL_MM);
            if(fill > wide - 2)
            {
                fill = wide - 2;
            }

            /* Green far, amber near, red very near - the colours a bumper
             * wants, so the same view is useful once this is on the car. */
            const UInt16 c = (mm < 150) ? GFX_RED
                           : (mm < 400) ? GFX_ORANGE
                           : GFX_GREEN;
            gfxRectFill(&screen, left + 1, y + 1, fill, barH - 2, c);
        }
        y += barH + 6;

        gfxTextSize(&screen, 1);
        gfxTextColour(&screen, GFX_DARKGREY);
        gfxTextAt(&screen, left, y, "0");
        gfxTextAligned(&screen, right, y, "2 M", GFX_ALIGN_RIGHT);
        y += 22;

        /* ---- status ------------------------------------------------------ */
        gfxTextColour(&screen, good ? GFX_GREEN : GFX_YELLOW);
        gfxTextAt(&screen, left, y, vl53StatusName(status));
        y += 20;

        /* ---- what it has managed so far ---------------------------------- */
        gfxTextColour(&screen, GFX_GREY);
        gfxCursor(&screen, left, y);
        if(seenMax > 0)
        {
            gfxPrintf(&screen, "SEEN %u - %u MM", seenMin, seenMax);
        }
        else
        {
            gfxPrint(&screen, "SEEN NOTHING YET");
        }
        y += 14;

        gfxCursor(&screen, left, y);
        gfxPrintf(&screen, "%u READS", reads);

        gfxPresent(&screen);

        /* ~20 fps. The sensor's own measurement takes longer than this, so
         * polling faster would only burn power to be told "not yet". */
        sleepMs(50);
    }

    return 0;
}
