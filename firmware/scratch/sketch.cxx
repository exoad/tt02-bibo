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

#include "../lib/tt02.hxx"

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

/*
 * `int`, not Int32, and this is the one place in the program where that is
 * right.
 *
 * C++ requires main to return literally `int`. Int32 is int32_t, and on this
 * toolchain int32_t is `long` - the same size, the same representation, a
 * different type as far as the language is concerned, and the compiler refuses
 * it. main's signature is the C runtime's contract, not this project's
 * vocabulary, so it is spelled the runtime's way.
 */
int main(Void)
{
    serial::open();

    tft::Screen screen;
    gfx::open(&screen, SCREEN_W, SCREEN_H, SCREEN_XOFF, SCREEN_YOFF);
    gfx::safeInset(&screen, SAFE_INSET);

    const Int32 left  = gfx::safeLeft(&screen);
    const Int32 right = gfx::safeRight(&screen);
    const Int32 wide  = gfx::safeWidth(&screen);

    const Bool haveBus = i2c::open(PIN_SDA, PIN_SCL, I2C_HZ);

    tof::Vl53 tof;
    const Bool haveTof = haveBus && tof::open(&tof, PIN_SDA, VL53_ADDR_DEFAULT);

    if(haveTof)
    {
        tof::startRanging(&tof);
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
        if(haveTof && tof::ready(&tof))
        {
            mm     = tof::distance(&tof);
            status = tof::status(&tof);
            tof::clear(&tof);
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
                serial::printf("range %u mm status %u\n", mm, status);
            }
        }

        gfx::clear(&screen, GFX_NAVY);

        Int32 y = gfx::safeTop(&screen);

        gfx::textTransparent(&screen);
        gfx::textColour(&screen, GFX_ORANGE);
        gfx::textSize(&screen, 2);
        gfx::textAt(&screen, left, y, "RANGE");
        y += 26;

        if(!haveBus)
        {
            gfx::textSize(&screen, 1);
            gfx::textColour(&screen, GFX_RED);
            gfx::textAt(&screen, left, y, "I2C PINS NOT A PAIR");
            gfx::present(&screen);
            timing::ms(1000);
            continue;
        }

        if(!haveTof)
        {
            gfx::textSize(&screen, 1);
            gfx::textColour(&screen, GFX_RED);
            gfx::textAt(&screen, left, y, "NO VL53L1X AT 0X29");
            y += 20;
            gfx::textColour(&screen, GFX_GREY);
            gfx::textAt(&screen, left, y, "SDA GP4  SCL GP5");
            y += 14;
            gfx::textAt(&screen, left, y, "VIN TO 3V3, GND TO GND");
            y += 14;
            gfx::textAt(&screen, left, y, "RUN THE I2C SCAN FIRST");
            gfx::present(&screen);
            timing::ms(1000);
            continue;
        }

        /* ---- the number -------------------------------------------------- */
        const Bool good = (status == 0);

        gfx::textColour(&screen, good ? GFX_WHITE : GFX_DARKGREY);
        gfx::textSize(&screen, 4);
        gfx::cursor(&screen, left, y);
        if(good)
        {
            gfx::printf(&screen, "%u", mm);
        }
        else
        {
            gfx::print(&screen, "----");
        }
        y += 36;

        gfx::textSize(&screen, 1);
        gfx::textColour(&screen, GFX_GREY);
        gfx::textAt(&screen, left, y, "MM");
        y += 18;

        gfx::textSize(&screen, 2);
        gfx::textColour(&screen, good ? GFX_CYAN : GFX_DARKGREY);
        gfx::cursor(&screen, left, y);
        if(good)
        {
            /* One decimal, done in integers - a float here would pull in the
             * whole soft-float formatting path for one number. */
            gfx::printf(&screen, "%u.%02u M", mm / 1000u, (mm % 1000u) / 10u);
        }
        else
        {
            gfx::print(&screen, "--.-- M");
        }
        y += 30;

        /* ---- the bar ----------------------------------------------------- */
        const Int32 barH = 16;
        gfx::rect(&screen, left, y, wide, barH, GFX_DARKGREY);

        if(good)
        {
            Int32 fill = static_cast<Int32>((static_cast<UInt32>(mm) * static_cast<UInt32>(wide - 2))
                                  / static_cast<UInt32>(BAR_FULL_MM));
            if(fill > wide - 2)
            {
                fill = wide - 2;
            }

            /* Green far, amber near, red very near - the colours a bumper
             * wants, so the same view is useful once this is on the car. */
            const UInt16 c = (mm < 150) ? GFX_RED
                           : (mm < 400) ? GFX_ORANGE
                           : GFX_GREEN;
            gfx::rectFill(&screen, left + 1, y + 1, fill, barH - 2, c);
        }
        y += barH + 6;

        gfx::textSize(&screen, 1);
        gfx::textColour(&screen, GFX_DARKGREY);
        gfx::textAt(&screen, left, y, "0");
        gfx::textAligned(&screen, right, y, "2 M", gfx::ALIGN_RIGHT);
        y += 22;

        /* ---- status ------------------------------------------------------ */
        gfx::textColour(&screen, good ? GFX_GREEN : GFX_YELLOW);
        gfx::textAt(&screen, left, y, tof::statusName(status));
        y += 20;

        /* ---- what it has managed so far ---------------------------------- */
        gfx::textColour(&screen, GFX_GREY);
        gfx::cursor(&screen, left, y);
        if(seenMax > 0)
        {
            gfx::printf(&screen, "SEEN %u - %u MM", seenMin, seenMax);
        }
        else
        {
            gfx::print(&screen, "SEEN NOTHING YET");
        }
        y += 14;

        gfx::cursor(&screen, left, y);
        gfx::printf(&screen, "%u READS", reads);

        gfx::present(&screen);

        /* ~20 fps. The sensor's own measurement takes longer than this, so
         * polling faster would only burn power to be told "not yet". */
        timing::ms(50);
    }

    return 0;
}
