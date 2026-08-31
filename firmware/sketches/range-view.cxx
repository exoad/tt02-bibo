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

#include "../lib/bibo.hxx"

/*
 * The whole library lives in namespace bibo, and this line opens it so a sketch
 * can write gpio::write rather than bibo::gpio::write on every call.
 *
 * A `using` at file scope is a thing to be careful with in a big program, and
 * this is not one: a sketch is one file, it links nothing else, and the names it
 * pulls in are the ones it exists to use. main.cxx does NOT do this - the
 * program that steers the car spells everything out.
 */
using namespace bibo;

/* ---- the screen ---------------------------------------------------------- */
#define SCREEN_W     240
#define SCREEN_H     280
#define SCREEN_XOFF  0
#define SCREEN_YOFF  20
#define SAFE_INSET   14

/* ---- the bus -------------------------------------------------------------
 *
 * The PADS are not here any more - they come from the map this program installs
 * in main(). Only the speed is a property of this sketch. */
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

    /* ===================================================================== 1
     * DECLARE - what is wired where.
     *
     * Before anything is opened, because every driver below reads the
     * installed map rather than holding pin numbers. The map starts empty, so a
     * program that skips this opens a display on nothing and looks broken in a
     * way that has no obvious cause.
     *
     * This sketch borrows the car's pads for the bus and the panel, so it
     * starts from car() and changes nothing. A sketch that wanted the panel
     * somewhere else would set five fields here and no driver would notice.
     * ================================================================== */
    /* serial, the map, and a blink if the map is refused. Stopping beats
     * returning: a Pico that returns from main is powered, silent and
     * indistinguishable from one that never flashed. */
    if(!boot::begin(pins::car()))
    {
        boot::halt();
    }

    /* ===================================================================== 2
     * BIND - hand the low-level parts to the high-level ones.
     *
     * tft::Screen is the panel. gfx does not own one and never opens one; it
     * DRAWS ONTO a Canvas, which is handed the panel once at open() and owns the
     * back buffer, the clip and the text state from then on. Same for the
     * sensor: tof::Vl53 is the device, i2c is the bus it
     * sits on, and the bus is opened before the device that needs it.
     * ================================================================== */
    tft::Screen panel;   /* the hardware: size, pads, the glass  */

    /* The canvas comes FROM open(); it is not declared and then filled in. */
    gfx::Canvas c = gfx::open(&panel, { SCREEN_W, SCREEN_H, SCREEN_XOFF, SCREEN_YOFF });
    c.safeInset(SAFE_INSET);

    const gfx::Box safe = c.safe();
    const Int32 left  = safe.x;
    const Int32 right = safe.x + safe.w;
    const Int32 wide  = safe.w;

    /* The four appearances this screen uses, named once. */
    const gfx::Paint TITLE { .fg = GFX_ORANGE,   .size = 2 };
    const gfx::Paint ALERT { .fg = GFX_RED      };
    const gfx::Paint MUTED { .fg = GFX_GREY     };
    const gfx::Paint FAINT { .fg = GFX_DARKGREY };

    const Bool haveBus = i2c::open(pins::active().i2cSda,
                                   pins::active().i2cScl, I2C_HZ);

    tof::Vl53 tof;
    /* The start is PART of the answer, not something done to it afterwards:
     * a sensor that opened but never started answers "not ready" forever
     * while haveTof still says yes. Short-circuiting is right here - there
     * is nothing to start on a sensor that did not open. */
    const Bool haveTof = haveBus
                      && tof::open(&tof, VL53_ADDR_DEFAULT)
                      && tof::startRanging(&tof);

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

        Int32 y = safe.y;

        c.clear(GFX_NAVY)
         .text({ left, y }, "RANGE", TITLE);
        y += 26;

        if(!haveBus)
        {
            c.text({ left, y }, "I2C PINS NOT A PAIR", ALERT)
             .present();
            timing::ms(1000);
            continue;
        }

        if(!haveTof)
        {
            c.text({ left, y },      "NO VL53L1X AT 0X29",     ALERT)
             .text({ left, y + 20 }, "SDA GP4  SCL GP5",       MUTED)
             .text({ left, y + 34 }, "VIN TO 3V3, GND TO GND", MUTED)
             .text({ left, y + 48 }, "RUN THE I2C SCAN FIRST", MUTED)
             .present();
            timing::ms(1000);
            continue;
        }

        /* ---- the number -------------------------------------------------- */
        const Bool good = (status == 0);

        const gfx::Paint BIG { .fg = good ? GFX_WHITE : GFX_DARKGREY, .size = 4 };
        if(good)
        {
            c.printf({ left, y }, BIG, "%u", mm);
        }
        else
        {
            c.text({ left, y }, "----", BIG);
        }
        y += 36;

        c.text({ left, y }, "MM", MUTED);
        y += 18;

        const gfx::Paint METRES { .fg = good ? GFX_CYAN : GFX_DARKGREY, .size = 2 };
        if(good)
        {
            /* One decimal, done in integers - a float here would pull in the
             * whole soft-float formatting path for one number. */
            c.printf({ left, y }, METRES, "%u.%02u M", mm / 1000u, (mm % 1000u) / 10u);
        }
        else
        {
            c.text({ left, y }, "--.-- M", METRES);
        }
        y += 30;

        /* ---- the bar ----------------------------------------------------- */
        const Int32 barH = 16;
        c.rect({ left, y, wide, barH }, GFX_DARKGREY);

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
            const UInt16 bar = (mm < 150) ? GFX_RED
                           : (mm < 400) ? GFX_ORANGE
                           : GFX_GREEN;
            c.rectFill({ left + 1, y + 1, fill, barH - 2 }, bar);
        }
        y += barH + 6;

        c.text({ left, y },  "0",   FAINT)
         .text({ right, y }, "2 M", FAINT, gfx::ALIGN_RIGHT);
        y += 22;

        /* ---- status ------------------------------------------------------ */
        const gfx::Paint STATUS { .fg = good ? GFX_GREEN : GFX_YELLOW };
        c.text({ left, y }, tof::statusName(status), STATUS);
        y += 20;

        /* ---- what it has managed so far ---------------------------------- */
        if(seenMax > 0)
        {
            c.printf({ left, y }, MUTED, "SEEN %u - %u MM", seenMin, seenMax);
        }
        else
        {
            c.text({ left, y }, "SEEN NOTHING YET", MUTED);
        }
        y += 14;

        c.printf({ left, y }, MUTED, "%u READS", reads)
         .present();

        /* ~20 fps. The sensor's own measurement takes longer than this, so
         * polling faster would only burn power to be told "not yet". */
        timing::ms(50);
    }

    return 0;
}
