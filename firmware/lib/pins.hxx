/* ---------------------------------------------------------------------------
 * pins - every pin a program uses, declared once, at startup.
 *
 * THE RULE: a program's first act is to say what is wired where. main.cxx does
 * it, every sketch does it, and nothing below this file contains a GPIO number.
 *
 *     pins::begin(pins::car());          the car, as it is wired today
 *     pins::begin(mine);                 a sketch, as the breadboard is
 *
 * Before this the numbers were scattered - PIN_SERVO 0 in chassis.hxx, ten
 * lamps in a table in lights.hxx - and "is GP14 taken?" was a question you
 * answered by grepping and hoping. It went stale the day somebody added a
 * driver, which is exactly how the DFPlayer nearly landed on the tail lamps.
 *
 * ---------------------------------------------------------------------------
 * A MAP, NOT A SET OF CONSTANTS, and that is the important part.
 *
 * The first version of this file was a wall of `constexpr Int32 SERVO = 0`.
 * That is right for the car and wrong for everything else: a sketch borrowing a
 * pad for an afternoon would have had to EDIT THE CAR'S WIRING to do it, and a
 * breadboard experiment must never be able to change what the finished vehicle
 * believes about itself.
 *
 * So a Map is a value. car() returns the vehicle's; a sketch builds its own, or
 * takes car() and overrides the fields it cares about. Both go through begin(),
 * both are checked the same way, and neither can affect the other.
 *
 * ---------------------------------------------------------------------------
 * DECLARING IS NOT OPENING.
 *
 * begin() records the map and validates it. It does NOT touch a GPIO, because
 * only a subsystem knows whether its pad wants a direction, a pull, a PWM slice
 * or an alternate function - lights::open() sets outputs, drive::open() starts
 * servo pulses, dfplayer::open() picks a UART funcsel. They read active()
 * instead of holding numbers.
 *
 * Which is why begin() must come FIRST. A subsystem opened before the map is
 * installed binds nothing at all - see `installed` below, which starts empty on
 * purpose.
 * ------------------------------------------------------------------------- */
#pragma once

#include "types.hxx"

/* For conflictText() only. text.hxx is a leaf - types.hxx and the C
 * headers - so this adds no cycle and no hardware dependency. */
#include "text.hxx"

namespace bibo
{

  namespace pins
  {

    /* Not wired. A subsystem holding this skips the pad entirely rather than
     * driving GPIO -1, which the SDK would do something undefined with. */
    constexpr Int32 NONE = -1;

    /* The highest GPIO this package brings out. A number past it is a typo, and
     * a typo in a pin number is invisible on a bench: the wire does nothing. */
    constexpr Int32 MAX_GPIO = 29;

    /* ---- the map ---------------------------------------------------------
     *
     * Every role the firmware knows about, whether or not a given program uses
     * it. A field left at NONE is NOT WIRED, and that is a normal answer rather
     * than an incomplete one - the rear indicators have been NONE since they
     * were added and cue::solve() computes them regardless.
     *
     * Every member is an Int32 and nothing else. That is load-bearing: it is
     * what lets the checks below walk the struct as an array instead of keeping
     * a hand-written list that goes stale the day a role is added. The
     * static_assert on sizeof enforces it. */
    struct Map
    {
        /* chassis */
        Int32 servo     = NONE;
        Int32 esc       = NONE;

        /* buses */
        Int32 i2cSda    = NONE;
        Int32 i2cScl    = NONE;

        /* sound - the DFPlayer Mini.
         *
         * THE DIRECTIONS ARE THE SILICON'S, not a preference. On RP2350 GPIO14
         * is UART0_TX and GPIO15 is UART0_RX - funcsel 0x0b, which the SDK
         * calls GPIO_FUNC_UART_AUX and not GPIO_FUNC_UART. Reversed, two
         * outputs face each other and two inputs face each other: nothing is
         * damaged, the port opens, every write succeeds, and no byte leaves the
         * chip.
         *
         * soundBusy is the module's own output and is LOW WHILE PLAYING. It is
         * the only honest way to know a track has finished - the serial reply
         * says a command was ACCEPTED, which is a different claim. */
        Int32 soundTx   = NONE;
        Int32 soundRx   = NONE;
        Int32 soundBusy = NONE;

        /* lamps, in lights::Lamp order */
        Int32 headL     = NONE;
        Int32 headR     = NONE;
        Int32 tailL     = NONE;
        Int32 tailR     = NONE;
        Int32 indFL     = NONE;
        Int32 indFR     = NONE;
        Int32 indRL     = NONE;
        Int32 indRR     = NONE;
        Int32 revL      = NONE;
        Int32 revR      = NONE;

        /* the SPI display.
         *
         * SCK and MOSI are fixed by the silicon for a given SPI block - GP18
         * and GP19 are SPI0 - while CS, DC and RES are plain GPIOs and are free
         * choices. Both kinds live here anyway: a caller should not have to
         * know which of the five it is allowed to move.
         *
         * A SHARED BUS IS TWO ROLES ON ONE PAD and begin() would reject it. The
         * MicroSD card is meant to sit on this same SCK/MOSI with its own chip
         * select, so when it goes on it wants a bus concept rather than another
         * pair of pin fields. Not solved here; written down so it is not a
         * surprise the day somebody adds sdSck and the map refuses to load. */
        Int32 tftSck    = NONE;
        Int32 tftMosi   = NONE;
        Int32 tftCs     = NONE;
        Int32 tftDc     = NONE;
        Int32 tftRes    = NONE;

        /* The panel's backlight. NONE means it is tied to 3V3 and always on,
         * which is how these boards ship - tft::brightness() then says so
         * rather than pretending to dim a pad nothing is driving. */
        Int32 tftBlk    = NONE;

        /* sensors */
        /* The MicroSD module. Its own SPI pads, NOT the panel's - the two
         * would be one bus shared by two devices at different clocks, and
         * begin() would reject the shared pads as two roles anyway. When a
         * bus becomes a first-class thing here, this is what changes. */
        Int32 sdSck     = NONE;
        Int32 sdMosi    = NONE;
        Int32 sdMiso    = NONE;
        Int32 sdCs      = NONE;

        Int32 encoder   = NONE;
    };

    constexpr Size FIELD_COUNT = 28;

    static_assert(sizeof(Map) == FIELD_COUNT * sizeof(Int32),
                  "pins::Map must be exactly FIELD_COUNT Int32 fields - a role "
                  "was added without updating FIELD_COUNT and NAMES");

    static CharSeq NAMES[FIELD_COUNT] =
    {
        "servo", "esc",
        "i2cSda", "i2cScl",
        "soundTx", "soundRx", "soundBusy",
        "headL", "headR", "tailL", "tailR",
        "indFL", "indFR", "indRL", "indRR",
        "revL", "revR",
        "tftSck", "tftMosi", "tftCs", "tftDc", "tftRes", "tftBlk",
        "sdSck", "sdMosi", "sdMiso", "sdCs",
        "encoder"
    };

    /* The map as a flat array. Legal because every member is an Int32 and the
     * static_assert above says so. */
    static const Int32* fields(const Map* m)
    {
        return &m->servo;
    }

    /* ---- the car ----------------------------------------------------------
     *
     * How THIS vehicle is wired today. A sketch may ignore it entirely, and
     * should read it first anyway so it borrows a pad that is free.
     *
     * EVERY LAMP IS ON A BORROWED PAD. GP10 to GP13 are the four ToF XSHUT
     * lines, free only because no ToF is fitted, and they stop being free the
     * moment one is. The permanent map is GP2/GP3 indicators, GP6/GP7 tails,
     * GP8 heads - moving there is editing this function and nothing else.
     *
     * THE TAIL LAMPS GAVE UP GP14 AND GP15 to the DFPlayer, which needed a
     * UART. Those two are the only pads carrying UART0 that cost neither
     * GP0/GP1 - the servo and the ESC - nor GP12/GP13, the front indicators. */
    static Map car(Void)
    {
        Map m;

        m.servo     = 0;
        m.esc       = 1;

        m.i2cSda    = 4;
        m.i2cScl    = 5;

        m.soundTx   = 14;   /* -> module RX, through 1k */
        m.soundRx   = 15;   /* <- module TX             */
        m.soundBusy = 9;    /* LOW while playing        */

        m.headL     = 11;
        m.headR     = 10;
        m.tailL     = NONE; /* was 15, now soundRx */
        m.tailR     = NONE; /* was 14, now soundTx */
        m.indFL     = 13;
        m.indFR     = 12;

        m.tftSck    = 18;   /* SPI0 SCK, fixed by the silicon */
        m.tftMosi   = 19;   /* SPI0 TX                        */
        m.tftCs     = 17;
        m.tftDc     = 21;
        m.tftRes    = 20;

        /* Where the module GOES. Its headers are unsoldered, so nothing is
         * on these pads yet and sd::open() will fail honestly until they
         * are - which beats the four #defines this replaced, which named
         * the same pads and could not be seen or checked from here. */
        m.sdSck     = 26;
        m.sdMosi    = 27;
        m.sdMiso    = 28;
        m.sdCs      = 22;

        m.encoder   = NONE; /* GP15 was earmarked; sound has it */

        return m;
    }

    /* ---- the installed map ------------------------------------------------
     *
     * One per program, because one program is one car. Subsystems read this
     * rather than holding numbers, so a sketch that installs a different map
     * gets different pads with no other change anywhere.
     *
     * It starts EMPTY - every field NONE - so a subsystem opened before begin()
     * binds nothing and is visibly dead. That is the right failure: a servo on
     * a pad chosen by whatever ran last is worse than a servo that does not
     * move. */
    static Map  installed;
    static Bool up = false;

    /* Where the last begin() found a problem. Kept so a caller can SAY which
     * pins clashed - "pin conflict" without the pin sends somebody back to the
     * file to work out which. */
    static Int32 clashPin = NONE;
    static Size  clashA   = 0;
    static Size  clashB   = 0;

    /* ---- validate and install ---------------------------------------------
     *
     * Returns false and installs NOTHING if two roles claim one pad, or a
     * number is not a GPIO. Refusing rather than applying a broken map is the
     * point: a half-applied wiring is worse than none, because the half that
     * worked makes it look like the map took.
     *
     * The car's own map is ALSO proved at compile time - see carIsSound() at
     * the bottom - so the vehicle can never ship a conflict. This runtime check
     * is what covers a SKETCH, whose map is written fresh against a breadboard
     * and is the one most likely to be wrong. */
    static Bool begin(const Map& m)
    {
        clashPin = NONE;

        const Int32* f = fields(&m);

        for(Size a = 0; a < FIELD_COUNT; ++a)
        {
            if(f[a] == NONE)
            {
                continue;
            }

            if(f[a] < 0 || f[a] > MAX_GPIO)
            {
                clashPin = f[a];
                clashA   = a;
                clashB   = a;
                return false;
            }

            for(Size b = a + 1; b < FIELD_COUNT; ++b)
            {
                if(f[b] != NONE && f[a] == f[b])
                {
                    clashPin = f[a];
                    clashA   = a;
                    clashB   = b;
                    return false;
                }
            }
        }

        installed = m;
        up        = true;
        return true;
    }

    /* What the last begin() refused. NONE and empty strings when it succeeded.
     * When clashA equals clashB the pad is out of range rather than doubly
     * claimed. */
    static Int32 conflictPin(Void)
    {
        return clashPin;
    }

    static CharSeq conflictFirst(Void)
    {
        return (clashPin == NONE) ? "" : NAMES[clashA];
    }

    static CharSeq conflictSecond(Void)
    {
        return (clashPin == NONE) ? "" : NAMES[clashB];
    }

    /*
     * The whole complaint, as one sentence, ready to print.
     *
     * The three accessors above are the parts, and every caller was assembling
     * them the same way - three calls into one printf, copied into main.cxx and
     * into each sketch. Copied WRONG, too: they all read
     *
     *     "pins %s and %s both want GP%d"
     *
     * which is a lie in the out-of-range case, where clashA and clashB are the
     * same role and the pad is simply not a pad. That message named one role
     * twice and blamed it for clashing with itself.
     *
     * One call, one sentence, both cases. The parts stay for a caller that
     * wants to say it differently.
     */
    static CharSeq conflictText(Void)
    {
        static Utf8 buf[96];

        if(clashPin == NONE)
        {
            return "";
        }
        if(clashA == clashB)
        {
            text::format(buf, sizeof(buf), "pin %s is GP%d, which is not a pad",
                         NAMES[clashA], clashPin);
        }
        else
        {
            text::format(buf, sizeof(buf), "pins %s and %s both want GP%d",
                         NAMES[clashA], NAMES[clashB], clashPin);
        }
        return buf;
    }

    /* The installed map. Every subsystem reads its pads from here. */
    static const Map& active(Void)
    {
        return installed;
    }

    /* False until begin() has succeeded. */
    static Bool ready(Void)
    {
        return up;
    }

    /* ---- the car's map, proved at compile time ----------------------------
     *
     * begin() catches a bad map when it runs. The VEHICLE'S map should never
     * get that far, so it is proved here too, where a conflict is a build error
     * naming this file rather than something found on a bench.
     *
     * A separate constexpr list, because car() has to stay an ordinary function
     * - it is what a sketch calls at runtime to start from. The two are kept in
     * step by hand, which is a real cost; the alternative was making car()
     * constexpr and losing the ability to build a map at runtime at all, and
     * that is the thing this file exists to allow. */
    constexpr Int32 CAR_PADS[] =
    {
        0, 1,             /* servo, esc        */
        4, 5,             /* i2c               */
        14, 15, 9,        /* sound tx, rx, busy */
        11, 10,           /* headlights        */
        13, 12,           /* front indicators  */
        18, 19, 17, 21, 20, /* display sck, mosi, cs, dc, res */
    };

    constexpr Size CAR_PAD_COUNT = sizeof(CAR_PADS) / sizeof(CAR_PADS[0]);

    constexpr Bool carIsSound(Void)
    {
        for(Size a = 0; a < CAR_PAD_COUNT; ++a)
        {
            if(CAR_PADS[a] < 0 || CAR_PADS[a] > MAX_GPIO)
            {
                return false;
            }
            for(Size b = a + 1; b < CAR_PAD_COUNT; ++b)
            {
                if(CAR_PADS[a] == CAR_PADS[b])
                {
                    return false;
                }
            }
        }
        return true;
    }

    static_assert(carIsSound(),
                  "two roles in pins::car() claim the same GPIO, or a pad is "
                  "not 0-29 - read car() above and decide which one gets it");

  } /* namespace pins */

} /* namespace bibo */
