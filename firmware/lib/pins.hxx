/*
 * ---------------------------------------------------------------------------
 * pins - every pin a program uses, declared once, at startup.
 *
 * THE RULE: a program's first act is to say what is wired where, and nothing
 * below this file contains a GPIO number. A map is a value, so a sketch
 * borrowing a pad builds its own instead of editing the car's wiring.
 *
 *     pins::begin(pins::car());          the car, as it is wired today
 *     pins::begin(mine);                 a sketch, as the breadboard is
 *
 * DECLARING IS NOT OPENING. begin() records and validates the map and touches
 * no GPIO - only a subsystem knows what its pad wants. Which is why begin()
 * must come FIRST: a subsystem opened before it binds nothing at all.
 * -------------------------------------------------------------------------
 */
#pragma once

#include "types.hxx"

/* For conflictText() only. text.hxx is a leaf, so this adds no cycle. */
#include "text.hxx"

namespace bibo::pins
{

    /**
     * @brief Marks a role as not wired to any pad.
     *
     * A subsystem holding this skips the pad entirely rather than driving
     * GPIO -1, which the SDK would do something undefined with.
     */
    constexpr Int32 NONE = -1;

    /**
     * @brief The highest GPIO this package brings out.
     *
     * A number past it is a typo, and a typo in a pin number is invisible on
     * a bench: the wire does nothing.
     */
    constexpr Int32 MAX_GPIO = 29;

    /**
     * @brief Every GPIO role the firmware knows about, whether or not a given
     *        program uses it.
     *
     * A field left at NONE is NOT WIRED, and that is a normal answer rather
     * than an incomplete one - the rear indicators have been NONE since they
     * were added and cue::solve() computes them regardless.
     *
     * Every member is an Int32 and nothing else. That is load-bearing: it is
     * what lets the checks below walk the struct as an array instead of
     * keeping a hand-written list that goes stale the day a role is added.
     * The static_assert on sizeof enforces it.
     *
     * @note Nothing in lib/ hardcodes a GPIO number - every subsystem reads
     *       its pads from the map that was installed with pins::begin(), and
     *       pins::car() is only a named default an image opts into rather
     *       than a binding this library imposes.
     */
    struct Map
    {
        /* chassis */
        Int32 servo     = NONE;
        Int32 esc       = NONE;

        /* buses */
        Int32 i2cSda    = NONE;
        Int32 i2cScl    = NONE;

        /*
         * sound - the DFPlayer Mini. THE DIRECTIONS ARE THE SILICON'S: on RP2350
         * GPIO14 is UART0_TX and GPIO15 is UART0_RX, funcsel 0x0b - that is
         * GPIO_FUNC_UART_AUX, not GPIO_FUNC_UART. Reversed, the port opens, every
         * write succeeds, and no byte leaves the chip. soundBusy is the module's
         * own output and is LOW WHILE PLAYING, the only honest end-of-track.
         */
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

        /*
         * the SPI display. SCK and MOSI are fixed by the silicon for a given
         * block - GP18 and GP19 are SPI0 - while CS, DC and RES are free
         * choices; both kinds live here so a caller need not know which of the
         * five it may move. A SHARED BUS IS TWO ROLES ON ONE PAD and begin()
         * rejects it, so a second device on this SCK/MOSI wants a bus concept
         * rather than another pair of pin fields.
         */
        Int32 tftSck    = NONE;
        Int32 tftMosi   = NONE;
        Int32 tftCs     = NONE;
        Int32 tftDc     = NONE;
        Int32 tftRes    = NONE;

        /* Backlight. NONE means tied to 3V3 and always on, as these boards ship. */
        Int32 tftBlk    = NONE;

        /* sensors */
        /*
         * The MicroSD module. Its own SPI pads, NOT the panel's - the two would
         * be one bus at different clocks, and begin() rejects shared pads.
         */
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

    inline CharSeq NAMES[FIELD_COUNT] =
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

    /**
     * @brief Views a map as a flat array of FIELD_COUNT GPIO numbers.
     *
     * Legal because every member of Map is an Int32 and the static_assert
     * above says so.
     *
     * @param m the map to view
     * @return a pointer to the first field, aliasing m's own storage
     */
    static const Int32* fields(const Map* m)
    {
        return &m->servo;
    }

    /**
     * @brief A named default: how THIS vehicle is wired today.
     *
     * A sketch may ignore this entirely, and should read it first anyway so
     * it borrows a pad that is free. This function is a default an image
     * opts into by calling pins::begin(pins::car()); nothing in lib/ calls
     * it for you.
     *
     * EVERY LAMP IS ON A BORROWED PAD. GP10 to GP13 are the four ToF XSHUT
     * lines, free only because no ToF is fitted, and they stop being free the
     * moment one is. The permanent map is GP2/GP3 indicators, GP6/GP7 tails,
     * GP8 heads - moving there is editing this function and nothing else.
     *
     * THE TAIL LAMPS GAVE UP GP14 AND GP15 to the DFPlayer, which needed a
     * UART. Those two are the only pads carrying UART0 that cost neither
     * GP0/GP1 - the servo and the ESC - nor GP12/GP13, the front indicators.
     *
     * @return the car's map, ready to hand to pins::begin()
     */
    inline Map car(Void)
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

        /* Where the module GOES. Headers unsoldered, so sd::open() fails until then. */
        m.sdSck     = 26;
        m.sdMosi    = 27;
        m.sdMiso    = 28;
        m.sdCs      = 22;

        m.encoder   = NONE; /* GP15 was earmarked; sound has it */

        return m;
    }

    /*
     * ---- the installed map ------------------------------------------------
     * One per program, because one program is one car. It starts EMPTY - every
     * field NONE - so a subsystem opened before begin() binds nothing and is
     * visibly dead, which beats a servo on a pad whatever ran last chose.
     */
    inline Map  installed;
    inline Bool up = false;

    /* Where the last begin() found a problem, so a caller can name the pins. */
    inline Int32 clashPin = NONE;
    inline Size  clashA   = 0;
    inline Size  clashB   = 0;

    /**
     * @brief Validates a map and, if it is sound, installs it as the pins
     *        every subsystem reads from.
     *
     * Installs NOTHING if two roles claim one pad, or a number is not a
     * GPIO. Refusing rather than applying a broken map is the point: a
     * half-applied wiring is worse than none, because the half that worked
     * makes it look like the map took.
     *
     * The car's own map is ALSO checked at compile time - see carIsSound()
     * at the bottom. This runtime check is what covers a SKETCH, whose map
     * is written fresh against a breadboard and is the one most likely to be
     * wrong.
     *
     * @param m the map to validate and, on success, install
     * @return true once m is installed and active() returns it; false if a
     *         pad is out of range or shared, in which case conflictText()
     *         describes why and nothing was installed
     *
     * @note This must run before any subsystem's open() call - see the file
     *       banner. A subsystem opened first binds to whatever installed
     *       held before, which starts out entirely NONE.
     * @note The car's own map is ALSO proved sound at compile time, by
     *       carIsSound() over CAR_PADS, so pins::car() cannot ship a conflict.
     *       That cover was incomplete until 2026-08-31 - the four MicroSD pads
     *       were assigned by car() and missing from CAR_PADS, so a clash on
     *       them would have reached this runtime check instead.
     */
    inline Bool begin(const Map& m)
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
        up = true;
        return true;
    }

    /**
     * @brief The GPIO number the last begin() refused, if any.
     *
     * When clashA equals clashB the pad was out of range rather than doubly
     * claimed.
     *
     * @return the offending GPIO number, or NONE when the last begin()
     *         succeeded
     */
    inline Int32 conflictPin(Void)
    {
        return clashPin;
    }

    /**
     * @brief The name of the first role that claimed the conflicting pad.
     *
     * @return a role name from NAMES, or "" when the last begin() succeeded
     */
    inline CharSeq conflictFirst(Void)
    {
        return clashPin == NONE ? "" : NAMES[clashA];
    }

    /**
     * @brief The name of the second role that claimed the conflicting pad.
     *
     * Equal to conflictFirst() when the pad itself was out of range rather
     * than shared between two roles.
     *
     * @return a role name from NAMES, or "" when the last begin() succeeded
     */
    inline CharSeq conflictSecond(Void)
    {
        return clashPin == NONE ? "" : NAMES[clashB];
    }

    /**
     * @brief The whole complaint from the last begin(), as one sentence,
     *        ready to print.
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
     *
     * @return the complaint as one sentence, or "" when the last begin()
     *         succeeded
     */
    inline CharSeq conflictText(Void)
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

    /**
     * @brief The installed map. Every subsystem reads its pads from here.
     *
     * @return the map given to the last successful begin()
     */
    static const Map& active(Void)
    {
        return installed;
    }

    /**
     * @brief Whether a map has been installed yet.
     *
     * @return false until begin() has succeeded
     */
    inline Bool ready(Void)
    {
        return up;
    }

    /**
     * @brief The car's map, restated as a flat list so it can be checked at
     *        compile time.
     *
     * begin() catches a bad map when it runs. The VEHICLE'S map should never
     * get that far, so it is checked here too, where a conflict is a build
     * error naming this file rather than something found on a bench.
     *
     * A separate constexpr list, because car() has to stay an ordinary
     * function - it is what a sketch calls at runtime to start from. The two
     * are kept in step by hand, which is a real cost; the alternative was
     * making car() constexpr and losing the ability to build a map at
     * runtime at all, and that is the thing this file exists to allow.
     *
     * @warning THIS LIST IS KEPT IN STEP WITH car() BY HAND, and it has gone
     *          stale once already. Until 2026-08-31 it omitted the four
     *          MicroSD pads that car() assigns (26, 27, 28, 22), so the
     *          compile-time proof quietly covered fifteen of the car's
     *          nineteen pads while reading as though it covered all of them.
     *          Add a pad to car() and it must be added here in the same edit.
     */
    constexpr Int32 CAR_PADS[] =
    {
        0, 1,             /* servo, esc        */
        4, 5,             /* i2c               */
        14, 15, 9,        /* sound tx, rx, busy */
        11, 10,           /* headlights        */
        13, 12,           /* front indicators  */
        18, 19, 17, 21, 20, /* display sck, mosi, cs, dc, res */
        26, 27, 28, 22,   /* microSD sck, mosi, miso, cs */
    };

    constexpr Size CAR_PAD_COUNT = sizeof(CAR_PADS) / sizeof(CAR_PADS[0]);

    /**
     * @brief Whether CAR_PADS has no out-of-range pad and no pad claimed
     *        twice.
     *
     * @return true when every entry in CAR_PADS is 0-MAX_GPIO and unique
     */
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

}
