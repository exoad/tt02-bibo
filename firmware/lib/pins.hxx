/*
 * ---------------------------------------------------------------------------
 * pins - every pin the car drives, declared once, at startup.
 *
 * A program's first act is pins::begin(pins::car()), and nothing below this
 * file holds a GPIO number. begin() records and checks the map and touches no
 * GPIO, so it must come before any subsystem opens: until it has run, every
 * role is NONE and binds nothing.
 * -------------------------------------------------------------------------
 */
#pragma once

#include "shared.hxx"

/* For conflictText() only. text.hxx is a leaf, so this adds no cycle. */
#include "text.hxx"

namespace bibo::pins
{

    /**
     * @brief Marks a role as not wired to any pad.
     *
     * A subsystem holding this skips the pad rather than driving GPIO -1.
     */
    constexpr Int32 NONE = -1;

    /**
     * @brief The highest GPIO this package brings out.
     */
    constexpr Int32 MAX_GPIO = 29;

    /**
     * @brief Every GPIO role the firmware drives.
     *
     * Every member is an Int32 and nothing else, so the checks below can walk
     * the struct as an array. The static_assert on sizeof enforces it.
     */
    struct Map
    {
        Int32 servo = NONE;
        Int32 esc = NONE;
    };

    constexpr Size FIELD_COUNT = 2;

    static_assert(
        sizeof(Map) == FIELD_COUNT * sizeof(Int32),
        "pins::Map must be exactly FIELD_COUNT Int32 fields - a role " "was added without updating FIELD_COUNT and NAMES"
    );

    inline CharSeq NAMES[FIELD_COUNT] =
    {
        "servo", "esc"
    };

    /**
     * @brief Views a map as a flat array of FIELD_COUNT GPIO numbers.
     *
     * @param m the map to view
     * @return a pointer to the first field, aliasing m's own storage
     */
    static const Int32* fields(const Map* m)
    {
        return &m->servo;
    }

    /**
     * @brief How this car is wired.
     *
     * @return the car's map, ready to hand to pins::begin()
     *
     * @warning Kept in step with CAR_PADS below by hand: a pad added here is
     *          added there in the same edit.
     */
    inline Map car(Void)
    {
        Map m;

        m.servo = 0;
        m.esc = 1;

        return m;
    }

    /*
     * ---- the installed map ------------------------------------------------
     * It starts EMPTY - every field NONE - so a subsystem opened before begin()
     * binds nothing and is visibly dead.
     */
    inline Map  installed;
    inline Bool up = false;

    /* Where the last begin() found a problem, so a caller can name the pins. */
    inline Int32 clashPin = NONE;
    inline Size  clashA = 0;
    inline Size  clashB = 0;

    /**
     * @brief Validates a map and, if it is sound, installs it as the pins
     *        every subsystem reads from.
     *
     * Installs NOTHING if two roles claim one pad or a number is not a GPIO:
     * a half-applied map looks like it took.
     *
     * @param m the map to validate and, on success, install
     * @return true once m is installed; false if a pad is out of range or
     *         shared, in which case conflictText() says why
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
                clashA = a;
                clashB = a;
                return false;
            }
            for(Size b = a + 1; b < FIELD_COUNT; ++b)
            {
                if(f[b] != NONE && f[a] == f[b])
                {
                    clashPin = f[a];
                    clashA = a;
                    clashB = b;
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
     * Equal to conflictFirst() when the pad was out of range rather than
     * shared.
     *
     * @return a role name from NAMES, or "" when the last begin() succeeded
     */
    inline CharSeq conflictSecond(Void)
    {
        return clashPin == NONE ? "" : NAMES[clashB];
    }

    /**
     * @brief The complaint from the last begin(), as one sentence to print.
     *
     * @return the complaint, or "" when the last begin() succeeded
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
            text::format(
                buf,
                sizeof(buf),
                "pin %s is GP%d, which is not a pad",
                NAMES[clashA],
                clashPin
            );
        }
        else
        {
            text::format(
                buf,
                sizeof(buf),
                "pins %s and %s both want GP%d",
                NAMES[clashA],
                NAMES[clashB],
                clashPin
            );
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
     * @brief The car's map as a flat list, so a conflict in it is a build error
     *        rather than something begin() finds on the bench.
     *
     * car() stays an ordinary function, so this list repeats it by hand.
     */
    constexpr Int32 CAR_PADS[] =
    {
        0, 1,             /* servo, esc */
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

    static_assert(
        carIsSound(),
        "two roles in pins::car() claim the same GPIO, or a pad is " "not 0-29 - read car() above and decide which one gets it"
    );

}
