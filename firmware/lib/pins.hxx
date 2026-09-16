/*
 * pins - every pin the car drives, declared once. A program's first act is
 * pins::begin(pins::car()), and nothing below this file holds a GPIO number.
 * begin() records and checks the map and touches no GPIO, so it must come before
 * any subsystem opens: until then every role is NONE and binds nothing.
 */
#pragma once

#include "shared.hxx"
/* For conflictText() only. text.hxx is a leaf, so this adds no cycle. */
#include "text.hxx"

namespace bibo::pins
{
    /** A role wired to no pad: a subsystem holding it skips the pad rather than driving GPIO -1. */
    constexpr Int32 NONE = -1;

    /** The highest GPIO this package brings out. */
    constexpr Int32 MAX_GPIO = 29;

    /**
     * Only Int32 members, so the checks below can walk it as an array; the
     * static_assert enforces that.
     */
    struct Map
    {
        Int32 servo = NONE;
        Int32 esc = NONE;
        Int32 hallA = NONE;   /* the motor's hall sensors, through dividers */
        Int32 hallB = NONE;
        Int32 hallC = NONE;
    };

    constexpr Size FIELD_COUNT = 5;

    static_assert(
        sizeof(Map) == FIELD_COUNT * sizeof(Int32),
        "pins::Map must be exactly FIELD_COUNT Int32 fields - a role " "was added without updating FIELD_COUNT and NAMES"
    );

    inline CharSeq NAMES[FIELD_COUNT] =
    {
        "servo", "esc", "hallA", "hallB", "hallC"
    };

    static const Int32* fields(const Map* m)
    {
        return &m->servo;
    }

    /** How this car is wired. Kept in step with CAR_PADS by hand, in the same edit. */
    inline Map car(Void)
    {
        Map m;
        m.servo = 0;
        m.esc = 1;
        m.hallA = 11;
        m.hallB = 12;
        m.hallC = 13;
        return m;
    }

    inline Map  installed;
    inline Bool up = false;

    /* Where the last begin() found a problem, so a caller can name the pins. */
    inline Int32 clashPin = NONE;
    inline Size  clashA = 0;
    inline Size  clashB = 0;

    /**
     * Installs NOTHING if two roles claim one pad or a number is not a GPIO: a
     * half-applied map looks like it took. On false, conflictText() says why.
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

    inline Int32 conflictPin(Void)
    {
        return clashPin;
    }

    inline CharSeq conflictFirst(Void)
    {
        return clashPin == NONE ? "" : NAMES[clashA];
    }

    /** Equal to conflictFirst() when the pad was out of range rather than shared. */
    inline CharSeq conflictSecond(Void)
    {
        return clashPin == NONE ? "" : NAMES[clashB];
    }

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

    static const Map& active(Void)
    {
        return installed;
    }

    inline Bool ready(Void)
    {
        return up;
    }

    /**
     * The car's map as a flat list, so a conflict in it is a build error rather
     * than something begin() finds on the bench. car() stays an ordinary
     * function, so this repeats it by hand.
     */
    constexpr Int32 CAR_PADS[] =
    {
        0, 1,             /* servo, esc */
        11, 12, 13,       /* hallA, hallB, hallC */
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

    static_assert(
        carIsSound(),
        "two roles in pins::car() claim the same GPIO, or a pad is " "not 0-29 - read car() above and decide which one gets it"
    );
}
