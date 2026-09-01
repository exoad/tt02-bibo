/**
 * @file range.hxx
 * @brief VL53L1X time-of-flight range sensor, over I2C.
 *
 * ---------------------------------------------------------------------------
 * WHAT IT ACTUALLY MEASURES
 *
 * It fires an infrared laser and times how long the light takes to come back.
 * Light covers a meter in about 3.3 nanoseconds, so a 4-meter reading is a
 * 26-nanosecond measurement - which is why this is a whole sensor with its own
 * processor rather than a pin you read.
 *
 * The answer is a distance in MILLIMETERS to whatever is in a cone in front of
 * it. Not a point: a cone, 27 degrees wide by default. At two meters that cone
 * is nearly a meter across, so "the distance" is really "the nearest thing in
 * a fairly wide view", which matters when using it to find a wall.
 *
 *     tof::Vl53 tof;
 *     tof::open(&tof, PIN_SDA, VL53_ADDR_DEFAULT);
 *     tof::startRanging(&tof);
 *
 *     if(tof::ready(&tof))
 *     {
 *         const UInt16 mm = tof::distance(&tof);
 *         tof::clear(&tof);              // arm the next measurement
 *     }
 *
 * ---------------------------------------------------------------------------
 * WIRING
 *
 *   Sensor   Pico     why
 *   VIN      3V3      the module regulates; the die is 2.8 V
 *   GND      GND
 *   SDA      GP4      I2C0
 *   SCL      GP5      I2C0
 *   XSHUT    -        optional; see below
 *   INT      -        optional; see below
 *
 * XSHUT holds the sensor in reset. It is only needed with SEVERAL sensors: they
 * all ship at address 0x29 and cannot share a bus until each has been given a
 * different one, which is done by holding all but one in reset and re-addressing
 * the one that is awake. With a single sensor there is nothing to disambiguate.
 *
 * INT pulses when a measurement is ready, so a program can sleep instead of
 * asking. tof::ready() asks, which costs one I2C read and needs no wire.
 *
 * ---------------------------------------------------------------------------
 * THE CONFIGURATION BLOCK
 *
 * Bringing this sensor up means writing 91 bytes into registers 0x2D..0x87.
 * Most of them are marked "not user-modifiable" by ST and are not documented
 * anywhere - they are the calibration ST's own driver writes, and the sensor
 * does not work without them.
 *
 * That is unsatisfying and it is also the situation. The block below is ST's
 * published default from the VL53L1X ULD driver, kept as one array rather than
 * scattered through the code, with the handful of bytes that ARE meaningful
 * called out by name in the comments beside them.
 */
#pragma once

#include "../hal.hxx"
#include "../pins.hxx"

namespace bibo::tof
{

#define VL53_ADDR_DEFAULT 0x29

    /* The registers worth naming. The rest live inside the config block. */
#define VL53_REG_CONFIG_START     0x002D
#define VL53_REG_GPIO_HV_MUX      0x0030
#define VL53_REG_GPIO_TIO_STATUS  0x0031
#define VL53_REG_RANGE_VALID_HIGH 0x0069
#define VL53_REG_SYSTEM_INTERRUPT 0x0086
#define VL53_REG_SYSTEM_START     0x0087
#define VL53_REG_RESULT_STATUS    0x0089
#define VL53_REG_RESULT_DISTANCE  0x0096
#define VL53_REG_FIRMWARE_STATUS  0x00E5
#define VL53_REG_MODEL_ID         0x010F

    /*
     * What the sensor says it is. Checked at open, because a wrong-but-present
     * device at 0x29 is a much clearer failure than a sensor that never ranges.
     */
#define VL53_MODEL_ID 0xEACC

    /*
     * ---- timing budget --------------------------------------------------------
     *
     * How long the sensor integrates for, per measurement. This is the single
     * biggest control over how far it reaches, and leaving it unset is a mistake
     * that hides well: the sensor still ranges, still reports status 0, and simply
     * cannot see anything far away.
     *
     * Longer means more photons collected, which means a weak return from a distant
     * or dark surface rises above the noise. Roughly:
     *
     *   20 ms    short reach, fast                       ~1.0 m in long mode
     *   50 ms    the sensible default                    ~2.5 m
     *   100 ms   what the 4 m figure on the box assumes  ~3.6 m
     *   200 ms   diminishing returns indoors             ~4.0 m
     *
     * The numbers below are ST's, from the ULD driver. They are pre-computed
     * macro-period timeouts rather than anything derivable from the milliseconds,
     * and they DIFFER BY DISTANCE MODE - which is why changing the mode has to
     * re-apply the budget, and why tof::setMode() does.
     */
    enum Budget
    {
        BUDGET_20MS = 0,
        BUDGET_33MS,
        BUDGET_50MS,
        BUDGET_100MS,
        BUDGET_200MS,
        BUDGET_500MS
    };

    /**
     * @brief Timing-budget register values for long distance mode.
     *
     * Registers 0x005E and 0x0061, indexed [budget][0..1].
     */
    static const UInt16 BUDGET_LONG[6][2] = {
        { 0x001E, 0x0022 },   /*  20 ms */
        { 0x0060, 0x006E },   /*  33 ms */
        { 0x00AD, 0x00C6 },   /*  50 ms */
        { 0x01CC, 0x01EA },   /* 100 ms */
        { 0x02D9, 0x02F8 },   /* 200 ms */
        { 0x048F, 0x04A4 }    /* 500 ms */
    };

    /**
     * @brief Timing-budget register values for short distance mode.
     *
     * NOTE the first row. ST publishes a 15 ms entry for short mode that
     * has no long-mode counterpart, so the two tables are not row-for-row
     * the same budget - this one starts at ST's 20 ms values, matching
     * the enum, and the 15 ms entry is simply not offered.
     */
    static const UInt16 BUDGET_SHORT[6][2] = {
        { 0x0051, 0x006E },   /*  20 ms */
        { 0x00D6, 0x006E },   /*  33 ms */
        { 0x01AE, 0x01E8 },   /*  50 ms */
        { 0x02E1, 0x0388 },   /* 100 ms */
        { 0x03E1, 0x0496 },   /* 200 ms */
        { 0x0591, 0x05C1 }    /* 500 ms */
    };

    /** @brief The two distance/window presets the sensor can run in. */
    enum Mode
    {
        /*
         * Up to about 1.3 m, and much better in bright light. The right default for
         * a bumper: the ambient infrared in daylight is what limits this sensor,
         * not the laser.
         */
        MODE_SHORT = 0,

        /* Up to about 4 m indoors, and easily blinded outdoors. */
        MODE_LONG
    };

    /** @brief One VL53L1X sensor: its bus pin, address, and configured state. */
    struct Vl53
    {
        Pin   sda;
        UInt8 addr;
        Bool  ok;

        /**
         * @brief The distance mode and timing budget this sensor is
         * currently configured with.
         *
         * Remembered because the timing budget's register values DEPEND
         * on the distance mode. Changing one without re-applying the
         * other leaves the sensor integrating for a length of time it
         * was not configured for, which shortens its reach without
         * reporting anything wrong.
         */
        Mode   mode;
        Budget budget;

        /**
         * @brief The interrupt polarity the sensor was configured with.
         *
         * tof::ready() compares against it, and reading it back rather
         * than assuming is what makes the check work on a module wired
         * either way round.
         */
        UInt8 intPolarity;
    };

    /**
     * @brief ST's published default configuration, registers 0x2D
     * through 0x87.
     *
     * Comments mark the bytes that are documented and meaningful.
     * Everything unmarked is "not user-modifiable" in ST's own words -
     * undocumented calibration that the sensor does not work without.
     */
    static const UInt8 DEFAULT_CONFIG[91] = {
        0x00,   /* 0x2D */
        0x00,   /* 0x2E  I2C pull-up level */
        0x00,   /* 0x2F  GPIO pull-up level */
        0x01,   /* 0x30  interrupt polarity lives in bit 4 */
        0x02,   /* 0x31 */
        0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00,
        0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00,
        0x00, 0x00,     /* 0x44, 0x45 */
        0x20,   /* 0x46  interrupt on "new sample ready" */
        0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21, 0x00, 0x00, 0x05,
        0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF,
        0x01, 0x00, 0x08, 0x00, 0x00, 0x01, 0xDB, 0x0F, 0x01,
        0xF1, 0x0D,
        0x01,   /* 0x64  sigma threshold, high byte - 90 mm by default */
        0x68,   /* 0x65  sigma threshold, low byte */
        0x00,   /* 0x66  minimum count rate, high byte */
        0x80,   /* 0x67  minimum count rate, low byte */
        0x08, 0xB8, 0x00, 0x00,
        0x00,   /* 0x6C  inter-measurement period, 32 bits from here */
        0x00, 0x0F, 0x89,
        0x00, 0x00,
        0x00,   /* 0x72  distance threshold high, high byte */
        0x00,   /* 0x73  distance threshold high, low byte */
        0x00,   /* 0x74  distance threshold low, high byte */
        0x00,   /* 0x75  distance threshold low, low byte */
        0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00, 0x00, 0x02,
        0xC7,   /* 0x7F  region-of-interest center */
        0xFF,   /* 0x80  region-of-interest size, X and Y */
        0x9B, 0x00, 0x00, 0x00, 0x01,
        0x00,   /* 0x86  clear interrupt */
        0x00    /* 0x87  start/stop ranging */
    };

    /* ---- distance mode ------------------------------------------------------- */

    /*
     * Forward declared: setting the mode re-applies the budget, and setting the
     * budget needs to know the mode.
     */
    [[nodiscard]] static Bool setBudget(Vl53* v, Budget budget);

    /**
     * @brief Configures the sensor for short or long distance mode.
     *
     * Short or long range: these four registers are the phase and timing
     * windows the sensor uses to decide what counts as a return. Short
     * mode narrows them, which is what makes it reject the ambient
     * infrared that swamps the long mode in daylight.
     *
     * @param v the sensor to configure; must have opened successfully
     * @param mode the distance mode to switch to
     * @return true once the mode and its (mode-specific) timing budget
     *         have both been written
     *
     * @note Re-applies the current timing budget after changing mode,
     * since the budget's register values differ by mode and would
     * otherwise be left set for the mode being left.
     */
    [[nodiscard]] static Bool setMode(Vl53* v, const Mode mode)
    {
        if(!v->ok)
        {
            return false;
        }

        v->mode = mode;

        if(mode == MODE_SHORT)
        {
            const Bool okShort =
                    i2c::writeReg16U8(v->sda, v->addr, 0x004B, 0x14)
                    && i2c::writeReg16U8(v->sda, v->addr, 0x0060, 0x07)
                    && i2c::writeReg16U8(v->sda, v->addr, 0x0063, 0x05)
                    && i2c::writeReg16U8(v->sda, v->addr, VL53_REG_RANGE_VALID_HIGH, 0x38)

                    /*
                 * The sigma-delta pair, and the reason short mode reported a
                 * hardware fault when these were missing.
                 *
                 * The configuration block leaves 0x78 = 0x0F0D and 0x7A = 0x0E0E,
                 * which are the LONG values. Changing the VCSEL periods above
                 * without changing these leaves the sensor with a phase window
                 * that does not match the period it is now pulsing at - which is
                 * not a wrong reading, it is an inconsistent configuration, and it
                 * reports as a fault.
                 *
                 * Long mode worked precisely BECAUSE it agreed with the block by
                 * accident. That is why only one of the two modes was broken.
                 */
                    && i2c::writeReg16U16(v->sda, v->addr, 0x0078, 0x0705)
                    && i2c::writeReg16U16(v->sda, v->addr, 0x007A, 0x0606);

            return okShort && setBudget(v, v->budget);
        }

        const Bool okLong =
                i2c::writeReg16U8(v->sda, v->addr, 0x004B, 0x0A)
                && i2c::writeReg16U8(v->sda, v->addr, 0x0060, 0x0F)
                && i2c::writeReg16U8(v->sda, v->addr, 0x0063, 0x0D)
                && i2c::writeReg16U8(v->sda, v->addr, VL53_REG_RANGE_VALID_HIGH, 0xB8)

                /*
             * Written explicitly even though the configuration block already
             * leaves these values. Relying on that meant long mode worked by
             * agreement rather than by instruction, and switching to short and
             * back would otherwise never restore them.
             */
                && i2c::writeReg16U16(v->sda, v->addr, 0x0078, 0x0F0D)
                && i2c::writeReg16U16(v->sda, v->addr, 0x007A, 0x0E0E);

        /* The budget's values are mode-specific, so it goes back in. */
        return okLong && setBudget(v, v->budget);
    }

    /**
     * @brief Sets how long each measurement integrates for.
     *
     * Must be set explicitly - the configuration block alone does not
     * leave a usable budget, and a sensor without one ranges happily and
     * cannot see past about a meter.
     *
     * @param v the sensor to configure; must have opened successfully
     * @param budget how long to integrate; longer reaches farther but
     *        samples more slowly - see the timing budget table above
     * @return true once written; false if v has not opened or budget is
     *         out of range
     */
    [[nodiscard]] static Bool setBudget(Vl53* v, const Budget budget)
    {
        if(!v->ok)
        {
            return false;
        }
        if(static_cast<Int32>(budget) < 0 || static_cast<Int32>(budget) > static_cast<Int32>(BUDGET_500MS))
        {
            return false;
        }

        v->budget = budget;

        const UInt16* row = v->mode == MODE_SHORT
                                ? BUDGET_SHORT[static_cast<Int32>(budget)]
                                : BUDGET_LONG[static_cast<Int32>(budget)];

        return i2c::writeReg16U16(v->sda, v->addr, 0x005E, row[0])
               && i2c::writeReg16U16(v->sda, v->addr, 0x0061, row[1]);
    }

    /* ---- bring-up ------------------------------------------------------------ */

    /**
     * @brief Wakes the sensor, checks it is one, and writes the
     * configuration.
     *
     * The bus must already be open - i2c::open() - because several
     * devices share it and it is not this driver's to configure.
     *
     * On a pad the caller names. For a sensor that is not on the
     * installed map - a second one on another bus, or a bench rig.
     *
     * @param v the sensor to initialize; must not be null
     * @param sda the I2C data pin the sensor is on
     * @param addr the I2C address to talk to; VL53_ADDR_DEFAULT unless
     *        the sensor has already been re-addressed
     * @return true once the sensor has answered, been identified, and
     *         been configured with the default block, mode and budget
     */
    [[nodiscard]] static Bool openOn(Vl53* v, const Pin sda, const UInt8 addr)
    {
        if(v == nullptr)
        {
            return false;
        }

        v->sda         = sda;
        v->addr        = addr;
        v->ok          = false;
        v->intPolarity = 1;
        v->mode        = MODE_LONG;
        v->budget      = BUDGET_50MS;

        /*
         * Is anything there at all? A separate check from the ID below, because
         * "nothing answers" and "something answers and is not a VL53L1X" are
         * different problems with different fixes.
         */
        if(!i2c::present(sda, addr))
        {
            return false;
        }

        UInt16 model = 0;
        if(!i2c::readReg16U16(sda, addr, VL53_REG_MODEL_ID, &model)
           || model != VL53_MODEL_ID)
        {
            return false;
        }

        /*
         * Wait for the firmware to finish booting. Writing configuration into a
         * sensor that is still starting up is the classic way to get one that
         * ranges but returns nonsense.
         */
        Bool booted = false;
        for(Int32 i = 0; i < 1000; ++i)
        {
            UInt8 st = 0;
            if(i2c::readReg16U8(sda, addr, VL53_REG_FIRMWARE_STATUS, &st)
               && (st & 0x01u) != 0u)
            {
                booted = true;
                break;
            }
            timing::ms(2);
        }
        if(!booted)
        {
            return false;
        }

        /*
         * The block goes in one register at a time. It could go as one burst, and
         * a burst would need a 93-byte buffer for a one-off - not worth it.
         */
        for(Int32 i = 0; i < 91; ++i)
        {
            const UInt16 reg = static_cast<UInt16>(VL53_REG_CONFIG_START + i);
            if(!i2c::writeReg16U8(sda, addr, reg, DEFAULT_CONFIG[i]))
            {
                return false;
            }
        }

        v->ok = true;

        /*
         * One throwaway measurement, which ST's own driver also does. The first
         * result after configuration is not trustworthy, and starting a program by
         * showing a wrong number is worse than starting it a hundred milliseconds
         * later.
         */
        static_cast<Void>(i2c::writeReg16U8(sda, addr, VL53_REG_SYSTEM_START, 0x40));
        for(Int32 i = 0; i < 500; ++i)
        {
            UInt8 st = 0;
            if(i2c::readReg16U8(sda, addr, VL53_REG_GPIO_TIO_STATUS, &st))
            {
                if((st & 0x01u) == v->intPolarity)
                {
                    break;
                }
            }
            timing::ms(2);
        }
        static_cast<Void>(i2c::writeReg16U8(sda, addr, VL53_REG_SYSTEM_INTERRUPT, 0x01));
        static_cast<Void>(i2c::writeReg16U8(sda, addr, VL53_REG_SYSTEM_START, 0x00));

        /* VHV config, which ST's driver writes after the first range. */
        static_cast<Void>(i2c::writeReg16U8(sda, addr, 0x0008, 0x09));
        static_cast<Void>(i2c::writeReg16U8(sda, addr, 0x000B, 0x00));

        /* Read back the polarity actually configured rather than assuming it. */
        {
            UInt8 mux = 0;
            if(i2c::readReg16U8(sda, addr, VL53_REG_GPIO_HV_MUX, &mux))
            {
                v->intPolarity = (mux & 0x10u) != 0u ? 0u : 1u;
            }
        }

        /*
         * Mode first, then budget - and tof::setMode re-applies the budget anyway,
         * because the two are not independent. 50 ms reaches about 2.5 m, which is
         * a useful indoor default; raise it for more reach at a lower rate.
         */
        /*
         * Both, and then the verdict - not `a && b`, which would skip the
         * budget write whenever the mode write failed. The sensor is left in
         * a known state either way and the caller is told whether it took.
         */
        const Bool modeSet   = setMode(v, MODE_LONG);
        const Bool budgetSet = setBudget(v, BUDGET_50MS);
        return modeSet && budgetSet;
    }

    /**
     * @brief Opens the sensor on the pads this program declared.
     *
     * The other four drivers already worked this way and this one did
     * not: the caller passed `pins::active().i2cSda` into it, which is
     * the sketch reading the map on the driver's behalf and then handing
     * it back. The point of pins::begin() is that a driver knows where
     * it is.
     *
     * @param v the sensor to initialize; must not be null
     * @param addr the I2C address to talk to
     * @return true once the sensor has answered, been identified, and
     *         been configured
     *
     * @note The BUS is still the caller's to open - i2c::open() - and
     * deliberately so. Several devices share one bus and each opening it
     * would be each one deciding the clock for all of them.
     */
    [[nodiscard]] static Bool open(Vl53* v, const UInt8 addr)
    {
        return openOn(v, pins::active().i2cSda, addr);
    }

    /**
     * @brief Reads the signal and ambient light rates from the last
     * measurement.
     *
     * The diagnostic that tells you WHY a reading is what it is. A
     * strong signal at a short distance means something really is close
     * - including a protective film still stuck on the lens, which is by
     * far the commonest reason a brand new sensor reads a few
     * centimeters and never changes.
     *
     * A high AMBIENT rate with a weak signal means the sensor is being
     * blinded by infrared in the room, which is what short mode exists
     * to fix.
     *
     * @param v the sensor to query; must have opened successfully
     * @param signalOut receives the signal rate, in mega-counts per
     *        second, 16.16 fixed point; left untouched if null
     * @param ambientOut receives the ambient rate, in mega-counts per
     *        second, 16.16 fixed point; left untouched if null
     * @return true once both rates have been read
     */
    [[nodiscard]] static Bool rates(const Vl53* v, UInt16* signalOut, UInt16* ambientOut)
    {
        if(!v->ok)
        {
            return false;
        }

        UInt16 sig = 0;
        UInt16 amb = 0;

        /*
         * 0x0098 is the crosstalk-corrected peak SIGNAL rate for this measurement,
         * and 0x0090 is the AMBIENT rate. Both are the sensor's own fixed point;
         * the ratio between them is what matters and the absolute units do not.
         *
         * The ambient register is 0x0090 and NOT 0x009A. Reading 0x009A returns a
         * different field entirely, and it returns a large, almost unchanging
         * number - which read exactly like a sensor being swamped by room light and
         * was nothing of the kind. Worth naming, because a plausible wrong answer
         * from a wrong register is the hardest kind to notice.
         */
        if(!i2c::readReg16U16(v->sda, v->addr, 0x0098, &sig)
           || !i2c::readReg16U16(v->sda, v->addr, 0x0090, &amb))
        {
            return false;
        }

        if(signalOut != nullptr)
        {
            *signalOut = sig;
        }
        if(ambientOut != nullptr)
        {
            *ambientOut = amb;
        }
        return true;
    }

    /* ---- ranging ------------------------------------------------------------- */

    /**
     * @brief Starts continuous ranging.
     *
     * @param v the sensor to start; must have opened successfully
     * @return true once the start command has been written
     */
    [[nodiscard]] static Bool startRanging(const Vl53* v)
    {
        return v->ok
               && i2c::writeReg16U8(v->sda, v->addr, VL53_REG_SYSTEM_START, 0x40);
    }

    /**
     * @brief Stops ranging.
     *
     * @param v the sensor to stop; must have opened successfully
     * @return true once the stop command has been written
     */
    [[nodiscard]] static Bool stopRanging(const Vl53* v)
    {
        return v->ok
               && i2c::writeReg16U8(v->sda, v->addr, VL53_REG_SYSTEM_START, 0x00);
    }

    /**
     * @brief Whether a new measurement is waiting to be read.
     *
     * Costs one register read.
     *
     * @param v the sensor to check
     * @return true when tof::distance() and tof::status() have a fresh
     *         result
     */
    [[nodiscard]] static Bool ready(const Vl53* v)
    {
        if(!v->ok)
        {
            return false;
        }
        UInt8 st = 0;
        if(!i2c::readReg16U8(v->sda, v->addr, VL53_REG_GPIO_TIO_STATUS, &st))
        {
            return false;
        }
        return (st & 0x01u) == v->intPolarity;
    }

    /**
     * @brief Arms the next measurement.
     *
     * Must be called after every reading. Without it the sensor holds
     * the same result forever and tof::ready() stays true - which looks
     * exactly like a distance that has frozen, and sends you looking at
     * the wrong thing.
     *
     * @param v the sensor to arm; must have opened successfully
     * @return true once the clear-interrupt command has been written
     */
    inline Bool clear(const Vl53* v)
    {
        return v->ok
               && i2c::writeReg16U8(v->sda, v->addr, VL53_REG_SYSTEM_INTERRUPT, 0x01);
    }

    /**
     * @brief Clears any stale interrupt and starts ranging again.
     *
     * The pair belongs together after a reconfiguration: the interrupt
     * left over from the previous settings would otherwise be handed
     * back as the first "reading" under the new ones, which is the one
     * measurement guaranteed to be wrong.
     *
     * @param v the sensor to restart; must have opened successfully
     * @return true once both the clear and the start have been written
     */
    [[nodiscard]] static Bool clearInterruptAndStart(Vl53* v)
    {
        return clear(v) && startRanging(v);
    }

    /**
     * @brief The most recent measurement, in millimeters.
     *
     * Meaningless unless tof::status() reports 0.
     *
     * @param v the sensor to read
     * @return the distance in millimeters, or 0 if v has not opened or
     *         the read failed
     */
    inline UInt16 distance(const Vl53* v)
    {
        if(!v->ok)
        {
            return 0;
        }
        UInt16 mm = 0;
        if(!i2c::readReg16U16(v->sda, v->addr, VL53_REG_RESULT_DISTANCE, &mm))
        {
            return 0;
        }
        return mm;
    }

    /**
     * @brief The most recent measurement's status, remapped to a small,
     * friendly set of codes.
     *
     * 0 means the reading is good. Anything else means it is not, and
     * the distance that came with it should be ignored rather than
     * shown.
     *
     * The raw codes are remapped by ST's driver into a friendlier set;
     * this returns the friendly one, because the raw values are not in a
     * useful order.
     *
     * @param v the sensor to read
     * @return 0 for a good reading; 1-8 naming a specific failure (see
     *         tof::statusName()); 255 if v has not opened or the read
     *         failed
     */
    inline UInt8 status(const Vl53* v)
    {
        if(!v->ok)
        {
            return 255;
        }

        UInt8 raw = 0;
        if(!i2c::readReg16U8(v->sda, v->addr, VL53_REG_RESULT_STATUS, &raw))
        {
            return 255;
        }
        raw &= 0x1Fu;

        switch(raw)
        {
            case 9:  return 0;    /* the only "good" raw code */
            case 6:  return 1;    /* sigma too high - the answer is too noisy */
            case 4:  return 2;    /* signal too weak - nothing came back */
            case 8:  return 3;    /* out of the valid phase - beyond range */
            case 5:  return 4;    /* hardware fail */
            case 12: return 5;    /* wrapped target - an echo from further than it says */
            case 18: return 6;    /* synchronisation */
            case 3:  return 7;    /* merged pulse */
            default: return 8;    /* something else */
        }
    }

    /**
     * @brief A short human-readable name for a status code.
     *
     * @param status a code as returned by tof::status()
     * @return a NUL-terminated string naming the status; "UNKNOWN" for
     *         anything not produced by tof::status()
     */
    static const Utf8* statusName(const UInt8 status)
    {
        switch(status)
        {
            case 0: return "OK";
            case 1: return "TOO NOISY";
            case 2: return "NO SIGNAL";
            case 3: return "OUT OF RANGE";
            case 4: return "HARDWARE FAIL";
            case 5: return "WRAPPED TARGET";
            case 6: return "SYNC";
            case 7: return "MERGED PULSE";
            default: return "UNKNOWN";
        }
    }


}
