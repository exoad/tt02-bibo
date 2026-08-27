/*
 * 07 - Reading a voltage
 *
 * TEACHES: the ADC - turning a real-world voltage into a number.
 * NEEDS:   a potentiometer. Without one, see "NO POT?" below.
 *
 * ---------------------------------------------------------------------------
 * WIRING a 10k potentiometer
 *
 *   outer leg 1 --- 3V3(OUT), physical pin 36
 *   middle leg  --- GP26, physical pin 31          <- the wiper, what we read
 *   outer leg 2 --- GND, physical pin 33 (AGND) or 38
 *
 * A pot is a resistor with a slider. The two outer legs sit across 3.3 V and
 * ground; the middle leg picks off whatever fraction of that the knob is at. So
 * turning it sweeps the voltage from 0 V to 3.3 V, and that is what we measure.
 *
 * NO POT? Wire GP26 to 3V3 and read 3.3 V, then to GND and read 0 V. Leave it
 * unconnected and watch it drift - a FLOATING input reads noise, the same
 * lesson as the button in sketch 05.
 *
 * ---------------------------------------------------------------------------
 * ONLY FOUR PINS CAN DO THIS
 *
 * GP26, GP27, GP28 and GP29 - ADC channels 0 to 3. No other pin has an ADC
 * attached, and asking for one is a programming error the hardware cannot
 * report. Note GP28 is the LED pin from the earlier sketches: it is ADC2 as
 * well, but not both at once.
 *
 * 3V3 IS THE CEILING. Never put 5 V on an ADC pin; it is not 5 V tolerant.
 *
 * ---------------------------------------------------------------------------
 * WHAT THE NUMBERS MEAN
 *
 * The reading is 12-bit: 0 to 4095 across 0 V to 3.3 V, so one count is about
 * 0.8 mV. The last bit or two will always jitter - that is normal for any ADC,
 * not a fault, and it is why the print below rounds to two decimals rather than
 * pretending to more precision than exists.
 */

#include "pico2w.h"

#define POT_PIN 26

Int32 main(Void)
{
    serialOpen();
    serialWaitForHost(3000);

    adcOpen(POT_PIN);

    serialPrintLine("raw    volts   bar");

    while(true)
    {
        const UInt16  raw   = adcRead(POT_PIN);
        const Float32 volts = adcReadVolts(POT_PIN);

        /* A crude bar graph, because a moving picture reads far better than a
         * moving number when you are turning a knob. */
        Utf8 bar[21];
        const Int32 filled = (Int32) ((Float32) raw / 4095.0f * 20.0f);
        for(Int32 i = 0; i < 20; ++i)
        {
            bar[i] = (i < filled) ? '=' : ' ';
        }
        bar[20] = '\0';

        serialPrintf("%4u   %.2f   [%s]\n", raw, (Float64) volts, bar);
        sleepMs(200);
    }

    return 0;
}
