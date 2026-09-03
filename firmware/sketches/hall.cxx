/*
 * A3144 Hall switch on GP15. Prints what the pin reads.
 *
 *   VCC   VBUS (5 V)   the A3144 needs 4.5 V minimum, so not 3V3
 *   GND   GND
 *   OUT   GP15
 *
 * The output is OPEN COLLECTOR: it can pull low and it cannot drive high, so
 * without a pull-up the pin floats and reads as noise. The internal pull-up
 * below is the pull-up, and because it pulls to 3V3 the pin never sees more
 * than 3.3 V even though the sensor runs at 5 V.
 *
 * ACTIVE LOW. 0 means a magnet is present, 1 means it is not. Unipolar too -
 * only one pole triggers it, so if nothing happens, turn the magnet over.
 */
#include "../lib/bibo.hxx"

using namespace bibo;

#define HALL_PIN 15

PROGRAM
{
    serial::open();
    static_cast<Void>(serial::waitForHost(2000));
    const Bool ledUp = led::open();
    led::write(true);
    gpio::open(HALL_PIN, PIN_DIR_IN);
    gpio::pull(HALL_PIN, PIN_PULL_UP);
    serial::printf("hall on GP%d, 0 = magnet, led=%s\n", HALL_PIN, ledUp ? "on" : "FAILED to open");
    Bool last = !gpio::read(HALL_PIN);
    Int32 edges = 0;
    FOREVER
    {
        const Bool now = gpio::read(HALL_PIN);
        if(now != last)
        {
            ++edges;
            serial::printf("%d  %s  (edge %d)\n", now ? 1 : 0, now ? "-" : "MAGNET", edges);
            last = now;
        }
        /*
         * Inverted because the pin is ACTIVE LOW: 0 is a magnet present.
         */
        led::write(!now);
        timing::ms(5);
    }
}
