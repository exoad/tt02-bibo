/*
 * ULN2803 channel 1 driven from GP15, with an LED on it. Blinks.
 *
 *   Pico GP15  ->  IN1  (pin 1)
 *   Pico GND   ->  GND  (pin 9)      the grounds MUST be common
 *   +V         ->  LED anode
 *   LED cathode -> resistor -> OC1 (pin 18)
 *
 * ---------------------------------------------------------------------------
 * IT SINKS. IT DOES NOT SOURCE. This is the whole thing to get right.
 *
 * The ULN2803 is eight Darlington pairs with their emitters on GND. An output
 * pin does exactly one thing: pull itself to ground, or let go. It can never
 * push current OUT. So the load hangs BETWEEN +V AND THE OUTPUT - the LED's
 * anode goes to the supply and its cathode comes back down to OC1 through a
 * resistor.
 *
 * Wiring it the intuitive way - OC1 to the LED to ground, as if the chip were a
 * bigger GPIO - lights nothing, ever, with no clue as to why. That is the
 * commonest way this part is wired wrong and it is why the map above is written
 * out rather than assumed.
 *
 * The LOGIC is not inverted even though the output is: IN1 high turns the
 * Darlington on, which pulls OC1 to ground, which completes the LED's circuit.
 * High means lit. The inversion is in the current, not in the sense.
 *
 * ---------------------------------------------------------------------------
 * WHAT ELSE IS WORTH KNOWING
 *
 * A saturated Darlington drops about 0.9-1.1 V rather than the ~0.2 V a plain
 * transistor would, so size the resistor against (+V - Vf - 1.0), not
 * (+V - Vf). At 5 V with a red LED that is roughly 5 - 2 - 1 = 2 V across it.
 *
 * COM (pin 10) is the common cathode of the built-in flyback diodes. It matters
 * for relays, solenoids and motors; an LED needs nothing there, so leaving it
 * unconnected is correct here rather than an omission.
 *
 * GP15 is the pin the Hall sketch used as an INPUT with a pull-up. Here it is an
 * output and the pull-up is neither set nor wanted.
 *
 * ---------------------------------------------------------------------------
 * THE ONBOARD LAMP BLINKS IN STEP, on purpose. If the board's own LED is
 * blinking and the one on the breadboard is not, the program is running and the
 * fault is in the wiring or the chip - which is a completely different search
 * from "did it flash at all".
 */
#include "../lib/bibo.hxx"

using namespace bibo;

#define ULN_PIN 15

#define ON_MS  400u
#define OFF_MS 400u

PROGRAM
{
    serial::open();
    static_cast<Void>(serial::waitForHost(2000));

    /*
     * Reported rather than assumed: on the Pico 2 W the lamp is on the CYW43439
     * and bringing it up can fail. A dark onboard lamp that failed to open and a
     * dark onboard lamp because the program never started look identical, and
     * the whole point of blinking it here is to tell those apart.
     */
    const Bool ledUp = led::open();

    gpio::open(ULN_PIN, PIN_DIR_OUT);
    gpio::write(ULN_PIN, false);

    serial::printf(
        "uln2803 on GP%d, high = lit, onboard lamp %s\n",
        ULN_PIN,
        ledUp ? "in step" : "FAILED to open"
    );

    Int32 n = 0;
    FOREVER
    {
        gpio::write(ULN_PIN, true);
        led::write(true);
        serial::printf("%d  on\n", n);
        timing::ms(ON_MS);

        gpio::write(ULN_PIN, false);
        led::write(false);
        timing::ms(OFF_MS);

        ++n;
    }
}
