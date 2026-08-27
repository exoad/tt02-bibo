/*
 * ===========================================================================
 *  START HERE
 * ===========================================================================
 *
 * These sketches go in order and each one adds a single idea. Open one, press
 * Build & Flash, watch what it does, then change a number and flash it again.
 * Changing something and seeing what breaks teaches far more than reading does.
 *
 * ---------------------------------------------------------------------------
 * THE ORDER, AND WHAT EACH NEEDS
 *
 *   01-led-onboard    nothing        the LED on the board is not a GPIO
 *   02-blink          nothing        set up once, then loop forever
 *   03-breadboard-led LED + resistor what a GPIO actually is
 *   04-serial-print   nothing        printing, and why the first lines vanish
 *   05-button         a jumper wire  inputs, pull-ups, why pressed reads false
 *   06-fade-pwm       LED + resistor PWM: brightness from an on/off pin
 *   07-analog-read    a pot, or not  the ADC, and what a floating pin does
 *   08-two-leds       2 LEDs + 2 R   arrays of pins instead of copy-paste
 *   09-turn-signal    2 LEDs + 2 R   timing WITHOUT sleep - the important one
 *   10-servo-sweep    a servo        the car's actual next step. Read its
 *                                    safety notes before wiring anything.
 *
 * NO BUTTON? 05 works with a jumper wire touched between two pins. A wire is a
 * button. NO POTENTIOMETER? 07 says what to do instead.
 *
 * ---------------------------------------------------------------------------
 * THE TWO RULES THAT WILL SAVE YOU AN EVENING
 *
 * 1. CALL serialOpen() FIRST, IN EVERY SKETCH, even one that prints nothing.
 *
 *    It starts the USB stack. A sketch without it runs perfectly and never
 *    appears on USB - no COM port, nothing for the flasher to reboot into the
 *    bootloader - and the only way to flash again is holding BOOTSEL while
 *    plugging the cable in. It reads exactly like a dead board. It is not.
 *
 * 2. AN LED ALWAYS NEEDS A RESISTOR, 220R to 1k.
 *
 *    An LED is not a resistor. Past its forward voltage it draws whatever the
 *    source will give, so a bare LED across a 3.3 V pin is a short across that
 *    pin. The resistor works on either side of the LED.
 *
 * ---------------------------------------------------------------------------
 * IF THE BOARD STOPS APPEARING
 *
 * Hold BOOTSEL while plugging the USB in. The RP2350's bootloader lives in mask
 * ROM and nothing you flash can damage it, so this always works. The board
 * shows up as a drive called RP2350 and you can flash it again from there.
 *
 * ---------------------------------------------------------------------------
 * PINS TO LEAVE ALONE once the car is wired - see docs/wiring.md
 *
 *   GP0, GP1      servo and ESC signal
 *   GP4, GP5      I2C for the ToF sensors and IMU
 *   GP10 - GP13   ToF XSHUT lines
 *   GP15          wheel encoder
 *   GP16 - GP19   SPI for the SD card
 *
 * Free and used by these sketches: GP21, GP22, GP26, GP27, GP28.
 *
 * Everything the wrapper offers is in the editor's completion list - start
 * typing gpio, led, serial, pwm, adc, servo or watchdog and it will show you
 * the signature and what it does.
 */

#include "pico2w.h"

Int32 main(Void)
{
    serialOpen();

    if(ledOpen())
    {
        ledWrite(true);
    }

    while(true)
    {
        serialPrintLine("open a numbered sketch to begin - 01-led-onboard is first");
        sleepMs(3000);
    }

    return 0;
}
