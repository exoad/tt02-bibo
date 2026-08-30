/*
 * The speaker, on a breadboard: bring up the DFPlayer Mini and play a track.
 *
 * ---------------------------------------------------------------------------
 * WIRING
 *
 *   DFPlayer          Pico              why
 *   VCC               VBUS (5 V)        100 uF across VCC/GND, see below
 *   GND               GND               must be common, even on its own supply
 *   RX                GP14 via 1k       GP14 is UART0 TX on this chip
 *   TX                GP15              GP15 is UART0 RX
 *   SPK_1             speaker +
 *   SPK_2             speaker -         do NOT ground either one
 *
 * THE TWO DATA WIRES ARE NOT INTERCHANGEABLE and getting them the other way
 * round is silent, not broken. On RP2350 the silicon fixes the directions:
 * GPIO14 is UART0_TX (funcsel 0x0b) and GPIO15 is UART0_RX. Swap them and you
 * have two outputs facing each other and two inputs facing each other -
 * nothing is damaged, the port opens, writes succeed, and no sound ever comes
 * out.
 *
 * There is a second trap on this pair, handled in hal.hxx: their UART DATA
 * function is GPIO_FUNC_UART_AUX, not the GPIO_FUNC_UART every example uses.
 * Funcsel 2 on GP14/GP15 is CTS/RTS - flow control - so the usual call sends
 * every byte out on a handshake line nobody is listening to. uart::open()
 * picks the right one from the pin.
 *
 * ---------------------------------------------------------------------------
 * THE CARD
 *
 * One file, mp3/0001.mp3, so this plays with DFP_CMD_MP3 and track 1. The
 * "mp3" folder is the easy addressing mode: fixed folder name, four-digit
 * numbering, up to 3000 files. Folders named 01..99 with 001.mp3 inside are
 * the other mode - dfplayer::playFolder - and neither is better; this one has
 * fewer things to get wrong on the first try.
 *
 * ---------------------------------------------------------------------------
 * VOLUME
 *
 * The protocol takes 0-30. These modules are widely reported as painfully loud
 * well below half, so this starts at 8 - about a quarter - and that is still
 * likely to be more than enough in a room. Turn it UP if you want it louder;
 * do not start at 20 to find out.
 *
 * The amplifier is class-D and its current follows the audio, so it peaks well
 * above its average. On VBUS - laptop USB through the Pico's polyfuse - a loud
 * passage can brown out the rail and RESET THE PICO mid-track, which looks
 * exactly like a firmware crash. If this sketch restarts when the sound gets
 * loud, that is the supply and not the code: 100 uF across the module's
 * VCC/GND, or give it its own 5 V.
 */

#include "../lib/bibo.hxx"

using namespace bibo;

/* ---- the wiring, in one place ------------------------------------------
 *
 * A sketch declares its own pins rather than reading lib/pins.hxx. That file
 * is the CAR's map, and a breadboard experiment should not have to edit the
 * car to try something. What it should do is read it first - and this pair is
 * why pins.hxx now says the tail lamps are unwired: GP14/GP15 were TAIL_R and
 * TAIL_L, and sound took them because they are the only UART0 pins that do not
 * cost the servo, the ESC or the front indicators. */
#define SOUND_TX    14
#define SOUND_RX    15

/* Not wired yet. With no BUSY line the module cannot tell us when a track
 * ends, so this sketch waits by the clock instead - see the loop. */
#define SOUND_BUSY  (-1)

/* 0-30. Start low. Really. */
#define VOLUME      8

/* mp3/0001.mp3 */
#define TRACK       1

/* How long to leave between plays. Longer than the track, or the next play
 * command interrupts the one still sounding - which is a legitimate thing to
 * do and a confusing thing to do by accident. */
#define GAP_MS      6000

int main(Void)
{
    /* FIRST, always. This brings up USB CDC; without it the board runs fine
     * and never enumerates, and the only way back in is BOOTSEL. */
    serial::open();

    /* Not required, but this sketch is worth watching, and the console is the
     * only way to tell "the module never answered" from "the speaker is
     * silent". Two seconds, then carry on regardless - the sound should play
     * whether or not anybody is looking. */
    static_cast<Void>(serial::waitForHost(2000));

    serial::printLine("");
    serial::printLine("speaker: DFPlayer Mini on UART0");
    serial::printf("  TX  GP%d  -> module RX (through 1k)\n", SOUND_TX);
    serial::printf("  RX  GP%d  <- module TX\n", SOUND_RX);
    serial::printLine("");

    led::open();

    dfplayer::Bus sound;
    dfplayer::open(&sound, uart0, SOUND_TX, SOUND_RX, SOUND_BUSY);

    /* Two seconds while the card mounts. A play sent before this is simply
     * lost - no error, no sound - which is the single most common reason a
     * first DFPlayer bring-up looks dead. */
    serial::printLine("resetting the module, waiting for the card...");
    dfplayer::reset(&sound);

    dfplayer::useCard(&sound);
    dfplayer::volume(&sound, VOLUME);
    serial::printf("volume %d of %d\n", VOLUME, DFP_VOLUME_MAX);

    UInt32 plays = 0;

    while(true)
    {
        ++plays;
        serial::printf("play %u: mp3/%04d.mp3\n", plays, TRACK);

        dfplayer::playMp3(&sound, TRACK);

        /* The LED is the whole diagnostic when there is no console attached
         * and no sound coming out: if it is blinking, the loop is running and
         * the frames are going out, so the problem is downstream of the Pico -
         * the wiring, the card, the supply or the speaker.
         *
         * If it is NOT blinking, and it was, that is the brownout described at
         * the top of this file. */
        for(Int32 i = 0; i < GAP_MS / 500; ++i)
        {
            led::write(true);
            timing::ms(60);
            led::write(false);
            timing::ms(440);

            /* Anything the module says, echoed. Nothing here depends on it -
             * with ACK off the module only volunteers a frame when a track
             * finishes - but seeing bytes arrive at all is proof the RX wire
             * and its direction are right, which is otherwise invisible. */
            while(uart::readable(uart0))
            {
                const Int32 b = uart::readByte(uart0, 0);
                if(b >= 0)
                {
                    serial::printf("  rx %02X\n", b);
                }
            }
        }
    }
}
