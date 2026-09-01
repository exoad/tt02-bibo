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
 *   BUSY              GP9               LOW while playing, pulled up here
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

/*
 * ---- the wiring, declared here and installed at startup -----------------
 *
 * A SKETCH DECLARES ITS OWN MAP. pins::car() is the vehicle's, and a breadboard
 * experiment must not be able to change what the finished car believes about
 * itself - so this builds its own and installs it, and the two cannot affect
 * each other.
 *
 * It is the same mechanism main.cxx uses and it is checked the same way: two
 * roles on one pad and pins::begin() refuses, naming both. That check matters
 * more here than in the car, whose map is proved at compile time - a sketch's
 * map is written fresh against whatever is plugged in this afternoon.
 *
 * These three are the only roles this program has. Everything else in the map
 * stays NONE, which is why a Map defaults to nothing wired.
 */
#define SOUND_TX    14
#define SOUND_RX    15

/*
 * GP9, physical pin 12. The one pad claimed by nothing - GP2/3/6/7/8 are
 * earmarked for the permanent lamp move, GP16-19 are SPI, GP20/21 the display
 * and GP22 the SD card's chip select.
 *
 * LOW WHILE PLAYING, so the firmware pulls it up: with nothing attached the pad
 * would float and read as playing about half the time.
 */
#define SOUND_BUSY  9

/* 0-30. Start low. Really. */
#define VOLUME      8

/* mp3/0001.mp3 */
#define TRACK       1

/*
 * How long to leave between plays. Longer than the track, or the next play
 * command interrupts the one still sounding - which is a legitimate thing to
 * do and a confusing thing to do by accident.
 */
#define GAP_MS      6000

/*
 * `int`, NOT Int32, and this is the one place in the tree where the vocabulary
 * does not apply. On arm-none-eabi int32_t is `long int` - the same size, the
 * same representation, a different type as far as the language is concerned -
 * and the compiler rejects it outright:
 *
 *     error: '::main' must return 'int'
 *
 * It builds clean on the host, where MSVC's int32_t IS `int`, so the host
 * suites cannot catch this and only a board build will. main's signature is the
 * C runtime's contract rather than this project's vocabulary, so it is spelled
 * the runtime's way - same as app/main.cxx and range-view.cxx.
 */
int main(Void)
{
    /*
     * FIRST, always. This brings up USB CDC; without it the board runs fine
     * and never enumerates, and the only way back in is BOOTSEL.
     */

    /*
     * Not required, but this sketch is worth watching, and the console is the
     * only way to tell "the module never answered" from "the speaker is
     * silent". Two seconds, then carry on regardless - the sound should play
     * whether or not anybody is looking.
     */
    static_cast<Void>(serial::waitForHost(2000));

    serial::printLine("");
    serial::printLine("speaker: DFPlayer Mini on UART0");
    serial::printf("  TX  GP%d  -> module RX (through 1k)\n", SOUND_TX);
    serial::printf("  RX  GP%d  <- module TX\n", SOUND_RX);
    serial::printLine("");

    led::open();

    /*
     * WHAT IS WIRED WHERE, before anything is opened. Everything below reads
     * the installed map rather than these three numbers.
     */
    pins::Map wiring;
    wiring.soundTx = SOUND_TX;
    wiring.soundRx = SOUND_RX;
    wiring.soundBusy = SOUND_BUSY;

    /*
     * Said out loud and then STOPPED. Carrying on with no map would open a
     * UART on nothing and sit there looking like a wiring fault, which is
     * the exact confusion this file is trying to avoid. This blink is
     * where boot::halt() came from.
     */
    if(!boot::begin(wiring))
    {
        boot::halt();
    }

    dfplayer::Bus sound;
    dfplayer::open(
        &sound,
        uart0,
        pins::active().soundTx,
        pins::active().soundRx,
        pins::active().soundBusy
    );

    /*
     * Two seconds while the card mounts. A play sent before this is simply
     * lost - no error, no sound - which is the single most common reason a
     * first DFPlayer bring-up looks dead.
     */
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

        /*
         * WAIT FOR THE MODULE, not for the clock.
         *
         * This loop used to count out GAP_MS because BUSY was not wired and
         * there was nothing else to wait on. A fixed gap is a guess: too short
         * and the next play cuts the last one off mid-word, too long and the
         * car stands there silent. Now the module says when it is done.
         *
         * BUSY is LOW WHILE PLAYING and the pad is pulled up, so a module that
         * never starts reads "idle" immediately - which is why there is a grace
         * period first. Without it a failed play would spin this loop as fast
         * as the link allows.
         */
        timing::ms(400);

        UInt32 waited = 0;
        while(dfplayer::playing(&sound) && waited < 60000)
        {
            /*
             * The LED is the whole diagnostic with no console attached: it
             * blinks while a track is sounding. Steady-off means the module
             * says it is idle; a blink that STOPS mid-track is the brownout
             * described at the top of this file, not a crash.
             */
            led::write(true);
            timing::ms(60);
            led::write(false);
            timing::ms(140);
            waited += 200;

            /*
             * Anything the module volunteers, echoed. With ACK off it only
             * speaks when a track finishes - but seeing bytes arrive at all is
             * proof the RX wire and its direction are right, which is
             * otherwise invisible.
             */
            while(uart::readable(uart0))
            {
                const Int32 b = uart::readByte(uart0, 0);
                if(b >= 0)
                {
                    serial::printf("  rx %02X\n", b);
                }
            }
        }

        if(waited >= 60000)
        {
            /*
             * A track that never ends is a BUSY line that is not telling the
             * truth - unwired, on the wrong pad, or the module never started.
             * Said plainly rather than hanging here forever.
             */
            serial::printLine("WARN busy never went idle - check GP9");
        }

        serial::printf("  done after %u ms\n", waited);
        timing::ms(GAP_MS);
    }
}
