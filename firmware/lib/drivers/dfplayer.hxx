/* ---------------------------------------------------------------------------
 * dfplayer - the DFPlayer Mini, a serial MP3 player on a DIP-16 module.
 *
 * It plays a file off a microSD card when you send it a ten-byte frame at 9600
 * baud. That is the entire interface: there is no filesystem to drive, no
 * decoding to do and no audio to clock out. The car says "track 1" and the
 * module does the rest, which is why it is worth having at all.
 *
 * ---------------------------------------------------------------------------
 * WIRING, and the two things that go wrong.
 *
 *   VCC        3.2-5.0 V. NOT off a laptop's USB through the Pico if the
 *              speaker is doing real work - see the note on current below.
 *   GND        common with the Pico's. If the module is on its own supply and
 *              the grounds are not tied, the serial link has no reference and
 *              the module ignores everything or hears noise.
 *   RX         <- the Pico's TX, THROUGH 1k. The module's RX has no series
 *              protection and is documented as noise-sensitive.
 *   TX         -> the Pico's RX. Optional: every command below works without
 *              it. You lose the replies, not the sound.
 *   SPK_1/2    the speaker, directly. A bridged output - do NOT also use
 *              DAC_L/DAC_R, and do not ground either speaker terminal.
 *   BUSY       the module's own output, LOW WHILE PLAYING.
 *
 * CURRENT IS THE ONE THAT BITES. The on-board amplifier is class-D and its
 * draw follows the audio, so it peaks well above its average. Off a Pico's
 * VBUS - which is laptop USB through the board's polyfuse - a loud passage can
 * brown out the rail and RESET THE PICO mid-track. That reads exactly like a
 * firmware bug and is not one. 100 uF across VCC/GND is the minimum; its own
 * supply is better.
 *
 * ---------------------------------------------------------------------------
 * THE PROTOCOL.
 *
 *     7E FF 06 CMD ACK PARAM_HI PARAM_LO SUM_HI SUM_LO EF
 *
 * 0x7E starts and 0xEF ends. 0xFF is the version and 0x06 is the length of the
 * bytes the checksum covers. The checksum is the two's complement negation of
 * the sum of those six - see frame() - and the module silently ignores a frame
 * whose sum is wrong, which is worth knowing because a wrong checksum and a
 * disconnected wire look identical from here.
 *
 * ACK asks the module to answer. Left at 0 by default: this driver is fire and
 * forget, and a reply that nobody reads sits in the receive FIFO until it is
 * mistaken for the answer to a later question.
 * ------------------------------------------------------------------------- */
#pragma once

#include "../hal.hxx"

/* The protocol itself - frames, checksums, command numbers - lives next door
 * and needs NOTHING from the SDK, so it can be tested on the host. The one
 * part of this driver that fails silently is the checksum, and a checksum that
 * can only be exercised by flashing a microcontroller is a checksum nobody
 * exercises. Same split, and the same reason, as lib/text.hxx.
 *
 * firmware	estsuild_dfplayer_test.bat run
 */
#include "dfplayer_proto.hxx"

namespace bibo
{

  namespace dfplayer
  {

    /* The port and the BUSY pin. STAYS HERE rather than in the protocol
     * header: uart_inst_t is the SDK's, and dragging it next door would
     * cost that header the one property it exists for - compiling on the
     * host, with no board and no SDK. */
    typedef struct Bus
    {
        uart_inst_t* port;
        Int32        busyPin;   /* pins::NONE if not wired */
    } Bus;

    /* Sends one command.
     *
     * The 40 ms afterwards is not politeness. The module processes a frame before
     * it will look at the next one, and back-to-back commands - a volume followed
     * immediately by a play, which is the obvious thing to write - drop the second
     * one often enough to look intermittent. The datasheet asks for 20 ms; 40 is
     * cheap and has never been the problem. */
    static Void send(const Bus* bus, UInt8 cmd, UInt16 param)
    {
        UInt8 buf[10];
        frame(buf, cmd, 0x00u, param);
        uart::write(bus->port, buf, sizeof(buf));
        timing::ms(40);
    }

    /* ---------------------------------------------------------------------------
     * Brings the link up. Does NOT reset the module - see below.
     *
     * `tx` goes to the module's RX and `rx` comes from its TX; either may be
     * pins::NONE. 9600 baud is not configurable on this module.
     * ------------------------------------------------------------------------- */
    static Void open(Bus* bus, uart_inst_t* port, Pin tx, Pin rx, Int32 busyPin)
    {
        bus->port    = port;
        bus->busyPin = busyPin;

        uart::open(port, 9600, tx, rx);

        if(busyPin != -1)
        {
            gpio::open(static_cast<Pin>(busyPin), PIN_DIR_IN);

            /* Pulled up because the module drives it LOW to mean "playing". With
             * nothing attached the pin would float and read as playing about half
             * the time. */
            gpio::pull(static_cast<Pin>(busyPin), PIN_PULL_UP);
        }
    }

    /* Resets the module and waits for the card to mount.
     *
     * Separate from open() because it costs two seconds and not every program
     * wants to pay them - a sketch that is only sending a volume change does not
     * need the card remounted. Any program that is about to PLAY something after
     * power-on does. */
    static Void reset(const Bus* bus)
    {
        uart::drain(bus->port);
        send(bus, DFP_CMD_RESET, 0);
        timing::ms(DFP_BOOT_MS);

        /* The module chatters an init frame on its way up. Nothing here reads
         * replies, so leaving it in the FIFO would mean the first byte anybody
         * ever reads is stale. */
        uart::drain(bus->port);
    }

    /* Selects the SD card as the source. The module usually picks it on its own if
     * it is the only thing present, and "usually" is not a thing to build on. */
    static Void useCard(const Bus* bus)
    {
        send(bus, DFP_CMD_SOURCE, 0x0002u);
    }

    /* 0 to 30, clamped. */
    static Void volume(const Bus* bus, UInt8 level)
    {
        send(bus, DFP_CMD_VOLUME,
             static_cast<UInt16>(level > DFP_VOLUME_MAX ? DFP_VOLUME_MAX : level));
    }

    /* Plays mp3/000N.mp3. One-based, matching the filename. */
    static Void playMp3(const Bus* bus, UInt16 track)
    {
        send(bus, DFP_CMD_MP3, track);
    }

    /* Plays NN/TTT.mp3 - folder 1-99, track 1-255. */
    static Void playFolder(const Bus* bus, UInt8 folder, UInt8 track)
    {
        send(bus, DFP_CMD_FOLDER,
             static_cast<UInt16>((static_cast<UInt16>(folder) << 8) | track));
    }

    static Void play(const Bus* bus)
    {
        send(bus, DFP_CMD_PLAY, 0);
    }

    static Void pause(const Bus* bus)
    {
        send(bus, DFP_CMD_PAUSE, 0);
    }

    static Void stop(const Bus* bus)
    {
        send(bus, DFP_CMD_STOP, 0);
    }

    /* ---------------------------------------------------------------------------
     * True while a track is playing.
     *
     * Reads the BUSY pin, which is the module telling the truth about itself. With
     * no BUSY pin wired this returns false - "not playing" - and says so here
     * because a caller that waits on it would otherwise hang forever on a wire
     * that was never fitted.
     *
     * The serial reply is NOT used for this even when RX is connected: it says a
     * command was accepted, which is a different claim from a track still being
     * audible.
     * ------------------------------------------------------------------------- */
    [[nodiscard]] static Bool playing(const Bus* bus)
    {
        if(bus->busyPin == -1)
        {
            return false;
        }
        return !gpio::read(static_cast<Pin>(bus->busyPin));
    }

  } /* namespace dfplayer */

} /* namespace bibo */
