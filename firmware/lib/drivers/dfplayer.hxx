/**
 * @file dfplayer.hxx
 * @brief Drives the DFPlayer Mini over UART: the second link in the sound
 * chain.
 *
 * dfplayer_proto.hxx (below this file) encodes the wire format; this file
 * sends and receives it; sound.hxx (above this file) is what a program
 * actually calls to make a sound; sfx.hxx names which numbered file on the
 * card is which clip. Land here to see how a command frame goes out and a
 * query's answer comes back.
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
 *
 * @note On this project's pin map the module's TX/RX live on GP14/GP15 -
 * the same pads that carried the tail lamps before sound needed a UART. See
 * the pin-map banner in lights.hxx for why they moved.
 */
#pragma once

#include "../hal.hxx"
#include "../pins.hxx"

/*
 * The protocol itself lives next door and needs NOTHING from the SDK, so the
 * checksum - the one part of this driver that fails silently - can be tested
 * on the host: firmware\tests\build_dfplayer_test.bat run
 */
#include "dfplayer_proto.hxx"

namespace bibo::dfplayer
{

    /**
     * @brief One DFPlayer connection: the UART port and its optional BUSY
     * pin.
     *
     * STAYS HERE rather than in the protocol header: uart_inst_t is the
     * SDK's, and dragging it next door would cost that header the one
     * property it exists for - compiling on the host, with no board and no
     * SDK.
     */
    struct Bus
    {
        uart_inst_t* port;
        Int32        busyPin;   /* pins::NONE if not wired */
    };

    /**
     * @brief Encodes and sends one command frame, then waits out the
     * module's processing time.
     *
     * The 40 ms afterwards is not politeness. The module processes a frame
     * before it will look at the next one, and back-to-back commands - a
     * volume followed immediately by a play, which is the obvious thing to
     * write - drop the second one often enough to look intermittent.
     *
     * @param bus the connection to send on
     * @param cmd the command byte, one of the DFP_CMD_* values
     * @param param the command's parameter; meaning depends on cmd
     *
     * @note The datasheet asks for a 20 ms gap between commands; 40 ms is
     * cheap and has never been the problem.
     */
    inline Void send(const Bus* bus, const UInt8 cmd, const UInt16 param)
    {
        UInt8 buf[10];
        frame(buf, cmd, 0x00u, param);
        uart::write(bus->port, buf, sizeof(buf));
        timing::ms(40);
    }

    /**
     * @brief Opens the UART link and the BUSY pin, if any. Does NOT reset
     * the module - see reset().
     *
     * @param bus the connection to fill in
     * @param port the SDK UART instance to use
     * @param tx goes to the module's RX; pins::NONE to leave it unconnected
     * @param rx comes from the module's TX; pins::NONE to leave it
     *        unconnected and lose the replies, not the sound
     * @param busyPin the module's BUSY output, or pins::NONE if not wired
     *
     * @note 9600 baud is not configurable on this module.
     * @note On this project's pin map, tx/rx are GP14/GP15 - the pads that
     * carried the tail lamps before sound needed a UART.
     */
    inline Void open(Bus* bus, uart_inst_t* port, const Pin tx, const Pin rx, const Int32 busyPin)
    {
        bus->port    = port;
        bus->busyPin = busyPin;

        uart::open(port, 9600, tx, rx);

        if(busyPin != pins::NONE)
        {
            gpio::open(busyPin, PIN_DIR_IN);

            /* Pulled up because the module drives it LOW to mean "playing"; unattached it would float. */
            gpio::pull(busyPin, PIN_PULL_UP);
        }
    }

    /**
     * @brief Resets the module and waits for the card to mount.
     *
     * Separate from open() because it costs two seconds and not every
     * program wants to pay them - a sketch that is only sending a volume
     * change does not need the card remounted. Any program that is about
     * to PLAY something after power-on does.
     *
     * @param bus the connection to reset
     *
     * @note Blocks for DFP_BOOT_MS (about two seconds) while the card
     * mounts.
     */
    inline Void reset(const Bus* bus)
    {
        uart::drain(bus->port);
        send(bus, DFP_CMD_RESET, 0);
        timing::ms(DFP_BOOT_MS);

        /* The module chatters an init frame on its way up; left in the FIFO it is somebody's stale first byte. */
        uart::drain(bus->port);
    }

    /**
     * @brief Selects the SD card as the playback source.
     *
     * The module usually picks it on its own if it is the only thing
     * present, and "usually" is not a thing to build on.
     *
     * @param bus the connection to send on
     */
    inline Void useCard(const Bus* bus)
    {
        send(bus, DFP_CMD_SOURCE, 0x0002u);
    }

    /**
     * @brief Sets the playback volume.
     *
     * @param bus the connection to send on
     * @param level 0 to 30, clamped to that range
     */
    inline Void volume(const Bus* bus, const UInt8 level)
    {
        send(bus, DFP_CMD_VOLUME,
             static_cast<UInt16>(level > DFP_VOLUME_MAX ? DFP_VOLUME_MAX : level));
    }

    /**
     * @brief Sets the equaliser preset.
     *
     * See the note in dfplayer_proto.hxx: this changes tone, not level,
     * and 30 remains the loudest this module goes.
     *
     * @param bus the connection to send on
     * @param mode the equaliser preset, 0-5, clamped to that range
     */
    inline Void eq(const Bus* bus, const UInt8 mode)
    {
        send(bus, DFP_CMD_EQ,
             static_cast<UInt16>(mode > DFP_EQ_MAX ? DFP_EQ_MAX : mode));
    }

    /**
     * @brief Plays mp3/000N.mp3 by number.
     *
     * @param bus the connection to send on
     * @param track the file number, one-based, matching the filename
     */
    inline Void playMp3(const Bus* bus, const UInt16 track)
    {
        send(bus, DFP_CMD_MP3, track);
    }

    /**
     * @brief Plays NN/TTT.mp3 by folder and track number.
     *
     * @param bus the connection to send on
     * @param folder the folder number, 1-99
     * @param track the track number within that folder, 1-255
     */
    inline Void playFolder(const Bus* bus, const UInt8 folder, const UInt8 track)
    {
        send(bus, DFP_CMD_FOLDER,
             static_cast<UInt16>((static_cast<UInt32>(folder) << 8u)
                                 | static_cast<UInt32>(track)));
    }

    /**
     * @brief Resumes or starts playback of the current track.
     *
     * @param bus the connection to send on
     */
    inline Void play(const Bus* bus)
    {
        send(bus, DFP_CMD_PLAY, 0);
    }

    /**
     * @brief Pauses playback.
     *
     * @param bus the connection to send on
     */
    inline Void pause(const Bus* bus)
    {
        send(bus, DFP_CMD_PAUSE, 0);
    }

    /**
     * @brief Stops playback.
     *
     * @param bus the connection to send on
     */
    inline Void stop(const Bus* bus)
    {
        send(bus, DFP_CMD_STOP, 0);
    }

    /**
     * @brief Asks the module something and waits for the answer.
     *
     * THE ONLY PLACE THIS DRIVER READS. Everything else is fire and
     * forget, which is why ACK is off: a reply nobody reads sits in the
     * FIFO until it is mistaken for the answer to a later question. A
     * query must read, so it drains first for exactly that reason.
     *
     * FRAMES ARE MATCHED ON THE COMMAND BYTE. The module volunteers a
     * track-finished frame whenever a track ends, so whatever is sitting
     * in the FIFO may be about something else entirely; anything that is
     * not the answer to THIS question is skipped rather than returned as
     * the value.
     *
     * @param bus the connection to query
     * @param cmd the query command byte, DFP_Q_FILES or DFP_Q_TRACK
     * @param out receives the answer's parameter; left untouched on
     *        failure
     * @return true when a valid, matching reply arrived; false when
     *         nothing valid arrived, which is a USEFUL answer on a bench
     *         rather than a failure - it separates a module that is alive
     *         and listening from a dead one, a missing card, or an RX
     *         wire on the wrong pad, none of which the fire-and-forget
     *         path can tell apart, because none of them talks back
     */
    inline Bool query(const Bus* bus, const UInt8 cmd, UInt16* out)
    {
        uart::drain(bus->port);

        UInt8 sent[10];
        frame(sent, cmd, 0x00u, 0);
        uart::write(bus->port, sent, sizeof(sent));

        /*
         * A 10-byte reply at 9600 baud is about 10 ms. 40 gives the module time
         * to think and still fails fast enough not to hang the console.
         */

        UInt8 f[10];
        Size  n = 0;

        for(Int32 guard = 0; guard < 64; ++guard)
        {
            constexpr UInt32 SLICE_US = 40000u;
            const Int32 b = uart::readByte(bus->port, SLICE_US);
            if(b < 0)
            {
                return false;   /* silence */
            }

            /* Resynchronise on the start byte: half a leftover frame would shift every byte after it. */
            if(n == 0 && b != 0x7E)
            {
                continue;
            }

            f[n] = static_cast<UInt8>(b);
            ++n;
            if(n < 10)
            {
                continue;
            }
            n = 0;

            if(f[9] != 0xEF)
            {
                continue;   /* not a frame after all */
            }

            /* The invariant the host test asserts: body and checksum word cancel in 16 bits. */
            UInt16 sum = 0;
            for(Int32 i = 1; i <= 6; ++i)
            {
                sum = static_cast<UInt16>(sum + f[i]);
            }
            if(const auto chk = static_cast<UInt16>((static_cast<UInt32>(f[7]) << 8u) | static_cast<UInt32>(f[8])); static_cast<UInt16>(sum + chk) != 0u)
            {
                continue;   /* corrupted - keep looking */
            }
            if(f[3] != cmd)
            {
                continue;   /* somebody else's answer */
            }
            *out = static_cast<UInt16>((static_cast<UInt32>(f[5]) << 8u) | static_cast<UInt32>(f[6]));
            return true;
        }
        return false;
    }

    /**
     * @brief Asks how many files the card holds.
     *
     * THE CARD IS THE SOURCE OF TRUTH. Nothing in this firmware keeps a
     * list of tracks, so adding one to the card is the whole of adding one
     * - there is no table here to keep in step and no build to redo.
     *
     * @param bus the connection to query
     * @param out receives the file count; left untouched on failure
     * @return true when the module answered; false otherwise
     */
    inline Bool fileCount(const Bus* bus, UInt16* out)
    {
        return query(bus, DFP_Q_FILES, out);
    }

    /**
     * @brief Whether this bus has a BUSY line at all.
     *
     * SEPARATE FROM playing(), because "not playing" and "cannot tell" are
     * different answers and a caller that shows them the same way is
     * lying in one of the two cases. main.cxx used to ask
     * pins::active().soundBusy to work this out - the app reaching past
     * the driver for something the Bus already knows.
     *
     * Same shape as tft::hasBacklight(), and for the same reason: the
     * wire is either there or it is not, and that is the driver's fact to
     * report.
     *
     * @param bus the connection to ask
     * @return true when a BUSY pin was given to open()
     */
    [[nodiscard]] static Bool hasBusy(const Bus* bus)
    {
        return bus->busyPin != pins::NONE;
    }

    /**
     * @brief Whether a track is playing right now.
     *
     * Reads the BUSY pin, which is the module telling the truth about
     * itself. With no BUSY pin wired this returns false - "not playing" -
     * and says so here because a caller that waits on it would otherwise
     * hang forever on a wire that was never fitted.
     *
     * The serial reply is NOT used for this even when RX is connected: it
     * says a command was accepted, which is a different claim from a
     * track still being audible.
     *
     * @param bus the connection to ask
     * @return true while BUSY reports the module is playing; false when
     *         idle or when no BUSY pin is wired
     */
    [[nodiscard]] static Bool playing(const Bus* bus)
    {
        if(!hasBusy(bus))
        {
            return false;
        }
        return !gpio::read(bus->busyPin);
    }

}
