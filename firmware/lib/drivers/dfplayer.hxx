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

namespace bibo
{

namespace dfplayer
{

/* ---- commands, as the datasheet numbers them --------------------------- */
#define DFP_CMD_NEXT        0x01u
#define DFP_CMD_PREV        0x02u
#define DFP_CMD_TRACK       0x03u   /* by index across the whole card       */
#define DFP_CMD_VOL_UP      0x04u
#define DFP_CMD_VOL_DOWN    0x05u
#define DFP_CMD_VOLUME      0x06u   /* 0-30                                  */
#define DFP_CMD_EQ          0x07u
#define DFP_CMD_SOURCE      0x09u   /* 2 = the SD card                       */
#define DFP_CMD_SLEEP       0x0Au
#define DFP_CMD_RESET       0x0Cu
#define DFP_CMD_PLAY        0x0Du
#define DFP_CMD_PAUSE       0x0Eu
#define DFP_CMD_FOLDER      0x0Fu   /* hi = folder, lo = track               */
#define DFP_CMD_MP3         0x12u   /* the "mp3" folder, by number           */
#define DFP_CMD_STOP        0x16u

/* Volume is 0-30 in the protocol. THIRTY IS EXTREMELY LOUD - these modules are
 * widely reported as painful well below half, and there is no reason to find
 * that out with your head next to the speaker. Everything here clamps. */
#define DFP_VOLUME_MAX      30u

/* Where a track lives, which decides which command plays it.
 *
 *     mp3/0001.mp3    DFP_CMD_MP3 with 1
 *     01/001.mp3      DFP_CMD_FOLDER with folder 1, track 1
 *
 * The "mp3" folder is the easier of the two and the one to start with: the
 * name is fixed, the numbering is four digits, and the card can hold up to
 * 3000 of them. */

/* The SD card takes a moment to mount after power-on or a reset, and a command
 * sent before it is ready is simply lost - no error, no sound. The datasheet
 * says 1.5 s to 3 s depending on the card. */
#define DFP_BOOT_MS         2000u

typedef struct Bus
{
    uart_inst_t* port;
    Int32        busyPin;   /* pins::NONE if not wired */
} Bus;

/* ---------------------------------------------------------------------------
 * Builds one frame into `out`, which must hold 10 bytes.
 *
 * The checksum is the 16-bit two's complement of the sum of bytes 1..6 - that
 * is, the value which, added to them, gives zero. Computed as a UInt16 so the
 * wrap is defined rather than something the compiler is entitled to have
 * opinions about.
 * ------------------------------------------------------------------------- */
static Void frame(UInt8* out, UInt8 cmd, UInt8 ack, UInt16 param)
{
    const UInt8 paramHi = static_cast<UInt8>((param >> 8) & 0xFFu);
    const UInt8 paramLo = static_cast<UInt8>(param & 0xFFu);

    const UInt16 sum = static_cast<UInt16>(0xFFu + 0x06u + cmd + ack
                                           + paramHi + paramLo);
    const UInt16 chk = static_cast<UInt16>(-static_cast<Int32>(sum));

    out[0] = 0x7Eu;
    out[1] = 0xFFu;
    out[2] = 0x06u;
    out[3] = cmd;
    out[4] = ack;
    out[5] = paramHi;
    out[6] = paramLo;
    out[7] = static_cast<UInt8>((chk >> 8) & 0xFFu);
    out[8] = static_cast<UInt8>(chk & 0xFFu);
    out[9] = 0xEFu;
}

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
static Bool playing(const Bus* bus)
{
    if(bus->busyPin == -1)
    {
        return false;
    }
    return !gpio::read(static_cast<Pin>(bus->busyPin));
}

} /* namespace dfplayer */

} /* namespace bibo */
