/* ---------------------------------------------------------------------------
 * dfplayer_proto - the DFPlayer Mini's wire format, and nothing else.
 *
 * SEPARATE FROM dfplayer.hxx SO IT CAN BE TESTED ON THE HOST. This half needs
 * only types.hxx - no UART, no GPIO, no SDK - and that is a property worth
 * keeping for exactly the reason lib/text.hxx says it: a thing that can only be
 * exercised by flashing a microcontroller is a thing nobody exercises.
 *
 * It matters more here than most places. A frame whose checksum is wrong is
 * IGNORED SILENTLY by the module - no error, no reply, no sound - which is
 * indistinguishable from a disconnected wire, a wrong pin, an unmounted card or
 * a dead speaker. Of every failure this driver can have, the checksum is the
 * one that cannot be told apart from the others by looking, so it is the one
 * that has to be settled off the bench.
 *
 *     firmware	estsuild_dfplayer_test.bat run
 *
 * ---------------------------------------------------------------------------
 * THE FRAME
 *
 *     7E FF 06 CMD ACK PARAM_HI PARAM_LO SUM_HI SUM_LO EF
 *      0  1  2  3   4      5        6       7      8    9
 *
 * 0x7E starts and 0xEF ends. 0xFF is the version, 0x06 the number of bytes the
 * checksum covers - that is bytes 1 through 6, version through parameter.
 *
 * The checksum is the two's complement NEGATION of the sum of those six, so the
 * defining property is that bytes 1..8 add to zero in 16 bits. That is what the
 * test asserts, because it catches a byte-order mistake as well as an
 * arithmetic one - swapping SUM_HI and SUM_LO still "looks like a checksum".
 * ------------------------------------------------------------------------- */
#pragma once

#include "../types.hxx"

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

    /* ---- queries, which ANSWER ------------------------------------------
     *
     * The module replies to these with a frame of the same shape carrying the
     * value. Everything above it simply obeys. */
#define DFP_Q_FILES         0x48u   /* how many files on the card */
#define DFP_Q_TRACK         0x4Cu   /* which one is playing now   */

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

  } /* namespace dfplayer */

} /* namespace bibo */
