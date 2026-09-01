/*
 * The DFPlayer Mini wire format, in lib/drivers/dfplayer_proto.hxx.
 *
 *   firmware\tests\build_dfplayer_test.bat run
 *
 * THIS TEST EXISTS BECAUSE THE FAILURE IS SILENT. A frame whose checksum is
 * wrong is discarded by the module without a reply, an error or a sound - which
 * is exactly what a swapped data wire looks like, and an unmounted SD card, and
 * a dead speaker, and a supply brownout. On a bench those five are one symptom.
 * The checksum is the only one of them that can be settled without hardware, so
 * it is settled here and crossed off the list before anybody starts guessing.
 *
 * The defining property is that BYTES 1..6 PLUS THE CHECKSUM WORD CANCEL in 16
 * bits: the checksum is the two's complement negation of that body, so adding
 * it back gives zero. Asserting that rather than comparing against captured
 * frames is deliberate - it catches a byte-order mistake too. SUM_HI and SUM_LO
 * written the wrong way round still looks like a plausible checksum, and would
 * pass any test that only checked "is there a checksum in there".
 *
 * The checksum is a WORD and has to be read as one. Adding f[7] and f[8]
 * separately - which is what this test did at first - reports a correct frame
 * as broken, and would have sent somebody to a breadboard looking for a fault
 * in the wiring.
 *
 * Compiled for the HOST, not the board: dfplayer_proto.hxx needs only shared.hxx.
 *
 * Exits 0 on PASS, 1 on FAIL.
 */

#include "../lib/drivers/dfplayer_proto.hxx"

#include <stdio.h>

using namespace bibo;

static Int32 failures = 0;
static Int32 checks = 0;

/**
 * @brief Records a pass/fail check and prints its outcome.
 *
 * @param ok true if the property being checked held
 * @param what the description printed alongside the outcome
 */
static Void check(Bool ok, CharSeq what)
{
    ++checks;
    if(ok)
    {
        printf("  ok    %s\n", what);
    }
    else
    {
        printf("  FAIL  %s\n", what);
        ++failures;
    }
}

/**
 * @brief Bytes 1..6 plus the checksum word, summed as 16-bit.
 *
 * @param f the ten-byte frame to check
 * @return zero for a well-formed frame, a nonzero residue otherwise
 *
 * @note THE CHECKSUM IS ONE BIG-ENDIAN WORD, not two bytes, and this
 *       function got that wrong on its first draft - it added f[7] and
 *       f[8] separately, which gave 0x02FE for a frame that is perfectly
 *       correct. 0xFE + 0xE8 is 486; 0xFEE8 is 65256, and only the second
 *       one cancels a sum of 280 in 16 bits.
 *
 *       Reading the checksum as a word is also what keeps this honest
 *       about byte ORDER: swapping SUM_HI and SUM_LO produces a different
 *       word and a non-zero residue, where a byte-wise sum would happily
 *       accept either arrangement.
 */
static UInt16 residue(const UInt8* f)
{
    UInt16 sum = 0;
    for(Int32 i = 1; i <= 6; ++i)
    {
        sum = static_cast<UInt16>(sum + f[i]);
    }

    const UInt16 chk = static_cast<UInt16>((static_cast<UInt32>(f[7]) << 8u)
                                           | static_cast<UInt32>(f[8]));
    return static_cast<UInt16>(sum + chk);
}

/**
 * @brief Prints a frame's bytes in hex, alongside its checksum residue.
 *
 * @param label text printed before the bytes, identifying which frame this is
 * @param f the ten-byte frame to print
 */
static Void dump(CharSeq label, const UInt8* f)
{
    printf("        %s: ", label);
    for(Int32 i = 0; i < 10; ++i)
    {
        printf("%02X ", f[i]);
    }
    printf(" (residue %04X)\n", residue(f));
}

/**
 * @brief Runs every DFPlayer frame-checksum check and reports the pass/fail count.
 *
 * @return 0 if every check passed, 1 if any failed
 */
Int32 main(Void)
{
    UInt8 f[10];

    printf("\ndfplayer frame format\n\n");

    /* ---- the envelope ---------------------------------------------------- */
    dfplayer::frame(f, DFP_CMD_MP3, 0x00u, 1u);
    dump("play mp3/0001", f);

    check(f[0] == 0x7Eu, "starts with 7E");
    check(f[1] == 0xFFu, "version FF");
    check(f[2] == 0x06u, "length 06");
    check(f[3] == DFP_CMD_MP3, "command in byte 3");
    check(f[4] == 0x00u, "ACK off by default");
    check(f[9] == 0xEFu, "ends with EF");

    /* ---- the parameter is BIG endian ------------------------------------- */
    dfplayer::frame(f, DFP_CMD_FOLDER, 0x00u, 0x0102u);
    dump("folder 1 track 2", f);
    check(f[5] == 0x01u, "parameter high byte first");
    check(f[6] == 0x02u, "parameter low byte second");

    /* ---- the checksum, which is the whole point -------------------------- */
    check(residue(f) == 0u, "body plus checksum cancels in 16 bits");

    /*
     * Across every command, and across parameters chosen to make the sum
     * wrap - a checksum that is right for small numbers and wrong when the
     * total crosses 0x100 would otherwise pass a single spot check, and
     * "works for track 1, silent for track 300" is a genuinely nasty bug to
     * meet on a bench.
     */
    const UInt8 CMDS[] =
    {
        DFP_CMD_NEXT, DFP_CMD_PREV, DFP_CMD_TRACK, DFP_CMD_VOLUME,
        DFP_CMD_EQ, DFP_CMD_SOURCE, DFP_CMD_RESET, DFP_CMD_PLAY,
        DFP_CMD_PAUSE, DFP_CMD_FOLDER, DFP_CMD_MP3, DFP_CMD_STOP,
    };
    const UInt16 PARAMS[] = { 0u, 1u, 0x00FFu, 0x0100u, 0x0101u,
                              0x7FFFu, 0xFF00u, 0xFFFFu };

    Int32 bad = 0;
    for(const UInt8 cmd : CMDS)
    {
        for(const UInt16 param : PARAMS)
        {
            for(UInt8 ack = 0; ack <= 1; ++ack)
            {
                dfplayer::frame(f, cmd, ack, param);
                if(residue(f) != 0u)
                {
                    if(bad == 0)
                    {
                        printf("        first bad: cmd %02X param %04X ack %d\n", cmd, param, ack);
                        dump("bad frame", f);
                    }
                    ++bad;
                }
            }
        }
    }
    check(bad == 0, "checksum holds for every command x parameter x ack");

    /* ---- the wrap is the interesting case -------------------------------- */
    dfplayer::frame(f, DFP_CMD_MP3, 0x00u, 0xFFFFu);
    dump("param FFFF", f);
    check(residue(f) == 0u, "checksum survives a parameter that wraps the sum");

    /* ---- volume is clamped by the caller, not here ------------------------ */
    dfplayer::frame(f, DFP_CMD_VOLUME, 0x00u, DFP_VOLUME_MAX);
    dump("volume 30", f);
    check(f[6] == 30u, "volume 30 reaches the wire");
    check(residue(f) == 0u, "volume frame checksums");

    printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
