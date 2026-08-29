/*
 * MicroSD over SPI: bring the card up, read its size, read and write blocks.
 *
 * ---------------------------------------------------------------------------
 * WHAT THIS IS AND IS NOT
 *
 * This is the BLOCK layer. It talks to the card and moves 512-byte sectors.
 * There is no filesystem here - no FAT, no files, no names - because those are
 * a separate problem and mixing them makes both harder to debug. Prove the card
 * answers first; a filesystem on top of a card that does not respond is a lot
 * of code failing for a reason none of it can see.
 *
 *     sd::Card sd;
 *     if(sd::open(&sd))
 *     {
 *         UInt8 block[512];
 *         sd::readBlock(&sd, 0, block);      // the boot sector
 *     }
 *
 * ---------------------------------------------------------------------------
 * WIRING
 *
 *   Card      Pico      why
 *   3V3       3V3       these modules regulate to 3.3 V; the card is a 3.3 V part
 *   GND       GND
 *   CLK       GP26      SPI1 SCK - fixed by the silicon
 *   MOSI      GP27      SPI1 TX
 *   MISO      GP28      SPI1 RX - and it MUST be GP8/12/24/28
 *   CS        GP22      any GPIO at all; this one happens to be free
 *
 * MISO is the pin that catches people. CS can be any pin because it is just an
 * output something wiggles, so "I moved CS to a free pin" is always fine - but
 * SCK, MOSI and MISO are specific peripheral signals on specific pads, and
 * moving one of those to a pad that cannot carry it fails silently. The card
 * never answers and every symptom points at the card.
 *
 * The display is on SPI0 (GP17/18/19) and the card is on SPI1 here, so the two
 * do not share a bus at all. They could - that is what CS is for - but separate
 * controllers means neither can stall the other.
 *
 * ---------------------------------------------------------------------------
 * WHY THE CLOCK STARTS SLOW
 *
 * A card powers up in a legacy mode that is only specified up to 400 kHz, and
 * it is not in SPI mode at all until it has been told to be. So sd::open() runs
 * the whole handshake at 400 kHz and only then asks for a real speed. Starting
 * fast produces a card that never leaves idle, which reads as a dead card.
 */
#ifndef TT02_SDCARD_H
#define TT02_SDCARD_H

#include "../hal.hxx"

namespace sd
{

/* Default pins - the wiring above. sd::openOn() takes them explicitly. */
#define PIN_SD_SCK   26
#define PIN_SD_MOSI  27
#define PIN_SD_MISO  28
#define PIN_SD_CS    22

/* 400 kHz for the handshake, because that is all a card promises before it is
 * in SPI mode. 12 MHz afterwards - well inside what any card manages, and slow
 * enough that jumper wires and a breadboard are not the limiting factor. */
#define SD_INIT_HZ   400000u
#define SD_FAST_HZ   12000000u

#define SD_BLOCK_SIZE 512

typedef enum Kind
{
    KIND_NONE = 0,
    KIND_V1,        /* SDSC, byte addressed */
    KIND_V2,        /* SDSC v2, byte addressed */
    KIND_HC         /* SDHC/SDXC, BLOCK addressed - the common case */
} Kind;

typedef struct Card
{
    Pin sck;
    Pin mosi;
    Pin miso;
    Pin cs;

    Kind kind;

    /* True when the card is addressed in BLOCKS rather than bytes. Getting this
     * backwards reads sector 0 for every request on a large card, or lands 512
     * times too far into a small one. */
    Bool blockAddressed;

    UInt32 blocks;      /* capacity in 512-byte blocks, 0 if unknown */
} Card;

/* ---- the wire ------------------------------------------------------------ */

static UInt8 xfer(const Card* c, UInt8 out)
{
    UInt8 in = 0xFF;
    spi::transfer(c->sck, &out, &in, 1);
    return in;
}

static Void select(const Card* c)
{
    gpio::write(c->cs, false);
}

static Void deselect(const Card* c)
{
    gpio::write(c->cs, true);

    /* Eight extra clocks with CS high. The specification asks for them and
     * cards genuinely need them: they are what lets the card finish releasing
     * the bus before anyone else uses it. */
    static_cast<Void>(xfer(c, 0xFF));
}

/*
 * Sends a command and returns its R1 response.
 *
 * A command is six bytes: 0x40 | index, a four-byte argument big-endian, then a
 * CRC. CRC is ignored in SPI mode EXCEPT for the two commands sent before the
 * card is in SPI mode - CMD0 and CMD8 - which is why those two have hard-coded
 * values here rather than a CRC routine that would only ever be used twice.
 *
 * The card answers with 0xFF while it thinks. R1 is the first byte with the top
 * bit clear, and 0xFF back after eight tries means it never answered at all.
 */
static UInt8 command(const Card* c, UInt8 cmd, UInt32 arg, UInt8 crc)
{
    static_cast<Void>(xfer(c, static_cast<UInt8>(0x40 | cmd)));
    static_cast<Void>(xfer(c, static_cast<UInt8>(arg >> 24)));
    static_cast<Void>(xfer(c, static_cast<UInt8>(arg >> 16)));
    static_cast<Void>(xfer(c, static_cast<UInt8>(arg >> 8)));
    static_cast<Void>(xfer(c, static_cast<UInt8>(arg)));
    static_cast<Void>(xfer(c, crc));

    for(Int32 i = 0; i < 10; ++i)
    {
        const UInt8 r = xfer(c, 0xFF);
        if((r & 0x80u) == 0u)
        {
            return r;
        }
    }
    return 0xFF;
}

/* CMD55 then the command - which is what an "application" command is. */
static UInt8 appCommand(const Card* c, UInt8 cmd, UInt32 arg)
{
    static_cast<Void>(command(c, 55, 0, 0x01));
    return command(c, cmd, arg, 0x01);
}

/* ---- bring-up ------------------------------------------------------------ */

/*
 * Brings a card up on the given pins.
 *
 * Returns false if the pins are wrong, if nothing answers, or if the card never
 * leaves idle. It does NOT distinguish those from each other, because from here
 * they are the same fact - `kind` stays sd::KIND_NONE and there is nothing to
 * talk to.
 */
static Bool openOn(Card* c, Pin sck, Pin mosi, Pin miso, Pin cs)
{
    if(c == NULL)
    {
        return false;
    }

    c->sck  = sck;
    c->mosi = mosi;
    c->miso = miso;
    c->cs   = cs;
    c->kind = KIND_NONE;
    c->blockAddressed = false;
    c->blocks = 0;

    /* spi::openFull refuses a MISO pin that cannot carry it, which is the whole
     * reason it takes MISO at all. */
    if(!spi::openFull(sck, mosi, miso, cs, SD_INIT_HZ))
    {
        return false;
    }

    /*
     * At least 74 clocks with CS HIGH, before anything else. This is how a card
     * is told to wake up, and skipping it is the classic reason a card works on
     * one board and not another - some cards are more forgiving than others
     * about how many they have had.
     */
    gpio::write(cs, true);
    for(Int32 i = 0; i < 10; ++i)
    {
        static_cast<Void>(xfer(c, 0xFF));
    }

    /* ---- CMD0: go idle, and enter SPI mode ------------------------------- */
    Bool idle = false;
    select(c);
    for(Int32 i = 0; i < 20; ++i)
    {
        if(command(c, 0, 0, 0x95) == 0x01)
        {
            idle = true;
            break;
        }
        timing::ms(10);
    }
    if(!idle)
    {
        deselect(c);
        return false;
    }

    /* ---- CMD8: which specification version? ------------------------------
     *
     * 0x1AA asks "are you 2.7-3.6 V, and echo 0xAA back". A v2 card answers and
     * repeats the pattern; a v1 card rejects the command outright, which is how
     * the two are told apart.
     */
    Bool v2 = false;
    if((command(c, 8, 0x1AAu, 0x87) & 0x04u) == 0u)
    {
        UInt8 r7[4];
        for(Int32 i = 0; i < 4; ++i)
        {
            r7[i] = xfer(c, 0xFF);
        }
        v2 = (r7[2] == 0x01 && r7[3] == 0xAA);
    }

    /* ---- ACMD41: start initialisation, and wait for it -------------------
     *
     * The HCS bit says "I understand high capacity", which a card needs to hear
     * before it will admit to being one. This can take a second on a cold card,
     * so the loop is generous.
     */
    const UInt32 hcs = v2 ? 0x40000000u : 0u;
    Bool ready = false;
    for(Int32 i = 0; i < 2000; ++i)
    {
        if(appCommand(c, 41, hcs) == 0x00)
        {
            ready = true;
            break;
        }
        timing::ms(1);
    }
    if(!ready)
    {
        deselect(c);
        return false;
    }

    /* ---- CMD58: read the OCR, for the capacity class ---------------------
     *
     * Bit 30 - CCS - is the one that matters: set means the card is addressed
     * in BLOCKS. Every modern card is. Getting this wrong reads sector 0 for
     * every request, which looks like a card full of identical data.
     */
    c->kind = v2 ? KIND_V2 : KIND_V1;
    if(v2 && command(c, 58, 0, 0x01) == 0x00)
    {
        UInt8 ocr[4];
        for(Int32 i = 0; i < 4; ++i)
        {
            ocr[i] = xfer(c, 0xFF);
        }
        if((ocr[0] & 0x40u) != 0u)
        {
            c->kind = KIND_HC;
            c->blockAddressed = true;
        }
    }

    /* ---- CMD16: 512-byte blocks --------------------------------------------
     * Only meaningful on a byte-addressed card; harmless on the others, and
     * sending it unconditionally means one less branch to get wrong. */
    static_cast<Void>(command(c, 16, SD_BLOCK_SIZE, 0x01));

    /* ---- CMD9: the CSD, for capacity ------------------------------------- */
    if(command(c, 9, 0, 0x01) == 0x00)
    {
        /* Wait for the data token, then sixteen bytes and a two-byte CRC. */
        UInt8 tok = 0xFF;
        for(Int32 i = 0; i < 2000; ++i)
        {
            tok = xfer(c, 0xFF);
            if(tok != 0xFF)
            {
                break;
            }
        }

        if(tok == 0xFE)
        {
            UInt8 csd[16];
            for(Int32 i = 0; i < 16; ++i)
            {
                csd[i] = xfer(c, 0xFF);
            }
            static_cast<Void>(xfer(c, 0xFF));
            static_cast<Void>(xfer(c, 0xFF));

            if((csd[0] >> 6) == 1)
            {
                /* CSD version 2: C_SIZE is 22 bits and capacity is
                 * (C_SIZE + 1) * 512 KB, so blocks = (C_SIZE + 1) * 1024. */
                const UInt32 cSize = ((static_cast<UInt32>(csd[7] & 0x3F)) << 16)
                                   | ((static_cast<UInt32>(csd[8])) << 8)
                                   | (static_cast<UInt32>(csd[9]));
                c->blocks = (cSize + 1u) * 1024u;
            }
            else
            {
                /* CSD version 1, on the small old cards. The size is spread
                 * across three fields and scaled by two exponents, which is
                 * exactly why version 2 replaced it. */
                const UInt32 cSize = (((static_cast<UInt32>(csd[6] & 0x03)) << 10)
                                      | ((static_cast<UInt32>(csd[7])) << 2)
                                      | ((static_cast<UInt32>(csd[8])) >> 6));
                const UInt32 mult  = static_cast<UInt32>(((csd[9] & 0x03) << 1)
                                               | (csd[10] >> 7));
                const UInt32 rdLen = static_cast<UInt32>(csd[5] & 0x0F);
                const UInt32 bytes = (cSize + 1u) * (1u << (mult + 2u))
                                   * (1u << rdLen);
                c->blocks = bytes / SD_BLOCK_SIZE;
            }
        }
    }

    deselect(c);

    /* The handshake is over, so the clock can go up. */
    static_cast<Void>(spi::baud(sck, SD_FAST_HZ));
    return true;
}

static Bool open(Card* c)
{
    return openOn(c, PIN_SD_SCK, PIN_SD_MOSI, PIN_SD_MISO, PIN_SD_CS);
}

/* Capacity in megabytes, for showing a person. */
static UInt32 megabytes(const Card* c)
{
    return (c->blocks / 2048u);
}

static const Utf8* kindName(const Card* c)
{
    switch(c->kind)
    {
    case KIND_HC: return "SDHC/SDXC";
    case KIND_V2: return "SD v2";
    case KIND_V1: return "SD v1";
    case KIND_NONE:
    default: return "none";
    }
}

/* ---- blocks -------------------------------------------------------------- */

/*
 * Reads one 512-byte block.
 *
 * `block` is a BLOCK index, always - the byte-versus-block distinction is
 * handled here so no caller has to remember which kind of card it has.
 */
static Bool readBlock(const Card* c, UInt32 block, UInt8* out)
{
    if(c->kind == KIND_NONE || out == NULL)
    {
        return false;
    }

    const UInt32 addr = c->blockAddressed ? block : (block * SD_BLOCK_SIZE);

    select(c);
    if(command(c, 17, addr, 0x01) != 0x00)
    {
        deselect(c);
        return false;
    }

    /* The card sends 0xFF until its data is ready, then 0xFE to say "here it
     * comes". Anything else is an error token and the read has failed. */
    UInt8 tok = 0xFF;
    for(Int32 i = 0; i < 20000; ++i)
    {
        tok = xfer(c, 0xFF);
        if(tok != 0xFF)
        {
            break;
        }
    }
    if(tok != 0xFE)
    {
        deselect(c);
        return false;
    }

    for(Int32 i = 0; i < SD_BLOCK_SIZE; ++i)
    {
        out[i] = xfer(c, 0xFF);
    }
    static_cast<Void>(xfer(c, 0xFF));           /* CRC, which SPI mode ignores */
    static_cast<Void>(xfer(c, 0xFF));

    deselect(c);
    return true;
}

/*
 * Writes one 512-byte block.
 *
 * Waits for the card to finish before returning. A card can hold the line low
 * for a surprisingly long time after a write, and returning early means the
 * next command lands while it is still busy and quietly fails.
 */
static Bool writeBlock(const Card* c, UInt32 block, const UInt8* data)
{
    if(c->kind == KIND_NONE || data == NULL)
    {
        return false;
    }

    const UInt32 addr = c->blockAddressed ? block : (block * SD_BLOCK_SIZE);

    select(c);
    if(command(c, 24, addr, 0x01) != 0x00)
    {
        deselect(c);
        return false;
    }

    static_cast<Void>(xfer(c, 0xFF));
    static_cast<Void>(xfer(c, 0xFE));           /* the start-of-data token */

    for(Int32 i = 0; i < SD_BLOCK_SIZE; ++i)
    {
        static_cast<Void>(xfer(c, data[i]));
    }
    static_cast<Void>(xfer(c, 0xFF));           /* CRC, ignored */
    static_cast<Void>(xfer(c, 0xFF));

    /* The bottom five bits of the response say whether it was accepted; 0x05
     * means yes. */
    const UInt8 resp = xfer(c, 0xFF);
    if((resp & 0x1Fu) != 0x05u)
    {
        deselect(c);
        return false;
    }

    /* The card holds MISO low while it programs. Wait it out. */
    for(Int32 i = 0; i < 100000; ++i)
    {
        if(xfer(c, 0xFF) != 0x00)
        {
            deselect(c);
            return true;
        }
    }

    deselect(c);
    return false;
}


} // namespace sd
#endif
