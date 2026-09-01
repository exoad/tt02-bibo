/**
 * @file storage.hxx
 * @brief MicroSD over SPI: bring the card up, read its size, read and
 * write blocks.
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
#pragma once

#include "../hal.hxx"
#include "../pins.hxx"

namespace bibo::sd
{

    /**
     * @brief Default pins, matching the wiring table above.
     *
     * sd::openOn() takes them explicitly if a board differs.
     */
#define PIN_SD_SCK   26
#define PIN_SD_MOSI  27
#define PIN_SD_MISO  28
#define PIN_SD_CS    22

    /**
     * @brief SPI clock rates for the card, in Hz: the handshake speed
     * and the working speed.
     *
     * 400 kHz for the handshake, because that is all a card promises
     * before it is in SPI mode. 12 MHz afterwards - well inside what any
     * card manages, and slow enough that jumper wires and a breadboard
     * are not the limiting factor.
     */
#define SD_INIT_HZ   400000u
#define SD_FAST_HZ   12000000u

#define SD_BLOCK_SIZE 512

    enum Kind
    {
        KIND_NONE = 0,
        KIND_V1,        /* SDSC, byte addressed */
        KIND_V2,        /* SDSC v2, byte addressed */
        KIND_HC         /* SDHC/SDXC, BLOCK addressed - the common case */
    };

    /** @brief One microSD card: its SPI pins, kind, and addressing. */
    struct Card
    {
        Pin sck;
        Pin mosi;
        Pin miso;
        Pin cs;

        Kind kind;

        /**
         * @brief True when the card is addressed in BLOCKS rather than
         * bytes.
         *
         * Getting this backwards reads sector 0 for every request on a
         * large card, or lands 512 times too far into a small one.
         */
        Bool blockAddressed;

        UInt32 blocks;      /* capacity in 512-byte blocks, 0 if unknown */
    };

    /* ---- the wire ------------------------------------------------------------ */

    /**
     * @brief Exchanges one byte over SPI, full duplex.
     *
     * @param c the card whose bus to use
     * @param out the byte to send
     * @return the byte the card sent back at the same time
     */
    inline UInt8 xfer(const Card* c, const UInt8 out)
    {
        UInt8 in = 0xFF;
        spi::transfer(c->sck, &out, &in, 1);
        return in;
    }

    /**
     * @brief Pulls CS low, claiming the SPI bus for this card.
     *
     * @param c the card whose CS pin is asserted
     */
    inline Void select(const Card* c)
    {
        gpio::write(c->cs, false);
    }

    /**
     * @brief Raises CS, ending the card's SPI transaction.
     *
     * @param c the card whose CS pin is released
     */
    inline Void deselect(const Card* c)
    {
        gpio::write(c->cs, true);

        /* Eight extra clocks with CS high, so the card can finish releasing the bus. */
        static_cast<Void>(xfer(c, 0xFF));
    }

    /**
     * @brief Sends a command and returns its R1 response.
     *
     * A command is six bytes: 0x40 | index, a four-byte argument
     * big-endian, then a CRC. CRC is ignored in SPI mode EXCEPT for the
     * two commands sent before the card is in SPI mode - CMD0 and CMD8 -
     * which is why those two have hard-coded values here rather than a
     * CRC routine that would only ever be used twice.
     *
     * The card answers with 0xFF while it thinks. R1 is the first byte
     * with the top bit clear, and 0xFF back after eight tries means it
     * never answered at all.
     *
     * @param c the card to address
     * @param cmd the command index, 0-63
     * @param arg the command's 32-bit argument
     * @param crc the CRC7 byte (with the stop bit); needed only for
     *        CMD0 and CMD8
     * @return the R1 response byte, or 0xFF if the card never answered
     */
    inline UInt8 command(const Card* c, const UInt8 cmd, const UInt32 arg, const UInt8 crc)
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

    /**
     * @brief Sends an "application" command: CMD55, then the command
     * itself.
     *
     * @param c the card to address
     * @param cmd the ACMD index to send after CMD55
     * @param arg the command's 32-bit argument
     * @return the R1 response byte from the ACMD, or 0xFF if the card
     *         never answered
     */
    inline UInt8 appCommand(const Card* c, const UInt8 cmd, const UInt32 arg)
    {
        static_cast<Void>(command(c, 55, 0, 0x01));
        return command(c, cmd, arg, 0x01);
    }

    /* ---- bring-up ------------------------------------------------------------ */

    /**
     * @brief Brings a card up on the given pins.
     *
     * Returns false if the pins are wrong, if nothing answers, or if the
     * card never leaves idle. It does NOT distinguish those from each
     * other, because from here they are the same fact - `kind` stays
     * sd::KIND_NONE and there is nothing to talk to.
     *
     * @param c the card to initialize; must not be null
     * @param sck the SPI clock pin
     * @param mosi the SPI data-out pin
     * @param miso the SPI data-in pin; must be a pin the SPI peripheral
     *        can actually receive on
     * @param cs the chip-select pin for this card
     * @return true once the card has left idle and been sized; false on
     *         any wiring or protocol failure
     *
     * @note Runs the whole handshake at SD_INIT_HZ and only raises the
     * clock to SD_FAST_HZ once it succeeds - a card powers up in a
     * legacy mode that is only specified up to 400 kHz, and starting
     * fast produces a card that never leaves idle.
     */
    [[nodiscard]] static Bool openOn(Card* c, const Pin sck, const Pin mosi, const Pin miso, const Pin cs)
    {
        if(c == nullptr)
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

        /* spi::openFull refuses a MISO pin that cannot carry it. */
        if(!spi::openFull(sck, mosi, miso, cs, SD_INIT_HZ))
        {
            return false;
        }

        /*
         * At least 74 clocks with CS HIGH first, or some cards never wake at all.
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

        /* ---- CMD8: 0x1AA asks for 2.7-3.6 V and a 0xAA echo; a v1 card refuses - */
        Bool v2 = false;
        if((command(c, 8, 0x1AAu, 0x87) & 0x04u) == 0u)
        {
            UInt8 r7[4];
            for(Int32 i = 0; i < 4; ++i)
            {
                r7[i] = xfer(c, 0xFF);
            }
            v2 = r7[2] == 0x01 && r7[3] == 0xAA;
        }

        /*
         * ---- ACMD41: start initialization, and wait for it -------------------
         * HCS says "I understand high capacity". A cold card can take a second.
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

        /*
         * ---- CMD58: the OCR. Bit 30, CCS - set means the card is BLOCK addressed.
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

        /* ---- CMD16: 512-byte blocks; only a byte-addressed card cares -------- */
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

                if(csd[0] >> 6 == 1)
                {
                    /*
                     * CSD version 2: C_SIZE is 22 bits and capacity is
                     * (C_SIZE + 1) * 512 KB, so blocks = (C_SIZE + 1) * 1024.
                     */
                    const UInt32 cSize = (static_cast<UInt32>(csd[7] & 0x3F) << 16)
                                         | (static_cast<UInt32>(csd[8]) << 8)
                                         | static_cast<UInt32>(csd[9]);
                    c->blocks = (cSize + 1u) * 1024u;
                }
                else
                {
                    /*
                     * CSD version 1: size across three fields, two exponents.
                     */
                    const UInt32 cSize = (static_cast<UInt32>(csd[6] & 0x03) << 10)
                                         | (static_cast<UInt32>(csd[7]) << 2)
                                         | (static_cast<UInt32>(csd[8]) >> 6);
                    const UInt32 mult  = ((static_cast<UInt32>(csd[9]) & 0x03u) << 1u)
                                         | (static_cast<UInt32>(csd[10]) >> 7u);
                    const UInt32 rdLen = static_cast<UInt32>(csd[5] & 0x0F);
                    const UInt32 bytes = (cSize + 1u) * (1u << (mult + 2u))
                                         * (1u << rdLen);
                    c->blocks = bytes / SD_BLOCK_SIZE;
                }
            }
        }

        deselect(c);

        static_cast<Void>(spi::baud(sck, SD_FAST_HZ));
        return true;
    }

    /**
     * @brief Brings the card up on this project's pins from
     * pins::active().
     *
     * @param c the card to initialize; must not be null
     * @return true once the card has left idle and been sized; false on
     *         any wiring or protocol failure
     *
     * @note pins::begin() must have run first, so pins::active() has a
     * real map to read.
     * @note The installed map may route the card onto the same physical
     * SPI bus as the display - SCK and MOSI shared, told apart only by
     * separate CS lines (see display.hxx). On this file's own default
     * pins the card is on its own SPI controller and cannot collide with
     * anything.
     */
    [[nodiscard]] static Bool open(Card* c)
    {
        const pins::Map& m = pins::active();
        return openOn(c, m.sdSck, m.sdMosi, m.sdMiso, m.sdCs);
    }

    /**
     * @brief The card's capacity in megabytes, for showing a person.
     *
     * @param c the card to ask
     * @return capacity in megabytes; 0 if the card's size could not be
     *         read
     */
    inline UInt32 megabytes(const Card* c)
    {
        return c->blocks / 2048u;
    }

    /**
     * @brief A short human-readable name for the card's kind.
     *
     * @param c the card to ask
     * @return a NUL-terminated string naming the card's kind; "none" if
     *         no card has been brought up
     */
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

    /**
     * @brief Reads one 512-byte block.
     *
     * `block` is a BLOCK index, always - the byte-versus-block
     * distinction is handled here so no caller has to remember which
     * kind of card it has.
     *
     * @param c the card to read from; must have been opened successfully
     * @param block the zero-based block index to read
     * @param out receives the SD_BLOCK_SIZE bytes read; must not be null
     *        and must hold at least that many bytes
     * @return true once the block has been read into `out`
     */
    [[nodiscard]] static Bool readBlock(const Card* c, const UInt32 block, UInt8* out)
    {
        if(c->kind == KIND_NONE || out == nullptr)
        {
            return false;
        }

        const UInt32 addr = c->blockAddressed ? block : block * SD_BLOCK_SIZE;

        select(c);
        if(command(c, 17, addr, 0x01) != 0x00)
        {
            deselect(c);
            return false;
        }

        /* 0xFF until the data is ready, then 0xFE. Anything else is an error token. */
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

    /**
     * @brief Writes one 512-byte block.
     *
     * Waits for the card to finish before returning. A card can hold the
     * line low for a surprisingly long time after a write, and returning
     * early means the next command lands while it is still busy and
     * quietly fails.
     *
     * @param c the card to write to; must have been opened successfully
     * @param block the zero-based block index to write
     * @param data the SD_BLOCK_SIZE bytes to write; must not be null and
     *        must hold at least that many bytes
     * @return true once the card has accepted and finished programming
     *         the block
     *
     * @warning A write that fails partway through can leave the block
     * holding a mix of its old and new contents; there is no way to
     * detect this from here short of reading the block back.
     */
    [[nodiscard]] static Bool writeBlock(const Card* c, const UInt32 block, const UInt8* data)
    {
        if(c->kind == KIND_NONE || data == nullptr)
        {
            return false;
        }

        const UInt32 addr = c->blockAddressed ? block : block * SD_BLOCK_SIZE;

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

        /* The bottom five bits of the response are 0x05 when the block was taken. */
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


}
