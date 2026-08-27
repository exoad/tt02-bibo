/*
 * Bring up the MicroSD card and prove it works, on the LCD.
 *
 * ---------------------------------------------------------------------------
 * WIRING
 *
 *   Card      Pico      why
 *   3V3       3V3       the module regulates; the card is a 3.3 V part
 *   GND       GND
 *   CLK       GP26      SPI1 SCK - fixed by the silicon
 *   MOSI      GP27      SPI1 TX
 *   MISO      GP28      SPI1 RX - and it MUST be one of GP8/12/24/28
 *   CS        GP22      any GPIO; this one is free
 *
 *   Display   Pico
 *   SCL/SCK   GP18      SPI0 - a different controller, so neither can stall
 *   SDA/MOSI  GP19      the other
 *   RES/DC/CS GP20/21/17
 *
 * ---------------------------------------------------------------------------
 * WHAT IT CHECKS, IN ORDER
 *
 *   1. The card answers at all             - wiring, power, MISO on a real RX pad
 *   2. What kind it is, and how big        - the handshake completed
 *   3. Block 0 reads                       - data actually flows back
 *   4. Is there a filesystem on it         - the boot signature, 0x55AA
 *   5. A block round-trips                 - written, read back, compared
 *
 * Step 5 writes to the LAST block on the card, on purpose. It is past the end
 * of any filesystem, so a card with files on it keeps them. Writing to block 0
 * would destroy the partition table of whatever you plugged in, which is a poor
 * way to find out that writing works.
 */

#include "pico2w.h"
#include "gfx.h"
#include "sdcard.h"

/* ---- the screen ---------------------------------------------------------- */
#define SCREEN_W        240
#define SCREEN_H        280
#define SCREEN_XOFF     0
#define SCREEN_YOFF     20
#define SAFE_INSET      14

#define LINE_STEP       14

Int32 main(Void)
{
    serialOpen();

    Screen screen;
    gfxOpen(&screen, SCREEN_W, SCREEN_H, SCREEN_XOFF, SCREEN_YOFF);
    gfxSafeInset(&screen, SAFE_INSET);

    const Int32 left = gfxSafeLeft(&screen);

    SdCard sd;
    const Bool up = sdOpen(&sd);

    /* Everything below is decided once, not every frame: the card is not going
     * to change its mind, and re-running a write test in a loop would be a
     * strange thing to do to somebody's card. */
    UInt8  block[SD_BLOCK_SIZE];
    Bool   read0     = false;
    Bool   haveFs    = false;
    Bool   roundTrip = false;

    if(up)
    {
        read0 = sdReadBlock(&sd, 0, block);

        /* 0x55AA at the end of the first sector is the boot signature. Its
         * presence means something formatted this card; its absence means the
         * card is raw, which is not a fault. */
        if(read0)
        {
            haveFs = (block[510] == 0x55 && block[511] == 0xAA);
        }

        if(sd.blocks > 1u)
        {
            const UInt32 last = sd.blocks - 1u;

            UInt8 out[SD_BLOCK_SIZE];
            for(Int32 i = 0; i < SD_BLOCK_SIZE; ++i)
            {
                out[i] = (UInt8) (i & 0xFF);
            }
            out[0] = 0x54;    /* 'T' */
            out[1] = 0x54;    /* 'T' */
            out[2] = 0x30;    /* '0' */
            out[3] = 0x32;    /* '2' */

            if(sdWriteBlock(&sd, last, out))
            {
                UInt8 back[SD_BLOCK_SIZE];
                if(sdReadBlock(&sd, last, back))
                {
                    roundTrip = true;
                    for(Int32 i = 0; i < SD_BLOCK_SIZE; ++i)
                    {
                        if(back[i] != out[i])
                        {
                            roundTrip = false;
                            break;
                        }
                    }
                }
            }
        }
    }

    serialPrintf("sd: up=%d kind=%s blocks=%u read0=%d fs=%d rt=%d\n",
                 (Int32) up, sdKindName(&sd), sd.blocks,
                 (Int32) read0, (Int32) haveFs, (Int32) roundTrip);

    while(true)
    {
        gfxClear(&screen, GFX_NAVY);

        Int32 y = gfxSafeTop(&screen);

        gfxTextTransparent(&screen);
        gfxTextColour(&screen, GFX_ORANGE);
        gfxTextSize(&screen, 2);
        gfxTextAt(&screen, left, y, "MICROSD");
        y += 26;

        gfxTextSize(&screen, 1);

        if(!up)
        {
            gfxTextColour(&screen, GFX_RED);
            gfxTextAt(&screen, left, y, "NO CARD RESPONSE");
            y += LINE_STEP + 6;

            gfxTextColour(&screen, GFX_GREY);
            gfxTextAt(&screen, left, y, "MISO MUST BE GP28");
            y += LINE_STEP;
            gfxTextAt(&screen, left, y, "CLK GP26  MOSI GP27");
            y += LINE_STEP;
            gfxTextAt(&screen, left, y, "CS GP22   3V3 AND GND");
            y += LINE_STEP;
            gfxTextAt(&screen, left, y, "IS A CARD INSERTED");
        }
        else
        {
            gfxTextColour(&screen, GFX_GREEN);
            gfxCursor(&screen, left, y);
            gfxPrintf(&screen, "%s", sdKindName(&sd));
            y += LINE_STEP;

            gfxTextColour(&screen, GFX_WHITE);
            gfxCursor(&screen, left, y);
            gfxPrintf(&screen, "%u MB", sdMegabytes(&sd));
            y += LINE_STEP;

            gfxTextColour(&screen, GFX_GREY);
            gfxCursor(&screen, left, y);
            gfxPrintf(&screen, "%u BLOCKS", sd.blocks);
            y += LINE_STEP + 8;

            gfxTextColour(&screen, read0 ? GFX_GREEN : GFX_RED);
            gfxTextAt(&screen, left, y, read0 ? "READ BLOCK 0 OK"
                                              : "READ BLOCK 0 FAILED");
            y += LINE_STEP;

            gfxTextColour(&screen, haveFs ? GFX_GREEN : GFX_YELLOW);
            gfxTextAt(&screen, left, y, haveFs ? "FILESYSTEM FOUND"
                                               : "NO BOOT SIGNATURE");
            y += LINE_STEP;

            gfxTextColour(&screen, roundTrip ? GFX_GREEN : GFX_RED);
            gfxTextAt(&screen, left, y, roundTrip ? "WRITE + READ OK"
                                                  : "WRITE TEST FAILED");
            y += LINE_STEP + 8;

            /* The first bytes of sector 0, which is a fingerprint of the card
             * rather than anything meaningful - but it is DATA, and seeing it
             * change between two different cards is the proof that it is real. */
            gfxTextColour(&screen, GFX_CYAN);
            gfxTextAt(&screen, left, y, "SECTOR 0");
            y += LINE_STEP;

            gfxTextColour(&screen, GFX_GREY);
            for(Int32 row = 0; row < 4; ++row)
            {
                gfxCursor(&screen, left, y);
                gfxPrintf(&screen, "%02X %02X %02X %02X %02X %02X",
                          block[row * 6 + 0], block[row * 6 + 1],
                          block[row * 6 + 2], block[row * 6 + 3],
                          block[row * 6 + 4], block[row * 6 + 5]);
                y += LINE_STEP;
            }
        }

        gfxPresent(&screen);
        sleepMs(1000);
    }

    return 0;
}
