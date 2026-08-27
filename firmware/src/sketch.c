/*
 * Paint the LCD yellow. That is the whole program.
 *
 * Wiring: SCK GP18, MOSI GP19, CS GP17, DC GP21, RES GP20, VCC and BLK to 3V3.
 *
 * THE SIZE IS SET HERE, not in st77xx.h. This panel is 240x280, and the 20 is
 * not padding: the controller has 240x320 of RAM behind it whatever glass is on
 * the front, so a 240x280 shows a WINDOW into that RAM starting 20 rows down.
 * Without the offset the picture sits 20 pixels too high and the bottom 20 rows
 * are never written - which is why filling it "worked" and still left a band.
 */

#include "pico2w.h"
#include "st77xx.h"

#define SCREEN_W     240
#define SCREEN_H     280
#define SCREEN_XOFF  0
#define SCREEN_YOFF  20

Int32 main(Void)
{
    /* The one line that is not about the screen, and it stays.
     *
     * Without it the board never enumerates over USB: no COM port, nothing for
     * the flasher to reboot, and the only way back in is the BOOTSEL button. */
    serialOpen();

    tftInitSize(SCREEN_W, SCREEN_H, SCREEN_XOFF, SCREEN_YOFF);
    tftFill(TFT_YELLOW);

    while(true)
    {
        sleepMs(1000);
    }

    return 0;
}
