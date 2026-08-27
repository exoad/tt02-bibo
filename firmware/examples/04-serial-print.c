/*
 * 04 - Talking back
 *
 * TEACHES: printing to the host, and the one reason your first prints vanish.
 * NEEDS:   nothing. Open the Pico 2 W tab in the hub and connect to see it.
 *
 * ---------------------------------------------------------------------------
 * WHY YOUR FIRST PRINTS DISAPPEAR
 *
 * USB enumeration takes a moment. serialOpen() starts the stack, but the host
 * is not listening yet - so anything printed in the first fraction of a second
 * goes nowhere and the program looks broken when it is merely early.
 *
 * serialWaitForHost() waits for the host to open the port. Pass a timeout in
 * milliseconds; pass 0 to wait forever.
 *
 * 0 IS RIGHT ON THE BENCH AND WRONG ON THE CAR. A board waiting for a terminal
 * that will never arrive is a board that never starts driving. Use a timeout in
 * anything that has to run on its own.
 *
 * ---------------------------------------------------------------------------
 * TRY
 *
 * Delete the serialWaitForHost line, flash, and watch the first few lines go
 * missing. Then put it back.
 */

#include "pico2w.h"

Int32 main(Void)
{
    serialOpen();

    /* Up to three seconds, then carry on regardless. */
    const Bool listening = serialWaitForHost(3000);

    serialPrintLine("hello from the Pico 2 W");
    serialPrintf("host was %s when we started\n",
                 listening ? "listening" : "not there yet");

    if(watchdogCausedReboot())
    {
        serialPrintLine("note: the previous boot ended in a watchdog reset");
    }

    UInt32 tick = 0;

    while(true)
    {
        /*
         * nowMs() is milliseconds since boot, so it also shows how long the
         * board has been up - a quick way to notice a board that is silently
         * resetting in a loop, because the number keeps starting over.
         */
        serialPrintf("tick %u   up %u ms   chip %.1f C\n",
                     tick, nowMs(), (Float64) tempC());
        ++tick;

        sleepMs(1000);
    }

    return 0;
}
