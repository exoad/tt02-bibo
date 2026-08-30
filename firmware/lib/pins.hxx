/* ---------------------------------------------------------------------------
 * pins - every pin the car uses, in one place.
 *
 * THE POINT OF THIS FILE is that it is the only place a GPIO number appears.
 * Before it, `PIN_SERVO 0` and `PIN_ESC 1` lived in chassis.hxx, the ten lamps
 * lived in a table in lights.hxx, and a sketch picked whatever looked free by
 * reading those two files and hoping. Nothing checked them against each other,
 * so "is GP14 taken?" was a question you answered by grepping - and the answer
 * went stale the moment somebody added a driver.
 *
 * It is answered by the compiler now. See the static_assert at the bottom: two
 * subsystems claiming one pin is a build error, not a mystery on the bench.
 *
 * ---------------------------------------------------------------------------
 * THIS IS A DECLARATION, NOT AN INITIALISATION.
 *
 * Nothing here opens anything. A pin number is a fact about how the car is
 * wired; opening it is a thing a subsystem does when it starts, because only
 * the subsystem knows whether the pin wants a direction, a pull, a PWM slice or
 * an alternate function. chassis::open() still opens the servo and the ESC.
 * lights::open() still opens the lamps. They just no longer decide WHICH.
 *
 * That split is why the numbers can be constexpr: they are compile-time facts,
 * so they cost nothing and can be asserted on.
 *
 * ---------------------------------------------------------------------------
 * SKETCHES ARE NOT REQUIRED TO USE THIS, and that is deliberate.
 *
 * A sketch in firmware/sketches/ is one file asking one question, often about a
 * part that is not on the car yet and may never be. Making it declare its pins
 * here would mean editing the car's pin map to try something on a breadboard,
 * which is backwards. A sketch writes its own numbers at the top of its own
 * file, where the person holding the jumper wire can see them.
 *
 * What a sketch SHOULD do is read this file first, so it borrows a pin that is
 * free rather than one the car is already driving.
 * ------------------------------------------------------------------------- */
#pragma once

#include "types.hxx"

namespace bibo
{

  namespace pins
  {

    /* Not wired. A subsystem holding this must skip the pin entirely rather than
     * driving GPIO -1, which the SDK would happily do something undefined with. */
    constexpr Int32 NONE = -1;

    /* ---- chassis -------------------------------------------------------------
     *
     * Both are servo-style PWM, and both are on the two pins the whole project has
     * assumed since the first day. UART stdio is off in CMakeLists precisely so
     * these stay free - GP0/GP1 are the default UART pins and stdio would take
     * them. */
    constexpr Int32 SERVO = 0;
    constexpr Int32 ESC   = 1;

    /* ---- I2C0 ----------------------------------------------------------------
     *
     * The ToF sensors, the IMU and the display share this. SCAN answers nothing
     * today because none of them is fitted. */
    constexpr Int32 I2C_SDA = 4;
    constexpr Int32 I2C_SCL = 5;

    /* ---- sound: DFPlayer Mini on UART0 ---------------------------------------
     *
     * THE DIRECTIONS ARE THE SILICON'S, NOT A CHOICE. On RP2350, GPIO14 is
     * UART0_TX and GPIO15 is UART0_RX - funcsel 0x0b, which the SDK calls
     * GPIO_FUNC_UART_AUX and not GPIO_FUNC_UART. Getting the pair backwards puts
     * two outputs against each other and two inputs against each other, which is
     * silent rather than broken: nothing is damaged and nothing happens.
     *
     * SOUND_TX drives the DFPlayer's RX THROUGH A 1k RESISTOR. The module's RX has
     * no series protection of its own and is documented as noise-sensitive; the
     * resistor is standard practice for it and costs nothing.
     *
     * BUSY is the module's own output and is LOW WHILE A TRACK IS PLAYING. It is
     * the only honest way to know a cue has finished - the serial reply says a
     * command was accepted, which is a different claim. Not wired yet.
     *
     * These two pins were TAIL_L and TAIL_R until the DFPlayer went on. See the
     * lamp block below. */
    constexpr Int32 SOUND_TX   = 14;
    constexpr Int32 SOUND_RX   = 15;
    constexpr Int32 SOUND_BUSY = NONE;

    /* ---- lamps ---------------------------------------------------------------
     *
     * ALL OF THESE ARE BORROWED and the banner in lights.hxx says from what. In
     * short: GP10-GP13 are the four ToF XSHUT lines, free only because no ToF is
     * fitted, and they stop being free the moment one is.
     *
     * THE TAIL LAMPS ARE UNWIRED AS OF THE DFPLAYER GOING ON. They were GP15 and
     * GP14, which are the only two pins on this chip that can carry UART0 without
     * taking GP0/GP1 from the servo and the ESC or GP12/GP13 from the front
     * indicators. Sound won them.
     *
     * That is not a loss the code has to care about: cue::solve() computes all ten
     * lamps whether or not an LED exists, and lights::write() skips any lamp whose
     * pin is NONE. The rear indicators have been in exactly this state since they
     * were added. Their permanent home is GP6/GP7 - docs/wiring.md - and moving
     * there is editing two lines in this file. */
    constexpr Int32 HEAD_L = 11;
    constexpr Int32 HEAD_R = 10;
    constexpr Int32 TAIL_L = NONE;   /* was 15, now SOUND_RX */
    constexpr Int32 TAIL_R = NONE;   /* was 14, now SOUND_TX */
    constexpr Int32 IND_FL = 13;
    constexpr Int32 IND_FR = 12;
    constexpr Int32 IND_RL = NONE;
    constexpr Int32 IND_RR = NONE;
    constexpr Int32 REV_L  = NONE;
    constexpr Int32 REV_R  = NONE;

    /* ---- not fitted ----------------------------------------------------------
     *
     * Declared rather than left out, so that "which pin is the encoder on" has an
     * answer in the same file as everything else, and so the conflict check below
     * covers them the day they are wired. */
    constexpr Int32 ENCODER = NONE;   /* GP15 was reserved; sound has it now */

    /* ===========================================================================
     * THE CONFLICT CHECK.
     *
     * Every assigned pin above, once. Two subsystems on one number is the bug this
     * file exists to make impossible, and it is exactly the bug that was sitting
     * here: the DFPlayer wants GP14 and GP15, and the tail lamps were already on
     * them. On a breadboard that is a confusing evening. Here it is a build error
     * naming both claimants.
     *
     * constexpr and evaluated at compile time, so it costs nothing at run time and
     * cannot be forgotten.
     * ======================================================================== */
    constexpr Int32 ASSIGNED[] =
    {
        SERVO, ESC,
        I2C_SDA, I2C_SCL,
        SOUND_TX, SOUND_RX, SOUND_BUSY,
        HEAD_L, HEAD_R, TAIL_L, TAIL_R,
        IND_FL, IND_FR, IND_RL, IND_RR,
        REV_L, REV_R,
        ENCODER,
    };

    constexpr Size ASSIGNED_COUNT = sizeof(ASSIGNED) / sizeof(ASSIGNED[0]);

    /* NONE is skipped: it is the one value many entries share on purpose. */
    constexpr Bool noPinIsClaimedTwice(Void)
    {
        for(Size a = 0; a < ASSIGNED_COUNT; ++a)
        {
            if(ASSIGNED[a] == NONE)
            {
                continue;
            }
            for(Size b = a + 1; b < ASSIGNED_COUNT; ++b)
            {
                if(ASSIGNED[a] == ASSIGNED[b])
                {
                    return false;
                }
            }
        }
        return true;
    }

    static_assert(noPinIsClaimedTwice(),
                  "two subsystems in pins.hxx claim the same GPIO - read the "
                  "ASSIGNED table above and decide which one gets it");

    /* A pin that does not exist on the package is a typo, and a typo in a pin
     * number is invisible on a bench: the wire just does nothing. RP2350 in this
     * package brings out GP0-GP29. */
    constexpr Bool everyPinExists(Void)
    {
        for(Size i = 0; i < ASSIGNED_COUNT; ++i)
        {
            if(ASSIGNED[i] != NONE && (ASSIGNED[i] < 0 || ASSIGNED[i] > 29))
            {
                return false;
            }
        }
        return true;
    }

    static_assert(everyPinExists(),
                  "a pin number in pins.hxx is not a GPIO on this package (0-29)");

  } /* namespace pins */

} /* namespace bibo */
