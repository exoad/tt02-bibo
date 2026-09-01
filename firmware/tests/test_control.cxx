/* PID, feedforward and odometry - lib/control.hxx and lib/chassis/odom.hxx.
 *
 *   firmware\tests\build_control_test.bat run
 *
 * ALL OF THIS IS ARITHMETIC, which is exactly why it is testable and exactly
 * why it is worth testing. A control loop that is subtly wrong does not throw:
 * it drives, badly, and the symptom is a car that oscillates or creeps or leaps
 * when a wheel comes free. Every one of those is expensive to diagnose from the
 * driver's seat and trivial to catch here against numbers somebody chose.
 *
 * The cases below are the four failures a textbook PID has on a vehicle -
 * derivative kick, first-step kick, integral windup, and a static term that
 * will not let the car stand still - plus the ones odometry has when nothing is
 * moving.
 *
 * Compiled for the HOST. Neither header touches the SDK.
 *
 * Exits 0 on PASS, 1 on FAIL.
 */

#include "../lib/control.hxx"
#include "../lib/chassis/odom.hxx"

#include <stdio.h>
#include <math.h>

using namespace bibo;

static Int32 failures = 0;
static Int32 checks   = 0;

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

static Bool near(Float32 a, Float32 b, Float32 tol)
{
    const Float32 d = a - b;
    return ((d < 0.0f) ? -d : d) <= tol;
}

Int32 main(Void)
{
    printf("\ncontrol: feedforward\n\n");

    control::Feedforward ff;
    ff.gainS = 40.0f;
    ff.gainV = 100.0f;
    ff.gainA = 0.0f;

    /* THE ONE THAT KEEPS THE CAR STILL. A static term applied at a target of
     * zero asks for the pulse that just barely moves it, while being told to
     * stand still - a car that will not hold on a bench and reads as a
     * calibration fault rather than a controller bug. */
    check(near(control::predict(&ff, 0.0f, 0.0f), 0.0f, 0.001f),
          "zero demand is zero output, static term included");

    check(near(control::predict(&ff, 1.0f, 0.0f), 140.0f, 0.001f),
          "1 m/s is gainS + gainV");

    check(near(control::predict(&ff, -1.0f, 0.0f), -140.0f, 0.001f),
          "the static term takes the sign of the demand");

    check(control::predict(nullptr, 1.0f, 0.0f) == 0.0f,
          "a null model is 0, not a crash");

    printf("\ncontrol: PID\n\n");

    control::Pid pid;
    pid.kp     = 10.0f;
    pid.ki     = 0.0f;
    pid.kd     = 5.0f;
    pid.outMin = -500.0f;
    pid.outMax = 500.0f;
    control::reset(&pid);

    /* NO DERIVATIVE ON THE FIRST STEP. With no previous measurement the naive
     * version differentiates against zero and produces a large output from a
     * standing start - the car lurches the instant the loop is enabled. */
    const Float32 first = control::step(&pid, 1.0f, 0.0f, 0.02f);
    check(near(first, 10.0f, 0.001f),
          "the first step is proportional only - no derivative kick");

    /* DERIVATIVE ON MEASUREMENT, NOT ERROR. Moving the setpoint must not spike
     * the output: the measurement has not changed, so the damping term has
     * nothing to say. Only the proportional term may move. */
    control::reset(&pid);
    static_cast<Void>(control::step(&pid, 0.0f, 0.0f, 0.02f));
    const Float32 jumped = control::step(&pid, 1.0f, 0.0f, 0.02f);
    check(near(jumped, 10.0f, 0.001f),
          "a setpoint step does not kick the derivative");

    /* And the measurement moving DOES damp. */
    control::reset(&pid);
    static_cast<Void>(control::step(&pid, 1.0f, 0.0f, 0.02f));
    const Float32 damped = control::step(&pid, 1.0f, 0.1f, 0.02f);
    check(damped < 10.0f,
          "a rising measurement damps the output");

    /* A BAD TIMESTEP MUST NOT POISON THE STATE. */
    control::reset(&pid);
    const Float32 zeroDt = control::step(&pid, 1.0f, 0.0f, 0.0f);
    check(near(zeroDt, 10.0f, 0.001f),
          "dt of zero returns the proportional term, not a division by zero");
    check(!isnan(zeroDt) && !isinf(zeroDt), "and it is a real number");

    printf("\ncontrol: windup\n\n");

    /* INTEGRAL WINDUP is the one that hurts on a car. Hold a target it cannot
     * reach - a wheel against a curb - and a naive integral grows for as long
     * as it is held, then dumps the moment the wheel comes free. */
    control::Pid wind;
    wind.kp     = 1.0f;
    wind.ki     = 100.0f;
    wind.kd     = 0.0f;
    wind.iMax   = 200.0f;
    wind.outMin = -100.0f;
    wind.outMax = 100.0f;
    control::reset(&wind);

    for(Int32 i = 0; i < 200; ++i)
    {
        static_cast<Void>(control::step(&wind, 5.0f, 0.0f, 0.02f));
    }
    check(wind.integral <= 200.0f + 0.001f,
          "the integral stays inside iMax while saturated");

    /* The real test: once the obstruction clears, the output must come back
     * promptly rather than holding full throttle while a wound-up integral
     * drains. One step at the target should already be at or below the limit. */
    const Float32 freed = control::step(&wind, 5.0f, 5.0f, 0.02f);
    check(freed <= 100.0f + 0.001f, "and the output never exceeds outMax");

    /* Error pointing back into range must still integrate, or the controller
     * can never unwind at all. */
    control::Pid back;
    back.kp = 0.0f;
    back.ki = 10.0f;
    back.outMin = -10.0f;
    back.outMax = 10.0f;
    control::reset(&back);
    for(Int32 i = 0; i < 50; ++i)
    {
        static_cast<Void>(control::step(&back, 1.0f, 0.0f, 0.02f));
    }
    const Float32 wound = back.integral;
    static_cast<Void>(control::step(&back, -1.0f, 0.0f, 0.02f));
    check(back.integral < wound,
          "an error pointing back into range unwinds the integral");

    check(control::step(nullptr, 1.0f, 0.0f, 0.02f) == 0.0f,
          "a null controller is 0, not a crash");

    printf("\nodom\n\n");

    check(!odom::calibrated(),
          "odometry reports itself UNcalibrated - nothing is measured yet");

    check(odom::metersPerTick() > 0.0f, "meters per tick is positive");

    odom::Wheel w;
    odom::reset(&w, 0u);

    check(near(odom::distance(&w), 0.0f, 0.0001f), "a fresh wheel has gone nowhere");

    for(Int32 i = 0; i < 10; ++i)
    {
        odom::tick(&w);
    }
    check(near(odom::distance(&w), 10.0f * odom::metersPerTick(), 0.0001f),
          "ten ticks is ten tick-lengths");

    /* FIRST UPDATE HAS NO WINDOW. There is no previous timestamp, so a speed
     * computed from one would be distance over zero - or worse, over the whole
     * uptime. It must report zero and prime instead. */
    odom::Wheel s;
    odom::reset(&s, 1000u);
    check(near(odom::update(&s, 2000u, 0.0f), 0.0f, 0.0001f),
          "the first update primes and reports zero");

    /* A clock that does not advance must not divide by zero. */
    check(!isnan(odom::update(&s, 2000u, 0.0f)), "a repeated timestamp is safe");

    /* Unfiltered: one tick in exactly one second is one tick-length per second. */
    odom::Wheel r;
    odom::reset(&r, 0u);
    static_cast<Void>(odom::update(&r, 0u, 0.0f));
    odom::tick(&r);
    const Float32 v = odom::update(&r, 1000000u, 0.0f);
    check(near(v, odom::metersPerTick(), 0.0001f),
          "one tick in one second is one tick-length per second");

    /* Filtered, the same input must approach but not exceed it - a lag that
     * overshoots is not a lag. */
    odom::Wheel fw;
    odom::reset(&fw, 0u);
    static_cast<Void>(odom::update(&fw, 0u, 0.5f));
    odom::tick(&fw);
    const Float32 lagged = odom::update(&fw, 1000000u, 0.5f);
    check(lagged > 0.0f && lagged <= odom::metersPerTick() + 0.0001f,
          "the filter approaches the raw value without overshooting it");

    check(odom::distance(nullptr) == 0.0f, "a null wheel is 0, not a crash");

    printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
