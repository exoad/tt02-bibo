// The vehicle lighting rules, from docs/conventions.md "Lighting behavior".
//
//   tests\build_lights_test.bat run
//
// These exist because three of the rules look right and are not, and all three
// are the kind that a person glancing at a blinking LED would accept:
//
//   * The indicator OVERRIDES the brake on its own side. Braking while
//     signalling right means the right rear alternates and the left stays
//     solid. Get it wrong and both stay solid, which looks fine and is wrong.
//   * Hazards are both sides IN PHASE. Alternating is what a film prop does.
//   * Front and rear on one side share ONE clock. Per-lamp timers drift, and
//     drift is invisible for the first minute.
//
// No hardware and no ImGui. Exits 0 on PASS, 1 on FAIL.

#include "../src/shared.hpp"
#include "../src/lights.hpp"

#include <cmath>
#include <cstdio>

namespace {

Int32 failures = 0;
Int32 checks   = 0;

Void check(Bool ok, const Char* what)
{
    ++checks;
    if(!ok) { ++failures; std::printf("  FAIL  %s\n", what); }
    else    { std::printf("  ok    %s\n", what); }
}

Void checkNear(Float32 got, Float32 want, const Char* what)
{
    ++checks;
    if(std::fabs(got - want) > 0.001f)
    {
        ++failures;
        std::printf("  FAIL  %s: got %.3f, want %.3f\n",
                    what, static_cast<Float64>(got), static_cast<Float64>(want));
    }
    else
    {
        std::printf("  ok    %s = %.2f\n", what, static_cast<Float64>(got));
    }
}

// A time inside the ON phase, and one inside the OFF phase, of the same cycle.
constexpr Float64 T_ON  = 0.100;
constexpr Float64 T_OFF = 0.500;

Void testTiming()
{
    std::printf("flasher timing\n");

    checkNear(static_cast<Float32>(lights::BLINK_PERIOD_S), 0.667f, "period (s)");

    // 1.5 Hz. The period is what the standard fixes, so that is what is checked
    // rather than the two halves separately.
    const Float32 hz = static_cast<Float32>(1.0 / lights::BLINK_PERIOD_S);
    check(hz > 1.45f && hz < 1.55f, "rate is 1.5 Hz");

    check(lights::blinkPhase(T_ON),  "on inside the on phase");
    check(!lights::blinkPhase(T_OFF), "off inside the off phase");

    // And it repeats, rather than running once and latching.
    check(lights::blinkPhase(T_ON + lights::BLINK_PERIOD_S * 5.0),
          "still on one whole cycle later");
    check(!lights::blinkPhase(T_OFF + lights::BLINK_PERIOD_S * 5.0),
          "still off one whole cycle later");
}

Void testOneClock()
{
    std::printf("one clock for the whole vehicle\n");

    lights::Input in;
    in.turn = lights::Turn::TURN_LEFT;

    // Sampled across a whole cycle: front and rear on a side must never
    // disagree, at any instant.
    Bool everDiffered = false;
    for(Int32 i = 0; i < 200; ++i)
    {
        const Float64 t = static_cast<Float64>(i) * (lights::BLINK_PERIOD_S / 37.0);
        const lights::Lamps l = lights::solve(in, t);
        if(l.indFL != l.indRL)
            everDiffered = true;
    }
    check(!everDiffered, "front and rear left never disagree");
}

Void testHazard()
{
    std::printf("hazards\n");

    lights::Input in;
    in.turn = lights::Turn::TURN_HAZARD;

    Bool everDiffered = false;
    Bool sawOn = false, sawOff = false;
    for(Int32 i = 0; i < 200; ++i)
    {
        const Float64 t = static_cast<Float64>(i) * (lights::BLINK_PERIOD_S / 37.0);
        const lights::Lamps l = lights::solve(in, t);
        if(l.indFL != l.indFR || l.indRL != l.indRR)
            everDiffered = true;
        if(l.indFL > 0.5f) sawOn = true;
        else               sawOff = true;
    }

    check(!everDiffered, "both sides in phase, never alternating");
    check(sawOn && sawOff, "and they do actually blink");
}

Void testTailAndBrake()
{
    std::printf("tail and brake share one red lamp\n");

    lights::Input in;
    checkNear(lights::solve(in, T_OFF).tailL, 0.0f, "dark with everything off");

    in.head = lights::Head::HEAD_ON;
    checkNear(lights::solve(in, T_OFF).tailL, 0.30f, "tail at 30% with lights on");

    in.brake = true;
    checkNear(lights::solve(in, T_OFF).tailL, 1.00f, "brake at 100%");

    // Braking with the lights OFF still lights the brake: a brake lamp does not
    // depend on the headlight switch.
    in.head = lights::Head::HEAD_OFF;
    checkNear(lights::solve(in, T_OFF).tailL, 1.00f, "brake works with lights off");

    lights::Input drl;
    drl.head = lights::Head::HEAD_DRL;
    checkNear(lights::solve(drl, T_OFF).headL, 0.45f, "DRL is the headlamp dimmed");
    drl.head = lights::Head::HEAD_ON;
    checkNear(lights::solve(drl, T_OFF).headL, 1.00f, "headlight full");
}

Void testOverride()
{
    std::printf("the indicator overrides the brake on its own side\n");

    lights::Input in;
    in.brake = true;
    in.turn  = lights::Turn::TURN_RIGHT;

    const lights::Lamps on  = lights::solve(in, T_ON);
    const lights::Lamps off = lights::solve(in, T_OFF);

    // THE ASYMMETRY. This is the whole test.
    checkNear(on.tailR,  0.0f, "right rear dark while its indicator is lit");
    checkNear(off.tailR, 1.00f, "right rear back to brake between flashes");
    checkNear(on.tailL,  1.00f, "left rear stays solid throughout (on phase)");
    checkNear(off.tailL, 1.00f, "left rear stays solid throughout (off phase)");

    checkNear(on.indRR,  1.0f, "and the right indicator is what is lit instead");
    checkNear(off.indRR, 0.0f, "right indicator dark between flashes");

    // Hazards while braking: BOTH sides alternate, neither stays solid.
    lights::Input haz;
    haz.brake = true;
    haz.turn  = lights::Turn::TURN_HAZARD;
    const lights::Lamps h = lights::solve(haz, T_ON);
    check(h.tailL == 0.0f && h.tailR == 0.0f,
          "hazards while braking interrupt both sides");
}

Void testReverse()
{
    std::printf("reverse\n");

    lights::Input in;
    checkNear(lights::solve(in, T_ON).revL, 0.0f, "off by default");

    in.reverse = true;
    in.turn    = lights::Turn::TURN_HAZARD;
    in.brake   = true;

    // Nothing interrupts reverse: it reports the gearbox, which no other signal
    // contradicts.
    checkNear(lights::solve(in, T_ON).revL,  1.0f, "lit through a flash");
    checkNear(lights::solve(in, T_OFF).revR, 1.0f, "lit between flashes");
}

} // namespace

int main()
{
    std::printf("vehicle lighting tests\n\n");

    testTiming();
    testOneClock();
    testHazard();
    testTailAndBrake();
    testOverride();
    testReverse();

    std::printf("\n%d checks, %d failed\n", checks, failures);
    std::printf("%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
