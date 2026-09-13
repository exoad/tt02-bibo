// weave - drive ahead steering toward the side with more room, and stop while
// something is in front.
//
//   weave                            dry run: prints what it would do, the car cannot move
//   weave --drive --seconds 10       drives for 10 s. Car on a stand the first time.
//                                    Once the forward angle is measured (docs/start.md
//                                    step 2), or with --forward DEG.
//
// Ctrl-C, or the ESTOP button in the viewer, stops it at any moment.
#include "car.hxx"

#include <algorithm>

// ---- DECLARE ---------------------------------------------------------------
constexpr Float32 CRUISE = 0.12f;
constexpr Float32 STOP_AT_M = 0.60f;
constexpr Float32 PLENTY_M = 2.0f;     // room beyond this counts the same

Int32 main(Int32 argc, Char** argv)
{
    bibo::Car car(argc, argv);

    // ---- BIND --------------------------------------------------------------
    if(!car.arm())
    {
        return car.finish();
    }

    // ---- RUN ---------------------------------------------------------------
    while(car.ok())
    {
        const bibo::Scan scan = car.scan();
        // Negative bearings are LEFT and positive are RIGHT - the same sign as steer.
        const Float32 left = std::min(scan.nearest(-80.0f, -15.0f), PLENTY_M);
        const Float32 right = std::min(scan.nearest(15.0f, 80.0f), PLENTY_M);
        // More room on the right gives a positive value, which steers right.
        // Always -1..1, and 0 when blind (both sides read 0).
        const Float32 steer = (right - left) / PLENTY_M;
        const Bool clear = scan.ahead() > STOP_AT_M;
        car.drive(clear ? CRUISE : 0.0f, steer);
    }
    return car.finish();
}
