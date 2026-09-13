// forward - creep straight ahead, stop while something is in front, and go
// again once it has moved away.
//
//   forward                          dry run: prints what it would do, the car cannot move
//   forward --drive --seconds 10     drives for 10 s. Car on a stand the first time.
//                                    Once the forward angle is measured (docs/start.md
//                                    step 2), or with --forward DEG.
//
// Ctrl-C, or the ESTOP button in the viewer, stops it at any moment.
//
// To end the run at the first obstacle instead, replace `blocked = true;` inside the loop
// with `break;`.

#include "car.hxx"

// ---- DECLARE -------------------------------------------------------------------
constexpr Float32 CREEP = 0.10f;       // a tenth of the car's forward range
constexpr Float32 STOP_AT_M = 0.60f;   // something nearer than this ahead: stop
constexpr Float32 GO_AT_M = 0.80f;     // move again only once it is farther than this

Int32 main(Int32 argc, Char** argv)
{
    bibo::Car car(argc, argv);

    // ---- BIND --------------------------------------------------------------------
    if(!car.arm())
    {
        return car.finish();
    }

    // ---- RUN ---------------------------------------------------------------------
    // The 0.2 m between STOP_AT_M and GO_AT_M stops the car twitching at the
    // edge. A blind scan reads ahead() == 0, so it stops too.
    Bool blocked = true;
    while(car.ok())
    {
        const bibo::Scan scan = car.scan();
        const Float32 ahead = scan.ahead();

        if(ahead < STOP_AT_M)
        {
            blocked = true;
        }
        else if(ahead > GO_AT_M)
        {
            blocked = false;
        }

        car.drive(blocked ? 0.0f : CREEP, 0.0f);
    }
    return car.finish();
}
