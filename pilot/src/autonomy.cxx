// See autonomy.hxx. The tunings are real; step() is not, and says so.

#include "autonomy.hxx"

namespace autonomy
{
  namespace
  {

    Config current;

  } // namespace

  CharSeq why(Status s)
  {
      switch(s)
      {
      case Status::STATUS_OK:              return "ok";
      case Status::STATUS_NOT_IMPLEMENTED: return "not implemented yet";
      case Status::STATUS_NO_LINK:         return "the car is not reachable";
      case Status::STATUS_NO_PATH:         return "no path to follow";
      case Status::STATUS_NO_ODOM:         return "no wheel data";
      case Status::STATUS_ARRIVED:         return "the path is finished";
      case Status::STATUS_BAD_TUNING:      return "the tuning is not usable";
      case Status::STATUS_STALE:           return "the car has gone quiet";
      default:                             return "?";
      }
  }

  const Config& tuning()
  {
      return current;
  }

  Bool configure(const Config& c)
  {
      // Refused rather than clamped. A tick rate of zero is somebody's mistake
      // and rounding it to one hertz hides which line has the bug.
      if(c.tickHz <= 0.0f || c.tickHz > 1000.0f || c.silenceMs <= 0)
      {
          return false;
      }
      current = c;
      return true;
  }

  Status step(const Inputs& in, const bibo::pursuit::Path* path, Outputs* out)
  {
      static_cast<Void>(in);
      static_cast<Void>(path);
      static_cast<Void>(out);

      // The shape this will take, so the next person does not have to rederive
      // it from the headers:
      //
      //   1. ticks and dt          -> bibo::chassis::odom, wheel speed
      //   2. speed and steer       -> bibo::kinematics::integrate, a new pose
      //   3. pose and path         -> bibo::pursuit::aim, a curvature
      //   4. curvature             -> bibo::kinematics::steerFraction
      //   5. curvature and limits  -> bibo::plan::speedFor, a target speed
      //   6. target and measured   -> bibo::control::command, a pulse
      //
      // Steps 3 and 4 exist today. Step 5 is a stub in plan.hxx. Steps 1, 2 and
      // 6 have their arithmetic and no caller. None of that is written here yet
      // because none of it can be checked against a car that has no encoder -
      // see the note in autonomy.hxx about what a plausible steering angle
      // looks like.
      return Status::STATUS_NOT_IMPLEMENTED;
  }

} // namespace autonomy
