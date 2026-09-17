#include "chain.hxx"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace chain
{
  namespace
  {
    // NaN counts as 0 everywhere here, as bibo::Car::drive does, so one bad
    // float in a behaviour cannot make a comparison below answer false and let
    // a ceiling through.
    [[nodiscard]] Float32 real(Float32 v)
    {
        return std::isnan(v) ? 0.0f : v;
    }

    [[nodiscard]] Float32 unitThrottle(Float32 v)
    {
        return std::clamp(real(v), 0.0f, 1.0f);
    }

    [[nodiscard]] Float32 unitSteer(Float32 v)
    {
        return std::clamp(real(v), -1.0f, 1.0f);
    }

    [[nodiscard]] Bool sameId(CharSeq a, CharSeq b)
    {
        return a != nullptr && b != nullptr && std::strcmp(a, b) == 0;
    }

    // Metres ahead, with a null or blind scan reading 0 - something touching
    // the car. car.hxx's Scan::ahead already answers 0 for a blind scan; this
    // extends the same answer to having no scan at all.
    [[nodiscard]] Float32 aheadM(const Pass& p)
    {
        return (p.scan != nullptr) ? p.scan->ahead() : 0.0f;
    }
  }

  Reply nothing()
  {
      Reply r;
      r.act = Act::ACT_NOTHING;
      return r;
  }

  Reply clampTo(Float32 ceiling)
  {
      Reply r;
      r.act = Act::ACT_CLAMP;
      r.throttle = unitThrottle(ceiling);
      return r;
  }

  Reply propose(Float32 throttle, Float32 steer)
  {
      Reply r;
      r.act = Act::ACT_PROPOSE;
      r.throttle = unitThrottle(throttle);
      r.steer = unitSteer(steer);
      return r;
  }

  Bool Behaviour::mayDrive() const
  {
      return false;
  }

  Void Behaviour::onLoad()
  {
  }

  Void Behaviour::onUnload()
  {
  }

  Bool Chain::load(UniqPtr<Behaviour> b, Str& why)
  {
      if(!b)
      {
          why = "no behaviour";
          return false;
      }
      const CharSeq wanted = b->id();
      if(wanted == nullptr || wanted[0] == '\0')
      {
          why = "a behaviour with no id";
          return false;
      }
      // Two windows keyed on one id would be one window, so the second bundle
      // would load and then be invisible to whoever loaded it.
      if(has(wanted))
      {
          why = Str("already loaded: ") + wanted;
          return false;
      }
      b->onLoad();
      loaded.push_back(std::move(b));
      return true;
  }

  Bool Chain::unload(CharSeq wanted)
  {
      for(Size i = 0; i < loaded.size(); ++i)
      {
          if(sameId(loaded[i]->id(), wanted))
          {
              loaded[i]->onUnload();
              loaded.erase(loaded.begin() + static_cast<ISize>(i));
              return true;
          }
      }
      return false;
  }

  Size Chain::size() const
  {
      return loaded.size();
  }

  Bool Chain::has(CharSeq wanted) const
  {
      for(const UniqPtr<Behaviour>& b : loaded)
      {
          if(sameId(b->id(), wanted))
          {
              return true;
          }
      }
      return false;
  }

  const Behaviour* Chain::at(Size i) const
  {
      return (i < loaded.size()) ? loaded[i].get() : nullptr;
  }

  Outcome Chain::run(const Pass& seed)
  {
      Outcome o;
      Pass p = seed;
      // A pass always opens at full authority, whatever the caller put in
      // seed.soFar: see the header. Everything below can only take some away.
      p.soFar = Intent();
      for(const UniqPtr<Behaviour>& b : loaded)
      {
          const Reply r = b->step(p);
          if(r.act == Act::ACT_CLAMP)
          {
              // Accumulated, not applied here. Applying it to the proposal so
              // far would let a driver loaded afterwards escape it.
              p.soFar.ceiling = std::min(p.soFar.ceiling, unitThrottle(r.throttle));
          }
          else if(r.act == Act::ACT_PROPOSE)
          {
              Float32 want = unitThrottle(r.throttle);
              if(want > 0.0f && !b->mayDrive())
              {
                  want = 0.0f;
                  ++o.refused;
                  o.lastRefusal = Str(b->id()) + " asked for throttle but may not drive";
              }
              // LAST PROPOSAL WINS. docs/bundles.md section 10 leaves the rule
              // for two drivers open; this is the choice, and it is written
              // down there rather than only here.
              p.soFar.asked = want;
              p.soFar.steer = unitSteer(r.steer);
              p.soFar.driven = true;
          }
      }
      o.intent = p.soFar;
      o.drive = p.soFar.driven;
      o.steer = p.soFar.steer;
      o.throttle = std::min(p.soFar.asked, p.soFar.ceiling);
      // At most one proposal survives a pass, so this counts the resolved one.
      if(o.drive && p.soFar.asked > p.soFar.ceiling)
      {
          ++o.clamped;
      }
      return o;
  }

  namespace
  {
    // The viewer's WASD, as pilot --manual drives today. It proposes what the
    // host read from the control slot and nothing else: the deadman, the arm
    // epoch and the control-slot rules stay in viewfeed, above this.
    class Wasd final : public Behaviour
    {
    public:
        CharSeq id() const override
        {
            return ID_WASD;
        }

        CharSeq name() const override
        {
            return "wasd";
        }

        Bool mayDrive() const override
        {
            return true;
        }

        Reply step(const Pass& p) override
        {
            // NOBODY HOLDING THE WHEEL IS NOTHING TO SAY, NOT A STOP. Saying
            // stop would fight an autonomous behaviour loaded beside this one,
            // and with nothing else loaded the Car's freshness rules bring the
            // throttle to neutral anyway - car.hxx, DRIVE_FRESH_MS.
            if(!p.haveHolder)
            {
                return nothing();
            }
            return propose(p.manualThrottle, p.manualSteer);
        }
    };

    // programs/forward.cxx as a behaviour, hysteresis and all. It keeps its own
    // stopping rather than leaning on the stop clamp, so that loading it alone
    // is not a car that drives into a wall; stop is then defence in depth.
    class Forward final : public Behaviour
    {
    public:
        CharSeq id() const override
        {
            return ID_FORWARD;
        }

        CharSeq name() const override
        {
            return "forward";
        }

        Bool mayDrive() const override
        {
            return true;
        }

        Void onLoad() override
        {
            blocked = true;
        }

        Reply step(const Pass& p) override
        {
            const Float32 ahead = aheadM(p);
            if(ahead < STOP_AT_M)
            {
                blocked = true;
            }
            else if(ahead > GO_AT_M)
            {
                blocked = false;
            }
            return propose(blocked ? 0.0f : CREEP_THROTTLE, 0.0f);
        }

    private:
        // Starts blocked, so the first pass after a load never creeps on a scan
        // that has not arrived yet.
        Bool blocked = true;
    };

    // The reactive stop, as a CLAMP: it holds the throttle at zero while
    // something is in front and never touches the steering, so it composes with
    // any driver and cannot fight one for the wheel. mayDrive() stays false,
    // which costs it nothing - a clamp never proposes.
    class Stop final : public Behaviour
    {
    public:
        CharSeq id() const override
        {
            return ID_STOP;
        }

        CharSeq name() const override
        {
            return "stop";
        }

        Void onLoad() override
        {
            blocked = true;
        }

        Reply step(const Pass& p) override
        {
            // The gap between the two distances stops the car twitching at the
            // edge, and a blind scan reads 0, so it blocks.
            const Float32 ahead = aheadM(p);
            if(ahead < STOP_AT_M)
            {
                blocked = true;
            }
            else if(ahead > GO_AT_M)
            {
                blocked = false;
            }
            // Nothing, not clampTo(1), while the way is clear: a ceiling of 1
            // is the same answer and this way the chain shows who is limiting.
            return blocked ? clampTo(0.0f) : nothing();
        }

    private:
        Bool blocked = true;
    };
  }

  namespace
  {
    // programs/weave.cxx as a behaviour: steer toward the side with more room,
    // and stop while something is in front. The second DRIVER, which is what
    // makes the "last proposal wins" rule in run() something a person can hit
    // rather than a line in a document.
    class Weave final : public Behaviour
    {
    public:
        CharSeq id() const override
        {
            return ID_WEAVE;
        }

        CharSeq name() const override
        {
            return "weave";
        }

        Bool mayDrive() const override
        {
            return true;
        }

        Reply step(const Pass& p) override
        {
            if(p.scan == nullptr)
            {
                return propose(0.0f, 0.0f);
            }
            // Negative bearings are LEFT and positive are RIGHT, the same sign
            // as steer, so more room on the right gives a positive value that
            // steers right. Both read 0 on a blind scan, which steers straight.
            const Float32 left = std::min(p.scan->nearest(-80.0f, -15.0f), PLENTY_M);
            const Float32 right = std::min(p.scan->nearest(15.0f, 80.0f), PLENTY_M);
            const Float32 steer = (right - left) / PLENTY_M;
            return propose(aheadM(p) > STOP_AT_M ? WEAVE_CRUISE : 0.0f, steer);
        }
    };
  }

  namespace
  {
    class Trim final : public Behaviour
    {
    public:
        CharSeq id() const override
        {
            return ID_TRIM;
        }

        CharSeq name() const override
        {
            return "trim";
        }

        Reply step(const Pass&) override
        {
            return nothing();
        }
    };
  }

  namespace
  {
    class Apriltag final : public Behaviour
    {
    public:
        CharSeq id() const override
        {
            return ID_APRILTAG;
        }

        CharSeq name() const override
        {
            return "apriltag";
        }

        Reply step(const Pass&) override
        {
            return nothing();
        }
    };
  }

  UniqPtr<Behaviour> makeApriltag()
  {
      return makeUniq<Apriltag>();
  }

  namespace
  {
    class Creep final : public Behaviour
    {
    public:
        CharSeq id() const override
        {
            return ID_CREEP;
        }

        CharSeq name() const override
        {
            return "creep";
        }

        Bool mayDrive() const override
        {
            return true;
        }

        Reply step(const Pass& p) override
        {
            // Nothing seen, or nothing fresh: an ACTIVE zero, wheels straight,
            // so losing the tag stops the car rather than handing the throttle
            // to whatever else is loaded.
            if(p.tags == nullptr || p.tagsAgeMs > TAGS_FRESH_MS || p.tags->tags.empty())
            {
                return propose(0.0f, 0.0f);
            }
            return propose(TAG_CREEP_THROTTLE, 0.0f);
        }
    };
  }

  UniqPtr<Behaviour> makeCreep()
  {
      return makeUniq<Creep>();
  }

  namespace
  {
    // The nearest tag by range, or the first when no range is known.
    [[nodiscard]] const bibowire::Tag* nearest(const bibowire::Tags& t)
    {
        const bibowire::Tag* best = nullptr;
        for(const bibowire::Tag& tag : t.tags)
        {
            if(best == nullptr || (tag.rangeMm > 0 && (best->rangeMm <= 0 || tag.rangeMm < best->rangeMm)))
            {
                best = &tag;
            }
        }
        return best;
    }

    class Follow final : public Behaviour
    {
    public:
        CharSeq id() const override
        {
            return ID_FOLLOW;
        }

        CharSeq name() const override
        {
            return "follow";
        }

        Bool mayDrive() const override
        {
            return true;
        }

        Void onLoad() override
        {
            blocked = true;
        }

        Reply step(const Pass& p) override
        {
            // Lost: an ACTIVE zero with the wheels straight, not nothing(), so a
            // driver that stops seeing its target stops the car rather than
            // handing the throttle back to whatever else is loaded.
            if(p.tags == nullptr || p.tagsAgeMs > TAGS_FRESH_MS || p.tags->tags.empty())
            {
                blocked = true;
                return propose(0.0f, 0.0f);
            }
            const bibowire::Tag* tag = nearest(*p.tags);
            const Float32 bearingDeg = static_cast<Float32>(tag->bearingCdeg) / 100.0f;
            const Float32 steer = std::clamp(bearingDeg / FOLLOW_FULL_LOCK_DEG, -1.0f, 1.0f);
            // Uncalibrated: the tag is somewhere ahead and that is all that is
            // known. Aim at it, and do not move toward a distance nobody measured.
            if(tag->rangeMm <= 0)
            {
                blocked = true;
                return propose(0.0f, steer);
            }
            const Float32 rangeM = static_cast<Float32>(tag->rangeMm) / 1000.0f;
            if(rangeM < FOLLOW_STOP_AT_M)
            {
                blocked = true;
            }
            else if(rangeM > FOLLOW_GO_AT_M)
            {
                blocked = false;
            }
            return propose(blocked ? 0.0f : FOLLOW_CRUISE, steer);
        }

    private:
        Bool blocked = true;
    };
  }

  UniqPtr<Behaviour> makeFollow()
  {
      return makeUniq<Follow>();
  }

  namespace
  {
    class Odometry final : public Behaviour
    {
    public:
        CharSeq id() const override
        {
            return ID_ODOMETRY;
        }

        CharSeq name() const override
        {
            return "odometry";
        }

        Reply step(const Pass&) override
        {
            return nothing();
        }
    };
  }

  UniqPtr<Behaviour> makeOdometry()
  {
      return makeUniq<Odometry>();
  }

  UniqPtr<Behaviour> makeTrim()
  {
      return makeUniq<Trim>();
  }

  UniqPtr<Behaviour> makeWeave()
  {
      return makeUniq<Weave>();
  }

  UniqPtr<Behaviour> makeWasd()
  {
      return makeUniq<Wasd>();
  }

  UniqPtr<Behaviour> makeForward()
  {
      return makeUniq<Forward>();
  }

  UniqPtr<Behaviour> makeStop()
  {
      return makeUniq<Stop>();
  }

  const Vec<Entry>& catalog()
  {
      // Written in id order by hand rather than sorted at run time: the order
      // is part of the wire contract, and a table a person can read in the
      // order a viewer will show it is worth more than one sorted behind them.
      // tests/test_chain.cxx checks the order really is sorted, and checks
      // every row against the behaviour it names, so the two cannot drift.
      static const Vec<Entry> all = {
          // Needs the camera and nothing else: it looks and reports, and
          // never proposes, so a car whose lidar is off can still run it.
          { ID_APRILTAG,
            "apriltag",
            "find tag36h11 AprilTags in the camera and show them in the viewer",
            bibowire::BUNDLE_NEEDS_CAMERA,
            false },
          // Straight ahead while a tag is in view, stopped the frame it is
          // not: the simplest camera driver, and the one to try first.
          { ID_CREEP,
            "creep",
            "creep straight ahead while an AprilTag is in view, stop the instant it is not",
            bibowire::BUNDLE_NEEDS_CAMERA | bibowire::BUNDLE_NEEDS_PICO,
            true },
          // The first camera driver. The host runs the detector for it.
          { ID_FOLLOW,
            "follow",
            "steer toward the nearest AprilTag and creep up to it, stopping short",
            bibowire::BUNDLE_NEEDS_CAMERA | bibowire::BUNDLE_NEEDS_PICO,
            true },
          { ID_FORWARD,
            "forward",
            "creep ahead, stop while something is in front",
            bibowire::BUNDLE_NEEDS_LIDAR | bibowire::BUNDLE_NEEDS_PICO,
            true },
          // Needs the Pico, whose replies carry the wheel count, and nothing
          // else: it reckons and reports, and never proposes.
          { ID_ODOMETRY,
            "odometry",
            "reckon where the car is from the wheel encoder and leave a trail in the viewer",
            bibowire::BUNDLE_NEEDS_PICO,
            false },
          // Needs the lidar and NOT the Pico: it only ever takes throttle away,
          // so it is useful on a car whose board is not talking.
          { ID_STOP,
            "stop",
            "hold the throttle at zero while something is in front",
            bibowire::BUNDLE_NEEDS_LIDAR,
            false },
          // The other way round: it drives the car from a viewer and never
          // looks at a scan.
          { ID_TRIM,
            "trim",
            "set the steering and throttle limits, centre and slew",
            bibowire::BUNDLE_NEEDS_PICO,
            false },
          { ID_WASD,
            "wasd",
            "drive from the viewer's keyboard",
            bibowire::BUNDLE_NEEDS_PICO,
            true },
          { ID_WEAVE,
            "weave",
            "steer toward the side with more room, stop while something is in front",
            bibowire::BUNDLE_NEEDS_LIDAR | bibowire::BUNDLE_NEEDS_PICO,
            true },
      };
      return all;
  }

  UniqPtr<Behaviour> make(CharSeq wanted)
  {
      if(sameId(wanted, ID_WASD))
      {
          return makeWasd();
      }
      if(sameId(wanted, ID_FORWARD))
      {
          return makeForward();
      }
      if(sameId(wanted, ID_STOP))
      {
          return makeStop();
      }
      if(sameId(wanted, ID_WEAVE))
      {
          return makeWeave();
      }
      if(sameId(wanted, ID_TRIM))
      {
          return makeTrim();
      }
      if(sameId(wanted, ID_APRILTAG))
      {
          return makeApriltag();
      }
      if(sameId(wanted, ID_CREEP))
      {
          return makeCreep();
      }
      if(sameId(wanted, ID_FOLLOW))
      {
          return makeFollow();
      }
      if(sameId(wanted, ID_ODOMETRY))
      {
          return makeOdometry();
      }
      return nullptr;
  }

  Bool readyFor(UInt8 needs, const Present& have)
  {
      if((needs & bibowire::BUNDLE_NEEDS_LIDAR) != 0u && !have.lidar)
      {
          return false;
      }
      if((needs & bibowire::BUNDLE_NEEDS_PICO) != 0u && !have.pico)
      {
          return false;
      }
      if((needs & bibowire::BUNDLE_NEEDS_CAMERA) != 0u && !have.camera)
      {
          return false;
      }
      return true;
  }

  Str formatSet(const Chain& live)
  {
      Str out = "# bibo bundles - loaded at the last change, one id per line, in chain order\n";
      for(Size i = 0; i < live.size(); ++i)
      {
          out += live.at(i)->id();
          out += '\n';
      }
      return out;
  }

  Bool parseSet(const Str& text, Vec<Str>& ids, Str& unknown)
  {
      ids.clear();
      unknown.clear();
      Size at = 0;
      while(at < text.size())
      {
          Size nl = text.find('\n', at);
          if(nl == Str::npos)
          {
              nl = text.size();
          }
          Str line = text.substr(at, nl - at);
          at = nl + 1u;
          const Size hash = line.find('#');
          if(hash != Str::npos)
          {
              line.erase(hash);
          }
          // Both ends, a CR included.
          while(!line.empty() && (line.back() == ' ' || line.back() == 9 || line.back() == 13))
          {
              line.pop_back();
          }
          Size lead = 0;
          while(lead < line.size() && (line[lead] == ' ' || line[lead] == 9))
          {
              ++lead;
          }
          line.erase(0, lead);
          if(line.empty())
          {
              continue;
          }
          Bool known = false;
          for(const Entry& e : catalog())
          {
              known = known || line == e.id;
          }
          if(!known)
          {
              unknown += unknown.empty() ? line : ", " + line;
              continue;
          }
          Bool dup = false;
          for(const Str& have : ids)
          {
              dup = dup || have == line;
          }
          if(!dup)
          {
              ids.push_back(line);
          }
      }
      return unknown.empty();
  }
}
