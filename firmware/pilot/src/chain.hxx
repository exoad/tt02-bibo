// chain - the loaded behaviours, in order, and the one steer and throttle they
// resolve to.
//
// A bundle is a Behaviour: it sees a scan and says what the car should do.
// Several are loaded at once and cooperate, so the chain is what makes
// "several" mean ONE pair of numbers by the time carrules::Inputs is filled -
// that struct carries one steer and one throttle and never learns that bundles
// exist. See docs/bundles.md section 2.
//
// PURE, like reactive.hxx and the decisions in carrules.hxx: no port, socket,
// clock or lidar SDK. The scan arrives as a bibo::Scan, the elapsed time as
// dtMs, and the viewer's WASD as two numbers the HOST already read. A behaviour
// calling viewfeed itself would be Linux-only and would drag a socket into
// every test; kept out here, the whole safety argument below runs on synthetic
// scans in tests/test_chain.cxx, with no board and no viewer.
//
// THE CHAIN STARTS AT FULL AUTHORITY AND ONLY EVER DESCENDS. This is the whole
// design, and it is the opposite of the obvious one. If a pass started at zero
// and behaviours ADDED throttle, "a bundle may only reduce authority" could not
// be true of the first one, and a safety behaviour would be a promise about how
// bundles are written rather than a fact about what they can express. So a pass
// opens at ceiling 1.0 - everything this car can do - and each behaviour can
// only take some away.
//
// A CLAMP CANNOT EXPRESS A STEERING CHANGE. Reply has three shapes rather than
// one: nothing(), clampTo(ceiling) and propose(throttle, steer). A clamp
// carries a ceiling and no steering at all, so two clamps commute because there
// is nothing for them to disagree about. That is a property of the TYPE, not of
// the two clamps that happen to be written today.
//
// CEILINGS ACCUMULATE AND ARE APPLIED LAST, so a driver loaded after a safety
// behaviour is still clamped by it. Load order cannot be used, by accident or
// by a viewer, to get out from under a clamp.
//
// AN EMPTY CHAIN PROPOSES NOTHING, which is not the same as proposing zero.
// Outcome::drive is then false and the host calls nothing, so the Car's
// freshness rules bring the throttle to neutral (car.hxx, DRIVE_FRESH_MS). A
// behaviour proposing zero is a different thing: an active zero that holds the
// steering where it is.
#pragma once

#include "shared.hxx"

#include "car.hxx"

namespace chain
{
  // The behaviours this build ships with. Reverse-DNS and stable: a viewer keys
  // a window's saved position on the id, so it outlives a rename of the name.
  constexpr CharSeq ID_APRILTAG = "net.exoad.tt02bibo.apriltag";
  constexpr CharSeq ID_WASD = "net.exoad.tt02bibo.wasd";
  constexpr CharSeq ID_FORWARD = "net.exoad.tt02bibo.forward";
  constexpr CharSeq ID_STOP = "net.exoad.tt02bibo.stop";
  constexpr CharSeq ID_WEAVE = "net.exoad.tt02bibo.weave";
  constexpr CharSeq ID_TRIM = "net.exoad.tt02bibo.trim";

  // forward, and the stop clamp: the numbers from programs/forward.cxx, which
  // stays a standalone Car program running the same idea.
  constexpr Float32 CREEP_THROTTLE = 0.10f;
  constexpr Float32 STOP_AT_M = 0.60f;
  constexpr Float32 GO_AT_M = 0.80f;

  // weave: the numbers from programs/weave.cxx. PLENTY_M is the room beyond
  // which more room counts the same, so an open room steers straight instead
  // of chasing the furthest wall.
  constexpr Float32 WEAVE_CRUISE = 0.12f;
  constexpr Float32 PLENTY_M = 2.0f;

  // What a behaviour is allowed to say. Three shapes, so that what a clamp
  // cannot do is checked by the compiler rather than by the host.
  enum class Act
  {
      ACT_NOTHING = 0,   // no opinion this pass; the trim bundle is always this
      ACT_CLAMP,         // lower the ceiling. CARRIES NO STEERING, on purpose
      ACT_PROPOSE,       // a driver: steering, and a throttle every ceiling still cuts
  };

  struct Reply
  {
      Act act = Act::ACT_NOTHING;
      Float32 throttle = 0.0f;   // ACT_CLAMP: the ceiling. ACT_PROPOSE: what it asks for
      Float32 steer = 0.0f;      // ACT_PROPOSE only; unread otherwise
  };

  [[nodiscard]] Reply nothing();

  // Throttle may not exceed ceiling for the rest of this pass. NaN and anything
  // below 0 are read as 0: a clamp that cannot be understood is the strongest
  // clamp, never the weakest. Above 1 is a no-op rather than a stop, since a
  // pass never opens above 1 and refusing to move is a strange answer to a
  // number that asked for no limit at all.
  [[nodiscard]] Reply clampTo(Float32 ceiling);

  // Steering, and a throttle asked for. Both are clamped to their ranges and
  // NaN counts as 0, as bibo::Car::drive does.
  [[nodiscard]] Reply propose(Float32 throttle, Float32 steer);

  // What the chain has decided so far, and what each behaviour is shown.
  struct Intent
  {
      Float32 ceiling = 1.0f;    // 0..1; only ever descends within a pass
      Float32 asked = 0.0f;      // the newest proposal's throttle, before ceilings
      Float32 steer = 0.0f;      // -1..1, negative left, as car.hxx spells it
      Bool driven = false;       // some behaviour has proposed
  };

  // What every behaviour is shown for one pass.
  struct Pass
  {
      // Never null while a run is going; a null scan reads as blind.
      const bibo::Scan* scan = nullptr;

      Intent soFar;              // what the behaviours before this one decided
      Int32 dtMs = 0;            // since the previous pass
      Int64 nowMs = 0;           // one steady clock, as carrules::Inputs uses

      // The viewer's WASD as the host read it this pass. Here rather than
      // fetched, so the chain needs no socket - see the header note.
      Bool haveHolder = false;
      Float32 manualSteer = 0.0f;
      Float32 manualThrottle = 0.0f;
  };

  // One loaded bundle. Owned by the Chain.
  class Behaviour
  {
  public:
      virtual ~Behaviour() = default;

      [[nodiscard]] virtual CharSeq id() const = 0;
      [[nodiscard]] virtual CharSeq name() const = 0;

      // Whether this may ever propose a non-zero throttle. FALSE BY DEFAULT,
      // which is docs/bundles.md section 3's rule for anything new: a behaviour
      // that has not said it drives cannot start driving by being written
      // carelessly. The host clamps a proposal from one that may not.
      [[nodiscard]] virtual Bool mayDrive() const;

      // Joining and leaving the chain, between passes. Default to nothing.
      virtual Void onLoad();
      virtual Void onUnload();

      // One pass. Called in load order, with p.soFar carrying what the
      // behaviours before it decided.
      [[nodiscard]] virtual Reply step(const Pass& p) = 0;
  };

  // How one pass came out.
  struct Outcome
  {
      Intent intent;

      // Whether to call bibo::Car::drive at all. False for an empty chain and
      // for one where nobody proposed: the freshness rules then bring the
      // throttle to neutral, which is not the same as being told zero.
      Bool drive = false;

      Float32 throttle = 0.0f;   // 0..1, already min(asked, ceiling)
      Float32 steer = 0.0f;      // -1..1

      // Proposals a ceiling cut down, and acts refused outright (a proposal
      // from a behaviour that may not drive). Counted so the board can say so:
      // a rule enforced only by convention is not a rule.
      Size clamped = 0;
      Size refused = 0;

      // The newest refusal in words, for EVENT_CODE_BUNDLE and BUNDLE_STATE.
      Str lastRefusal;
  };

  // The loaded behaviours, in load order. Not thread-safe: only the host's own
  // loop touches it, and a load or an unload takes effect BETWEEN passes.
  class Chain
  {
  public:
      // Adds at the end. False, with why set, for a null behaviour, an empty
      // id, or an id already loaded - two windows keyed on one id would be one
      // window, and the second bundle would be invisible.
      [[nodiscard]] Bool load(UniqPtr<Behaviour> b, Str& why);

      // Removes the behaviour with this id. False when nothing had it.
      [[nodiscard]] Bool unload(CharSeq wanted);

      [[nodiscard]] Size size() const;
      [[nodiscard]] Bool has(CharSeq wanted) const;

      // Borrowed, and null past the end. Load order, which is chain order.
      [[nodiscard]] const Behaviour* at(Size i) const;

      // Runs every loaded behaviour over one scan. seed carries the scan, the
      // times and the manual input; its soFar is ignored, because a pass always
      // opens at full authority.
      [[nodiscard]] Outcome run(const Pass& seed);

  private:
      Vec<UniqPtr<Behaviour>> loaded;
  };

  // The behaviours this build ships with, each the same shape as any other
  // bundle. Ports of what already exists, so nothing here is a new idea about
  // how the car drives:
  //
  //   wasd     PROPOSES the viewer's steering and throttle, and nothing while
  //            no one holds the wheel. What pilot --manual does today.
  //   forward  PROPOSES a straight creep. programs/forward.cxx, which stays.
  //   stop     CLAMPS the throttle to zero while something is in front, and
  //            never steers. The reactive stop, as a clamp, so it composes with
  //            either driver above and cannot fight it for the wheel.
  [[nodiscard]] UniqPtr<Behaviour> makeWasd();
  [[nodiscard]] UniqPtr<Behaviour> makeForward();
  [[nodiscard]] UniqPtr<Behaviour> makeStop();
  [[nodiscard]] UniqPtr<Behaviour> makeWeave();

  // trim never says anything in a pass. It is a bundle so that trimming has a
  // window a person loads like any other, and so a viewer's list is the whole
  // of what the car does; the tuning verbs themselves stay in viewfeed,
  // reviewed once, rather than growing a second path through here.
  [[nodiscard]] UniqPtr<Behaviour> makeTrim();

  // apriltag says nothing in a pass either. Loading it is what starts the
  // detector thread (tags.hxx) and unloading it stops it, done by the host
  // that sees the chain change, so a camera and a thread stay out of here.
  [[nodiscard]] UniqPtr<Behaviour> makeApriltag();

  // THE CATALOG IS THE SOURCE OF TRUTH FOR WHAT CAN BE LOADED, because the
  // chain is what does the loading. A .bundle manifest may describe a
  // behaviour registered here; it can never conjure one that is not, any more
  // than a manifest naming no programs/<name>.cxx can - CMakeLists.txt refuses
  // that at build time, and this is the same rule at run time.
  //
  // Before this existed the board listed MANIFESTS and the chain held CODE, so
  // `weave` would have been offered to a viewer with no behaviour behind it
  // while `wasd` and `stop` could never be listed at all. docs/bundles.md
  // section 11 asks whether manifests survive as tunables; either way they are
  // not what the list is built from.
  struct Entry
  {
      CharSeq id = nullptr;
      CharSeq name = nullptr;
      CharSeq about = nullptr;
      UInt8 needs = 0;       // bibowire::BUNDLE_NEEDS_*
      Bool drives = false;   // agrees with the behaviour's own mayDrive()
  };

  // Every behaviour this build can load, SORTED BY ID and so stable across
  // runs: VERB_LOAD_BUNDLE names one by its index in this list, so two runs of
  // the same board must number the same bundle the same way, or a viewer's
  // Load button means something different after a restart.
  [[nodiscard]] const Vec<Entry>& catalog();

  // A fresh behaviour for a catalog id, or null when nothing has that id.
  [[nodiscard]] UniqPtr<Behaviour> make(CharSeq wanted);

  // What this board has, MEASURED BY THE HOST and never declared by a bundle:
  // the difference between a row a viewer greys out with a reason and a car
  // that loads something it cannot run.
  struct Present
  {
      Bool lidar = false;
      Bool pico = false;
      Bool camera = false;
  };

  // Whether every bit in `needs` is met. BUNDLE_MAY_DRIVE is not a need.
  [[nodiscard]] Bool readyFor(UInt8 needs, const Present& have);

  // THE LOADED SET AS A FILE, so it survives a restart of the board: one id per
  // line in chain order, '#' to end of line a comment. Chain order and not
  // only the set, because two drivers do not commute (run()).
  [[nodiscard]] Str formatSet(const Chain& live);

  // Reads one back into `ids`, in file order, each id once. An id the catalog
  // lacks is skipped and named in `unknown`, never a reason to refuse the
  // rest: a bundle dropped from a build must not stop the others loading.
  // False when anything was skipped.
  [[nodiscard]] Bool parseSet(const Str& text, Vec<Str>& ids, Str& unknown);
}
