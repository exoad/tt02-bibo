// ---------------------------------------------------------------------------
// The car's face: the PARAMETERS, not the picture.
//
// The windshield carries TWO screens, one per eye, side by side. Each is
// 240 x 320, PORTRAIT - the long axis vertical. Each shows exactly one thing: a
// rounded rectangle. That is the whole vocabulary: no pupil, no brow, no lid, no
// mouth. Five numbers per eye.
//
// EVERYTHING HERE IS IN PANEL PIXELS. Not fractions, not normalized units -
// pixels of a 240 x 320 canvas, with the origin at the screen's center. This
// module was written in fractions first and converting it was the right call:
// these numbers are going to become the firmware's numbers, the firmware has a
// 240 x 320 framebuffer and no floating-point unit worth using, and a table of
// fractions would have to be multiplied out by hand by whoever ports it - which
// is a table of opportunities to be off by one panel dimension.
//
// A pixel table is also the only form in which "will this read at true size?"
// is a question anybody can answer. At four times scale every corner looks
// generous; at 240 x 320 a 32 px radius is a third of the eye's width.
//
// WHY PARAMETERS AND NOT SPRITES. A sprite sheet can only cut from one frame to
// another, so every pair of expressions needs its own hand-drawn tween and the
// set grows as the square of the number of faces. Interpolating ten numbers
// gives all of them for free, at any frame rate. It is also the only form that
// survives the trip to the real panel: 240 x 320 with no filesystem can
// rasterize a rounded rectangle from five numbers and cannot hold a sprite
// sheet.
//
// WHY THE CORNER IS NOT IN HERE. It used to be, as a per-expression radius
// expressed as a fraction of the eye's OWN shorter side, and that was wrong in a
// way worth writing down: it gave a short eye small corners and a tall eye large
// ones, so every expression spoke a different corner language and the set
// stopped looking like one face. The corner is now ONE ABSOLUTE size in panel
// pixels - CORNER_PX below - applied identically to every pose. The only thing
// that may change it is geometry: a corner can never exceed half the rectangle's
// own shorter side, so a nearly-shut eye degrades to a stadium bar rather than
// drawing wrong.
//
// WHY SO LITTLE IS ENOUGH. Every expression has to be readable from a
// rectangle's proportions, its place on the panel and its angle, because that is
// all there is. TILT carries anger and sadness - inner end down is angry, inner
// end up is sad, and it is the strongest cue in the set. HEIGHT carries
// sleepiness and surprise. The x/y offsets carry looking about. ASYMMETRY
// between the two eyes carries confusion and thinking. And the rest is carried
// by HOW IT MOVES, which is the other half of this file.
//
// The eye is a ROUNDED RECTANGLE and has to keep looking like one, so the
// resting shape is 150 x 200 - about 1 : 1.33 - and every pose stays clearly
// taller or clearly wider than it is the other way. A pose that wants a shorter
// eye takes the HEIGHT down and keeps the width, which makes a wide bar - still
// a rectangle. A rectangle whose sides are equal is a square, and a square with
// a generous corner is a circle.
//
// ---------------------------------------------------------------------------
// THE MOTION IS THE PERSONALITY
//
// Nothing here eases. Every shape parameter is a SPRING - a position and a
// velocity integrated per frame - pulled toward the target expression's resting
// value. Springs were chosen over the smoothstep this started with for one
// reason: an eased transition arrives and stops, and a face that arrives and
// stops looks like a slide changing. A spring OVERSHOOTS and settles, which is
// follow-through, and it is what makes the same twelve poses read as a thing
// that is alive.
//
// Each expression carries its OWN frequency and damping, so they do not all move
// alike: SURPRISED snaps and rings, SLEEPY is slow and heavy, FOCUSED arrives
// and stays put. See `Spring` and the table in face.cxx.
//
// On top of the springs, the four animation principles a rectangle can still
// express:
//
//   ANTICIPATION    the height target is briefly pulled to the WRONG SIDE of
//                   where the eye already is, so surprise pops and sleepy sags.
//   SQUASH/STRETCH  width is derived from the height's VELOCITY - collapsing
//                   fast makes the eye wider, snapping open makes it narrower.
//                   A few percent, and it is what gives the shape weight.
//   FOLLOW-THROUGH  the spring's own overshoot, plus a second-order settle on
//                   the gaze so the eyes never stop dead.
//   A BEAT          WINK is not a pose, it is a scripted gesture: shut in 70 ms,
//                   hold 90 ms, reopen past open and settle, while the other eye
//                   widens and tips a little in sympathy.
//
// NOTHING MAY LEAVE THE PANEL. Overshoot is the point of the file, so the
// guardrail is not "do not exceed the target" - it is that the drawn rectangle,
// rotated, always fits inside 240 x 320. step() shrinks and then re-centers to
// guarantee it; see fitToPanel in face.cxx.
//
// Pure, in the manner of lights.hxx: a State the caller owns, a clock in
// milliseconds, no globals, no device, nothing to initialize. Deterministic
// under its own seeded generator, which is what makes all of the above testable,
// and it is tested (tests/test_face.cxx).
//
// THE TRAP, which is why the test exists. A blink MULTIPLIES the current height;
// it does not replace it. Written the obvious way - "during a blink, h = the
// blink curve" - a SLEEPY face whose eyes are meant to sit at 38 px snaps to
// full height every few seconds, and a wink un-winks itself mid-hold.
// Multiplying composes: 38 * 1.0 is still 38, and a shut eye stays shut through
// any blink, because 0 times anything is 0.
// ---------------------------------------------------------------------------
#pragma once

#include "shared.hxx"

namespace face
{

  // THE PANEL. Two of these, one per eye, portrait. Every number in this module
  // is in these pixels.
  inline constexpr Int32 PANEL_W = 240;
  inline constexpr Int32 PANEL_H = 320;

  // The corner, for every pose, in panel pixels. One number, one place. On a
  // 150 px wide resting eye that is a little under half the half-width, which
  // reads as a corner rather than as a pill - and at true size it is the
  // difference between a friendly rectangle and a lozenge.
  inline constexpr Float32 CORNER_PX = 32.0f;

  // One eye, in panel pixels.
  struct Eye
  {
      // The rectangle's size. h = 0 is a closed eye - the renderer still draws
      // it, as a thin bar, because a panel that goes blank reads as a panel that
      // went out.
      Float32 w = 150.0f;
      Float32 h = 200.0f;

      // The center's offset from the PANEL's center. Screen axes, so +x is right
      // and NEGATIVE y IS UP - the same sense as every framebuffer this will be
      // handed to, and the opposite of the one a reader assumes.
      Float32 x = 0.0f;
      Float32 y = 0.0f;

      // Degrees. POSITIVE TIPS THE INNER END DOWN, which is the angry brow that
      // this face does not have. The inner end is the one nearer the nose, so
      // the two eyes mirror each other and the renderer has to know which side it
      // is drawing - see the `inner` argument at the call site.
      Float32 tilt = 0.0f;
  };

  // One frame's worth of geometry. Everything a renderer needs and nothing about
  // how it is drawn.
  struct Pose
  {
      Eye left;
      Eye right;
  };

  // Per-parameter velocity, in units per second. The same fields as Eye and
  // deliberately NOT an Eye: an Eye's defaults are the neutral face, and a
  // velocity that starts at the neutral face is a face that hurls itself off the
  // panel on its first frame.
  struct EyeRate
  {
      Float32 w = 0.0f;
      Float32 h = 0.0f;
      Float32 x = 0.0f;
      Float32 y = 0.0f;
      Float32 tilt = 0.0f;
  };

  struct PoseRate
  {
      EyeRate left;
      EyeRate right;
  };

  // How an expression MOVES. Natural frequency in radians per second and damping
  // ratio: below 1 overshoots and rings, 1 arrives without overshooting, above 1
  // crawls in. Everything in the table sits between about 0.42 and 1.0, because
  // a face that never overshoots is a slideshow and one that rings for a second
  // is a toy.
  struct Spring
  {
      Float32 freq = 17.0f;
      Float32 damp = 0.60f;
  };

  // CURIOUS and EXCITED were in the first list and are not here.
  //
  // With no pupil and no brow, CURIOUS reduces to "asymmetric, and looking off
  // to one side", which is exactly what CONFUSED reduces to; the two were told
  // apart by a couple of degrees of tilt, and nobody was going to name them
  // correctly. EXCITED reduces to "both eyes large", which is SURPRISED. Two
  // names for one picture is worse than one name, so each pair kept the member
  // the rest of the set needs.
  //
  // The members are EXPRESSION_-prefixed rather than EXPR_-prefixed because
  // docs/conventions.md requires an enum's members to carry the ENUM'S NAME, and
  // hub/tools/style_audit.py checks it.
  enum class Expression
  {
      EXPRESSION_NEUTRAL = 0,
      EXPRESSION_HAPPY,
      EXPRESSION_SURPRISED,
      EXPRESSION_SLEEPY,
      EXPRESSION_FOCUSED,
      EXPRESSION_CONFUSED,
      EXPRESSION_SAD,
      EXPRESSION_ANGRY,
      EXPRESSION_SCANNING,
      EXPRESSION_THINKING,
      EXPRESSION_WINK,
      EXPRESSION_DEAD,
  };

  inline constexpr Int32 EXPRESSION_COUNT = 12;

  [[nodiscard]] CharSeq name(Expression e) noexcept;

  // The expression's RESTING pose, in panel pixels: where the springs are
  // pulling, before any blink, wink, saccade, drift or overshoot.
  [[nodiscard]] Pose posed(Expression e) noexcept;

  // How that expression gets there. Exposed rather than buried so the viewer can
  // show it and a test can assert on it.
  [[nodiscard]] Spring springOf(Expression e) noexcept;

  // True when this rectangle, rotated by its tilt, lies entirely inside the
  // panel. The guarantee step() makes, and the thing the test checks on every
  // frame of a long run.
  [[nodiscard]] Bool insidePanel(const Eye& e) noexcept;

  struct Config
  {
      // NOT a duration any more, and the name is kept because it is still the
      // dial a person reaches for: it SCALES every spring's frequency, with 260
      // meaning "as tuned". Halve it and the whole face moves twice as fast. A
      // spring has no arrival time to name, which is the one thing an eased
      // transition had going for it.
      Float32 transitionMs = 260.0f;

      Float32 blinkEveryMinS = 2.4f;
      Float32 blinkEveryMaxS = 6.5f;

      // 40 ms down, 90 ms up. See BLINK_CLOSE_RATIO in face.cxx.
      Float32 blinkMs = 130.0f;

      // Covers the breathing drift, the saccades and the micro-twitches. They
      // are three mechanisms, but the checkbox says "Idle motion", and somebody
      // who unchecks it wants the face to hold still so they can look at a pose.
      // An expression's OWN motion - SCANNING's sweep, WINK's beat - is not idle
      // motion and keeps running.
      Bool    idleMotion = true;

      Bool    autoBlink = true;
      Float32 speed = 1.0f;
  };

  // Owned by the caller, the way reactive.hxx does it - no statics anywhere in
  // this module, so two faces can run side by side and a test can hold a
  // thousand of them.
  //
  // A default-constructed State is already a drawable NEUTRAL face. Nothing has
  // to be called before `pose` is worth reading.
  struct State
  {
      Pose     pose;    // what to draw THIS frame: shape, plus blink, wink, drift
      Pose     shape;   // where the springs currently ARE, before any of that
      PoseRate rate;    // and how fast each of them is moving

      Expression source = Expression::EXPRESSION_NEUTRAL;
      Expression target = Expression::EXPRESSION_NEUTRAL;

      // 0..1 since the last set(). The SHAPE does not use it - the springs own
      // that - but the sweep weight and the liveliness do, so that leaving
      // SCANNING fades the sweep out instead of dropping it on one frame.
      Float32 blend = 1.0f;

      // Anticipation: where the height is pulled BEFORE it is allowed to go
      // anywhere, decaying over ANTICIPATE_MS. Per eye, because a wink-shaped
      // change is not symmetric.
      Float32 anticipMs = 0.0f;
      Float32 anticipLeftH = 0.0f;
      Float32 anticipRightH = 0.0f;

      // Milliseconds into the current blink. NEGATIVE means no blink is running,
      // which is a state a duration cannot express - 0 is the first frame of one.
      Float32 blinkT = -1.0f;
      Float32 nextBlinkS = 0.0f;   // clock left before the next automatic blink

      // The wink BEAT, on the same footing as the blink and multiplied into the
      // right eye the same way, so the two compose instead of fighting.
      Float32 winkT = -1.0f;
      Float32 winkAgainS = 0.0f;

      // The gaze is a spring too, in panel pixels, which is what gives a saccade
      // its overshoot and its settle rather than a glide that stops dead.
      Float32 gazeX = 0.0f, gazeY = 0.0f;
      Float32 gazeVelX = 0.0f, gazeVelY = 0.0f;
      Float32 gazeToX = 0.0f, gazeToY = 0.0f;
      Float32 saccadeHoldS = 0.0f;
      Float32 microHoldS = 0.0f;   // the small twitches between saccades

      // Last frame's drawn height, for the squash. Squash is derived from the
      // height's RATE, and the rate that matters is the one the viewer sees -
      // spring, blink and wink together - not the spring's alone.
      Float32 prevLeftH = 0.0f, prevRightH = 0.0f;
      Bool    havePrev = false;

      Float32 idlePhase = 0.0f;   // radians - the slow breathing drift
      Float32 scanPhase = 0.0f;   // radians - SCANNING's own sweep

      // The randomness lives HERE rather than in rand(), so a test can step ten
      // thousand frames and assert on the result, and so two faces on one screen
      // do not blink in lockstep. Change it and the whole run changes; leave it
      // and the run repeats exactly.
      UInt32 rng = 0x1BADB002u;

      // The first step draws the first blink and saccade delays. Doing it here
      // rather than in a constructor keeps State an aggregate and keeps the
      // Config out of it.
      Bool seeded = false;
  };

  // Points the springs at a new expression. There is no transition to begin: the
  // springs are already where the face is and already carry its momentum, which
  // is what makes interrupting one mid-move free rather than a case somebody has
  // to handle. Selecting WINK starts its beat.
  Void set(State* st, Expression e) noexcept;

  // Forces a blink now. A blink already running is left alone rather than
  // restarted - restarting it mid-close is a visible stutter.
  Void blink(State* st) noexcept;

  // Forces the wink beat now, whatever the expression. Same rule.
  Void wink(State* st) noexcept;

  // Advances everything by `dtMs` of wall clock and leaves `st->pose` ready to
  // draw. `dtMs` is scaled by cfg.speed, so speed 0 freezes the face exactly, and
  // a dt of 0 changes nothing at all. Long frames are SUB-STEPPED: a spring
  // integrated in one 200 ms leap does not settle, it detonates.
  Void step(State* st, const Config& cfg, Float32 dtMs) noexcept;

  [[nodiscard]] Bool blinking(const State& st) noexcept;
  [[nodiscard]] Bool winking(const State& st) noexcept;

}
