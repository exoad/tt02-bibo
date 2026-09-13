// The Trim window: the car's steering and throttle limits, centre and slew
// rates, set over bibowire COMMAND.
//
// `trimview`, not `trim`: main.cxx holds a `trimview::View trim;`, which would
// hide a namespace of that name.
//
// The Pico holds these in RAM only. The board saves every tuning change it
// accepts to a file on the Pi and re-sends the set whenever the Pico connects;
// this laptop also saves the pane's values (settings.hxx). THE BOARD'S COPY
// WINS: it reports what it has saved on WELCOME and after each save
// (EVENT_CODE_TRIM), follow() takes each report into the sliders, and the
// laptop's file follows the sliders. A board with nothing saved says so, the
// sliders keep the laptop's copy, and "save to car" puts it on the board. The
// window says in one line which copy it shows.
//
// The board refuses tuning verbs while the car is armed (result = 3): changing
// the range a live throttle is clamped to could hurt somebody. The controls are
// disabled with the reason shown, and a refusal that still arrives is shown,
// because the board is the authority.
//
// Every microsecond is an integer and there is no Float32 slider: bibowire
// carries µs as UInt16, and a float slider meets the locale trap (vlog.hxx).
#pragma once

#include "shared.hxx"

#include "link.hxx"

namespace trimview
{
  // CROSS-BOUNDARY COPIES of firmware/lib/chassis/cal.hxx, each correct alone
  // and wrong as a pair once one side moves: IF cal.hxx CHANGES, THESE CHANGE
  // WITH IT. They are only the pane's starting position, used when neither the
  // laptop nor the board has saved trim - never "what the Pico uses now".
  constexpr Int32 STEER_MIN_DEFAULT = 1230;    // cal.hxx STEER_CAL_LEFT
  constexpr Int32 STEER_CENTRE_DEFAULT = 1480; // cal.hxx STEER_CAL_CENTER
  constexpr Int32 STEER_MAX_DEFAULT = 1660;    // cal.hxx STEER_CAL_RIGHT

  // cal.hxx's throttle band predates the current brushless drivetrain, so it is
  // a stale starting point, not measured truth; it stays because a 59 µs band
  // cannot launch the car.
  constexpr Int32 ESC_MIN_DEFAULT = 1541;      // cal.hxx THROTTLE_CAL_MIN
  constexpr Int32 ESC_MAX_DEFAULT = 1600;      // cal.hxx THROTTLE_CAL_MAX

  // Neutral, which is reverse OFF: chassis.hxx ESC_REVERSE_DEFAULT.
  constexpr Int32 ESC_REVERSE_DEFAULT = 1500;

  constexpr Int32 STEER_SLEW_DEFAULT = 8;      // cal.hxx SLEW_CAL_STEER
  constexpr Int32 THROTTLE_SLEW_DEFAULT = 8;   // cal.hxx SLEW_CAL_THROTTLE

  // µs per tick is a unit nobody pictures; a travel time is. These are inline
  // so viewer/tests can check them without linking ImGui.
  // µs per tick to µs per second, at SLEW_TICKS_PER_S ticks a second.
  [[nodiscard]] inline Int32 slewUsPerSec(Int32 usPerTick)
  {
      return usPerTick * static_cast<Int32>(bibowire::SLEW_TICKS_PER_S);
  }

  // Hundredths of a second to travel `spanUs` at `usPerTick`, or -1 for a zero
  // span or rate rather than a divide by zero.
  [[nodiscard]] inline Int64 crossCentis(Int32 spanUs, Int32 usPerTick)
  {
      const Int32 rate = slewUsPerSec(usPerTick);
      if(spanUs <= 0 || rate <= 0)
      {
          return -1;
      }
      return (static_cast<Int64>(spanUs) * 100) / static_cast<Int64>(rate);
  }

  struct View
  {
      // Bound to ImGui::Begin's close button. Unlike the camera window, this one
      // costs the board nothing while open.
      Bool open = false;

      // What the sliders hold; they outlive the window. The board's saved trim
      // once it has reported (follow()), otherwise this laptop's copy. Never a
      // reading from the Pico itself: no message carries one.
      Int32 steerMinUs = STEER_MIN_DEFAULT;
      Int32 steerMaxUs = STEER_MAX_DEFAULT;
      Int32 steerTrimUs = STEER_CENTRE_DEFAULT;
      Int32 escMinUs = ESC_MIN_DEFAULT;
      Int32 escMaxUs = ESC_MAX_DEFAULT;

      // The lowest pulse brake and reverse may reach. Neutral turns reverse OFF
      // and is the default, so S stays a plain stop until this is set on purpose.
      Int32 escReverseUs = ESC_REVERSE_DEFAULT;

      // µs per 20 ms tick, SLEW_US_MIN..SLEW_US_MAX. The slider is logarithmic
      // because the useful end is 1..20.
      Int32 steerSlewUs = STEER_SLEW_DEFAULT;
      Int32 throttleSlewUs = THROTTLE_SLEW_DEFAULT;

      // COMMANDs this pane has sent, shown in the window.
      UInt32 sent = 0;

      // The board report the sliders last took (Session's boardTrimAtMs and
      // boardTrimCount), so each is taken exactly once: every frame would pin the
      // sliders against a drag, and only the first would miss later saves.
      Int64 adoptedAtMs = 0;
      UInt32 adoptedCount = 0;

      // THE IDLE TEST. While ticked and the car armed, CONTROL carries
      // bibowire::BUTTON_IDLE_TEST: the pilot holds the ESC at exactly the idle
      // pulse and the board takes ESC limit changes despite the arm, so the idle
      // can be dragged while the motor is watched. Never saved. follow() clears
      // it after IDLE_TEST_MS, when the car is not armed, and when the window
      // is closed.
      Bool idleTest = false;
      Int64 idleTestSinceMs = 0;

      // When this pane last sent a tuning command, and whether the held slider
      // has an unsent edit. follow() leaves the board's reports alone until the
      // pane has gone quiet.
      Int64 lastSendMs = 0;
      Bool dragUnsent = false;
  };

  // Ctrl+click turns an ImGui slider into a text box, so a slider's range
  // guarantees nothing. Everything is re-clamped before it is sent, which also
  // keeps min below max. In the header so settings.cxx clamps a loaded file to
  // exactly the sliders' ranges.
  [[nodiscard]] inline Int32 clampTo(Int32 value, Int32 lo, Int32 hi)
  {
      if(value < lo)
      {
          return lo;
      }
      if(value > hi)
      {
          return hi;
      }
      return value;
  }

  inline Void settleSteer(View& v)
  {
      const Int32 hardLo = static_cast<Int32>(bibowire::SERVO_US_HARD_MIN);
      const Int32 hardHi = static_cast<Int32>(bibowire::SERVO_US_HARD_MAX);
      v.steerMinUs = clampTo(v.steerMinUs, hardLo, hardHi - 1);
      v.steerMaxUs = clampTo(v.steerMaxUs, v.steerMinUs + 1, hardHi);
      // The centre is bounded by the ends, not the servo's range: a centre
      // outside the limits is unreachable, and 1500 says nothing about where a
      // TT-02's wheels point straight.
      v.steerTrimUs = clampTo(v.steerTrimUs, v.steerMinUs, v.steerMaxUs);
  }

  inline Void settleEsc(View& v)
  {
      const Int32 hardLo = static_cast<Int32>(bibowire::ESC_US_HARD_MIN);
      const Int32 hardHi = static_cast<Int32>(bibowire::ESC_US_HARD_MAX);
      v.escMinUs = clampTo(v.escMinUs, hardLo, hardHi - 1);
      v.escMaxUs = clampTo(v.escMaxUs, v.escMinUs + 1, hardHi);
      // From the hard minimum up to neutral, which is reverse off.
      v.escReverseUs = clampTo(
          v.escReverseUs,
          hardLo,
          static_cast<Int32>(bibowire::ESC_NEUTRAL_US)
      );
  }

  inline Void settleSlew(View& v)
  {
      const Int32 slewLo = static_cast<Int32>(bibowire::SLEW_US_MIN);
      const Int32 slewHi = static_cast<Int32>(bibowire::SLEW_US_MAX);
      v.steerSlewUs = clampTo(v.steerSlewUs, slewLo, slewHi);
      v.throttleSlewUs = clampTo(v.throttleSlewUs, slewLo, slewHi);
  }

  // Every field, in the order the pairs depend on each other.
  inline Void settleAll(View& v)
  {
      settleSteer(v);
      settleEsc(v);
      settleSlew(v);
  }

  // One to five digits and nothing else: no setting is negative.
  [[nodiscard]] inline Opt<Int32> reportNumber(StrView word)
  {
      if(word.empty() || word.size() > 5)
      {
          return {};
      }
      Int32 value = 0;
      for(const Char c : word)
      {
          if(c < '0' || c > '9')
          {
              return {};
          }
          value = (value * 10) + static_cast<Int32>(c - '0');
      }
      return value;
  }

  // `report` is the board's bibowire::EVENT_CODE_TRIM text: the Pico's trim
  // lines joined by "; ", as trimfile::report writes them. EVERY SETTING IT
  // NAMES REPLACES THE PANE'S; a setting it does not name keeps the pane's
  // value. A line that is not exactly one of the five settings with plain
  // digits is skipped whole, never half-read. The result is settled. Returns
  // how many settings were taken. Inline so viewer/tests can check it.
  [[nodiscard]] inline Int32 adoptReport(View& v, StrView report)
  {
      // The longest setting is three words; a fourth makes a line unknown.
      constexpr Size WORDS_MAX = 3;
      Int32 taken = 0;
      while(!report.empty())
      {
          const Size cut = report.find(';');
          StrView line = report.substr(0, cut);
          report = cut == StrView::npos ? StrView() : report.substr(cut + 1);
          Array<StrView, WORDS_MAX + 1> w = {};
          Size count = 0;
          while(!line.empty() && count <= WORDS_MAX)
          {
              if(line.front() == ' ')
              {
                  line.remove_prefix(1);
                  continue;
              }
              const Size gap = line.find(' ');
              w[count] = line.substr(0, gap);
              ++count;
              line = gap == StrView::npos ? StrView() : line.substr(gap);
          }
          if(count == 3 && (w[0] == "SERVOLIMITS" || w[0] == "ESCLIMITS"))
          {
              const Opt<Int32> lo = reportNumber(w[1]);
              const Opt<Int32> hi = reportNumber(w[2]);
              if(!lo.has_value() || !hi.has_value())
              {
                  continue;
              }
              if(w[0] == "SERVOLIMITS")
              {
                  v.steerMinUs = *lo;
                  v.steerMaxUs = *hi;
              }
              else
              {
                  v.escMinUs = *lo;
                  v.escMaxUs = *hi;
              }
              ++taken;
          }
          else if(count == 2 && w[0] == "ESCREVERSE")
          {
              const Opt<Int32> lowest = reportNumber(w[1]);
              if(lowest.has_value())
              {
                  v.escReverseUs = *lowest;
                  ++taken;
              }
          }
          else if(count == 2 && w[0] == "SERVOTRIM")
          {
              const Opt<Int32> centre = reportNumber(w[1]);
              if(centre.has_value())
              {
                  v.steerTrimUs = *centre;
                  ++taken;
              }
          }
          else if(count == 3 && w[0] == "SLEW")
          {
              const Opt<Int32> rate = reportNumber(w[2]);
              if(rate.has_value() && w[1] == "STEER")
              {
                  v.steerSlewUs = *rate;
                  ++taken;
              }
              else if(rate.has_value() && w[1] == "THROTTLE")
              {
                  v.throttleSlewUs = *rate;
                  ++taken;
              }
          }
      }
      settleAll(v);
      return taken;
  }

  // The DPI multiplier the layout uses. Called once, after ImGui exists. This
  // window owns no texture, so there is no shutdown.
  Void init(Float32 scale);

  // One frame. Draws the window when it is open and sends a COMMAND for whatever
  // the operator just finished changing.
  Void drawWindow(View& v, link::Client& lk, const link::Snapshot& snap, Int64 nowMs);

  // Takes the board's newest saved-trim report into `v`, once per report, and
  // logs it. Call EVERY frame, window open or not, so a closed window cannot
  // leave the laptop's file holding numbers the car has already replaced.
  Void follow(View& v, const link::Snapshot& snap);
}
