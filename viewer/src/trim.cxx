#include "shared.hxx"

#include <cstdio>

// No <windows.h> and no <d3d11.h> here, unlike camera.cxx: this window owns no
// texture and needs no device, so none of the `small`/`near`/`far`/SEVERITY_ERROR
// macro surgery that file performs is needed.
#include "imgui.h"

#include "trim.hxx"
#include "vlog.hxx"

namespace trimview
{

  namespace
  {

    Float32 uiScale = 1.0f;

    constexpr Float32 READOUT_COLUMN = 104.0f;

    // Every slider is this wide, leaving room for its label. Named once so the
    // column of controls lines up rather than each call picking its own.
    constexpr Float32 ITEM_WIDTH = -128.0f;

    // How often a slider being dragged sends its value. The steering follows the
    // drag on the car, and ten a second reads as live without flooding a queue
    // the pilot drains two lines a tick.
    constexpr Int64 PREVIEW_SEND_MS = 100;

    // How long after this pane's last send the board's trim report is left
    // alone. A report goes out when the board's queue drains, which during a
    // drag falls between two of this pane's own sends - taken then, it would
    // snap the slider back to where the hand has already left.
    constexpr Int64 FOLLOW_QUIET_MS = 800;

    // The idle test turns itself off after this long. A motor held at idle and
    // forgotten about is the thing the timeout exists for.
    constexpr Int64 IDLE_TEST_MS = 20000;

    const ImVec4 TEXT_WARN = ImVec4(1.0f, 0.72f, 0.30f, 1.0f);
    const ImVec4 TEXT_GOOD = ImVec4(0.45f, 0.85f, 0.50f, 1.0f);
    const ImVec4 TEXT_BAD = ImVec4(1.0f, 0.45f, 0.35f, 1.0f);

    // ---- text ---------------------------------------------------------------

    // Centiseconds as "1.07 s", built from two integers. NEVER printf("%.2f"):
    // the decimal point honours the locale, and a comma decimal writes "1,07".
    [[nodiscard]] Str centisText(Int64 centis)
    {
        if(centis < 0)
        {
            return "-";
        }
        Array<Char, 32> t = {};
        const Int64 whole = centis / 100;
        const Int64 rest = centis % 100;
        std::snprintf(t.data(), t.size(), "%lld.%02lld s", whole, rest);
        return Str(t.data());
    }

    Void readout(CharSeq label, CharSeq value)
    {
        ImGui::TextUnformatted(label);
        ImGui::SameLine(READOUT_COLUMN * uiScale);
        ImGui::TextUnformatted(value);
    }

    // ---- is the car in a state that will accept this ------------------------

    // Whether the car is armed, from the freshest thing that says so. CTLSTATE
    // is the authority; BOARD's picoArmed stands in, and its 2 (UNKNOWN) is not
    // taken as armed - the board's own refusal is the backstop if it is.
    [[nodiscard]] Bool carArmed(const link::Snapshot& snap, Int64 nowMs)
    {
        const Opt<link::Control> ctl = snap.state.controlState(nowMs);
        if(ctl.has_value())
        {
            return ctl->state.armed != 0u;
        }
        const Opt<link::Board> board = snap.state.boardState(nowMs);
        if(board.has_value())
        {
            return board->state.picoArmed == 1u;
        }
        return false;
    }

    [[nodiscard]] Bool isTrimVerb(bibowire::Verb v)
    {
        return v == bibowire::Verb::VERB_SET_SERVO_LIMITS
            || v == bibowire::Verb::VERB_SET_SERVO_TRIM
            || v == bibowire::Verb::VERB_SET_ESC_LIMITS
            || v == bibowire::Verb::VERB_SET_ESC_REVERSE
            || v == bibowire::Verb::VERB_SET_SLEW;
    }

    // ---- sending ------------------------------------------------------------

    Void sendServoLimits(View& v, link::Client& lk)
    {
        const UInt16 lo = static_cast<UInt16>(v.steerMinUs);
        const UInt16 hi = static_cast<UInt16>(v.steerMaxUs);
        link::sendCommand(lk, bibowire::Verb::VERB_SET_SERVO_LIMITS, 0, lo, hi);
        ++v.sent;
    }

    Void sendServoTrim(View& v, link::Client& lk)
    {
        const UInt16 centre = static_cast<UInt16>(v.steerTrimUs);
        link::sendCommand(lk, bibowire::Verb::VERB_SET_SERVO_TRIM, 0, centre, 0);
        ++v.sent;
    }

    Void sendEscLimits(View& v, link::Client& lk)
    {
        const UInt16 lo = static_cast<UInt16>(v.escMinUs);
        const UInt16 hi = static_cast<UInt16>(v.escMaxUs);
        link::sendCommand(lk, bibowire::Verb::VERB_SET_ESC_LIMITS, 0, lo, hi);
        ++v.sent;
    }

    Void sendSlew(View& v, link::Client& lk, UInt8 axis, Int32 usPerTick)
    {
        const UInt16 rate = static_cast<UInt16>(usPerTick);
        link::sendCommand(lk, bibowire::Verb::VERB_SET_SLEW, axis, rate, 0);
        ++v.sent;
    }

    // Neutral is a valid reverse limit, and is how reverse is turned off.
    Void sendEscReverse(View& v, link::Client& lk)
    {
        const UInt16 lowest = static_cast<UInt16>(v.escReverseUs);
        link::sendCommand(lk, bibowire::Verb::VERB_SET_ESC_REVERSE, 0, lowest, 0);
        ++v.sent;
    }

    // THE WHOLE SET, in the order the board needs it: limits before the centre
    // that must sit inside them. Settled first, so what goes out is exactly what
    // the sliders show and what settings.cxx saves.
    Void sendAll(View& v, link::Client& lk, Int64 nowMs)
    {
        settleAll(v);
        vlog::line(
            "trim: save all to the car - servo %d..%d centre %d, esc %d..%d reverse %d, slew %d/%d",
            v.steerMinUs,
            v.steerMaxUs,
            v.steerTrimUs,
            v.escMinUs,
            v.escMaxUs,
            v.escReverseUs,
            v.steerSlewUs,
            v.throttleSlewUs
        );
        sendServoLimits(v, lk);
        sendServoTrim(v, lk);
        sendEscLimits(v, lk);
        sendEscReverse(v, lk);
        sendSlew(v, lk, bibowire::SLEW_AXIS_STEER, v.steerSlewUs);
        sendSlew(v, lk, bibowire::SLEW_AXIS_THROTTLE, v.throttleSlewUs);
        v.lastSendMs = nowMs;
    }

    // WHETHER THE SLIDER JUST SUBMITTED SHOULD SEND NOW - for the sliders the
    // car shows as they move. At most every PREVIEW_SEND_MS while held, and once
    // more on release, so the position the hand stopped on is always the one
    // the board saves. Asked straight after the slider, like IsItemEdited.
    [[nodiscard]] Bool liveSend(View& v, Int64 nowMs)
    {
        if(ImGui::IsItemEdited())
        {
            v.dragUnsent = true;
        }
        const Bool released = ImGui::IsItemDeactivatedAfterEdit();
        const Bool due = v.dragUnsent && ImGui::IsItemActive() && nowMs - v.lastSendMs >= PREVIEW_SEND_MS;
        if(!released && !due)
        {
            return false;
        }
        v.dragUnsent = false;
        v.lastSendMs = nowMs;
        return true;
    }

    // On release only, for the sliders the car cannot show - a slew or the
    // reverse limit changes nothing visible until somebody drives.
    [[nodiscard]] Bool releaseSend(View& v, Int64 nowMs)
    {
        if(!ImGui::IsItemDeactivatedAfterEdit())
        {
            return false;
        }
        v.lastSendMs = nowMs;
        return true;
    }

    // The travel time under a speed slider, as one short line.
    Void slewReadout(Int32 usPerTick, Int32 spanUs, CharSeq what)
    {
        const Int64 centis = crossCentis(spanUs, usPerTick);
        const Str time = centisText(centis);
        ImGui::Indent(12.0f * uiScale);
        ImGui::TextDisabled("%s %s", what, time.c_str());
        ImGui::Unindent(12.0f * uiScale);
    }

    // ONE LINE saying whose numbers the sliders hold.
    Void drawStatus(Bool conn, const link::Snapshot& snap)
    {
        const link::Session& s = snap.state;
        if(!conn)
        {
            ImGui::TextDisabled("not connected");
            return;
        }
        if(!s.haveBoardTrim)
        {
            ImGui::TextDisabled("the car has not reported its trim");
            return;
        }
        if(s.boardTrimText.empty())
        {
            ImGui::TextColored(TEXT_WARN, "nothing saved on the car - press save to car");
            return;
        }
        ImGui::TextColored(TEXT_GOOD, "saved on the car");
    }

    // A REFUSAL MUST BE VISIBLE: a refused limit leaves the slider at a value
    // the car never took, which looks exactly like one it did. Only the trim
    // verbs, and only when the newest answer said no.
    Void drawRefusal(const link::Snapshot& snap)
    {
        const Opt<link::Ack> got = snap.state.newestAck();
        if(!got.has_value() || got->ack.result == 0u || !isTrimVerb(got->ack.verb))
        {
            return;
        }
        const CharSeq why = got->ack.text.empty() ? link::ackResultName(got->ack.result) : got->ack.text.c_str();
        ImGui::PushStyleColor(ImGuiCol_Text, TEXT_BAD);
        ImGui::TextWrapped("refused: %s", why);
        ImGui::PopStyleColor();
    }

  }

  Void init(Float32 scale)
  {
      uiScale = scale > 0.0f ? scale : 1.0f;
  }

  Void follow(View& v, const link::Snapshot& snap)
  {
      const Int64 nowMs = link::monoMs();

      // THE IDLE TEST ENDS ON ITS OWN. Every frame, window open or not, because
      // a closed window is one of the reasons.
      if(v.idleTest)
      {
          CharSeq why = nullptr;
          if(!v.open)
          {
              why = "the Trim window was closed";
          }
          else if(!carArmed(snap, nowMs))
          {
              why = "the car is not armed";
          }
          else if(nowMs - v.idleTestSinceMs >= IDLE_TEST_MS)
          {
              why = "its 20 s ran out";
          }
          if(why != nullptr)
          {
              v.idleTest = false;
              vlog::line("trim: idle test OFF - %s", why);
          }
      }

      const link::Session& s = snap.state;
      if(!s.haveBoardTrim)
      {
          return;
      }
      if(s.boardTrimAtMs == v.adoptedAtMs && s.boardTrimCount == v.adoptedCount)
      {
          return;
      }
      // NOT WHILE THE OPERATOR IS MOVING SOMETHING - see FOLLOW_QUIET_MS. Not
      // marked as taken, so it is taken on the first quiet frame, by which time
      // it describes where the drag ended.
      if(ImGui::IsAnyItemActive() || nowMs - v.lastSendMs < FOLLOW_QUIET_MS)
      {
          return;
      }
      v.adoptedAtMs = s.boardTrimAtMs;
      v.adoptedCount = s.boardTrimCount;

      if(s.boardTrimText.empty())
      {
          vlog::line(
              "trim: the board has NO saved trim - the sliders keep this laptop's copy "
              "(servo %d..%d centre %d, esc %d..%d reverse %d, slew %d/%d)",
              v.steerMinUs,
              v.steerMaxUs,
              v.steerTrimUs,
              v.escMinUs,
              v.escMaxUs,
              v.escReverseUs,
              v.steerSlewUs,
              v.throttleSlewUs
          );
          return;
      }
      const Int32 taken = adoptReport(v, s.boardTrimText);
      vlog::line(
          "trim: took %d setting(s) the board has saved (%s) - the sliders are now "
          "servo %d..%d centre %d, esc %d..%d reverse %d, slew %d/%d",
          taken,
          s.boardTrimText.c_str(),
          v.steerMinUs,
          v.steerMaxUs,
          v.steerTrimUs,
          v.escMinUs,
          v.escMaxUs,
          v.escReverseUs,
          v.steerSlewUs,
          v.throttleSlewUs
      );
  }

  Void drawWindow(View& v, link::Client& lk, const link::Snapshot& snap, Int64 nowMs)
  {
      if(!v.open)
      {
          return;
      }

      ImGui::SetNextWindowPos(ImVec2(820.0f * uiScale, 16.0f * uiScale), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(400.0f * uiScale, 440.0f * uiScale), ImGuiCond_FirstUseEver);

      if(!ImGui::Begin("Trim", &v.open))
      {
          ImGui::End();
          return;
      }

      // WHAT MAY BE TOUCHED. The board refuses every trim verb while the car is
      // armed, so the controls say so by being disabled - with one exception,
      // the ESC limits during the idle test, which the board accepts too.
      const Bool conn = link::isOpen(lk) && snap.state.haveWelcome;
      const Bool armed = conn && carArmed(snap, nowMs);
      const Bool tunable = conn && !armed;
      const Bool escTunable = conn && (!armed || v.idleTest);

      drawStatus(conn, snap);
      if(armed && !v.idleTest)
      {
          ImGui::TextColored(TEXT_WARN, "armed - disarm to change trim");
      }

      ImGui::Separator();

      const Int32 slewLo = static_cast<Int32>(bibowire::SLEW_US_MIN);
      const Int32 slewHi = static_cast<Int32>(bibowire::SLEW_US_MAX);

      // ---- steering ---------------------------------------------------------
      //
      // THESE MOVE THE WHEELS AS THEY ARE DRAGGED, while the car is disarmed:
      // the pilot engages the servo and points it at the value being set - the
      // centre, or the end whose limit moved - and lets it go limp once the
      // sliders are left alone.

      ImGui::TextUnformatted("Steering");
      ImGui::BeginDisabled(!tunable);

      const Int32 servoLo = static_cast<Int32>(bibowire::SERVO_US_HARD_MIN);
      const Int32 servoHi = static_cast<Int32>(bibowire::SERVO_US_HARD_MAX);

      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      ImGui::SliderInt("min us", &v.steerMinUs, servoLo, servoHi, "%d us");
      if(liveSend(v, nowMs))
      {
          settleSteer(v);
          sendServoLimits(v, lk);
      }

      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      ImGui::SliderInt("max us", &v.steerMaxUs, servoLo, servoHi, "%d us");
      if(liveSend(v, nowMs))
      {
          settleSteer(v);
          sendServoLimits(v, lk);
      }

      // Bounded by the ends above: a centre outside them is unreachable.
      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      ImGui::SliderInt("centre us", &v.steerTrimUs, v.steerMinUs, v.steerMaxUs, "%d us");
      if(liveSend(v, nowMs))
      {
          settleSteer(v);
          sendServoTrim(v, lk);
      }

      // LOGARITHMIC: the useful range is the bottom tenth of 1..200.
      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      ImGui::SliderInt("speed##steer", &v.steerSlewUs, slewLo, slewHi, "%d us/tick", ImGuiSliderFlags_Logarithmic);
      if(releaseSend(v, nowMs))
      {
          v.steerSlewUs = clampTo(v.steerSlewUs, slewLo, slewHi);
          sendSlew(v, lk, bibowire::SLEW_AXIS_STEER, v.steerSlewUs);
      }
      slewReadout(v.steerSlewUs, v.steerMaxUs - v.steerMinUs, "lock to lock");

      ImGui::EndDisabled();
      ImGui::Spacing();

      // ---- throttle ---------------------------------------------------------

      ImGui::TextUnformatted("Throttle");

      const Int32 escLo = static_cast<Int32>(bibowire::ESC_US_HARD_MIN);
      const Int32 escHi = static_cast<Int32>(bibowire::ESC_US_HARD_MAX);

      // Live while the idle test runs, so the motor follows the drag.
      ImGui::BeginDisabled(!escTunable);
      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      ImGui::SliderInt("idle us", &v.escMinUs, escLo, escHi, "%d us");
      if(liveSend(v, nowMs))
      {
          settleEsc(v);
          sendEscLimits(v, lk);
      }

      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      ImGui::SliderInt("full us", &v.escMaxUs, escLo, escHi, "%d us");
      if(liveSend(v, nowMs))
      {
          settleEsc(v);
          sendEscLimits(v, lk);
      }
      ImGui::EndDisabled();

      ImGui::BeginDisabled(!tunable);
      const Int32 neutral = static_cast<Int32>(bibowire::ESC_NEUTRAL_US);
      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      ImGui::SliderInt("reverse us", &v.escReverseUs, escLo, neutral, v.escReverseUs >= neutral ? "off" : "%d us");
      if(releaseSend(v, nowMs))
      {
          settleEsc(v);
          sendEscReverse(v, lk);
      }
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip("the pulse S sends - lower is stronger; 1500 is off");
      }

      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      ImGui::SliderInt("speed##throttle", &v.throttleSlewUs, slewLo, slewHi, "%d us/tick", ImGuiSliderFlags_Logarithmic);
      if(releaseSend(v, nowMs))
      {
          v.throttleSlewUs = clampTo(v.throttleSlewUs, slewLo, slewHi);
          sendSlew(v, lk, bibowire::SLEW_AXIS_THROTTLE, v.throttleSlewUs);
      }
      slewReadout(v.throttleSlewUs, v.escMaxUs - v.escMinUs, "idle to full");
      ImGui::EndDisabled();

      // THE IDLE TEST: only while armed, because that is when the pilot drives
      // the ESC. follow() ends it.
      ImGui::BeginDisabled(!armed);
      if(ImGui::Checkbox("test idle", &v.idleTest))
      {
          v.idleTestSinceMs = nowMs;
          vlog::line("trim: idle test %s", v.idleTest ? "ON - the motor is held at the idle pulse" : "OFF - unticked");
      }
      ImGui::EndDisabled();
      if(v.idleTest)
      {
          ImGui::SameLine();
          ImGui::TextColored(TEXT_WARN, "motor at idle - wheels up");
      }
      else if(ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
      {
          ImGui::SetTooltip("arm first; holds the motor at the idle pulse");
      }

      ImGui::Separator();

      // BLUE-GREY, the Drive window's colour for housekeeping.
      ImGui::BeginDisabled(!tunable);
      ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.24f, 0.36f, 0.54f, 1.0f));
      ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.31f, 0.45f, 0.65f, 1.0f));
      ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.19f, 0.29f, 0.44f, 1.0f));
      const Bool savePressed = ImGui::Button("save to car");
      ImGui::PopStyleColor(3);
      ImGui::EndDisabled();
      if(savePressed)
      {
          sendAll(v, lk, nowMs);
      }

      drawRefusal(snap);

      // A COMMAND THAT NEVER LEFT, counted so the drop is visible.
      const UInt32 lost = link::commandsDropped(lk);
      if(lost > 0u)
      {
          Array<Char, 64> bad = {};
          std::snprintf(bad.data(), bad.size(), "%u never sent", lost);
          readout("dropped", bad.data());
      }

      ImGui::End();
  }

}
