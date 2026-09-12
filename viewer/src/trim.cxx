#include "shared.hxx"

#include <cstdio>

// No <windows.h> and no <d3d11.h> here, unlike camera.cxx: this window owns no
// texture and needs no device, so none of the `small`/`near`/`far`/SEVERITY_ERROR
// macro surgery that file performs is needed. Worth saying out loud - the undef
// block is easy to copy into a file that does not need it, and then it looks
// like this module touches Windows when it does not.
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

    // ---- text ---------------------------------------------------------------

    // Centiseconds as "1.07 s", built from two integers.
    //
    // NEVER printf("%.2f"): the decimal point honours the locale and a machine
    // set to a comma decimal writes "1,07". That is the bug this project has met
    // three times, and this readout is the one place in the pane where a decimal
    // point appears at all - so it is the one place worth being careful.
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

    // Whether the car is armed, from the freshest thing that says so.
    //
    // CTLSTATE is the authority and arrives at 20 Hz; BOARD is the fallback at
    // 5 Hz, and its picoArmed carries 2 for UNKNOWN - which is deliberately NOT
    // treated as armed. An unknown state that disabled the controls would leave
    // somebody unable to tune a disarmed car with no way to find out why, and the
    // board's own refusal is the backstop that makes guessing safe here: if it is
    // armed after all, the command is refused with a sentence this pane shows.
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

    // WHY the controls are disabled, or empty when they are not. A disabled
    // slider with no sentence beside it is a program refusing to say what it
    // wants, which is the failure this whole window is careful about.
    [[nodiscard]] Str whyDisabled(link::Client& lk, const link::Snapshot& snap, Int64 nowMs)
    {
        if(!link::isOpen(lk))
        {
            return "not connected - a limit is set on the car, not in this window";
        }
        if(!snap.state.haveWelcome)
        {
            return "handshaking - nothing is sent until the board has answered HELLO";
        }
        if(carArmed(snap, nowMs))
        {
            return "the car is ARMED - the board refuses tuning while armed, and it is "
                   "right to. Disarm first.";
        }
        return "";
    }

    // ---- sending ------------------------------------------------------------
    //
    // ON RELEASE, NEVER ON DRAG. ImGui reports a slider as changed on every pixel
    // of movement, which at 60 Hz is dozens of COMMANDs a second on a link that
    // acknowledges each one individually - and every one of them would be a
    // deliberate act as far as the board's journal is concerned.
    // IsItemDeactivatedAfterEdit() fires once, when the mouse comes up or the
    // typed value is committed, which is the moment the operator actually meant.

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

    // THE WHOLE SET, in the order the board needs it: limits before the centre
    // that must sit inside them. Settled first, so what goes out is exactly what
    // the sliders show and what settings.cxx saves. clampTo, settleSteer and
    // settleEsc live in trim.hxx now - see the note there.
    Void sendAll(View& v, link::Client& lk)
    {
        settleAll(v);
        vlog::line(
            "trim: send all to the car - servo %d..%d centre %d, esc %d..%d, slew %d/%d",
            v.steerMinUs,
            v.steerMaxUs,
            v.steerTrimUs,
            v.escMinUs,
            v.escMaxUs,
            v.steerSlewUs,
            v.throttleSlewUs
        );
        sendServoLimits(v, lk);
        sendServoTrim(v, lk);
        sendEscLimits(v, lk);
        sendSlew(v, lk, bibowire::SLEW_AXIS_STEER, v.steerSlewUs);
        sendSlew(v, lk, bibowire::SLEW_AXIS_THROTTLE, v.throttleSlewUs);
    }

    // ---- the derived line, which is the point of the window -----------------
    //
    // Drawn UNDER its slider and on its own line, not in a tooltip and not as a
    // suffix: "how long does it take to get there" is the question this pane was
    // asked for, and an answer nobody sees without hovering is not an answer.
    Void slewReadout(Int32 usPerTick, Int32 spanUs, CharSeq what)
    {
        const Int32 perSec = slewUsPerSec(usPerTick);
        const Int64 centis = crossCentis(spanUs, usPerTick);

        Array<Char, 96> line = {};
        if(centis < 0)
        {
            // No span to cross - the limits are equal or crossed. Saying so beats
            // printing "0.00 s", which reads as "instantly" rather than as "this
            // number is not currently meaningful".
            std::snprintf(line.data(), line.size(), "%d us/s - no travel to cross", perSec);
        }
        else
        {
            const Str time = centisText(centis);
            std::snprintf(line.data(), line.size(), "%d us/s - %s %s", perSec, what, time.c_str());
        }

        // Indented under the slider it belongs to, so two of these in a column
        // cannot be read as belonging to the wrong control.
        ImGui::Indent(12.0f * uiScale);
        ImGui::TextUnformatted(line.data());
        ImGui::Unindent(12.0f * uiScale);
    }

    // ---- the board's answer -------------------------------------------------
    //
    // A REFUSAL MUST BE VISIBLE. Silence after a refused command is the failure
    // this repo keeps finding, and it is worse here than usual: a refused limit
    // leaves the slider sitting at a value the car never took, which looks
    // exactly like a limit that was applied.
    Void drawAck(const link::Snapshot& snap)
    {
        const Opt<link::Ack> got = snap.state.newestAck();
        if(!got.has_value())
        {
            ImGui::TextDisabled("the board has not answered a command yet");
            return;
        }

        const bibowire::CmdAck& ack = got->ack;
        const Bool refused = ack.result != 0u;

        Array<Char, 96> head = {};
        std::snprintf(
            head.data(),
            head.size(),
            "command %u - %s",
            ack.cmdId,
            link::ackResultName(ack.result)
        );

        if(refused)
        {
            // Coloured, because this is the one line in the window that means
            // "what you just did did not happen".
            ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.35f, 1.0f), "%s", head.data());
        }
        else
        {
            ImGui::TextUnformatted(head.data());
        }

        // VERBATIM, and shown for an ok as well as a refusal - the board may have
        // clamped what it was sent, and the sentence is where it would say so.
        if(!ack.text.empty())
        {
            ImGui::TextWrapped("board: %s", ack.text.c_str());
        }
        else if(refused)
        {
            ImGui::TextWrapped("board: (refused with no sentence)");
        }
    }

  }

  Void init(Float32 scale)
  {
      uiScale = scale > 0.0f ? scale : 1.0f;
  }

  Void drawWindow(View& v, link::Client& lk, const link::Snapshot& snap, Int64 nowMs)
  {
      if(!v.open)
      {
          return;
      }

      ImGui::SetNextWindowPos(ImVec2(820.0f * uiScale, 16.0f * uiScale), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(430.0f * uiScale, 560.0f * uiScale), ImGuiCond_FirstUseEver);

      if(!ImGui::Begin("Trim", &v.open))
      {
          ImGui::End();
          return;
      }

      // SAID FIRST, BEFORE ANY CONTROL: where these values are kept. It used to
      // say they were lost on the Pico's next reboot, which was true until the
      // board started saving them - and a warning that has stopped being true
      // teaches an operator to ignore the line it sits on.
      ImGui::TextWrapped(
          "The board saves every change it accepts on the Pi and re-sends it to "
          "the Pico whenever the Pico connects. This laptop saves these sliders "
          "too - \"send all to the car\" pushes them when the two disagree."
      );

      ImGui::Separator();

      // What the sliders are, and are not. There is no "read my limits" message
      // in bibowire, so this pane cannot show the car's current values and must
      // not look as though it does.
      ImGui::TextDisabled("what this window asks for - the car never reports its own limits back");

      const Str why = whyDisabled(lk, snap, nowMs);
      const Bool blocked = !why.empty();
      if(blocked)
      {
          ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.30f, 1.0f), "%s", why.c_str());
      }

      ImGui::Separator();

      ImGui::BeginDisabled(blocked);

      // ---- steering ---------------------------------------------------------

      ImGui::TextUnformatted("Steering");

      const Int32 servoLo = static_cast<Int32>(bibowire::SERVO_US_HARD_MIN);
      const Int32 servoHi = static_cast<Int32>(bibowire::SERVO_US_HARD_MAX);

      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      ImGui::SliderInt("min us", &v.steerMinUs, servoLo, servoHi, "%d us");
      if(ImGui::IsItemDeactivatedAfterEdit())
      {
          settleSteer(v);
          sendServoLimits(v, lk);
      }
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip("full lock one way - cal.hxx has 1230 for this car");
      }

      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      ImGui::SliderInt("max us", &v.steerMaxUs, servoLo, servoHi, "%d us");
      if(ImGui::IsItemDeactivatedAfterEdit())
      {
          settleSteer(v);
          sendServoLimits(v, lk);
      }
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip("full lock the other way - cal.hxx has 1660 for this car");
      }

      // BOUNDED BY THE ENDS ABOVE, not by the servo's own range: a centre outside
      // the limits is a neutral the steering can never reach.
      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      ImGui::SliderInt("centre us", &v.steerTrimUs, v.steerMinUs, v.steerMaxUs, "%d us");
      if(ImGui::IsItemDeactivatedAfterEdit())
      {
          settleSteer(v);
          sendServoTrim(v, lk);
      }
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "where the wheels point STRAIGHT, which is not the middle\n"
              "of the range and usually not 1500 - the horn only fits the\n"
              "spline at whole-tooth intervals. cal.hxx has 1480."
          );
      }

      ImGui::Spacing();

      // ---- throttle ---------------------------------------------------------

      ImGui::TextUnformatted("Throttle");

      const Int32 escLo = static_cast<Int32>(bibowire::ESC_US_HARD_MIN);
      const Int32 escHi = static_cast<Int32>(bibowire::ESC_US_HARD_MAX);

      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      ImGui::SliderInt("idle us", &v.escMinUs, escLo, escHi, "%d us");
      if(ImGui::IsItemDeactivatedAfterEdit())
      {
          settleEsc(v);
          sendEscLimits(v, lk);
      }
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "the pulse at which this motor sits still and the next\n"
              "microsecond starts it turning. NOT the ESC's neutral."
          );
      }

      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      ImGui::SliderInt("full us", &v.escMaxUs, escLo, escHi, "%d us");
      if(ImGui::IsItemDeactivatedAfterEdit())
      {
          settleEsc(v);
          sendEscLimits(v, lk);
      }
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip("forward only - the board refuses anything below 1500");
      }

      // THE NUMBERS ABOVE WERE MEASURED ON A MOTOR THAT NO LONGER EXISTS, and the
      // pane says so rather than presenting them as this drivetrain's truth.
      ImGui::TextWrapped(
          "1541/1600 were measured on the brushed 1060 and 540, both GONE - this car "
          "is a QuicRun 10BL160 with a 21.5T brushless and a 17T pinion. Treat them as "
          "a narrow safe starting point, not as measurements. Re-measure on a stand."
      );

      ImGui::Spacing();
      ImGui::Separator();

      // ---- response, and the time it takes ----------------------------------
      //
      // The reason this window exists in the shape it does: a rate in µs-per-tick
      // is unreadable, and what somebody tuning steering actually wants to know is
      // how long the wheel takes to go from one lock to the other.

      ImGui::TextUnformatted("Response");

      const Int32 slewLo = static_cast<Int32>(bibowire::SLEW_US_MIN);
      const Int32 slewHi = static_cast<Int32>(bibowire::SLEW_US_MAX);

      // LOGARITHMIC, as the old hub's Drive view had it: the useful range is the
      // bottom tenth, and a linear 1..200 spends most of its travel on rates that
      // are all indistinguishably immediate.
      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      ImGui::SliderInt(
          "steering",
          &v.steerSlewUs,
          slewLo,
          slewHi,
          "%d us/tick",
          ImGuiSliderFlags_Logarithmic
      );
      if(ImGui::IsItemDeactivatedAfterEdit())
      {
          v.steerSlewUs = clampTo(v.steerSlewUs, slewLo, slewHi);
          sendSlew(v, lk, bibowire::SLEW_AXIS_STEER, v.steerSlewUs);
      }
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip("Response - how fast the servo may move");
      }
      slewReadout(v.steerSlewUs, v.steerMaxUs - v.steerMinUs, "lock to lock");

      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      ImGui::SliderInt(
          "throttle",
          &v.throttleSlewUs,
          slewLo,
          slewHi,
          "%d us/tick",
          ImGuiSliderFlags_Logarithmic
      );
      if(ImGui::IsItemDeactivatedAfterEdit())
      {
          v.throttleSlewUs = clampTo(v.throttleSlewUs, slewLo, slewHi);
          sendSlew(v, lk, bibowire::SLEW_AXIS_THROTTLE, v.throttleSlewUs);
      }
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip("Response - how fast the ESC may move");
      }
      slewReadout(v.throttleSlewUs, v.escMaxUs - v.escMinUs, "idle to full");

      ImGui::Spacing();

      // INSIDE THE DISABLED BLOCK, like every slider above. Five tuning verbs at
      // once are five chances to re-tune a live throttle's range, so this obeys
      // the armed rule exactly as each of them does one at a time.
      // BLUE-GREY, the Drive window's colour for housekeeping, so a glance
      // tells it apart from the sliders' grey and from any act that moves the car.
      ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.24f, 0.36f, 0.54f, 1.0f));
      ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.31f, 0.45f, 0.65f, 1.0f));
      ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.19f, 0.29f, 0.44f, 1.0f));
      const Bool sendPressed = ImGui::Button("send all to the car");
      ImGui::PopStyleColor(3);
      if(sendPressed)
      {
          sendAll(v, lk);
      }
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "sends every value above: servo limits, centre, ESC limits,\n"
              "and both slews - five commands, each answered on its own.\n"
              "For when this laptop's copy and the car's have drifted,\n"
              "such as after a change made from another viewer."
          );
      }

      ImGui::EndDisabled();

      ImGui::Separator();

      // ---- what the board said ----------------------------------------------

      drawAck(snap);

      Array<Char, 48> count = {};
      std::snprintf(count.data(), count.size(), "%u", v.sent);
      readout("sent", count.data());

      // A COMMAND THAT NEVER LEFT. Queued while the link was down and therefore
      // dropped rather than held - see link.hxx for why that is the right way
      // round - and counted here so the drop is visible instead of silent.
      const UInt32 lost = link::commandsDropped(lk);
      if(lost > 0u)
      {
          Array<Char, 96> bad = {};
          std::snprintf(bad.data(), bad.size(), "%u never sent - the link was down", lost);
          readout("dropped", bad.data());
      }

      ImGui::End();
  }

}
