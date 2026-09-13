#include "shared.hxx"

#include <cmath>
#include <cstdio>

#include "imgui.h"

#include "drive.hxx"
#include "vlog.hxx"

namespace driveview
{
  namespace
  {
    Float32 uiScale = 1.0f;

    constexpr Float32 READOUT_COLUMN = 124.0f;
    constexpr Float32 ITEM_WIDTH = -148.0f;

    // Indexed by bibowire::PilotMode's wire value, so there is no mapping.
    constexpr Array<CharSeq, 3> MODE_NAMES = { "manual", "look", "drive" };

    [[nodiscard]] Str msText(Int64 ms)
    {
        Array<Char, 32> t = {};
        std::snprintf(t.data(), t.size(), "%lld ms", ms);
        return Str(t.data());
    }

    [[nodiscard]] Str milliText(Int32 v)
    {
        Array<Char, 32> t = {};
        std::snprintf(t.data(), t.size(), "%+d / 1000", v);
        return Str(t.data());
    }

    Void readout(CharSeq label, CharSeq value)
    {
        ImGui::TextUnformatted(label);
        ImGui::SameLine(READOUT_COLUMN * uiScale);
        ImGui::TextUnformatted(value);
    }

    Void readoutStr(CharSeq label, const Str& value)
    {
        readout(label, value.c_str());
    }

    // CTLSTATE's holder byte: 0 nobody, 1 you, 2 another viewer. With nobody
    // holding it the deadman does not apply and the pilot runs under its own
    // rules.
    [[nodiscard]] CharSeq holderText(UInt8 v)
    {
        if(v == 0u)
        {
            return "nobody";
        }
        if(v == 1u)
        {
            return "you";
        }
        if(v == 2u)
        {
            return "another viewer";
        }
        return "--";
    }

    [[nodiscard]] CharSeq deadmanText(UInt8 v)
    {
        if(v > static_cast<UInt8>(bibowire::deadman::State::STATE_ESTOP))
        {
            return "--";
        }
        return bibowire::deadman::stateName(static_cast<bibowire::deadman::State>(v));
    }

    // WELCOME's capabilities bit 0 (docs/bibowire.md section 4).
    [[nodiscard]] Bool canDrive(const link::Snapshot& snap)
    {
        return snap.state.haveWelcome && (snap.state.welcome.capabilities & 0x01u) != 0u;
    }

    // Why the keys are dead, or empty when they are not, so a disabled control
    // always has its reason beside it.
    [[nodiscard]] Str whyBlocked(link::Client& lk, const link::Snapshot& snap)
    {
        if(!link::isOpen(lk))
        {
            return "not connected";
        }
        if(!snap.state.haveWelcome)
        {
            return "connecting";
        }
        if(!canDrive(snap))
        {
            return "this board cannot drive (pilot running --dry)";
        }
        if(!link::holdsSlot(snap.state))
        {
            return "watching only - tick request control and Reconnect";
        }
        return "";
    }

    // Keys count only while this window has focus and nothing is being typed
    // (the caller's `accept`): `bibobox` typed into the host field must never
    // become a throttle command. WantTextInput covers a text field,
    // IsAnyItemActive a slider mid-drag, and focus the rest of the program.
    // ImGui clears key state when the app loses focus, and not accepting returns
    // all keys up, which is neutral.
    [[nodiscard]] Keys readKeys(Bool accept)
    {
        Keys k;
        if(!accept)
        {
            return k;
        }
        k.left = ImGui::IsKeyDown(ImGuiKey_A);
        k.right = ImGui::IsKeyDown(ImGuiKey_D);
        k.forward = ImGui::IsKeyDown(ImGuiKey_W);
        k.brake = ImGui::IsKeyDown(ImGuiKey_S);
        k.estop = ImGui::IsKeyDown(ImGuiKey_Space);
        k.centre = ImGui::IsKeyDown(ImGuiKey_C);
        return k;
    }

    // The round-trip log gets EDGES ONLY. drawWindow publishes a level every
    // frame; the log needs each key press, enable change and block once,
    // stamped, to lay beside the worker's CONTROL lines and the board's journal.
    struct Journal
    {
        Bool enabled = false;
        Bool accepting = false;
        Keys keys;
        Bool haveWhy = false;
        Str why;
    };

    Journal journal;

    Void noteEnabled(Bool now, CharSeq because)
    {
        if(now == journal.enabled)
        {
            return;
        }
        journal.enabled = now;
        vlog::line("drive: enable %s - %s", now ? "ON" : "OFF", because);
    }

    Void noteKey(CharSeq name, Bool was, Bool now)
    {
        if(was != now)
        {
            vlog::line("drive: key %s %s", name, now ? "DOWN" : "up");
        }
    }

    // Key edges only while keys are accepted. When acceptance ends every key
    // reads as up at once, and one line says so rather than five "up" lines for
    // fingers that may still be resting there.
    Void noteKeys(const Keys& k, Bool accept)
    {
        if(accept != journal.accepting)
        {
            journal.accepting = accept;
            if(accept)
            {
                vlog::line("drive: keys ACCEPTED - focused, nothing being typed");
            }
            else
            {
                vlog::line("drive: keys IGNORED - unfocused, typing or blocked; all read as up");
            }
        }
        if(accept)
        {
            noteKey("W", journal.keys.forward, k.forward);
            noteKey("A", journal.keys.left, k.left);
            noteKey("S", journal.keys.brake, k.brake);
            noteKey("D", journal.keys.right, k.right);
            noteKey("Space", journal.keys.estop, k.estop);
            noteKey("C", journal.keys.centre, k.centre);
        }
        journal.keys = k;
    }

    Void noteWhy(const Str& why)
    {
        if(journal.haveWhy && why == journal.why)
        {
            return;
        }
        journal.haveWhy = true;
        journal.why = why;
        if(why.empty())
        {
            vlog::line("drive: unblocked - the enable and the keys can be used");
            return;
        }
        vlog::line("drive: BLOCKED - %s", why.c_str());
    }

    // Back to centre, logged once with the reason, and only when off centre: a
    // closed window calls this every frame.
    Void resetSteer(View& v, CharSeq why)
    {
        if(v.steerHeldMilli == 0)
        {
            return;
        }
        vlog::line(
            "drive: steering centred (reset: %s) - was %d / 1000",
            why,
            static_cast<Int32>(v.steerHeldMilli)
        );
        v.steerHeldMilli = 0;
    }

    // This frame's length in whole milliseconds; steerHeldStep clamps it.
    [[nodiscard]] Int32 frameMs(const ImGuiIO& io)
    {
        return static_cast<Int32>(std::lround(io.DeltaTime * 1000.0f));
    }

    // One frame of the held steering. drawWindow has already reset it for a
    // closed, collapsed or blocked pane; this handles the enable and the stop.
    Void steerFrame(View& v, const Keys& k, Bool centrePressed, Int32 dtMs)
    {
        if(!v.enabled)
        {
            resetSteer(v, "enable is off");
            return;
        }
        if(k.estop)
        {
            resetSteer(v, "ESTOP (Space)");
            return;
        }
        if(centrePressed && v.steerHeldMilli != 0)
        {
            vlog::line(
                "drive: steering centred (C) - was %d / 1000",
                static_cast<Int32>(v.steerHeldMilli)
            );
        }
        v.steerHeldMilli = steerHeldStep(v.steerHeldMilli, k, v.steerRateMilliPerS, dtMs);
    }

    // The held steering as a bar filling from the middle; a ProgressBar filling
    // from the left would read half full as "halfway". Left of the tick is left
    // on the car (test_link.cxx, testSteerSigns).
    Void steerBar(Int16 heldMilli)
    {
        const Float32 width = ImGui::GetContentRegionAvail().x;
        const Float32 height = ImGui::GetTextLineHeight();
        const ImVec2 at = ImGui::GetCursorScreenPos();
        ImGui::Dummy(ImVec2(width > 1.0f ? width : 1.0f, height));
        if(width <= 1.0f)
        {
            return;
        }
        const ImVec2 end(at.x + width, at.y + height);
        const Float32 mid = at.x + (width * 0.5f);
        const Float32 frac = static_cast<Float32>(heldMilli) / static_cast<Float32>(STEER_FULL);
        const Float32 tip = mid + (frac * width * 0.5f);
        ImDrawList* draw = ImGui::GetWindowDrawList();
        draw->AddRectFilled(at, end, ImGui::GetColorU32(ImGuiCol_FrameBg));
        draw->AddRectFilled(
            ImVec2(tip < mid ? tip : mid, at.y),
            ImVec2(tip < mid ? mid : tip, end.y),
            ImGui::GetColorU32(ImGuiCol_PlotHistogram)
        );
        draw->AddLine(ImVec2(mid, at.y), ImVec2(mid, end.y), ImGui::GetColorU32(ImGuiCol_Text));
    }

    // A refusal must be VISIBLE, in the board's own words: a refused ARM not
    // shown is an operator who believes the car is armed. Accepted acts show in
    // the badge, and refused trim verbs are the Trim pane's to show.
    Void drawAck(const link::Snapshot& snap)
    {
        const Opt<link::Ack> got = snap.state.newestAck();
        if(!got.has_value() || got->ack.result == 0u)
        {
            return;
        }
        const bibowire::CmdAck& ack = got->ack;
        const Bool trimVerb = ack.verb == bibowire::Verb::VERB_SET_SERVO_LIMITS
            || ack.verb == bibowire::Verb::VERB_SET_SERVO_TRIM
            || ack.verb == bibowire::Verb::VERB_SET_ESC_LIMITS
            || ack.verb == bibowire::Verb::VERB_SET_ESC_REVERSE
            || ack.verb == bibowire::Verb::VERB_SET_SLEW;
        if(trimVerb)
        {
            return;
        }
        const CharSeq why = ack.text.empty() ? link::ackResultName(ack.result) : ack.text.c_str();
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.45f, 0.35f, 1.0f));
        ImGui::TextWrapped("refused: %s", why);
        ImGui::PopStyleColor();
    }

    // ONE MEANING PER COLOUR, for a button and the state it produces, so
    // opposite acts never look alike:
    //
    //   AMBER  makes the car LIVE      - ARM, and the ARMED state
    //   GREEN  makes the car SAFE      - DISARM, and DISARMED / a live deadman
    //   RED    STOPS it                - ESTOP, and a latched estop or a dead deadman
    //   BLUE   housekeeping            - CLEAR ESTOP
    //
    // Amber for ARM, not green: green says "go", and ARM is what lets the car move.
    struct Tone
    {
        ImVec4 base;
        ImVec4 hover;
        ImVec4 active;
        ImVec4 text;
    };

    const Tone TONE_LIVE = {
        ImVec4(0.80f, 0.52f, 0.08f, 1.0f),
        ImVec4(0.92f, 0.62f, 0.12f, 1.0f),
        ImVec4(0.66f, 0.42f, 0.06f, 1.0f),
        ImVec4(0.08f, 0.05f, 0.01f, 1.0f),
    };
    const Tone TONE_SAFE = {
        ImVec4(0.14f, 0.52f, 0.26f, 1.0f),
        ImVec4(0.18f, 0.64f, 0.32f, 1.0f),
        ImVec4(0.10f, 0.42f, 0.20f, 1.0f),
        ImVec4(1.00f, 1.00f, 1.00f, 1.0f),
    };
    const Tone TONE_STOP = {
        ImVec4(0.74f, 0.11f, 0.11f, 1.0f),
        ImVec4(0.88f, 0.17f, 0.17f, 1.0f),
        ImVec4(0.58f, 0.07f, 0.07f, 1.0f),
        ImVec4(1.00f, 1.00f, 1.00f, 1.0f),
    };
    const Tone TONE_CHORE = {
        ImVec4(0.24f, 0.36f, 0.54f, 1.0f),
        ImVec4(0.31f, 0.45f, 0.65f, 1.0f),
        ImVec4(0.19f, 0.29f, 0.44f, 1.0f),
        ImVec4(1.00f, 1.00f, 1.00f, 1.0f),
    };

    // The same meanings as text, lighter to read on the window background.
    const ImVec4 TEXT_LIVE = ImVec4(1.00f, 0.72f, 0.24f, 1.0f);
    const ImVec4 TEXT_SAFE = ImVec4(0.42f, 0.86f, 0.52f, 1.0f);
    const ImVec4 TEXT_STOP = ImVec4(1.00f, 0.40f, 0.36f, 1.0f);

    [[nodiscard]] Bool toneButton(CharSeq label, const Tone& t, const ImVec2& size)
    {
        ImGui::PushStyleColor(ImGuiCol_Button, t.base);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, t.hover);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, t.active);
        ImGui::PushStyleColor(ImGuiCol_Text, t.text);
        const Bool pressed = ImGui::Button(label, size);
        ImGui::PopStyleColor(4);
        return pressed;
    }

    // Discrete acts are TCP COMMANDs, each answered by one CMDACK, never CONTROL
    // fields: a deliberate act repeated twenty times a second is an act nobody
    // can point at.
    //
    // Laid out so a slip cannot land on the opposite act: ARM and DISARM share a
    // row, half width each with a gap; ESTOP has its own full-width, taller row,
    // never beside ARM; CLEAR ESTOP sits small below it.
    Void drawCommands(View& v, link::Client& lk, const link::Snapshot& snap)
    {
        const Bool live = link::isOpen(lk) && snap.state.haveWelcome;
        const ImGuiStyle& style = ImGui::GetStyle();
        const Float32 gap = 16.0f * uiScale;
        const Float32 rowWidth = ImGui::GetContentRegionAvail().x;
        const Float32 half = (rowWidth - gap) * 0.5f;
        const Float32 tall = ImGui::GetFrameHeight() * 1.6f;
        ImGui::BeginDisabled(!live);
        if(toneButton("ARM", TONE_LIVE, ImVec2(half, tall)))
        {
            vlog::line("drive: ARM pressed");
            link::sendCommand(lk, bibowire::Verb::VERB_ARM, 0, 0, 0);
            ++v.sent;
        }
        ImGui::SameLine(0.0f, gap);
        if(toneButton("DISARM", TONE_SAFE, ImVec2(half, tall)))
        {
            vlog::line("drive: DISARM pressed");
            link::sendCommand(lk, bibowire::Verb::VERB_DISARM, 0, 0, 0);
            ++v.sent;
        }
        ImGui::EndDisabled();
        ImGui::Dummy(ImVec2(0.0f, style.ItemSpacing.y));
        // NEVER DISABLED: the stop works in every state. Two paths on purpose:
        // this one rides TCP and is acknowledged, Space's bit rides the 20 Hz
        // stream with no round trip. Either latches.
        if(toneButton("ESTOP  (Space)", TONE_STOP, ImVec2(rowWidth, tall * 1.25f)))
        {
            vlog::line("drive: ESTOP pressed");
            resetSteer(v, "ESTOP button");
            link::sendCommand(lk, bibowire::Verb::VERB_ESTOP, 0, 0, 0);
            ++v.sent;
        }
        // The latch is undone only by CLEAR ESTOP while disarmed, then ARM, so a
        // released key can never undo a stop.
        ImGui::BeginDisabled(!live);
        if(toneButton("CLEAR ESTOP", TONE_CHORE, ImVec2(0.0f, 0.0f)))
        {
            vlog::line("drive: CLEAR ESTOP pressed");
            link::sendCommand(lk, bibowire::Verb::VERB_CLEAR_ESTOP, 0, 0, 0);
            ++v.sent;
        }
        if(ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("only while disarmed; does not re-arm");
        }
        ImGui::EndDisabled();
    }

    // The state in one word and its colour. The ESTOP latch outranks armed.
    // With no CTLSTATE it says so, because a blank would read as disarmed.
    Void drawBadge(const link::Snapshot& snap, Int64 nowMs)
    {
        const Opt<link::Control> ctl = snap.state.controlState(nowMs);
        if(!ctl.has_value())
        {
            ImGui::TextColored(
                ImVec4(1.0f, 0.72f, 0.30f, 1.0f),
                "%s",
                snap.state.haveControl ? "car state stopped arriving" : "no car state yet"
            );
            return;
        }
        const bibowire::CtlState& s = ctl->state;
        if(s.deadman == static_cast<UInt8>(bibowire::deadman::State::STATE_ESTOP))
        {
            ImGui::TextColored(TEXT_STOP, "ESTOP - clear it, then ARM");
        }
        else if(s.armed != 0u)
        {
            ImGui::TextColored(TEXT_LIVE, "ARMED");
        }
        else
        {
            ImGui::TextColored(TEXT_SAFE, "DISARMED");
        }
    }

    // All from CTLSTATE. The countdowns come from the board's own
    // bibowire::deadman::step, and this file deliberately never calls it, so no
    // second arithmetic can disagree with the one that stops the car.
    Void drawBoardSide(const link::Snapshot& snap, Int64 nowMs)
    {
        const Opt<link::Control> ctl = snap.state.controlState(nowMs);
        // Missing CTLSTATE is said once, by drawBadge; "--" rows would read as zero.
        if(!ctl.has_value())
        {
            return;
        }
        const bibowire::CtlState& s = ctl->state;
        readout("holder", holderText(s.holder));
        ImGui::TextUnformatted("armed");
        ImGui::SameLine(READOUT_COLUMN * uiScale);
        ImGui::TextColored(s.armed != 0u ? TEXT_LIVE : TEXT_SAFE, "%s", s.armed != 0u ? "yes" : "no");
        // live green, soft amber, dead and estop red.
        const ImVec4 deadmanTone = s.deadman == 0u ? TEXT_SAFE : (s.deadman == 1u ? TEXT_LIVE : TEXT_STOP);
        ImGui::TextUnformatted("deadman");
        ImGui::SameLine(READOUT_COLUMN * uiScale);
        ImGui::TextColored(deadmanTone, "%s", deadmanText(s.deadman));
        // Why throttle is not applied, named: otherwise "throttle dead, steering
        // fine" looks like an ESC or Pico fault.
        const Bool refusing = s.refuse != bibowire::Refuse::REFUSE_NONE;
        ImGui::TextUnformatted("refusing");
        ImGui::SameLine(READOUT_COLUMN * uiScale);
        if(refusing)
        {
            ImGui::TextColored(
                ImVec4(1.0f, 0.45f, 0.35f, 1.0f),
                "%s",
                bibowire::refuseName(s.refuse)
            );
        }
        else
        {
            ImGui::TextUnformatted(bibowire::refuseName(s.refuse));
        }
        // 0 means already fired: the board sends 0 from the instant it trips on.
        readoutStr("neutral in", msText(static_cast<Int64>(s.neutralInMs)));
        readoutStr("disarm in", msText(static_cast<Int64>(s.disarmInMs)));
        if(s.controlAgeMs == bibowire::CONTROL_AGE_NEVER)
        {
            // Never, not "long ago": the board has applied nothing from this viewer.
            readout("control age", "never");
        }
        else
        {
            readoutStr("control age", msText(static_cast<Int64>(s.controlAgeMs)));
        }
        // Sent against applied catches a stream the board stopped taking while
        // every socket still looks healthy.
        Array<Char, 80> seqs = {};
        std::snprintf(seqs.data(), seqs.size(), "sent %u, applied %u", snap.controlSeq, s.ackSeq);
        readout("seq", seqs.data());
        readoutStr("steer now", milliText(s.steerNowMilli));
        readoutStr("throttle now", milliText(s.throttleMilli));
        if(s.escUs == bibowire::ESC_ABSENT)
        {
            readout("esc", "unknown");
        }
        else
        {
            Array<Char, 32> esc = {};
            std::snprintf(esc.data(), esc.size(), "%u us", static_cast<UInt32>(s.escUs));
            readout("esc", esc.data());
        }
        Array<Char, 48> epoch = {};
        std::snprintf(epoch.data(), epoch.size(), "%u", static_cast<UInt32>(s.armEpoch));
        readout("arm epoch", epoch.data());
        if(ctl->stale)
        {
            ImGui::TextColored(
                ImVec4(1.0f, 0.72f, 0.30f, 1.0f),
                "the numbers above are %lld ms old",
                ctl->ageMs
            );
        }
    }
  }

  Void init(Float32 scale)
  {
      uiScale = scale > 0.0f ? scale : 1.0f;
  }

  Void drawWindow(View& v, link::Client& lk, const link::Snapshot& snap, Int64 nowMs)
  {
      // Closed publishes a neutral intent with consent withdrawn rather than
      // going silent (View::open).
      if(!v.open)
      {
          v.enabled = false;
          resetSteer(v, "the Drive window was closed");
          noteEnabled(false, "the Drive window was closed");
          noteKeys(Keys(), false);
          link::setControl(lk, link::Intent());
          return;
      }
      ImGui::SetNextWindowPos(ImVec2(380.0f * uiScale, 440.0f * uiScale), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(430.0f * uiScale, 520.0f * uiScale), ImGuiCond_FirstUseEver);
      if(!ImGui::Begin("Drive", &v.open))
      {
          // Collapsed: nobody can see the state, so it stops as if closed.
          v.enabled = false;
          resetSteer(v, "the Drive window was collapsed");
          noteEnabled(false, "the Drive window was collapsed");
          noteKeys(Keys(), false);
          link::setControl(lk, link::Intent());
          ImGui::End();
          return;
      }
      const Str why = whyBlocked(lk, snap);
      const Bool blocked = !why.empty();
      noteWhy(why);
      if(blocked)
      {
          // FORCED OFF, not just greyed: consent left set through a disconnect
          // would ride the first datagram of the next session.
          v.enabled = false;
          v.boardArmed = false;
          resetSteer(v, "the pane is blocked");
          noteEnabled(false, "the pane is blocked");
      }
      // The BOARD's word ticks and clears the enable, not the button press, so a
      // refused ARM ticks nothing. CTLSTATE first, BOARD's picoArmed otherwise
      // (2 is UNKNOWN). A frame with neither is not a disarm: a dropped datagram
      // must not cut the throttle. Edges only (View::boardArmed).
      if(!blocked)
      {
          Opt<Bool> armedNow;
          const Opt<link::Control> ctlNow = snap.state.controlState(nowMs);
          const Opt<link::Board> boardNow = snap.state.boardState(nowMs);
          if(ctlNow.has_value())
          {
              armedNow = ctlNow->state.armed != 0u;
          }
          else if(boardNow.has_value() && boardNow->state.picoArmed != 2u)
          {
              armedNow = boardNow->state.picoArmed == 1u;
          }
          if(armedNow.has_value() && *armedNow != v.boardArmed)
          {
              v.boardArmed = *armedNow;
              v.enabled = *armedNow;
              if(!*armedNow)
              {
                  resetSteer(v, "the board is no longer armed");
              }
              noteEnabled(
                  *armedNow,
                  *armedNow ? "the board confirmed ARM" : "the board is no longer armed"
              );
          }
      }
      // Read before anything is drawn, so IsAnyItemActive describes the widget
      // actually held, not one this frame has not submitted yet.
      const ImGuiIO& io = ImGui::GetIO();
      const Bool focused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
      const Bool typing = io.WantTextInput || ImGui::IsAnyItemActive();
      const Bool accept = focused && !typing && !blocked;
      const Keys keys = readKeys(accept);
      // C's edge, taken before noteKeys overwrites last frame's keys: holding C
      // centres once, not every frame.
      const Bool centrePressed = keys.centre && !journal.keys.centre;
      noteKeys(keys, accept);
      // Steering moves before the intent is built, so this frame's keys are in it.
      steerFrame(v, keys, centrePressed, frameMs(io));
      // EVERY FRAME: a level, sampled by the worker on the board's 50 ms period.
      // Publishing only on change would make "no key pressed" and "the UI thread
      // hung" identical on the wire.
      const link::Intent in = intentFrom(keys, v);
      link::setControl(lk, in);
      // HELLO alone carries the slot request, so a changed box applies to the
      // NEXT connection - hence Reconnect beside it.
      Bool ask = link::controlSlotWanted(lk);
      if(ImGui::Checkbox("request control", &ask))
      {
          link::wantControlSlot(lk, ask);
          vlog::line(
              "drive: request control on connect %s - applies to the NEXT HELLO",
              ask ? "ON" : "OFF"
          );
      }
      ImGui::SameLine();
      ImGui::BeginDisabled(!link::isOpen(lk));
      if(ImGui::Button("Reconnect"))
      {
          // A new session and epoch; the car stays stopped until armed again.
          vlog::line(
              "drive: Reconnect pressed - request control on connect is %s",
              ask ? "ON" : "OFF"
          );
          static_cast<Void>(link::reconnect(lk));
      }
      ImGui::EndDisabled();
      if(link::isOpen(lk) && ask != link::holdsSlot(snap.state))
      {
          ImGui::TextDisabled("Reconnect to apply");
      }
      if(blocked)
      {
          ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.30f, 1.0f), "%s", why.c_str());
      }
      ImGui::Separator();
      ImGui::BeginDisabled(blocked);
      if(ImGui::Checkbox("enable", &v.enabled))
      {
          noteEnabled(v.enabled, "the enable checkbox");
      }
      ImGui::TextDisabled(
          accept ? "A/D steer  W forward  S brake, again to reverse  C centre  Space ESTOP"
                 : "click this window to drive"
      );
      // S with reverse off only stops, which looks like a broken reverse, so it
      // is said while S is held.
      if(keys.brake && v.reverseOff)
      {
          ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.30f, 1.0f), "reverse is off - set reverse us in Trim");
      }
      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      ImGui::SliderInt("power", &v.throttleCapMilli, 0, THROTTLE_CAP_MAX, "%d / 1000");
      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      ImGui::SliderInt(
          "steer rate",
          &v.steerRateMilliPerS,
          STEER_RATE_MIN,
          STEER_RATE_MAX,
          "%d /s"
      );
      // Ctrl+click turns a slider into a text box, so the ranges are re-applied
      // rather than trusted.
      settle(v);
      Array<Char, 64> lock = {};
      const Int32 lockMs = steerLockMs(v.steerRateMilliPerS);
      std::snprintf(lock.data(), lock.size(), "centre to full lock in %d ms", lockMs);
      ImGui::Indent(12.0f * uiScale);
      ImGui::TextUnformatted(lock.data());
      ImGui::Unindent(12.0f * uiScale);
      // The held value is what goes on the wire while enabled; "steer now" in
      // the details is where the board says the servo actually is.
      readoutStr("steering", milliText(v.steerHeldMilli));
      steerBar(v.steerHeldMilli);
      // Measured before the combo, while the cursor is at the start of the row.
      const Float32 rowWidth = ImGui::GetContentRegionAvail().x;
      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      const Int32 modeCount = static_cast<Int32>(MODE_NAMES.size());
      ImGui::Combo("mode", &v.assumedMode, MODE_NAMES.data(), modeCount);
      // Beside the combo only when it fits. The combo's item rect includes its
      // label, so this is the whole row as drawn.
      const ImGuiStyle& style = ImGui::GetStyle();
      const Float32 askWidth = ImGui::CalcTextSize("ask the board").x + (style.FramePadding.x * 2.0f);
      if(ImGui::GetItemRectSize().x + style.ItemSpacing.x + askWidth <= rowWidth)
      {
          ImGui::SameLine();
      }
      if(ImGui::Button("ask the board"))
      {
          // SET_MODE is a COMMAND, refused while the car is armed and moving.
          // Asking is a separate act from asserting, so it is a button, not
          // something the combo does.
          const UInt8 want = modeOf(v.assumedMode);
          vlog::line("drive: ask the board pressed - SET_MODE %u", static_cast<UInt32>(want));
          link::sendCommand(lk, bibowire::Verb::VERB_SET_MODE, want, 0, 0);
          ++v.sent;
      }
      ImGui::EndDisabled();
      // A mode asserted at a board in another mode is why the throttle does nothing.
      const Opt<link::Control> ctl = snap.state.controlState(nowMs);
      if(ctl.has_value() && ctl->state.pilotMode != modeOf(v.assumedMode))
      {
          ImGui::TextColored(
              ImVec4(1.0f, 0.72f, 0.30f, 1.0f),
              "board is in %s - throttle refused",
              bibowire::pilotModeName(ctl->state.pilotMode)
          );
      }
      // Control over TCP works under the same rules, but queued behind telemetry.
      if(snap.controlOnTcp)
      {
          ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.30f, 1.0f), "control over TCP - degraded");
      }
      ImGui::Separator();
      drawBadge(snap, nowMs);
      drawCommands(v, lk, snap);
      drawAck(snap);
      // Dropped commands stay outside the details fold.
      const UInt32 lost = link::commandsDropped(lk);
      if(lost > 0u)
      {
          Array<Char, 64> bad = {};
          std::snprintf(bad.data(), bad.size(), "%u never sent", lost);
          readout("dropped", bad.data());
      }
      if(ImGui::CollapsingHeader("details"))
      {
          if(ctl.has_value())
          {
              readout("board mode", bibowire::pilotModeName(ctl->state.pilotMode));
          }
          readoutStr("steer", milliText(in.steerMilli));
          readoutStr("throttle", milliText(in.throttleMilli));
          Array<Char, 64> bits = {};
          std::snprintf(
              bits.data(),
              bits.size(),
              "%s%s%s%s",
              (in.buttons & bibowire::BUTTON_ESTOP) != 0u ? "ESTOP " : "",
              (in.buttons & bibowire::BUTTON_ENABLE) != 0u ? "ENABLE " : "",
              (in.buttons & bibowire::BUTTON_IDLE_TEST) != 0u ? "IDLE_TEST " : "",
              in.buttons == 0u ? "none" : ""
          );
          readout("buttons", bits.data());
          Array<Char, 96> tally = {};
          std::snprintf(
              tally.data(),
              tally.size(),
              "%u sent, %u failed (%s)",
              snap.controlSent,
              snap.controlFailed,
              snap.controlOnTcp ? "TCP" : "UDP"
          );
          readout("datagrams", tally.data());
          Array<Char, 48> count = {};
          std::snprintf(count.data(), count.size(), "%u", v.sent);
          readout("commands", count.data());
          ImGui::Separator();
          drawBoardSide(snap, nowMs);
      }
      ImGui::End();
  }
}
