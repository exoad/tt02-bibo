#include "shared.hxx"

#include <cmath>
#include <cstdio>

// No <windows.h> and no <d3d11.h>, for trim.cxx's reason: this window owns no
// texture and needs no device, so none of camera.cxx's `small`/`near`/`far`/
// SEVERITY_ERROR macro surgery belongs here. Copying that block into a file
// that does not need it makes the file look as though it touches Windows.
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

    // bibowire::PilotMode, spelled for the combo. The WIRE values are 0, 1, 2
    // and the order here is those numbers - a table whose index IS the value,
    // so there is no mapping to get wrong.
    constexpr Array<CharSeq, 3> MODE_NAMES = { "manual", "look", "drive" };

    // ---- text ---------------------------------------------------------------

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

    // 0 nobody, 1 you, 2 another viewer - CTLSTATE's own byte. "Nobody" is a
    // real and important answer: with no holder the board's deadman does not
    // apply at all and the pilot runs under its own rules, which is a different
    // car from the one this window thinks it is driving.
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

    // Named by the protocol module, so the viewer and the board cannot disagree
    // about what a 2 means. main.cxx has the same three lines for the Car
    // window; they are four tokens of duplication rather than a shared helper
    // because the alternative is a header of UI vocabulary that two panes would
    // then have to agree on.
    [[nodiscard]] CharSeq deadmanText(UInt8 v)
    {
        if(v > static_cast<UInt8>(bibowire::deadman::State::STATE_ESTOP))
        {
            return "--";
        }
        return bibowire::deadman::stateName(static_cast<bibowire::deadman::State>(v));
    }

    // ---- may this pane drive at all -----------------------------------------

    // WELCOME's capabilities, bit 0 - docs/bibowire.md section 4. A pilot
    // started `--dry` clears it, and a viewer that offered a throttle anyway
    // would be offering a control the board has already said it will not apply.
    [[nodiscard]] Bool canDrive(const link::Snapshot& snap)
    {
        return snap.state.haveWelcome && (snap.state.welcome.capabilities & 0x01u) != 0u;
    }

    // WHY the keys are dead, or empty when they are not. A disabled control with
    // no sentence beside it is a program refusing to say what it wants, and here
    // it would be a person pressing W at a car that cannot hear them.
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
            // WELCOME capabilities bit 0 clear: the pilot is running --dry.
            return "this board cannot drive (pilot running --dry)";
        }
        if(!link::holdsSlot(snap.state))
        {
            // bibowire v1 has no message that takes the slot mid-session: it is
            // asked for in HELLO, so taking it means reconnecting.
            return "watching only - tick request control and Reconnect";
        }
        return "";
    }

    // ---- the keyboard --------------------------------------------------------

    // SUPPRESSED UNLESS THIS WINDOW HAS FOCUS AND NOTHING IS BEING TYPED. This
    // is not a nicety: `bibobox` typed into the host field contains an `o`, an
    // `x` and a `b`, and one careless mapping later a hostname is a throttle
    // command. `WantTextInput` covers a text field with a
    // caret in it, `IsAnyItemActive` covers a slider mid-drag, and the focus
    // test covers the whole rest of the program - including the 3D view, whose
    // R key is read the same way one file over.
    //
    // Dear ImGui clears its key state when the application loses focus, so a key
    // cannot be left latched down by alt-tabbing away mid-corner. The return
    // here is all-false rather than "the last keys", which is the safe
    // direction: no key is neutral, and neutral is what the board applies.
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

    // ---- what the round-trip log is told --------------------------------------
    //
    // EDGES ONLY. drawWindow runs every frame and publishes a LEVEL, which is
    // right for the wire and ruinous for a log. What a person reconstructing a
    // round trip needs is the moment a key went down, the moment the enable was
    // ticked, and the moment this pane decided the keys were dead - each ONCE,
    // stamped, so it can be laid beside the worker's CONTROL lines and the
    // board's own journal.
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

    // Key edges ONLY WHILE KEYS ARE ACCEPTED. When they stop being accepted
    // every key reads as up at once, and one line saying so is the truth; five
    // "up" lines would claim five fingers lifted that may still be resting there.
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

    // ---- the held steering ---------------------------------------------------
    //
    // EDGES ONLY IN THE LOG, for the Journal's reason. A ramp step is sixty lines
    // a second of nothing; a reset is one line that says why the wheel went back.

    // Back to centre, said once with the reason - and ONLY when there was
    // something to put back. A closed window calls this every frame, and must
    // not write a line a frame about a wheel that is already straight.
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

    // This frame's length in whole milliseconds. The float stops here: nothing
    // after this line prints one, and steerHeldStep clamps what it is given.
    [[nodiscard]] Int32 frameMs(const ImGuiIO& io)
    {
        return static_cast<Int32>(std::lround(io.DeltaTime * 1000.0f));
    }

    // One frame of the held steering. drawWindow has already reset it for a
    // closed, collapsed or blocked pane; what is left is the enable and the stop.
    //
    // NOT RESET FOR LOST FOCUS. An unfocused window reads every key as up, so
    // steerHeldStep springs the wheel back to centre at the rate, exactly as
    // letting go of the keys would.
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

    // THE HELD STEERING AS A BAR THAT FILLS FROM THE MIDDLE. A ProgressBar fills
    // from the left, and half full reads as "halfway" rather than "straight".
    // Left of the tick is left on the car: positive steer is right
    // (test_link.cxx, testSteerSigns).
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

    // ---- the board's answer to a COMMAND ------------------------------------
    //
    // trim.cxx's drawAck, and the same argument: a refusal must be VISIBLE.
    // Here it is worse than a slider that did not take - "ARM" refused and not
    // shown is an operator who believes the car is armed.
    Void drawAck(const link::Snapshot& snap)
    {
        // ONLY A REFUSAL, in the board's own words. An accepted ARM needs no line
        // - the badge above already says what it did - and a refused trim is the
        // Trim pane's to show.
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

    // ---- colour, so a glance tells the acts apart -----------------------------
    //
    // ONE MEANING PER COLOUR, used the same way for a button and for the state
    // it produces. Four grey buttons in a row were reported as "so many buttons
    // it can be easy to misclick one", and the two that sat side by side were
    // ARM and DISARM - opposite acts, identical to the eye.
    //
    //   AMBER  makes the car LIVE      - ARM, and the ARMED state
    //   GREEN  makes the car SAFE      - DISARM, and DISARMED / a live deadman
    //   RED    STOPS it                - ESTOP, and a latched estop or a dead deadman
    //   BLUE   housekeeping            - CLEAR ESTOP
    //
    // Amber for ARM rather than green, deliberately: green would say "go", and
    // the act it names is the one that lets the car move.
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

    // Text in the same four meanings, a little lighter so it reads on the
    // window background rather than on a button.
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

    // ---- the discrete acts ---------------------------------------------------
    //
    // TCP COMMANDs, each answered by exactly one CMDACK. They are not CONTROL
    // fields and must not be: a deliberate act carried twenty times a second by
    // a stream whose whole purpose is repetition is an act nobody can point at.
    //
    // LAID OUT SO A SLIP CANNOT LAND ON THE OPPOSITE ACT. ARM and DISARM share a
    // row, each half its width with a clear gap between them; ESTOP has a row
    // of its own, full width and taller, so it is the easiest thing in the
    // window to hit and never beside "arm"; CLEAR ESTOP sits below it, small.
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

        // NOT DISABLED WITH THE REST. The stop is the one act that must be
        // available in every state this window can be in, and it has two paths
        // on purpose: this one rides TCP and is acknowledged, the Space bar's
        // bit rides the 20 Hz stream and needs no round trip. Either latches.
        if(toneButton("ESTOP  (Space)", TONE_STOP, ImVec2(rowWidth, tall * 1.25f)))
        {
            vlog::line("drive: ESTOP pressed");
            resetSteer(v, "ESTOP button");
            link::sendCommand(lk, bibowire::Verb::VERB_ESTOP, 0, 0, 0);
            ++v.sent;
        }
        // The latch is undone by CLEAR ESTOP while disarmed and then an ARM - a
        // stop a released key could undo would be undone by accident.
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

    // THE STATE IN ONE WORD, IN ITS COLOUR, above the buttons that change it.
    // The ESTOP latch outranks armed: a latched car is stopped whatever else is
    // true. With no CTLSTATE there is nothing true to say, and a blank would
    // read as "disarmed".
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

    // ---- what the board says it is doing -------------------------------------
    //
    // ALL OF IT FROM CTLSTATE, and the countdowns especially. The board computes
    // neutralInMs and disarmInMs with bibowire::deadman::step - the same pure
    // function that does the tripping - so a viewer that ran its own copy beside
    // them would be showing a second arithmetic that can disagree with the one
    // that stops the car. There is deliberately no deadman::step call in this
    // file.
    Void drawBoardSide(const link::Snapshot& snap, Int64 nowMs)
    {
        const Opt<link::Control> ctl = snap.state.controlState(nowMs);
        // Missing CTLSTATE is said once, by drawBadge; "--" in every row here
        // would read as "all zero".
        if(!ctl.has_value())
        {
            return;
        }

        const bibowire::CtlState& s = ctl->state;

        readout("holder", holderText(s.holder));

        ImGui::TextUnformatted("armed");
        ImGui::SameLine(READOUT_COLUMN * uiScale);
        ImGui::TextColored(s.armed != 0u ? TEXT_LIVE : TEXT_SAFE, "%s", s.armed != 0u ? "yes" : "no");

        // live green, soft amber, dead and estop red - the deadman's own meanings.
        const ImVec4 deadmanTone = s.deadman == 0u ? TEXT_SAFE : (s.deadman == 1u ? TEXT_LIVE : TEXT_STOP);
        ImGui::TextUnformatted("deadman");
        ImGui::SameLine(READOUT_COLUMN * uiScale);
        ImGui::TextColored(deadmanTone, "%s", deadmanText(s.deadman));

        // WHY THROTTLE IS NOT BEING APPLIED, named rather than deduced. A
        // refusal whose only symptom is "throttle dead, steering fine" looks
        // like an ESC or a Pico fault, and those are where a person goes
        // looking first.
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

        // THE DEADMAN'S OWN COUNTDOWN. 0 means it has already fired, which is
        // not the same fact as "it fires now" - the board sends 0 for both the
        // instant it trips and every instant after.
        readoutStr("neutral in", msText(static_cast<Int64>(s.neutralInMs)));
        readoutStr("disarm in", msText(static_cast<Int64>(s.disarmInMs)));

        if(s.controlAgeMs == bibowire::CONTROL_AGE_NEVER)
        {
            // NEVER is not "a long time ago". The board has applied nothing from
            // this viewer at all, which is what an observer's stream looks like
            // from the other end.
            readout("control age", "never");
        }
        else
        {
            readoutStr("control age", msText(static_cast<Int64>(s.controlAgeMs)));
        }

        // SENT AGAINST APPLIED. This is the pair that catches a stream the board
        // stopped taking while every socket still looks perfect: "sent 412,
        // applied 411" is a link working, "sent 412, applied 96" is one that
        // stopped three hundred datagrams ago.
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
      // CLOSED IS NEUTRAL, AND IT IS PUBLISHED RATHER THAN ASSUMED. The slot is
      // not released - it belongs to the connection - so the worker keeps the
      // cadence and the board keeps hearing a live viewer; what it hears is a
      // viewer asking for nothing with its consent withdrawn, which is a SOFT
      // stop at 150 ms rather than the full stop silence buys at 300.
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
          // Collapsed. Not closed, but not watched either - and a throttle
          // nobody can see the state of is exactly the thing this pane is
          // careful about.
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
          // FORCED OFF, not merely greyed out. The enable is the bit the
          // deadman reads as consent, and leaving it set through a disconnect
          // would mean the first datagram of the NEXT connection carried an
          // operator's consent that they gave to a different session.
          v.enabled = false;
          v.boardArmed = false;
          resetSteer(v, "the pane is blocked");
          noteEnabled(false, "the pane is blocked");
      }

      // ARM TICKS THE ENABLE, AND A DISARM CLEARS IT - on the BOARD's word, not
      // on the button press, so a refused ARM ticks nothing. CTLSTATE is the
      // authority, and BOARD's picoArmed stands in when it is absent (its 2 is
      // UNKNOWN and says nothing). A frame with neither is not a disarm: a
      // dropped datagram must not cut the throttle. Edges only - see
      // View::boardArmed.
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
              noteEnabled(*armedNow, *armedNow ? "the board confirmed ARM" : "the board is no longer armed");
          }
      }

      // READ BEFORE ANYTHING IS DRAWN, so IsAnyItemActive describes the widget
      // the operator is actually holding rather than one this frame has not
      // submitted yet.
      const ImGuiIO& io = ImGui::GetIO();
      const Bool focused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
      const Bool typing = io.WantTextInput || ImGui::IsAnyItemActive();
      const Bool accept = focused && !typing && !blocked;
      const Keys keys = readKeys(accept);
      // C's EDGE, taken before noteKeys overwrites the journal's copy of last
      // frame's keys - holding C down is one centring, not one a frame.
      const Bool centrePressed = keys.centre && !journal.keys.centre;
      noteKeys(keys, accept);

      // THE HELD STEERING MOVES BEFORE THE INTENT IS BUILT, so what is
      // published this frame includes this frame's keys.
      steerFrame(v, keys, centrePressed, frameMs(io));

      // EVERY FRAME, whatever happened. This is a level and not an edge: the
      // worker samples it on the board's own 50 ms period, and a pane that
      // published only on change would make "no key pressed" and "the UI thread
      // hung" the same thing on the wire - which is the failure section 6 is
      // written to prevent, one layer further in.
      const link::Intent in = intentFrom(keys, v);
      link::setControl(lk, in);

      // ---- the slot ---------------------------------------------------------

      // HELLO carries the slot request and nothing else can, so a changed box
      // applies to the NEXT connection - hence Reconnect beside it.
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
          // A close and an open. The session ends, the sessionId and the epoch
          // are new, and the car - which stopped when the old stream did - stays
          // stopped until somebody arms it again.
          vlog::line(
              "drive: Reconnect pressed - request control on connect is %s",
              ask ? "ON" : "OFF"
          );
          static_cast<Void>(link::reconnect(lk));
      }
      ImGui::EndDisabled();

      if(link::isOpen(lk) && ask != link::holdsSlot(snap.state))
      {
          // The checkbox and the connection disagree: the box did nothing yet.
          ImGui::TextDisabled("Reconnect to apply");
      }

      if(blocked)
      {
          ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.30f, 1.0f), "%s", why.c_str());
      }

      ImGui::Separator();

      // ---- the hand on the wheel --------------------------------------------

      ImGui::BeginDisabled(blocked);

      // BUTTON_ENABLE on every datagram while ticked, and ARM ticks it (see
      // View::enabled). Unticking is a soft stop until the next ARM.
      if(ImGui::Checkbox("enable", &v.enabled))
      {
          noteEnabled(v.enabled, "the enable checkbox");
      }

      ImGui::TextDisabled(
          accept ? "A/D steer  W forward  S brake, again to reverse  C centre  Space ESTOP"
                 : "click this window to drive"
      );

      // S WITH REVERSE OFF stops and goes no further, which looks exactly like a
      // reverse that is broken - so it is said where the key is being pressed.
      if(keys.brake && v.reverseOff)
      {
          ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.30f, 1.0f), "reverse is off - set reverse us in Trim");
      }

      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      // W and S are switches, so this cap is the throttle's whole resolution -
      // a fraction the board maps onto the ESC limits the Trim pane set.
      ImGui::SliderInt("power", &v.throttleCapMilli, 0, THROTTLE_CAP_MAX, "%d / 1000");

      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      ImGui::SliderInt(
          "steer rate",
          &v.steerRateMilliPerS,
          STEER_RATE_MIN,
          STEER_RATE_MAX,
          "%d /s"
      );

      // Ctrl+click turns either slider into a text box, so the ranges are
      // re-applied here rather than trusted - the same settle settings.cxx
      // uses on a loaded file.
      settle(v);

      Array<Char, 64> lock = {};
      const Int32 lockMs = steerLockMs(v.steerRateMilliPerS);
      std::snprintf(lock.data(), lock.size(), "centre to full lock in %d ms", lockMs);
      ImGui::Indent(12.0f * uiScale);
      ImGui::TextUnformatted(lock.data());
      ImGui::Unindent(12.0f * uiScale);

      // WHERE THE WHEEL HAS BEEN LEFT, as a number and as a bar. The number is
      // what goes on the wire while enabled; "steer now" further down is what
      // the board says the servo is actually at.
      readoutStr("steering", milliText(v.steerHeldMilli));
      steerBar(v.steerHeldMilli);

      // ---- the mode this viewer ASSERTS -------------------------------------

      // Measured BEFORE the combo, while the cursor is still at the start of
      // the row - see the button below for what it decides.
      const Float32 rowWidth = ImGui::GetContentRegionAvail().x;
      ImGui::SetNextItemWidth(ITEM_WIDTH * uiScale);
      const Int32 modeCount = static_cast<Int32>(MODE_NAMES.size());
      // assumedMode: what the OPERATOR believes is active, carried on every
      // datagram and compared by the board - never copied from its answer.
      ImGui::Combo("mode", &v.assumedMode, MODE_NAMES.data(), modeCount);

      // BESIDE THE COMBO ONLY WHEN IT FITS. A plain SameLine ran the button past
      // the window's right edge at its default width. The combo's item rect
      // includes its label, so the arithmetic is the whole row as drawn.
      const ImGuiStyle& style = ImGui::GetStyle();
      const Float32 askWidth = ImGui::CalcTextSize("ask the board").x + (style.FramePadding.x * 2.0f);
      if(ImGui::GetItemRectSize().x + style.ItemSpacing.x + askWidth <= rowWidth)
      {
          ImGui::SameLine();
      }
      if(ImGui::Button("ask the board"))
      {
          // SET_MODE is a COMMAND, answered by a CMDACK, and it is refused while
          // the car is armed and moving. Asking is a separate act from
          // asserting, which is why this is a button beside the combo rather
          // than something the combo does.
          const UInt8 want = modeOf(v.assumedMode);
          vlog::line("drive: ask the board pressed - SET_MODE %u", static_cast<UInt32>(want));
          link::sendCommand(lk, bibowire::Verb::VERB_SET_MODE, want, 0, 0);
          ++v.sent;
      }

      ImGui::EndDisabled();

      // THE DISAGREEMENT NAMED - "the throttle does nothing" turned into its
      // reason: a mode asserted at a board that is in another one.
      const Opt<link::Control> ctl = snap.state.controlState(nowMs);
      if(ctl.has_value() && ctl->state.pilotMode != modeOf(v.assumedMode))
      {
          ImGui::TextColored(
              ImVec4(1.0f, 0.72f, 0.30f, 1.0f),
              "board is in %s - throttle refused",
              bibowire::pilotModeName(ctl->state.pilotMode)
          );
      }

      // DEGRADED IS A BANNER, not a footnote: control on TCP still works, with
      // the same rules at the far end, but queued behind telemetry.
      if(snap.controlOnTcp)
      {
          ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.30f, 1.0f), "control over TCP - degraded");
      }

      ImGui::Separator();

      // ---- what a person drives by ------------------------------------------

      drawBadge(snap, nowMs);
      drawCommands(v, lk, snap);
      drawAck(snap);

      // A COMMAND THAT NEVER LEFT stays visible; it is not a diagnostic.
      const UInt32 lost = link::commandsDropped(lk);
      if(lost > 0u)
      {
          Array<Char, 64> bad = {};
          std::snprintf(bad.data(), bad.size(), "%u never sent", lost);
          readout("dropped", bad.data());
      }

      // ---- everything else is for diagnosing, folded away -------------------

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
