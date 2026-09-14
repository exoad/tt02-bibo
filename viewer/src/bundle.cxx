#include "bundle.hxx"

#include "imgui.h"
#include "vlog.hxx"

namespace bundleview
{
  namespace
  {
    Float32 uiScale = 1.0f;

    // The two ids whose windows already exist. The id is the wire's stable
    // identity (docs/bundles.md section 8), so naming it here names the
    // contract, not an implementation detail of the board.
    constexpr CharSeq ID_WASD = "net.exoad.tt02bibo.wasd";
    constexpr CharSeq ID_TRIM = "net.exoad.tt02bibo.trim";

    // WELCOME's capabilities bit 4 (docs/bibowire.md section 4).
    [[nodiscard]] Bool boardOffersBundles(const link::Snapshot& snap)
    {
        return snap.state.haveWelcome && (snap.state.welcome.capabilities & 0x10u) != 0u;
    }

    [[nodiscard]] Str needsText(UInt8 needs)
    {
        Str out;
        if((needs & bibowire::BUNDLE_NEEDS_LIDAR) != 0u)
        {
            out += "lidar";
        }
        if((needs & bibowire::BUNDLE_NEEDS_PICO) != 0u)
        {
            out += out.empty() ? "pico" : " pico";
        }
        if((needs & bibowire::BUNDLE_NEEDS_CAMERA) != 0u)
        {
            out += out.empty() ? "camera" : " camera";
        }
        if(out.empty())
        {
            out = "nothing";
        }
        if((needs & bibowire::BUNDLE_MAY_DRIVE) != 0u)
        {
            out += ", drives";
        }
        return out;
    }

    // The newest answer to a load or an unload, or null.
    [[nodiscard]] const link::Ack* newestBundleAck(const link::Snapshot& snap)
    {
        for(Size i = snap.state.acks.size(); i > 0; --i)
        {
            const link::Ack& a = snap.state.acks[i - 1];
            const Bool ours = a.ack.verb == bibowire::Verb::VERB_LOAD_BUNDLE
                           || a.ack.verb == bibowire::Verb::VERB_STOP_BUNDLE;
            if(ours)
            {
                return &a;
            }
        }
        return nullptr;
    }

    // BY INDEX AND GENERATION, never by string: COMMAND has no text field, and
    // the generation is what makes the index safe against a list the board
    // replaced while this window was open. 16 bits, as arg1 is.
    Void request(link::Client& lk, const link::Snapshot& snap, const bibowire::Bundle& b, Bool load)
    {
        const bibowire::Verb verb = load ? bibowire::Verb::VERB_LOAD_BUNDLE : bibowire::Verb::VERB_STOP_BUNDLE;
        const UInt16 gen = static_cast<UInt16>(snap.state.bundleGeneration & 0xFFFFu);
        link::sendCommand(lk, verb, static_cast<UInt8>(b.index), gen, 0u);
        vlog::line(
            "bundles: %s %s (index %u, generation %u)",
            load ? "load" : "unload",
            b.id.c_str(),
            static_cast<unsigned>(b.index),
            static_cast<unsigned>(gen)
        );
    }

    Void drawRows(link::Client& lk, const link::Snapshot& snap);

    Void drawMaster(View& v, link::Client& lk, const link::Snapshot& snap)
    {
        ImGui::SetNextWindowPos(ImVec2(372.0f * uiScale, 16.0f * uiScale), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(560.0f * uiScale, 0.0f), ImGuiCond_FirstUseEver);
        if(!ImGui::Begin("Bundles", &v.open))
        {
            ImGui::End();
            return;
        }
        if(snap.phase != link::Phase::PHASE_LIVE)
        {
            ImGui::TextDisabled("not connected");
        }
        else if(!boardOffersBundles(snap))
        {
            ImGui::TextDisabled("this board does not offer bundles");
        }
        else if(!snap.state.haveBundles)
        {
            ImGui::TextDisabled("waiting for the board's list");
        }
        else if(snap.state.bundles.empty())
        {
            ImGui::TextDisabled("this board runs no bundles");
        }
        else
        {
            drawRows(lk, snap);
        }
        ImGui::End();
    }
  }

  namespace
  {
    Void drawRows(link::Client& lk, const link::Snapshot& snap)
    {
        const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
        if(ImGui::BeginTable("bundles", 4, flags))
        {
            ImGui::TableSetupColumn("bundle", ImGuiTableColumnFlags_None, 1.0f);
            ImGui::TableSetupColumn("does", ImGuiTableColumnFlags_None, 3.2f);
            ImGui::TableSetupColumn("needs", ImGuiTableColumnFlags_None, 1.3f);
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 76.0f * uiScale);
            ImGui::TableHeadersRow();
            for(const bibowire::Bundle& b : snap.state.bundles)
            {
                ImGui::TableNextRow();
                ImGui::PushID(b.id.c_str());
                const Bool ready = b.ready != 0u;
                const Bool loaded = b.loaded != 0u;
                // GREYED WITH THE REASON, NOT HIDDEN: "not ready" is the answer to
                // why the car cannot do this, and a vanished row answers nothing.
                if(!ready)
                {
                    ImGui::BeginDisabled();
                }
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(b.name.c_str());
                if(loaded)
                {
                    ImGui::SameLine();
                    ImGui::TextDisabled("(loaded)");
                }
                ImGui::TableNextColumn();
                ImGui::TextWrapped("%s", b.about.c_str());
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(needsText(b.needs).c_str());
                if(!ready)
                {
                    ImGui::EndDisabled();
                }
                ImGui::TableNextColumn();
                if(!ready)
                {
                    ImGui::TextDisabled("not ready");
                    if(ImGui::IsItemHovered())
                    {
                        ImGui::SetTooltip("something it needs is missing on this car - the board decides that");
                    }
                }
                else if(ImGui::Button(loaded ? "Unload" : "Load"))
                {
                    request(lk, snap, b, !loaded);
                }
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        ImGui::Separator();
        if(snap.state.haveBundleState)
        {
            ImGui::Text("%u loaded", static_cast<unsigned>(snap.state.bundleState.loadedCount));
        }
        // The board's answer to the last Load or Unload, in its own words: a
        // refusal (a stale list, unmet needs) is why the button did nothing.
        if(const link::Ack* a = newestBundleAck(snap); a != nullptr)
        {
            const Bool refused = a->ack.result != 0u;
            const ImVec4 tone = refused ? ImVec4(1.0f, 0.55f, 0.35f, 1.0f) : ImVec4(
                0.6f,
                0.85f,
                0.6f,
                1.0f
            );
            ImGui::TextColored(tone, "%s: %s", link::ackResultName(a->ack.result), a->ack.text.c_str());
        }
        if(snap.state.haveBundleNote)
        {
            ImGui::TextWrapped("%s", snap.state.bundleNote.text.c_str());
        }
    }

    // The read-only window for an id this viewer has no pane for. Keyed on the
    // id past ###, so a renamed bundle keeps its saved position.
    Void drawFallback(View& v, const link::Snapshot& snap, const bibowire::Bundle& b)
    {
        Bool open = true;
        const Str title = b.name + "###bundle:" + b.id;
        ImGui::SetNextWindowSize(ImVec2(320.0f * uiScale, 0.0f), ImGuiCond_FirstUseEver);
        if(ImGui::Begin(title.c_str(), &open))
        {
            ImGui::TextWrapped("%s", b.about.c_str());
            ImGui::TextDisabled("%s", b.id.c_str());
            ImGui::Text("needs %s", needsText(b.needs).c_str());
            if(snap.state.haveBundleNote && snap.state.bundleNote.text.find(b.name) != Str::npos)
            {
                ImGui::TextWrapped("%s", snap.state.bundleNote.text.c_str());
            }
            ImGui::TextDisabled("read-only: this bundle declares no controls");
        }
        ImGui::End();
        if(!open)
        {
            v.closed.insert(b.id);
        }
    }
  }

  Void init(Float32 scale)
  {
      uiScale = scale > 0.0f ? scale : 1.0f;
  }

  Void drawWindow(View& v, link::Client& lk, const link::Snapshot& snap, Int64 nowMs)
  {
      static_cast<Void>(nowMs);
      if(v.open)
      {
          drawMaster(v, lk, snap);
      }
      // Load EDGES open a window once; a window the operator closed stays closed
      // until the bundle is loaded again. While no list is held (a reconnect),
      // nothing counts as loaded, so the next list opens every loaded bundle's
      // window afresh, which is what a viewer joining mid-run wants.
      Set<Str> nowLoaded;
      if(snap.state.haveBundles)
      {
          for(const bibowire::Bundle& b : snap.state.bundles)
          {
              if(b.loaded == 0u)
              {
                  continue;
              }
              nowLoaded.insert(b.id);
              const Bool edge = v.wasLoaded.count(b.id) == 0u;
              if(edge)
              {
                  v.closed.erase(b.id);
                  vlog::line("bundles: %s is loaded", b.id.c_str());
              }
              if(b.id == ID_WASD)
              {
                  if(edge && v.drive != nullptr)
                  {
                      v.drive->open = true;
                  }
              }
              else if(b.id == ID_TRIM)
              {
                  if(edge && v.trim != nullptr)
                  {
                      v.trim->open = true;
                  }
              }
              else if(v.closed.count(b.id) == 0u)
              {
                  drawFallback(v, snap, b);
              }
          }
      }
      v.wasLoaded = nowLoaded;
  }
}
