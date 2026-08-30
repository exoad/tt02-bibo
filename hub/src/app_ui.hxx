// Application layer. The platform shell (main.cxx) owns the window, the D3D11
// device and the ImGui frame lifecycle; everything inside the frame is here.
#pragma once

#include "lights.hxx"

#include "shared.hxx"

namespace app
{

  // Called once after ImGui is initialised and fonts/style are loaded.
  Void init(Float32 dpiScale);

  // Called once per frame, between ImGui::NewFrame() and ImGui::Render().
  Void frame();

  // Called once before ImGui shuts down. Must stop any worker threads.
  Void shutdown();

  // Called on WM_DPICHANGED, alongside ui::setDpiScale, so the layout's own
  // scaling stays in step with the rescaled style and fonts.
  Void setDpiScale(Float32 dpiScale);

  // Something was plugged in or unplugged. Called from the window procedure on
  // WM_DEVICECHANGE; safe to call as often as Windows sends it, because the
  // actual rescan is debounced.
  //
  // Event-driven rather than polled because the answer is almost always "nothing
  // changed", and asking Windows for the port list sixty times a second to find
  // that out would be sixty registry walks a second for no reason.
  Void notifyDeviceChange();



} // namespace app
