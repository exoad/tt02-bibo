// Application layer. The platform shell (main.cpp) owns the window, the D3D11
// device and the ImGui frame lifecycle; everything inside the frame is here.
#pragma once

#include "lights.hpp"

#include "shared.hpp"

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



} // namespace app
