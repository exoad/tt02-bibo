// Application layer. The platform shell (main.cpp) owns the window, the D3D11
// device and the ImGui frame lifecycle; everything inside the frame is here.
#pragma once

namespace app {

// Called once after ImGui is initialised and fonts/style are loaded.
void Init(float dpi_scale);

// Called once per frame, between ImGui::NewFrame() and ImGui::Render().
void Frame();

// Called once before ImGui shuts down. Must stop any worker threads.
void Shutdown();

// Called on WM_DPICHANGED, alongside m3::SetDpiScale, so the layout's own
// scaling stays in step with the rescaled style and fonts.
void SetDpiScale(float dpi_scale);

} // namespace app
