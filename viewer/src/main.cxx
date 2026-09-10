// bibo viewer - a Win32 window, a D3D11 device, and Dear ImGui's frame loop.
//
// This file owns the platform and the three floating panels. Everything that is
// three-dimensional lives in scene.cxx and draws through ImGui's own draw list.
//
// It replaces a 45,000-line predecessor on purpose. There is no editor here, no
// language server and no reference browser, because the code is written
// elsewhere: one 3D view, and a few plain windows floating over it.

#include "shared.hxx"

#include <cstdio>
#include <cmath>
#include <windows.h>
#include <dwmapi.h>
#include <d3d11.h>

// <rpcndr.h>, dragged in by the headers above, does `#define small char`; near
// and far come from the same era. Kill them before any header of ours.
#undef small
#undef near
#undef far

#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"

#include "scene.hxx"

#ifndef DWMWA_USE_IMMERSIVE_DARK_MODE
#define DWMWA_USE_IMMERSIVE_DARK_MODE 20
#endif

// Logical (96 dpi) pixels; scaled by the monitor's DPI before use.
static constexpr Int32 DEFAULT_WIDTH = 1400;
static constexpr Int32 DEFAULT_HEIGHT = 900;

// The viewport's own background. Deliberately NOT ImGui's WindowBg: the 3D view
// fills the whole client area, and a panel floating over it has to be readable
// AS a panel. Making the ground darker than the windows is what separates them.
static constexpr Array<Float32, 4> CLEAR_COLOR = { 0.086f, 0.094f, 0.110f, 1.0f };

static constexpr CharSeq DEFAULT_HOST = "bibobox.local";
static constexpr Int32 DEFAULT_PORT = 8020;

// Where a readout's value starts, in logical pixels from the window's left.
static constexpr Float32 READOUT_COLUMN = 92.0f;

static ID3D11Device* d3dDevice = nullptr;
static ID3D11DeviceContext* d3dContext = nullptr;
static IDXGISwapChain* swapchain = nullptr;
static ID3D11RenderTargetView* rtv = nullptr;
static UINT resizeW = 0;
static UINT resizeH = 0;

// Monitor DPI as a multiplier: 1.25 on this machine. Everything laid out in
// logical pixels multiplies by it.
static Float32 uiScale = 1.0f;

// ---------------------------------------------------------------------------
// THE NETWORK SEAM.
//
// This is the whole of the viewer's link state, and it is NOT WIRED TO A
// SOCKET. The protocol module is being written in parallel; when it lands, the
// two `// SEAM` branches below become its open and close calls, `status`
// becomes whatever the client reports, and the scan it delivers replaces the
// single scene::fillSyntheticCloud() call in the frame loop.
//
// Nothing else in this program has to change for that, which is the reason the
// state is a struct here rather than four loose variables inside the window
// that happens to draw them.
// ---------------------------------------------------------------------------
struct Link
{
    Array<Char, 64> host = {};
    Int32 port = DEFAULT_PORT;
    Bool connected = false;
    Str status = "not connected";
};

static Void initLink(Link& link)
{
    // snprintf rather than a brace-initialised literal: docs/conventions.md is
    // explicit that a long literal in an Array<Char, N> is better written as
    // the format call than as a list of character constants.
    std::snprintf(link.host.data(), link.host.size(), "%s", DEFAULT_HOST);
}

// ---------------------------------------------------------------------------
// D3D11
// ---------------------------------------------------------------------------
static Void createRenderTarget()
{
    ID3D11Texture2D* backbuffer = nullptr;
    if(SUCCEEDED(swapchain->GetBuffer(0, IID_PPV_ARGS(&backbuffer))) && backbuffer)
    {
        d3dDevice->CreateRenderTargetView(backbuffer, nullptr, &rtv);
        backbuffer->Release();
    }
}

static Void cleanupRenderTarget()
{
    if(rtv)
    {
        rtv->Release();
        rtv = nullptr;
    }
}

static Bool createDeviceD3D(HWND hwnd)
{
    DXGI_SWAP_CHAIN_DESC sd;
    ZeroMemory(&sd, sizeof(sd));
    sd.BufferCount = 2;
    sd.BufferDesc.Width = 0;        // match the window
    sd.BufferDesc.Height = 0;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate.Numerator = 60;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = hwnd;
    sd.SampleDesc.Count = 1;
    sd.SampleDesc.Quality = 0;
    sd.Windowed = TRUE;
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

    constexpr D3D_FEATURE_LEVEL LEVELS[2] = { D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0 };
    D3D_FEATURE_LEVEL got = D3D_FEATURE_LEVEL_11_0;

    HRESULT hr = D3D11CreateDeviceAndSwapChain(
        nullptr,
        D3D_DRIVER_TYPE_HARDWARE,
        nullptr,
        0,
        LEVELS,
        2,
        D3D11_SDK_VERSION,
        &sd,
        &swapchain,
        &d3dDevice,
        &got,
        &d3dContext
    );

    // WARP, so the viewer still opens over RDP or on a machine with no usable
    // 3D driver. It draws lines and discs; software is fast enough for that.
    if(hr == DXGI_ERROR_UNSUPPORTED)
    {
        hr = D3D11CreateDeviceAndSwapChain(
            nullptr,
            D3D_DRIVER_TYPE_WARP,
            nullptr,
            0,
            LEVELS,
            2,
            D3D11_SDK_VERSION,
            &sd,
            &swapchain,
            &d3dDevice,
            &got,
            &d3dContext
        );
    }

    if(FAILED(hr))
    {
        return false;
    }

    createRenderTarget();
    return true;
}

static Void cleanupDeviceD3D()
{
    cleanupRenderTarget();
    if(swapchain)
    {
        swapchain->Release();
        swapchain = nullptr;
    }
    if(d3dContext)
    {
        d3dContext->Release();
        d3dContext = nullptr;
    }
    if(d3dDevice)
    {
        d3dDevice->Release();
        d3dDevice = nullptr;
    }
}

// ---------------------------------------------------------------------------
// Window
// ---------------------------------------------------------------------------
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

static LRESULT WINAPI wndProc(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam)
{
    if(ImGui_ImplWin32_WndProcHandler(hwnd, msg, wparam, lparam))
    {
        return 1;
    }

    switch(msg)
    {
    case WM_SIZE:
    {
        if(wparam == SIZE_MINIMIZED)
        {
            return 0;
        }
        // Queued, not applied: resizing the swap chain from inside the message
        // pump is unsafe, and a zero extent must never reach ResizeBuffers.
        const UINT w = static_cast<UINT>(LOWORD(lparam));
        const UINT h = static_cast<UINT>(HIWORD(lparam));
        if(w == 0 || h == 0)
        {
            return 0;
        }
        resizeW = w;
        resizeH = h;
        return 0;
    }

    case WM_SYSCOMMAND:
        if((wparam & 0xfff0) == SC_KEYMENU)     // swallow the ALT menu
        {
            return 0;
        }
        break;

    case WM_DESTROY:
        ::PostQuitMessage(0);
        return 0;
    }

    return ::DefWindowProcW(hwnd, msg, wparam, lparam);
}

// ---------------------------------------------------------------------------
// The floating windows. Plain ImGui: no theme, no design system, no status bar.
// ---------------------------------------------------------------------------
static Void readout(CharSeq label, CharSeq value)
{
    ImGui::TextUnformatted(label);
    ImGui::SameLine(READOUT_COLUMN * uiScale);
    ImGui::TextUnformatted(value);
}

static Void drawConnectionWindow(Link& link)
{
    ImGui::SetNextWindowPos(ImVec2(16.0f * uiScale, 16.0f * uiScale), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300.0f * uiScale, 0.0f), ImGuiCond_FirstUseEver);
    if(!ImGui::Begin("Connection"))
    {
        ImGui::End();
        return;
    }

    // A HOSTNAME, not an address. The field network is a phone hotspot and its
    // DHCP hands out a different address every time; the board answers to
    // bibobox.local over mDNS and that is the thing that stays true.
    ImGui::SetNextItemWidth(-70.0f * uiScale);
    ImGui::InputText("host", link.host.data(), link.host.size());
    ImGui::SetNextItemWidth(-70.0f * uiScale);
    ImGui::InputInt("port", &link.port);

    ImGui::BeginDisabled(link.connected);
    if(ImGui::Button("Connect"))
    {
        // SEAM: the protocol client's open call goes here.
        link.connected = true;
        link.status = "connect requested - no client wired yet";
    }
    ImGui::EndDisabled();

    ImGui::SameLine();

    ImGui::BeginDisabled(!link.connected);
    if(ImGui::Button("Disconnect"))
    {
        // SEAM: and its close call here.
        link.connected = false;
        link.status = "not connected";
    }
    ImGui::EndDisabled();

    ImGui::Separator();
    ImGui::TextUnformatted(link.status.c_str());
    ImGui::End();
}

static Void drawViewWindow(scene::Scene& sc)
{
    static constexpr Array<CharSeq, 2> COLOR_NAMES = { "uniform", "by distance" };

    ImGui::SetNextWindowPos(ImVec2(16.0f * uiScale, 176.0f * uiScale), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300.0f * uiScale, 0.0f), ImGuiCond_FirstUseEver);
    if(!ImGui::Begin("View"))
    {
        ImGui::End();
        return;
    }

    ImGui::Checkbox("grid", &sc.opt.grid);
    ImGui::Checkbox("points", &sc.opt.points);
    ImGui::Checkbox("car", &sc.opt.car);
    ImGui::Checkbox("axes", &sc.opt.axes);

    ImGui::Separator();

    ImGui::SetNextItemWidth(-88.0f * uiScale);
    ImGui::SliderFloat("size", &sc.opt.pointSize, 1.0f, 10.0f, "%.1f px");

    Int32 coloring = static_cast<Int32>(sc.opt.coloring);
    const Int32 colorCount = static_cast<Int32>(COLOR_NAMES.size());
    ImGui::SetNextItemWidth(-88.0f * uiScale);
    if(ImGui::Combo("color", &coloring, COLOR_NAMES.data(), colorCount))
    {
        sc.opt.coloring = static_cast<scene::PointColor>(coloring);
    }

    ImGui::Separator();

    if(ImGui::Button("Reset camera"))
    {
        scene::resetCamera(sc.cam);
    }
    ImGui::SameLine();
    ImGui::TextDisabled("or press R");
    ImGui::End();
}

static Void drawCarWindow()
{
    ImGui::SetNextWindowPos(ImVec2(16.0f * uiScale, 424.0f * uiScale), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300.0f * uiScale, 0.0f), ImGuiCond_FirstUseEver);
    if(!ImGui::Begin("Car"))
    {
        ImGui::End();
        return;
    }

    // Every one of these is a placeholder, and "--" is the honest reading for a
    // value this program has never been told. mode, clearance, steer and
    // throttle come from the protocol's DECIDE message - what the pilot chose -
    // and armed comes from CTLSTATE, which is what the board is actually doing.
    // Those are two different questions and the panel keeps them side by side
    // for that reason: a car that is commanded and not armed does not move.
    readout("mode", "--");
    readout("clearance", "--");
    readout("steer", "--");
    readout("throttle", "--");
    readout("armed", "--");
    ImGui::End();
}

// ---------------------------------------------------------------------------
// Camera input. Only when no ImGui window wants the mouse - otherwise dragging
// a slider would orbit the world behind it.
// ---------------------------------------------------------------------------
static Void handleCameraInput(scene::Camera& cam)
{
    const ImGuiIO& io = ImGui::GetIO();

    if(!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_R, false))
    {
        scene::resetCamera(cam);
    }

    if(io.WantCaptureMouse)
    {
        return;
    }

    if(ImGui::IsMouseDragging(ImGuiMouseButton_Left))
    {
        const ImVec2 d = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
        scene::orbit(cam, d.x, d.y);
        ImGui::ResetMouseDragDelta(ImGuiMouseButton_Left);
    }

    if(ImGui::IsMouseDragging(ImGuiMouseButton_Right))
    {
        const ImVec2 d = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
        scene::pan(cam, d.x, d.y);
        ImGui::ResetMouseDragDelta(ImGuiMouseButton_Right);
    }

    if(io.MouseWheel != 0.0f)
    {
        scene::zoom(cam, io.MouseWheel);
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
Int32 APIENTRY WinMain(HINSTANCE hinstance, HINSTANCE, LPSTR, Int32)
{
    ImGui_ImplWin32_EnableDpiAwareness();

    HMONITOR primary = ::MonitorFromPoint(POINT{ 0, 0 }, MONITOR_DEFAULTTOPRIMARY);
    uiScale = ImGui_ImplWin32_GetDpiScaleForMonitor(primary);
    if(uiScale <= 0.0f)
    {
        uiScale = 1.0f;
    }

    WNDCLASSEXW wc;
    ZeroMemory(&wc, sizeof(wc));
    wc.cbSize = sizeof(wc);
    wc.style = CS_CLASSDC;
    wc.lpfnWndProc = wndProc;
    wc.hInstance = hinstance;
    wc.hCursor = ::LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = nullptr;         // every pixel is painted by us
    wc.lpszClassName = L"BiboViewerWindow";
    ::RegisterClassExW(&wc);

    const DWORD style = WS_OVERLAPPEDWINDOW;
    // lround, not a cast of x + 0.5f: the two agree for the positive numbers a
    // window size always is, but the cast is the shape that rounds the wrong way
    // on a negative, so it reads as a bug wherever it is copied to next.
    RECT rc = { 0, 0,
                std::lround(DEFAULT_WIDTH * uiScale),
                std::lround(DEFAULT_HEIGHT * uiScale) };
    ::AdjustWindowRectEx(&rc, style, FALSE, 0);

    HWND hwnd = ::CreateWindowExW(
        0,
        wc.lpszClassName,
        L"bibo viewer",
        style,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        rc.right - rc.left,
        rc.bottom - rc.top,
        nullptr,
        nullptr,
        hinstance,
        nullptr
    );
    if(!hwnd)
    {
        ::UnregisterClassW(wc.lpszClassName, hinstance);
        return 1;
    }

    // Said explicitly: Windows picks the frame from the SYSTEM theme, so on a
    // light-mode machine the title bar would come back white above a dark UI.
    {
        BOOL dark = TRUE;
        ::DwmSetWindowAttribute(hwnd, DWMWA_USE_IMMERSIVE_DARK_MODE, &dark, sizeof(dark));
    }

    if(!createDeviceD3D(hwnd))
    {
        cleanupDeviceD3D();
        ::DestroyWindow(hwnd);
        ::UnregisterClassW(wc.lpszClassName, hinstance);
        ::MessageBoxW(nullptr, L"Failed to create a Direct3D 11 device.", L"bibo viewer", MB_ICONERROR);
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Docking is deliberately off - it is not in this branch of Dear ImGui and
    // it is not wanted: these are three small floating panels over one view.
    ImGui::StyleColorsDark();
    ImGui::GetStyle().WindowRounding = 4.0f;
    ImGui::GetStyle().FrameRounding = 3.0f;

    // ScaleAllSizes bakes the DPI into every padding and thickness; FontScaleDpi
    // is 1.92's own hook and re-rasterises the font at the new size rather than
    // stretching the old atlas. Both, or the text is sharp and tiny inside
    // widgets sized for it, or blurry inside widgets that are not.
    ImGui::GetStyle().ScaleAllSizes(uiScale);
    ImGui::GetStyle().FontScaleDpi = uiScale;

    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(d3dDevice, d3dContext);

    scene::Scene sc;
    scene::resetCamera(sc.cam);

    Link link;
    initLink(link);

    const TimePoint started = monoNow();

    ::ShowWindow(hwnd, SW_SHOWDEFAULT);
    ::UpdateWindow(hwnd);

    Bool done = false;
    while(!done)
    {
        MSG msg;
        while(::PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE))
        {
            ::TranslateMessage(&msg);
            ::DispatchMessage(&msg);
            if(msg.message == WM_QUIT)
            {
                done = true;
            }
        }
        if(done)
        {
            break;
        }

        if(resizeW != 0 && resizeH != 0)
        {
            const UINT rw = resizeW;
            const UINT rh = resizeH;
            resizeW = 0;
            resizeH = 0;
            cleanupRenderTarget();
            if(SUCCEEDED(swapchain->ResizeBuffers(0, rw, rh, DXGI_FORMAT_UNKNOWN, 0)))
            {
                createRenderTarget();
            }
        }

        if(rtv == nullptr)
        {
            createRenderTarget();
            if(rtv == nullptr)
            {
                ::Sleep(10);
                continue;
            }
        }

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        // THE SCAN. One call, and the real one replaces it: when the protocol
        // client exists, sc.cloud is filled from the board's scan message here
        // and scene::fillSyntheticCloud goes away.
        scene::fillSyntheticCloud(sc.cloud, elapsedS(started));

        handleCameraInput(sc.cam);

        // The view fills the window and the panels float over it, so it draws
        // into the BACKGROUND list - behind every ImGui window, whatever order
        // they were submitted in.
        const ImGuiViewport* area = ImGui::GetMainViewport();
        const scene::Viewport where = { area->WorkPos, area->WorkSize };
        scene::draw(ImGui::GetBackgroundDrawList(), where, sc);

        drawConnectionWindow(link);
        drawViewWindow(sc);
        drawCarWindow();

        ImGui::Render();

        d3dContext->OMSetRenderTargets(1, &rtv, nullptr);
        d3dContext->ClearRenderTargetView(rtv, CLEAR_COLOR.data());
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

        swapchain->Present(1, 0);       // vsync
    }

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    cleanupDeviceD3D();
    ::DestroyWindow(hwnd);
    ::UnregisterClassW(wc.lpszClassName, hinstance);
    return 0;
}
