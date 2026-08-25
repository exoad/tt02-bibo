// Platform shell: Win32 window + D3D11 device + Dear ImGui frame lifecycle.
//
// Everything that draws lives behind app::Init/Frame/Shutdown (app_ui.h). This
// file owns nothing but the window, the swap chain and the DPI bookkeeping.

#include <windows.h>
#include <dwmapi.h>
#include <d3d11.h>
#include <dxgi.h>

// <rpcndr.h>, dragged in by windows.h/d3d11.h, does `#define small char`, which
// collides with ui::shape::small in theme.h. Kill it before any project header.
#undef small
#undef near
#undef far

#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"

#include "app_ui.h"
#include "theme.h"


// ---------------------------------------------------------------------------
// Layout constants, in logical (96-dpi) pixels. Every one of them is scaled by
// the DPI of the monitor the window is actually on and then CLAMPED to that
// monitor's work area - see MinTrackSizeForWindow() / ClampToWorkArea(). A
// logical minimum that is merely multiplied by the scale is a trap: 1100x720
// logical at 175% is 1925x1260 physical, taller than a 1920x1080 laptop panel,
// and the window then cannot be shrunk or even fully seen.
// ---------------------------------------------------------------------------
static const int kDefaultWidth  = 1400;
static const int kDefaultHeight = 900;
static const int kMinWidth      = 880;   // rail (360) + gap + a usable radar
static const int kMinHeight     = 600;

// Absolute floor, in physical px, for the enforced minimum window size. Only
// reachable on a comically small work area; keeps the window grabbable.
static const LONG kFloorWidth   = 320;
static const LONG kFloorHeight  = 240;

// Fallback only. The real clear colour is taken from ImGui's own WindowBg each
// frame, so the backbuffer can never seam against the UI drawn on top of it.
static const float kClearColor[4] = { 0.06f, 0.06f, 0.06f, 1.0f };

#ifndef DWMWA_USE_IMMERSIVE_DARK_MODE
#define DWMWA_USE_IMMERSIVE_DARK_MODE 20
#endif

// ---------------------------------------------------------------------------
// D3D11 state
// ---------------------------------------------------------------------------
static ID3D11Device*           g_device        = nullptr;
static ID3D11DeviceContext*    g_context       = nullptr;
static IDXGISwapChain*         g_swapchain     = nullptr;
static ID3D11RenderTargetView* g_rtv           = nullptr;
static bool                    g_occluded      = false;
static UINT                    g_resize_w      = 0;
static UINT                    g_resize_h      = 0;

// ---------------------------------------------------------------------------
// DPI state
// ---------------------------------------------------------------------------
static float g_dpi_scale      = 1.0f;   // current window scale (1.0 == 96 dpi)
static float g_font_dpi_base  = 1.0f;   // scale the fonts were rasterised at
static bool  g_style_dirty    = false;  // set by WM_DPICHANGED

// ---------------------------------------------------------------------------
// Dynamically resolved user32 entry points. Resolved at runtime so the binary
// still starts on Windows versions that predate per-monitor-v2 DPI awareness.
// ---------------------------------------------------------------------------
typedef HANDLE  DpiAwarenessContext;
typedef BOOL  (WINAPI* PFN_SetProcessDpiAwarenessContext)(DpiAwarenessContext);
typedef UINT  (WINAPI* PFN_GetDpiForWindow)(HWND);
typedef BOOL  (WINAPI* PFN_AdjustWindowRectExForDpi)(LPRECT, DWORD, BOOL, DWORD, UINT);

static PFN_GetDpiForWindow          g_GetDpiForWindow          = nullptr;
static PFN_AdjustWindowRectExForDpi g_AdjustWindowRectExForDpi = nullptr;

// DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2
#define DPI_CONTEXT_PER_MONITOR_V2 ((DpiAwarenessContext)-4)

static void EnableDpiAwareness()
{
    HMODULE user32 = ::GetModuleHandleW(L"user32.dll");
    if (user32)
    {
        g_GetDpiForWindow          = (PFN_GetDpiForWindow)::GetProcAddress(user32, "GetDpiForWindow");
        g_AdjustWindowRectExForDpi = (PFN_AdjustWindowRectExForDpi)::GetProcAddress(user32, "AdjustWindowRectExForDpi");

        PFN_SetProcessDpiAwarenessContext set_ctx =
            (PFN_SetProcessDpiAwarenessContext)::GetProcAddress(user32, "SetProcessDpiAwarenessContext");
        if (set_ctx && set_ctx(DPI_CONTEXT_PER_MONITOR_V2))
            return;     // Windows 10 1703+ : per-monitor-v2, the good path.
    }

    // Windows 8.1 : per-monitor v1 via shcore.
    HMODULE shcore = ::LoadLibraryW(L"shcore.dll");
    if (shcore)
    {
        typedef HRESULT (WINAPI* PFN_SetProcessDpiAwareness)(int);
        PFN_SetProcessDpiAwareness set_awareness =
            (PFN_SetProcessDpiAwareness)::GetProcAddress(shcore, "SetProcessDpiAwareness");
        if (set_awareness && SUCCEEDED(set_awareness(2 /*PROCESS_PER_MONITOR_DPI_AWARE*/)))
            return;
    }

    // Vista+ : system DPI aware.
    ::SetProcessDPIAware();
}

static UINT DpiForWindow(HWND hwnd)
{
    if (g_GetDpiForWindow)
        return g_GetDpiForWindow(hwnd);
    HDC dc = ::GetDC(hwnd);
    UINT dpi = dc ? (UINT)::GetDeviceCaps(dc, LOGPIXELSX) : 96;
    if (dc) ::ReleaseDC(hwnd, dc);
    return dpi ? dpi : 96;
}

static void AdjustRectForDpi(LPRECT rc, DWORD style, DWORD ex_style, UINT dpi)
{
    if (g_AdjustWindowRectExForDpi)
        g_AdjustWindowRectExForDpi(rc, style, FALSE, ex_style, dpi);
    else
        ::AdjustWindowRectEx(rc, style, FALSE, ex_style);
}

// ---------------------------------------------------------------------------
// Work-area helpers. Sizes here are always PHYSICAL pixels.
// ---------------------------------------------------------------------------

// Work area (screen minus taskbar) of the monitor that `hwnd` is on. Pass a
// null hwnd before the window exists to get the primary monitor's.
static bool WorkAreaFor(HWND hwnd, RECT* out)
{
    HMONITOR mon = hwnd ? ::MonitorFromWindow(hwnd, MONITOR_DEFAULTTONEAREST)
                        : ::MonitorFromPoint(POINT{ 0, 0 }, MONITOR_DEFAULTTOPRIMARY);
    if (!mon)
        return false;

    MONITORINFO mi;
    ZeroMemory(&mi, sizeof(mi));
    mi.cbSize = sizeof(mi);
    if (!::GetMonitorInfoW(mon, &mi))
        return false;

    *out = mi.rcWork;
    return (out->right > out->left) && (out->bottom > out->top);
}

static void ClampToWorkArea(HWND hwnd, LONG* w, LONG* h)
{
    RECT work;
    if (WorkAreaFor(hwnd, &work))
    {
        const LONG max_w = work.right - work.left;
        const LONG max_h = work.bottom - work.top;
        if (*w > max_w) *w = max_w;
        if (*h > max_h) *h = max_h;
    }
    if (*w < kFloorWidth)  *w = kFloorWidth;
    if (*h < kFloorHeight) *h = kFloorHeight;
}

// Smallest OUTER window size we let the user drag to, in physical px:
// the logical minimum scaled by this window's real DPI, grown by the frame,
// then clamped so it can never exceed the monitor's work area.
static void MinTrackSizeForWindow(HWND hwnd, LONG* out_w, LONG* out_h)
{
    // Ask the window itself rather than trusting g_dpi_scale: WM_GETMINMAXINFO
    // fires during CreateWindow and around WM_DPICHANGED, when the cached
    // scale may not match this window's monitor yet.
    const UINT  dpi   = DpiForWindow(hwnd);
    const float scale = (float)dpi / 96.0f;

    RECT rc = { 0, 0,
                (LONG)(kMinWidth  * scale + 0.5f),
                (LONG)(kMinHeight * scale + 0.5f) };
    AdjustRectForDpi(&rc, WS_OVERLAPPEDWINDOW, 0, dpi);

    LONG w = rc.right - rc.left;
    LONG h = rc.bottom - rc.top;
    ClampToWorkArea(hwnd, &w, &h);

    *out_w = w;
    *out_h = h;
}

// ---------------------------------------------------------------------------
// D3D11 helpers
// ---------------------------------------------------------------------------
static void CreateRenderTarget()
{
    ID3D11Texture2D* backbuffer = nullptr;
    if (SUCCEEDED(g_swapchain->GetBuffer(0, IID_PPV_ARGS(&backbuffer))) && backbuffer)
    {
        g_device->CreateRenderTargetView(backbuffer, nullptr, &g_rtv);
        backbuffer->Release();
    }
}

static void CleanupRenderTarget()
{
    if (g_rtv) { g_rtv->Release(); g_rtv = nullptr; }
}

static bool CreateDeviceD3D(HWND hwnd)
{
    DXGI_SWAP_CHAIN_DESC sd;
    ZeroMemory(&sd, sizeof(sd));
    sd.BufferCount                        = 2;
    sd.BufferDesc.Width                   = 0;   // match the window
    sd.BufferDesc.Height                  = 0;
    sd.BufferDesc.Format                  = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate.Numerator   = 60;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.Flags                              = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    sd.BufferUsage                        = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow                       = hwnd;
    sd.SampleDesc.Count                   = 1;
    sd.SampleDesc.Quality                 = 0;
    sd.Windowed                           = TRUE;
    sd.SwapEffect                         = DXGI_SWAP_EFFECT_DISCARD;

    UINT flags = 0;
#ifdef _DEBUG
    // flags |= D3D11_CREATE_DEVICE_DEBUG;   // needs the graphics tools feature
#endif

    const D3D_FEATURE_LEVEL levels[2] = { D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0 };
    D3D_FEATURE_LEVEL got = D3D_FEATURE_LEVEL_11_0;

    HRESULT hr = D3D11CreateDeviceAndSwapChain(
        nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, flags, levels, 2,
        D3D11_SDK_VERSION, &sd, &g_swapchain, &g_device, &got, &g_context);

    if (hr == DXGI_ERROR_UNSUPPORTED)   // fall back to the WARP software rasteriser
        hr = D3D11CreateDeviceAndSwapChain(
            nullptr, D3D_DRIVER_TYPE_WARP, nullptr, flags, levels, 2,
            D3D11_SDK_VERSION, &sd, &g_swapchain, &g_device, &got, &g_context);

    if (FAILED(hr))
        return false;

    CreateRenderTarget();
    return true;
}

static void CleanupDeviceD3D()
{
    CleanupRenderTarget();
    if (g_swapchain) { g_swapchain->Release(); g_swapchain = nullptr; }
    if (g_context)   { g_context->Release();   g_context   = nullptr; }
    if (g_device)    { g_device->Release();    g_device    = nullptr; }
}

// ---------------------------------------------------------------------------
// App-layer lifetime.
//
// app::Shutdown() stops the scan, which is what stops the lidar MOTOR, so it
// must run on EVERY path out of the process that we can still observe: the
// normal quit, the window being destroyed from under us, and a session end
// (logoff/reboot), where Windows may terminate us right after WM_ENDSESSION
// without ever pumping another message. Idempotent, so all of them can fire.
// ---------------------------------------------------------------------------
static bool g_app_started = false;

static void ShutdownAppOnce()
{
    if (!g_app_started)
        return;
    g_app_started = false;
    app::Shutdown();
}

// Runs app::Shutdown() even if we leave WinMain through an early error return.
struct AppLifetimeGuard { ~AppLifetimeGuard() { ShutdownAppOnce(); } };

// ---------------------------------------------------------------------------
// Re-derive everything that depends on the DPI scale. Called between frames,
// never from inside the window procedure.
//
// Ordering, deliberately:
//   1. ui::ApplyStyle(dpi) - resets ImGuiStyle to defaults ITSELF and re-bakes
//      every size for the new scale. We must NOT reset the style here first:
//      ApplyStyle preserves an externally-set FontScaleDpi, and an external
//      `GetStyle() = ImGuiStyle()` would destroy the value it preserves.
//   2. ui::SetDpiScale / app::SetDpiScale - the widget + app layers size their
//      hand-drawn geometry off this, so they must track the style.
//   3. FontScaleDpi LAST, so this assignment always wins whatever ApplyStyle
//      did with it.
//
// Fonts are deliberately NOT reloaded. They are rasterised once at
// g_font_dpi_base; 1.92's dynamic atlas re-rasterises them for the ratio below
// on the next frame. Calling ui::LoadFonts() here as well would double-scale:
// LoadFonts re-bakes LegacySize at the new DPI *and* the ratio would still be
// applied on top of it.
// ---------------------------------------------------------------------------
static void RescaleUiForDpi()
{
    ui::ApplyStyle(g_dpi_scale);
    ui::SetDpiScale(g_dpi_scale);
    app::SetDpiScale(g_dpi_scale);

    ImGui::GetStyle().FontScaleDpi =
        (g_font_dpi_base > 0.0f) ? (g_dpi_scale / g_font_dpi_base) : 1.0f;
}

// ---------------------------------------------------------------------------
// Window procedure
// ---------------------------------------------------------------------------
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

static LRESULT WINAPI WndProc(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam)
{
    if (ImGui_ImplWin32_WndProcHandler(hwnd, msg, wparam, lparam))
        return 1;

    switch (msg)
    {
    case WM_SIZE:
    {
        if (wparam == SIZE_MINIMIZED)
            return 0;
        // Queue it; resizing the swap chain mid-message-pump is unsafe. A zero
        // extent (minimise races, SIZE_MAXHIDE) must never reach ResizeBuffers:
        // a 0-sized buffer is invalid and drops the render target on the floor.
        const UINT w = (UINT)LOWORD(lparam);
        const UINT h = (UINT)HIWORD(lparam);
        if (w == 0 || h == 0)
            return 0;
        g_resize_w = w;
        g_resize_h = h;
        return 0;
    }

    case WM_GETMINMAXINFO:
    {
        MINMAXINFO* mmi = (MINMAXINFO*)lparam;
        LONG w = 0, h = 0;
        MinTrackSizeForWindow(hwnd, &w, &h);
        mmi->ptMinTrackSize.x = w;
        mmi->ptMinTrackSize.y = h;
        return 0;
    }

    case WM_DPICHANGED:
    {
        // Windows hands us the rect that keeps the window the same physical
        // size on the new monitor; honouring it is required for v2 awareness.
        const RECT* suggested = (const RECT*)lparam;
        ::SetWindowPos(hwnd, nullptr,
                       suggested->left, suggested->top,
                       suggested->right - suggested->left,
                       suggested->bottom - suggested->top,
                       SWP_NOZORDER | SWP_NOACTIVATE);

        g_dpi_scale   = (float)HIWORD(wparam) / 96.0f;
        g_style_dirty = true;
        return 0;
    }

    case WM_SYSCOMMAND:
        if ((wparam & 0xfff0) == SC_KEYMENU)    // swallow the ALT menu
            return 0;
        break;

    case WM_ENDSESSION:
        // The session really is ending; we may be killed the moment we return.
        // Stop the motor now rather than hoping to reach the loop's exit.
        if (wparam)
            ShutdownAppOnce();
        return 0;

    case WM_DESTROY:
        // Covers close-button / Alt+F4 / taskbar-close, including the case
        // where the loop never gets another turn.
        ShutdownAppOnce();
        ::PostQuitMessage(0);
        return 0;
    }

    return ::DefWindowProcW(hwnd, msg, wparam, lparam);
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
int APIENTRY WinMain(HINSTANCE hinstance, HINSTANCE, LPSTR, int)
{
    EnableDpiAwareness();

    // Scale of the monitor the window will most likely open on.
    {
        HMONITOR mon = ::MonitorFromPoint(POINT{ 0, 0 }, MONITOR_DEFAULTTOPRIMARY);
        g_dpi_scale = ImGui_ImplWin32_GetDpiScaleForMonitor(mon);
        if (g_dpi_scale <= 0.0f)
            g_dpi_scale = 1.0f;
    }

    WNDCLASSEXW wc;
    ZeroMemory(&wc, sizeof(wc));
    wc.cbSize        = sizeof(wc);
    wc.style         = CS_CLASSDC;
    wc.lpfnWndProc   = WndProc;
    wc.hInstance     = hinstance;
    wc.hCursor       = ::LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = nullptr;     // we paint every pixel ourselves
    wc.lpszClassName = L"RplidarC1Window";
    ::RegisterClassExW(&wc);

    // Logical size -> physical, then grow by the non-client frame, then clamp
    // to the work area: 1400x900 logical is 2450x1575 at 175% and would open
    // far off the bottom of a 1080p panel.
    const DWORD style = WS_OVERLAPPEDWINDOW;
    RECT rc = { 0, 0,
                (LONG)(kDefaultWidth  * g_dpi_scale + 0.5f),
                (LONG)(kDefaultHeight * g_dpi_scale + 0.5f) };
    AdjustRectForDpi(&rc, style, 0, (UINT)(g_dpi_scale * 96.0f + 0.5f));

    LONG win_w = rc.right - rc.left;
    LONG win_h = rc.bottom - rc.top;
    ClampToWorkArea(nullptr, &win_w, &win_h);

    // If we know the work area, centre in it so the whole window is on-screen;
    // otherwise let the shell cascade it.
    int win_x = CW_USEDEFAULT;
    int win_y = CW_USEDEFAULT;
    {
        RECT work;
        if (WorkAreaFor(nullptr, &work))
        {
            win_x = (int)(work.left + ((work.right  - work.left) - win_w) / 2);
            win_y = (int)(work.top  + ((work.bottom - work.top)  - win_h) / 2);
        }
    }

    HWND hwnd = ::CreateWindowExW(
        0, wc.lpszClassName, L"RPLIDAR C1", style,
        win_x, win_y, win_w, win_h,
        nullptr, nullptr, hinstance, nullptr);
    if (!hwnd)
    {
        ::UnregisterClassW(wc.lpszClassName, hinstance);
        return 1;
    }

    // Dark title bar + frame, so the chrome matches the M3 dark surface.
    {
        BOOL dark = TRUE;
        ::DwmSetWindowAttribute(hwnd, DWMWA_USE_IMMERSIVE_DARK_MODE, &dark, sizeof(dark));
    }

    // The real DPI of the monitor the window actually landed on.
    g_dpi_scale = (float)DpiForWindow(hwnd) / 96.0f;

    if (!CreateDeviceD3D(hwnd))
    {
        CleanupDeviceD3D();
        ::DestroyWindow(hwnd);
        ::UnregisterClassW(wc.lpszClassName, hinstance);
        ::MessageBoxW(nullptr, L"Failed to create a Direct3D 11 device.", L"RPLIDAR C1", MB_ICONERROR | MB_OK);
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = nullptr;       // no imgui.ini next to the exe

    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(g_device, g_context);

    // Fonts are rasterised once, at this scale; remember it before anything
    // derives a ratio from it.
    ui::LoadFonts(g_dpi_scale);
    g_font_dpi_base = g_dpi_scale;

    ui::ApplyStyle(g_dpi_scale);
    ui::SetDpiScale(g_dpi_scale);
    ImGui::GetStyle().FontScaleDpi = 1.0f;   // baked size == requested size

    app::Init(g_dpi_scale);
    g_app_started = true;
    AppLifetimeGuard app_guard;              // Shutdown() on every return below

    ::ShowWindow(hwnd, SW_SHOWDEFAULT);
    ::UpdateWindow(hwnd);

    bool done = false;
    while (!done)
    {
        MSG msg;
        while (::PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE))
        {
            ::TranslateMessage(&msg);
            ::DispatchMessage(&msg);
            if (msg.message == WM_QUIT)
                done = true;
        }
        if (done)
            break;

        // Minimised or the screen is locked: don't burn a core spinning.
        if (g_occluded && g_swapchain->Present(0, DXGI_PRESENT_TEST) == DXGI_STATUS_OCCLUDED)
        {
            ::Sleep(10);
            continue;
        }
        g_occluded = false;

        if (g_resize_w != 0 && g_resize_h != 0)
        {
            const UINT rw = g_resize_w, rh = g_resize_h;
            g_resize_w = g_resize_h = 0;
            CleanupRenderTarget();
            if (SUCCEEDED(g_swapchain->ResizeBuffers(0, rw, rh, DXGI_FORMAT_UNKNOWN, 0)))
                CreateRenderTarget();
        }

        // A failed resize (or a lost device) leaves us without a target; retry
        // once per tick rather than drawing into nothing.
        if (g_rtv == nullptr)
        {
            CreateRenderTarget();
            if (g_rtv == nullptr)
            {
                ::Sleep(10);
                continue;
            }
        }

        if (g_style_dirty)
        {
            RescaleUiForDpi();
            g_style_dirty = false;
        }

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        app::Frame();

        ImGui::Render();

        // Track ImGui's own window background, so restyling the UI can never
        // leave the backbuffer showing a different colour behind it.
        const ImVec4 bg = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
        const float clear[4] = { bg.x, bg.y, bg.z, 1.0f };

        g_context->OMSetRenderTargets(1, &g_rtv, nullptr);
        g_context->ClearRenderTargetView(g_rtv, bg.w > 0.0f ? clear : kClearColor);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

        HRESULT hr = g_swapchain->Present(1, 0);    // vsync
        g_occluded = (hr == DXGI_STATUS_OCCLUDED);
    }

    ShutdownAppOnce();      // usually already done by WM_DESTROY; idempotent

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    CleanupDeviceD3D();
    ::DestroyWindow(hwnd);
    ::UnregisterClassW(wc.lpszClassName, hinstance);
    return 0;
}
