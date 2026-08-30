// Platform shell: Win32 window + D3D11 device + Dear ImGui frame lifecycle.
//
// Everything that draws lives behind app::init/Frame/Shutdown (app_ui.h). This
// file owns nothing but the window, the swap chain and the DPI bookkeeping.

#include "shared.hxx"
#include <cstdio>
#include <cstdlib>
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

#include "app_ui.hxx"
#include "icons.hxx"
#include "resource.h"
#include "scene_gpu.hxx"
#include "settings.hxx"
#include "theme.hxx"


// ---------------------------------------------------------------------------
// Layout constants, in logical (96-dpi) pixels. Every one of them is scaled by
// the DPI of the monitor the window is actually on and then CLAMPED to that
// monitor's work area - see minTrackSizeForWindow() / clampToWorkArea(). A
// logical minimum that is merely multiplied by the scale is a trap: 1100x720
// logical at 175% is 1925x1260 physical, taller than a 1920x1080 laptop panel,
// and the window then cannot be shrunk or even fully seen.
// ---------------------------------------------------------------------------
static const Int32 DEFAULT_WIDTH  = 1400;
static const Int32 DEFAULT_HEIGHT = 900;
static const Int32 MIN_WIDTH      = 880;   // rail (360) + gap + a usable radar
static const Int32 MIN_HEIGHT     = 600;

// Absolute floor, in physical px, for the enforced minimum window size. Only
// reachable on a comically small work area; keeps the window grabbable.
static const LONG FLOOR_WIDTH   = 320;
static const LONG FLOOR_HEIGHT  = 240;

// Fallback only. The real clear colour is taken from ImGui's own WindowBg each
// frame, so the backbuffer can never seam against the UI drawn on top of it.
// Fallback only - the frame normally clears to ImGuiCol_WindowBg. It still has
// to match the theme: anything the UI does not cover (the sliver during a
// resize, the moment before the first frame) shows this.
static const Float32 CLEAR_COLOR[4] = { 0.139f, 0.144f, 0.154f, 1.0f };

#ifndef DWMWA_USE_IMMERSIVE_DARK_MODE
#define DWMWA_USE_IMMERSIVE_DARK_MODE 20
#endif

// ---------------------------------------------------------------------------
// D3D11 state
// ---------------------------------------------------------------------------
static ID3D11Device*           d3dDevice        = nullptr;
static ID3D11DeviceContext*    d3dContext       = nullptr;
static IDXGISwapChain*         swapchain     = nullptr;
static ID3D11RenderTargetView* rtv           = nullptr;
static Bool                    occluded      = false;
static UINT                    resizeW      = 0;
static UINT                    resizeH      = 0;

// ---------------------------------------------------------------------------
// The UI scale, one line, its own file. Small enough that a format is overkill.
// ---------------------------------------------------------------------------
static Void loadUiScale()
{
    const Str txt = settings::read("ui-scale.txt");
    if(txt.empty())
        return;

    const Float64 v = std::atof(txt.c_str());
    if(v > 0.0)
        ui::setUserScale(static_cast<Float32>(v));
}

static Void saveUiScale()
{
    Char buf[32];
    std::snprintf(buf, sizeof(buf), "%.2f\n", static_cast<Float64>(ui::userScale()));
    settings::write("ui-scale.txt", Str(buf));
}

// ---------------------------------------------------------------------------
// The frame ceiling.
//
// Present(1, 0) syncs to the MONITOR, and this one runs at 240 Hz - so vsync
// alone was drawing this UI at 240 fps. Nothing on screen changes faster than
// the lidar produces it, which is 10 Hz: the other 230 frames a second were
// redrawing the same revolution at four times the power draw, on a laptop that
// is going to be sitting next to a car in a car park.
//
// A CreateWaitableTimerEx with HIGH_RESOLUTION rather than Sleep(): the default
// system timer resolution is 15.6 ms, so a plain Sleep(1) between frames would
// have produced something closer to 60 Hz by luck than by design - and on a
// machine where someone else had raised the global timer resolution it would
// have silently changed behaviour. The timer is a hard fallback to Sleep when
// the flag is unsupported (pre-1803), which caps at roughly the right rate and
// says so here rather than pretending.
// ---------------------------------------------------------------------------
static const Int32 TARGET_FPS = 60;

#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

static HANDLE          frameTimer = nullptr;
static LARGE_INTEGER   qpcFreq    = {};
static LARGE_INTEGER   frameNext  = {};

static Void initFrameLimiter()
{
    ::QueryPerformanceFrequency(&qpcFreq);
    ::QueryPerformanceCounter(&frameNext);

    frameTimer = ::CreateWaitableTimerExW(nullptr, nullptr,
                                          CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
                                          TIMER_ALL_ACCESS);
}

static Void shutdownFrameLimiter()
{
    if(frameTimer != nullptr)
    {
        ::CloseHandle(frameTimer);
        frameTimer = nullptr;
    }
}

static Void waitForNextFrame()
{
    if(qpcFreq.QuadPart == 0)
        return;

    frameNext.QuadPart += qpcFreq.QuadPart / TARGET_FPS;

    LARGE_INTEGER now;
    ::QueryPerformanceCounter(&now);

    // Behind schedule - a resize, a device reset, a stall. Resync to now rather
    // than trying to catch up, which would run a burst of uncapped frames to
    // "make up" time that is already spent.
    if(now.QuadPart >= frameNext.QuadPart)
    {
        frameNext = now;
        return;
    }

    const LONGLONG left = frameNext.QuadPart - now.QuadPart;

    if(frameTimer != nullptr)
    {
        // 100 ns units, negative for a relative wait.
        LARGE_INTEGER due;
        due.QuadPart = -((left * 10000000LL) / qpcFreq.QuadPart);
        if(due.QuadPart < 0
           && ::SetWaitableTimer(frameTimer, &due, 0, nullptr, nullptr, FALSE))
        {
            ::WaitForSingleObject(frameTimer, INFINITE);
            return;
        }
    }

    const DWORD ms = static_cast<DWORD>((left * 1000) / qpcFreq.QuadPart);
    if(ms > 0)
        ::Sleep(ms);
}

// ---------------------------------------------------------------------------
// DPI state
// ---------------------------------------------------------------------------
// What Windows says the monitor is, and what the app actually lays out against.
// The second is the first times the user's own zoom - see ui::userScale().
static Float32 monitorDpiScale       = 1.0f;   // 1.0 == 96 dpi
static Float32 geometryDpiScale      = 1.0f;   // monitorDpiScale * ui::userScale()
static Float32 fontDpiBase  = 1.0f;   // scale the fonts were rasterised at
static Bool  styleDirty    = false;  // set by WM_DPICHANGED

// ---------------------------------------------------------------------------
// Dynamically resolved user32 entry points. Resolved at runtime so the binary
// still starts on Windows versions that predate per-monitor-v2 DPI awareness.
// ---------------------------------------------------------------------------
typedef HANDLE  DpiAwarenessContext;
typedef BOOL  (WINAPI* PFN_SetProcessDpiAwarenessContext)(DpiAwarenessContext);
typedef UINT  (WINAPI* PFN_GetDpiForWindow)(HWND);
typedef BOOL  (WINAPI* PFN_AdjustWindowRectExForDpi)(LPRECT, DWORD, BOOL, DWORD, UINT);

static PFN_GetDpiForWindow          getDpiForWindowFn          = nullptr;
static PFN_AdjustWindowRectExForDpi adjustWindowRectExForDpiFn = nullptr;

// DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2
// The SDK spells this as a C-style cast in its own headers; this is the same
// sentinel written the way the style guide asks for.
static const DpiAwarenessContext DPI_CONTEXT_PER_MONITOR_V2 =
    reinterpret_cast<DpiAwarenessContext>(static_cast<INT_PTR>(-4));

static Void enableDpiAwareness()
{
    HMODULE user32 = ::GetModuleHandleW(L"user32.dll");
    if(user32)
    {
        // reinterpret_cast, not a C-style cast: GetProcAddress hands back a
        // FARPROC and turning one function-pointer type into another is exactly
        // what reinterpret_cast is for. The style guide bans the C-style
        // spelling; it does not ban the conversion.
        getDpiForWindowFn = reinterpret_cast<PFN_GetDpiForWindow>(
            ::GetProcAddress(user32, "GetDpiForWindow"));
        adjustWindowRectExForDpiFn = reinterpret_cast<PFN_AdjustWindowRectExForDpi>(
            ::GetProcAddress(user32, "AdjustWindowRectExForDpi"));

        PFN_SetProcessDpiAwarenessContext setCtx =
            reinterpret_cast<PFN_SetProcessDpiAwarenessContext>(
                ::GetProcAddress(user32, "SetProcessDpiAwarenessContext"));
        if(setCtx && setCtx(DPI_CONTEXT_PER_MONITOR_V2))
            return;     // Windows 10 1703+ : per-monitor-v2, the good path.
    }

    // Windows 8.1 : per-monitor v1 via shcore.
    HMODULE shcore = ::LoadLibraryW(L"shcore.dll");
    if(shcore)
    {
        typedef HRESULT (WINAPI* PFN_SetProcessDpiAwareness)(Int32);
        PFN_SetProcessDpiAwareness setAwareness =
            reinterpret_cast<PFN_SetProcessDpiAwareness>(
                ::GetProcAddress(shcore, "SetProcessDpiAwareness"));
        if(setAwareness && SUCCEEDED(setAwareness(2 /*PROCESS_PER_MONITOR_DPI_AWARE*/)))
            return;
    }

    // Vista+ : system DPI aware.
    ::SetProcessDPIAware();
}

static UINT dpiForWindow(HWND hwnd)
{
    if(getDpiForWindowFn)
        return getDpiForWindowFn(hwnd);
    HDC dc = ::GetDC(hwnd);
    UINT dpi = dc ? static_cast<UINT>(::GetDeviceCaps(dc, LOGPIXELSX)) : 96;
    if(dc)
    {
        ::ReleaseDC(hwnd, dc);
    }
    return dpi ? dpi : 96;
}

static Void adjustRectForDpi(LPRECT rc, DWORD style, DWORD exStyle, UINT dpi)
{
    if(adjustWindowRectExForDpiFn)
        adjustWindowRectExForDpiFn(rc, style, FALSE, exStyle, dpi);
    else
        ::AdjustWindowRectEx(rc, style, FALSE, exStyle);
}

// ---------------------------------------------------------------------------
// Work-area helpers. Sizes here are always PHYSICAL pixels.
// ---------------------------------------------------------------------------

// Work area (screen minus taskbar) of the monitor that `hwnd` is on. Pass a
// null hwnd before the window exists to get the primary monitor's.
static Bool workAreaFor(HWND hwnd, RECT* out)
{
    HMONITOR mon = hwnd ? ::MonitorFromWindow(hwnd, MONITOR_DEFAULTTONEAREST)
                        : ::MonitorFromPoint(POINT{ 0, 0 }, MONITOR_DEFAULTTOPRIMARY);
    if(!mon)
        return false;

    MONITORINFO mi;
    ZeroMemory(&mi, sizeof(mi));
    mi.cbSize = sizeof(mi);
    if(!::GetMonitorInfoW(mon, &mi))
        return false;

    *out = mi.rcWork;
    return (out->right > out->left) && (out->bottom > out->top);
}

static Void clampToWorkArea(HWND hwnd, LONG* w, LONG* h)
{
    RECT work;
    if(workAreaFor(hwnd, &work))
    {
        const LONG maxW = work.right - work.left;
        const LONG maxH = work.bottom - work.top;
        if(*w > maxW)
        {
            *w = maxW;
        }
        if(*h > maxH)
        {
            *h = maxH;
        }
    }
    if(*w < FLOOR_WIDTH)
    {
        *w = FLOOR_WIDTH;
    }
    if(*h < FLOOR_HEIGHT)
    {
        *h = FLOOR_HEIGHT;
    }
}

// Smallest OUTER window size we let the user drag to, in physical px:
// the logical minimum scaled by this window's real DPI, grown by the frame,
// then clamped so it can never exceed the monitor's work area.
static Void minTrackSizeForWindow(HWND hwnd, LONG* outW, LONG* outH)
{
    // Ask the window itself rather than trusting geometryDpiScale: WM_GETMINMAXINFO
    // fires during CreateWindow and around WM_DPICHANGED, when the cached
    // scale may not match this window's monitor yet.
    const UINT  dpi   = dpiForWindow(hwnd);
    const Float32 scale = (static_cast<Float32>(dpi) / 96.0f) * ui::userScale();

    RECT rc = { 0, 0,
                static_cast<LONG>((MIN_WIDTH  * scale + 0.5f)),
                static_cast<LONG>((MIN_HEIGHT * scale + 0.5f)) };
    adjustRectForDpi(&rc, WS_OVERLAPPEDWINDOW, 0, dpi);

    LONG w = rc.right - rc.left;
    LONG h = rc.bottom - rc.top;
    clampToWorkArea(hwnd, &w, &h);

    *outW = w;
    *outH = h;
}

// ---------------------------------------------------------------------------
// D3D11 helpers
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
        D3D11_SDK_VERSION, &sd, &swapchain, &d3dDevice, &got, &d3dContext);

    if(hr == DXGI_ERROR_UNSUPPORTED)   // fall back to the WARP software rasteriser
        hr = D3D11CreateDeviceAndSwapChain(
            nullptr, D3D_DRIVER_TYPE_WARP, nullptr, flags, levels, 2,
            D3D11_SDK_VERSION, &sd, &swapchain, &d3dDevice, &got, &d3dContext);

    if(FAILED(hr))
        return false;

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
        d3dContext   = nullptr;
    }
    if(d3dDevice)
    {
        d3dDevice->Release();
        d3dDevice    = nullptr;
    }
}

// ---------------------------------------------------------------------------
// App-layer lifetime.
//
// app::shutdown() stops the scan, which is what stops the lidar MOTOR, so it
// must run on EVERY path out of the process that we can still observe: the
// normal quit, the window being destroyed from under us, and a session end
// (logoff/reboot), where Windows may terminate us right after WM_ENDSESSION
// without ever pumping another message. Idempotent, so all of them can fire.
// ---------------------------------------------------------------------------
static Bool appStarted = false;

static Void shutdownAppOnce()
{
    if(!appStarted)
        return;
    appStarted = false;
    app::shutdown();
}

// Runs app::shutdown() even if we leave WinMain through an early error return.
struct AppLifetimeGuard
{
    ~AppLifetimeGuard()
    {
        shutdownAppOnce();
    }
};

// ---------------------------------------------------------------------------
// Re-derive everything that depends on the DPI scale. Called between frames,
// never from inside the window procedure.
//
// Ordering, deliberately:
//   1. ui::applyStyle(dpi) - resets ImGuiStyle to defaults ITSELF and re-bakes
//      every size for the new scale. We must NOT reset the style here first:
//      ApplyStyle preserves an externally-set FontScaleDpi, and an external
//      `GetStyle() = ImGuiStyle()` would destroy the value it preserves.
//   2. ui::setDpiScale / app::setDpiScale - the widget + app layers size their
//      hand-drawn geometry off this, so they must track the style.
//   3. FontScaleDpi LAST, so this assignment always wins whatever ApplyStyle
//      did with it.
//
// Fonts are deliberately NOT reloaded. They are rasterised once at
// fontDpiBase; 1.92's dynamic atlas re-rasterises them for the ratio below
// on the next frame. Calling ui::loadFonts() here as well would double-scale:
// LoadFonts re-bakes LegacySize at the new DPI *and* the ratio would still be
// applied on top of it.
// ---------------------------------------------------------------------------
static Void rescaleUiForDpi()
{
    ui::applyStyle(geometryDpiScale);
    ui::setDpiScale(geometryDpiScale);
    app::setDpiScale(geometryDpiScale);

    ImGui::GetStyle().FontScaleDpi =
        (fontDpiBase > 0.0f) ? (geometryDpiScale / fontDpiBase) : 1.0f;
}

// ---------------------------------------------------------------------------
// Window procedure
// ---------------------------------------------------------------------------
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

static LRESULT WINAPI wndProc(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam)
{
    if(ImGui_ImplWin32_WndProcHandler(hwnd, msg, wparam, lparam))
        return 1;

    switch(msg)
    {
    case WM_DEVICECHANGE:
        // Windows broadcasts DBT_DEVNODES_CHANGED to top-level windows whenever
        // the device tree changes, with no registration needed for that
        // particular event - which is exactly the "something was plugged in"
        // signal, for free. Not consumed: it is passed on below like any other
        // message we merely observe.
        app::notifyDeviceChange();
        break;

    case WM_SIZE:
    {
        if(wparam == SIZE_MINIMIZED)
            return 0;
        // Queue it; resizing the swap chain mid-message-pump is unsafe. A zero
        // extent (minimise races, SIZE_MAXHIDE) must never reach ResizeBuffers:
        // a 0-sized buffer is invalid and drops the render target on the floor.
        const UINT w = static_cast<UINT>(LOWORD(lparam));
        const UINT h = static_cast<UINT>(HIWORD(lparam));
        if(w == 0 || h == 0)
            return 0;
        resizeW = w;
        resizeH = h;
        return 0;
    }

    case WM_GETMINMAXINFO:
    {
        // reinterpret_cast: Windows passes the struct's address in an LPARAM,
        // and recovering a pointer from an integer is what that cast is for.
        MINMAXINFO* mmi = reinterpret_cast<MINMAXINFO*>(lparam);
        LONG w = 0, h = 0;
        minTrackSizeForWindow(hwnd, &w, &h);
        mmi->ptMinTrackSize.x = w;
        mmi->ptMinTrackSize.y = h;
        return 0;
    }

    case WM_DPICHANGED:
    {
        // Windows hands us the rect that keeps the window the same physical
        // size on the new monitor; honouring it is required for v2 awareness.
        const RECT* suggested = reinterpret_cast<const RECT*>(lparam);
        ::SetWindowPos(hwnd, nullptr,
                       suggested->left, suggested->top,
                       suggested->right - suggested->left,
                       suggested->bottom - suggested->top,
                       SWP_NOZORDER | SWP_NOACTIVATE);

        monitorDpiScale    = static_cast<Float32>(HIWORD(wparam)) / 96.0f;
        geometryDpiScale   = monitorDpiScale * ui::userScale();
        styleDirty = true;
        return 0;
    }

    case WM_SYSCOMMAND:
        if((wparam & 0xfff0) == SC_KEYMENU)    // swallow the ALT menu
            return 0;
        break;

    case WM_ENDSESSION:
        // The session really is ending; we may be killed the moment we return.
        // Stop the motor now rather than hoping to reach the loop's exit.
        if(wparam)
            shutdownAppOnce();
        return 0;

    case WM_DESTROY:
        // Covers close-button / Alt+F4 / taskbar-close, including the case
        // where the loop never gets another turn.
        shutdownAppOnce();
        ::PostQuitMessage(0);
        return 0;
    }

    return ::DefWindowProcW(hwnd, msg, wparam, lparam);
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
Int32 APIENTRY WinMain(HINSTANCE hinstance, HINSTANCE, LPSTR, Int32)
{
    enableDpiAwareness();

    // Before any size is derived from it.
    loadUiScale();
    // Consumed, not acted on: the load IS the initial state.
    if(ui::consumeUserScaleChanged()) { }

    // Scale of the monitor the window will most likely open on.
    {
        HMONITOR mon = ::MonitorFromPoint(POINT{ 0, 0 }, MONITOR_DEFAULTTOPRIMARY);
        monitorDpiScale = ImGui_ImplWin32_GetDpiScaleForMonitor(mon);
        if(monitorDpiScale <= 0.0f)
            monitorDpiScale = 1.0f;
        geometryDpiScale = monitorDpiScale * ui::userScale();
    }

    WNDCLASSEXW wc;
    ZeroMemory(&wc, sizeof(wc));
    wc.cbSize        = sizeof(wc);
    wc.style         = CS_CLASSDC;
    wc.lpfnWndProc   = wndProc;
    wc.hInstance     = hinstance;
    wc.hCursor       = ::LoadCursor(nullptr, IDC_ARROW);

    // The same icon the executable carries, so the title bar, the taskbar and
    // Explorer all show one image. LoadIconW picks the size Windows asks for out
    // of the multi-size .ico; hIconSm is set explicitly because the class would
    // otherwise scale the large one down and it looks it.
    wc.hIcon         = static_cast<HICON>(::LoadImageW(hinstance,
                            MAKEINTRESOURCEW(IDI_APPICON), IMAGE_ICON,
                            ::GetSystemMetrics(SM_CXICON),
                            ::GetSystemMetrics(SM_CYICON), LR_DEFAULTCOLOR));
    wc.hIconSm       = static_cast<HICON>(::LoadImageW(hinstance,
                            MAKEINTRESOURCEW(IDI_APPICON), IMAGE_ICON,
                            ::GetSystemMetrics(SM_CXSMICON),
                            ::GetSystemMetrics(SM_CYSMICON), LR_DEFAULTCOLOR));
    wc.hbrBackground = nullptr;     // we paint every pixel ourselves
    wc.lpszClassName = L"RplidarC1Window";
    ::RegisterClassExW(&wc);

    // Logical size -> physical, then grow by the non-client frame, then clamp
    // to the work area: 1400x900 logical is 2450x1575 at 175% and would open
    // far off the bottom of a 1080p panel.
    const DWORD style = WS_OVERLAPPEDWINDOW;
    RECT rc = { 0, 0,
                static_cast<LONG>((DEFAULT_WIDTH  * monitorDpiScale + 0.5f)),
                static_cast<LONG>((DEFAULT_HEIGHT * monitorDpiScale + 0.5f)) };
    adjustRectForDpi(&rc, style, 0, static_cast<UINT>((monitorDpiScale * 96.0f + 0.5f)));

    LONG winW = rc.right - rc.left;
    LONG winH = rc.bottom - rc.top;
    clampToWorkArea(nullptr, &winW, &winH);

    // If we know the work area, centre in it so the whole window is on-screen;
    // otherwise let the shell cascade it.
    Int32 winX = CW_USEDEFAULT;
    Int32 winY = CW_USEDEFAULT;
    {
        RECT work;
        if(workAreaFor(nullptr, &work))
        {
            winX = static_cast<Int32>((work.left + ((work.right  - work.left) - winW) / 2));
            winY = static_cast<Int32>((work.top  + ((work.bottom - work.top)  - winH) / 2));
        }
    }

    HWND hwnd = ::CreateWindowExW(
        0, wc.lpszClassName, L"bibo", style,
        winX, winY, winW, winH,
        nullptr, nullptr, hinstance, nullptr);
    if(!hwnd)
    {
        ::UnregisterClassW(wc.lpszClassName, hinstance);
        return 1;
    }

    // Dark title bar and frame, matching the graphite casing the app paints.
    //
    // Stated explicitly rather than left alone: Windows picks the frame from the
    // SYSTEM theme, so on a light-mode machine this would come back white above
    // a dark UI. The window chrome is part of the app's look.
    {
        BOOL dark = TRUE;
        ::DwmSetWindowAttribute(hwnd, DWMWA_USE_IMMERSIVE_DARK_MODE, &dark, sizeof(dark));
    }

    // The real DPI of the monitor the window actually landed on.
    monitorDpiScale  = static_cast<Float32>(dpiForWindow(hwnd)) / 96.0f;
    geometryDpiScale = monitorDpiScale * ui::userScale();

    if(!createDeviceD3D(hwnd))
    {
        cleanupDeviceD3D();
        ::DestroyWindow(hwnd);
        ::UnregisterClassW(wc.lpszClassName, hinstance);
        ::MessageBoxW(nullptr, L"Failed to create a Direct3D 11 device.", L"bibo", MB_ICONERROR | MB_OK);
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Floating panels remember where the user put them, across runs. The file
    // lives with the rest of the per-user state, NOT next to the exe - a layout
    // that resets on every rebuild is not a layout.
    static Str iniPath = settings::path("layout.ini");
    io.IniFilename = iniPath.empty() ? nullptr : iniPath.c_str();

    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(d3dDevice, d3dContext);

    // Fonts are rasterised once, at this scale; remember it before anything
    // derives a ratio from it.
    ui::loadFonts(geometryDpiScale);
    fontDpiBase = geometryDpiScale;

    ui::applyStyle(geometryDpiScale);
    ui::setDpiScale(geometryDpiScale);
    ImGui::GetStyle().FontScaleDpi = 1.0f;   // baked size == requested size

    initFrameLimiter();

    ui::loadIcons(d3dDevice);

    // The 3D scene's own depth-buffered pass. Needs the same device and
    // context, and is torn down before them.
    scenegpu::init(d3dDevice, d3dContext);

    app::init(geometryDpiScale);
    appStarted = true;
    AppLifetimeGuard appGuard;              // Shutdown() on every return below

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
                done = true;
        }
        if(done)
            break;

        // Minimised or the screen is locked: don't burn a core spinning.
        if(occluded && swapchain->Present(0, DXGI_PRESENT_TEST) == DXGI_STATUS_OCCLUDED)
        {
            ::Sleep(10);
            continue;
        }
        occluded = false;

        if(resizeW != 0 && resizeH != 0)
        {
            const UINT rw = resizeW, rh = resizeH;
            resizeW = resizeH = 0;
            cleanupRenderTarget();
            if(SUCCEEDED(swapchain->ResizeBuffers(0, rw, rh, DXGI_FORMAT_UNKNOWN, 0)))
                createRenderTarget();
        }

        // A failed resize (or a lost device) leaves us without a target; retry
        // once per tick rather than drawing into nothing.
        if(rtv == nullptr)
        {
            createRenderTarget();
            if(rtv == nullptr)
            {
                ::Sleep(10);
                continue;
            }
        }

        // Between frames, never inside one: restyling mid-frame would leave
        // half the widgets laid out at the old size.
        if(ui::consumeUserScaleChanged())
        {
            geometryDpiScale = monitorDpiScale * ui::userScale();
            styleDirty       = true;
            saveUiScale();
        }

        if(styleDirty)
        {
            rescaleUiForDpi();
            styleDirty = false;
        }

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        app::frame();

        // Ctrl +/-/0, the shortcut every application with a zoom already uses.
        // Guarded on WantTextInput so it cannot fire while a port name or a file
        // path is being typed.
        if(io.KeyCtrl && !io.WantTextInput)
        {
            const Float32 cur = ui::userScale();
            if(ImGui::IsKeyPressed(ImGuiKey_Equal, false) ||
               ImGui::IsKeyPressed(ImGuiKey_KeypadAdd, false))
                ui::setUserScale(cur + ui::USER_SCALE_STEP);
            else if(ImGui::IsKeyPressed(ImGuiKey_Minus, false) ||
                    ImGui::IsKeyPressed(ImGuiKey_KeypadSubtract, false))
                ui::setUserScale(cur - ui::USER_SCALE_STEP);
            else if(ImGui::IsKeyPressed(ImGuiKey_0, false))
                ui::setUserScale(1.0f);
        }

        ImGui::Render();

        // Track ImGui's own window background, so restyling the UI can never
        // leave the backbuffer showing a different colour behind it.
        const ImVec4 bg = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
        const Float32 clear[4] = { bg.x, bg.y, bg.z, 1.0f };

        d3dContext->OMSetRenderTargets(1, &rtv, nullptr);
        d3dContext->ClearRenderTargetView(rtv, bg.w > 0.0f ? clear : CLEAR_COLOR);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

        HRESULT hr = swapchain->Present(1, 0);    // vsync
        occluded = (hr == DXGI_STATUS_OCCLUDED);

        // Vsync alone is the monitor's rate, not ours. See TARGET_FPS.
        waitForNextFrame();
    }

    shutdownAppOnce();      // usually already done by WM_DESTROY; idempotent

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    shutdownFrameLimiter();

    // Before the device: both of these hold textures created from it.
    scenegpu::shutdown();
    ui::releaseIcons();
    cleanupDeviceD3D();
    ::DestroyWindow(hwnd);
    ::UnregisterClassW(wc.lpszClassName, hinstance);
    return 0;
}
