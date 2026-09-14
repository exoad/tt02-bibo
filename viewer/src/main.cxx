// bibo viewer: a Win32 window, a D3D11 device and Dear ImGui's frame loop. This
// file owns the platform and the floating windows; the 3D view is scene.cxx,
// drawn through ImGui's own draw list.
#include "shared.hxx"

#include <cstdio>
#include <cmath>
#include <windows.h>
#include <dwmapi.h>
#include <d3d11.h>

// <rpcndr.h>, dragged in above, defines `small` as char, with near and far from
// the same era. Undefined before any header of ours.
#undef small
#undef near
#undef far

// <winnt.h> defines SEVERITY_SUCCESS and SEVERITY_ERROR as macros, and
// bibowire::Severity has a SEVERITY_ERROR member, which would become
// Severity::1 and break the protocol header. Undefined at the boundary.
#undef SEVERITY_SUCCESS
#undef SEVERITY_ERROR

#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"

#include "scene.hxx"
#include "link.hxx"
#include "camera.hxx"
#include "trim.hxx"
#include "drive.hxx"
#include "bundle.hxx"
#include "settings.hxx"
#include "vlog.hxx"

#ifndef DWMWA_USE_IMMERSIVE_DARK_MODE
#define DWMWA_USE_IMMERSIVE_DARK_MODE 20
#endif

// Logical (96 dpi) pixels; scaled by the monitor's DPI before use.
static constexpr Int32 DEFAULT_WIDTH = 1400;
static constexpr Int32 DEFAULT_HEIGHT = 900;

// Darker than ImGui's WindowBg, so panels floating over the view read as panels.
static constexpr Array<Float32, 4> CLEAR_COLOR = { 0.086f, 0.094f, 0.110f, 1.0f };

// A NAME, not an address: the field hotspot's DHCP hands out a different
// address every outing, so the client resolves the name on every attempt.
// `bibobox` is Tailscale MagicDNS; `bibobox.local` is mDNS, which this laptop
// cannot resolve (getaddrinfo 11001). The field is editable for other networks.
static constexpr CharSeq DEFAULT_HOST = "bibobox";

// From bibowire rather than retyped, so the viewer cannot drift to another port.
static constexpr Int32 DEFAULT_PORT = static_cast<Int32>(bibowire::PORT);

// Where a readout's value starts, in logical pixels from the window's left.
static constexpr Float32 READOUT_COLUMN = 92.0f;

static ID3D11Device* d3dDevice = nullptr;
static ID3D11DeviceContext* d3dContext = nullptr;
static IDXGISwapChain* swapchain = nullptr;
static ID3D11RenderTargetView* rtv = nullptr;
static UINT resizeW = 0;
static UINT resizeH = 0;

// Monitor DPI as a multiplier; everything laid out in logical pixels uses it.
static Float32 uiScale = 1.0f;

// What a person typed, plus the client. The client runs its own thread and
// hands over copies of its decoded state, so the frame loop never waits on a
// socket.
struct Link
{
    Array<Char, 64> host = {};
    Int32 port = DEFAULT_PORT;
    link::Client client;
};

static Void initLink(Link& lk)
{
    // snprintf rather than a brace list of characters (docs/conventions.md).
    std::snprintf(lk.host.data(), lk.host.size(), "%s", DEFAULT_HOST);
}

// Integer digits, never "%f" (the locale trap, vlog.hxx).
static Str metresText(UInt32 mm)
{
    Array<Char, 32> t = {};
    std::snprintf(t.data(), t.size(), "%u.%03u m", mm / 1000u, mm % 1000u);
    return Str(t.data());
}

static Str voltsText(UInt16 milliV)
{
    Array<Char, 32> t = {};
    const UInt32 mv = milliV;
    std::snprintf(t.data(), t.size(), "%u.%03u V", mv / 1000u, mv % 1000u);
    return Str(t.data());
}

static Str milliText(Int32 v)
{
    Array<Char, 32> t = {};
    std::snprintf(t.data(), t.size(), "%+d / 1000", v);
    return Str(t.data());
}

static Str msText(Int64 ms)
{
    Array<Char, 32> t = {};
    std::snprintf(t.data(), t.size(), "%lld ms", ms);
    return Str(t.data());
}

static CharSeq severityMark(bibowire::Severity s)
{
    if(s == bibowire::Severity::SEVERITY_ERROR)
    {
        return "[error]";
    }
    if(s == bibowire::Severity::SEVERITY_WARN)
    {
        return "[warn] ";
    }
    return "[info] ";
}

// 0 live, 1 soft, 2 dead, 3 estop latched - named by bibowire, so the viewer
// and the board cannot disagree about what a 2 means.
static CharSeq deadmanText(UInt8 v)
{
    if(v > static_cast<UInt8>(bibowire::deadman::State::STATE_ESTOP))
    {
        return "--";
    }
    return bibowire::deadman::stateName(static_cast<bibowire::deadman::State>(v));
}

static CharSeq picoText(UInt8 v)
{
    if(v == 0u)
    {
        return "down";
    }
    if(v == 1u)
    {
        return "up";
    }
    if(v == 2u)
    {
        return "up but silent";
    }
    return "--";
}

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
    // WARP, so the viewer still opens over RDP or without a usable 3D driver.
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
        // Queued, not applied: resizing the swap chain inside the message pump
        // is unsafe, and a zero extent must never reach ResizeBuffers.
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

static Void readout(CharSeq label, CharSeq value)
{
    ImGui::TextUnformatted(label);
    ImGui::SameLine(READOUT_COLUMN * uiScale);
    ImGui::TextUnformatted(value);
}

static Void readoutStr(CharSeq label, const Str& value)
{
    readout(label, value.c_str());
}

static Void drawConnectionWindow(Link& lk, const link::Snapshot& snap, Int64 nowMs)
{
    ImGui::SetNextWindowPos(ImVec2(16.0f * uiScale, 16.0f * uiScale), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(340.0f * uiScale, 0.0f), ImGuiCond_FirstUseEver);
    if(!ImGui::Begin("Connection"))
    {
        ImGui::End();
        return;
    }
    const Bool up = link::isOpen(lk.client);
    ImGui::BeginDisabled(up);
    ImGui::SetNextItemWidth(-70.0f * uiScale);
    ImGui::InputText("host", lk.host.data(), lk.host.size());
    ImGui::SetNextItemWidth(-70.0f * uiScale);
    ImGui::InputInt("port", &lk.port);
    ImGui::EndDisabled();
    ImGui::BeginDisabled(up);
    if(ImGui::Button("Connect"))
    {
        if(lk.port < 1 || lk.port > 65535)
        {
            lk.port = DEFAULT_PORT;
        }
        link::open(lk.client, lk.host.data(), static_cast<UInt16>(lk.port));
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!up);
    if(ImGui::Button("Disconnect"))
    {
        link::close(lk.client);
    }
    ImGui::EndDisabled();
    ImGui::Separator();
    // One sentence: connecting, handshaking, live, retrying in N ms, or the
    // reason the board gave for saying BYE.
    ImGui::TextWrapped("%s", snap.status.c_str());
    const Opt<link::Revolution> rev = snap.state.revolution(nowMs);
    if(rev.has_value())
    {
        const Str age = msText(rev->ageMs) + (rev->stale ? " - STALE" : "");
        readoutStr("scan", age);
    }
    else if(snap.state.haveScan)
    {
        // Possibly over a healthy link: a healthy link must never make a dead
        // sensor look alive.
        readout("scan", "too old - not drawn");
    }
    else
    {
        readout("scan", "--");
    }
    // The sender's own count of what it dropped for this client, so a thin
    // stream reads as dropped rather than as a slow lidar.
    if(rev.has_value() && rev->droppedSinceLast > 0u)
    {
        Array<Char, 48> t = {};
        const UInt32 dropped = rev->droppedSinceLast;
        std::snprintf(t.data(), t.size(), "%u since last", dropped);
        readout("dropped", t.data());
    }
    // The network's number, a different question from the ages above: an 8 ms
    // round trip can carry a two-second-old picture. Current and the minimum of
    // the last 16, never the mean, which on a hotspot tracks the worst moment
    // rather than the path.
    const Opt<Int64> rtt = snap.state.rttMs();
    const Opt<Int64> bestRtt = snap.state.bestRttMs();
    if(rtt.has_value() && bestRtt.has_value())
    {
        Array<Char, 64> t = {};
        std::snprintf(t.data(), t.size(), "%lld ms  (min 16: %lld)", *rtt, *bestRtt);
        readout("round trip", t.data());
    }
    else
    {
        readout("round trip", "--");
    }
    // The board's own scan age, apart from the network number. It rides
    // CTLSTATE on UDP, so for an observer it may never arrive.
    const Opt<link::Control> ctlAge = snap.state.controlState(nowMs);
    if(ctlAge.has_value())
    {
        readoutStr("scan age (board)", msText(static_cast<Int64>(ctlAge->state.scanAgeMs)));
    }
    if(snap.state.haveLidar)
    {
        Array<Char, 48> fw = {};
        std::snprintf(
            fw.data(),
            fw.size(),
            "model %u fw %u.%u",
            static_cast<UInt32>(snap.state.lidar.model),
            static_cast<UInt32>(snap.state.lidar.fwMajor),
            static_cast<UInt32>(snap.state.lidar.fwMinor)
        );
        readout("lidar", fw.data());
    }
    else
    {
        readout("lidar", "--");
    }
    // Counted, never smoothed, and shown.
    if(snap.state.missedRevs > 0u && !snap.state.gapText.empty())
    {
        readoutStr("missed", snap.state.gapText);
    }
    if(snap.state.resyncBytes > 0u || snap.state.refusedFrames > 0u)
    {
        Array<Char, 64> junk = {};
        std::snprintf(
            junk.data(),
            junk.size(),
            "%u resync, %u refused",
            snap.state.resyncBytes,
            snap.state.refusedFrames
        );
        readout("junk", junk.data());
    }
    // The board's sentences for a person (lidar::reason(), carlink::detail()),
    // verbatim: they are what makes a fault diagnosable.
    if(!snap.state.notes.empty())
    {
        ImGui::Separator();
        const Size have = snap.state.notes.size();
        const Size from = have > 6 ? (have - 6) : 0;
        for(Size i = from; i < have; ++i)
        {
            const link::Note& note = snap.state.notes[i];
            ImGui::TextWrapped("%s %s", severityMark(note.severity), note.text.c_str());
        }
    }
    ImGui::End();
}

static Void drawViewWindow(scene::Scene& sc, camview::View& cam, trimview::View& trim, driveview::View& drive, bundleview::View& bundles)
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
    ImGui::Checkbox("heading", &sc.opt.heading);
    if(ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("solid: where the wheels are - ghost: where asked");
    }
    ImGui::Separator();
    // Not a view option: this box and the Camera window's X set the same flag,
    // which opens the window AND subscribes.
    ImGui::Checkbox("camera", &cam.open);
    if(ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("streams about 1 MB/s while open");
    }
    ImGui::Checkbox("trim", &trim.open);
    // Only puts the pane on screen; it is not one of drive.hxx's three gates.
    ImGui::Checkbox("drive", &drive.open);
    ImGui::Checkbox("bundles", &bundles.open);
    if(ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("what the board can run, and what is loaded");
    }
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

static Void drawCarWindow(const link::Snapshot& snap, Int64 nowMs)
{
    ImGui::SetNextWindowPos(ImVec2(16.0f * uiScale, 424.0f * uiScale), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(340.0f * uiScale, 0.0f), ImGuiCond_FirstUseEver);
    if(!ImGui::Begin("Car"))
    {
        ImGui::End();
        return;
    }
    // "--" for anything never reported. mode, clearance, steer and throttle come
    // from DECIDE (what the pilot CHOSE); armed comes from CTLSTATE (what the
    // board is DOING). A car that is commanded and not armed does not move.
    const Opt<link::Decision> dec = snap.state.decision(nowMs);
    const Opt<link::Board> brd = snap.state.boardState(nowMs);
    const Opt<link::Control> ctl = snap.state.controlState(nowMs);
    if(dec.has_value())
    {
        const Str mode = Str(bibowire::driveModeName(dec->decide.mode)) + " by "
                         + bibowire::pilotModeName(dec->decide.source);
        readoutStr("mode", dec->stale ? (mode + " (stale)") : mode);
        readoutStr("clearance", metresText(dec->decide.clearanceMm));
        readoutStr("steer", milliText(dec->decide.steerMilli));
        readoutStr("throttle", milliText(dec->decide.throttleMilli));
    }
    else
    {
        readout("mode", "--");
        readout("clearance", "--");
        readout("steer", "--");
        readout("throttle", "--");
    }
    // CTLSTATE first. It rides UDP, so where UDP is blocked BOARD's picoArmed
    // answers instead, with 2 meaning unknown, not no.
    if(ctl.has_value())
    {
        readout("armed", ctl->state.armed != 0u ? "yes" : "no");
        readout("refusing", bibowire::refuseName(ctl->state.refuse));
        // CONTROL_AGE_NEVER means never applied, not a long time ago.
        if(ctl->state.controlAgeMs == bibowire::CONTROL_AGE_NEVER)
        {
            readout("control age", "never");
        }
        else
        {
            readoutStr("control age", msText(static_cast<Int64>(ctl->state.controlAgeMs)));
        }
    }
    else if(brd.has_value() && brd->state.picoArmed <= 1u)
    {
        readout("armed", brd->state.picoArmed != 0u ? "yes" : "no");
    }
    else
    {
        readout("armed", "--");
    }
    if(brd.has_value())
    {
        readout("deadman", deadmanText(brd->state.deadman));
        readout("pico", picoText(brd->state.picoLink));
        // PICO_SILENT_ABSENT means no link at all, not a quiet one.
        if(brd->state.picoSilentMs == bibowire::PICO_SILENT_ABSENT)
        {
            readout("pico silent", "no link");
        }
        else
        {
            readoutStr("pico silent", msText(static_cast<Int64>(brd->state.picoSilentMs)));
        }
        // BATT_ABSENT is not measured; 0 is a real reading of a dead pack, which
        // is why the sentinel is not 0.
        if(brd->state.battMilliV == bibowire::BATT_ABSENT)
        {
            readout("battery", "not measured");
        }
        else
        {
            readoutStr("battery", voltsText(brd->state.battMilliV));
        }
    }
    else
    {
        readout("deadman", "--");
        readout("pico", "--");
        readout("battery", "--");
    }
    ImGui::End();
}

// The mouse moves the view only when no ImGui window wants it, or dragging a
// slider would orbit the world behind it.
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

Int32 APIENTRY WinMain(HINSTANCE hinstance, HINSTANCE, LPSTR, Int32)
{
    // First, before anything that can fail: with no console, the log is the only
    // place a failed start can say why.
    vlog::open();
    vlog::line("bibo viewer starting - built %s %s", __DATE__, __TIME__);
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
    // lround, not a cast of x + 0.5f, which rounds negatives the wrong way.
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
        vlog::line(
            "no window: CreateWindowExW error %u - exiting",
            static_cast<UInt32>(::GetLastError())
        );
        vlog::close();
        ::UnregisterClassW(wc.lpszClassName, hinstance);
        return 1;
    }
    // Explicit, because Windows follows the SYSTEM theme and would draw a white
    // title bar above the dark UI on a light-mode machine.
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
        vlog::line("no Direct3D 11 device - exiting");
        vlog::close();
        return 1;
    }
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    // No docking: it is not in this Dear ImGui branch, and the panels float.
    ImGui::StyleColorsDark();
    ImGui::GetStyle().WindowRounding = 4.0f;
    ImGui::GetStyle().FrameRounding = 3.0f;
    // ScaleAllSizes scales every padding and thickness; FontScaleDpi re-rasterises
    // the font. Both, or text and widgets end up sized for different DPIs.
    ImGui::GetStyle().ScaleAllSizes(uiScale);
    ImGui::GetStyle().FontScaleDpi = uiScale;
    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(d3dDevice, d3dContext);
    scene::Scene sc;
    scene::resetCamera(sc.cam);
    // Closed at startup: open costs the board about a megabyte a second.
    camview::View cam;
    camview::init(d3dDevice, d3dContext, uiScale);
    // Closed at startup. Trim and Drive own no device resource, so neither has a
    // shutdown.
    trimview::View trim;
    trimview::init(uiScale);
    driveview::View drive;
    driveview::init(uiScale);
    // Open at startup, since the operator connects and then presses ARM here.
    // Opening moves nothing: the enable is off and the car disarmed until ARM
    // is pressed and confirmed.
    drive.open = true;
    // The master window, open at startup like Drive: it is how the operator
    // chooses what the car does. Its two known bundles' windows ARE the Trim
    // and Drive panes, so it borrows both.
    bundleview::View bundles;
    bundleview::init(uiScale);
    bundles.trim = &trim;
    bundles.drive = &drive;
    bundles.open = true;
    // Last run's numbers into the panes before the first frame. Nothing is sent
    // to the car: the board keeps its own saved copy, and once connected it
    // replaces these in the sliders (trimview::follow).
    const Str settingsPath = settings::defaultPath();
    {
        settings::Values loaded = settings::capture(trim, drive);
        if(settings::load(settingsPath, loaded).has_value())
        {
            settings::apply(loaded, trim, drive);
        }
        else
        {
            // The legacy file beside bibo.exe, which build.bat clean deletes:
            // read once and written to the new path so this branch does not run
            // again. The old file is left alone.
            const Str oldPath = settings::legacyPath();
            if(!oldPath.empty() && oldPath != settingsPath && settings::load(oldPath, loaded).has_value())
            {
                settings::apply(loaded, trim, drive);
                settings::save(settingsPath, settings::capture(trim, drive));
            }
        }
    }
    // What this run believes the file holds, compared once a frame below.
    settings::Values savedSettings = settings::capture(trim, drive);
    // `net`, not `link`: a variable named `link` would hide the namespace.
    Link net;
    initLink(net);
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
        // NOTHING when the newest revolution is too old: the staleness test
        // lives inside revolution(), so no branch here can forget it.
        const Int64 nowMs = link::monoMs();
        const link::Snapshot snap = link::snapshot(net.client);
        const Opt<link::Revolution> rev = snap.state.revolution(nowMs);
        if(rev.has_value())
        {
            sc.cloud = rev->cloud;
            sc.cloudStale = rev->stale;
        }
        else
        {
            sc.cloud.clear();
            sc.cloudStale = false;
        }
        // From accessors with their own staleness test. Absent stays absent:
        // centre is not a fallback.
        const Opt<link::Control> steerCtl = snap.state.controlState(nowMs);
        const Opt<link::Decision> steerDec = snap.state.decision(nowMs);
        sc.haveSteerNow = steerCtl.has_value();
        sc.haveSteerWant = steerDec.has_value();
        sc.steerNow = steerCtl.has_value()
            ? static_cast<Float32>(steerCtl->state.steerNowMilli) / 1000.0f
            : 0.0f;
        sc.steerWant = steerDec.has_value()
            ? static_cast<Float32>(steerDec->decide.steerMilli) / 1000.0f
            : 0.0f;
        handleCameraInput(sc.cam);
        // Into the BACKGROUND list, behind every ImGui window.
        const ImGuiViewport* area = ImGui::GetMainViewport();
        const scene::Viewport where = { area->WorkPos, area->WorkSize };
        scene::draw(ImGui::GetBackgroundDrawList(), where, sc);
        drawConnectionWindow(net, snap, nowMs);
        drawViewWindow(sc, cam, trim, drive, bundles);
        drawCarWindow(snap, nowMs);
        // Before the Trim and Drive panes, so a pane a load just opened draws
        // this frame rather than next.
        bundleview::drawWindow(bundles, net.client, snap, nowMs);
        camview::drawWindow(cam, net.client, snap, nowMs);
        // Before the sliders draw and before the settings check below.
        trimview::follow(trim, snap);
        trimview::drawWindow(trim, net.client, snap, nowMs);
        // LAST, and every frame whether or not its window is open: this publishes
        // the control intent, and a skipped frame would leave the worker sending
        // the last one - a key still held down as far as the car knows.
        drive.idleTest = trim.idleTest;
        drive.reverseOff = trim.escReverseUs >= static_cast<Int32>(bibowire::ESC_NEUTRAL_US);
        driveview::drawWindow(drive, net.client, snap, nowMs);
        // Saved on a change, never per frame, and never while a widget is active:
        // the release is the moment the operator meant.
        if(!ImGui::IsAnyItemActive())
        {
            const settings::Values current = settings::capture(trim, drive);
            if(current != savedSettings)
            {
                // Recorded as saved even when the write failed, so an unwritable
                // folder logs once per change rather than every frame.
                settings::save(settingsPath, current);
                savedSettings = current;
            }
        }
        ImGui::Render();
        d3dContext->OMSetRenderTargets(1, &rtv, nullptr);
        d3dContext->ClearRenderTargetView(rtv, CLEAR_COLOR.data());
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
        swapchain->Present(1, 0);       // vsync
    }
    // Once more on the way out: a value still being dragged at close never
    // reached the per-frame check.
    settings::save(settingsPath, settings::capture(trim, drive));
    // Joined before anything is torn down: exiting through a thread sitting in
    // recv() crashes.
    link::close(net.client);
    // Before the device goes: releasing a texture after its device is destroyed
    // crashes on exit.
    camview::shutdown(cam);
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
    cleanupDeviceD3D();
    ::DestroyWindow(hwnd);
    ::UnregisterClassW(wc.lpszClassName, hinstance);
    // Last, after the link is joined, so the worker's closing lines are in the file.
    vlog::line("viewer exiting");
    vlog::close();
    return 0;
}
