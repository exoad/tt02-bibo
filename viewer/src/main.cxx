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

// <winnt.h> spells the HRESULT severity bits SEVERITY_SUCCESS and
// SEVERITY_ERROR, as macros, and bibowire::Severity has a member by the second
// of those names - so `Severity::SEVERITY_ERROR` becomes `Severity::1` and the
// protocol header stops parsing three hundred lines before anything of ours is
// read. Undefined here for the same reason and in the same place as the three
// above: a Windows macro that collides with a name of ours is killed at the
// boundary, not worked around at every use.
#undef SEVERITY_SUCCESS
#undef SEVERITY_ERROR

#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"

#include "scene.hxx"
#include "link.hxx"

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

// From the protocol's own header rather than typed again here: the port is a
// fact about bibowire, and a viewer carrying its own copy of it is a viewer that
// can be pointed at the wrong one by a one-character edit nobody links to the
// board.
static constexpr Int32 DEFAULT_PORT = static_cast<Int32>(bibowire::PORT);

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
// THE NETWORK.
//
// What a person typed, plus the client that turns it into a connection. The
// client runs its own thread and hands over copies of its decoded state; this
// file never names a socket and the frame loop never waits for one.
// ---------------------------------------------------------------------------
struct Link
{
    Array<Char, 64> host = {};
    Int32 port = DEFAULT_PORT;
    link::Client client;
};

static Void initLink(Link& lk)
{
    // snprintf rather than a brace-initialised literal: docs/conventions.md is
    // explicit that a long literal in an Array<Char, N> is better written as
    // the format call than as a list of character constants.
    std::snprintf(lk.host.data(), lk.host.size(), "%s", DEFAULT_HOST);
}

// ---------------------------------------------------------------------------
// Numbers, rendered.
//
// Every one of these is integer arithmetic. printf's "%.3f" honours the locale,
// a machine set to a comma decimal writes "3,410", and that is the bug this
// project already met twice - so the decimal point in this program is a
// character somebody wrote, not one a locale chose.
// ---------------------------------------------------------------------------
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

// 0 live, 1 soft, 2 dead, 3 estop latched - named by the protocol module, so the
// viewer and the board cannot disagree about what a 2 means.
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

    // A HOSTNAME, not an address. The field network is a phone hotspot and its
    // DHCP hands out a different address every time; the board answers to
    // bibobox.local over mDNS and that is the thing that stays true. The client
    // resolves it on EVERY attempt for the same reason.
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

    // The truth, in a sentence: connecting, handshaking, live, retrying in N ms,
    // or the reason the board gave for saying BYE.
    ImGui::TextWrapped("%s", snap.status.c_str());

    const Opt<link::Revolution> rev = snap.state.revolution(nowMs);
    if(rev.has_value())
    {
        const Str age = msText(rev->ageMs) + (rev->stale ? " - STALE" : "");
        readoutStr("scan", age);
    }
    else if(snap.state.haveScan)
    {
        // The link can be perfectly healthy while this says so. That is the
        // point: a link that is healthy must never make a dead sensor look
        // alive.
        readout("scan", "too old - not drawn");
    }
    else
    {
        readout("scan", "--");
    }

    // The sender's own count of what it threw away for this client, so a thin
    // stream reads as dropped rather than as a slow lidar.
    if(rev.has_value() && rev->droppedSinceLast > 0u)
    {
        Array<Char, 48> t = {};
        const UInt32 dropped = rev->droppedSinceLast;
        std::snprintf(t.data(), t.size(), "%u since last", dropped);
        readout("dropped", t.data());
    }

    // THE NETWORK'S NUMBER, and it answers a different question from the ages
    // above: the round trip can be 8 ms while the picture behind it is two
    // seconds old. Current, and the minimum of the last 16 - never the mean,
    // which on a hotspot measures the worst moment of the last sixteen seconds
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

    // THE BOARD MEASURING ITSELF, beside the value it describes rather than
    // lumped in with the network number. This is the second lie: a link that is
    // perfectly healthy with dead data behind it. It rides CTLSTATE on UDP, so
    // for an observer it may never arrive at all - and then it says so.
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

    // Counted, never smoothed - and shown, because a count nobody can read off
    // the running system is the same species of bug as a test that measures
    // nothing.
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

    // The prose channel. Every lidar::reason() and carlink::detail() the board
    // writes for a person arrives here verbatim, and a binary protocol that
    // dropped these would lose the one thing that makes a fault diagnosable.
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

static Void drawCarWindow(const link::Snapshot& snap, Int64 nowMs)
{
    ImGui::SetNextWindowPos(ImVec2(16.0f * uiScale, 424.0f * uiScale), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(340.0f * uiScale, 0.0f), ImGuiCond_FirstUseEver);
    if(!ImGui::Begin("Car"))
    {
        ImGui::End();
        return;
    }

    // "--" is the honest reading for a value this program has never been told,
    // and every row below can still say it. mode, clearance, steer and throttle
    // come from DECIDE - what the pilot CHOSE - and armed comes from CTLSTATE,
    // which is what the board is actually DOING. Those are two different
    // questions and the panel keeps them side by side for that reason: a car
    // that is commanded and not armed does not move.
    const Opt<link::Decision> dec = snap.state.decision(nowMs);
    const Opt<link::Board> brd = snap.state.boardState(nowMs);
    const Opt<link::Control> ctl = snap.state.controlState(nowMs);

    if(dec.has_value())
    {
        const Str mode = Str(link::modeName(dec->decide.mode)) + " by "
                         + link::sourceName(dec->decide.source);
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

    // CTLSTATE first, because it is the board reporting what it DID. It rides
    // UDP at 20 Hz, so on a network that blocks UDP it never arrives at all -
    // and then BOARD's own picoArmed answers, with 2 meaning unknown rather
    // than meaning no.
    if(ctl.has_value())
    {
        readout("armed", ctl->state.armed != 0u ? "yes" : "no");
        readout("refusing", bibowire::refuseName(ctl->state.refuse));
        // The board's own age for the control it last APPLIED. 0xFFFFFFFF is
        // "never", which is not the same fact as "a long time ago".
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
        // Another of the board's own ages, beside the thing it describes. The
        // sentinel means there is no link at all, which reads differently from
        // a link that has simply been quiet for 900 ms.
        if(brd->state.picoSilentMs == bibowire::PICO_SILENT_ABSENT)
        {
            readout("pico silent", "no link");
        }
        else
        {
            readoutStr("pico silent", msText(static_cast<Int64>(brd->state.picoSilentMs)));
        }
        // 0xFFFF is NOT MEASURED, and 0 is a real reading of a dead pack - which
        // is exactly why the sentinel is not 0 and why this is not an if(mv).
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

    // `net`, not `link`: the module is namespace `link`, and a variable of that
    // name would hide it for the rest of the function.
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

        // THE SCAN. Whatever the board has sent that is still true - and NOTHING
        // when the newest revolution is too old to draw, which is what an empty
        // Opt from revolution() means. There is no branch here that can forget
        // to check the age, because when it is too old there is no value to
        // draw: the staleness test lives inside the accessor.
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

        handleCameraInput(sc.cam);

        // The view fills the window and the panels float over it, so it draws
        // into the BACKGROUND list - behind every ImGui window, whatever order
        // they were submitted in.
        const ImGuiViewport* area = ImGui::GetMainViewport();
        const scene::Viewport where = { area->WorkPos, area->WorkSize };
        scene::draw(ImGui::GetBackgroundDrawList(), where, sc);

        drawConnectionWindow(net, snap, nowMs);
        drawViewWindow(sc);
        drawCarWindow(snap, nowMs);

        ImGui::Render();

        d3dContext->OMSetRenderTargets(1, &rtv, nullptr);
        d3dContext->ClearRenderTargetView(rtv, CLEAR_COLOR.data());
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

        swapchain->Present(1, 0);       // vsync
    }

    // Joined before anything else is torn down: the worker owns a socket and a
    // thread, and a process that exits through a thread sitting in recv() is a
    // crash report nobody can read.
    link::close(net.client);

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    cleanupDeviceD3D();
    ::DestroyWindow(hwnd);
    ::UnregisterClassW(wc.lpszClassName, hinstance);
    return 0;
}
