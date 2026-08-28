// Throwaway preview harness for board::draw().
//
// A window, a dx11 device, and one full-window call into board_view.cxx. It
// exists so the diagram can be looked at before app_ui.cxx has a tab for it.
// Nothing here is part of the application.
//
//   build.bat        compile + link
//   build.bat run    compile + link + run
//
// Optional args:  --w N --h N   client size in physical px
//                 --frames N    render N frames then exit (0 = run until closed)
//
// The device/swapchain names here deliberately match hub/src/main.cxx's. This
// is the same boilerplate against the same API, and two spellings of `rtv`
// across two files that do the identical thing is how a reader starts believing
// they differ.

#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"

#include "board_view.hxx"
#include "shared.hxx"
#include "theme.hxx"

#include <d3d11.h>
#include <tchar.h>
#include <cstdlib>
#include <cstring>

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")

static ID3D11Device*           d3dDevice  = nullptr;
static ID3D11DeviceContext*    d3dContext = nullptr;
static IDXGISwapChain*         swapChain  = nullptr;
static ID3D11RenderTargetView* rtv        = nullptr;
static Bool                    wantResize = false;

static const Int32 DEFAULT_WIDTH  = 980;
static const Int32 DEFAULT_HEIGHT = 760;

// Below this a reported DPI is not a scale factor, it is a failed query.
static const Float32 MIN_SANE_DPI_SCALE = 0.5f;

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND, UINT, WPARAM, LPARAM);

static Void createRenderTarget()
{
    ID3D11Texture2D* back = nullptr;
    swapChain->GetBuffer(0, IID_PPV_ARGS(&back));
    if(back)
    {
        d3dDevice->CreateRenderTargetView(back, nullptr, &rtv);
        back->Release();
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
    sd.BufferCount       = 2;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferUsage       = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow      = hwnd;
    sd.SampleDesc.Count  = 1;
    sd.Windowed          = TRUE;
    sd.SwapEffect        = DXGI_SWAP_EFFECT_DISCARD;

    const D3D_FEATURE_LEVEL LEVELS[2] = { D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0 };
    D3D_FEATURE_LEVEL got = D3D_FEATURE_LEVEL_11_0;

    HRESULT hr = D3D11CreateDeviceAndSwapChain(
        nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, LEVELS, 2,
        D3D11_SDK_VERSION, &sd, &swapChain, &d3dDevice, &got, &d3dContext);

    // WARP is the software rasteriser. A preview harness that will not open on
    // a machine without a D3D11 GPU is a preview harness nobody can use.
    if(hr == DXGI_ERROR_UNSUPPORTED)
    {
        hr = D3D11CreateDeviceAndSwapChain(
            nullptr, D3D_DRIVER_TYPE_WARP, nullptr, 0, LEVELS, 2,
            D3D11_SDK_VERSION, &sd, &swapChain, &d3dDevice, &got, &d3dContext);
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
    if(swapChain)
    {
        swapChain->Release();
        swapChain = nullptr;
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

static LRESULT WINAPI wndProc(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam)
{
    if(ImGui_ImplWin32_WndProcHandler(hwnd, msg, wparam, lparam))
    {
        return 1;
    }

    switch(msg)
    {
    case WM_SIZE:
        if(wparam != SIZE_MINIMIZED)
        {
            wantResize = true;
        }
        return 0;
    case WM_SYSCOMMAND:
        if((wparam & 0xfff0) == SC_KEYMENU)
        {
            return 0;
        }
        break;
    case WM_DESTROY:
        ::PostQuitMessage(0);
        return 0;
    default:
        break;
    }
    return ::DefWindowProcW(hwnd, msg, wparam, lparam);
}

static Int32 argInt(Int32 argc, Char** argv, const Char* key, Int32 fallback)
{
    for(Int32 i = 1; i + 1 < argc; ++i)
    {
        if(std::strcmp(argv[i], key) == 0)
        {
            return std::atoi(argv[i + 1]);
        }
    }
    return fallback;
}

// Same shape as hub/src/main.cxx's enableDpiAwareness(), minus the 8.1 and Vista
// fallbacks: this is a bench tool for this machine, not a shipped binary.
typedef HANDLE DpiAwarenessContext;
typedef BOOL (WINAPI* PFN_SetProcessDpiAwarenessContext)(DpiAwarenessContext);
typedef UINT (WINAPI* PFN_GetDpiForWindow)(HWND);

// DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2
static const DpiAwarenessContext DPI_CONTEXT_PER_MONITOR_V2 =
    reinterpret_cast<DpiAwarenessContext>(static_cast<INT_PTR>(-4));

static Void enableDpiAwareness()
{
    HMODULE user32 = ::GetModuleHandleW(L"user32.dll");
    if(!user32)
    {
        return;
    }

    // reinterpret_cast, not a C-style cast: GetProcAddress hands back a FARPROC
    // and turning one function-pointer type into another is what it is for.
    PFN_SetProcessDpiAwarenessContext setCtx =
        reinterpret_cast<PFN_SetProcessDpiAwarenessContext>(
            ::GetProcAddress(user32, "SetProcessDpiAwarenessContext"));
    if(setCtx)
    {
        setCtx(DPI_CONTEXT_PER_MONITOR_V2);
    }
}

static Float32 dpiScaleFor(HWND hwnd)
{
    Float32 dpi = 1.0f;

    HMODULE user32 = ::GetModuleHandleW(L"user32.dll");
    if(user32)
    {
        PFN_GetDpiForWindow getDpi = reinterpret_cast<PFN_GetDpiForWindow>(
            ::GetProcAddress(user32, "GetDpiForWindow"));
        if(getDpi)
        {
            dpi = static_cast<Float32>(getDpi(hwnd)) / 96.0f;
        }
    }

    return dpi < MIN_SANE_DPI_SCALE ? 1.0f : dpi;
}

Int32 main(Int32 argc, Char** argv)
{
    const Int32 wantW = argInt(argc, argv, "--w", DEFAULT_WIDTH);
    const Int32 wantH = argInt(argc, argv, "--h", DEFAULT_HEIGHT);
    Int32 framesLeft  = argInt(argc, argv, "--frames", 0);

    enableDpiAwareness();

    WNDCLASSEXW wc = { sizeof(wc), CS_CLASSDC, wndProc, 0, 0,
                       ::GetModuleHandleW(nullptr), nullptr, nullptr, nullptr,
                       nullptr, L"BoardPreview", nullptr };
    ::RegisterClassExW(&wc);

    RECT r = { 0, 0, wantW, wantH };
    ::AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    HWND hwnd = ::CreateWindowW(wc.lpszClassName, L"board_view preview",
                                WS_OVERLAPPEDWINDOW, 80, 40,
                                r.right - r.left, r.bottom - r.top,
                                nullptr, nullptr, wc.hInstance, nullptr);
    if(!hwnd)
    {
        return 1;
    }

    if(!createDeviceD3D(hwnd))
    {
        cleanupDeviceD3D();
        ::UnregisterClassW(wc.lpszClassName, wc.hInstance);
        return 1;
    }

    ::ShowWindow(hwnd, SW_SHOWDEFAULT);
    ::UpdateWindow(hwnd);

    const Float32 dpi = dpiScaleFor(hwnd);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;

    ui::loadFonts(dpi);
    ui::applyStyle(dpi);
    ui::setDpiScale(dpi);

    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(d3dDevice, d3dContext);

    Bool running = true;
    while(running)
    {
        MSG msg;
        while(::PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE))
        {
            ::TranslateMessage(&msg);
            ::DispatchMessage(&msg);
            if(msg.message == WM_QUIT)
            {
                running = false;
            }
        }
        if(!running)
        {
            break;
        }

        if(wantResize)
        {
            wantResize = false;
            cleanupRenderTarget();
            swapChain->ResizeBuffers(0, 0, 0, DXGI_FORMAT_UNKNOWN, 0);
            createRenderTarget();
        }

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(vp->WorkPos);
        ImGui::SetNextWindowSize(vp->WorkSize);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::Begin("##preview", nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoBringToFrontOnFocus |
                     ImGuiWindowFlags_NoSavedSettings);

        // A default-constructed Live is the "nothing is connected" state, which
        // is the right thing for a preview: it renders the board and the pin
        // map without inventing any telemetry.
        const board::Live live;
        board::draw(board::Which::WHICH_PICO2_W,
                    ImGui::GetContentRegionAvail(),
                    live);

        ImGui::End();
        ImGui::PopStyleVar();

        ImGui::Render();
        const Float32 CLEAR[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
        d3dContext->OMSetRenderTargets(1, &rtv, nullptr);
        d3dContext->ClearRenderTargetView(rtv, CLEAR);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
        swapChain->Present(1, 0);

        if(framesLeft > 0 && --framesLeft == 0)
        {
            running = false;
        }
    }

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
    cleanupDeviceD3D();
    ::DestroyWindow(hwnd);
    ::UnregisterClassW(wc.lpszClassName, wc.hInstance);
    return 0;
}
