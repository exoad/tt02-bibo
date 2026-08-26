// Throwaway preview harness for board::Draw().
//
// A window, a dx11 device, and one full-window call into board_view.cpp. It
// exists so the diagram can be looked at before app_ui.cpp has a tab for it.
// Nothing here is part of the application.
//
//   build.bat        compile + link
//   build.bat run    compile + link + run
//
// Optional args:  --w N --h N   client size in physical px
//                 --frames N    render N frames then exit (0 = run until closed)

#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"

#include "board_view.hpp"
#include "theme.hpp"

#include <d3d11.h>
#include <tchar.h>
#include <cstdlib>
#include <cstring>

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")

static ID3D11Device*           g_device    = nullptr;
static ID3D11DeviceContext*    g_context   = nullptr;
static IDXGISwapChain*         g_swapchain = nullptr;
static ID3D11RenderTargetView* g_rtv       = nullptr;
static bool                    g_resize    = false;

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND, UINT, WPARAM, LPARAM);

static void CreateRenderTarget()
{
    ID3D11Texture2D* back = nullptr;
    g_swapchain->GetBuffer(0, IID_PPV_ARGS(&back));
    if (back)
    {
        g_device->CreateRenderTargetView(back, nullptr, &g_rtv);
        back->Release();
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
    sd.BufferCount       = 2;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferUsage       = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow      = hwnd;
    sd.SampleDesc.Count  = 1;
    sd.Windowed          = TRUE;
    sd.SwapEffect        = DXGI_SWAP_EFFECT_DISCARD;

    const D3D_FEATURE_LEVEL levels[2] = { D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0 };
    D3D_FEATURE_LEVEL got = D3D_FEATURE_LEVEL_11_0;

    HRESULT hr = D3D11CreateDeviceAndSwapChain(
        nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, levels, 2,
        D3D11_SDK_VERSION, &sd, &g_swapchain, &g_device, &got, &g_context);
    if (hr == DXGI_ERROR_UNSUPPORTED)
        hr = D3D11CreateDeviceAndSwapChain(
            nullptr, D3D_DRIVER_TYPE_WARP, nullptr, 0, levels, 2,
            D3D11_SDK_VERSION, &sd, &g_swapchain, &g_device, &got, &g_context);
    if (FAILED(hr)) return false;

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

static LRESULT WINAPI WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp)
{
    if (ImGui_ImplWin32_WndProcHandler(hwnd, msg, wp, lp)) return 1;

    switch (msg)
    {
    case WM_SIZE:
        if (wp != SIZE_MINIMIZED) g_resize = true;
        return 0;
    case WM_SYSCOMMAND:
        if ((wp & 0xfff0) == SC_KEYMENU) return 0;
        break;
    case WM_DESTROY:
        ::PostQuitMessage(0);
        return 0;
    default:
        break;
    }
    return ::DefWindowProcW(hwnd, msg, wp, lp);
}

static int ArgInt(int argc, char** argv, const char* key, int def)
{
    for (int i = 1; i + 1 < argc; ++i)
        if (std::strcmp(argv[i], key) == 0) return std::atoi(argv[i + 1]);
    return def;
}

int main(int argc, char** argv)
{
    const int want_w = ArgInt(argc, argv, "--w", 980);
    const int want_h = ArgInt(argc, argv, "--h", 760);
    int frames_left  = ArgInt(argc, argv, "--frames", 0);

    typedef BOOL (WINAPI* PFN_SetCtx)(HANDLE);
    if (HMODULE u32 = ::GetModuleHandleW(L"user32.dll"))
        if (PFN_SetCtx f = (PFN_SetCtx)::GetProcAddress(u32, "SetProcessDpiAwarenessContext"))
            f((HANDLE)-4);   // PER_MONITOR_AWARE_V2

    WNDCLASSEXW wc = { sizeof(wc), CS_CLASSDC, WndProc, 0, 0,
                       ::GetModuleHandleW(nullptr), nullptr, nullptr, nullptr,
                       nullptr, L"BoardPreview", nullptr };
    ::RegisterClassExW(&wc);

    RECT r = { 0, 0, want_w, want_h };
    ::AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    HWND hwnd = ::CreateWindowW(wc.lpszClassName, L"board_view preview",
                                WS_OVERLAPPEDWINDOW, 80, 40,
                                r.right - r.left, r.bottom - r.top,
                                nullptr, nullptr, wc.hInstance, nullptr);
    if (!hwnd) return 1;

    if (!CreateDeviceD3D(hwnd))
    {
        CleanupDeviceD3D();
        ::UnregisterClassW(wc.lpszClassName, wc.hInstance);
        return 1;
    }

    ::ShowWindow(hwnd, SW_SHOWDEFAULT);
    ::UpdateWindow(hwnd);

    float dpi = 1.0f;
    if (HMODULE u32 = ::GetModuleHandleW(L"user32.dll"))
    {
        typedef UINT (WINAPI* PFN_GetDpi)(HWND);
        if (PFN_GetDpi f = (PFN_GetDpi)::GetProcAddress(u32, "GetDpiForWindow"))
            dpi = (float)f(hwnd) / 96.0f;
    }
    if (dpi < 0.5f) dpi = 1.0f;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;

    ui::loadFonts(dpi);
    ui::applyStyle(dpi);
    ui::setDpiScale(dpi);

    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(g_device, g_context);

    bool running = true;
    while (running)
    {
        MSG msg;
        while (::PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE))
        {
            ::TranslateMessage(&msg);
            ::DispatchMessage(&msg);
            if (msg.message == WM_QUIT) running = false;
        }
        if (!running) break;

        if (g_resize)
        {
            g_resize = false;
            CleanupRenderTarget();
            g_swapchain->ResizeBuffers(0, 0, 0, DXGI_FORMAT_UNKNOWN, 0);
            CreateRenderTarget();
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
        const float clear[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
        g_context->OMSetRenderTargets(1, &g_rtv, nullptr);
        g_context->ClearRenderTargetView(g_rtv, clear);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
        g_swapchain->Present(1, 0);

        if (frames_left > 0 && --frames_left == 0) running = false;
    }

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
    CleanupDeviceD3D();
    ::DestroyWindow(hwnd);
    ::UnregisterClassW(wc.lpszClassName, wc.hInstance);
    return 0;
}
