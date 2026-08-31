#include "scene_gpu.hxx"

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <d3d11.h>
#include <d3dcompiler.h>

#include <algorithm>
#include <vector>

namespace scenegpu
{

  namespace
  {

    // One shader pair for everything. Untextured geometry binds a 1x1 white texel,
    // so the pixel shader has no branch and there is no second pipeline to keep in
    // step with this one.
    const Char* SHADER_SRC = R"(
    cbuffer Constants : register(b0)
    {
        row_major float4x4 mvp;
    };

    struct VSIn
    {
        float3 pos : POSITION;
        float4 col : COLOR0;
        float2 uv  : TEXCOORD0;
    };

    struct PSIn
    {
        float4 pos : SV_POSITION;
        float4 col : COLOR0;
        float2 uv  : TEXCOORD0;
    };

    PSIn VSMain(VSIn i)
    {
        PSIn o;
        o.pos = mul(float4(i.pos, 1.0f), mvp);
        o.col = i.col;
        o.uv  = i.uv;
        return o;
    }

    Texture2D    tex0 : register(t0);
    SamplerState samp : register(s0);

    float4 PSMain(PSIn i) : SV_Target
    {
        return i.col * tex0.Sample(samp, i.uv);
    }
    )";

    ID3D11Device*        dev = nullptr;
    ID3D11DeviceContext* ctx = nullptr;

    ID3D11VertexShader*      vs         = nullptr;
    ID3D11PixelShader*       ps         = nullptr;
    ID3D11InputLayout*       layout     = nullptr;
    ID3D11Buffer*            cbuf       = nullptr;
    ID3D11Buffer*            vbuf       = nullptr;
    Int32                    vbufCap    = 0;
    ID3D11RasterizerState*   rasterizer = nullptr;
    ID3D11DepthStencilState* dsOpaque   = nullptr;
    ID3D11DepthStencilState* dsBlended  = nullptr;
    ID3D11BlendState*        blendOn    = nullptr;
    ID3D11SamplerState*      sampler    = nullptr;

    // The 1x1 white texel untextured geometry samples.
    ID3D11Texture2D*          whiteTex = nullptr;
    ID3D11ShaderResourceView* whiteSrv = nullptr;

    // The off-screen target, remade when the viewport changes size.
    ID3D11Texture2D*          colorTex = nullptr;
    ID3D11RenderTargetView*   colorRtv = nullptr;
    ID3D11ShaderResourceView* colorSrv = nullptr;
    ID3D11Texture2D*          depthTex = nullptr;
    ID3D11DepthStencilView*   depthDsv = nullptr;
    Int32                     rtW = 0, rtH = 0;

    struct Tri
    {
        Vertex      v[3];
        ImTextureID tex = 0;
    };

    Vec<Tri> opaqueTris;
    Vec<Tri> blendedTris;

    Bool started = false;

    template <typename T>
    Void release(T*& p)
    {
        if(p != nullptr)
        {
            p->Release();
            p = nullptr;
        }
    }

    Void releaseTargets()
    {
        release(colorSrv);
        release(colorRtv);
        release(colorTex);
        release(depthDsv);
        release(depthTex);
        rtW = rtH = 0;
    }

    Bool ensureTargets(Int32 w, Int32 h)
    {
        if(w == rtW && h == rtH && colorRtv != nullptr && depthDsv != nullptr)
        {
            return true;
        }

        releaseTargets();

        D3D11_TEXTURE2D_DESC td = {};
        td.Width            = static_cast<UINT>(w);
        td.Height           = static_cast<UINT>(h);
        td.MipLevels        = 1;
        td.ArraySize        = 1;
        td.Format           = DXGI_FORMAT_R8G8B8A8_UNORM;
        td.SampleDesc.Count = 1;
        td.Usage            = D3D11_USAGE_DEFAULT;
        td.BindFlags        = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;

        if(FAILED(dev->CreateTexture2D(&td, nullptr, &colorTex)))
        {
            releaseTargets();
            return false;
        }
        if(FAILED(dev->CreateRenderTargetView(colorTex, nullptr, &colorRtv)))
        {
            releaseTargets();
            return false;
        }
        if(FAILED(dev->CreateShaderResourceView(colorTex, nullptr, &colorSrv)))
        {
            releaseTargets();
            return false;
        }

        D3D11_TEXTURE2D_DESC dd = td;
        dd.Format    = DXGI_FORMAT_D32_FLOAT;
        dd.BindFlags = D3D11_BIND_DEPTH_STENCIL;
        if(FAILED(dev->CreateTexture2D(&dd, nullptr, &depthTex)))
        {
            releaseTargets();
            return false;
        }
        if(FAILED(dev->CreateDepthStencilView(depthTex, nullptr, &depthDsv)))
        {
            releaseTargets();
            return false;
        }

        rtW = w;
        rtH = h;
        return true;
    }

    Bool ensureVertexBuffer(Int32 count)
    {
        if(count <= vbufCap && vbuf != nullptr)
        {
            return true;
        }

        release(vbuf);

        // Grown in chunks so a scene that gains a few returns does not reallocate
        // every frame.
        vbufCap = ((count * 3) / 2 + 4096);

        D3D11_BUFFER_DESC bd = {};
        bd.ByteWidth      = static_cast<UINT>(vbufCap * static_cast<Int32>(sizeof(Vertex)));
        bd.Usage          = D3D11_USAGE_DYNAMIC;
        bd.BindFlags      = D3D11_BIND_VERTEX_BUFFER;
        bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;

        if(FAILED(dev->CreateBuffer(&bd, nullptr, &vbuf)))
        {
            vbufCap = 0;
            return false;
        }
        return true;
    }

  }

  Void init(ID3D11Device* device, ID3D11DeviceContext* context)
  {
      shutdown();

      dev = device;
      ctx = context;
      if(dev == nullptr || ctx == nullptr)
      {
          return;
      }

      ID3DBlob* vsBlob = nullptr;
      ID3DBlob* psBlob = nullptr;
      ID3DBlob* err    = nullptr;

      const UINT flags = D3DCOMPILE_OPTIMIZATION_LEVEL3;
      const Size srcLen = std::strlen(SHADER_SRC);

      if(FAILED(::D3DCompile(SHADER_SRC, srcLen, nullptr, nullptr, nullptr,
                             "VSMain", "vs_5_0", flags, 0, &vsBlob, &err)))
      {
          release(err);
          shutdown();
          return;
      }
      release(err);

      if(FAILED(::D3DCompile(SHADER_SRC, srcLen, nullptr, nullptr, nullptr,
                             "PSMain", "ps_5_0", flags, 0, &psBlob, &err)))
      {
          release(err);
          release(vsBlob);
          shutdown();
          return;
      }
      release(err);

      if(FAILED(dev->CreateVertexShader(vsBlob->GetBufferPointer(),
                                        vsBlob->GetBufferSize(), nullptr, &vs))
         || FAILED(dev->CreatePixelShader(psBlob->GetBufferPointer(),
                                          psBlob->GetBufferSize(), nullptr, &ps)))
      {
          release(vsBlob); release(psBlob); shutdown(); return;
      }

      constexpr D3D11_INPUT_ELEMENT_DESC elems[] = {
          { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0,  0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
          { "COLOR",    0, DXGI_FORMAT_R8G8B8A8_UNORM,  0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
          { "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT,    0, 16, D3D11_INPUT_PER_VERTEX_DATA, 0 },
      };
      if(FAILED(dev->CreateInputLayout(elems, 3, vsBlob->GetBufferPointer(),
                                       vsBlob->GetBufferSize(), &layout)))
      {
          release(vsBlob); release(psBlob); shutdown(); return;
      }
      release(vsBlob);
      release(psBlob);

      D3D11_BUFFER_DESC cb = {};
      cb.ByteWidth      = 16 * sizeof(Float32);
      cb.Usage          = D3D11_USAGE_DYNAMIC;
      cb.BindFlags      = D3D11_BIND_CONSTANT_BUFFER;
      cb.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
      if(FAILED(dev->CreateBuffer(&cb, nullptr, &cbuf)))
      {
          shutdown();
          return;
      }

      // Cull NOTHING. The depth buffer decides visibility now, and letting it do
      // so means a mesh with inconsistent winding - which any downloaded model may
      // have - renders correctly instead of developing holes.
      D3D11_RASTERIZER_DESC rd = {};
      rd.FillMode        = D3D11_FILL_SOLID;
      rd.CullMode        = D3D11_CULL_NONE;
      rd.DepthClipEnable = TRUE;
      if(FAILED(dev->CreateRasterizerState(&rd, &rasterizer)))
      {
          shutdown();
          return;
      }

      D3D11_DEPTH_STENCIL_DESC ds = {};
      ds.DepthEnable    = TRUE;
      ds.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
      ds.DepthFunc      = D3D11_COMPARISON_LESS;
      if(FAILED(dev->CreateDepthStencilState(&ds, &dsOpaque)))
      {
          shutdown();
          return;
      }

      // Translucent geometry tests depth but does not write it, so two see-through
      // surfaces do not hide each other.
      ds.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
      if(FAILED(dev->CreateDepthStencilState(&ds, &dsBlended)))
      {
          shutdown();
          return;
      }

      D3D11_BLEND_DESC bd = {};
      bd.RenderTarget[0].BlendEnable           = TRUE;
      bd.RenderTarget[0].SrcBlend              = D3D11_BLEND_SRC_ALPHA;
      bd.RenderTarget[0].DestBlend             = D3D11_BLEND_INV_SRC_ALPHA;
      bd.RenderTarget[0].BlendOp               = D3D11_BLEND_OP_ADD;
      bd.RenderTarget[0].SrcBlendAlpha         = D3D11_BLEND_ONE;
      bd.RenderTarget[0].DestBlendAlpha        = D3D11_BLEND_INV_SRC_ALPHA;
      bd.RenderTarget[0].BlendOpAlpha          = D3D11_BLEND_OP_ADD;
      bd.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
      if(FAILED(dev->CreateBlendState(&bd, &blendOn)))
      {
          shutdown();
          return;
      }

      D3D11_SAMPLER_DESC sd = {};
      sd.Filter   = D3D11_FILTER_MIN_MAG_MIP_POINT;   // a palette atlas; do not blur it
      sd.AddressU = D3D11_TEXTURE_ADDRESS_CLAMP;
      sd.AddressV = D3D11_TEXTURE_ADDRESS_CLAMP;
      sd.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
      if(FAILED(dev->CreateSamplerState(&sd, &sampler)))
      {
          shutdown();
          return;
      }

      const UInt32 WHITE = 0xFFFFFFFFu;
      D3D11_TEXTURE2D_DESC wd = {};
      wd.Width = wd.Height = 1;
      wd.MipLevels = wd.ArraySize = 1;
      wd.Format            = DXGI_FORMAT_R8G8B8A8_UNORM;
      wd.SampleDesc.Count  = 1;
      wd.Usage             = D3D11_USAGE_IMMUTABLE;
      wd.BindFlags         = D3D11_BIND_SHADER_RESOURCE;

      D3D11_SUBRESOURCE_DATA wsd = {};
      wsd.pSysMem     = &WHITE;
      wsd.SysMemPitch = 4;
      if(FAILED(dev->CreateTexture2D(&wd, &wsd, &whiteTex))
         || FAILED(dev->CreateShaderResourceView(whiteTex, nullptr, &whiteSrv)))
      {
          shutdown();
          return;
      }
  }

  Void shutdown()
  {
      releaseTargets();
      release(whiteSrv);
      release(whiteTex);
      release(sampler);
      release(blendOn);
      release(dsBlended);
      release(dsOpaque);
      release(rasterizer);
      release(vbuf);
      release(cbuf);
      release(layout);
      release(ps);
      release(vs);
      vbufCap = 0;
      dev = nullptr;
      ctx = nullptr;
      opaqueTris.clear();
      blendedTris.clear();
      started = false;
  }

  Bool ready() noexcept
  {
      return dev != nullptr && vs != nullptr && ps != nullptr && layout != nullptr;
  }

  Bool begin(Int32 widthPx, Int32 heightPx)
  {
      started = false;
      opaqueTris.clear();
      blendedTris.clear();

      if(!ready() || widthPx < 8 || heightPx < 8)
      {
          return false;
      }
      if(!ensureTargets(widthPx, heightPx))
      {
          return false;
      }

      started = true;
      return true;
  }

  Void addOpaque(const Vertex& a, const Vertex& b, const Vertex& c, ImTextureID tex)
  {
      if(!started)
      {
          return;
      }
      Tri t;
      t.v[0] = a; t.v[1] = b; t.v[2] = c;
      t.tex  = tex;
      opaqueTris.push_back(t);
  }

  Void addBlended(const Vertex& a, const Vertex& b, const Vertex& c, ImTextureID tex)
  {
      if(!started)
      {
          return;
      }
      Tri t;
      t.v[0] = a; t.v[1] = b; t.v[2] = c;
      t.tex  = tex;
      blendedTris.push_back(t);
  }

  ImTextureID end(const Float32* mvp)
  {
      if(!started || mvp == nullptr)
      {
          return 0;
      }
      started = false;

      constexpr Array<Float32, 4> CLEAR= { 0.0f, 0.0f, 0.0f, 1.0f };
      ctx->ClearRenderTargetView(colorRtv, CLEAR.data());
      ctx->ClearDepthStencilView(depthDsv, D3D11_CLEAR_DEPTH, 1.0f, 0);

      if(opaqueTris.empty() && blendedTris.empty())
      {
          return reinterpret_cast<ImTextureID>(colorSrv);
      }

      // Opaque triangles may be reordered freely - the depth buffer decides the
      // result, not the order - so they are grouped by texture to collapse the
      // draw calls. Blended ones may not: they are sorted back to front, and the
      // batching has to follow that order.
      std::stable_sort(opaqueTris.begin(), opaqueTris.end(),
                       [](const Tri& x, const Tri& y) { return x.tex < y.tex; });

      const auto viewDepth = [&](const Tri& t) {
          // w after the row-vector transform is the view-space depth.
          Float32 d = 0.0f;
          for(Int32 i = 0; i < 3; ++i)
          {
              d += t.v[i].x * mvp[3] + t.v[i].y * mvp[7] + t.v[i].z * mvp[11] + mvp[15];
          }
          return d;
      };
      std::stable_sort(blendedTris.begin(), blendedTris.end(),
                       [&](const Tri& x, const Tri& y) { return viewDepth(x) > viewDepth(y); });

      const Int32 total = static_cast<Int32>((opaqueTris.size() + blendedTris.size()) * 3);
      if(!ensureVertexBuffer(total))
      {
          return reinterpret_cast<ImTextureID>(colorSrv);
      }

      D3D11_MAPPED_SUBRESOURCE ms = {};
      if(FAILED(ctx->Map(vbuf, 0, D3D11_MAP_WRITE_DISCARD, 0, &ms)))
      {
          return reinterpret_cast<ImTextureID>(colorSrv);
      }

      Vertex* out = static_cast<Vertex*>(ms.pData);
      Int32   n   = 0;
      for(const Tri& t : opaqueTris)
      {
          for(Int32 i = 0; i < 3; ++i)
          {
              out[n++] = t.v[i];
          }
      }
      for(const Tri& t : blendedTris)
      {
          for(Int32 i = 0; i < 3; ++i)
          {
              out[n++] = t.v[i];
          }
      }
      ctx->Unmap(vbuf, 0);

      if(SUCCEEDED(ctx->Map(cbuf, 0, D3D11_MAP_WRITE_DISCARD, 0, &ms)))
      {
          std::memcpy(ms.pData, mvp, 16 * sizeof(Float32));
          ctx->Unmap(cbuf, 0);
      }

      // ---- state ------------------------------------------------------------
      //
      // ImGui's DX11 backend saves and restores the whole pipeline around its own
      // render, and this runs inside app::frame() - before that render - so it is
      // free to set state without putting it back.
      ID3D11RenderTargetView* rtv = colorRtv;
      ctx->OMSetRenderTargets(1, &rtv, depthDsv);

      D3D11_VIEWPORT vp = {};
      vp.Width    = static_cast<FLOAT>(rtW);
      vp.Height   = static_cast<FLOAT>(rtH);
      vp.MaxDepth = 1.0f;
      ctx->RSSetViewports(1, &vp);

      const UINT stride = sizeof(Vertex);
      const UINT offset = 0;
      ctx->IASetInputLayout(layout);
      ctx->IASetVertexBuffers(0, 1, &vbuf, &stride, &offset);
      ctx->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
      ctx->VSSetShader(vs, nullptr, 0);
      ctx->VSSetConstantBuffers(0, 1, &cbuf);
      ctx->PSSetShader(ps, nullptr, 0);
      ctx->PSSetSamplers(0, 1, &sampler);
      ctx->RSSetState(rasterizer);

      constexpr Array<Float32, 4> BLEND_FACTOR= { 0.0f, 0.0f, 0.0f, 0.0f };
      ctx->OMSetBlendState(blendOn, BLEND_FACTOR.data(), 0xFFFFFFFFu);

      // Runs of one texture, drawn in one call each.
      const auto drawRuns = [&](const Vec<Tri>& tris, Int32 base) {
          Size i = 0;
          while(i < tris.size())
          {
              const ImTextureID tex = tris[i].tex;
              Size j = i + 1;
              while(j < tris.size() && tris[j].tex == tex)
              {
                  ++j;
              }

              ID3D11ShaderResourceView* srv =
                  (tex != 0) ? reinterpret_cast<ID3D11ShaderResourceView*>(tex) : whiteSrv;
              ctx->PSSetShaderResources(0, 1, &srv);

              ctx->Draw(static_cast<UINT>((j - i) * 3),
                        static_cast<UINT>(base + static_cast<Int32>(i) * 3));
              i = j;
          }
      };

      ctx->OMSetDepthStencilState(dsOpaque, 0);
      drawRuns(opaqueTris, 0);

      ctx->OMSetDepthStencilState(dsBlended, 0);
      drawRuns(blendedTris, static_cast<Int32>(opaqueTris.size()) * 3);

      // The render target must not stay bound: ImGui is about to draw to the back
      // buffer, and leaving our color texture as an output while it is also about
      // to be read as a shader resource is exactly the hazard D3D warns about.
      ID3D11RenderTargetView* none = nullptr;
      ctx->OMSetRenderTargets(1, &none, nullptr);

      return reinterpret_cast<ImTextureID>(colorSrv);
  }

}
