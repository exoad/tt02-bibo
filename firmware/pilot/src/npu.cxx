#include "npu.hxx"

#include <cstdio>

#if defined(__linux__) && defined(PILOT_HAVE_VIPLITE)

#include <algorithm>
#include <cmath>
#include <cstring>

#include <vip_lite.h>

namespace npu
{
  namespace
  {
    // How a tensor is quantised, read from the binary.
    struct Quant
    {
        vip_enum format = VIP_BUFFER_FORMAT_UINT8;
        vip_enum quantizer = VIP_BUFFER_QUANTIZE_NONE;
        Int32 fixedPos = 0;
        Float32 scale = 1.0f;
        Int32 zero = 0;
    };

    struct Tensor
    {
        vip_buffer_create_params_t params = {};
        vip_buffer buffer = nullptr;
        Size bytes = 0;
        Quant quant;
        Array<UInt32, 6> dims = {};   // innermost first, as VIPLite reports them
        UInt32 ndims = 0;
    };

    Bool opened = false;
    vip_network net = nullptr;
    Tensor in;
    Tensor out;
    Info current;

    [[nodiscard]] CharSeq formatName(vip_enum f)
    {
        switch(f)
        {
            case VIP_BUFFER_FORMAT_UINT8:
                return "uint8";
            case VIP_BUFFER_FORMAT_INT8:
                return "int8";
            case VIP_BUFFER_FORMAT_INT16:
                return "int16";
            case VIP_BUFFER_FORMAT_FP16:
                return "fp16";
            case VIP_BUFFER_FORMAT_BFP16:
                return "bf16";
            case VIP_BUFFER_FORMAT_FP32:
                return "fp32";
            default:
                return "other";
        }
    }

    [[nodiscard]] Bool describe(Bool input, UInt32 index, Tensor& t, Str& why)
    {
        auto query = [&](vip_enum prop, void* value) {
            return input ? vip_query_input(net, index, prop, value) : vip_query_output(
                net,
                index,
                prop,
                value
            );
        };
        if(query(VIP_BUFFER_PROP_NUM_OF_DIMENSION, &t.ndims) != VIP_SUCCESS || t.ndims == 0u || t.ndims > 6u)
        {
            why = "the binary will not say its tensor's dimensions";
            return false;
        }
        t.params.num_of_dims = t.ndims;
        if(query(VIP_BUFFER_PROP_SIZES_OF_DIMENSION, t.params.sizes) != VIP_SUCCESS
           || query(VIP_BUFFER_PROP_DATA_FORMAT, &t.params.data_format) != VIP_SUCCESS
           || query(VIP_BUFFER_PROP_QUANT_FORMAT, &t.params.quant_format) != VIP_SUCCESS)
        {
            why = "the binary will not say its tensor's format";
            return false;
        }
        for(UInt32 d = 0; d < t.ndims; ++d)
        {
            t.dims[d] = t.params.sizes[d];
        }
        t.quant.format = t.params.data_format;
        t.quant.quantizer = t.params.quant_format;
        if(t.params.quant_format == VIP_BUFFER_QUANTIZE_DYNAMIC_FIXED_POINT)
        {
            vip_uint8_t pos = 0;
            static_cast<Void>(query(VIP_BUFFER_PROP_FIXED_POINT_POS, &pos));
            t.params.quant_data.dfp.fixed_point_pos = pos;
            t.quant.fixedPos = pos;
        }
        else if(t.params.quant_format != VIP_BUFFER_QUANTIZE_NONE)
        {
            static_cast<Void>(query(VIP_BUFFER_PROP_TF_SCALE, &t.params.quant_data.affine.scale));
            static_cast<Void>(query(
                VIP_BUFFER_PROP_TF_ZERO_POINT,
                &t.params.quant_data.affine.zeroPoint
            ));
            t.quant.scale = t.params.quant_data.affine.scale;
            t.quant.zero = t.params.quant_data.affine.zeroPoint;
        }
        t.params.memory_type = VIP_BUFFER_MEMORY_TYPE_DEFAULT;
        const vip_enum f = t.params.data_format;
        if(f != VIP_BUFFER_FORMAT_UINT8 && f != VIP_BUFFER_FORMAT_INT8 && f != VIP_BUFFER_FORMAT_INT16)
        {
            why = Str("a ") + formatName(f) + " tensor, which this module does not fill";
            return false;
        }
        return true;
    }

    Void release()
    {
        if(in.buffer != nullptr)
        {
            static_cast<Void>(vip_destroy_buffer(in.buffer));
            in.buffer = nullptr;
        }
        if(out.buffer != nullptr)
        {
            static_cast<Void>(vip_destroy_buffer(out.buffer));
            out.buffer = nullptr;
        }
        if(net != nullptr)
        {
            static_cast<Void>(vip_finish_network(net));
            static_cast<Void>(vip_destroy_network(net));
            net = nullptr;
        }
        if(opened)
        {
            static_cast<Void>(vip_destroy());
            opened = false;
        }
    }
  }

  Bool available()
  {
      return true;
  }

  Bool open(const Str& path, Info* info, Str& why)
  {
      if(info == nullptr)
      {
          why = "nowhere to describe the network";
          return false;
      }
      close();
      if(vip_init() != VIP_SUCCESS)
      {
          why = "vip_init failed - is /dev/vipcore there";
          return false;
      }
      opened = true;
      if(vip_create_network(path.c_str(), 0, VIP_CREATE_NETWORK_FROM_FILE, &net) != VIP_SUCCESS)
      {
          net = nullptr;
          why = path + ": not a network binary for this NPU";
          release();
          return false;
      }
      if(vip_prepare_network(net) != VIP_SUCCESS)
      {
          why = path + ": the NPU could not prepare it";
          release();
          return false;
      }
      vip_uint32_t inputs = 0;
      vip_uint32_t outputs = 0;
      static_cast<Void>(vip_query_network(net, VIP_NETWORK_PROP_INPUT_COUNT, &inputs));
      static_cast<Void>(vip_query_network(net, VIP_NETWORK_PROP_OUTPUT_COUNT, &outputs));
      if(inputs != 1u || outputs != 1u)
      {
          why = path + ": " + std::to_string(inputs) + " inputs and " + std::to_string(outputs) + " outputs, not one of each";
          release();
          return false;
      }
      if(!describe(true, 0, in, why) || !describe(false, 0, out, why))
      {
          release();
          return false;
      }
      if(in.ndims < 3u || in.dims[2] != 1u)
      {
          why = path + ": the input is not one channel";
          release();
          return false;
      }
      if(vip_create_buffer(&in.params, sizeof(in.params), &in.buffer) != VIP_SUCCESS
         || vip_set_input(net, 0, in.buffer) != VIP_SUCCESS
         || vip_create_buffer(&out.params, sizeof(out.params), &out.buffer) != VIP_SUCCESS
         || vip_set_output(net, 0, out.buffer) != VIP_SUCCESS)
      {
          why = path + ": could not make or attach its buffers";
          release();
          return false;
      }
      in.bytes = vip_get_buffer_size(in.buffer);
      out.bytes = vip_get_buffer_size(out.buffer);
      Array<Char, 64> name = {};
      static_cast<Void>(vip_query_network(net, VIP_NETWORK_PROP_NETWORK_NAME, name.data()));
      vip_uint32_t layers = 0;
      static_cast<Void>(vip_query_network(net, VIP_NETWORK_PROP_LAYER_COUNT, &layers));
      current = Info();
      current.name = name.data();
      current.layers = layers;
      current.inW = static_cast<Int32>(in.dims[0]);
      current.inH = static_cast<Int32>(in.dims[1]);
      current.inC = static_cast<Int32>(in.dims[2]);
      current.outW = static_cast<Int32>(out.dims[0]);
      current.outH = static_cast<Int32>(out.dims[1]);
      current.outC = out.ndims > 2u ? static_cast<Int32>(out.dims[2]) : 1;
      current.inFormat = formatName(in.quant.format);
      current.outFormat = formatName(out.quant.format);
      *info = current;
      return true;
  }

  Bool run(const UInt8* grey, Float32* result, UInt32* us, Str& why)
  {
      if(net == nullptr || in.buffer == nullptr)
      {
          why = "no network is open";
          return false;
      }
      const Size count = static_cast<Size>(current.inW) * static_cast<Size>(current.inH);
      void* mapped = vip_map_buffer(in.buffer);
      if(mapped == nullptr)
      {
          why = "the input buffer would not map";
          return false;
      }
      // The picture into the input's own quantisation. The graph carries its
      // own 1/255, so the number quantised is the raw intensity.
      switch(in.quant.format)
      {
          case VIP_BUFFER_FORMAT_UINT8:
          {
              UInt8* p = static_cast<UInt8*>(mapped);
              const Float32 inv = 1.0f / std::max(1e-9f, in.quant.scale);
              for(Size i = 0; i < count; ++i)
              {
                  const Float32 q = std::round(static_cast<Float32>(grey[i]) * inv) + static_cast<Float32>(in.quant.zero);
                  p[i] = static_cast<UInt8>(std::clamp(q, 0.0f, 255.0f));
              }
              break;
          }
          case VIP_BUFFER_FORMAT_INT8:
          {
              Int8* p = static_cast<Int8*>(mapped);
              const Float32 inv = 1.0f / std::max(1e-9f, in.quant.scale);
              for(Size i = 0; i < count; ++i)
              {
                  const Float32 q = std::round(static_cast<Float32>(grey[i]) * inv) + static_cast<Float32>(in.quant.zero);
                  p[i] = static_cast<Int8>(std::clamp(q, -128.0f, 127.0f));
              }
              break;
          }
          default:
          {
              Int16* p = static_cast<Int16*>(mapped);
              const Float32 unit = static_cast<Float32>(1 << in.quant.fixedPos);
              for(Size i = 0; i < count; ++i)
              {
                  const Float32 q = std::round(static_cast<Float32>(grey[i]) * unit);
                  p[i] = static_cast<Int16>(std::clamp(q, -32768.0f, 32767.0f));
              }
              break;
          }
      }
      static_cast<Void>(vip_unmap_buffer(in.buffer));
      static_cast<Void>(vip_flush_buffer(in.buffer, VIP_BUFFER_OPER_TYPE_FLUSH));
      const TimePoint before = monoNow();
      if(vip_run_network(net) != VIP_SUCCESS)
      {
          why = "vip_run_network failed";
          return false;
      }
      const Float64 took = elapsedMs(before) * 1000.0;
      *us = took > 0.0 ? static_cast<UInt32>(took) : 0u;
      static_cast<Void>(vip_flush_buffer(out.buffer, VIP_BUFFER_OPER_TYPE_INVALIDATE));
      const void* got = vip_map_buffer(out.buffer);
      if(got == nullptr)
      {
          why = "the output buffer would not map";
          return false;
      }
      const Size values = static_cast<Size>(current.outW) * static_cast<Size>(current.outH) * static_cast<Size>(current.outC);
      switch(out.quant.format)
      {
          case VIP_BUFFER_FORMAT_UINT8:
          {
              const UInt8* p = static_cast<const UInt8*>(got);
              for(Size i = 0; i < values; ++i)
              {
                  result[i] = (static_cast<Float32>(p[i]) - static_cast<Float32>(out.quant.zero)) * out.quant.scale;
              }
              break;
          }
          case VIP_BUFFER_FORMAT_INT8:
          {
              const Int8* p = static_cast<const Int8*>(got);
              for(Size i = 0; i < values; ++i)
              {
                  result[i] = (static_cast<Float32>(p[i]) - static_cast<Float32>(out.quant.zero)) * out.quant.scale;
              }
              break;
          }
          default:
          {
              const Int16* p = static_cast<const Int16*>(got);
              const Float32 unit = 1.0f / static_cast<Float32>(1 << out.quant.fixedPos);
              for(Size i = 0; i < values; ++i)
              {
                  result[i] = static_cast<Float32>(p[i]) * unit;
              }
              break;
          }
      }
      static_cast<Void>(vip_unmap_buffer(out.buffer));
      return true;
  }

  Void close()
  {
      release();
  }
}

#else

// No VIPLite on this platform: open() refuses, saying so.
namespace npu
{
  namespace
  {
    constexpr CharSeq NOT_BUILT = "no VIPLite on this platform - the NPU is the board's";
  }

  Bool available()
  {
      return false;
  }

  Bool open(const Str& path, Info* info, Str& why)
  {
      static_cast<Void>(path);
      static_cast<Void>(info);
      why = NOT_BUILT;
      return false;
  }

  Bool run(const UInt8* grey, Float32* out, UInt32* us, Str& why)
  {
      static_cast<Void>(grey);
      static_cast<Void>(out);
      static_cast<Void>(us);
      why = NOT_BUILT;
      return false;
  }

  Void close()
  {
  }
}

#endif
