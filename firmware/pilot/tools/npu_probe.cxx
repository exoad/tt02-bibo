// npu_probe: load one network binary on the board's NPU through VIPLite, say
// what it takes and gives, run it N times, and time it. The first rung of the
// NPU port (docs/bundles.md): before any model of ours exists, this proves the
// runtime, its version, and the shape of the call sequence a pilot module
// would make.
//
//   npu_probe <network.nb> [input.dat] [runs] [dump-prefix]
//
// With a dump prefix, every output tensor is written raw to <prefix>.<i>.bin
// after the last run, so the NPU's answer can be compared with the model's
// on a PC (tools/npu/compare_npu.py).
//
// Linux with VIPLite only (PILOT_HAVE_VIPLITE, found by CMake on the board).
#include "shared.hxx"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#if defined(__linux__) && defined(PILOT_HAVE_VIPLITE)

#include <vip_lite.h>

namespace
{
  [[nodiscard]] CharSeq formatName(vip_enum f)
  {
      switch(f)
      {
          case VIP_BUFFER_FORMAT_FP32:
              return "fp32";
          case VIP_BUFFER_FORMAT_FP16:
              return "fp16";
          case VIP_BUFFER_FORMAT_UINT8:
              return "uint8";
          case VIP_BUFFER_FORMAT_INT8:
              return "int8";
          case VIP_BUFFER_FORMAT_UINT16:
              return "uint16";
          case VIP_BUFFER_FORMAT_INT16:
              return "int16";
          case VIP_BUFFER_FORMAT_BFP16:
              return "bf16";
          case VIP_BUFFER_FORMAT_INT32:
              return "int32";
          default:
              return "other";
      }
  }

  [[nodiscard]] Size bytesPer(vip_enum f)
  {
      switch(f)
      {
          case VIP_BUFFER_FORMAT_UINT8:
          case VIP_BUFFER_FORMAT_INT8:
          case VIP_BUFFER_FORMAT_CHAR:
          case VIP_BUFFER_FORMAT_BOOL8:
              return 1;
          case VIP_BUFFER_FORMAT_FP16:
          case VIP_BUFFER_FORMAT_UINT16:
          case VIP_BUFFER_FORMAT_INT16:
          case VIP_BUFFER_FORMAT_BFP16:
              return 2;
          case VIP_BUFFER_FORMAT_INT64:
          case VIP_BUFFER_FORMAT_UINT64:
          case VIP_BUFFER_FORMAT_FP64:
              return 8;
          default:
              return 4;
      }
  }

  // One tensor of the network, as VIPLite describes it, and the buffer made for it.
  struct Port
  {
      vip_buffer_create_params_t params = {};
      vip_buffer buffer = nullptr;
      Size bytes = 0;
      Array<Char, 64> name = {};
  };

  // Fills `p` from the network's own description of input or output `index`.
  [[nodiscard]] Bool describe(vip_network net, Bool input, UInt32 index, Port& p)
  {
      auto query = [&](vip_enum prop, void* out) {
          return input ? vip_query_input(net, index, prop, out) : vip_query_output(
              net,
              index,
              prop,
              out
          );
      };
      vip_uint32_t dims = 0;
      if(query(VIP_BUFFER_PROP_NUM_OF_DIMENSION, &dims) != VIP_SUCCESS || dims == 0u || dims > 6u)
      {
          return false;
      }
      p.params.num_of_dims = dims;
      if(query(VIP_BUFFER_PROP_SIZES_OF_DIMENSION, p.params.sizes) != VIP_SUCCESS)
      {
          return false;
      }
      if(query(VIP_BUFFER_PROP_DATA_FORMAT, &p.params.data_format) != VIP_SUCCESS)
      {
          return false;
      }
      if(query(VIP_BUFFER_PROP_QUANT_FORMAT, &p.params.quant_format) != VIP_SUCCESS)
      {
          return false;
      }
      if(p.params.quant_format == VIP_BUFFER_QUANTIZE_DYNAMIC_FIXED_POINT)
      {
          vip_uint8_t pos = 0;
          static_cast<Void>(query(VIP_BUFFER_PROP_FIXED_POINT_POS, &pos));
          p.params.quant_data.dfp.fixed_point_pos = pos;
      }
      else if(p.params.quant_format != VIP_BUFFER_QUANTIZE_NONE)
      {
          static_cast<Void>(query(VIP_BUFFER_PROP_TF_SCALE, &p.params.quant_data.affine.scale));
          static_cast<Void>(query(
              VIP_BUFFER_PROP_TF_ZERO_POINT,
              &p.params.quant_data.affine.zeroPoint
          ));
      }
      static_cast<Void>(query(VIP_BUFFER_PROP_NAME, p.name.data()));
      p.params.memory_type = VIP_BUFFER_MEMORY_TYPE_DEFAULT;
      Size count = 1;
      for(UInt32 d = 0; d < dims; ++d)
      {
          count *= p.params.sizes[d];
      }
      p.bytes = count * bytesPer(p.params.data_format);
      std::printf("    (%zu values by the dims)\n", count);
      return true;
  }

  Void print(CharSeq what, UInt32 index, const Port& p)
  {
      std::printf(
          "  %s %u \"%s\": %s",
          what,
          static_cast<unsigned>(index),
          p.name.data(),
          formatName(p.params.data_format)
      );
      for(UInt32 d = 0; d < p.params.num_of_dims; ++d)
      {
          std::printf("%s%u", d == 0u ? " [" : " x ", static_cast<unsigned>(p.params.sizes[d]));
      }
      std::printf("]");
      if(p.params.quant_format == VIP_BUFFER_QUANTIZE_DYNAMIC_FIXED_POINT)
      {
          std::printf(" dfp pos %d", p.params.quant_data.dfp.fixed_point_pos);
      }
      else if(p.params.quant_format != VIP_BUFFER_QUANTIZE_NONE)
      {
          std::printf(
              " affine scale %g zero %d",
              static_cast<Float64>(p.params.quant_data.affine.scale),
              p.params.quant_data.affine.zeroPoint
          );
      }
      std::printf(", %zu bytes\n", p.bytes);
  }
}

int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::printf("npu_probe <network.nb> [input.dat] [runs]\n");
        return 2;
    }
    const Str path = argv[1];
    const Str inputPath = argc > 2 ? argv[2] : "";
    const Int32 runs = argc > 3 ? std::atoi(argv[3]) : 20;
    const Str dumpPrefix = argc > 4 ? argv[4] : "";
    const UInt32 version = vip_get_version();
    std::printf(
        "viplite %u.%u.%u.%u\n",
        static_cast<unsigned>(version >> 24),
        static_cast<unsigned>((version >> 16) & 0xFFu),
        static_cast<unsigned>((version >> 8) & 0xFFu),
        static_cast<unsigned>(version & 0xFFu)
    );
    if(vip_init() != VIP_SUCCESS)
    {
        std::printf("vip_init failed: is /dev/vipcore there and readable?\n");
        return 1;
    }
    vip_uint32_t cid = 0;
    vip_uint32_t devices = 0;
    static_cast<Void>(vip_query_hardware(VIP_QUERY_HW_PROP_CID, sizeof(cid), &cid));
    static_cast<Void>(vip_query_hardware(VIP_QUERY_HW_PROP_DEVICE_COUNT, sizeof(devices), &devices));
    std::printf(
        "hardware cid 0x%x, %u device(s)\n",
        static_cast<unsigned>(cid),
        static_cast<unsigned>(devices)
    );
    vip_network net = nullptr;
    const TimePoint loadAt = monoNow();
    if(vip_create_network(path.c_str(), 0, VIP_CREATE_NETWORK_FROM_FILE, &net) != VIP_SUCCESS)
    {
        std::printf(
            "%s: vip_create_network refused it - not an NBG for this NPU or driver?\n",
            path.c_str()
        );
        static_cast<Void>(vip_destroy());
        return 1;
    }
    if(vip_prepare_network(net) != VIP_SUCCESS)
    {
        std::printf("%s: vip_prepare_network failed\n", path.c_str());
        static_cast<Void>(vip_destroy_network(net));
        static_cast<Void>(vip_destroy());
        return 1;
    }
    Array<Char, 64> netName = {};
    static_cast<Void>(vip_query_network(net, VIP_NETWORK_PROP_NETWORK_NAME, netName.data()));
    vip_uint32_t inputs = 0;
    vip_uint32_t outputs = 0;
    vip_uint32_t layers = 0;
    static_cast<Void>(vip_query_network(net, VIP_NETWORK_PROP_INPUT_COUNT, &inputs));
    static_cast<Void>(vip_query_network(net, VIP_NETWORK_PROP_OUTPUT_COUNT, &outputs));
    static_cast<Void>(vip_query_network(net, VIP_NETWORK_PROP_LAYER_COUNT, &layers));
    std::printf(
        "%s: \"%s\", %u layer(s), %u input(s), %u output(s), prepared in %.1f ms\n",
        path.c_str(),
        netName.data(),
        static_cast<unsigned>(layers),
        static_cast<unsigned>(inputs),
        static_cast<unsigned>(outputs),
        elapsedMs(loadAt)
    );
    Vec<Port> in(inputs);
    Vec<Port> out(outputs);
    Bool ok = true;
    for(UInt32 i = 0; i < inputs && ok; ++i)
    {
        ok = describe(net, true, i, in[i]);
        if(ok)
        {
            print("input", i, in[i]);
            ok = vip_create_buffer(&in[i].params, sizeof(in[i].params), &in[i].buffer) == VIP_SUCCESS
              && vip_set_input(net, i, in[i].buffer) == VIP_SUCCESS;
            if(ok)
            {
                in[i].bytes = vip_get_buffer_size(in[i].buffer);
                std::printf("    buffer of %zu bytes\n", in[i].bytes);
            }
        }
    }
    for(UInt32 i = 0; i < outputs && ok; ++i)
    {
        ok = describe(net, false, i, out[i]);
        if(ok)
        {
            print("output", i, out[i]);
            ok = vip_create_buffer(&out[i].params, sizeof(out[i].params), &out[i].buffer) == VIP_SUCCESS
              && vip_set_output(net, i, out[i].buffer) == VIP_SUCCESS;
            if(ok)
            {
                out[i].bytes = vip_get_buffer_size(out[i].buffer);
                std::printf("    buffer of %zu bytes\n", out[i].bytes);
            }
        }
    }
    if(!ok)
    {
        std::printf("describing or attaching the tensors failed\n");
    }
    // The first input from the file when one was given, zeros otherwise: the
    // timing is the same, and zeros still prove the run completes.
    if(ok && inputs > 0u)
    {
        void* mapped = vip_map_buffer(in[0].buffer);
        const Size cap = vip_get_buffer_size(in[0].buffer);
        std::memset(mapped, 0, cap);
        if(!inputPath.empty())
        {
            std::FILE* f = std::fopen(inputPath.c_str(), "rb");
            if(f == nullptr)
            {
                std::printf("%s: cannot open, input left as zeros\n", inputPath.c_str());
            }
            else
            {
                const Size got = std::fread(mapped, 1, cap, f);
                std::fclose(f);
                std::printf("input 0 from %s: %zu of %zu bytes\n", inputPath.c_str(), got, cap);
            }
        }
        static_cast<Void>(vip_unmap_buffer(in[0].buffer));
        static_cast<Void>(vip_flush_buffer(in[0].buffer, VIP_BUFFER_OPER_TYPE_FLUSH));
    }
    if(ok)
    {
        Float64 total = 0.0;
        Float64 worst = 0.0;
        Int32 done = 0;
        for(Int32 r = 0; r < runs; ++r)
        {
            const TimePoint before = monoNow();
            if(vip_run_network(net) != VIP_SUCCESS)
            {
                std::printf("run %d: vip_run_network failed\n", r);
                ok = false;
                break;
            }
            const Float64 ms = elapsedMs(before);
            total += ms;
            worst = ms > worst ? ms : worst;
            ++done;
        }
        if(done > 0)
        {
            std::printf(
                "%d run(s): %.2f ms average, %.2f ms worst, wall clock around the call\n",
                done,
                total / done,
                worst
            );
        }
        for(UInt32 i = 0; i < outputs; ++i)
        {
            static_cast<Void>(vip_flush_buffer(out[i].buffer, VIP_BUFFER_OPER_TYPE_INVALIDATE));
            const UInt8* p = static_cast<const UInt8*>(vip_map_buffer(out[i].buffer));
            std::printf("  output %u first bytes:", static_cast<unsigned>(i));
            for(Size b = 0; b < 16u && b < out[i].bytes; ++b)
            {
                std::printf(" %02x", p[b]);
            }
            std::printf("\n");
            if(!dumpPrefix.empty())
            {
                const Str path = dumpPrefix + "." + std::to_string(i) + ".bin";
                std::FILE* f = std::fopen(path.c_str(), "wb");
                if(f != nullptr)
                {
                    static_cast<Void>(std::fwrite(p, 1, out[i].bytes, f));
                    std::fclose(f);
                    std::printf(
                        "  output %u written to %s\n",
                        static_cast<unsigned>(i),
                        path.c_str()
                    );
                }
            }
            static_cast<Void>(vip_unmap_buffer(out[i].buffer));
        }
    }
    for(Port& p : in)
    {
        if(p.buffer != nullptr)
        {
            static_cast<Void>(vip_destroy_buffer(p.buffer));
        }
    }
    for(Port& p : out)
    {
        if(p.buffer != nullptr)
        {
            static_cast<Void>(vip_destroy_buffer(p.buffer));
        }
    }
    static_cast<Void>(vip_finish_network(net));
    static_cast<Void>(vip_destroy_network(net));
    static_cast<Void>(vip_destroy());
    return ok ? 0 : 1;
}

#else

int main()
{
    std::printf(
        "npu_probe: no VIPLite on this platform - it runs on the board, where CMake finds vip_lite.h\n"
    );
    return 1;
}

#endif
