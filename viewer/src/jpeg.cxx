#include "jpeg.hxx"

// stb_image's implementation is compiled HERE and in no other translation
// unit: the header is the library, and STB_IMAGE_IMPLEMENTATION is the define
// that turns it into one.
//
// STBI_ONLY_JPEG and STBI_NO_STDIO are not size tuning. They are the two lines
// that keep this module's attack surface to the one parser it actually needs -
// see the header. Nothing here opens a file, so the stdio path is not built.
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_NO_STDIO

// Somebody else's code, held to its own standard rather than ours. The viewer
// builds at /W4 and is warning-clean; stb_image is a single 8000-line header
// written to compile everywhere since 2010 and it is not going to be /W4 clean
// on MSVC. Silencing it AT THE INCLUDE is what keeps "zero warnings" a fact
// about our code instead of a number nobody trusts - the alternative is
// lowering the warning level for the whole file, which would hide ours too.
#pragma warning(push, 0)
#include "stb_image.h"
#pragma warning(pop)

namespace jpeg
{

  namespace
  {

    // stb reports its refusals through a global set by the last failing call.
    // It can be null, and a null there would otherwise become a Str
    // construction from nullptr - which is undefined behaviour reached only on
    // the error path, so it would survive every test that passes.
    [[nodiscard]] Str reasonOr(CharSeq fallback)
    {
        CharSeq why = ::stbi_failure_reason();
        return Str(why == nullptr ? fallback : why);
    }

  }

  Bool decode(const UInt8* bytes, Size len, Picture* out, Str* why)
  {
      if(out == nullptr || why == nullptr)
      {
          return false;
      }
      *out = Picture();
      if(bytes == nullptr || len == 0)
      {
          *why = "no bytes to decode";
          return false;
      }

      // int, because that is stb's signature. The cast is safe by the bound
      // above it and not by hope: MAX_PAYLOAD is 256 KiB, so a length that
      // could overflow an Int32 cannot have got this far through the codec.
      const Int32 size = static_cast<Int32>(len);

      // THE DIMENSIONS FIRST, AND NOTHING ALLOCATED YET. stbi_info reads the
      // header only, so a frame claiming 30000x30000 is refused here rather
      // than after the decoder has tried to find 3.6 GB for it.
      Int32 width = 0;
      Int32 height = 0;
      Int32 channels = 0;
      if(::stbi_info_from_memory(bytes, size, &width, &height, &channels) == 0)
      {
          *why = reasonOr("not a picture this build can read");
          return false;
      }
      if(width <= 0 || height <= 0)
      {
          *why = "the frame claims an empty picture";
          return false;
      }
      if(width > MAX_SIDE || height > MAX_SIDE)
      {
          Array<Char, 96> t = {};
          std::snprintf(
              t.data(),
              t.size(),
              "the frame claims %dx%d, larger than this viewer will decode",
              width,
              height
          );
          *why = Str(t.data());
          return false;
      }

      // reqComp 4: RGBA whatever the file holds, so the upload below is one
      // shape and never a branch on what the camera happened to send.
      Int32 gotW = 0;
      Int32 gotH = 0;
      Int32 gotChannels = 0;
      stbi_uc* pixels = ::stbi_load_from_memory(bytes, size, &gotW, &gotH, &gotChannels, 4);
      if(pixels == nullptr)
      {
          *why = reasonOr("the picture would not decode");
          return false;
      }

      // The header said one thing and the decoder produced another. That is
      // not a picture with a cosmetic discrepancy, it is a frame whose own two
      // accounts of itself disagree, and the bytes below are sized from one of
      // them.
      if(gotW != width || gotH != height)
      {
          ::stbi_image_free(pixels);
          *why = "the picture's header and its pixels disagree about its size";
          return false;
      }

      const Size count = static_cast<Size>(gotW) * static_cast<Size>(gotH) * 4u;
      out->width = gotW;
      out->height = gotH;
      out->rgba.assign(pixels, pixels + count);
      ::stbi_image_free(pixels);
      return true;
  }

}
