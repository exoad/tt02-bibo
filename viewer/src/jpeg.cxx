#include "jpeg.hxx"

// The only translation unit that compiles stb_image. STBI_ONLY_JPEG is the
// attack-surface limit described in jpeg.hxx, not size tuning.
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_NO_STDIO

// Third-party code is not /W4 clean. Silence it at the include so the viewer's
// zero-warnings rule still covers our own code in this file.
#pragma warning(push, 0)
#include "stb_image.h"
#pragma warning(pop)

namespace jpeg
{
  namespace
  {
    // stbi_failure_reason() can be null, and Str(nullptr) is undefined behaviour.
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
      // Safe: MAX_PAYLOAD bounds a frame far below Int32's range.
      const Int32 size = static_cast<Int32>(len);
      // Header only, nothing allocated yet: MAX_SIDE is enforced here.
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
      // reqComp 4: always RGBA, whatever the file holds.
      Int32 gotW = 0;
      Int32 gotH = 0;
      Int32 gotChannels = 0;
      stbi_uc* pixels = ::stbi_load_from_memory(bytes, size, &gotW, &gotH, &gotChannels, 4);
      if(pixels == nullptr)
      {
          *why = reasonOr("the picture would not decode");
          return false;
      }
      // The copy below is sized from the decoder's answer, so a header that
      // disagrees with it is refused.
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
