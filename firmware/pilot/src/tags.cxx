#include "tags.hxx"

#include <cstdio>

#if defined(__linux__) && defined(PILOT_HAVE_APRILTAG)

#include "viewfeed.hxx"

#include <algorithm>
#include <cmath>
#include <csetjmp>
#include <cstring>

#include <pthread.h>
#include <sched.h>
#include <unistd.h>

#include <jpeglib.h>

#include <apriltag.h>
#include <common/image_u8.h>
#include <tag36h11.h>

namespace tags
{
  namespace
  {
    // How long the thread waits for a frame before looking at its quit flag.
    constexpr Int32 WAIT_MS = 200;

    // libjpeg's default error_exit calls exit(). A truncated frame off a USB
    // camera is ordinary, so the exit is turned into a longjmp back into
    // decode(), which is the library's own documented way out.
    struct JpegError
    {
        jpeg_error_mgr pub;
        jmp_buf jump;
        Array<Char, JMSG_LENGTH_MAX> text = {};
    };

    Void onJpegError(j_common_ptr info)
    {
        JpegError* e = reinterpret_cast<JpegError*>(info->err);
        (*info->err->format_message)(info, e->text.data());
        longjmp(e->jump, 1);
    }

    Void onJpegMessage(j_common_ptr info)
    {
        // Warnings are not printed: a frame that decodes is a frame.
        static_cast<Void>(info);
    }

    // The JPEG as one grey plane in the library's own image, or null with why.
    [[nodiscard]] image_u8_t* decode(const UInt8* jpeg, Size len, Str& why)
    {
        jpeg_decompress_struct info;
        JpegError err;
        info.err = jpeg_std_error(&err.pub);
        err.pub.error_exit = onJpegError;
        err.pub.output_message = onJpegMessage;
        image_u8_t* im = nullptr;
        if(setjmp(err.jump) != 0)
        {
            jpeg_destroy_decompress(&info);
            if(im != nullptr)
            {
                image_u8_destroy(im);
            }
            why = err.text.data();
            return nullptr;
        }
        jpeg_create_decompress(&info);
        jpeg_mem_src(&info, jpeg, static_cast<unsigned long>(len));
        if(jpeg_read_header(&info, TRUE) != JPEG_HEADER_OK)
        {
            jpeg_destroy_decompress(&info);
            why = "not a JPEG header";
            return nullptr;
        }
        info.out_color_space = JCS_GRAYSCALE;
        info.dct_method = JDCT_FASTEST;
        info.do_fancy_upsampling = FALSE;
        static_cast<Void>(jpeg_start_decompress(&info));
        if(info.output_width == 0u || info.output_height == 0u || info.output_width > 0xFFFFu
           || info.output_height > 0xFFFFu)
        {
            jpeg_destroy_decompress(&info);
            why = "a picture of no size";
            return nullptr;
        }
        im = image_u8_create(info.output_width, info.output_height);
        while(info.output_scanline < info.output_height)
        {
            const Size line = static_cast<Size>(info.output_scanline) * static_cast<Size>(im->stride);
            UInt8* row = im->buf + line;
            static_cast<Void>(jpeg_read_scanlines(&info, &row, 1));
        }
        static_cast<Void>(jpeg_finish_decompress(&info));
        jpeg_destroy_decompress(&info);
        return im;
    }

    [[nodiscard]] Int16 deci(Float64 px)
    {
        const Float64 v = static_cast<Float64>(std::lround(px * 10.0));
        return static_cast<Int16>(std::clamp(v, -32768.0, 32767.0));
    }

    // One detector, reconfigured per call and used by one thread at a time:
    // the library's detect is not reentrant on one detector.
    Mutex detM;
    apriltag_detector_t* det = nullptr;
    apriltag_family_t* fam = nullptr;

    [[nodiscard]] apriltag_detector_t* detector(const Config& cfg)
    {
        if(det == nullptr)
        {
            fam = tag36h11_create();
            det = apriltag_detector_create();
            apriltag_detector_add_family(det, fam);
            det->quad_sigma = 0.0f;
            det->refine_edges = 1;
        }
        det->quad_decimate = cfg.decimate >= 1.0f ? cfg.decimate : 1.0f;
        det->nthreads = cfg.threads >= 1 ? cfg.threads : 1;
        return det;
    }

    // The big cores, by the kernel's own cpu_capacity: on this board 1024 on
    // the two A76s against 385 on the A55s, and the scheduler left to itself
    // put the detector on an A55 at three times the cost. Pinned to every
    // cpu at the highest capacity when there is more than one capacity;
    // a board with one kind of core is left alone. In words, for stats().
    [[nodiscard]] Str pinToBigCores()
    {
        const long count = ::sysconf(_SC_NPROCESSORS_CONF);
        Vec<Int64> capacity;
        Int64 best = -1;
        for(long cpu = 0; cpu < count && cpu < CPU_SETSIZE; ++cpu)
        {
            Array<Char, 96> path = {};
            std::snprintf(
                path.data(),
                path.size(),
                "/sys/devices/system/cpu/cpu%ld/cpu_capacity",
                cpu
            );
            std::FILE* f = std::fopen(path.data(), "r");
            long long value = -1;
            if(f != nullptr)
            {
                if(std::fscanf(f, "%lld", &value) != 1)
                {
                    value = -1;
                }
                std::fclose(f);
            }
            capacity.push_back(static_cast<Int64>(value));
            best = std::max(best, static_cast<Int64>(value));
        }
        Bool mixed = false;
        for(Int64 c : capacity)
        {
            mixed = mixed || (c >= 0 && c != best);
        }
        if(best < 0 || !mixed)
        {
            return "any cpu (no capacity mix reported)";
        }
        cpu_set_t set;
        CPU_ZERO(&set);
        Str names;
        for(Size cpu = 0; cpu < capacity.size(); ++cpu)
        {
            if(capacity[cpu] == best)
            {
                CPU_SET(cpu, &set);
                names += (names.empty() ? "" : ",") + std::to_string(cpu);
            }
        }
        if(::pthread_setaffinity_np(::pthread_self(), sizeof(set), &set) != 0)
        {
            return "any cpu (pinning to " + names + " refused)";
        }
        return "cpu " + names + " (capacity " + std::to_string(best) + ")";
    }

    Config config;
    Atomic<Bool> quit{ false };
    Atomic<Bool> live{ false };
    Thread worker;
    Mutex statM;
    Stats stat;

    Void loop()
    {
        const Str cores = pinToBigCores();
        std::printf("apriltag: detector thread on %s\n", cores.c_str());
        {
            LockGuard<Mutex> lock(statM);
            stat.cores = cores;
        }
        UInt32 seen = 0;
        viewfeed::CameraFrame f;
        while(!quit.load())
        {
            if(!viewfeed::waitCameraFrame(&seen, &f, WAIT_MS))
            {
                continue;
            }
            bibowire::Tags t;
            Str why;
            if(!find(f.jpeg, config, &t, why))
            {
                LockGuard<Mutex> lock(statM);
                ++stat.undecodable;
                stat.why = why;
                continue;
            }
            t.tMonoUs = f.tMonoUs;
            t.frameIndex = f.frameIndex;
            viewfeed::publishTags(t);
            LockGuard<Mutex> lock(statM);
            ++stat.frames;
            stat.seen += t.tags.size();
            stat.lastDetectUs = t.detectUs;
            stat.lastCount = static_cast<UInt32>(t.tags.size());
        }
    }
  }

  Bool available()
  {
      return true;
  }

  Bool find(const Vec<UInt8>& pic, const Config& cfg, bibowire::Tags* out, Str& why)
  {
      if(pic.empty() || out == nullptr)
      {
          why = "no picture";
          return false;
      }
      image_u8_t* im = decode(pic.data(), pic.size(), why);
      if(im == nullptr)
      {
          return false;
      }
      bibowire::Tags t;
      t.width = static_cast<UInt16>(im->width);
      t.height = static_cast<UInt16>(im->height);
      t.family = bibowire::TAG_FAMILY_36H11;
      {
          LockGuard<Mutex> lock(detM);
          const TimePoint before = monoNow();
          zarray_t* found = apriltag_detector_detect(detector(cfg), im);
          const Float64 us = elapsedMs(before) * 1000.0;
          t.detectUs = us > 0.0 ? static_cast<UInt32>(us) : 0u;
          const Size n = static_cast<Size>(zarray_size(found));
          for(Size i = 0; i < n && i < bibowire::MAX_TAGS; ++i)
          {
              apriltag_detection_t* d = nullptr;
              zarray_get(found, static_cast<int>(i), &d);
              bibowire::Tag one;
              one.id = static_cast<UInt16>(std::clamp(d->id, 0, 0xFFFF));
              one.hamming = static_cast<UInt8>(std::clamp(d->hamming, 0, 255));
              const Float64 margin = static_cast<Float64>(d->decision_margin) * 1000.0;
              const Float64 milli = static_cast<Float64>(std::lround(margin));
              one.marginMilli = static_cast<Int32>(std::clamp(milli, -2147483648.0, 2147483647.0));
              for(Size c = 0; c < 4u; ++c)
              {
                  one.corners[c].xDeci = deci(d->p[c][0]);
                  one.corners[c].yDeci = deci(d->p[c][1]);
              }
              t.tags.push_back(one);
          }
          apriltag_detections_destroy(found);
      }
      image_u8_destroy(im);
      *out = t;
      return true;
  }

  Bool start(const Config& cfg, Str& why)
  {
      if(live.load())
      {
          why = "the detector is already running";
          return false;
      }
      config = cfg;
      quit.store(false);
      {
          LockGuard<Mutex> lock(statM);
          stat = Stats();
          stat.built = true;
          stat.running = true;
      }
      // Subscribed before the thread starts, so its first wait can be answered.
      viewfeed::wantCamera(true, cfg.fps);
      worker = Thread(loop);
      live.store(true);
      return true;
  }

  Bool running()
  {
      return live.load();
  }

  Stats stats()
  {
      LockGuard<Mutex> lock(statM);
      Stats s = stat;
      s.built = true;
      s.running = live.load();
      return s;
  }

  Void stop()
  {
      if(!live.load())
      {
          return;
      }
      quit.store(true);
      worker.join();
      live.store(false);
      viewfeed::wantCamera(false, 0);
      LockGuard<Mutex> lock(statM);
      stat.running = false;
  }
}

#else

// No detector in this build: every call refuses, saying so.
namespace tags
{
  namespace
  {
    constexpr CharSeq NOT_BUILT = "no AprilTag detector in this build (configure with PILOT_APRILTAG)";
  }

  Bool available()
  {
      return false;
  }

  Bool find(const Vec<UInt8>& pic, const Config& cfg, bibowire::Tags* out, Str& why)
  {
      static_cast<Void>(pic);
      static_cast<Void>(cfg);
      static_cast<Void>(out);
      why = NOT_BUILT;
      return false;
  }

  Bool start(const Config& cfg, Str& why)
  {
      static_cast<Void>(cfg);
      why = NOT_BUILT;
      std::printf("tags: %s\n", NOT_BUILT);
      return false;
  }

  Bool running()
  {
      return false;
  }

  Stats stats()
  {
      Stats s;
      s.why = NOT_BUILT;
      return s;
  }

  Void stop()
  {
  }
}

#endif
