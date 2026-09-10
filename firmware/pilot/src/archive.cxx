#include "archive.hxx"

#include <cstdio>
#include <cstring>

namespace archive
{

  // The footer is a fixed layout on disk and every offset below is spelled once
  // here, so a field that moves without these moving does not compile. 148 is a
  // multiple of 4 for the same reason every bibowire payload is: nothing on this
  // disk is read through an unaligned struct, but a size that is not a multiple
  // of 4 is a size somebody will eventually pad by hand.
  namespace
  {

    constexpr Size OFF_MAGIC = 0;
    constexpr Size OFF_VERSION = 4;
    constexpr Size OFF_RESERVED0 = 5;
    constexpr Size OFF_RESERVED1 = 6;
    constexpr Size OFF_WALL = 8;
    constexpr Size OFF_FIRST_MONO = 16;
    constexpr Size OFF_FRAME_COUNT = 24;
    constexpr Size OFF_FRAME_BYTES = 32;
    constexpr Size OFF_INDEX_COUNT = 40;
    constexpr Size OFF_TYPE_COUNTS = 48;
    constexpr Size OFF_FILE_BYTES = 136;
    constexpr Size OFF_CRC = 144;

    static_assert(
        OFF_TYPE_COUNTS + 4u * bibowire::TYPE_COUNT == OFF_FILE_BYTES,
        "the per-type counts must end exactly where fileBytes begins"
    );
    static_assert(OFF_CRC + 4u == FOOTER_BYTES, "the CRC is the last four bytes of the footer");
    static_assert((FOOTER_BYTES % 4u) == 0u, "the footer is a multiple of 4 bytes");
    static_assert((ENTRY_BYTES % 4u) == 0u, "an index entry is a multiple of 4 bytes");

    // The footer magic must not be the frame magic. The fallback reader scans
    // this same file for the pair 0x42 0x57, so a footer spelling that pair
    // would be handed to take() as a frame candidate on every scan.
    static_assert(
        (MAGIC & 0xFFFFu) != bibowire::MAGIC,
        "the footer magic must not begin with the two bytes take() scans for"
    );

    // ---- index entries -----------------------------------------------------

    Void packEntry(const Entry& e, UInt8* out)
    {
        bibowire::wr64(out, e.offset);
        bibowire::wr32(out + 8u, e.frameLen);
        bibowire::wr8(out + 12u, e.type);
        bibowire::wr8(out + 13u, e.ver);
        bibowire::wr16(out + 14u, e.seq);
    }

    Void unpackEntry(const UInt8* in, Entry* out)
    {
        out->offset = bibowire::rd64(in);
        out->frameLen = bibowire::rd32(in + 8u);
        out->type = bibowire::rd8(in + 12u);
        out->ver = bibowire::rd8(in + 13u);
        out->seq = bibowire::rd16(in + 14u);
    }

    // ---- the one funnel every read goes through ----------------------------
    //
    // The clamp lives HERE and nowhere else, so "never read past the end" is one
    // line that can be checked rather than a discipline spread over six call
    // sites. A source's own read() is therefore only ever called with arguments
    // already inside its total.
    [[nodiscard]] Size readAt(const Bytes& b, UInt64 at, UInt8* out, Size want)
    {
        if(b.read == nullptr || out == nullptr || at >= b.total)
        {
            return 0;
        }
        const UInt64 avail = b.total - at;
        Size n = want;
        if(static_cast<UInt64>(n) > avail)
        {
            n = static_cast<Size>(avail);
        }
        if(n == 0u)
        {
            return 0;
        }
        return b.read(b.ctx, at, out, n);
    }

    // ---- the memory source and sink ----------------------------------------

    Size memRead(const Void* ctx, UInt64 at, UInt8* out, Size want)
    {
        if(ctx == nullptr)
        {
            return 0;
        }
        const UInt8* base = static_cast<const UInt8*>(ctx);
        std::memcpy(out, base + at, want);
        return want;
    }

    Bool memWrite(Void* ctx, const UInt8* data, Size len)
    {
        Vec<UInt8>* v = static_cast<Vec<UInt8>*>(ctx);
        if(v == nullptr || (len > 0u && data == nullptr))
        {
            return false;
        }
        v->insert(v->end(), data, data + len);
        return true;
    }

    Bool memFlush(Void* ctx)
    {
        // Memory is already durable in the only sense this sink has. Named and
        // returning true rather than left null, so the writer's "a sink without
        // a flush is not a sink" check stays a real check.
        static_cast<Void>(ctx);
        return true;
    }

    // ---- the file source and sink ------------------------------------------
    //
    // std::fseek and std::ftell take a `long`, which is 64-bit on the board
    // (aarch64 Linux) and 32-bit on MSVC - so on Windows alone they would cap an
    // archive at 2 GiB, silently, at exactly the size a camera run reaches. The
    // MSVC-only branch below is what removes that; there is no branch for the
    // board because none is needed there.

    [[nodiscard]] Bool seekTo(std::FILE* fh, UInt64 at)
    {
#ifdef _MSC_VER
        return _fseeki64(fh, static_cast<Int64>(at), SEEK_SET) == 0;
#else
        return std::fseek(fh, static_cast<long>(at), SEEK_SET) == 0;
#endif
    }

    [[nodiscard]] Bool sizeOfFile(std::FILE* fh, UInt64* out)
    {
#ifdef _MSC_VER
        if(_fseeki64(fh, 0, SEEK_END) != 0)
        {
            return false;
        }
        const Int64 n = _ftelli64(fh);
#else
        if(std::fseek(fh, 0, SEEK_END) != 0)
        {
            return false;
        }
        const Int64 n = static_cast<Int64>(std::ftell(fh));
#endif
        if(n < 0)
        {
            return false;
        }
        *out = static_cast<UInt64>(n);
        return seekTo(fh, 0);
    }

    Size fileRead(const Void* ctx, UInt64 at, UInt8* out, Size want)
    {
        if(ctx == nullptr)
        {
            return 0;
        }
        // The handle is mutable state behind a const source: reading a file
        // moves its cursor, which is exactly the detail `Bytes` exists to hide.
        std::FILE* fh = static_cast<std::FILE*>(const_cast<Void*>(ctx));
        if(!seekTo(fh, at))
        {
            return 0;
        }
        return std::fread(out, 1, want, fh);
    }

    Bool fileWrite(Void* ctx, const UInt8* data, Size len)
    {
        std::FILE* fh = static_cast<std::FILE*>(ctx);
        if(fh == nullptr)
        {
            return false;
        }
        if(len == 0u)
        {
            return true;
        }
        return std::fwrite(data, 1, len, fh) == len;
    }

    Bool fileFlush(Void* ctx)
    {
        std::FILE* fh = static_cast<std::FILE*>(ctx);
        if(fh == nullptr)
        {
            return false;
        }
        return std::fflush(fh) == 0;
    }

    // ---- footer arithmetic -------------------------------------------------
    //
    // A CRC is a check against CORRUPTION, not against a lie. These are the
    // checks that ask whether the numbers could describe the file actually in
    // front of the reader, and they are why an absurd count or an offset past
    // EOF falls back rather than being believed and acted on.
    [[nodiscard]] Bool footerSane(const Footer& f, UInt64 total)
    {
        if(f.version != VERSION)
        {
            return false;
        }
        if(f.frameCount != f.indexCount)
        {
            return false;
        }
        if(f.fileBytes != total)
        {
            return false;
        }
        if(total < FOOTER_BYTES)
        {
            return false;
        }
        // Before multiplying: an indexCount near 2^64 would otherwise wrap into
        // a small, plausible-looking product.
        if(f.indexCount > (total / ENTRY_BYTES))
        {
            return false;
        }
        const UInt64 idxBytes = f.indexCount * ENTRY_BYTES;
        if(f.frameBytes > total)
        {
            return false;
        }
        if(idxBytes > total - f.frameBytes)
        {
            return false;
        }
        return f.frameBytes + idxBytes + FOOTER_BYTES == total;
    }

  }

  // ---- the small public helpers ----------------------------------------------

  Bytes overMemory(const UInt8* data, Size len)
  {
      Bytes b;
      b.ctx = data;
      b.total = static_cast<UInt64>(len);
      b.read = &memRead;
      return b;
  }

  Sink intoMemory(Vec<UInt8>* out)
  {
      Sink s;
      s.ctx = out;
      s.write = &memWrite;
      s.flush = &memFlush;
      return s;
  }

  Bool openWrite(const Str& path, File* out)
  {
      if(out == nullptr)
      {
          return false;
      }
      std::FILE* fh = std::fopen(path.c_str(), "wb");
      if(fh == nullptr)
      {
          return false;
      }
      out->handle = fh;
      out->total = 0;
      return true;
  }

  Bool openRead(const Str& path, File* out)
  {
      if(out == nullptr)
      {
          return false;
      }
      std::FILE* fh = std::fopen(path.c_str(), "rb");
      if(fh == nullptr)
      {
          return false;
      }
      UInt64 total = 0;
      if(!sizeOfFile(fh, &total))
      {
          std::fclose(fh);
          return false;
      }
      out->handle = fh;
      out->total = total;
      return true;
  }

  Void closeFile(File* f)
  {
      if(f == nullptr || f->handle == nullptr)
      {
          return;
      }
      std::fclose(static_cast<std::FILE*>(f->handle));
      f->handle = nullptr;
      f->total = 0;
  }

  Sink sinkOf(File* f)
  {
      Sink s;
      if(f == nullptr)
      {
          return s;
      }
      s.ctx = f->handle;
      s.write = &fileWrite;
      s.flush = &fileFlush;
      return s;
  }

  Bytes bytesOf(const File* f)
  {
      Bytes b;
      if(f == nullptr)
      {
          return b;
      }
      b.ctx = f->handle;
      b.total = f->total;
      b.read = &fileRead;
      return b;
  }

  Size typeSlot(UInt8 tag)
  {
      const bibowire::Desc* rows = bibowire::catalog();
      const Size n = bibowire::catalogCount();
      for(Size i = 0; i < n; ++i)
      {
          if(static_cast<UInt8>(rows[i].type) == tag)
          {
              return i;
          }
      }
      return bibowire::TYPE_COUNT;
  }

  Bool monoOf(bibowire::Type t, const bibowire::Body& b, UInt64* out)
  {
      if(out == nullptr || b.bytes == nullptr)
      {
          return false;
      }
      // WHERE the clock sits, per type, read off the layouts in docs/bibowire.md
      // section 5. A table rather than an assumption that every body starts with
      // one: CONTROL carries the VIEWER's clock at offset 8 and WELCOME the
      // board's at 28, and a reader that took offset 0 for all of them would
      // silently timestamp a run with a session id.
      Size at = 0;
      switch(t)
      {
          case bibowire::Type::TYPE_SCAN:
          case bibowire::Type::TYPE_BOARD:
          case bibowire::Type::TYPE_EVENT:
          case bibowire::Type::TYPE_CTLSTATE:
          case bibowire::Type::TYPE_CAMERA:
          case bibowire::Type::TYPE_POSE:
          case bibowire::Type::TYPE_PATH:
          case bibowire::Type::TYPE_WAYPOINT:
              at = 0;
              break;
          case bibowire::Type::TYPE_PING:
          case bibowire::Type::TYPE_PONG:
          case bibowire::Type::TYPE_CONTROL:
              at = 8;
              break;
          case bibowire::Type::TYPE_WELCOME:
              at = 28;
              break;
          default:
              // HELLO, BYE, LEAVE, DECIDE, LIDAR_INFO, CMDACK, COMMAND,
              // SUBSCRIBE, DESCRIBE and SCHEMA carry no clock at all. Returning
              // a zero for them would put a fabricated timestamp in a recording,
              // which is bibowire's absent-sentinel argument applied to a file.
              return false;
      }
      if(b.len < at + 8u)
      {
          return false;
      }
      *out = bibowire::rd64(b.bytes + at);
      return true;
  }

  UInt64 wallUsOf(const Footer& f, UInt64 monoUs)
  {
      if(monoUs >= f.firstMonoUs)
      {
          return f.wallEpochUs + (monoUs - f.firstMonoUs);
      }
      // A frame stamped BEFORE the pairing instant maps backwards. Done on the
      // unsigned values so nothing has to negate the most negative Int64, and
      // clamped at the epoch so a wild timestamp cannot wrap into the year
      // 586524 and be printed as if it meant something.
      const UInt64 back = f.firstMonoUs - monoUs;
      if(back > f.wallEpochUs)
      {
          return 0;
      }
      return f.wallEpochUs - back;
  }

  Void packFooter(const Footer& f, UInt8* out)
  {
      if(out == nullptr)
      {
          return;
      }
      std::memset(out, 0, FOOTER_BYTES);
      bibowire::wr32(out + OFF_MAGIC, MAGIC);
      bibowire::wr8(out + OFF_VERSION, f.version);
      bibowire::wr8(out + OFF_RESERVED0, 0);
      bibowire::wr16(out + OFF_RESERVED1, 0);
      bibowire::wr64(out + OFF_WALL, f.wallEpochUs);
      bibowire::wr64(out + OFF_FIRST_MONO, f.firstMonoUs);
      bibowire::wr64(out + OFF_FRAME_COUNT, f.frameCount);
      bibowire::wr64(out + OFF_FRAME_BYTES, f.frameBytes);
      bibowire::wr64(out + OFF_INDEX_COUNT, f.indexCount);
      for(Size i = 0; i < bibowire::TYPE_COUNT; ++i)
      {
          bibowire::wr32(out + OFF_TYPE_COUNTS + i * 4u, f.typeCounts[i]);
      }
      bibowire::wr64(out + OFF_FILE_BYTES, f.fileBytes);
      bibowire::wr32(out + OFF_CRC, bibowire::crc32c(out, OFF_CRC));
  }

  Bool unpackFooter(const UInt8* in, Footer* out)
  {
      if(in == nullptr || out == nullptr)
      {
          return false;
      }
      if(bibowire::rd32(in + OFF_MAGIC) != MAGIC)
      {
          return false;
      }
      if(bibowire::crc32c(in, OFF_CRC) != bibowire::rd32(in + OFF_CRC))
      {
          return false;
      }
      Footer f;
      f.version = bibowire::rd8(in + OFF_VERSION);
      f.wallEpochUs = bibowire::rd64(in + OFF_WALL);
      f.firstMonoUs = bibowire::rd64(in + OFF_FIRST_MONO);
      f.frameCount = bibowire::rd64(in + OFF_FRAME_COUNT);
      f.frameBytes = bibowire::rd64(in + OFF_FRAME_BYTES);
      f.indexCount = bibowire::rd64(in + OFF_INDEX_COUNT);
      for(Size i = 0; i < bibowire::TYPE_COUNT; ++i)
      {
          f.typeCounts[i] = bibowire::rd32(in + OFF_TYPE_COUNTS + i * 4u);
      }
      f.fileBytes = bibowire::rd64(in + OFF_FILE_BYTES);
      *out = f;
      return true;
  }

  CharSeq sourceName(Source s)
  {
      switch(s)
      {
          case Source::SOURCE_NONE:
              return "none";
          case Source::SOURCE_INDEX:
              return "index";
          case Source::SOURCE_SCAN:
              return "scan";
          default:
              return "?";
      }
  }

  CharSeq fallbackName(Fallback f)
  {
      switch(f)
      {
          case Fallback::FALLBACK_NONE:
              return "none";
          case Fallback::FALLBACK_NO_FOOTER:
              return "no_footer";
          case Fallback::FALLBACK_BAD_CRC:
              return "bad_crc";
          case Fallback::FALLBACK_BAD_VERSION:
              return "bad_version";
          case Fallback::FALLBACK_INSANE:
              return "insane";
          case Fallback::FALLBACK_BAD_INDEX:
              return "bad_index";
          default:
              return "?";
      }
  }

  // ---- writing ---------------------------------------------------------------

  Bool Writer::begin(const Sink& s, UInt64 wallUs)
  {
      // A sink missing either half is refused here rather than at the first
      // frame: a recorder that discovers it cannot flush after an hour of
      // driving has already lost the hour.
      if(s.write == nullptr || s.flush == nullptr)
      {
          return false;
      }
      sink = s;
      wallEpochUs = wallUs;
      firstMonoUs = 0;
      frameCount = 0;
      frameBytes = 0;
      typeCounts = Array<UInt32, bibowire::TYPE_COUNT>{};
      index.clear();
      started = true;
      haveMono = false;
      failed = false;
      return true;
  }

  Bool Writer::put(const UInt8* frame, Size len)
  {
      if(!started || failed || frame == nullptr)
      {
          return false;
      }
      if(len < bibowire::FRAME_OVERHEAD || len > MAX_FRAME_BYTES)
      {
          return false;
      }
      // The bytes must be EXACTLY one whole frame, checked by the same decoder
      // that will read them back. A recorder that appends whatever it is handed
      // makes its own corruption indistinguishable from a power cut, and those
      // two want opposite responses from a person holding the car.
      bibowire::Frame f;
      Size used = 0;
      if(bibowire::take(frame, len, &f, &used) != bibowire::Take::TAKE_FRAME)
      {
          return false;
      }
      if(used != len)
      {
          return false;
      }
      if(!sink.write(sink.ctx, frame, len))
      {
          failed = true;
          return false;
      }
      // Flushed EVERY frame, on purpose. The difference between losing one frame
      // to a pulled battery and losing the buffered tail of an outing is this
      // call, and an outing is not repeatable.
      if(!sink.flush(sink.ctx))
      {
          failed = true;
          return false;
      }
      Entry e;
      e.offset = frameBytes;
      e.frameLen = static_cast<UInt32>(len);
      e.type = static_cast<UInt8>(f.head.type);
      e.ver = f.head.ver;
      e.seq = f.head.seq;
      index.push_back(e);
      frameBytes += static_cast<UInt64>(len);
      ++frameCount;
      const Size slot = typeSlot(e.type);
      if(slot < bibowire::TYPE_COUNT && typeCounts[slot] != 0xFFFFFFFFu)
      {
          ++typeCounts[slot];
      }
      if(!haveMono)
      {
          UInt64 mono = 0;
          if(monoOf(f.head.type, f.body, &mono))
          {
              firstMonoUs = mono;
              haveMono = true;
          }
      }
      return true;
  }

  Bool Writer::finish()
  {
      if(!started || failed)
      {
          return false;
      }
      Array<UInt8, ENTRY_BYTES> row{};
      for(const Entry& e : index)
      {
          packEntry(e, row.data());
          if(!sink.write(sink.ctx, row.data(), ENTRY_BYTES))
          {
              failed = true;
              return false;
          }
      }
      Footer f;
      f.version = VERSION;
      f.wallEpochUs = wallEpochUs;
      f.firstMonoUs = firstMonoUs;
      f.frameCount = frameCount;
      f.frameBytes = frameBytes;
      f.indexCount = static_cast<UInt64>(index.size());
      f.typeCounts = typeCounts;
      f.fileBytes = frameBytes + f.indexCount * ENTRY_BYTES + FOOTER_BYTES;
      Array<UInt8, FOOTER_BYTES> raw{};
      packFooter(f, raw.data());
      if(!sink.write(sink.ctx, raw.data(), FOOTER_BYTES))
      {
          failed = true;
          return false;
      }
      if(!sink.flush(sink.ctx))
      {
          failed = true;
          return false;
      }
      started = false;
      return true;
  }

  Void Writer::abandon()
  {
      // What a pulled battery does, on purpose and repeatably. Everything
      // already written stays written; no index, no footer.
      started = false;
  }

  // ---- reading ---------------------------------------------------------------

  namespace
  {

    Void fillRecord(Record* out, const bibowire::Frame& f, UInt64 off, Size len)
    {
        out->head = f.head;
        out->body.assign(f.body.bytes, f.body.bytes + f.body.len);
        out->offset = off;
        out->frameLen = len;
    }

    Void resetScan(Reader* r, UInt64 from)
    {
        r->winFill = 0;
        r->winAt = 0;
        r->winBase = from;
        r->srcAt = from;
    }

    [[nodiscard]] Bool loadAt(Reader* r, UInt64 off, Size len, Record* out)
    {
        if(out == nullptr || len < bibowire::FRAME_OVERHEAD || len > MAX_FRAME_BYTES)
        {
            return false;
        }
        if(off > r->src.total || (r->src.total - off) < static_cast<UInt64>(len))
        {
            return false;
        }
        r->scratch.resize(len);
        if(readAt(r->src, off, r->scratch.data(), len) != len)
        {
            return false;
        }
        bibowire::Frame f;
        Size used = 0;
        if(bibowire::take(r->scratch.data(), len, &f, &used) != bibowire::Take::TAKE_FRAME)
        {
            return false;
        }
        if(used != len)
        {
            return false;
        }
        fillRecord(out, f, off, len);
        return true;
    }

    // Does the index describe a CONTIGUOUS TILING of the frame region - first
    // entry at zero, each one starting where the last ended, the last ending
    // exactly at frameBytes? Anything else is not an index of this file, whatever
    // its CRC says. Checked without reading a single payload, so the cost is 16
    // bytes per frame once rather than a pass over the whole run.
    [[nodiscard]] Bool indexTiles(Reader* r, const Footer& f)
    {
        Array<UInt8, ENTRY_BYTES * 64u> blk{};
        UInt64 want = 0;
        UInt64 i = 0;
        while(i < f.indexCount)
        {
            UInt64 n = f.indexCount - i;
            if(n > 64u)
            {
                n = 64u;
            }
            const UInt64 at = f.frameBytes + i * ENTRY_BYTES;
            const Size bytes = static_cast<Size>(n) * ENTRY_BYTES;
            if(readAt(r->src, at, blk.data(), bytes) != bytes)
            {
                return false;
            }
            for(UInt64 k = 0; k < n; ++k)
            {
                Entry e;
                unpackEntry(blk.data() + static_cast<Size>(k) * ENTRY_BYTES, &e);
                if(e.offset != want)
                {
                    return false;
                }
                if(e.frameLen < bibowire::FRAME_OVERHEAD || e.frameLen > MAX_FRAME_BYTES)
                {
                    return false;
                }
                if(((e.frameLen - bibowire::FRAME_OVERHEAD) % 4u) != 0u)
                {
                    return false;
                }
                want += e.frameLen;
                if(want > f.frameBytes)
                {
                    return false;
                }
            }
            i += n;
        }
        return want == f.frameBytes;
    }

    // Every way a footer can fail to earn trust, each with its own name. The
    // order matters: "there is no footer" and "the footer is lying" send a
    // person to two different places, so they are never collapsed into one word.
    [[nodiscard]] Bool loadFooter(Reader* r)
    {
        if(r->src.total < FOOTER_BYTES)
        {
            r->fallback = Fallback::FALLBACK_NO_FOOTER;
            return false;
        }
        Array<UInt8, FOOTER_BYTES> raw{};
        const UInt64 at = r->src.total - FOOTER_BYTES;
        if(readAt(r->src, at, raw.data(), FOOTER_BYTES) != FOOTER_BYTES)
        {
            r->fallback = Fallback::FALLBACK_NO_FOOTER;
            return false;
        }
        if(bibowire::rd32(raw.data()) != MAGIC)
        {
            r->fallback = Fallback::FALLBACK_NO_FOOTER;
            return false;
        }
        Footer f;
        if(!unpackFooter(raw.data(), &f))
        {
            r->fallback = Fallback::FALLBACK_BAD_CRC;
            return false;
        }
        if(f.version != VERSION)
        {
            r->fallback = Fallback::FALLBACK_BAD_VERSION;
            return false;
        }
        if(!footerSane(f, r->src.total))
        {
            r->fallback = Fallback::FALLBACK_INSANE;
            return false;
        }
        if(!indexTiles(r, f))
        {
            r->fallback = Fallback::FALLBACK_BAD_INDEX;
            return false;
        }
        r->foot = f;
        return true;
    }

    [[nodiscard]] Bool refill(Reader* r)
    {
        if(r->srcAt >= r->src.total)
        {
            return false;
        }
        if(r->winAt > 0u)
        {
            const Size keep = r->winFill - r->winAt;
            if(keep > 0u)
            {
                std::memmove(r->win.data(), r->win.data() + r->winAt, keep);
            }
            r->winBase += r->winAt;
            r->winFill = keep;
            r->winAt = 0;
        }
        if(r->winFill >= r->win.size())
        {
            // The window already holds a whole frame at the wire's largest and
            // the decoder still wants more, which take() cannot ask for. Guarded
            // anyway: the alternative to this line is an infinite loop.
            return false;
        }
        const Size room = r->win.size() - r->winFill;
        const Size got = readAt(r->src, r->srcAt, r->win.data() + r->winFill, room);
        if(got == 0u)
        {
            return false;
        }
        r->winFill += got;
        r->srcAt += static_cast<UInt64>(got);
        return true;
    }

    // THE FALLBACK. Every complete frame in the file, found with the same
    // take() the live link uses, with the junk between them counted rather than
    // swallowed. Every path through the loop consumes at least one byte, which
    // is what makes it terminate on any input at all - including one that is
    // pure noise.
    [[nodiscard]] Bool scanNext(Reader* r, Record* out)
    {
        for(;;)
        {
            const Size have = r->winFill - r->winAt;
            if(have > 0u)
            {
                bibowire::Frame f;
                Size used = 0;
                const bibowire::Take got = bibowire::take(
                    r->win.data() + r->winAt,
                    have,
                    &f,
                    &used
                );
                if(got == bibowire::Take::TAKE_FRAME)
                {
                    fillRecord(out, f, r->winBase + r->winAt, used);
                    r->winAt += used;
                    ++r->frames;
                    r->servedTo = r->winBase + r->winAt;
                    return true;
                }
                if(got == bibowire::Take::TAKE_RESYNC)
                {
                    r->junkBytes += static_cast<UInt64>(used);
                    r->winAt += used;
                    continue;
                }
                if(got != bibowire::Take::TAKE_NEED_MORE)
                {
                    // TOO_BIG and BAD_FLAG consume nothing and are terminal on a
                    // SOCKET, where there is a peer to answer and a connection to
                    // close. A file has neither: the only useful answer is to step
                    // over the byte and keep looking for the frames after it.
                    r->junkBytes += 1u;
                    r->winAt += 1u;
                    continue;
                }
            }
            if(refill(r))
            {
                continue;
            }
            const Size left = r->winFill - r->winAt;
            if(left == 0u)
            {
                r->done = true;
                return false;
            }
            if(left < bibowire::FRAME_OVERHEAD)
            {
                r->tailBytes += static_cast<UInt64>(left);
                r->winAt += left;
                r->done = true;
                return false;
            }
            // A shaped header at the end of the file wanting bytes the file does
            // not have. That is precisely what a pulled battery leaves behind -
            // but it can equally be a false lock inside junk, and stopping here
            // would lose every real frame after it. Step one byte and let the
            // scan go on; this always consumes one, so the loop still ends.
            r->tailBytes += 1u;
            r->winAt += 1u;
        }
    }

  }

  Bool Reader::begin(const Bytes& b)
  {
      src = b;
      foot = Footer();
      source = Source::SOURCE_NONE;
      fallback = Fallback::FALLBACK_NONE;
      frames = 0;
      junkBytes = 0;
      tailBytes = 0;
      winFill = 0;
      winAt = 0;
      winBase = 0;
      srcAt = 0;
      nextEntry = 0;
      servedTo = 0;
      done = false;
      scratch.clear();
      if(src.read == nullptr)
      {
          return false;
      }
      win.assign(MAX_FRAME_BYTES, 0);
      if(loadFooter(this))
      {
          source = Source::SOURCE_INDEX;
          fallback = Fallback::FALLBACK_NONE;
          return true;
      }
      // An empty file, a headless file and a file of pure noise are all
      // SUCCESSFUL begins that simply yield no records. Refusing them would make
      // "could not open" and "nothing survived" the same answer, and those are
      // the two things a person most needs told apart after a run.
      source = Source::SOURCE_SCAN;
      resetScan(this, 0);
      return true;
  }

  Bool Reader::next(Record* out)
  {
      if(out == nullptr || src.read == nullptr || done)
      {
          return false;
      }
      if(source == Source::SOURCE_INDEX)
      {
          if(nextEntry >= foot.indexCount)
          {
              done = true;
              return false;
          }
          Entry e;
          if(entry(nextEntry, &e) && loadAt(this, e.offset, e.frameLen, out))
          {
              ++nextEntry;
              ++frames;
              servedTo = e.offset + e.frameLen;
              return true;
          }
          // The index promised a frame and the file does not have one there, so
          // the index is not describing this file and nothing further in it is
          // worth believing. Resume the scan just past the last record already
          // handed out - never at zero, which would serve the run twice.
          fallback = Fallback::FALLBACK_BAD_INDEX;
          source = Source::SOURCE_SCAN;
          resetScan(this, servedTo);
      }
      return scanNext(this, out);
  }

  Bool Reader::at(UInt64 i, Record* out)
  {
      Entry e;
      if(!entry(i, &e))
      {
          return false;
      }
      return loadAt(this, e.offset, e.frameLen, out);
  }

  Bool Reader::entry(UInt64 i, Entry* out) const
  {
      if(out == nullptr || source != Source::SOURCE_INDEX || i >= foot.indexCount)
      {
          return false;
      }
      Array<UInt8, ENTRY_BYTES> raw{};
      const UInt64 at = foot.frameBytes + i * ENTRY_BYTES;
      if(readAt(src, at, raw.data(), ENTRY_BYTES) != ENTRY_BYTES)
      {
          return false;
      }
      unpackEntry(raw.data(), out);
      return true;
  }

}
