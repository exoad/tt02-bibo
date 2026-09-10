// The .bibo archive: one run off the board's SD card, replayable byte for byte.
//
// ---------------------------------------------------------------------------
// WHY THE FRAMES ARE STORED VERBATIM
//
// A recorded frame is the EXACT bytes bibowire::put() emitted onto the wire -
// header, payload and CRC32C, with not one byte moved. A replay is therefore
// not a reconstruction of a run, it IS the run, and it goes back in through the
// same bibowire::take() the live viewer uses, with the 312 checks already
// standing behind that decoder.
//
// The alternative - a second, archive-shaped encoding of every message - is a
// second codec to keep in step with the first, and the day the two drift the
// recording of a crash decodes into something that never happened. There is no
// archive-side layout for any message here, deliberately: this file defines a
// CONTAINER and nothing about what is inside a frame.
//
// The camera rides inline, as CAMERA (0x20) bodies with codec = 1 and the
// sensor's MJPEG bytes copied untouched. THERE IS NO RE-ENCODE ANYWHERE IN THIS
// DESIGN. A transcode would make the file a picture OF the run rather than the
// run, and it would put a lossy step between the operator and the thing they
// are trying to explain.
//
// ---------------------------------------------------------------------------
// THE FOOTER IS AN INDEX, NEVER A PREREQUISITE
//
// A run on a car ends when the power is pulled. That is not the exceptional
// case, it is the ordinary one, and it means THE FOOTER WILL OFTEN BE MISSING
// OR HALF-WRITTEN. A reader that answered "zero frames" to that while reporting
// success would be this repo's named recurring bug - reports success while
// measuring nothing - with a whole outing inside it.
//
// So the footer only ever buys speed. Whenever it is absent, truncated, fails
// its CRC32C, or carries numbers that cannot describe the file in front of it,
// the reader falls back to a LINEAR SCAN with bibowire::take(), recovers every
// complete frame, and SAYS WHICH PATH IT USED: `source` and `fallback` below,
// both readable before the first record comes out. A footer this reader has not
// verified is never believed, and "verified" means the CRC AND the arithmetic -
// a CRC is a check against corruption, not against a lie.
//
// ---------------------------------------------------------------------------
// NO FLOATING POINT ON DISK, THE SAME RULE THE WIRE HAS
//
// Every quantity in the footer is a fixed-width integer with its unit in its
// name, for the reason bibowire.hxx already gives at length: there is no
// formatting step for a locale to reach, no NaN, no denormal, and every
// round-trip test is byte-exact rather than approximately equal. Wall-clock
// time is UInt64 unix MICROSECONDS, monotonic time is the tMonoUs the frame
// already carried, and the mapping between them is integer subtraction.
//
// ---------------------------------------------------------------------------
// STREAMING, BOTH WAYS
//
// Writing appends and flushes one frame at a time and never holds a run in
// memory - the recorder's resident cost is its index, 16 bytes per frame, and
// nothing that grows with a payload. Reading never maps the whole file either:
// every access is a bounded read at an offset through `Bytes`, so a two-
// gigabyte outing costs the same resident memory as an empty one. Two
// implementations of that interface exist, a file and a span of memory, and the
// reader cannot tell them apart - which is what lets the whole test suite run
// with no filesystem at all and still prove the thing that will run on the Pi.
#pragma once

#include "bibowire.hxx"

namespace archive
{

  // 'B','I','B','O' in that order on disk. Explicitly NOT bibowire::MAGIC
  // (0x5742, the bytes 0x42 0x57): the fallback reader scans this same file for
  // the pair 0x42 0x57 looking for frames, so a footer that spelled the frame
  // magic would be offered up to take() as a frame candidate on every scan of
  // every archive. The two must differ in the FIRST TWO BYTES, because that is
  // the window take() slides. 'B','I' shares only the 'B'.
  //
  // Four bytes and not two, because unlike a frame there is exactly one of
  // these per file and it is found by seeking rather than by scanning: the
  // extra two bytes cost nothing per run and take a false footer lock from
  // 2^-16 to 2^-32 before the CRC is even consulted.
  constexpr UInt32 MAGIC = 0x4F424942u;

  // The footer's own version, not bibowire's. They move independently: a new
  // message type is a bibowire change and no business of the container's.
  constexpr UInt8 VERSION = 1;

  constexpr Size ENTRY_BYTES = 16;
  constexpr Size FOOTER_BYTES = 148;

  // One whole frame at the wire's largest. The scan window is sized from this,
  // so a 256 KiB CAMERA frame can be assembled without the reader ever holding
  // two of them.
  constexpr Size MAX_FRAME_BYTES = bibowire::FRAME_OVERHEAD + bibowire::MAX_PAYLOAD;

  // Which path the records came out of. Readable BEFORE the first next(), so a
  // caller can print it rather than discover it - "recovered by scan" is the
  // single most useful sentence this module produces, and a reader that knew it
  // and did not say it would be hiding the interesting half of the answer.
  enum class Source : UInt8
  {
      SOURCE_NONE = 0,   // begin() has not run, or it refused
      SOURCE_INDEX,      // the footer verified and its index is being walked
      SOURCE_SCAN,       // linear scan with bibowire::take()
  };

  // WHY the index was not used. FALLBACK_NONE together with SOURCE_INDEX is the
  // only pairing that means "this file was closed properly"; every other value
  // names a specific way the footer failed to earn trust, because "fell back"
  // on its own sends a person to read the file with a hex editor.
  enum class Fallback : UInt8
  {
      FALLBACK_NONE = 0,      // the footer verified; nothing was fallen back from
      FALLBACK_NO_FOOTER,     // shorter than a footer, or the magic is not there
      FALLBACK_BAD_CRC,       // the magic is there and the CRC32C is not
      FALLBACK_BAD_VERSION,   // a footer version this build does not know
      FALLBACK_INSANE,        // CRC-valid, and its numbers cannot describe this file
      FALLBACK_BAD_INDEX,     // an index entry does not point at a parsable frame
  };

  // One index row. The offset is where the frame's first byte sits in the file
  // and `frameLen` is what it occupies, so an entry is a seek and a read with
  // no arithmetic in between. `type`, `ver` and `seq` are copies of the frame's
  // own header: a caller filtering a two-hour run for its EVENTs should not
  // have to read two hundred thousand payloads to find them.
  struct Entry
  {
      UInt64 offset = 0;
      UInt32 frameLen = 0;
      UInt8 type = 0;
      UInt8 ver = 0;
      UInt16 seq = 0;
  };

  // What sits in the last FOOTER_BYTES of a properly closed archive.
  //
  //    0   4  magic         MAGIC
  //    4   1  version       VERSION
  //    5   1  reserved0     zero
  //    6   2  reserved1     zero
  //    8   8  wallEpochUs   unix microseconds, paired with firstMonoUs
  //   16   8  firstMonoUs   the FIRST frame's tMonoUs
  //   24   8  frameCount
  //   32   8  frameBytes    bytes of frames; the index begins here
  //   40   8  indexCount
  //   48  88  typeCounts    one UInt32 per bibowire Type, in catalog order
  //  136   8  fileBytes     what the writer believes the whole file measures
  //  144   4  crc32c        bibowire::crc32c over bytes [0, 144)
  //
  // wallEpochUs and firstMonoUs are a PAIR and that is the whole point of them:
  // tMonoUs is a board-boot-relative number that means nothing next week, and
  // one pairing with the wall clock is what turns every timestamp in the file
  // into a time of day. See wallUsOf(). The residual error is the gap between
  // Writer::begin() and the first recorded frame - one tick, if the caller
  // starts the recorder in the tick that produces the frame.
  struct Footer
  {
      UInt8 version = VERSION;
      UInt64 wallEpochUs = 0;
      UInt64 firstMonoUs = 0;
      UInt64 frameCount = 0;
      UInt64 frameBytes = 0;
      UInt64 indexCount = 0;
      UInt64 fileBytes = 0;
      Array<UInt32, bibowire::TYPE_COUNT> typeCounts{};
  };

  // A bounded read at an offset, and the total the source is willing to serve.
  // `read` places at most `want` bytes at `out` and returns how many it really
  // placed, so a reader that asks past the end is answered with a short count
  // rather than with undefined behaviour. NOTHING here ever maps a whole file.
  struct Bytes
  {
      const Void* ctx = nullptr;
      UInt64 total = 0;
      Size (*read)(const Void* ctx, UInt64 at, UInt8* out, Size want) = nullptr;
  };

  // The write side of the same idea. `flush` is separate because the recorder
  // calls it after EVERY frame: a run ends when the power is pulled, and the
  // difference between losing one frame and losing the buffered tail of an
  // outing is this one call.
  struct Sink
  {
      Void* ctx = nullptr;
      Bool (*write)(Void* ctx, const UInt8* data, Size len) = nullptr;
      Bool (*flush)(Void* ctx) = nullptr;
  };

  [[nodiscard]] Bytes overMemory(const UInt8* data, Size len);
  [[nodiscard]] Sink intoMemory(Vec<UInt8>* out);

  // The board's side. The handle is opaque so this header stays free of
  // <cstdio>: the pilot includes this beside bibowire.hxx, and a header that
  // drags stdio in for two functions makes every consumer pay for it.
  struct File
  {
      Void* handle = nullptr;
      UInt64 total = 0;
  };

  [[nodiscard]] Bool openWrite(const Str& path, File* out);
  [[nodiscard]] Bool openRead(const Str& path, File* out);
  Void closeFile(File* f);
  [[nodiscard]] Sink sinkOf(File* f);
  [[nodiscard]] Bytes bytesOf(const File* f);

  // One recovered frame. The body is OWNED rather than borrowed, unlike
  // bibowire::Body: a reader that streams through a window cannot promise a
  // pointer into it stays valid across a refill, and a borrowed pointer whose
  // lifetime rule is "until you call next() again, unless the window moved" is
  // a rule somebody will get wrong. The Vec is reused across calls, so the
  // steady-state cost is a memcpy and not an allocation.
  struct Record
  {
      bibowire::Head head;
      Vec<UInt8> body;
      UInt64 offset = 0;
      Size frameLen = 0;
  };

  // ---- writing ---------------------------------------------------------------
  //
  // begin, put per frame, finish. put() takes the bytes put() already made -
  // this module never encodes a message and has no opinion about one.
  struct Writer
  {
      Sink sink;
      UInt64 wallEpochUs = 0;
      UInt64 firstMonoUs = 0;
      UInt64 frameCount = 0;
      UInt64 frameBytes = 0;
      Array<UInt32, bibowire::TYPE_COUNT> typeCounts{};
      Vec<Entry> index;
      Bool started = false;
      Bool haveMono = false;
      Bool failed = false;

      // `wallUs` is the wall clock now, in unix microseconds. It is paired with
      // the first frame's tMonoUs; see Footer.
      [[nodiscard]] Bool begin(const Sink& s, UInt64 wallUs);

      // The frame VERBATIM. Refused unless bibowire::take() agrees the bytes are
      // exactly one whole frame: a recorder that appends whatever it is handed
      // produces an archive whose corruption is indistinguishable from a power
      // cut, and the two want opposite responses from a person.
      [[nodiscard]] Bool put(const UInt8* frame, Size len);

      // Index and footer, then a flush. Not calling this is a supported ending -
      // it is what a power cut does - and the reader handles it.
      [[nodiscard]] Bool finish();

      // Stop without a footer, deliberately. Exists so the pulled-power case is
      // something the tests can produce on purpose rather than approximate.
      Void abandon();
  };

  // ---- reading ---------------------------------------------------------------
  struct Reader
  {
      Bytes src;
      Footer foot;
      Source source = Source::SOURCE_NONE;
      Fallback fallback = Fallback::FALLBACK_NONE;

      // What the run actually gave up, as opposed to what the footer claimed.
      // `junkBytes` is everything skipped between frames and `tailBytes` is the
      // incomplete frame at the end, if there was one. Both are counted rather
      // than swallowed: junk on a checksummed file that nobody counts is a fault
      // nobody discovers, which is bibowire's own argument for TAKE_RESYNC.
      UInt64 frames = 0;
      UInt64 junkBytes = 0;
      UInt64 tailBytes = 0;

      // The scan window. One whole frame at the wire's largest, so a 256 KiB
      // CAMERA can be assembled without ever holding two, and `winBase` is the
      // file offset of win[0] so a recovered frame can still say where it was.
      Vec<UInt8> win;
      Vec<UInt8> scratch;
      Size winFill = 0;
      Size winAt = 0;
      UInt64 winBase = 0;
      UInt64 srcAt = 0;
      UInt64 nextEntry = 0;

      // The file offset just past the last record handed out. A mid-run fall
      // back from the index resumes the scan HERE rather than at zero, so no
      // frame is ever served twice - a replay with a duplicated tick is worse
      // than one that stops, because it looks like the car did something twice.
      UInt64 servedTo = 0;
      Bool done = false;

      // Decides the path and records why. False only when the source itself is
      // unusable; an empty file, a headless file and a file of pure noise are
      // all successful begins that simply yield no records.
      [[nodiscard]] Bool begin(const Bytes& b);

      // The next complete frame, or false at the end. Never invents a record and
      // never reads past src.total.
      [[nodiscard]] Bool next(Record* out);

      // Entry `i` of the footer's index: seek straight to it and parse. Only on
      // SOURCE_INDEX, which is the honest answer - there is no index to seek in
      // when the footer did not survive.
      [[nodiscard]] Bool at(UInt64 i, Record* out);
      [[nodiscard]] Bool entry(UInt64 i, Entry* out) const;
  };

  // ---- the clocks ------------------------------------------------------------

  // The board's monotonic microseconds turned into unix microseconds, through
  // the one pairing the footer carries. Integer arithmetic, signed in the middle
  // so a frame stamped BEFORE the pairing instant maps backwards rather than
  // wrapping to the year 586524.
  [[nodiscard]] UInt64 wallUsOf(const Footer& f, UInt64 monoUs);

  // The tMonoUs a frame carries, if its type carries one. Not every type does -
  // HELLO, LEAVE and SUBSCRIBE have no clock in them at all - and inventing a
  // zero for those would put a fabricated timestamp in a recording, which is the
  // absent-sentinel argument from bibowire.hxx applied to a file.
  [[nodiscard]] Bool monoOf(bibowire::Type t, const bibowire::Body& b, UInt64* out);

  // Where this type's count sits in Footer::typeCounts, or TYPE_COUNT for a tag
  // this build has no name for. An unknown type is still counted in frameCount
  // and still recovered; it simply has no column of its own, which is the same
  // bargain the wire strikes when it skips one by length.
  [[nodiscard]] Size typeSlot(UInt8 tag);

  // Footer bytes in and out, exposed because the tests corrupt footers on
  // purpose and a test that hand-rolled the layout would be testing its own copy
  // of it rather than this one.
  Void packFooter(const Footer& f, UInt8* out);
  [[nodiscard]] Bool unpackFooter(const UInt8* in, Footer* out);

  [[nodiscard]] CharSeq sourceName(Source s);
  [[nodiscard]] CharSeq fallbackName(Fallback f);

}
