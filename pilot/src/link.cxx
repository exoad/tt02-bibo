// See link.hxx. The refusing implementation.
//
// Every entry point answers honestly that there is no transport here. When
// linkPosix.cxx exists it replaces this file in the build; nothing above the
// header changes, and that is what makes the swap safe rather than a rewrite.

#include "link.hxx"

namespace link
{
  namespace
  {

    // Kept even though nothing can set it true yet, because the shape of the
    // state is part of what this file is settling. isOpen() reading a real
    // variable rather than `return false` is what lets the test below assert
    // the sequence - open, fail, still closed - instead of a constant.
    Bool opened = false;

  }

  CharSeq why(Result r)
  {
      switch(r)
      {
      case Result::RESULT_OK:           return "ok";
      case Result::RESULT_NO_PLATFORM:  return "no serial transport is built for this platform yet";
      case Result::RESULT_NO_PORT:      return "no such port";
      case Result::RESULT_DENIED:       return "permission denied opening the port";
      case Result::RESULT_NOT_OPEN:     return "the link is not open";
      case Result::RESULT_WRITE_FAILED: return "the write failed";
      case Result::RESULT_CLOSED:       return "the link closed";
      default:                          return "?";
      }
  }

  Result open(const Config& cfg)
  {
      static_cast<Void>(cfg);

      // NOT RESULT_NO_PORT. That would read as "plug the cable in" and send
      // somebody looking at the car, when the truth is that the code to talk to
      // it has not been written. Naming the right absence is the whole job of
      // an error value.
      return Result::RESULT_NO_PLATFORM;
  }

  Void close()
  {
      opened = false;
  }

  Bool isOpen()
  {
      return opened;
  }

  Result send(const Str& line)
  {
      static_cast<Void>(line);
      return opened ? Result::RESULT_NO_PLATFORM : Result::RESULT_NOT_OPEN;
  }

  Result drain(Vec<Str>& out)
  {
      // `out` is left ALONE rather than cleared. A caller that gathers from
      // several sources into one vector should not have its earlier lines
      // deleted by a transport that had nothing to add.
      static_cast<Void>(out);
      return opened ? Result::RESULT_NO_PLATFORM : Result::RESULT_NOT_OPEN;
  }

  Int32 silentForMs()
  {
      return -1;
  }

}
