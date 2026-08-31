// clangd, driven over stdio, for the code view's completion popup.
//
// WHY A LANGUAGE SERVER AND NOT A BIGGER TABLE. complete.cxx holds forty names
// written out by hand, and that was the right call while the only thing worth
// completing was pico2w.h. It is not any more. `pins::Map` has 24 fields that a
// designated initializer wants by name, `sfx::Clip` has three, gfx's colors
// just stopped being macros and became namespace members, and none of that is
// knowable without actually parsing the translation unit. clangd already does,
// and it is already installed for the build.
//
// So: two sources, one popup. The table answers instantly and always; clangd
// answers a beat later and answers about the REAL code. When clangd is not
// installed, has not finished its first parse, or falls over, the table is what
// is left and the editor still works.
//
// ------------------------------------------------------------------ shape ---
// One child process, two threads.
//
//   the writer   owns the child's stdin. A queue and a CondVar, because
//                WriteFile on a full pipe BLOCKS, and a blocked UI thread is
//                the worst failure this could have - worse than no completion
//                at all, which is at least visibly nothing.
//   the reader   owns the child's stdout. Content-Length framing in, JSON out,
//                and the one answer we care about into a mutex-guarded slot.
//
// The UI thread only ever pushes a request and polls the slot. It never blocks.
//
// -------------------------------------------------------------- the errors --
// state() and status() report what actually happened, in the words the failure
// used. "clangd not found" and "clangd exited during startup" and "no
// compile_commands.json" are three different problems with three different
// fixes, and a popup that is merely empty tells you none of them.
#pragma once

#include "complete.hxx"
#include "diagnostics.hxx"
#include "shared.hxx"

namespace lsp
{

  enum class State
  {
      STATE_OFF = 0,     // never started
      STATE_STARTING,    // spawned, handshake not finished
      STATE_READY,       // answering
      STATE_FAILED,      // see status() - it says which of the ways
  };

  // One suggestion. Deliberately the same fields complete.cxx's Item carries,
  // so the popup renders both from one loop.
  //
  // Owning strings, not pointers: these come off the wire and the table's do
  // not, and a popup that has to know which is which is a popup with a
  // dangling-pointer bug in its future.
  struct Item
  {
      Str        name;     // what gets inserted
      Str        detail;   // the signature, or the field's type
      Str        doc;      // first line of the doc comment, may be empty
      cmpl::Kind kind = cmpl::Kind::KIND_FUNCTION;
  };

  // A completed reply, tagged with the position it was asked about.
  //
  // The position matters: by the time this lands the caret has usually moved,
  // and an answer about a place the user has left is worse than no answer. The
  // caller compares before it shows anything.
  struct Answer
  {
      UInt64    serial = 0;   // increments per reply; 0 means "nothing yet"
      Str       path;
      Int32     line = -1;    // 0-based, as LSP counts
      Int32     col  = -1;
      Vec<Item> items;
  };

  // Spawns clangd. Safe to call repeatedly - a second call while one is running
  // does nothing. Returns false if it could not be started, and status() then
  // says why in the failure's own words.
  //
  // Not called at startup: an editor session that never opens the code view
  // should not be paying for a clangd index. app_ui starts it when the view is
  // first shown.
  Bool start();

  // Ends the child and joins both threads. Idempotent.
  Void stop();

  [[nodiscard]] State state();

  // One line for the status bar. Always says something true, including while
  // starting ("clangd: indexing") and after a failure.
  [[nodiscard]] Str status();

  // Ask about `line`:`col` in `path`, whose current contents are `text`.
  //
  // `version` is the caller's own edit counter. The document is only re-sent
  // when it changes, so holding the caret still and asking twice costs one
  // message rather than two copies of the file.
  //
  // Fire and forget. At most one request is outstanding: asking again while one
  // is in flight REPLACES it, because the older question is about a caret
  // position the user has already left.
  //
  // Returns whether the question was actually sent. It is refused while clangd
  // is still building an AST for the file - asked earlier, clangd answers from
  // an identifier index instead and `dfplayer::` comes back containing `printf`
  // and `define`. The document itself is still sent, which is what starts that
  // build; only the question waits. A caller that gets false should ask again
  // rather than conclude there were no suggestions.
  Bool ask(const Str& path, const Str& text, UInt64 version, Int32 line, Int32 col);

  // Moves the newest reply into `out` if one arrived since the last take().
  // Returns false and leaves `out` alone otherwise.
  Bool take(Answer& out);

  // Whether a request is in flight. The popup uses this to decide between
  // "clangd has nothing" and "clangd has not answered yet", which look
  // identical and mean opposite things.
  [[nodiscard]] Bool busy();

  // clangd's own diagnostics for the open file - its parse errors and, where a
  // .clangd enables them, clang-tidy's checks.
  //
  // These arrive on textDocument/publishDiagnostics, which this module already
  // received and used ONLY as a signal that the AST was ready: the payload was
  // read for its URI and then dropped. So the Code view underlined build output
  // and the hub's own linter, and never anything clangd found - including the
  // narrowing conversions and use-after-move that are the reason to run a
  // linter at all.
  //
  // Replaces whatever was last published for the file rather than appending;
  // clangd sends the complete set each time, and an empty array is the message
  // "this file is clean now".
  //
  // Returns false when nothing new has arrived since the last call, leaving
  // `out` untouched.
  Bool diagnostics(Vec<diag::Item>& out);

}
