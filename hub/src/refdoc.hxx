// ---------------------------------------------------------------------------
// .bdoc - the reference document format, parsed and drawn.
//
// A very small subset of XML: elements, attributes, text, comments. No
// namespaces, no DTD, no processing instructions, no CDATA, and only the four
// entities that make the four reserved characters writable.
//
// WHY A FORMAT. The pinouts were C++ arrays with a drawing function written for
// each one, which works exactly once. The second component needs either a
// second routine or a generalisation of the first, and by the fourth the hub is
// carrying a documentation system nobody planned. These are files: adding a
// component is adding one, and the hub does not change. A wiring note can be
// corrected without a build - which matters, because a wiring note is usually
// being corrected while the wires are in your hands.
//
// ---- the case rules, which are load-bearing -------------------------------
//
//   Elements     PascalCase    <Doc> <Section> <Pinout> <Pin> <Bold>
//   Attributes   camelCase     title= subtitle= class= from=
//   Class values camelCase     power ground serial audio
//
// Not decoration. An element and an attribute are different kinds of thing, and
// reading one as the other is the commonest mistake in a hand-written format;
// making them visibly different makes the mistake visible too. The parser does
// not enforce the casing - it cannot, without refusing documents over a style
// rule - but check() reports it, so a document can be wrong loudly.
// ---------------------------------------------------------------------------
#pragma once

#include "shared.hxx"

#include "imgui.h"

namespace refdoc
{

  // The extension, in one place. Anything that has to recognise one of these -
  // the tree, the icon, the toggle, the linter's exemption - asks here.
  inline constexpr const Char* EXT = ".bdoc";

  [[nodiscard]] Bool isDocPath(const Str& path);

  struct Attr
  {
      Str name;
      Str value;
  };

  // One element, or one run of text.
  //
  // A text run is a node with an EMPTY name, which is what lets a paragraph hold
  // a mixture of prose and <Bold> without two container types. Walking children
  // in order and asking isText() reproduces the source exactly.
  struct Node
  {
      Str       name;    // empty => this is a text run
      Str       text;    // set only for text runs
      Vec<Attr> attrs;
      Vec<Node> kids;
      Int32     line = 0;   // where it opened, for error messages

      [[nodiscard]] Bool isText() const
      {
          return name.empty();
      }

      // The attribute's value, or `fallback` if it is absent. Never null, so a
      // call site can print it without a guard.
      [[nodiscard]] const Char* attr(const Char* want, const Char* fallback = "") const;

      [[nodiscard]] Bool hasAttr(const Char* want) const;
  };

  // A parsed document, or the reason it is not one.
  //
  // A failed parse is a VALUE, not an exception and not an empty document. The
  // renderer draws the error where the page would have been, with the line
  // number, because the alternative is a blank panel and no way to tell a broken
  // document from an empty one.
  struct Doc
  {
      Node  root;
      Str   error;          // empty when it parsed
      Int32 errorLine = 0;

      [[nodiscard]] Bool ok() const
      {
          return error.empty();
      }
  };

  // `baseDir` is the folder the text came from, and is what <Include file="..."/>
  // resolves against. Pass it empty and an include reports that it cannot be
  // resolved rather than guessing at a working directory.
  [[nodiscard]] Doc parse(const Str& text, const Str& baseDir = Str());

  // Style complaints - casing, unknown elements, a Pin with no name. Separate
  // from parse() because these are things a document should not do, not things
  // that stop it being a document.
  [[nodiscard]] Vec<Str> check(const Doc& d);

  // Draws the whole document into the current window, wrapping to `width`.
  // Scrolling is the caller's child window, not ours.
  Void draw(const Doc& d, Float32 width);

  // Where the reader is on the page: how far in, and how far across.
  //
  // ONE VALUE rather than a zoom and two loose floats, because they are only
  // ever meaningful together - a pan means nothing without the zoom it was made
  // at, and resetting one without the other leaves the page somewhere nobody
  // asked for.
  struct View
  {
      Float32 zoom = 1.0f;
      Float32 panX = 0.0f;
      Float32 panY = 0.0f;

      // Mid-drag. Latched when the press lands on the page and held until the
      // button comes up, so a pan that wanders off the panel keeps going - see
      // drawPage.
      Bool    panning = false;
  };

  // The whole page, with its own frame: wheel zoom, drag to pan, and a reading
  // measure that stays a sensible number of words wide.
  //
  // Here rather than at the call site so that a document behaves identically
  // wherever it is opened from.
  //
  // THE PAN IS UNBOUNDED. A document is a canvas, not a scrolled column, so it
  // can be pushed off any edge in any direction - see the long note in
  // drawPage for why the old scroll-based version could not. Double-click puts
  // it back, which is the only way home once it is off-panel.
  //
  // `view` is the caller's, so each surface keeps its own place on the page.
  Void drawPage(const Doc& d, const ImVec2& size, View& view, Float32 dpiScale);

} // namespace refdoc
