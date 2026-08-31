// A small JSON reader, for talking to clangd.
//
// WHY NOT SCAN FOR THE FIELDS. An LSP completion response is a few hundred
// kilobytes of nested objects, and the things worth having - label, insertText,
// detail, kind - sit inside an array inside an object. Finding them with
// strstr works until a doc comment contains `"label"`, or a signature contains
// a brace, or a string contains an escaped quote. Every one of those produces a
// plausible wrong answer rather than an error, which is the failure mode this
// project has spent a day removing.
//
// So it parses. It is deliberately not a general-purpose library: no writer, no
// pretty-printer, no number formatting beyond what a response contains.
//
// ImGui-free and Win32-free, so it is tested on its own - hub/tests.
#pragma once

#include "shared.hxx"

namespace js
{

  enum class Type
  {
      TYPE_NULL = 0,
      TYPE_BOOL,
      TYPE_NUMBER,
      TYPE_STRING,
      TYPE_ARRAY,
      TYPE_OBJECT,
  };

  // One node. Children live in `kids`; for an object, `keys[i]` names `kids[i]`.
  //
  // Parallel vectors rather than a map: a completion response has a handful of
  // keys per object and is walked once, so a hash per lookup would cost more
  // than the linear scan it replaces - and the order is preserved, which makes
  // a failure legible when it is printed back out.
  struct Value
  {
      Type       type = Type::TYPE_NULL;
      Bool       b    = false;
      Float64    num  = 0.0;
      Str        str;
      Vec<Str>   keys;
      Vec<Value> kids;

      [[nodiscard]] Bool isNull() const
      {
          return type == Type::TYPE_NULL;
      }

      [[nodiscard]] Bool isObject() const
      {
          return type == Type::TYPE_OBJECT;
      }

      [[nodiscard]] Bool isArray() const
      {
          return type == Type::TYPE_ARRAY;
      }

      // A member by name, or a null Value. NEVER a pointer that can be null:
      // every caller here is walking a response it did not write, so chaining
      // `at("result").at("items")` has to be safe when the server sent an
      // error instead.
      [[nodiscard]] const Value& at(const Char* key) const;

      // An element by index, or a null Value. Same reasoning.
      [[nodiscard]] const Value& operator[](Size i) const;

      [[nodiscard]] Size size() const
      {
          return kids.size();
      }

      // Typed reads with a fallback, so a caller never has to test the type
      // first. A field that is missing and a field that is the wrong type are
      // the same thing to somebody reading a reply: not usable.
      [[nodiscard]] Str   string(const Char* fallback = "") const;
      [[nodiscard]] Int32 integer(Int32 fallback = 0) const;
      [[nodiscard]] Bool  boolean(Bool fallback = false) const;
  };

  // Parses one value. `ok` is false and the result is null on malformed input.
  //
  // Reports WHERE it stopped, because a truncated frame and a genuinely
  // malformed one need different fixes - the first is a framing bug on our
  // side, the second is a server we are misreading.
  [[nodiscard]] Value parse(const Str& text, Bool& ok, Size& stoppedAt);

  // A JSON string literal, quoted and escaped. The only writing this needs:
  // requests are assembled from a handful of known shapes, and the one part
  // that can contain anything is a file's text.
  [[nodiscard]] Str quote(const Str& raw);

}
