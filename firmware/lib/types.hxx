/*
Copyright (c) 2026, Jiaming Meng (jackm@exoad.net)

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * From manbox (github.com/exoad/manbox), C_STYLE_GUIDE.md. This is the C
 * counterpart of shared/shared.hpp, which the HUB uses. This one lives in
 * firmware/lib because the firmware is its only caller. The two are kept in
 * step by hand; they are the same vocabulary spelled the way C - the same vocabulary, spelled the way C
 * spells it, so a value that is an Int32 in the viewer is an Int32 in the
 * firmware too.
 *
 * The two headers sit side by side and are kept in step by hand. They are not
 * one file because C has no templates and no namespaces, and a header trying to
 * serve both languages would be mostly #ifdef.
 *
 * The Utf16/Utf32 typedefs at the bottom are a LOCAL ADDITION: the upstream
 * header uses those names in the CharSeq16/CharSeq32 macros without defining
 * them, so the macros do not compile as they stand. They are declared here so
 * the header is usable as written. Everything above that block is upstream.
 */

#pragma once

/*
 * <cstdint> and <cstddef>, not <stdint.h> and <stddef.h>: this is C++ and the
 * counterpart vocabulary in shared/shared.hxx has always said so. <stdbool.h>
 * was here too and is a no-op - bool is a keyword.
 */
#include <cstddef>
#include <cstdint>

/* ---- integers ------------------------------------------------------------ */
using Int8    = std::int8_t;
using Int16   = std::int16_t;
using Int32   = std::int32_t;
using Int64   = std::int64_t;

using UInt8   = std::uint8_t;
using UInt16  = std::uint16_t;
using UInt32  = std::uint32_t;
using UInt64  = std::uint64_t;

using Size    = std::size_t;
using UPtr    = std::uintptr_t;

/* ---- floating point ------------------------------------------------------ */
using Float32 = float;
using Float64 = double;

/* ---- other fundamentals -------------------------------------------------- */
using Bool    = bool;
using Void    = void;

/*
 * ---- text ----------------------------------------------------------------
 *
 * CharSeq is a borrowed, NUL-terminated pointer. It is a `using` and no longer
 * a #define, which matters: shared.hxx has always spelled it as an alias, and
 * a macro and an alias disagree the moment anyone writes `const CharSeq` -
 * the macro producing `const const Utf8*`, which does not compile, and the
 * alias producing `Utf8* const`, which does and means something else.
 *
 * Utf16 and Utf32 are declared BEFORE the sequences built from them. As macros
 * the order did not matter, because a macro body is not looked at until it is
 * used. An alias is.
 */
using Utf8      = char;
using Utf8Byte  = unsigned char;
using Utf16     = std::uint16_t;
using Utf32     = std::uint32_t;

using CharSeq   = const Utf8*;
using CharSeq16 = const Utf16*;
using CharSeq32 = const Utf32*;

/*
 * A macro's VALUE as a string literal.
 *
 * The two-step is not decoration. A parameter is NOT macro-expanded before the
 * # operator sees it, so a single-level STRINGIFY(PICO_DEFAULT_LED_PIN) yields
 * the string "PICO_DEFAULT_LED_PIN" rather than "25". Passing it through an
 * outer macro first forces the expansion, and the inner one then stringifies
 * what it became. Every C codebase writes this pair eventually; here it is
 * once, spelled out, instead of three times in three headers.
 *
 * Guarded because it is a common enough name that a vendor header may already
 * have claimed it, and a redefinition warning in a build this quiet would be
 * noise nobody reads.
 */
#ifndef STRINGIFY
#define STRINGIFY_INNER(x) #x
#define STRINGIFY(x) STRINGIFY_INNER(x)
#endif
