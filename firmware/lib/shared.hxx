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
 * The firmware's vocabulary and idiom. From manbox (github.com/exoad/manbox),
 * C_STYLE_GUIDE.md.
 *
 * ONE NAME, TWO FILES, ON PURPOSE. shared/shared.hxx is the hub's; this is the
 * firmware's. Same name because they are the same idea - an Int32 in the hub is
 * an Int32 on the board - and separate files because the firmware is
 * FREESTANDING: no heap, no exceptions, no STL, so the hub's templates are
 * unusable here. Nothing ever includes both, and nothing can: firmware targets
 * put only firmware/lib on the include path, the hub only ../shared.
 *
 * They are kept in step by hand, and only where it makes sense to. The hub has
 * ISize; this has the idiom macros below, which the hub has no use for.
 *
 * Utf16/Utf32 are a LOCAL ADDITION - upstream names them in CharSeq16/CharSeq32
 * without defining them, so the header does not compile as it stands.
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
using Int8 = std::int8_t;
using Int16 = std::int16_t;
using Int32 = std::int32_t;
using Int64 = std::int64_t;

using UInt8 = std::uint8_t;
using UInt16 = std::uint16_t;
using UInt32 = std::uint32_t;
using UInt64 = std::uint64_t;

using Size = std::size_t;
using UPtr = std::uintptr_t;

/* ---- floating point ------------------------------------------------------ */
using Float32 = float;
using Float64 = double;

/* ---- other fundamentals -------------------------------------------------- */
using Bool = bool;
using Void = void;

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
using Utf8 = char;
using Utf8Byte = unsigned char;
using Utf16 = std::uint16_t;
using Utf32 = std::uint32_t;

using CharSeq = const Utf8*;
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

/*
 * ---- idiom ---------------------------------------------------------------
 *
 * Guarded like STRINGIFY: these are ordinary enough names that a vendor header
 * could claim one, and a redefinition warning in a build this quiet is noise
 * nobody reads.
 */

/*
 * PROGRAM - an entry point, spelled the one way that is correct everywhere.
 *
 * `int`, NOT Int32, and that is the whole reason this exists. int32_t is
 * `long int` on arm-none-eabi and `int` on MSVC - same size, same
 * representation, a different type as far as the language cares - so
 * `Int32 main` compiles clean on the host and the board rejects it:
 *
 *     error: '::main' must return 'int'
 *
 * Host suites cannot catch that; only a board build can. It has been
 * rediscovered three times. main's signature is the C runtime's contract
 * rather than this project's vocabulary, so it is spelled the runtime's way,
 * once, here.
 */
#ifndef PROGRAM
#define PROGRAM int main(Void)
#endif

/* An intentional forever loop. Takes its own braces: FOREVER { ... } */
#ifndef FOREVER
#define FOREVER while(true)
#endif
