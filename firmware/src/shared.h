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
 * counterpart of hub/src/shared.hpp - the same vocabulary, spelled the way C
 * spells it, so a value that is an Int32 in the viewer is an Int32 in the
 * firmware too.
 *
 * The Utf16/Utf32 typedefs at the bottom are a LOCAL ADDITION: the upstream
 * header uses those names in the CharSeq16/CharSeq32 macros without defining
 * them, so the macros do not compile as they stand. They are declared here so
 * the header is usable as written. Everything above that block is upstream.
 */

#ifndef SHARED_H
#define SHARED_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

typedef int32_t Int32;
typedef int64_t Int64;
typedef uint32_t UInt32;
typedef uint64_t UInt64;
typedef int8_t Int8;
typedef uint8_t UInt8;
typedef char Utf8;
typedef uintptr_t UPtr;
typedef size_t Size;
typedef float Float32;
typedef double Float64;
typedef int16_t Int16;
typedef uint16_t UInt16;
typedef void Void;
typedef FILE CFile;
typedef bool Bool;
typedef Void* Any;

#define CharSeq const Utf8*
#define CharSeq16 const Utf16*
#define CharSeq32 const Utf32*

/* ---- local addition, see the note at the top of this file ---------------- */
typedef uint16_t Utf16;
typedef uint32_t Utf32;

#endif
