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
 * The vocabulary of the Pico, the pilot and the viewer. From manbox
 * (github.com/exoad/manbox), C_STYLE_GUIDE.md.
 *
 * Each alias IS the type it names, so it passes to a third-party API without a
 * conversion; third-party signatures keep their own spellings. Global scope,
 * because a namespace would need a qualified spelling or a `using namespace` in
 * a header.
 */
#pragma once

#include <cstddef>
#include <cstdint>

using Int8 = std::int8_t;
using Int16 = std::int16_t;
using Int32 = std::int32_t;
using Int64 = std::int64_t;
using UInt8 = std::uint8_t;
using UInt16 = std::uint16_t;
using UInt32 = std::uint32_t;
using UInt64 = std::uint64_t;
using Size = std::size_t;
using ISize = std::ptrdiff_t;
using UPtr = std::uintptr_t;

using Float32 = float;
using Float64 = double;

using Bool = bool;
using Void = void;
using Char = char;
using Utf8 = char;
using Utf8Byte = unsigned char;
/* Borrowed and NUL-terminated. On a host, StrView unless a C API wants the pointer. */
using CharSeq = const Utf8*;

/*
 * A macro's value as a string literal. Two levels because # does not expand its
 * argument: one level turns STRINGIFY(PICO_DEFAULT_LED_PIN) into
 * "PICO_DEFAULT_LED_PIN". Guarded because a vendor header may define it.
 */
#ifndef STRINGIFY
#define STRINGIFY_INNER(x) #x
#define STRINGIFY(x) STRINGIFY_INNER(x)
#endif

/*
 * Hosts only. The Pico's firmware is freestanding: no heap, no exceptions, no
 * STL. PICO_ON_DEVICE is the Pico SDK's switch, 1 in every device build and 0 or
 * undefined everywhere else. <newlib.h> is the Pico toolchain's C library, which
 * firmware/.clangd gives the editor without PICO_ON_DEVICE, and whose <atomic>
 * does not parse there.
 */
#if !PICO_ON_DEVICE && !__has_include(<newlib.h>)
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

using Str = std::string;
using StrView = std::string_view;

template<typename T>
using Vec = std::vector<T>;

template<typename T, Size N>
using Array = std::array<T, N>;

/* Only when things are pushed and popped at both ends. */
template<typename T>
using Deque = std::deque<T>;

template<typename T>
using Set = std::set<T>;

template<typename T>
using UniqPtr = std::unique_ptr<T>;

template<typename T>
using Opt = std::optional<T>;

/*
 * For a STORED callable. A callback that is not kept takes a template parameter
 * and skips the heap.
 */
template<typename Sig>
using Fn = std::function<Sig>;

using Thread = std::thread;
using Mutex = std::mutex;
using CondVar = std::condition_variable;

template<typename T>
using LockGuard = std::lock_guard<T>;

template<typename T>
using UniqueLock = std::unique_lock<T>;

template<typename T>
using Atomic = std::atomic<T>;

/* Steady, for intervals: the system clock can step backwards. */
using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
/*
 * Only for a timestamp another process compares, such as the pilot's bootId;
 * never for an interval.
 */
using WallClock = std::chrono::system_clock;
using Millis = std::chrono::milliseconds;

template<typename Rep, typename Period = std::ratio<1>>
using Duration = std::chrono::duration<Rep, Period>;

/* monoNow, not now: this is global, and `now` is a local name people write. */
[[nodiscard]] inline TimePoint monoNow()
{
    return Clock::now();
}

[[nodiscard]] inline Float64 elapsedMs(TimePoint since)
{
    return Duration<Float64, std::milli>(Clock::now() - since).count();
}

[[nodiscard]] inline Float64 elapsedS(TimePoint since)
{
    return Duration<Float64>(Clock::now() - since).count();
}

inline Void sleepMs(Int64 ms)
{
    std::this_thread::sleep_for(Millis(ms));
}

template<typename T, typename... Args>
[[nodiscard]] inline UniqPtr<T> makeUniq(Args&&... args)
{
    return std::make_unique<T>(std::forward<Args>(args)...);
}
#endif
