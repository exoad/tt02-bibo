// The project's C++ vocabulary. Include this before anything else.
//
// ---------------------------------------------------------------------------
// WHAT THIS IS
//
// One alias for every type this codebase names, so a width is a Float32 and a
// count is a Size EVERYWHERE, rather than float/int/size_t/unsigned varying by
// whoever wrote the line. The naming follows manbox (github.com/exoad/manbox):
// PascalCase types, camelCase functions, SCREAMING_SNAKE macros.
//
// These are `using` declarations and nothing more - no wrapper types, no new
// semantics. Every alias below IS the exact type it names, so handing a Float32
// to an ImGui function that wants a float is not a conversion, it is the same
// type spelled the project's way. Vec<T> is std::vector<T>; it has the same
// members, the same iterators and the same cost.
//
// ---------------------------------------------------------------------------
// WHY THE STANDARD LIBRARY IS ALIASED TOO
//
// Because a convention with a hole in it is not a convention. A file that
// declares `Int32 count` on one line and `std::vector<std::string> names` on the
// next has two naming schemes in it, and the reader has to hold both. Aliasing
// the containers costs one line each here and removes the seam entirely.
//
// The rule at the boundary is unchanged: third-party signatures (ImGui, Win32,
// the Slamtec SDK, the Pico SDK) keep their own spellings. These aliases are for
// OUR declarations. Do not alias someone else's API to make it look like ours -
// that hides which side of the seam you are on, which is the one thing the
// spelling is there to show.
//
// ---------------------------------------------------------------------------
// SCOPE
//
// Declared at GLOBAL scope on purpose. Wrapping them in a namespace would force
// either a qualified spelling in every header or a `using namespace` in one, and
// `using namespace` in a header is exactly what the style guide forbids.
//
// The counterpart is firmware/lib/shared.hxx - same name on purpose, because it
// is the same idea for the other side. They are kept in step by hand and
// deliberately do not share a file - not because one is C any more (the firmware
// is C++ now too), but because the firmware is freestanding: no heap, no
// exceptions, no STL. Every template below would be unusable there, and a header
// that tried to serve both would be mostly #ifdef.
//
// Nothing includes both, and nothing can: the hub puts only ../shared on its
// include path and firmware targets only firmware/lib, so `#include
// "shared.hxx"` resolves to exactly one file on each side.
// ---------------------------------------------------------------------------
#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <thread>
#include <atomic>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

// ---- integers -------------------------------------------------------------
using Int8   = std::int8_t;
using Int16  = std::int16_t;
using Int32  = std::int32_t;
using Int64  = std::int64_t;

using UInt8  = std::uint8_t;
using UInt16 = std::uint16_t;
using UInt32 = std::uint32_t;
using UInt64 = std::uint64_t;

using Size  = std::size_t;
using ISize = std::ptrdiff_t;
using UPtr  = std::uintptr_t;

// ---- floating point -------------------------------------------------------
using Float32 = float;
using Float64 = double;

// ---- other fundamentals ---------------------------------------------------
using Bool = bool;
using Void = void;
using Char = char;

// `Utf8` and `CharSeq` exist so the C and C++ layers name a borrowed string the
// same way. CharSeq is a raw NUL-terminated pointer - prefer StrView in C++
// unless you are crossing to a C API that wants the pointer.
using Utf8    = char;
using CharSeq = const Utf8*;

// ---- strings --------------------------------------------------------------
using Str     = std::string;
using StrView = std::string_view;

// ---- containers -----------------------------------------------------------
// Vec is the default sequence. Reach for anything else only when the access
// pattern demands it, and say why in a comment when you do.
template<typename T>
using Vec = std::vector<T>;

template<typename T, Size N>
using Array = std::array<T, N>;

// Deque, not Vec, when things are pushed and popped at BOTH ends - a bounded
// log, a frame queue. The only reason to pay for its indirection.
template<typename T>
using Deque = std::deque<T>;

// Ordered. HashMap unless iteration order matters or the key has no hash.
template<typename K, typename V>
using Map = std::map<K, V>;

template<typename K, typename V>
using HashMap = std::unordered_map<K, V>;

template<typename T>
using Set = std::set<T>;

template<typename T>
using HashSet = std::unordered_set<T>;

template<typename A, typename B>
using Pair = std::pair<A, B>;

template<typename... Ts>
using Tuple = std::tuple<Ts...>;

// ---- ownership ------------------------------------------------------------
template<typename T>
using UniqPtr = std::unique_ptr<T>;

template<typename T>
using SharedPtr = std::shared_ptr<T>;

template<typename T>
using WeakPtr = std::weak_ptr<T>;

// `Opt<T>` expresses "this value might not exist" without a heap allocation or
// a nullable pointer. Prefer it over T* whenever no ownership is implied.
template<typename T>
using Opt = std::optional<T>;

// `Variant<Ts...>` replaces a manual union plus an enum discriminant.
template<typename... Ts>
using Variant = std::variant<Ts...>;

// `Fn<Sig>` is for STORED callables. For a callback parameter that is invoked
// and not retained, prefer a constrained template parameter - it avoids the
// heap allocation and type erasure this carries.
template<typename Sig>
using Fn = std::function<Sig>;

// ---- threading ------------------------------------------------------------
using Thread    = std::thread;
using Mutex     = std::mutex;
using RecMutex  = std::recursive_mutex;

// For a worker that WAITS rather than polls. A thread that sleeps for five
// milliseconds in a loop looks like it works and is really a busy-wait with
// latency bolted on; this is the thing that lets it block until there is
// something to do and wake immediately when there is.
using CondVar   = std::condition_variable;

template<typename T>
using LockGuard = std::lock_guard<T>;

template<typename T>
using UniqueLock = std::unique_lock<T>;

template<typename T>
using Atomic = std::atomic<T>;

// ---- time -----------------------------------------------------------------
// STEADY, not system: every use here measures an interval, and the system
// clock can step backwards when something adjusts it. A scan that appears to
// finish before it started is a clock bug wearing a timing bug's clothes.
using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Millis    = std::chrono::milliseconds;
using Nanos     = std::chrono::nanoseconds;

template<typename Rep, typename Period = std::ratio<1>>
using Duration = std::chrono::duration<Rep, Period>;

// ---- streams --------------------------------------------------------------
// Most file reading here is C stdio, which needs no alias - these are for the
// places already written against iostream, so the seam is not half-spelled.
using InFile  = std::ifstream;
using OutFile = std::ofstream;

// ---- helpers --------------------------------------------------------------
// These mirror firmware/lib/hal.hxx's timing:: namespace on purpose, so the
// two halves of the project name the same idea the same way. `monoNow` rather
// than `now` because these sit at global scope and `now` is a local variable
// people actually write.
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

template<typename T, typename... Args>
[[nodiscard]] inline SharedPtr<T> makeShared(Args&&... args)
{
    return std::make_shared<T>(std::forward<Args>(args)...);
}
