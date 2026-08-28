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
// The C counterpart is shared/shared.h. The two are kept in step by hand and
// deliberately do not share a file: C has no templates and no namespaces, and a
// header that tried to serve both would be mostly #ifdef.
// ---------------------------------------------------------------------------
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
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

template<typename T>
using LockGuard = std::lock_guard<T>;

template<typename T>
using UniqueLock = std::unique_lock<T>;

template<typename T>
using Atomic = std::atomic<T>;

// ---- helpers --------------------------------------------------------------
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
