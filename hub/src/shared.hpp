// The project's thin alias layer, per the style guide.
//
// These are `using` declarations and nothing more - no macros, no wrapper types.
// Every alias below is the exact type it names, so handing a Float32 to an ImGui
// function that wants a float is not a conversion, it is the same type spelled
// the project's way.
//
// The point is that a width is a Float32 and a count is a Size everywhere in the
// codebase, rather than float/int/size_t/unsigned varying by whoever wrote the
// line. Third-party signatures (ImGui, Win32, the Slamtec SDK) keep their own
// spellings at the boundary; these aliases are for OUR declarations.
//
// Declared at GLOBAL scope on purpose. Wrapping them in a namespace would force
// either a qualified spelling in every header or a `using namespace` in one -
// and `using namespace` in a header is exactly what the guide forbids.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

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

// ---- floating point -------------------------------------------------------
using Float32 = float;
using Float64 = double;

// ---- other fundamentals ---------------------------------------------------
using Bool = bool;
using Void = void;
using Char = char;

// ---- strings --------------------------------------------------------------
using Str     = std::string;
using StrView = std::string_view;

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

template<typename T, typename... Args>
[[nodiscard]] inline UniqPtr<T> makeUniq(Args&&... args)
{
    return std::make_unique<T>(std::forward<Args>(args)...);
}
