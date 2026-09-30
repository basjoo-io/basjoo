/**
 * @file fixed.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief Fixed-point number type, adapted from https://github.com/MikeLankamp/fpm
 * @version 0.1
 * @date 2026-05-25
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 * This file contains code adapted from fpm (https://github.com/MikeLankamp/fpm),
 * distributed under the following license:
 *
 *   MIT License
 *
 *   Copyright (c) 2019 Mike Lankamp
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 *
 */

#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cassert>
#include <cctype>
#include <climits>
#include <cmath>
#include <compare>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <ios>
#include <istream>
#include <iterator>
#include <limits>
#include <locale>
#include <ostream>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#if defined(_MSC_VER)
#include <boost/multiprecision/cpp_int.hpp>
#include <intrin.h>
#endif

#include "zpp_bits.h"

#include "basjoo/math/concepts.hpp"

namespace basjoo::math {

namespace detail {

// 128-bit integer used as IntermediateType for 64-bit Fixed types.
// GCC/Clang provide a native __int128 that already satisfies std::integral; MSVC has no
// built-in 128-bit integer, so it falls back to Boost.Multiprecision's software int128.
#if defined(_MSC_VER)
using int128_t = boost::multiprecision::int128_t;
#else
__extension__ using int128_t = __int128;
#endif

// Signed integer usable as ValueType or IntermediateType: a native signed integral type, or the
// platform-specific 128-bit fallback above (which has no unsigned counterpart in use here).
template <typename T>
concept SignedIntegral = (std::integral<T> && std::is_signed_v<T>) || std::same_as<T, int128_t>;

[[nodiscard]] constexpr auto maxDigits10(int bits) noexcept -> int {
    using T = long long;
    return static_cast<int>((T{bits} * 5050445 + (T{1} << 24) - 1) >> 24);
}

[[nodiscard]] constexpr auto digits10(int bits) noexcept -> int {
    using T = long long;
    return static_cast<int>((T{bits} * 5050445) >> 24);
}

[[nodiscard]] inline auto findHighestBit(unsigned long long value) noexcept -> long {
    assert(value != 0);
#if defined(_MSC_VER)
    unsigned long index;
#if defined(_WIN64)
    _BitScanReverse64(&index, value);
#else
    if (_BitScanReverse(&index, static_cast<unsigned long>(value >> 32)) != 0) {
        index += 32;
    } else {
        _BitScanReverse(&index, static_cast<unsigned long>(value & 0xfffffffful));
    }
#endif
    return static_cast<long>(index);
#elif defined(__GNUC__) || defined(__clang__)
    return static_cast<long>(sizeof(value) * 8) - 1L - static_cast<long>(__builtin_clzll(value));
#else
#error "findHighestBit(): unsupported platform"
#endif
}

} // namespace detail

template <
    detail::SignedIntegral ValueType, detail::SignedIntegral IntermediateType,
    unsigned int FractionBits, bool EnableRounding = true>
    requires(
        sizeof(IntermediateType) > sizeof(ValueType) && FractionBits < sizeof(ValueType) * 8 &&
        FractionBits > 0
    )
class Fixed;

template <typename T>
struct isFixedPoint final : std::false_type {};

template <
    typename ValueType, typename IntermediateType, unsigned int FractionBits, bool EnableRounding>
struct isFixedPoint<Fixed<ValueType, IntermediateType, FractionBits, EnableRounding>> final
    : std::true_type {};

template <typename T>
inline constexpr bool isFixedPointV = isFixedPoint<T>::value;

template <typename T>
concept FixedPoint = isFixedPointV<T>;

//! Fixed-point number type
//! \tparam ValueType         integer type used to store the fixed-point number
//! \tparam IntermediateType integer type for intermediate calculations; must be wider than
//! ValueType
//! \tparam FractionBits     number of bits of ValueType used for the fractional part
//! \tparam EnableRounding   round the LSB on multiply, divide, and type conversion
template <
    detail::SignedIntegral ValueType, detail::SignedIntegral IntermediateType,
    unsigned int FractionBits, bool EnableRounding>
    requires(
        sizeof(IntermediateType) > sizeof(ValueType) && FractionBits < sizeof(ValueType) * 8 &&
        FractionBits > 0
    )
class Fixed final {
  public:
    using value_type = ValueType;
    using intermediate_type = IntermediateType;
    static constexpr unsigned int kFractionBits{FractionBits};
    static constexpr bool kEnableRounding{EnableRounding};

    constexpr Fixed() noexcept = default;

    // Converts an integral number. Like static_cast, truncates bits that don't fit.
    constexpr explicit Fixed(std::integral auto val) noexcept
        : m_raw_value{static_cast<value_type>(val * kFractionMult)} {}

    // Converts a floating-point number. Like static_cast, truncates bits that don't fit.
    constexpr explicit Fixed(std::floating_point auto val) noexcept
        : m_raw_value{static_cast<value_type>(
              kEnableRounding
                  ? (val >= decltype(val){0}
                         ? val * static_cast<decltype(val)>(kFractionMult) + decltype(val){0.5}
                         : val * static_cast<decltype(val)>(kFractionMult) - decltype(val){0.5})
                  : val * static_cast<decltype(val)>(kFractionMult)
          )} {}

    // Converts from another fixed-point type. Like static_cast, truncates bits that don't fit.
    constexpr explicit Fixed(FixedPoint auto val) noexcept
        : m_raw_value(
              Fixed::template fromFixedPoint<decltype(val)::kFractionBits>(val.raw_value())
                  .raw_value()
          ) {}

    template <std::floating_point T>
    [[nodiscard]] constexpr explicit operator T() const noexcept {
        return static_cast<T>(m_raw_value) / static_cast<T>(kFractionMult);
    }

    template <std::integral T>
    [[nodiscard]] constexpr explicit operator T() const noexcept {
        return static_cast<T>(m_raw_value / kFractionMult);
    }

    [[nodiscard]] constexpr auto raw_value() const noexcept -> value_type { return m_raw_value; }

    template <unsigned int NumFractionBits>
        requires(NumFractionBits > FractionBits)
    [[nodiscard]] static constexpr auto fromFixedPoint(auto value) noexcept -> Fixed {
        return kEnableRounding
                   ? Fixed(
                         static_cast<value_type>(
                             value / (decltype(value)(1) << (NumFractionBits - FractionBits)) +
                             (value / (decltype(value)(1) << (NumFractionBits - FractionBits - 1)) %
                              2)
                         ),
                         RawConstructTag{}
                     )
                   : Fixed(
                         static_cast<value_type>(
                             value / (decltype(value)(1) << (NumFractionBits - FractionBits))
                         ),
                         RawConstructTag{}
                     );
    }

    template <unsigned int NumFractionBits>
        requires(NumFractionBits <= FractionBits)
    [[nodiscard]] static constexpr auto fromFixedPoint(auto value) noexcept -> Fixed {
        return Fixed(
            static_cast<value_type>(
                value * (decltype(value)(1) << (FractionBits - NumFractionBits))
            ),
            RawConstructTag{}
        );
    }

    [[nodiscard]] static constexpr auto fromRawValue(value_type value) noexcept -> Fixed {
        return Fixed(value, RawConstructTag{});
    }

    //! Constructs from integer part and fraction part.
    //! \tparam NumFraction denominator of fraction_value (e.g. 1'000'000 for millionths)
    template <unsigned long long NumFraction, std::integral T>
        requires(NumFraction > (static_cast<unsigned long long>(1) << kFractionBits))
    [[nodiscard]] static constexpr auto fromCustomFraction(
        T integer_value, T fraction_value
    ) noexcept -> Fixed {
        const intermediate_type int_part =
            static_cast<intermediate_type>(integer_value) * kFractionMult;
        const intermediate_type frac_part = static_cast<intermediate_type>(fraction_value) *
                                            kFractionMult /
                                            static_cast<intermediate_type>(NumFraction);
        const intermediate_type two_frac = static_cast<intermediate_type>(fraction_value) *
                                           kFractionMult * 2 /
                                           static_cast<intermediate_type>(NumFraction);
        return kEnableRounding
                   ? Fixed(
                         static_cast<value_type>(int_part + frac_part + two_frac % 2),
                         RawConstructTag{}
                     )
                   : Fixed(static_cast<value_type>(int_part + frac_part), RawConstructTag{});
    }

    template <unsigned long long NumFraction, std::integral T>
        requires(NumFraction <= (static_cast<unsigned long long>(1) << FractionBits))
    [[nodiscard]] static constexpr auto fromCustomFraction(
        T integer_value, T fraction_value
    ) noexcept -> Fixed {
        const intermediate_type int_part =
            static_cast<intermediate_type>(integer_value) * kFractionMult;
        const intermediate_type frac_part = static_cast<intermediate_type>(fraction_value) *
                                            kFractionMult /
                                            static_cast<intermediate_type>(NumFraction);
        return Fixed(static_cast<value_type>(int_part + frac_part), RawConstructTag{});
    }

    // Mathematical constants
    [[nodiscard]] static constexpr auto e() noexcept -> Fixed {
        return fromFixedPoint<61>(6267931151224907085LL);
    }
    [[nodiscard]] static constexpr auto pi() noexcept -> Fixed {
        return fromFixedPoint<61>(7244019458077122842LL);
    }
    [[nodiscard]] static constexpr auto halfPi() noexcept -> Fixed {
        return fromFixedPoint<62>(7244019458077122842LL);
    }
    [[nodiscard]] static constexpr auto twoPi() noexcept -> Fixed {
        return fromFixedPoint<60>(7244019458077122842LL);
    }

    // Binary/unary operators on rvalue operands mutate and return the operand itself as `Fixed&&`
    // (zero copy); lvalue-only operands yield a new `Fixed` by value. Binding the result of an
    // expression whose reused operand is a temporary to a reference outlives that temporary.
    template <typename Self>
        requires std::same_as<std::remove_cvref_t<Self>, Fixed>
    [[nodiscard]] [[using gnu: always_inline, hot]]
    constexpr auto operator-(this Self&& self) noexcept -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self.m_raw_value = -self.m_raw_value;
            return static_cast<Self&&>(self);
        } else {
            return Fixed::fromRawValue(-self.m_raw_value);
        }
    }

    template <typename Self, typename Obj>
        requires(!std::is_const_v<std::remove_reference_t<Self>>) &&
                (std::same_as<Obj, Fixed> || std::integral<Obj>)
    [[using gnu: always_inline, hot]]
    constexpr auto operator+=(this Self&& self, const Obj& obj) noexcept -> Fixed& {
        if constexpr (std::integral<Obj>) {
            self.m_raw_value += static_cast<value_type>(obj * kFractionMult);
        } else {
            self.m_raw_value += obj.m_raw_value;
        }
        return self;
    }

    template <typename Self, typename Obj>
        requires std::same_as<std::remove_cvref_t<Self>, Fixed> &&
                 (std::same_as<std::remove_cvref_t<Obj>, Fixed> ||
                  std::integral<std::remove_cvref_t<Obj>>)
    [[using gnu: always_inline, hot]]
    constexpr auto operator+(this Self&& self, Obj&& obj) noexcept -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self += obj;
            return static_cast<Self&&>(self);
        } else if constexpr (Reusable<Obj> && std::same_as<std::remove_cvref_t<Obj>, Fixed>) {
            obj += self;
            return static_cast<Obj&&>(obj);
        } else {
            Fixed result{self};
            result += obj;
            return result;
        }
    }

    template <typename Self, typename Obj>
        requires(!std::is_const_v<std::remove_reference_t<Self>>) &&
                (std::same_as<Obj, Fixed> || std::integral<Obj>)
    [[using gnu: always_inline, hot]]
    constexpr auto operator-=(this Self&& self, const Obj& obj) noexcept -> Fixed& {
        if constexpr (std::integral<Obj>) {
            self.m_raw_value -= static_cast<value_type>(obj * kFractionMult);
        } else {
            self.m_raw_value -= obj.m_raw_value;
        }
        return self;
    }

    template <typename Self, typename Obj>
        requires std::same_as<std::remove_cvref_t<Self>, Fixed> &&
                 (std::same_as<std::remove_cvref_t<Obj>, Fixed> ||
                  std::integral<std::remove_cvref_t<Obj>>)
    [[using gnu: always_inline, hot]]
    constexpr auto operator-(this Self&& self, Obj&& obj) noexcept -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self -= obj;
            return static_cast<Self&&>(self);
        } else if constexpr (Reusable<Obj> && std::same_as<std::remove_cvref_t<Obj>, Fixed>) {
            Fixed result{self};
            result -= obj;
            obj = result;
            return static_cast<Obj&&>(obj);
        } else {
            Fixed result{self};
            result -= obj;
            return result;
        }
    }

    template <typename Self, typename Obj>
        requires(!std::is_const_v<std::remove_reference_t<Self>>) &&
                (std::same_as<Obj, Fixed> || std::integral<Obj>)
    [[using gnu: always_inline, hot]]
    constexpr auto operator*=(this Self&& self, const Obj& obj) noexcept -> Fixed& {
        if constexpr (std::integral<Obj>) {
            self.m_raw_value *= obj;
        } else if constexpr (kEnableRounding) {
            // Multiply by two before dividing, add the LSB to round correctly.
            auto value = (static_cast<intermediate_type>(self.m_raw_value) * obj.m_raw_value) /
                         (kFractionMult / 2);
            self.m_raw_value = static_cast<value_type>((value / 2) + (value % 2));
        } else {
            self.m_raw_value = static_cast<value_type>(
                (static_cast<intermediate_type>(self.m_raw_value) * obj.m_raw_value) / kFractionMult
            );
        }
        return self;
    }

    template <typename Self, typename Obj>
        requires std::same_as<std::remove_cvref_t<Self>, Fixed> &&
                 (std::same_as<std::remove_cvref_t<Obj>, Fixed> ||
                  std::integral<std::remove_cvref_t<Obj>>)
    [[using gnu: always_inline, hot]]
    constexpr auto operator*(this Self&& self, Obj&& obj) noexcept -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self *= obj;
            return static_cast<Self&&>(self);
        } else if constexpr (Reusable<Obj> && std::same_as<std::remove_cvref_t<Obj>, Fixed>) {
            obj *= self;
            return static_cast<Obj&&>(obj);
        } else {
            Fixed result{self};
            result *= obj;
            return result;
        }
    }

    template <typename Self, typename Obj>
        requires(!std::is_const_v<std::remove_reference_t<Self>>) &&
                (std::same_as<Obj, Fixed> || std::integral<Obj>)
    [[using gnu: always_inline, hot]]
    constexpr auto operator/=(this Self&& self, const Obj& obj) noexcept -> Fixed& {
        if constexpr (std::integral<Obj>) {
            assert(obj != 0);
            self.m_raw_value /= obj;
        } else {
            assert(obj.m_raw_value != 0);
            if constexpr (kEnableRounding) {
                // Multiply by two before dividing, add the LSB to round correctly.
                auto value =
                    (static_cast<intermediate_type>(self.m_raw_value) * kFractionMult * 2) /
                    obj.m_raw_value;
                self.m_raw_value = static_cast<value_type>((value / 2) + (value % 2));
            } else {
                self.m_raw_value = static_cast<value_type>(
                    (static_cast<intermediate_type>(self.m_raw_value) * kFractionMult) /
                    obj.m_raw_value
                );
            }
        }
        return self;
    }

    template <typename Self, typename Obj>
        requires std::same_as<std::remove_cvref_t<Self>, Fixed> &&
                 (std::same_as<std::remove_cvref_t<Obj>, Fixed> ||
                  std::integral<std::remove_cvref_t<Obj>>)
    [[using gnu: always_inline, hot]]
    constexpr auto operator/(this Self&& self, Obj&& obj) noexcept -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self /= obj;
            return static_cast<Self&&>(self);
        } else if constexpr (Reusable<Obj> && std::same_as<std::remove_cvref_t<Obj>, Fixed>) {
            Fixed result{self};
            result /= obj;
            obj = result;
            return static_cast<Obj&&>(obj);
        } else {
            Fixed result{self};
            result /= obj;
            return result;
        }
    }

    [[nodiscard]] friend constexpr auto operator==(const Fixed& x, const Fixed& y) noexcept
        -> bool {
        return x.m_raw_value == y.m_raw_value;
    }

    [[nodiscard]] friend constexpr auto operator<=>(const Fixed& x, const Fixed& y) noexcept
        -> std::strong_ordering {
        return x.m_raw_value <=> y.m_raw_value;
    }

  private:
    static constexpr intermediate_type kFractionMult{intermediate_type(1) << FractionBits};

    struct RawConstructTag {};
    constexpr Fixed(value_type val, RawConstructTag /*tag*/) noexcept : m_raw_value{val} {}

    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    value_type m_raw_value{0};
};

using Fixed32f8 = Fixed<std::int32_t, std::int64_t, 8>;
using Fixed32f16 = Fixed<std::int32_t, std::int64_t, 16>;
using Fixed32f24 = Fixed<std::int32_t, std::int64_t, 24>;

using Fixed64f8 = Fixed<std::int64_t, detail::int128_t, 8>;
using Fixed64f16 = Fixed<std::int64_t, detail::int128_t, 16>;
using Fixed64f24 = Fixed<std::int64_t, detail::int128_t, 24>;
using Fixed64f32 = Fixed<std::int64_t, detail::int128_t, 32>;
using Fixed64f40 = Fixed<std::int64_t, detail::int128_t, 40>;
using Fixed64f48 = Fixed<std::int64_t, detail::int128_t, 48>;
using Fixed64f56 = Fixed<std::int64_t, detail::int128_t, 56>;

template <typename T>
    requires FixedPoint<std::remove_cvref_t<T>>
[[using gnu: always_inline, hot]]
inline constexpr auto operator+(const std::integral auto& fac, T&& obj) noexcept -> decltype(auto) {
    if constexpr (Reusable<T>) {
        obj += fac;
        return static_cast<T&&>(obj);
    } else {
        std::remove_cvref_t<T> result{obj};
        result += fac;
        return result;
    }
}

template <typename T>
    requires FixedPoint<std::remove_cvref_t<T>>
[[using gnu: always_inline, hot]]
inline constexpr auto operator-(const std::integral auto& fac, T&& obj) noexcept -> decltype(auto) {
    std::remove_cvref_t<T> result(fac);
    result -= obj;
    if constexpr (Reusable<T>) {
        obj = result;
        return static_cast<T&&>(obj);
    } else {
        return result;
    }
}

template <typename T>
    requires FixedPoint<std::remove_cvref_t<T>>
[[using gnu: always_inline, hot]]
inline constexpr auto operator*(const std::integral auto& fac, T&& obj) noexcept -> decltype(auto) {
    if constexpr (Reusable<T>) {
        obj *= fac;
        return static_cast<T&&>(obj);
    } else {
        std::remove_cvref_t<T> result{obj};
        result *= fac;
        return result;
    }
}

template <typename T>
    requires FixedPoint<std::remove_cvref_t<T>>
[[using gnu: always_inline, hot]]
inline constexpr auto operator/(const std::integral auto& fac, T&& obj) noexcept -> decltype(auto) {
    std::remove_cvref_t<T> result(fac);
    result /= obj;
    if constexpr (Reusable<T>) {
        obj = result;
        return static_cast<T&&>(obj);
    } else {
        return result;
    }
}

} // namespace basjoo::math

// NOLINTBEGIN(cert-dcl58-cpp)
namespace std {

template <::basjoo::math::FixedPoint T>
struct hash<T> {
    using argument_type = T;
    using result_type = std::size_t;

    auto operator()(argument_type arg) const noexcept(
        noexcept(std::declval<std::hash<typename argument_type::value_type>>()(arg.raw_value()))
    ) -> result_type {
        return m_hash(arg.raw_value());
    }

  private:
    std::hash<typename argument_type::value_type> m_hash;
};

template <::basjoo::math::FixedPoint T>
struct numeric_limits<T> {
    using fixed_type = T;
    using value_type = typename T::value_type;
    static constexpr unsigned int kFractionBits{T::kFractionBits};

    static constexpr bool is_specialized = true;
    static constexpr bool is_signed = std::numeric_limits<value_type>::is_signed;
    static constexpr bool is_integer = false;
    static constexpr bool is_exact = true;
    static constexpr bool has_infinity = false;
    static constexpr bool has_quiet_NaN = false;
    static constexpr bool has_signaling_NaN = false;
    static constexpr std::float_round_style round_style = std::round_to_nearest;
    static constexpr bool is_iec559 = false;
    static constexpr bool is_bounded = true;
    static constexpr bool is_modulo = std::numeric_limits<value_type>::is_modulo;
    static constexpr int digits = std::numeric_limits<value_type>::digits;
    static constexpr int digits10 = 1;
    static constexpr int max_digits10 =
        ::basjoo::math::detail::maxDigits10(
            std::numeric_limits<value_type>::digits - static_cast<int>(kFractionBits)
        ) +
        ::basjoo::math::detail::maxDigits10(static_cast<int>(kFractionBits));
    static constexpr int radix = 2;
    static constexpr int min_exponent = 1 - static_cast<int>(kFractionBits);
    static constexpr int min_exponent10 =
        -::basjoo::math::detail::digits10(static_cast<int>(kFractionBits));
    static constexpr int max_exponent =
        std::numeric_limits<value_type>::digits - static_cast<int>(kFractionBits);
    static constexpr int max_exponent10 = ::basjoo::math::detail::digits10(
        std::numeric_limits<value_type>::digits - static_cast<int>(kFractionBits)
    );
    static constexpr bool traps = true;
    static constexpr bool tinyness_before = false;

    [[nodiscard]] static constexpr auto lowest() noexcept -> fixed_type {
        return fixed_type::fromRawValue(std::numeric_limits<value_type>::lowest());
    }
    [[nodiscard]] static constexpr auto min() noexcept -> fixed_type { return lowest(); }
    [[nodiscard]] static constexpr auto max() noexcept -> fixed_type {
        return fixed_type::fromRawValue(std::numeric_limits<value_type>::max());
    }
    [[nodiscard]] static constexpr auto epsilon() noexcept -> fixed_type {
        return fixed_type::fromRawValue(1);
    }
    [[nodiscard]] static constexpr auto round_error() noexcept -> fixed_type {
        return fixed_type{1} / 2;
    }
};

} // namespace std
// NOLINTEND(cert-dcl58-cpp)

namespace basjoo::math {

[[nodiscard]] constexpr auto fpclassify(FixedPoint auto x) noexcept -> int {
    return (x.raw_value() == 0) ? FP_ZERO : FP_NORMAL;
}

[[nodiscard]] constexpr auto isfinite(FixedPoint auto /*x*/) noexcept -> bool { return true; }

[[nodiscard]] constexpr auto isinf(FixedPoint auto /*x*/) noexcept -> bool { return false; }

[[nodiscard]] constexpr auto isnan(FixedPoint auto /*x*/) noexcept -> bool { return false; }

[[nodiscard]] constexpr auto isnormal(FixedPoint auto x) noexcept -> bool {
    return x.raw_value() != 0;
}

[[nodiscard]] constexpr auto signbit(FixedPoint auto x) noexcept -> bool {
    return x.raw_value() < 0;
}

template <FixedPoint T>
[[nodiscard]] constexpr auto isgreater(T x, T y) noexcept -> bool {
    return x > y;
}

template <FixedPoint T>
[[nodiscard]] constexpr auto isgreaterequal(T x, T y) noexcept -> bool {
    return x >= y;
}

template <FixedPoint T>
[[nodiscard]] constexpr auto isless(T x, T y) noexcept -> bool {
    return x < y;
}

template <FixedPoint T>
[[nodiscard]] constexpr auto islessequal(T x, T y) noexcept -> bool {
    return x <= y;
}

template <FixedPoint T>
[[nodiscard]] constexpr auto islessgreater(T x, T y) noexcept -> bool {
    return x != y;
}

template <FixedPoint T>
[[nodiscard]] constexpr auto isunordered(T /*x*/, T /*y*/) noexcept -> bool {
    return false;
}

[[nodiscard]] constexpr auto ceil(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    constexpr auto kFrac = typename Self::value_type(1) << Self::kFractionBits;
    auto value = x.raw_value();
    if (value > 0) {
        value += kFrac - 1;
    }
    return Self::fromRawValue(value / kFrac * kFrac);
}

[[nodiscard]] constexpr auto floor(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    constexpr auto kFrac = typename Self::value_type(1) << Self::kFractionBits;
    auto value = x.raw_value();
    if (value < 0) {
        value -= kFrac - 1;
    }
    return Self::fromRawValue(value / kFrac * kFrac);
}

[[nodiscard]] constexpr auto trunc(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    constexpr auto kFrac = typename Self::value_type(1) << Self::kFractionBits;
    return Self::fromRawValue(x.raw_value() / kFrac * kFrac);
}

[[nodiscard]] constexpr auto round(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    constexpr auto kFrac = typename Self::value_type(1) << Self::kFractionBits;
    auto value = x.raw_value() / (kFrac / 2);
    return Self::fromRawValue(((value / 2) + (value % 2)) * kFrac);
}

[[nodiscard]] constexpr auto nearbyint(FixedPoint auto x) noexcept -> decltype(x) {
    // Rounding mode assumed to be FE_TONEAREST (round half to even)
    using Self = std::remove_cvref_t<decltype(x)>;
    constexpr auto kFrac = typename Self::value_type(1) << Self::kFractionBits;
    auto value = x.raw_value();
    const auto rem = value % kFrac;
    const bool is_half = (rem < 0 ? -rem : rem) == kFrac / 2;
    value /= kFrac / 2;
    value = (value / 2) + (value % 2);
    value -= (value % 2) * is_half;
    return Self::fromRawValue(value * kFrac);
}

[[nodiscard]] constexpr auto rint(FixedPoint auto x) noexcept -> decltype(x) {
    return nearbyint(x);
}

[[nodiscard]] constexpr auto abs(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    return (x >= Self{0}) ? x : -x;
}

template <FixedPoint T>
[[nodiscard]] constexpr auto fmod(T x, T y) noexcept -> T {
    assert(y.raw_value() != 0);
    return T::fromRawValue(x.raw_value() % y.raw_value());
}

template <FixedPoint T>
[[nodiscard]] constexpr auto remainder(T x, T y) noexcept -> T {
    assert(y.raw_value() != 0);
    return x - nearbyint(x / y) * y;
}

template <FixedPoint T>
auto remquo(T x, T y, int* quo) noexcept -> T {
    assert(y.raw_value() != 0);
    assert(quo != nullptr);
    *quo = x.raw_value() / y.raw_value();
    return T::fromRawValue(x.raw_value() % y.raw_value());
}

[[nodiscard]] constexpr auto copysign(FixedPoint auto x, FixedPoint auto y) noexcept
    -> decltype(x) {
    using SelfY = std::remove_cvref_t<decltype(y)>;
    x = abs(x);
    return (y >= SelfY{0}) ? x : -x;
}

template <FixedPoint T>
[[nodiscard]] constexpr auto nextafter(T from, T to) noexcept -> T {
    return from == to  ? to
           : to > from ? T::fromRawValue(from.raw_value() + 1)
                       : T::fromRawValue(from.raw_value() - 1);
}

template <FixedPoint T>
[[nodiscard]] constexpr auto nexttoward(T from, T to) noexcept -> T {
    return nextafter(from, to);
}

template <FixedPoint T>
auto modf(T x, T* iptr) noexcept -> T {
    const auto raw = x.raw_value();
    constexpr auto kFrac = typename T::value_type{1} << T::kFractionBits;
    *iptr = T::fromRawValue(raw / kFrac * kFrac);
    return T::fromRawValue(raw % kFrac);
}

// Forward declarations needed for mutual recursion in pow / exp2 / log2
[[nodiscard]] auto exp2(FixedPoint auto x) noexcept -> decltype(x);
[[nodiscard]] auto log2(FixedPoint auto x) noexcept -> decltype(x);

[[nodiscard]] auto pow(FixedPoint auto base, std::integral auto exp) noexcept -> decltype(base) {
    using fixed_type = std::remove_cvref_t<decltype(base)>;
    if (base == fixed_type{0}) {
        assert(exp > 0);
        return fixed_type{0};
    }
    fixed_type result{1};
    if (exp < 0) {
        for (fixed_type inter = base; exp != 0; exp /= 2, inter *= inter) {
            if (exp % 2 != 0) {
                result /= inter;
            }
        }
    } else {
        for (fixed_type inter = base; exp != 0; exp /= 2, inter *= inter) {
            if (exp % 2 != 0) {
                result *= inter;
            }
        }
    }
    return result;
}

template <FixedPoint T>
[[nodiscard]] auto pow(T base, T exp) noexcept -> T {
    if (base == T{0}) {
        assert(exp > T{0});
        return T{0};
    }
    if (exp < T{0}) {
        return T{1} / pow(base, -exp);
    }
    constexpr auto kFrac = typename T::value_type(1) << T::kFractionBits;
    if (exp.raw_value() % kFrac == 0) {
        return pow(base, static_cast<int>(exp.raw_value() / kFrac));
    }
    assert(base > T{0});
    return exp2(log2(base) * exp);
}

[[nodiscard]] auto exp(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    if (x < Self{0}) {
        return Self{1} / exp(-x);
    }
    constexpr auto kFrac = typename Self::value_type(1) << Self::kFractionBits;
    const typename Self::value_type x_int = x.raw_value() / kFrac;
    x -= x_int;
    assert(x >= Self{0} && x < Self{1});

    constexpr auto fA = Self::template fromFixedPoint<63>(128239257017632854LL);
    constexpr auto fB = Self::template fromFixedPoint<63>(320978614890280666LL);
    constexpr auto fC = Self::template fromFixedPoint<63>(1571680799599592947LL);
    constexpr auto fD = Self::template fromFixedPoint<63>(4603349000587966862LL);
    constexpr auto fE = Self::template fromFixedPoint<62>(4612052447974689712LL);
    constexpr auto fF = Self::template fromFixedPoint<63>(9223361618412247875LL);
    return pow(Self::e(), x_int) * (((((fA * x + fB) * x + fC) * x + fD) * x + fE) * x + fF);
}

[[nodiscard]] auto exp2(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    if (x < Self{0}) {
        return Self{1} / exp2(-x);
    }
    constexpr auto kFrac = typename Self::value_type(1) << Self::kFractionBits;
    const typename Self::value_type x_int = x.raw_value() / kFrac;
    x -= x_int;
    assert(x >= Self{0} && x < Self{1});

    constexpr auto fA = Self::template fromFixedPoint<63>(17491766697771214LL);
    constexpr auto fB = Self::template fromFixedPoint<63>(82483038782406547LL);
    constexpr auto fC = Self::template fromFixedPoint<63>(515275173969157690LL);
    constexpr auto fD = Self::template fromFixedPoint<63>(2214897896212987987LL);
    constexpr auto fE = Self::template fromFixedPoint<63>(6393224161192452326LL);
    constexpr auto fF = Self::template fromFixedPoint<63>(9223371050976163566LL);
    return Self{1 << static_cast<int>(x_int)} *
           (((((fA * x + fB) * x + fC) * x + fD) * x + fE) * x + fF);
}

[[nodiscard]] auto expm1(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    return exp(x) - Self{1};
}

[[nodiscard]] auto log2(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    assert(x > Self{0});

    typename Self::value_type value = x.raw_value();
    const long highest = detail::findHighestBit(static_cast<unsigned long long>(value));
    const int shift = static_cast<int>(highest) - static_cast<int>(Self::kFractionBits);
    if (shift >= 0) {
        value >>= static_cast<unsigned int>(shift);
    } else {
        value <<= static_cast<unsigned int>(-shift);
    }
    x = Self::fromRawValue(value);
    assert(x >= Self{1} && x < Self{2});

    constexpr auto fA = Self::template fromFixedPoint<63>(413886001457275979LL);
    constexpr auto fB = Self::template fromFixedPoint<63>(-3842121857793256941LL);
    constexpr auto fC = Self::template fromFixedPoint<62>(7522345947206307744LL);
    constexpr auto fD = Self::template fromFixedPoint<61>(-8187571043052183818LL);
    constexpr auto fE = Self::template fromFixedPoint<60>(5870342889289496598LL);
    constexpr auto fF = Self::template fromFixedPoint<61>(-6457199832668582866LL);
    return Self{static_cast<int>(highest) - static_cast<int>(Self::kFractionBits)} +
           (((((fA * x + fB) * x + fC) * x + fD) * x + fE) * x + fF);
}

[[nodiscard]] auto log(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    return log2(x) / log2(Self::e());
}

[[nodiscard]] auto log10(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    return log2(x) / log2(Self{10});
}

[[nodiscard]] auto log1p(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    return log(Self{1} + x);
}

[[nodiscard]] auto cbrt(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    if (x == Self{0}) {
        return x;
    }
    if (x < Self{0}) {
        return -cbrt(-x);
    }
    assert(x > Self{0});

    // Hacker's Delight cube-root algorithm (integer, adapted for fixed-point)
    int ofs = static_cast<int>(
        (detail::findHighestBit(static_cast<unsigned long long>(x.raw_value())) +
         2L * static_cast<long>(Self::kFractionBits)) /
        3L * 3L
    );
    typename Self::intermediate_type num = typename Self::intermediate_type{x.raw_value()};
    typename Self::intermediate_type res = 0;

    const auto do_round = [&]() {
        for (; ofs >= 0; ofs -= 3) {
            res += res;
            const typename Self::intermediate_type val = (3 * res * (res + 1) + 1) << ofs;
            if (num >= val) {
                num -= val;
                res++;
            }
        }
    };

    num <<= Self::kFractionBits;
    ofs -= static_cast<int>(Self::kFractionBits);
    do_round();

    num <<= Self::kFractionBits;
    ofs += static_cast<int>(Self::kFractionBits);
    do_round();

    return Self::fromRawValue(static_cast<typename Self::value_type>(res));
}

[[nodiscard]] auto sqrt(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    assert(x >= Self{0});
    if (x == Self{0}) {
        return x;
    }

    // Binary square root, shifted by F for fixed-point precision
    typename Self::intermediate_type num = typename Self::intermediate_type{x.raw_value()}
                                           << Self::kFractionBits;
    typename Self::intermediate_type res = 0;

    const long hbit = detail::findHighestBit(static_cast<unsigned long long>(x.raw_value()));
    for (typename Self::intermediate_type bit =
             typename Self::intermediate_type{1}
             << static_cast<int>((hbit + static_cast<long>(Self::kFractionBits)) / 2L * 2L);
         bit != 0; bit >>= 2) {
        const typename Self::intermediate_type val = res + bit;
        res >>= 1;
        if (num >= val) {
            num -= val;
            res += bit;
        }
    }
    if (num > res) {
        res++;
    }
    return Self::fromRawValue(static_cast<typename Self::value_type>(res));
}

template <FixedPoint T>
[[nodiscard]] auto hypot(T x, T y) noexcept -> T {
    assert(
        (x.raw_value() != typename T::value_type{} || y.raw_value() != typename T::value_type{})
    );
    return sqrt(x * x + y * y);
}

[[nodiscard]] auto sin(FixedPoint auto x) noexcept -> decltype(x) {
    // 5th-order curve-fitting approximation by Jasper Vijn, worst-case relative error ~0.07%
    using Self = std::remove_cvref_t<decltype(x)>;
    x = fmod(x, Self::twoPi());
    x = x / Self::halfPi();

    if (x < Self{0}) {
        x += Self{4};
    }

    int sign = +1;
    if (x > Self{2}) {
        sign = -1;
        x -= Self{2};
    }
    if (x > Self{1}) {
        x = Self{2} - x;
    }

    const Self x2 = x * x;
    return sign * x * (Self::pi() - x2 * (Self::twoPi() - 5 - x2 * (Self::pi() - 3))) / 2;
}

[[nodiscard]] auto cos(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    if (x > Self{0}) {
        return sin(x - (Self::twoPi() - Self::halfPi()));
    }
    return sin(Self::halfPi() + x);
}

[[nodiscard]] auto tan(FixedPoint auto x) noexcept -> decltype(x) {
    auto cx = cos(x);
    assert(abs(cx).raw_value() > 1);
    return sin(x) / cx;
}

namespace detail {

// Calculates atan(x) for x in [0, 1]
[[nodiscard]] auto atanSanitized(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    assert(x >= Self{0} && x <= Self{1});

    constexpr auto fA = Self::template fromFixedPoint<63>(716203666280654660LL);
    constexpr auto fB = Self::template fromFixedPoint<63>(-2651115102768076601LL);
    constexpr auto fC = Self::template fromFixedPoint<63>(9178930894564541004LL);
    const auto xx = x * x;
    return ((fA * xx + fB) * xx + fC) * x;
}

// Calculates atan(y / x), avoiding division overflow for very small x
template <FixedPoint T>
[[nodiscard]] auto atanDiv(T y, T x) noexcept -> T {
    assert(x != T{0});

    if (y < T{0}) {
        return (x < T{0}) ? atanDiv(-y, -x) : -atanDiv(-y, x);
    }
    if (x < T{0}) {
        return -atanDiv(y, -x);
    }
    assert(y >= T{0} && x > T{0});

    return (y > x) ? T::halfPi() - atanSanitized(x / y) : atanSanitized(y / x);
}

} // namespace detail

[[nodiscard]] auto atan(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    if (x < Self{0}) {
        return -atan(-x);
    }
    if (x > Self{1}) {
        return Self::halfPi() - detail::atanSanitized(Self{1} / x);
    }
    return detail::atanSanitized(x);
}

[[nodiscard]] auto asin(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    assert(x >= Self{-1} && x <= Self{1});

    const auto yy = Self{1} - x * x;
    if (yy == Self{0}) {
        return copysign(Self::halfPi(), x);
    }
    return detail::atanDiv(x, sqrt(yy));
}

[[nodiscard]] auto acos(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    assert(x >= Self{-1} && x <= Self{1});

    if (x == Self{-1}) {
        return Self::pi();
    }
    const auto yy = Self{1} - x * x;
    return Self{2} * detail::atanDiv(sqrt(yy), Self{1} + x);
}

template <FixedPoint T>
[[nodiscard]] auto atan2(T y, T x) noexcept -> T {
    if (x == T{0}) {
        assert(y != T{0});
        return (y > T{0}) ? T::halfPi() : -T::halfPi();
    }
    auto ret = detail::atanDiv(y, x);
    if (x < T{0}) {
        return (y >= T{0}) ? ret + T::pi() : ret - T::pi();
    }
    return ret;
}

namespace detail {

enum class Align : std::uint8_t {
    NONE,
    LEFT,
    RIGHT,
    CENTER,
    INTERNAL
};

// A std-format-spec for Fixed. std::formatter fills it from the format string; operator<< fills
// it from the std::ios_base state. `type` is one of "fFeEgGaA", or '\0' for the shortest form
// that reads back as the same Fixed.
template <typename Char>
struct FormatSpec {
    Char fill{static_cast<Char>(' ')};
    Align align{Align::NONE};
    char sign{'-'};
    char type{'\0'};
    bool alt{false};
    bool zero{false};
    bool localized{false};
    bool hex_prefix{false}; // "0x" in front of hexadecimal output, as std::ostream prints it
    std::size_t width{0};
    int precision{-1};
    int width_arg{-1};
    int precision_arg{-1};
};

// The type letter of the format spec that `c` is, or '\0'.
template <typename Char>
constexpr auto toType(Char c) noexcept -> char {
    for (const char type : std::string_view{"fFeEgGaA"}) {
        if (c == static_cast<Char>(type)) {
            return type;
        }
    }
    return '\0';
}

template <typename Char>
constexpr auto parseSpec(std::basic_format_parse_context<Char>& ctx, FormatSpec<Char>& spec)
    -> std::basic_format_parse_context<Char>::iterator {
    auto it{ctx.begin()};
    const auto end{ctx.end()};
    const auto is = [&](char c) constexpr noexcept -> bool {
        return it != end && *it == static_cast<Char>(c);
    };
    const auto at_end = [&]() constexpr noexcept -> bool { return it == end || is('}'); };
    const auto to_align = [](Char c) constexpr noexcept -> Align {
        return c == '<'   ? Align::LEFT
               : c == '>' ? Align::RIGHT
               : c == '^' ? Align::CENTER
                          : Align::NONE;
    };
    const auto number = [&]() constexpr -> int {
        long long value{0};
        while (it != end && *it >= '0' && *it <= '9') {
            value = value * 10 + (*it - '0');
            if (value > std::numeric_limits<int>::max()) {
                throw std::format_error("number is too large in the format spec");
            }
            ++it;
        }
        return static_cast<int>(value);
    };
    // "{}" or "{n}": the width or precision comes from another argument.
    const auto arg_id = [&]() constexpr -> int {
        ++it;
        int id{0};
        if (is('}')) {
            id = static_cast<int>(ctx.next_arg_id());
        } else {
            id = number();
            ctx.check_arg_id(static_cast<std::size_t>(id));
        }
        if (!is('}')) {
            throw std::format_error("invalid dynamic width or precision in the format spec");
        }
        ++it;
        return id;
    };

    if (at_end()) {
        return it;
    }
    if (it + 1 != end && to_align(it[1]) != Align::NONE) {
        if (is('{')) {
            throw std::format_error("'{' is not a valid fill character");
        }
        spec.fill = *it;
        spec.align = to_align(it[1]);
        it += 2;
    } else if (to_align(*it) != Align::NONE) {
        spec.align = to_align(*it);
        ++it;
    }
    if (is('+') || is('-') || is(' ')) {
        spec.sign = static_cast<char>(*it);
        ++it;
    }
    if (is('#')) {
        spec.alt = true;
        ++it;
    }
    if (is('0')) {
        spec.zero = true;
        ++it;
    }
    if (is('{')) {
        spec.width_arg = arg_id();
    } else if (it != end && *it >= '1' && *it <= '9') {
        spec.width = static_cast<std::size_t>(number());
    }
    if (is('.')) {
        ++it;
        if (is('{')) {
            spec.precision_arg = arg_id();
        } else if (it != end && *it >= '0' && *it <= '9') {
            spec.precision = number();
        } else {
            throw std::format_error("missing precision in the format spec");
        }
    }
    if (is('L')) {
        spec.localized = true;
        ++it;
    }
    if (!at_end()) {
        spec.type = toType(*it);
        if (spec.type == '\0') {
            throw std::format_error("invalid type in the format spec");
        }
        ++it;
    }
    if (!at_end()) {
        throw std::format_error("invalid format spec");
    }
    return it;
}

// Width or precision taken from an argument of the format call.
template <typename FmtCtx>
auto dynamicSize(const std::basic_format_arg<FmtCtx>& arg) -> int {
    return std::visit_format_arg(
        [](auto value) -> int {
            using Value = decltype(value);
            if constexpr (
                std::integral<Value> && !std::same_as<Value, bool> &&
                !std::same_as<Value, typename FmtCtx::char_type>
            ) {
                if (std::cmp_less(value, 0) ||
                    std::cmp_greater(value, std::numeric_limits<int>::max())) {
                    throw std::format_error("width or precision argument is out of range");
                }
                return static_cast<int>(value);
            } else {
                throw std::format_error("width or precision argument is not an integer");
            }
        },
        arg
    );
}

inline constexpr auto kPow10 = []() constexpr {
    std::array<std::uint64_t, 20> table{1};
    for (std::size_t i{1}; i < table.size(); ++i) {
        table[i] = table[i - 1] * 10;
    }
    return table;
}();

// Writes `value` in `Base` as digit values 0..Base-1, at least `min_digits` of them, backwards from
// `end`. Returns the first written digit. std::to_chars is of no use here: it writes characters.
template <unsigned Base>
constexpr auto writeUnsigned(std::uint8_t* end, std::uint64_t value, int min_digits = 1) noexcept
    -> std::uint8_t* {
    for (int count{0}; value != 0 || count < min_digits; ++count) {
        *--end = static_cast<std::uint8_t>(value % Base);
        value /= Base;
    }
    return end;
}

// Writes the digits of `frac / 2^bits`, which is below 1, to `out`. A fraction with `bits` bits has
// at most `bits` decimal digits, so the expansion is exact and ends on its own. Digits come in
// chunks as large as fit in `Int` when multiplied by 10^chunk. Returns the number of digits, which
// may exceed the exact count by trailing zeros of the last chunk.
template <typename Int>
constexpr auto fractionDigits(Int frac, unsigned bits, std::uint8_t* out) noexcept -> int {
    const Int limit{(Int{1} << (sizeof(Int) * 8 - 2)) >> bits};
    int chunk{18};
    while (chunk > 1 && static_cast<Int>(kPow10[static_cast<std::size_t>(chunk)]) > limit) {
        --chunk;
    }
    const Int mask{(Int{1} << bits) - 1};
    int count{0};
    while (frac != 0) {
        const Int scaled{frac * static_cast<Int>(kPow10[static_cast<std::size_t>(chunk)])};
        count += chunk;
        writeUnsigned<10>(out + count, static_cast<std::uint64_t>(scaled >> bits), chunk);
        frac = scaled & mask;
    }
    return count;
}

// The decimal digits of a non-negative number 0.d[0]d[1]...d[n-1] * 10^point, with d[0] != 0 and
// d[n-1] != 0. Zero has n == 0.
struct Digits {
    std::array<std::uint8_t, 128> d;
    int n{0};
    int point{1};

    [[nodiscard]] constexpr auto at(int i) const noexcept -> std::uint8_t {
        return i >= 0 && i < n ? d[static_cast<std::size_t>(i)] : std::uint8_t{0};
    }
};

// The exact decimal digits of `numerator / 2^bits`.
template <typename Int>
constexpr auto toDigits(Int numerator, unsigned bits) noexcept -> Digits {
    Digits r;
    std::array<std::uint8_t, 20> whole;
    const std::uint8_t* const end{whole.data() + whole.size()};
    const std::uint8_t* const first{writeUnsigned<10>(
        whole.data() + whole.size(), static_cast<std::uint64_t>(numerator >> bits), 0
    )};
    r.n = static_cast<int>(end - first);
    r.point = r.n;
    std::copy(first, end, r.d.begin());
    r.n += fractionDigits(numerator & ((Int{1} << bits) - 1), bits, r.d.data() + r.n);

    int lead{0};
    while (lead < r.n && r.d[static_cast<std::size_t>(lead)] == 0) {
        ++lead;
    }
    if (lead == r.n) {
        r.n = 0;
        r.point = 1;
        return r;
    }
    std::copy(r.d.begin() + lead, r.d.begin() + r.n, r.d.begin());
    r.n -= lead;
    r.point -= lead;
    while (r.d[static_cast<std::size_t>(r.n - 1)] == 0) {
        --r.n;
    }
    return r;
}

// Rounds to the `keep` leading digits (to even on a tie), which may be none or negative.
constexpr auto roundDigits(Digits& x, int keep) noexcept -> void {
    if (keep >= x.n) {
        return;
    }
    if (keep < 0) {
        x.n = 0;
        x.point = 1;
        return;
    }
    const auto rest{x.d[static_cast<std::size_t>(keep)]};
    const bool up{
        rest > 5 || (rest == 5 && (x.n > keep + 1 ||
                                   (keep > 0 && x.d[static_cast<std::size_t>(keep - 1)] % 2 == 1)))
    };
    x.n = keep;
    if (up) {
        while (x.n > 0 && x.d[static_cast<std::size_t>(x.n - 1)] == 9) {
            --x.n;
        }
        if (x.n == 0) {
            x.d[0] = 1;
            x.n = 1;
            ++x.point;
        } else {
            ++x.d[static_cast<std::size_t>(x.n - 1)];
        }
    }
    while (x.n > 0 && x.d[static_cast<std::size_t>(x.n - 1)] == 0) {
        --x.n;
    }
    if (x.n == 0) {
        x.point = 1;
    }
}

// The fewest digits after the point at which the nearest decimal number reads back as the same
// Fixed `mag / 2^bits`, i.e. lies strictly less than half a step away. With r the distance of
// mag * 10^p above a multiple of 2^bits, that is 2 * min(r, 2^bits - r) < 10^p. Since 2^bits
// divides 2^64, r comes out of a wrapping 64-bit product. A longer p only gets closer, so the first
// p that fits is the shortest.
[[nodiscard]] constexpr auto shortestPrecision(std::uint64_t mag, unsigned bits) noexcept -> int {
    const std::uint64_t step{std::uint64_t{1} << bits};
    const std::uint64_t frac{mag & (step - 1)};
    int p{0};
    while (p < 19) {
        const std::uint64_t r{(frac * kPow10[static_cast<std::size_t>(p)]) & (step - 1)};
        if (2 * std::min(r, step - r) < kPow10[static_cast<std::size_t>(p)]) {
            break;
        }
        ++p;
    }
    return p;
}

template <typename Char>
struct Numpunct {
    Char point{static_cast<Char>('.')};
    Char sep{static_cast<Char>(',')};
    std::string grouping;
};

template <typename Char>
auto makeNumpunct(const std::locale& loc) -> Numpunct<Char> {
    const auto& facet{std::use_facet<std::numpunct<Char>>(loc)};
    return {facet.decimal_point(), facet.thousands_sep(), facet.grouping()};
}

template <typename Char>
[[nodiscard]] constexpr auto digitChar(std::uint8_t value, bool upper = false) noexcept -> Char {
    return static_cast<Char>(value < 10 ? '0' + value : (upper ? 'A' : 'a') + (value - 10));
}

// The formatted number without sign and padding: `head`, `zeros` zeros, `tail`. Zeros beyond the
// digits of the exact value are counted instead of stored, so a huge precision costs no memory.
template <typename Char>
struct Body {
    std::array<Char, 256> head;
    std::array<Char, 8> tail;
    int head_n{0};
    int tail_n{0};
    std::size_t zeros{0};

    constexpr auto put(Char c) noexcept -> void { head[static_cast<std::size_t>(head_n++)] = c; }
    constexpr auto putDigit(std::uint8_t v, bool upper = false) noexcept -> void {
        put(digitChar<Char>(v, upper));
    }
    [[nodiscard]] constexpr auto size() const noexcept -> std::size_t {
        return static_cast<std::size_t>(head_n) + zeros + static_cast<std::size_t>(tail_n);
    }
};

// The first `count` digits of `x` as the integer part, "0" when there is none.
template <typename Char>
constexpr auto putInteger(Body<Char>& b, const Digits& x, int count, const Numpunct<Char>& np)
    -> void {
    if (count <= 0) {
        b.put(static_cast<Char>('0'));
        return;
    }
    std::array<bool, 24> separated{};
    int pos{0};
    for (std::size_t g{0}; !np.grouping.empty(); ++g) {
        const int size{np.grouping[std::min(g, np.grouping.size() - 1)]};
        pos += size;
        if (size <= 0 || size >= CHAR_MAX || pos >= count) {
            break;
        }
        separated[static_cast<std::size_t>(count - pos)] = true;
    }
    for (int i{0}; i < count; ++i) {
        if (separated[static_cast<std::size_t>(i)]) {
            b.put(np.sep);
        }
        b.putDigit(x.at(i));
    }
}

template <typename Char>
constexpr auto putExponent(Body<Char>& b, char letter, int exponent, int min_digits) noexcept
    -> void {
    std::array<std::uint8_t, 12> digits;
    std::uint8_t* const end{digits.data() + digits.size()};
    const std::uint8_t* const first{writeUnsigned<10>(
        end, static_cast<std::uint64_t>(exponent < 0 ? -exponent : exponent), min_digits
    )};
    b.tail[0] = static_cast<Char>(letter);
    b.tail[1] = static_cast<Char>(exponent < 0 ? '-' : '+');
    b.tail_n = 2;
    for (const std::uint8_t* p{first}; p != end; ++p) {
        b.tail[static_cast<std::size_t>(b.tail_n++)] = digitChar<Char>(*p);
    }
}

// ddd.ddd with `precision` digits after the point.
template <typename Char>
constexpr auto layoutFixed(
    Body<Char>& b, const Digits& x, int precision, bool show_point, const Numpunct<Char>& np
) -> void {
    putInteger(b, x, x.point, np);
    if (precision > 0 || show_point) {
        b.put(np.point);
    }
    const int explicit_digits{std::clamp(x.n - x.point, 0, precision)};
    for (int j{0}; j < explicit_digits; ++j) {
        b.putDigit(x.at(x.point + j));
    }
    b.zeros += static_cast<std::size_t>(precision - explicit_digits);
}

// d.ddde+xx with `precision` digits after the point.
template <typename Char>
constexpr auto layoutScientific(
    Body<Char>& b, const Digits& x, int precision, bool show_point, bool upper,
    const Numpunct<Char>& np
) -> void {
    b.putDigit(x.at(0));
    if (precision > 0 || show_point) {
        b.put(np.point);
    }
    const int explicit_digits{std::clamp(x.n - 1, 0, precision)};
    for (int j{1}; j <= explicit_digits; ++j) {
        b.putDigit(x.at(j));
    }
    b.zeros += static_cast<std::size_t>(precision - explicit_digits);
    putExponent(b, upper ? 'E' : 'e', x.n == 0 ? 0 : x.point - 1, 2);
}

// h.hhhp+d: the leading hexadecimal digit is 1 for a non-zero value. `mag` is the magnitude with
// `bits` fraction bits.
template <typename Char>
constexpr auto layoutHex(
    Body<Char>& b, std::uint64_t mag, unsigned bits, int precision, bool show_point, bool upper,
    const Numpunct<Char>& np
) -> void {
    std::uint64_t lead{0};
    std::uint64_t frac{0};
    int digits{0};
    int exponent{0};
    if (mag != 0) {
        const int top{std::bit_width(mag) - 1};
        lead = 1;
        digits = (top + 3) / 4;
        frac = (mag ^ (std::uint64_t{1} << top)) << (4 * digits - top);
        exponent = top - static_cast<int>(bits);
        if (precision < 0) {
            while (digits > 0 && (frac & 0xF) == 0) {
                frac >>= 4;
                --digits;
            }
        }
    }
    if (precision >= 0 && precision < digits) {
        const int drop{4 * (digits - precision)};
        const std::uint64_t kept{drop >= 64 ? 0 : frac >> drop};
        const std::uint64_t rest{drop >= 64 ? frac : frac & ((std::uint64_t{1} << drop) - 1)};
        const std::uint64_t half{std::uint64_t{1} << (drop - 1)};
        const bool up{rest > half || (rest == half && ((precision > 0 ? kept : lead) & 1) == 1)};
        digits = precision;
        frac = kept + (up ? 1 : 0);
        if (precision == 0) {
            lead += frac;
            frac = 0;
        } else if ((frac >> (4 * precision)) != 0) {
            frac = 0;
            ++lead;
        }
    }
    const int shown{precision < 0 ? digits : precision};
    b.putDigit(static_cast<std::uint8_t>(lead));
    if (shown > 0 || show_point) {
        b.put(np.point);
    }
    std::array<std::uint8_t, 16> hex;
    std::uint8_t* const end{hex.data() + hex.size()};
    for (const std::uint8_t* p{writeUnsigned<16>(end, frac, digits)}; p != end; ++p) {
        b.putDigit(*p, upper);
    }
    b.zeros += static_cast<std::size_t>(shown - digits);
    putExponent(b, upper ? 'P' : 'p', exponent, 1);
}

template <typename Out, typename Char, typename Fn>
auto writePadded(
    Out out, const FormatSpec<Char>& spec, std::basic_string_view<Char> prefix, std::size_t size,
    Fn write_body
) -> Out {
    const std::size_t total{prefix.size() + size};
    const std::size_t pad{spec.width > total ? spec.width - total : 0};
    if (spec.align == Align::INTERNAL) {
        out = std::copy(prefix.begin(), prefix.end(), out);
        out = std::fill_n(out, pad, spec.fill);
        return write_body(out);
    }
    const std::size_t left{
        spec.align == Align::LEFT     ? 0
        : spec.align == Align::CENTER ? pad / 2
                                      : pad
    };
    out = std::fill_n(out, left, spec.fill);
    out = std::copy(prefix.begin(), prefix.end(), out);
    out = write_body(out);
    return std::fill_n(out, pad - left, spec.fill);
}

// Lays out the magnitude `mag` with `bits` fraction bits in decimal notation: fixed, scientific,
// general or, without a type or precision, the shortest form that reads back the same.
template <typename Char, typename Int>
auto layoutDecimal(
    Body<Char>& body, Int mag, unsigned bits, const FormatSpec<Char>& spec, char type, bool upper,
    const Numpunct<Char>& np
) -> void {
    Digits value{toDigits<Int>(mag, bits)};
    const int precision{spec.precision < 0 ? 6 : spec.precision};
    // Rounding past the digits that exist changes nothing, so a huge precision is capped.
    const int keep{std::min(precision, 256)};
    if (type == 'f') {
        roundDigits(value, value.point + keep);
        layoutFixed(body, value, precision, spec.alt, np);
    } else if (type == 'e') {
        roundDigits(value, keep + 1);
        layoutScientific(body, value, precision, spec.alt, upper, np);
    } else if (type == 'g' || spec.precision >= 0) {
        const int significant{std::max(precision, 1)};
        roundDigits(value, std::min(significant, 256));
        const int exponent{value.n == 0 ? 0 : value.point - 1};
        if (exponent >= -4 && exponent < significant) {
            const int frac{
                spec.alt ? significant - 1 - exponent : std::max(0, value.n - value.point)
            };
            layoutFixed(body, value, frac, spec.alt, np);
        } else {
            const int frac{spec.alt ? significant - 1 : std::max(0, value.n - 1)};
            layoutScientific(body, value, frac, spec.alt, upper, np);
        }
    } else {
        // Shortest round-trip: the fewest digits that fix the Fixed, then fixed or scientific
        // notation, whichever is shorter and fixed on a tie (like std::to_chars).
        if (value.n != 0) {
            // At least one digit, so that a value below the first fraction digit is not
            // rounded away in favour of a farther number of the same length.
            roundDigits(
                value,
                std::max(1, value.point + shortestPrecision(static_cast<std::uint64_t>(mag), bits))
            );
        }
        const int frac_fixed{std::max(0, value.n - value.point)};
        const int frac_sci{std::max(0, value.n - 1)};
        const int point_len{spec.alt ? 1 : 0};
        const int len_fixed{
            std::max(value.point, 1) + frac_fixed + (frac_fixed > 0 ? 1 : point_len)
        };
        // "e+xx": the exponent of a Fixed has two digits.
        const int len_sci{1 + frac_sci + (frac_sci > 0 ? 1 : point_len) + 4};
        if (len_sci < len_fixed) {
            layoutScientific(body, value, frac_sci, spec.alt, false, np);
        } else {
            layoutFixed(body, value, frac_fixed, spec.alt, np);
        }
    }
}

// Writes `x` as `spec` asks, to `out`. A null `loc` formats independent of any locale.
template <typename Char, typename Out, FixedPoint T>
auto writeFixed(Out out, T x, FormatSpec<Char> spec, const std::locale* loc) -> Out {
    using intermediate_type = typename T::intermediate_type;
    constexpr unsigned kBits{T::kFractionBits};

    const bool negative{x.raw_value() < 0};
    const intermediate_type mag{
        negative ? -static_cast<intermediate_type>(x.raw_value())
                 : static_cast<intermediate_type>(x.raw_value())
    };
    std::array<Char, 3> prefix;
    std::size_t prefix_n{0};
    if (negative) {
        prefix[prefix_n++] = static_cast<Char>('-');
    } else if (spec.sign == '+' || spec.sign == ' ') {
        prefix[prefix_n++] = static_cast<Char>(spec.sign);
    }

    const Numpunct<Char> np{loc != nullptr ? makeNumpunct<Char>(*loc) : Numpunct<Char>{}};
    const bool upper{spec.type >= 'A' && spec.type <= 'Z'};
    const char type{static_cast<char>(upper ? spec.type + ('a' - 'A') : spec.type)};
    Body<Char> body;

    if (type == 'a') {
        if (spec.hex_prefix) {
            prefix[prefix_n++] = static_cast<Char>('0');
            prefix[prefix_n++] = static_cast<Char>(upper ? 'X' : 'x');
        }
        layoutHex(
            body, static_cast<std::uint64_t>(mag), kBits, spec.precision, spec.alt, upper, np
        );
    } else {
        layoutDecimal(body, mag, kBits, spec, type, upper, np);
    }

    if (spec.zero && spec.align == Align::NONE) {
        spec.align = Align::INTERNAL;
        spec.fill = static_cast<Char>('0');
    }
    return writePadded(
        out, spec, std::basic_string_view<Char>{prefix.data(), prefix_n}, body.size(),
        [&body](Out o) -> Out {
            o = std::copy_n(body.head.data(), body.head_n, o);
            o = std::fill_n(o, body.zeros, static_cast<Char>('0'));
            return std::copy_n(body.tail.data(), body.tail_n, o);
        }
    );
}

} // namespace detail

template <typename CharT>
auto operator<<(std::basic_ostream<CharT>& os, FixedPoint auto x) noexcept
    -> std::basic_ostream<CharT>& {
    const auto flags{os.flags()};
    const bool upper{(flags & std::ios_base::uppercase) != 0};
    const auto adjust{flags & std::ios_base::adjustfield};

    detail::FormatSpec<CharT> spec;
    spec.fill = os.fill();
    spec.width = static_cast<std::size_t>(std::max<std::streamsize>(os.width(), 0));
    spec.align = adjust == std::ios_base::left       ? detail::Align::LEFT
                 : adjust == std::ios_base::internal ? detail::Align::INTERNAL
                                                     : detail::Align::RIGHT;
    spec.sign = (flags & std::ios_base::showpos) != 0 ? '+' : '-';
    spec.alt = (flags & std::ios_base::showpoint) != 0;
    switch (flags & std::ios_base::floatfield) {
    case std::ios_base::fixed:
        spec.type = upper ? 'F' : 'f';
        break;
    case std::ios_base::scientific:
        spec.type = upper ? 'E' : 'e';
        break;
    case std::ios_base::fixed | std::ios_base::scientific:
        spec.type = upper ? 'A' : 'a';
        spec.hex_prefix = true;
        break;
    default:
        spec.type = upper ? 'G' : 'g';
        break;
    }
    // A hexadecimal float ignores the precision, which defaults to 6 if invalid.
    if (spec.type != 'a' && spec.type != 'A') {
        spec.precision = os.precision() < 0 ? 6 : static_cast<int>(os.precision());
    }

    // The classic locale has neither grouping nor another decimal point; skip its facets.
    const std::locale loc{os.getloc()};
    detail::writeFixed(
        std::ostreambuf_iterator<CharT>{os}, x, spec, loc == std::locale::classic() ? nullptr : &loc
    );
    os.width(0);
    return os;
}

template <typename CharT, class Traits>
auto operator>>(std::basic_istream<CharT, Traits>& is, FixedPoint auto& x)
    -> std::basic_istream<CharT, Traits>& {
    using FixedType = std::remove_cvref_t<decltype(x)>;
    typename std::basic_istream<CharT, Traits>::sentry sentry(is);
    if (!sentry) {
        return is;
    }

    const auto& ctype = std::use_facet<std::ctype<CharT>>(is.getloc());
    const auto& numpunct = std::use_facet<std::numpunct<CharT>>(is.getloc());

    bool thousands_separator_allowed = false;
    const bool supports_thousands_separators = !numpunct.grouping().empty();

    const auto is_valid_character = [](char ch) {
        return std::isxdigit(ch) || ch == 'x' || ch == 'X' || ch == 'p' || ch == 'P' || ch == 'i' ||
               ch == 'I' || ch == 'n' || ch == 'N' || ch == 't' || ch == 'T' || ch == 'y' ||
               ch == 'Y' || ch == '-' || ch == '+';
    };

    const auto peek = [&]() {
        for (;;) {
            auto ch = is.rdbuf()->sgetc();
            if (ch == Traits::eof()) {
                is.setstate(std::ios::eofbit);
                return '\0';
            }
            if (ch == numpunct.decimal_point()) {
                return '.';
            }
            if (ch == numpunct.thousands_sep()) {
                if (!supports_thousands_separators || !thousands_separator_allowed) {
                    return '\0';
                }
                is.rdbuf()->sbumpc();
                continue;
            }
            auto res = ctype.narrow(ch, 0);
            if (!is_valid_character(res)) {
                return '\0';
            }
            return res;
        }
    };

    const auto bump = [&]() { is.rdbuf()->sbumpc(); };
    const auto next = [&]() {
        bump();
        return peek();
    };

    bool negate = false;
    auto ch = peek();
    if (ch == '-') {
        negate = true;
        ch = next();
    } else if (ch == '+') {
        ch = next();
    }

    constexpr std::string_view kInfinity = "infinity";
    int i = 0;
    while (i < 8 && ch == kInfinity[static_cast<std::size_t>(i)]) {
        ++i;
        ch = next();
    }
    if (i > 0) {
        if (i == 3 || i == 8) {
            x = negate ? std::numeric_limits<FixedType>::min()
                       : std::numeric_limits<FixedType>::max();
        } else {
            is.setstate(std::ios::failbit);
        }
        return is;
    }

    char exponent_char = 'e';
    int base = 10;

    constexpr auto kNoFraction = std::numeric_limits<std::size_t>::max();
    std::size_t fraction_start = kNoFraction;
    std::vector<unsigned char> significand;

    if (ch == '0') {
        ch = next();
        if (ch == 'x' || ch == 'X') {
            exponent_char = 'p';
            base = 16;
            ch = next();
        } else {
            significand.push_back(0);
        }
    }

    thousands_separator_allowed = true;
    for (;; ch = next()) {
        if (ch == '.') {
            if (fraction_start != kNoFraction) {
                break;
            }
            fraction_start = significand.size();
            thousands_separator_allowed = false;
        } else {
            auto val = static_cast<unsigned char>(base);
            if (ch >= '0' && ch <= '9') {
                val = static_cast<unsigned char>(ch - '0');
            } else if (ch >= 'a' && ch <= 'f') {
                val = static_cast<unsigned char>(ch - 'a' + 10);
            } else if (ch >= 'A' && ch <= 'F') {
                val = static_cast<unsigned char>(ch - 'A' + 10);
            }
            if (val >= static_cast<unsigned char>(base)) {
                break;
            }
            significand.push_back(val);
        }
    }
    if (significand.empty()) {
        is.setstate(std::ios::failbit);
        return is;
    }
    thousands_separator_allowed = false;

    if (fraction_start == kNoFraction) {
        fraction_start = significand.size();
    }

    bool exponent_overflow = false;
    std::size_t exponent = 0;
    bool exponent_negate = false;
    if (std::tolower(ch) == exponent_char) {
        ch = next();
        if (ch == '-') {
            exponent_negate = true;
            ch = next();
        } else if (ch == '+') {
            ch = next();
        }
        bool parsed = false;
        while (std::isdigit(ch)) {
            if (exponent <= static_cast<std::size_t>(std::numeric_limits<int>::max()) / 10) {
                exponent = exponent * 10 + static_cast<std::size_t>(ch - '0');
            } else {
                exponent_overflow = true;
            }
            parsed = true;
            ch = next();
        }
        if (!parsed) {
            is.setstate(std::ios::failbit);
            return is;
        }
    }

    if (exponent_overflow) {
        if (std::all_of(significand.begin(), significand.end(), [](unsigned char v) {
                return v == 0;
            })) {
            x = FixedType(0);
        } else if (exponent_negate) {
            x = FixedType::fromRawValue(0);
        } else {
            x = std::numeric_limits<FixedType>::max();
        }
        return is;
    }

    {
        const auto exponent_mult = (base == 10) ? 1 : 4;
        if (exponent_negate) {
            const auto adjust = std::min(exponent / exponent_mult, fraction_start);
            fraction_start -= adjust;
            exponent -= adjust * exponent_mult;
        } else {
            const auto adjust =
                std::min(exponent / exponent_mult, significand.size() - fraction_start);
            fraction_start += adjust;
            exponent -= adjust * exponent_mult;
        }
    }

    constexpr auto kIsSigned = std::is_signed_v<typename FixedType::value_type>;
    constexpr auto kIntBits = sizeof(typename FixedType::value_type) * 8 -
                              FixedType::kFractionBits - (kIsSigned ? 1u : 0u);
    constexpr auto kMaxInt = (typename FixedType::intermediate_type{1} << kIntBits) - 1;
    constexpr auto kMaxFraction =
        (typename FixedType::intermediate_type{1} << FixedType::kFractionBits) - 1;
    constexpr auto kMaxValue =
        (typename FixedType::intermediate_type{1} << sizeof(typename FixedType::value_type) * 8) -
        1;

    typename FixedType::intermediate_type integer = 0;
    for (std::size_t j = 0; j < fraction_start; ++j) {
        if (integer > kMaxInt / base) {
            x = negate ? std::numeric_limits<FixedType>::min()
                       : std::numeric_limits<FixedType>::max();
            return is;
        }
        assert(significand[j] < static_cast<unsigned char>(base));
        integer = integer * base + significand[j];
    }

    typename FixedType::intermediate_type fraction = 0;
    typename FixedType::intermediate_type divisor = 1;
    for (std::size_t j = fraction_start; j < significand.size(); ++j) {
        assert(significand[j] < static_cast<unsigned char>(base));
        if (divisor > kMaxFraction / base) {
            break;
        }
        fraction = fraction * base + significand[j];
        divisor *= base;
    }

    typename FixedType::intermediate_type raw_value =
        (integer << FixedType::kFractionBits) + (fraction << FixedType::kFractionBits) / divisor;

    if (exponent_char == 'p') {
        if (exponent_negate) {
            raw_value >>= exponent;
        } else {
            raw_value <<= exponent;
        }
    } else {
        if (exponent_negate) {
            typename FixedType::intermediate_type rem = 0;
            for (std::size_t e = 0; e < exponent; ++e) {
                rem = raw_value % 10;
                raw_value /= 10;
            }
            raw_value += rem / 5;
        } else {
            for (std::size_t e = 0; e < exponent; ++e) {
                if (raw_value > kMaxValue / 10) {
                    x = negate ? std::numeric_limits<FixedType>::min()
                               : std::numeric_limits<FixedType>::max();
                    return is;
                }
                raw_value *= 10;
            }
        }
    }
    x = FixedType::fromRawValue(
        static_cast<typename FixedType::value_type>(negate ? -raw_value : raw_value)
    );
    return is;
}

} // namespace basjoo::math

// NOLINTBEGIN(cert-dcl58-cpp)
namespace std {

template <basjoo::math::FixedPoint T, typename CharT>
struct formatter<T, CharT> {
    constexpr auto parse(std::basic_format_parse_context<CharT>& ctx)
        -> std::basic_format_parse_context<CharT>::iterator {
        return basjoo::math::detail::parseSpec(ctx, m_spec);
    }

    template <typename FmtCtx>
    auto format(const T& value, FmtCtx& ctx) const -> typename FmtCtx::iterator {
        auto spec{m_spec};
        if (spec.width_arg >= 0) {
            spec.width = static_cast<std::size_t>(
                basjoo::math::detail::dynamicSize(ctx.arg(static_cast<std::size_t>(spec.width_arg)))
            );
        }
        if (spec.precision_arg >= 0) {
            spec.precision = basjoo::math::detail::dynamicSize(
                ctx.arg(static_cast<std::size_t>(spec.precision_arg))
            );
        }
        if (spec.localized) {
            const std::locale loc{ctx.locale()};
            return basjoo::math::detail::writeFixed(ctx.out(), value, spec, &loc);
        }
        return basjoo::math::detail::writeFixed(ctx.out(), value, spec, nullptr);
    }

  private:
    basjoo::math::detail::FormatSpec<CharT> m_spec;
};

} // namespace std
// NOLINTEND(cert-dcl58-cpp)
