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
 */

#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cctype>
#include <climits>
#include <cmath>
#include <compare>
#include <concepts>
#include <cstdint>
#include <format>
#include <functional>
#include <ios>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#if defined(_MSC_VER)
#include <boost/multiprecision/cpp_int.hpp>
#include <intrin.h>
#endif

#include "zpp_bits.h"

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

    Fixed() noexcept = default;

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
        const intermediate_type int_part = integer_value * (T(1) << kFractionBits);
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
        const intermediate_type int_part = integer_value * (T(1) << FractionBits);
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

    [[nodiscard]] constexpr auto operator-() const& noexcept -> Fixed {
        return Fixed::fromRawValue(-m_raw_value);
    }

    [[nodiscard]] constexpr auto operator-() && noexcept -> Fixed&& {
        m_raw_value = -m_raw_value;
        return std::move(*this);
    }

    auto operator+=(Fixed obj) noexcept -> Fixed& {
        m_raw_value += obj.m_raw_value;
        return *this;
    }
    auto operator+(const Fixed& obj) const& noexcept -> Fixed {
        Fixed result{*this};
        result += obj;
        return result;
    }
    auto operator+(Fixed&& obj) const& noexcept -> Fixed&& {
        obj += *this;
        return std::move(obj);
    }
    auto operator+(const Fixed& obj) && noexcept -> Fixed&& {
        operator+=(obj);
        return std::move(*this);
    }
    auto operator+(Fixed&& obj) && noexcept -> Fixed&& {
        operator+=(obj);
        return std::move(*this);
    }

    auto operator+=(std::integral auto obj) noexcept -> Fixed& {
        m_raw_value += static_cast<value_type>(obj * kFractionMult);
        return *this;
    }
    auto operator+(std::integral auto obj) const& noexcept -> Fixed {
        Fixed result{*this};
        result += obj;
        return result;
    }
    auto operator+(std::integral auto obj) && noexcept -> Fixed&& {
        operator+=(obj);
        return std::move(*this);
    }

    auto operator-=(Fixed obj) noexcept -> Fixed& {
        m_raw_value -= obj.m_raw_value;
        return *this;
    }
    auto operator-(const Fixed& obj) const& noexcept -> Fixed {
        Fixed result{*this};
        result -= obj;
        return result;
    }
    auto operator-(Fixed&& obj) const& noexcept -> Fixed&& {
        obj.m_raw_value = m_raw_value - obj.m_raw_value;
        return std::move(obj);
    }
    auto operator-(const Fixed& obj) && noexcept -> Fixed&& {
        operator-=(obj);
        return std::move(*this);
    }
    auto operator-(Fixed&& obj) && noexcept -> Fixed&& {
        operator-=(obj);
        return std::move(*this);
    }

    auto operator-=(std::integral auto obj) noexcept -> Fixed& {
        m_raw_value -= static_cast<value_type>(obj * kFractionMult);
        return *this;
    }
    auto operator-(std::integral auto obj) const& noexcept -> Fixed {
        Fixed result{*this};
        result -= obj;
        return result;
    }
    auto operator-(std::integral auto obj) && noexcept -> Fixed&& {
        operator-=(obj);
        return std::move(*this);
    }

    auto operator*=(const Fixed& obj) noexcept -> Fixed& {
        if constexpr (kEnableRounding) {
            // Multiply by two before dividing, add the LSB to round correctly.
            auto value = (static_cast<intermediate_type>(m_raw_value) * obj.m_raw_value) /
                         (kFractionMult / 2);
            m_raw_value = static_cast<value_type>((value / 2) + (value % 2));
        } else {
            m_raw_value = static_cast<value_type>(
                (static_cast<intermediate_type>(m_raw_value) * obj.m_raw_value) / kFractionMult
            );
        }
        return *this;
    }
    auto operator*(const Fixed& obj) const& noexcept -> Fixed {
        Fixed result{*this};
        result *= obj;
        return result;
    }
    auto operator*(Fixed&& obj) const& noexcept -> Fixed&& {
        obj *= *this;
        return std::move(obj);
    }
    auto operator*(const Fixed& obj) && noexcept -> Fixed&& {
        operator*=(obj);
        return std::move(*this);
    }
    auto operator*(Fixed&& obj) && noexcept -> Fixed&& {
        operator*=(obj);
        return std::move(*this);
    }

    auto operator*=(std::integral auto obj) noexcept -> Fixed& {
        m_raw_value *= obj;
        return *this;
    }
    auto operator*(std::integral auto obj) const& noexcept -> Fixed {
        Fixed result{*this};
        result *= obj;
        return result;
    }
    auto operator*(std::integral auto obj) && noexcept -> Fixed&& {
        operator*=(obj);
        return std::move(*this);
    }

    auto operator/=(const Fixed& obj) noexcept -> Fixed& {
        assert(obj.m_raw_value != 0);
        if constexpr (kEnableRounding) {
            // Multiply by two before dividing, add the LSB to round correctly.
            auto value =
                (static_cast<value_type>(m_raw_value) * kFractionMult * 2) / obj.m_raw_value;
            m_raw_value = static_cast<value_type>((value / 2) + (value % 2));
        } else {
            m_raw_value = static_cast<value_type>(
                (static_cast<intermediate_type>(m_raw_value) * kFractionMult) / obj.m_raw_value
            );
        }
        return *this;
    }
    auto operator/(const Fixed& obj) const& noexcept -> Fixed {
        Fixed result{*this};
        result /= obj;
        return result;
    }
    auto operator/(Fixed&& obj) const& noexcept -> Fixed&& {
        assert(obj.m_raw_value != 0);
        if constexpr (kEnableRounding) {
            auto value =
                (static_cast<intermediate_type>(m_raw_value) * kFractionMult * 2) / obj.m_raw_value;
            obj.m_raw_value = static_cast<value_type>((value / 2) + (value % 2));
        } else {
            obj.m_raw_value = static_cast<value_type>(
                (static_cast<intermediate_type>(m_raw_value) * kFractionMult) / obj.m_raw_value
            );
        }
        return std::move(obj);
    }
    auto operator/(const Fixed& obj) && noexcept -> Fixed&& {
        operator/=(obj);
        return std::move(*this);
    }
    auto operator/(Fixed&& obj) && noexcept -> Fixed&& {
        operator/=(obj);
        return std::move(*this);
    }

    auto operator/=(std::integral auto obj) noexcept -> Fixed& {
        assert(obj != 0);
        m_raw_value /= obj;
        return *this;
    }
    auto operator/(std::integral auto obj) const& noexcept -> Fixed {
        Fixed result{*this};
        result /= obj;
        return result;
    }
    auto operator/(std::integral auto obj) && noexcept -> Fixed&& {
        operator/=(obj);
        return std::move(*this);
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

    value_type m_raw_value;
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

template <FixedPoint T>
[[using gnu: pure, always_inline, hot]]
inline constexpr auto operator+(const std::integral auto& fac, const T& obj) noexcept -> T {
    return obj + fac;
}

template <FixedPoint T>
[[using gnu: always_inline, hot]]
inline constexpr auto operator+(const std::integral auto& fac, T&& obj) noexcept -> T&& {
    obj += fac;
    return std::move(obj);
}

template <FixedPoint T>
[[using gnu: pure, always_inline, hot]]
inline constexpr auto operator-(const std::integral auto& fac, const T& obj) noexcept -> T {
    return T(fac) - obj;
}

template <FixedPoint T>
[[using gnu: always_inline, hot]]
inline constexpr auto operator-(const std::integral auto& fac, T&& obj) noexcept -> T&& {
    obj *= -1;
    obj += fac;
    return std::move(obj);
}

template <FixedPoint T>
[[using gnu: pure, always_inline, hot]]
inline constexpr auto operator*(const std::integral auto& fac, const T& obj) noexcept -> T {
    return obj * fac;
}

template <FixedPoint T>
[[using gnu: always_inline, hot]]
inline constexpr auto operator*(const std::integral auto& fac, T&& obj) noexcept -> T&& {
    obj *= fac;
    return std::move(obj);
}

template <FixedPoint T>
[[using gnu: pure, always_inline, hot]]
inline constexpr auto operator/(const std::integral auto& fac, const T& obj) noexcept -> T {
    return T(fac) / obj;
}

template <FixedPoint T>
[[using gnu: always_inline, hot]]
inline constexpr auto operator/(const std::integral auto& fac, T&& obj) noexcept -> T&& {
    obj = T(1) / obj;
    obj *= fac;
    return std::move(obj);
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

[[nodiscard]] auto ceil(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    constexpr auto kFrac = typename Self::value_type(1) << Self::kFractionBits;
    auto value = x.raw_value();
    if (value > 0) {
        value += kFrac - 1;
    }
    return Self::fromRawValue(value / kFrac * kFrac);
}

[[nodiscard]] auto floor(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    constexpr auto kFrac = typename Self::value_type(1) << Self::kFractionBits;
    auto value = x.raw_value();
    if (value < 0) {
        value -= kFrac - 1;
    }
    return Self::fromRawValue(value / kFrac * kFrac);
}

[[nodiscard]] auto trunc(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    constexpr auto kFrac = typename Self::value_type(1) << Self::kFractionBits;
    return Self::fromRawValue(x.raw_value() / kFrac * kFrac);
}

[[nodiscard]] auto round(FixedPoint auto x) noexcept -> decltype(x) {
    using Self = std::remove_cvref_t<decltype(x)>;
    constexpr auto kFrac = typename Self::value_type(1) << Self::kFractionBits;
    auto value = x.raw_value() / (kFrac / 2);
    return Self::fromRawValue(((value / 2) + (value % 2)) * kFrac);
}

[[nodiscard]] auto nearbyint(FixedPoint auto x) noexcept -> decltype(x) {
    // Rounding mode assumed to be FE_TONEAREST (round half to even)
    using Self = std::remove_cvref_t<decltype(x)>;
    constexpr auto kFrac = typename Self::value_type(1) << Self::kFractionBits;
    auto value = x.raw_value();
    const bool is_half = std::abs(value % kFrac) == kFrac / 2;
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

template <typename CharT>
auto operator<<(std::basic_ostream<CharT>& os, FixedPoint auto x) noexcept
    -> std::basic_ostream<CharT>& {
    using FixedType = std::remove_cvref_t<decltype(x)>;
    const auto uppercase = ((os.flags() & std::ios_base::uppercase) != 0);
    const auto showpoint = ((os.flags() & std::ios_base::showpoint) != 0);
    const auto adjustfield = (os.flags() & std::ios_base::adjustfield);
    const auto width = os.width();
    const auto& ctype = std::use_facet<std::ctype<CharT>>(os.getloc());
    const auto& numpunct = std::use_facet<std::numpunct<CharT>>(os.getloc());

    auto floatfield = (os.flags() & std::ios_base::floatfield);
    auto precision = os.precision();
    auto show_trailing_zeros = true;
    auto use_significant_digits = false;

    if (precision < 0) {
        precision = 6;
    }

    constexpr auto kWorstCaseConstantSize = 6;
    constexpr auto kWorstCaseDigitCount =
        std::numeric_limits<typename FixedType::value_type>::digits10 + 2;
    constexpr auto kWorstCaseSuffixSize =
        std::numeric_limits<typename FixedType::value_type>::digits;
    using buffer_t =
        std::array<CharT, kWorstCaseConstantSize + kWorstCaseDigitCount * 2 + kWorstCaseSuffixSize>;
    buffer_t buffer;

    auto end = buffer.begin();
    auto internal_pad = buffer.end();

    struct number_t {
        typename FixedType::intermediate_type raw;
        typename FixedType::intermediate_type divisor;
        int exponent;
    };

    const auto as_scientific = [](number_t value) {
        assert(value.exponent == 0);
        if (value.raw > 0) {
            while (value.raw / 10 >= value.divisor) {
                value.divisor *= 10;
                ++value.exponent;
            }
            while (value.raw < value.divisor) {
                value.raw *= 10;
                --value.exponent;
            }
        }
        return value;
    };

    number_t value = {
        x.raw_value(), typename FixedType::intermediate_type{1} << FixedType::kFractionBits, 0
    };
    auto base = typename FixedType::value_type{10};

    if (value.raw < 0) {
        *end++ = ctype.widen('-');
        value.raw = -value.raw;
        internal_pad = end;
    } else if (os.flags() & std::ios_base::showpos) {
        *end++ = ctype.widen('+');
        internal_pad = end;
    }
    assert(value.raw >= 0);

    switch (floatfield) {
    case std::ios_base::fixed | std::ios_base::scientific:
        if (value.raw > 0) {
            auto bit = detail::findHighestBit(static_cast<unsigned long long>(value.raw));
            value.exponent = static_cast<int>(bit) - static_cast<int>(FixedType::kFractionBits);
            value.divisor = typename FixedType::intermediate_type{1} << static_cast<int>(bit);
            precision = (static_cast<int>(bit) + 3) / 4;
        }
        base = 16;
        show_trailing_zeros = false;
        *end++ = ctype.widen('0');
        *end++ = ctype.widen(uppercase ? 'X' : 'x');
        break;

    case std::ios_base::scientific:
        value = as_scientific(value);
        break;

    case std::ios_base::fixed:
        break;

    default: {
        const number_t sci_value = as_scientific(value);
        use_significant_digits = true;
        precision = std::max<std::streamsize>(precision, 1);
        if (sci_value.exponent >= precision || sci_value.exponent < -4) {
            floatfield = std::ios_base::scientific;
            value = sci_value;
        } else {
            floatfield = std::ios_base::fixed;
            show_trailing_zeros = showpoint;
        }
        break;
    }
    }

    if (internal_pad == buffer.end()) {
        internal_pad = end;
    }

    typename FixedType::intermediate_type integral = value.raw / value.divisor;
    value.raw %= value.divisor;

    const char* const digits = uppercase ? "0123456789ABCDEF" : "0123456789abcdef";
    const auto digits_start = end;

    bool significant_digits = !use_significant_digits;
    int last_digit = 0;

    if (integral == 0) {
        *end++ = ctype.widen('0');
        if (value.raw == 0) {
            significant_digits = true;
        }
    } else {
        while (integral > 0) {
            last_digit = static_cast<int>(integral % base);
            *end++ = ctype.widen(digits[last_digit]);
            integral /= base;
        }
        std::reverse(digits_start, end);
        significant_digits = true;
    }

    if (use_significant_digits && significant_digits) {
        precision -= (end - digits_start);
    }

    assert(value.raw < value.divisor);
    assert(precision >= 0);

    auto point = buffer.end();
    auto trailing_zeros_start = buffer.end();
    std::streamsize trailing_zeros_count = 0;

    if (precision > 0) {
        *(point = end++) = numpunct.decimal_point();

        for (int i = 0; i < precision; ++i) {
            if (value.raw == 0) {
                trailing_zeros_start = end;
                trailing_zeros_count = precision - i;
                break;
            }
            if (value.divisor % base == 0) {
                value.divisor /= base;
            } else {
                value.raw *= base;
            }
            assert(value.divisor > 0 && value.raw >= 0);
            last_digit = static_cast<int>((value.raw / value.divisor) % base);
            value.raw %= value.divisor;
            *end++ = ctype.widen(digits[last_digit]);

            if (!significant_digits) {
                if (last_digit != 0) {
                    significant_digits = true;
                } else {
                    ++precision;
                }
            }
        }
    } else if (showpoint) {
        *(point = end++) = numpunct.decimal_point();
    }

    const auto insert_character = [&](typename buffer_t::iterator position, CharT ch) {
        assert(position >= buffer.begin() && position < end);
        std::move_backward(position, end, end + 1);
        if (point != buffer.end() && position < point) {
            ++point;
        }
        if (trailing_zeros_start != buffer.end() && position < trailing_zeros_start) {
            ++trailing_zeros_start;
        }
        ++end;
        *position = ch;
    };

    bool increment = false;
    if (value.raw > value.divisor / 2) {
        increment = true;
    } else if (value.raw == value.divisor / 2) {
        increment = ((last_digit % 2) == 1);
    }

    if (increment) {
        auto p = end - 1;
        while (p >= digits_start) {
            if (p == point) {
                --p;
            }
            if ((*p)++ != ctype.widen('9')) {
                break;
            }
            *p-- = ctype.widen('0');
        }
        if (p < digits_start) {
            assert(p == digits_start - 1);
            insert_character(++p, ctype.widen('1'));
            if (floatfield == std::ios::scientific) {
                if (point != buffer.end()) {
                    assert(p + 2 == point);
                    std::swap(*(point - 1), *point);
                    --point;
                }
                ++value.exponent;
                --end;
            }
        }
        if (use_significant_digits && *p == ctype.widen('1') && point != buffer.end()) {
            --end;
        }
    }

    if (point != buffer.end()) {
        if (!show_trailing_zeros) {
            while (*(end - 1) == ctype.widen('0')) {
                --end;
            }
            trailing_zeros_start = buffer.end();
            trailing_zeros_count = 0;
        }
        if (end - 1 == point && trailing_zeros_count == 0 && !showpoint) {
            --end;
        }
    }

    const auto& grouping = numpunct.grouping();
    if (!grouping.empty()) {
        const CharT thousands_sep = ctype.widen(numpunct.thousands_sep());
        std::size_t group = 0;
        auto p = point != buffer.end() ? point : end;
        auto size = static_cast<int>(grouping[group]);
        while (size > 0 && size < CHAR_MAX && p - digits_start > size) {
            p -= size;
            insert_character(p, thousands_sep);
            if (group < grouping.size() - 1) {
                size = static_cast<int>(grouping[++group]);
            }
        }
    }

    assert(floatfield != 0);
    if (floatfield & std::ios_base::scientific) {
        if (floatfield & std::ios_base::fixed) {
            *end++ = ctype.widen(uppercase ? 'P' : 'p');
        } else {
            *end++ = ctype.widen(uppercase ? 'E' : 'e');
        }
        if (value.exponent < 0) {
            *end++ = ctype.widen('-');
            value.exponent = -value.exponent;
        } else {
            *end++ = ctype.widen('+');
        }
        if (floatfield == std::ios_base::scientific) {
            if (value.exponent < 10) {
                *end++ = ctype.widen('0');
            }
        }
        const auto exponent_start = end;
        if (value.exponent == 0) {
            *end++ = ctype.widen('0');
        } else {
            while (value.exponent > 0) {
                *end++ = ctype.widen(digits[value.exponent % 10]);
                value.exponent /= 10;
            }
        }
        std::reverse(exponent_start, end);
    }

    const auto sputcn = [&](CharT ch, std::streamsize count) {
        constexpr std::streamsize kChunkSize = 64;
        std::array<CharT, kChunkSize> fill_buffer;
        std::fill_n(fill_buffer.begin(), std::min(count, kChunkSize), ch);
        for (std::streamsize left = count; left > 0;) {
            const std::streamsize size = std::min(kChunkSize, left);
            os.rdbuf()->sputn(fill_buffer.data(), size);
            left -= size;
        }
    };

    const auto put_range = [&](typename buffer_t::const_iterator begin,
                               typename buffer_t::const_iterator rend) {
        assert(rend >= begin);
        if (trailing_zeros_start >= begin && trailing_zeros_start <= rend) {
            assert(trailing_zeros_count > 0);
            os.rdbuf()->sputn(&*begin, trailing_zeros_start - begin);
            sputcn(ctype.widen('0'), trailing_zeros_count);
            os.rdbuf()->sputn(&*trailing_zeros_start, rend - trailing_zeros_start);
        } else {
            os.rdbuf()->sputn(&*begin, rend - begin);
        }
    };

    const auto content_size = end - buffer.begin() + trailing_zeros_count;
    if (content_size >= width) {
        put_range(buffer.begin(), end);
    } else {
        const auto pad_size = width - content_size;
        switch (adjustfield) {
        case std::ios_base::left:
            put_range(buffer.begin(), end);
            sputcn(os.fill(), pad_size);
            break;
        case std::ios_base::internal:
            put_range(buffer.begin(), internal_pad);
            sputcn(os.fill(), pad_size);
            put_range(internal_pad, end);
            break;
        default:
            sputcn(os.fill(), pad_size);
            put_range(buffer.begin(), end);
            break;
        }
    }

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
    CharT fill_{CharT(' ')};
    char align_{'\0'};
    char sign_{'-'};
    bool alt_{false};
    bool zero_{false};
    int width_{0};
    int prec_{-1};
    char type_{'g'};

    template <typename ParseCtx>
    constexpr auto parse(ParseCtx& ctx) -> typename ParseCtx::iterator {
        auto it = ctx.begin();
        const auto end = ctx.end();

        auto is_align = [](CharT c) noexcept -> bool {
            return c == CharT('<') || c == CharT('>') || c == CharT('^');
        };

        // fill + align
        if (it != end && (it + 1) != end && is_align(*(it + 1))) {
            fill_ = *it++;
            align_ = static_cast<char>(*it++);
        } else if (it != end && is_align(*it)) {
            align_ = static_cast<char>(*it++);
        }

        // sign
        if (it != end) {
            const auto c = static_cast<char>(*it);
            if (c == '+' || c == '-' || c == ' ') {
                sign_ = c;
                ++it;
            }
        }

        // alt (#)
        if (it != end && *it == CharT('#')) {
            alt_ = true;
            ++it;
        }

        // zero fill
        if (it != end && *it == CharT('0')) {
            zero_ = true;
            ++it;
        }

        // width
        while (it != end && *it >= CharT('0') && *it <= CharT('9')) {
            width_ = width_ * 10 + static_cast<int>(*it - CharT('0'));
            ++it;
        }

        // precision
        if (it != end && *it == CharT('.')) {
            ++it;
            prec_ = 0;
            while (it != end && *it >= CharT('0') && *it <= CharT('9')) {
                prec_ = prec_ * 10 + static_cast<int>(*it - CharT('0'));
                ++it;
            }
        }

        // type
        if (it != end && *it != CharT('}')) {
            type_ = static_cast<char>(*it++);
        }

        return it;
    }

    template <typename FmtCtx>
    auto format(T val, FmtCtx& ctx) const -> typename FmtCtx::iterator {
        basic_ostringstream<CharT> oss;

        const char ltype = static_cast<char>(std::tolower(static_cast<unsigned char>(type_)));
        const bool upper = (type_ != ltype);

        switch (ltype) {
        case 'f':
            oss << std::fixed;
            break;
        case 'e':
            oss << std::scientific;
            break;
        case 'a':
            oss << std::hexfloat;
            break;
        default:
            break;
        }
        if (upper) {
            oss << std::uppercase;
        }
        if (sign_ == '+') {
            oss << std::showpos;
        }
        if (alt_) {
            oss << std::showpoint;
        }
        if (prec_ >= 0) {
            oss.precision(prec_);
        }

        oss << val;
        basic_string<CharT> str = oss.str();

        // handle space-sign: prepend space before non-negative values
        if (sign_ == ' ' && !str.empty() && str[0] != CharT('-') && str[0] != CharT('+')) {
            str.insert(str.begin(), CharT(' '));
        }

        const int len = static_cast<int>(str.size());
        const int pad = width_ > len ? width_ - len : 0;
        auto out = ctx.out();

        if (pad == 0) {
            return ranges::copy(str, out).out;
        }

        // zero-fill: sign/prefix first, then zeros, then digits
        if (zero_ && align_ == '\0') {
            if (!str.empty() &&
                (str[0] == CharT('-') || str[0] == CharT('+') || str[0] == CharT(' '))) {
                *out++ = str[0];
                for (int p = 0; p < pad; ++p) {
                    *out++ = CharT('0');
                }
                return ranges::copy(str.begin() + 1, str.end(), out).out;
            }
            for (int p = 0; p < pad; ++p) {
                *out++ = CharT('0');
            }
            return ranges::copy(str, out).out;
        }

        const char eff_align = (align_ == '\0') ? '>' : align_;
        if (eff_align == '<') {
            out = ranges::copy(str, out).out;
            for (int p = 0; p < pad; ++p) {
                *out++ = fill_;
            }
        } else if (eff_align == '^') {
            const int left_pad = pad / 2;
            const int right_pad = pad - left_pad;
            for (int p = 0; p < left_pad; ++p) {
                *out++ = fill_;
            }
            out = ranges::copy(str, out).out;
            for (int p = 0; p < right_pad; ++p) {
                *out++ = fill_;
            }
        } else {
            for (int p = 0; p < pad; ++p) {
                *out++ = fill_;
            }
            out = ranges::copy(str, out).out;
        }
        return out;
    }
};

} // namespace std
// NOLINTEND(cert-dcl58-cpp)
