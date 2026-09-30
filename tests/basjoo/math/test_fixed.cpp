/**
 * @file test_fixed.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief Unit tests for Fixed, adapted from https://github.com/MikeLankamp/fpm/tree/master/tests
 * @version 0.1
 * @date 2026-05-27
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/fixed.hpp"

#include <cmath>
#include <compare>
#include <format>
#include <limits>
#include <sstream>
#include <string>

#include "zpp_bits.h"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::math {

// Returns true when |got - ref| <= max(|ref|, abs_floor) * max_error_perc/100.
// The abs_floor prevents unbounded relative error when ref is near zero
// (e.g. cos(π/2) ≈ 6e-17 in double but 0 in fixed-point).
template <typename T>
[[nodiscard]] static auto hasMaxRelativeError(
    T got, T ref, double max_error_perc, double abs_floor = 1e-4
) -> bool {
    const double denom = std::max(std::abs(ref), abs_floor);
    return std::abs(got - ref) / denom <= max_error_perc / 100.0;
}

// ─── Construction / Conversion ───────────────────────────────────────────────

TEST_CASE_TEMPLATE(
    "Construction from integers and floats", T, Fixed32f8, Fixed32f16, Fixed32f24, Fixed64f8,
    Fixed64f16, Fixed64f24, Fixed64f32, Fixed64f40, Fixed64f48, Fixed64f56
) {
    CHECK_EQ(static_cast<double>(T{0}), doctest::Approx(0.0));
    CHECK_EQ(static_cast<double>(T{1}), doctest::Approx(1.0));
    CHECK_EQ(static_cast<double>(T{-3}), doctest::Approx(-3.0));
    CHECK_EQ(static_cast<double>(T{1.5}), doctest::Approx(1.5).epsilon(1e-3));
    CHECK_EQ(static_cast<double>(T{-2.75}), doctest::Approx(-2.75).epsilon(1e-3));
    CHECK_EQ(static_cast<int>(T{7}), 7);
    CHECK_EQ(static_cast<int>(T{-4}), -4);
}

TEST_CASE_TEMPLATE(
    "Float rounding on construction (EnableRounding=true)", T,
    Fixed<std::int32_t, std::int64_t, 2, true>
) {
    // 1.125 rounds up to 1.25 with 2 fraction bits
    CHECK_EQ(static_cast<double>(T{1.125}), doctest::Approx(1.25));
    CHECK_EQ(static_cast<double>(T{1.375}), doctest::Approx(1.5));
    CHECK_EQ(static_cast<double>(T{-1.125}), doctest::Approx(-1.25));
    CHECK_EQ(static_cast<double>(T{-1.375}), doctest::Approx(-1.5));
}

TEST_CASE_TEMPLATE(
    "Float truncation on construction (EnableRounding=false)", T,
    Fixed<std::int32_t, std::int64_t, 2, false>
) {
    CHECK_EQ(static_cast<double>(T{1.125}), doctest::Approx(1.0));
    CHECK_EQ(static_cast<double>(T{1.375}), doctest::Approx(1.25));
    CHECK_EQ(static_cast<double>(T{1.499}), doctest::Approx(1.25));
    CHECK_EQ(static_cast<double>(T{-1.125}), doctest::Approx(-1.0));
    CHECK_EQ(static_cast<double>(T{-1.375}), doctest::Approx(-1.25));
}

TEST_CASE_TEMPLATE("Fixed-to-fixed conversion with rounding", T, Fixed32f16) {
    // Widen: zero-extend lower bits
    using Q = Fixed<std::int32_t, std::int64_t, 8>;
    const auto q = Q::fromRawValue(0x12);
    const T p = T{q};
    CHECK_EQ(p.raw_value(), T{q}.raw_value());

    // Narrow: round when losing bits (Fixed32f16 -> Fixed32f8)
    using P16 = Fixed<std::int32_t, std::int64_t, 16>;
    using P8 = Fixed<std::int32_t, std::int64_t, 8>;
    // 0x12ff >> 8 should round up to 0x13
    CHECK_EQ(P8::fromRawValue(0x13), P8{P16::fromRawValue(0x12ff)});
    CHECK_EQ(P8::fromRawValue(0x12), P8{P16::fromRawValue(0x127f)});
}

TEST_CASE("fromFixedPoint precision and rounding") {
    using P = Fixed32f16;

    // From fewer fraction bits: zero-extend
    CHECK_EQ(P::fromRawValue(-1 << 16), P::fromFixedPoint<0>(-1));
    CHECK_EQ(P::fromRawValue(1 << 16), P::fromFixedPoint<0>(1));

    // 18 fraction bits -> 16: round half-up
    // 0x2_0000 + 2 = mid-point, rounds to 0x8001
    CHECK_EQ(P::fromRawValue(0x8001), P::fromFixedPoint<18>(0x20003LL));
    CHECK_EQ(P::fromRawValue(0x8000), P::fromFixedPoint<18>(0x20000LL));
}

// ─── Constants ───────────────────────────────────────────────────────────────

TEST_CASE_TEMPLATE(
    "Mathematical constants", T, Fixed32f16, Fixed32f24, Fixed64f16, Fixed64f24, Fixed64f32,
    Fixed64f40, Fixed64f48
) {
    CHECK_EQ(static_cast<double>(T::pi()), doctest::Approx(3.14159265358979).epsilon(1e-3));
    CHECK_EQ(static_cast<double>(T::e()), doctest::Approx(2.71828182845905).epsilon(1e-3));
    CHECK_EQ(static_cast<double>(T::halfPi()), doctest::Approx(1.57079632679490).epsilon(1e-3));
    CHECK_EQ(static_cast<double>(T::twoPi()), doctest::Approx(6.28318530717959).epsilon(1e-3));
}

// ─── Arithmetic ──────────────────────────────────────────────────────────────

TEST_CASE_TEMPLATE(
    "Basic arithmetic operators", T, Fixed32f8, Fixed32f16, Fixed32f24, Fixed64f8, Fixed64f16,
    Fixed64f24, Fixed64f32, Fixed64f40, Fixed64f48, Fixed64f56
) {
    CHECK_EQ(T{3.5} + T{7.25}, T{10.75});
    CHECK_EQ(T{3.5} - T{7.25}, T{-3.75});
    CHECK_EQ(T{-7.25} + T{3.5}, T{-3.75});
    CHECK_EQ(-T{13.125}, T{-13.125});
    CHECK_EQ(-T{-13.125}, T{13.125});

    // Integer operands
    CHECK_EQ(static_cast<double>(T{1} + 2), doctest::Approx(3.0));
    CHECK_EQ(static_cast<double>(2 + T{1}), doctest::Approx(3.0));
    CHECK_EQ(static_cast<double>(T{5} - 2), doctest::Approx(3.0));
    CHECK_EQ(static_cast<double>(T{3} * 2), doctest::Approx(6.0));
    CHECK_EQ(static_cast<double>(2 * T{3}), doctest::Approx(6.0));
    CHECK_EQ(static_cast<double>(T{6} / 2), doctest::Approx(3.0));
}

TEST_CASE_TEMPLATE(
    "Multiplication with exact fractional values", T, Fixed32f8, Fixed32f16, Fixed32f24, Fixed64f16,
    Fixed64f24, Fixed64f32, Fixed64f40, Fixed64f48
) {
    CHECK_EQ(T{3.5} * T{-7.25}, T{-25.375});
}

TEST_CASE_TEMPLATE(
    "Division exactness and sign combinations", T, Fixed32f16, Fixed64f16, Fixed64f24, Fixed64f32,
    Fixed64f40, Fixed64f48
) {
    CHECK_EQ(T{3.5} / T{7.25}, T{3.5 / 7.25});
    CHECK_EQ(T{-3.5} / T{7.25}, T{-3.5 / 7.25});
    CHECK_EQ(T{3.5} / T{-7.25}, T{3.5 / -7.25});
    CHECK_EQ(T{-3.5} / T{-7.25}, T{-3.5 / -7.25});
}

TEST_CASE("Division uses intermediate type to avoid overflow") {
    using P = Fixed<std::int32_t, std::int64_t, 12>;
    CHECK_EQ(P{32}, P{256} / P{8});
}

TEST_CASE("Multiplication rounding vs truncation") {
    using QRound = Fixed<std::int32_t, std::int64_t, 1, true>;
    using QTrunc = Fixed<std::int32_t, std::int64_t, 1, false>;

    CHECK_EQ(QRound{1.0}, QRound{1.5} * QRound{0.5});
    CHECK_EQ(QRound{0.5}, QRound{0.5} * QRound{0.5});
    CHECK_EQ(QTrunc{0.5}, QTrunc{1.5} * QTrunc{0.5});
    CHECK_EQ(QTrunc{0.0}, QTrunc{0.5} * QTrunc{0.5});
}

TEST_CASE("Division rounding vs truncation") {
    using QRound = Fixed<std::int32_t, std::int64_t, 1, true>;
    using QTrunc = Fixed<std::int32_t, std::int64_t, 1, false>;

    CHECK_EQ(QRound{2.5}, QRound{3.5} / QRound{1.5});
    CHECK_EQ(QRound{0.5}, QRound{1.0} / QRound{1.5});
    CHECK_EQ(QTrunc{2.0}, QTrunc{3.5} / QTrunc{1.5});
    CHECK_EQ(QTrunc{0.5}, QTrunc{1.0} / QTrunc{1.5});
}

TEST_CASE_TEMPLATE("Compound assignment operators", T, Fixed32f16, Fixed32f24) {
    T x{5};
    x += T{3};
    CHECK_EQ(static_cast<double>(x), doctest::Approx(8.0));
    x -= T{2};
    CHECK_EQ(static_cast<double>(x), doctest::Approx(6.0));
    x *= T{2};
    CHECK_EQ(static_cast<double>(x), doctest::Approx(12.0));
    x /= T{4};
    CHECK_EQ(static_cast<double>(x), doctest::Approx(3.0));
}

// ─── Comparison ──────────────────────────────────────────────────────────────

TEST_CASE_TEMPLATE(
    "Relational operators and spaceship", T, Fixed32f16, Fixed32f24, Fixed64f8, Fixed64f16,
    Fixed64f24, Fixed64f32, Fixed64f40, Fixed64f48, Fixed64f56
) {
    CHECK(T{1} == T{1});
    CHECK(T{1} != T{2});
    CHECK(T{1} < T{2});
    CHECK(T{2} > T{1});
    CHECK(T{1} <= T{1});
    CHECK(T{1} <= T{2});
    CHECK(T{2} >= T{2});
    CHECK(T{2} >= T{1});
    CHECK((T{1} <=> T{2}) == std::strong_ordering::less);
    CHECK((T{2} <=> T{1}) == std::strong_ordering::greater);
    CHECK((T{1} <=> T{1}) == std::strong_ordering::equal);
}

// ─── Classification ──────────────────────────────────────────────────────────

TEST_CASE_TEMPLATE("fpclassify", T, Fixed<std::int32_t, std::int64_t, 12>) {
    CHECK_EQ(FP_NORMAL, fpclassify(T{1.0}));
    CHECK_EQ(FP_NORMAL, fpclassify(T{-1.0}));
    CHECK_EQ(FP_NORMAL, fpclassify(T{0.5}));
    CHECK_EQ(FP_ZERO, fpclassify(T{0}));
}

TEST_CASE_TEMPLATE(
    "isfinite / isinf / isnan / isnormal / signbit", T, Fixed32f16, Fixed32f24, Fixed64f8,
    Fixed64f16, Fixed64f24, Fixed64f32, Fixed64f40, Fixed64f48, Fixed64f56
) {
    CHECK(isfinite(T{1}));
    CHECK(isfinite(T{0}));
    CHECK_FALSE(isinf(T{1}));
    CHECK_FALSE(isinf(T{0}));
    CHECK_FALSE(isnan(T{1}));
    CHECK_FALSE(isnan(T{0}));
    CHECK(isnormal(T{1}));
    CHECK_FALSE(isnormal(T{0}));
    CHECK(signbit(T{-1}));
    CHECK_FALSE(signbit(T{1}));
    CHECK_FALSE(signbit(T{0}));
}

TEST_CASE_TEMPLATE(
    "isgreater / isgreaterequal / isless / islessequal / islessgreater / isunordered", T,
    Fixed<std::int32_t, std::int64_t, 12>
) {
    CHECK_FALSE(isgreater(T{1.0}, T{2.0}));
    CHECK_FALSE(isgreater(T{1.0}, T{1.0}));
    CHECK(isgreater(T{2.0}, T{1.0}));
    CHECK(isgreater(T{-1.0}, T{-2.0}));

    CHECK(isgreaterequal(T{1.0}, T{1.0}));
    CHECK(isgreaterequal(T{2.0}, T{1.0}));
    CHECK_FALSE(isgreaterequal(T{1.0}, T{2.0}));

    CHECK(isless(T{1.0}, T{2.0}));
    CHECK_FALSE(isless(T{1.0}, T{1.0}));
    CHECK_FALSE(isless(T{2.0}, T{1.0}));

    CHECK(islessequal(T{1.0}, T{1.0}));
    CHECK(islessequal(T{1.0}, T{2.0}));
    CHECK_FALSE(islessequal(T{2.0}, T{1.0}));

    CHECK(islessgreater(T{1.0}, T{2.0}));
    CHECK_FALSE(islessgreater(T{1.0}, T{1.0}));

    CHECK_FALSE(isunordered(T{1.0}, T{2.0}));
    CHECK_FALSE(isunordered(T{1.0}, T{1.0}));
}

// ─── Rounding ────────────────────────────────────────────────────────────────

TEST_CASE_TEMPLATE(
    "ceil / floor / trunc / round — positive, negative, zero, exact integer", T, Fixed32f8,
    Fixed32f16, Fixed32f24, Fixed64f8, Fixed64f16, Fixed64f24, Fixed64f32, Fixed64f40, Fixed64f48,
    Fixed64f56
) {
    // ceil
    CHECK_EQ(ceil(T{1}), T{1});
    CHECK_EQ(ceil(T{-1}), T{-1});
    CHECK_EQ(ceil(T{2.4}), T{3});
    CHECK_EQ(ceil(T{-2.4}), T{-2});
    CHECK_EQ(ceil(T{0}), T{0});

    // floor
    CHECK_EQ(floor(T{1}), T{1});
    CHECK_EQ(floor(T{-1}), T{-1});
    CHECK_EQ(floor(T{2.7}), T{2});
    CHECK_EQ(floor(T{-2.7}), T{-3});
    CHECK_EQ(floor(T{0}), T{0});

    // trunc
    CHECK_EQ(trunc(T{1}), T{1});
    CHECK_EQ(trunc(T{-1}), T{-1});
    CHECK_EQ(trunc(T{2.7}), T{2});
    CHECK_EQ(trunc(T{-2.9}), T{-2});
    CHECK_EQ(trunc(T{0}), T{0});

    // round (half-away-from-zero)
    CHECK_EQ(round(T{2.3}), T{2});
    CHECK_EQ(round(T{2.5}), T{3});
    CHECK_EQ(round(T{2.7}), T{3});
    CHECK_EQ(round(T{-2.3}), T{-2});
    CHECK_EQ(round(T{-2.5}), T{-3});
    CHECK_EQ(round(T{-2.7}), T{-3});
    CHECK_EQ(round(T{0}), T{0});
}

TEST_CASE_TEMPLATE("nearbyint and rint — round half to even", T, Fixed32f8, Fixed32f16) {
    CHECK_EQ(nearbyint(T{2.3}), T{2});
    CHECK_EQ(nearbyint(T{2.5}), T{2}); // half: even (2)
    CHECK_EQ(nearbyint(T{3.5}), T{4}); // half: even (4)
    CHECK_EQ(nearbyint(T{-2.3}), T{-2});
    CHECK_EQ(nearbyint(T{-2.5}), T{-2}); // half: even (-2)
    CHECK_EQ(nearbyint(T{-3.5}), T{-4}); // half: even (-4)
    CHECK_EQ(nearbyint(T{0}), T{0});

    CHECK_EQ(rint(T{2.3}), T{2});
    CHECK_EQ(rint(T{2.5}), T{2});
    CHECK_EQ(rint(T{3.5}), T{4});
    CHECK_EQ(rint(T{-2.5}), T{-2});
    CHECK_EQ(rint(T{-3.5}), T{-4});
}

// ─── Basic math ──────────────────────────────────────────────────────────────

TEST_CASE_TEMPLATE(
    "abs", T, Fixed32f8, Fixed32f16, Fixed32f24, Fixed64f8, Fixed64f16, Fixed64f24, Fixed64f32,
    Fixed64f40, Fixed64f48, Fixed64f56
) {
    CHECK_EQ(abs(T{-13.125}), T{13.125});
    CHECK_EQ(abs(T{13.125}), T{13.125});
    CHECK_EQ(abs(T{-1}), T{1});
    CHECK_EQ(abs(T{1}), T{1});
}

TEST_CASE_TEMPLATE("fmod — sign follows dividend", T, Fixed32f8, Fixed32f16) {
    CHECK_EQ(fmod(T{9.5}, T{2}), T{1.5});
    CHECK_EQ(fmod(T{-9.5}, T{2}), T{-1.5});
    CHECK_EQ(fmod(T{9.5}, T{-2}), T{1.5});
    CHECK_EQ(fmod(T{-9.5}, T{-2}), T{-1.5});
}

TEST_CASE_TEMPLATE("remainder — IEEE remainder (nearest integer quotient)", T, Fixed32f16) {
    CHECK_EQ(remainder(T{9.5}, T{2}), T{-0.5});
    CHECK_EQ(remainder(T{-9.5}, T{2}), T{0.5});
    CHECK_EQ(remainder(T{9}, T{2}), T{1});
    CHECK_EQ(remainder(T{11}, T{2}), T{-1});
    CHECK_EQ(remainder(T{0}, T{1}), T{0});
}

TEST_CASE_TEMPLATE("remquo quotient bits", T, Fixed32f16) {
    int quo = 0;
    CHECK_EQ(remquo(T{9.5}, T{2}, &quo), T{1.5});
    CHECK_EQ(quo % 8, 4);
    CHECK_EQ(remquo(T{-9.5}, T{2}, &quo), T{-1.5});
    CHECK_EQ(quo % 8, -4);
    CHECK_EQ(remquo(T{0}, T{1}, &quo), T{0});
    CHECK_EQ(quo % 8, 0);
}

TEST_CASE_TEMPLATE("modf integer and fraction parts", T, Fixed32f16, Fixed32f24) {
    T ipart{};
    const T frac = modf(T{3.75}, &ipart);
    CHECK_EQ(static_cast<double>(ipart), doctest::Approx(3.0));
    CHECK_EQ(static_cast<double>(frac), doctest::Approx(0.75).epsilon(1e-3));

    const T frac2 = modf(T{-2.25}, &ipart);
    CHECK_EQ(static_cast<double>(ipart), doctest::Approx(-2.0));
    CHECK_EQ(static_cast<double>(frac2), doctest::Approx(-0.25).epsilon(1e-3));
}

TEST_CASE_TEMPLATE("copysign and nextafter", T, Fixed32f16, Fixed32f24) {
    CHECK_EQ(copysign(T{3}, T{-1}), T{-3});
    CHECK_EQ(copysign(T{-3}, T{1}), T{3});
    CHECK_EQ(copysign(T{3}, T{0}), T{3});

    const T eps = std::numeric_limits<T>::epsilon();
    CHECK_EQ(nextafter(T{1}, T{2}), T{1} + eps);
    CHECK_EQ(nextafter(T{1}, T{0}), T{1} - eps);
    CHECK_EQ(nextafter(T{1}, T{1}), T{1});
}

// ─── Power / Exponential / Logarithm ─────────────────────────────────────────

TEST_CASE_TEMPLATE(
    "sqrt exact cases", T, Fixed32f8, Fixed32f16, Fixed32f24, Fixed64f8, Fixed64f16, Fixed64f24,
    Fixed64f32, Fixed64f40, Fixed64f48, Fixed64f56
) {
    CHECK_EQ(static_cast<double>(sqrt(T{4})), doctest::Approx(2.0).epsilon(5e-3));
    CHECK_EQ(static_cast<double>(sqrt(T{9})), doctest::Approx(3.0).epsilon(5e-3));
    CHECK_EQ(static_cast<double>(sqrt(T{0})), doctest::Approx(0.0));
}

TEST_CASE_TEMPLATE(
    "cbrt exact cases", T, Fixed32f16, Fixed32f24, Fixed64f16, Fixed64f24, Fixed64f32, Fixed64f40,
    Fixed64f48, Fixed64f56
) {
    CHECK_EQ(static_cast<double>(cbrt(T{8})), doctest::Approx(2.0).epsilon(5e-3));
    CHECK_EQ(static_cast<double>(cbrt(T{-8})), doctest::Approx(-2.0).epsilon(5e-3));
    CHECK_EQ(static_cast<double>(cbrt(T{0})), doctest::Approx(0.0));
}

TEST_CASE_TEMPLATE(
    "hypot", T, Fixed32f16, Fixed32f24, Fixed64f16, Fixed64f24, Fixed64f32, Fixed64f40, Fixed64f48,
    Fixed64f56
) {
    CHECK_EQ(static_cast<double>(hypot(T{3}, T{4})), doctest::Approx(5.0).epsilon(5e-3));
}

TEST_CASE_TEMPLATE(
    "pow integer exponent", T, Fixed32f16, Fixed64f8, Fixed64f16, Fixed64f24, Fixed64f32,
    Fixed64f40, Fixed64f48
) {
    // Fixed32f24/Fixed64f56 have too few integer bits (max ≈127), so 2^10=1024 overflows;
    // exclude them here (see "pow integer exponent (small range)" below).
    CHECK_EQ(static_cast<double>(pow(T{2}, 10)), doctest::Approx(1024.0).epsilon(5e-3));
    CHECK_EQ(static_cast<double>(pow(T{3}, 3)), doctest::Approx(27.0).epsilon(5e-3));
    CHECK_EQ(static_cast<double>(pow(T{2}, -1)), doctest::Approx(0.5).epsilon(5e-3));
}

TEST_CASE_TEMPLATE("pow integer exponent (small range)", T, Fixed32f24, Fixed64f56) {
    CHECK_EQ(static_cast<double>(pow(T{2}, 3)), doctest::Approx(8.0).epsilon(5e-3));
    CHECK_EQ(static_cast<double>(pow(T{3}, 3)), doctest::Approx(27.0).epsilon(5e-3));
    CHECK_EQ(static_cast<double>(pow(T{2}, -1)), doctest::Approx(0.5).epsilon(5e-3));
}

TEST_CASE_TEMPLATE(
    "exp / exp2 / expm1 relative error over [-5, 5]", T, Fixed32f16, Fixed64f16, Fixed64f24,
    Fixed64f32
) {
    for (int i = -50; i <= 50; ++i) {
        const double v = i * 0.1;
        CHECK(hasMaxRelativeError(static_cast<double>(exp(T{v})), std::exp(v), 0.1));
        CHECK(hasMaxRelativeError(static_cast<double>(exp2(T{v})), std::exp2(v), 0.1));
    }
}

TEST_CASE_TEMPLATE(
    "log / log2 / log10 relative error over [0.1, 100]", T, Fixed32f16, Fixed64f16, Fixed64f24,
    Fixed64f32
) {
    // fpm uses 1% for log functions
    for (int i = 1; i <= 100; ++i) {
        const double v = i * 0.1 + 0.01;
        CHECK(hasMaxRelativeError(static_cast<double>(log(T{v})), std::log(v), 1.0));
        CHECK(hasMaxRelativeError(static_cast<double>(log2(T{v})), std::log2(v), 1.0));
        CHECK(hasMaxRelativeError(static_cast<double>(log10(T{v})), std::log10(v), 1.0));
    }
}

// ─── Trigonometry ─────────────────────────────────────────────────────────────

TEST_CASE_TEMPLATE(
    "sin / cos relative error over [-180°, 180°]", T, Fixed32f16, Fixed64f16, Fixed64f24, Fixed64f32
) {
    // abs_floor=1.0 because sin/cos ∈ [-1,1]: near-zero reference values (sin(π), cos(π/2))
    // have |ref| ≪ 1e-4 in double due to floating-point representation, while the fixed-point
    // result is at most 1 LSB away from zero. Using abs_floor=1.0 turns the check into a
    // pure absolute-error test: |got - ref| ≤ 0.001, which 1 LSB always satisfies.
    for (int deg = -180; deg <= 180; deg += 5) {
        const double rad = deg * 3.14159265358979 / 180.0;
        CHECK(hasMaxRelativeError(static_cast<double>(sin(T{rad})), std::sin(rad), 0.1, 1.0));
        CHECK(hasMaxRelativeError(static_cast<double>(cos(T{rad})), std::cos(rad), 0.1, 1.0));
    }
}

TEST_CASE_TEMPLATE(
    "tan relative error (excluding ±90°)", T, Fixed32f16, Fixed64f16, Fixed64f24, Fixed64f32
) {
    for (int deg = -80; deg <= 80; deg += 10) {
        const double rad = deg * 3.14159265358979 / 180.0;
        CHECK(hasMaxRelativeError(static_cast<double>(tan(T{rad})), std::tan(rad), 0.2));
    }
}

TEST_CASE_TEMPLATE(
    "atan / asin / acos / atan2 key values", T, Fixed32f16, Fixed64f16, Fixed64f24, Fixed64f32
) {
    constexpr double kPi = 3.14159265358979;
    CHECK_EQ(static_cast<double>(atan(T{1})), doctest::Approx(kPi / 4).epsilon(0.01));
    CHECK_EQ(static_cast<double>(atan(T{0})), doctest::Approx(0.0).epsilon(0.01));
    CHECK_EQ(static_cast<double>(asin(T{1})), doctest::Approx(kPi / 2).epsilon(0.01));
    CHECK_EQ(static_cast<double>(asin(T{0})), doctest::Approx(0.0).epsilon(0.01));
    CHECK_EQ(static_cast<double>(acos(T{1})), doctest::Approx(0.0).epsilon(0.01));
    CHECK_EQ(static_cast<double>(acos(T{0})), doctest::Approx(kPi / 2).epsilon(0.01));
    CHECK_EQ(static_cast<double>(atan2(T{1}, T{1})), doctest::Approx(kPi / 4).epsilon(0.01));
    CHECK_EQ(static_cast<double>(atan2(T{1}, T{0})), doctest::Approx(kPi / 2).epsilon(0.01));
}

// ─── Type traits / numeric_limits ────────────────────────────────────────────

TEST_CASE_TEMPLATE(
    "isFixedPointV trait", T, Fixed32f8, Fixed32f16, Fixed32f24, Fixed64f8, Fixed64f16, Fixed64f24,
    Fixed64f32, Fixed64f40, Fixed64f48, Fixed64f56
) {
    CHECK(isFixedPointV<T>);
    CHECK_FALSE(isFixedPointV<int>);
    CHECK_FALSE(isFixedPointV<double>);
}

TEST_CASE_TEMPLATE(
    "numeric_limits", T, Fixed32f8, Fixed32f16, Fixed32f24, Fixed64f8, Fixed64f16, Fixed64f24,
    Fixed64f32, Fixed64f40, Fixed64f48, Fixed64f56
) {
    CHECK(std::numeric_limits<T>::is_specialized);
    CHECK_FALSE(std::numeric_limits<T>::is_integer);
    CHECK(std::numeric_limits<T>::is_exact);
    CHECK_FALSE(std::numeric_limits<T>::has_infinity);
    CHECK(std::numeric_limits<T>::max() > std::numeric_limits<T>::min());
    CHECK_EQ(std::numeric_limits<T>::epsilon().raw_value(), 1);
}

// ─── I/O ─────────────────────────────────────────────────────────────────────

TEST_CASE_TEMPLATE(
    "operator<< and operator>> round-trip", T, Fixed32f16, Fixed32f24, Fixed64f16, Fixed64f24,
    Fixed64f32, Fixed64f40, Fixed64f48
) {
    std::ostringstream oss;
    oss << T{3.14};
    CHECK_FALSE(oss.str().empty());

    T val{0};
    std::istringstream iss("2.5");
    iss >> val;
    CHECK_EQ(static_cast<double>(val), doctest::Approx(2.5).epsilon(1e-2));

    // negative
    std::istringstream iss2("-1.25");
    iss2 >> val;
    CHECK_EQ(static_cast<double>(val), doctest::Approx(-1.25).epsilon(1e-2));
}

TEST_CASE_TEMPLATE(
    "std::format fixed / scientific / width / align / sign", T, Fixed32f16, Fixed32f24, Fixed64f16,
    Fixed64f24, Fixed64f32, Fixed64f40, Fixed64f48
) {
    CHECK_FALSE(std::format("{}", T{3.14}).empty());
    CHECK_EQ(std::stod(std::format("{:.2f}", T{3.14})), doctest::Approx(3.14).epsilon(1e-2));
    CHECK_FALSE(std::format("{:e}", T{3.14}).empty());

    const std::string wide = std::format("{:10.2f}", T{1.5});
    CHECK_EQ(static_cast<int>(wide.size()), 10);

    const std::string left = std::format("{:<10.2f}", T{1.5});
    CHECK_EQ(static_cast<int>(left.size()), 10);
    CHECK_EQ(left.back(), ' ');

    const std::string center = std::format("{:^10.2f}", T{1.5});
    CHECK_EQ(static_cast<int>(center.size()), 10);

    const std::string pos = std::format("{:+.2f}", T{1.5});
    CHECK_EQ(pos.front(), '+');

    const std::string spc = std::format("{: .2f}", T{1.5});
    CHECK_EQ(spc.front(), ' ');
}

// ─── Serialization ───────────────────────────────────────────────────────────

TEST_CASE_TEMPLATE(
    "Serialization roundtrip", T, Fixed32f8, Fixed32f16, Fixed32f24, Fixed64f8, Fixed64f16,
    Fixed64f24, Fixed64f32, Fixed64f40, Fixed64f48, Fixed64f56
) {
    const T values[] = {
        T{0}, T{3.14}, T{-2.71828}, std::numeric_limits<T>::max(), std::numeric_limits<T>::min()
    };
    for (const T a : values) {
        auto [data, out] = zpp::bits::data_out();
        out(a).or_throw();

        T b{};
        auto in = zpp::bits::in(data);
        in(b).or_throw();

        CHECK_EQ(a.raw_value(), b.raw_value());
    }
}

} // namespace basjoo::math
