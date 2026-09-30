/**
 * @file test_concepts.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief Unit tests for the scalar arithmetic concepts.
 * @version 0.1
 * @date 2026-10-14
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/concepts.hpp"

#include <complex>
#include <vector>

// Brings the dense operator templates constrained on ScalarArithmetic into scope, the setting in
// which an operator-probing ComplexArithmetic recursed into itself on MSVC.
#include "basjoo/math/dense/vector.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace test_concepts {

// Exposes the full std::complex surface without being a std::complex specialization. Declared
// only: the members are named in unevaluated operands alone.
struct ComplexLike {
    using value_type = double;

    auto real() const -> double;
    auto imag() const -> double;

    friend auto operator+(ComplexLike, ComplexLike) -> ComplexLike;
    friend auto operator-(ComplexLike, ComplexLike) -> ComplexLike;
    friend auto operator*(ComplexLike, ComplexLike) -> ComplexLike;
    friend auto operator/(ComplexLike, ComplexLike) -> ComplexLike;
    friend auto operator+(ComplexLike, double) -> ComplexLike;
    friend auto operator-(ComplexLike, double) -> ComplexLike;
    friend auto operator*(ComplexLike, double) -> ComplexLike;
    friend auto operator/(ComplexLike, double) -> ComplexLike;
    friend auto operator+(double, ComplexLike) -> ComplexLike;
    friend auto operator-(double, ComplexLike) -> ComplexLike;
    friend auto operator*(double, ComplexLike) -> ComplexLike;
    friend auto operator/(double, ComplexLike) -> ComplexLike;
};

} // namespace test_concepts

TEST_CASE_TEMPLATE(
    "ComplexArithmetic accepts std::complex over floating-point types", T, std::complex<float>,
    std::complex<double>, std::complex<long double>
) {
    using basjoo::math::ComplexArithmetic;
    using basjoo::math::ScalarArithmetic;

    static_assert(ComplexArithmetic<T>);
    static_assert(ComplexArithmetic<const T>);
    static_assert(ComplexArithmetic<T&>);
    static_assert(ComplexArithmetic<const T&>);
    static_assert(ComplexArithmetic<T&&>);
    static_assert(ScalarArithmetic<T>);
    static_assert(ScalarArithmetic<const T&>);
}

TEST_CASE_TEMPLATE(
    "ComplexArithmetic rejects real scalars and non-complex types", T, float, double, int,
    std::complex<int>, test_concepts::ComplexLike, std::vector<std::complex<double>>,
    basjoo::math::Vec2d, basjoo::math::Vec2z, basjoo::math::Vec3z
) {
    static_assert(!basjoo::math::ComplexArithmetic<T>);
}

TEST_CASE_TEMPLATE(
    "ScalarArithmetic accepts arithmetic types", T, float, double, int, std::size_t, const double&
) {
    static_assert(basjoo::math::ScalarArithmetic<T>);
}

TEST_CASE_TEMPLATE(
    "ScalarArithmetic rejects non-scalar types", T, std::complex<int>, test_concepts::ComplexLike,
    basjoo::math::Vec2d, basjoo::math::Vec3z
) {
    static_assert(!basjoo::math::ScalarArithmetic<T>);
}
