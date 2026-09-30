/**
 * @file test_type_traits.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief Unit tests for the scalar arithmetic type traits.
 * @version 0.1
 * @date 2026-10-14
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/type_traits.hpp"

#include <complex>
#include <cstddef>
#include <mdspan>
#include <vector>

#include "basjoo/math/dense/dense_arithmetic_base.hpp"
#include "basjoo/math/dense/matrix.hpp"
// Brings the dense operator templates constrained on ScalarArithmetic into scope, the setting in
// which an operator-probing complex check recursed into itself on MSVC.
#include "basjoo/math/dense/vector.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace test_type_traits {

// A vector that is a struct of named fields, the shape of the bicycle states: it has none of
// coeff(), euclideanTo() or stride(), only what isVecArithmetic asks for.
struct FieldVec final : public basjoo::math::DenseArithmeticBase {
    using value_type = double;
    using size_type = std::size_t;
    using index_type = std::size_t;
    using rank_type = std::size_t;

    static constexpr auto rank() noexcept -> rank_type { return 1; }

    static constexpr auto extent([[maybe_unused]] rank_type r) noexcept -> index_type {
        return size();
    }

    static constexpr auto size() noexcept -> size_type { return 3; }

    constexpr auto data() noexcept -> value_type* { return &x; }

    constexpr auto data() const noexcept -> const value_type* { return &x; }

    constexpr auto operator[](index_type i) noexcept -> value_type& { return data()[i]; }

    double x{0.0};
    double y{0.0};
    double z{0.0};
};

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

} // namespace test_type_traits

TEST_CASE_TEMPLATE(
    "isComplexArithmetic accepts std::complex over floating-point types", T, std::complex<float>,
    std::complex<double>, std::complex<long double>
) {
    using basjoo::math::isComplexArithmeticV;
    using basjoo::math::isScalarArithmeticV;

    static_assert(isComplexArithmeticV<T>);
    static_assert(isComplexArithmeticV<const T>);
    static_assert(isComplexArithmeticV<T&>);
    static_assert(isComplexArithmeticV<const T&>);
    static_assert(isComplexArithmeticV<T&&>);
    static_assert(isScalarArithmeticV<T>);
    static_assert(isScalarArithmeticV<const T&>);
}

TEST_CASE_TEMPLATE(
    "isComplexArithmetic rejects real scalars and non-complex types", T, float, double, int,
    std::complex<int>, test_type_traits::ComplexLike, std::vector<std::complex<double>>,
    basjoo::math::Vec2d, basjoo::math::Vec2z, basjoo::math::Vec3z
) {
    static_assert(!basjoo::math::isComplexArithmeticV<T>);
}

TEST_CASE_TEMPLATE(
    "isScalarArithmetic accepts arithmetic types", T, float, double, int, std::size_t, const double&
) {
    static_assert(basjoo::math::isScalarArithmeticV<T>);
}

TEST_CASE_TEMPLATE(
    "isScalarArithmetic rejects non-scalar types", T, std::complex<int>,
    test_type_traits::ComplexLike, basjoo::math::Vec2d, basjoo::math::Vec3z
) {
    static_assert(!basjoo::math::isScalarArithmeticV<T>);
}

TEST_CASE_TEMPLATE(
    "isVecArithmetic accepts dense vectors of rank one", T, basjoo::math::Vec2d,
    basjoo::math::Vec3z, basjoo::math::Vector<float, std::size_t, 5>, basjoo::math::Vector<double>,
    basjoo::math::Vector<double, int, 4>, basjoo::math::pmr::Vector<std::complex<double>>,
    test_type_traits::FieldVec
) {
    using basjoo::math::isGeneralArithmeticV;
    using basjoo::math::isMatArithmeticV;
    using basjoo::math::isVecArithmeticV;

    static_assert(isVecArithmeticV<T>);
    static_assert(isVecArithmeticV<const T&>);
    static_assert(isVecArithmeticV<T&&>);
    static_assert(isGeneralArithmeticV<T>);
    static_assert(!isMatArithmeticV<T>);
}

TEST_CASE_TEMPLATE(
    "isMatArithmetic accepts dense matrices of rank two", T,
    basjoo::math::Matrix<double, std::size_t, 3, 3>, basjoo::math::Matrix<std::complex<float>>,
    basjoo::math::Matrix<double, int>,
    basjoo::math::pmr::Matrix<double, std::size_t, std::layout_right>
) {
    using basjoo::math::isGeneralArithmeticV;
    using basjoo::math::isMatArithmeticV;
    using basjoo::math::isVecArithmeticV;

    static_assert(isMatArithmeticV<T>);
    static_assert(isMatArithmeticV<const T&>);
    static_assert(isMatArithmeticV<T&&>);
    static_assert(isGeneralArithmeticV<T>);
    // A matrix has size(), data() and a linear operator[] as well, so the rank alone keeps it from
    // passing for a vector.
    static_assert(!isVecArithmeticV<T>);
}

TEST_CASE_TEMPLATE(
    "isVecArithmetic and isMatArithmetic reject scalars and containers", T, double, int,
    std::complex<double>, std::vector<double>, test_type_traits::ComplexLike
) {
    static_assert(!basjoo::math::isVecArithmeticV<T>);
    static_assert(!basjoo::math::isMatArithmeticV<T>);
}
