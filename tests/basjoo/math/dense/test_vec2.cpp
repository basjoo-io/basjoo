/**
 * @file test_vec2.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-07-17
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/dense/vec2.hpp"

#include <cmath>
#include <format>
#include <numbers>

#include "zpp_bits.h"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::math {

TEST_CASE_TEMPLATE(
    "Constructor", T, Vec2<std::int32_t>, Vec2<std::int64_t>, Vec2s, Vec2d, Vec2c, Vec2z
) {
    const T a(0.0, 0.0);
    CHECK_EQ(a.x, static_cast<typename T::value_type>(0.0));
    CHECK_EQ(a.y, static_cast<typename T::value_type>(0.0));

    const T b(4.0);
    CHECK_EQ(b.x, static_cast<typename T::value_type>(4.0));
    CHECK_EQ(b.y, static_cast<typename T::value_type>(4.0));

    const T c(std::make_pair(1.0, 6.0));
    CHECK_EQ(c.x, static_cast<typename T::value_type>(1.0));
    CHECK_EQ(c.y, static_cast<typename T::value_type>(6.0));

    const T d(c);
    CHECK_EQ(d.x, c.x);
    CHECK_EQ(d.y, c.y);
}

TEST_CASE_TEMPLATE("Basic", T, Vec2s, Vec2d) {
    const T a(1.0, std::numbers::sqrt3);
    CHECK_EQ(a.euclidean(), doctest::Approx(2.0).epsilon(1E-12));
    CHECK_EQ(a.radian(), doctest::Approx(M_PI / 3.0).epsilon(1E-6));
    CHECK_EQ(std::hypot(a), doctest::Approx(2.0).epsilon(1E-12));
    CHECK_EQ(a[0], a.x);
    CHECK_EQ(a[1], a.y);

    const T b = a.rotate(static_cast<typename T::value_type>(M_PI / 6.0));
    CHECK_EQ(b.x, doctest::Approx(0.0).epsilon(1E-12));
    CHECK_EQ(b.y, doctest::Approx(2.0).epsilon(1E-12));
    CHECK_EQ(b.radian(), doctest::Approx(M_PI_2).epsilon(1E-6));

    const T c = a.normalized();
    CHECK_EQ(c.x, doctest::Approx(0.5).epsilon(1E-12));
    CHECK_EQ(c.y, doctest::Approx(std::sqrt(3.0) * 0.5).epsilon(1E-8));
}

TEST_CASE_TEMPLATE("IntegralArithmetic", T, Vec2<std::int32_t>, Vec2<std::int64_t>) {
    constexpr T a(94, -37);
    constexpr T b(-18, 27);

    constexpr auto fac_1(10);
    constexpr auto fac_2(4);
    constexpr auto den(5);

    const T result = (a * fac_1) + (b / den) - (fac_2 * a);

    CHECK_EQ(result.x, a.x * fac_1 + b.x / den - fac_2 * a.x);
    CHECK_EQ(result.y, a.y * fac_1 + b.y / den - fac_2 * a.y);

    const typename T::value_type a_euclidean = a.euclidean();
    const typename T::value_type exact_a_euclidean = std::sqrt(std::norm(a.x) + std::norm(a.y));
    CHECK_EQ(a_euclidean, exact_a_euclidean);

    const typename T::value_type a_dot_b = a.dot(b);
    const typename T::value_type exact_a_dot_b = a.x * b.x + a.y * b.y;
    CHECK_EQ(a_dot_b, exact_a_dot_b);
}

TEST_CASE_TEMPLATE("FloatingArithmetic", T, Vec2s, Vec2d) {
    constexpr T a(1.0, 0.0);
    constexpr T b(0.5, std::numbers::sqrt3 * 0.5);

    constexpr auto fac_1(10.5634);
    constexpr auto fac_2(0.67);
    constexpr auto den(5.34);

    const T result = (a * fac_1) + (b / den) - (fac_2 * a);

    CHECK_EQ(result.x, doctest::Approx(a.x * fac_1 + b.x / den - fac_2 * a.x).epsilon(1E-8));
    CHECK_EQ(result.y, doctest::Approx(a.y * fac_1 + b.y / den - fac_2 * a.y).epsilon(1E-8));

    const typename T::value_type a_euclidean = a.euclidean();
    const typename T::value_type exact_a_euclidean = std::sqrt(std::norm(a.x) + std::norm(a.y));
    CHECK_EQ(a_euclidean, doctest::Approx(exact_a_euclidean).epsilon(1E-12));

    const typename T::value_type a_dot_b = a.dot(b);
    const typename T::value_type exact_a_dot_b = a.x * b.x + a.y * b.y;
    CHECK_EQ(a_dot_b, doctest::Approx(exact_a_dot_b).epsilon(1E-12));

    const typename T::value_type a_crossproj_b = a.crossProj(b);
    const typename T::value_type exact_a_crossproj_b = a.x * b.y - a.y * b.x;
    CHECK_EQ(a_crossproj_b, doctest::Approx(exact_a_crossproj_b).epsilon(1E-12));
}

TEST_CASE_TEMPLATE("ComplexArithmetic", T, Vec2c, Vec2z) {
    constexpr T a(typename T::value_type(9.573, 364.26), typename T::value_type(-423.4, 17.44));
    constexpr T b(typename T::value_type(1.545, -5.35), typename T::value_type(49.234, 39.23));

    const T a_conj = a.conjugated();
    CHECK_EQ(a_conj.x.real(), a.x.real());
    CHECK_EQ(a_conj.x.imag(), -a.x.imag());
    CHECK_EQ(a_conj.y.real(), a.y.real());
    CHECK_EQ(a_conj.y.imag(), -a.y.imag());

    constexpr auto fac_1{typename T::value_type(10.5634, -3.65)};
    constexpr auto fac_2{typename T::value_type(0.67, -0.38)};
    constexpr auto den{typename T::value_type(5.34, 7.35)};

    const T result = (a * fac_1) + (b / den) - (fac_2 * a);

    CHECK_EQ(
        result.x.real(),
        doctest::Approx((a.x * fac_1 + b.x / den - fac_2 * a.x).real()).epsilon(1E-12)
    );
    CHECK_EQ(
        result.x.imag(),
        doctest::Approx((a.x * fac_1 + b.x / den - fac_2 * a.x).imag()).epsilon(1E-12)
    );
    CHECK_EQ(
        result.y.real(),
        doctest::Approx((a.y * fac_1 + b.y / den - fac_2 * a.y).real()).epsilon(1E-12)
    );
    CHECK_EQ(
        result.y.imag(),
        doctest::Approx((a.y * fac_1 + b.y / den - fac_2 * a.y).imag()).epsilon(1E-12)
    );

    const typename T::value_type::value_type a_euclidean = a.euclidean();
    const typename T::value_type::value_type exact_a_euclidean =
        std::sqrt(std::norm(a.x) + std::norm(a.y));
    CHECK_EQ(a_euclidean, doctest::Approx(exact_a_euclidean).epsilon(1E-12));

    const typename T::value_type a_dot_b = a.dot(b);
    const typename T::value_type exact_a_dot_b = a.x * b.x + a.y * b.y;
    CHECK_EQ(a_dot_b.real(), doctest::Approx(exact_a_dot_b.real()).epsilon(1E-12));
    CHECK_EQ(a_dot_b.imag(), doctest::Approx(exact_a_dot_b.imag()).epsilon(1E-6));

    const typename T::value_type a_crossproj_b = a.crossProj(b);
    const typename T::value_type exact_a_crossproj_b = a.x * b.y - a.y * b.x;
    CHECK_EQ(a_crossproj_b.real(), doctest::Approx(exact_a_crossproj_b.real()).epsilon(1E-12));
    CHECK_EQ(a_crossproj_b.imag(), doctest::Approx(exact_a_crossproj_b.imag()).epsilon(1E-12));
}

TEST_CASE("OstreamOperator") {
    constexpr Vec2d a(1274.12, 4454.23);

    std::ostringstream oss;
    oss << a;

    CHECK_EQ(oss.str(), "(x: 1274.12, y: 4454.23)");
}

TEST_CASE("Format") {
    constexpr Vec2d a(1274.12, 4454.23);

    CHECK_EQ(std::format("{}", a), "(x: 1274.12, y: 4454.23)");
    CHECK_EQ(std::format("{0}", a), "(x: 1274.12, y: 4454.23)");
    CHECK_EQ(std::format("{0:.2}", a), "(x: 1274.12, y: 4454.23)");
    CHECK_EQ(std::format("{0:12}", a), "(x:  1274.120000, y:  4454.230000)");
    CHECK_EQ(std::format("{0:12.2}", a), "(x:      1274.12, y:      4454.23)");
}

TEST_CASE_TEMPLATE("Serialization", T, Vec2s, Vec2d, Vec2c, Vec2z) {
    constexpr T a(1274.12, 4454.23);

    auto [data, out] = zpp::bits::data_out();
    out(a).or_throw();

    T b;
    auto in = zpp::bits::in(data);
    in(b).or_throw();

    CHECK_EQ(a.x, b.x);
    CHECK_EQ(a.y, b.y);
}

} // namespace basjoo::math
