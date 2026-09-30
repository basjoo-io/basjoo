/**
 * @file test_vec3.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-07-17
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/dense/dense_formatter.hpp"
#include "basjoo/math/dense/vec3.hpp"

#include <array>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <format>
#include <mdspan>
#include <numbers>
#include <type_traits>
#include <vector>

#include "zpp_bits.h"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::math {

TEST_CASE_TEMPLATE(
    "Constructor", T, Vec3<std::int32_t>, Vec3<std::int64_t>, Vec3s, Vec3d, Vec3s, Vec3z
) {
    const T a(0.0, 0.0, 0.0);
    CHECK_EQ(a.x, static_cast<typename T::value_type>(0.0));
    CHECK_EQ(a.y, static_cast<typename T::value_type>(0.0));
    CHECK_EQ(a.z, static_cast<typename T::value_type>(0.0));

    const T b(4.0);
    CHECK_EQ(b.x, static_cast<typename T::value_type>(4.0));
    CHECK_EQ(b.y, static_cast<typename T::value_type>(4.0));
    CHECK_EQ(b.z, static_cast<typename T::value_type>(4.0));

    const T c(std::make_tuple(1.0, 6.0, 9.0));
    CHECK_EQ(c.x, static_cast<typename T::value_type>(1.0));
    CHECK_EQ(c.y, static_cast<typename T::value_type>(6.0));
    CHECK_EQ(c.z, static_cast<typename T::value_type>(9.0));

    const T d(c);
    CHECK_EQ(d.x, c.x);
    CHECK_EQ(d.y, c.y);
    CHECK_EQ(d.z, c.z);
}

TEST_CASE_TEMPLATE("Basic", T, Vec3s, Vec3d) {
    const T a(1.0, std::sqrt(3.0), std::sqrt(5.0));
    CHECK_EQ(a.euclidean(), doctest::Approx(3.0).epsilon(2E-7));
    CHECK_EQ(std::hypot(a), doctest::Approx(3.0).epsilon(2E-7));
    CHECK_EQ(a[0], a.x);
    CHECK_EQ(a[1], a.y);
    CHECK_EQ(a[2], a.z);

    const T b = a.normalized();
    CHECK_EQ(b.x, doctest::Approx(1.0 / 3.0).epsilon(2E-7));
    CHECK_EQ(b.y, doctest::Approx(std::sqrt(3.0) / 3.0).epsilon(2E-7));
    CHECK_EQ(b.z, doctest::Approx(std::sqrt(5.0) / 3.0).epsilon(2E-7));
}

TEST_CASE_TEMPLATE("IntegralArithmetic", T, Vec3<std::int32_t>, Vec3<std::int64_t>) {
    constexpr T a(94, -37, 42);
    constexpr T b(-18, 27, 75);

    constexpr auto fac_1(10);
    constexpr auto fac_2(4);
    constexpr auto den(5);

    const T result = (a * fac_1) + (b / den) - (fac_2 * a);

    CHECK_EQ(result.x, a.x * fac_1 + b.x / den - fac_2 * a.x);
    CHECK_EQ(result.y, a.y * fac_1 + b.y / den - fac_2 * a.y);
    CHECK_EQ(result.z, a.z * fac_1 + b.z / den - fac_2 * a.z);

    const typename T::value_type a_euclidean = a.euclidean();
    const typename T::value_type exact_a_euclidean =
        std::sqrt(std::norm(a.x) + std::norm(a.y) + std::norm(a.z));
    CHECK_EQ(a_euclidean, exact_a_euclidean);

    const typename T::value_type a_dot_b = a.dot(b);
    const typename T::value_type exact_a_dot_b = a.x * b.x + a.y * b.y + a.z * b.z;
    CHECK_EQ(a_dot_b, exact_a_dot_b);

    const T neg = -a;
    CHECK_EQ(neg.x, -a.x);
    CHECK_EQ(neg.y, -a.y);
    CHECK_EQ(neg.z, -a.z);
}

TEST_CASE_TEMPLATE("FloatingArithmetic", T, Vec3s, Vec3d) {
    constexpr T a(1.0, 0.0, 2.13);
    constexpr T b(0.5, std::numbers::sqrt3 * 0.5, 1.45);

    constexpr auto fac_1(10.5634);
    constexpr auto fac_2(0.67);
    constexpr auto den(5.34);

    const T result = (a * fac_1) + (b / den) - (fac_2 * a);

    CHECK_EQ(result.x, doctest::Approx(a.x * fac_1 + b.x / den - fac_2 * a.x).epsilon(1E-8));
    CHECK_EQ(result.y, doctest::Approx(a.y * fac_1 + b.y / den - fac_2 * a.y).epsilon(1E-8));
    CHECK_EQ(result.z, doctest::Approx(a.z * fac_1 + b.z / den - fac_2 * a.z).epsilon(1E-6));

    const typename T::value_type a_euclidean = a.euclidean();
    const typename T::value_type exact_a_euclidean =
        std::sqrt(std::norm(a.x) + std::norm(a.y) + std::norm(a.z));
    CHECK_EQ(a_euclidean, doctest::Approx(exact_a_euclidean).epsilon(1E-12));

    const typename T::value_type a_dot_b = a.dot(b);
    const typename T::value_type exact_a_dot_b = a.x * b.x + a.y * b.y + a.z * b.z;
    CHECK_EQ(a_dot_b, exact_a_dot_b);

    // Separately inlined copies of the same expression may be fused into FMAs differently.
    constexpr double roundoff{std::same_as<typename T::value_type, float> ? 1E-6 : 1E-12};

    const T a_cross_b = a.cross(b);
    const typename T::value_type exact_a_cross_b_x = a.y * b.z - a.z * b.y;
    const typename T::value_type exact_a_cross_b_y = a.z * b.x - a.x * b.z;
    const typename T::value_type exact_a_cross_b_z = a.x * b.y - a.y * b.x;
    CHECK_EQ(a_cross_b.x, doctest::Approx(exact_a_cross_b_x).epsilon(roundoff));
    CHECK_EQ(a_cross_b.y, doctest::Approx(exact_a_cross_b_y).epsilon(roundoff));
    CHECK_EQ(a_cross_b.z, doctest::Approx(exact_a_cross_b_z).epsilon(roundoff));

    const typename T::value_type a_crossproj_b = a.crossProj(b);
    const typename T::value_type exact_a_crossproj_b = a_cross_b.euclidean();
    CHECK_EQ(a_crossproj_b, doctest::Approx(exact_a_crossproj_b).epsilon(roundoff));
}

TEST_CASE_TEMPLATE("ComplexArithmetic", T, Vec3c, Vec3z) {
    constexpr T a(
        typename T::value_type(9.573, 364.26), typename T::value_type(-423.4, 17.44),
        typename T::value_type(23.635, 0.464)
    );
    constexpr T b(
        typename T::value_type(1.545, -5.35), typename T::value_type(49.234, 39.23),
        typename T::value_type(-0.564, -4.65)
    );

    const T a_conj = a.conjugated();
    CHECK_EQ(a_conj.x.real(), a.x.real());
    CHECK_EQ(a_conj.x.imag(), -a.x.imag());
    CHECK_EQ(a_conj.y.real(), a.y.real());
    CHECK_EQ(a_conj.y.imag(), -a.y.imag());
    CHECK_EQ(a_conj.z.real(), a.z.real());
    CHECK_EQ(a_conj.z.imag(), -a.z.imag());

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
    CHECK_EQ(
        result.z.real(),
        doctest::Approx((a.z * fac_1 + b.z / den - fac_2 * a.z).real()).epsilon(1E-12)
    );
    CHECK_EQ(
        result.z.imag(),
        doctest::Approx((a.z * fac_1 + b.z / den - fac_2 * a.z).imag()).epsilon(1E-12)
    );

    const typename T::value_type::value_type a_euclidean = a.euclidean();
    const typename T::value_type::value_type exact_a_euclidean =
        std::sqrt(std::norm(a.x) + std::norm(a.y) + std::norm(a.z));
    CHECK_EQ(a_euclidean, doctest::Approx(exact_a_euclidean).epsilon(1E-12));

    const typename T::value_type a_dot_b = a.dot(b);
    const typename T::value_type exact_a_dot_b = a.x * b.x + a.y * b.y + a.z * b.z;
    CHECK_EQ(a_dot_b.real(), doctest::Approx(exact_a_dot_b.real()).epsilon(1E-6));
    CHECK_EQ(a_dot_b.imag(), doctest::Approx(exact_a_dot_b.imag()).epsilon(1E-6));

    const T a_cross_b = a.cross(b);
    const typename T::value_type exact_a_cross_b_x = a.y * b.z - a.z * b.y;
    const typename T::value_type exact_a_cross_b_y = a.z * b.x - a.x * b.z;
    const typename T::value_type exact_a_cross_b_z = a.x * b.y - a.y * b.x;
    CHECK_EQ(a_cross_b.x.real(), doctest::Approx(exact_a_cross_b_x.real()).epsilon(1E-6));
    CHECK_EQ(a_cross_b.x.imag(), doctest::Approx(exact_a_cross_b_x.imag()).epsilon(1E-6));
    CHECK_EQ(a_cross_b.y.real(), doctest::Approx(exact_a_cross_b_y.real()).epsilon(1E-6));
    CHECK_EQ(a_cross_b.y.imag(), doctest::Approx(exact_a_cross_b_y.imag()).epsilon(1E-6));
    CHECK_EQ(a_cross_b.z.real(), doctest::Approx(exact_a_cross_b_z.real()).epsilon(1E-6));
    CHECK_EQ(a_cross_b.z.imag(), doctest::Approx(exact_a_cross_b_z.imag()).epsilon(1E-6));

    const typename T::value_type a_crossproj_b = a.crossProj(b);
    const typename T::value_type exact_a_crossproj_b = a_cross_b.euclidean();
    CHECK_EQ(a_crossproj_b.real(), doctest::Approx(exact_a_crossproj_b.real()).epsilon(1E-6));
    CHECK_EQ(a_crossproj_b.imag(), doctest::Approx(exact_a_crossproj_b.imag()).epsilon(1E-6));
}

TEST_CASE("OstreamOperator") {
    const Vec3d a(1274.12, 4454.23, 1289.24);

    std::ostringstream oss;
    oss << a;

    CHECK_EQ(oss.str(), "[1274.12, 4454.23, 1289.24]");
}

TEST_CASE("Format") {
    constexpr Vec3d a(1274.12, 4454.23, -23.5745);

    CHECK_EQ(std::format("{}", a), "[1274.12, 4454.23, -23.5745]");
    CHECK_EQ(std::format("{0}", a), "[1274.12, 4454.23, -23.5745]");
    CHECK_EQ(std::format("{0:.2f}", a), "[1274.12, 4454.23, -23.57]");
    CHECK_EQ(std::format("{0:12}", a), std::format("[{:12}, {:12}, {:12}]", a.x, a.y, a.z));
    CHECK_EQ(
        std::format("{0:>12.1e}", a),
        std::format("[{:>12.1e}, {:>12.1e}, {:>12.1e}]", a.x, a.y, a.z)
    );

    const Vec3c c({1.0F, 2.0F}, {3.0F, -4.0F}, {0.0F, 0.5F});
    CHECK_EQ(std::format("{}", c), "[(1+2i), (3-4i), (0+0.5i)]");
    CHECK_EQ(std::format("{:.1f}", c), "[(1.0+2.0i), (3.0-4.0i), (0.0+0.5i)]");
}

TEST_CASE_TEMPLATE("Serialization", T, Vec3s, Vec3d, Vec3c, Vec3z) {
    constexpr T a(1274.12, 4454.23, 1289.24);

    auto [data, out] = zpp::bits::data_out();
    out(a).or_throw();

    T b;
    auto in = zpp::bits::in(data);
    in(b).or_throw();

    CHECK_EQ(a.x, b.x);
    CHECK_EQ(a.y, b.y);
}

TEST_CASE("squeeze zips coordinate ranges of distinct iterator types up to the shortest one") {
    const std::vector<double> xs{1.0, 2.0, 3.0};
    const std::array<float, 3> ys{4.0F, 5.0F, 6.0F};
    const std::array<float, 2> zs{7.0F, 8.0F};
    std::array<Vec3d, 3> out{};

    const auto [it1, it2, it3, res] =
        squeeze(xs.begin(), xs.end(), ys.begin(), ys.end(), zs.begin(), zs.end(), out.begin());

    CHECK_EQ(it1, xs.begin() + 2);
    CHECK_EQ(it2, ys.begin() + 2);
    CHECK_EQ(it3, zs.end());
    CHECK_EQ(res, out.begin() + 2);
    CHECK_EQ(out[0], Vec3d(1.0, 4.0, 7.0));
    CHECK_EQ(out[1], Vec3d(2.0, 5.0, 8.0));
}

TEST_CASE("Vec3 offers the size, fill, range and matrix constructors of a static Vector") {
    using V = Vec3d;

    SUBCASE("size and fill constructors") {
        const V a(std::size_t{3});
        CHECK_EQ(a.x, 0.0);
        CHECK_EQ(a.y, 0.0);
        CHECK_EQ(a.z, 0.0);
        const V b(std::size_t{3}, 7.5);
        CHECK_EQ(b.x, 7.5);
        CHECK_EQ(b.y, 7.5);
        CHECK_EQ(b.z, 7.5);
        const V c(std::size_t{3}, typename V::allocator_type{});
        CHECK_EQ(c.x, 0.0);
        CHECK_EQ(c.y, 0.0);
        CHECK_EQ(c.z, 0.0);
    }

    SUBCASE("range constructor") {
        const std::array<double, 3> arr{1.5, -2.0, 4.25};
        const V a(arr);
        CHECK_EQ(a.x, 1.5);
        CHECK_EQ(a.y, -2.0);
        CHECK_EQ(a.z, 4.25);
        const std::vector<float> vec{1.5F, -2.0F, 4.25F};
        const V b(vec);
        CHECK_EQ(b.x, 1.5);
        CHECK_EQ(b.y, -2.0);
        CHECK_EQ(b.z, 4.25);
    }

    SUBCASE("row and column matrices") {
        const std::array<double, 3> buf{1.5, -2.0, 4.25};
        const V row(std::mdspan(buf.data(), 1, 3));
        CHECK_EQ(row.x, 1.5);
        CHECK_EQ(row.y, -2.0);
        CHECK_EQ(row.z, 4.25);
        const V col(std::mdspan(buf.data(), 3, 1));
        CHECK_EQ(col.x, 1.5);
        CHECK_EQ(col.y, -2.0);
        CHECK_EQ(col.z, 4.25);
    }

    SUBCASE("scalar broadcast and per-component construction keep their meaning") {
        static_assert(std::is_same_v<decltype(V(1)), V>);
        const V a(1);
        CHECK_EQ(a.x, 1.0);
        CHECK_EQ(a.y, 1.0);
        CHECK_EQ(a.z, 1.0);
        const Vec3<std::int64_t> b(94, -37, 42);
        CHECK_EQ(b.x, 94);
        CHECK_EQ(b.y, -37);
    }
}

} // namespace basjoo::math
