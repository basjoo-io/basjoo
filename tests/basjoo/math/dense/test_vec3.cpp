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
#include <complex>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <format>
#include <mdspan>
#include <numbers>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include "zpp_bits.h"

#include "basjoo/math/dense/matrix.hpp"
#include "basjoo/math/dense/vector.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::math {

TEST_CASE_TEMPLATE(
    "Constructor", T, Vec3<std::int32_t>, Vec3<std::int64_t>, Vec3s, Vec3d, Vec3s, Vec3z
) {
    const T a(0.0, 0.0, 0.0);
    CHECK_EQ(a.x(), static_cast<typename T::value_type>(0.0));
    CHECK_EQ(a.y(), static_cast<typename T::value_type>(0.0));
    CHECK_EQ(a.z(), static_cast<typename T::value_type>(0.0));

    const T b(4.0);
    CHECK_EQ(b.x(), static_cast<typename T::value_type>(4.0));
    CHECK_EQ(b.y(), static_cast<typename T::value_type>(4.0));
    CHECK_EQ(b.z(), static_cast<typename T::value_type>(4.0));

    const T c(std::make_tuple(1.0, 6.0, 9.0));
    CHECK_EQ(c.x(), static_cast<typename T::value_type>(1.0));
    CHECK_EQ(c.y(), static_cast<typename T::value_type>(6.0));
    CHECK_EQ(c.z(), static_cast<typename T::value_type>(9.0));

    const T d(c);
    CHECK_EQ(d.x(), c.x());
    CHECK_EQ(d.y(), c.y());
    CHECK_EQ(d.z(), c.z());
}

TEST_CASE_TEMPLATE("Basic", T, Vec3s, Vec3d) {
    const T a(1.0, std::sqrt(3.0), std::sqrt(5.0));
    CHECK_EQ(a.euclidean(), doctest::Approx(3.0).epsilon(2E-7));
    CHECK_EQ(std::hypot(a), doctest::Approx(3.0).epsilon(2E-7));
    CHECK_EQ(a[0], a.x());
    CHECK_EQ(a[1], a.y());
    CHECK_EQ(a[2], a.z());

    const T b = a.normalized();
    CHECK_EQ(b.x(), doctest::Approx(1.0 / 3.0).epsilon(2E-7));
    CHECK_EQ(b.y(), doctest::Approx(std::sqrt(3.0) / 3.0).epsilon(2E-7));
    CHECK_EQ(b.z(), doctest::Approx(std::sqrt(5.0) / 3.0).epsilon(2E-7));
}

TEST_CASE_TEMPLATE("IntegralArithmetic", T, Vec3<std::int32_t>, Vec3<std::int64_t>) {
    constexpr T a(94, -37, 42);
    constexpr T b(-18, 27, 75);

    constexpr auto fac_1(10);
    constexpr auto fac_2(4);
    constexpr auto den(5);

    const T result = (a * fac_1) + (b / den) - (fac_2 * a);

    CHECK_EQ(result.x(), a.x() * fac_1 + b.x() / den - fac_2 * a.x());
    CHECK_EQ(result.y(), a.y() * fac_1 + b.y() / den - fac_2 * a.y());
    CHECK_EQ(result.z(), a.z() * fac_1 + b.z() / den - fac_2 * a.z());

    const typename T::value_type a_euclidean = a.euclidean();
    const typename T::value_type exact_a_euclidean =
        std::sqrt(std::norm(a.x()) + std::norm(a.y()) + std::norm(a.z()));
    CHECK_EQ(a_euclidean, exact_a_euclidean);

    const typename T::value_type a_dot_b = a.dot(b);
    const typename T::value_type exact_a_dot_b = a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
    CHECK_EQ(a_dot_b, exact_a_dot_b);

    const T neg = -a;
    CHECK_EQ(neg.x(), -a.x());
    CHECK_EQ(neg.y(), -a.y());
    CHECK_EQ(neg.z(), -a.z());
}

TEST_CASE_TEMPLATE("FloatingArithmetic", T, Vec3s, Vec3d) {
    constexpr T a(1.0, 0.0, 2.13);
    constexpr T b(0.5, std::numbers::sqrt3 * 0.5, 1.45);

    constexpr auto fac_1(10.5634);
    constexpr auto fac_2(0.67);
    constexpr auto den(5.34);

    const T result = (a * fac_1) + (b / den) - (fac_2 * a);

    CHECK_EQ(
        result.x(), doctest::Approx(a.x() * fac_1 + b.x() / den - fac_2 * a.x()).epsilon(1E-8)
    );
    CHECK_EQ(
        result.y(), doctest::Approx(a.y() * fac_1 + b.y() / den - fac_2 * a.y()).epsilon(1E-8)
    );
    CHECK_EQ(
        result.z(), doctest::Approx(a.z() * fac_1 + b.z() / den - fac_2 * a.z()).epsilon(1E-6)
    );

    const typename T::value_type a_euclidean = a.euclidean();
    const typename T::value_type exact_a_euclidean =
        std::sqrt(std::norm(a.x()) + std::norm(a.y()) + std::norm(a.z()));
    CHECK_EQ(a_euclidean, doctest::Approx(exact_a_euclidean).epsilon(1E-12));

    const typename T::value_type a_dot_b = a.dot(b);
    const typename T::value_type exact_a_dot_b = a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
    CHECK_EQ(a_dot_b, exact_a_dot_b);

    // Separately inlined copies of the same expression may be fused into FMAs differently.
    constexpr double roundoff{std::same_as<typename T::value_type, float> ? 1E-6 : 1E-12};

    const T a_cross_b = a.cross(b);
    const typename T::value_type exact_a_cross_b_x = a.y() * b.z() - a.z() * b.y();
    const typename T::value_type exact_a_cross_b_y = a.z() * b.x() - a.x() * b.z();
    const typename T::value_type exact_a_cross_b_z = a.x() * b.y() - a.y() * b.x();
    CHECK_EQ(a_cross_b.x(), doctest::Approx(exact_a_cross_b_x).epsilon(roundoff));
    CHECK_EQ(a_cross_b.y(), doctest::Approx(exact_a_cross_b_y).epsilon(roundoff));
    CHECK_EQ(a_cross_b.z(), doctest::Approx(exact_a_cross_b_z).epsilon(roundoff));

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
    CHECK_EQ(a_conj.x().real(), a.x().real());
    CHECK_EQ(a_conj.x().imag(), -a.x().imag());
    CHECK_EQ(a_conj.y().real(), a.y().real());
    CHECK_EQ(a_conj.y().imag(), -a.y().imag());
    CHECK_EQ(a_conj.z().real(), a.z().real());
    CHECK_EQ(a_conj.z().imag(), -a.z().imag());

    constexpr auto fac_1{typename T::value_type(10.5634, -3.65)};
    constexpr auto fac_2{typename T::value_type(0.67, -0.38)};
    constexpr auto den{typename T::value_type(5.34, 7.35)};

    const T result = (a * fac_1) + (b / den) - (fac_2 * a);

    CHECK_EQ(
        result.x().real(),
        doctest::Approx((a.x() * fac_1 + b.x() / den - fac_2 * a.x()).real()).epsilon(1E-12)
    );
    CHECK_EQ(
        result.x().imag(),
        doctest::Approx((a.x() * fac_1 + b.x() / den - fac_2 * a.x()).imag()).epsilon(1E-12)
    );
    CHECK_EQ(
        result.y().real(),
        doctest::Approx((a.y() * fac_1 + b.y() / den - fac_2 * a.y()).real()).epsilon(1E-12)
    );
    CHECK_EQ(
        result.y().imag(),
        doctest::Approx((a.y() * fac_1 + b.y() / den - fac_2 * a.y()).imag()).epsilon(1E-12)
    );
    CHECK_EQ(
        result.z().real(),
        doctest::Approx((a.z() * fac_1 + b.z() / den - fac_2 * a.z()).real()).epsilon(1E-12)
    );
    CHECK_EQ(
        result.z().imag(),
        doctest::Approx((a.z() * fac_1 + b.z() / den - fac_2 * a.z()).imag()).epsilon(1E-12)
    );

    const typename T::value_type::value_type a_euclidean = a.euclidean();
    const typename T::value_type::value_type exact_a_euclidean =
        std::sqrt(std::norm(a.x()) + std::norm(a.y()) + std::norm(a.z()));
    CHECK_EQ(a_euclidean, doctest::Approx(exact_a_euclidean).epsilon(1E-12));

    const typename T::value_type a_dot_b = a.dot(b);
    const typename T::value_type exact_a_dot_b = a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
    CHECK_EQ(a_dot_b.real(), doctest::Approx(exact_a_dot_b.real()).epsilon(1E-6));
    CHECK_EQ(a_dot_b.imag(), doctest::Approx(exact_a_dot_b.imag()).epsilon(1E-6));

    const T a_cross_b = a.cross(b);
    const typename T::value_type exact_a_cross_b_x = a.y() * b.z() - a.z() * b.y();
    const typename T::value_type exact_a_cross_b_y = a.z() * b.x() - a.x() * b.z();
    const typename T::value_type exact_a_cross_b_z = a.x() * b.y() - a.y() * b.x();
    CHECK_EQ(a_cross_b.x().real(), doctest::Approx(exact_a_cross_b_x.real()).epsilon(1E-6));
    CHECK_EQ(a_cross_b.x().imag(), doctest::Approx(exact_a_cross_b_x.imag()).epsilon(1E-6));
    CHECK_EQ(a_cross_b.y().real(), doctest::Approx(exact_a_cross_b_y.real()).epsilon(1E-6));
    CHECK_EQ(a_cross_b.y().imag(), doctest::Approx(exact_a_cross_b_y.imag()).epsilon(1E-6));
    CHECK_EQ(a_cross_b.z().real(), doctest::Approx(exact_a_cross_b_z.real()).epsilon(1E-6));
    CHECK_EQ(a_cross_b.z().imag(), doctest::Approx(exact_a_cross_b_z.imag()).epsilon(1E-6));

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
    CHECK_EQ(std::format("{0:12}", a), std::format("[{:12}, {:12}, {:12}]", a.x(), a.y(), a.z()));
    CHECK_EQ(
        std::format("{0:>12.1e}", a),
        std::format("[{:>12.1e}, {:>12.1e}, {:>12.1e}]", a.x(), a.y(), a.z())
    );

    const Vec3c c({1.0F, 2.0F}, {3.0F, -4.0F}, {0.0F, 0.5F});
    CHECK_EQ(std::format("{}", c), "[(1+2i), (3-4i), (0+0.5i)]");
    CHECK_EQ(std::format("{:.1f}", c), "[(1.0+2.0i), (3.0-4.0i), (0.0+0.5i)]");
}

TEST_CASE_TEMPLATE("Serialization", T, Vec3s, Vec3d, Vec3c, Vec3z) {
    constexpr T a(1274.12, 4454.23, 1289.24);

    auto [data, out] = zpp::bits::data_out();
    out(a).or_throw();
    // All four slots, the padding included.
    CHECK_EQ(data.size(), 4 * sizeof(typename T::value_type));

    T b;
    auto in = zpp::bits::in(data);
    in(b).or_throw();

    CHECK_EQ(a.x(), b.x());
    CHECK_EQ(a.y(), b.y());
    CHECK_EQ(a.z(), b.z());
    CHECK_EQ(b[3], typename T::value_type(0.0));
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
        const V a(std::size_t{3}, typename V::allocator_type{});
        CHECK_EQ(a.x(), 0.0);
        CHECK_EQ(a.y(), 0.0);
        CHECK_EQ(a.z(), 0.0);
        const V b(std::size_t{3}, 7.5, typename V::allocator_type{});
        CHECK_EQ(b.x(), 7.5);
        CHECK_EQ(b.y(), 7.5);
        CHECK_EQ(b.z(), 7.5);
    }

    SUBCASE("bare scalars are element values, integral ones included") {
        static_assert(!std::is_convertible_v<double, V>);
        CHECK_EQ(V(std::size_t{3}), V(3.0, 3.0, 3.0));
        CHECK_EQ(V(std::size_t{3}, 7.5, -1.0), V(3.0, 7.5, -1.0));

        using U = Vec3<std::size_t>;
        CHECK_EQ(U(std::size_t{3}), U(3, 3, 3));
        CHECK_EQ(U(std::size_t{3}, typename U::allocator_type{}), U(0, 0, 0));
    }

    SUBCASE("range constructor") {
        const std::array<double, 3> arr{1.5, -2.0, 4.25};
        const V a(arr);
        CHECK_EQ(a.x(), 1.5);
        CHECK_EQ(a.y(), -2.0);
        CHECK_EQ(a.z(), 4.25);
        const std::vector<float> vec{1.5F, -2.0F, 4.25F};
        const V b(vec);
        CHECK_EQ(b.x(), 1.5);
        CHECK_EQ(b.y(), -2.0);
        CHECK_EQ(b.z(), 4.25);
    }

    SUBCASE("row and column matrices") {
        const std::array<double, 3> buf{1.5, -2.0, 4.25};
        const V row(std::mdspan(buf.data(), 1, 3));
        CHECK_EQ(row.x(), 1.5);
        CHECK_EQ(row.y(), -2.0);
        CHECK_EQ(row.z(), 4.25);
        const V col(std::mdspan(buf.data(), 3, 1));
        CHECK_EQ(col.x(), 1.5);
        CHECK_EQ(col.y(), -2.0);
        CHECK_EQ(col.z(), 4.25);
    }

    SUBCASE("scalar broadcast and per-component construction keep their meaning") {
        static_assert(std::is_same_v<decltype(V(1)), V>);
        const V a(1);
        CHECK_EQ(a.x(), 1.0);
        CHECK_EQ(a.y(), 1.0);
        CHECK_EQ(a.z(), 1.0);
        const Vec3<std::int64_t> b(94, -37, 42);
        CHECK_EQ(b.x(), 94);
        CHECK_EQ(b.y(), -37);
    }
}

TEST_CASE_TEMPLATE(
    "Vec3 stores its three elements and a zero padding in four slots", T, Vec3<std::int32_t>, Vec3d,
    Vec3z
) {
    using value_type = typename T::value_type;
    static_assert(T::size() == 4 && T::extent(0) == 3);
    static_assert(std::is_same_v<decltype(T::get_allocator()), typename T::allocator_type>);
    static_assert(sizeof(T) == T::size() * sizeof(value_type));
    const value_type zero(0.0);

    SUBCASE("every constructor and fill leave the padding zero") {
        const T a;
        CHECK_EQ(a[3], zero);
        CHECK_EQ(T(1.0, 2.0, 3.0)[3], zero);
        CHECK_EQ(T(5.0)[3], zero);
        CHECK_EQ(T(std::size_t{3}, value_type(7.0), typename T::allocator_type{})[3], zero);
        CHECK_EQ(
            T(std::array<value_type, 3>{value_type(1.0), value_type(2.0), value_type(3.0)})[3], zero
        );
        T b(1.0, 2.0, 3.0);
        b.fill(value_type(2.0));
        CHECK_EQ(b[3], zero);
        CHECK_EQ(b.coeff(3), zero);
    }

    SUBCASE("arithmetic keeps the padding zero") {
        const T a(1.0, 2.0, 3.0);
        const T b(4.0, -5.0, 6.0);
        CHECK_EQ((a + b)[3], zero);
        CHECK_EQ((a - b)[3], zero);
        CHECK_EQ((-a)[3], zero);
        CHECK_EQ((a * value_type(3.0))[3], zero);
    }

    SUBCASE("the accessors are references into the storage") {
        T a(1.0, 2.0, 3.0);
        a.x() = value_type(4.0);
        a.z() += value_type(1.0);
        CHECK_EQ(a, T(4.0, 2.0, 4.0));
        CHECK_EQ(&a.y(), a.data() + 1);
        CHECK_EQ(&std::as_const(a).z(), a.data() + 2);
    }

    SUBCASE("views and the dimension cover the three elements only") {
        const T a(1.0, 2.0, 3.0);
        CHECK_EQ(std::span<const value_type>(a).size(), 3);
        CHECK_EQ(a.subspan(1).size(), 2);
    }

    SUBCASE("the padding is not writable through updateCoeff") {
        T a(1.0, 2.0, 3.0);
        CHECK_NOTHROW(a.updateCoeff(2, value_type(9.0)));
        CHECK_EQ(a.z(), value_type(9.0));
#if BASJOO_CHECK_PARAMS == 1
        CHECK_THROWS_AS(a.updateCoeff(3, value_type(1.0)), std::out_of_range);
        CHECK_THROWS_AS(static_cast<void>(a.coeff(4)), std::out_of_range);
#endif
        CHECK_EQ(a[3], zero);
    }
}

// Array storage lets the element-wise arithmetic and the accessors run in constant expressions.
consteval auto constantVec3() -> bool {
    Vec3d v{1.0, 2.0, 3.0};
    v += Vec3d{1.0, 1.0, 1.0};
    v *= 2.0;
    v.z() -= 1.0;
    return v.x() == 4.0 && v[2] == 7.0 && v[3] == 0.0 && -v == v * -1.0 &&
           v.euclideanSqr() == 101.0;
}
static_assert(constantVec3());

// The products and norms of the constant evaluation path, v^T M with a Mat33 included.
consteval auto constantVec3Products() -> bool {
    const Vec3d a{3.0, -1.0, 2.0};
    const Mat33d m{1.0, 2.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 2.0};
    const Vec3d r{a.dot(m)};
    return a.dot(Vec3d{2.0, 4.0, 1.0}) == 4.0 && a.euclideanSqr() == 14.0 && r.x() == 5.0 &&
           r.y() == 5.0 && r.z() == 3.0 && r[3] == 0.0;
}
static_assert(constantVec3Products());

// clang-format off
TEST_CASE_TEMPLATE(
    "Vec3 norms and products agree with their definitions for every scalar type", T,
    Vec3s, Vec3d, Vec3c, Vec3z, Vec3<std::int32_t>, Vec3<std::uint32_t>, Vec3<std::int64_t>,
    Vec3<std::uint64_t>, Vec3<std::int16_t>, Vec3<long double>
) {
    // clang-format on
    using V = typename T::value_type;
    // Small non-negative parts, exact in every type, unsigned ones included.
    const auto make = [](int re, int im) -> V {
        if constexpr (isComplexArithmeticV<V>) {
            return V(
                static_cast<typename V::value_type>(re), static_cast<typename V::value_type>(im)
            );
        } else {
            return static_cast<V>(re);
        }
    };
    const auto check_close = [](V got, V want) -> void {
        if constexpr (isComplexArithmeticV<V>) {
            CHECK_EQ(got.real(), doctest::Approx(want.real()).epsilon(1E-6));
            CHECK_EQ(got.imag(), doctest::Approx(want.imag()).epsilon(1E-6));
        } else if constexpr (std::is_floating_point_v<V>) {
            CHECK_EQ(
                static_cast<double>(got), doctest::Approx(static_cast<double>(want)).epsilon(1E-6)
            );
        } else {
            CHECK_EQ(got, want);
        }
    };
    const T a{make(3, 1), make(2, 4), make(1, 3)};
    const T b{make(1, 2), make(5, 1), make(4, 2)};

    SUBCASE("euclideanSqr, euclidean and dot") {
        if constexpr (isComplexArithmeticV<V>) {
            const auto sum{std::norm(a.x()) + std::norm(a.y()) + std::norm(a.z())};
            CHECK_EQ(a.euclideanSqr(), doctest::Approx(sum));
            CHECK_EQ(a.euclidean(), doctest::Approx(std::sqrt(sum)));
        } else {
            check_close(
                a.euclideanSqr(), static_cast<V>(a.x() * a.x() + a.y() * a.y() + a.z() * a.z())
            );
        }
        check_close(a.dot(b), static_cast<V>(a.x() * b.x() + a.y() * b.y() + a.z() * b.z()));
    }

    SUBCASE("the product with a matrix of three rows, under both layouts and every width") {
        const auto check = [&]<std::size_t N, typename Layout>() -> void {
            Matrix<V, std::size_t, 3, N, Layout> m{};
            for (std::size_t r{0}; r < 3; ++r) {
                for (std::size_t c{0}; c < N; ++c) {
                    m[r, c] = make(static_cast<int>(r + 2 * c + 1), static_cast<int>(c + r));
                }
            }
            const auto product{a.dot(m)};
            for (std::size_t c{0}; c < N; ++c) {
                check_close(
                    product[c], static_cast<V>(a.x() * m[0, c] + a.y() * m[1, c] + a.z() * m[2, c])
                );
            }
            if constexpr (N == 3) {
                // A Vec3 result keeps its padding zero.
                CHECK_EQ(product.data()[3], V(0));
            }
        };
        check.template operator()<1, std::layout_left>();
        check.template operator()<2, std::layout_left>();
        check.template operator()<3, std::layout_left>();
        check.template operator()<4, std::layout_left>();
        check.template operator()<5, std::layout_left>();
        check.template operator()<7, std::layout_left>();
        check.template operator()<1, std::layout_right>();
        check.template operator()<2, std::layout_right>();
        check.template operator()<3, std::layout_right>();
        check.template operator()<4, std::layout_right>();
        check.template operator()<5, std::layout_right>();
        check.template operator()<7, std::layout_right>();
    }

    SUBCASE("orthogonalTo takes an exact zero for integers and conjugates complex vectors") {
        const T e0{make(2, 0), make(0, 0), make(0, 0)};
        const T e2{make(0, 0), make(0, 0), make(3, 0)};
        CHECK(e0.orthogonalTo(e2));
        CHECK_FALSE(e2.orthogonalTo(e2));
        CHECK_FALSE(a.orthogonalTo(b));
        if constexpr (isComplexArithmeticV<V>) {
            // (1, i, 0) has the conjugated inner product 2 with itself, where the bilinear one
            // vanishes, and is orthogonal to (1, -i, 1).
            const T u{V(1, 0), V(0, 1), V(0, 0)};
            CHECK_FALSE(u.orthogonalTo(u));
            CHECK(u.orthogonalTo(T{V(1, 0), V(0, -1), V(1, 0)}));
        }
    }
}

} // namespace basjoo::math
