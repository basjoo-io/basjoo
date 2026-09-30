/**
 * @file math/dense/test_mat22.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2026-11-03
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/dense/mat22.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <mdspan>
#include <numbers>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>

#include "zpp_bits.h"

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_formatter.hpp"
#include "basjoo/math/dense/matrix.hpp"
#include "basjoo/math/dense/vector.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::math {

namespace {

template <typename T>
constexpr auto kTol{
    std::is_same_v<detail::DenseNormTraitT<typename T::value_type>, float> ? 1E-5 : 1E-12
};

// The reference product.
template <typename T>
auto product(const T& lhs, const T& rhs) -> T {
    T result(2, 2, typename T::value_type(0.0));
    for (std::size_t i{0}; i < 2; ++i) {
        for (std::size_t j{0}; j < 2; ++j) {
            for (std::size_t k{0}; k < 2; ++k) {
                result[i, j] += lhs[i, k] * rhs[k, j];
            }
        }
    }
    return result;
}

template <typename T>
auto sample() -> T {
    using V = typename T::value_type;
    if constexpr (ComplexArithmetic<V>) {
        return T(V(1.0, -0.5), V(-2.0, 1.5), V(3.5, 0.25), V(4.0, -3.0));
    } else {
        return T(V(1.0), V(-2.0), V(3.5), V(4.0));
    }
}

// A value with an imaginary part when V is complex.
template <typename V>
auto value(double re, double im) -> V {
    if constexpr (ComplexArithmetic<V>) {
        return V(re, im);
    } else {
        return V(re);
    }
}

// Exact for integers, within kTol relative to max(|expected|, 1) otherwise.
template <typename T, typename V>
auto checkClose(const V& actual, const V& expected) -> void {
    if constexpr (std::is_integral_v<V>) {
        CHECK_EQ(actual, expected);
    } else {
        CHECK_LE(
            static_cast<double>(std::abs(actual - expected)),
            kTol<T> * std::max(1.0, static_cast<double>(std::abs(expected)))
        );
    }
}

} // namespace

TEST_CASE_TEMPLATE(
    "Mat22 constructors take the elements in reading order under either layout", T,
    Mat22<std::int64_t>, Mat22s, Mat22d, Mat22c, Mat22z,
    Mat22<std::int64_t, std::size_t, std::layout_right>,
    Mat22<float, std::size_t, std::layout_right>, Mat22<double, std::size_t, std::layout_right>,
    Mat22<std::complex<float>, std::size_t, std::layout_right>,
    Mat22<std::complex<double>, std::size_t, std::layout_right>
) {
    using V = typename T::value_type;

    const T m(V(1.0), V(2.0), V(3.0), V(4.0));
    CHECK_EQ(m[0, 0], V(1.0));
    CHECK_EQ(m[0, 1], V(2.0));
    CHECK_EQ(m[1, 0], V(3.0));
    CHECK_EQ(m[1, 1], V(4.0));
    // The columns are stored contiguously under layout_left, the rows under layout_right.
    CHECK_EQ(
        m.data()[1], std::is_same_v<typename T::layout_type, std::layout_left> ? V(3.0) : V(2.0)
    );

    const T zero(V(0.0), V(0.0), V(0.0), V(0.0));
    CHECK_EQ(T{}, zero);
    CHECK_EQ(T(typename T::allocator_type{}), zero);
    CHECK_EQ(T(2, 2), zero);
    CHECK_EQ(T(2, 2, V(7.0)), T(V(7.0), V(7.0), V(7.0), V(7.0)));
    CHECK_EQ(T::identity(), T(V(1.0), V(0.0), V(0.0), V(1.0)));

    const std::array<V, 4> raw{V(1.0), V(2.0), V(3.0), V(4.0)};
    const std::mdspan<const V, std::extents<std::size_t, 2, 2>, std::layout_right> view{raw.data()};
    CHECK_EQ(T(view), m);

    using Vec = Vector<V, std::size_t, 2>;
    CHECK_EQ(T::fromColumns(Vec(V(1.0), V(3.0)), Vec(V(2.0), V(4.0))), m);
    CHECK_EQ(T::fromRows(Vec(V(1.0), V(2.0)), Vec(V(3.0), V(4.0))), m);

    T n;
    n.fill(V(5.0));
    CHECK_EQ(n, T(2, 2, V(5.0)));
    n.assign(2, 2, V(6.0));
    CHECK_EQ(n, T(2, 2, V(6.0)));
    n.setIdentity();
    CHECK_EQ(n, T::identity());
    n.updateCoeff(0, 1, V(9.0));
    CHECK_EQ(n.coeff(0, 1), V(9.0));

#if BASJOO_CHECK_PARAMS == 1
    CHECK_THROWS_AS(T(3, 2), std::invalid_argument);
    CHECK_THROWS_AS(n.resize(2, 3), std::invalid_argument);
    CHECK_THROWS_AS(std::ignore = n.coeff(2, 0), std::out_of_range);
#endif
}

TEST_CASE("Mat22 converts between scalar types") {
    const Mat22s a(1.5F, -2.0F, 0.25F, 4.0F);
    const Mat22d b(a);
    CHECK_EQ(b, Mat22d(1.5, -2.0, 0.25, 4.0));
}

TEST_CASE_TEMPLATE(
    "Mat22 is a static, exhaustive 2x2 matrix", T, Mat22d, Mat22z,
    Mat22<double, std::size_t, std::layout_right>, Mat22s
) {
    using V = typename T::value_type;

    static_assert(MatArithmetic<T>);
    static_assert(!VecArithmetic<T>);
    static_assert(Mat22Arithmetic<T>);
    static_assert(!Mat22Arithmetic<Vector<V, std::size_t, 2>> && !Mat22Arithmetic<V>);
    static_assert(std::is_trivially_copyable_v<T>);
    static_assert(sizeof(T) == 4 * sizeof(V) && alignof(T) == 4 * sizeof(V));
    static_assert(T::rank() == 2 && T::rank_dynamic() == 0);
    static_assert(T::static_extent(0) == 2 && T::static_extent(1) == 2);
    static_assert(T::nrows() == 2 && T::ncols() == 2 && T::size() == 4);
    static_assert(T::is_always_exhaustive() && T::is_always_unique() && T::is_always_strided());
    static_assert(!T::empty());
    static_assert(std::is_same_v<decltype(T::get_allocator()), typename T::allocator_type>);
    if constexpr (std::is_same_v<typename T::layout_type, std::layout_left>) {
        static_assert(T::stride(0) == 1 && T::stride(1) == 2);
    } else {
        static_assert(T::stride(0) == 2 && T::stride(1) == 1);
    }

    T m{sample<T>()};
    const std::mdspan<V, typename T::extents_type, typename T::layout_type> view{m};
    const std::mdspan<const V, std::dextents<std::size_t, 2>, std::layout_stride> strided{
        std::as_const(m)
    };
    for (std::size_t i{0}; i < 2; ++i) {
        for (std::size_t j{0}; j < 2; ++j) {
            CHECK_EQ(view[i, j], m[i, j]);
            CHECK_EQ(strided[i, j], m[i, j]);
        }
    }
    view[1, 0] = V(8.0);
    CHECK_EQ(m[1, 0], V(8.0));

    const auto row{m.submdspan({1, 2}, {0, 2})};
    CHECK_EQ(row.extent(0), std::size_t{1});
    CHECK_EQ(row.extent(1), std::size_t{2});
    CHECK_EQ(row[0, 0], m[1, 0]);
    CHECK_EQ(row[0, 1], m[1, 1]);
}

TEST_CASE_TEMPLATE(
    "Mat22 offers single-index access at the storage offsets", T, Mat22d, Mat22c,
    Mat22<double, std::size_t, std::layout_right>
) {
    using V = typename T::value_type;

    T m{sample<T>()};
    for (std::size_t k{0}; k < T::size(); ++k) {
        CHECK_EQ(&m[k], m.data() + k);
        CHECK_EQ(m.coeff(k), m.data()[k]);
    }
    // Element (0, 1) sits at 0 + 2 * 1 under layout_left and at 2 * 0 + 1 under layout_right.
    const std::size_t offset{std::is_same_v<typename T::layout_type, std::layout_left> ? 2 : 1};
    CHECK_EQ(m[offset], m[0, 1]);
    m.updateCoeff(offset, V(7.0));
    CHECK_EQ(m[0, 1], V(7.0));
#if BASJOO_CHECK_PARAMS == 1
    CHECK_THROWS_AS(static_cast<void>(m.coeff(4)), std::out_of_range);
    CHECK_THROWS_AS(m.updateCoeff(4, V(1.0)), std::out_of_range);
    CHECK_THROWS_AS(static_cast<void>(m.submdspan({0, 2})), std::invalid_argument);
#else
    const auto row{m.submdspan({0, 2})};
    CHECK_EQ(row.extent(0), std::size_t{1});
    CHECK_EQ(row.extent(1), std::size_t{2});
    CHECK_EQ(row[0, 1], m[0, 1]);
#endif
}

TEST_CASE_TEMPLATE(
    "Mat22 arithmetic is element-wise", T, Mat22<std::int64_t>, Mat22s, Mat22d, Mat22c, Mat22z,
    Mat22<double, std::size_t, std::layout_right>,
    Mat22<std::complex<float>, std::size_t, std::layout_right>
) {
    using V = typename T::value_type;

    const T a{sample<T>()};
    const T b(V(0.5), V(6.0), V(-1.0), V(2.0));
    const auto each = [&](auto op) -> T {
        T result;
        for (std::size_t i{0}; i < 2; ++i) {
            for (std::size_t j{0}; j < 2; ++j) {
                result[i, j] = op(a[i, j], b[i, j]);
            }
        }
        return result;
    };

    CHECK_EQ(a + b, each([](V x, V y) { return x + y; }));
    CHECK_EQ(a - b, each([](V x, V y) { return x - y; }));
    CHECK_EQ(a * 3, each([](V x, V) { return x * V(3.0); }));
    CHECK_EQ(3 * a, a * 3);
    CHECK_EQ(a / 2, each([](V x, V) { return x / V(2.0); }));
    CHECK_EQ(-a, each([](V x, V) { return -x; }));
    CHECK_FALSE(a == b);

    T c{a};
    c += b;
    c -= b;
    CHECK_EQ(c, a);

    // Rvalue operands are recycled in place.
    T d{a};
    T&& sum{std::move(d) + b};
    CHECK_EQ(&sum, &d);
    CHECK_EQ(sum, a + b);
    T e{a};
    T&& diff{b - std::move(e)};
    CHECK_EQ(&diff, &e);
    CHECK_EQ(diff, b - a);
    T f{a};
    T&& scaled{2 * std::move(f)};
    CHECK_EQ(&scaled, &f);
    CHECK_EQ(scaled, a * 2);

    if constexpr (ComplexArithmetic<V>) {
        const V z(0.75, -1.25);
        CHECK((a * z).identicalTo(each([z](V x, V) { return x * z; }), kTol<T>));
        CHECK((a / z).identicalTo(each([z](V x, V) { return x / z; }), kTol<T>));
        CHECK_EQ(std::conj(a), each([](V x, V) { return std::conj(x); }));
        CHECK_EQ(a.adjoint(), std::conj(a).transposed());
    } else {
        CHECK_EQ(a.conjugated(), a);
        CHECK_EQ(a.adjoint(), a.transposed());
    }
}

TEST_CASE_TEMPLATE(
    "Mat22 norms, trace, determinant and transpose", T, Mat22<std::int64_t>, Mat22s, Mat22d, Mat22c,
    Mat22z, Mat22<float, std::size_t, std::layout_right>,
    Mat22<std::complex<double>, std::size_t, std::layout_right>
) {
    using V = typename T::value_type;
    using R = detail::DenseNormTraitT<V>;

    const T a{sample<T>()};
    R norm_sqr(0.0);
    for (std::size_t i{0}; i < 2; ++i) {
        for (std::size_t j{0}; j < 2; ++j) {
            norm_sqr += std::norm(a[i, j]);
        }
    }
    if constexpr (std::is_integral_v<R>) {
        CHECK_EQ(a.euclideanSqr(), norm_sqr);
        CHECK_EQ(std::norm(a), norm_sqr);
    } else {
        CHECK_EQ(a.euclideanSqr(), doctest::Approx(norm_sqr).epsilon(kTol<T>));
        CHECK_EQ(a.euclidean(), doctest::Approx(std::sqrt(norm_sqr)).epsilon(kTol<T>));
        CHECK_EQ(std::abs(a), doctest::Approx(std::sqrt(norm_sqr)).epsilon(kTol<T>));
        CHECK(a.identicalTo(a + T(2, 2, V(1E-3 * kTol<T>)), kTol<T>));
        CHECK_FALSE(a.identicalTo(a + T(2, 2, V(1E-2)), kTol<T>));
    }

    CHECK_EQ(a.trace(), a[0, 0] + a[1, 1]);
    const V det{a[0, 0] * a[1, 1] - a[0, 1] * a[1, 0]};
    if constexpr (std::is_integral_v<V>) {
        CHECK_EQ(a.determinant(), det);
    } else {
        CHECK_LT(std::abs(a.determinant() - det), kTol<T> * std::abs(det));
    }

    const T t{a.transposed()};
    for (std::size_t i{0}; i < 2; ++i) {
        for (std::size_t j{0}; j < 2; ++j) {
            CHECK_EQ(t[i, j], a[j, i]);
        }
    }
    T u{a};
    u.selfTransposed().selfTransposed();
    CHECK_EQ(u, a);
}

TEST_CASE_TEMPLATE(
    "Mat22 inverse", T, Mat22s, Mat22d, Mat22c, Mat22z,
    Mat22<double, std::size_t, std::layout_right>,
    Mat22<std::complex<float>, std::size_t, std::layout_right>
) {
    const T a{sample<T>()};
    CHECK(product(a, a.inversed()).identicalTo(T::identity(), kTol<T>));
    CHECK(product(a.inversed(), a).identicalTo(T::identity(), kTol<T>));
    T b{a};
    T&& c{std::move(b).inversed()};
    CHECK_EQ(&c, &b);
    CHECK(c.inversed().identicalTo(a, kTol<T>));

#if BASJOO_CHECK_PARAMS == 1
    using V = typename T::value_type;
    T singular(V(1.0), V(2.0), V(2.0), V(4.0));
    CHECK_THROWS_AS(singular.selfInversed(), std::runtime_error);
#endif
}

TEST_CASE_TEMPLATE(
    "Mat22 rotations", T, Mat22s, Mat22d, Mat22<float, std::size_t, std::layout_right>,
    Mat22<double, std::size_t, std::layout_right>
) {
    using V = typename T::value_type;
    using Vec = Vector<V, std::size_t, 2>;

    const V theta{V(0.7)};
    const T r{T::fromRadian(theta)};
    CHECK_EQ(r, T(std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta)));
    CHECK_EQ(r.radian(), doctest::Approx(theta).epsilon(kTol<T>));
    CHECK(r.isRotation(kTol<T>));
    CHECK_EQ(T::fromRadian(V(-3.0)).radian(), doctest::Approx(-3.0).epsilon(kTol<T>));

    // Rotating a matrix turns each of its columns, as Vec2::rotate does.
    const T m(V(1.0), V(-2.0), V(3.5), V(4.0));
    const T turned{m.rotate(theta)};
    CHECK(turned.identicalTo(product(r, m), kTol<T>));
    for (std::size_t j{0}; j < 2; ++j) {
        const Vec column{Vec(m[0, j], m[1, j]).rotate(theta)};
        CHECK_EQ(turned[0, j], doctest::Approx(column.x()).epsilon(kTol<T>));
        CHECK_EQ(turned[1, j], doctest::Approx(column.y()).epsilon(kTol<T>));
    }
    CHECK(m.rotateHalfPi().identicalTo(m.rotate(std::numbers::pi_v<V> / 2), kTol<T>));
    T n{m};
    n.selfRotate(theta).selfRotate(-theta);
    CHECK(n.identicalTo(m, kTol<T>));

    CHECK_FALSE(m.isRotation(kTol<T>));
    CHECK_FALSE((r * V(1.01)).isRotation(kTol<T>));
    CHECK_FALSE(T(V(1.0), V(0.0), V(0.0), V(-1.0)).isRotation(kTol<T>));
}

TEST_CASE_TEMPLATE(
    "Mat22 products agree with the definition", T, Mat22<std::int64_t>, Mat22s, Mat22d, Mat22c,
    Mat22z, Mat22<std::int64_t, std::size_t, std::layout_right>,
    Mat22<float, std::size_t, std::layout_right>, Mat22<double, std::size_t, std::layout_right>,
    Mat22<std::complex<float>, std::size_t, std::layout_right>,
    Mat22<std::complex<double>, std::size_t, std::layout_right>
) {
    using V = typename T::value_type;
    using Vec = Vector<V, std::size_t, 2>;
    using Wide = Matrix<V, std::size_t, 2, 3, typename T::layout_type>;

    const T a{sample<T>()};
    const T b(value<V>(0.5, 1.0), value<V>(6.0, -0.5), value<V>(-1.0, 0.25), value<V>(2.0, 2.0));
    const Vec v(value<V>(1.5, -0.25), value<V>(-0.5, 0.75));
    Wide w(2, 3);
    for (std::size_t i{0}; i < 2; ++i) {
        for (std::size_t j{0}; j < 3; ++j) {
            w[i, j] = value<V>(static_cast<double>(3 * i + j) - 2.0, static_cast<double>(j) * 0.5);
        }
    }

    SUBCASE("matrix times matrix") {
        const T ab{a.dot(b)};
        const T expected{product(a, b)};
        for (std::size_t i{0}; i < 2; ++i) {
            for (std::size_t j{0}; j < 2; ++j) {
                checkClose<T>(ab[i, j], expected[i, j]);
            }
        }
        CHECK_EQ(T::identity().dot(a), a);
        CHECK_EQ(a.dot(T::identity()), a);
    }

    SUBCASE("matrix times vector and vector times matrix") {
        const Vec av{a.dot(v)};
        const Vec va{v.dot(a)};
        for (std::size_t i{0}; i < 2; ++i) {
            checkClose<T>(av[i], a[i, 0] * v[0] + a[i, 1] * v[1]);
            checkClose<T>(va[i], v[0] * a[0, i] + v[1] * a[1, i]);
        }
    }

    SUBCASE("products with a generic matrix of two rows") {
        const Wide aw{a.dot(w)};
        const auto vw{v.dot(w)};
        for (std::size_t j{0}; j < 3; ++j) {
            for (std::size_t i{0}; i < 2; ++i) {
                checkClose<T>(aw[i, j], a[i, 0] * w[0, j] + a[i, 1] * w[1, j]);
            }
            checkClose<T>(vw[j], v[0] * w[0, j] + v[1] * w[1, j]);
        }
    }
}

TEST_CASE_TEMPLATE(
    "Mat22 serializes four scalars in storage order", T, Mat22<std::int64_t>, Mat22s, Mat22d,
    Mat22c, Mat22z, Mat22<double, std::size_t, std::layout_right>
) {
    const T a{sample<T>()};

    auto [data, out] = zpp::bits::data_out();
    out(a).or_throw();
    CHECK_EQ(data.size(), 4 * sizeof(typename T::value_type));

    auto [raw, raw_out] = zpp::bits::data_out();
    raw_out(
        std::array<typename T::value_type, 4>{a.data()[0], a.data()[1], a.data()[2], a.data()[3]}
    )
        .or_throw();
    CHECK_EQ(data, raw);

    T b;
    auto in = zpp::bits::in(data);
    in(b).or_throw();
    CHECK_EQ(b, a);
}

// Array storage lets the element-wise arithmetic and the products run in constant expressions.
consteval auto constantMat22() -> bool {
    const Mat22d m{Mat22d::identity() * 2.0};
    const Vec2d v{m.dot(Vec2d{1.0, -3.0})};
    Mat22d n{1.0, 2.0, 3.0, 4.0};
    n -= Mat22d::identity();
    return v.y() == -6.0 && n[1, 1] == 3.0 && n[0] == 0.0 && m.dot(n) == n * 2.0;
}
static_assert(constantMat22());

// The products and norms of the constant evaluation path under layout_right, the general matrix
// product included.
consteval auto constantMat22Right() -> bool {
    using M = Mat22<double, std::size_t, std::layout_right>;
    const M a{1.0, 2.0, 3.0, 4.0};
    const Vec2d v{a.dot(Vec2d{1.0, -1.0})};
    const M p{a.dot(M{0.0, 1.0, 1.0, 0.0})};
    Matrix<double, std::size_t, 2, 3, std::layout_right> b{};
    b[0, 2] = 1.0;
    b[1, 0] = 2.0;
    const auto q{a.dot(b)};
    // Exact in binary: the inverse of diag(2, 4) is diag(1/2, 1/4).
    const M inverse{M{2.0, 0.0, 0.0, 4.0}.inversed()};
    return v.x() == -1.0 && v.y() == -1.0 && p[0, 0] == 2.0 && p[1, 1] == 3.0 && q[0, 0] == 4.0 &&
           q[1, 2] == 3.0 && a.euclideanSqr() == 30.0 && a.determinant() == -2.0 &&
           inverse[0, 0] == 0.5 && inverse[0, 1] == 0.0 && inverse[1, 1] == 0.25;
}
static_assert(constantMat22Right());

// clang-format off
TEST_CASE_TEMPLATE(
    "Mat22 norms and products agree with their definitions for every scalar type", T,
    Mat22s, Mat22d, Mat22c, Mat22z, Mat22<std::int32_t>, Mat22<std::uint32_t>, Mat22<std::int64_t>,
    Mat22<std::uint64_t>, Mat22<std::int16_t>, Mat22<long double>,
    Mat22<float, std::size_t, std::layout_right>, Mat22<double, std::size_t, std::layout_right>,
    Mat22<std::complex<float>, std::size_t, std::layout_right>,
    Mat22<std::complex<double>, std::size_t, std::layout_right>,
    Mat22<std::int32_t, std::size_t, std::layout_right>, Mat22<std::int64_t, std::size_t, std::layout_right>
) {
    // clang-format on
    using V = typename T::value_type;
    using L = typename T::layout_type;
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
    const T a{make(3, 1), make(2, 4), make(1, 3), make(5, 2)};
    const T b{make(1, 2), make(4, 1), make(2, 2), make(3, 5)};

    SUBCASE("euclideanSqr and euclidean") {
        if constexpr (isComplexArithmeticV<V>) {
            const auto sum{
                std::norm(a[0, 0]) + std::norm(a[0, 1]) + std::norm(a[1, 0]) + std::norm(a[1, 1])
            };
            CHECK_EQ(a.euclideanSqr(), doctest::Approx(sum));
            CHECK_EQ(a.euclidean(), doctest::Approx(std::sqrt(sum)));
        } else {
            check_close(
                a.euclideanSqr(),
                static_cast<V>(
                    a[0, 0] * a[0, 0] + a[0, 1] * a[0, 1] + a[1, 0] * a[1, 0] + a[1, 1] * a[1, 1]
                )
            );
        }
    }

    SUBCASE("the products with a vector and with a Mat22") {
        const Vec2<V> v{make(2, 1), make(1, 3)};
        const auto mv{a.dot(v)};
        for (std::size_t i{0}; i < 2; ++i) {
            check_close(mv[i], static_cast<V>(a[i, 0] * v[0] + a[i, 1] * v[1]));
        }
        const T ab{a.dot(b)};
        for (std::size_t i{0}; i < 2; ++i) {
            for (std::size_t j{0}; j < 2; ++j) {
                check_close(ab[i, j], static_cast<V>(a[i, 0] * b[0, j] + a[i, 1] * b[1, j]));
            }
        }
    }

    SUBCASE("the product with a general matrix of two rows, at every width") {
        const auto check = [&]<std::size_t N>() -> void {
            Matrix<V, std::size_t, 2, N, L> m{};
            for (std::size_t r{0}; r < 2; ++r) {
                for (std::size_t c{0}; c < N; ++c) {
                    m[r, c] = make(static_cast<int>(r + 2 * c + 1), static_cast<int>(c + r));
                }
            }
            const auto product{a.dot(m)};
            for (std::size_t r{0}; r < 2; ++r) {
                for (std::size_t c{0}; c < N; ++c) {
                    check_close(
                        product[r, c], static_cast<V>(a[r, 0] * m[0, c] + a[r, 1] * m[1, c])
                    );
                }
            }
        };
        check.template operator()<1>();
        check.template operator()<3>();
        check.template operator()<4>();
        check.template operator()<5>();
        check.template operator()<7>();
    }
}

} // namespace basjoo::math
