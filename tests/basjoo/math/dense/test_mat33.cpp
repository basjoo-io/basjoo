/**
 * @file math/dense/test_mat33.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2026-11-03
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/dense/mat33.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mdspan>
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

// The reference products: Mat33 has no dot of its own yet.
template <typename T>
auto product(const T& lhs, const T& rhs) -> T {
    T result(3, 3, typename T::value_type(0.0));
    for (std::size_t i{0}; i < 3; ++i) {
        for (std::size_t j{0}; j < 3; ++j) {
            for (std::size_t k{0}; k < 3; ++k) {
                result[i, j] += lhs[i, k] * rhs[k, j];
            }
        }
    }
    return result;
}

template <typename T>
auto matVec(const T& m, const Vector<typename T::value_type, std::size_t, 3>& v)
    -> Vector<typename T::value_type, std::size_t, 3> {
    return {
        m[0, 0] * v.x() + m[0, 1] * v.y() + m[0, 2] * v.z(),
        m[1, 0] * v.x() + m[1, 1] * v.y() + m[1, 2] * v.z(),
        m[2, 0] * v.x() + m[2, 1] * v.y() + m[2, 2] * v.z()
    };
}

template <typename T>
auto sample() -> T {
    using V = typename T::value_type;
    if constexpr (ComplexArithmetic<V>) {
        return T(
            V(2.0, 0.5), V(-1.0, 1.0), V(0.5, -0.25), V(1.0, -1.5), V(3.0, 0.75), V(-2.0, 0.5),
            V(0.5, 1.0), V(1.0, -0.5), V(4.0, 2.0)
        );
    } else {
        return T(V(2.0), V(-1.0), V(0.5), V(1.0), V(3.0), V(-2.0), V(0.5), V(1.0), V(4.0));
    }
}

template <typename T>
auto padding(const T& m) -> std::array<typename T::value_type, 3> {
    return {m.data()[3], m.data()[7], m.data()[11]};
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
    "Mat33 constructors take the elements in reading order under either layout", T,
    Mat33<std::int32_t>, Mat33s, Mat33d, Mat33c, Mat33z,
    Mat33<std::int32_t, std::size_t, std::layout_right>,
    Mat33<float, std::size_t, std::layout_right>, Mat33<double, std::size_t, std::layout_right>,
    Mat33<std::complex<float>, std::size_t, std::layout_right>,
    Mat33<std::complex<double>, std::size_t, std::layout_right>
) {
    using V = typename T::value_type;
    const std::array<V, 3> zeros{};

    const T m(V(1.0), V(2.0), V(3.0), V(4.0), V(5.0), V(6.0), V(7.0), V(8.0), V(9.0));
    for (std::size_t i{0}; i < 3; ++i) {
        for (std::size_t j{0}; j < 3; ++j) {
            CHECK_EQ(m[i, j], V(static_cast<double>(3 * i + j + 1)));
        }
    }
    // The columns are stored under layout_left, the rows under layout_right, each padded to four.
    CHECK_EQ(
        m.data()[1], std::is_same_v<typename T::layout_type, std::layout_left> ? V(4.0) : V(2.0)
    );
    CHECK_EQ(
        m.data()[4], std::is_same_v<typename T::layout_type, std::layout_left> ? V(2.0) : V(4.0)
    );
    CHECK_EQ(padding(m), zeros);

    const T zero(3, 3, V(0.0));
    CHECK_EQ(T(3, 3), zero);
    CHECK_EQ(T(typename T::allocator_type{}), zero);
    const T filled(3, 3, V(7.0));
    CHECK_EQ(filled, T(V(7.0), V(7.0), V(7.0), V(7.0), V(7.0), V(7.0), V(7.0), V(7.0), V(7.0)));
    CHECK_EQ(padding(filled), zeros);
    CHECK_EQ(
        T::identity(), T(V(1.0), V(0.0), V(0.0), V(0.0), V(1.0), V(0.0), V(0.0), V(0.0), V(1.0))
    );

    // Default construction zeroes the padding together with the elements.
    const T d;
    CHECK_EQ(d, zero);
    CHECK_EQ(padding(d), zeros);

    const std::array<V, 9> raw{V(1.0), V(2.0), V(3.0), V(4.0), V(5.0),
                               V(6.0), V(7.0), V(8.0), V(9.0)};
    const std::mdspan<const V, std::extents<std::size_t, 3, 3>, std::layout_right> view{raw.data()};
    CHECK_EQ(T(view), m);

    using Vec = Vector<V, std::size_t, 3>;
    CHECK_EQ(
        T::fromColumns(
            Vec(V(1.0), V(4.0), V(7.0)), Vec(V(2.0), V(5.0), V(8.0)), Vec(V(3.0), V(6.0), V(9.0))
        ),
        m
    );
    CHECK_EQ(
        T::fromRows(
            Vec(V(1.0), V(2.0), V(3.0)), Vec(V(4.0), V(5.0), V(6.0)), Vec(V(7.0), V(8.0), V(9.0))
        ),
        m
    );

    T n;
    n.fill(V(5.0));
    CHECK_EQ(n, T(3, 3, V(5.0)));
    n.assign(3, 3, V(6.0));
    CHECK_EQ(n, T(3, 3, V(6.0)));
    n.setIdentity();
    CHECK_EQ(n, T::identity());
    n.updateCoeff(2, 1, V(9.0));
    CHECK_EQ(n.coeff(2, 1), V(9.0));

#if BASJOO_CHECK_PARAMS == 1
    CHECK_THROWS_AS(T(3, 2), std::invalid_argument);
    CHECK_THROWS_AS(n.resize(2, 3), std::invalid_argument);
    CHECK_THROWS_AS(std::ignore = n.coeff(3, 0), std::out_of_range);
#endif
}

TEST_CASE("Mat33 converts between scalar types") {
    const Mat33s a(1.5F, -2.0F, 0.25F, 4.0F, 0.5F, -1.0F, 2.0F, 3.0F, -0.75F);
    const Mat33d b(a);
    CHECK_EQ(b, Mat33d(1.5, -2.0, 0.25, 4.0, 0.5, -1.0, 2.0, 3.0, -0.75));
}

TEST_CASE_TEMPLATE(
    "Mat33 is a static 3x3 matrix with padded, strided storage", T, Mat33d, Mat33z,
    Mat33<double, std::size_t, std::layout_right>, Mat33s
) {
    using V = typename T::value_type;

    static_assert(MatArithmetic<T>);
    static_assert(!VecArithmetic<T>);
    static_assert(Mat33Arithmetic<T>);
    static_assert(!Mat33Arithmetic<Vector<V, std::size_t, 3>> && !Mat33Arithmetic<V>);
    static_assert(std::is_trivially_copyable_v<T>);
    static_assert(sizeof(T) == 12 * sizeof(V) && alignof(T) == 4 * sizeof(V));
    static_assert(T::rank() == 2 && T::rank_dynamic() == 0);
    static_assert(T::static_extent(0) == 3 && T::static_extent(1) == 3);
    static_assert(T::nrows() == 3 && T::ncols() == 3 && T::size() == 12);
    static_assert(!T::is_always_exhaustive() && T::is_always_unique() && T::is_always_strided());
    static_assert(std::is_same_v<decltype(T::get_allocator()), typename T::allocator_type>);
    // The leading dimension LAPACK reads is the product of the strides.
    if constexpr (std::is_same_v<typename T::layout_type, std::layout_left>) {
        static_assert(T::stride(0) == 1 && T::stride(1) == 4);
    } else {
        static_assert(T::stride(0) == 4 && T::stride(1) == 1);
    }

    T m{sample<T>()};
    const std::mdspan<V, typename T::extents_type, std::layout_stride> view{m};
    const std::mdspan<const V, std::dextents<std::size_t, 2>, std::layout_stride> strided{
        std::as_const(m)
    };
    for (std::size_t i{0}; i < 3; ++i) {
        for (std::size_t j{0}; j < 3; ++j) {
            CHECK_EQ(view[i, j], m[i, j]);
            CHECK_EQ(strided[i, j], m[i, j]);
        }
    }
    view[2, 1] = V(8.0);
    CHECK_EQ(m[2, 1], V(8.0));

    const auto block{m.submdspan({1, 3}, {0, 2})};
    CHECK_EQ(block.extent(0), std::size_t{2});
    CHECK_EQ(block.extent(1), std::size_t{2});
    CHECK_EQ(block[1, 0], m[2, 0]);
    CHECK_EQ(block[0, 1], m[1, 1]);
}

TEST_CASE_TEMPLATE(
    "Mat33 offers single-index access at the storage offsets, padding included", T, Mat33d, Mat33z,
    Mat33<double, std::size_t, std::layout_right>
) {
    using V = typename T::value_type;

    T m{sample<T>()};
    for (std::size_t k{0}; k < T::size(); ++k) {
        CHECK_EQ(&m[k], m.data() + k);
        CHECK_EQ(m.coeff(k), m.data()[k]);
    }
    // Element (1, 2) sits at 1 + 4 * 2 under layout_left and at 4 * 1 + 2 under layout_right.
    const std::size_t offset{std::is_same_v<typename T::layout_type, std::layout_left> ? 9 : 6};
    CHECK_EQ(m[offset], m[1, 2]);
    m.updateCoeff(offset, V(7.0));
    CHECK_EQ(m[1, 2], V(7.0));
    for (const std::size_t pad : {3, 7, 11}) {
        CHECK_EQ(m[pad], V(0.0));
    }
#if BASJOO_CHECK_PARAMS == 1
    for (const std::size_t pad : {3, 7, 11}) {
        CHECK_THROWS_AS(m.updateCoeff(pad, V(1.0)), std::out_of_range);
    }
    CHECK_THROWS_AS(static_cast<void>(m.coeff(12)), std::out_of_range);
    CHECK_THROWS_AS(m.updateCoeff(12, V(1.0)), std::out_of_range);
    CHECK_THROWS_AS(static_cast<void>(m.submdspan({0, 2})), std::invalid_argument);
#else
    const auto row{m.submdspan({0, 2})};
    CHECK_EQ(row.extent(0), std::size_t{1});
    CHECK_EQ(row.extent(1), std::size_t{2});
    CHECK_EQ(row[0, 1], m[0, 1]);
#endif
}

TEST_CASE_TEMPLATE(
    "Mat33 arithmetic is element-wise", T, Mat33<std::int32_t>, Mat33s, Mat33d, Mat33c, Mat33z,
    Mat33<double, std::size_t, std::layout_right>,
    Mat33<std::complex<float>, std::size_t, std::layout_right>
) {
    using V = typename T::value_type;

    const T a{sample<T>()};
    const T b(V(0.5), V(6.0), V(-1.0), V(2.0), V(-3.0), V(1.5), V(0.25), V(-4.0), V(1.0));
    const auto each = [&](auto op) -> T {
        T result;
        for (std::size_t i{0}; i < 3; ++i) {
            for (std::size_t j{0}; j < 3; ++j) {
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
    "Mat33 keeps its padding zero, so comparisons and norms see the nine elements", T,
    Mat33<std::int32_t>, Mat33s, Mat33d, Mat33c, Mat33z,
    Mat33<float, std::size_t, std::layout_right>,
    Mat33<std::complex<double>, std::size_t, std::layout_right>
) {
    using V = typename T::value_type;
    using R = detail::DenseNormTraitT<V>;

    const T a{sample<T>()};
    R norm_sqr(0.0);
    for (std::size_t i{0}; i < 3; ++i) {
        for (std::size_t j{0}; j < 3; ++j) {
            norm_sqr += std::norm(a[i, j]);
        }
    }

    // The operations sweep over the padding and leave it zero.
    const auto padding_is_zero = [](const T& m) -> bool {
        return m[3] == V(0.0) && m[7] == V(0.0) && m[11] == V(0.0);
    };
    CHECK(padding_is_zero(a));
    CHECK(padding_is_zero(T{}));
    CHECK(padding_is_zero(a + a));
    CHECK(padding_is_zero(a - T(3, 3, V(1.0))));
    CHECK(padding_is_zero(-a));
    CHECK(padding_is_zero(a * V(3.0)));
    CHECK(padding_is_zero(a.dot(a)));
    CHECK(padding_is_zero(a.transposed()));
    if constexpr (!std::is_integral_v<V>) {
        CHECK(padding_is_zero(a / V(2.0)));
        CHECK(padding_is_zero(a.inversed()));
    }
    CHECK_EQ(T{a}, a);

    if constexpr (std::is_integral_v<R>) {
        CHECK_EQ(a.euclideanSqr(), norm_sqr);
        CHECK_EQ(std::norm(a), norm_sqr);
    } else {
        CHECK_EQ(a.euclideanSqr(), doctest::Approx(norm_sqr).epsilon(kTol<T>));
        CHECK_EQ(a.euclidean(), doctest::Approx(std::sqrt(norm_sqr)).epsilon(kTol<T>));
        CHECK_EQ(std::abs(a), doctest::Approx(std::sqrt(norm_sqr)).epsilon(kTol<T>));
        CHECK(a.identicalTo(a + T(3, 3, V(1E-3 * kTol<T>)), kTol<T>));
        CHECK_FALSE(a.identicalTo(a + T(3, 3, V(1E-2)), kTol<T>));
    }

    // Each element differing once breaks equality.
    for (std::size_t i{0}; i < 3; ++i) {
        for (std::size_t j{0}; j < 3; ++j) {
            T b{a};
            b[i, j] += V(1.0);
            CHECK_FALSE(b == a);
        }
    }
}

TEST_CASE_TEMPLATE(
    "Mat33 trace, determinant and transpose", T, Mat33<std::int32_t>, Mat33s, Mat33d, Mat33c,
    Mat33z, Mat33<float, std::size_t, std::layout_right>,
    Mat33<double, std::size_t, std::layout_right>,
    Mat33<std::complex<double>, std::size_t, std::layout_right>
) {
    using V = typename T::value_type;

    const T a{sample<T>()};
    CHECK_EQ(a.trace(), a[0, 0] + a[1, 1] + a[2, 2]);

    // Laplace expansion along the first row.
    const V det{
        a[0, 0] * (a[1, 1] * a[2, 2] - a[1, 2] * a[2, 1]) -
        a[0, 1] * (a[1, 0] * a[2, 2] - a[1, 2] * a[2, 0]) +
        a[0, 2] * (a[1, 0] * a[2, 1] - a[1, 1] * a[2, 0])
    };
    if constexpr (std::is_integral_v<V>) {
        CHECK_EQ(a.determinant(), det);
    } else {
        CHECK_LT(std::abs(a.determinant() - det), kTol<T> * std::abs(det));
    }

    const T t{a.transposed()};
    for (std::size_t i{0}; i < 3; ++i) {
        for (std::size_t j{0}; j < 3; ++j) {
            CHECK_EQ(t[i, j], a[j, i]);
        }
    }
    CHECK_EQ(padding(t), std::array<V, 3>{});
    T u{a};
    u.selfTransposed().selfTransposed();
    CHECK_EQ(u, a);
}

TEST_CASE_TEMPLATE(
    "Mat33 inverse", T, Mat33s, Mat33d, Mat33c, Mat33z,
    Mat33<float, std::size_t, std::layout_right>, Mat33<double, std::size_t, std::layout_right>,
    Mat33<std::complex<float>, std::size_t, std::layout_right>
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
    T singular(V(1.0), V(2.0), V(3.0), V(2.0), V(4.0), V(6.0), V(0.5), V(1.0), V(-1.0));
    CHECK_THROWS_AS(singular.selfInversed(), std::runtime_error);
#endif
}

TEST_CASE_TEMPLATE(
    "Mat33 rotation from two vectors", T, Mat33s, Mat33d,
    Mat33<float, std::size_t, std::layout_right>, Mat33<double, std::size_t, std::layout_right>
) {
    using V = typename T::value_type;
    using Vec = Vector<V, std::size_t, 3>;

    const auto check_maps = [](const Vec& from, const Vec& to) {
        const T r{T::fromTwoVectors(from, to)};
        CHECK(r.isRotation(kTol<T>));
        const Vec image{matVec(r, from.normalized())};
        const Vec expected{to.normalized()};
        CHECK(image.identicalTo(expected, kTol<T>));
        CHECK(r.dot(from.normalized()).identicalTo(expected, kTol<T>));
        return r;
    };

    SUBCASE("generic directions, minimal rotation") {
        const Vec from(V(1.0), V(2.0), V(-0.5));
        const Vec to(V(-3.0), V(0.25), V(1.5));
        const T r{check_maps(from, to)};
        // The axis from x to is fixed and the angle is the one between the vectors.
        const Vec axis{from.cross(to).normalized()};
        CHECK(matVec(r, axis).identicalTo(axis, kTol<T>));
        const V cos_angle{from.normalized().dot(to.normalized())};
        CHECK_EQ(r.trace(), doctest::Approx(V(1.0) + V(2.0) * cos_angle).epsilon(kTol<T>));
        // Lengths do not matter.
        CHECK(T::fromTwoVectors(from * V(4.0), to * V(0.125)).identicalTo(r, kTol<T>));
    }

    SUBCASE("parallel directions give the identity") {
        const Vec from(V(0.5), V(-1.0), V(2.0));
        CHECK(check_maps(from, from * V(3.0)).identicalTo(T::identity(), kTol<T>));
    }

    SUBCASE("antiparallel directions give half a turn") {
        for (const Vec& from :
             {Vec(V(1.0), V(0.0), V(0.0)), Vec(V(0.0), V(-2.0), V(0.0)),
              Vec(V(0.0), V(0.0), V(3.0)), Vec(V(1.0), V(2.0), V(3.0)),
              Vec(V(-0.5), V(0.25), V(-4.0))}) {
            const T r{check_maps(from, from * V(-2.0))};
            CHECK(product(r, r).identicalTo(T::identity(), kTol<T>));
        }
    }

    SUBCASE("a zero vector has no direction") {
        const Vec from(V(1.0), V(2.0), V(3.0));
        const Vec zero(V(0.0), V(0.0), V(0.0));
#if BASJOO_CHECK_PARAMS == 1
        CHECK_THROWS_AS(std::ignore = T::fromTwoVectors(zero, from), std::invalid_argument);
        CHECK_THROWS_AS(std::ignore = T::fromTwoVectors(from, zero), std::invalid_argument);
#else
        CHECK(std::isnan(T::fromTwoVectors(zero, from)[0, 0]));
        CHECK(std::isnan(T::fromTwoVectors(from, zero)[0, 0]));
#endif
    }
}

TEST_CASE_TEMPLATE(
    "Mat33 rotation checks", T, Mat33s, Mat33d, Mat33<float, std::size_t, std::layout_right>,
    Mat33<double, std::size_t, std::layout_right>
) {
    using V = typename T::value_type;
    using Vec = Vector<V, std::size_t, 3>;

    const T r{T::fromTwoVectors(Vec(V(1.0), V(2.0), V(-0.5)), Vec(V(-3.0), V(0.25), V(1.5)))};
    CHECK(r.isRotation(kTol<T>));
    CHECK(T::identity().isRotation(kTol<T>));
    CHECK_FALSE((r * V(1.01)).isRotation(kTol<T>));
    CHECK_FALSE(sample<T>().isRotation(kTol<T>));
    const T reflection(V(1.0), V(0.0), V(0.0), V(0.0), V(1.0), V(0.0), V(0.0), V(0.0), V(-1.0));
    CHECK_FALSE(reflection.isRotation(kTol<T>));
}

TEST_CASE_TEMPLATE(
    "Mat33 products agree with the definition", T, Mat33<std::int32_t>, Mat33s, Mat33d, Mat33c,
    Mat33z, Mat33<std::int32_t, std::size_t, std::layout_right>,
    Mat33<float, std::size_t, std::layout_right>, Mat33<double, std::size_t, std::layout_right>,
    Mat33<std::complex<float>, std::size_t, std::layout_right>,
    Mat33<std::complex<double>, std::size_t, std::layout_right>
) {
    using V = typename T::value_type;
    using Vec = Vector<V, std::size_t, 3>;
    using Wide = Matrix<V, std::size_t, 3, 4, typename T::layout_type>;

    const T a{sample<T>()};
    const T b(
        value<V>(0.5, 1.0), value<V>(6.0, -0.5), value<V>(-1.0, 0.25), value<V>(2.0, 2.0),
        value<V>(-3.0, 0.5), value<V>(1.5, -1.0), value<V>(0.25, 0.0), value<V>(-4.0, 1.5),
        value<V>(1.0, -2.0)
    );
    const Vec v(value<V>(1.5, -0.25), value<V>(-0.5, 0.75), value<V>(2.0, 0.5));
    Wide w(3, 4);
    for (std::size_t i{0}; i < 3; ++i) {
        for (std::size_t j{0}; j < 4; ++j) {
            w[i, j] = value<V>(static_cast<double>(4 * i + j) - 2.0, static_cast<double>(j) * 0.5);
        }
    }

    SUBCASE("matrix times matrix") {
        const T ab{a.dot(b)};
        const T expected{product(a, b)};
        for (std::size_t i{0}; i < 3; ++i) {
            for (std::size_t j{0}; j < 3; ++j) {
                checkClose<T>(ab[i, j], expected[i, j]);
            }
        }
        CHECK_EQ(T::identity().dot(a), a);
        CHECK_EQ(a.dot(T::identity()), a);
    }

    SUBCASE("matrix times vector and vector times matrix") {
        const Vec av{a.dot(v)};
        const Vec va{v.dot(a)};
        for (std::size_t i{0}; i < 3; ++i) {
            checkClose<T>(av[i], a[i, 0] * v[0] + a[i, 1] * v[1] + a[i, 2] * v[2]);
            checkClose<T>(va[i], v[0] * a[0, i] + v[1] * a[1, i] + v[2] * a[2, i]);
        }
    }

    SUBCASE("products with a generic matrix of three rows") {
        const Wide aw{a.dot(w)};
        const auto vw{v.dot(w)};
        for (std::size_t j{0}; j < 4; ++j) {
            for (std::size_t i{0}; i < 3; ++i) {
                checkClose<T>(aw[i, j], a[i, 0] * w[0, j] + a[i, 1] * w[1, j] + a[i, 2] * w[2, j]);
            }
            checkClose<T>(vw[j], v[0] * w[0, j] + v[1] * w[1, j] + v[2] * w[2, j]);
        }
    }
}

TEST_CASE_TEMPLATE(
    "Mat33 serializes its twelve storage slots, the padding included", T, Mat33<std::int32_t>,
    Mat33s, Mat33d, Mat33c, Mat33z, Mat33<double, std::size_t, std::layout_right>
) {
    using V = typename T::value_type;

    const T a{sample<T>()};

    auto [data, out] = zpp::bits::data_out();
    out(a).or_throw();
    CHECK_EQ(data.size(), 12 * sizeof(V));

    const V* p{a.data()};
    auto [raw, raw_out] = zpp::bits::data_out();
    raw_out(
        std::array<V, 12>{p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11]}
    )
        .or_throw();
    CHECK_EQ(data, raw);

    T b;
    auto in = zpp::bits::in(data);
    in(b).or_throw();
    CHECK_EQ(b, a);
}

// Array storage lets the element-wise arithmetic and the products run in constant expressions.
consteval auto constantMat33() -> bool {
    const Mat33d m{Mat33d::identity() * 2.0};
    const Vec3d v{m.dot(Vec3d{1.0, -3.0, 0.5})};
    Mat33d n{Mat33d::identity()};
    n += m;
    return v.z() == 1.0 && v[3] == 0.0 && n[2, 2] == 3.0 && n[3] == 0.0 && m.dot(n) == n * 2.0;
}
static_assert(constantMat33());

// The products and norms of the constant evaluation path under layout_right, the general matrix
// product included.
consteval auto constantMat33Right() -> bool {
    using M = Mat33<double, std::size_t, std::layout_right>;
    const M a{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0};
    const Vec3d v{a.dot(Vec3d{1.0, -1.0, 0.0})};
    // Swaps the first two columns of a.
    const M p{a.dot(M{0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0})};
    Matrix<double, std::size_t, 3, 2, std::layout_right> b{};
    b[0, 1] = 1.0;
    b[2, 0] = 2.0;
    const auto q{a.dot(b)};
    // Exact in binary: the cofactors are integers and the determinant 2.
    const M inverse{M{1.0, 2.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0}.inversed()};
    return v.x() == -1.0 && v.y() == -1.0 && v.z() == -1.0 && v[3] == 0.0 && p[0, 0] == 2.0 &&
           p[2, 1] == 7.0 && p[3] == 0.0 && q[0, 0] == 6.0 && q[2, 0] == 20.0 && q[2, 1] == 7.0 &&
           a.euclideanSqr() == 304.0 && a.determinant() == -3.0 && inverse[0, 1] == -2.0 &&
           inverse[1, 1] == 1.0 && inverse[2, 2] == 0.5 && inverse[3] == 0.0;
}
static_assert(constantMat33Right());

// clang-format off
TEST_CASE_TEMPLATE(
    "Mat33 norms and products agree with their definitions for every scalar type", T,
    Mat33s, Mat33d, Mat33c, Mat33z, Mat33<std::int32_t>, Mat33<std::uint32_t>, Mat33<std::int64_t>,
    Mat33<std::uint64_t>, Mat33<std::int16_t>, Mat33<long double>,
    Mat33<float, std::size_t, std::layout_right>, Mat33<double, std::size_t, std::layout_right>,
    Mat33<std::complex<float>, std::size_t, std::layout_right>,
    Mat33<std::complex<double>, std::size_t, std::layout_right>,
    Mat33<std::int32_t, std::size_t, std::layout_right>, Mat33<std::int64_t, std::size_t, std::layout_right>
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
    const T a{make(3, 1), make(2, 4), make(1, 3), make(5, 2), make(4, 1),
              make(2, 2), make(1, 5), make(3, 3), make(2, 1)};
    const T b{make(1, 2), make(4, 1), make(2, 2), make(3, 5), make(1, 1),
              make(2, 3), make(5, 1), make(2, 4), make(3, 2)};
    const std::array<V, 3> zeros{};

    SUBCASE("euclideanSqr and euclidean") {
        if constexpr (isComplexArithmeticV<V>) {
            typename V::value_type sum{0};
            for (std::size_t i{0}; i < 3; ++i) {
                for (std::size_t j{0}; j < 3; ++j) {
                    sum += std::norm(a[i, j]);
                }
            }
            CHECK_EQ(a.euclideanSqr(), doctest::Approx(sum));
            CHECK_EQ(a.euclidean(), doctest::Approx(std::sqrt(sum)));
        } else {
            V sum{0};
            for (std::size_t i{0}; i < 3; ++i) {
                for (std::size_t j{0}; j < 3; ++j) {
                    sum = static_cast<V>(sum + a[i, j] * a[i, j]);
                }
            }
            check_close(a.euclideanSqr(), sum);
        }
    }

    SUBCASE("the products with a vector and with a Mat33 keep the padding zero") {
        const Vector<V, std::size_t, 3> v{make(2, 1), make(1, 3), make(3, 2)};
        const auto mv{a.dot(v)};
        for (std::size_t i{0}; i < 3; ++i) {
            check_close(mv[i], static_cast<V>(a[i, 0] * v[0] + a[i, 1] * v[1] + a[i, 2] * v[2]));
        }
        CHECK_EQ(mv.data()[3], V(0));
        const T ab{a.dot(b)};
        for (std::size_t i{0}; i < 3; ++i) {
            for (std::size_t j{0}; j < 3; ++j) {
                check_close(
                    ab[i, j],
                    static_cast<V>(a[i, 0] * b[0, j] + a[i, 1] * b[1, j] + a[i, 2] * b[2, j])
                );
            }
        }
        CHECK_EQ(padding(ab), zeros);
    }

    SUBCASE("the product with a general matrix of three rows, at every width") {
        const auto check = [&]<std::size_t N>() -> void {
            Matrix<V, std::size_t, 3, N, L> m{};
            for (std::size_t r{0}; r < 3; ++r) {
                for (std::size_t c{0}; c < N; ++c) {
                    m[r, c] = make(static_cast<int>(r + 2 * c + 1), static_cast<int>(c + r));
                }
            }
            const auto product{a.dot(m)};
            for (std::size_t r{0}; r < 3; ++r) {
                for (std::size_t c{0}; c < N; ++c) {
                    check_close(
                        product[r, c],
                        static_cast<V>(a[r, 0] * m[0, c] + a[r, 1] * m[1, c] + a[r, 2] * m[2, c])
                    );
                }
            }
        };
        check.template operator()<1>();
        check.template operator()<2>();
        check.template operator()<4>();
        check.template operator()<5>();
        check.template operator()<7>();
        check.template operator()<9>();
    }
}

} // namespace basjoo::math
