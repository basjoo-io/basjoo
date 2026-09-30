/**
 * @file math/dense/test_quaternion.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2026-11-04
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/dense/quaternion.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <limits>
#include <mdspan>
#include <numbers>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>

#include "zpp_bits.h"

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_arithmetic_base.hpp"
#include "basjoo/math/dense/dense_formatter.hpp"
#include "basjoo/math/dense/mat33.hpp"
#include "basjoo/math/dense/vec3.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::math {

namespace {

template <typename T>
constexpr auto kTol{std::is_same_v<T, float> ? 1E-5 : 1E-12};

// Unit quaternions that take every branch of Mat33 -> Quaternion: the trace (twice, once with
// w < 0), the largest diagonal element x, y (with w < 0) and z, and the identity.
template <typename T>
auto samples() -> std::array<Quaternion<T>, 6> {
    using Q = Quaternion<T>;
    return {
        Q(T(0.2), T(-0.4), T(0.6), T(0.66)).normalized(),
        Q(T(-0.5), T(0.1), T(0.3), T(-0.8)).normalized(),
        Q(T(0.9), T(0.2), T(-0.1), T(0.05)).normalized(),
        Q(T(0.1), T(0.95), T(-0.2), T(-0.1)).normalized(),
        Q(T(0.0), T(0.6), T(0.8), T(0.0)),
        Q::identity(),
    };
}

// The vector part of q (v, 0) q*, by two Hamilton products.
template <typename T>
auto conjugateBy(const Quaternion<T>& q, const Vector<T, std::size_t, 3>& v)
    -> Vector<T, std::size_t, 3> {
    const Quaternion<T> image{q * Quaternion<T>(v.x(), v.y(), v.z(), T(0.0)) * q.conjugated()};
    return {image.x(), image.y(), image.z()};
}

// Each of the three returned values within kTol of the expected one, relative to max(|it|, 1).
template <typename T>
auto checkClose(const std::tuple<T, T, T>& actual, T e0, T e1, T e2) -> void {
    const auto [a0, a1, a2] = actual;
    CHECK_EQ(a0, doctest::Approx(e0).epsilon(kTol<T>));
    CHECK_EQ(a1, doctest::Approx(e1).epsilon(kTol<T>));
    CHECK_EQ(a2, doctest::Approx(e2).epsilon(kTol<T>));
}

template <typename M, typename Q>
concept ConvertsFrom = requires(const Q& q) { M::fromQuaternion(q); };

} // namespace

TEST_CASE_TEMPLATE("Quaternion stores x, y, z, w in the order it takes them", T, float, double) {
    using Q = Quaternion<T>;
    using Other = std::conditional_t<std::is_same_v<T, float>, double, float>;

    static_assert(QuaternionArithmetic<Q>);
    static_assert(!QuaternionArithmetic<Vector<T, std::size_t, 3>> && !QuaternionArithmetic<T>);
    static_assert(!VecArithmetic<Q> && !MatArithmetic<Q> && !GeneralArithmetic<Q>);
    static_assert(std::is_trivially_copyable_v<Q> && std::is_trivially_default_constructible_v<Q>);
    static_assert(sizeof(Q) == 4 * sizeof(T) && alignof(Q) == 4 * sizeof(T));
    // Only a 3x3 matrix of the same scalar takes a quaternion.
    static_assert(ConvertsFrom<Matrix<T, std::size_t, 3, 3>, Q>);
    static_assert(ConvertsFrom<Matrix<T, std::size_t, 3, 3, std::layout_right>, Q>);
    static_assert(!ConvertsFrom<Matrix<Other, std::size_t, 3, 3>, Q>);
    static_assert(
        !ConvertsFrom<Matrix<int, std::size_t, 3, 3>, Q> &&
        !ConvertsFrom<Matrix<std::complex<double>, std::size_t, 3, 3>, Q>
    );

    const Q q(T(1.0), T(2.0), T(3.0), T(4.0));
    CHECK_EQ(q.x(), T(1.0));
    CHECK_EQ(q.y(), T(2.0));
    CHECK_EQ(q.z(), T(3.0));
    CHECK_EQ(q.w(), T(4.0));
    CHECK_EQ(std::bit_cast<std::array<T, 4>>(q), std::array<T, 4>{T(1.0), T(2.0), T(3.0), T(4.0)});

    // Value initialization zeroes the components that default construction leaves alone.
    CHECK_EQ(Q{}, Q(T(0.0), T(0.0), T(0.0), T(0.0)));
    CHECK_EQ(Q::identity(), Q(T(0.0), T(0.0), T(0.0), T(1.0)));

    const Quaternion<Other> converted(q);
    CHECK_EQ(converted, Quaternion<Other>(Other(1.0), Other(2.0), Other(3.0), Other(4.0)));
}

TEST_CASE_TEMPLATE(
    "Quaternion addition and scaling are element-wise", T, float, double, long double
) {
    using Q = Quaternion<T>;
    // The element-wise arithmetic is DenseArithmeticBase's, and no vector or matrix concept takes
    // a quaternion.
    static_assert(std::derived_from<Q, DenseArithmeticBase>);
    static_assert(!VecArithmetic<Q> && !MatArithmetic<Q>);

    const Q a(T(1.0), T(-2.0), T(0.5), T(3.0));
    const Q b(T(0.25), T(4.0), T(-1.5), T(2.0));
    CHECK_EQ(a + b, Q(T(1.25), T(2.0), T(-1.0), T(5.0)));
    CHECK_EQ(a - b, Q(T(0.75), T(-6.0), T(2.0), T(1.0)));
    CHECK_EQ(-a, Q(T(-1.0), T(2.0), T(-0.5), T(-3.0)));
    CHECK_EQ(a * 2, Q(T(2.0), T(-4.0), T(1.0), T(6.0)));
    CHECK_EQ(T(2.0) * a, a * 2);
    CHECK_EQ(a / 2, Q(T(0.5), T(-1.0), T(0.25), T(1.5)));

    Q c{a};
    c += b;
    c -= b;
    CHECK_EQ(c, a);
    c *= T(4.0);
    c /= T(4.0);
    CHECK_EQ(c, a);

    // Rvalue operands are recycled in place.
    Q d{a};
    Q&& sum{std::move(d) + b};
    CHECK_EQ(&sum, &d);
    CHECK_EQ(sum, a + b);
    Q e{a};
    Q&& diff{b - std::move(e)};
    CHECK_EQ(&diff, &e);
    CHECK_EQ(diff, b - a);
    Q f{a};
    Q&& scaled{2 * std::move(f)};
    CHECK_EQ(&scaled, &f);
    CHECK_EQ(scaled, a * 2);
    Q g{a};
    Q&& product{std::move(g) * b};
    CHECK_EQ(&product, &g);
    CHECK_EQ(product, a * b);
}

TEST_CASE_TEMPLATE("Quaternion products follow Hamilton's rules", T, float, double, long double) {
    using Q = Quaternion<T>;

    const Q one{Q::identity()};
    const Q i(T(1.0), T(0.0), T(0.0), T(0.0));
    const Q j(T(0.0), T(1.0), T(0.0), T(0.0));
    const Q k(T(0.0), T(0.0), T(1.0), T(0.0));
    CHECK_EQ(i * j, k);
    CHECK_EQ(j * k, i);
    CHECK_EQ(k * i, j);
    CHECK_EQ(j * i, -k);
    CHECK_EQ(i * i, -one);
    CHECK_EQ(j * j, -one);
    CHECK_EQ(k * k, -one);
    CHECK_EQ(i * j * k, -one);

    // (s, u)(t, v) = (s t - u.v, s v + t u + u x v), written out by component: the vector parts
    // of general quaternions are no vectors of physical space.
    const Q a(T(0.5), T(-1.0), T(2.0), T(1.5));
    const Q b(T(-0.25), T(0.75), T(1.0), T(-2.0));
    const Q expected(
        a.w() * b.x() + b.w() * a.x() + (a.y() * b.z() - a.z() * b.y()),
        a.w() * b.y() + b.w() * a.y() + (a.z() * b.x() - a.x() * b.z()),
        a.w() * b.z() + b.w() * a.z() + (a.x() * b.y() - a.y() * b.x()),
        a.w() * b.w() - (a.x() * b.x() + a.y() * b.y() + a.z() * b.z())
    );
    const Q ab{a * b};
    CHECK(ab.identicalTo(expected, kTol<T>));
    CHECK_FALSE((b * a).identicalTo(ab, kTol<T>));
    CHECK(((a * b) * k).identicalTo(a * (b * k), kTol<T>));
    CHECK_EQ(one * a, a);
    CHECK_EQ(a * one, a);

    Q c{a};
    c *= b;
    CHECK_EQ(c, ab);
    Q d{a};
    d *= d;
    CHECK(d.identicalTo(a * a, kTol<T>));
}

TEST_CASE_TEMPLATE("Quaternion norms, conjugate and inverse", T, float, double, long double) {
    using Q = Quaternion<T>;

    const Q a(T(0.5), T(-1.0), T(2.0), T(1.5));
    const Q b(T(-0.25), T(0.75), T(1.0), T(-2.0));
    CHECK_EQ(a.dot(b), doctest::Approx(-0.125 - 0.75 + 2.0 - 3.0).epsilon(kTol<T>));
    CHECK_EQ(a.euclideanSqr(), doctest::Approx(7.5).epsilon(kTol<T>));
    CHECK_EQ(std::norm(a), a.euclideanSqr());
    CHECK_EQ(a.euclidean(), doctest::Approx(std::sqrt(7.5)).epsilon(kTol<T>));
    CHECK_EQ(std::abs(a), a.euclidean());

    CHECK_EQ(a.conjugated(), Q(T(-0.5), T(1.0), T(-2.0), T(1.5)));
    CHECK_EQ(std::conj(a), a.conjugated());
    CHECK((a * b).conjugated().identicalTo(b.conjugated() * a.conjugated(), kTol<T>));

    // The general inverse, which is the conjugate of a unit quaternion.
    CHECK((a * a.inversed()).identicalTo(Q::identity(), kTol<T>));
    CHECK((a.inversed() * a).identicalTo(Q::identity(), kTol<T>));
    const Q u{a.normalized()};
    CHECK_EQ(u.euclidean(), doctest::Approx(1.0).epsilon(kTol<T>));
    CHECK(u.inversed().identicalTo(u.conjugated(), kTol<T>));
    Q e{a};
    CHECK_EQ(&e.selfNormalized(), &e);
    CHECK(e.identicalTo(u, kTol<T>));
    Q f{a};
    Q&& g{std::move(f).inversed()};
    CHECK_EQ(&g, &f);

    // isUnit lets |q|^2 stray from 1 by sqrt(eps) by default.
    const T sqrt_eps{std::sqrt(std::numeric_limits<T>::epsilon())};
    CHECK(Q::identity().isUnit());
    CHECK(u.isUnit());
    CHECK((u * (T(1.0) + T(0.25) * sqrt_eps)).isUnit());
    CHECK_FALSE((u * (T(1.0) + sqrt_eps)).isUnit());
    CHECK_FALSE(a.isUnit());
    CHECK(a.isUnit(T(10.0)));

    // == and identicalTo compare the components: -q is the same rotation, but not the same
    // quaternion.
    CHECK_FALSE(u == -u);
    CHECK_FALSE(u.identicalTo(-u, kTol<T>));
    CHECK(u.identicalTo(u + Q(T(1E-3 * kTol<T>), T(0.0), T(0.0), T(0.0)), kTol<T>));
    CHECK_FALSE(u.identicalTo(u + Q(T(1E-2), T(0.0), T(0.0), T(0.0)), kTol<T>));

#if BASJOO_CHECK_PARAMS == 1
    CHECK_THROWS_AS(std::ignore = Q{}.inversed(), std::runtime_error);
#else
    CHECK(std::isnan(Q{}.inversed().w()));
#endif
}

// Exact inputs, whose products and sums are exact, so that the kernels run at run time must give
// the constant evaluation results bit for bit.
TEST_CASE_TEMPLATE(
    "Quaternion kernels agree with the constant evaluation path", T, float, double, long double
) {
    using Q = Quaternion<T>;

    constexpr Q kA(T(0.5), T(-1.0), T(2.0), T(1.5));
    constexpr Q kB(T(-0.25), T(0.75), T(1.0), T(-2.0));
    constexpr Q kProduct{kA * kB};
    constexpr Q kSquare{kA * kA};
    constexpr Q kConjugate{kA.conjugated()};
    constexpr T kDot{kA.dot(kB)};
    constexpr T kNormSqr{kA.euclideanSqr()};
    static_assert(kProduct == Q(T(-3.875), T(2.125), T(-2.375), T(-4.125)));
    static_assert(kSquare == Q(T(1.5), T(-3.0), T(6.0), T(-3.0)));
    static_assert(kConjugate == Q(T(-0.5), T(1.0), T(-2.0), T(1.5)));
    static_assert(kDot == T(-1.875) && kNormSqr == T(7.5));

    const Q a{kA};
    const Q b{kB};
    CHECK_EQ(a * b, kProduct);
    CHECK_EQ(b * a, Q(T(1.125), T(4.125), T(-2.625), T(-4.125)));
    Q c{a};
    c *= c;
    CHECK_EQ(c, kSquare);
    CHECK_EQ(a.conjugated(), kConjugate);
    CHECK_EQ(a.dot(b), kDot);
    CHECK_EQ(a.euclideanSqr(), kNormSqr);
    CHECK_EQ(a.euclidean(), std::sqrt(kNormSqr));

    // Negation flips the sign of zeros too.
    const Q conjugate_identity{Q::identity().conjugated()};
    CHECK(std::signbit(conjugate_identity.x()));
    CHECK(std::signbit(conjugate_identity.y()));
    CHECK(std::signbit(conjugate_identity.z()));
    CHECK_FALSE(std::signbit(conjugate_identity.w()));
}

TEST_CASE_TEMPLATE("Quaternion rotates vectors as q v q*", T, float, double) {
    using Q = Quaternion<T>;
    using V = Vector<T, std::size_t, 3>;

    const V v(T(1.0), T(-0.5), T(2.0));
    for (const Q& q : samples<T>()) {
        const V image{q.rotate(v)};
        CHECK(image.identicalTo(conjugateBy(q, v), kTol<T>));
        CHECK_EQ(image.euclidean(), doctest::Approx(v.euclidean()).epsilon(kTol<T>));
        CHECK(Matrix<T, std::size_t, 3, 3>::fromQuaternion(q).dot(v).identicalTo(image, kTol<T>));
        // -q rotates the same way.
        CHECK((-q).rotate(v).identicalTo(image, kTol<T>));
    }

    // q * p applies p first.
    const Q q{std::get<0>(samples<T>())};
    const Q p{std::get<1>(samples<T>())};
    CHECK((q * p).rotate(v).identicalTo(q.rotate(p.rotate(v)), kTol<T>));

    // A quarter turn about z takes x to y.
    const Q quarter{Q::fromAxisAngle(V(T(0.0), T(0.0), T(1.0)), std::numbers::pi_v<T> / 2)};
    CHECK(
        quarter.rotate(V(T(1.0), T(0.0), T(0.0))).identicalTo(V(T(0.0), T(1.0), T(0.0)), kTol<T>)
    );

    // The image has the type of the argument.
    using IndexedVec = Vector<T, int, 3>;
    static_assert(
        std::is_same_v<decltype(q.rotate(std::declval<const IndexedVec&>())), IndexedVec>
    );

#if BASJOO_CHECK_PARAMS == 1
    CHECK_THROWS_AS(
        std::ignore = Q(T(0.0), T(0.0), T(1.0), T(1.0)).rotate(v), std::invalid_argument
    );
#endif
}

TEST_CASE_TEMPLATE(
    "Quaternion and Mat33 convert into each other", M, Mat33s, Mat33d,
    Mat33<float, std::size_t, std::layout_right>, Mat33<double, std::size_t, std::layout_right>
) {
    using T = typename M::value_type;
    using Q = Quaternion<T>;
    using V = Vector<T, std::size_t, 3>;

    const std::array<V, 3> basis{
        V(T(1.0), T(0.0), T(0.0)), V(T(0.0), T(1.0), T(0.0)), V(T(0.0), T(0.0), T(1.0))
    };
    CHECK_EQ(M::fromQuaternion(Q::identity()), M::identity());
    for (const Q& q : samples<T>()) {
        const M m{M::fromQuaternion(q)};
        CHECK(m.isRotation(kTol<T>));
        // Column j is the image of the j-th coordinate axis.
        for (std::size_t j{0}; const V& axis : basis) {
            const V column{conjugateBy(q, axis)};
            CHECK_EQ(m[0, j], doctest::Approx(column.x()).epsilon(kTol<T>));
            CHECK_EQ(m[1, j], doctest::Approx(column.y()).epsilon(kTol<T>));
            CHECK_EQ(m[2, j], doctest::Approx(column.z()).epsilon(kTol<T>));
            ++j;
        }
        CHECK(M::fromQuaternion(-q).identicalTo(m, kTol<T>));
        // Back with w >= 0: q itself, or -q.
        const Q back{Q::fromRotationMatrix(m)};
        CHECK_GE(back.w(), T(0.0));
        CHECK(back.identicalTo(q.w() < T(0.0) ? -q : q, kTol<T>));
    }

    // A rotation built otherwise survives the round trip.
    const M r{M::fromTwoVectors(V(T(1.0), T(2.0), T(-0.5)), V(T(-3.0), T(0.25), T(1.5)))};
    CHECK(M::fromQuaternion(Q::fromRotationMatrix(r)).identicalTo(r, kTol<T>));

#if BASJOO_CHECK_PARAMS == 1
    CHECK_THROWS_AS(
        std::ignore = M::fromQuaternion(Q(T(0.0), T(0.0), T(1.0), T(1.0))), std::invalid_argument
    );
    CHECK_THROWS_AS(
        std::ignore = Q::fromRotationMatrix(M::identity() * T(2.0)), std::invalid_argument
    );
    const M reflection(T(1.0), T(0.0), T(0.0), T(0.0), T(1.0), T(0.0), T(0.0), T(0.0), T(-1.0));
    CHECK_THROWS_AS(std::ignore = Q::fromRotationMatrix(reflection), std::invalid_argument);
#endif
}

TEST_CASE_TEMPLATE("Quaternion axis, angle and rotation vector", T, float, double) {
    using Q = Quaternion<T>;
    using V = Vector<T, std::size_t, 3>;

    // Three components to take apart, like rpy().
    static_assert(std::is_same_v<decltype(Q::identity().rotationVector()), std::tuple<T, T, T>>);

    const T pi{std::numbers::pi_v<T>};
    // |axis| = 3
    const V axis(T(1.0), T(2.0), T(-2.0));
    const V u{axis / T(3.0)};

    SUBCASE("angles within [0, pi] read back as given") {
        const Q q{Q::fromAxisAngle(axis, T(1.2))};
        const T s{std::sin(T(0.6))};
        CHECK(q.identicalTo(Q(s * u.x(), s * u.y(), s * u.z(), std::cos(T(0.6))), kTol<T>));
        CHECK(q.isUnit());
        CHECK(q.axis().identicalTo(u, kTol<T>));
        CHECK_EQ(q.angle(), doctest::Approx(1.2).epsilon(kTol<T>));
        checkClose(q.rotationVector(), T(1.2) * u.x(), T(1.2) * u.y(), T(1.2) * u.z());
        CHECK(
            Q::fromRotationVector(T(1.2) * u.x(), T(1.2) * u.y(), T(1.2) * u.z())
                .identicalTo(q, kTol<T>)
        );
    }

    SUBCASE("angles beyond pi keep w < 0 and read back as the shorter turn") {
        const Q q{Q::fromAxisAngle(axis, T(4.0))};
        CHECK_LT(q.w(), T(0.0));
        CHECK(q.axis().identicalTo(-u, kTol<T>));
        CHECK_EQ(q.angle(), doctest::Approx(2 * pi - 4.0).epsilon(kTol<T>));
        const T shorter{T(2.0) * pi - T(4.0)};
        checkClose(q.rotationVector(), -shorter * u.x(), -shorter * u.y(), -shorter * u.z());
        CHECK(
            Q::fromRotationVector(T(4.0) * u.x(), T(4.0) * u.y(), T(4.0) * u.z())
                .identicalTo(q, kTol<T>)
        );
        // The rotation vector maps back to -q, the same rotation, and the tuple feeds
        // fromRotationVector as it is.
        CHECK(std::apply(&Q::fromRotationVector, q.rotationVector()).identicalTo(-q, kTol<T>));
    }

    SUBCASE("half a turn") {
        const Q q{Q::fromAxisAngle(axis, pi)};
        CHECK_EQ(q.angle(), doctest::Approx(pi).epsilon(kTol<T>));
        const auto [rx, ry, rz] = q.rotationVector();
        CHECK_EQ(std::hypot(rx, ry, rz), doctest::Approx(pi).epsilon(kTol<T>));
    }

    SUBCASE("the identity and tiny rotations") {
        CHECK_EQ(Q::identity().axis(), V(T(1.0), T(0.0), T(0.0)));
        CHECK_EQ(Q::identity().angle(), T(0.0));
        CHECK_EQ(Q::identity().rotationVector(), std::tuple<T, T, T>{});
        CHECK_EQ(Q::fromRotationVector(T(0.0), T(0.0), T(0.0)), Q::identity());
        // Far below eps and above it, the rotation vector survives the round trip to relative
        // rounding.
        for (const T scale : {T(1E-30), T(1E-6)}) {
            const T rx{scale * u.x()};
            const T ry{scale * u.y()};
            const T rz{scale * u.z()};
            const auto [rx_back, ry_back, rz_back] =
                Q::fromRotationVector(rx, ry, rz).rotationVector();
            CHECK_EQ(rx_back, doctest::Approx(rx).epsilon(kTol<T>).scale(0.0));
            CHECK_EQ(ry_back, doctest::Approx(ry).epsilon(kTol<T>).scale(0.0));
            CHECK_EQ(rz_back, doctest::Approx(rz).epsilon(kTol<T>).scale(0.0));
        }
    }

    SUBCASE("the norm does not matter") {
        const Q q{Q::fromAxisAngle(axis, T(1.2))};
        const Q scaled{q * T(3.0)};
        CHECK(scaled.axis().identicalTo(q.axis(), kTol<T>));
        CHECK_EQ(scaled.angle(), doctest::Approx(q.angle()).epsilon(kTol<T>));
        const auto [rx, ry, rz] = q.rotationVector();
        checkClose(scaled.rotationVector(), rx, ry, rz);
        const auto [roll, pitch, yaw] = q.rpy();
        checkClose(scaled.rpy(), roll, pitch, yaw);
        CHECK_EQ(scaled.angularDistance(q), doctest::Approx(0.0).epsilon(kTol<T>));
    }

    SUBCASE("a zero axis has no direction") {
        const V zero(T(0.0), T(0.0), T(0.0));
#if BASJOO_CHECK_PARAMS == 1
        CHECK_THROWS_AS(std::ignore = Q::fromAxisAngle(zero, T(1.0)), std::invalid_argument);
#else
        CHECK(std::isnan(Q::fromAxisAngle(zero, T(1.0)).x()));
#endif
    }
}

TEST_CASE_TEMPLATE("Quaternion roll, pitch and yaw follow REP-103", T, float, double) {
    using Q = Quaternion<T>;
    using V = Vector<T, std::size_t, 3>;
    using M = Matrix<T, std::size_t, 3, 3>;

    // Three angles to take apart, not a vector.
    static_assert(std::is_same_v<decltype(Q::identity().rpy()), std::tuple<T, T, T>>);

    const T pi{std::numbers::pi_v<T>};
    const T zero(0.0);
    const T one(1.0);
    const auto rx = [&](T a) -> M {
        return M(one, zero, zero, zero, std::cos(a), -std::sin(a), zero, std::sin(a), std::cos(a));
    };
    const auto ry = [&](T a) -> M {
        return M(std::cos(a), zero, std::sin(a), zero, one, zero, -std::sin(a), zero, std::cos(a));
    };
    const auto rz = [&](T a) -> M {
        return M(std::cos(a), -std::sin(a), zero, std::sin(a), std::cos(a), zero, zero, zero, one);
    };

    SUBCASE("fromRpy turns about the fixed x, then y, then z axes") {
        for (const auto& [roll, pitch, yaw] :
             {std::array<T, 3>{T(0.3), T(-0.2), T(1.1)}, std::array<T, 3>{T(-2.5), T(1.2), T(-3.0)},
              std::array<T, 3>{T(3.0), T(-1.5), T(0.4)}}) {
            const Q q{Q::fromRpy(roll, pitch, yaw)};
            const Q turns{
                Q::fromAxisAngle(V(zero, zero, one), yaw) *
                Q::fromAxisAngle(V(zero, one, zero), pitch) *
                Q::fromAxisAngle(V(one, zero, zero), roll)
            };
            CHECK(q.identicalTo(turns, kTol<T>));
            CHECK(M::fromQuaternion(q).identicalTo(rz(yaw).dot(ry(pitch)).dot(rx(roll)), kTol<T>));
        }
    }

    SUBCASE("rpy reads the angles back within their ranges") {
        for (const T roll : {T(-3.0), T(-1.2), T(0.0), T(0.7), T(2.9)}) {
            for (const T pitch : {T(-1.5), T(-0.4), T(0.0), T(0.9), T(1.5)}) {
                for (const T yaw : {T(-3.1), T(-0.6), T(0.0), T(1.3), T(3.0)}) {
                    const auto [roll_back, pitch_back, yaw_back] =
                        Q::fromRpy(roll, pitch, yaw).rpy();
                    CHECK_EQ(roll_back, doctest::Approx(roll).epsilon(kTol<T>));
                    CHECK_EQ(pitch_back, doctest::Approx(pitch).epsilon(kTol<T>));
                    CHECK_EQ(yaw_back, doctest::Approx(yaw).epsilon(kTol<T>));
                }
            }
        }
        // A pitch beyond pi / 2 comes back as the equivalent angles within the ranges.
        const Q q{Q::fromRpy(T(0.5), T(2.0), T(0.25))};
        const auto [roll, pitch, yaw] = q.rpy();
        CHECK_LE(std::abs(pitch), pi / 2);
        CHECK_LT(Q::fromRpy(roll, pitch, yaw).angularDistance(q), kTol<T>);
        // The tuple feeds fromRpy back as it is.
        CHECK_EQ(std::apply(&Q::fromRpy, q.rpy()), Q::fromRpy(roll, pitch, yaw));
    }

    SUBCASE("at gimbal lock yaw is 0 and roll takes the combined angle") {
        for (const T sign : {T(1.0), T(-1.0)}) {
            const Q q{Q::fromRpy(T(0.5), sign * pi / 2, T(0.3))};
            const auto [roll, pitch, yaw] = q.rpy();
            CHECK_EQ(yaw, T(0.0));
            CHECK_EQ(pitch, doctest::Approx(sign * pi / 2).epsilon(kTol<T>));
            CHECK_EQ(roll, doctest::Approx(T(0.5) - sign * T(0.3)).epsilon(kTol<T>));
            CHECK(
                M::fromQuaternion(Q::fromRpy(roll, pitch, yaw))
                    .identicalTo(M::fromQuaternion(q), kTol<T>)
            );
        }
    }

    SUBCASE("next to gimbal lock the angles still rebuild the rotation") {
        const T offset{std::is_same_v<T, float> ? T(1E-3) : T(1E-6)};
        for (const T near_lock : {pi / 2 - offset, -pi / 2 + offset}) {
            const Q q{Q::fromRpy(T(0.5), near_lock, T(0.3))};
            const auto [roll, pitch, yaw] = q.rpy();
            CHECK(
                M::fromQuaternion(Q::fromRpy(roll, pitch, yaw))
                    .identicalTo(M::fromQuaternion(q), kTol<T>)
            );
        }
    }
}

TEST_CASE_TEMPLATE("Quaternion from two vectors matches Mat33::fromTwoVectors", T, float, double) {
    using Q = Quaternion<T>;
    using V = Vector<T, std::size_t, 3>;
    using M = Matrix<T, std::size_t, 3, 3>;

    const auto check_maps = [](const V& from, const V& to) -> Q {
        const Q q{Q::fromTwoVectors(from, to)};
        CHECK(q.isUnit());
        CHECK_GE(q.w(), T(0.0));
        CHECK(q.rotate(from.normalized()).identicalTo(to.normalized(), kTol<T>));
        CHECK(M::fromQuaternion(q).identicalTo(M::fromTwoVectors(from, to), kTol<T>));
        return q;
    };

    SUBCASE("generic directions, minimal rotation") {
        const V from(T(1.0), T(2.0), T(-0.5));
        const V to(T(-3.0), T(0.25), T(1.5));
        const Q q{check_maps(from, to)};
        CHECK(q.axis().identicalTo(from.cross(to).normalized(), kTol<T>));
        CHECK_EQ(
            q.angle(),
            doctest::Approx(std::acos(from.normalized().dot(to.normalized()))).epsilon(kTol<T>)
        );
        // Lengths do not matter.
        CHECK(Q::fromTwoVectors(from * T(4.0), to * T(0.125)).identicalTo(q, kTol<T>));
    }

    SUBCASE("parallel directions give the identity") {
        const V from(T(0.5), T(-1.0), T(2.0));
        CHECK(check_maps(from, from * T(3.0)).identicalTo(Q::identity(), kTol<T>));
    }

    SUBCASE("antiparallel directions give half a turn") {
        for (const V& from :
             {V(T(1.0), T(0.0), T(0.0)), V(T(0.0), T(-2.0), T(0.0)), V(T(0.0), T(0.0), T(3.0)),
              V(T(1.0), T(2.0), T(3.0)), V(T(-0.5), T(0.25), T(-4.0))}) {
            const Q q{check_maps(from, from * T(-2.0))};
            CHECK_EQ(q.w(), T(0.0));
            CHECK_EQ(q.axis().dot(from.normalized()), doctest::Approx(0.0).epsilon(kTol<T>));
        }
    }

    SUBCASE("a zero vector has no direction") {
        const V from(T(1.0), T(2.0), T(3.0));
        const V zero(T(0.0), T(0.0), T(0.0));
#if BASJOO_CHECK_PARAMS == 1
        CHECK_THROWS_AS(std::ignore = Q::fromTwoVectors(zero, from), std::invalid_argument);
        CHECK_THROWS_AS(std::ignore = Q::fromTwoVectors(from, zero), std::invalid_argument);
#else
        CHECK(std::isnan(Q::fromTwoVectors(zero, from).w()));
        CHECK(std::isnan(Q::fromTwoVectors(from, zero).w()));
#endif
    }
}

TEST_CASE_TEMPLATE("Quaternion slerp and angular distance", T, float, double) {
    using Q = Quaternion<T>;
    using V = Vector<T, std::size_t, 3>;

    const V axis{V(T(1.0), T(-2.0), T(0.5)).normalized()};
    const Q q{Q::fromRpy(T(0.3), T(-0.2), T(1.1))};
    // 1.4 away from q.
    const Q p{Q::fromAxisAngle(axis, T(1.4)) * q};

    SUBCASE("the angular distance is the angle of the rotation between") {
        CHECK_EQ(q.angularDistance(p), doctest::Approx(1.4).epsilon(kTol<T>));
        CHECK_EQ(p.angularDistance(q), doctest::Approx(1.4).epsilon(kTol<T>));
        CHECK_EQ(q.angularDistance(-p), doctest::Approx(1.4).epsilon(kTol<T>));
        CHECK_EQ(q.angularDistance(-q), doctest::Approx(0.0).epsilon(kTol<T>));
    }

    SUBCASE("slerp moves at a constant rate along the shorter arc") {
        CHECK(slerp(q, p, T(0.0)).identicalTo(q, kTol<T>));
        CHECK(slerp(q, p, T(1.0)).identicalTo(p, kTol<T>));
        for (const T ratio : {T(0.25), T(0.5), T(0.75)}) {
            const Q s{slerp(q, p, ratio)};
            CHECK(s.isUnit());
            CHECK_EQ(q.angularDistance(s), doctest::Approx(ratio * 1.4).epsilon(kTol<T>));
            CHECK_EQ(s.angularDistance(p), doctest::Approx((1 - ratio) * 1.4).epsilon(kTol<T>));
            CHECK(s.identicalTo(Q::fromAxisAngle(axis, ratio * T(1.4)) * q, kTol<T>));
            // -p is the same rotation, reached along the same path.
            CHECK(slerp(q, -p, ratio).identicalTo(s, kTol<T>));
        }
        // Like lerp(), the ratio is any floating-point type.
        CHECK(slerp(q, p, 0.25).identicalTo(slerp(q, p, T(0.25)), kTol<T>));
    }

    SUBCASE("nearly equal rotations are interpolated linearly") {
        const T tiny{std::is_same_v<T, float> ? T(1E-4) : T(1E-9)};
        const Q nearby{Q::fromAxisAngle(axis, tiny) * q};
        const Q s{slerp(q, nearby, T(0.5))};
        CHECK(s.isUnit());
        CHECK_EQ(q.angularDistance(s), doctest::Approx(T(0.5) * tiny).epsilon(kTol<T>));
        CHECK_EQ(s.angularDistance(nearby), doctest::Approx(T(0.5) * tiny).epsilon(kTol<T>));
        CHECK(slerp(q, q, T(0.5)).identicalTo(q, kTol<T>));
    }

#if BASJOO_CHECK_PARAMS == 1
    SUBCASE("slerp takes unit quaternions only") {
        CHECK_THROWS_AS(std::ignore = slerp(q * T(2.0), p, T(0.5)), std::invalid_argument);
        CHECK_THROWS_AS(std::ignore = slerp(q, p * T(2.0), T(0.5)), std::invalid_argument);
    }
#endif
}

TEST_CASE_TEMPLATE("Quaternion serializes x, y, z, w", T, float, double) {
    using Q = Quaternion<T>;

    const Q a(T(0.5), T(-1.0), T(2.0), T(1.5));

    auto [data, out] = zpp::bits::data_out();
    out(a).or_throw();
    CHECK_EQ(data.size(), 4 * sizeof(T));

    auto [raw, raw_out] = zpp::bits::data_out();
    raw_out(std::array<T, 4>{a.x(), a.y(), a.z(), a.w()}).or_throw();
    CHECK_EQ(data, raw);

    Q b{};
    auto in = zpp::bits::in(data);
    in(b).or_throw();
    CHECK_EQ(b, a);
}

TEST_CASE_TEMPLATE("Quaternion accessors are references into the storage", T, float, double) {
    using Q = Quaternion<T>;
    static_assert(sizeof(Q) == 4 * sizeof(T));
    Q q{T(0.0), T(0.0), T(0.0), T(1.0)};
    q.x() = T(0.5);
    q.w() -= T(0.5);
    CHECK_EQ(q, Q{T(0.5), T(0.0), T(0.0), T(0.5)});
    CHECK_EQ(&std::as_const(q).w(), &q.x() + 3);
}

// Array storage lets the element-wise arithmetic and the Hamilton product run in constant
// expressions.
consteval auto constantQuaternion() -> bool {
    using Q = Quaternion<double>;
    Q q{0.0, 0.0, 1.0, 0.0};
    q += Q{0.0, 0.0, 0.0, 1.0};
    q *= 2.0;
    const Q r{q * q};
    return q.w() == 2.0 && q.z() == 2.0 && r == Q{0.0, 0.0, 8.0, 0.0} && q.euclideanSqr() == 8.0;
}
static_assert(constantQuaternion());

} // namespace basjoo::math
