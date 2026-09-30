/**
 * @file math/dense/test_dense_lanes.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2026-11-03
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/dense/dense_lanes.hpp"

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <vector>

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/detail/dense_norm_trait.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::math {

namespace {

template <typename T>
inline constexpr bool kComplex{ComplexArithmetic<T>};

template <typename T>
using real_of = detail::DenseNormTraitT<T>;

// Every tail of every register width, up to 16 lanes, and a few long sweeps; 47 and 95 follow the
// four-register rounds with full registers and one block of each narrower width.
constexpr std::array<std::size_t, 20> kLengths{0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
                                               15, 16, 17, 31, 32, 33, 47, 64, 67, 95};

// Small values, exact in every scalar type, that differ between seeds.
template <typename T>
auto value(std::size_t k, std::size_t seed) -> T {
    const auto re{static_cast<real_of<T>>(static_cast<int>((k + seed) % 9) - 4)};
    if constexpr (kComplex<T>) {
        const auto im{static_cast<real_of<T>>(static_cast<int>((3 * k + seed) % 7) - 3)};
        return T{re * real_of<T>(0.5), im * real_of<T>(0.25)};
    } else if constexpr (std::is_floating_point_v<T>) {
        return re * T(0.5);
    } else {
        return re;
    }
}

// n values behind one leading slot, so that the data starts off any register alignment.
template <typename T>
auto sample(std::size_t n, std::size_t seed) -> std::vector<T> {
    std::vector<T> v(n + 1);
    for (std::size_t k{0}; k < n; ++k) {
        v[k + 1] = value<T>(k, seed);
    }
    return v;
}

template <typename T>
auto checkClose(const T& actual, const T& expected) -> void {
    if constexpr (kComplex<T>) {
        CHECK_EQ(
            static_cast<double>(actual.real()),
            doctest::Approx(static_cast<double>(expected.real())).epsilon(1E-6)
        );
        CHECK_EQ(
            static_cast<double>(actual.imag()),
            doctest::Approx(static_cast<double>(expected.imag())).epsilon(1E-6)
        );
    } else if constexpr (std::is_floating_point_v<T>) {
        CHECK_EQ(
            static_cast<double>(actual),
            doctest::Approx(static_cast<double>(expected)).epsilon(1E-6)
        );
    } else {
        CHECK_EQ(actual, expected);
    }
}

// v times c as the complex kernels compute it, (a re - b im, b re + a im).
template <typename T>
auto times(const T& v, const T& c) -> T {
    if constexpr (kComplex<T>) {
        return T{
            v.real() * c.real() - v.imag() * c.imag(), v.imag() * c.real() + v.real() * c.imag()
        };
    } else {
        return static_cast<T>(v * c);
    }
}

} // namespace

// clang-format off
#define BASJOO_DENSE_LANES_SCALARS                                                                 \
    double, float, std::complex<double>, std::complex<float>, std::int64_t, std::uint64_t,          \
    std::int32_t, std::uint32_t, long double, std::int16_t, std::complex<long double>
// clang-format on

TEST_CASE_TEMPLATE(
    "DenseLanes element-wise kernels agree with scalar code at every length", T,
    BASJOO_DENSE_LANES_SCALARS
) {
    using K = DenseLanes<T>;

    for (const std::size_t n : kLengths) {
        CAPTURE(n);
        const std::vector<T> x{sample<T>(n, 1)};
        const std::vector<T> y{sample<T>(n, 4)};
        const T* const px{x.data() + 1};
        const T* const py{y.data() + 1};

        std::vector<T> z(n + 1);
        K::add(n, px, py, z.data() + 1);
        for (std::size_t k{0}; k < n; ++k) {
            CHECK_EQ(z[k + 1], static_cast<T>(px[k] + py[k]));
        }
        // The output may be either input.
        std::vector<T> a{x};
        K::add(n, a.data() + 1, py, a.data() + 1);
        CHECK_EQ(a, z);

        K::subtract(n, px, py, z.data() + 1);
        for (std::size_t k{0}; k < n; ++k) {
            CHECK_EQ(z[k + 1], static_cast<T>(px[k] - py[k]));
        }
        std::vector<T> b{y};
        K::subtract(n, px, b.data() + 1, b.data() + 1);
        CHECK_EQ(b, z);

        std::vector<T> c{x};
        K::negate(n, c.data() + 1);
        for (std::size_t k{0}; k < n; ++k) {
            CHECK_EQ(c[k + 1], static_cast<T>(-px[k]));
        }

        const real_of<T> alpha(3);
        std::vector<T> d{x};
        K::scale(n, alpha, d.data() + 1);
        for (std::size_t k{0}; k < n; ++k) {
            CHECK_EQ(d[k + 1], static_cast<T>(px[k] * alpha));
        }

        if constexpr (kComplex<T>) {
            const T beta{real_of<T>(0.75), real_of<T>(-1.25)};
            std::vector<T> e{x};
            K::scale(n, beta, e.data() + 1);
            for (std::size_t k{0}; k < n; ++k) {
                checkClose(e[k + 1], times(px[k], beta));
            }

            std::vector<T> f{x};
            K::conjugate(n, f.data() + 1);
            for (std::size_t k{0}; k < n; ++k) {
                CHECK_EQ(f[k + 1], std::conj(px[k]));
            }
        }
    }
}

TEST_CASE_TEMPLATE(
    "DenseLanes equal finds a difference in any element, NaN included", T,
    BASJOO_DENSE_LANES_SCALARS
) {
    using K = DenseLanes<T>;

    for (const std::size_t n : kLengths) {
        CAPTURE(n);
        const std::vector<T> x{sample<T>(n, 2)};
        CHECK(K::equal(n, x.data() + 1, x.data() + 1));
        for (std::size_t k{0}; k < n; ++k) {
            std::vector<T> y{x};
            y[k + 1] = static_cast<T>(y[k + 1] + T(1));
            CHECK_FALSE(K::equal(n, x.data() + 1, y.data() + 1));
        }
        if constexpr (std::is_floating_point_v<real_of<T>>) {
            if (n > 0) {
                std::vector<T> nan{x};
                nan[n] = T(std::numeric_limits<real_of<T>>::quiet_NaN());
                CHECK_FALSE(K::equal(n, nan.data() + 1, nan.data() + 1));
            }
        }
    }
}

#undef BASJOO_DENSE_LANES_SCALARS

// The scalar code is what runs in constant evaluation.
consteval auto constantReal() -> bool {
    std::array<double, 5> x{1.0, 2.0, 3.0, 4.0, 5.0};
    const std::array<double, 5> y{0.5, 0.5, 0.5, 0.5, 0.5};
    std::array<double, 5> z{};
    DenseLanes<double>::add(5, x.data(), y.data(), z.data());
    DenseLanes<double>::scale(5, 2.0, z.data());
    DenseLanes<double>::negate(5, z.data());
    DenseLanes<double>::subtract(5, x.data(), z.data(), x.data());
    return z[4] == -11.0 && x[0] == 4.0 && DenseLanes<double>::equal(5, z.data(), z.data()) &&
           !DenseLanes<double>::equal(5, x.data(), z.data());
}
static_assert(constantReal());

consteval auto constantComplex() -> bool {
    using C = std::complex<double>;
    std::array<C, 2> x{C{1.0, 2.0}, C{-1.0, 0.5}};
    DenseLanes<C>::scale(2, C{0.0, 1.0}, x.data());
    DenseLanes<C>::conjugate(2, x.data());
    return x[0] == C{-2.0, -1.0} && x[1] == C{-0.5, 1.0};
}
static_assert(constantComplex());

consteval auto constantUnsigned() -> bool {
    std::array<std::uint32_t, 3> x{1U, 2U, 3U};
    DenseLanes<std::uint32_t>::negate(3, x.data());
    DenseLanes<std::uint32_t>::scale(3, 2U, x.data());
    // Wrapping arithmetic: 2 (2^32 - k) = 2^32 - 2 k modulo 2^32.
    return x[0] == 0xFFFF'FFFEU && x[2] == 0xFFFF'FFFAU;
}
static_assert(constantUnsigned());

} // namespace basjoo::math
