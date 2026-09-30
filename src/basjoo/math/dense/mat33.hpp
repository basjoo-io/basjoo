/**
 * @file math/dense/mat33.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief 3x3 matrix with padded SIMD storage and spatial rotations.
 * @version 0.1
 * @date 2026-11-03
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <array>
#include <bit>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mdspan>
#include <stdexcept>
#include <type_traits>
#include <utility>

#if defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>
#elif defined(__aarch64__) || defined(_M_ARM64)
#include <arm_neon.h>
#endif

#include "zpp_bits.h"

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_trait.hpp"
#include "basjoo/math/dense/detail/dense_norm_trait.hpp"
#include "basjoo/math/dense/matrix_base.hpp"
#include "basjoo/math/dense/vec3.hpp"
#include "basjoo/math/type_traits.hpp"

namespace basjoo::math {

// Defined in basjoo/math/dense/quaternion.hpp, which includes this header.
template <std::floating_point Scalar>
class Quaternion;

// Three stored vectors of four scalars, the columns under layout_left and the rows under
// layout_right, each padded after its third element so that it fills a SIMD register. The padding
// (storage offsets 3, 7 and 11) holds zeros that users must not change: the single-index access
// reaches it, size() counts it, and the element-wise operations and serialization go over it.
template <ScalarArithmetic Scalar, std::integral Index, DenseLayout Layout, Allocatory Alloc>
class alignas(4 * sizeof(Scalar)) Matrix<Scalar, Index, 3, 3, Layout, Alloc> final
    : public MatrixBase<false> {
  public:
    using element_type = typename DenseTrait<Matrix>::element_type;
    using value_type = typename DenseTrait<Matrix>::value_type;
    using pointer = typename DenseTrait<Matrix>::pointer;
    using const_pointer = typename DenseTrait<Matrix>::const_pointer;
    using reference = typename DenseTrait<Matrix>::reference;
    using const_reference = typename DenseTrait<Matrix>::const_reference;
    using extents_type = typename DenseTrait<Matrix>::extents_type;
    // Still names the storage order, which LAPACK callers read; the padding makes the mapping
    // strided rather than the exhaustive one of Layout.
    using layout_type = typename DenseTrait<Matrix>::layout_type;
    using allocator_type = typename DenseTrait<Matrix>::allocator_type;
    using size_type = typename DenseTrait<Matrix>::size_type;
    using difference_type = typename DenseTrait<Matrix>::difference_type;
    using index_type = typename DenseTrait<Matrix>::index_type;
    using rank_type = typename DenseTrait<Matrix>::rank_type;
    using mapping_type = std::layout_stride::mapping<extents_type>;
    using storage_type = std::array<Scalar, 12>;

    [[using gnu: always_inline]]
    constexpr Matrix() noexcept = default;
    [[using gnu: always_inline]]
    constexpr Matrix(const Matrix& other) noexcept = default;
    [[using gnu: always_inline]]
    constexpr auto operator=(const Matrix& other) noexcept -> Matrix& = default;
    [[using gnu: always_inline]]
    constexpr Matrix(Matrix&& other) noexcept = default;
    [[using gnu: always_inline]]
    constexpr auto operator=(Matrix&& other) noexcept -> Matrix& = default;
    constexpr ~Matrix() noexcept = default;

    // The elements in row-major reading order, whatever the storage layout: one parameter each.
    // NOLINTBEGIN(readability-function-size)
    [[using gnu: always_inline]]
    constexpr Matrix(
        value_type m00, value_type m01, value_type m02, value_type m10, value_type m11,
        value_type m12, value_type m20, value_type m21, value_type m22
    ) noexcept {
        const value_type pad(0.0);
        if constexpr (std::is_same_v<layout_type, std::layout_left>) {
            m_data = storage_type{m00, m10, m20, pad, m01, m11, m21, pad, m02, m12, m22, pad};
        } else {
            m_data = storage_type{m00, m01, m02, pad, m10, m11, m12, pad, m20, m21, m22, pad};
        }
    }
    // NOLINTEND(readability-function-size)

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto get_allocator() noexcept -> allocator_type {
        return allocator_type{};
    }

    [[using gnu: always_inline]]
    constexpr explicit Matrix([[maybe_unused]] const allocator_type& alloc) noexcept {}

    [[using gnu: always_inline]]
    constexpr explicit Matrix(
        [[maybe_unused]] index_type rows, [[maybe_unused]] index_type cols,
        [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS) {
#if BASJOO_CHECK_PARAMS == 1
        if (rows != nrows() || cols != ncols()) [[unlikely]] {
            throw std::invalid_argument("Matrix constructor: mismatch matrix sizes detected.");
        }
#endif
    }

    [[using gnu: always_inline]]
    constexpr explicit Matrix(
        [[maybe_unused]] index_type rows, [[maybe_unused]] index_type cols, const_reference value,
        [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS) {
#if BASJOO_CHECK_PARAMS == 1
        if (rows != nrows() || cols != ncols()) [[unlikely]] {
            throw std::invalid_argument("Matrix constructor: mismatch matrix sizes detected.");
        }
#endif
        fill(value);
    }

    template <typename T>
        requires std::same_as<std::remove_cv_t<typename T::value_type>, value_type> &&
                 (T::rank() == 2)
    [[using gnu: always_inline]] constexpr explicit Matrix(
        const T& matrix_like, [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS) {
        using other_index_type = typename T::index_type;

#if BASJOO_CHECK_PARAMS == 1
        if (std::cmp_not_equal(matrix_like.extent(0), nrows()) ||
            std::cmp_not_equal(matrix_like.extent(1), ncols())) [[unlikely]] {
            throw std::invalid_argument("Matrix constructor: mismatch matrix sizes detected.");
        }
#endif
        operator[](0, 0) = matrix_like[other_index_type{0}, other_index_type{0}];
        operator[](0, 1) = matrix_like[other_index_type{0}, other_index_type{1}];
        operator[](0, 2) = matrix_like[other_index_type{0}, other_index_type{2}];
        operator[](1, 0) = matrix_like[other_index_type{1}, other_index_type{0}];
        operator[](1, 1) = matrix_like[other_index_type{1}, other_index_type{1}];
        operator[](1, 2) = matrix_like[other_index_type{1}, other_index_type{2}];
        operator[](2, 0) = matrix_like[other_index_type{2}, other_index_type{0}];
        operator[](2, 1) = matrix_like[other_index_type{2}, other_index_type{1}];
        operator[](2, 2) = matrix_like[other_index_type{2}, other_index_type{2}];
    }

    template <ScalarArithmetic U>
    [[using gnu: always_inline]]
    constexpr explicit Matrix(const Matrix<U, Index, 3, 3, Layout>& other) noexcept {
        // All twelve slots, the zero padding included.
        for (std::size_t k{0}; k < 12; ++k) {
            m_data[k] = static_cast<value_type>(other.data()[k]);
        }
    }

    [[using gnu: const, always_inline]]
    static constexpr auto identity() noexcept -> Matrix {
        return Matrix{value_type(1.0), value_type(0.0), value_type(0.0),
                      value_type(0.0), value_type(1.0), value_type(0.0),
                      value_type(0.0), value_type(0.0), value_type(1.0)};
    }

    [[using gnu: pure, always_inline]]
    static constexpr auto fromColumns(
        const Vector<value_type, Index, 3, Alloc>& c0,
        const Vector<value_type, Index, 3, Alloc>& c1, const Vector<value_type, Index, 3, Alloc>& c2
    ) noexcept -> Matrix {
        return Matrix{c0.x(), c1.x(), c2.x(), c0.y(), c1.y(), c2.y(), c0.z(), c1.z(), c2.z()};
    }

    [[using gnu: pure, always_inline]]
    static constexpr auto fromRows(
        const Vector<value_type, Index, 3, Alloc>& r0,
        const Vector<value_type, Index, 3, Alloc>& r1, const Vector<value_type, Index, 3, Alloc>& r2
    ) noexcept -> Matrix {
        return Matrix{r0.x(), r0.y(), r0.z(), r1.x(), r1.y(), r1.z(), r2.x(), r2.y(), r2.z()};
    }

    // The minimal rotation turning the direction of `from` into that of `to`; neither needs to be
    // of unit length. A zero vector throws when parameters are checked and yields NaNs otherwise.
    // Not gnu::pure: GCC deletes an unused call to a pure function together with its exception.
    static constexpr auto fromTwoVectors(
        const Vector<value_type, Index, 3, Alloc>& from,
        const Vector<value_type, Index, 3, Alloc>& to
    ) noexcept(!BASJOO_CHECK_PARAMS) -> Matrix
        requires std::floating_point<value_type>
    {
        using vector_type = Vector<value_type, Index, 3, Alloc>;

#if BASJOO_CHECK_PARAMS == 1
        if (from.euclideanSqr() == value_type(0.0) || to.euclideanSqr() == value_type(0.0))
            [[unlikely]] {
            throw std::invalid_argument("Matrix::fromTwoVectors: zero-length vector.");
        }
#endif
        const vector_type a{from.normalized()};
        const vector_type b{to.normalized()};
        // The quaternion (1 + a.b, a x b), once normalized, rotates a onto b.
        const value_type w{value_type(1.0) + a.dot(b)};
        if (w <= std::numeric_limits<value_type>::epsilon()) {
            // Antiparallel: half a turn about an axis perpendicular to `from`, its cross product
            // with the coordinate axis least aligned with it. R = 2 u u^T - I.
            const value_type ax{std::abs(a.x())};
            const value_type ay{std::abs(a.y())};
            const value_type az{std::abs(a.z())};
            const value_type one(1.0);
            const value_type zero(0.0);
            const vector_type axis{
                ax <= ay && ax <= az ? vector_type{one, zero, zero}
                : ay <= az           ? vector_type{zero, one, zero}
                                     : vector_type{zero, zero, one}
            };
            const vector_type u{a.cross(axis).normalized()};
            const value_type two(2.0);
            return Matrix{two * u.x() * u.x() - one, two * u.x() * u.y(),
                          two * u.x() * u.z(),       two * u.y() * u.x(),
                          two * u.y() * u.y() - one, two * u.y() * u.z(),
                          two * u.z() * u.x(),       two * u.z() * u.y(),
                          two * u.z() * u.z() - one};
        }
        const vector_type v{a.cross(b)};
        const value_type n{std::sqrt(w * w + v.euclideanSqr())};
        const value_type qw{w / n};
        const value_type qx{v.x() / n};
        const value_type qy{v.y() / n};
        const value_type qz{v.z() / n};
        const value_type one(1.0);
        const value_type two(2.0);
        return Matrix{one - two * (qy * qy + qz * qz), two * (qx * qy - qw * qz),
                      two * (qx * qz + qw * qy),       two * (qx * qy + qw * qz),
                      one - two * (qx * qx + qz * qz), two * (qy * qz - qw * qx),
                      two * (qx * qz - qw * qy),       two * (qy * qz + qw * qx),
                      one - two * (qx * qx + qy * qy)};
    }

    // The rotation of a unit quaternion. A quaternion off unit norm throws when parameters are
    // checked and otherwise yields (1 - |q|^2) I + |q|^2 R, which is no rotation. A template, so
    // that a caller holding a quaternion has included basjoo/math/dense/quaternion.hpp by the time
    // it is instantiated.
    // Not gnu::pure: GCC deletes an unused call to a pure function together with its exception.
    template <std::floating_point Real>
        requires std::same_as<Real, value_type>
    [[using gnu: always_inline]]
    static constexpr auto fromQuaternion(const Quaternion<Real>& q) noexcept(!BASJOO_CHECK_PARAMS)
        -> Matrix {
#if BASJOO_CHECK_PARAMS == 1
        if (!q.isUnit()) [[unlikely]] {
            throw std::invalid_argument("Matrix::fromQuaternion: not a unit quaternion.");
        }
#endif
        const value_type one(1.0);
        const value_type tx{q.x() + q.x()};
        const value_type ty{q.y() + q.y()};
        const value_type tz{q.z() + q.z()};
        const value_type twx{tx * q.w()};
        const value_type twy{ty * q.w()};
        const value_type twz{tz * q.w()};
        const value_type txx{tx * q.x()};
        const value_type txy{ty * q.x()};
        const value_type txz{tz * q.x()};
        const value_type tyy{ty * q.y()};
        const value_type tyz{tz * q.y()};
        const value_type tzz{tz * q.z()};
        return Matrix{one - (tyy + tzz), txy - twz, txz + twy, txy + twz,        one - (txx + tzz),
                      tyz - twx,         txz - twy, tyz + twx, one - (txx + tyy)};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::mdspan<value_type, extents_type, std::layout_stride>() noexcept {
        return {data(), mapping()};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::mdspan<
        const value_type, extents_type, std::layout_stride>() const noexcept {
        return {data(), mapping()};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::mdspan<
        value_type, std::dextents<index_type, 2>, std::layout_stride>() noexcept {
        return {
            data(), std::layout_stride::mapping<std::dextents<index_type, 2>>{
                        std::dextents<index_type, 2>{nrows(), ncols()},
                        std::array<index_type, 2>{stride(0), stride(1)}
                    }
        };
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::mdspan<
        const value_type, std::dextents<index_type, 2>, std::layout_stride>() const noexcept {
        return {
            data(), std::layout_stride::mapping<std::dextents<index_type, 2>>{
                        std::dextents<index_type, 2>{nrows(), ncols()},
                        std::array<index_type, 2>{stride(0), stride(1)}
                    }
        };
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto rank() noexcept -> rank_type {
        return extents_type::rank();
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto rank_dynamic() noexcept -> rank_type {
        return extents_type::rank_dynamic();
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto static_extent(rank_type r) noexcept -> std::size_t {
        return extents_type::static_extent(r);
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto extents() noexcept -> extents_type {
        return extents_type{};
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto mapping() noexcept -> mapping_type {
        return mapping_type{extents_type{}, std::array<index_type, 2>{stride(0), stride(1)}};
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto extent([[maybe_unused]] rank_type r) noexcept -> index_type {
        return 3;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto nrows() noexcept -> index_type {
        return 3;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto ncols() noexcept -> index_type {
        return 3;
    }

    [[using gnu: const, always_inline, leaf]]
    // The storage footprint, padding included; the element count is nrows() * ncols().
    static constexpr auto size() noexcept -> size_type {
        return 12;
    }

    // One stride is 1 and the other the padded length 4, so their product is the leading
    // dimension whichever layout is in use.
    [[using gnu: const, always_inline, leaf]]
    static constexpr auto stride(rank_type r) noexcept -> index_type {
        return (r == 0) == std::is_same_v<layout_type, std::layout_left> ? 1 : 4;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto empty() noexcept -> bool {
        return false;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto is_unique() noexcept -> bool {
        return true;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto is_exhaustive() noexcept -> bool {
        return false;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto is_strided() noexcept -> bool {
        return true;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto is_always_unique() noexcept -> bool {
        return true;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto is_always_exhaustive() noexcept -> bool {
        return false;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto is_always_strided() noexcept -> bool {
        return true;
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto data(this Self&& self) noexcept {
        return self.m_data.data();
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto data_handle(this Self&& self) noexcept {
        return self.m_data.data();
    }

    [[using gnu: always_inline]]
    constexpr auto resize([[maybe_unused]] index_type rows, [[maybe_unused]] index_type cols)
        -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (rows != nrows() || cols != ncols()) [[unlikely]] {
            throw std::invalid_argument("Matrix reshape: mismatch matrix sizes detected.");
        }
#endif
        return;
    }

    [[using gnu: always_inline]]
    constexpr auto assign(
        [[maybe_unused]] index_type rows, [[maybe_unused]] index_type cols, const_reference value
    ) -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (rows != nrows() || cols != ncols()) [[unlikely]] {
            throw std::invalid_argument("Matrix assign: mismatch matrix sizes detected.");
        }
#endif
        fill(value);
        return;
    }

    // Frobenius norm.
    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclidean() const noexcept -> detail::DenseNormTraitT<value_type> {
        return std::sqrt(euclideanSqr());
    }
    // The padding is zero, so that the twelve slots square to the nine-element sum.
    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto euclideanSqr() const noexcept -> detail::DenseNormTraitT<value_type> {
        using norm_type = detail::DenseNormTraitT<value_type>;

        if !consteval {
            if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE2__) || defined(_M_X64)
                const __m128 v0{_mm_load_ps(data())};
                const __m128 v1{_mm_load_ps(data() + 4)};
                const __m128 v2{_mm_load_ps(data() + 8)};
                const __m128 sq{_mm_add_ps(
                    _mm_add_ps(_mm_mul_ps(v0, v0), _mm_mul_ps(v1, v1)), _mm_mul_ps(v2, v2)
                )};
                const __m128 pairs{_mm_add_ps(sq, _mm_movehl_ps(sq, sq))};
                return _mm_cvtss_f32(
                    _mm_add_ss(pairs, _mm_shuffle_ps(pairs, pairs, _MM_SHUFFLE(1, 1, 1, 1)))
                );
#elif defined(__ARM_NEON)
                const float32x4_t v0{vld1q_f32(data())};
                const float32x4_t v1{vld1q_f32(data() + 4)};
                const float32x4_t v2{vld1q_f32(data() + 8)};
                return vaddvq_f32(
                    vaddq_f32(vaddq_f32(vmulq_f32(v0, v0), vmulq_f32(v1, v1)), vmulq_f32(v2, v2))
                );
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX__)
                const __m256d v0{_mm256_load_pd(data())};
                const __m256d v1{_mm256_load_pd(data() + 4)};
                const __m256d v2{_mm256_load_pd(data() + 8)};
                const __m256d sq{_mm256_add_pd(
                    _mm256_add_pd(_mm256_mul_pd(v0, v0), _mm256_mul_pd(v1, v1)),
                    _mm256_mul_pd(v2, v2)
                )};
                const __m128d pairs{
                    _mm_add_pd(_mm256_castpd256_pd128(sq), _mm256_extractf128_pd(sq, 1))
                };
                return _mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)));
#elif defined(__SSE2__) || defined(_M_X64)
                __m128d pairs{_mm_setzero_pd()};
                for (std::size_t k{0}; k < 12; k += 2) {
                    const __m128d v{_mm_load_pd(data() + k)};
                    pairs = _mm_add_pd(pairs, _mm_mul_pd(v, v));
                }
                return _mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)));
#elif defined(__ARM_NEON)
                float64x2_t pairs{vdupq_n_f64(0.0)};
                for (std::size_t k{0}; k < 12; k += 2) {
                    const float64x2_t v{vld1q_f64(data() + k)};
                    pairs = vaddq_f64(pairs, vmulq_f64(v, v));
                }
                return vaddvq_f64(pairs);
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                // The squared magnitudes sum the squares of the real lanes.
#if defined(__AVX__)
                const float* const p{reinterpret_cast<const float*>(data())};
                __m256 sq{_mm256_setzero_ps()};
                for (std::size_t k{0}; k < 24; k += 8) {
                    const __m256 v{_mm256_load_ps(p + k)};
                    sq = _mm256_add_ps(sq, _mm256_mul_ps(v, v));
                }
                const __m128 quads{
                    _mm_add_ps(_mm256_castps256_ps128(sq), _mm256_extractf128_ps(sq, 1))
                };
                const __m128 pairs{_mm_add_ps(quads, _mm_movehl_ps(quads, quads))};
                return _mm_cvtss_f32(
                    _mm_add_ss(pairs, _mm_shuffle_ps(pairs, pairs, _MM_SHUFFLE(1, 1, 1, 1)))
                );
#elif defined(__SSE2__) || defined(_M_X64)
                const float* const p{reinterpret_cast<const float*>(data())};
                __m128 quads{_mm_setzero_ps()};
                for (std::size_t k{0}; k < 24; k += 4) {
                    const __m128 v{_mm_load_ps(p + k)};
                    quads = _mm_add_ps(quads, _mm_mul_ps(v, v));
                }
                const __m128 pairs{_mm_add_ps(quads, _mm_movehl_ps(quads, quads))};
                return _mm_cvtss_f32(
                    _mm_add_ss(pairs, _mm_shuffle_ps(pairs, pairs, _MM_SHUFFLE(1, 1, 1, 1)))
                );
#elif defined(__ARM_NEON)
                const float* const p{reinterpret_cast<const float*>(data())};
                float32x4_t quads{vdupq_n_f32(0.0F)};
                for (std::size_t k{0}; k < 24; k += 4) {
                    const float32x4_t v{vld1q_f32(p + k)};
                    quads = vaddq_f32(quads, vmulq_f32(v, v));
                }
                return vaddvq_f32(quads);
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                // The squared magnitudes sum the squares of the real lanes, added as a tree rather
                // than along one chain.
#if defined(__AVX__)
                const double* const p{reinterpret_cast<const double*>(data())};
                const __m256d v0{_mm256_load_pd(p)};
                const __m256d v1{_mm256_load_pd(p + 4)};
                const __m256d v2{_mm256_load_pd(p + 8)};
                const __m256d v3{_mm256_load_pd(p + 12)};
                const __m256d v4{_mm256_load_pd(p + 16)};
                const __m256d v5{_mm256_load_pd(p + 20)};
                const __m256d sq{_mm256_add_pd(
                    _mm256_add_pd(
                        _mm256_add_pd(_mm256_mul_pd(v0, v0), _mm256_mul_pd(v1, v1)),
                        _mm256_add_pd(_mm256_mul_pd(v2, v2), _mm256_mul_pd(v3, v3))
                    ),
                    _mm256_add_pd(_mm256_mul_pd(v4, v4), _mm256_mul_pd(v5, v5))
                )};
                const __m128d pairs{
                    _mm_add_pd(_mm256_castpd256_pd128(sq), _mm256_extractf128_pd(sq, 1))
                };
                return _mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)));
#elif defined(__SSE2__) || defined(_M_X64)
                // The three elements of each stored vector, its padding left out.
                const double* const p{reinterpret_cast<const double*>(data())};
                __m128d pairs{_mm_setzero_pd()};
                for (std::size_t s{0}; s < 24; s += 8) {
                    const __m128d e0{_mm_load_pd(p + s)};
                    const __m128d e1{_mm_load_pd(p + s + 2)};
                    const __m128d e2{_mm_load_pd(p + s + 4)};
                    pairs = _mm_add_pd(
                        pairs,
                        _mm_add_pd(
                            _mm_add_pd(_mm_mul_pd(e0, e0), _mm_mul_pd(e1, e1)), _mm_mul_pd(e2, e2)
                        )
                    );
                }
                return _mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)));
#elif defined(__ARM_NEON)
                // The three elements of each stored vector, its padding left out.
                const double* const p{reinterpret_cast<const double*>(data())};
                float64x2_t pairs{vdupq_n_f64(0.0)};
                for (std::size_t s{0}; s < 24; s += 8) {
                    const float64x2_t e0{vld1q_f64(p + s)};
                    const float64x2_t e1{vld1q_f64(p + s + 2)};
                    const float64x2_t e2{vld1q_f64(p + s + 4)};
                    pairs = vaddq_f64(
                        pairs,
                        vaddq_f64(
                            vaddq_f64(vmulq_f64(e0, e0), vmulq_f64(e1, e1)), vmulq_f64(e2, e2)
                        )
                    );
                }
                return vaddvq_f64(pairs);
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int32_t> ||
                std::is_same_v<value_type, std::uint32_t>
            ) {
#if defined(__SSE4_1__) || defined(__AVX__)
                const auto* const p{reinterpret_cast<const __m128i*>(data())};
                const __m128i v0{_mm_load_si128(p)};
                const __m128i v1{_mm_load_si128(p + 1)};
                const __m128i v2{_mm_load_si128(p + 2)};
                const __m128i sq{_mm_add_epi32(
                    _mm_add_epi32(_mm_mullo_epi32(v0, v0), _mm_mullo_epi32(v1, v1)),
                    _mm_mullo_epi32(v2, v2)
                )};
                const __m128i pairs{
                    _mm_add_epi32(sq, _mm_shuffle_epi32(sq, _MM_SHUFFLE(1, 0, 3, 2)))
                };
                return static_cast<norm_type>(_mm_cvtsi128_si32(
                    _mm_add_epi32(pairs, _mm_shuffle_epi32(pairs, _MM_SHUFFLE(2, 3, 0, 1)))
                ));
#elif defined(__ARM_NEON)
                const auto* const p{reinterpret_cast<const std::int32_t*>(data())};
                const int32x4_t v0{vld1q_s32(p)};
                const int32x4_t v1{vld1q_s32(p + 4)};
                const int32x4_t v2{vld1q_s32(p + 8)};
                return static_cast<norm_type>(vaddvq_s32(
                    vaddq_s32(vaddq_s32(vmulq_s32(v0, v0), vmulq_s32(v1, v1)), vmulq_s32(v2, v2))
                ));
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int64_t> ||
                std::is_same_v<value_type, std::uint64_t>
            ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
                const auto* const p{reinterpret_cast<const __m256i*>(data())};
                const __m256i v0{_mm256_load_si256(p)};
                const __m256i v1{_mm256_load_si256(p + 1)};
                const __m256i v2{_mm256_load_si256(p + 2)};
                const __m256i sq{_mm256_add_epi64(
                    _mm256_add_epi64(_mm256_mullo_epi64(v0, v0), _mm256_mullo_epi64(v1, v1)),
                    _mm256_mullo_epi64(v2, v2)
                )};
                const __m128i pairs{
                    _mm_add_epi64(_mm256_castsi256_si128(sq), _mm256_extracti128_si256(sq, 1))
                };
                return static_cast<norm_type>(
                    _mm_cvtsi128_si64(_mm_add_epi64(pairs, _mm_unpackhi_epi64(pairs, pairs)))
                );
#endif
            }
        }
        // Over all twelve slots, contiguous for the compiler to vectorize.
        norm_type sum(0.0);
        for (const value_type& a : m_data) {
            if constexpr (ComplexArithmetic<value_type>) {
                sum += a.real() * a.real() + a.imag() * a.imag();
            } else {
                sum = static_cast<norm_type>(sum + a * a);
            }
        }
        return sum;
    }

    // The product with a column vector: under layout_left, a combination of the stored columns;
    // under layout_right, the dot products with the stored rows. The zero padding of either sums to
    // the padding of the result, built in one register: a 16-byte result is bit_cast from it, a
    // 32-byte one stored whole, as GCC spills a bit_cast of that size.
    // NOLINTBEGIN(readability-function-size)
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(const Vector<value_type, Index, 3, Alloc>& obj) const noexcept
        -> Vector<value_type, Index, 3, Alloc> {
        using vector_type = Vector<value_type, Index, 3, Alloc>;
        [[maybe_unused]] constexpr bool kLeft{std::is_same_v<layout_type, std::layout_left>};

        if !consteval {
            [[maybe_unused]] const value_type* const m{data()};
            if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE3__) || defined(__AVX__)
                const __m128 v{_mm_load_ps(obj.data())};
                if constexpr (kLeft) {
                    return std::bit_cast<vector_type>(_mm_add_ps(
                        _mm_add_ps(
                            _mm_mul_ps(
                                _mm_load_ps(m), _mm_shuffle_ps(v, v, _MM_SHUFFLE(0, 0, 0, 0))
                            ),
                            _mm_mul_ps(
                                _mm_load_ps(m + 4), _mm_shuffle_ps(v, v, _MM_SHUFFLE(1, 1, 1, 1))
                            )
                        ),
                        _mm_mul_ps(
                            _mm_load_ps(m + 8), _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 2, 2, 2))
                        )
                    ));
                } else {
                    // hadd folds the three row products and a zero fourth into (r0, r1, r2, 0).
                    const __m128 h01{_mm_hadd_ps(
                        _mm_mul_ps(_mm_load_ps(m), v), _mm_mul_ps(_mm_load_ps(m + 4), v)
                    )};
                    const __m128 h2{
                        _mm_hadd_ps(_mm_mul_ps(_mm_load_ps(m + 8), v), _mm_setzero_ps())
                    };
                    return std::bit_cast<vector_type>(_mm_hadd_ps(h01, h2));
                }
#elif defined(__ARM_NEON)
                const float32x4_t v{vld1q_f32(obj.data())};
                if constexpr (kLeft) {
                    return std::bit_cast<vector_type>(vaddq_f32(
                        vaddq_f32(
                            vmulq_laneq_f32(vld1q_f32(m), v, 0),
                            vmulq_laneq_f32(vld1q_f32(m + 4), v, 1)
                        ),
                        vmulq_laneq_f32(vld1q_f32(m + 8), v, 2)
                    ));
                } else {
                    // Pairwise adds fold the three row products and a zero fourth into
                    // (r0, r1, r2, 0).
                    const float32x4_t p01{
                        vpaddq_f32(vmulq_f32(vld1q_f32(m), v), vmulq_f32(vld1q_f32(m + 4), v))
                    };
                    const float32x4_t p2{
                        vpaddq_f32(vmulq_f32(vld1q_f32(m + 8), v), vdupq_n_f32(0.0F))
                    };
                    return std::bit_cast<vector_type>(vpaddq_f32(p01, p2));
                }
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX__)
                const __m256d v{_mm256_load_pd(obj.data())};
                if constexpr (kLeft) {
                    vector_type result;
                    _mm256_store_pd(
                        result.data(),
                        _mm256_add_pd(
                            _mm256_add_pd(
                                _mm256_mul_pd(_mm256_load_pd(m), _mm256_set1_pd(obj.x())),
                                _mm256_mul_pd(_mm256_load_pd(m + 4), _mm256_set1_pd(obj.y()))
                            ),
                            _mm256_mul_pd(_mm256_load_pd(m + 8), _mm256_set1_pd(obj.z()))
                        )
                    );
                    return result;
                } else {
                    // hadd gives (a01, b01, a23, b23) and (c01, 0, c23, 0) for the row products a,
                    // b and c; their 128-bit halves sum to (a, b) and (c, 0).
                    const __m256d h01{_mm256_hadd_pd(
                        _mm256_mul_pd(_mm256_load_pd(m), v), _mm256_mul_pd(_mm256_load_pd(m + 4), v)
                    )};
                    const __m256d h2{
                        _mm256_hadd_pd(_mm256_mul_pd(_mm256_load_pd(m + 8), v), _mm256_setzero_pd())
                    };
                    vector_type result;
                    _mm256_store_pd(
                        result.data(),
                        _mm256_insertf128_pd(
                            _mm256_castpd128_pd256(_mm_add_pd(
                                _mm256_castpd256_pd128(h01), _mm256_extractf128_pd(h01, 1)
                            )),
                            _mm_add_pd(_mm256_castpd256_pd128(h2), _mm256_extractf128_pd(h2, 1)), 1
                        )
                    );
                    return result;
                }
#elif defined(__SSE2__) || defined(_M_X64)
                if constexpr (kLeft) {
                    // The columns combined, (c0, c1) halves and (c2, 0) halves apart.
                    const __m128d vx{_mm_set1_pd(obj.x())};
                    const __m128d vy{_mm_set1_pd(obj.y())};
                    const __m128d vz{_mm_set1_pd(obj.z())};
                    const __m128d lo{_mm_add_pd(
                        _mm_add_pd(
                            _mm_mul_pd(_mm_load_pd(m), vx), _mm_mul_pd(_mm_load_pd(m + 4), vy)
                        ),
                        _mm_mul_pd(_mm_load_pd(m + 8), vz)
                    )};
                    const __m128d hi{_mm_add_pd(
                        _mm_add_pd(
                            _mm_mul_pd(_mm_load_pd(m + 2), vx), _mm_mul_pd(_mm_load_pd(m + 6), vy)
                        ),
                        _mm_mul_pd(_mm_load_pd(m + 10), vz)
                    )};
                    return vector_type{
                        _mm_cvtsd_f64(lo), _mm_cvtsd_f64(_mm_unpackhi_pd(lo, lo)), _mm_cvtsd_f64(hi)
                    };
                } else {
                    // Each stored row (r0, r1 | r2, 0) against (x, y | z, 0).
                    const __m128d v_lo{_mm_load_pd(obj.data())};
                    const __m128d v_hi{_mm_load_pd(obj.data() + 2)};
                    const __m128d p0{_mm_add_pd(
                        _mm_mul_pd(_mm_load_pd(m), v_lo), _mm_mul_pd(_mm_load_pd(m + 2), v_hi)
                    )};
                    const __m128d p1{_mm_add_pd(
                        _mm_mul_pd(_mm_load_pd(m + 4), v_lo), _mm_mul_pd(_mm_load_pd(m + 6), v_hi)
                    )};
                    const __m128d p2{_mm_add_pd(
                        _mm_mul_pd(_mm_load_pd(m + 8), v_lo), _mm_mul_pd(_mm_load_pd(m + 10), v_hi)
                    )};
                    const __m128d ab{_mm_add_pd(_mm_unpacklo_pd(p0, p1), _mm_unpackhi_pd(p0, p1))};
                    return vector_type{
                        _mm_cvtsd_f64(ab), _mm_cvtsd_f64(_mm_unpackhi_pd(ab, ab)),
                        _mm_cvtsd_f64(_mm_add_sd(p2, _mm_unpackhi_pd(p2, p2)))
                    };
                }
#elif defined(__ARM_NEON)
                if constexpr (kLeft) {
                    const float64x2_t lo{vaddq_f64(
                        vaddq_f64(
                            vmulq_n_f64(vld1q_f64(m), obj.x()),
                            vmulq_n_f64(vld1q_f64(m + 4), obj.y())
                        ),
                        vmulq_n_f64(vld1q_f64(m + 8), obj.z())
                    )};
                    const float64x2_t hi{vaddq_f64(
                        vaddq_f64(
                            vmulq_n_f64(vld1q_f64(m + 2), obj.x()),
                            vmulq_n_f64(vld1q_f64(m + 6), obj.y())
                        ),
                        vmulq_n_f64(vld1q_f64(m + 10), obj.z())
                    )};
                    return vector_type{
                        vgetq_lane_f64(lo, 0), vgetq_lane_f64(lo, 1), vgetq_lane_f64(hi, 0)
                    };
                } else {
                    const float64x2_t v_lo{vld1q_f64(obj.data())};
                    const float64x2_t v_hi{vld1q_f64(obj.data() + 2)};
                    const float64x2_t p0{
                        vaddq_f64(vmulq_f64(vld1q_f64(m), v_lo), vmulq_f64(vld1q_f64(m + 2), v_hi))
                    };
                    const float64x2_t p1{vaddq_f64(
                        vmulq_f64(vld1q_f64(m + 4), v_lo), vmulq_f64(vld1q_f64(m + 6), v_hi)
                    )};
                    const float64x2_t p2{vaddq_f64(
                        vmulq_f64(vld1q_f64(m + 8), v_lo), vmulq_f64(vld1q_f64(m + 10), v_hi)
                    )};
                    const float64x2_t ab{vpaddq_f64(p0, p1)};
                    return vector_type{
                        vgetq_lane_f64(ab, 0), vgetq_lane_f64(ab, 1), vaddvq_f64(p2)
                    };
                }
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__AVX__)
                const float* const lanes{reinterpret_cast<const float*>(m)};
                const float* const v{reinterpret_cast<const float*>(obj.data())};
                if constexpr (kLeft) {
                    // The columns combined, each times its coefficient (re, im) as
                    // (a re - b im, b re + a im).
                    const __m256 c0{_mm256_load_ps(lanes)};
                    const __m256 c1{_mm256_load_ps(lanes + 8)};
                    const __m256 c2{_mm256_load_ps(lanes + 16)};
                    vector_type result;
                    _mm256_store_ps(
                        reinterpret_cast<float*>(result.data()),
                        _mm256_add_ps(
                            _mm256_add_ps(
                                _mm256_addsub_ps(
                                    _mm256_mul_ps(c0, _mm256_set1_ps(v[0])),
                                    _mm256_mul_ps(
                                        _mm256_permute_ps(c0, _MM_SHUFFLE(2, 3, 0, 1)),
                                        _mm256_set1_ps(v[1])
                                    )
                                ),
                                _mm256_addsub_ps(
                                    _mm256_mul_ps(c1, _mm256_set1_ps(v[2])),
                                    _mm256_mul_ps(
                                        _mm256_permute_ps(c1, _MM_SHUFFLE(2, 3, 0, 1)),
                                        _mm256_set1_ps(v[3])
                                    )
                                )
                            ),
                            _mm256_addsub_ps(
                                _mm256_mul_ps(c2, _mm256_set1_ps(v[4])),
                                _mm256_mul_ps(
                                    _mm256_permute_ps(c2, _MM_SHUFFLE(2, 3, 0, 1)),
                                    _mm256_set1_ps(v[5])
                                )
                            )
                        )
                    );
                    return result;
                } else {
                    // The complex products (a c - b d, b c + a d) with each stored row, folded to
                    // one element each and gathered as (r0, r1 | r2, 0).
                    const __m256 vv{_mm256_load_ps(v)};
                    const __m256 v_re{_mm256_moveldup_ps(vv)};
                    const __m256 v_im{_mm256_movehdup_ps(vv)};
                    const __m256 r0{_mm256_load_ps(lanes)};
                    const __m256 r1{_mm256_load_ps(lanes + 8)};
                    const __m256 r2{_mm256_load_ps(lanes + 16)};
                    const __m256 p0{_mm256_addsub_ps(
                        _mm256_mul_ps(r0, v_re),
                        _mm256_mul_ps(_mm256_permute_ps(r0, _MM_SHUFFLE(2, 3, 0, 1)), v_im)
                    )};
                    const __m256 p1{_mm256_addsub_ps(
                        _mm256_mul_ps(r1, v_re),
                        _mm256_mul_ps(_mm256_permute_ps(r1, _MM_SHUFFLE(2, 3, 0, 1)), v_im)
                    )};
                    const __m256 p2{_mm256_addsub_ps(
                        _mm256_mul_ps(r2, v_re),
                        _mm256_mul_ps(_mm256_permute_ps(r2, _MM_SHUFFLE(2, 3, 0, 1)), v_im)
                    )};
                    const __m128 s0{
                        _mm_add_ps(_mm256_castps256_ps128(p0), _mm256_extractf128_ps(p0, 1))
                    };
                    const __m128 s1{
                        _mm_add_ps(_mm256_castps256_ps128(p1), _mm256_extractf128_ps(p1, 1))
                    };
                    const __m128 s2{
                        _mm_add_ps(_mm256_castps256_ps128(p2), _mm256_extractf128_ps(p2, 1))
                    };
                    vector_type result;
                    _mm256_store_ps(
                        reinterpret_cast<float*>(result.data()),
                        _mm256_insertf128_ps(
                            _mm256_castps128_ps256(_mm_movelh_ps(
                                _mm_add_ps(s0, _mm_movehl_ps(s0, s0)),
                                _mm_add_ps(s1, _mm_movehl_ps(s1, s1))
                            )),
                            _mm_movelh_ps(_mm_add_ps(s2, _mm_movehl_ps(s2, s2)), _mm_setzero_ps()),
                            1
                        )
                    );
                    return result;
                }
#elif defined(__SSE3__)
                const float* const lanes{reinterpret_cast<const float*>(m)};
                const float* const v{reinterpret_cast<const float*>(obj.data())};
                // Two complex elements a register, (a re - b im, b re + a im) per product: the
                // halves of the stored vectors, and (x, y) and (z, 0) of obj.
                vector_type result;
                float* const out{reinterpret_cast<float*>(result.data())};
                if constexpr (kLeft) {
                    for (std::size_t h{0}; h < 2; ++h) {
                        __m128 sum{_mm_setzero_ps()};
                        for (std::size_t k{0}; k < 3; ++k) {
                            const __m128 c{_mm_load_ps(lanes + 8 * k + 4 * h)};
                            sum = _mm_add_ps(
                                sum, _mm_addsub_ps(
                                         _mm_mul_ps(c, _mm_set1_ps(v[2 * k])),
                                         _mm_mul_ps(
                                             _mm_shuffle_ps(c, c, _MM_SHUFFLE(2, 3, 0, 1)),
                                             _mm_set1_ps(v[2 * k + 1])
                                         )
                                     )
                            );
                        }
                        _mm_store_ps(out + 4 * h, sum);
                    }
                } else {
                    const __m128 v0{_mm_load_ps(v)};
                    const __m128 v1{_mm_load_ps(v + 4)};
                    for (std::size_t i{0}; i < 3; ++i) {
                        const __m128 r0{_mm_load_ps(lanes + 8 * i)};
                        const __m128 r1{_mm_load_ps(lanes + 8 * i + 4)};
                        const __m128 sum{_mm_add_ps(
                            _mm_addsub_ps(
                                _mm_mul_ps(r0, _mm_moveldup_ps(v0)),
                                _mm_mul_ps(
                                    _mm_shuffle_ps(r0, r0, _MM_SHUFFLE(2, 3, 0, 1)),
                                    _mm_movehdup_ps(v0)
                                )
                            ),
                            _mm_addsub_ps(
                                _mm_mul_ps(r1, _mm_moveldup_ps(v1)),
                                _mm_mul_ps(
                                    _mm_shuffle_ps(r1, r1, _MM_SHUFFLE(2, 3, 0, 1)),
                                    _mm_movehdup_ps(v1)
                                )
                            )
                        )};
                        _mm_storel_pi(
                            reinterpret_cast<__m64*>(result.data() + i),
                            _mm_add_ps(sum, _mm_movehl_ps(sum, sum))
                        );
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                const float* const lanes{reinterpret_cast<const float*>(m)};
                const float* const v{reinterpret_cast<const float*>(obj.data())};
                // Two complex elements a register, (a re, b re) plus (b im, a im) with its real
                // lane negated per product: the halves of the stored vectors, and (x, y) and (z, 0)
                // of obj.
                const uint32x4_t sign{vreinterpretq_u32_u64(vdupq_n_u64(0x80000000ULL))};
                vector_type result;
                float* const out{reinterpret_cast<float*>(result.data())};
                if constexpr (kLeft) {
                    for (std::size_t h{0}; h < 2; ++h) {
                        float32x4_t sum{vdupq_n_f32(0.0F)};
                        for (std::size_t k{0}; k < 3; ++k) {
                            const float32x4_t c{vld1q_f32(lanes + 8 * k + 4 * h)};
                            sum = vaddq_f32(
                                sum, vaddq_f32(
                                         vmulq_n_f32(c, v[2 * k]),
                                         vreinterpretq_f32_u32(veorq_u32(
                                             vreinterpretq_u32_f32(
                                                 vmulq_n_f32(vrev64q_f32(c), v[2 * k + 1])
                                             ),
                                             sign
                                         ))
                                     )
                            );
                        }
                        vst1q_f32(out + 4 * h, sum);
                    }
                } else {
                    const float32x4_t v0{vld1q_f32(v)};
                    const float32x4_t v1{vld1q_f32(v + 4)};
                    for (std::size_t i{0}; i < 3; ++i) {
                        const float32x4_t r0{vld1q_f32(lanes + 8 * i)};
                        const float32x4_t r1{vld1q_f32(lanes + 8 * i + 4)};
                        const float32x4_t sum{vaddq_f32(
                            vaddq_f32(
                                vmulq_f32(r0, vtrn1q_f32(v0, v0)),
                                vreinterpretq_f32_u32(veorq_u32(
                                    vreinterpretq_u32_f32(
                                        vmulq_f32(vrev64q_f32(r0), vtrn2q_f32(v0, v0))
                                    ),
                                    sign
                                ))
                            ),
                            vaddq_f32(
                                vmulq_f32(r1, vtrn1q_f32(v1, v1)),
                                vreinterpretq_f32_u32(veorq_u32(
                                    vreinterpretq_u32_f32(
                                        vmulq_f32(vrev64q_f32(r1), vtrn2q_f32(v1, v1))
                                    ),
                                    sign
                                ))
                            )
                        )};
                        vst1_f32(
                            reinterpret_cast<float*>(result.data() + i),
                            vadd_f32(vget_low_f32(sum), vget_high_f32(sum))
                        );
                    }
                }
                return result;
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX__)
                // Two complex elements a register, (x, y) and (z, 0) of obj and the halves of each
                // stored vector, multiplied as (a re - b im, b re + a im); the result is stored in
                // its two halves.
                const double* const lanes{reinterpret_cast<const double*>(m)};
                const double* const v{reinterpret_cast<const double*>(obj.data())};
                vector_type result;
                double* const out{reinterpret_cast<double*>(result.data())};
                if constexpr (kLeft) {
                    // The columns combined, each times its coefficient, half by half: their zero
                    // padding sums to the padding of the result.
                    const __m256d re0{_mm256_set1_pd(v[0])};
                    const __m256d im0{_mm256_set1_pd(v[1])};
                    const __m256d re1{_mm256_set1_pd(v[2])};
                    const __m256d im1{_mm256_set1_pd(v[3])};
                    const __m256d re2{_mm256_set1_pd(v[4])};
                    const __m256d im2{_mm256_set1_pd(v[5])};
                    for (std::size_t h{0}; h < 8; h += 4) {
                        const __m256d c0{_mm256_load_pd(lanes + h)};
                        const __m256d c1{_mm256_load_pd(lanes + 8 + h)};
                        const __m256d c2{_mm256_load_pd(lanes + 16 + h)};
                        _mm256_store_pd(
                            out + h, _mm256_add_pd(
                                         _mm256_add_pd(
                                             _mm256_addsub_pd(
                                                 _mm256_mul_pd(c0, re0),
                                                 _mm256_mul_pd(_mm256_permute_pd(c0, 0b0101), im0)
                                             ),
                                             _mm256_addsub_pd(
                                                 _mm256_mul_pd(c1, re1),
                                                 _mm256_mul_pd(_mm256_permute_pd(c1, 0b0101), im1)
                                             )
                                         ),
                                         _mm256_addsub_pd(
                                             _mm256_mul_pd(c2, re2),
                                             _mm256_mul_pd(_mm256_permute_pd(c2, 0b0101), im2)
                                         )
                                     )
                        );
                    }
                } else {
                    // The products with stored row i, (r0 x, r1 y) plus (r2 z, 0), fold to element
                    // i; the elements are gathered as (e0, e1 | e2, 0).
                    const __m256d v_lo{_mm256_load_pd(v)};
                    const __m256d v_hi{_mm256_load_pd(v + 4)};
                    const __m256d re_lo{_mm256_movedup_pd(v_lo)};
                    const __m256d im_lo{_mm256_permute_pd(v_lo, 0b1111)};
                    const __m256d re_hi{_mm256_movedup_pd(v_hi)};
                    const __m256d im_hi{_mm256_permute_pd(v_hi, 0b1111)};
                    const __m256d r0_lo{_mm256_load_pd(lanes)};
                    const __m256d r0_hi{_mm256_load_pd(lanes + 4)};
                    const __m256d r1_lo{_mm256_load_pd(lanes + 8)};
                    const __m256d r1_hi{_mm256_load_pd(lanes + 12)};
                    const __m256d r2_lo{_mm256_load_pd(lanes + 16)};
                    const __m256d r2_hi{_mm256_load_pd(lanes + 20)};
                    const __m256d s0{_mm256_add_pd(
                        _mm256_addsub_pd(
                            _mm256_mul_pd(r0_lo, re_lo),
                            _mm256_mul_pd(_mm256_permute_pd(r0_lo, 0b0101), im_lo)
                        ),
                        _mm256_addsub_pd(
                            _mm256_mul_pd(r0_hi, re_hi),
                            _mm256_mul_pd(_mm256_permute_pd(r0_hi, 0b0101), im_hi)
                        )
                    )};
                    const __m256d s1{_mm256_add_pd(
                        _mm256_addsub_pd(
                            _mm256_mul_pd(r1_lo, re_lo),
                            _mm256_mul_pd(_mm256_permute_pd(r1_lo, 0b0101), im_lo)
                        ),
                        _mm256_addsub_pd(
                            _mm256_mul_pd(r1_hi, re_hi),
                            _mm256_mul_pd(_mm256_permute_pd(r1_hi, 0b0101), im_hi)
                        )
                    )};
                    const __m256d s2{_mm256_add_pd(
                        _mm256_addsub_pd(
                            _mm256_mul_pd(r2_lo, re_lo),
                            _mm256_mul_pd(_mm256_permute_pd(r2_lo, 0b0101), im_lo)
                        ),
                        _mm256_addsub_pd(
                            _mm256_mul_pd(r2_hi, re_hi),
                            _mm256_mul_pd(_mm256_permute_pd(r2_hi, 0b0101), im_hi)
                        )
                    )};
                    _mm256_store_pd(
                        out,
                        _mm256_insertf128_pd(
                            _mm256_castpd128_pd256(
                                _mm_add_pd(_mm256_castpd256_pd128(s0), _mm256_extractf128_pd(s0, 1))
                            ),
                            _mm_add_pd(_mm256_castpd256_pd128(s1), _mm256_extractf128_pd(s1, 1)), 1
                        )
                    );
                    _mm256_store_pd(
                        out + 4, _mm256_insertf128_pd(
                                     _mm256_castpd128_pd256(_mm_add_pd(
                                         _mm256_castpd256_pd128(s2), _mm256_extractf128_pd(s2, 1)
                                     )),
                                     _mm_setzero_pd(), 1
                                 )
                    );
                }
                return result;
#elif defined(__SSE3__)
                // One complex element a 128-bit register: element i of the product sums the
                // elements (i, k) times the elements k of obj, stored vectors four elements apart.
                const double* const lanes{reinterpret_cast<const double*>(m)};
                const double* const v{reinterpret_cast<const double*>(obj.data())};
                vector_type result;
                double* const out{reinterpret_cast<double*>(result.data())};
                for (std::size_t i{0}; i < 3; ++i) {
                    __m128d sum{_mm_setzero_pd()};
                    for (std::size_t k{0}; k < 3; ++k) {
                        const __m128d a{_mm_load_pd(lanes + 2 * (kLeft ? 4 * k + i : 4 * i + k))};
                        sum = _mm_add_pd(
                            sum,
                            _mm_addsub_pd(
                                _mm_mul_pd(a, _mm_set1_pd(v[2 * k])),
                                _mm_mul_pd(_mm_shuffle_pd(a, a, 0b01), _mm_set1_pd(v[2 * k + 1]))
                            )
                        );
                    }
                    _mm_store_pd(out + 2 * i, sum);
                }
                return result;
#elif defined(__ARM_NEON)
                const double* const lanes{reinterpret_cast<const double*>(m)};
                const double* const v{reinterpret_cast<const double*>(obj.data())};
                vector_type result;
                double* const out{reinterpret_cast<double*>(result.data())};
                const uint64x2_t sign{vsetq_lane_u64(0x8000000000000000ULL, vdupq_n_u64(0), 0)};
                for (std::size_t i{0}; i < 3; ++i) {
                    float64x2_t sum{vdupq_n_f64(0.0)};
                    for (std::size_t k{0}; k < 3; ++k) {
                        const float64x2_t a{vld1q_f64(lanes + 2 * (kLeft ? 4 * k + i : 4 * i + k))};
                        sum = vaddq_f64(
                            sum, vaddq_f64(
                                     vmulq_n_f64(a, v[2 * k]),
                                     vreinterpretq_f64_u64(veorq_u64(
                                         vreinterpretq_u64_f64(
                                             vmulq_n_f64(vextq_f64(a, a, 1), v[2 * k + 1])
                                         ),
                                         sign
                                     ))
                                 )
                        );
                    }
                    vst1q_f64(out + 2 * i, sum);
                }
                return result;
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int32_t> ||
                std::is_same_v<value_type, std::uint32_t>
            ) {
#if defined(__SSE4_1__) || defined(__AVX__)
                const auto* const lanes{reinterpret_cast<const std::int32_t*>(m)};
                const __m128i v{_mm_load_si128(reinterpret_cast<const __m128i*>(obj.data()))};
                if constexpr (kLeft) {
                    return std::bit_cast<vector_type>(_mm_add_epi32(
                        _mm_add_epi32(
                            _mm_mullo_epi32(
                                _mm_load_si128(reinterpret_cast<const __m128i*>(lanes)),
                                _mm_shuffle_epi32(v, _MM_SHUFFLE(0, 0, 0, 0))
                            ),
                            _mm_mullo_epi32(
                                _mm_load_si128(reinterpret_cast<const __m128i*>(lanes + 4)),
                                _mm_shuffle_epi32(v, _MM_SHUFFLE(1, 1, 1, 1))
                            )
                        ),
                        _mm_mullo_epi32(
                            _mm_load_si128(reinterpret_cast<const __m128i*>(lanes + 8)),
                            _mm_shuffle_epi32(v, _MM_SHUFFLE(2, 2, 2, 2))
                        )
                    ));
                } else {
                    // hadd folds the three row products and a zero fourth into (r0, r1, r2, 0).
                    const __m128i h01{_mm_hadd_epi32(
                        _mm_mullo_epi32(_mm_load_si128(reinterpret_cast<const __m128i*>(lanes)), v),
                        _mm_mullo_epi32(
                            _mm_load_si128(reinterpret_cast<const __m128i*>(lanes + 4)), v
                        )
                    )};
                    const __m128i h2{_mm_hadd_epi32(
                        _mm_mullo_epi32(
                            _mm_load_si128(reinterpret_cast<const __m128i*>(lanes + 8)), v
                        ),
                        _mm_setzero_si128()
                    )};
                    return std::bit_cast<vector_type>(_mm_hadd_epi32(h01, h2));
                }
#elif defined(__ARM_NEON)
                const auto* const lanes{reinterpret_cast<const std::int32_t*>(m)};
                const int32x4_t v{vld1q_s32(reinterpret_cast<const std::int32_t*>(obj.data()))};
                if constexpr (kLeft) {
                    return std::bit_cast<vector_type>(vaddq_s32(
                        vaddq_s32(
                            vmulq_laneq_s32(vld1q_s32(lanes), v, 0),
                            vmulq_laneq_s32(vld1q_s32(lanes + 4), v, 1)
                        ),
                        vmulq_laneq_s32(vld1q_s32(lanes + 8), v, 2)
                    ));
                } else {
                    // Pairwise adds fold the three row products and a zero fourth into
                    // (r0, r1, r2, 0).
                    const int32x4_t p01{vpaddq_s32(
                        vmulq_s32(vld1q_s32(lanes), v), vmulq_s32(vld1q_s32(lanes + 4), v)
                    )};
                    const int32x4_t p2{
                        vpaddq_s32(vmulq_s32(vld1q_s32(lanes + 8), v), vdupq_n_s32(0))
                    };
                    return std::bit_cast<vector_type>(vpaddq_s32(p01, p2));
                }
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int64_t> ||
                std::is_same_v<value_type, std::uint64_t>
            ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
                if constexpr (kLeft) {
                    vector_type result;
                    _mm256_store_si256(
                        reinterpret_cast<__m256i*>(result.data()),
                        _mm256_add_epi64(
                            _mm256_add_epi64(
                                _mm256_mullo_epi64(
                                    _mm256_load_si256(reinterpret_cast<const __m256i*>(m)),
                                    _mm256_set1_epi64x(static_cast<long long>(obj.x()))
                                ),
                                _mm256_mullo_epi64(
                                    _mm256_load_si256(reinterpret_cast<const __m256i*>(m + 4)),
                                    _mm256_set1_epi64x(static_cast<long long>(obj.y()))
                                )
                            ),
                            _mm256_mullo_epi64(
                                _mm256_load_si256(reinterpret_cast<const __m256i*>(m + 8)),
                                _mm256_set1_epi64x(static_cast<long long>(obj.z()))
                            )
                        )
                    );
                    return result;
                } else {
                    // The dot product with each stored row.
                    const __m256i v{
                        _mm256_load_si256(reinterpret_cast<const __m256i*>(obj.data()))
                    };
                    vector_type result;
                    for (std::size_t i{0}; i < 3; ++i) {
                        const __m256i prod{_mm256_mullo_epi64(
                            _mm256_load_si256(reinterpret_cast<const __m256i*>(m + 4 * i)), v
                        )};
                        const __m128i pairs{_mm_add_epi64(
                            _mm256_castsi256_si128(prod), _mm256_extracti128_si256(prod, 1)
                        )};
                        result[i] = static_cast<value_type>(_mm_cvtsi128_si64(
                            _mm_add_epi64(pairs, _mm_unpackhi_epi64(pairs, pairs))
                        ));
                    }
                    return result;
                }
#endif
            }
        }
        return vector_type{
            static_cast<value_type>(
                operator[](0, 0) * obj.x() + operator[](0, 1) * obj.y() + operator[](0, 2) * obj.z()
            ),
            static_cast<value_type>(
                operator[](1, 0) * obj.x() + operator[](1, 1) * obj.y() + operator[](1, 2) * obj.z()
            ),
            static_cast<value_type>(
                operator[](2, 0) * obj.x() + operator[](2, 1) * obj.y() + operator[](2, 2) * obj.z()
            )
        };
    }
    // NOLINTEND(readability-function-size)

    // Stored vector s of the product combines the three stored vectors of one factor with the
    // elements of stored vector s of the other: the columns of *this with those of obj under
    // layout_left, the rows of obj with those of *this under layout_right. Their zero padding sums
    // to the padding of the product.
    // NOLINTBEGIN(readability-function-size)
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(const Matrix& obj) const noexcept -> Matrix {
        constexpr bool kLeft{std::is_same_v<layout_type, std::layout_left>};

        // Every element is written below, and every return names it, so that it is built in
        // place.
        Matrix result;
        const value_type* const vecs{kLeft ? data() : obj.data()};
        const value_type* const coefs{kLeft ? obj.data() : data()};
        if !consteval {
            if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE2__) || defined(_M_X64)
                const __m128 v0{_mm_load_ps(vecs)};
                const __m128 v1{_mm_load_ps(vecs + 4)};
                const __m128 v2{_mm_load_ps(vecs + 8)};
                for (std::size_t s{0}; s < 3; ++s) {
                    const __m128 c{_mm_load_ps(coefs + 4 * s)};
                    _mm_store_ps(
                        result.data() + 4 * s,
                        _mm_add_ps(
                            _mm_add_ps(
                                _mm_mul_ps(v0, _mm_shuffle_ps(c, c, _MM_SHUFFLE(0, 0, 0, 0))),
                                _mm_mul_ps(v1, _mm_shuffle_ps(c, c, _MM_SHUFFLE(1, 1, 1, 1)))
                            ),
                            _mm_mul_ps(v2, _mm_shuffle_ps(c, c, _MM_SHUFFLE(2, 2, 2, 2)))
                        )
                    );
                }
                return result;
#elif defined(__ARM_NEON)
                const float32x4_t v0{vld1q_f32(vecs)};
                const float32x4_t v1{vld1q_f32(vecs + 4)};
                const float32x4_t v2{vld1q_f32(vecs + 8)};
                for (std::size_t s{0}; s < 3; ++s) {
                    const float32x4_t c{vld1q_f32(coefs + 4 * s)};
                    vst1q_f32(
                        result.data() + 4 * s,
                        vaddq_f32(
                            vaddq_f32(vmulq_laneq_f32(v0, c, 0), vmulq_laneq_f32(v1, c, 1)),
                            vmulq_laneq_f32(v2, c, 2)
                        )
                    );
                }
                return result;
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX__)
                const __m256d v0{_mm256_load_pd(vecs)};
                const __m256d v1{_mm256_load_pd(vecs + 4)};
                const __m256d v2{_mm256_load_pd(vecs + 8)};
                for (std::size_t s{0}; s < 3; ++s) {
                    _mm256_store_pd(
                        result.data() + 4 * s,
                        _mm256_add_pd(
                            _mm256_add_pd(
                                _mm256_mul_pd(v0, _mm256_set1_pd(coefs[4 * s])),
                                _mm256_mul_pd(v1, _mm256_set1_pd(coefs[4 * s + 1]))
                            ),
                            _mm256_mul_pd(v2, _mm256_set1_pd(coefs[4 * s + 2]))
                        )
                    );
                }
                return result;
#elif defined(__SSE2__) || defined(_M_X64)
                // Each stored vector in the halves (e0, e1) and (e2, 0).
                for (std::size_t s{0}; s < 3; ++s) {
                    const __m128d c0{_mm_set1_pd(coefs[4 * s])};
                    const __m128d c1{_mm_set1_pd(coefs[4 * s + 1])};
                    const __m128d c2{_mm_set1_pd(coefs[4 * s + 2])};
                    for (std::size_t h{0}; h < 4; h += 2) {
                        _mm_store_pd(
                            result.data() + 4 * s + h,
                            _mm_add_pd(
                                _mm_add_pd(
                                    _mm_mul_pd(_mm_load_pd(vecs + h), c0),
                                    _mm_mul_pd(_mm_load_pd(vecs + 4 + h), c1)
                                ),
                                _mm_mul_pd(_mm_load_pd(vecs + 8 + h), c2)
                            )
                        );
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                // Each stored vector in the halves (e0, e1) and (e2, 0).
                for (std::size_t s{0}; s < 3; ++s) {
                    for (std::size_t h{0}; h < 4; h += 2) {
                        vst1q_f64(
                            result.data() + 4 * s + h,
                            vaddq_f64(
                                vaddq_f64(
                                    vmulq_n_f64(vld1q_f64(vecs + h), coefs[4 * s]),
                                    vmulq_n_f64(vld1q_f64(vecs + 4 + h), coefs[4 * s + 1])
                                ),
                                vmulq_n_f64(vld1q_f64(vecs + 8 + h), coefs[4 * s + 2])
                            )
                        );
                    }
                }
                return result;
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__AVX__)
                const float* const v{reinterpret_cast<const float*>(vecs)};
                const float* const c{reinterpret_cast<const float*>(coefs)};
                float* const out{reinterpret_cast<float*>(result.data())};
                // Each stored vector of the product: v0, v1 and v2 times their complex
                // coefficients, as (a re - b im, b re + a im).
                const __m256 v0{_mm256_load_ps(v)};
                const __m256 v1{_mm256_load_ps(v + 8)};
                const __m256 v2{_mm256_load_ps(v + 16)};
                for (std::size_t s{0}; s < 3; ++s) {
                    _mm256_store_ps(
                        out + 8 * s, _mm256_add_ps(
                                         _mm256_add_ps(
                                             _mm256_addsub_ps(
                                                 _mm256_mul_ps(v0, _mm256_set1_ps(c[8 * s])),
                                                 _mm256_mul_ps(
                                                     _mm256_permute_ps(v0, _MM_SHUFFLE(2, 3, 0, 1)),
                                                     _mm256_set1_ps(c[8 * s + 1])
                                                 )
                                             ),
                                             _mm256_addsub_ps(
                                                 _mm256_mul_ps(v1, _mm256_set1_ps(c[8 * s + 2])),
                                                 _mm256_mul_ps(
                                                     _mm256_permute_ps(v1, _MM_SHUFFLE(2, 3, 0, 1)),
                                                     _mm256_set1_ps(c[8 * s + 3])
                                                 )
                                             )
                                         ),
                                         _mm256_addsub_ps(
                                             _mm256_mul_ps(v2, _mm256_set1_ps(c[8 * s + 4])),
                                             _mm256_mul_ps(
                                                 _mm256_permute_ps(v2, _MM_SHUFFLE(2, 3, 0, 1)),
                                                 _mm256_set1_ps(c[8 * s + 5])
                                             )
                                         )
                                     )
                    );
                }
                return result;
#elif defined(__SSE3__)
                const float* const v{reinterpret_cast<const float*>(vecs)};
                const float* const c{reinterpret_cast<const float*>(coefs)};
                float* const out{reinterpret_cast<float*>(result.data())};
                // Each half of a stored vector of the product: the halves of v0, v1 and v2 times
                // their complex coefficients, as (a re - b im, b re + a im).
                for (std::size_t s{0}; s < 3; ++s) {
                    for (std::size_t h{0}; h < 8; h += 4) {
                        __m128 sum{_mm_setzero_ps()};
                        for (std::size_t k{0}; k < 3; ++k) {
                            const __m128 a{_mm_load_ps(v + 8 * k + h)};
                            sum = _mm_add_ps(
                                sum, _mm_addsub_ps(
                                         _mm_mul_ps(a, _mm_set1_ps(c[8 * s + 2 * k])),
                                         _mm_mul_ps(
                                             _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 3, 0, 1)),
                                             _mm_set1_ps(c[8 * s + 2 * k + 1])
                                         )
                                     )
                            );
                        }
                        _mm_store_ps(out + 8 * s + h, sum);
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                const float* const v{reinterpret_cast<const float*>(vecs)};
                const float* const c{reinterpret_cast<const float*>(coefs)};
                float* const out{reinterpret_cast<float*>(result.data())};
                // Each half of a stored vector of the product: the halves of v0, v1 and v2 times
                // their complex coefficients, as (a re, b re) plus (b im, a im) with its real lane
                // negated.
                const uint32x4_t sign{vreinterpretq_u32_u64(vdupq_n_u64(0x80000000ULL))};
                for (std::size_t s{0}; s < 3; ++s) {
                    for (std::size_t h{0}; h < 8; h += 4) {
                        float32x4_t sum{vdupq_n_f32(0.0F)};
                        for (std::size_t k{0}; k < 3; ++k) {
                            const float32x4_t a{vld1q_f32(v + 8 * k + h)};
                            sum = vaddq_f32(
                                sum, vaddq_f32(
                                         vmulq_n_f32(a, c[8 * s + 2 * k]),
                                         vreinterpretq_f32_u32(veorq_u32(
                                             vreinterpretq_u32_f32(
                                                 vmulq_n_f32(vrev64q_f32(a), c[8 * s + 2 * k + 1])
                                             ),
                                             sign
                                         ))
                                     )
                            );
                        }
                        vst1q_f32(out + 8 * s + h, sum);
                    }
                }
                return result;
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX__)
                const double* const v{reinterpret_cast<const double*>(vecs)};
                const double* const c{reinterpret_cast<const double*>(coefs)};
                double* const out{reinterpret_cast<double*>(result.data())};
                // Each stored vector of the product, half by half: the halves of v0, v1 and v2,
                // loaded once, times their complex coefficients, as (a re - b im, b re + a im).
                const __m256d v0_lo{_mm256_load_pd(v)};
                const __m256d v0_hi{_mm256_load_pd(v + 4)};
                const __m256d v1_lo{_mm256_load_pd(v + 8)};
                const __m256d v1_hi{_mm256_load_pd(v + 12)};
                const __m256d v2_lo{_mm256_load_pd(v + 16)};
                const __m256d v2_hi{_mm256_load_pd(v + 20)};
                for (std::size_t s{0}; s < 3; ++s) {
                    const __m256d re0{_mm256_set1_pd(c[8 * s])};
                    const __m256d im0{_mm256_set1_pd(c[8 * s + 1])};
                    const __m256d re1{_mm256_set1_pd(c[8 * s + 2])};
                    const __m256d im1{_mm256_set1_pd(c[8 * s + 3])};
                    const __m256d re2{_mm256_set1_pd(c[8 * s + 4])};
                    const __m256d im2{_mm256_set1_pd(c[8 * s + 5])};
                    _mm256_store_pd(
                        out + 8 * s,
                        _mm256_add_pd(
                            _mm256_add_pd(
                                _mm256_addsub_pd(
                                    _mm256_mul_pd(v0_lo, re0),
                                    _mm256_mul_pd(_mm256_permute_pd(v0_lo, 0b0101), im0)
                                ),
                                _mm256_addsub_pd(
                                    _mm256_mul_pd(v1_lo, re1),
                                    _mm256_mul_pd(_mm256_permute_pd(v1_lo, 0b0101), im1)
                                )
                            ),
                            _mm256_addsub_pd(
                                _mm256_mul_pd(v2_lo, re2),
                                _mm256_mul_pd(_mm256_permute_pd(v2_lo, 0b0101), im2)
                            )
                        )
                    );
                    _mm256_store_pd(
                        out + 8 * s + 4,
                        _mm256_add_pd(
                            _mm256_add_pd(
                                _mm256_addsub_pd(
                                    _mm256_mul_pd(v0_hi, re0),
                                    _mm256_mul_pd(_mm256_permute_pd(v0_hi, 0b0101), im0)
                                ),
                                _mm256_addsub_pd(
                                    _mm256_mul_pd(v1_hi, re1),
                                    _mm256_mul_pd(_mm256_permute_pd(v1_hi, 0b0101), im1)
                                )
                            ),
                            _mm256_addsub_pd(
                                _mm256_mul_pd(v2_hi, re2),
                                _mm256_mul_pd(_mm256_permute_pd(v2_hi, 0b0101), im2)
                            )
                        )
                    );
                }
                return result;
#elif defined(__SSE3__)
                const double* const v{reinterpret_cast<const double*>(vecs)};
                const double* const c{reinterpret_cast<const double*>(coefs)};
                double* const out{reinterpret_cast<double*>(result.data())};
                // Element i of stored vector s of the product, as (a re - b im, b re + a im); the
                // padding keeps the zero it was constructed with.
                for (std::size_t s{0}; s < 3; ++s) {
                    for (std::size_t i{0}; i < 3; ++i) {
                        __m128d sum{_mm_setzero_pd()};
                        for (std::size_t k{0}; k < 3; ++k) {
                            const __m128d a{_mm_load_pd(v + 8 * k + 2 * i)};
                            sum = _mm_add_pd(
                                sum, _mm_addsub_pd(
                                         _mm_mul_pd(a, _mm_set1_pd(c[8 * s + 2 * k])),
                                         _mm_mul_pd(
                                             _mm_shuffle_pd(a, a, 0b01),
                                             _mm_set1_pd(c[8 * s + 2 * k + 1])
                                         )
                                     )
                            );
                        }
                        _mm_store_pd(out + 8 * s + 2 * i, sum);
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                const double* const v{reinterpret_cast<const double*>(vecs)};
                const double* const c{reinterpret_cast<const double*>(coefs)};
                double* const out{reinterpret_cast<double*>(result.data())};
                // Element i of stored vector s of the product, as (a re, b re) plus (b im, a im)
                // with its real lane negated; the padding keeps the zero it was constructed with.
                const uint64x2_t sign{vsetq_lane_u64(0x8000000000000000ULL, vdupq_n_u64(0), 0)};
                for (std::size_t s{0}; s < 3; ++s) {
                    for (std::size_t i{0}; i < 3; ++i) {
                        float64x2_t sum{vdupq_n_f64(0.0)};
                        for (std::size_t k{0}; k < 3; ++k) {
                            const float64x2_t a{vld1q_f64(v + 8 * k + 2 * i)};
                            sum = vaddq_f64(
                                sum, vaddq_f64(
                                         vmulq_n_f64(a, c[8 * s + 2 * k]),
                                         vreinterpretq_f64_u64(veorq_u64(
                                             vreinterpretq_u64_f64(vmulq_n_f64(
                                                 vextq_f64(a, a, 1), c[8 * s + 2 * k + 1]
                                             )),
                                             sign
                                         ))
                                     )
                            );
                        }
                        vst1q_f64(out + 8 * s + 2 * i, sum);
                    }
                }
                return result;
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int32_t> ||
                std::is_same_v<value_type, std::uint32_t>
            ) {
#if defined(__SSE4_1__) || defined(__AVX__)
                const auto* const v{reinterpret_cast<const __m128i*>(vecs)};
                const auto* const c{reinterpret_cast<const __m128i*>(coefs)};
                auto* const out{reinterpret_cast<__m128i*>(result.data())};
                const __m128i v0{_mm_load_si128(v)};
                const __m128i v1{_mm_load_si128(v + 1)};
                const __m128i v2{_mm_load_si128(v + 2)};
                for (std::size_t s{0}; s < 3; ++s) {
                    const __m128i cs{_mm_load_si128(c + s)};
                    _mm_store_si128(
                        out + s,
                        _mm_add_epi32(
                            _mm_add_epi32(
                                _mm_mullo_epi32(v0, _mm_shuffle_epi32(cs, _MM_SHUFFLE(0, 0, 0, 0))),
                                _mm_mullo_epi32(v1, _mm_shuffle_epi32(cs, _MM_SHUFFLE(1, 1, 1, 1)))
                            ),
                            _mm_mullo_epi32(v2, _mm_shuffle_epi32(cs, _MM_SHUFFLE(2, 2, 2, 2)))
                        )
                    );
                }
                return result;
#elif defined(__ARM_NEON)
                const auto* const v{reinterpret_cast<const std::int32_t*>(vecs)};
                const auto* const c{reinterpret_cast<const std::int32_t*>(coefs)};
                auto* const out{reinterpret_cast<std::int32_t*>(result.data())};
                const int32x4_t v0{vld1q_s32(v)};
                const int32x4_t v1{vld1q_s32(v + 4)};
                const int32x4_t v2{vld1q_s32(v + 8)};
                for (std::size_t s{0}; s < 3; ++s) {
                    const int32x4_t cs{vld1q_s32(c + 4 * s)};
                    vst1q_s32(
                        out + 4 * s,
                        vaddq_s32(
                            vaddq_s32(vmulq_laneq_s32(v0, cs, 0), vmulq_laneq_s32(v1, cs, 1)),
                            vmulq_laneq_s32(v2, cs, 2)
                        )
                    );
                }
                return result;
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int64_t> ||
                std::is_same_v<value_type, std::uint64_t>
            ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
                const __m256i v0{_mm256_load_si256(reinterpret_cast<const __m256i*>(vecs))};
                const __m256i v1{_mm256_load_si256(reinterpret_cast<const __m256i*>(vecs + 4))};
                const __m256i v2{_mm256_load_si256(reinterpret_cast<const __m256i*>(vecs + 8))};
                for (std::size_t s{0}; s < 3; ++s) {
                    _mm256_store_si256(
                        reinterpret_cast<__m256i*>(result.data() + 4 * s),
                        _mm256_add_epi64(
                            _mm256_add_epi64(
                                _mm256_mullo_epi64(
                                    v0, _mm256_set1_epi64x(static_cast<long long>(coefs[4 * s]))
                                ),
                                _mm256_mullo_epi64(
                                    v1, _mm256_set1_epi64x(static_cast<long long>(coefs[4 * s + 1]))
                                )
                            ),
                            _mm256_mullo_epi64(
                                v2, _mm256_set1_epi64x(static_cast<long long>(coefs[4 * s + 2]))
                            )
                        )
                    );
                }
                return result;
#endif
            }
        }
        // Stored vector by stored vector, the padding included, contiguous for the compiler to
        // vectorize.
        for (std::size_t s{0}; s < 3; ++s) {
            for (std::size_t i{0}; i < 4; ++i) {
                result.m_data[4 * s + i] = static_cast<value_type>(
                    vecs[i] * coefs[4 * s] + vecs[4 + i] * coefs[4 * s + 1] +
                    vecs[8 + i] * coefs[4 * s + 2]
                );
            }
        }
        return result;
    }
    // NOLINTEND(readability-function-size)

    // The product with a general matrix of three rows. Under layout_left, each column of the
    // product combines the columns of *this and is stored without writing past it; under
    // layout_right, each row combines the rows of obj.
    // NOLINTBEGIN(readability-function-size, readability-function-cognitive-complexity)
    template <std::size_t M>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(const Matrix<value_type, Index, 3, M, Layout, Alloc>& obj) const noexcept
        -> Matrix<value_type, Index, 3, M, Layout, Alloc> {
        [[maybe_unused]] constexpr bool kLeft{std::is_same_v<layout_type, std::layout_left>};

        // Every element is written below.
        Matrix<value_type, Index, 3, M, Layout, Alloc> result;
        if !consteval {
            [[maybe_unused]] const value_type* const o{obj.data()};
            [[maybe_unused]] value_type* const out{result.data()};
            if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE2__) || defined(_M_X64)
                if constexpr (kLeft) {
                    // Each column: the columns a0, a1 and a2 of *this times its three elements.
                    const __m128 a0{_mm_load_ps(data())};
                    const __m128 a1{_mm_load_ps(data() + 4)};
                    const __m128 a2{_mm_load_ps(data() + 8)};
                    for (std::size_t j{0}; j < M; ++j) {
                        const __m128 col{_mm_add_ps(
                            _mm_add_ps(
                                _mm_mul_ps(a0, _mm_set1_ps(o[3 * j])),
                                _mm_mul_ps(a1, _mm_set1_ps(o[3 * j + 1]))
                            ),
                            _mm_mul_ps(a2, _mm_set1_ps(o[3 * j + 2]))
                        )};
                        _mm_storel_pi(reinterpret_cast<__m64*>(out + 3 * j), col);
                        _mm_store_ss(out + 3 * j + 2, _mm_movehl_ps(col, col));
                    }
                } else {
                    // Four columns a step of the three rows: row i of the product combines the
                    // rows of obj with the elements (i, 0), (i, 1) and (i, 2) of *this.
                    const __m128 a00{_mm_set1_ps(m_data[0])};
                    const __m128 a01{_mm_set1_ps(m_data[1])};
                    const __m128 a02{_mm_set1_ps(m_data[2])};
                    const __m128 a10{_mm_set1_ps(m_data[4])};
                    const __m128 a11{_mm_set1_ps(m_data[5])};
                    const __m128 a12{_mm_set1_ps(m_data[6])};
                    const __m128 a20{_mm_set1_ps(m_data[8])};
                    const __m128 a21{_mm_set1_ps(m_data[9])};
                    const __m128 a22{_mm_set1_ps(m_data[10])};
                    std::size_t j{0};
                    for (; j + 4 <= M; j += 4) {
                        const __m128 r0{_mm_loadu_ps(o + j)};
                        const __m128 r1{_mm_loadu_ps(o + M + j)};
                        const __m128 r2{_mm_loadu_ps(o + 2 * M + j)};
                        _mm_storeu_ps(
                            out + j, _mm_add_ps(
                                         _mm_add_ps(_mm_mul_ps(a00, r0), _mm_mul_ps(a01, r1)),
                                         _mm_mul_ps(a02, r2)
                                     )
                        );
                        _mm_storeu_ps(
                            out + M + j, _mm_add_ps(
                                             _mm_add_ps(_mm_mul_ps(a10, r0), _mm_mul_ps(a11, r1)),
                                             _mm_mul_ps(a12, r2)
                                         )
                        );
                        _mm_storeu_ps(
                            out + 2 * M + j,
                            _mm_add_ps(
                                _mm_add_ps(_mm_mul_ps(a20, r0), _mm_mul_ps(a21, r1)),
                                _mm_mul_ps(a22, r2)
                            )
                        );
                    }
                    for (; j < M; ++j) {
                        out[j] = m_data[0] * o[j] + m_data[1] * o[M + j] + m_data[2] * o[2 * M + j];
                        out[M + j] =
                            m_data[4] * o[j] + m_data[5] * o[M + j] + m_data[6] * o[2 * M + j];
                        out[2 * M + j] =
                            m_data[8] * o[j] + m_data[9] * o[M + j] + m_data[10] * o[2 * M + j];
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                if constexpr (kLeft) {
                    const float32x4_t a0{vld1q_f32(data())};
                    const float32x4_t a1{vld1q_f32(data() + 4)};
                    const float32x4_t a2{vld1q_f32(data() + 8)};
                    for (std::size_t j{0}; j < M; ++j) {
                        const float32x4_t col{vaddq_f32(
                            vaddq_f32(vmulq_n_f32(a0, o[3 * j]), vmulq_n_f32(a1, o[3 * j + 1])),
                            vmulq_n_f32(a2, o[3 * j + 2])
                        )};
                        vst1_f32(out + 3 * j, vget_low_f32(col));
                        vst1q_lane_f32(out + 3 * j + 2, col, 2);
                    }
                } else {
                    std::size_t j{0};
                    for (; j + 4 <= M; j += 4) {
                        const float32x4_t r0{vld1q_f32(o + j)};
                        const float32x4_t r1{vld1q_f32(o + M + j)};
                        const float32x4_t r2{vld1q_f32(o + 2 * M + j)};
                        vst1q_f32(
                            out + j,
                            vaddq_f32(
                                vaddq_f32(vmulq_n_f32(r0, m_data[0]), vmulq_n_f32(r1, m_data[1])),
                                vmulq_n_f32(r2, m_data[2])
                            )
                        );
                        vst1q_f32(
                            out + M + j,
                            vaddq_f32(
                                vaddq_f32(vmulq_n_f32(r0, m_data[4]), vmulq_n_f32(r1, m_data[5])),
                                vmulq_n_f32(r2, m_data[6])
                            )
                        );
                        vst1q_f32(
                            out + 2 * M + j,
                            vaddq_f32(
                                vaddq_f32(vmulq_n_f32(r0, m_data[8]), vmulq_n_f32(r1, m_data[9])),
                                vmulq_n_f32(r2, m_data[10])
                            )
                        );
                    }
                    for (; j < M; ++j) {
                        out[j] = m_data[0] * o[j] + m_data[1] * o[M + j] + m_data[2] * o[2 * M + j];
                        out[M + j] =
                            m_data[4] * o[j] + m_data[5] * o[M + j] + m_data[6] * o[2 * M + j];
                        out[2 * M + j] =
                            m_data[8] * o[j] + m_data[9] * o[M + j] + m_data[10] * o[2 * M + j];
                    }
                }
                return result;
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__SSE2__) || defined(_M_X64)
                if constexpr (kLeft) {
                    // Each column: the columns of *this, in the halves (a0, a1) and (a2, 0), times
                    // its three elements.
                    const __m128d a0_lo{_mm_load_pd(data())};
                    const __m128d a0_hi{_mm_load_pd(data() + 2)};
                    const __m128d a1_lo{_mm_load_pd(data() + 4)};
                    const __m128d a1_hi{_mm_load_pd(data() + 6)};
                    const __m128d a2_lo{_mm_load_pd(data() + 8)};
                    const __m128d a2_hi{_mm_load_pd(data() + 10)};
                    for (std::size_t j{0}; j < M; ++j) {
                        const __m128d b0{_mm_set1_pd(o[3 * j])};
                        const __m128d b1{_mm_set1_pd(o[3 * j + 1])};
                        const __m128d b2{_mm_set1_pd(o[3 * j + 2])};
                        _mm_storeu_pd(
                            out + 3 * j,
                            _mm_add_pd(
                                _mm_add_pd(_mm_mul_pd(a0_lo, b0), _mm_mul_pd(a1_lo, b1)),
                                _mm_mul_pd(a2_lo, b2)
                            )
                        );
                        _mm_store_sd(
                            out + 3 * j + 2,
                            _mm_add_pd(
                                _mm_add_pd(_mm_mul_pd(a0_hi, b0), _mm_mul_pd(a1_hi, b1)),
                                _mm_mul_pd(a2_hi, b2)
                            )
                        );
                    }
                } else {
                    // Two columns a step of the three rows, as under float.
                    const __m128d a00{_mm_set1_pd(m_data[0])};
                    const __m128d a01{_mm_set1_pd(m_data[1])};
                    const __m128d a02{_mm_set1_pd(m_data[2])};
                    const __m128d a10{_mm_set1_pd(m_data[4])};
                    const __m128d a11{_mm_set1_pd(m_data[5])};
                    const __m128d a12{_mm_set1_pd(m_data[6])};
                    const __m128d a20{_mm_set1_pd(m_data[8])};
                    const __m128d a21{_mm_set1_pd(m_data[9])};
                    const __m128d a22{_mm_set1_pd(m_data[10])};
                    std::size_t j{0};
                    for (; j + 2 <= M; j += 2) {
                        const __m128d r0{_mm_loadu_pd(o + j)};
                        const __m128d r1{_mm_loadu_pd(o + M + j)};
                        const __m128d r2{_mm_loadu_pd(o + 2 * M + j)};
                        _mm_storeu_pd(
                            out + j, _mm_add_pd(
                                         _mm_add_pd(_mm_mul_pd(a00, r0), _mm_mul_pd(a01, r1)),
                                         _mm_mul_pd(a02, r2)
                                     )
                        );
                        _mm_storeu_pd(
                            out + M + j, _mm_add_pd(
                                             _mm_add_pd(_mm_mul_pd(a10, r0), _mm_mul_pd(a11, r1)),
                                             _mm_mul_pd(a12, r2)
                                         )
                        );
                        _mm_storeu_pd(
                            out + 2 * M + j,
                            _mm_add_pd(
                                _mm_add_pd(_mm_mul_pd(a20, r0), _mm_mul_pd(a21, r1)),
                                _mm_mul_pd(a22, r2)
                            )
                        );
                    }
                    if constexpr (M % 2 == 1) {
                        out[j] = m_data[0] * o[j] + m_data[1] * o[M + j] + m_data[2] * o[2 * M + j];
                        out[M + j] =
                            m_data[4] * o[j] + m_data[5] * o[M + j] + m_data[6] * o[2 * M + j];
                        out[2 * M + j] =
                            m_data[8] * o[j] + m_data[9] * o[M + j] + m_data[10] * o[2 * M + j];
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                if constexpr (kLeft) {
                    const float64x2_t a0_lo{vld1q_f64(data())};
                    const float64x2_t a0_hi{vld1q_f64(data() + 2)};
                    const float64x2_t a1_lo{vld1q_f64(data() + 4)};
                    const float64x2_t a1_hi{vld1q_f64(data() + 6)};
                    const float64x2_t a2_lo{vld1q_f64(data() + 8)};
                    const float64x2_t a2_hi{vld1q_f64(data() + 10)};
                    for (std::size_t j{0}; j < M; ++j) {
                        vst1q_f64(
                            out + 3 * j,
                            vaddq_f64(
                                vaddq_f64(
                                    vmulq_n_f64(a0_lo, o[3 * j]), vmulq_n_f64(a1_lo, o[3 * j + 1])
                                ),
                                vmulq_n_f64(a2_lo, o[3 * j + 2])
                            )
                        );
                        vst1q_lane_f64(
                            out + 3 * j + 2,
                            vaddq_f64(
                                vaddq_f64(
                                    vmulq_n_f64(a0_hi, o[3 * j]), vmulq_n_f64(a1_hi, o[3 * j + 1])
                                ),
                                vmulq_n_f64(a2_hi, o[3 * j + 2])
                            ),
                            0
                        );
                    }
                } else {
                    std::size_t j{0};
                    for (; j + 2 <= M; j += 2) {
                        const float64x2_t r0{vld1q_f64(o + j)};
                        const float64x2_t r1{vld1q_f64(o + M + j)};
                        const float64x2_t r2{vld1q_f64(o + 2 * M + j)};
                        vst1q_f64(
                            out + j,
                            vaddq_f64(
                                vaddq_f64(vmulq_n_f64(r0, m_data[0]), vmulq_n_f64(r1, m_data[1])),
                                vmulq_n_f64(r2, m_data[2])
                            )
                        );
                        vst1q_f64(
                            out + M + j,
                            vaddq_f64(
                                vaddq_f64(vmulq_n_f64(r0, m_data[4]), vmulq_n_f64(r1, m_data[5])),
                                vmulq_n_f64(r2, m_data[6])
                            )
                        );
                        vst1q_f64(
                            out + 2 * M + j,
                            vaddq_f64(
                                vaddq_f64(vmulq_n_f64(r0, m_data[8]), vmulq_n_f64(r1, m_data[9])),
                                vmulq_n_f64(r2, m_data[10])
                            )
                        );
                    }
                    if constexpr (M % 2 == 1) {
                        out[j] = m_data[0] * o[j] + m_data[1] * o[M + j] + m_data[2] * o[2 * M + j];
                        out[M + j] =
                            m_data[4] * o[j] + m_data[5] * o[M + j] + m_data[6] * o[2 * M + j];
                        out[2 * M + j] =
                            m_data[8] * o[j] + m_data[9] * o[M + j] + m_data[10] * o[2 * M + j];
                    }
                }
                return result;
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__SSE3__) || defined(__AVX__)
                // One element a 64-bit half register, as (a re - b im, b re + a im): element (i, j)
                // of the product sums the elements (i, k) of *this times the elements (k, j) of
                // obj.
                const float* const a{reinterpret_cast<const float*>(data())};
                const float* const b{reinterpret_cast<const float*>(o)};
                float* const c{reinterpret_cast<float*>(out)};
                for (std::size_t j{0}; j < M; ++j) {
                    for (std::size_t i{0}; i < 3; ++i) {
                        __m128 sum{_mm_setzero_ps()};
                        for (std::size_t k{0}; k < 3; ++k) {
                            const std::size_t ik{kLeft ? i + 4 * k : 4 * i + k};
                            const std::size_t kj{kLeft ? k + 3 * j : k * M + j};
                            const __m128 e{_mm_castsi128_ps(
                                _mm_loadl_epi64(reinterpret_cast<const __m128i*>(b + 2 * kj))
                            )};
                            sum = _mm_add_ps(
                                sum, _mm_addsub_ps(
                                         _mm_mul_ps(e, _mm_set1_ps(a[2 * ik])),
                                         _mm_mul_ps(
                                             _mm_shuffle_ps(e, e, _MM_SHUFFLE(2, 3, 0, 1)),
                                             _mm_set1_ps(a[2 * ik + 1])
                                         )
                                     )
                            );
                        }
                        _mm_storel_pi(
                            reinterpret_cast<__m64*>(c + 2 * (kLeft ? i + 3 * j : i * M + j)), sum
                        );
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                // One element a 64-bit register, as (a re, b re) plus (b im, a im) with its real
                // lane negated.
                const float* const a{reinterpret_cast<const float*>(data())};
                const float* const b{reinterpret_cast<const float*>(o)};
                float* const c{reinterpret_cast<float*>(out)};
                const uint32x2_t sign{vreinterpret_u32_u64(vdup_n_u64(0x80000000ULL))};
                for (std::size_t j{0}; j < M; ++j) {
                    for (std::size_t i{0}; i < 3; ++i) {
                        float32x2_t sum{vdup_n_f32(0.0F)};
                        for (std::size_t k{0}; k < 3; ++k) {
                            const std::size_t ik{kLeft ? i + 4 * k : 4 * i + k};
                            const std::size_t kj{kLeft ? k + 3 * j : k * M + j};
                            const float32x2_t e{vld1_f32(b + 2 * kj)};
                            sum = vadd_f32(
                                sum, vadd_f32(
                                         vmul_n_f32(e, a[2 * ik]),
                                         vreinterpret_f32_u32(veor_u32(
                                             vreinterpret_u32_f32(
                                                 vmul_n_f32(vrev64_f32(e), a[2 * ik + 1])
                                             ),
                                             sign
                                         ))
                                     )
                            );
                        }
                        vst1_f32(c + 2 * (kLeft ? i + 3 * j : i * M + j), sum);
                    }
                }
                return result;
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__SSE3__) || defined(__AVX__)
                // One element a register, as (a re - b im, b re + a im): element (i, j) of the
                // product sums the elements (i, k) of *this times the elements (k, j) of obj.
                const double* const a{reinterpret_cast<const double*>(data())};
                const double* const b{reinterpret_cast<const double*>(o)};
                double* const c{reinterpret_cast<double*>(out)};
                for (std::size_t j{0}; j < M; ++j) {
                    for (std::size_t i{0}; i < 3; ++i) {
                        __m128d sum{_mm_setzero_pd()};
                        for (std::size_t k{0}; k < 3; ++k) {
                            const std::size_t ik{kLeft ? i + 4 * k : 4 * i + k};
                            const std::size_t kj{kLeft ? k + 3 * j : k * M + j};
                            const __m128d e{_mm_loadu_pd(b + 2 * kj)};
                            sum = _mm_add_pd(
                                sum, _mm_addsub_pd(
                                         _mm_mul_pd(e, _mm_set1_pd(a[2 * ik])),
                                         _mm_mul_pd(
                                             _mm_shuffle_pd(e, e, 0b01), _mm_set1_pd(a[2 * ik + 1])
                                         )
                                     )
                            );
                        }
                        _mm_storeu_pd(c + 2 * (kLeft ? i + 3 * j : i * M + j), sum);
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                // One element a register, as (a re, b re) plus (b im, a im) with its real lane
                // negated.
                const double* const a{reinterpret_cast<const double*>(data())};
                const double* const b{reinterpret_cast<const double*>(o)};
                double* const c{reinterpret_cast<double*>(out)};
                const uint64x2_t sign{vsetq_lane_u64(0x8000000000000000ULL, vdupq_n_u64(0), 0)};
                for (std::size_t j{0}; j < M; ++j) {
                    for (std::size_t i{0}; i < 3; ++i) {
                        float64x2_t sum{vdupq_n_f64(0.0)};
                        for (std::size_t k{0}; k < 3; ++k) {
                            const std::size_t ik{kLeft ? i + 4 * k : 4 * i + k};
                            const std::size_t kj{kLeft ? k + 3 * j : k * M + j};
                            const float64x2_t e{vld1q_f64(b + 2 * kj)};
                            sum = vaddq_f64(
                                sum, vaddq_f64(
                                         vmulq_n_f64(e, a[2 * ik]),
                                         vreinterpretq_f64_u64(veorq_u64(
                                             vreinterpretq_u64_f64(
                                                 vmulq_n_f64(vextq_f64(e, e, 1), a[2 * ik + 1])
                                             ),
                                             sign
                                         ))
                                     )
                            );
                        }
                        vst1q_f64(c + 2 * (kLeft ? i + 3 * j : i * M + j), sum);
                    }
                }
                return result;
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int32_t> ||
                std::is_same_v<value_type, std::uint32_t>
            ) {
#if defined(__SSE4_1__) || defined(__AVX__)
                const auto* const lanes{reinterpret_cast<const std::int32_t*>(data())};
                const auto* const ob{reinterpret_cast<const std::int32_t*>(o)};
                auto* const ou{reinterpret_cast<std::int32_t*>(out)};
                if constexpr (kLeft) {
                    // Each column: the columns a0, a1 and a2 of *this times its three elements.
                    const __m128i a0{_mm_load_si128(reinterpret_cast<const __m128i*>(lanes))};
                    const __m128i a1{_mm_load_si128(reinterpret_cast<const __m128i*>(lanes + 4))};
                    const __m128i a2{_mm_load_si128(reinterpret_cast<const __m128i*>(lanes + 8))};
                    for (std::size_t j{0}; j < M; ++j) {
                        const __m128i col{_mm_add_epi32(
                            _mm_add_epi32(
                                _mm_mullo_epi32(a0, _mm_set1_epi32(ob[3 * j])),
                                _mm_mullo_epi32(a1, _mm_set1_epi32(ob[3 * j + 1]))
                            ),
                            _mm_mullo_epi32(a2, _mm_set1_epi32(ob[3 * j + 2]))
                        )};
                        _mm_storel_epi64(reinterpret_cast<__m128i*>(ou + 3 * j), col);
                        ou[3 * j + 2] = _mm_extract_epi32(col, 2);
                    }
                } else {
                    // Four columns a step of the three rows, as under float.
                    const __m128i a00{_mm_set1_epi32(lanes[0])};
                    const __m128i a01{_mm_set1_epi32(lanes[1])};
                    const __m128i a02{_mm_set1_epi32(lanes[2])};
                    const __m128i a10{_mm_set1_epi32(lanes[4])};
                    const __m128i a11{_mm_set1_epi32(lanes[5])};
                    const __m128i a12{_mm_set1_epi32(lanes[6])};
                    const __m128i a20{_mm_set1_epi32(lanes[8])};
                    const __m128i a21{_mm_set1_epi32(lanes[9])};
                    const __m128i a22{_mm_set1_epi32(lanes[10])};
                    std::size_t j{0};
                    for (; j + 4 <= M; j += 4) {
                        const __m128i r0{_mm_loadu_si128(reinterpret_cast<const __m128i*>(ob + j))};
                        const __m128i r1{
                            _mm_loadu_si128(reinterpret_cast<const __m128i*>(ob + M + j))
                        };
                        const __m128i r2{
                            _mm_loadu_si128(reinterpret_cast<const __m128i*>(ob + 2 * M + j))
                        };
                        _mm_storeu_si128(
                            reinterpret_cast<__m128i*>(ou + j),
                            _mm_add_epi32(
                                _mm_add_epi32(_mm_mullo_epi32(a00, r0), _mm_mullo_epi32(a01, r1)),
                                _mm_mullo_epi32(a02, r2)
                            )
                        );
                        _mm_storeu_si128(
                            reinterpret_cast<__m128i*>(ou + M + j),
                            _mm_add_epi32(
                                _mm_add_epi32(_mm_mullo_epi32(a10, r0), _mm_mullo_epi32(a11, r1)),
                                _mm_mullo_epi32(a12, r2)
                            )
                        );
                        _mm_storeu_si128(
                            reinterpret_cast<__m128i*>(ou + 2 * M + j),
                            _mm_add_epi32(
                                _mm_add_epi32(_mm_mullo_epi32(a20, r0), _mm_mullo_epi32(a21, r1)),
                                _mm_mullo_epi32(a22, r2)
                            )
                        );
                    }
                    for (; j < M; ++j) {
                        out[j] = m_data[0] * o[j] + m_data[1] * o[M + j] + m_data[2] * o[2 * M + j];
                        out[M + j] =
                            m_data[4] * o[j] + m_data[5] * o[M + j] + m_data[6] * o[2 * M + j];
                        out[2 * M + j] =
                            m_data[8] * o[j] + m_data[9] * o[M + j] + m_data[10] * o[2 * M + j];
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                const auto* const lanes{reinterpret_cast<const std::int32_t*>(data())};
                const auto* const ob{reinterpret_cast<const std::int32_t*>(o)};
                auto* const ou{reinterpret_cast<std::int32_t*>(out)};
                const int32x4_t a0{vld1q_s32(lanes)};
                const int32x4_t a1{vld1q_s32(lanes + 4)};
                const int32x4_t a2{vld1q_s32(lanes + 8)};
                if constexpr (kLeft) {
                    for (std::size_t j{0}; j < M; ++j) {
                        const int32x4_t col{vaddq_s32(
                            vaddq_s32(vmulq_n_s32(a0, ob[3 * j]), vmulq_n_s32(a1, ob[3 * j + 1])),
                            vmulq_n_s32(a2, ob[3 * j + 2])
                        )};
                        vst1_s32(ou + 3 * j, vget_low_s32(col));
                        vst1q_lane_s32(ou + 3 * j + 2, col, 2);
                    }
                } else {
                    // The stored rows a0, a1 and a2 of *this give the coefficients of each row of
                    // the product.
                    std::size_t j{0};
                    for (; j + 4 <= M; j += 4) {
                        const int32x4_t r0{vld1q_s32(ob + j)};
                        const int32x4_t r1{vld1q_s32(ob + M + j)};
                        const int32x4_t r2{vld1q_s32(ob + 2 * M + j)};
                        vst1q_s32(
                            ou + j,
                            vaddq_s32(
                                vaddq_s32(vmulq_laneq_s32(r0, a0, 0), vmulq_laneq_s32(r1, a0, 1)),
                                vmulq_laneq_s32(r2, a0, 2)
                            )
                        );
                        vst1q_s32(
                            ou + M + j,
                            vaddq_s32(
                                vaddq_s32(vmulq_laneq_s32(r0, a1, 0), vmulq_laneq_s32(r1, a1, 1)),
                                vmulq_laneq_s32(r2, a1, 2)
                            )
                        );
                        vst1q_s32(
                            ou + 2 * M + j,
                            vaddq_s32(
                                vaddq_s32(vmulq_laneq_s32(r0, a2, 0), vmulq_laneq_s32(r1, a2, 1)),
                                vmulq_laneq_s32(r2, a2, 2)
                            )
                        );
                    }
                    for (; j < M; ++j) {
                        out[j] = m_data[0] * o[j] + m_data[1] * o[M + j] + m_data[2] * o[2 * M + j];
                        out[M + j] =
                            m_data[4] * o[j] + m_data[5] * o[M + j] + m_data[6] * o[2 * M + j];
                        out[2 * M + j] =
                            m_data[8] * o[j] + m_data[9] * o[M + j] + m_data[10] * o[2 * M + j];
                    }
                }
                return result;
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int64_t> ||
                std::is_same_v<value_type, std::uint64_t>
            ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
                if constexpr (kLeft) {
                    // Each column: the columns a0, a1 and a2 of *this times its three elements,
                    // stored under a mask.
                    const __m256i a0{_mm256_load_si256(reinterpret_cast<const __m256i*>(data()))};
                    const __m256i a1{
                        _mm256_load_si256(reinterpret_cast<const __m256i*>(data() + 4))
                    };
                    const __m256i a2{
                        _mm256_load_si256(reinterpret_cast<const __m256i*>(data() + 8))
                    };
                    for (std::size_t j{0}; j < M; ++j) {
                        _mm256_mask_storeu_epi64(
                            out + 3 * j, 0b0111,
                            _mm256_add_epi64(
                                _mm256_add_epi64(
                                    _mm256_mullo_epi64(
                                        a0, _mm256_set1_epi64x(static_cast<long long>(o[3 * j]))
                                    ),
                                    _mm256_mullo_epi64(
                                        a1, _mm256_set1_epi64x(static_cast<long long>(o[3 * j + 1]))
                                    )
                                ),
                                _mm256_mullo_epi64(
                                    a2, _mm256_set1_epi64x(static_cast<long long>(o[3 * j + 2]))
                                )
                            )
                        );
                    }
                } else {
                    for (std::size_t j{0}; j < M; ++j) {
                        out[j] = m_data[0] * o[j] + m_data[1] * o[M + j] + m_data[2] * o[2 * M + j];
                        out[M + j] =
                            m_data[4] * o[j] + m_data[5] * o[M + j] + m_data[6] * o[2 * M + j];
                        out[2 * M + j] =
                            m_data[8] * o[j] + m_data[9] * o[M + j] + m_data[10] * o[2 * M + j];
                    }
                }
                return result;
#endif
            }
        }
        for (index_type j{0}; j < obj.ncols(); ++j) {
            result[0, j] = operator[](0, 0) * obj[0, j] + operator[](0, 1) * obj[1, j] +
                                                          operator[](0, 2) * obj[2, j];
            result[1, j] = operator[](1, 0) * obj[0, j] + operator[](1, 1) * obj[1, j] +
                                                          operator[](1, 2) * obj[2, j];
            result[2, j] = operator[](2, 0) * obj[0, j] + operator[](2, 1) * obj[1, j] +
                                                          operator[](2, 2) * obj[2, j];
        }
        return result;
    }
    // NOLINTEND(readability-function-size, readability-function-cognitive-complexity)

    [[using gnu: pure, always_inline]]
    constexpr auto trace() const noexcept -> value_type {
        return operator[](0, 0) + operator[](1, 1) + operator[](2, 2);
    }

    // Expanded along the first row, with the cofactors of its elements.
    [[using gnu: pure, always_inline]]
    constexpr auto determinant() const noexcept -> value_type {
        return operator[](0, 0) *
                   (operator[](1, 1) * operator[](2, 2) - operator[](1, 2) * operator[](2, 1)) +
               operator[](0, 1) *
                   (operator[](1, 2) * operator[](2, 0) - operator[](1, 0) * operator[](2, 2)) +
               operator[](0, 2) *
                   (operator[](1, 0) * operator[](2, 1) - operator[](1, 1) * operator[](2, 0));
    }

    // A 4x4 transpose of the stored vectors and a zero fourth one, which also zeroes the padding.
    [[using gnu: always_inline, leaf, hot]]
    constexpr auto selfTransposed() noexcept -> Matrix& {
        if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX__)
            const __m256d r0{_mm256_load_pd(data())};
            const __m256d r1{_mm256_load_pd(data() + 4)};
            const __m256d r2{_mm256_load_pd(data() + 8)};
            const __m256d r3{_mm256_setzero_pd()};
            const __m256d t0{_mm256_unpacklo_pd(r0, r1)};
            const __m256d t1{_mm256_unpackhi_pd(r0, r1)};
            const __m256d t2{_mm256_unpacklo_pd(r2, r3)};
            const __m256d t3{_mm256_unpackhi_pd(r2, r3)};
            _mm256_store_pd(data(), _mm256_permute2f128_pd(t0, t2, 0x20));
            _mm256_store_pd(data() + 4, _mm256_permute2f128_pd(t1, t3, 0x20));
            _mm256_store_pd(data() + 8, _mm256_permute2f128_pd(t0, t2, 0x31));
#elif defined(__SSE2__)
            const __m128d zero{_mm_setzero_pd()};
            const __m128d lo0{_mm_load_pd(data())};
            const __m128d hi0{_mm_load_pd(data() + 2)};
            const __m128d lo1{_mm_load_pd(data() + 4)};
            const __m128d hi1{_mm_load_pd(data() + 6)};
            const __m128d lo2{_mm_load_pd(data() + 8)};
            const __m128d hi2{_mm_load_pd(data() + 10)};
            _mm_store_pd(data(), _mm_unpacklo_pd(lo0, lo1));
            _mm_store_pd(data() + 2, _mm_unpacklo_pd(lo2, zero));
            _mm_store_pd(data() + 4, _mm_unpackhi_pd(lo0, lo1));
            _mm_store_pd(data() + 6, _mm_unpackhi_pd(lo2, zero));
            _mm_store_pd(data() + 8, _mm_unpacklo_pd(hi0, hi1));
            _mm_store_pd(data() + 10, _mm_unpacklo_pd(hi2, zero));
#elif defined(__ARM_NEON)
            const float64x2_t zero{vdupq_n_f64(0.0)};
            const float64x2_t lo0{vld1q_f64(data())};
            const float64x2_t hi0{vld1q_f64(data() + 2)};
            const float64x2_t lo1{vld1q_f64(data() + 4)};
            const float64x2_t hi1{vld1q_f64(data() + 6)};
            const float64x2_t lo2{vld1q_f64(data() + 8)};
            const float64x2_t hi2{vld1q_f64(data() + 10)};
            vst1q_f64(data(), vzip1q_f64(lo0, lo1));
            vst1q_f64(data() + 2, vzip1q_f64(lo2, zero));
            vst1q_f64(data() + 4, vzip2q_f64(lo0, lo1));
            vst1q_f64(data() + 6, vzip2q_f64(lo2, zero));
            vst1q_f64(data() + 8, vzip1q_f64(hi0, hi1));
            vst1q_f64(data() + 10, vzip1q_f64(hi2, zero));
#else
            std::swap(operator[](0, 1), operator[](1, 0));
            std::swap(operator[](0, 2), operator[](2, 0));
            std::swap(operator[](1, 2), operator[](2, 1));
#endif
        } else if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE__)
            __m128 r0{_mm_load_ps(data())};
            __m128 r1{_mm_load_ps(data() + 4)};
            __m128 r2{_mm_load_ps(data() + 8)};
            __m128 r3{_mm_setzero_ps()};
            _MM_TRANSPOSE4_PS(r0, r1, r2, r3);
            _mm_store_ps(data(), r0);
            _mm_store_ps(data() + 4, r1);
            _mm_store_ps(data() + 8, r2);
#elif defined(__ARM_NEON)
            const float32x4_t r0{vld1q_f32(data())};
            const float32x4_t r1{vld1q_f32(data() + 4)};
            const float32x4_t r2{vld1q_f32(data() + 8)};
            const float32x4_t r3{vdupq_n_f32(0.0F)};
            const float64x2_t t0{vreinterpretq_f64_f32(vtrn1q_f32(r0, r1))};
            const float64x2_t t1{vreinterpretq_f64_f32(vtrn2q_f32(r0, r1))};
            const float64x2_t t2{vreinterpretq_f64_f32(vtrn1q_f32(r2, r3))};
            const float64x2_t t3{vreinterpretq_f64_f32(vtrn2q_f32(r2, r3))};
            vst1q_f32(data(), vreinterpretq_f32_f64(vtrn1q_f64(t0, t2)));
            vst1q_f32(data() + 4, vreinterpretq_f32_f64(vtrn1q_f64(t1, t3)));
            vst1q_f32(data() + 8, vreinterpretq_f32_f64(vtrn2q_f64(t0, t2)));
#else
            std::swap(operator[](0, 1), operator[](1, 0));
            std::swap(operator[](0, 2), operator[](2, 0));
            std::swap(operator[](1, 2), operator[](2, 1));
#endif
        } else {
            std::swap(operator[](0, 1), operator[](1, 0));
            std::swap(operator[](0, 2), operator[](2, 0));
            std::swap(operator[](1, 2), operator[](2, 1));
        }
        return *this;
    }
    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto transposed(this Self&& self) noexcept -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self.selfTransposed();
            return static_cast<Self&&>(self);
        } else {
            Matrix result{self};
            result.selfTransposed();
            return result;
        }
    }

    // The conjugate transpose.
    template <typename Self>
    [[using gnu: always_inline]]
    constexpr auto adjoint(this Self&& self) noexcept -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self.selfTransposed().selfConjugated();
            return static_cast<Self&&>(self);
        } else {
            Matrix result{self};
            result.selfTransposed().selfConjugated();
            return result;
        }
    }

    // The adjugate over the determinant, by Cramer's rule: element (i, j) of the inverse is the
    // cofactor of element (j, i) over the determinant, which expands along the first row. A
    // singular matrix throws when parameters are checked and yields infinities or NaNs otherwise.
    [[using gnu: always_inline, hot]]
    constexpr auto selfInversed() noexcept(!BASJOO_CHECK_PARAMS) -> Matrix&
        requires(!std::integral<value_type>)
    {
        const value_type
            c00{operator[](1, 1) * operator[](2, 2) - operator[](1, 2) * operator[](2, 1)};
        const value_type
            c01{operator[](1, 2) * operator[](2, 0) - operator[](1, 0) * operator[](2, 2)};
        const value_type
            c02{operator[](1, 0) * operator[](2, 1) - operator[](1, 1) * operator[](2, 0)};
        const value_type
            c10{operator[](0, 2) * operator[](2, 1) - operator[](0, 1) * operator[](2, 2)};
        const value_type
            c11{operator[](0, 0) * operator[](2, 2) - operator[](0, 2) * operator[](2, 0)};
        const value_type
            c12{operator[](0, 1) * operator[](2, 0) - operator[](0, 0) * operator[](2, 1)};
        const value_type
            c20{operator[](0, 1) * operator[](1, 2) - operator[](0, 2) * operator[](1, 1)};
        const value_type
            c21{operator[](0, 2) * operator[](1, 0) - operator[](0, 0) * operator[](1, 2)};
        const value_type
            c22{operator[](0, 0) * operator[](1, 1) - operator[](0, 1) * operator[](1, 0)};
        const value_type
            det{operator[](0, 0) * c00 + operator[](0, 1) * c01 + operator[](0, 2) * c02};
#if BASJOO_CHECK_PARAMS == 1
        if (det == value_type(0.0)) [[unlikely]] {
            throw std::runtime_error("Matrix::selfInversed: this matrix is singular.");
        }
#endif
        const value_type rdet{value_type(1.0) / det};
        *this = Matrix{c00 * rdet, c10 * rdet, c20 * rdet, c01 * rdet, c11 * rdet,
                       c21 * rdet, c02 * rdet, c12 * rdet, c22 * rdet};
        return *this;
    }
    template <typename Self>
        requires(!std::integral<value_type>)
    [[using gnu: always_inline, hot]]
    constexpr auto inversed(this Self&& self) noexcept(!BASJOO_CHECK_PARAMS) -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self.selfInversed();
            return static_cast<Self&&>(self);
        } else {
            Matrix result{self};
            result.selfInversed();
            return result;
        }
    }

    // Orthonormal columns, every entry of M^T M - I within `tol`, and a positive determinant.
    [[using gnu: pure, always_inline]]
    constexpr auto isRotation(value_type tol = 1E-8) const noexcept -> bool
        requires std::floating_point<value_type>
    {
        const value_type
            g00{operator[](0, 0) * operator[](0, 0) + operator[](1, 0) * operator[](1, 0) +
                                                      operator[](2, 0) * operator[](2, 0)};
        const value_type
            g01{operator[](0, 0) * operator[](0, 1) + operator[](1, 0) * operator[](1, 1) +
                                                      operator[](2, 0) * operator[](2, 1)};
        const value_type
            g02{operator[](0, 0) * operator[](0, 2) + operator[](1, 0) * operator[](1, 2) +
                                                      operator[](2, 0) * operator[](2, 2)};
        const value_type
            g11{operator[](0, 1) * operator[](0, 1) + operator[](1, 1) * operator[](1, 1) +
                                                      operator[](2, 1) * operator[](2, 1)};
        const value_type
            g12{operator[](0, 1) * operator[](0, 2) + operator[](1, 1) * operator[](1, 2) +
                                                      operator[](2, 1) * operator[](2, 2)};
        const value_type
            g22{operator[](0, 2) * operator[](0, 2) + operator[](1, 2) * operator[](1, 2) +
                                                      operator[](2, 2) * operator[](2, 2)};
        return std::abs(g00 - value_type(1.0)) < tol && std::abs(g01) < tol &&
               std::abs(g02) < tol && std::abs(g11 - value_type(1.0)) < tol &&
               std::abs(g12) < tol && std::abs(g22 - value_type(1.0)) < tol &&
               determinant() > value_type(0.0);
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    // Value-initialized, so that the padding never holds garbage: a denormal there would slow
    // every SIMD kernel down.
    storage_type m_data{};
};

template <
    ScalarArithmetic Scalar, std::integral Index = std::size_t,
    DenseLayout Layout = std::layout_left>
using Mat33 = Matrix<Scalar, Index, 3, 3, Layout>;

using Mat33s = Mat33<float>;
using Mat33d = Mat33<double>;
using Mat33c = Mat33<std::complex<float>>;
using Mat33z = Mat33<std::complex<double>>;

template <typename T>
struct isMat33Arithmetic : std::false_type {};

template <ScalarArithmetic T, std::integral Index, DenseLayout Layout, Allocatory Alloc>
struct isMat33Arithmetic<Matrix<T, Index, 3, 3, Layout, Alloc>> : std::true_type {};

template <typename T>
inline constexpr bool isMat33ArithmeticV = isMat33Arithmetic<T>::value;

template <typename T>
concept Mat33Arithmetic = isMat33ArithmeticV<T>;

} // namespace basjoo::math
