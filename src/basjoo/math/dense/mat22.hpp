/**
 * @file math/dense/mat22.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief 2x2 matrix with inline SIMD arithmetic and planar rotations.
 * @version 0.1
 * @date 2026-11-03
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <cstdint>
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
#include "basjoo/math/dense/vec2.hpp"
#include "basjoo/math/type_traits.hpp"

namespace basjoo::math {

// Four contiguous scalars: the columns under layout_left, the rows under layout_right. The
// element-wise kernels run on the storage as packed real lanes, which a complex matrix shares.
template <ScalarArithmetic Scalar, std::integral Index, DenseLayout Layout, Allocatory Alloc>
class alignas(4 * sizeof(Scalar)) Matrix<Scalar, Index, 2, 2, Layout, Alloc> final
    : public MatrixBase<false> {
  public:
    using element_type = typename DenseTrait<Matrix>::element_type;
    using value_type = typename DenseTrait<Matrix>::value_type;
    using pointer = typename DenseTrait<Matrix>::pointer;
    using const_pointer = typename DenseTrait<Matrix>::const_pointer;
    using reference = typename DenseTrait<Matrix>::reference;
    using const_reference = typename DenseTrait<Matrix>::const_reference;
    using extents_type = typename DenseTrait<Matrix>::extents_type;
    using layout_type = typename DenseTrait<Matrix>::layout_type;
    using allocator_type = typename DenseTrait<Matrix>::allocator_type;
    using size_type = typename DenseTrait<Matrix>::size_type;
    using difference_type = typename DenseTrait<Matrix>::difference_type;
    using index_type = typename DenseTrait<Matrix>::index_type;
    using rank_type = typename DenseTrait<Matrix>::rank_type;
    using mapping_type = typename DenseTrait<Matrix>::mapping_type;
    using storage_type = typename DenseTrait<Matrix>::storage_type;

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

    // The elements in row-major reading order, whatever the storage layout.
    [[using gnu: always_inline]]
    constexpr Matrix(value_type m00, value_type m01, value_type m10, value_type m11) noexcept
        : m_data{
              std::is_same_v<layout_type, std::layout_left> ? storage_type{m00, m10, m01, m11}
                                                            : storage_type{m00, m01, m10, m11}
          } {}

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto get_allocator() noexcept -> allocator_type {
        return allocator_type{};
    }

    [[using gnu: always_inline]]
    constexpr explicit Matrix([[maybe_unused]] const allocator_type& alloc) noexcept
        : m_data{} {}

    [[using gnu: always_inline]]
    constexpr explicit Matrix(
        [[maybe_unused]] index_type rows, [[maybe_unused]] index_type cols,
        [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : m_data{} {
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
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : m_data{value, value, value, value} {
#if BASJOO_CHECK_PARAMS == 1
        if (rows != nrows() || cols != ncols()) [[unlikely]] {
            throw std::invalid_argument("Matrix constructor: mismatch matrix sizes detected.");
        }
#endif
    }

    template <typename T>
        requires std::same_as<std::remove_cv_t<typename T::value_type>, value_type> &&
                 (T::rank() == 2)
    [[using gnu: always_inline]] constexpr explicit Matrix(
        const T& matrix_like, [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : m_data{} {
        using other_index_type = typename T::index_type;

#if BASJOO_CHECK_PARAMS == 1
        if (std::cmp_not_equal(matrix_like.extent(0), nrows()) ||
            std::cmp_not_equal(matrix_like.extent(1), ncols())) [[unlikely]] {
            throw std::invalid_argument("Matrix constructor: mismatch matrix sizes detected.");
        }
#endif
        operator[](0, 0) = matrix_like[other_index_type{0}, other_index_type{0}];
        operator[](0, 1) = matrix_like[other_index_type{0}, other_index_type{1}];
        operator[](1, 0) = matrix_like[other_index_type{1}, other_index_type{0}];
        operator[](1, 1) = matrix_like[other_index_type{1}, other_index_type{1}];
    }

    template <ScalarArithmetic U>
    [[using gnu: always_inline]]
    constexpr explicit Matrix(const Matrix<U, Index, 2, 2, Layout>& other) noexcept
        : m_data{
              static_cast<value_type>(other.data()[0]), static_cast<value_type>(other.data()[1]),
              static_cast<value_type>(other.data()[2]), static_cast<value_type>(other.data()[3])
          } {}

    [[using gnu: const, always_inline]]
    static constexpr auto identity() noexcept -> Matrix {
        return Matrix{value_type(1.0), value_type(0.0), value_type(0.0), value_type(1.0)};
    }

    [[using gnu: pure, always_inline]]
    static constexpr auto fromColumns(
        const Vector<value_type, Index, 2, Alloc>& c0, const Vector<value_type, Index, 2, Alloc>& c1
    ) noexcept -> Matrix {
        return Matrix{c0.x(), c1.x(), c0.y(), c1.y()};
    }

    [[using gnu: pure, always_inline]]
    static constexpr auto fromRows(
        const Vector<value_type, Index, 2, Alloc>& r0, const Vector<value_type, Index, 2, Alloc>& r1
    ) noexcept -> Matrix {
        return Matrix{r0.x(), r0.y(), r1.x(), r1.y()};
    }

    // The counterclockwise rotation by `radian`.
    [[using gnu: pure, always_inline]]
    static constexpr auto fromRadian(const std::floating_point auto& radian) noexcept -> Matrix
        requires std::floating_point<value_type>
    {
        const auto c{static_cast<value_type>(std::cos(radian))};
        const auto s{static_cast<value_type>(std::sin(radian))};
        return Matrix{c, -s, s, c};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::mdspan<value_type, extents_type, layout_type>() noexcept {
        return {data(), mapping()};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::mdspan<const value_type, extents_type, layout_type>() const noexcept {
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
        return mapping_type{};
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto extent([[maybe_unused]] rank_type r) noexcept -> index_type {
        return 2;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto nrows() noexcept -> index_type {
        return 2;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto ncols() noexcept -> index_type {
        return 2;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto size() noexcept -> size_type {
        return 4;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto stride(rank_type r) noexcept -> index_type {
        return mapping_type{}.stride(r);
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
        return true;
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
        return true;
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
    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto euclideanSqr() const noexcept -> detail::DenseNormTraitT<value_type> {
        using norm_type = detail::DenseNormTraitT<value_type>;

        if !consteval {
            if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE2__) || defined(_M_X64)
                const __m128 v{_mm_load_ps(data())};
                const __m128 sq{_mm_mul_ps(v, v)};
                const __m128 pairs{_mm_add_ps(sq, _mm_movehl_ps(sq, sq))};
                return _mm_cvtss_f32(
                    _mm_add_ss(pairs, _mm_shuffle_ps(pairs, pairs, _MM_SHUFFLE(1, 1, 1, 1)))
                );
#elif defined(__ARM_NEON)
                const float32x4_t v{vld1q_f32(data())};
                return vaddvq_f32(vmulq_f32(v, v));
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX__)
                const __m256d v{_mm256_load_pd(data())};
                const __m256d sq{_mm256_mul_pd(v, v)};
                const __m128d pairs{
                    _mm_add_pd(_mm256_castpd256_pd128(sq), _mm256_extractf128_pd(sq, 1))
                };
                return _mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)));
#elif defined(__SSE2__) || defined(_M_X64)
                const __m128d lo{_mm_load_pd(data())};
                const __m128d hi{_mm_load_pd(data() + 2)};
                const __m128d pairs{_mm_add_pd(_mm_mul_pd(lo, lo), _mm_mul_pd(hi, hi))};
                return _mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)));
#elif defined(__ARM_NEON)
                const float64x2_t lo{vld1q_f64(data())};
                const float64x2_t hi{vld1q_f64(data() + 2)};
                return vaddvq_f64(vaddq_f64(vmulq_f64(lo, lo), vmulq_f64(hi, hi)));
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                // The squared magnitudes sum the squares of the eight real lanes.
#if defined(__AVX__)
                const float* const p{reinterpret_cast<const float*>(data())};
                const __m256 v{_mm256_load_ps(p)};
                const __m256 sq{_mm256_mul_ps(v, v)};
                const __m128 quads{
                    _mm_add_ps(_mm256_castps256_ps128(sq), _mm256_extractf128_ps(sq, 1))
                };
                const __m128 pairs{_mm_add_ps(quads, _mm_movehl_ps(quads, quads))};
                return _mm_cvtss_f32(
                    _mm_add_ss(pairs, _mm_shuffle_ps(pairs, pairs, _MM_SHUFFLE(1, 1, 1, 1)))
                );
#elif defined(__SSE2__) || defined(_M_X64)
                const float* const p{reinterpret_cast<const float*>(data())};
                const __m128 lo{_mm_load_ps(p)};
                const __m128 hi{_mm_load_ps(p + 4)};
                const __m128 quads{_mm_add_ps(_mm_mul_ps(lo, lo), _mm_mul_ps(hi, hi))};
                const __m128 pairs{_mm_add_ps(quads, _mm_movehl_ps(quads, quads))};
                return _mm_cvtss_f32(
                    _mm_add_ss(pairs, _mm_shuffle_ps(pairs, pairs, _MM_SHUFFLE(1, 1, 1, 1)))
                );
#elif defined(__ARM_NEON)
                const float* const p{reinterpret_cast<const float*>(data())};
                const float32x4_t lo{vld1q_f32(p)};
                const float32x4_t hi{vld1q_f32(p + 4)};
                return vaddvq_f32(vaddq_f32(vmulq_f32(lo, lo), vmulq_f32(hi, hi)));
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                // The squared magnitudes sum the squares of the eight real lanes.
#if defined(__AVX__)
                const double* const p{reinterpret_cast<const double*>(data())};
                const __m256d v0{_mm256_load_pd(p)};
                const __m256d v1{_mm256_load_pd(p + 4)};
                const __m256d sq{_mm256_add_pd(_mm256_mul_pd(v0, v0), _mm256_mul_pd(v1, v1))};
                const __m128d pairs{
                    _mm_add_pd(_mm256_castpd256_pd128(sq), _mm256_extractf128_pd(sq, 1))
                };
                return _mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)));
#elif defined(__SSE2__) || defined(_M_X64)
                const double* const p{reinterpret_cast<const double*>(data())};
                const __m128d v0{_mm_load_pd(p)};
                const __m128d v1{_mm_load_pd(p + 2)};
                const __m128d v2{_mm_load_pd(p + 4)};
                const __m128d v3{_mm_load_pd(p + 6)};
                const __m128d pairs{_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(v0, v0), _mm_mul_pd(v1, v1)),
                    _mm_add_pd(_mm_mul_pd(v2, v2), _mm_mul_pd(v3, v3))
                )};
                return _mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)));
#elif defined(__ARM_NEON)
                const double* const p{reinterpret_cast<const double*>(data())};
                const float64x2_t v0{vld1q_f64(p)};
                const float64x2_t v1{vld1q_f64(p + 2)};
                const float64x2_t v2{vld1q_f64(p + 4)};
                const float64x2_t v3{vld1q_f64(p + 6)};
                return vaddvq_f64(vaddq_f64(
                    vaddq_f64(vmulq_f64(v0, v0), vmulq_f64(v1, v1)),
                    vaddq_f64(vmulq_f64(v2, v2), vmulq_f64(v3, v3))
                ));
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int32_t> ||
                std::is_same_v<value_type, std::uint32_t>
            ) {
#if defined(__SSE4_1__) || defined(__AVX__)
                const __m128i v{_mm_load_si128(reinterpret_cast<const __m128i*>(data()))};
                const __m128i sq{_mm_mullo_epi32(v, v)};
                const __m128i pairs{
                    _mm_add_epi32(sq, _mm_shuffle_epi32(sq, _MM_SHUFFLE(1, 0, 3, 2)))
                };
                return static_cast<norm_type>(_mm_cvtsi128_si32(
                    _mm_add_epi32(pairs, _mm_shuffle_epi32(pairs, _MM_SHUFFLE(2, 3, 0, 1)))
                ));
#elif defined(__ARM_NEON)
                const int32x4_t v{vld1q_s32(reinterpret_cast<const std::int32_t*>(data()))};
                return static_cast<norm_type>(vaddvq_s32(vmulq_s32(v, v)));
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int64_t> ||
                std::is_same_v<value_type, std::uint64_t>
            ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
                const __m256i v{_mm256_load_si256(reinterpret_cast<const __m256i*>(data()))};
                const __m256i sq{_mm256_mullo_epi64(v, v)};
                const __m128i pairs{
                    _mm_add_epi64(_mm256_castsi256_si128(sq), _mm256_extracti128_si256(sq, 1))
                };
                return static_cast<norm_type>(
                    _mm_cvtsi128_si64(_mm_add_epi64(pairs, _mm_unpackhi_epi64(pairs, pairs)))
                );
#endif
            }
        }
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

    // The product with a column vector: a combination of the columns, which a register transposes
    // from the stored rows under layout_right. The result is built in registers: a 16-byte one is
    // bit_cast or assembled from them, a 32-byte one stored whole, as GCC spills a bit_cast of that
    // size.
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(const Vector<value_type, Index, 2, Alloc>& obj) const noexcept
        -> Vector<value_type, Index, 2, Alloc> {
        using vector_type = Vector<value_type, Index, 2, Alloc>;
        [[maybe_unused]] constexpr bool kLeft{std::is_same_v<layout_type, std::layout_left>};

        if !consteval {
            if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE2__) || defined(_M_X64)
                // The columns (m00, m10, m01, m11) times (x, x, y, y): the halves sum to the two
                // elements, side by side.
                const __m128 stored{_mm_load_ps(data())};
                const __m128 cols{
                    kLeft ? stored : _mm_shuffle_ps(stored, stored, _MM_SHUFFLE(3, 1, 2, 0))
                };
                const __m128 v{
                    _mm_castsi128_ps(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(obj.data())))
                };
                const __m128 prod{_mm_mul_ps(cols, _mm_unpacklo_ps(v, v))};
                const __m128 sums{_mm_add_ps(prod, _mm_movehl_ps(prod, prod))};
                return vector_type{
                    _mm_cvtss_f32(sums),
                    _mm_cvtss_f32(_mm_shuffle_ps(sums, sums, _MM_SHUFFLE(1, 1, 1, 1)))
                };
#elif defined(__ARM_NEON)
                const float32x2_t s0{vld1_f32(data())};
                const float32x2_t s1{vld1_f32(data() + 2)};
                const float32x2_t c0{kLeft ? s0 : vzip1_f32(s0, s1)};
                const float32x2_t c1{kLeft ? s1 : vzip2_f32(s0, s1)};
                const float32x2_t v{vld1_f32(obj.data())};
                const float32x2_t r{vadd_f32(vmul_lane_f32(c0, v, 0), vmul_lane_f32(c1, v, 1))};
                return vector_type{vget_lane_f32(r, 0), vget_lane_f32(r, 1)};
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__SSE2__) || defined(_M_X64)
                // The columns (m00, m10) and (m01, m11) times x and y.
                const __m128d s0{_mm_load_pd(data())};
                const __m128d s1{_mm_load_pd(data() + 2)};
                const __m128d c0{kLeft ? s0 : _mm_unpacklo_pd(s0, s1)};
                const __m128d c1{kLeft ? s1 : _mm_unpackhi_pd(s0, s1)};
                const __m128d r{_mm_add_pd(
                    _mm_mul_pd(c0, _mm_set1_pd(obj.x())), _mm_mul_pd(c1, _mm_set1_pd(obj.y()))
                )};
                return vector_type{_mm_cvtsd_f64(r), _mm_cvtsd_f64(_mm_unpackhi_pd(r, r))};
#elif defined(__ARM_NEON)
                const float64x2_t s0{vld1q_f64(data())};
                const float64x2_t s1{vld1q_f64(data() + 2)};
                const float64x2_t c0{kLeft ? s0 : vzip1q_f64(s0, s1)};
                const float64x2_t c1{kLeft ? s1 : vzip2q_f64(s0, s1)};
                const float64x2_t r{vaddq_f64(vmulq_n_f64(c0, obj.x()), vmulq_n_f64(c1, obj.y()))};
                return vector_type{vgetq_lane_f64(r, 0), vgetq_lane_f64(r, 1)};
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__SSE3__) || defined(__AVX__)
                const float* const p{reinterpret_cast<const float*>(data())};
                const float* const v{reinterpret_cast<const float*>(obj.data())};
                const __m128 s0{_mm_load_ps(p)};
                const __m128 s1{_mm_load_ps(p + 4)};
                if constexpr (kLeft) {
                    // The columns combined, each times its coefficient (re, im) as
                    // (a re - b im, b re + a im).
                    return std::bit_cast<vector_type>(_mm_add_ps(
                        _mm_addsub_ps(
                            _mm_mul_ps(s0, _mm_set1_ps(v[0])),
                            _mm_mul_ps(
                                _mm_shuffle_ps(s0, s0, _MM_SHUFFLE(2, 3, 0, 1)), _mm_set1_ps(v[1])
                            )
                        ),
                        _mm_addsub_ps(
                            _mm_mul_ps(s1, _mm_set1_ps(v[2])),
                            _mm_mul_ps(
                                _mm_shuffle_ps(s1, s1, _MM_SHUFFLE(2, 3, 0, 1)), _mm_set1_ps(v[3])
                            )
                        )
                    ));
                } else {
                    // The complex products of each row with (x, y), each folded to one element:
                    // the low halves of the two rows side by side plus their high halves.
                    const __m128 vv{_mm_load_ps(v)};
                    const __m128 v_re{_mm_moveldup_ps(vv)};
                    const __m128 v_im{_mm_movehdup_ps(vv)};
                    const __m128 p0{_mm_addsub_ps(
                        _mm_mul_ps(s0, v_re),
                        _mm_mul_ps(_mm_shuffle_ps(s0, s0, _MM_SHUFFLE(2, 3, 0, 1)), v_im)
                    )};
                    const __m128 p1{_mm_addsub_ps(
                        _mm_mul_ps(s1, v_re),
                        _mm_mul_ps(_mm_shuffle_ps(s1, s1, _MM_SHUFFLE(2, 3, 0, 1)), v_im)
                    )};
                    return std::bit_cast<vector_type>(
                        _mm_add_ps(_mm_movelh_ps(p0, p1), _mm_movehl_ps(p1, p0))
                    );
                }
#elif defined(__ARM_NEON)
                // Each product of an element and a coefficient (re, im) as (a re, b re) plus
                // (b im, a im) with its real lane negated.
                const float* const p{reinterpret_cast<const float*>(data())};
                const float* const v{reinterpret_cast<const float*>(obj.data())};
                const uint32x4_t sign{vreinterpretq_u32_u64(vdupq_n_u64(0x80000000ULL))};
                const float32x4_t s0{vld1q_f32(p)};
                const float32x4_t s1{vld1q_f32(p + 4)};
                if constexpr (kLeft) {
                    return std::bit_cast<vector_type>(vaddq_f32(
                        vaddq_f32(
                            vmulq_n_f32(s0, v[0]),
                            vreinterpretq_f32_u32(veorq_u32(
                                vreinterpretq_u32_f32(vmulq_n_f32(vrev64q_f32(s0), v[1])), sign
                            ))
                        ),
                        vaddq_f32(
                            vmulq_n_f32(s1, v[2]),
                            vreinterpretq_f32_u32(veorq_u32(
                                vreinterpretq_u32_f32(vmulq_n_f32(vrev64q_f32(s1), v[3])), sign
                            ))
                        )
                    ));
                } else {
                    const float32x4_t vv{vld1q_f32(v)};
                    const float32x4_t v_re{vtrn1q_f32(vv, vv)};
                    const float32x4_t v_im{vtrn2q_f32(vv, vv)};
                    const float32x4_t p0{vaddq_f32(
                        vmulq_f32(s0, v_re),
                        vreinterpretq_f32_u32(
                            veorq_u32(vreinterpretq_u32_f32(vmulq_f32(vrev64q_f32(s0), v_im)), sign)
                        )
                    )};
                    const float32x4_t p1{vaddq_f32(
                        vmulq_f32(s1, v_re),
                        vreinterpretq_f32_u32(
                            veorq_u32(vreinterpretq_u32_f32(vmulq_f32(vrev64q_f32(s1), v_im)), sign)
                        )
                    )};
                    return std::bit_cast<vector_type>(vaddq_f32(
                        vcombine_f32(vget_low_f32(p0), vget_low_f32(p1)),
                        vcombine_f32(vget_high_f32(p0), vget_high_f32(p1))
                    ));
                }
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX__)
                const double* const p{reinterpret_cast<const double*>(data())};
                const double* const v{reinterpret_cast<const double*>(obj.data())};
                const __m256d s0{_mm256_load_pd(p)};
                const __m256d s1{_mm256_load_pd(p + 4)};
                if constexpr (kLeft) {
                    // The columns combined, each times its coefficient (re, im) as
                    // (a re - b im, b re + a im).
                    vector_type result;
                    _mm256_store_pd(
                        reinterpret_cast<double*>(result.data()),
                        _mm256_add_pd(
                            _mm256_addsub_pd(
                                _mm256_mul_pd(s0, _mm256_set1_pd(v[0])),
                                _mm256_mul_pd(_mm256_permute_pd(s0, 0b0101), _mm256_set1_pd(v[1]))
                            ),
                            _mm256_addsub_pd(
                                _mm256_mul_pd(s1, _mm256_set1_pd(v[2])),
                                _mm256_mul_pd(_mm256_permute_pd(s1, 0b0101), _mm256_set1_pd(v[3]))
                            )
                        )
                    );
                    return result;
                } else {
                    // The complex products of each row with (x, y), each folded to one element.
                    const __m256d vv{_mm256_load_pd(v)};
                    const __m256d v_re{_mm256_movedup_pd(vv)};
                    const __m256d v_im{_mm256_permute_pd(vv, 0b1111)};
                    const __m256d p0{_mm256_addsub_pd(
                        _mm256_mul_pd(s0, v_re), _mm256_mul_pd(_mm256_permute_pd(s0, 0b0101), v_im)
                    )};
                    const __m256d p1{_mm256_addsub_pd(
                        _mm256_mul_pd(s1, v_re), _mm256_mul_pd(_mm256_permute_pd(s1, 0b0101), v_im)
                    )};
                    vector_type result;
                    _mm256_store_pd(
                        reinterpret_cast<double*>(result.data()),
                        _mm256_add_pd(
                            _mm256_permute2f128_pd(p0, p1, 0x20),
                            _mm256_permute2f128_pd(p0, p1, 0x31)
                        )
                    );
                    return result;
                }
#elif defined(__SSE3__)
                const double* const p{reinterpret_cast<const double*>(data())};
                const double* const v{reinterpret_cast<const double*>(obj.data())};
                // Element i of the product: element i of each column times its coefficient, or the
                // two elements of row i times (x, y), as (a re - b im, b re + a im).
                vector_type result;
                for (std::size_t i{0}; i < 2; ++i) {
                    const __m128d a0{_mm_load_pd(p + 2 * (kLeft ? i : 2 * i))};
                    const __m128d a1{_mm_load_pd(p + 2 * (kLeft ? i + 2 : 2 * i + 1))};
                    _mm_storeu_pd(
                        reinterpret_cast<double*>(result.data() + i),
                        _mm_add_pd(
                            _mm_addsub_pd(
                                _mm_mul_pd(a0, _mm_set1_pd(v[0])),
                                _mm_mul_pd(_mm_shuffle_pd(a0, a0, 0b01), _mm_set1_pd(v[1]))
                            ),
                            _mm_addsub_pd(
                                _mm_mul_pd(a1, _mm_set1_pd(v[2])),
                                _mm_mul_pd(_mm_shuffle_pd(a1, a1, 0b01), _mm_set1_pd(v[3]))
                            )
                        )
                    );
                }
                return result;
#elif defined(__ARM_NEON)
                const double* const p{reinterpret_cast<const double*>(data())};
                const double* const v{reinterpret_cast<const double*>(obj.data())};
                // Element i of the product: element i of each column times its coefficient, or the
                // two elements of row i times (x, y), as (a re, b re) plus (b im, a im) with its
                // real lane negated.
                const uint64x2_t sign{vsetq_lane_u64(0x8000000000000000ULL, vdupq_n_u64(0), 0)};
                vector_type result;
                for (std::size_t i{0}; i < 2; ++i) {
                    const float64x2_t a0{vld1q_f64(p + 2 * (kLeft ? i : 2 * i))};
                    const float64x2_t a1{vld1q_f64(p + 2 * (kLeft ? i + 2 : 2 * i + 1))};
                    vst1q_f64(
                        reinterpret_cast<double*>(result.data() + i),
                        vaddq_f64(
                            vaddq_f64(
                                vmulq_n_f64(a0, v[0]),
                                vreinterpretq_f64_u64(veorq_u64(
                                    vreinterpretq_u64_f64(vmulq_n_f64(vextq_f64(a0, a0, 1), v[1])),
                                    sign
                                ))
                            ),
                            vaddq_f64(
                                vmulq_n_f64(a1, v[2]),
                                vreinterpretq_f64_u64(veorq_u64(
                                    vreinterpretq_u64_f64(vmulq_n_f64(vextq_f64(a1, a1, 1), v[3])),
                                    sign
                                ))
                            )
                        )
                    );
                }
                return result;
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int32_t> ||
                std::is_same_v<value_type, std::uint32_t>
            ) {
#if defined(__SSE4_1__) || defined(__AVX__)
                // The columns (m00, m10, m01, m11) times (x, x, y, y): the halves sum to the two
                // elements, side by side.
                const __m128i stored{_mm_load_si128(reinterpret_cast<const __m128i*>(data()))};
                const __m128i cols{
                    kLeft ? stored : _mm_shuffle_epi32(stored, _MM_SHUFFLE(3, 1, 2, 0))
                };
                const __m128i v{_mm_loadl_epi64(reinterpret_cast<const __m128i*>(obj.data()))};
                const __m128i prod{_mm_mullo_epi32(cols, _mm_unpacklo_epi32(v, v))};
                const __m128i sums{_mm_add_epi32(prod, _mm_unpackhi_epi64(prod, prod))};
                return vector_type{
                    static_cast<value_type>(_mm_cvtsi128_si32(sums)),
                    static_cast<value_type>(_mm_extract_epi32(sums, 1))
                };
#elif defined(__ARM_NEON)
                const int32x2_t s0{vld1_s32(reinterpret_cast<const std::int32_t*>(data()))};
                const int32x2_t s1{vld1_s32(reinterpret_cast<const std::int32_t*>(data()) + 2)};
                const int32x2_t c0{kLeft ? s0 : vzip1_s32(s0, s1)};
                const int32x2_t c1{kLeft ? s1 : vzip2_s32(s0, s1)};
                const int32x2_t v{vld1_s32(reinterpret_cast<const std::int32_t*>(obj.data()))};
                const int32x2_t r{vadd_s32(vmul_lane_s32(c0, v, 0), vmul_lane_s32(c1, v, 1))};
                return vector_type{
                    static_cast<value_type>(vget_lane_s32(r, 0)),
                    static_cast<value_type>(vget_lane_s32(r, 1))
                };
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int64_t> ||
                std::is_same_v<value_type, std::uint64_t>
            ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
                // The columns (m00, m10) and (m01, m11) times x and y.
                const __m128i s0{_mm_load_si128(reinterpret_cast<const __m128i*>(data()))};
                const __m128i s1{_mm_load_si128(reinterpret_cast<const __m128i*>(data() + 2))};
                const __m128i c0{kLeft ? s0 : _mm_unpacklo_epi64(s0, s1)};
                const __m128i c1{kLeft ? s1 : _mm_unpackhi_epi64(s0, s1)};
                const __m128i r{_mm_add_epi64(
                    _mm_mullo_epi64(c0, _mm_set1_epi64x(static_cast<long long>(obj.x()))),
                    _mm_mullo_epi64(c1, _mm_set1_epi64x(static_cast<long long>(obj.y())))
                )};
                return vector_type{
                    static_cast<value_type>(_mm_cvtsi128_si64(r)),
                    static_cast<value_type>(_mm_extract_epi64(r, 1))
                };
#endif
            }
        }
        return vector_type{
            static_cast<value_type>(operator[](0, 0) * obj.x() + operator[](0, 1) * obj.y()),
            static_cast<value_type>(operator[](1, 0) * obj.x() + operator[](1, 1) * obj.y())
        };
    }

    // Stored vector s of the product combines the two stored vectors of one factor with the
    // elements of stored vector s of the other: the columns of *this with those of obj under
    // layout_left, the rows of obj with those of *this under layout_right. The result is built in
    // registers: a 16-byte one is bit_cast from them, a 32-byte one stored whole, as GCC spills a
    // bit_cast of that size.
    // NOLINTBEGIN(readability-function-size)
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(const Matrix& obj) const noexcept -> Matrix {
        constexpr bool kLeft{std::is_same_v<layout_type, std::layout_left>};

        if !consteval {
            [[maybe_unused]] const value_type* const vecs{kLeft ? data() : obj.data()};
            [[maybe_unused]] const value_type* const coefs{kLeft ? obj.data() : data()};
            if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE2__) || defined(_M_X64)
                // (v0, v0) times (c00, c00, c10, c10) plus (v1, v1) times (c01, c01, c11, c11).
                const __m128 v{_mm_load_ps(vecs)};
                const __m128 c{_mm_load_ps(coefs)};
                return std::bit_cast<Matrix>(_mm_add_ps(
                    _mm_mul_ps(_mm_movelh_ps(v, v), _mm_shuffle_ps(c, c, _MM_SHUFFLE(2, 2, 0, 0))),
                    _mm_mul_ps(_mm_movehl_ps(v, v), _mm_shuffle_ps(c, c, _MM_SHUFFLE(3, 3, 1, 1)))
                ));
#elif defined(__ARM_NEON)
                const float32x2_t v0{vld1_f32(vecs)};
                const float32x2_t v1{vld1_f32(vecs + 2)};
                const float32x4_t c{vld1q_f32(coefs)};
                return std::bit_cast<Matrix>(vcombine_f32(
                    vadd_f32(vmul_laneq_f32(v0, c, 0), vmul_laneq_f32(v1, c, 1)),
                    vadd_f32(vmul_laneq_f32(v0, c, 2), vmul_laneq_f32(v1, c, 3))
                ));
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX__)
                // (v0 | v0) times (c00, c00 | c10, c10) plus (v1 | v1) times (c01, c01 | c11, c11).
                const __m256d v{_mm256_load_pd(vecs)};
                const __m256d c{_mm256_load_pd(coefs)};
                Matrix result;
                _mm256_store_pd(
                    result.data(),
                    _mm256_add_pd(
                        _mm256_mul_pd(_mm256_permute2f128_pd(v, v, 0x00), _mm256_movedup_pd(c)),
                        _mm256_mul_pd(
                            _mm256_permute2f128_pd(v, v, 0x11), _mm256_permute_pd(c, 0b1111)
                        )
                    )
                );
                return result;
#elif defined(__SSE2__) || defined(_M_X64)
                const __m128d v0{_mm_load_pd(vecs)};
                const __m128d v1{_mm_load_pd(vecs + 2)};
                Matrix result;
                _mm_store_pd(
                    result.data(),
                    _mm_add_pd(
                        _mm_mul_pd(v0, _mm_set1_pd(coefs[0])), _mm_mul_pd(v1, _mm_set1_pd(coefs[1]))
                    )
                );
                _mm_store_pd(
                    result.data() + 2,
                    _mm_add_pd(
                        _mm_mul_pd(v0, _mm_set1_pd(coefs[2])), _mm_mul_pd(v1, _mm_set1_pd(coefs[3]))
                    )
                );
                return result;
#elif defined(__ARM_NEON)
                const float64x2_t v0{vld1q_f64(vecs)};
                const float64x2_t v1{vld1q_f64(vecs + 2)};
                Matrix result;
                vst1q_f64(
                    result.data(), vaddq_f64(vmulq_n_f64(v0, coefs[0]), vmulq_n_f64(v1, coefs[1]))
                );
                vst1q_f64(
                    result.data() + 2,
                    vaddq_f64(vmulq_n_f64(v0, coefs[2]), vmulq_n_f64(v1, coefs[3]))
                );
                return result;
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__AVX__)
                const float* const v{reinterpret_cast<const float*>(vecs)};
                const float* const c{reinterpret_cast<const float*>(coefs)};
                // (v0 | v0) and (v1 | v1) times the complex coefficients broadcast in each 128-bit
                // lane, (c00 | c10) and (c01 | c11), as (a re - b im, b re + a im).
                const __m256 vv{_mm256_load_ps(v)};
                const __m256 cc{_mm256_load_ps(c)};
                const __m256 v0{_mm256_permute2f128_ps(vv, vv, 0x00)};
                const __m256 v1{_mm256_permute2f128_ps(vv, vv, 0x11)};
                const __m256 c0{_mm256_permute_ps(cc, _MM_SHUFFLE(1, 0, 1, 0))};
                const __m256 c1{_mm256_permute_ps(cc, _MM_SHUFFLE(3, 2, 3, 2))};
                Matrix result;
                _mm256_store_ps(
                    reinterpret_cast<float*>(result.data()),
                    _mm256_add_ps(
                        _mm256_addsub_ps(
                            _mm256_mul_ps(v0, _mm256_moveldup_ps(c0)),
                            _mm256_mul_ps(
                                _mm256_permute_ps(v0, _MM_SHUFFLE(2, 3, 0, 1)),
                                _mm256_movehdup_ps(c0)
                            )
                        ),
                        _mm256_addsub_ps(
                            _mm256_mul_ps(v1, _mm256_moveldup_ps(c1)),
                            _mm256_mul_ps(
                                _mm256_permute_ps(v1, _MM_SHUFFLE(2, 3, 0, 1)),
                                _mm256_movehdup_ps(c1)
                            )
                        )
                    )
                );
                return result;
#elif defined(__SSE3__)
                const float* const v{reinterpret_cast<const float*>(vecs)};
                const float* const c{reinterpret_cast<const float*>(coefs)};
                // Each stored vector of the product: v0 and v1 times their complex coefficients,
                // as (a re - b im, b re + a im).
                const __m128 v0{_mm_load_ps(v)};
                const __m128 v1{_mm_load_ps(v + 4)};
                Matrix result;
                float* const out{reinterpret_cast<float*>(result.data())};
                for (std::size_t s{0}; s < 2; ++s) {
                    _mm_store_ps(
                        out + 4 * s, _mm_add_ps(
                                         _mm_addsub_ps(
                                             _mm_mul_ps(v0, _mm_set1_ps(c[4 * s])),
                                             _mm_mul_ps(
                                                 _mm_shuffle_ps(v0, v0, _MM_SHUFFLE(2, 3, 0, 1)),
                                                 _mm_set1_ps(c[4 * s + 1])
                                             )
                                         ),
                                         _mm_addsub_ps(
                                             _mm_mul_ps(v1, _mm_set1_ps(c[4 * s + 2])),
                                             _mm_mul_ps(
                                                 _mm_shuffle_ps(v1, v1, _MM_SHUFFLE(2, 3, 0, 1)),
                                                 _mm_set1_ps(c[4 * s + 3])
                                             )
                                         )
                                     )
                    );
                }
                return result;
#elif defined(__ARM_NEON)
                const float* const v{reinterpret_cast<const float*>(vecs)};
                const float* const c{reinterpret_cast<const float*>(coefs)};
                // Each stored vector of the product: v0 and v1 times their complex coefficients,
                // as (a re, b re) plus (b im, a im) with its real lane negated.
                const uint32x4_t sign{vreinterpretq_u32_u64(vdupq_n_u64(0x80000000ULL))};
                const float32x4_t v0{vld1q_f32(v)};
                const float32x4_t v1{vld1q_f32(v + 4)};
                Matrix result;
                float* const out{reinterpret_cast<float*>(result.data())};
                for (std::size_t s{0}; s < 2; ++s) {
                    vst1q_f32(
                        out + 4 * s, vaddq_f32(
                                         vaddq_f32(
                                             vmulq_n_f32(v0, c[4 * s]),
                                             vreinterpretq_f32_u32(veorq_u32(
                                                 vreinterpretq_u32_f32(
                                                     vmulq_n_f32(vrev64q_f32(v0), c[4 * s + 1])
                                                 ),
                                                 sign
                                             ))
                                         ),
                                         vaddq_f32(
                                             vmulq_n_f32(v1, c[4 * s + 2]),
                                             vreinterpretq_f32_u32(veorq_u32(
                                                 vreinterpretq_u32_f32(
                                                     vmulq_n_f32(vrev64q_f32(v1), c[4 * s + 3])
                                                 ),
                                                 sign
                                             ))
                                         )
                                     )
                    );
                }
                return result;
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX__)
                const double* const v{reinterpret_cast<const double*>(vecs)};
                const double* const c{reinterpret_cast<const double*>(coefs)};
                // Each stored vector of the product: v0 and v1 times their complex coefficients,
                // as (a re - b im, b re + a im).
                const __m256d v0{_mm256_load_pd(v)};
                const __m256d v1{_mm256_load_pd(v + 4)};
                Matrix result;
                double* const out{reinterpret_cast<double*>(result.data())};
                for (std::size_t s{0}; s < 2; ++s) {
                    _mm256_store_pd(
                        out + 4 * s,
                        _mm256_add_pd(
                            _mm256_addsub_pd(
                                _mm256_mul_pd(v0, _mm256_set1_pd(c[4 * s])),
                                _mm256_mul_pd(
                                    _mm256_permute_pd(v0, 0b0101), _mm256_set1_pd(c[4 * s + 1])
                                )
                            ),
                            _mm256_addsub_pd(
                                _mm256_mul_pd(v1, _mm256_set1_pd(c[4 * s + 2])),
                                _mm256_mul_pd(
                                    _mm256_permute_pd(v1, 0b0101), _mm256_set1_pd(c[4 * s + 3])
                                )
                            )
                        )
                    );
                }
                return result;
#elif defined(__SSE3__)
                const double* const v{reinterpret_cast<const double*>(vecs)};
                const double* const c{reinterpret_cast<const double*>(coefs)};
                // Element i of stored vector s of the product, as (a re - b im, b re + a im).
                Matrix result;
                double* const out{reinterpret_cast<double*>(result.data())};
                for (std::size_t s{0}; s < 2; ++s) {
                    for (std::size_t i{0}; i < 2; ++i) {
                        const __m128d a0{_mm_load_pd(v + 2 * i)};
                        const __m128d a1{_mm_load_pd(v + 4 + 2 * i)};
                        _mm_store_pd(
                            out + 4 * s + 2 * i,
                            _mm_add_pd(
                                _mm_addsub_pd(
                                    _mm_mul_pd(a0, _mm_set1_pd(c[4 * s])),
                                    _mm_mul_pd(
                                        _mm_shuffle_pd(a0, a0, 0b01), _mm_set1_pd(c[4 * s + 1])
                                    )
                                ),
                                _mm_addsub_pd(
                                    _mm_mul_pd(a1, _mm_set1_pd(c[4 * s + 2])),
                                    _mm_mul_pd(
                                        _mm_shuffle_pd(a1, a1, 0b01), _mm_set1_pd(c[4 * s + 3])
                                    )
                                )
                            )
                        );
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                const double* const v{reinterpret_cast<const double*>(vecs)};
                const double* const c{reinterpret_cast<const double*>(coefs)};
                // Element i of stored vector s of the product, as (a re, b re) plus (b im, a im)
                // with its real lane negated.
                const uint64x2_t sign{vsetq_lane_u64(0x8000000000000000ULL, vdupq_n_u64(0), 0)};
                Matrix result;
                double* const out{reinterpret_cast<double*>(result.data())};
                for (std::size_t s{0}; s < 2; ++s) {
                    for (std::size_t i{0}; i < 2; ++i) {
                        const float64x2_t a0{vld1q_f64(v + 2 * i)};
                        const float64x2_t a1{vld1q_f64(v + 4 + 2 * i)};
                        vst1q_f64(
                            out + 4 * s + 2 * i,
                            vaddq_f64(
                                vaddq_f64(
                                    vmulq_n_f64(a0, c[4 * s]),
                                    vreinterpretq_f64_u64(veorq_u64(
                                        vreinterpretq_u64_f64(
                                            vmulq_n_f64(vextq_f64(a0, a0, 1), c[4 * s + 1])
                                        ),
                                        sign
                                    ))
                                ),
                                vaddq_f64(
                                    vmulq_n_f64(a1, c[4 * s + 2]),
                                    vreinterpretq_f64_u64(veorq_u64(
                                        vreinterpretq_u64_f64(
                                            vmulq_n_f64(vextq_f64(a1, a1, 1), c[4 * s + 3])
                                        ),
                                        sign
                                    ))
                                )
                            )
                        );
                    }
                }
                return result;
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int32_t> ||
                std::is_same_v<value_type, std::uint32_t>
            ) {
#if defined(__SSE4_1__) || defined(__AVX__)
                // (v0, v0) times (c00, c00, c10, c10) plus (v1, v1) times (c01, c01, c11, c11).
                const __m128i v{_mm_load_si128(reinterpret_cast<const __m128i*>(vecs))};
                const __m128i c{_mm_load_si128(reinterpret_cast<const __m128i*>(coefs))};
                return std::bit_cast<Matrix>(_mm_add_epi32(
                    _mm_mullo_epi32(
                        _mm_unpacklo_epi64(v, v), _mm_shuffle_epi32(c, _MM_SHUFFLE(2, 2, 0, 0))
                    ),
                    _mm_mullo_epi32(
                        _mm_unpackhi_epi64(v, v), _mm_shuffle_epi32(c, _MM_SHUFFLE(3, 3, 1, 1))
                    )
                ));
#elif defined(__ARM_NEON)
                const int32x2_t v0{vld1_s32(reinterpret_cast<const std::int32_t*>(vecs))};
                const int32x2_t v1{vld1_s32(reinterpret_cast<const std::int32_t*>(vecs) + 2)};
                const int32x4_t c{vld1q_s32(reinterpret_cast<const std::int32_t*>(coefs))};
                return std::bit_cast<Matrix>(vcombine_s32(
                    vadd_s32(vmul_laneq_s32(v0, c, 0), vmul_laneq_s32(v1, c, 1)),
                    vadd_s32(vmul_laneq_s32(v0, c, 2), vmul_laneq_s32(v1, c, 3))
                ));
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int64_t> ||
                std::is_same_v<value_type, std::uint64_t>
            ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
                // (v0 | v0) times (c00, c00 | c10, c10) plus (v1 | v1) times (c01, c01 | c11, c11).
                const __m256i v{_mm256_load_si256(reinterpret_cast<const __m256i*>(vecs))};
                const __m256i c{_mm256_load_si256(reinterpret_cast<const __m256i*>(coefs))};
                Matrix result;
                _mm256_store_si256(
                    reinterpret_cast<__m256i*>(result.data()),
                    _mm256_add_epi64(
                        _mm256_mullo_epi64(
                            _mm256_permute4x64_epi64(v, _MM_SHUFFLE(1, 0, 1, 0)),
                            _mm256_permute4x64_epi64(c, _MM_SHUFFLE(2, 2, 0, 0))
                        ),
                        _mm256_mullo_epi64(
                            _mm256_permute4x64_epi64(v, _MM_SHUFFLE(3, 2, 3, 2)),
                            _mm256_permute4x64_epi64(c, _MM_SHUFFLE(3, 3, 1, 1))
                        )
                    )
                );
                return result;
#endif
            }
        }
        return Matrix{
            static_cast<value_type>(operator[](0, 0) * obj[0, 0] + operator[](0, 1) * obj[1, 0]),
            static_cast<value_type>(operator[](0, 0) * obj[0, 1] + operator[](0, 1) * obj[1, 1]),
            static_cast<value_type>(operator[](1, 0) * obj[0, 0] + operator[](1, 1) * obj[1, 0]),
            static_cast<value_type>(operator[](1, 0) * obj[0, 1] + operator[](1, 1) * obj[1, 1])
        };
    }
    // NOLINTEND(readability-function-size)

    // The product with a general matrix of two rows. Under layout_left, each column of the product
    // combines the columns of *this; under layout_right, each row combines the rows of obj.
    // NOLINTBEGIN(readability-function-size, readability-function-cognitive-complexity)
    template <std::size_t M>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(const Matrix<value_type, Index, 2, M, Layout, Alloc>& obj) const noexcept
        -> Matrix<value_type, Index, 2, M, Layout, Alloc> {
        [[maybe_unused]] constexpr bool kLeft{std::is_same_v<layout_type, std::layout_left>};

        // Every element is written below.
        Matrix<value_type, Index, 2, M, Layout, Alloc> result;
        if !consteval {
            [[maybe_unused]] const value_type* const o{obj.data()};
            [[maybe_unused]] value_type* const out{result.data()};
            if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE2__) || defined(_M_X64)
                const __m128 a{_mm_load_ps(data())};
                if constexpr (kLeft) {
                    // Two columns a step: (a0, a0) times (o0j, o0j, o0j', o0j') plus (a1, a1) times
                    // (o1j, o1j, o1j', o1j'), for the columns a0 and a1 of *this.
                    const __m128 a0{_mm_movelh_ps(a, a)};
                    const __m128 a1{_mm_movehl_ps(a, a)};
                    std::size_t j{0};
                    for (; j + 2 <= M; j += 2) {
                        const __m128 c{_mm_loadu_ps(o + 2 * j)};
                        _mm_storeu_ps(
                            out + 2 * j,
                            _mm_add_ps(
                                _mm_mul_ps(a0, _mm_shuffle_ps(c, c, _MM_SHUFFLE(2, 2, 0, 0))),
                                _mm_mul_ps(a1, _mm_shuffle_ps(c, c, _MM_SHUFFLE(3, 3, 1, 1)))
                            )
                        );
                    }
                    if constexpr (M % 2 == 1) {
                        const __m128 c{_mm_castsi128_ps(
                            _mm_loadl_epi64(reinterpret_cast<const __m128i*>(o + 2 * j))
                        )};
                        _mm_storel_pi(
                            reinterpret_cast<__m64*>(out + 2 * j),
                            _mm_add_ps(
                                _mm_mul_ps(a0, _mm_shuffle_ps(c, c, _MM_SHUFFLE(0, 0, 0, 0))),
                                _mm_mul_ps(a1, _mm_shuffle_ps(c, c, _MM_SHUFFLE(1, 1, 1, 1)))
                            )
                        );
                    }
                } else {
                    // Four columns a step of both rows: (a00, a01) and (a10, a11) against the rows
                    // of obj.
                    const __m128 a00{_mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 0, 0, 0))};
                    const __m128 a01{_mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 1, 1, 1))};
                    const __m128 a10{_mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 2, 2, 2))};
                    const __m128 a11{_mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 3, 3, 3))};
                    std::size_t j{0};
                    for (; j + 4 <= M; j += 4) {
                        const __m128 r0{_mm_loadu_ps(o + j)};
                        const __m128 r1{_mm_loadu_ps(o + M + j)};
                        _mm_storeu_ps(
                            out + j, _mm_add_ps(_mm_mul_ps(a00, r0), _mm_mul_ps(a01, r1))
                        );
                        _mm_storeu_ps(
                            out + M + j, _mm_add_ps(_mm_mul_ps(a10, r0), _mm_mul_ps(a11, r1))
                        );
                    }
                    for (; j < M; ++j) {
                        out[j] = m_data[0] * o[j] + m_data[1] * o[M + j];
                        out[M + j] = m_data[2] * o[j] + m_data[3] * o[M + j];
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                if constexpr (kLeft) {
                    const float32x2_t a0{vld1_f32(data())};
                    const float32x2_t a1{vld1_f32(data() + 2)};
                    for (std::size_t j{0}; j < M; ++j) {
                        const float32x2_t c{vld1_f32(o + 2 * j)};
                        vst1_f32(
                            out + 2 * j, vadd_f32(vmul_lane_f32(a0, c, 0), vmul_lane_f32(a1, c, 1))
                        );
                    }
                } else {
                    std::size_t j{0};
                    for (; j + 4 <= M; j += 4) {
                        const float32x4_t r0{vld1q_f32(o + j)};
                        const float32x4_t r1{vld1q_f32(o + M + j)};
                        vst1q_f32(
                            out + j,
                            vaddq_f32(vmulq_n_f32(r0, m_data[0]), vmulq_n_f32(r1, m_data[1]))
                        );
                        vst1q_f32(
                            out + M + j,
                            vaddq_f32(vmulq_n_f32(r0, m_data[2]), vmulq_n_f32(r1, m_data[3]))
                        );
                    }
                    for (; j < M; ++j) {
                        out[j] = m_data[0] * o[j] + m_data[1] * o[M + j];
                        out[M + j] = m_data[2] * o[j] + m_data[3] * o[M + j];
                    }
                }
                return result;
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__SSE2__) || defined(_M_X64)
                if constexpr (kLeft) {
                    // Each column: the columns a0 and a1 of *this times its two elements.
                    const __m128d a0{_mm_load_pd(data())};
                    const __m128d a1{_mm_load_pd(data() + 2)};
                    for (std::size_t j{0}; j < M; ++j) {
                        const __m128d c{_mm_loadu_pd(o + 2 * j)};
                        _mm_storeu_pd(
                            out + 2 * j, _mm_add_pd(
                                             _mm_mul_pd(a0, _mm_unpacklo_pd(c, c)),
                                             _mm_mul_pd(a1, _mm_unpackhi_pd(c, c))
                                         )
                        );
                    }
                } else {
                    const __m128d a00{_mm_set1_pd(m_data[0])};
                    const __m128d a01{_mm_set1_pd(m_data[1])};
                    const __m128d a10{_mm_set1_pd(m_data[2])};
                    const __m128d a11{_mm_set1_pd(m_data[3])};
                    std::size_t j{0};
                    for (; j + 2 <= M; j += 2) {
                        const __m128d r0{_mm_loadu_pd(o + j)};
                        const __m128d r1{_mm_loadu_pd(o + M + j)};
                        _mm_storeu_pd(
                            out + j, _mm_add_pd(_mm_mul_pd(a00, r0), _mm_mul_pd(a01, r1))
                        );
                        _mm_storeu_pd(
                            out + M + j, _mm_add_pd(_mm_mul_pd(a10, r0), _mm_mul_pd(a11, r1))
                        );
                    }
                    if constexpr (M % 2 == 1) {
                        out[j] = m_data[0] * o[j] + m_data[1] * o[M + j];
                        out[M + j] = m_data[2] * o[j] + m_data[3] * o[M + j];
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                if constexpr (kLeft) {
                    const float64x2_t a0{vld1q_f64(data())};
                    const float64x2_t a1{vld1q_f64(data() + 2)};
                    for (std::size_t j{0}; j < M; ++j) {
                        const float64x2_t c{vld1q_f64(o + 2 * j)};
                        vst1q_f64(
                            out + 2 * j,
                            vaddq_f64(vmulq_laneq_f64(a0, c, 0), vmulq_laneq_f64(a1, c, 1))
                        );
                    }
                } else {
                    std::size_t j{0};
                    for (; j + 2 <= M; j += 2) {
                        const float64x2_t r0{vld1q_f64(o + j)};
                        const float64x2_t r1{vld1q_f64(o + M + j)};
                        vst1q_f64(
                            out + j,
                            vaddq_f64(vmulq_n_f64(r0, m_data[0]), vmulq_n_f64(r1, m_data[1]))
                        );
                        vst1q_f64(
                            out + M + j,
                            vaddq_f64(vmulq_n_f64(r0, m_data[2]), vmulq_n_f64(r1, m_data[3]))
                        );
                    }
                    if constexpr (M % 2 == 1) {
                        out[j] = m_data[0] * o[j] + m_data[1] * o[M + j];
                        out[M + j] = m_data[2] * o[j] + m_data[3] * o[M + j];
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
                    for (std::size_t i{0}; i < 2; ++i) {
                        __m128 sum{_mm_setzero_ps()};
                        for (std::size_t k{0}; k < 2; ++k) {
                            const std::size_t ik{kLeft ? i + 2 * k : 2 * i + k};
                            const std::size_t kj{kLeft ? k + 2 * j : k * M + j};
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
                            reinterpret_cast<__m64*>(c + 2 * (kLeft ? i + 2 * j : i * M + j)), sum
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
                    for (std::size_t i{0}; i < 2; ++i) {
                        float32x2_t sum{vdup_n_f32(0.0F)};
                        for (std::size_t k{0}; k < 2; ++k) {
                            const std::size_t ik{kLeft ? i + 2 * k : 2 * i + k};
                            const std::size_t kj{kLeft ? k + 2 * j : k * M + j};
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
                        vst1_f32(c + 2 * (kLeft ? i + 2 * j : i * M + j), sum);
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
                    for (std::size_t i{0}; i < 2; ++i) {
                        __m128d sum{_mm_setzero_pd()};
                        for (std::size_t k{0}; k < 2; ++k) {
                            const std::size_t ik{kLeft ? i + 2 * k : 2 * i + k};
                            const std::size_t kj{kLeft ? k + 2 * j : k * M + j};
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
                        _mm_storeu_pd(c + 2 * (kLeft ? i + 2 * j : i * M + j), sum);
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
                    for (std::size_t i{0}; i < 2; ++i) {
                        float64x2_t sum{vdupq_n_f64(0.0)};
                        for (std::size_t k{0}; k < 2; ++k) {
                            const std::size_t ik{kLeft ? i + 2 * k : 2 * i + k};
                            const std::size_t kj{kLeft ? k + 2 * j : k * M + j};
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
                        vst1q_f64(c + 2 * (kLeft ? i + 2 * j : i * M + j), sum);
                    }
                }
                return result;
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int32_t> ||
                std::is_same_v<value_type, std::uint32_t>
            ) {
#if defined(__SSE4_1__) || defined(__AVX__)
                const auto* const ob{reinterpret_cast<const std::int32_t*>(o)};
                auto* const ou{reinterpret_cast<std::int32_t*>(out)};
                const __m128i a{_mm_load_si128(reinterpret_cast<const __m128i*>(data()))};
                if constexpr (kLeft) {
                    // Two columns a step: (a0, a0) times (o0j, o0j, o0j', o0j') plus (a1, a1) times
                    // (o1j, o1j, o1j', o1j'), for the columns a0 and a1 of *this.
                    const __m128i a0{_mm_unpacklo_epi64(a, a)};
                    const __m128i a1{_mm_unpackhi_epi64(a, a)};
                    std::size_t j{0};
                    for (; j + 2 <= M; j += 2) {
                        const __m128i c{
                            _mm_loadu_si128(reinterpret_cast<const __m128i*>(ob + 2 * j))
                        };
                        _mm_storeu_si128(
                            reinterpret_cast<__m128i*>(ou + 2 * j),
                            _mm_add_epi32(
                                _mm_mullo_epi32(a0, _mm_shuffle_epi32(c, _MM_SHUFFLE(2, 2, 0, 0))),
                                _mm_mullo_epi32(a1, _mm_shuffle_epi32(c, _MM_SHUFFLE(3, 3, 1, 1)))
                            )
                        );
                    }
                    if constexpr (M % 2 == 1) {
                        const __m128i c{
                            _mm_loadl_epi64(reinterpret_cast<const __m128i*>(ob + 2 * j))
                        };
                        _mm_storel_epi64(
                            reinterpret_cast<__m128i*>(ou + 2 * j),
                            _mm_add_epi32(
                                _mm_mullo_epi32(a0, _mm_shuffle_epi32(c, _MM_SHUFFLE(0, 0, 0, 0))),
                                _mm_mullo_epi32(a1, _mm_shuffle_epi32(c, _MM_SHUFFLE(1, 1, 1, 1)))
                            )
                        );
                    }
                } else {
                    const __m128i a00{_mm_shuffle_epi32(a, _MM_SHUFFLE(0, 0, 0, 0))};
                    const __m128i a01{_mm_shuffle_epi32(a, _MM_SHUFFLE(1, 1, 1, 1))};
                    const __m128i a10{_mm_shuffle_epi32(a, _MM_SHUFFLE(2, 2, 2, 2))};
                    const __m128i a11{_mm_shuffle_epi32(a, _MM_SHUFFLE(3, 3, 3, 3))};
                    std::size_t j{0};
                    for (; j + 4 <= M; j += 4) {
                        const __m128i r0{_mm_loadu_si128(reinterpret_cast<const __m128i*>(ob + j))};
                        const __m128i r1{
                            _mm_loadu_si128(reinterpret_cast<const __m128i*>(ob + M + j))
                        };
                        _mm_storeu_si128(
                            reinterpret_cast<__m128i*>(ou + j),
                            _mm_add_epi32(_mm_mullo_epi32(a00, r0), _mm_mullo_epi32(a01, r1))
                        );
                        _mm_storeu_si128(
                            reinterpret_cast<__m128i*>(ou + M + j),
                            _mm_add_epi32(_mm_mullo_epi32(a10, r0), _mm_mullo_epi32(a11, r1))
                        );
                    }
                    for (; j < M; ++j) {
                        out[j] = m_data[0] * o[j] + m_data[1] * o[M + j];
                        out[M + j] = m_data[2] * o[j] + m_data[3] * o[M + j];
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                const auto* const ob{reinterpret_cast<const std::int32_t*>(o)};
                auto* const ou{reinterpret_cast<std::int32_t*>(out)};
                if constexpr (kLeft) {
                    const int32x2_t a0{vld1_s32(reinterpret_cast<const std::int32_t*>(data()))};
                    const int32x2_t a1{vld1_s32(reinterpret_cast<const std::int32_t*>(data()) + 2)};
                    for (std::size_t j{0}; j < M; ++j) {
                        const int32x2_t c{vld1_s32(ob + 2 * j)};
                        vst1_s32(
                            ou + 2 * j, vadd_s32(vmul_lane_s32(a0, c, 0), vmul_lane_s32(a1, c, 1))
                        );
                    }
                } else {
                    const int32x4_t a{vld1q_s32(reinterpret_cast<const std::int32_t*>(data()))};
                    std::size_t j{0};
                    for (; j + 4 <= M; j += 4) {
                        const int32x4_t r0{vld1q_s32(ob + j)};
                        const int32x4_t r1{vld1q_s32(ob + M + j)};
                        vst1q_s32(
                            ou + j, vaddq_s32(vmulq_laneq_s32(r0, a, 0), vmulq_laneq_s32(r1, a, 1))
                        );
                        vst1q_s32(
                            ou + M + j,
                            vaddq_s32(vmulq_laneq_s32(r0, a, 2), vmulq_laneq_s32(r1, a, 3))
                        );
                    }
                    for (; j < M; ++j) {
                        out[j] = m_data[0] * o[j] + m_data[1] * o[M + j];
                        out[M + j] = m_data[2] * o[j] + m_data[3] * o[M + j];
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
                    // Each column: the columns a0 and a1 of *this times its two elements.
                    const __m128i a0{_mm_load_si128(reinterpret_cast<const __m128i*>(data()))};
                    const __m128i a1{_mm_load_si128(reinterpret_cast<const __m128i*>(data() + 2))};
                    for (std::size_t j{0}; j < M; ++j) {
                        const __m128i c{
                            _mm_loadu_si128(reinterpret_cast<const __m128i*>(o + 2 * j))
                        };
                        _mm_storeu_si128(
                            reinterpret_cast<__m128i*>(out + 2 * j),
                            _mm_add_epi64(
                                _mm_mullo_epi64(a0, _mm_unpacklo_epi64(c, c)),
                                _mm_mullo_epi64(a1, _mm_unpackhi_epi64(c, c))
                            )
                        );
                    }
                } else {
                    for (std::size_t j{0}; j < M; ++j) {
                        out[j] = m_data[0] * o[j] + m_data[1] * o[M + j];
                        out[M + j] = m_data[2] * o[j] + m_data[3] * o[M + j];
                    }
                }
                return result;
#endif
            }
        }
        for (index_type j{0}; j < obj.ncols(); ++j) {
            result[0, j] = operator[](0, 0) * obj[0, j] + operator[](0, 1) * obj[1, j];
            result[1, j] = operator[](1, 0) * obj[0, j] + operator[](1, 1) * obj[1, j];
        }
        return result;
    }
    // NOLINTEND(readability-function-size, readability-function-cognitive-complexity)

    [[using gnu: pure, always_inline]]
    constexpr auto trace() const noexcept -> value_type {
        return operator[](0, 0) + operator[](1, 1);
    }

    [[using gnu: pure, always_inline]]
    constexpr auto determinant() const noexcept -> value_type {
        return operator[](0, 0) * operator[](1, 1) - operator[](0, 1) * operator[](1, 0);
    }

    // Swaps a1 and a2 in storage order under either layout.
    [[using gnu: always_inline, leaf, hot]]
    constexpr auto selfTransposed() noexcept -> Matrix& {
        if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX2__)
            _mm256_store_pd(
                data(), _mm256_permute4x64_pd(_mm256_load_pd(data()), _MM_SHUFFLE(3, 1, 2, 0))
            );
#elif defined(__SSE2__)
            const __m128d lo{_mm_load_pd(data())};
            const __m128d hi{_mm_load_pd(data() + 2)};
            _mm_store_pd(data(), _mm_unpacklo_pd(lo, hi));
            _mm_store_pd(data() + 2, _mm_unpackhi_pd(lo, hi));
#elif defined(__ARM_NEON)
            const float64x2_t lo{vld1q_f64(data())};
            const float64x2_t hi{vld1q_f64(data() + 2)};
            vst1q_f64(data(), vzip1q_f64(lo, hi));
            vst1q_f64(data() + 2, vzip2q_f64(lo, hi));
#else
            std::swap(operator[](0, 1), operator[](1, 0));
#endif
        } else if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE__)
            const __m128 v{_mm_load_ps(data())};
            _mm_store_ps(data(), _mm_shuffle_ps(v, v, _MM_SHUFFLE(3, 1, 2, 0)));
#elif defined(__ARM_NEON)
            const float32x4_t v{vld1q_f32(data())};
            const float32x2_t lo{vget_low_f32(v)};
            const float32x2_t hi{vget_high_f32(v)};
            vst1q_f32(data(), vcombine_f32(vzip1_f32(lo, hi), vzip2_f32(lo, hi)));
#else
            std::swap(operator[](0, 1), operator[](1, 0));
#endif
        } else {
            std::swap(operator[](0, 1), operator[](1, 0));
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

    // The adjugate over the determinant, by Cramer's rule. A singular matrix throws when parameters
    // are checked and yields infinities or NaNs otherwise.
    [[using gnu: always_inline, hot]]
    constexpr auto selfInversed() noexcept(!BASJOO_CHECK_PARAMS) -> Matrix&
        requires(!std::integral<value_type>)
    {
        const value_type
            det{operator[](0, 0) * operator[](1, 1) - operator[](0, 1) * operator[](1, 0)};
#if BASJOO_CHECK_PARAMS == 1
        if (det == value_type(0.0)) [[unlikely]] {
            throw std::runtime_error("Matrix::selfInversed: this matrix is singular.");
        }
#endif
        const value_type rdet{value_type(1.0) / det};
        *this = Matrix{
            operator[](1, 1) * rdet, -operator[](0, 1) * rdet,
            -operator[](1, 0) * rdet, operator[](0, 0) * rdet
        };
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

    // The angle of the rotation, read off its first column.
    [[using gnu: pure, always_inline, hot]]
    constexpr auto radian() const noexcept -> value_type
        requires std::floating_point<value_type>
    {
        return std::atan2(operator[](1, 0), operator[](0, 0));
    }

    // Left-multiplies by the rotation, so every column turns by `radian`.
    [[using gnu: always_inline, hot]]
    constexpr auto selfRotate(const std::floating_point auto& radian) noexcept -> Matrix&
        requires std::floating_point<value_type>
    {
        const auto c{static_cast<value_type>(std::cos(radian))};
        const auto s{static_cast<value_type>(std::sin(radian))};
        *this = Matrix{
            c * operator[](0, 0) - s * operator[](1, 0),
            c * operator[](0, 1) - s * operator[](1, 1),
            s * operator[](0, 0) + c * operator[](1, 0), s * operator[](0, 1) + c * operator[](1, 1)
        };
        return *this;
    }
    template <typename Self>
        requires std::floating_point<value_type>
    [[using gnu: always_inline, hot]]
    constexpr auto rotate(this Self&& self, const std::floating_point auto& radian) noexcept
        -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self.selfRotate(radian);
            return static_cast<Self&&>(self);
        } else {
            Matrix result{self};
            result.selfRotate(radian);
            return result;
        }
    }

    [[using gnu: always_inline, hot]]
    constexpr auto selfRotateHalfPi() noexcept -> Matrix&
        requires std::floating_point<value_type>
    {
        *this = Matrix{-operator[](1, 0), -operator[](1, 1), operator[](0, 0), operator[](0, 1)};
        return *this;
    }
    template <typename Self>
        requires std::floating_point<value_type>
    [[using gnu: always_inline, hot]]
    constexpr auto rotateHalfPi(this Self&& self) noexcept -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self.selfRotateHalfPi();
            return static_cast<Self&&>(self);
        } else {
            Matrix result{self};
            result.selfRotateHalfPi();
            return result;
        }
    }

    // Orthonormal columns, every entry of M^T M - I within `tol`, and a positive determinant.
    [[using gnu: pure, always_inline]]
    constexpr auto isRotation(value_type tol = 1E-8) const noexcept -> bool
        requires std::floating_point<value_type>
    {
        const value_type
            c00{operator[](0, 0) * operator[](0, 0) + operator[](1, 0) * operator[](1, 0)};
        const value_type
            c11{operator[](0, 1) * operator[](0, 1) + operator[](1, 1) * operator[](1, 1)};
        const value_type
            c01{operator[](0, 0) * operator[](0, 1) + operator[](1, 0) * operator[](1, 1)};
        return std::abs(c00 - value_type(1.0)) < tol && std::abs(c11 - value_type(1.0)) < tol &&
               std::abs(c01) < tol && determinant() > value_type(0.0);
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    storage_type m_data;
};

template <
    ScalarArithmetic Scalar, std::integral Index = std::size_t,
    DenseLayout Layout = std::layout_left>
using Mat22 = Matrix<Scalar, Index, 2, 2, Layout>;

using Mat22s = Mat22<float>;
using Mat22d = Mat22<double>;
using Mat22c = Mat22<std::complex<float>>;
using Mat22z = Mat22<std::complex<double>>;

template <typename T>
struct isMat22Arithmetic : std::false_type {};

template <ScalarArithmetic T, std::integral Index, DenseLayout Layout, Allocatory Alloc>
struct isMat22Arithmetic<Matrix<T, Index, 2, 2, Layout, Alloc>> : std::true_type {};

template <typename T>
inline constexpr bool isMat22ArithmeticV = isMat22Arithmetic<T>::value;

template <typename T>
concept Mat22Arithmetic = isMat22ArithmeticV<T>;

} // namespace basjoo::math
