/**
 * @file vec2.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-07-17
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <mdspan>
#include <ranges>
#include <span>
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
#include "basjoo/math/dense/vector_base.hpp"
#include "basjoo/math/type_traits.hpp"

namespace basjoo::math {

template <ScalarArithmetic Scalar, std::integral Index, Allocatory Alloc>
class alignas(2 * sizeof(Scalar)) Vector<Scalar, Index, 2, Alloc> final : public VectorBase<false> {
  public:
    using value_type = typename DenseTrait<Vector>::value_type;
    using reference = typename DenseTrait<Vector>::reference;
    using const_reference = typename DenseTrait<Vector>::const_reference;
    using pointer = typename DenseTrait<Vector>::pointer;
    using const_pointer = typename DenseTrait<Vector>::const_pointer;
    using size_type = typename DenseTrait<Vector>::size_type;
    using difference_type = typename DenseTrait<Vector>::difference_type;
    using allocator_type = typename DenseTrait<Vector>::allocator_type;
    using element_type = typename DenseTrait<Vector>::element_type;
    using extents_type = typename DenseTrait<Vector>::extents_type;
    using layout_type = typename DenseTrait<Vector>::layout_type;
    using index_type = typename DenseTrait<Vector>::index_type;
    using rank_type = typename DenseTrait<Vector>::rank_type;
    using mapping_type = typename DenseTrait<Vector>::mapping_type;

    [[using gnu: always_inline]]
    constexpr explicit Vector(value_type cv) noexcept
        : m_data{cv, cv} {}
    [[using gnu: always_inline]]
    constexpr Vector(value_type cx, value_type cy) noexcept
        : m_data{cx, cy} {}
    [[using gnu: always_inline]]
    constexpr Vector(const Vector& other) noexcept = default;
    [[using gnu: always_inline]]
    constexpr auto operator=(const Vector& other) noexcept -> Vector& = default;
    [[using gnu: always_inline]]
    constexpr Vector(Vector&& other) noexcept = default;
    [[using gnu: always_inline]]
    constexpr auto operator=(Vector&& other) noexcept -> Vector& = default;
    ~Vector() noexcept = default;

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto get_allocator() noexcept -> allocator_type {
        return allocator_type{};
    }

    [[using gnu: always_inline]]
    constexpr Vector() noexcept = default;

    [[using gnu: always_inline]]
    constexpr explicit Vector([[maybe_unused]] const allocator_type& alloc) noexcept
        : m_data{} {}

    // The sized constructors of the generic container protocol take the allocator without a
    // default, so bare scalar arguments always mean element values, whatever the scalar type.
    [[using gnu: always_inline]]
    constexpr explicit Vector(
        [[maybe_unused]] size_type count, [[maybe_unused]] const allocator_type& alloc
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : m_data{} {
#if BASJOO_CHECK_PARAMS == 1
        if (count != size()) [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
    }

    [[using gnu: always_inline]]
    constexpr explicit Vector(
        [[maybe_unused]] size_type count, const_reference value,
        [[maybe_unused]] const allocator_type& alloc
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : m_data{value, value} {
#if BASJOO_CHECK_PARAMS == 1
        if (count != size()) [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
    }

    template <std::ranges::sized_range R>
        requires std::convertible_to<std::ranges::range_reference_t<R>, value_type>
    [[using gnu: always_inline]] constexpr explicit Vector(
        R&& rg, [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : m_data{} {
#if BASJOO_CHECK_PARAMS == 1
        if (std::ranges::size(rg) != size()) [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
        auto it{std::ranges::begin(rg)};
        m_data[0] = *it;
        ++it;
        m_data[1] = *it;
    }

    template <typename T>
        requires std::same_as<std::remove_cv_t<typename T::value_type>, value_type> &&
                 (T::rank() == 2)
    [[using gnu: always_inline]] constexpr explicit Vector(
        const T& matrix_like, [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : m_data{} {
        using other_index_type = typename T::index_type;

#if BASJOO_CHECK_PARAMS == 1
        if (matrix_like.extent(0) != 1 && matrix_like.extent(1) != 1) [[unlikely]] {
            throw std::invalid_argument(
                "Vector constructor: matrix is neither a row nor a column!"
            );
        }
        if (std::cmp_not_equal(matrix_like.extent(0) * matrix_like.extent(1), size()))
            [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
        const bool row_like{matrix_like.extent(1) != 1};
        for (size_type i{0}; i < size(); ++i) {
            const auto k{static_cast<other_index_type>(i)};
            operator[](i) = row_like ? matrix_like[0, k] : matrix_like[k, 0];
        }
    }

    template <ScalarArithmetic U>
    [[using gnu: always_inline]]
    constexpr explicit Vector(const std::pair<U, U>& other) noexcept
        : m_data{static_cast<value_type>(other.first), static_cast<value_type>(other.second)} {}

    template <ScalarArithmetic U>
    [[using gnu: always_inline]]
    constexpr explicit Vector(const Vector<U, Index, 2>& other) noexcept
        : m_data{static_cast<value_type>(other.x()), static_cast<value_type>(other.y())} {}

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto size() noexcept -> size_type {
        return 2;
    }
    [[using gnu: const, always_inline, leaf]]
    static constexpr auto stride([[maybe_unused]] rank_type r = 0) noexcept -> index_type {
        return 1;
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto data(this Self&& self) noexcept {
        return self.m_data.data();
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto data_handle(this Self&& self) noexcept {
        return self.data();
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto x(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[0];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto y(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[1];
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto empty() noexcept -> bool {
        return false;
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
    static constexpr auto extent([[maybe_unused]] rank_type r) noexcept -> index_type {
        return 2;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto mapping() noexcept -> mapping_type {
        return mapping_type{};
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

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<value_type, 2>() noexcept {
        return std::span<value_type, 2>{data(), 2};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<const value_type, 2>() const noexcept {
        return std::span<const value_type, 2>{data(), 2};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<value_type>() noexcept {
        return std::span<value_type>{data(), 2};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<const value_type>() const noexcept {
        return std::span<const value_type>{data(), 2};
    }

    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto dot(const Vector& obj) const noexcept -> value_type {
        if !consteval {
            if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE2__) || defined(_M_X64)
                const __m128 prod{_mm_mul_ps(
                    _mm_castsi128_ps(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(data()))),
                    _mm_castsi128_ps(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(obj.data())))
                )};
                return _mm_cvtss_f32(
                    _mm_add_ss(prod, _mm_shuffle_ps(prod, prod, _MM_SHUFFLE(1, 1, 1, 1)))
                );
#elif defined(__ARM_NEON)
                return vaddv_f32(vmul_f32(vld1_f32(data()), vld1_f32(obj.data())));
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__SSE2__) || defined(_M_X64)
                const __m128d prod{_mm_mul_pd(_mm_load_pd(data()), _mm_load_pd(obj.data()))};
                return _mm_cvtsd_f64(_mm_add_sd(prod, _mm_unpackhi_pd(prod, prod)));
#elif defined(__ARM_NEON)
                return vaddvq_f64(vmulq_f64(vld1q_f64(data()), vld1q_f64(obj.data())));
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__SSE3__) || defined(__AVX__)
                const __m128 lhs{_mm_load_ps(reinterpret_cast<const float*>(data()))};
                const __m128 rhs{_mm_load_ps(reinterpret_cast<const float*>(obj.data()))};
                // (a c - b d, b c + a d) per element, for a + b i of *this and c + d i of obj.
                const __m128 prod{_mm_addsub_ps(
                    _mm_mul_ps(lhs, _mm_moveldup_ps(rhs)),
                    _mm_mul_ps(
                        _mm_shuffle_ps(lhs, lhs, _MM_SHUFFLE(2, 3, 0, 1)), _mm_movehdup_ps(rhs)
                    )
                )};
                value_type result;
                _mm_storel_pi(
                    reinterpret_cast<__m64*>(&result), _mm_add_ps(prod, _mm_movehl_ps(prod, prod))
                );
                return result;
#elif defined(__ARM_NEON)
                const float32x4_t lhs{vld1q_f32(reinterpret_cast<const float*>(data()))};
                const float32x4_t rhs{vld1q_f32(reinterpret_cast<const float*>(obj.data()))};
                // (a c, b c) plus (b d, a d) with its real lane negated, per element.
                const float32x4_t by_im{vmulq_f32(vrev64q_f32(lhs), vtrn2q_f32(rhs, rhs))};
                const float32x4_t prod{vaddq_f32(
                    vmulq_f32(lhs, vtrn1q_f32(rhs, rhs)),
                    vreinterpretq_f32_u32(veorq_u32(
                        vreinterpretq_u32_f32(by_im),
                        vreinterpretq_u32_u64(vdupq_n_u64(0x80000000ULL))
                    ))
                )};
                value_type result;
                vst1_f32(
                    reinterpret_cast<float*>(&result),
                    vadd_f32(vget_low_f32(prod), vget_high_f32(prod))
                );
                return result;
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX__)
                const __m256d lhs{_mm256_load_pd(reinterpret_cast<const double*>(data()))};
                const __m256d rhs{_mm256_load_pd(reinterpret_cast<const double*>(obj.data()))};
                // (a c - b d, b c + a d) per element, for a + b i of *this and c + d i of obj.
                const __m256d prod{_mm256_addsub_pd(
                    _mm256_mul_pd(lhs, _mm256_movedup_pd(rhs)),
                    _mm256_mul_pd(_mm256_permute_pd(lhs, 0b0101), _mm256_permute_pd(rhs, 0b1111))
                )};
                value_type result;
                _mm_storeu_pd(
                    reinterpret_cast<double*>(&result),
                    _mm_add_pd(_mm256_castpd256_pd128(prod), _mm256_extractf128_pd(prod, 1))
                );
                return result;
#elif defined(__SSE3__)
                const double* const lhs{reinterpret_cast<const double*>(data())};
                const double* const rhs{reinterpret_cast<const double*>(obj.data())};
                const __m128d l0{_mm_load_pd(lhs)};
                const __m128d l1{_mm_load_pd(lhs + 2)};
                const __m128d r0{_mm_load_pd(rhs)};
                const __m128d r1{_mm_load_pd(rhs + 2)};
                // (a c - b d, b c + a d) per element, for a + b i of *this and c + d i of obj.
                const __m128d p0{_mm_addsub_pd(
                    _mm_mul_pd(l0, _mm_movedup_pd(r0)),
                    _mm_mul_pd(_mm_shuffle_pd(l0, l0, 0b01), _mm_unpackhi_pd(r0, r0))
                )};
                const __m128d p1{_mm_addsub_pd(
                    _mm_mul_pd(l1, _mm_movedup_pd(r1)),
                    _mm_mul_pd(_mm_shuffle_pd(l1, l1, 0b01), _mm_unpackhi_pd(r1, r1))
                )};
                value_type result;
                _mm_storeu_pd(reinterpret_cast<double*>(&result), _mm_add_pd(p0, p1));
                return result;
#elif defined(__ARM_NEON)
                const double* const lhs{reinterpret_cast<const double*>(data())};
                const double* const rhs{reinterpret_cast<const double*>(obj.data())};
                const float64x2_t l0{vld1q_f64(lhs)};
                const float64x2_t l1{vld1q_f64(lhs + 2)};
                const float64x2_t r0{vld1q_f64(rhs)};
                const float64x2_t r1{vld1q_f64(rhs + 2)};
                // (a c, b c) plus (b d, a d) with its real lane negated, per element.
                const uint64x2_t sign{vsetq_lane_u64(0x8000000000000000ULL, vdupq_n_u64(0), 0)};
                const float64x2_t p0{vaddq_f64(
                    vmulq_laneq_f64(l0, r0, 0),
                    vreinterpretq_f64_u64(veorq_u64(
                        vreinterpretq_u64_f64(vmulq_laneq_f64(vextq_f64(l0, l0, 1), r0, 1)), sign
                    ))
                )};
                const float64x2_t p1{vaddq_f64(
                    vmulq_laneq_f64(l1, r1, 0),
                    vreinterpretq_f64_u64(veorq_u64(
                        vreinterpretq_u64_f64(vmulq_laneq_f64(vextq_f64(l1, l1, 1), r1, 1)), sign
                    ))
                )};
                value_type result;
                vst1q_f64(reinterpret_cast<double*>(&result), vaddq_f64(p0, p1));
                return result;
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int32_t> ||
                std::is_same_v<value_type, std::uint32_t>
            ) {
#if defined(__SSE4_1__) || defined(__AVX__)
                const __m128i prod{_mm_mullo_epi32(
                    _mm_loadl_epi64(reinterpret_cast<const __m128i*>(data())),
                    _mm_loadl_epi64(reinterpret_cast<const __m128i*>(obj.data()))
                )};
                return static_cast<value_type>(_mm_cvtsi128_si32(
                    _mm_add_epi32(prod, _mm_shuffle_epi32(prod, _MM_SHUFFLE(1, 1, 1, 1)))
                ));
#elif defined(__ARM_NEON)
                return static_cast<value_type>(vaddv_s32(vmul_s32(
                    vld1_s32(reinterpret_cast<const std::int32_t*>(data())),
                    vld1_s32(reinterpret_cast<const std::int32_t*>(obj.data()))
                )));
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int64_t> ||
                std::is_same_v<value_type, std::uint64_t>
            ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
                const __m128i prod{_mm_mullo_epi64(
                    _mm_load_si128(reinterpret_cast<const __m128i*>(data())),
                    _mm_load_si128(reinterpret_cast<const __m128i*>(obj.data()))
                )};
                return static_cast<value_type>(
                    _mm_cvtsi128_si64(_mm_add_epi64(prod, _mm_unpackhi_epi64(prod, prod)))
                );
#endif
            }
        }
        return x() * obj.x() + y() * obj.y();
    }

    // The product v^T M with a matrix of two rows. Under layout_left, each element combines the
    // two stored elements of a column of M; under layout_right, the result combines the two stored
    // rows.
    // NOLINTBEGIN(readability-function-size, readability-function-cognitive-complexity)
    template <std::size_t OuterSize, DenseLayout ObjLayout>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(
        const Matrix<value_type, Index, 2, OuterSize, ObjLayout, Alloc>& obj
    ) const noexcept -> Vector<value_type, Index, OuterSize, Alloc> {
        [[maybe_unused]] constexpr bool kLeft{std::is_same_v<ObjLayout, std::layout_left>};

        // Every element is written below; a Vec3 result constructs its padding.
        Vector<value_type, Index, OuterSize, Alloc> result;
        if !consteval {
            [[maybe_unused]] const value_type* const m{obj.data()};
            [[maybe_unused]] value_type* const out{result.data()};
            if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE2__) || defined(_M_X64)
                // A Mat22 product is returned from a register: through memory, it costs a stack
                // copy and the stack protector's check.
                const __m128 v{
                    _mm_castsi128_ps(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(data())))
                };
                if constexpr (OuterSize == 2) {
                    // The rows (m00, m01, m10, m11), transposed into place under layout_left, times
                    // (x, x, y, y): the halves sum to the two elements, side by side.
                    const __m128 stored{_mm_loadu_ps(m)};
                    const __m128 rows{
                        kLeft ? _mm_shuffle_ps(stored, stored, _MM_SHUFFLE(3, 1, 2, 0)) : stored
                    };
                    const __m128 prod{_mm_mul_ps(rows, _mm_unpacklo_ps(v, v))};
                    const __m128 sums{_mm_add_ps(prod, _mm_movehl_ps(prod, prod))};
                    return Vector{
                        _mm_cvtss_f32(sums),
                        _mm_cvtss_f32(_mm_shuffle_ps(sums, sums, _MM_SHUFFLE(1, 1, 1, 1)))
                    };
                } else if constexpr (kLeft) {
                    // Four columns a step, (x, y, x, y) against two of them a register, so that a
                    // whole register is stored.
                    const __m128 vv{_mm_movelh_ps(v, v)};
                    std::size_t j{0};
                    for (; j + 4 <= OuterSize; j += 4) {
                        const __m128 p{_mm_mul_ps(_mm_loadu_ps(m + 2 * j), vv)};
                        const __m128 q{_mm_mul_ps(_mm_loadu_ps(m + 2 * j + 4), vv)};
                        _mm_storeu_ps(
                            out + j, _mm_add_ps(
                                         _mm_shuffle_ps(p, q, _MM_SHUFFLE(2, 0, 2, 0)),
                                         _mm_shuffle_ps(p, q, _MM_SHUFFLE(3, 1, 3, 1))
                                     )
                        );
                    }
                    if constexpr (OuterSize % 4 >= 2) {
                        const __m128 p{_mm_mul_ps(_mm_loadu_ps(m + 2 * j), vv)};
                        _mm_storel_pi(
                            reinterpret_cast<__m64*>(out + j),
                            _mm_add_ps(
                                _mm_shuffle_ps(p, p, _MM_SHUFFLE(2, 0, 2, 0)),
                                _mm_shuffle_ps(p, p, _MM_SHUFFLE(3, 1, 3, 1))
                            )
                        );
                        j += 2;
                    }
                    if constexpr (OuterSize % 2 == 1) {
                        const __m128 prod{_mm_mul_ps(
                            _mm_castsi128_ps(
                                _mm_loadl_epi64(reinterpret_cast<const __m128i*>(m + 2 * j))
                            ),
                            v
                        )};
                        out[j] = _mm_cvtss_f32(
                            _mm_add_ss(prod, _mm_shuffle_ps(prod, prod, _MM_SHUFFLE(1, 1, 1, 1)))
                        );
                    }
                } else {
                    const __m128 vx{_mm_set1_ps(x())};
                    const __m128 vy{_mm_set1_ps(y())};
                    std::size_t j{0};
                    for (; j + 4 <= OuterSize; j += 4) {
                        _mm_storeu_ps(
                            out + j, _mm_add_ps(
                                         _mm_mul_ps(_mm_loadu_ps(m + j), vx),
                                         _mm_mul_ps(_mm_loadu_ps(m + OuterSize + j), vy)
                                     )
                        );
                    }
                    if constexpr (OuterSize % 4 >= 2) {
                        const __m128 r0{_mm_castsi128_ps(
                            _mm_loadl_epi64(reinterpret_cast<const __m128i*>(m + j))
                        )};
                        const __m128 r1{_mm_castsi128_ps(
                            _mm_loadl_epi64(reinterpret_cast<const __m128i*>(m + OuterSize + j))
                        )};
                        _mm_storel_pi(
                            reinterpret_cast<__m64*>(out + j),
                            _mm_add_ps(_mm_mul_ps(r0, vx), _mm_mul_ps(r1, vy))
                        );
                        j += 2;
                    }
                    if constexpr (OuterSize % 2 == 1) {
                        out[j] = x() * m[j] + y() * m[OuterSize + j];
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                if constexpr (kLeft) {
                    // (x, y) against one column, (x, y, x, y) against two.
                    const float32x2_t v{vld1_f32(data())};
                    const float32x4_t vv{vcombine_f32(v, v)};
                    std::size_t j{0};
                    for (; j + 2 <= OuterSize; j += 2) {
                        const float32x4_t prod{vmulq_f32(vld1q_f32(m + 2 * j), vv)};
                        vst1_f32(out + j, vpadd_f32(vget_low_f32(prod), vget_high_f32(prod)));
                    }
                    if constexpr (OuterSize % 2 == 1) {
                        out[j] = vaddv_f32(vmul_f32(vld1_f32(m + 2 * j), v));
                    }
                } else {
                    std::size_t j{0};
                    for (; j + 4 <= OuterSize; j += 4) {
                        vst1q_f32(
                            out + j, vaddq_f32(
                                         vmulq_n_f32(vld1q_f32(m + j), x()),
                                         vmulq_n_f32(vld1q_f32(m + OuterSize + j), y())
                                     )
                        );
                    }
                    if constexpr (OuterSize % 4 >= 2) {
                        vst1_f32(
                            out + j, vadd_f32(
                                         vmul_n_f32(vld1_f32(m + j), x()),
                                         vmul_n_f32(vld1_f32(m + OuterSize + j), y())
                                     )
                        );
                        j += 2;
                    }
                    if constexpr (OuterSize % 2 == 1) {
                        out[j] = x() * m[j] + y() * m[OuterSize + j];
                    }
                }
                return result;
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__SSE2__) || defined(_M_X64)
                // A Mat22 product is returned from a register, as for float.
                if constexpr (OuterSize == 2 && kLeft) {
                    const __m128d v{_mm_load_pd(data())};
                    const __m128d a{_mm_mul_pd(_mm_loadu_pd(m), v)};
                    const __m128d b{_mm_mul_pd(_mm_loadu_pd(m + 2), v)};
                    const __m128d sums{_mm_add_pd(_mm_unpacklo_pd(a, b), _mm_unpackhi_pd(a, b))};
                    return Vector{_mm_cvtsd_f64(sums), _mm_cvtsd_f64(_mm_unpackhi_pd(sums, sums))};
                } else if constexpr (OuterSize == 2) {
                    const __m128d sums{_mm_add_pd(
                        _mm_mul_pd(_mm_loadu_pd(m), _mm_set1_pd(x())),
                        _mm_mul_pd(_mm_loadu_pd(m + 2), _mm_set1_pd(y()))
                    )};
                    return Vector{_mm_cvtsd_f64(sums), _mm_cvtsd_f64(_mm_unpackhi_pd(sums, sums))};
                } else if constexpr (kLeft) {
                    const __m128d v{_mm_load_pd(data())};
                    std::size_t j{0};
                    for (; j + 2 <= OuterSize; j += 2) {
                        const __m128d a{_mm_mul_pd(_mm_loadu_pd(m + 2 * j), v)};
                        const __m128d b{_mm_mul_pd(_mm_loadu_pd(m + 2 * j + 2), v)};
                        _mm_storeu_pd(
                            out + j, _mm_add_pd(_mm_unpacklo_pd(a, b), _mm_unpackhi_pd(a, b))
                        );
                    }
                    if constexpr (OuterSize % 2 == 1) {
                        const __m128d a{_mm_mul_pd(_mm_loadu_pd(m + 2 * j), v)};
                        out[j] = _mm_cvtsd_f64(_mm_add_sd(a, _mm_unpackhi_pd(a, a)));
                    }
                } else {
                    const __m128d vx{_mm_set1_pd(x())};
                    const __m128d vy{_mm_set1_pd(y())};
                    std::size_t j{0};
                    for (; j + 2 <= OuterSize; j += 2) {
                        _mm_storeu_pd(
                            out + j, _mm_add_pd(
                                         _mm_mul_pd(_mm_loadu_pd(m + j), vx),
                                         _mm_mul_pd(_mm_loadu_pd(m + OuterSize + j), vy)
                                     )
                        );
                    }
                    if constexpr (OuterSize % 2 == 1) {
                        out[j] = x() * m[j] + y() * m[OuterSize + j];
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                if constexpr (kLeft) {
                    const float64x2_t v{vld1q_f64(data())};
                    std::size_t j{0};
                    for (; j + 2 <= OuterSize; j += 2) {
                        vst1q_f64(
                            out + j, vpaddq_f64(
                                         vmulq_f64(vld1q_f64(m + 2 * j), v),
                                         vmulq_f64(vld1q_f64(m + 2 * j + 2), v)
                                     )
                        );
                    }
                    if constexpr (OuterSize % 2 == 1) {
                        out[j] = vaddvq_f64(vmulq_f64(vld1q_f64(m + 2 * j), v));
                    }
                } else {
                    std::size_t j{0};
                    for (; j + 2 <= OuterSize; j += 2) {
                        vst1q_f64(
                            out + j, vaddq_f64(
                                         vmulq_n_f64(vld1q_f64(m + j), x()),
                                         vmulq_n_f64(vld1q_f64(m + OuterSize + j), y())
                                     )
                        );
                    }
                    if constexpr (OuterSize % 2 == 1) {
                        out[j] = x() * m[j] + y() * m[OuterSize + j];
                    }
                }
                return result;
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__SSE3__) || defined(__AVX__)
                if constexpr (kLeft) {
                    // A column is two complex elements: one register against (x, y).
                    const __m128 v{_mm_load_ps(reinterpret_cast<const float*>(data()))};
                    const __m128 v_re{_mm_moveldup_ps(v)};
                    const __m128 v_im{_mm_movehdup_ps(v)};
                    const float* const lanes{reinterpret_cast<const float*>(m)};
                    for (std::size_t j{0}; j < OuterSize; ++j) {
                        const __m128 col{_mm_loadu_ps(lanes + 4 * j)};
                        const __m128 prod{_mm_addsub_ps(
                            _mm_mul_ps(col, v_re),
                            _mm_mul_ps(_mm_shuffle_ps(col, col, _MM_SHUFFLE(2, 3, 0, 1)), v_im)
                        )};
                        _mm_storel_pi(
                            reinterpret_cast<__m64*>(out + j),
                            _mm_add_ps(prod, _mm_movehl_ps(prod, prod))
                        );
                    }
                } else {
                    // Each element of a row times (re, im), as (a re - b im, b re + a im).
                    const float* const row0{reinterpret_cast<const float*>(m)};
                    const float* const row1{reinterpret_cast<const float*>(m + OuterSize)};
                    const __m128 x_re{_mm_set1_ps(x().real())};
                    const __m128 x_im{_mm_set1_ps(x().imag())};
                    const __m128 y_re{_mm_set1_ps(y().real())};
                    const __m128 y_im{_mm_set1_ps(y().imag())};
                    std::size_t j{0};
                    for (; j + 2 <= OuterSize; j += 2) {
                        const __m128 r0{_mm_loadu_ps(row0 + 2 * j)};
                        const __m128 r1{_mm_loadu_ps(row1 + 2 * j)};
                        _mm_storeu_ps(
                            reinterpret_cast<float*>(out + j),
                            _mm_add_ps(
                                _mm_addsub_ps(
                                    _mm_mul_ps(r0, x_re),
                                    _mm_mul_ps(
                                        _mm_shuffle_ps(r0, r0, _MM_SHUFFLE(2, 3, 0, 1)), x_im
                                    )
                                ),
                                _mm_addsub_ps(
                                    _mm_mul_ps(r1, y_re),
                                    _mm_mul_ps(
                                        _mm_shuffle_ps(r1, r1, _MM_SHUFFLE(2, 3, 0, 1)), y_im
                                    )
                                )
                            )
                        );
                    }
                    if constexpr (OuterSize % 2 == 1) {
                        const __m128 r0{_mm_castsi128_ps(
                            _mm_loadl_epi64(reinterpret_cast<const __m128i*>(row0 + 2 * j))
                        )};
                        const __m128 r1{_mm_castsi128_ps(
                            _mm_loadl_epi64(reinterpret_cast<const __m128i*>(row1 + 2 * j))
                        )};
                        _mm_storel_pi(
                            reinterpret_cast<__m64*>(out + j),
                            _mm_add_ps(
                                _mm_addsub_ps(
                                    _mm_mul_ps(r0, x_re),
                                    _mm_mul_ps(
                                        _mm_shuffle_ps(r0, r0, _MM_SHUFFLE(2, 3, 0, 1)), x_im
                                    )
                                ),
                                _mm_addsub_ps(
                                    _mm_mul_ps(r1, y_re),
                                    _mm_mul_ps(
                                        _mm_shuffle_ps(r1, r1, _MM_SHUFFLE(2, 3, 0, 1)), y_im
                                    )
                                )
                            )
                        );
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                // Each element times (re, im) as (a re, b re) plus (b im, a im) with its real lane
                // negated.
                const uint32x4_t sign{vreinterpretq_u32_u64(vdupq_n_u64(0x80000000ULL))};
                if constexpr (kLeft) {
                    const float32x4_t v{vld1q_f32(reinterpret_cast<const float*>(data()))};
                    const float32x4_t v_re{vtrn1q_f32(v, v)};
                    const float32x4_t v_im{vtrn2q_f32(v, v)};
                    const float* const lanes{reinterpret_cast<const float*>(m)};
                    for (std::size_t j{0}; j < OuterSize; ++j) {
                        const float32x4_t col{vld1q_f32(lanes + 4 * j)};
                        const float32x4_t prod{vaddq_f32(
                            vmulq_f32(col, v_re),
                            vreinterpretq_f32_u32(veorq_u32(
                                vreinterpretq_u32_f32(vmulq_f32(vrev64q_f32(col), v_im)), sign
                            ))
                        )};
                        vst1_f32(
                            reinterpret_cast<float*>(out + j),
                            vadd_f32(vget_low_f32(prod), vget_high_f32(prod))
                        );
                    }
                } else {
                    const float* const row0{reinterpret_cast<const float*>(m)};
                    const float* const row1{reinterpret_cast<const float*>(m + OuterSize)};
                    const float32x4_t x_im{vdupq_n_f32(x().imag())};
                    const float32x4_t y_im{vdupq_n_f32(y().imag())};
                    std::size_t j{0};
                    for (; j + 2 <= OuterSize; j += 2) {
                        const float32x4_t r0{vld1q_f32(row0 + 2 * j)};
                        const float32x4_t r1{vld1q_f32(row1 + 2 * j)};
                        const float32x4_t p0{vaddq_f32(
                            vmulq_n_f32(r0, x().real()),
                            vreinterpretq_f32_u32(veorq_u32(
                                vreinterpretq_u32_f32(vmulq_f32(vrev64q_f32(r0), x_im)), sign
                            ))
                        )};
                        const float32x4_t p1{vaddq_f32(
                            vmulq_n_f32(r1, y().real()),
                            vreinterpretq_f32_u32(veorq_u32(
                                vreinterpretq_u32_f32(vmulq_f32(vrev64q_f32(r1), y_im)), sign
                            ))
                        )};
                        vst1q_f32(reinterpret_cast<float*>(out + j), vaddq_f32(p0, p1));
                    }
                    if constexpr (OuterSize % 2 == 1) {
                        const float32x2_t r0{vld1_f32(row0 + 2 * j)};
                        const float32x2_t r1{vld1_f32(row1 + 2 * j)};
                        const uint32x2_t low_sign{vget_low_u32(sign)};
                        const float32x2_t p0{vadd_f32(
                            vmul_n_f32(r0, x().real()),
                            vreinterpret_f32_u32(veor_u32(
                                vreinterpret_u32_f32(vmul_n_f32(vrev64_f32(r0), x().imag())),
                                low_sign
                            ))
                        )};
                        const float32x2_t p1{vadd_f32(
                            vmul_n_f32(r1, y().real()),
                            vreinterpret_f32_u32(veor_u32(
                                vreinterpret_u32_f32(vmul_n_f32(vrev64_f32(r1), y().imag())),
                                low_sign
                            ))
                        )};
                        vst1_f32(reinterpret_cast<float*>(out + j), vadd_f32(p0, p1));
                    }
                }
                return result;
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__SSE3__) || defined(__AVX__)
                if constexpr (kLeft) {
#if defined(__AVX__)
                    // A column is two complex elements: one register against (x, y).
                    const __m256d v{_mm256_load_pd(reinterpret_cast<const double*>(data()))};
                    const __m256d v_re{_mm256_movedup_pd(v)};
                    const __m256d v_im{_mm256_permute_pd(v, 0b1111)};
                    const double* const lanes{reinterpret_cast<const double*>(m)};
                    for (std::size_t j{0}; j < OuterSize; ++j) {
                        const __m256d col{_mm256_loadu_pd(lanes + 4 * j)};
                        const __m256d prod{_mm256_addsub_pd(
                            _mm256_mul_pd(col, v_re),
                            _mm256_mul_pd(_mm256_permute_pd(col, 0b0101), v_im)
                        )};
                        _mm_storeu_pd(
                            reinterpret_cast<double*>(out + j),
                            _mm_add_pd(_mm256_castpd256_pd128(prod), _mm256_extractf128_pd(prod, 1))
                        );
                    }
#else
                    const double* const v{reinterpret_cast<const double*>(data())};
                    const __m128d x_re{_mm_set1_pd(v[0])};
                    const __m128d x_im{_mm_set1_pd(v[1])};
                    const __m128d y_re{_mm_set1_pd(v[2])};
                    const __m128d y_im{_mm_set1_pd(v[3])};
                    const double* const lanes{reinterpret_cast<const double*>(m)};
                    for (std::size_t j{0}; j < OuterSize; ++j) {
                        const __m128d c0{_mm_loadu_pd(lanes + 4 * j)};
                        const __m128d c1{_mm_loadu_pd(lanes + 4 * j + 2)};
                        _mm_storeu_pd(
                            reinterpret_cast<double*>(out + j),
                            _mm_add_pd(
                                _mm_addsub_pd(
                                    _mm_mul_pd(c0, x_re),
                                    _mm_mul_pd(_mm_shuffle_pd(c0, c0, 0b01), x_im)
                                ),
                                _mm_addsub_pd(
                                    _mm_mul_pd(c1, y_re),
                                    _mm_mul_pd(_mm_shuffle_pd(c1, c1, 0b01), y_im)
                                )
                            )
                        );
                    }
#endif
                } else {
                    // Each element of a row times (re, im), as (a re - b im, b re + a im).
                    const double* const row0{reinterpret_cast<const double*>(m)};
                    const double* const row1{reinterpret_cast<const double*>(m + OuterSize)};
                    std::size_t j{0};
#if defined(__AVX__)
                    const __m256d x_re4{_mm256_set1_pd(x().real())};
                    const __m256d x_im4{_mm256_set1_pd(x().imag())};
                    const __m256d y_re4{_mm256_set1_pd(y().real())};
                    const __m256d y_im4{_mm256_set1_pd(y().imag())};
                    for (; j + 2 <= OuterSize; j += 2) {
                        const __m256d r0{_mm256_loadu_pd(row0 + 2 * j)};
                        const __m256d r1{_mm256_loadu_pd(row1 + 2 * j)};
                        _mm256_storeu_pd(
                            reinterpret_cast<double*>(out + j),
                            _mm256_add_pd(
                                _mm256_addsub_pd(
                                    _mm256_mul_pd(r0, x_re4),
                                    _mm256_mul_pd(_mm256_permute_pd(r0, 0b0101), x_im4)
                                ),
                                _mm256_addsub_pd(
                                    _mm256_mul_pd(r1, y_re4),
                                    _mm256_mul_pd(_mm256_permute_pd(r1, 0b0101), y_im4)
                                )
                            )
                        );
                    }
#endif
                    const __m128d x_re{_mm_set1_pd(x().real())};
                    const __m128d x_im{_mm_set1_pd(x().imag())};
                    const __m128d y_re{_mm_set1_pd(y().real())};
                    const __m128d y_im{_mm_set1_pd(y().imag())};
                    for (; j < OuterSize; ++j) {
                        const __m128d r0{_mm_loadu_pd(row0 + 2 * j)};
                        const __m128d r1{_mm_loadu_pd(row1 + 2 * j)};
                        _mm_storeu_pd(
                            reinterpret_cast<double*>(out + j),
                            _mm_add_pd(
                                _mm_addsub_pd(
                                    _mm_mul_pd(r0, x_re),
                                    _mm_mul_pd(_mm_shuffle_pd(r0, r0, 0b01), x_im)
                                ),
                                _mm_addsub_pd(
                                    _mm_mul_pd(r1, y_re),
                                    _mm_mul_pd(_mm_shuffle_pd(r1, r1, 0b01), y_im)
                                )
                            )
                        );
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                // Each element times (re, im) as (a re, b re) plus (b im, a im) with its real lane
                // negated.
                const uint64x2_t sign{vsetq_lane_u64(0x8000000000000000ULL, vdupq_n_u64(0), 0)};
                const float64x2_t xv{vld1q_f64(reinterpret_cast<const double*>(data()))};
                const float64x2_t yv{vld1q_f64(reinterpret_cast<const double*>(data()) + 2)};
                const double* const lanes{reinterpret_cast<const double*>(m)};
                for (std::size_t j{0}; j < OuterSize; ++j) {
                    // The two factors of element j: (m00, m10) of column j, or (m0j, m1j).
                    const float64x2_t a{vld1q_f64(kLeft ? lanes + 4 * j : lanes + 2 * j)};
                    const float64x2_t b{
                        vld1q_f64(kLeft ? lanes + 4 * j + 2 : lanes + 2 * (OuterSize + j))
                    };
                    const float64x2_t pa{vaddq_f64(
                        vmulq_laneq_f64(a, xv, 0),
                        vreinterpretq_f64_u64(veorq_u64(
                            vreinterpretq_u64_f64(vmulq_laneq_f64(vextq_f64(a, a, 1), xv, 1)), sign
                        ))
                    )};
                    const float64x2_t pb{vaddq_f64(
                        vmulq_laneq_f64(b, yv, 0),
                        vreinterpretq_f64_u64(veorq_u64(
                            vreinterpretq_u64_f64(vmulq_laneq_f64(vextq_f64(b, b, 1), yv, 1)), sign
                        ))
                    )};
                    vst1q_f64(reinterpret_cast<double*>(out + j), vaddq_f64(pa, pb));
                }
                return result;
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int32_t> ||
                std::is_same_v<value_type, std::uint32_t>
            ) {
#if defined(__SSE4_1__) || defined(__AVX__)
                const auto* const lanes{reinterpret_cast<const std::int32_t*>(m)};
                auto* const lanes_out{reinterpret_cast<std::int32_t*>(out)};
                // A Mat22 product is returned from a register, as for float.
                const __m128i v{_mm_loadl_epi64(reinterpret_cast<const __m128i*>(data()))};
                if constexpr (OuterSize == 2) {
                    // The rows (m00, m01, m10, m11), transposed into place under layout_left, times
                    // (x, x, y, y): the halves sum to the two elements, side by side.
                    const __m128i stored{_mm_loadu_si128(reinterpret_cast<const __m128i*>(lanes))};
                    const __m128i rows{
                        kLeft ? _mm_shuffle_epi32(stored, _MM_SHUFFLE(3, 1, 2, 0)) : stored
                    };
                    const __m128i prod{_mm_mullo_epi32(rows, _mm_unpacklo_epi32(v, v))};
                    const __m128i sums{_mm_add_epi32(prod, _mm_unpackhi_epi64(prod, prod))};
                    return Vector{
                        static_cast<value_type>(_mm_cvtsi128_si32(sums)),
                        static_cast<value_type>(_mm_extract_epi32(sums, 1))
                    };
                } else if constexpr (kLeft) {
                    // Four columns a step, (x, y, x, y) against two of them a register, so that a
                    // whole register is stored.
                    const __m128i vv{_mm_unpacklo_epi64(v, v)};
                    std::size_t j{0};
                    for (; j + 4 <= OuterSize; j += 4) {
                        _mm_storeu_si128(
                            reinterpret_cast<__m128i*>(lanes_out + j),
                            _mm_hadd_epi32(
                                _mm_mullo_epi32(
                                    _mm_loadu_si128(
                                        reinterpret_cast<const __m128i*>(lanes + 2 * j)
                                    ),
                                    vv
                                ),
                                _mm_mullo_epi32(
                                    _mm_loadu_si128(
                                        reinterpret_cast<const __m128i*>(lanes + 2 * j + 4)
                                    ),
                                    vv
                                )
                            )
                        );
                    }
                    if constexpr (OuterSize % 4 >= 2) {
                        const __m128i prod{_mm_mullo_epi32(
                            _mm_loadu_si128(reinterpret_cast<const __m128i*>(lanes + 2 * j)), vv
                        )};
                        _mm_storel_epi64(
                            reinterpret_cast<__m128i*>(lanes_out + j), _mm_hadd_epi32(prod, prod)
                        );
                        j += 2;
                    }
                    if constexpr (OuterSize % 2 == 1) {
                        const __m128i prod{_mm_mullo_epi32(
                            _mm_loadl_epi64(reinterpret_cast<const __m128i*>(lanes + 2 * j)), v
                        )};
                        lanes_out[j] = _mm_cvtsi128_si32(
                            _mm_add_epi32(prod, _mm_shuffle_epi32(prod, _MM_SHUFFLE(1, 1, 1, 1)))
                        );
                    }
                } else {
                    const __m128i vx{_mm_set1_epi32(static_cast<std::int32_t>(x()))};
                    const __m128i vy{_mm_set1_epi32(static_cast<std::int32_t>(y()))};
                    std::size_t j{0};
                    for (; j + 4 <= OuterSize; j += 4) {
                        _mm_storeu_si128(
                            reinterpret_cast<__m128i*>(lanes_out + j),
                            _mm_add_epi32(
                                _mm_mullo_epi32(
                                    _mm_loadu_si128(reinterpret_cast<const __m128i*>(lanes + j)), vx
                                ),
                                _mm_mullo_epi32(
                                    _mm_loadu_si128(
                                        reinterpret_cast<const __m128i*>(lanes + OuterSize + j)
                                    ),
                                    vy
                                )
                            )
                        );
                    }
                    if constexpr (OuterSize % 4 >= 2) {
                        _mm_storel_epi64(
                            reinterpret_cast<__m128i*>(lanes_out + j),
                            _mm_add_epi32(
                                _mm_mullo_epi32(
                                    _mm_loadl_epi64(reinterpret_cast<const __m128i*>(lanes + j)), vx
                                ),
                                _mm_mullo_epi32(
                                    _mm_loadl_epi64(
                                        reinterpret_cast<const __m128i*>(lanes + OuterSize + j)
                                    ),
                                    vy
                                )
                            )
                        );
                        j += 2;
                    }
                    if constexpr (OuterSize % 2 == 1) {
                        out[j] = x() * m[j] + y() * m[OuterSize + j];
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                const auto* const lanes{reinterpret_cast<const std::int32_t*>(m)};
                auto* const lanes_out{reinterpret_cast<std::int32_t*>(out)};
                if constexpr (kLeft) {
                    // (x, y) against one column, (x, y, x, y) against two.
                    const int32x2_t v{vld1_s32(reinterpret_cast<const std::int32_t*>(data()))};
                    const int32x4_t vv{vcombine_s32(v, v)};
                    std::size_t j{0};
                    for (; j + 2 <= OuterSize; j += 2) {
                        const int32x4_t prod{vmulq_s32(vld1q_s32(lanes + 2 * j), vv)};
                        vst1_s32(lanes_out + j, vpadd_s32(vget_low_s32(prod), vget_high_s32(prod)));
                    }
                    if constexpr (OuterSize % 2 == 1) {
                        lanes_out[j] = vaddv_s32(vmul_s32(vld1_s32(lanes + 2 * j), v));
                    }
                } else {
                    const auto cx{static_cast<std::int32_t>(x())};
                    const auto cy{static_cast<std::int32_t>(y())};
                    std::size_t j{0};
                    for (; j + 4 <= OuterSize; j += 4) {
                        vst1q_s32(
                            lanes_out + j, vaddq_s32(
                                               vmulq_n_s32(vld1q_s32(lanes + j), cx),
                                               vmulq_n_s32(vld1q_s32(lanes + OuterSize + j), cy)
                                           )
                        );
                    }
                    if constexpr (OuterSize % 4 >= 2) {
                        vst1_s32(
                            lanes_out + j, vadd_s32(
                                               vmul_n_s32(vld1_s32(lanes + j), cx),
                                               vmul_n_s32(vld1_s32(lanes + OuterSize + j), cy)
                                           )
                        );
                        j += 2;
                    }
                    if constexpr (OuterSize % 2 == 1) {
                        out[j] = x() * m[j] + y() * m[OuterSize + j];
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
                    const __m128i v{_mm_load_si128(reinterpret_cast<const __m128i*>(data()))};
                    for (std::size_t j{0}; j < OuterSize; ++j) {
                        const __m128i prod{_mm_mullo_epi64(
                            _mm_loadu_si128(reinterpret_cast<const __m128i*>(m + 2 * j)), v
                        )};
                        out[j] = static_cast<value_type>(
                            _mm_cvtsi128_si64(_mm_add_epi64(prod, _mm_unpackhi_epi64(prod, prod)))
                        );
                    }
                } else {
                    const __m128i vx{_mm_set1_epi64x(static_cast<long long>(x()))};
                    const __m128i vy{_mm_set1_epi64x(static_cast<long long>(y()))};
                    std::size_t j{0};
                    for (; j + 2 <= OuterSize; j += 2) {
                        _mm_storeu_si128(
                            reinterpret_cast<__m128i*>(out + j),
                            _mm_add_epi64(
                                _mm_mullo_epi64(
                                    _mm_loadu_si128(reinterpret_cast<const __m128i*>(m + j)), vx
                                ),
                                _mm_mullo_epi64(
                                    _mm_loadu_si128(
                                        reinterpret_cast<const __m128i*>(m + OuterSize + j)
                                    ),
                                    vy
                                )
                            )
                        );
                    }
                    if constexpr (OuterSize % 2 == 1) {
                        out[j] = x() * m[j] + y() * m[OuterSize + j];
                    }
                }
                return result;
#endif
            }
        }
        for (size_type j{0}; j < OuterSize; ++j) {
            const auto column{static_cast<Index>(j)};
            result[j] = x() * obj[0, column] + y() * obj[1, column];
        }
        return result;
    }
    // NOLINTEND(readability-function-size, readability-function-cognitive-complexity)

    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto crossProj(const Vector& obj) const noexcept -> value_type {
        if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
            const __m128i sum{_mm_mullo_epi64(
                _mm_load_si128(reinterpret_cast<const __m128i*>(data())),
                _mm_shuffle_epi32(
                    _mm_load_si128(reinterpret_cast<const __m128i*>(obj.data())),
                    _MM_SHUFFLE(1, 0, 3, 2)
                )
            )};
            return _mm_cvtsi128_si64(
                _mm_sub_epi64(sum, _mm_shuffle_epi32(sum, _MM_SHUFFLE(1, 0, 3, 2)))
            );
#else
            return x() * obj.y() - y() * obj.x();
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX__)
            const __m128d sum{_mm_mul_pd(
                _mm_load_pd(reinterpret_cast<const double*>(data())),
                _mm_permute_pd(
                    _mm_load_pd(reinterpret_cast<const double*>(obj.data())), _MM_SHUFFLE2(0, 1)
                )
            )};
            return _mm_cvtsd_f64(_mm_sub_pd(sum, _mm_permute_pd(sum, _MM_SHUFFLE2(0, 1))));
#else
            return x() * obj.y() - y() * obj.x();
#endif
        } else {
            return x() * obj.y() - y() * obj.x();
        }
    }

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
                const __m128 v{
                    _mm_castsi128_ps(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(data())))
                };
                const __m128 sq{_mm_mul_ps(v, v)};
                return _mm_cvtss_f32(
                    _mm_add_ss(sq, _mm_shuffle_ps(sq, sq, _MM_SHUFFLE(1, 1, 1, 1)))
                );
#elif defined(__ARM_NEON)
                const float32x2_t v{vld1_f32(data())};
                return vaddv_f32(vmul_f32(v, v));
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__SSE2__) || defined(_M_X64)
                const __m128d v{_mm_load_pd(data())};
                const __m128d sq{_mm_mul_pd(v, v)};
                return _mm_cvtsd_f64(_mm_add_sd(sq, _mm_unpackhi_pd(sq, sq)));
#elif defined(__ARM_NEON)
                const float64x2_t v{vld1q_f64(data())};
                return vaddvq_f64(vmulq_f64(v, v));
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                // The squared magnitudes sum the squares of the four real lanes.
#if defined(__SSE2__) || defined(_M_X64)
                const __m128 v{_mm_load_ps(reinterpret_cast<const float*>(data()))};
                const __m128 sq{_mm_mul_ps(v, v)};
                const __m128 pairs{_mm_add_ps(sq, _mm_movehl_ps(sq, sq))};
                return _mm_cvtss_f32(
                    _mm_add_ss(pairs, _mm_shuffle_ps(pairs, pairs, _MM_SHUFFLE(1, 1, 1, 1)))
                );
#elif defined(__ARM_NEON)
                const float32x4_t v{vld1q_f32(reinterpret_cast<const float*>(data()))};
                return vaddvq_f32(vmulq_f32(v, v));
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                // The squared magnitudes sum the squares of the four real lanes.
#if defined(__AVX__)
                const __m256d v{_mm256_load_pd(reinterpret_cast<const double*>(data()))};
                const __m256d sq{_mm256_mul_pd(v, v)};
                const __m128d pairs{
                    _mm_add_pd(_mm256_castpd256_pd128(sq), _mm256_extractf128_pd(sq, 1))
                };
                return _mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)));
#elif defined(__SSE2__) || defined(_M_X64)
                const double* const p{reinterpret_cast<const double*>(data())};
                const __m128d lo{_mm_load_pd(p)};
                const __m128d hi{_mm_load_pd(p + 2)};
                const __m128d pairs{_mm_add_pd(_mm_mul_pd(lo, lo), _mm_mul_pd(hi, hi))};
                return _mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)));
#elif defined(__ARM_NEON)
                const double* const p{reinterpret_cast<const double*>(data())};
                const float64x2_t lo{vld1q_f64(p)};
                const float64x2_t hi{vld1q_f64(p + 2)};
                return vaddvq_f64(vaddq_f64(vmulq_f64(lo, lo), vmulq_f64(hi, hi)));
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int32_t> ||
                std::is_same_v<value_type, std::uint32_t>
            ) {
#if defined(__SSE4_1__) || defined(__AVX__)
                const __m128i v{_mm_loadl_epi64(reinterpret_cast<const __m128i*>(data()))};
                const __m128i sq{_mm_mullo_epi32(v, v)};
                return static_cast<norm_type>(_mm_cvtsi128_si32(
                    _mm_add_epi32(sq, _mm_shuffle_epi32(sq, _MM_SHUFFLE(1, 1, 1, 1)))
                ));
#elif defined(__ARM_NEON)
                const int32x2_t v{vld1_s32(reinterpret_cast<const std::int32_t*>(data()))};
                return static_cast<norm_type>(vaddv_s32(vmul_s32(v, v)));
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int64_t> ||
                std::is_same_v<value_type, std::uint64_t>
            ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
                const __m128i v{_mm_load_si128(reinterpret_cast<const __m128i*>(data()))};
                const __m128i sq{_mm_mullo_epi64(v, v)};
                return static_cast<norm_type>(
                    _mm_cvtsi128_si64(_mm_add_epi64(sq, _mm_unpackhi_epi64(sq, sq)))
                );
#endif
            }
        }
        if constexpr (ComplexArithmetic<value_type>) {
            return x().real() * x().real() + x().imag() * x().imag() + y().real() * y().real() +
                   y().imag() * y().imag();
        } else {
            return static_cast<norm_type>(x() * x() + y() * y());
        }
    }
    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto euclideanSqrTo(const Vector& obj) const noexcept
        -> detail::DenseNormTraitT<value_type> {
        return operator-(obj).euclideanSqr();
    }
    // |<*this, obj>| within tol^2, the inner product conjugating *this. The bound is inclusive, so
    // that an integer tol, 1E-8 truncated to 0 by default, asks for an exactly zero product.
    [[using gnu: pure, always_inline, hot]]
    constexpr auto orthogonalTo(
        const Vector& obj,
        detail::DenseNormTraitT<value_type> tol = detail::DenseNormTraitT<value_type>(1E-8)
    ) const noexcept -> bool {
        if !consteval {
            if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE2__) || defined(_M_X64)
                const __m128 prod{_mm_mul_ps(
                    _mm_castsi128_ps(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(data()))),
                    _mm_castsi128_ps(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(obj.data())))
                )};
                return std::abs(_mm_cvtss_f32(
                           _mm_add_ss(prod, _mm_shuffle_ps(prod, prod, _MM_SHUFFLE(1, 1, 1, 1)))
                       )) <= tol * tol;
#elif defined(__ARM_NEON)
                return std::abs(vaddv_f32(vmul_f32(vld1_f32(data()), vld1_f32(obj.data())))) <=
                       tol * tol;
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__SSE2__) || defined(_M_X64)
                const __m128d prod{_mm_mul_pd(_mm_load_pd(data()), _mm_load_pd(obj.data()))};
                return std::abs(_mm_cvtsd_f64(_mm_add_sd(prod, _mm_unpackhi_pd(prod, prod)))) <=
                       tol * tol;
#elif defined(__ARM_NEON)
                return std::abs(vaddvq_f64(vmulq_f64(vld1q_f64(data()), vld1q_f64(obj.data())))) <=
                       tol * tol;
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__SSE3__) || defined(__AVX__)
                // *this conjugated by flipping the signs of its imaginary lanes.
                const __m128 lhs{_mm_xor_ps(
                    _mm_load_ps(reinterpret_cast<const float*>(data())),
                    _mm_set_ps(-0.0F, 0.0F, -0.0F, 0.0F)
                )};
                const __m128 rhs{_mm_load_ps(reinterpret_cast<const float*>(obj.data()))};
                const __m128 prod{_mm_addsub_ps(
                    _mm_mul_ps(lhs, _mm_moveldup_ps(rhs)),
                    _mm_mul_ps(
                        _mm_shuffle_ps(lhs, lhs, _MM_SHUFFLE(2, 3, 0, 1)), _mm_movehdup_ps(rhs)
                    )
                )};
                value_type inner;
                _mm_storel_pi(
                    reinterpret_cast<__m64*>(&inner), _mm_add_ps(prod, _mm_movehl_ps(prod, prod))
                );
                return std::abs(inner) <= tol * tol;
#elif defined(__ARM_NEON)
                // *this conjugated by flipping the signs of its imaginary lanes.
                const float32x4_t lhs{vreinterpretq_f32_u32(veorq_u32(
                    vreinterpretq_u32_f32(vld1q_f32(reinterpret_cast<const float*>(data()))),
                    vreinterpretq_u32_u64(vdupq_n_u64(0x8000000000000000ULL))
                ))};
                const float32x4_t rhs{vld1q_f32(reinterpret_cast<const float*>(obj.data()))};
                const float32x4_t by_im{vmulq_f32(vrev64q_f32(lhs), vtrn2q_f32(rhs, rhs))};
                const float32x4_t prod{vaddq_f32(
                    vmulq_f32(lhs, vtrn1q_f32(rhs, rhs)),
                    vreinterpretq_f32_u32(veorq_u32(
                        vreinterpretq_u32_f32(by_im),
                        vreinterpretq_u32_u64(vdupq_n_u64(0x80000000ULL))
                    ))
                )};
                value_type inner;
                vst1_f32(
                    reinterpret_cast<float*>(&inner),
                    vadd_f32(vget_low_f32(prod), vget_high_f32(prod))
                );
                return std::abs(inner) <= tol * tol;
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX__)
                // *this conjugated by flipping the signs of its imaginary lanes.
                const __m256d lhs{_mm256_xor_pd(
                    _mm256_load_pd(reinterpret_cast<const double*>(data())),
                    _mm256_set_pd(-0.0, 0.0, -0.0, 0.0)
                )};
                const __m256d rhs{_mm256_load_pd(reinterpret_cast<const double*>(obj.data()))};
                const __m256d prod{_mm256_addsub_pd(
                    _mm256_mul_pd(lhs, _mm256_movedup_pd(rhs)),
                    _mm256_mul_pd(_mm256_permute_pd(lhs, 0b0101), _mm256_permute_pd(rhs, 0b1111))
                )};
                value_type inner;
                _mm_storeu_pd(
                    reinterpret_cast<double*>(&inner),
                    _mm_add_pd(_mm256_castpd256_pd128(prod), _mm256_extractf128_pd(prod, 1))
                );
                return std::abs(inner) <= tol * tol;
#elif defined(__SSE3__)
                // *this conjugated by flipping the signs of its imaginary lanes.
                const double* const lhs{reinterpret_cast<const double*>(data())};
                const double* const rhs{reinterpret_cast<const double*>(obj.data())};
                const __m128d conj_sign{_mm_set_pd(-0.0, 0.0)};
                const __m128d l0{_mm_xor_pd(_mm_load_pd(lhs), conj_sign)};
                const __m128d l1{_mm_xor_pd(_mm_load_pd(lhs + 2), conj_sign)};
                const __m128d r0{_mm_load_pd(rhs)};
                const __m128d r1{_mm_load_pd(rhs + 2)};
                const __m128d p0{_mm_addsub_pd(
                    _mm_mul_pd(l0, _mm_movedup_pd(r0)),
                    _mm_mul_pd(_mm_shuffle_pd(l0, l0, 0b01), _mm_unpackhi_pd(r0, r0))
                )};
                const __m128d p1{_mm_addsub_pd(
                    _mm_mul_pd(l1, _mm_movedup_pd(r1)),
                    _mm_mul_pd(_mm_shuffle_pd(l1, l1, 0b01), _mm_unpackhi_pd(r1, r1))
                )};
                value_type inner;
                _mm_storeu_pd(reinterpret_cast<double*>(&inner), _mm_add_pd(p0, p1));
                return std::abs(inner) <= tol * tol;
#elif defined(__ARM_NEON)
                // *this conjugated by flipping the signs of its imaginary lanes.
                const double* const lhs{reinterpret_cast<const double*>(data())};
                const double* const rhs{reinterpret_cast<const double*>(obj.data())};
                const uint64x2_t conj_sign{
                    vsetq_lane_u64(0x8000000000000000ULL, vdupq_n_u64(0), 1)
                };
                const uint64x2_t sign{vsetq_lane_u64(0x8000000000000000ULL, vdupq_n_u64(0), 0)};
                const float64x2_t l0{vreinterpretq_f64_u64(
                    veorq_u64(vreinterpretq_u64_f64(vld1q_f64(lhs)), conj_sign)
                )};
                const float64x2_t l1{vreinterpretq_f64_u64(
                    veorq_u64(vreinterpretq_u64_f64(vld1q_f64(lhs + 2)), conj_sign)
                )};
                const float64x2_t r0{vld1q_f64(rhs)};
                const float64x2_t r1{vld1q_f64(rhs + 2)};
                const float64x2_t p0{vaddq_f64(
                    vmulq_laneq_f64(l0, r0, 0),
                    vreinterpretq_f64_u64(veorq_u64(
                        vreinterpretq_u64_f64(vmulq_laneq_f64(vextq_f64(l0, l0, 1), r0, 1)), sign
                    ))
                )};
                const float64x2_t p1{vaddq_f64(
                    vmulq_laneq_f64(l1, r1, 0),
                    vreinterpretq_f64_u64(veorq_u64(
                        vreinterpretq_u64_f64(vmulq_laneq_f64(vextq_f64(l1, l1, 1), r1, 1)), sign
                    ))
                )};
                value_type inner;
                vst1q_f64(reinterpret_cast<double*>(&inner), vaddq_f64(p0, p1));
                return std::abs(inner) <= tol * tol;
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int32_t> ||
                std::is_same_v<value_type, std::uint32_t>
            ) {
#if defined(__SSE4_1__) || defined(__AVX__)
                const __m128i prod{_mm_mullo_epi32(
                    _mm_loadl_epi64(reinterpret_cast<const __m128i*>(data())),
                    _mm_loadl_epi64(reinterpret_cast<const __m128i*>(obj.data()))
                )};
                const auto inner{static_cast<value_type>(_mm_cvtsi128_si32(
                    _mm_add_epi32(prod, _mm_shuffle_epi32(prod, _MM_SHUFFLE(1, 1, 1, 1)))
                ))};
                if constexpr (std::is_unsigned_v<value_type>) {
                    return inner <= tol * tol;
                } else {
                    return std::abs(inner) <= tol * tol;
                }
#elif defined(__ARM_NEON)
                const auto inner{static_cast<value_type>(vaddv_s32(vmul_s32(
                    vld1_s32(reinterpret_cast<const std::int32_t*>(data())),
                    vld1_s32(reinterpret_cast<const std::int32_t*>(obj.data()))
                )))};
                if constexpr (std::is_unsigned_v<value_type>) {
                    return inner <= tol * tol;
                } else {
                    return std::abs(inner) <= tol * tol;
                }
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int64_t> ||
                std::is_same_v<value_type, std::uint64_t>
            ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
                const __m128i prod{_mm_mullo_epi64(
                    _mm_load_si128(reinterpret_cast<const __m128i*>(data())),
                    _mm_load_si128(reinterpret_cast<const __m128i*>(obj.data()))
                )};
                const auto inner{static_cast<value_type>(
                    _mm_cvtsi128_si64(_mm_add_epi64(prod, _mm_unpackhi_epi64(prod, prod)))
                )};
                if constexpr (std::is_unsigned_v<value_type>) {
                    return inner <= tol * tol;
                } else {
                    return std::abs(inner) <= tol * tol;
                }
#endif
            }
        }
        if constexpr (ComplexArithmetic<value_type>) {
            return std::abs(std::conj(x()) * obj.x() + std::conj(y()) * obj.y()) <= tol * tol;
        } else if constexpr (std::is_unsigned_v<value_type>) {
            return static_cast<value_type>(x() * obj.x() + y() * obj.y()) <= tol * tol;
        } else {
            return std::abs(x() * obj.x() + y() * obj.y()) <= tol * tol;
        }
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto radian() const noexcept -> value_type
        requires std::floating_point<value_type>
    {
        return std::atan2(y(), x());
    }

    [[using gnu: always_inline, hot]]
    constexpr auto selfRotate(const std::floating_point auto& radian) noexcept -> Vector&
        requires std::floating_point<value_type>
    {
        *this = Vector{
            x() * std::cos(radian) - y() * std::sin(radian),
            x() * std::sin(radian) + y() * std::cos(radian)
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
            Vector result{self};
            result.selfRotate(radian);
            return result;
        }
    }

    [[using gnu: always_inline, hot]]
    constexpr auto selfRotateHalfPi() noexcept -> Vector&
        requires std::floating_point<value_type>
    {
        *this = Vector{-y(), x()};
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
            Vector result{self};
            result.selfRotateHalfPi();
            return result;
        }
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    std::array<value_type, 2> m_data;
};

template <ScalarArithmetic Scalar, std::integral Index = std::size_t>
using Vec2 = Vector<Scalar, Index, 2>;

using Vec2s = Vec2<float>;
using Vec2d = Vec2<double>;
using Vec2c = Vec2<std::complex<float>>;
using Vec2z = Vec2<std::complex<double>>;

template <typename T>
struct isVec2Arithmetic : std::false_type {};

template <ScalarArithmetic T, std::integral Index, Allocatory Alloc>
struct isVec2Arithmetic<Vector<T, Index, 2, Alloc>> : std::true_type {};

template <typename T>
inline constexpr bool isVec2ArithmeticV = isVec2Arithmetic<T>::value;

template <typename T>
concept Vec2Arithmetic = isVec2ArithmeticV<T>;

template <
    std::input_iterator InputIt1, std::input_iterator InputIt2, std::weakly_incrementable OutputIt>
    requires std::floating_point<typename std::iterator_traits<InputIt1>::value_type> &&
             std::floating_point<typename std::iterator_traits<InputIt2>::value_type> &&
             Vec2Arithmetic<typename std::iterator_traits<OutputIt>::value_type>
[[using gnu: always_inline, hot]]
inline auto squeeze(
    InputIt1 first1, std::sentinel_for<InputIt1> auto last1, InputIt2 first2,
    std::sentinel_for<InputIt2> auto last2, OutputIt result
) noexcept -> std::ranges::in_in_out_result<InputIt1, InputIt2, OutputIt> {
    for (; first1 != last1 && first2 != last2; ++first1, ++first2, ++result) {
        *result = typename std::iterator_traits<OutputIt>::value_type{*first1, *first2};
    }
    return std::ranges::in_in_out_result<InputIt1, InputIt2, OutputIt>{first1, first2, result};
}

template <
    std::ranges::input_range InputRange1, std::ranges::input_range InputRange2,
    std::weakly_incrementable OutputIt>
    requires std::floating_point<typename std::ranges::range_value_t<InputRange1>> &&
             std::floating_point<typename std::ranges::range_value_t<InputRange2>> &&
             Vec2Arithmetic<typename std::iterator_traits<OutputIt>::value_type>
[[using gnu: always_inline, hot]]
inline auto squeeze(InputRange1&& xs, InputRange2&& ys, OutputIt result) noexcept
    -> std::ranges::in_in_out_result<
        std::ranges::iterator_t<InputRange1>, std::ranges::iterator_t<InputRange2>, OutputIt> {
    return squeeze(
        std::ranges::begin(xs), std::ranges::end(xs), std::ranges::begin(ys), std::ranges::end(ys),
        result
    );
}

} // namespace basjoo::math

// NOLINTBEGIN(cert-dcl58-cpp)

namespace std {

template <::basjoo::math::Vec2Arithmetic V>
    requires std::floating_point<typename V::value_type>
[[using gnu: pure, always_inline]]
inline constexpr auto hypot(const V& obj) noexcept
    -> ::basjoo::math::detail::DenseNormTraitT<typename V::value_type> {
    return std::hypot(obj.x(), obj.y());
}

template <::basjoo::math::Vec2Arithmetic V>
    requires std::floating_point<typename V::value_type>
[[using gnu: pure, always_inline]]
inline constexpr auto atan2(const V& obj) noexcept -> typename V::value_type {
    return std::atan2(obj.y(), obj.x());
}

} // namespace std

// NOLINTEND(cert-dcl58-cpp)
