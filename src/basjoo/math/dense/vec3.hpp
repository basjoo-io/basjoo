/**
 * @file vec3.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-08-02
 *
 * @copyright Copyright (c) 2023 basjoo development team
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

#include "basjoo/common/in_in_in_out_result.hpp"
#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_trait.hpp"
#include "basjoo/math/dense/detail/dense_norm_trait.hpp"
#include "basjoo/math/dense/vector_base.hpp"
#include "basjoo/math/type_traits.hpp"

namespace basjoo::math {

template <ScalarArithmetic Scalar, std::integral Index, Allocatory Alloc>
class alignas(4 * sizeof(Scalar)) Vector<Scalar, Index, 3, Alloc> final : public VectorBase<false> {
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
        : m_data{cv, cv, cv, value_type(0.0)} {}
    [[using gnu: always_inline]]
    constexpr Vector(value_type cx, value_type cy, value_type cz) noexcept
        : m_data{cx, cy, cz, value_type(0.0)} {}
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

    // Leaves x, y and z uninitialized; only the padding is set, to zero.
    [[using gnu: always_inline]]
    constexpr Vector() noexcept {
        m_data[3] = value_type(0.0);
    }

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
        if (count != static_cast<size_type>(extent(0))) [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
    }

    [[using gnu: always_inline]]
    constexpr explicit Vector(
        [[maybe_unused]] size_type count, const_reference value,
        [[maybe_unused]] const allocator_type& alloc
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : m_data{value, value, value, value_type(0.0)} {
#if BASJOO_CHECK_PARAMS == 1
        if (count != static_cast<size_type>(extent(0))) [[unlikely]] {
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
        if (std::ranges::size(rg) != static_cast<size_type>(extent(0))) [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
        auto it{std::ranges::begin(rg)};
        m_data[0] = *it;
        ++it;
        m_data[1] = *it;
        ++it;
        m_data[2] = *it;
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
        if (std::cmp_not_equal(matrix_like.extent(0) * matrix_like.extent(1), extent(0)))
            [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
        const bool row_like{matrix_like.extent(1) != 1};
        for (size_type i{0}; i < static_cast<size_type>(extent(0)); ++i) {
            const auto k{static_cast<other_index_type>(i)};
            operator[](i) = row_like ? matrix_like[0, k] : matrix_like[k, 0];
        }
    }

    template <ScalarArithmetic U>
    [[using gnu: always_inline]]
    constexpr explicit Vector(const std::tuple<U, U, U>& other) noexcept
        : m_data{
              static_cast<value_type>(std::get<0>(other)),
              static_cast<value_type>(std::get<1>(other)),
              static_cast<value_type>(std::get<2>(other)), value_type(0.0)
          } {}

    template <ScalarArithmetic U>
    [[using gnu: always_inline]]
    constexpr explicit Vector(const Vector<U, Index, 3>& other) noexcept
        : m_data{
              static_cast<value_type>(other.x()), static_cast<value_type>(other.y()),
              static_cast<value_type>(other.z()), value_type(0.0)
          } {}

    [[using gnu: const, always_inline, leaf]]
    // The storage footprint, padding included; the dimension is extent(0).
    static constexpr auto size() noexcept -> size_type {
        return 4;
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

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto z(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[2];
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
        return 3;
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
    constexpr operator std::span<value_type, 3>() noexcept {
        return std::span<value_type, 3>{data(), 3};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<const value_type, 3>() const noexcept {
        return std::span<const value_type, 3>{data(), 3};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<value_type>() noexcept {
        return std::span<value_type>{data(), 3};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<const value_type>() const noexcept {
        return std::span<const value_type>{data(), 3};
    }

    // The padding of both vectors is zero, so that the four slots multiply to the three-element
    // product.
    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto dot(const Vector& obj) const noexcept -> value_type {
        if !consteval {
            if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE2__) || defined(_M_X64)
                const __m128 prod{_mm_mul_ps(_mm_load_ps(data()), _mm_load_ps(obj.data()))};
                const __m128 pairs{_mm_add_ps(prod, _mm_movehl_ps(prod, prod))};
                return _mm_cvtss_f32(
                    _mm_add_ss(pairs, _mm_shuffle_ps(pairs, pairs, _MM_SHUFFLE(1, 1, 1, 1)))
                );
#elif defined(__ARM_NEON)
                return vaddvq_f32(vmulq_f32(vld1q_f32(data()), vld1q_f32(obj.data())));
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX__)
                const __m256d prod{
                    _mm256_mul_pd(_mm256_load_pd(data()), _mm256_load_pd(obj.data()))
                };
                const __m128d pairs{
                    _mm_add_pd(_mm256_castpd256_pd128(prod), _mm256_extractf128_pd(prod, 1))
                };
                return _mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)));
#elif defined(__SSE2__) || defined(_M_X64)
                const __m128d pairs{_mm_add_pd(
                    _mm_mul_pd(_mm_load_pd(data()), _mm_load_pd(obj.data())),
                    _mm_mul_pd(_mm_load_pd(data() + 2), _mm_load_pd(obj.data() + 2))
                )};
                return _mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)));
#elif defined(__ARM_NEON)
                return vaddvq_f64(vaddq_f64(
                    vmulq_f64(vld1q_f64(data()), vld1q_f64(obj.data())),
                    vmulq_f64(vld1q_f64(data() + 2), vld1q_f64(obj.data() + 2))
                ));
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__AVX__)
                const __m256 lhs{_mm256_load_ps(reinterpret_cast<const float*>(data()))};
                const __m256 rhs{_mm256_load_ps(reinterpret_cast<const float*>(obj.data()))};
                // (a c - b d, b c + a d) per element, for a + b i of *this and c + d i of obj.
                const __m256 prod{_mm256_addsub_ps(
                    _mm256_mul_ps(lhs, _mm256_moveldup_ps(rhs)),
                    _mm256_mul_ps(
                        _mm256_permute_ps(lhs, _MM_SHUFFLE(2, 3, 0, 1)), _mm256_movehdup_ps(rhs)
                    )
                )};
                const __m128 halves{
                    _mm_add_ps(_mm256_castps256_ps128(prod), _mm256_extractf128_ps(prod, 1))
                };
                value_type result;
                _mm_storel_pi(
                    reinterpret_cast<__m64*>(&result),
                    _mm_add_ps(halves, _mm_movehl_ps(halves, halves))
                );
                return result;
#elif defined(__SSE3__)
                const float* const lhs{reinterpret_cast<const float*>(data())};
                const float* const rhs{reinterpret_cast<const float*>(obj.data())};
                const __m128 l0{_mm_load_ps(lhs)};
                const __m128 l1{_mm_load_ps(lhs + 4)};
                const __m128 r0{_mm_load_ps(rhs)};
                const __m128 r1{_mm_load_ps(rhs + 4)};
                // (a c - b d, b c + a d) per element, for a + b i of *this and c + d i of obj.
                const __m128 sum{_mm_add_ps(
                    _mm_addsub_ps(
                        _mm_mul_ps(l0, _mm_moveldup_ps(r0)),
                        _mm_mul_ps(
                            _mm_shuffle_ps(l0, l0, _MM_SHUFFLE(2, 3, 0, 1)), _mm_movehdup_ps(r0)
                        )
                    ),
                    _mm_addsub_ps(
                        _mm_mul_ps(l1, _mm_moveldup_ps(r1)),
                        _mm_mul_ps(
                            _mm_shuffle_ps(l1, l1, _MM_SHUFFLE(2, 3, 0, 1)), _mm_movehdup_ps(r1)
                        )
                    )
                )};
                value_type result;
                _mm_storel_pi(
                    reinterpret_cast<__m64*>(&result), _mm_add_ps(sum, _mm_movehl_ps(sum, sum))
                );
                return result;
#elif defined(__ARM_NEON)
                const float* const lhs{reinterpret_cast<const float*>(data())};
                const float* const rhs{reinterpret_cast<const float*>(obj.data())};
                const float32x4_t l0{vld1q_f32(lhs)};
                const float32x4_t l1{vld1q_f32(lhs + 4)};
                const float32x4_t r0{vld1q_f32(rhs)};
                const float32x4_t r1{vld1q_f32(rhs + 4)};
                // (a c, b c) plus (b d, a d) with its real lane negated, per element.
                const uint32x4_t sign{vreinterpretq_u32_u64(vdupq_n_u64(0x80000000ULL))};
                const float32x4_t sum{vaddq_f32(
                    vaddq_f32(
                        vmulq_f32(l0, vtrn1q_f32(r0, r0)),
                        vreinterpretq_f32_u32(veorq_u32(
                            vreinterpretq_u32_f32(vmulq_f32(vrev64q_f32(l0), vtrn2q_f32(r0, r0))),
                            sign
                        ))
                    ),
                    vaddq_f32(
                        vmulq_f32(l1, vtrn1q_f32(r1, r1)),
                        vreinterpretq_f32_u32(veorq_u32(
                            vreinterpretq_u32_f32(vmulq_f32(vrev64q_f32(l1), vtrn2q_f32(r1, r1))),
                            sign
                        ))
                    )
                )};
                value_type result;
                vst1_f32(
                    reinterpret_cast<float*>(&result),
                    vadd_f32(vget_low_f32(sum), vget_high_f32(sum))
                );
                return result;
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX__)
                const double* const lhs{reinterpret_cast<const double*>(data())};
                const double* const rhs{reinterpret_cast<const double*>(obj.data())};
                const __m256d l0{_mm256_load_pd(lhs)};
                const __m256d l1{_mm256_load_pd(lhs + 4)};
                const __m256d r0{_mm256_load_pd(rhs)};
                const __m256d r1{_mm256_load_pd(rhs + 4)};
                // (a c - b d, b c + a d) per element, for a + b i of *this and c + d i of obj.
                const __m256d sum{_mm256_add_pd(
                    _mm256_addsub_pd(
                        _mm256_mul_pd(l0, _mm256_movedup_pd(r0)),
                        _mm256_mul_pd(_mm256_permute_pd(l0, 0b0101), _mm256_permute_pd(r0, 0b1111))
                    ),
                    _mm256_addsub_pd(
                        _mm256_mul_pd(l1, _mm256_movedup_pd(r1)),
                        _mm256_mul_pd(_mm256_permute_pd(l1, 0b0101), _mm256_permute_pd(r1, 0b1111))
                    )
                )};
                value_type result;
                _mm_storeu_pd(
                    reinterpret_cast<double*>(&result),
                    _mm_add_pd(_mm256_castpd256_pd128(sum), _mm256_extractf128_pd(sum, 1))
                );
                return result;
#elif defined(__SSE3__)
                const double* const lhs{reinterpret_cast<const double*>(data())};
                const double* const rhs{reinterpret_cast<const double*>(obj.data())};
                // The three elements, the padding left out.
                __m128d sum{_mm_setzero_pd()};
                for (std::size_t k{0}; k < 3; ++k) {
                    const __m128d l{_mm_load_pd(lhs + 2 * k)};
                    const __m128d r{_mm_load_pd(rhs + 2 * k)};
                    sum = _mm_add_pd(
                        sum, _mm_addsub_pd(
                                 _mm_mul_pd(l, _mm_movedup_pd(r)),
                                 _mm_mul_pd(_mm_shuffle_pd(l, l, 0b01), _mm_unpackhi_pd(r, r))
                             )
                    );
                }
                value_type result;
                _mm_storeu_pd(reinterpret_cast<double*>(&result), sum);
                return result;
#elif defined(__ARM_NEON)
                const double* const lhs{reinterpret_cast<const double*>(data())};
                const double* const rhs{reinterpret_cast<const double*>(obj.data())};
                // The three elements, the padding left out; (a c, b c) plus (b d, a d) with its
                // real lane negated, per element.
                const uint64x2_t sign{vsetq_lane_u64(0x8000000000000000ULL, vdupq_n_u64(0), 0)};
                float64x2_t sum{vdupq_n_f64(0.0)};
                for (std::size_t k{0}; k < 3; ++k) {
                    const float64x2_t l{vld1q_f64(lhs + 2 * k)};
                    const float64x2_t r{vld1q_f64(rhs + 2 * k)};
                    sum = vaddq_f64(
                        sum,
                        vaddq_f64(
                            vmulq_laneq_f64(l, r, 0),
                            vreinterpretq_f64_u64(veorq_u64(
                                vreinterpretq_u64_f64(vmulq_laneq_f64(vextq_f64(l, l, 1), r, 1)),
                                sign
                            ))
                        )
                    );
                }
                value_type result;
                vst1q_f64(reinterpret_cast<double*>(&result), sum);
                return result;
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int32_t> ||
                std::is_same_v<value_type, std::uint32_t>
            ) {
#if defined(__SSE4_1__) || defined(__AVX__)
                const __m128i prod{_mm_mullo_epi32(
                    _mm_load_si128(reinterpret_cast<const __m128i*>(data())),
                    _mm_load_si128(reinterpret_cast<const __m128i*>(obj.data()))
                )};
                const __m128i pairs{
                    _mm_add_epi32(prod, _mm_shuffle_epi32(prod, _MM_SHUFFLE(1, 0, 3, 2)))
                };
                return static_cast<value_type>(_mm_cvtsi128_si32(
                    _mm_add_epi32(pairs, _mm_shuffle_epi32(pairs, _MM_SHUFFLE(2, 3, 0, 1)))
                ));
#elif defined(__ARM_NEON)
                return static_cast<value_type>(vaddvq_s32(vmulq_s32(
                    vld1q_s32(reinterpret_cast<const std::int32_t*>(data())),
                    vld1q_s32(reinterpret_cast<const std::int32_t*>(obj.data()))
                )));
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int64_t> ||
                std::is_same_v<value_type, std::uint64_t>
            ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
                const __m256i prod{_mm256_mullo_epi64(
                    _mm256_load_si256(reinterpret_cast<const __m256i*>(data())),
                    _mm256_load_si256(reinterpret_cast<const __m256i*>(obj.data()))
                )};
                const __m128i pairs{
                    _mm_add_epi64(_mm256_castsi256_si128(prod), _mm256_extracti128_si256(prod, 1))
                };
                return static_cast<value_type>(
                    _mm_cvtsi128_si64(_mm_add_epi64(pairs, _mm_unpackhi_epi64(pairs, pairs)))
                );
#endif
            }
        }
        return x() * obj.x() + y() * obj.y() + z() * obj.z();
    }

    // The product v^T M with a matrix of three rows. Under layout_left, each element is the dot
    // product with a column of M; under layout_right, the result combines the rows of M. A Mat33
    // stores them in four slots, and its product is built in one register: a 16-byte result is
    // bit_cast from it, a 32-byte one stored whole, as GCC spills a bit_cast of that size.
    // NOLINTBEGIN(readability-function-size, readability-function-cognitive-complexity)
    template <std::size_t OuterSize, DenseLayout ObjLayout>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(
        const Matrix<value_type, Index, 3, OuterSize, ObjLayout, Alloc>& obj
    ) const noexcept -> Vector<value_type, Index, OuterSize, Alloc> {
        using result_type = Vector<value_type, Index, OuterSize, Alloc>;
        [[maybe_unused]] constexpr bool kLeft{std::is_same_v<ObjLayout, std::layout_left>};

        // Every element is written below; a Vec3 result constructs its padding.
        result_type result;
        if !consteval {
            [[maybe_unused]] const value_type* const m{obj.data()};
            [[maybe_unused]] value_type* const out{result.data()};
            if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE3__) || defined(__AVX__)
                const __m128 v{_mm_load_ps(data())};
                if constexpr (OuterSize == 3 && kLeft) {
                    // hadd folds the three column products and a zero fourth into (c0, c1, c2, 0).
                    const __m128 h01{_mm_hadd_ps(
                        _mm_mul_ps(v, _mm_load_ps(m)), _mm_mul_ps(v, _mm_load_ps(m + 4))
                    )};
                    const __m128 h2{
                        _mm_hadd_ps(_mm_mul_ps(v, _mm_load_ps(m + 8)), _mm_setzero_ps())
                    };
                    return std::bit_cast<result_type>(_mm_hadd_ps(h01, h2));
                } else if constexpr (OuterSize == 3) {
                    // The rows combined: their zero padding sums to the padding of the result.
                    return std::bit_cast<result_type>(_mm_add_ps(
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
                } else if constexpr (kLeft) {
                    // Four columns a step: their twelve elements (c0 c1 c2 d0), (d1 d2 e0 e1) and
                    // (e2 f0 f1 f2) are shuffled into (c0 d0 e0 f0), (c1 d1 e1 f1) and
                    // (c2 d2 e2 f2), so that a whole register is stored. The other columns go one
                    // at a time, loaded without reading past them.
                    std::size_t j{0};
                    for (; j + 4 <= OuterSize; j += 4) {
                        const __m128 p0{_mm_loadu_ps(m + 3 * j)};
                        const __m128 p1{_mm_loadu_ps(m + 3 * j + 4)};
                        const __m128 p2{_mm_loadu_ps(m + 3 * j + 8)};
                        const __m128 r0{_mm_shuffle_ps(
                            p0, _mm_shuffle_ps(p1, p2, _MM_SHUFFLE(1, 1, 2, 2)),
                            _MM_SHUFFLE(2, 0, 3, 0)
                        )};
                        const __m128 r1{_mm_shuffle_ps(
                            _mm_shuffle_ps(p0, p1, _MM_SHUFFLE(0, 0, 1, 1)),
                            _mm_shuffle_ps(p1, p2, _MM_SHUFFLE(2, 2, 3, 3)), _MM_SHUFFLE(2, 0, 2, 0)
                        )};
                        const __m128 r2{_mm_shuffle_ps(
                            _mm_shuffle_ps(p0, p1, _MM_SHUFFLE(1, 1, 2, 2)), p2,
                            _MM_SHUFFLE(3, 0, 2, 0)
                        )};
                        _mm_storeu_ps(
                            out + j,
                            _mm_add_ps(
                                _mm_add_ps(
                                    _mm_mul_ps(r0, _mm_shuffle_ps(v, v, _MM_SHUFFLE(0, 0, 0, 0))),
                                    _mm_mul_ps(r1, _mm_shuffle_ps(v, v, _MM_SHUFFLE(1, 1, 1, 1)))
                                ),
                                _mm_mul_ps(r2, _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 2, 2, 2)))
                            )
                        );
                    }
                    for (; j < OuterSize; ++j) {
                        const __m128 prod{_mm_mul_ps(
                            _mm_movelh_ps(
                                _mm_castsi128_ps(
                                    _mm_loadl_epi64(reinterpret_cast<const __m128i*>(m + 3 * j))
                                ),
                                _mm_load_ss(m + 3 * j + 2)
                            ),
                            v
                        )};
                        const __m128 pairs{_mm_add_ps(prod, _mm_movehl_ps(prod, prod))};
                        out[j] = _mm_cvtss_f32(
                            _mm_add_ss(pairs, _mm_shuffle_ps(pairs, pairs, _MM_SHUFFLE(1, 1, 1, 1)))
                        );
                    }
                } else {
                    const __m128 vx{_mm_set1_ps(x())};
                    const __m128 vy{_mm_set1_ps(y())};
                    const __m128 vz{_mm_set1_ps(z())};
                    std::size_t j{0};
                    for (; j + 4 <= OuterSize; j += 4) {
                        _mm_storeu_ps(
                            out + j, _mm_add_ps(
                                         _mm_add_ps(
                                             _mm_mul_ps(_mm_loadu_ps(m + j), vx),
                                             _mm_mul_ps(_mm_loadu_ps(m + OuterSize + j), vy)
                                         ),
                                         _mm_mul_ps(_mm_loadu_ps(m + 2 * OuterSize + j), vz)
                                     )
                        );
                    }
                    for (; j < OuterSize; ++j) {
                        out[j] = x() * m[j] + y() * m[OuterSize + j] + z() * m[2 * OuterSize + j];
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                const float32x4_t v{vld1q_f32(data())};
                if constexpr (OuterSize == 3 && kLeft) {
                    // Pairwise adds fold the three column products and a zero fourth into
                    // (c0, c1, c2, 0).
                    const float32x4_t p01{
                        vpaddq_f32(vmulq_f32(v, vld1q_f32(m)), vmulq_f32(v, vld1q_f32(m + 4)))
                    };
                    const float32x4_t p2{
                        vpaddq_f32(vmulq_f32(v, vld1q_f32(m + 8)), vdupq_n_f32(0.0F))
                    };
                    return std::bit_cast<result_type>(vpaddq_f32(p01, p2));
                } else if constexpr (OuterSize == 3) {
                    // The rows combined: their zero padding sums to the padding of the result.
                    return std::bit_cast<result_type>(vaddq_f32(
                        vaddq_f32(
                            vmulq_laneq_f32(vld1q_f32(m), v, 0),
                            vmulq_laneq_f32(vld1q_f32(m + 4), v, 1)
                        ),
                        vmulq_laneq_f32(vld1q_f32(m + 8), v, 2)
                    ));
                } else if constexpr (kLeft) {
                    // Four columns a step, deinterleaved into (c0 d0 e0 f0), (c1 d1 e1 f1) and
                    // (c2 d2 e2 f2) by vld3, so that a whole register is stored. The other columns
                    // go one at a time, loaded without reading past them.
                    std::size_t j{0};
                    for (; j + 4 <= OuterSize; j += 4) {
                        const float32x4x3_t rows{vld3q_f32(m + 3 * j)};
                        vst1q_f32(
                            out + j, vaddq_f32(
                                         vaddq_f32(
                                             vmulq_laneq_f32(rows.val[0], v, 0),
                                             vmulq_laneq_f32(rows.val[1], v, 1)
                                         ),
                                         vmulq_laneq_f32(rows.val[2], v, 2)
                                     )
                        );
                    }
                    for (; j < OuterSize; ++j) {
                        const float32x4_t col{vcombine_f32(
                            vld1_f32(m + 3 * j), vld1_lane_f32(m + 3 * j + 2, vdup_n_f32(0.0F), 0)
                        )};
                        out[j] = vaddvq_f32(vmulq_f32(col, v));
                    }
                } else {
                    std::size_t j{0};
                    for (; j + 4 <= OuterSize; j += 4) {
                        vst1q_f32(
                            out + j, vaddq_f32(
                                         vaddq_f32(
                                             vmulq_n_f32(vld1q_f32(m + j), x()),
                                             vmulq_n_f32(vld1q_f32(m + OuterSize + j), y())
                                         ),
                                         vmulq_n_f32(vld1q_f32(m + 2 * OuterSize + j), z())
                                     )
                        );
                    }
                    for (; j < OuterSize; ++j) {
                        out[j] = x() * m[j] + y() * m[OuterSize + j] + z() * m[2 * OuterSize + j];
                    }
                }
                return result;
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
                if constexpr (OuterSize == 3) {
#if defined(__AVX__)
                    const __m256d v{_mm256_load_pd(data())};
                    if constexpr (kLeft) {
                        // hadd gives (a01, b01, a23, b23) and (c01, 0, c23, 0) for the column
                        // products a, b and c; their 128-bit halves sum to (a, b) and (c, 0).
                        const __m256d h01{_mm256_hadd_pd(
                            _mm256_mul_pd(v, _mm256_load_pd(m)),
                            _mm256_mul_pd(v, _mm256_load_pd(m + 4))
                        )};
                        const __m256d h2{_mm256_hadd_pd(
                            _mm256_mul_pd(v, _mm256_load_pd(m + 8)), _mm256_setzero_pd()
                        )};
                        _mm256_store_pd(
                            out, _mm256_insertf128_pd(
                                     _mm256_castpd128_pd256(_mm_add_pd(
                                         _mm256_castpd256_pd128(h01), _mm256_extractf128_pd(h01, 1)
                                     )),
                                     _mm_add_pd(
                                         _mm256_castpd256_pd128(h2), _mm256_extractf128_pd(h2, 1)
                                     ),
                                     1
                                 )
                        );
                        return result;
                    } else {
                        // The rows combined: their zero padding sums to the padding of the result.
                        _mm256_store_pd(
                            out, _mm256_add_pd(
                                     _mm256_add_pd(
                                         _mm256_mul_pd(_mm256_load_pd(m), _mm256_set1_pd(x())),
                                         _mm256_mul_pd(_mm256_load_pd(m + 4), _mm256_set1_pd(y()))
                                     ),
                                     _mm256_mul_pd(_mm256_load_pd(m + 8), _mm256_set1_pd(z()))
                                 )
                        );
                        return result;
                    }
#elif defined(__SSE2__) || defined(_M_X64)
                    const __m128d v_lo{_mm_load_pd(data())};
                    const __m128d v_hi{_mm_load_pd(data() + 2)};
                    if constexpr (kLeft) {
                        // Each stored column (c0, c1 | c2, 0) against (x, y | z, 0).
                        const __m128d p0{_mm_add_pd(
                            _mm_mul_pd(_mm_load_pd(m), v_lo), _mm_mul_pd(_mm_load_pd(m + 2), v_hi)
                        )};
                        const __m128d p1{_mm_add_pd(
                            _mm_mul_pd(_mm_load_pd(m + 4), v_lo),
                            _mm_mul_pd(_mm_load_pd(m + 6), v_hi)
                        )};
                        const __m128d p2{_mm_add_pd(
                            _mm_mul_pd(_mm_load_pd(m + 8), v_lo),
                            _mm_mul_pd(_mm_load_pd(m + 10), v_hi)
                        )};
                        const __m128d ab{
                            _mm_add_pd(_mm_unpacklo_pd(p0, p1), _mm_unpackhi_pd(p0, p1))
                        };
                        return Vector{
                            _mm_cvtsd_f64(ab), _mm_cvtsd_f64(_mm_unpackhi_pd(ab, ab)),
                            _mm_cvtsd_f64(_mm_add_sd(p2, _mm_unpackhi_pd(p2, p2)))
                        };
                    } else {
                        // The rows combined, (x, y) halves and (z, 0) halves apart.
                        const __m128d vx{_mm_set1_pd(x())};
                        const __m128d vy{_mm_set1_pd(y())};
                        const __m128d vz{_mm_set1_pd(z())};
                        const __m128d lo{_mm_add_pd(
                            _mm_add_pd(
                                _mm_mul_pd(_mm_load_pd(m), vx), _mm_mul_pd(_mm_load_pd(m + 4), vy)
                            ),
                            _mm_mul_pd(_mm_load_pd(m + 8), vz)
                        )};
                        const __m128d hi{_mm_add_pd(
                            _mm_add_pd(
                                _mm_mul_pd(_mm_load_pd(m + 2), vx),
                                _mm_mul_pd(_mm_load_pd(m + 6), vy)
                            ),
                            _mm_mul_pd(_mm_load_pd(m + 10), vz)
                        )};
                        return Vector{
                            _mm_cvtsd_f64(lo), _mm_cvtsd_f64(_mm_unpackhi_pd(lo, lo)),
                            _mm_cvtsd_f64(hi)
                        };
                    }
#elif defined(__ARM_NEON)
                    const float64x2_t v_lo{vld1q_f64(data())};
                    const float64x2_t v_hi{vld1q_f64(data() + 2)};
                    if constexpr (kLeft) {
                        // Each stored column (c0, c1 | c2, 0) against (x, y | z, 0).
                        const float64x2_t p0{vaddq_f64(
                            vmulq_f64(vld1q_f64(m), v_lo), vmulq_f64(vld1q_f64(m + 2), v_hi)
                        )};
                        const float64x2_t p1{vaddq_f64(
                            vmulq_f64(vld1q_f64(m + 4), v_lo), vmulq_f64(vld1q_f64(m + 6), v_hi)
                        )};
                        const float64x2_t p2{vaddq_f64(
                            vmulq_f64(vld1q_f64(m + 8), v_lo), vmulq_f64(vld1q_f64(m + 10), v_hi)
                        )};
                        const float64x2_t ab{vpaddq_f64(p0, p1)};
                        return Vector{vgetq_lane_f64(ab, 0), vgetq_lane_f64(ab, 1), vaddvq_f64(p2)};
                    } else {
                        // The rows combined, (x, y) halves and (z, 0) halves apart.
                        const float64x2_t lo{vaddq_f64(
                            vaddq_f64(
                                vmulq_n_f64(vld1q_f64(m), x()), vmulq_n_f64(vld1q_f64(m + 4), y())
                            ),
                            vmulq_n_f64(vld1q_f64(m + 8), z())
                        )};
                        const float64x2_t hi{vaddq_f64(
                            vaddq_f64(
                                vmulq_n_f64(vld1q_f64(m + 2), x()),
                                vmulq_n_f64(vld1q_f64(m + 6), y())
                            ),
                            vmulq_n_f64(vld1q_f64(m + 10), z())
                        )};
                        return Vector{
                            vgetq_lane_f64(lo, 0), vgetq_lane_f64(lo, 1), vgetq_lane_f64(hi, 0)
                        };
                    }
#endif
                } else {
#if defined(__SSE2__) || defined(_M_X64)
                    if constexpr (kLeft) {
                        // Two columns a step: their six elements (c0 c1), (c2 d0) and (d1 d2) are
                        // shuffled into (c0 d0), (c1 d1) and (c2 d2), so that a whole register is
                        // stored. An odd last column takes (c0, c1) against (x, y) and c2 against
                        // z, so that no load reads past it.
                        const __m128d v_lo{_mm_load_pd(data())};
                        const __m128d v_hi{_mm_load_pd(data() + 2)};
                        std::size_t j{0};
                        for (; j + 2 <= OuterSize; j += 2) {
                            const __m128d q0{_mm_loadu_pd(m + 3 * j)};
                            const __m128d q1{_mm_loadu_pd(m + 3 * j + 2)};
                            const __m128d q2{_mm_loadu_pd(m + 3 * j + 4)};
                            _mm_storeu_pd(
                                out + j,
                                _mm_add_pd(
                                    _mm_add_pd(
                                        _mm_mul_pd(
                                            _mm_shuffle_pd(q0, q1, 0b10),
                                            _mm_unpacklo_pd(v_lo, v_lo)
                                        ),
                                        _mm_mul_pd(
                                            _mm_shuffle_pd(q0, q2, 0b01),
                                            _mm_unpackhi_pd(v_lo, v_lo)
                                        )
                                    ),
                                    _mm_mul_pd(
                                        _mm_shuffle_pd(q1, q2, 0b10), _mm_unpacklo_pd(v_hi, v_hi)
                                    )
                                )
                            );
                        }
                        if constexpr (OuterSize % 2 == 1) {
                            const __m128d prod{_mm_mul_pd(_mm_loadu_pd(m + 3 * j), v_lo)};
                            out[j] = _mm_cvtsd_f64(_mm_add_sd(prod, _mm_unpackhi_pd(prod, prod))) +
                                     m[3 * j + 2] * z();
                        }
                    } else {
                        const __m128d vx{_mm_set1_pd(x())};
                        const __m128d vy{_mm_set1_pd(y())};
                        const __m128d vz{_mm_set1_pd(z())};
                        std::size_t j{0};
                        for (; j + 2 <= OuterSize; j += 2) {
                            _mm_storeu_pd(
                                out + j, _mm_add_pd(
                                             _mm_add_pd(
                                                 _mm_mul_pd(_mm_loadu_pd(m + j), vx),
                                                 _mm_mul_pd(_mm_loadu_pd(m + OuterSize + j), vy)
                                             ),
                                             _mm_mul_pd(_mm_loadu_pd(m + 2 * OuterSize + j), vz)
                                         )
                            );
                        }
                        if constexpr (OuterSize % 2 == 1) {
                            out[j] =
                                x() * m[j] + y() * m[OuterSize + j] + z() * m[2 * OuterSize + j];
                        }
                    }
                    return result;
#elif defined(__ARM_NEON)
                    if constexpr (kLeft) {
                        // Two columns a step, deinterleaved into (c0 d0), (c1 d1) and (c2 d2) by
                        // vld3, so that a whole register is stored. An odd last column takes
                        // (c0, c1) against (x, y) and c2 against z, so that no load reads past it.
                        const float64x2_t v_lo{vld1q_f64(data())};
                        std::size_t j{0};
                        for (; j + 2 <= OuterSize; j += 2) {
                            const float64x2x3_t rows{vld3q_f64(m + 3 * j)};
                            vst1q_f64(
                                out + j, vaddq_f64(
                                             vaddq_f64(
                                                 vmulq_laneq_f64(rows.val[0], v_lo, 0),
                                                 vmulq_laneq_f64(rows.val[1], v_lo, 1)
                                             ),
                                             vmulq_n_f64(rows.val[2], z())
                                         )
                            );
                        }
                        if constexpr (OuterSize % 2 == 1) {
                            out[j] = vaddvq_f64(vmulq_f64(vld1q_f64(m + 3 * j), v_lo)) +
                                     m[3 * j + 2] * z();
                        }
                    } else {
                        std::size_t j{0};
                        for (; j + 2 <= OuterSize; j += 2) {
                            vst1q_f64(
                                out + j, vaddq_f64(
                                             vaddq_f64(
                                                 vmulq_n_f64(vld1q_f64(m + j), x()),
                                                 vmulq_n_f64(vld1q_f64(m + OuterSize + j), y())
                                             ),
                                             vmulq_n_f64(vld1q_f64(m + 2 * OuterSize + j), z())
                                         )
                            );
                        }
                        if constexpr (OuterSize % 2 == 1) {
                            out[j] =
                                x() * m[j] + y() * m[OuterSize + j] + z() * m[2 * OuterSize + j];
                        }
                    }
                    return result;
#endif
                }
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                if constexpr (OuterSize == 3) {
#if defined(__AVX__)
                    const float* const lanes{reinterpret_cast<const float*>(m)};
                    const __m256 v{_mm256_load_ps(reinterpret_cast<const float*>(data()))};
                    if constexpr (kLeft) {
                        // The complex products (a c - b d, b c + a d) with each stored column,
                        // folded to one element each and gathered as (r0, r1 | r2, 0).
                        const __m256 v_re{_mm256_moveldup_ps(v)};
                        const __m256 v_im{_mm256_movehdup_ps(v)};
                        const __m256 c0{_mm256_load_ps(lanes)};
                        const __m256 c1{_mm256_load_ps(lanes + 8)};
                        const __m256 c2{_mm256_load_ps(lanes + 16)};
                        const __m256 p0{_mm256_addsub_ps(
                            _mm256_mul_ps(c0, v_re),
                            _mm256_mul_ps(_mm256_permute_ps(c0, _MM_SHUFFLE(2, 3, 0, 1)), v_im)
                        )};
                        const __m256 p1{_mm256_addsub_ps(
                            _mm256_mul_ps(c1, v_re),
                            _mm256_mul_ps(_mm256_permute_ps(c1, _MM_SHUFFLE(2, 3, 0, 1)), v_im)
                        )};
                        const __m256 p2{_mm256_addsub_ps(
                            _mm256_mul_ps(c2, v_re),
                            _mm256_mul_ps(_mm256_permute_ps(c2, _MM_SHUFFLE(2, 3, 0, 1)), v_im)
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
                        const __m128 r0{_mm_add_ps(s0, _mm_movehl_ps(s0, s0))};
                        const __m128 r1{_mm_add_ps(s1, _mm_movehl_ps(s1, s1))};
                        const __m128 r2{_mm_add_ps(s2, _mm_movehl_ps(s2, s2))};
                        _mm256_store_ps(
                            reinterpret_cast<float*>(out),
                            _mm256_insertf128_ps(
                                _mm256_castps128_ps256(_mm_movelh_ps(r0, r1)),
                                _mm_movelh_ps(r2, _mm_setzero_ps()), 1
                            )
                        );
                        return result;
                    } else {
                        // The rows combined, each times its coefficient (re, im) as
                        // (a re - b im, b re + a im).
                        const __m256 r0{_mm256_load_ps(lanes)};
                        const __m256 r1{_mm256_load_ps(lanes + 8)};
                        const __m256 r2{_mm256_load_ps(lanes + 16)};
                        _mm256_store_ps(
                            reinterpret_cast<float*>(out),
                            _mm256_add_ps(
                                _mm256_add_ps(
                                    _mm256_addsub_ps(
                                        _mm256_mul_ps(r0, _mm256_set1_ps(x().real())),
                                        _mm256_mul_ps(
                                            _mm256_permute_ps(r0, _MM_SHUFFLE(2, 3, 0, 1)),
                                            _mm256_set1_ps(x().imag())
                                        )
                                    ),
                                    _mm256_addsub_ps(
                                        _mm256_mul_ps(r1, _mm256_set1_ps(y().real())),
                                        _mm256_mul_ps(
                                            _mm256_permute_ps(r1, _MM_SHUFFLE(2, 3, 0, 1)),
                                            _mm256_set1_ps(y().imag())
                                        )
                                    )
                                ),
                                _mm256_addsub_ps(
                                    _mm256_mul_ps(r2, _mm256_set1_ps(z().real())),
                                    _mm256_mul_ps(
                                        _mm256_permute_ps(r2, _MM_SHUFFLE(2, 3, 0, 1)),
                                        _mm256_set1_ps(z().imag())
                                    )
                                )
                            )
                        );
                        return result;
                    }
#elif defined(__SSE3__)
                    const float* const lanes{reinterpret_cast<const float*>(m)};
                    // Two complex elements a register: (x, y) and (z, 0) of *this, and the halves
                    // of each stored vector.
                    const float* const v{reinterpret_cast<const float*>(data())};
                    float* const out_lanes{reinterpret_cast<float*>(out)};
                    if constexpr (kLeft) {
                        const __m128 v0{_mm_load_ps(v)};
                        const __m128 v1{_mm_load_ps(v + 4)};
                        for (std::size_t j{0}; j < 3; ++j) {
                            const __m128 c0{_mm_load_ps(lanes + 8 * j)};
                            const __m128 c1{_mm_load_ps(lanes + 8 * j + 4)};
                            const __m128 sum{_mm_add_ps(
                                _mm_addsub_ps(
                                    _mm_mul_ps(c0, _mm_moveldup_ps(v0)),
                                    _mm_mul_ps(
                                        _mm_shuffle_ps(c0, c0, _MM_SHUFFLE(2, 3, 0, 1)),
                                        _mm_movehdup_ps(v0)
                                    )
                                ),
                                _mm_addsub_ps(
                                    _mm_mul_ps(c1, _mm_moveldup_ps(v1)),
                                    _mm_mul_ps(
                                        _mm_shuffle_ps(c1, c1, _MM_SHUFFLE(2, 3, 0, 1)),
                                        _mm_movehdup_ps(v1)
                                    )
                                )
                            )};
                            _mm_storel_pi(
                                reinterpret_cast<__m64*>(out + j),
                                _mm_add_ps(sum, _mm_movehl_ps(sum, sum))
                            );
                        }
                    } else {
                        // Each half of the result: the halves of the rows, each times its
                        // coefficient (re, im) as (a re - b im, b re + a im).
                        for (std::size_t h{0}; h < 2; ++h) {
                            __m128 sum{_mm_setzero_ps()};
                            for (std::size_t k{0}; k < 3; ++k) {
                                const __m128 r{_mm_load_ps(lanes + 8 * k + 4 * h)};
                                sum = _mm_add_ps(
                                    sum, _mm_addsub_ps(
                                             _mm_mul_ps(r, _mm_set1_ps(v[2 * k])),
                                             _mm_mul_ps(
                                                 _mm_shuffle_ps(r, r, _MM_SHUFFLE(2, 3, 0, 1)),
                                                 _mm_set1_ps(v[2 * k + 1])
                                             )
                                         )
                                );
                            }
                            _mm_store_ps(out_lanes + 4 * h, sum);
                        }
                    }
                    return result;
#elif defined(__ARM_NEON)
                    const float* const lanes{reinterpret_cast<const float*>(m)};
                    // Two complex elements a register: (x, y) and (z, 0) of *this, and the halves
                    // of each stored vector; (a c, b c) plus (b d, a d) with its real lane negated.
                    const float* const v{reinterpret_cast<const float*>(data())};
                    float* const out_lanes{reinterpret_cast<float*>(out)};
                    const uint32x4_t sign{vreinterpretq_u32_u64(vdupq_n_u64(0x80000000ULL))};
                    if constexpr (kLeft) {
                        const float32x4_t v0{vld1q_f32(v)};
                        const float32x4_t v1{vld1q_f32(v + 4)};
                        for (std::size_t j{0}; j < 3; ++j) {
                            const float32x4_t c0{vld1q_f32(lanes + 8 * j)};
                            const float32x4_t c1{vld1q_f32(lanes + 8 * j + 4)};
                            const float32x4_t sum{vaddq_f32(
                                vaddq_f32(
                                    vmulq_f32(c0, vtrn1q_f32(v0, v0)),
                                    vreinterpretq_f32_u32(veorq_u32(
                                        vreinterpretq_u32_f32(
                                            vmulq_f32(vrev64q_f32(c0), vtrn2q_f32(v0, v0))
                                        ),
                                        sign
                                    ))
                                ),
                                vaddq_f32(
                                    vmulq_f32(c1, vtrn1q_f32(v1, v1)),
                                    vreinterpretq_f32_u32(veorq_u32(
                                        vreinterpretq_u32_f32(
                                            vmulq_f32(vrev64q_f32(c1), vtrn2q_f32(v1, v1))
                                        ),
                                        sign
                                    ))
                                )
                            )};
                            vst1_f32(
                                reinterpret_cast<float*>(out + j),
                                vadd_f32(vget_low_f32(sum), vget_high_f32(sum))
                            );
                        }
                    } else {
                        for (std::size_t h{0}; h < 2; ++h) {
                            float32x4_t sum{vdupq_n_f32(0.0F)};
                            for (std::size_t k{0}; k < 3; ++k) {
                                const float32x4_t r{vld1q_f32(lanes + 8 * k + 4 * h)};
                                sum = vaddq_f32(
                                    sum, vaddq_f32(
                                             vmulq_n_f32(r, v[2 * k]),
                                             vreinterpretq_f32_u32(veorq_u32(
                                                 vreinterpretq_u32_f32(
                                                     vmulq_n_f32(vrev64q_f32(r), v[2 * k + 1])
                                                 ),
                                                 sign
                                             ))
                                         )
                                );
                            }
                            vst1q_f32(out_lanes + 4 * h, sum);
                        }
                    }
                    return result;
#endif
                } else {
#if defined(__SSE3__) || defined(__AVX__)
                    const float* const lanes{reinterpret_cast<const float*>(m)};
                    // One complex element at a time: of a column against (x, y, z) under
                    // layout_left, of each row times its coefficient otherwise.
                    const float* const v{reinterpret_cast<const float*>(data())};
                    for (std::size_t j{0}; j < OuterSize; ++j) {
                        __m128 sum{_mm_setzero_ps()};
                        for (std::size_t k{0}; k < 3; ++k) {
                            const __m128 a{_mm_castsi128_ps(_mm_loadl_epi64(
                                reinterpret_cast<const __m128i*>(
                                    kLeft ? lanes + 6 * j + 2 * k : lanes + 2 * (k * OuterSize + j)
                                )
                            ))};
                            sum = _mm_add_ps(
                                sum, _mm_addsub_ps(
                                         _mm_mul_ps(a, _mm_set1_ps(v[2 * k])),
                                         _mm_mul_ps(
                                             _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 3, 0, 1)),
                                             _mm_set1_ps(v[2 * k + 1])
                                         )
                                     )
                            );
                        }
                        _mm_storel_pi(reinterpret_cast<__m64*>(out + j), sum);
                    }
                    return result;
#elif defined(__ARM_NEON)
                    const float* const lanes{reinterpret_cast<const float*>(m)};
                    // One complex element at a time: of a column against (x, y, z) under
                    // layout_left, of each row times its coefficient otherwise.
                    const float* const v{reinterpret_cast<const float*>(data())};
                    const uint32x2_t sign{vreinterpret_u32_u64(vdup_n_u64(0x80000000ULL))};
                    for (std::size_t j{0}; j < OuterSize; ++j) {
                        float32x2_t sum{vdup_n_f32(0.0F)};
                        for (std::size_t k{0}; k < 3; ++k) {
                            const float32x2_t a{vld1_f32(
                                kLeft ? lanes + 6 * j + 2 * k : lanes + 2 * (k * OuterSize + j)
                            )};
                            sum = vadd_f32(
                                sum, vadd_f32(
                                         vmul_n_f32(a, v[2 * k]),
                                         vreinterpret_f32_u32(veor_u32(
                                             vreinterpret_u32_f32(
                                                 vmul_n_f32(vrev64_f32(a), v[2 * k + 1])
                                             ),
                                             sign
                                         ))
                                     )
                            );
                        }
                        vst1_f32(reinterpret_cast<float*>(out + j), sum);
                    }
                    return result;
#endif
                }
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                if constexpr (OuterSize == 3) {
#if defined(__AVX__)
                    // Two complex elements a register, (x, y) and (z, 0) of *this and the halves
                    // of each stored vector, multiplied as (a re - b im, b re + a im); the result
                    // is stored in its two halves.
                    const double* const lanes{reinterpret_cast<const double*>(m)};
                    const double* const v{reinterpret_cast<const double*>(data())};
                    double* const out_lanes{reinterpret_cast<double*>(out)};
                    if constexpr (kLeft) {
                        // The products with stored column j, (x c0, y c1) plus (z c2, 0), fold to
                        // element j; the elements are gathered as (r0, r1 | r2, 0).
                        const __m256d v_lo{_mm256_load_pd(v)};
                        const __m256d v_hi{_mm256_load_pd(v + 4)};
                        const __m256d re_lo{_mm256_movedup_pd(v_lo)};
                        const __m256d im_lo{_mm256_permute_pd(v_lo, 0b1111)};
                        const __m256d re_hi{_mm256_movedup_pd(v_hi)};
                        const __m256d im_hi{_mm256_permute_pd(v_hi, 0b1111)};
                        const __m256d c0_lo{_mm256_load_pd(lanes)};
                        const __m256d c0_hi{_mm256_load_pd(lanes + 4)};
                        const __m256d c1_lo{_mm256_load_pd(lanes + 8)};
                        const __m256d c1_hi{_mm256_load_pd(lanes + 12)};
                        const __m256d c2_lo{_mm256_load_pd(lanes + 16)};
                        const __m256d c2_hi{_mm256_load_pd(lanes + 20)};
                        const __m256d s0{_mm256_add_pd(
                            _mm256_addsub_pd(
                                _mm256_mul_pd(c0_lo, re_lo),
                                _mm256_mul_pd(_mm256_permute_pd(c0_lo, 0b0101), im_lo)
                            ),
                            _mm256_addsub_pd(
                                _mm256_mul_pd(c0_hi, re_hi),
                                _mm256_mul_pd(_mm256_permute_pd(c0_hi, 0b0101), im_hi)
                            )
                        )};
                        const __m256d s1{_mm256_add_pd(
                            _mm256_addsub_pd(
                                _mm256_mul_pd(c1_lo, re_lo),
                                _mm256_mul_pd(_mm256_permute_pd(c1_lo, 0b0101), im_lo)
                            ),
                            _mm256_addsub_pd(
                                _mm256_mul_pd(c1_hi, re_hi),
                                _mm256_mul_pd(_mm256_permute_pd(c1_hi, 0b0101), im_hi)
                            )
                        )};
                        const __m256d s2{_mm256_add_pd(
                            _mm256_addsub_pd(
                                _mm256_mul_pd(c2_lo, re_lo),
                                _mm256_mul_pd(_mm256_permute_pd(c2_lo, 0b0101), im_lo)
                            ),
                            _mm256_addsub_pd(
                                _mm256_mul_pd(c2_hi, re_hi),
                                _mm256_mul_pd(_mm256_permute_pd(c2_hi, 0b0101), im_hi)
                            )
                        )};
                        _mm256_store_pd(
                            out_lanes,
                            _mm256_insertf128_pd(
                                _mm256_castpd128_pd256(_mm_add_pd(
                                    _mm256_castpd256_pd128(s0), _mm256_extractf128_pd(s0, 1)
                                )),
                                _mm_add_pd(
                                    _mm256_castpd256_pd128(s1), _mm256_extractf128_pd(s1, 1)
                                ),
                                1
                            )
                        );
                        _mm256_store_pd(
                            out_lanes + 4,
                            _mm256_insertf128_pd(
                                _mm256_castpd128_pd256(_mm_add_pd(
                                    _mm256_castpd256_pd128(s2), _mm256_extractf128_pd(s2, 1)
                                )),
                                _mm_setzero_pd(), 1
                            )
                        );
                    } else {
                        // The rows combined, each times its coefficient, half by half: their zero
                        // padding sums to the padding of the result.
                        const __m256d re0{_mm256_set1_pd(v[0])};
                        const __m256d im0{_mm256_set1_pd(v[1])};
                        const __m256d re1{_mm256_set1_pd(v[2])};
                        const __m256d im1{_mm256_set1_pd(v[3])};
                        const __m256d re2{_mm256_set1_pd(v[4])};
                        const __m256d im2{_mm256_set1_pd(v[5])};
                        for (std::size_t h{0}; h < 8; h += 4) {
                            const __m256d r0{_mm256_load_pd(lanes + h)};
                            const __m256d r1{_mm256_load_pd(lanes + 8 + h)};
                            const __m256d r2{_mm256_load_pd(lanes + 16 + h)};
                            _mm256_store_pd(
                                out_lanes + h,
                                _mm256_add_pd(
                                    _mm256_add_pd(
                                        _mm256_addsub_pd(
                                            _mm256_mul_pd(r0, re0),
                                            _mm256_mul_pd(_mm256_permute_pd(r0, 0b0101), im0)
                                        ),
                                        _mm256_addsub_pd(
                                            _mm256_mul_pd(r1, re1),
                                            _mm256_mul_pd(_mm256_permute_pd(r1, 0b0101), im1)
                                        )
                                    ),
                                    _mm256_addsub_pd(
                                        _mm256_mul_pd(r2, re2),
                                        _mm256_mul_pd(_mm256_permute_pd(r2, 0b0101), im2)
                                    )
                                )
                            );
                        }
                    }
                    return result;
#elif defined(__SSE3__)
                    // One complex element a 128-bit register, the stored vectors four elements
                    // apart.
                    const double* const lanes{reinterpret_cast<const double*>(m)};
                    const double* const v{reinterpret_cast<const double*>(data())};
                    for (std::size_t j{0}; j < OuterSize; ++j) {
                        __m128d sum{_mm_setzero_pd()};
                        for (std::size_t k{0}; k < 3; ++k) {
                            const __m128d a{
                                _mm_loadu_pd(lanes + 2 * (kLeft ? 4 * j + k : 4 * k + j))
                            };
                            sum = _mm_add_pd(
                                sum, _mm_addsub_pd(
                                         _mm_mul_pd(a, _mm_set1_pd(v[2 * k])),
                                         _mm_mul_pd(
                                             _mm_shuffle_pd(a, a, 0b01), _mm_set1_pd(v[2 * k + 1])
                                         )
                                     )
                            );
                        }
                        _mm_storeu_pd(reinterpret_cast<double*>(out + j), sum);
                    }
                    return result;
#elif defined(__ARM_NEON)
                    const double* const lanes{reinterpret_cast<const double*>(m)};
                    const double* const v{reinterpret_cast<const double*>(data())};
                    const uint64x2_t sign{vsetq_lane_u64(0x8000000000000000ULL, vdupq_n_u64(0), 0)};
                    for (std::size_t j{0}; j < OuterSize; ++j) {
                        float64x2_t sum{vdupq_n_f64(0.0)};
                        for (std::size_t k{0}; k < 3; ++k) {
                            const float64x2_t a{
                                vld1q_f64(lanes + 2 * (kLeft ? 4 * j + k : 4 * k + j))
                            };
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
                        vst1q_f64(reinterpret_cast<double*>(out + j), sum);
                    }
                    return result;
#endif
                } else {
                    // One complex element a 128-bit register, the columns or rows of the matrix
                    // three or OuterSize elements apart.
#if defined(__SSE3__) || defined(__AVX__)
                    const double* const lanes{reinterpret_cast<const double*>(m)};
                    const double* const v{reinterpret_cast<const double*>(data())};
                    for (std::size_t j{0}; j < OuterSize; ++j) {
                        __m128d sum{_mm_setzero_pd()};
                        for (std::size_t k{0}; k < 3; ++k) {
                            const __m128d a{
                                _mm_loadu_pd(lanes + 2 * (kLeft ? 3 * j + k : k * OuterSize + j))
                            };
                            sum = _mm_add_pd(
                                sum, _mm_addsub_pd(
                                         _mm_mul_pd(a, _mm_set1_pd(v[2 * k])),
                                         _mm_mul_pd(
                                             _mm_shuffle_pd(a, a, 0b01), _mm_set1_pd(v[2 * k + 1])
                                         )
                                     )
                            );
                        }
                        _mm_storeu_pd(reinterpret_cast<double*>(out + j), sum);
                    }
                    return result;
#elif defined(__ARM_NEON)
                    const double* const lanes{reinterpret_cast<const double*>(m)};
                    const double* const v{reinterpret_cast<const double*>(data())};
                    const uint64x2_t sign{vsetq_lane_u64(0x8000000000000000ULL, vdupq_n_u64(0), 0)};
                    for (std::size_t j{0}; j < OuterSize; ++j) {
                        float64x2_t sum{vdupq_n_f64(0.0)};
                        for (std::size_t k{0}; k < 3; ++k) {
                            const float64x2_t a{
                                vld1q_f64(lanes + 2 * (kLeft ? 3 * j + k : k * OuterSize + j))
                            };
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
                        vst1q_f64(reinterpret_cast<double*>(out + j), sum);
                    }
                    return result;
#endif
                }
            } else if constexpr (
                std::is_same_v<value_type, std::int32_t> ||
                std::is_same_v<value_type, std::uint32_t>
            ) {
#if defined(__SSE4_1__) || defined(__AVX__)
                const auto* const lanes{reinterpret_cast<const std::int32_t*>(m)};
                auto* const lanes_out{reinterpret_cast<std::int32_t*>(out)};
                const __m128i v{_mm_load_si128(reinterpret_cast<const __m128i*>(data()))};
                if constexpr (OuterSize == 3 && kLeft) {
                    // hadd folds the three column products and a zero fourth into (c0, c1, c2, 0).
                    const __m128i h01{_mm_hadd_epi32(
                        _mm_mullo_epi32(v, _mm_load_si128(reinterpret_cast<const __m128i*>(lanes))),
                        _mm_mullo_epi32(
                            v, _mm_load_si128(reinterpret_cast<const __m128i*>(lanes + 4))
                        )
                    )};
                    const __m128i h2{_mm_hadd_epi32(
                        _mm_mullo_epi32(
                            v, _mm_load_si128(reinterpret_cast<const __m128i*>(lanes + 8))
                        ),
                        _mm_setzero_si128()
                    )};
                    return std::bit_cast<result_type>(_mm_hadd_epi32(h01, h2));
                } else if constexpr (OuterSize == 3) {
                    // The rows combined: their zero padding sums to the padding of the result.
                    return std::bit_cast<result_type>(_mm_add_epi32(
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
                } else if constexpr (kLeft) {
                    // Four columns a step, shuffled into rows as under float, so that a whole
                    // register is stored. The other columns go one at a time, loaded without
                    // reading past them.
                    std::size_t j{0};
                    for (; j + 4 <= OuterSize; j += 4) {
                        const __m128 p0{_mm_castsi128_ps(
                            _mm_loadu_si128(reinterpret_cast<const __m128i*>(lanes + 3 * j))
                        )};
                        const __m128 p1{_mm_castsi128_ps(
                            _mm_loadu_si128(reinterpret_cast<const __m128i*>(lanes + 3 * j + 4))
                        )};
                        const __m128 p2{_mm_castsi128_ps(
                            _mm_loadu_si128(reinterpret_cast<const __m128i*>(lanes + 3 * j + 8))
                        )};
                        const __m128i r0{_mm_castps_si128(_mm_shuffle_ps(
                            p0, _mm_shuffle_ps(p1, p2, _MM_SHUFFLE(1, 1, 2, 2)),
                            _MM_SHUFFLE(2, 0, 3, 0)
                        ))};
                        const __m128i r1{_mm_castps_si128(_mm_shuffle_ps(
                            _mm_shuffle_ps(p0, p1, _MM_SHUFFLE(0, 0, 1, 1)),
                            _mm_shuffle_ps(p1, p2, _MM_SHUFFLE(2, 2, 3, 3)), _MM_SHUFFLE(2, 0, 2, 0)
                        ))};
                        const __m128i r2{_mm_castps_si128(_mm_shuffle_ps(
                            _mm_shuffle_ps(p0, p1, _MM_SHUFFLE(1, 1, 2, 2)), p2,
                            _MM_SHUFFLE(3, 0, 2, 0)
                        ))};
                        _mm_storeu_si128(
                            reinterpret_cast<__m128i*>(lanes_out + j),
                            _mm_add_epi32(
                                _mm_add_epi32(
                                    _mm_mullo_epi32(
                                        r0, _mm_shuffle_epi32(v, _MM_SHUFFLE(0, 0, 0, 0))
                                    ),
                                    _mm_mullo_epi32(
                                        r1, _mm_shuffle_epi32(v, _MM_SHUFFLE(1, 1, 1, 1))
                                    )
                                ),
                                _mm_mullo_epi32(r2, _mm_shuffle_epi32(v, _MM_SHUFFLE(2, 2, 2, 2)))
                            )
                        );
                    }
                    for (; j < OuterSize; ++j) {
                        const __m128i col{_mm_unpacklo_epi64(
                            _mm_loadl_epi64(reinterpret_cast<const __m128i*>(lanes + 3 * j)),
                            _mm_cvtsi32_si128(lanes[3 * j + 2])
                        )};
                        const __m128i prod{_mm_mullo_epi32(col, v)};
                        const __m128i pairs{
                            _mm_add_epi32(prod, _mm_shuffle_epi32(prod, _MM_SHUFFLE(1, 0, 3, 2)))
                        };
                        lanes_out[j] = _mm_cvtsi128_si32(
                            _mm_add_epi32(pairs, _mm_shuffle_epi32(pairs, _MM_SHUFFLE(2, 3, 0, 1)))
                        );
                    }
                } else {
                    const __m128i vx{_mm_shuffle_epi32(v, _MM_SHUFFLE(0, 0, 0, 0))};
                    const __m128i vy{_mm_shuffle_epi32(v, _MM_SHUFFLE(1, 1, 1, 1))};
                    const __m128i vz{_mm_shuffle_epi32(v, _MM_SHUFFLE(2, 2, 2, 2))};
                    std::size_t j{0};
                    for (; j + 4 <= OuterSize; j += 4) {
                        _mm_storeu_si128(
                            reinterpret_cast<__m128i*>(lanes_out + j),
                            _mm_add_epi32(
                                _mm_add_epi32(
                                    _mm_mullo_epi32(
                                        _mm_loadu_si128(
                                            reinterpret_cast<const __m128i*>(lanes + j)
                                        ),
                                        vx
                                    ),
                                    _mm_mullo_epi32(
                                        _mm_loadu_si128(
                                            reinterpret_cast<const __m128i*>(lanes + OuterSize + j)
                                        ),
                                        vy
                                    )
                                ),
                                _mm_mullo_epi32(
                                    _mm_loadu_si128(
                                        reinterpret_cast<const __m128i*>(lanes + 2 * OuterSize + j)
                                    ),
                                    vz
                                )
                            )
                        );
                    }
                    for (; j < OuterSize; ++j) {
                        out[j] = x() * m[j] + y() * m[OuterSize + j] + z() * m[2 * OuterSize + j];
                    }
                }
                return result;
#elif defined(__ARM_NEON)
                const auto* const lanes{reinterpret_cast<const std::int32_t*>(m)};
                auto* const lanes_out{reinterpret_cast<std::int32_t*>(out)};
                const int32x4_t v{vld1q_s32(reinterpret_cast<const std::int32_t*>(data()))};
                if constexpr (OuterSize == 3 && kLeft) {
                    // Pairwise adds fold the three column products and a zero fourth into
                    // (c0, c1, c2, 0).
                    const int32x4_t p01{vpaddq_s32(
                        vmulq_s32(v, vld1q_s32(lanes)), vmulq_s32(v, vld1q_s32(lanes + 4))
                    )};
                    const int32x4_t p2{
                        vpaddq_s32(vmulq_s32(v, vld1q_s32(lanes + 8)), vdupq_n_s32(0))
                    };
                    return std::bit_cast<result_type>(vpaddq_s32(p01, p2));
                } else if constexpr (OuterSize == 3) {
                    // The rows combined: their zero padding sums to the padding of the result.
                    return std::bit_cast<result_type>(vaddq_s32(
                        vaddq_s32(
                            vmulq_laneq_s32(vld1q_s32(lanes), v, 0),
                            vmulq_laneq_s32(vld1q_s32(lanes + 4), v, 1)
                        ),
                        vmulq_laneq_s32(vld1q_s32(lanes + 8), v, 2)
                    ));
                } else if constexpr (kLeft) {
                    // Four columns a step, deinterleaved into rows by vld3, so that a whole
                    // register is stored. The other columns go one at a time, loaded without
                    // reading past them.
                    std::size_t j{0};
                    for (; j + 4 <= OuterSize; j += 4) {
                        const int32x4x3_t rows{vld3q_s32(lanes + 3 * j)};
                        vst1q_s32(
                            lanes_out + j, vaddq_s32(
                                               vaddq_s32(
                                                   vmulq_laneq_s32(rows.val[0], v, 0),
                                                   vmulq_laneq_s32(rows.val[1], v, 1)
                                               ),
                                               vmulq_laneq_s32(rows.val[2], v, 2)
                                           )
                        );
                    }
                    for (; j < OuterSize; ++j) {
                        const int32x4_t col{vcombine_s32(
                            vld1_s32(lanes + 3 * j),
                            vld1_lane_s32(lanes + 3 * j + 2, vdup_n_s32(0), 0)
                        )};
                        lanes_out[j] = vaddvq_s32(vmulq_s32(col, v));
                    }
                } else {
                    std::size_t j{0};
                    for (; j + 4 <= OuterSize; j += 4) {
                        vst1q_s32(
                            lanes_out + j,
                            vaddq_s32(
                                vaddq_s32(
                                    vmulq_laneq_s32(vld1q_s32(lanes + j), v, 0),
                                    vmulq_laneq_s32(vld1q_s32(lanes + OuterSize + j), v, 1)
                                ),
                                vmulq_laneq_s32(vld1q_s32(lanes + 2 * OuterSize + j), v, 2)
                            )
                        );
                    }
                    for (; j < OuterSize; ++j) {
                        out[j] = x() * m[j] + y() * m[OuterSize + j] + z() * m[2 * OuterSize + j];
                    }
                }
                return result;
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int64_t> ||
                std::is_same_v<value_type, std::uint64_t>
            ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
                const __m256i v{_mm256_load_si256(reinterpret_cast<const __m256i*>(data()))};
                if constexpr (OuterSize == 3 && !kLeft) {
                    // The rows combined: their zero padding sums to the padding of the result.
                    _mm256_store_si256(
                        reinterpret_cast<__m256i*>(out),
                        _mm256_add_epi64(
                            _mm256_add_epi64(
                                _mm256_mullo_epi64(
                                    _mm256_load_si256(reinterpret_cast<const __m256i*>(m)),
                                    _mm256_set1_epi64x(static_cast<long long>(x()))
                                ),
                                _mm256_mullo_epi64(
                                    _mm256_load_si256(reinterpret_cast<const __m256i*>(m + 4)),
                                    _mm256_set1_epi64x(static_cast<long long>(y()))
                                )
                            ),
                            _mm256_mullo_epi64(
                                _mm256_load_si256(reinterpret_cast<const __m256i*>(m + 8)),
                                _mm256_set1_epi64x(static_cast<long long>(z()))
                            )
                        )
                    );
                    return result;
                } else if constexpr (kLeft) {
                    // The dot product with each column: the stored ones of a Mat33, or three
                    // elements loaded under a mask, so that no load reads past them.
                    for (std::size_t j{0}; j < OuterSize; ++j) {
                        const __m256i col{
                            OuterSize == 3
                                ? _mm256_load_si256(reinterpret_cast<const __m256i*>(m + 4 * j))
                                : _mm256_maskz_loadu_epi64(0b0111, m + 3 * j)
                        };
                        const __m256i prod{_mm256_mullo_epi64(col, v)};
                        const __m128i pairs{_mm_add_epi64(
                            _mm256_castsi256_si128(prod), _mm256_extracti128_si256(prod, 1)
                        )};
                        out[j] = static_cast<value_type>(_mm_cvtsi128_si64(
                            _mm_add_epi64(pairs, _mm_unpackhi_epi64(pairs, pairs))
                        ));
                    }
                } else {
                    for (std::size_t j{0}; j < OuterSize; ++j) {
                        out[j] = x() * m[j] + y() * m[OuterSize + j] + z() * m[2 * OuterSize + j];
                    }
                }
                return result;
#endif
            }
        }
        if constexpr (OuterSize == 3) {
            // With constant indices, so that the result of a Mat33 can stay in registers.
            result[0] = x() * obj[0, 0] + y() * obj[1, 0] + z() * obj[2, 0];
            result[1] = x() * obj[0, 1] + y() * obj[1, 1] + z() * obj[2, 1];
            result[2] = x() * obj[0, 2] + y() * obj[1, 2] + z() * obj[2, 2];
        } else {
            for (size_type j{0}; j < OuterSize; ++j) {
                const auto column{static_cast<Index>(j)};
                result[j] = x() * obj[0, column] + y() * obj[1, column] + z() * obj[2, column];
            }
        }
        return result;
    }
    // NOLINTEND(readability-function-size, readability-function-cognitive-complexity)

    [[using gnu: pure, always_inline, hot]]
    constexpr auto cross(const Vector& obj) const noexcept -> Vector {
        Vector result;
        if constexpr (
            std::is_same_v<value_type, std::int32_t> || std::is_same_v<value_type, std::uint32_t>
        ) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            const __m128i lhs{
                _mm_maskz_load_epi32(0b0111, reinterpret_cast<const std::int32_t*>(data()))
            };
            const __m128i rhs{
                _mm_maskz_load_epi32(0b0111, reinterpret_cast<const std::int32_t*>(obj.data()))
            };
            _mm_store_si128(
                reinterpret_cast<__m128i*>(result.data()),
                _mm_sub_epi32(
                    _mm_mullo_epi32(
                        _mm_shuffle_epi32(lhs, _MM_SHUFFLE(3, 0, 2, 1)),
                        _mm_shuffle_epi32(rhs, _MM_SHUFFLE(3, 1, 0, 2))
                    ),
                    _mm_mullo_epi32(
                        _mm_shuffle_epi32(lhs, _MM_SHUFFLE(3, 1, 0, 2)),
                        _mm_shuffle_epi32(rhs, _MM_SHUFFLE(3, 0, 2, 1))
                    )
                )
            );
#else
            result.x() = y() * obj.z() - z() * obj.y();
            result.y() = z() * obj.x() - x() * obj.z();
            result.z() = x() * obj.y() - y() * obj.x();
#endif
        } else if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
            const __m256i lhs{
                _mm256_maskz_load_epi64(0b0111, reinterpret_cast<const std::int64_t*>(data()))
            };
            const __m256i rhs{
                _mm256_maskz_load_epi64(0b0111, reinterpret_cast<const std::int64_t*>(obj.data()))
            };
            _mm256_store_si256(
                reinterpret_cast<__m256i*>(result.data()),
                _mm256_sub_epi64(
                    _mm256_mullo_epi64(
                        _mm256_permute4x64_epi64(lhs, _MM_SHUFFLE(3, 0, 2, 1)),
                        _mm256_permute4x64_epi64(rhs, _MM_SHUFFLE(3, 1, 0, 2))
                    ),
                    _mm256_mullo_epi64(
                        _mm256_permute4x64_epi64(lhs, _MM_SHUFFLE(3, 1, 0, 2)),
                        _mm256_permute4x64_epi64(rhs, _MM_SHUFFLE(3, 0, 2, 1))
                    )
                )
            );
#else
            result.x() = y() * obj.z() - z() * obj.y();
            result.y() = z() * obj.x() - x() * obj.z();
            result.z() = x() * obj.y() - y() * obj.x();
#endif
        } else if constexpr (std::is_same_v<value_type, float>) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            const __m128 lhs{_mm_maskz_load_ps(0b0111, reinterpret_cast<const float*>(data()))};
            const __m128 rhs{_mm_maskz_load_ps(0b0111, reinterpret_cast<const float*>(obj.data()))};
            _mm_store_ps(
                reinterpret_cast<float*>(result.data()),
                _mm_sub_ps(
                    _mm_mul_ps(
                        _mm_permute_ps(lhs, _MM_SHUFFLE(3, 0, 2, 1)),
                        _mm_permute_ps(rhs, _MM_SHUFFLE(3, 1, 0, 2))
                    ),
                    _mm_mul_ps(
                        _mm_permute_ps(lhs, _MM_SHUFFLE(3, 1, 0, 2)),
                        _mm_permute_ps(rhs, _MM_SHUFFLE(3, 0, 2, 1))
                    )
                )
            );
#else
            result.x() = y() * obj.z() - z() * obj.y();
            result.y() = z() * obj.x() - x() * obj.z();
            result.z() = x() * obj.y() - y() * obj.x();
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            const __m256d lhs{
                _mm256_maskz_load_pd(0b0111, reinterpret_cast<const double*>(data()))
            };
            const __m256d rhs{
                _mm256_maskz_load_pd(0b0111, reinterpret_cast<const double*>(obj.data()))
            };
            _mm256_store_pd(
                reinterpret_cast<double*>(result.data()),
                _mm256_sub_pd(
                    _mm256_mul_pd(
                        _mm256_permute4x64_pd(lhs, _MM_SHUFFLE(3, 0, 2, 1)),
                        _mm256_permute4x64_pd(rhs, _MM_SHUFFLE(3, 1, 0, 2))
                    ),
                    _mm256_mul_pd(
                        _mm256_permute4x64_pd(lhs, _MM_SHUFFLE(3, 1, 0, 2)),
                        _mm256_permute4x64_pd(rhs, _MM_SHUFFLE(3, 0, 2, 1))
                    )
                )
            );
#else
            result.x() = y() * obj.z() - z() * obj.y();
            result.y() = z() * obj.x() - x() * obj.z();
            result.z() = x() * obj.y() - y() * obj.x();
#endif
        } else {
            result.x() = y() * obj.z() - z() * obj.y();
            result.y() = z() * obj.x() - x() * obj.z();
            result.z() = x() * obj.y() - y() * obj.x();
        }
        return result;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto crossProj(const Vector& obj) const noexcept -> value_type {
        return cross(obj).euclidean();
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclidean() const noexcept -> detail::DenseNormTraitT<value_type> {
        return std::sqrt(euclideanSqr());
    }
    // The padding is zero, so that the four slots square to the three-element sum.
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
                // The squared magnitudes sum the squares of the real lanes.
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
                // The squared magnitudes sum the squares of the real lanes, the padding left out.
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
                const __m128d pairs{_mm_add_pd(
                    _mm_add_pd(_mm_mul_pd(v0, v0), _mm_mul_pd(v1, v1)), _mm_mul_pd(v2, v2)
                )};
                return _mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)));
#elif defined(__ARM_NEON)
                const double* const p{reinterpret_cast<const double*>(data())};
                const float64x2_t v0{vld1q_f64(p)};
                const float64x2_t v1{vld1q_f64(p + 2)};
                const float64x2_t v2{vld1q_f64(p + 4)};
                return vaddvq_f64(
                    vaddq_f64(vaddq_f64(vmulq_f64(v0, v0), vmulq_f64(v1, v1)), vmulq_f64(v2, v2))
                );
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
        if constexpr (ComplexArithmetic<value_type>) {
            return x().real() * x().real() + x().imag() * x().imag() + y().real() * y().real() +
                   y().imag() * y().imag() + z().real() * z().real() + z().imag() * z().imag();
        } else {
            return static_cast<norm_type>(x() * x() + y() * y() + z() * z());
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
                const __m128 prod{_mm_mul_ps(_mm_load_ps(data()), _mm_load_ps(obj.data()))};
                const __m128 pairs{_mm_add_ps(prod, _mm_movehl_ps(prod, prod))};
                return std::abs(_mm_cvtss_f32(
                           _mm_add_ss(pairs, _mm_shuffle_ps(pairs, pairs, _MM_SHUFFLE(1, 1, 1, 1)))
                       )) <= tol * tol;
#elif defined(__ARM_NEON)
                return std::abs(vaddvq_f32(vmulq_f32(vld1q_f32(data()), vld1q_f32(obj.data())))) <=
                       tol * tol;
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX__)
                const __m256d prod{
                    _mm256_mul_pd(_mm256_load_pd(data()), _mm256_load_pd(obj.data()))
                };
                const __m128d pairs{
                    _mm_add_pd(_mm256_castpd256_pd128(prod), _mm256_extractf128_pd(prod, 1))
                };
                return std::abs(_mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)))) <=
                       tol * tol;
#elif defined(__SSE2__) || defined(_M_X64)
                const __m128d pairs{_mm_add_pd(
                    _mm_mul_pd(_mm_load_pd(data()), _mm_load_pd(obj.data())),
                    _mm_mul_pd(_mm_load_pd(data() + 2), _mm_load_pd(obj.data() + 2))
                )};
                return std::abs(_mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)))) <=
                       tol * tol;
#elif defined(__ARM_NEON)
                return std::abs(vaddvq_f64(vaddq_f64(
                           vmulq_f64(vld1q_f64(data()), vld1q_f64(obj.data())),
                           vmulq_f64(vld1q_f64(data() + 2), vld1q_f64(obj.data() + 2))
                       ))) <= tol * tol;
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__AVX__)
                // *this conjugated by flipping the signs of its imaginary lanes.
                const __m256 lhs{_mm256_xor_ps(
                    _mm256_load_ps(reinterpret_cast<const float*>(data())),
                    _mm256_set_ps(-0.0F, 0.0F, -0.0F, 0.0F, -0.0F, 0.0F, -0.0F, 0.0F)
                )};
                const __m256 rhs{_mm256_load_ps(reinterpret_cast<const float*>(obj.data()))};
                const __m256 prod{_mm256_addsub_ps(
                    _mm256_mul_ps(lhs, _mm256_moveldup_ps(rhs)),
                    _mm256_mul_ps(
                        _mm256_permute_ps(lhs, _MM_SHUFFLE(2, 3, 0, 1)), _mm256_movehdup_ps(rhs)
                    )
                )};
                const __m128 halves{
                    _mm_add_ps(_mm256_castps256_ps128(prod), _mm256_extractf128_ps(prod, 1))
                };
                value_type inner;
                _mm_storel_pi(
                    reinterpret_cast<__m64*>(&inner),
                    _mm_add_ps(halves, _mm_movehl_ps(halves, halves))
                );
                return std::abs(inner) <= tol * tol;
#elif defined(__SSE3__)
                // *this conjugated by flipping the signs of its imaginary lanes.
                const float* const lhs{reinterpret_cast<const float*>(data())};
                const float* const rhs{reinterpret_cast<const float*>(obj.data())};
                const __m128 conj_sign{_mm_set_ps(-0.0F, 0.0F, -0.0F, 0.0F)};
                const __m128 l0{_mm_xor_ps(_mm_load_ps(lhs), conj_sign)};
                const __m128 l1{_mm_xor_ps(_mm_load_ps(lhs + 4), conj_sign)};
                const __m128 r0{_mm_load_ps(rhs)};
                const __m128 r1{_mm_load_ps(rhs + 4)};
                const __m128 sum{_mm_add_ps(
                    _mm_addsub_ps(
                        _mm_mul_ps(l0, _mm_moveldup_ps(r0)),
                        _mm_mul_ps(
                            _mm_shuffle_ps(l0, l0, _MM_SHUFFLE(2, 3, 0, 1)), _mm_movehdup_ps(r0)
                        )
                    ),
                    _mm_addsub_ps(
                        _mm_mul_ps(l1, _mm_moveldup_ps(r1)),
                        _mm_mul_ps(
                            _mm_shuffle_ps(l1, l1, _MM_SHUFFLE(2, 3, 0, 1)), _mm_movehdup_ps(r1)
                        )
                    )
                )};
                value_type inner;
                _mm_storel_pi(
                    reinterpret_cast<__m64*>(&inner), _mm_add_ps(sum, _mm_movehl_ps(sum, sum))
                );
                return std::abs(inner) <= tol * tol;
#elif defined(__ARM_NEON)
                // *this conjugated by flipping the signs of its imaginary lanes; then (a c, b c)
                // plus (b d, a d) with its real lane negated, per element.
                const float* const lhs{reinterpret_cast<const float*>(data())};
                const float* const rhs{reinterpret_cast<const float*>(obj.data())};
                const uint32x4_t conj_sign{
                    vreinterpretq_u32_u64(vdupq_n_u64(0x8000000000000000ULL))
                };
                const uint32x4_t sign{vreinterpretq_u32_u64(vdupq_n_u64(0x80000000ULL))};
                const float32x4_t l0{vreinterpretq_f32_u32(
                    veorq_u32(vreinterpretq_u32_f32(vld1q_f32(lhs)), conj_sign)
                )};
                const float32x4_t l1{vreinterpretq_f32_u32(
                    veorq_u32(vreinterpretq_u32_f32(vld1q_f32(lhs + 4)), conj_sign)
                )};
                const float32x4_t r0{vld1q_f32(rhs)};
                const float32x4_t r1{vld1q_f32(rhs + 4)};
                const float32x4_t sum{vaddq_f32(
                    vaddq_f32(
                        vmulq_f32(l0, vtrn1q_f32(r0, r0)),
                        vreinterpretq_f32_u32(veorq_u32(
                            vreinterpretq_u32_f32(vmulq_f32(vrev64q_f32(l0), vtrn2q_f32(r0, r0))),
                            sign
                        ))
                    ),
                    vaddq_f32(
                        vmulq_f32(l1, vtrn1q_f32(r1, r1)),
                        vreinterpretq_f32_u32(veorq_u32(
                            vreinterpretq_u32_f32(vmulq_f32(vrev64q_f32(l1), vtrn2q_f32(r1, r1))),
                            sign
                        ))
                    )
                )};
                value_type inner;
                vst1_f32(
                    reinterpret_cast<float*>(&inner),
                    vadd_f32(vget_low_f32(sum), vget_high_f32(sum))
                );
                return std::abs(inner) <= tol * tol;
#endif
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX__)
                const double* const lhs{reinterpret_cast<const double*>(data())};
                const double* const rhs{reinterpret_cast<const double*>(obj.data())};
                // *this conjugated by flipping the signs of its imaginary lanes.
                const __m256d conj_sign{_mm256_set_pd(-0.0, 0.0, -0.0, 0.0)};
                const __m256d l0{_mm256_xor_pd(_mm256_load_pd(lhs), conj_sign)};
                const __m256d l1{_mm256_xor_pd(_mm256_load_pd(lhs + 4), conj_sign)};
                const __m256d r0{_mm256_load_pd(rhs)};
                const __m256d r1{_mm256_load_pd(rhs + 4)};
                const __m256d sum{_mm256_add_pd(
                    _mm256_addsub_pd(
                        _mm256_mul_pd(l0, _mm256_movedup_pd(r0)),
                        _mm256_mul_pd(_mm256_permute_pd(l0, 0b0101), _mm256_permute_pd(r0, 0b1111))
                    ),
                    _mm256_addsub_pd(
                        _mm256_mul_pd(l1, _mm256_movedup_pd(r1)),
                        _mm256_mul_pd(_mm256_permute_pd(l1, 0b0101), _mm256_permute_pd(r1, 0b1111))
                    )
                )};
                value_type inner;
                _mm_storeu_pd(
                    reinterpret_cast<double*>(&inner),
                    _mm_add_pd(_mm256_castpd256_pd128(sum), _mm256_extractf128_pd(sum, 1))
                );
                return std::abs(inner) <= tol * tol;
#elif defined(__SSE3__)
                const double* const lhs{reinterpret_cast<const double*>(data())};
                const double* const rhs{reinterpret_cast<const double*>(obj.data())};
                // *this conjugated by flipping the signs of its imaginary lanes; the padding left
                // out.
                const __m128d conj_sign{_mm_set_pd(-0.0, 0.0)};
                __m128d sum{_mm_setzero_pd()};
                for (std::size_t k{0}; k < 3; ++k) {
                    const __m128d l{_mm_xor_pd(_mm_load_pd(lhs + 2 * k), conj_sign)};
                    const __m128d r{_mm_load_pd(rhs + 2 * k)};
                    sum = _mm_add_pd(
                        sum, _mm_addsub_pd(
                                 _mm_mul_pd(l, _mm_movedup_pd(r)),
                                 _mm_mul_pd(_mm_shuffle_pd(l, l, 0b01), _mm_unpackhi_pd(r, r))
                             )
                    );
                }
                value_type inner;
                _mm_storeu_pd(reinterpret_cast<double*>(&inner), sum);
                return std::abs(inner) <= tol * tol;
#elif defined(__ARM_NEON)
                const double* const lhs{reinterpret_cast<const double*>(data())};
                const double* const rhs{reinterpret_cast<const double*>(obj.data())};
                // *this conjugated by flipping the signs of its imaginary lanes; the padding left
                // out.
                const uint64x2_t conj_sign{
                    vsetq_lane_u64(0x8000000000000000ULL, vdupq_n_u64(0), 1)
                };
                const uint64x2_t sign{vsetq_lane_u64(0x8000000000000000ULL, vdupq_n_u64(0), 0)};
                float64x2_t sum{vdupq_n_f64(0.0)};
                for (std::size_t k{0}; k < 3; ++k) {
                    const float64x2_t l{vreinterpretq_f64_u64(
                        veorq_u64(vreinterpretq_u64_f64(vld1q_f64(lhs + 2 * k)), conj_sign)
                    )};
                    const float64x2_t r{vld1q_f64(rhs + 2 * k)};
                    sum = vaddq_f64(
                        sum,
                        vaddq_f64(
                            vmulq_laneq_f64(l, r, 0),
                            vreinterpretq_f64_u64(veorq_u64(
                                vreinterpretq_u64_f64(vmulq_laneq_f64(vextq_f64(l, l, 1), r, 1)),
                                sign
                            ))
                        )
                    );
                }
                value_type inner;
                vst1q_f64(reinterpret_cast<double*>(&inner), sum);
                return std::abs(inner) <= tol * tol;
#endif
            } else if constexpr (
                std::is_same_v<value_type, std::int32_t> ||
                std::is_same_v<value_type, std::uint32_t>
            ) {
#if defined(__SSE4_1__) || defined(__AVX__)
                const __m128i prod{_mm_mullo_epi32(
                    _mm_load_si128(reinterpret_cast<const __m128i*>(data())),
                    _mm_load_si128(reinterpret_cast<const __m128i*>(obj.data()))
                )};
                const __m128i pairs{
                    _mm_add_epi32(prod, _mm_shuffle_epi32(prod, _MM_SHUFFLE(1, 0, 3, 2)))
                };
                const auto inner{static_cast<value_type>(_mm_cvtsi128_si32(
                    _mm_add_epi32(pairs, _mm_shuffle_epi32(pairs, _MM_SHUFFLE(2, 3, 0, 1)))
                ))};
                if constexpr (std::is_unsigned_v<value_type>) {
                    return inner <= tol * tol;
                } else {
                    return std::abs(inner) <= tol * tol;
                }
#elif defined(__ARM_NEON)
                const auto inner{static_cast<value_type>(vaddvq_s32(vmulq_s32(
                    vld1q_s32(reinterpret_cast<const std::int32_t*>(data())),
                    vld1q_s32(reinterpret_cast<const std::int32_t*>(obj.data()))
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
                const __m256i prod{_mm256_mullo_epi64(
                    _mm256_load_si256(reinterpret_cast<const __m256i*>(data())),
                    _mm256_load_si256(reinterpret_cast<const __m256i*>(obj.data()))
                )};
                const __m128i pairs{
                    _mm_add_epi64(_mm256_castsi256_si128(prod), _mm256_extracti128_si256(prod, 1))
                };
                const auto inner{static_cast<value_type>(
                    _mm_cvtsi128_si64(_mm_add_epi64(pairs, _mm_unpackhi_epi64(pairs, pairs)))
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
            return std::abs(
                       std::conj(x()) * obj.x() + std::conj(y()) * obj.y() +
                       std::conj(z()) * obj.z()
                   ) <= tol * tol;
        } else if constexpr (std::is_unsigned_v<value_type>) {
            return static_cast<value_type>(x() * obj.x() + y() * obj.y() + z() * obj.z()) <=
                   tol * tol;
        } else {
            return std::abs(x() * obj.x() + y() * obj.y() + z() * obj.z()) <= tol * tol;
        }
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    // x, y, z and the padding, which stays zero; serialization writes all four.
    std::array<value_type, 4> m_data;
};

template <ScalarArithmetic Scalar, std::integral Index = std::size_t>
using Vec3 = Vector<Scalar, Index, 3>;

using Vec3s = Vec3<float>;
using Vec3d = Vec3<double>;
using Vec3c = Vec3<std::complex<float>>;
using Vec3z = Vec3<std::complex<double>>;

template <typename T>
struct isVec3Arithmetic : std::false_type {};

template <ScalarArithmetic T, std::integral Index, Allocatory Alloc>
struct isVec3Arithmetic<Vector<T, Index, 3, Alloc>> : std::true_type {};

template <typename T>
inline constexpr bool isVec3ArithmeticV = isVec3Arithmetic<T>::value;

template <typename T>
concept Vec3Arithmetic = isVec3ArithmeticV<T>;

template <
    std::input_iterator InputIt1, std::input_iterator InputIt2, std::input_iterator InputIt3,
    std::weakly_incrementable OutputIt>
    requires std::floating_point<typename std::iterator_traits<InputIt1>::value_type> &&
             std::floating_point<typename std::iterator_traits<InputIt2>::value_type> &&
             std::floating_point<typename std::iterator_traits<InputIt3>::value_type> &&
             Vec3Arithmetic<typename std::iterator_traits<OutputIt>::value_type>
[[using gnu: always_inline, hot]]
inline auto squeeze(
    InputIt1 first1, std::sentinel_for<InputIt1> auto last1, InputIt2 first2,
    std::sentinel_for<InputIt2> auto last2, InputIt3 first3, std::sentinel_for<InputIt3> auto last3,
    OutputIt result
) noexcept -> ::basjoo::common::InInInOutResult<InputIt1, InputIt2, InputIt3, OutputIt> {
    for (; first1 != last1 && first2 != last2 && first3 != last3;
         ++first1, ++first2, ++first3, ++result) {
        *result = typename std::iterator_traits<OutputIt>::value_type{*first1, *first2, *first3};
    }
    return {first1, first2, first3, result};
}

template <
    std::ranges::input_range InputRange1, std::ranges::input_range InputRange2,
    std::ranges::input_range InputRange3, std::weakly_incrementable OutputIt>
    requires std::floating_point<typename std::ranges::range_value_t<InputRange1>> &&
             std::floating_point<typename std::ranges::range_value_t<InputRange2>> &&
             std::floating_point<typename std::ranges::range_value_t<InputRange3>> &&
             Vec3Arithmetic<typename std::iterator_traits<OutputIt>::value_type>
[[using gnu: always_inline, hot]]
inline auto squeeze(InputRange1&& xs, InputRange2&& ys, InputRange3&& zs, OutputIt result) noexcept
    -> ::basjoo::common::InInInOutResult<
        std::ranges::iterator_t<InputRange1>, std::ranges::iterator_t<InputRange2>,
        std::ranges::iterator_t<InputRange3>, OutputIt> {
    return squeeze(
        std::ranges::begin(xs), std::ranges::end(xs), std::ranges::begin(ys), std::ranges::end(ys),
        std::ranges::begin(zs), std::ranges::end(zs), result
    );
}

} // namespace basjoo::math

// NOLINTBEGIN(cert-dcl58-cpp)

namespace std {

template <::basjoo::math::Vec3Arithmetic V>
    requires std::floating_point<typename V::value_type>
[[using gnu: pure, always_inline]]
inline constexpr auto hypot(const V& obj) noexcept
    -> ::basjoo::math::detail::DenseNormTraitT<typename V::value_type> {
    return std::hypot(obj.x(), obj.y(), obj.z());
}

} // namespace std

// NOLINTEND(cert-dcl58-cpp)
