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
#include <cmath>
#include <complex>
#include <concepts>
#include <format>
#include <ostream>
#include <ranges>

#if defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>
#elif defined(__aarch64__) || defined(_M_ARM64)
#include <arm_neon.h>
#endif

#include "zpp_bits.h"

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_traits.hpp"
#include "basjoo/math/dense/detail/dense_norm_trait.hpp"
#include "basjoo/math/type_traits.hpp"

namespace basjoo::math {

template <ScalarArithmetic T>
class alignas(2 * sizeof(T)) Vector<T, 2> final {
  public:
    using value_type = typename DenseTraits<Vector>::value_type;
    using reference = typename DenseTraits<Vector>::reference;
    using const_reference = typename DenseTraits<Vector>::const_reference;
    using pointer = typename DenseTraits<Vector>::pointer;
    using const_pointer = typename DenseTraits<Vector>::const_pointer;
    using size_type = typename DenseTraits<Vector>::size_type;
    using difference_type = typename DenseTraits<Vector>::difference_type;
    using allocator_type = typename DenseTraits<Vector>::allocator_type;

    static constexpr size_type kSize = 2;

    [[using gnu: always_inline]]
    constexpr Vector(value_type cv) noexcept
        : x{cv}, y{cv} {}
    [[using gnu: always_inline]]
    constexpr Vector(value_type cx, value_type cy) noexcept
        : x{cx}, y{cy} {}
    [[using gnu: always_inline]]
    constexpr Vector(const Vector& other) noexcept = default;
    [[using gnu: always_inline]]
    constexpr auto operator=(const Vector& other) noexcept -> Vector& = default;
    [[using gnu: always_inline]]
    constexpr Vector(Vector&& other) noexcept = default;
    [[using gnu: always_inline]]
    constexpr auto operator=(Vector&& other) noexcept -> Vector& = default;
    ~Vector() noexcept = default;

    [[using gnu: always_inline]]
    constexpr auto get_allocator() const noexcept -> allocator_type {
        return allocator_type{};
    }

    [[using gnu: always_inline]]
    explicit Vector([[maybe_unused]] const allocator_type& alloc = {}) noexcept {}

    template <ScalarArithmetic U>
    [[using gnu: always_inline]]
    constexpr explicit Vector(const std::pair<U, U>& other) noexcept
        : x{static_cast<value_type>(other.first)}, y{static_cast<value_type>(other.second)} {}

    template <ScalarArithmetic U>
    [[using gnu: always_inline]]
    constexpr explicit Vector(const Vector<U, 2>& other) noexcept
        : x{static_cast<value_type>(other.x)}, y{static_cast<value_type>(other.y)} {}

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto size() noexcept -> size_type {
        return kSize;
    }
    [[using gnu: const, always_inline, leaf]]
    static constexpr auto stride() noexcept -> size_type {
        return 1;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() noexcept -> pointer {
        return &x;
    }
    [[using gnu: pure, always_inline]]
    constexpr auto data() const noexcept -> const_pointer {
        return &x;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator[](size_t i) noexcept -> reference {
        return (&x)[i];
    }
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator[](size_t i) const noexcept -> const_reference {
        return (&x)[i];
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto coeff(size_type i) const noexcept(!BASJOO_CHECK_PARAMS) -> value_type {
#if BASJOO_CHECK_PARAMS == 1
        if (i >= kSize) [[unlikely]] {
            throw std::out_of_range("Vector index out of range");
        }
#endif
        return (&x)[i];
    }

    [[using gnu: always_inline, hot]]
    constexpr auto updateCoeff(size_type i, const_reference value) noexcept(!BASJOO_CHECK_PARAMS)
        -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (i >= kSize) [[unlikely]] {
            throw std::out_of_range("Vector index out of range");
        }
#endif
        (&x)[i] = value;
        return;
    }

    [[using gnu: always_inline, leaf, hot]]
    constexpr auto operator+=(const Vector& obj) noexcept -> Vector& {
        if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__SSE2__)
            _mm_store_si128(
                reinterpret_cast<__m128i*>(&x),
                _mm_add_epi64(
                    _mm_load_si128(reinterpret_cast<const __m128i*>(&x)),
                    _mm_load_si128(reinterpret_cast<const __m128i*>(&obj.x))
                )
            );
#elif defined(__ARM_NEON)
            vst1q_s64(
                reinterpret_cast<std::int64_t*>(&x),
                vaddq_s64(
                    vld1q_s64(reinterpret_cast<const std::int64_t*>(&x)),
                    vld1q_s64(reinterpret_cast<const std::int64_t*>(&obj.x))
                )
            );
#else
            x += obj.x;
            y += obj.y;
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__SSE2__)
            _mm_store_pd(
                reinterpret_cast<double*>(&x),
                _mm_add_pd(
                    _mm_load_pd(reinterpret_cast<const double*>(&x)),
                    _mm_load_pd(reinterpret_cast<const double*>(&obj.x))
                )
            );
#elif defined(__ARM_NEON)
            vst1q_f64(
                reinterpret_cast<double*>(&x),
                vaddq_f64(
                    vld1q_f64(reinterpret_cast<const double*>(&x)),
                    vld1q_f64(reinterpret_cast<const double*>(&obj.x))
                )
            );
#else
            x += obj.x;
            y += obj.y;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__SSE__)
            _mm_store_ps(
                reinterpret_cast<float*>(&x),
                _mm_add_ps(
                    _mm_load_ps(reinterpret_cast<const float*>(&x)),
                    _mm_load_ps(reinterpret_cast<const float*>(&obj.x))
                )
            );
#elif defined(__ARM_NEON)
            vst1q_f32(
                reinterpret_cast<float*>(&x), vaddq_f32(
                                                  vld1q_f32(reinterpret_cast<const float*>(&x)),
                                                  vld1q_f32(reinterpret_cast<const float*>(&obj.x))
                                              )
            );
#else
            x += obj.x;
            y += obj.y;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX__)
            _mm256_store_pd(
                reinterpret_cast<double*>(&x),
                _mm256_add_pd(
                    _mm256_load_pd(reinterpret_cast<const double*>(&x)),
                    _mm256_load_pd(reinterpret_cast<const double*>(&obj.x))
                )
            );
#elif defined(__ARM_NEON)
            vst1q_f64(
                reinterpret_cast<double*>(&x),
                vaddq_f64(
                    vld1q_f64(reinterpret_cast<const double*>(&x)),
                    vld1q_f64(reinterpret_cast<const double*>(&obj.x))
                )
            );
            vst1q_f64(
                reinterpret_cast<double*>(&y),
                vaddq_f64(
                    vld1q_f64(reinterpret_cast<const double*>(&y)),
                    vld1q_f64(reinterpret_cast<const double*>(&obj.y))
                )
            );
#else
            x += obj.x;
            y += obj.y;
#endif
        } else {
            x += obj.x;
            y += obj.y;
        }
        return *this;
    }
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator+(const Vector& obj) const& noexcept -> Vector {
        Vector result{*this};
        result += obj;
        return result;
    }
    [[using gnu: always_inline, hot]]
    constexpr auto operator+(Vector&& obj) const& noexcept -> Vector&& {
        obj += (*this);
        return std::move(obj);
    }
    [[using gnu: always_inline, hot]]
    constexpr auto operator+(const Vector& obj) && noexcept -> Vector&& {
        operator+=(obj);
        return std::move(*this);
    }
    [[using gnu: always_inline, hot]]
    constexpr auto operator+(Vector&& obj) && noexcept -> Vector&& {
        operator+=(obj);
        return std::move(*this);
    }

    [[using gnu: always_inline, leaf, hot]]
    constexpr auto operator-=(const Vector& obj) noexcept -> Vector& {
        if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__SSE2__)
            _mm_store_si128(
                reinterpret_cast<__m128i*>(&x),
                _mm_sub_epi64(
                    _mm_load_si128(reinterpret_cast<const __m128i*>(&x)),
                    _mm_load_si128(reinterpret_cast<const __m128i*>(&obj.x))
                )
            );
#elif defined(__ARM_NEON)
            vst1q_s64(
                reinterpret_cast<std::int64_t*>(&x),
                vsubq_s64(
                    vld1q_s64(reinterpret_cast<const std::int64_t*>(&x)),
                    vld1q_s64(reinterpret_cast<const std::int64_t*>(&obj.x))
                )
            );
#else
            x -= obj.x;
            y -= obj.y;
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__SSE2__)
            _mm_store_pd(
                reinterpret_cast<double*>(&x),
                _mm_sub_pd(
                    _mm_load_pd(reinterpret_cast<const double*>(&x)),
                    _mm_load_pd(reinterpret_cast<const double*>(&obj.x))
                )
            );
#elif defined(__ARM_NEON)
            vst1q_f64(
                reinterpret_cast<double*>(&x),
                vsubq_f64(
                    vld1q_f64(reinterpret_cast<const double*>(&x)),
                    vld1q_f64(reinterpret_cast<const double*>(&obj.x))
                )
            );
#else
            x -= obj.x;
            y -= obj.y;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__SSE__)
            _mm_store_ps(
                reinterpret_cast<float*>(&x),
                _mm_sub_ps(
                    _mm_load_ps(reinterpret_cast<const float*>(&x)),
                    _mm_load_ps(reinterpret_cast<const float*>(&obj.x))
                )
            );
#elif defined(__ARM_NEON)
            vst1q_f32(
                reinterpret_cast<float*>(&x), vsubq_f32(
                                                  vld1q_f32(reinterpret_cast<const float*>(&x)),
                                                  vld1q_f32(reinterpret_cast<const float*>(&obj.x))
                                              )
            );
#else
            x -= obj.x;
            y -= obj.y;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX__)
            _mm256_store_pd(
                reinterpret_cast<double*>(&x),
                _mm256_sub_pd(
                    _mm256_load_pd(reinterpret_cast<const double*>(&x)),
                    _mm256_load_pd(reinterpret_cast<const double*>(&obj.x))
                )
            );
#elif defined(__ARM_NEON)
            vst1q_f64(
                reinterpret_cast<double*>(&x),
                vsubq_f64(
                    vld1q_f64(reinterpret_cast<const double*>(&x)),
                    vld1q_f64(reinterpret_cast<const double*>(&obj.x))
                )
            );
            vst1q_f64(
                reinterpret_cast<double*>(&y),
                vsubq_f64(
                    vld1q_f64(reinterpret_cast<const double*>(&y)),
                    vld1q_f64(reinterpret_cast<const double*>(&obj.y))
                )
            );
#else
            x -= obj.x;
            y -= obj.y;
#endif
        } else {
            x -= obj.x;
            y -= obj.y;
        }
        return *this;
    }
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator-(const Vector& obj) const& noexcept -> Vector {
        Vector result{*this};
        result -= obj;
        return result;
    }
    [[using gnu: always_inline, hot]]
    constexpr auto operator-(Vector&& obj) const& noexcept -> Vector&& {
        if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__SSE2__)
            _mm_store_si128(
                reinterpret_cast<__m128i*>(&obj.x),
                _mm_sub_epi64(
                    _mm_load_si128(reinterpret_cast<__m128i*>(&x)),
                    _mm_load_si128(reinterpret_cast<__m128i*>(&obj.x))
                )
            );
#elif defined(__ARM_NEON)
            vst1q_s64(
                reinterpret_cast<std::int64_t*>(&obj.x),
                vsubq_s64(
                    vld1q_s64(reinterpret_cast<const std::int64_t*>(&x)),
                    vld1q_s64(reinterpret_cast<const std::int64_t*>(&obj.x))
                )
            );
#else
            obj.x = x - obj.x;
            obj.y = y - obj.y;
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__SSE2__)
            _mm_store_pd(
                reinterpret_cast<double*>(&obj.x),
                _mm_sub_pd(
                    _mm_load_pd(reinterpret_cast<const double*>(&x)),
                    _mm_load_pd(reinterpret_cast<const double*>(&obj.x))
                )
            );
#elif defined(__ARM_NEON)
            vst1q_f64(
                reinterpret_cast<double*>(&obj.x),
                vsubq_f64(
                    vld1q_f64(reinterpret_cast<const double*>(&x)),
                    vld1q_f64(reinterpret_cast<const double*>(&obj.x))
                )
            );
#else
            obj.x = x - obj.x;
            obj.y = y - obj.y;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__SSE__)
            _mm_store_ps(
                reinterpret_cast<float*>(&obj.x),
                _mm_sub_ps(
                    _mm_load_ps(reinterpret_cast<const float*>(&x)),
                    _mm_load_ps(reinterpret_cast<const float*>(&obj.x))
                )
            );
#elif defined(__ARM_NEON)
            vst1q_f32(
                reinterpret_cast<float*>(&obj.x),
                vsubq_f32(
                    vld1q_f32(reinterpret_cast<const float*>(&x)),
                    vld1q_f32(reinterpret_cast<const float*>(&obj.x))
                )
            );
#else
            obj.x = x - obj.x;
            obj.y = y - obj.y;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX__)
            _mm256_store_pd(
                reinterpret_cast<double*>(&obj.x),
                _mm256_sub_pd(
                    _mm256_load_pd(reinterpret_cast<const double*>(&x)),
                    _mm256_load_pd(reinterpret_cast<const double*>(&obj.x))
                )
            );
#elif defined(__ARM_NEON)
            vst1q_f64(
                reinterpret_cast<double*>(&obj.x),
                vsubq_f64(
                    vld1q_f64(reinterpret_cast<const double*>(&x)),
                    vld1q_f64(reinterpret_cast<const double*>(&obj.x))
                )
            );
            vst1q_f64(
                reinterpret_cast<double*>(&obj.y),
                vsubq_f64(
                    vld1q_f64(reinterpret_cast<const double*>(&y)),
                    vld1q_f64(reinterpret_cast<const double*>(&obj.y))
                )
            );
#else
            obj.x = x - obj.x;
            obj.y = y - obj.y;
#endif
        } else {
            obj.x = x - obj.x;
            obj.y = y - obj.y;
        }
        return std::move(obj);
    }
    [[using gnu: always_inline, hot]]
    constexpr auto operator-(const Vector& obj) && noexcept -> Vector&& {
        operator-=(obj);
        return std::move(*this);
    }
    [[using gnu: always_inline, hot]]
    constexpr auto operator-(Vector&& obj) && noexcept -> Vector&& {
        operator-=(obj);
        return std::move(*this);
    }

    [[using gnu: always_inline, leaf, hot]]
    constexpr auto operator*=(const ScalarArithmetic auto& fac) noexcept -> Vector& {
        const auto alpha{static_cast<value_type>(fac)};
        if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
            _mm_store_si128(
                reinterpret_cast<__m128i*>(&x),
                _mm_mullo_epi64(
                    _mm_load_si128(reinterpret_cast<__m128i*>(&x)), _mm_set1_epi64x(alpha)
                )
            );
#else
            x *= alpha;
            y *= alpha;
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__SSE2__)
            _mm_store_pd(
                reinterpret_cast<double*>(&x),
                _mm_mul_pd(_mm_load_pd(reinterpret_cast<const double*>(&x)), _mm_set1_pd(alpha))
            );
#elif defined(__ARM_NEON)
            vst1q_f64(
                reinterpret_cast<double*>(&x),
                vmulq_f64(vld1q_f64(reinterpret_cast<const double*>(&x)), vdupq_n_f64(alpha))
            );
#else
            x *= alpha;
            y *= alpha;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__AVX__)
            const __m128 lhs{_mm_load_ps(reinterpret_cast<const float*>(&x))};
            _mm_store_ps(
                reinterpret_cast<float*>(&x),
                _mm_addsub_ps(
                    _mm_mul_ps(lhs, _mm_set1_ps(alpha.real())),
                    _mm_mul_ps(
                        _mm_permute_ps(lhs, _MM_SHUFFLE(2, 3, 0, 1)), _mm_set1_ps(alpha.imag())
                    )
                )
            );
#elif defined(__ARM_NEON)
            static constexpr std::uint32_t kAddsubMaskF[4]{0x80000000U, 0U, 0x80000000U, 0U};
            const float32x4_t lhs{vld1q_f32(reinterpret_cast<const float*>(&x))};
            const float32x4_t prod_re{vmulq_f32(lhs, vdupq_n_f32(alpha.real()))};
            const float32x4_t prod_im{vmulq_f32(vrev64q_f32(lhs), vdupq_n_f32(alpha.imag()))};
            const float32x4_t signed_im{vreinterpretq_f32_u32(
                veorq_u32(vreinterpretq_u32_f32(prod_im), vld1q_u32(kAddsubMaskF))
            )};
            vst1q_f32(reinterpret_cast<float*>(&x), vaddq_f32(prod_re, signed_im));
#else
            x *= alpha;
            y *= alpha;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX2__)
            const __m256d lhs{_mm256_load_pd(reinterpret_cast<const double*>(&x))};
            _mm256_store_pd(
                reinterpret_cast<double*>(&x),
                _mm256_addsub_pd(
                    _mm256_mul_pd(lhs, _mm256_set1_pd(alpha.real())),
                    _mm256_mul_pd(
                        _mm256_permute4x64_pd(lhs, _MM_SHUFFLE(2, 3, 0, 1)),
                        _mm256_set1_pd(alpha.imag())
                    )
                )
            );
#else
            x *= alpha;
            y *= alpha;
#endif
        } else {
            x *= alpha;
            y *= alpha;
        }
        return *this;
    }
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator*(const ScalarArithmetic auto& fac) const& noexcept -> Vector {
        Vector result{*this};
        result *= fac;
        return result;
    }
    [[using gnu: always_inline, hot]]
    constexpr auto operator*(const ScalarArithmetic auto& fac) && noexcept -> Vector&& {
        operator*=(fac);
        return std::move(*this);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator/=(const ScalarArithmetic auto& den) noexcept -> Vector& {
        if constexpr (std::is_integral_v<value_type>) {
            const auto alpha{static_cast<value_type>(den)};
            x /= alpha;
            y /= alpha;
        } else {
            const auto alpha{static_cast<value_type>(1.0) / static_cast<value_type>(den)};
            operator*=(alpha);
        }
        return *this;
    }
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator/(const ScalarArithmetic auto& den) const& noexcept -> Vector {
        Vector result{*this};
        result /= den;
        return result;
    }
    [[using gnu: always_inline, hot]]
    constexpr auto operator/(const ScalarArithmetic auto& den) && noexcept -> Vector&& {
        operator/=(den);
        return std::move(*this);
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator-() const& noexcept -> Vector {
        Vector result;
        if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            _mm_store_si128(
                reinterpret_cast<__m128i*>(&result.x),
                _mm_xor_epi64(
                    _mm_load_si128(reinterpret_cast<const __m128i*>(&x)), _mm_set1_epi64x(-0L)
                )
            );
#elif defined(__ARM_NEON)
            vst1q_s64(
                reinterpret_cast<std::int64_t*>(&result.x),
                vnegq_s64(vld1q_s64(reinterpret_cast<const std::int64_t*>(&x)))
            );
#else
            result.x = -x;
            result.y = -y;
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__SSE2__)
            _mm_store_pd(
                reinterpret_cast<double*>(&result.x),
                _mm_xor_pd(_mm_load_pd(reinterpret_cast<const double*>(&x)), _mm_set1_pd(-0.0))
            );
#elif defined(__ARM_NEON)
            vst1q_f64(
                reinterpret_cast<double*>(&result.x),
                vnegq_f64(vld1q_f64(reinterpret_cast<const double*>(&x)))
            );
#else
            result.x = -x;
            result.y = -y;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__SSE__)
            _mm_store_ps(
                reinterpret_cast<float*>(&result.x),
                _mm_xor_ps(_mm_load_ps(reinterpret_cast<const float*>(&x)), _mm_set1_ps(-0.0F))
            );
#elif defined(__ARM_NEON)
            vst1q_f32(
                reinterpret_cast<float*>(&result.x),
                vnegq_f32(vld1q_f32(reinterpret_cast<const float*>(&x)))
            );
#else
            result.x = -x;
            result.y = -y;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX__)
            _mm256_store_pd(
                reinterpret_cast<double*>(&result.x),
                _mm256_xor_pd(
                    _mm256_load_pd(reinterpret_cast<const double*>(&x)), _mm256_set1_pd(-0.0)
                )
            );
#elif defined(__ARM_NEON)
            vst1q_f64(
                reinterpret_cast<double*>(&result.x),
                vnegq_f64(vld1q_f64(reinterpret_cast<const double*>(&x)))
            );
            vst1q_f64(
                reinterpret_cast<double*>(&result.y),
                vnegq_f64(vld1q_f64(reinterpret_cast<const double*>(&y)))
            );
#else
            result.x = -x;
            result.y = -y;
#endif
        } else {
            result.x = -x;
            result.y = -y;
        }
        return result;
    }
    [[using gnu: always_inline, hot]]
    constexpr auto operator-() && noexcept -> Vector&& {
        if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            _mm_store_si128(
                reinterpret_cast<__m128i*>(&x),
                _mm_xor_epi64(
                    _mm_load_si128(reinterpret_cast<const __m128i*>(&x)), _mm_set1_epi64x(-0L)
                )
            );
#elif defined(__ARM_NEON)
            vst1q_s64(
                reinterpret_cast<std::int64_t*>(&x),
                vnegq_s64(vld1q_s64(reinterpret_cast<const std::int64_t*>(&x)))
            );
#else
            x = -x;
            y = -y;
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__SSE2__)
            _mm_store_pd(
                reinterpret_cast<double*>(&x),
                _mm_xor_pd(_mm_load_pd(reinterpret_cast<const double*>(&x)), _mm_set1_pd(-0.0))
            );
#elif defined(__ARM_NEON)
            vst1q_f64(
                reinterpret_cast<double*>(&x),
                vnegq_f64(vld1q_f64(reinterpret_cast<const double*>(&x)))
            );
#else
            x = -x;
            y = -y;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__SSE__)
            _mm_store_ps(
                reinterpret_cast<float*>(&x),
                _mm_xor_ps(_mm_load_ps(reinterpret_cast<const float*>(&x)), _mm_set1_ps(-0.0F))
            );
#elif defined(__ARM_NEON)
            vst1q_f32(
                reinterpret_cast<float*>(&x),
                vnegq_f32(vld1q_f32(reinterpret_cast<const float*>(&x)))
            );
#else
            x = -x;
            y = -y;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX__)
            _mm256_store_pd(
                reinterpret_cast<double*>(&x),
                _mm256_xor_pd(
                    _mm256_load_pd(reinterpret_cast<const double*>(&x)), _mm256_set1_pd(-0.0)
                )
            );
#elif defined(__ARM_NEON)
            vst1q_f64(
                reinterpret_cast<double*>(&x),
                vnegq_f64(vld1q_f64(reinterpret_cast<const double*>(&x)))
            );
            vst1q_f64(
                reinterpret_cast<double*>(&y),
                vnegq_f64(vld1q_f64(reinterpret_cast<const double*>(&y)))
            );
#else
            x = -x;
            y = -y;
#endif
        } else {
            x = -x;
            y = -y;
        }
        return std::move(*this);
    }

    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto operator==(const Vector& other) const noexcept -> bool {
        if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            return _mm_cmp_epi64_mask(
                       _mm_load_si128(reinterpret_cast<const __m128i*>(&x)),
                       _mm_load_si128(reinterpret_cast<const __m128i*>(&other.x)), _CMP_EQ_OQ
                   ) == 0x3;
#elif defined(__ARM_NEON)
            const uint64x2_t mask{vceqq_s64(
                vld1q_s64(reinterpret_cast<const std::int64_t*>(&x)),
                vld1q_s64(reinterpret_cast<const std::int64_t*>(&other.x))
            )};
            return vminvq_u32(vreinterpretq_u32_u64(mask)) == 0xFFFFFFFFU;
#else
            return x == other.x && y == other.y;
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            return _mm_cmp_pd_mask(
                       _mm_load_pd(reinterpret_cast<const double*>(&x)),
                       _mm_load_pd(reinterpret_cast<const double*>(&other.x)), _CMP_EQ_OQ
                   ) == 0x3;
#elif defined(__ARM_NEON)
            const uint64x2_t mask{vceqq_f64(
                vld1q_f64(reinterpret_cast<const double*>(&x)),
                vld1q_f64(reinterpret_cast<const double*>(&other.x))
            )};
            return vminvq_u32(vreinterpretq_u32_u64(mask)) == 0xFFFFFFFFU;
#else
            return x == other.x && y == other.y;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            return _mm_cmp_ps_mask(
                       _mm_load_ps(reinterpret_cast<const float*>(&x)),
                       _mm_load_ps(reinterpret_cast<const float*>(&other.x)), _CMP_EQ_OQ
                   ) == 0xF;
#elif defined(__ARM_NEON)
            const uint32x4_t mask{vceqq_f32(
                vld1q_f32(reinterpret_cast<const float*>(&x)),
                vld1q_f32(reinterpret_cast<const float*>(&other.x))
            )};
            return vminvq_u32(mask) == 0xFFFFFFFFU;
#else
            return x == other.x && y == other.y;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            return _mm256_cmp_pd_mask(
                       _mm256_load_pd(reinterpret_cast<const double*>(&x)),
                       _mm256_load_pd(reinterpret_cast<const double*>(&other.x)), _CMP_EQ_OQ
                   ) == 0xF;
#elif defined(__ARM_NEON)
            const uint64x2_t mask_lo{vceqq_f64(
                vld1q_f64(reinterpret_cast<const double*>(&x)),
                vld1q_f64(reinterpret_cast<const double*>(&other.x))
            )};
            const uint64x2_t mask_hi{vceqq_f64(
                vld1q_f64(reinterpret_cast<const double*>(&y)),
                vld1q_f64(reinterpret_cast<const double*>(&other.y))
            )};
            return vminvq_u32(vreinterpretq_u32_u64(vandq_u64(mask_lo, mask_hi))) == 0xFFFFFFFFU;
#else
            return x == other.x && y == other.y;
#endif
        } else {
            return x == other.x && y == other.y;
        }
    }

    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto dot(const Vector& obj) const noexcept -> value_type {
        if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
            const __m128i sum{_mm_mullo_epi64(
                _mm_load_si128(reinterpret_cast<const __m128i*>(&x)),
                _mm_load_si128(reinterpret_cast<const __m128i*>(&obj.x))
            )};
            return _mm_cvtsi128_si64(sum) + _mm_extract_epi64(sum, 1);
#else
            return x * obj.x + y * obj.y;
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__SSE3__)
            const __m128d sum{_mm_mul_pd(
                _mm_load_pd(reinterpret_cast<const double*>(&x)),
                _mm_load_pd(reinterpret_cast<const double*>(&obj.x))
            )};
            return _mm_cvtsd_f64(_mm_hadd_pd(sum, sum));
#elif defined(__ARM_NEON)
            return vaddvq_f64(vmulq_f64(
                vld1q_f64(reinterpret_cast<const double*>(&x)),
                vld1q_f64(reinterpret_cast<const double*>(&obj.x))
            ));
#else
            return x * obj.x + y * obj.y;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__AVX__)
            const __m128 lhs{_mm_load_ps(reinterpret_cast<const float*>(&x))};
            const __m128 rhs{_mm_load_ps(reinterpret_cast<const float*>(&obj.x))};
            const __m128 sum{_mm_addsub_ps(
                _mm_mul_ps(lhs, _mm_moveldup_ps(rhs)),
                _mm_mul_ps(_mm_permute_ps(lhs, _MM_SHUFFLE(2, 3, 0, 1)), _mm_movehdup_ps(rhs))
            )};
            value_type result;
            _mm_storel_pi(
                reinterpret_cast<__m64*>(&result),
                _mm_add_ps(sum, _mm_permute_ps(sum, _MM_SHUFFLE(1, 0, 3, 2)))
            );
            return result;
#else
            return x * obj.x + y * obj.y;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
            const __m256d lhs{_mm256_load_pd(reinterpret_cast<const double*>(&x))};
            const __m256d rhs{_mm256_load_pd(reinterpret_cast<const double*>(&obj.x))};
            const __m256d sum{
                _mm256_addsub_pd(
                    _mm256_mul_pd(lhs, _mm256_movedup_pd(rhs)),
                    _mm256_mul_pd(
                        _mm256_permute4x64_pd(lhs, _MM_SHUFFLE(2, 3, 0, 1)),
                        _mm256_permute4x64_pd(rhs, _MM_SHUFFLE(3, 3, 1, 1))
                    )
                ),
            };
            value_type result;
            _mm_store_pd(
                reinterpret_cast<double*>(&result),
                _mm_add_pd(_mm256_extractf64x2_pd(sum, 0), _mm256_extractf64x2_pd(sum, 1))
            );
            return result;
#else
            return x * obj.x + y * obj.y;
#endif
        } else {
            return x * obj.x + y * obj.y;
        }
    }

    template <size_type OuterSize, MatrixOrder ObjOrder>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(const Matrix<value_type, kSize, OuterSize, ObjOrder>& obj) const noexcept
        -> Vector {
        Vector<value_type, OuterSize> result(OuterSize, 0.0);
        for (size_type j{0}; j < OuterSize; ++j) {
            for (size_type i{0}; i < kSize; ++i) {
                result[j] += operator[](i) * obj[i, j];
            }
        }
        return result;
    }

    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto crossProj(const Vector& obj) const noexcept -> value_type {
        if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
            const __m128i sum{_mm_mullo_epi64(
                _mm_load_si128(reinterpret_cast<const __m128i*>(&x)),
                _mm_shuffle_epi32(
                    _mm_load_si128(reinterpret_cast<const __m128i*>(&obj.x)),
                    _MM_SHUFFLE(1, 0, 3, 2)
                )
            )};
            return _mm_cvtsi128_si64(
                _mm_sub_epi64(sum, _mm_shuffle_epi32(sum, _MM_SHUFFLE(1, 0, 3, 2)))
            );
#else
            return x * obj.y - y * obj.x;
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX__)
            const __m128d sum{_mm_mul_pd(
                _mm_load_pd(reinterpret_cast<const double*>(&x)),
                _mm_permute_pd(
                    _mm_load_pd(reinterpret_cast<const double*>(&obj.x)), _MM_SHUFFLE2(0, 1)
                )
            )};
            return _mm_cvtsd_f64(_mm_sub_pd(sum, _mm_permute_pd(sum, _MM_SHUFFLE2(0, 1))));
#else
            return x * obj.y - y * obj.x;
#endif
        } else {
            return x * obj.y - y * obj.x;
        }
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto normalized() const noexcept -> Vector {
        return *this / euclidean();
    }
    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclidean() const noexcept -> detail::DenseNormTraitT<value_type> {
        return std::sqrt(euclideanSqr());
    }
    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto euclideanSqr() const noexcept -> detail::DenseNormTraitT<value_type> {
        if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
            const __m128i lhs{_mm_load_si128(reinterpret_cast<const __m128i*>(&x))};
            const __m128i sum{_mm_mullo_epi64(lhs, lhs)};
            return _mm_cvtsi128_si64(sum) + _mm_extract_epi64(sum, 1);
#else
            return std::norm(x) + std::norm(y);
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__SSE3__)
            const __m128d lhs{_mm_load_pd(reinterpret_cast<const double*>(&x))};
            const __m128d sum{_mm_mul_pd(lhs, lhs)};
            return _mm_cvtsd_f64(_mm_hadd_pd(sum, sum));
#elif defined(__ARM_NEON)
            const float64x2_t lhs{vld1q_f64(reinterpret_cast<const double*>(&x))};
            return vaddvq_f64(vmulq_f64(lhs, lhs));
#else
            return std::norm(x) + std::norm(y);
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__SSE3__)
            const __m128 lhs{_mm_load_ps(reinterpret_cast<const float*>(&x))};
            __m128 sum{_mm_mul_ps(lhs, lhs)};
            sum = _mm_hadd_ps(sum, sum);
            return _mm_cvtss_f32(_mm_hadd_ps(sum, sum));
#elif defined(__ARM_NEON)
            const float32x4_t lhs{vld1q_f32(reinterpret_cast<const float*>(&x))};
            return vaddvq_f32(vmulq_f32(lhs, lhs));
#else
            return std::norm(x) + std::norm(y);
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
            const __m256d lhs{_mm256_load_pd(reinterpret_cast<const double*>(&x))};
            const __m256d sum{_mm256_mul_pd(lhs, lhs)};
            const __m128d sum2{
                _mm_add_pd(_mm256_castpd256_pd128(sum), _mm256_extractf64x2_pd(sum, 1))
            };
            return _mm_cvtsd_f64(_mm_hadd_pd(sum2, sum2));
#elif defined(__ARM_NEON)
            const float64x2_t lhs_lo{vld1q_f64(reinterpret_cast<const double*>(&x))};
            const float64x2_t lhs_hi{vld1q_f64(reinterpret_cast<const double*>(&y))};
            return vaddvq_f64(vaddq_f64(vmulq_f64(lhs_lo, lhs_lo), vmulq_f64(lhs_hi, lhs_hi)));
#else
            return std::norm(x) + std::norm(y);
#endif
        } else {
            return std::norm(x) + std::norm(y);
        }
    }
    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclideanTo(const Vector& obj) const noexcept
        -> detail::DenseNormTraitT<value_type> {
        return operator-(obj).euclidean();
    }
    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto euclideanSqrTo(const Vector& obj) const noexcept
        -> detail::DenseNormTraitT<value_type> {
        return operator-(obj).euclideanSqr();
    }
    [[using gnu: pure, always_inline, hot]]
    constexpr auto identicalTo(const Vector& obj, value_type tol = 1E-8) const noexcept -> bool {
        return euclideanSqrTo(obj) < tol * tol;
    }
    [[using gnu: pure, always_inline, hot]]
    constexpr auto orthogonalTo(const Vector& obj, value_type tol = 1E-8) const noexcept -> bool {
        return std::abs(dot(obj)) < tol * tol;
    }

    [[using gnu: always_inline]]
    constexpr auto selfConjugated() noexcept -> Vector& {
        if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__SSE__)
            _mm_store_ps(
                reinterpret_cast<float*>(&x), _mm_xor_ps(
                                                  _mm_load_ps(reinterpret_cast<const float*>(&x)),
                                                  _mm_set_ps(-0.0F, 0.0F, -0.0F, 0.0F)
                                              )
            );
#elif defined(__ARM_NEON)
            static constexpr std::uint32_t kConjMaskF[4]{0U, 0x80000000U, 0U, 0x80000000U};
            vst1q_f32(
                reinterpret_cast<float*>(&x),
                vreinterpretq_f32_u32(veorq_u32(
                    vreinterpretq_u32_f32(vld1q_f32(reinterpret_cast<const float*>(&x))),
                    vld1q_u32(kConjMaskF)
                ))
            );
#else
            x = std::conj(x);
            y = std::conj(y);
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX__)
            _mm256_store_pd(
                reinterpret_cast<double*>(&x),
                _mm256_xor_pd(
                    _mm256_load_pd(reinterpret_cast<const double*>(&x)),
                    _mm256_set_pd(-0.0, 0.0, -0.0, 0.0)
                )
            );
#elif defined(__ARM_NEON)
            static constexpr std::uint64_t kConjMaskD[2]{0ULL, 0x8000000000000000ULL};
            const uint64x2_t mask{vld1q_u64(kConjMaskD)};
            vst1q_f64(
                reinterpret_cast<double*>(&x),
                vreinterpretq_f64_u64(veorq_u64(
                    vreinterpretq_u64_f64(vld1q_f64(reinterpret_cast<const double*>(&x))), mask
                ))
            );
            vst1q_f64(
                reinterpret_cast<double*>(&y),
                vreinterpretq_f64_u64(veorq_u64(
                    vreinterpretq_u64_f64(vld1q_f64(reinterpret_cast<const double*>(&y))), mask
                ))
            );
#else
            x = std::conj(x);
            y = std::conj(y);
#endif
        } else if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
            x = std::conj(x);
            y = std::conj(y);
        }
        return *this;
    }
    [[using gnu: pure, always_inline]]
    constexpr auto conjugated() const& noexcept -> Vector {
        Vector result{*this};
        if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
            result.selfConjugated();
        }
        return result;
    }
    [[using gnu: always_inline]]
    constexpr auto conjugated() && noexcept -> Vector&& {
        if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
            selfConjugated();
        }
        return std::move(*this);
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto radian() const noexcept -> value_type
        requires std::floating_point<value_type>
    {
        return std::atan2(y, x);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto selfRotate(const std::floating_point auto& radian) noexcept -> Vector&
        requires std::floating_point<value_type>
    {
        *this = Vector{
            x * std::cos(radian) - y * std::sin(radian), x * std::sin(radian) + y * std::cos(radian)
        };
        return *this;
    }
    [[using gnu: pure, always_inline, hot]]
    constexpr auto rotate(const std::floating_point auto& radian) const& noexcept -> Vector
        requires std::floating_point<value_type>
    {
        return Vector{
            x * std::cos(radian) - y * std::sin(radian), x * std::sin(radian) + y * std::cos(radian)
        };
    }
    [[using gnu: always_inline, hot]]
    constexpr auto rotate(const std::floating_point auto& radian) && noexcept -> Vector&&
        requires std::floating_point<value_type>
    {
        this->selfRotate(radian);
        return std::move(*this);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto selfRotateHalfPi() noexcept -> Vector&
        requires std::floating_point<value_type>
    {
        *this = Vector{-y, x};
        return *this;
    }
    [[using gnu: pure, always_inline, hot]]
    constexpr auto rotateHalfPi() const& noexcept -> Vector
        requires std::floating_point<value_type>
    {
        return Vector{-y, x};
    }
    [[using gnu: pure, always_inline, hot]]
    constexpr auto rotateHalfPi() && noexcept -> Vector&&
        requires std::floating_point<value_type>
    {
        this->selfRotateHalfPi();
        return std::move(*this);
    }

    using serialize = zpp::bits::members<2>;

    value_type x, y;
};

template <ScalarArithmetic T>
[[using gnu: pure, always_inline, hot]]
inline constexpr auto operator*(const ScalarArithmetic auto& fac, const Vector<T, 2>& obj) noexcept
    -> Vector<T, 2> {
    return obj * fac;
}

template <ScalarArithmetic T>
[[using gnu: always_inline, hot]]
inline constexpr auto operator*(const ScalarArithmetic auto& fac, Vector<T, 2>&& obj) noexcept
    -> Vector<T, 2>&& {
    obj *= fac;
    return std::move(obj);
}

template <typename Char, ScalarArithmetic T>
[[using gnu: always_inline]]
inline auto operator<<(std::basic_ostream<Char>& os, const Vector<T, 2>& obj)
    -> std::basic_ostream<Char>& {
    os << std::format("{}", obj);
    return os;
}

template <ScalarArithmetic T>
using Vec2 = Vector<T, 2>;

using Vec2s = Vec2<float>;
using Vec2d = Vec2<double>;
using Vec2c = Vec2<std::complex<float>>;
using Vec2z = Vec2<std::complex<double>>;

template <typename T>
struct isVec2Arithmetic : std::false_type {};

template <ScalarArithmetic T>
struct isVec2Arithmetic<Vec2<T>> : std::true_type {};

template <typename T>
inline constexpr bool isVec2ArithmeticV = isVec2Arithmetic<T>::value;

template <typename T>
concept Vec2Arithmetic = isVec2ArithmeticV<T>;

template <
    std::input_iterator InputIt1, std::sentinel_for<InputIt1> SentinelIt1,
    std::input_iterator InputIt2, std::sentinel_for<InputIt1> SentinelIt2,
    std::weakly_incrementable OutputIt>
    requires std::floating_point<typename std::iterator_traits<InputIt1>::value_type> &&
             std::floating_point<typename std::iterator_traits<InputIt2>::value_type> &&
             Vec2Arithmetic<typename std::iterator_traits<OutputIt>::value_type>
[[using gnu: always_inline, hot]]
inline auto squeeze(
    InputIt1 first1, SentinelIt1 last1, InputIt2 first2, SentinelIt2 last2, OutputIt result
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

template <::basjoo::math::ScalarArithmetic T, typename Char>
struct formatter<::basjoo::math::Vec2<T>, Char> {
    [[using gnu: ]]
    constexpr auto parse(std::basic_format_parse_context<Char>& ctx) -> decltype(ctx.begin()) {
        auto it{ctx.begin()};
        const auto end{ctx.end()};
        if (it != end && *it >= static_cast<Char>('0') && *it <= static_cast<Char>('9')) {
            width = 0;
            for (; it != end && *it >= static_cast<Char>('0') && *it <= static_cast<Char>('9');
                 ++it) {
                width = width * 10 + (*it - static_cast<Char>('0'));
            }
        }
        if (it != end && *it == static_cast<Char>('.')) {
            ++it;
            precision = 0;
            for (; it != end && *it >= static_cast<Char>('0') && *it <= static_cast<Char>('9');
                 ++it) {
                precision = precision * 10 + (*it - static_cast<Char>('0'));
            }
        }
        return it;
    }

    template <typename FormatContext>
    [[using gnu: ]]
    auto format(const ::basjoo::math::Vec2<T>& obj, FormatContext& ctx) const
        -> decltype(ctx.out()) {
        auto out = ctx.out();
        if (width > 0 && precision >= 0) {
            return std::format_to(
                out, "(x: {:>{}.{}f}, y: {:>{}.{}f})", obj.x, width, precision, obj.y, width,
                precision
            );
        }
        if (width > 0) {
            return std::format_to(out, "(x: {:>{}f}, y: {:>{}f})", obj.x, width, obj.y, width);
        }
        if (precision >= 0) {
            return std::format_to(
                out, "(x: {:.{}f}, y: {:.{}f})", obj.x, precision, obj.y, precision
            );
        }
        return std::format_to(out, "(x: {}, y: {})", obj.x, obj.y);
    }

    int width{0};
    int precision{-1};
};

template <std::floating_point T>
[[using gnu: pure, always_inline]]
inline constexpr auto hypot(const ::basjoo::math::Vec2<T>& obj) noexcept
    -> ::basjoo::math::detail::DenseNormTraitT<T> {
    return std::hypot(obj.x, obj.y);
}

template <::basjoo::math::ScalarArithmetic T>
[[using gnu: pure, always_inline]]
inline constexpr auto abs(const ::basjoo::math::Vec2<T>& obj) noexcept
    -> ::basjoo::math::detail::DenseNormTraitT<T> {
    return obj.euclidean();
}

template <::basjoo::math::ScalarArithmetic T>
[[using gnu: pure, always_inline]]
inline constexpr auto norm(const ::basjoo::math::Vec2<T>& obj) noexcept
    -> ::basjoo::math::detail::DenseNormTraitT<T> {
    return obj.euclideanSqr();
}

template <::basjoo::math::ScalarArithmetic T>
[[using gnu: pure, always_inline]]
inline constexpr auto conj(const ::basjoo::math::Vec2<T>& vector) noexcept
    -> ::basjoo::math::Vec2<T> {
    return vector.conjugated();
}

template <::basjoo::math::ScalarArithmetic T>
[[using gnu: always_inline]]
inline constexpr auto conj(::basjoo::math::Vec2<T>&& vector) noexcept -> ::basjoo::math::Vec2<T>&& {
    vector.conjugate();
    return std::move(vector);
}

template <std::floating_point T>
[[using gnu: pure, always_inline]]
inline constexpr auto atan2(const ::basjoo::math::Vec2<T>& obj) noexcept -> T {
    return std::atan2(obj.y, obj.x);
}

} // namespace std

// NOLINTEND(cert-dcl58-cpp)
