/**
 * @file math/dense/dense_lanes.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief Element-wise SIMD kernels shared by the dense types, named after C++26 std::linalg.
 * @version 0.1
 * @date 2026-11-03
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

#if defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>
#elif defined(__aarch64__) || defined(_M_ARM64)
#include <arm_neon.h>
#endif

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/detail/dense_norm_trait.hpp"

namespace basjoo::math::detail {

// One SIMD register of W lanes of Lane (double, float, std::int64_t or std::int32_t) on the
// instruction sets of the build. Without a specialization there is no such register and DenseLanes
// runs scalar code. kHasMul tells whether the lanes multiply. Loads and stores take any alignment.
template <typename Lane, std::size_t W>
struct LaneRegister final {
    static constexpr bool kAvailable{false};
    static constexpr bool kHasMul{false};
};

#if defined(__SSE2__)
template <>
struct LaneRegister<double, 2> final {
    using reg_type = __m128d;
    static constexpr bool kAvailable{true};
    static constexpr bool kHasMul{true};

    [[using gnu: always_inline]]
    static auto load(const double* src) noexcept -> reg_type {
        return _mm_loadu_pd(src);
    }
    [[using gnu: always_inline]]
    static auto store(double* dst, reg_type v) noexcept -> void {
        _mm_storeu_pd(dst, v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(double s) noexcept -> reg_type {
        return _mm_set1_pd(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm_add_pd(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm_sub_pd(a, b);
    }
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm_mul_pd(a, b);
    }
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return _mm_xor_pd(v, _mm_set1_pd(-0.0));
    }
    [[using gnu: always_inline]]
    static auto conjugate(reg_type v) noexcept -> reg_type {
        return _mm_xor_pd(v, _mm_set_pd(-0.0, 0.0));
    }
    // v * (re, im) as (a re - b im, b re + a im) for v = (a, b).
    [[using gnu: always_inline]]
    static auto mul_complex(reg_type v, double re, double im) noexcept -> reg_type {
        const reg_type by_re{_mm_mul_pd(v, _mm_set1_pd(re))};
        const reg_type by_im{_mm_mul_pd(_mm_shuffle_pd(v, v, 0b01), _mm_set1_pd(im))};
#if defined(__SSE3__)
        return _mm_addsub_pd(by_re, by_im);
#else
        // Without SSE3's addsub: add by_im with its real lane negated.
        return _mm_add_pd(by_re, _mm_xor_pd(by_im, _mm_set_pd(0.0, -0.0)));
#endif
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return _mm_movemask_pd(_mm_cmpeq_pd(a, b)) == 0x3;
    }
};

template <>
struct LaneRegister<float, 4> final {
    using reg_type = __m128;
    static constexpr bool kAvailable{true};
    static constexpr bool kHasMul{true};

    [[using gnu: always_inline]]
    static auto load(const float* src) noexcept -> reg_type {
        return _mm_loadu_ps(src);
    }
    [[using gnu: always_inline]]
    static auto store(float* dst, reg_type v) noexcept -> void {
        _mm_storeu_ps(dst, v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(float s) noexcept -> reg_type {
        return _mm_set1_ps(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm_add_ps(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm_sub_ps(a, b);
    }
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm_mul_ps(a, b);
    }
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return _mm_xor_ps(v, _mm_set1_ps(-0.0F));
    }
    [[using gnu: always_inline]]
    static auto conjugate(reg_type v) noexcept -> reg_type {
        return _mm_xor_ps(v, _mm_set_ps(-0.0F, 0.0F, -0.0F, 0.0F));
    }
    // Each complex pair v = (a, b) times (re, im), as (a re - b im, b re + a im).
    [[using gnu: always_inline]]
    static auto mul_complex(reg_type v, float re, float im) noexcept -> reg_type {
        const reg_type by_re{_mm_mul_ps(v, _mm_set1_ps(re))};
        const reg_type by_im{
            _mm_mul_ps(_mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 3, 0, 1)), _mm_set1_ps(im))
        };
#if defined(__SSE3__)
        return _mm_addsub_ps(by_re, by_im);
#else
        // Without SSE3's addsub: add by_im with its real lanes negated.
        return _mm_add_ps(by_re, _mm_xor_ps(by_im, _mm_set_ps(0.0F, -0.0F, 0.0F, -0.0F)));
#endif
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return _mm_movemask_ps(_mm_cmpeq_ps(a, b)) == 0xF;
    }
};

template <>
struct LaneRegister<std::int64_t, 2> final {
    using reg_type = __m128i;
    static constexpr bool kAvailable{true};
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
    static constexpr bool kHasMul{true};
#else
    static constexpr bool kHasMul{false};
#endif

    [[using gnu: always_inline]]
    static auto load(const std::int64_t* src) noexcept -> reg_type {
        return _mm_loadu_si128(reinterpret_cast<const __m128i*>(src));
    }
    [[using gnu: always_inline]]
    static auto store(std::int64_t* dst, reg_type v) noexcept -> void {
        _mm_storeu_si128(reinterpret_cast<__m128i*>(dst), v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(std::int64_t s) noexcept -> reg_type {
        return _mm_set1_epi64x(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm_add_epi64(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm_sub_epi64(a, b);
    }
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm_mullo_epi64(a, b);
    }
#endif
    // Two's complement has no sign bit to flip: negate as 0 - x.
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return _mm_sub_epi64(_mm_setzero_si128(), v);
    }
    // A 64-bit lane is equal where both of its 32-bit halves are: SSE2 has no 64-bit compare.
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        const __m128i halves{_mm_cmpeq_epi32(a, b)};
        return _mm_movemask_pd(_mm_castsi128_pd(
                   _mm_and_si128(halves, _mm_shuffle_epi32(halves, _MM_SHUFFLE(2, 3, 0, 1)))
               )) == 0x3;
    }
};

// Two lanes in the low half of an SSE register, moved by 64-bit loads and stores.
template <>
struct LaneRegister<float, 2> final {
    using reg_type = __m128;
    static constexpr bool kAvailable{true};
    static constexpr bool kHasMul{true};

    [[using gnu: always_inline]]
    static auto load(const float* src) noexcept -> reg_type {
        return _mm_castsi128_ps(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(src)));
    }
    [[using gnu: always_inline]]
    static auto store(float* dst, reg_type v) noexcept -> void {
        _mm_storel_epi64(reinterpret_cast<__m128i*>(dst), _mm_castps_si128(v));
    }
    [[using gnu: always_inline]]
    static auto broadcast(float s) noexcept -> reg_type {
        return _mm_set1_ps(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm_add_ps(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm_sub_ps(a, b);
    }
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm_mul_ps(a, b);
    }
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return _mm_xor_ps(v, _mm_set1_ps(-0.0F));
    }
    [[using gnu: always_inline]]
    static auto conjugate(reg_type v) noexcept -> reg_type {
        return _mm_xor_ps(v, _mm_set_ps(-0.0F, 0.0F, -0.0F, 0.0F));
    }
    // v = (a, b) times (re, im), as (a re - b im, b re + a im).
    [[using gnu: always_inline]]
    static auto mul_complex(reg_type v, float re, float im) noexcept -> reg_type {
        const reg_type by_re{_mm_mul_ps(v, _mm_set1_ps(re))};
        const reg_type by_im{
            _mm_mul_ps(_mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 3, 0, 1)), _mm_set1_ps(im))
        };
#if defined(__SSE3__)
        return _mm_addsub_ps(by_re, by_im);
#else
        // Without SSE3's addsub: add by_im with its real lanes negated.
        return _mm_add_ps(by_re, _mm_xor_ps(by_im, _mm_set_ps(0.0F, -0.0F, 0.0F, -0.0F)));
#endif
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return (_mm_movemask_ps(_mm_cmpeq_ps(a, b)) & 0x3) == 0x3;
    }
};

// Two lanes in the low half of an SSE register, moved by 64-bit loads and stores.
template <>
struct LaneRegister<std::int32_t, 2> final {
    using reg_type = __m128i;
    static constexpr bool kAvailable{true};
#if defined(__SSE4_1__)
    static constexpr bool kHasMul{true};
#else
    static constexpr bool kHasMul{false};
#endif

    [[using gnu: always_inline]]
    static auto load(const std::int32_t* src) noexcept -> reg_type {
        return _mm_loadl_epi64(reinterpret_cast<const __m128i*>(src));
    }
    [[using gnu: always_inline]]
    static auto store(std::int32_t* dst, reg_type v) noexcept -> void {
        _mm_storel_epi64(reinterpret_cast<__m128i*>(dst), v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(std::int32_t s) noexcept -> reg_type {
        return _mm_set1_epi32(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm_add_epi32(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm_sub_epi32(a, b);
    }
#if defined(__SSE4_1__)
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm_mullo_epi32(a, b);
    }
#endif
    // Two's complement has no sign bit to flip: negate as 0 - x.
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return _mm_sub_epi32(_mm_setzero_si128(), v);
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return (_mm_movemask_ps(_mm_castsi128_ps(_mm_cmpeq_epi32(a, b))) & 0x3) == 0x3;
    }
};

template <>
struct LaneRegister<std::int32_t, 4> final {
    using reg_type = __m128i;
    static constexpr bool kAvailable{true};
#if defined(__SSE4_1__)
    static constexpr bool kHasMul{true};
#else
    static constexpr bool kHasMul{false};
#endif

    [[using gnu: always_inline]]
    static auto load(const std::int32_t* src) noexcept -> reg_type {
        return _mm_loadu_si128(reinterpret_cast<const __m128i*>(src));
    }
    [[using gnu: always_inline]]
    static auto store(std::int32_t* dst, reg_type v) noexcept -> void {
        _mm_storeu_si128(reinterpret_cast<__m128i*>(dst), v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(std::int32_t s) noexcept -> reg_type {
        return _mm_set1_epi32(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm_add_epi32(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm_sub_epi32(a, b);
    }
#if defined(__SSE4_1__)
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm_mullo_epi32(a, b);
    }
#endif
    // Two's complement has no sign bit to flip: negate as 0 - x.
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return _mm_sub_epi32(_mm_setzero_si128(), v);
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return _mm_movemask_ps(_mm_castsi128_ps(_mm_cmpeq_epi32(a, b))) == 0xF;
    }
};
#endif

#if defined(__AVX__)
template <>
struct LaneRegister<double, 4> final {
    using reg_type = __m256d;
    static constexpr bool kAvailable{true};
    static constexpr bool kHasMul{true};

    [[using gnu: always_inline]]
    static auto load(const double* src) noexcept -> reg_type {
        return _mm256_loadu_pd(src);
    }
    [[using gnu: always_inline]]
    static auto store(double* dst, reg_type v) noexcept -> void {
        _mm256_storeu_pd(dst, v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(double s) noexcept -> reg_type {
        return _mm256_set1_pd(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm256_add_pd(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm256_sub_pd(a, b);
    }
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm256_mul_pd(a, b);
    }
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return _mm256_xor_pd(v, _mm256_set1_pd(-0.0));
    }
    [[using gnu: always_inline]]
    static auto conjugate(reg_type v) noexcept -> reg_type {
        return _mm256_xor_pd(v, _mm256_set_pd(-0.0, 0.0, -0.0, 0.0));
    }
    // Each complex pair v = (a, b) times (re, im), as (a re - b im, b re + a im).
    [[using gnu: always_inline]]
    static auto mul_complex(reg_type v, double re, double im) noexcept -> reg_type {
        return _mm256_addsub_pd(
            _mm256_mul_pd(v, _mm256_set1_pd(re)),
            _mm256_mul_pd(_mm256_permute_pd(v, 0b0101), _mm256_set1_pd(im))
        );
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return _mm256_movemask_pd(_mm256_cmp_pd(a, b, _CMP_EQ_OQ)) == 0xF;
    }
};

template <>
struct LaneRegister<float, 8> final {
    using reg_type = __m256;
    static constexpr bool kAvailable{true};
    static constexpr bool kHasMul{true};

    [[using gnu: always_inline]]
    static auto load(const float* src) noexcept -> reg_type {
        return _mm256_loadu_ps(src);
    }
    [[using gnu: always_inline]]
    static auto store(float* dst, reg_type v) noexcept -> void {
        _mm256_storeu_ps(dst, v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(float s) noexcept -> reg_type {
        return _mm256_set1_ps(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm256_add_ps(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm256_sub_ps(a, b);
    }
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm256_mul_ps(a, b);
    }
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return _mm256_xor_ps(v, _mm256_set1_ps(-0.0F));
    }
    [[using gnu: always_inline]]
    static auto conjugate(reg_type v) noexcept -> reg_type {
        return _mm256_xor_ps(v, _mm256_set_ps(-0.0F, 0.0F, -0.0F, 0.0F, -0.0F, 0.0F, -0.0F, 0.0F));
    }
    // Each complex pair v = (a, b) times (re, im), as (a re - b im, b re + a im).
    [[using gnu: always_inline]]
    static auto mul_complex(reg_type v, float re, float im) noexcept -> reg_type {
        return _mm256_addsub_ps(
            _mm256_mul_ps(v, _mm256_set1_ps(re)),
            _mm256_mul_ps(_mm256_permute_ps(v, _MM_SHUFFLE(2, 3, 0, 1)), _mm256_set1_ps(im))
        );
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return _mm256_movemask_ps(_mm256_cmp_ps(a, b, _CMP_EQ_OQ)) == 0xFF;
    }
};
#endif

#if defined(__AVX2__)
template <>
struct LaneRegister<std::int64_t, 4> final {
    using reg_type = __m256i;
    static constexpr bool kAvailable{true};
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
    static constexpr bool kHasMul{true};
#else
    static constexpr bool kHasMul{false};
#endif

    [[using gnu: always_inline]]
    static auto load(const std::int64_t* src) noexcept -> reg_type {
        return _mm256_loadu_si256(reinterpret_cast<const __m256i*>(src));
    }
    [[using gnu: always_inline]]
    static auto store(std::int64_t* dst, reg_type v) noexcept -> void {
        _mm256_storeu_si256(reinterpret_cast<__m256i*>(dst), v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(std::int64_t s) noexcept -> reg_type {
        return _mm256_set1_epi64x(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm256_add_epi64(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm256_sub_epi64(a, b);
    }
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm256_mullo_epi64(a, b);
    }
#endif
    // Two's complement has no sign bit to flip: negate as 0 - x.
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return _mm256_sub_epi64(_mm256_setzero_si256(), v);
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return _mm256_movemask_pd(_mm256_castsi256_pd(_mm256_cmpeq_epi64(a, b))) == 0xF;
    }
};

template <>
struct LaneRegister<std::int32_t, 8> final {
    using reg_type = __m256i;
    static constexpr bool kAvailable{true};
    static constexpr bool kHasMul{true};

    [[using gnu: always_inline]]
    static auto load(const std::int32_t* src) noexcept -> reg_type {
        return _mm256_loadu_si256(reinterpret_cast<const __m256i*>(src));
    }
    [[using gnu: always_inline]]
    static auto store(std::int32_t* dst, reg_type v) noexcept -> void {
        _mm256_storeu_si256(reinterpret_cast<__m256i*>(dst), v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(std::int32_t s) noexcept -> reg_type {
        return _mm256_set1_epi32(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm256_add_epi32(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm256_sub_epi32(a, b);
    }
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm256_mullo_epi32(a, b);
    }
    // Two's complement has no sign bit to flip: negate as 0 - x.
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return _mm256_sub_epi32(_mm256_setzero_si256(), v);
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return _mm256_movemask_ps(_mm256_castsi256_ps(_mm256_cmpeq_epi32(a, b))) == 0xFF;
    }
};
#endif

#if defined(__AVX512F__)
template <>
struct LaneRegister<double, 8> final {
    using reg_type = __m512d;
    static constexpr bool kAvailable{true};
    static constexpr bool kHasMul{true};

    [[using gnu: always_inline]]
    static auto load(const double* src) noexcept -> reg_type {
        return _mm512_loadu_pd(src);
    }
    [[using gnu: always_inline]]
    static auto store(double* dst, reg_type v) noexcept -> void {
        _mm512_storeu_pd(dst, v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(double s) noexcept -> reg_type {
        return _mm512_set1_pd(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm512_add_pd(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm512_sub_pd(a, b);
    }
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm512_mul_pd(a, b);
    }
    // AVX-512F has no floating-point XOR, which DQ adds: the sign bits flip as integers.
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return _mm512_castsi512_pd(
            _mm512_xor_si512(_mm512_castpd_si512(v), _mm512_castpd_si512(_mm512_set1_pd(-0.0)))
        );
    }
    [[using gnu: always_inline]]
    static auto conjugate(reg_type v) noexcept -> reg_type {
        return _mm512_castsi512_pd(_mm512_xor_si512(
            _mm512_castpd_si512(v),
            _mm512_castpd_si512(_mm512_set_pd(-0.0, 0.0, -0.0, 0.0, -0.0, 0.0, -0.0, 0.0))
        ));
    }
    // Each complex pair v = (a, b) times (re, im), as (a re - b im, b re + a im).
    [[using gnu: always_inline]]
    static auto mul_complex(reg_type v, double re, double im) noexcept -> reg_type {
        const reg_type by_re{_mm512_mul_pd(v, _mm512_set1_pd(re))};
        const reg_type by_im{_mm512_mul_pd(_mm512_permute_pd(v, 0x55), _mm512_set1_pd(im))};
        // No 512-bit addsub: subtract in the real lanes, add in the imaginary ones.
        return _mm512_mask_sub_pd(_mm512_add_pd(by_re, by_im), 0x55, by_re, by_im);
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return _mm512_cmp_pd_mask(a, b, _CMP_EQ_OQ) == 0xFF;
    }
};

template <>
struct LaneRegister<float, 16> final {
    using reg_type = __m512;
    static constexpr bool kAvailable{true};
    static constexpr bool kHasMul{true};

    [[using gnu: always_inline]]
    static auto load(const float* src) noexcept -> reg_type {
        return _mm512_loadu_ps(src);
    }
    [[using gnu: always_inline]]
    static auto store(float* dst, reg_type v) noexcept -> void {
        _mm512_storeu_ps(dst, v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(float s) noexcept -> reg_type {
        return _mm512_set1_ps(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm512_add_ps(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm512_sub_ps(a, b);
    }
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm512_mul_ps(a, b);
    }
    // AVX-512F has no floating-point XOR, which DQ adds: the sign bits flip as integers.
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return _mm512_castsi512_ps(
            _mm512_xor_si512(_mm512_castps_si512(v), _mm512_castps_si512(_mm512_set1_ps(-0.0F)))
        );
    }
    [[using gnu: always_inline]]
    static auto conjugate(reg_type v) noexcept -> reg_type {
        return _mm512_castsi512_ps(_mm512_xor_si512(
            _mm512_castps_si512(v), _mm512_castps_si512(_mm512_set_ps(
                                        -0.0F, 0.0F, -0.0F, 0.0F, -0.0F, 0.0F, -0.0F, 0.0F, -0.0F,
                                        0.0F, -0.0F, 0.0F, -0.0F, 0.0F, -0.0F, 0.0F
                                    ))
        ));
    }
    // Each complex pair v = (a, b) times (re, im), as (a re - b im, b re + a im).
    [[using gnu: always_inline]]
    static auto mul_complex(reg_type v, float re, float im) noexcept -> reg_type {
        const reg_type by_re{_mm512_mul_ps(v, _mm512_set1_ps(re))};
        const reg_type by_im{
            _mm512_mul_ps(_mm512_permute_ps(v, _MM_SHUFFLE(2, 3, 0, 1)), _mm512_set1_ps(im))
        };
        // No 512-bit addsub: subtract in the real lanes, add in the imaginary ones.
        return _mm512_mask_sub_ps(_mm512_add_ps(by_re, by_im), 0x5555, by_re, by_im);
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return _mm512_cmp_ps_mask(a, b, _CMP_EQ_OQ) == 0xFFFF;
    }
};

template <>
struct LaneRegister<std::int64_t, 8> final {
    using reg_type = __m512i;
    static constexpr bool kAvailable{true};
#if defined(__AVX512DQ__)
    static constexpr bool kHasMul{true};
#else
    static constexpr bool kHasMul{false};
#endif

    [[using gnu: always_inline]]
    static auto load(const std::int64_t* src) noexcept -> reg_type {
        return _mm512_loadu_si512(src);
    }
    [[using gnu: always_inline]]
    static auto store(std::int64_t* dst, reg_type v) noexcept -> void {
        _mm512_storeu_si512(dst, v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(std::int64_t s) noexcept -> reg_type {
        return _mm512_set1_epi64(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm512_add_epi64(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm512_sub_epi64(a, b);
    }
#if defined(__AVX512DQ__)
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm512_mullo_epi64(a, b);
    }
#endif
    // Two's complement has no sign bit to flip: negate as 0 - x.
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return _mm512_sub_epi64(_mm512_setzero_si512(), v);
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return _mm512_cmpeq_epi64_mask(a, b) == 0xFF;
    }
};

template <>
struct LaneRegister<std::int32_t, 16> final {
    using reg_type = __m512i;
    static constexpr bool kAvailable{true};
    static constexpr bool kHasMul{true};

    [[using gnu: always_inline]]
    static auto load(const std::int32_t* src) noexcept -> reg_type {
        return _mm512_loadu_si512(src);
    }
    [[using gnu: always_inline]]
    static auto store(std::int32_t* dst, reg_type v) noexcept -> void {
        _mm512_storeu_si512(dst, v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(std::int32_t s) noexcept -> reg_type {
        return _mm512_set1_epi32(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm512_add_epi32(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm512_sub_epi32(a, b);
    }
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return _mm512_mullo_epi32(a, b);
    }
    // Two's complement has no sign bit to flip: negate as 0 - x.
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return _mm512_sub_epi32(_mm512_setzero_si512(), v);
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return _mm512_cmpeq_epi32_mask(a, b) == 0xFFFF;
    }
};
#endif

#if defined(__ARM_NEON)
template <>
struct LaneRegister<double, 2> final {
    using reg_type = float64x2_t;
    static constexpr bool kAvailable{true};
    static constexpr bool kHasMul{true};

    [[using gnu: always_inline]]
    static auto load(const double* src) noexcept -> reg_type {
        return vld1q_f64(src);
    }
    [[using gnu: always_inline]]
    static auto store(double* dst, reg_type v) noexcept -> void {
        vst1q_f64(dst, v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(double s) noexcept -> reg_type {
        return vdupq_n_f64(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return vaddq_f64(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return vsubq_f64(a, b);
    }
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return vmulq_f64(a, b);
    }
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return vnegq_f64(v);
    }
    [[using gnu: always_inline]]
    static auto conjugate(reg_type v) noexcept -> reg_type {
        static constexpr std::uint64_t kSign[2]{0ULL, 0x8000000000000000ULL};
        return vreinterpretq_f64_u64(veorq_u64(vreinterpretq_u64_f64(v), vld1q_u64(kSign)));
    }
    // v = (a, b) times (re, im), as (a re - b im, b re + a im).
    [[using gnu: always_inline]]
    static auto mul_complex(reg_type v, double re, double im) noexcept -> reg_type {
        const double im_signed[2]{-im, im};
        return vaddq_f64(vmulq_n_f64(v, re), vmulq_f64(vextq_f64(v, v, 1), vld1q_f64(im_signed)));
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return vminvq_u32(vreinterpretq_u32_u64(vceqq_f64(a, b))) == 0xFFFFFFFFU;
    }
};

template <>
struct LaneRegister<float, 4> final {
    using reg_type = float32x4_t;
    static constexpr bool kAvailable{true};
    static constexpr bool kHasMul{true};

    [[using gnu: always_inline]]
    static auto load(const float* src) noexcept -> reg_type {
        return vld1q_f32(src);
    }
    [[using gnu: always_inline]]
    static auto store(float* dst, reg_type v) noexcept -> void {
        vst1q_f32(dst, v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(float s) noexcept -> reg_type {
        return vdupq_n_f32(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return vaddq_f32(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return vsubq_f32(a, b);
    }
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return vmulq_f32(a, b);
    }
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return vnegq_f32(v);
    }
    [[using gnu: always_inline]]
    static auto conjugate(reg_type v) noexcept -> reg_type {
        static constexpr std::uint32_t kSign[4]{0U, 0x80000000U, 0U, 0x80000000U};
        return vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(v), vld1q_u32(kSign)));
    }
    // Each complex pair v = (a, b) times (re, im), as (a re - b im, b re + a im).
    [[using gnu: always_inline]]
    static auto mul_complex(reg_type v, float re, float im) noexcept -> reg_type {
        const float im_signed[4]{-im, im, -im, im};
        return vaddq_f32(vmulq_n_f32(v, re), vmulq_f32(vrev64q_f32(v), vld1q_f32(im_signed)));
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return vminvq_u32(vceqq_f32(a, b)) == 0xFFFFFFFFU;
    }
};

template <>
struct LaneRegister<std::int64_t, 2> final {
    using reg_type = int64x2_t;
    static constexpr bool kAvailable{true};
    // NEON has no 64-bit lane multiply.
    static constexpr bool kHasMul{false};

    [[using gnu: always_inline]]
    static auto load(const std::int64_t* src) noexcept -> reg_type {
        return vld1q_s64(src);
    }
    [[using gnu: always_inline]]
    static auto store(std::int64_t* dst, reg_type v) noexcept -> void {
        vst1q_s64(dst, v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(std::int64_t s) noexcept -> reg_type {
        return vdupq_n_s64(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return vaddq_s64(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return vsubq_s64(a, b);
    }
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return vnegq_s64(v);
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return vminvq_u32(vreinterpretq_u32_u64(vceqq_s64(a, b))) == 0xFFFFFFFFU;
    }
};

template <>
struct LaneRegister<float, 2> final {
    using reg_type = float32x2_t;
    static constexpr bool kAvailable{true};
    static constexpr bool kHasMul{true};

    [[using gnu: always_inline]]
    static auto load(const float* src) noexcept -> reg_type {
        return vld1_f32(src);
    }
    [[using gnu: always_inline]]
    static auto store(float* dst, reg_type v) noexcept -> void {
        vst1_f32(dst, v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(float s) noexcept -> reg_type {
        return vdup_n_f32(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return vadd_f32(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return vsub_f32(a, b);
    }
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return vmul_f32(a, b);
    }
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return vneg_f32(v);
    }
    [[using gnu: always_inline]]
    static auto conjugate(reg_type v) noexcept -> reg_type {
        static constexpr std::uint32_t kSign[2]{0U, 0x80000000U};
        return vreinterpret_f32_u32(veor_u32(vreinterpret_u32_f32(v), vld1_u32(kSign)));
    }
    // v = (a, b) times (re, im), as (a re - b im, b re + a im).
    [[using gnu: always_inline]]
    static auto mul_complex(reg_type v, float re, float im) noexcept -> reg_type {
        const float im_signed[2]{-im, im};
        return vadd_f32(vmul_n_f32(v, re), vmul_f32(vrev64_f32(v), vld1_f32(im_signed)));
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return vminv_u32(vceq_f32(a, b)) == 0xFFFFFFFFU;
    }
};

template <>
struct LaneRegister<std::int32_t, 2> final {
    using reg_type = int32x2_t;
    static constexpr bool kAvailable{true};
    static constexpr bool kHasMul{true};

    [[using gnu: always_inline]]
    static auto load(const std::int32_t* src) noexcept -> reg_type {
        return vld1_s32(src);
    }
    [[using gnu: always_inline]]
    static auto store(std::int32_t* dst, reg_type v) noexcept -> void {
        vst1_s32(dst, v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(std::int32_t s) noexcept -> reg_type {
        return vdup_n_s32(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return vadd_s32(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return vsub_s32(a, b);
    }
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return vmul_s32(a, b);
    }
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return vneg_s32(v);
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return vminv_u32(vceq_s32(a, b)) == 0xFFFFFFFFU;
    }
};

template <>
struct LaneRegister<std::int32_t, 4> final {
    using reg_type = int32x4_t;
    static constexpr bool kAvailable{true};
    static constexpr bool kHasMul{true};

    [[using gnu: always_inline]]
    static auto load(const std::int32_t* src) noexcept -> reg_type {
        return vld1q_s32(src);
    }
    [[using gnu: always_inline]]
    static auto store(std::int32_t* dst, reg_type v) noexcept -> void {
        vst1q_s32(dst, v);
    }
    [[using gnu: always_inline]]
    static auto broadcast(std::int32_t s) noexcept -> reg_type {
        return vdupq_n_s32(s);
    }
    [[using gnu: always_inline]]
    static auto add(reg_type a, reg_type b) noexcept -> reg_type {
        return vaddq_s32(a, b);
    }
    [[using gnu: always_inline]]
    static auto sub(reg_type a, reg_type b) noexcept -> reg_type {
        return vsubq_s32(a, b);
    }
    [[using gnu: always_inline]]
    static auto mul(reg_type a, reg_type b) noexcept -> reg_type {
        return vmulq_s32(a, b);
    }
    [[using gnu: always_inline]]
    static auto negate(reg_type v) noexcept -> reg_type {
        return vnegq_s32(v);
    }
    [[using gnu: always_inline]]
    static auto all_equal(reg_type a, reg_type b) noexcept -> bool {
        return vminvq_u32(vceqq_s32(a, b)) == 0xFFFFFFFFU;
    }
};
#endif

} // namespace basjoo::math::detail

namespace basjoo::math {

// Element-wise kernels on n contiguous elements of Scalar, named after the C++26 std::linalg
// algorithms where one exists; reductions and products are left to BLAS and the small types. A
// complex element spans two lanes, real then imaginary, and an unsigned integer shares the lanes of
// its signed twin. The widest register available sweeps the lanes, ever narrower ones take what is
// left, and scalar code the rest, which is all that runs in constant evaluation. No alignment is
// required, and an output may alias an input element for element, as in std::linalg.
template <typename Scalar>
struct DenseLanes final {
    using real_type = detail::DenseNormTraitT<Scalar>;

    // z = x + y
    [[using gnu: always_inline, hot]]
    static constexpr auto add(std::size_t n, const Scalar* x, const Scalar* y, Scalar* z) noexcept
        -> void {
        std::size_t k{0};
        if !consteval {
            constexpr std::size_t kTop{kWidest<false>};
            if constexpr (kTop > 1) {
                using R = detail::LaneRegister<lane_type, kTop>;
                const std::size_t m{n * kLanes};
                const auto* const a{reinterpret_cast<const lane_type*>(x)};
                const auto* const b{reinterpret_cast<const lane_type*>(y)};
                auto* const c{reinterpret_cast<lane_type*>(z)};
                std::size_t i{0};
                for (; i + 4 * kTop <= m; i += 4 * kTop) {
                    R::store(c + i, R::add(R::load(a + i), R::load(b + i)));
                    R::store(c + i + kTop, R::add(R::load(a + i + kTop), R::load(b + i + kTop)));
                    R::store(
                        c + i + 2 * kTop,
                        R::add(R::load(a + i + 2 * kTop), R::load(b + i + 2 * kTop))
                    );
                    R::store(
                        c + i + 3 * kTop,
                        R::add(R::load(a + i + 3 * kTop), R::load(b + i + 3 * kTop))
                    );
                }
                for (; i + kTop <= m; i += kTop) {
                    R::store(c + i, R::add(R::load(a + i), R::load(b + i)));
                }
                // Each narrower register takes one block at most.
                if constexpr (kEligible<kTop / 2, false>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 2>;
                    if (i + kTop / 2 <= m) {
                        Q::store(c + i, Q::add(Q::load(a + i), Q::load(b + i)));
                        i += kTop / 2;
                    }
                }
                if constexpr (kEligible<kTop / 4, false>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 4>;
                    if (i + kTop / 4 <= m) {
                        Q::store(c + i, Q::add(Q::load(a + i), Q::load(b + i)));
                        i += kTop / 4;
                    }
                }
                if constexpr (kEligible<kTop / 8, false>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 8>;
                    if (i + kTop / 8 <= m) {
                        Q::store(c + i, Q::add(Q::load(a + i), Q::load(b + i)));
                        i += kTop / 8;
                    }
                }
                k = i / kLanes;
            }
        }
        for (; k < n; ++k) {
            z[k] = static_cast<Scalar>(x[k] + y[k]);
        }
        return;
    }

    // z = x - y
    [[using gnu: always_inline, hot]]
    static constexpr auto subtract(
        std::size_t n, const Scalar* x, const Scalar* y, Scalar* z
    ) noexcept -> void {
        std::size_t k{0};
        if !consteval {
            constexpr std::size_t kTop{kWidest<false>};
            if constexpr (kTop > 1) {
                using R = detail::LaneRegister<lane_type, kTop>;
                const std::size_t m{n * kLanes};
                const auto* const a{reinterpret_cast<const lane_type*>(x)};
                const auto* const b{reinterpret_cast<const lane_type*>(y)};
                auto* const c{reinterpret_cast<lane_type*>(z)};
                std::size_t i{0};
                for (; i + 4 * kTop <= m; i += 4 * kTop) {
                    R::store(c + i, R::sub(R::load(a + i), R::load(b + i)));
                    R::store(c + i + kTop, R::sub(R::load(a + i + kTop), R::load(b + i + kTop)));
                    R::store(
                        c + i + 2 * kTop,
                        R::sub(R::load(a + i + 2 * kTop), R::load(b + i + 2 * kTop))
                    );
                    R::store(
                        c + i + 3 * kTop,
                        R::sub(R::load(a + i + 3 * kTop), R::load(b + i + 3 * kTop))
                    );
                }
                for (; i + kTop <= m; i += kTop) {
                    R::store(c + i, R::sub(R::load(a + i), R::load(b + i)));
                }
                // Each narrower register takes one block at most.
                if constexpr (kEligible<kTop / 2, false>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 2>;
                    if (i + kTop / 2 <= m) {
                        Q::store(c + i, Q::sub(Q::load(a + i), Q::load(b + i)));
                        i += kTop / 2;
                    }
                }
                if constexpr (kEligible<kTop / 4, false>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 4>;
                    if (i + kTop / 4 <= m) {
                        Q::store(c + i, Q::sub(Q::load(a + i), Q::load(b + i)));
                        i += kTop / 4;
                    }
                }
                if constexpr (kEligible<kTop / 8, false>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 8>;
                    if (i + kTop / 8 <= m) {
                        Q::store(c + i, Q::sub(Q::load(a + i), Q::load(b + i)));
                        i += kTop / 8;
                    }
                }
                k = i / kLanes;
            }
        }
        for (; k < n; ++k) {
            z[k] = static_cast<Scalar>(x[k] - y[k]);
        }
        return;
    }

    // x = -x
    [[using gnu: always_inline, hot]]
    static constexpr auto negate(std::size_t n, Scalar* x) noexcept -> void {
        std::size_t k{0};
        if !consteval {
            constexpr std::size_t kTop{kWidest<false>};
            if constexpr (kTop > 1) {
                using R = detail::LaneRegister<lane_type, kTop>;
                const std::size_t m{n * kLanes};
                auto* const a{reinterpret_cast<lane_type*>(x)};
                std::size_t i{0};
                for (; i + 4 * kTop <= m; i += 4 * kTop) {
                    R::store(a + i, R::negate(R::load(a + i)));
                    R::store(a + i + kTop, R::negate(R::load(a + i + kTop)));
                    R::store(a + i + 2 * kTop, R::negate(R::load(a + i + 2 * kTop)));
                    R::store(a + i + 3 * kTop, R::negate(R::load(a + i + 3 * kTop)));
                }
                for (; i + kTop <= m; i += kTop) {
                    R::store(a + i, R::negate(R::load(a + i)));
                }
                // Each narrower register takes one block at most.
                if constexpr (kEligible<kTop / 2, false>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 2>;
                    if (i + kTop / 2 <= m) {
                        Q::store(a + i, Q::negate(Q::load(a + i)));
                        i += kTop / 2;
                    }
                }
                if constexpr (kEligible<kTop / 4, false>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 4>;
                    if (i + kTop / 4 <= m) {
                        Q::store(a + i, Q::negate(Q::load(a + i)));
                        i += kTop / 4;
                    }
                }
                if constexpr (kEligible<kTop / 8, false>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 8>;
                    if (i + kTop / 8 <= m) {
                        Q::store(a + i, Q::negate(Q::load(a + i)));
                        i += kTop / 8;
                    }
                }
                k = i / kLanes;
            }
        }
        for (; k < n; ++k) {
            x[k] = static_cast<Scalar>(-x[k]);
        }
        return;
    }

    // x = alpha x, for a real alpha: every lane is scaled.
    [[using gnu: always_inline, hot]]
    static constexpr auto scale(std::size_t n, real_type alpha, Scalar* x) noexcept -> void {
        std::size_t k{0};
        if !consteval {
            constexpr std::size_t kTop{kWidest<true>};
            if constexpr (kTop > 1) {
                using R = detail::LaneRegister<lane_type, kTop>;
                const std::size_t m{n * kLanes};
                auto* const a{reinterpret_cast<lane_type*>(x)};
                const auto factor{static_cast<lane_type>(alpha)};
                const auto f{R::broadcast(factor)};
                std::size_t i{0};
                for (; i + 4 * kTop <= m; i += 4 * kTop) {
                    R::store(a + i, R::mul(R::load(a + i), f));
                    R::store(a + i + kTop, R::mul(R::load(a + i + kTop), f));
                    R::store(a + i + 2 * kTop, R::mul(R::load(a + i + 2 * kTop), f));
                    R::store(a + i + 3 * kTop, R::mul(R::load(a + i + 3 * kTop), f));
                }
                for (; i + kTop <= m; i += kTop) {
                    R::store(a + i, R::mul(R::load(a + i), f));
                }
                // Each narrower register takes one block at most.
                if constexpr (kEligible<kTop / 2, true>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 2>;
                    if (i + kTop / 2 <= m) {
                        Q::store(a + i, Q::mul(Q::load(a + i), Q::broadcast(factor)));
                        i += kTop / 2;
                    }
                }
                if constexpr (kEligible<kTop / 4, true>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 4>;
                    if (i + kTop / 4 <= m) {
                        Q::store(a + i, Q::mul(Q::load(a + i), Q::broadcast(factor)));
                        i += kTop / 4;
                    }
                }
                if constexpr (kEligible<kTop / 8, true>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 8>;
                    if (i + kTop / 8 <= m) {
                        Q::store(a + i, Q::mul(Q::load(a + i), Q::broadcast(factor)));
                        i += kTop / 8;
                    }
                }
                k = i / kLanes;
            }
        }
        for (; k < n; ++k) {
            x[k] = static_cast<Scalar>(x[k] * alpha);
        }
        return;
    }

    // x = alpha x, for a complex alpha.
    [[using gnu: always_inline, hot]]
    static constexpr auto scale(std::size_t n, Scalar alpha, Scalar* x) noexcept -> void
        requires ComplexArithmetic<Scalar>
    {
        const real_type re{alpha.real()};
        const real_type im{alpha.imag()};
        std::size_t k{0};
        if !consteval {
            constexpr std::size_t kTop{kWidest<true>};
            if constexpr (kTop > 1) {
                using R = detail::LaneRegister<lane_type, kTop>;
                const std::size_t m{n * kLanes};
                auto* const a{reinterpret_cast<lane_type*>(x)};
                std::size_t i{0};
                for (; i + 4 * kTop <= m; i += 4 * kTop) {
                    R::store(a + i, R::mul_complex(R::load(a + i), re, im));
                    R::store(a + i + kTop, R::mul_complex(R::load(a + i + kTop), re, im));
                    R::store(a + i + 2 * kTop, R::mul_complex(R::load(a + i + 2 * kTop), re, im));
                    R::store(a + i + 3 * kTop, R::mul_complex(R::load(a + i + 3 * kTop), re, im));
                }
                for (; i + kTop <= m; i += kTop) {
                    R::store(a + i, R::mul_complex(R::load(a + i), re, im));
                }
                // Each narrower register takes one block at most.
                if constexpr (kEligible<kTop / 2, true>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 2>;
                    if (i + kTop / 2 <= m) {
                        Q::store(a + i, Q::mul_complex(Q::load(a + i), re, im));
                        i += kTop / 2;
                    }
                }
                if constexpr (kEligible<kTop / 4, true>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 4>;
                    if (i + kTop / 4 <= m) {
                        Q::store(a + i, Q::mul_complex(Q::load(a + i), re, im));
                        i += kTop / 4;
                    }
                }
                if constexpr (kEligible<kTop / 8, true>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 8>;
                    if (i + kTop / 8 <= m) {
                        Q::store(a + i, Q::mul_complex(Q::load(a + i), re, im));
                        i += kTop / 8;
                    }
                }
                k = i / kLanes;
            }
        }
        // The product of the lanes in real arithmetic: Clang cannot evaluate the std::complex one
        // of libstdc++ in a constant expression.
        for (; k < n; ++k) {
            x[k] = Scalar{
                x[k].real() * alpha.real() - x[k].imag() * alpha.imag(),
                x[k].imag() * alpha.real() + x[k].real() * alpha.imag()
            };
        }
        return;
    }

    // x = conj(x): the imaginary lanes are negated.
    [[using gnu: always_inline, hot]]
    static constexpr auto conjugate(std::size_t n, Scalar* x) noexcept -> void
        requires ComplexArithmetic<Scalar>
    {
        std::size_t k{0};
        if !consteval {
            constexpr std::size_t kTop{kWidest<false>};
            if constexpr (kTop > 1) {
                using R = detail::LaneRegister<lane_type, kTop>;
                const std::size_t m{n * kLanes};
                auto* const a{reinterpret_cast<lane_type*>(x)};
                std::size_t i{0};
                for (; i + 4 * kTop <= m; i += 4 * kTop) {
                    R::store(a + i, R::conjugate(R::load(a + i)));
                    R::store(a + i + kTop, R::conjugate(R::load(a + i + kTop)));
                    R::store(a + i + 2 * kTop, R::conjugate(R::load(a + i + 2 * kTop)));
                    R::store(a + i + 3 * kTop, R::conjugate(R::load(a + i + 3 * kTop)));
                }
                for (; i + kTop <= m; i += kTop) {
                    R::store(a + i, R::conjugate(R::load(a + i)));
                }
                // Each narrower register takes one block at most.
                if constexpr (kEligible<kTop / 2, false>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 2>;
                    if (i + kTop / 2 <= m) {
                        Q::store(a + i, Q::conjugate(Q::load(a + i)));
                        i += kTop / 2;
                    }
                }
                if constexpr (kEligible<kTop / 4, false>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 4>;
                    if (i + kTop / 4 <= m) {
                        Q::store(a + i, Q::conjugate(Q::load(a + i)));
                        i += kTop / 4;
                    }
                }
                if constexpr (kEligible<kTop / 8, false>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 8>;
                    if (i + kTop / 8 <= m) {
                        Q::store(a + i, Q::conjugate(Q::load(a + i)));
                        i += kTop / 8;
                    }
                }
                k = i / kLanes;
            }
        }
        for (; k < n; ++k) {
            x[k] = Scalar{x[k].real(), -x[k].imag()};
        }
        return;
    }

    [[using gnu: pure, always_inline, hot]]
    static constexpr auto equal(std::size_t n, const Scalar* x, const Scalar* y) noexcept -> bool {
        std::size_t k{0};
        if !consteval {
            constexpr std::size_t kTop{kWidest<false>};
            if constexpr (kTop > 1) {
                using R = detail::LaneRegister<lane_type, kTop>;
                const std::size_t m{n * kLanes};
                const auto* const a{reinterpret_cast<const lane_type*>(x)};
                const auto* const b{reinterpret_cast<const lane_type*>(y)};
                // 0 or 1: Clang renormalizes a bool accumulator after every block.
                unsigned same{1U};
                std::size_t i{0};
                for (; i + 4 * kTop <= m; i += 4 * kTop) {
                    same &= static_cast<unsigned>(R::all_equal(R::load(a + i), R::load(b + i)));
                    same &= static_cast<unsigned>(
                        R::all_equal(R::load(a + i + kTop), R::load(b + i + kTop))
                    );
                    same &= static_cast<unsigned>(
                        R::all_equal(R::load(a + i + 2 * kTop), R::load(b + i + 2 * kTop))
                    );
                    same &= static_cast<unsigned>(
                        R::all_equal(R::load(a + i + 3 * kTop), R::load(b + i + 3 * kTop))
                    );
                }
                for (; i + kTop <= m; i += kTop) {
                    same &= static_cast<unsigned>(R::all_equal(R::load(a + i), R::load(b + i)));
                }
                // Each narrower register takes one block at most.
                if constexpr (kEligible<kTop / 2, false>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 2>;
                    if (i + kTop / 2 <= m) {
                        same &= static_cast<unsigned>(Q::all_equal(Q::load(a + i), Q::load(b + i)));
                        i += kTop / 2;
                    }
                }
                if constexpr (kEligible<kTop / 4, false>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 4>;
                    if (i + kTop / 4 <= m) {
                        same &= static_cast<unsigned>(Q::all_equal(Q::load(a + i), Q::load(b + i)));
                        i += kTop / 4;
                    }
                }
                if constexpr (kEligible<kTop / 8, false>) {
                    using Q = detail::LaneRegister<lane_type, kTop / 8>;
                    if (i + kTop / 8 <= m) {
                        same &= static_cast<unsigned>(Q::all_equal(Q::load(a + i), Q::load(b + i)));
                        i += kTop / 8;
                    }
                }
                if (same == 0U) {
                    return false;
                }
                k = i / kLanes;
            }
        }
        for (; k < n; ++k) {
            if (!(x[k] == y[k])) {
                return false;
            }
        }
        return true;
    }

  private:
    // The SIMD lane standing for the real type; an unsigned integer shares the lanes of its signed
    // twin.
    using lane_type = std::conditional_t<
        std::is_same_v<real_type, std::int64_t> || std::is_same_v<real_type, std::uint64_t>,
        std::int64_t,
        std::conditional_t<
            std::is_same_v<real_type, std::int32_t> || std::is_same_v<real_type, std::uint32_t>,
            std::int32_t, real_type>>;

    // Lanes per element.
    static constexpr std::size_t kLanes{ComplexArithmetic<Scalar> ? 2 : 1};

    template <std::size_t W, bool NeedMul>
    static constexpr bool kEligible{
        detail::LaneRegister<lane_type, W>::kAvailable &&
        (!NeedMul || detail::LaneRegister<lane_type, W>::kHasMul)
    };

    template <bool NeedMul>
    static constexpr std::size_t kWidest{
        kEligible<16, NeedMul>  ? 16
        : kEligible<8, NeedMul> ? 8
        : kEligible<4, NeedMul> ? 4
        : kEligible<2, NeedMul> ? 2
                                : 1
    };
};

} // namespace basjoo::math
