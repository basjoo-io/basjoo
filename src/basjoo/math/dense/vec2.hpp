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
#include <cstddef>
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
#include "basjoo/math/type_traits.hpp"

namespace basjoo::math {

template <ScalarArithmetic Scalar, std::integral Index, Allocatory Alloc>
class alignas(2 * sizeof(Scalar)) Vector<Scalar, Index, 2, Alloc> final {
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
    constexpr Vector() noexcept = default;

    [[using gnu: always_inline]]
    constexpr explicit Vector([[maybe_unused]] const allocator_type& alloc) noexcept
        : x{}, y{} {}

    // `count` must be exactly size_type: a looser parameter would make single-scalar broadcast
    // and per-component construction ambiguous. On a tie the non-template constructors win.
    [[using gnu: always_inline]]
    constexpr explicit Vector(
        [[maybe_unused]] std::same_as<size_type> auto count,
        [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : x{}, y{} {
#if BASJOO_CHECK_PARAMS == 1
        if (count != size()) [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
    }

    [[using gnu: always_inline]]
    constexpr explicit Vector(
        [[maybe_unused]] std::same_as<size_type> auto count, const_reference value,
        [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : x{value}, y{value} {
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
        : x{}, y{} {
#if BASJOO_CHECK_PARAMS == 1
        if (std::ranges::size(rg) != size()) [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
        auto it{std::ranges::begin(rg)};
        x = *it;
        ++it;
        y = *it;
    }

    template <typename T>
        requires std::same_as<std::remove_cv_t<typename T::value_type>, value_type> &&
                     (T::rank() == 2)
    [[using gnu: always_inline]] constexpr explicit Vector(
        const T& matrix_like, [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : x{}, y{} {
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
        : x{static_cast<value_type>(other.first)}, y{static_cast<value_type>(other.second)} {}

    template <ScalarArithmetic U>
    [[using gnu: always_inline]]
    constexpr explicit Vector(const Vector<U, Index, 2>& other) noexcept
        : x{static_cast<value_type>(other.x)}, y{static_cast<value_type>(other.y)} {}

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
        // The members are exposed as a contiguous array of 2 elements.
        static_assert(std::is_standard_layout_v<Vector>);
        static_assert(offsetof(Vector, y) == 1 * sizeof(value_type));
        return &self.x;
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto data_handle(this Self&& self) noexcept {
        return self.data();
    }

    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator[](this Self&& self, index_type i) noexcept -> decltype(auto) {
        return self.data()[i];
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto coeff(index_type i) const noexcept(!BASJOO_CHECK_PARAMS) -> value_type {
#if BASJOO_CHECK_PARAMS == 1
        if (static_cast<size_type>(i) >= size()) [[unlikely]] {
            throw std::out_of_range("Vector index out of range");
        }
#endif
        return data()[i];
    }

    [[using gnu: always_inline, hot]]
    constexpr auto updateCoeff(index_type i, const_reference value) noexcept(!BASJOO_CHECK_PARAMS)
        -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (static_cast<size_type>(i) >= size()) [[unlikely]] {
            throw std::out_of_range("Vector index out of range");
        }
#endif
        data()[i] = value;
        return;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto empty() noexcept -> bool {
        return false;
    }

    [[using gnu: always_inline, hot]]
    constexpr auto fill(const_reference value) noexcept -> void {
        x = value;
        y = value;
        return;
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

    template <std::size_t Count, typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto first(this Self&& self) noexcept {
        static_assert(Count <= 2, "Vector::first: count out of range.");
        using element_type = std::remove_pointer_t<decltype(self.data())>;

        return std::span<element_type, Count>{self.data(), Count};
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto first(this Self&& self, size_type count) noexcept(!BASJOO_CHECK_PARAMS) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;

#if BASJOO_CHECK_PARAMS == 1
        if (count > 2) [[unlikely]] {
            throw std::out_of_range("Vector::first: count out of range.");
        }
#endif
        return std::span<element_type>{self.data(), count};
    }

    template <std::size_t Count, typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto last(this Self&& self) noexcept {
        static_assert(Count <= 2, "Vector::last: count out of range.");
        using element_type = std::remove_pointer_t<decltype(self.data())>;

        return std::span<element_type, Count>{self.data() + (2 - Count), Count};
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto last(this Self&& self, size_type count) noexcept(!BASJOO_CHECK_PARAMS) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;

#if BASJOO_CHECK_PARAMS == 1
        if (count > 2) [[unlikely]] {
            throw std::out_of_range("Vector::last: count out of range.");
        }
#endif
        return std::span<element_type>{self.data() + (2 - count), count};
    }

    template <std::size_t Offset, std::size_t Count = std::dynamic_extent, typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto subspan(this Self&& self) noexcept {
        static_assert(
            Offset <= 2 && (Count == std::dynamic_extent || Count <= 2 - Offset),
            "Vector::subspan: invalid range."
        );
        using element_type = std::remove_pointer_t<decltype(self.data())>;
        constexpr std::size_t kCount{Count != std::dynamic_extent ? Count : 2 - Offset};

        return std::span<element_type, kCount>{self.data() + Offset, kCount};
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto subspan(this Self&& self, size_type offset) noexcept(!BASJOO_CHECK_PARAMS) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;

#if BASJOO_CHECK_PARAMS == 1
        if (offset > 2) [[unlikely]] {
            throw std::out_of_range("Vector::subspan: invalid range.");
        }
#endif
        return std::span<element_type>{self.data() + offset, 2 - offset};
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto subspan(this Self&& self, size_type offset, size_type count) noexcept(
        !BASJOO_CHECK_PARAMS
    ) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;

#if BASJOO_CHECK_PARAMS == 1
        if (offset > 2 || count > 2 - offset) [[unlikely]] {
            throw std::out_of_range("Vector::subspan: invalid range.");
        }
#endif
        return std::span<element_type>{self.data() + offset, count};
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
    template <typename Self, typename Obj>
        requires std::same_as<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>>
    [[using gnu: always_inline, hot]]
    constexpr auto operator+(this Self&& self, Obj&& obj) noexcept -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self += obj;
            return static_cast<Self&&>(self);
        } else if constexpr (Reusable<Obj>) {
            obj += self;
            return static_cast<Obj&&>(obj);
        } else {
            Vector result{self};
            result += obj;
            return result;
        }
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
    template <typename Self, typename Obj>
        requires std::same_as<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>>
    [[using gnu: always_inline, hot]]
    constexpr auto operator-(this Self&& self, Obj&& obj) noexcept -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self -= obj;
            return static_cast<Self&&>(self);
        } else if constexpr (Reusable<Obj>) {
            // self - obj == -(obj - self), computed in obj's storage
            obj -= self;
            return -static_cast<Obj&&>(obj);
        } else {
            Vector result{self};
            result -= obj;
            return result;
        }
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
    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto operator*(this Self&& self, const ScalarArithmetic auto& fac) noexcept
        -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self *= fac;
            return static_cast<Self&&>(self);
        } else {
            Vector result{self};
            result *= fac;
            return result;
        }
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
    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto operator/(this Self&& self, const ScalarArithmetic auto& den) noexcept
        -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self /= den;
            return static_cast<Self&&>(self);
        } else {
            Vector result{self};
            result /= den;
            return result;
        }
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto operator-(this Self&& self) noexcept -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self.selfNegate();
            return static_cast<Self&&>(self);
        } else {
            Vector result{self};
            result.selfNegate();
            return result;
        }
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

    template <std::size_t OuterSize, DenseLayout ObjLayout>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(
        const Matrix<value_type, Index, 2, OuterSize, ObjLayout, Alloc>& obj
    ) const noexcept -> Vector<value_type, Index, OuterSize, Alloc> {
        using result_type = Vector<value_type, Index, OuterSize, Alloc>;

        // Vector<Scalar, Index, 2/3> take a broadcast scalar, other sizes a (count, value) pair.
        result_type result{[] {
            if constexpr (OuterSize == 2 || OuterSize == 3) {
                return result_type(value_type(0.0));
            } else {
                return result_type(OuterSize, value_type(0.0));
            }
        }()};
        for (size_type j{0}; j < OuterSize; ++j) {
            for (size_type i{0}; i < size(); ++i) {
                result[j] += operator[](i) * obj[static_cast<Index>(i), static_cast<Index>(j)];
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

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto normalized(this Self&& self) noexcept -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self /= self.euclidean();
            return static_cast<Self&&>(self);
        } else {
            return self / self.euclidean();
        }
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
    constexpr auto identicalTo(
        const Vector& obj, detail::DenseNormTraitT<value_type> tol = 1E-8
    ) const noexcept -> bool {
        detail::DenseNormTraitT<value_type> test(0.0);
        for (size_type i{0}; i < size(); ++i) {
            test = std::max(
                test, std::abs(data()[i] - obj.data()[i]) /
                          std::max(std::abs(data()[i]), static_cast<decltype(test)>(1.0))
            );
        }
        return test < tol;
    }
    [[using gnu: pure, always_inline, hot]]
    constexpr auto orthogonalTo(
        const Vector& obj, detail::DenseNormTraitT<value_type> tol = 1E-8
    ) const noexcept -> bool {
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
    template <typename Self>
    [[using gnu: always_inline]]
    constexpr auto conjugated(this Self&& self) noexcept -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
                self.selfConjugated();
            }
            return static_cast<Self&&>(self);
        } else {
            Vector result{self};
            if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
                result.selfConjugated();
            }
            return result;
        }
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
        *this = Vector{-y, x};
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
    [[using gnu: always_inline, hot]]
    constexpr auto selfNegate() noexcept -> void {
        if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__SSE2__)
            // Two's complement has no sign bit to flip: negate as 0 - x.
            _mm_store_si128(
                reinterpret_cast<__m128i*>(&x),
                _mm_sub_epi64(
                    _mm_setzero_si128(), _mm_load_si128(reinterpret_cast<const __m128i*>(&x))
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
        return;
    }

  public:
    using serialize = zpp::bits::members<2>;

    value_type x, y;
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

template <typename T>
    requires Vec2Arithmetic<std::remove_cvref_t<T>>
[[using gnu: always_inline, hot]]
inline constexpr auto operator*(const ScalarArithmetic auto& fac, T&& obj) noexcept
    -> decltype(auto) {
    return std::forward<T>(obj) * fac;
}

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
    return std::hypot(obj.x, obj.y);
}

template <::basjoo::math::Vec2Arithmetic V>
[[using gnu: pure, always_inline]]
inline constexpr auto abs(const V& obj) noexcept
    -> ::basjoo::math::detail::DenseNormTraitT<typename V::value_type> {
    return obj.euclidean();
}

template <::basjoo::math::Vec2Arithmetic V>
[[using gnu: pure, always_inline]]
inline constexpr auto norm(const V& obj) noexcept
    -> ::basjoo::math::detail::DenseNormTraitT<typename V::value_type> {
    return obj.euclideanSqr();
}

template <typename T>
    requires ::basjoo::math::Vec2Arithmetic<std::remove_cvref_t<T>>
[[using gnu: always_inline]]
inline constexpr auto conj(T&& vector) noexcept -> decltype(auto) {
    return std::forward<T>(vector).conjugated();
}

template <::basjoo::math::Vec2Arithmetic V>
    requires std::floating_point<typename V::value_type>
[[using gnu: pure, always_inline]]
inline constexpr auto atan2(const V& obj) noexcept -> typename V::value_type {
    return std::atan2(obj.y, obj.x);
}

} // namespace std

// NOLINTEND(cert-dcl58-cpp)
