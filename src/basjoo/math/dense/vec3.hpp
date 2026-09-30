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

#include "basjoo/common/in_in_in_out_result.hpp"
#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_trait.hpp"
#include "basjoo/math/dense/detail/dense_norm_trait.hpp"
#include "basjoo/math/type_traits.hpp"

namespace basjoo::math {

template <ScalarArithmetic Scalar, std::integral Index, Allocatory Alloc>
class alignas(4 * sizeof(Scalar)) Vector<Scalar, Index, 3, Alloc> final {
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
        : x{cv}, y{cv}, z{cv} {}
    [[using gnu: always_inline]]
    constexpr Vector(value_type cx, value_type cy, value_type cz) noexcept
        : x{cx}, y{cy}, z{cz} {}
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
    auto get_allocator() const noexcept -> allocator_type {
        return allocator_type{};
    }

    [[using gnu: always_inline]]
    constexpr Vector() noexcept = default;

    [[using gnu: always_inline]]
    constexpr explicit Vector([[maybe_unused]] const allocator_type& alloc) noexcept
        : x{}, y{}, z{} {}

    // `count` must be exactly size_type: a looser parameter would make single-scalar broadcast
    // and per-component construction ambiguous. On a tie the non-template constructors win.
    [[using gnu: always_inline]]
    constexpr explicit Vector(
        [[maybe_unused]] std::same_as<size_type> auto count,
        [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : x{}, y{}, z{} {
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
        : x{value}, y{value}, z{value} {
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
        : x{}, y{}, z{} {
#if BASJOO_CHECK_PARAMS == 1
        if (std::ranges::size(rg) != size()) [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
        auto it{std::ranges::begin(rg)};
        x = *it;
        ++it;
        y = *it;
        ++it;
        z = *it;
    }

    template <typename T>
        requires std::same_as<std::remove_cv_t<typename T::value_type>, value_type> &&
                     (T::rank() == 2)
    [[using gnu: always_inline]] constexpr explicit Vector(
        const T& matrix_like, [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : x{}, y{}, z{} {
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
    constexpr explicit Vector(const std::tuple<U, U, U>& other) noexcept
        : x{static_cast<value_type>(std::get<0>(other))},
          y{static_cast<value_type>(std::get<1>(other))},
          z{static_cast<value_type>(std::get<2>(other))} {}

    template <ScalarArithmetic U>
    [[using gnu: always_inline]]
    constexpr explicit Vector(const Vector<U, Index, 3>& other) noexcept
        : x{static_cast<value_type>(other.x)}, y{static_cast<value_type>(other.y)},
          z{static_cast<value_type>(other.z)} {}

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto size() noexcept -> size_type {
        return 3;
    }
    [[using gnu: const, always_inline, leaf]]
    static constexpr auto stride([[maybe_unused]] rank_type r = 0) noexcept -> index_type {
        return 1;
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto data(this Self&& self) noexcept {
        // The members are exposed as a contiguous array of 3 elements.
        static_assert(std::is_standard_layout_v<Vector>);
        static_assert(
            offsetof(Vector, y) == 1 * sizeof(value_type) &&
            offsetof(Vector, z) == 2 * sizeof(value_type)
        );
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
        z = value;
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

    template <std::size_t Count, typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto first(this Self&& self) noexcept {
        static_assert(Count <= 3, "Vector::first: count out of range.");
        using element_type = std::remove_pointer_t<decltype(self.data())>;

        return std::span<element_type, Count>{self.data(), Count};
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto first(this Self&& self, size_type count) noexcept(!BASJOO_CHECK_PARAMS) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;

#if BASJOO_CHECK_PARAMS == 1
        if (count > 3) [[unlikely]] {
            throw std::out_of_range("Vector::first: count out of range.");
        }
#endif
        return std::span<element_type>{self.data(), count};
    }

    template <std::size_t Count, typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto last(this Self&& self) noexcept {
        static_assert(Count <= 3, "Vector::last: count out of range.");
        using element_type = std::remove_pointer_t<decltype(self.data())>;

        return std::span<element_type, Count>{self.data() + (3 - Count), Count};
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto last(this Self&& self, size_type count) noexcept(!BASJOO_CHECK_PARAMS) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;

#if BASJOO_CHECK_PARAMS == 1
        if (count > 3) [[unlikely]] {
            throw std::out_of_range("Vector::last: count out of range.");
        }
#endif
        return std::span<element_type>{self.data() + (3 - count), count};
    }

    template <std::size_t Offset, std::size_t Count = std::dynamic_extent, typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto subspan(this Self&& self) noexcept {
        static_assert(
            Offset <= 3 && (Count == std::dynamic_extent || Count <= 3 - Offset),
            "Vector::subspan: invalid range."
        );
        using element_type = std::remove_pointer_t<decltype(self.data())>;
        constexpr std::size_t kCount{Count != std::dynamic_extent ? Count : 3 - Offset};

        return std::span<element_type, kCount>{self.data() + Offset, kCount};
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto subspan(this Self&& self, size_type offset) noexcept(!BASJOO_CHECK_PARAMS) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;

#if BASJOO_CHECK_PARAMS == 1
        if (offset > 3) [[unlikely]] {
            throw std::out_of_range("Vector::subspan: invalid range.");
        }
#endif
        return std::span<element_type>{self.data() + offset, 3 - offset};
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto subspan(this Self&& self, size_type offset, size_type count) noexcept(
        !BASJOO_CHECK_PARAMS
    ) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;

#if BASJOO_CHECK_PARAMS == 1
        if (offset > 3 || count > 3 - offset) [[unlikely]] {
            throw std::out_of_range("Vector::subspan: invalid range.");
        }
#endif
        return std::span<element_type>{self.data() + offset, count};
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

    [[using gnu: always_inline, leaf, hot]]
    constexpr auto operator+=(const Vector& obj) noexcept -> Vector& {
        if constexpr (
            std::is_same_v<value_type, std::int32_t> || std::is_same_v<value_type, std::uint32_t>
        ) {
#if defined(__SSE2__)
            _mm_store_si128(
                reinterpret_cast<__m128i*>(&x),
                _mm_add_epi32(
                    _mm_load_si128(reinterpret_cast<const __m128i*>(&x)),
                    _mm_load_si128(reinterpret_cast<const __m128i*>(&obj.x))
                )
            );
#elif defined(__ARM_NEON)
            vst1q_s32(
                reinterpret_cast<std::int32_t*>(&x),
                vaddq_s32(
                    vld1q_s32(reinterpret_cast<const std::int32_t*>(&x)),
                    vld1q_s32(reinterpret_cast<const std::int32_t*>(&obj.x))
                )
            );
#else
            x += obj.x;
            y += obj.y;
            z += obj.z;
#endif
        } else if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__AVX2__)
            _mm256_store_si256(
                reinterpret_cast<__m256i*>(&x),
                _mm256_add_epi64(
                    _mm256_load_si256(reinterpret_cast<const __m256i*>(&x)),
                    _mm256_load_si256(reinterpret_cast<const __m256i*>(&obj.x))
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
            vst1q_s64(
                reinterpret_cast<std::int64_t*>(&z),
                vaddq_s64(
                    vld1q_s64(reinterpret_cast<const std::int64_t*>(&z)),
                    vld1q_s64(reinterpret_cast<const std::int64_t*>(&obj.z))
                )
            );
#else
            x += obj.x;
            y += obj.y;
            z += obj.z;
#endif
        } else if constexpr (std::is_same_v<value_type, float>) {
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
            z += obj.z;
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
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
                reinterpret_cast<double*>(&z),
                vaddq_f64(
                    vld1q_f64(reinterpret_cast<const double*>(&z)),
                    vld1q_f64(reinterpret_cast<const double*>(&obj.z))
                )
            );
#else
            x += obj.x;
            y += obj.y;
            z += obj.z;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__AVX__)
            _mm256_store_ps(
                reinterpret_cast<float*>(&x),
                _mm256_add_ps(
                    _mm256_load_ps(reinterpret_cast<const float*>(&x)),
                    _mm256_load_ps(reinterpret_cast<const float*>(&obj.x))
                )
            );
#elif defined(__ARM_NEON)
            vst1q_f32(
                reinterpret_cast<float*>(&x), vaddq_f32(
                                                  vld1q_f32(reinterpret_cast<const float*>(&x)),
                                                  vld1q_f32(reinterpret_cast<const float*>(&obj.x))
                                              )
            );
            vst1q_f32(
                reinterpret_cast<float*>(&z), vaddq_f32(
                                                  vld1q_f32(reinterpret_cast<const float*>(&z)),
                                                  vld1q_f32(reinterpret_cast<const float*>(&obj.z))
                                              )
            );
#else
            x += obj.x;
            y += obj.y;
            z += obj.z;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX512F__)
            _mm512_store_pd(
                reinterpret_cast<double*>(&x),
                _mm512_add_pd(
                    _mm512_load_pd(reinterpret_cast<const double*>(&x)),
                    _mm512_load_pd(reinterpret_cast<const double*>(&obj.x))
                )
            );
#elif defined(__ARM_NEON)
            double* dst{reinterpret_cast<double*>(&x)};
            const double* src{reinterpret_cast<const double*>(&obj.x)};
            vst1q_f64(dst + 0, vaddq_f64(vld1q_f64(dst + 0), vld1q_f64(src + 0)));
            vst1q_f64(dst + 2, vaddq_f64(vld1q_f64(dst + 2), vld1q_f64(src + 2)));
            vst1q_f64(dst + 4, vaddq_f64(vld1q_f64(dst + 4), vld1q_f64(src + 4)));
            vst1q_f64(dst + 6, vaddq_f64(vld1q_f64(dst + 6), vld1q_f64(src + 6)));
#else
            x += obj.x;
            y += obj.y;
            z += obj.z;
#endif
        } else {
            x += obj.x;
            y += obj.y;
            z += obj.z;
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
            std::is_same_v<value_type, std::int32_t> || std::is_same_v<value_type, std::uint32_t>
        ) {
#if defined(__SSE2__)
            _mm_store_si128(
                reinterpret_cast<__m128i*>(&x),
                _mm_sub_epi32(
                    _mm_load_si128(reinterpret_cast<const __m128i*>(&x)),
                    _mm_load_si128(reinterpret_cast<const __m128i*>(&obj.x))
                )
            );
#elif defined(__ARM_NEON)
            vst1q_s32(
                reinterpret_cast<std::int32_t*>(&x),
                vsubq_s32(
                    vld1q_s32(reinterpret_cast<const std::int32_t*>(&x)),
                    vld1q_s32(reinterpret_cast<const std::int32_t*>(&obj.x))
                )
            );
#else
            x -= obj.x;
            y -= obj.y;
            z -= obj.z;
#endif
        } else if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__AVX2__)
            _mm256_store_si256(
                reinterpret_cast<__m256i*>(&x),
                _mm256_sub_epi64(
                    _mm256_load_si256(reinterpret_cast<const __m256i*>(&x)),
                    _mm256_load_si256(reinterpret_cast<const __m256i*>(&obj.x))
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
            vst1q_s64(
                reinterpret_cast<std::int64_t*>(&z),
                vsubq_s64(
                    vld1q_s64(reinterpret_cast<const std::int64_t*>(&z)),
                    vld1q_s64(reinterpret_cast<const std::int64_t*>(&obj.z))
                )
            );
#else
            x -= obj.x;
            y -= obj.y;
            z -= obj.z;
#endif
        } else if constexpr (std::is_same_v<value_type, float>) {
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
            z -= obj.z;
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
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
                reinterpret_cast<double*>(&z),
                vsubq_f64(
                    vld1q_f64(reinterpret_cast<const double*>(&z)),
                    vld1q_f64(reinterpret_cast<const double*>(&obj.z))
                )
            );
#else
            x -= obj.x;
            y -= obj.y;
            z -= obj.z;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__AVX__)
            _mm256_store_ps(
                reinterpret_cast<float*>(&x),
                _mm256_sub_ps(
                    _mm256_load_ps(reinterpret_cast<const float*>(&x)),
                    _mm256_load_ps(reinterpret_cast<const float*>(&obj.x))
                )
            );
#elif defined(__ARM_NEON)
            vst1q_f32(
                reinterpret_cast<float*>(&x), vsubq_f32(
                                                  vld1q_f32(reinterpret_cast<const float*>(&x)),
                                                  vld1q_f32(reinterpret_cast<const float*>(&obj.x))
                                              )
            );
            vst1q_f32(
                reinterpret_cast<float*>(&z), vsubq_f32(
                                                  vld1q_f32(reinterpret_cast<const float*>(&z)),
                                                  vld1q_f32(reinterpret_cast<const float*>(&obj.z))
                                              )
            );
#else
            x -= obj.x;
            y -= obj.y;
            z -= obj.z;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX512F__)
            _mm512_store_pd(
                reinterpret_cast<double*>(&x),
                _mm512_sub_pd(
                    _mm512_load_pd(reinterpret_cast<const double*>(&x)),
                    _mm512_load_pd(reinterpret_cast<const double*>(&obj.x))
                )
            );
#elif defined(__ARM_NEON)
            double* dst{reinterpret_cast<double*>(&x)};
            const double* src{reinterpret_cast<const double*>(&obj.x)};
            vst1q_f64(dst + 0, vsubq_f64(vld1q_f64(dst + 0), vld1q_f64(src + 0)));
            vst1q_f64(dst + 2, vsubq_f64(vld1q_f64(dst + 2), vld1q_f64(src + 2)));
            vst1q_f64(dst + 4, vsubq_f64(vld1q_f64(dst + 4), vld1q_f64(src + 4)));
            vst1q_f64(dst + 6, vsubq_f64(vld1q_f64(dst + 6), vld1q_f64(src + 6)));
#else
            x -= obj.x;
            y -= obj.y;
            z -= obj.z;
#endif
        } else {
            x -= obj.x;
            y -= obj.y;
            z -= obj.z;
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
            std::is_same_v<value_type, std::int32_t> || std::is_same_v<value_type, std::uint32_t>
        ) {
#if defined(__SSE4_1__)
            _mm_store_si128(
                reinterpret_cast<__m128i*>(&x),
                _mm_mullo_epi32(
                    _mm_load_si128(reinterpret_cast<const __m128i*>(&x)), _mm_set1_epi32(alpha)
                )
            );
#elif defined(__ARM_NEON)
            vst1q_s32(
                reinterpret_cast<std::int32_t*>(&x),
                vmulq_s32(vld1q_s32(reinterpret_cast<const std::int32_t*>(&x)), vdupq_n_s32(alpha))
            );
#else
            x *= alpha;
            y *= alpha;
            z *= alpha;
#endif
        } else if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
            _mm256_store_si256(
                reinterpret_cast<__m256i*>(&x),
                _mm256_mullo_epi64(
                    _mm256_load_si256(reinterpret_cast<const __m256i*>(&x)),
                    _mm256_set1_epi64x(alpha)
                )
            );
#else
            x *= alpha;
            y *= alpha;
            z *= alpha;
#endif
        } else if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE__)
            _mm_store_ps(
                reinterpret_cast<float*>(&x),
                _mm_mul_ps(_mm_load_ps(reinterpret_cast<const float*>(&x)), _mm_set1_ps(alpha))
            );
#elif defined(__ARM_NEON)
            vst1q_f32(
                reinterpret_cast<float*>(&x),
                vmulq_f32(vld1q_f32(reinterpret_cast<const float*>(&x)), vdupq_n_f32(alpha))
            );
#else
            x *= alpha;
            y *= alpha;
            z *= alpha;
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX__)
            _mm256_store_pd(
                reinterpret_cast<double*>(&x),
                _mm256_mul_pd(
                    _mm256_load_pd(reinterpret_cast<const double*>(&x)), _mm256_set1_pd(alpha)
                )
            );
#elif defined(__ARM_NEON)
            const float64x2_t broadcast{vdupq_n_f64(alpha)};
            vst1q_f64(
                reinterpret_cast<double*>(&x),
                vmulq_f64(vld1q_f64(reinterpret_cast<const double*>(&x)), broadcast)
            );
            vst1q_f64(
                reinterpret_cast<double*>(&z),
                vmulq_f64(vld1q_f64(reinterpret_cast<const double*>(&z)), broadcast)
            );
#else
            x *= alpha;
            y *= alpha;
            z *= alpha;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__AVX__)
            const __m256 lhs{_mm256_load_ps(reinterpret_cast<const float*>(&x))};
            _mm256_store_ps(
                reinterpret_cast<float*>(&x),
                _mm256_addsub_ps(
                    _mm256_mul_ps(lhs, _mm256_set1_ps(alpha.real())),
                    _mm256_mul_ps(_mm256_permute_ps(lhs, 0xB1), _mm256_set1_ps(alpha.imag()))
                )
            );
#elif defined(__ARM_NEON)
            static constexpr std::uint32_t kAddsubMaskF[4]{0x80000000U, 0U, 0x80000000U, 0U};
            const uint32x4_t addsub_mask{vld1q_u32(kAddsubMaskF)};
            const float32x4_t re_bcast{vdupq_n_f32(alpha.real())};
            const float32x4_t im_bcast{vdupq_n_f32(alpha.imag())};
            float* dst_x{reinterpret_cast<float*>(&x)};
            {
                const float32x4_t lhs{vld1q_f32(dst_x)};
                const float32x4_t prod_re{vmulq_f32(lhs, re_bcast)};
                const float32x4_t prod_im{vmulq_f32(vrev64q_f32(lhs), im_bcast)};
                const float32x4_t signed_im{
                    vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(prod_im), addsub_mask))
                };
                vst1q_f32(dst_x, vaddq_f32(prod_re, signed_im));
            }
            float* dst_z{reinterpret_cast<float*>(&z)};
            {
                const float32x4_t lhs{vld1q_f32(dst_z)};
                const float32x4_t prod_re{vmulq_f32(lhs, re_bcast)};
                const float32x4_t prod_im{vmulq_f32(vrev64q_f32(lhs), im_bcast)};
                const float32x4_t signed_im{
                    vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(prod_im), addsub_mask))
                };
                vst1q_f32(dst_z, vaddq_f32(prod_re, signed_im));
            }
#else
            x *= alpha;
            y *= alpha;
            z *= alpha;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX512F__) && defined(__AVX512DQ__)
            const __m512d lhs{_mm512_load_pd(reinterpret_cast<const double*>(&x))};
            _mm512_store_pd(
                reinterpret_cast<double*>(&x),
                _mm512_add_pd(
                    _mm512_mul_pd(lhs, _mm512_set1_pd(alpha.real())),
                    _mm512_xor_pd(
                        _mm512_mul_pd(_mm512_permute_pd(lhs, 0x55), _mm512_set1_pd(alpha.imag())),
                        _mm512_set_pd(0.0, -0.0, 0.0, -0.0, 0.0, -0.0, 0.0, -0.0)
                    )
                )
            );
#elif defined(__ARM_NEON)
            static constexpr std::uint64_t kAddsubMaskD[2]{0x8000000000000000ULL, 0ULL};
            const uint64x2_t addsub_mask{vld1q_u64(kAddsubMaskD)};
            const float64x2_t re_bcast{vdupq_n_f64(alpha.real())};
            const float64x2_t im_bcast{vdupq_n_f64(alpha.imag())};
            double* base{reinterpret_cast<double*>(&x)};
            for (std::size_t k = 0; k < 4; ++k) {
                double* dst{base + k * 2};
                const float64x2_t lhs{vld1q_f64(dst)};
                const float64x2_t prod_re{vmulq_f64(lhs, re_bcast)};
                const float64x2_t prod_im{vmulq_f64(vextq_f64(lhs, lhs, 1), im_bcast)};
                const float64x2_t signed_im{
                    vreinterpretq_f64_u64(veorq_u64(vreinterpretq_u64_f64(prod_im), addsub_mask))
                };
                vst1q_f64(dst, vaddq_f64(prod_re, signed_im));
            }
#else
            x *= alpha;
            y *= alpha;
            z *= alpha;
#endif
        } else {
            x *= alpha;
            y *= alpha;
            z *= alpha;
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
            z /= alpha;
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
            std::is_same_v<value_type, std::int32_t> || std::is_same_v<value_type, std::uint32_t>
        ) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            return _mm_cmp_epi32_mask(
                       _mm_load_si128(reinterpret_cast<const __m128i*>(&x)),
                       _mm_load_si128(reinterpret_cast<const __m128i*>(&other.x)), _CMP_EQ_OQ
                   ) == 0x7;
#elif defined(__ARM_NEON)
            uint32x4_t mask{vceqq_s32(
                vld1q_s32(reinterpret_cast<const std::int32_t*>(&x)),
                vld1q_s32(reinterpret_cast<const std::int32_t*>(&other.x))
            )};
            mask = vsetq_lane_u32(0xFFFFFFFFU, mask, 3);
            return vminvq_u32(mask) == 0xFFFFFFFFU;
#else
            return x == other.x && y == other.y && z == other.z;
#endif
        } else if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            return _mm256_cmp_epi64_mask(
                       _mm256_load_si256(reinterpret_cast<const __m256i*>(&x)),
                       _mm256_load_si256(reinterpret_cast<const __m256i*>(&other.x)), _CMP_EQ_OQ
                   ) == 0x7;
#elif defined(__ARM_NEON)
            const uint64x2_t mask_lo{vceqq_s64(
                vld1q_s64(reinterpret_cast<const std::int64_t*>(&x)),
                vld1q_s64(reinterpret_cast<const std::int64_t*>(&other.x))
            )};
            uint64x2_t mask_hi{vceqq_s64(
                vld1q_s64(reinterpret_cast<const std::int64_t*>(&z)),
                vld1q_s64(reinterpret_cast<const std::int64_t*>(&other.z))
            )};
            mask_hi = vsetq_lane_u64(~0ULL, mask_hi, 1);
            return vminvq_u32(vreinterpretq_u32_u64(vandq_u64(mask_lo, mask_hi))) == 0xFFFFFFFFU;
#else
            return x == other.x && y == other.y && z == other.z;
#endif
        } else if constexpr (std::is_same_v<value_type, float>) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            return _mm_cmp_ps_mask(
                       _mm_load_ps(reinterpret_cast<const float*>(&x)),
                       _mm_load_ps(reinterpret_cast<const float*>(&other.x)), _CMP_EQ_OQ
                   ) == 0x7;
#elif defined(__ARM_NEON)
            uint32x4_t mask{vceqq_f32(
                vld1q_f32(reinterpret_cast<const float*>(&x)),
                vld1q_f32(reinterpret_cast<const float*>(&other.x))
            )};
            mask = vsetq_lane_u32(0xFFFFFFFFU, mask, 3);
            return vminvq_u32(mask) == 0xFFFFFFFFU;
#else
            return x == other.x && y == other.y && z == other.z;
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            return _mm256_cmp_pd_mask(
                       _mm256_load_pd(reinterpret_cast<const double*>(&x)),
                       _mm256_load_pd(reinterpret_cast<const double*>(&other.x)), _CMP_EQ_OQ
                   ) == 0x7;
#elif defined(__ARM_NEON)
            const uint64x2_t mask_lo{vceqq_f64(
                vld1q_f64(reinterpret_cast<const double*>(&x)),
                vld1q_f64(reinterpret_cast<const double*>(&other.x))
            )};
            uint64x2_t mask_hi{vceqq_f64(
                vld1q_f64(reinterpret_cast<const double*>(&z)),
                vld1q_f64(reinterpret_cast<const double*>(&other.z))
            )};
            mask_hi = vsetq_lane_u64(~0ULL, mask_hi, 1);
            return vminvq_u32(vreinterpretq_u32_u64(vandq_u64(mask_lo, mask_hi))) == 0xFFFFFFFFU;
#else
            return x == other.x && y == other.y && z == other.z;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            return _mm256_cmp_ps_mask(
                       _mm256_load_ps(reinterpret_cast<const float*>(&x)),
                       _mm256_load_ps(reinterpret_cast<const float*>(&other.x)), _CMP_EQ_OQ
                   ) == 0x3F;
#elif defined(__ARM_NEON)
            const uint32x4_t mask_lo{vceqq_f32(
                vld1q_f32(reinterpret_cast<const float*>(&x)),
                vld1q_f32(reinterpret_cast<const float*>(&other.x))
            )};
            uint32x4_t mask_hi{vceqq_f32(
                vld1q_f32(reinterpret_cast<const float*>(&z)),
                vld1q_f32(reinterpret_cast<const float*>(&other.z))
            )};
            mask_hi =
                vreinterpretq_u32_u64(vsetq_lane_u64(~0ULL, vreinterpretq_u64_u32(mask_hi), 1));
            return vminvq_u32(vandq_u32(mask_lo, mask_hi)) == 0xFFFFFFFFU;
#else
            return x == other.x && y == other.y && z == other.z;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX512F__)
            return _mm512_cmp_pd_mask(
                       _mm512_load_pd(reinterpret_cast<const double*>(&x)),
                       _mm512_load_pd(reinterpret_cast<const double*>(&other.x)), _CMP_EQ_OQ
                   ) == 0x3F;
#elif defined(__ARM_NEON)
            const double* lhs_p{reinterpret_cast<const double*>(&x)};
            const double* rhs_p{reinterpret_cast<const double*>(&other.x)};
            const uint64x2_t m0{vceqq_f64(vld1q_f64(lhs_p + 0), vld1q_f64(rhs_p + 0))};
            const uint64x2_t m1{vceqq_f64(vld1q_f64(lhs_p + 2), vld1q_f64(rhs_p + 2))};
            const uint64x2_t m2{vceqq_f64(vld1q_f64(lhs_p + 4), vld1q_f64(rhs_p + 4))};
            return vminvq_u32(vreinterpretq_u32_u64(vandq_u64(vandq_u64(m0, m1), m2))) ==
                   0xFFFFFFFFU;
#else
            return x == other.x && y == other.y && z == other.z;
#endif
        } else {
            return x == other.x && y == other.y && z == other.z;
        }
    }

    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto dot(const Vector& obj) const noexcept -> value_type {
        if constexpr (
            std::is_same_v<value_type, std::int32_t> || std::is_same_v<value_type, std::uint32_t>
        ) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            __m128i sum{_mm_mullo_epi32(
                _mm_maskz_load_epi32(0b0111, reinterpret_cast<const std::int32_t*>(&x)),
                _mm_maskz_load_epi32(0b0111, reinterpret_cast<const std::int32_t*>(&obj.x))
            )};
            sum = _mm_hadd_epi32(sum, sum);
            return _mm_cvtsi128_si32(_mm_hadd_epi32(sum, sum));
#elif defined(__ARM_NEON)
            int32x4_t prod{vmulq_s32(
                vld1q_s32(reinterpret_cast<const std::int32_t*>(&x)),
                vld1q_s32(reinterpret_cast<const std::int32_t*>(&obj.x))
            )};
            prod = vsetq_lane_s32(0, prod, 3);
            return vaddvq_s32(prod);
#else
            return x * obj.x + y * obj.y + z * obj.z;
#endif
        } else if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
            const __m256i sum{_mm256_mullo_epi64(
                _mm256_maskz_load_epi64(0b0111, reinterpret_cast<const std::int64_t*>(&x)),
                _mm256_maskz_load_epi64(0b0111, reinterpret_cast<const std::int64_t*>(&obj.x))
            )};
            const __m128i sum2{
                _mm_add_epi64(_mm256_castsi256_si128(sum), _mm256_extracti64x2_epi64(sum, 1))
            };
            return _mm_cvtsi128_si64(sum2) + _mm_extract_epi64(sum2, 1);
#else
            return x * obj.x + y * obj.y + z * obj.z;
#endif
        } else if constexpr (std::is_same_v<value_type, float>) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            __m128 sum{_mm_mul_ps(
                _mm_maskz_load_ps(0b0111, reinterpret_cast<const float*>(&x)),
                _mm_maskz_load_ps(0b0111, reinterpret_cast<const float*>(&obj.x))
            )};
            sum = _mm_hadd_ps(sum, sum);
            return _mm_cvtss_f32(_mm_hadd_ps(sum, sum));
#elif defined(__ARM_NEON)
            float32x4_t prod{vmulq_f32(
                vld1q_f32(reinterpret_cast<const float*>(&x)),
                vld1q_f32(reinterpret_cast<const float*>(&obj.x))
            )};
            prod = vsetq_lane_f32(0.0F, prod, 3);
            return vaddvq_f32(prod);
#else
            return x * obj.x + y * obj.y + z * obj.z;
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
            const __m256d sum{_mm256_mul_pd(
                _mm256_maskz_load_pd(0b0111, reinterpret_cast<const double*>(&x)),
                _mm256_maskz_load_pd(0b0111, reinterpret_cast<const double*>(&obj.x))
            )};
            const __m128d sum2{
                _mm_add_pd(_mm256_castpd256_pd128(sum), _mm256_extractf64x2_pd(sum, 1))
            };
            return _mm_cvtsd_f64(_mm_hadd_pd(sum2, sum2));
#elif defined(__ARM_NEON)
            const float64x2_t prod_lo{vmulq_f64(
                vld1q_f64(reinterpret_cast<const double*>(&x)),
                vld1q_f64(reinterpret_cast<const double*>(&obj.x))
            )};
            float64x2_t prod_hi{vmulq_f64(
                vld1q_f64(reinterpret_cast<const double*>(&z)),
                vld1q_f64(reinterpret_cast<const double*>(&obj.z))
            )};
            prod_hi = vsetq_lane_f64(0.0, prod_hi, 1);
            return vaddvq_f64(vaddq_f64(prod_lo, prod_hi));
#else
            return x * obj.x + y * obj.y + z * obj.z;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            const __m256 lhs{_mm256_maskz_load_ps(0b00111111, reinterpret_cast<const float*>(&x))};
            const __m256 rhs{
                _mm256_maskz_load_ps(0b00111111, reinterpret_cast<const float*>(&obj.x))
            };
            const __m256 sum{_mm256_addsub_ps(
                _mm256_mul_ps(lhs, _mm256_moveldup_ps(rhs)),
                _mm256_mul_ps(_mm256_permute_ps(lhs, 0xB1), _mm256_movehdup_ps(rhs))
            )};
            const __m128 sum2{
                _mm_add_ps(_mm256_castps256_ps128(sum), _mm256_extractf32x4_ps(sum, 1))
            };
            value_type result;
            _mm_storel_pi(
                reinterpret_cast<__m64*>(&result),
                _mm_add_ps(sum2, _mm_permute_ps(sum2, _MM_SHUFFLE(1, 0, 3, 2)))
            );
            return result;
#else
            return x * obj.x + y * obj.y + z * obj.z;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX512F__) && defined(__AVX512DQ__)
            const __m512d lhs{
                _mm512_maskz_load_pd(0b00111111, reinterpret_cast<const double*>(&x))
            };
            const __m512d rhs{
                _mm512_maskz_load_pd(0b00111111, reinterpret_cast<const double*>(&obj.x))
            };
            const __m512d sum{_mm512_add_pd(
                _mm512_mul_pd(lhs, _mm512_movedup_pd(rhs)),
                _mm512_xor_pd(
                    _mm512_mul_pd(_mm512_permute_pd(lhs, 0x55), _mm512_permute_pd(rhs, 0xFF)),
                    _mm512_set_pd(0.0, -0.0, 0.0, -0.0, 0.0, -0.0, 0.0, -0.0)
                )
            )};
            const __m256d sum2{
                _mm256_add_pd(_mm512_castpd512_pd256(sum), _mm512_extractf64x4_pd(sum, 1))
            };
            value_type result;
            _mm_store_pd(
                reinterpret_cast<double*>(&result),
                _mm_add_pd(_mm256_extractf64x2_pd(sum2, 0), _mm256_extractf64x2_pd(sum2, 1))
            );
            return result;
#elif defined(__ARM_NEON)
            static constexpr std::uint64_t kAddsubMaskD[2]{0x8000000000000000ULL, 0ULL};
            const uint64x2_t addsub_mask{vld1q_u64(kAddsubMaskD)};
            const double* lhs_p{reinterpret_cast<const double*>(&x)};
            const double* rhs_p{reinterpret_cast<const double*>(&obj.x)};
            float64x2_t acc{vdupq_n_f64(0.0)};
            for (std::size_t k = 0; k < 3; ++k) {
                const float64x2_t lhs{vld1q_f64(lhs_p + k * 2)};
                const float64x2_t rhs{vld1q_f64(rhs_p + k * 2)};
                const float64x2_t rhs_re{vdupq_laneq_f64(rhs, 0)};
                const float64x2_t rhs_im{vdupq_laneq_f64(rhs, 1)};
                const float64x2_t prod_re{vmulq_f64(lhs, rhs_re)};
                const float64x2_t prod_im{vmulq_f64(vextq_f64(lhs, lhs, 1), rhs_im)};
                const float64x2_t signed_im{
                    vreinterpretq_f64_u64(veorq_u64(vreinterpretq_u64_f64(prod_im), addsub_mask))
                };
                acc = vaddq_f64(acc, vaddq_f64(prod_re, signed_im));
            }
            value_type result;
            vst1q_f64(reinterpret_cast<double*>(&result), acc);
            return result;
#else
            return x * obj.x + y * obj.y + z * obj.z;
#endif
        } else {
            return x * obj.x + y * obj.y + z * obj.z;
        }
    }

    template <std::size_t OuterSize, DenseLayout ObjLayout>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(
        const Matrix<value_type, Index, 3, OuterSize, ObjLayout, Alloc>& obj
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

    [[using gnu: pure, always_inline, hot]]
    constexpr auto cross(const Vector& obj) const noexcept -> Vector {
        Vector result;
        if constexpr (
            std::is_same_v<value_type, std::int32_t> || std::is_same_v<value_type, std::uint32_t>
        ) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            const __m128i lhs{
                _mm_maskz_load_epi32(0b0111, reinterpret_cast<const std::int32_t*>(&x))
            };
            const __m128i rhs{
                _mm_maskz_load_epi32(0b0111, reinterpret_cast<const std::int32_t*>(&obj.x))
            };
            _mm_store_si128(
                reinterpret_cast<__m128i*>(&result.x),
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
            result.x = y * obj.z - z * obj.y;
            result.y = z * obj.x - x * obj.z;
            result.z = x * obj.y - y * obj.x;
#endif
        } else if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
            const __m256i lhs{
                _mm256_maskz_load_epi64(0b0111, reinterpret_cast<const std::int64_t*>(&x))
            };
            const __m256i rhs{
                _mm256_maskz_load_epi64(0b0111, reinterpret_cast<const std::int64_t*>(&obj.x))
            };
            _mm256_store_si256(
                reinterpret_cast<__m256i*>(&result.x),
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
            result.x = y * obj.z - z * obj.y;
            result.y = z * obj.x - x * obj.z;
            result.z = x * obj.y - y * obj.x;
#endif
        } else if constexpr (std::is_same_v<value_type, float>) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            const __m128 lhs{_mm_maskz_load_ps(0b0111, reinterpret_cast<const float*>(&x))};
            const __m128 rhs{_mm_maskz_load_ps(0b0111, reinterpret_cast<const float*>(&obj.x))};
            _mm_store_ps(
                reinterpret_cast<float*>(&result.x),
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
            result.x = y * obj.z - z * obj.y;
            result.y = z * obj.x - x * obj.z;
            result.z = x * obj.y - y * obj.x;
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            const __m256d lhs{_mm256_maskz_load_pd(0b0111, reinterpret_cast<const double*>(&x))};
            const __m256d rhs{
                _mm256_maskz_load_pd(0b0111, reinterpret_cast<const double*>(&obj.x))
            };
            _mm256_store_pd(
                reinterpret_cast<double*>(&result.x),
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
            result.x = y * obj.z - z * obj.y;
            result.y = z * obj.x - x * obj.z;
            result.z = x * obj.y - y * obj.x;
#endif
        } else {
            result.x = y * obj.z - z * obj.y;
            result.y = z * obj.x - x * obj.z;
            result.z = x * obj.y - y * obj.x;
        }
        return result;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto crossProj(const Vector& obj) const noexcept -> value_type {
        return cross(obj).euclidean();
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
            std::is_same_v<value_type, std::int32_t> || std::is_same_v<value_type, std::uint32_t>
        ) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            const __m128i lhs{
                _mm_maskz_load_epi32(0b0111, reinterpret_cast<const std::int32_t*>(&x))
            };
            __m128i sum{_mm_mullo_epi32(lhs, lhs)};
            sum = _mm_hadd_epi32(sum, sum);
            return _mm_cvtsi128_si32(_mm_hadd_epi32(sum, sum));
#elif defined(__ARM_NEON)
            const int32x4_t lhs{vld1q_s32(reinterpret_cast<const std::int32_t*>(&x))};
            int32x4_t sq{vmulq_s32(lhs, lhs)};
            sq = vsetq_lane_s32(0, sq, 3);
            return vaddvq_s32(sq);
#else
            return std::norm(x) + std::norm(y) + std::norm(z);
#endif
        } else if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
            const __m256i lhs{
                _mm256_maskz_load_epi64(0b0111, reinterpret_cast<const std::int64_t*>(&x))
            };
            const __m256i sum{_mm256_mullo_epi64(lhs, lhs)};
            const __m128i sum2{
                _mm_add_epi64(_mm256_castsi256_si128(sum), _mm256_extracti128_si256(sum, 1))
            };
            return _mm_cvtsi128_si64(sum2) + _mm_extract_epi64(sum2, 1);
#else
            return std::norm(x) + std::norm(y) + std::norm(z);
#endif
        } else if constexpr (std::is_same_v<value_type, float>) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            const __m128 lhs{_mm_maskz_load_ps(0b0111, reinterpret_cast<const float*>(&x))};
            __m128 sum{_mm_mul_ps(lhs, lhs)};
            sum = _mm_hadd_ps(sum, sum);
            return _mm_cvtss_f32(_mm_hadd_ps(sum, sum));
#elif defined(__ARM_NEON)
            const float32x4_t lhs{vld1q_f32(reinterpret_cast<const float*>(&x))};
            float32x4_t sq{vmulq_f32(lhs, lhs)};
            sq = vsetq_lane_f32(0.0F, sq, 3);
            return vaddvq_f32(sq);
#else
            return std::norm(x) + std::norm(y) + std::norm(z);
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX512DQ__) && defined(__AVX512VL__)
            const __m256d lhs{_mm256_maskz_load_pd(0b0111, reinterpret_cast<const double*>(&x))};
            const __m256d sum{_mm256_mul_pd(lhs, lhs)};
            const __m128d sum2{
                _mm_add_pd(_mm256_castpd256_pd128(sum), _mm256_extractf64x2_pd(sum, 1))
            };
            return _mm_cvtsd_f64(_mm_hadd_pd(sum2, sum2));
#elif defined(__ARM_NEON)
            const float64x2_t lhs_lo{vld1q_f64(reinterpret_cast<const double*>(&x))};
            const float64x2_t lhs_hi{vld1q_f64(reinterpret_cast<const double*>(&z))};
            float64x2_t sq_hi{vmulq_f64(lhs_hi, lhs_hi)};
            sq_hi = vsetq_lane_f64(0.0, sq_hi, 1);
            return vaddvq_f64(vaddq_f64(vmulq_f64(lhs_lo, lhs_lo), sq_hi));
#else
            return std::norm(x) + std::norm(y) + std::norm(z);
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__AVX512F__) && defined(__AVX512VL__)
            const __m256 lhs{_mm256_maskz_load_ps(0b00111111, reinterpret_cast<const float*>(&x))};
            const __m256 sum{_mm256_mul_ps(lhs, lhs)};
            __m128 sum2{_mm_add_ps(_mm256_extractf32x4_ps(sum, 0), _mm256_extractf32x4_ps(sum, 1))};
            sum2 = _mm_hadd_ps(sum2, sum2);
            return _mm_cvtss_f32(_mm_hadd_ps(sum2, sum2));
#elif defined(__ARM_NEON)
            const float32x4_t lhs_lo{vld1q_f32(reinterpret_cast<const float*>(&x))};
            const float32x4_t lhs_hi{vld1q_f32(reinterpret_cast<const float*>(&z))};
            float32x4_t sq_hi{vmulq_f32(lhs_hi, lhs_hi)};
            sq_hi = vreinterpretq_f32_u64(vsetq_lane_u64(0ULL, vreinterpretq_u64_f32(sq_hi), 1));
            return vaddvq_f32(vaddq_f32(vmulq_f32(lhs_lo, lhs_lo), sq_hi));
#else
            return std::norm(x) + std::norm(y) + std::norm(z);
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX512F__) && defined(__AVX512DQ__)
            const __m512d lhs{
                _mm512_maskz_load_pd(0b00111111, reinterpret_cast<const double*>(&x))
            };
            const __m512d sum{_mm512_mul_pd(lhs, lhs)};
            const __m256d sum2{
                _mm256_add_pd(_mm512_extractf64x4_pd(sum, 0), _mm512_extractf64x4_pd(sum, 1))
            };
            const __m128d sum3{
                _mm_add_pd(_mm256_extractf64x2_pd(sum2, 0), _mm256_extractf64x2_pd(sum2, 1))
            };
            return _mm_cvtsd_f64(_mm_hadd_pd(sum3, sum3));
#elif defined(__ARM_NEON)
            const double* base{reinterpret_cast<const double*>(&x)};
            const float64x2_t l0{vld1q_f64(base + 0)};
            const float64x2_t l1{vld1q_f64(base + 2)};
            const float64x2_t l2{vld1q_f64(base + 4)};
            const float64x2_t acc{
                vaddq_f64(vaddq_f64(vmulq_f64(l0, l0), vmulq_f64(l1, l1)), vmulq_f64(l2, l2))
            };
            return vaddvq_f64(acc);
#else
            return std::norm(x) + std::norm(y) + std::norm(z);
#endif
        } else {
            return std::norm(x) + std::norm(y) + std::norm(z);
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
#if defined(__AVX__)
            _mm256_store_ps(
                reinterpret_cast<float*>(&x),
                _mm256_xor_ps(
                    _mm256_load_ps(reinterpret_cast<const float*>(&x)),
                    _mm256_set_ps(-0.0F, 0.0F, -0.0F, 0.0F, -0.0F, 0.0F, -0.0F, 0.0F)
                )
            );
#elif defined(__ARM_NEON)
            static constexpr std::uint32_t kConjMaskF[4]{0U, 0x80000000U, 0U, 0x80000000U};
            const uint32x4_t mask{vld1q_u32(kConjMaskF)};
            vst1q_f32(
                reinterpret_cast<float*>(&x),
                vreinterpretq_f32_u32(veorq_u32(
                    vreinterpretq_u32_f32(vld1q_f32(reinterpret_cast<const float*>(&x))), mask
                ))
            );
            vst1q_f32(
                reinterpret_cast<float*>(&z),
                vreinterpretq_f32_u32(veorq_u32(
                    vreinterpretq_u32_f32(vld1q_f32(reinterpret_cast<const float*>(&z))), mask
                ))
            );
#else
            x = std::conj(x);
            y = std::conj(y);
            z = std::conj(z);
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX512F__) && defined(__AVX512DQ__)
            _mm512_store_pd(
                reinterpret_cast<double*>(&x),
                _mm512_xor_pd(
                    _mm512_load_pd(reinterpret_cast<const double*>(&x)),
                    _mm512_set_pd(-0.0, 0.0, -0.0, 0.0, -0.0, 0.0, -0.0, 0.0)
                )
            );
#elif defined(__ARM_NEON)
            static constexpr std::uint64_t kConjMaskD[2]{0ULL, 0x8000000000000000ULL};
            const uint64x2_t mask{vld1q_u64(kConjMaskD)};
            double* base{reinterpret_cast<double*>(&x)};
            for (std::size_t k = 0; k < 4; ++k) {
                double* dst{base + k * 2};
                vst1q_f64(
                    dst,
                    vreinterpretq_f64_u64(veorq_u64(vreinterpretq_u64_f64(vld1q_f64(dst)), mask))
                );
            }
#else
            x = std::conj(x);
            y = std::conj(y);
            z = std::conj(z);
#endif
        } else if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
            x = std::conj(x);
            y = std::conj(y);
            z = std::conj(z);
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

  private:
    [[using gnu: always_inline, hot]]
    constexpr auto selfNegate() noexcept -> void {
        if constexpr (
            std::is_same_v<value_type, std::int32_t> || std::is_same_v<value_type, std::uint32_t>
        ) {
#if defined(__SSE2__)
            // Two's complement has no sign bit to flip: negate as 0 - x.
            _mm_store_si128(
                reinterpret_cast<__m128i*>(&x),
                _mm_sub_epi32(
                    _mm_setzero_si128(), _mm_load_si128(reinterpret_cast<const __m128i*>(&x))
                )
            );
#elif defined(__ARM_NEON)
            vst1q_s32(
                reinterpret_cast<std::int32_t*>(&x),
                vnegq_s32(vld1q_s32(reinterpret_cast<const std::int32_t*>(&x)))
            );
#else
            x = -x;
            y = -y;
            z = -z;
#endif
        } else if constexpr (
            std::is_same_v<value_type, std::int64_t> || std::is_same_v<value_type, std::uint64_t>
        ) {
#if defined(__AVX2__)
            // Two's complement has no sign bit to flip: negate as 0 - x.
            _mm256_store_si256(
                reinterpret_cast<__m256i*>(&x),
                _mm256_sub_epi64(
                    _mm256_setzero_si256(), _mm256_load_si256(reinterpret_cast<const __m256i*>(&x))
                )
            );
#elif defined(__ARM_NEON)
            vst1q_s64(
                reinterpret_cast<std::int64_t*>(&x),
                vnegq_s64(vld1q_s64(reinterpret_cast<const std::int64_t*>(&x)))
            );
            vst1q_s64(
                reinterpret_cast<std::int64_t*>(&z),
                vnegq_s64(vld1q_s64(reinterpret_cast<const std::int64_t*>(&z)))
            );
#else
            x = -x;
            y = -y;
            z = -z;
#endif
        } else if constexpr (std::is_same_v<value_type, float>) {
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
            z = -z;
#endif
        } else if constexpr (std::is_same_v<value_type, double>) {
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
                reinterpret_cast<double*>(&z),
                vnegq_f64(vld1q_f64(reinterpret_cast<const double*>(&z)))
            );
#else
            x = -x;
            y = -y;
            z = -z;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
#if defined(__AVX__)
            _mm256_store_ps(
                reinterpret_cast<float*>(&x),
                _mm256_xor_ps(
                    _mm256_load_ps(reinterpret_cast<const float*>(&x)), _mm256_set1_ps(-0.0F)
                )
            );
#elif defined(__ARM_NEON)
            vst1q_f32(
                reinterpret_cast<float*>(&x),
                vnegq_f32(vld1q_f32(reinterpret_cast<const float*>(&x)))
            );
            vst1q_f32(
                reinterpret_cast<float*>(&z),
                vnegq_f32(vld1q_f32(reinterpret_cast<const float*>(&z)))
            );
#else
            x = -x;
            y = -y;
            z = -z;
#endif
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
#if defined(__AVX512F__) && defined(__AVX512DQ__)
            _mm512_store_pd(
                reinterpret_cast<double*>(&x),
                _mm512_xor_pd(
                    _mm512_load_pd(reinterpret_cast<const double*>(&x)), _mm512_set1_pd(-0.0)
                )
            );
#elif defined(__ARM_NEON)
            double* base{reinterpret_cast<double*>(&x)};
            vst1q_f64(base + 0, vnegq_f64(vld1q_f64(base + 0)));
            vst1q_f64(base + 2, vnegq_f64(vld1q_f64(base + 2)));
            vst1q_f64(base + 4, vnegq_f64(vld1q_f64(base + 4)));
            vst1q_f64(base + 6, vnegq_f64(vld1q_f64(base + 6)));
#else
            x = -x;
            y = -y;
            z = -z;
#endif
        } else {
            x = -x;
            y = -y;
            z = -z;
        }
        return;
    }

  public:
    using serialize = zpp::bits::members<3>;

    value_type x, y, z;
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

template <typename T>
    requires Vec3Arithmetic<std::remove_cvref_t<T>>
[[using gnu: always_inline, hot]]
inline constexpr auto operator*(const ScalarArithmetic auto& fac, T&& obj) noexcept
    -> decltype(auto) {
    return std::forward<T>(obj) * fac;
}

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
    return std::hypot(obj.x, obj.y, obj.z);
}

template <::basjoo::math::Vec3Arithmetic V>
[[using gnu: pure, always_inline]]
inline constexpr auto abs(const V& obj) noexcept
    -> ::basjoo::math::detail::DenseNormTraitT<typename V::value_type> {
    return obj.euclidean();
}

template <::basjoo::math::Vec3Arithmetic V>
[[using gnu: pure, always_inline]]
inline constexpr auto norm(const V& obj) noexcept
    -> ::basjoo::math::detail::DenseNormTraitT<typename V::value_type> {
    return obj.euclideanSqr();
}

template <typename T>
    requires ::basjoo::math::Vec3Arithmetic<std::remove_cvref_t<T>>
[[using gnu: always_inline]]
inline constexpr auto conj(T&& vector) noexcept -> decltype(auto) {
    return std::forward<T>(vector).conjugated();
}

} // namespace std

// NOLINTEND(cert-dcl58-cpp)
