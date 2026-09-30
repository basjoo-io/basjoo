/**
 * @file vector.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-04-02
 *
 * @copyright Copyright (c) 2025 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <initializer_list>
#include <ranges>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include "zpp_bits.h"

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_trait.hpp"
#include "basjoo/math/dense/vector_base.hpp"

namespace basjoo::math {

template <ScalarArithmetic Scalar, std::integral Index, std::size_t N, Allocatory Alloc>
class alignas(32) Vector final : public VectorBase {
  public:
    using element_type = typename DenseTrait<Vector>::element_type;
    using value_type = typename DenseTrait<Vector>::value_type;
    using pointer = typename DenseTrait<Vector>::pointer;
    using const_pointer = typename DenseTrait<Vector>::const_pointer;
    using reference = typename DenseTrait<Vector>::reference;
    using const_reference = typename DenseTrait<Vector>::const_reference;
    using extents_type = typename DenseTrait<Vector>::extents_type;
    using layout_type = typename DenseTrait<Vector>::layout_type;
    using allocator_type = typename DenseTrait<Vector>::allocator_type;
    using size_type = typename DenseTrait<Vector>::size_type;
    using index_type = typename DenseTrait<Vector>::index_type;
    using rank_type = typename DenseTrait<Vector>::rank_type;
    using difference_type = typename DenseTrait<Vector>::difference_type;
    using mapping_type = typename DenseTrait<Vector>::mapping_type;
    using storage_type = typename DenseTrait<Vector>::storage_type;

    [[using gnu: always_inline]]
    constexpr Vector() noexcept = default;
    constexpr Vector(const Vector& other) noexcept = default;
    constexpr auto operator=(const Vector& other) noexcept -> Vector& = default;
    constexpr Vector(Vector&& other) noexcept = default;
    constexpr auto operator=(Vector&& other) noexcept -> Vector& = default;
    constexpr ~Vector() noexcept = default;

    [[using gnu: pure, always_inline]]
    constexpr auto get_allocator() const noexcept -> allocator_type {
        return allocator_type{};
    }

    [[using gnu: always_inline]]
    constexpr explicit Vector([[maybe_unused]] const allocator_type& alloc) noexcept
        : m_data{} {}

    [[using gnu: always_inline]]
    constexpr explicit Vector(
        [[maybe_unused]] size_type count, [[maybe_unused]] const allocator_type& alloc = {}
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
        [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : m_data{} {
#if BASJOO_CHECK_PARAMS == 1
        if (count != size()) [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
        m_data.fill(value);
    }

    [[using gnu: always_inline]]
    constexpr Vector(
        std::initializer_list<value_type> list, [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : m_data{} {
#if BASJOO_CHECK_PARAMS == 1
        if (list.size() != size()) [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
        std::ranges::copy(list, m_data.begin());
    }

    template <std::ranges::sized_range R>
        requires std::indirectly_copyable<
            std::ranges::iterator_t<R>, std::ranges::iterator_t<storage_type>>
    [[using gnu: always_inline]] constexpr explicit Vector(
        R&& rg, [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : m_data{} {
#if BASJOO_CHECK_PARAMS == 1
        if (std::ranges::size(rg) != size()) [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
        std::ranges::copy(rg, m_data.begin());
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
            m_data[i] = row_like ? matrix_like[0, k] : matrix_like[k, 0];
        }
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::array<value_type, N>() const noexcept {
        return {m_data};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<value_type, N>() noexcept {
        return {m_data};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<const value_type, N>() const noexcept {
        return {m_data};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<value_type, std::dynamic_extent>() noexcept {
        return {m_data.data(), m_data.size()};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<const value_type, std::dynamic_extent>() const noexcept {
        return {m_data.data(), m_data.size()};
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

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto extents() const noexcept -> extents_type {
        return extents_type{};
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto mapping() const noexcept -> mapping_type {
        return mapping_type{};
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto extent([[maybe_unused]] rank_type r) const noexcept -> index_type {
        return N;
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto size() const noexcept -> size_type {
        return N;
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto stride([[maybe_unused]] rank_type r = 0) const noexcept -> index_type {
        return 1;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto is_unique() const noexcept -> bool {
        return true;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto is_exhaustive() const noexcept -> bool {
        return true;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto is_strided() const noexcept -> bool {
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
    constexpr auto data() noexcept -> pointer {
        return m_data.data();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() const noexcept -> const_pointer {
        return m_data.data();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data_handle() noexcept -> pointer {
        return m_data.data();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data_handle() const noexcept -> const_pointer {
        return m_data.data();
    }

    [[using gnu: always_inline]]
    constexpr auto resize([[maybe_unused]] size_type count) -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (count != size()) [[unlikely]] {
            throw std::invalid_argument{"Vector::resize: size mismatches"};
        }
#endif
        return;
    }

    [[using gnu: always_inline]]
    constexpr auto assign([[maybe_unused]] size_type count, [[maybe_unused]] const_reference value)
        -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (count != size()) [[unlikely]] {
            throw std::invalid_argument{"Vector::assign: size mismatch"};
        }
#endif
        m_data.fill(value);
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    storage_type m_data;
};

template <ScalarArithmetic Scalar, std::integral Index, Allocatory Alloc>
class Vector<Scalar, Index, std::dynamic_extent, Alloc> final : public VectorBase {
  public:
    using element_type = typename DenseTrait<Vector>::element_type;
    using value_type = typename DenseTrait<Vector>::value_type;
    using pointer = typename DenseTrait<Vector>::pointer;
    using const_pointer = typename DenseTrait<Vector>::const_pointer;
    using reference = typename DenseTrait<Vector>::reference;
    using const_reference = typename DenseTrait<Vector>::const_reference;
    using extents_type = typename DenseTrait<Vector>::extents_type;
    using layout_type = typename DenseTrait<Vector>::layout_type;
    using allocator_type = typename DenseTrait<Vector>::allocator_type;
    using size_type = typename DenseTrait<Vector>::size_type;
    using index_type = typename DenseTrait<Vector>::index_type;
    using rank_type = typename DenseTrait<Vector>::rank_type;
    using difference_type = typename DenseTrait<Vector>::difference_type;
    using mapping_type = typename DenseTrait<Vector>::mapping_type;
    using storage_type = typename DenseTrait<Vector>::storage_type;

    [[using gnu: always_inline]]
    constexpr Vector() noexcept = default;
    constexpr Vector(const Vector& other) = default;
    constexpr auto operator=(const Vector& other) -> Vector& = default;
    constexpr Vector(Vector&& other) noexcept = default;
    constexpr auto operator=(Vector&& other) noexcept -> Vector& = default;
    constexpr ~Vector() noexcept = default;

    [[using gnu: pure, always_inline]]
    constexpr auto get_allocator() const noexcept -> allocator_type {
        return m_data.get_allocator();
    }

    [[using gnu: always_inline]]
    constexpr explicit Vector(const allocator_type& alloc) noexcept
        : m_data(alloc) {}

    [[using gnu: always_inline]]
    constexpr explicit Vector(size_type count, const allocator_type& alloc = {})
        : m_data(count, alloc) {
        m_data.shrink_to_fit();
    }

    [[using gnu: always_inline]]
    constexpr explicit Vector(
        size_type count, const_reference value, const allocator_type& alloc = {}
    )
        : m_data(count, value, alloc) {
        m_data.shrink_to_fit();
    }

    [[using gnu: always_inline]]
    constexpr Vector(std::initializer_list<value_type> list, const allocator_type& alloc = {})
        : m_data(list, alloc) {}

    [[using gnu: always_inline]]
    constexpr Vector(std::vector<value_type, allocator_type> data)
        : m_data{std::move(data)} {
        m_data.shrink_to_fit();
    }

    template <std::ranges::sized_range R>
        requires std::indirectly_copyable<
                     std::ranges::iterator_t<R>, std::ranges::iterator_t<storage_type>> &&
                 (!std::same_as<std::remove_cvref_t<R>, storage_type>)
    [[using gnu: always_inline]]
    constexpr explicit Vector(R&& rg, const allocator_type& alloc = {})
        : m_data(std::from_range, std::forward<R>(rg), alloc) {
        m_data.shrink_to_fit();
    }

    template <typename T>
        requires std::same_as<std::remove_cv_t<typename T::value_type>, value_type> &&
                 (T::rank() == 2)
    [[using gnu: always_inline]]
    constexpr explicit Vector(const T& matrix_like, const allocator_type& alloc = {})
        : m_data(alloc) {
        using other_index_type = typename T::index_type;

#if BASJOO_CHECK_PARAMS == 1
        if (matrix_like.extent(0) != 1 && matrix_like.extent(1) != 1) [[unlikely]] {
            throw std::invalid_argument(
                "Vector constructor: matrix is neither a row nor a column!"
            );
        }
#endif
        // Allocated only now, once the shape is known to be valid.
        m_data.resize(
            static_cast<size_type>(matrix_like.extent(0)) *
            static_cast<size_type>(matrix_like.extent(1))
        );
        m_data.shrink_to_fit();
        const bool row_like{matrix_like.extent(1) != 1};
        for (size_type i{0}; i < size(); ++i) {
            const auto k{static_cast<other_index_type>(i)};
            m_data[i] = row_like ? matrix_like[0, k] : matrix_like[k, 0];
        }
    }

    template <typename Self>
    [[using gnu: always_inline]]
    constexpr operator std::vector<value_type, allocator_type>(
        this Self&& self
    ) noexcept(std::is_rvalue_reference_v<Self&&>) {
        if constexpr (std::is_rvalue_reference_v<Self&&>) {
            return std::move(self.m_data);
        } else {
            return {self.m_data.cbegin(), self.m_data.cend(), self.m_data.get_allocator()};
        }
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<value_type>() noexcept {
        return {m_data.data(), m_data.size()};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<const value_type>() const noexcept {
        return {m_data.data(), m_data.size()};
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

    [[using gnu: pure, always_inline]]
    constexpr auto extents() const noexcept -> extents_type {
        return extents_type{static_cast<index_type>(m_data.size())};
    }

    [[using gnu: pure, always_inline]]
    constexpr auto mapping() const noexcept -> mapping_type {
        return mapping_type{extents()};
    }

    [[using gnu: pure, always_inline]]
    constexpr auto extent([[maybe_unused]] rank_type r) const noexcept -> index_type {
        return static_cast<index_type>(m_data.size());
    }

    [[using gnu: pure, always_inline]]
    constexpr auto size() const noexcept -> size_type {
        return static_cast<size_type>(m_data.size());
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto stride([[maybe_unused]] rank_type r = 0) const noexcept -> index_type {
        return 1;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto is_unique() const noexcept -> bool {
        return true;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto is_exhaustive() const noexcept -> bool {
        return true;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto is_strided() const noexcept -> bool {
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
    constexpr auto data() noexcept -> pointer {
        return m_data.data();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() const noexcept -> const_pointer {
        return m_data.data();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data_handle() noexcept -> pointer {
        return m_data.data();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data_handle() const noexcept -> const_pointer {
        return m_data.data();
    }

    [[using gnu: always_inline]]
    constexpr auto resize(size_type count) -> void {
        m_data.resize(count);
        m_data.shrink_to_fit();
        return;
    }

    [[using gnu: always_inline]]
    constexpr auto assign(size_type count, const_reference value) -> void {
        m_data.assign(count, value);
        m_data.shrink_to_fit();
        return;
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    storage_type m_data;
};

} // namespace basjoo::math

// Vector<T, Index, 2> and Vector<T, Index, 3> are partial specializations: make them visible
// wherever the primary template is, so that no translation unit instantiates the primary for N == 2
// or 3.
#include "basjoo/math/dense/vec2.hpp"
#include "basjoo/math/dense/vec3.hpp"
