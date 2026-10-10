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
#include <mdspan>
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

template <ScalarArithmetic Scalar, std::size_t Extent, Allocatory Alloc>
class alignas(32) Vector final : public VectorBase {
  public:
    using element_type = typename DenseTrait<Vector>::element_type;
    using value_type = typename DenseTrait<Vector>::value_type;
    using reference = typename DenseTrait<Vector>::reference;
    using const_reference = typename DenseTrait<Vector>::const_reference;
    using pointer = typename DenseTrait<Vector>::pointer;
    using const_pointer = typename DenseTrait<Vector>::const_pointer;
    using iterator = typename DenseTrait<Vector>::iterator;
    using const_iterator = typename DenseTrait<Vector>::const_iterator;
    using reverse_iterator = typename DenseTrait<Vector>::reverse_iterator;
    using const_reverse_iterator = typename DenseTrait<Vector>::const_reverse_iterator;
    using size_type = typename DenseTrait<Vector>::size_type;
    using difference_type = typename DenseTrait<Vector>::difference_type;
    using allocator_type = typename DenseTrait<Vector>::allocator_type;
    using storage_type = typename DenseTrait<Vector>::storage_type;
    static constexpr size_type extent{DenseTrait<Vector>::extent};

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

    template <std::ranges::sized_range R = std::initializer_list<value_type>>
    [[using gnu: always_inline]]
    constexpr explicit Vector(R&& rg, [[maybe_unused]] const allocator_type& alloc = {}) noexcept(
        !BASJOO_CHECK_PARAMS
    )
        requires std::indirectly_copyable<
            std::ranges::iterator_t<R>, std::ranges::iterator_t<storage_type>>
        : m_data{} {
#if BASJOO_CHECK_PARAMS == 1
        if (std::ranges::size(rg) != size()) [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
        std::ranges::copy(rg, m_data.begin());
    }

    template <typename T, DenseExtents Extents>
    [[using gnu: always_inline]]
    constexpr explicit Vector(
        std::mdspan<T, Extents, std::layout_stride> matrix_mdspan,
        [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        requires std::same_as<std::remove_cv_t<T>, value_type> && (Extents::rank() == 2)
    {
#if BASJOO_CHECK_PARAMS == 1
        if (matrix_mdspan.extent(0) != 1 && matrix_mdspan.extent(1) != 1) [[unlikely]] {
            throw std::out_of_range("this matrix does not convert to a vector.");
        }
        if (matrix_mdspan.extent(0) * matrix_mdspan.extent(1) != size()) [[unlikely]] {
            throw std::invalid_argument("Matrix size must be equal to Vector size");
        }
#endif
        const bool row_like{matrix_mdspan.extent(1) != 1};
        for (size_type i = 0; i < size(); ++i) {
            m_data[i] = row_like ? matrix_mdspan[0, i] : matrix_mdspan[i, 0];
        }
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::array<value_type, extent>() const noexcept {
        return std::array<value_type, extent>{m_data};
    }

    [[using gnu: pure, always_inline]]
    constexpr explicit operator std::span<value_type, extent>() noexcept {
        return std::span<value_type, extent>(m_data);
    }

    [[using gnu: pure, always_inline]]
    constexpr explicit operator std::span<const value_type, extent>() const noexcept {
        return std::span<const value_type, extent>(m_data);
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<value_type>() noexcept {
        return std::span<value_type>(m_data.data(), m_data.size());
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<const value_type>() const noexcept {
        return std::span<const value_type>(m_data.data(), m_data.size());
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto size() const noexcept -> size_type {
        return extent;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() noexcept -> pointer {
        return m_data.data();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() const noexcept -> const_pointer {
        return m_data.data();
    }

    [[using gnu: always_inline]]
    constexpr auto resize([[maybe_unused]] size_type count) -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (count != extent()) [[unlikely]] {
            throw std::invalid_argument{"Vector::resize: size mismatches"};
        }
#endif
        return;
    }

    [[using gnu: always_inline]]
    constexpr auto assign([[maybe_unused]] size_type count, [[maybe_unused]] const_reference value)
        -> void
        requires(extent != std::dynamic_extent)
    {
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

template <ScalarArithmetic Scalar, Allocatory Alloc>
class Vector<Scalar, std::dynamic_extent, Alloc> final : public VectorBase {
  public:
    using element_type = typename DenseTrait<Vector>::element_type;
    using value_type = typename DenseTrait<Vector>::value_type;
    using reference = typename DenseTrait<Vector>::reference;
    using const_reference = typename DenseTrait<Vector>::const_reference;
    using pointer = typename DenseTrait<Vector>::pointer;
    using const_pointer = typename DenseTrait<Vector>::const_pointer;
    using iterator = typename DenseTrait<Vector>::iterator;
    using const_iterator = typename DenseTrait<Vector>::const_iterator;
    using reverse_iterator = typename DenseTrait<Vector>::reverse_iterator;
    using const_reverse_iterator = typename DenseTrait<Vector>::const_reverse_iterator;
    using size_type = typename DenseTrait<Vector>::size_type;
    using difference_type = typename DenseTrait<Vector>::difference_type;
    using allocator_type = typename DenseTrait<Vector>::allocator_type;
    using storage_type = typename DenseTrait<Vector>::storage_type;
    static constexpr size_type extent{DenseTrait<Vector>::extent};

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
    constexpr Vector(std::vector<value_type, allocator_type> data)
        : m_data{std::move(data)} {
        m_data.shrink_to_fit();
    }

    template <std::ranges::sized_range R = std::initializer_list<value_type>>
    [[using gnu: always_inline]]
    constexpr explicit Vector(R&& rg, [[maybe_unused]] const allocator_type& alloc = {})
        requires std::indirectly_copyable<
                     std::ranges::iterator_t<R>, std::ranges::iterator_t<storage_type>> &&
                 (!std::same_as<R, storage_type>)
        : m_data(std::from_range_t{}, rg, alloc) {
        m_data.shrink_to_fit();
    }

    template <typename T, DenseExtents Extents>
    [[using gnu: always_inline]] constexpr explicit Vector(
        std::mdspan<T, Extents, std::layout_stride> matrix_mdspan,
        [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        requires std::same_as<std::remove_cv_t<T>, value_type> && (Extents::ranks() == 2)
        : m_data(matrix_mdspan.extent(0) * matrix_mdspan.extent(1), alloc) {
#if BASJOO_CHECK_PARAMS == 1
        if (matrix_mdspan.extent(0) != 1 && matrix_mdspan.extent(1) != 1) [[unlikely]] {
            throw std::out_of_range("this matrix does not convert to a vector.");
        }
        if (matrix_mdspan.extent(0) * matrix_mdspan.extent(1) != size()) [[unlikely]] {
            throw std::invalid_argument("Matrix size must be equal to Vector size");
        }
#endif
        m_data.shrink_to_fit();
        const bool row_like{matrix_mdspan.extent(1) != 1};
        for (size_type i = 0; i < size(); ++i) {
            m_data[i] = row_like ? matrix_mdspan[0, i] : matrix_mdspan[i, 0];
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
            return std::vector<value_type, allocator_type>(
                self.m_data.cbegin(), self.m_data.cend(), self.m_data.get_allocator()
            );
        }
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<value_type>() noexcept {
        return std::span<value_type>(m_data.data(), m_data.size());
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<const value_type>() const noexcept {
        return std::span<const value_type>(m_data.data(), m_data.size());
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto size() const noexcept -> size_type {
        return m_data.size();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() noexcept -> pointer {
        return m_data.data();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() const noexcept -> const_pointer {
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
