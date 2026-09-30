/**
 * @file matrix.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-03-31
 *
 * @copyright Copyright (c) 2025 basjoo development team.
 *            All rights reserved.
 *
 */

#pragma once

#include <array>
#include <cstddef>
#include <mdspan>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "zpp_bits.h"

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_trait.hpp"
#include "basjoo/math/dense/matrix_base.hpp"

namespace basjoo::math {

template <
    ScalarArithmetic Scalar, std::integral Index, std::size_t NRows, std::size_t NCols,
    DenseLayout Layout, Allocatory Alloc>
    requires((NRows == std::dynamic_extent) == (NCols == std::dynamic_extent))
class alignas(32) Matrix final : public MatrixBase {
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
    constexpr Matrix(const Matrix& other) noexcept = default;
    constexpr auto operator=(const Matrix& other) noexcept -> Matrix& = default;
    constexpr Matrix(Matrix&& other) noexcept = default;
    constexpr auto operator=(Matrix&& other) noexcept -> Matrix& = default;
    constexpr ~Matrix() noexcept = default;

    [[using gnu: pure, always_inline]]
    constexpr auto get_allocator() const noexcept -> allocator_type {
        return allocator_type{};
    }

    [[using gnu: always_inline]]
    constexpr explicit Matrix([[maybe_unused]] const allocator_type& alloc) noexcept
        : m_data{}, m_map{} {}

    [[using gnu: always_inline]]
    constexpr explicit Matrix(
        [[maybe_unused]] index_type rows, [[maybe_unused]] index_type cols,
        [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : m_data{}, m_map{} {
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
        : m_data{}, m_map{} {
#if BASJOO_CHECK_PARAMS == 1
        if (rows != nrows() || cols != ncols()) [[unlikely]] {
            throw std::invalid_argument("Matrix constructor: mismatch matrix sizes detected.");
        }
#endif
        m_data.fill(value);
    }

    template <typename T>
        requires std::same_as<std::remove_cv_t<typename T::value_type>, value_type> &&
                     (T::rank() == 2)
    [[using gnu: always_inline]] constexpr explicit Matrix(
        const T& matrix_like, [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : m_data{}, m_map{} {
        using other_index_type = typename T::index_type;

#if BASJOO_CHECK_PARAMS == 1
        if (std::cmp_not_equal(matrix_like.extent(0), nrows()) ||
            std::cmp_not_equal(matrix_like.extent(1), ncols())) [[unlikely]] {
            throw std::invalid_argument("Matrix constructor: mismatch matrix sizes detected.");
        }
#endif
        for (index_type i{0}; i < nrows(); ++i) {
            for (index_type j{0}; j < ncols(); ++j) {
                operator[](i, j) =
                    matrix_like[static_cast<other_index_type>(i), static_cast<other_index_type>(j)];
            }
        }
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::mdspan<value_type, extents_type, layout_type>() noexcept {
        return {data(), m_map};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::mdspan<const value_type, extents_type, layout_type>() const noexcept {
        return {data(), m_map};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::mdspan<
        value_type, std::dextents<index_type, 2>, std::layout_stride>() noexcept {
        return {data(), strided_mapping()};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::mdspan<
        const value_type, std::dextents<index_type, 2>, std::layout_stride>() const noexcept {
        return {data(), strided_mapping()};
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
    constexpr auto extents() const noexcept -> const extents_type& {
        return m_map.extents();
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto mapping() const noexcept -> const mapping_type& {
        return m_map;
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto extent(rank_type r) const noexcept -> index_type {
        return m_map.extents().extent(r);
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto nrows() const noexcept -> index_type {
        return m_map.extents().extent(0);
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto ncols() const noexcept -> index_type {
        return m_map.extents().extent(1);
    }

    [[using gnu: pure, always_inline]]
    constexpr auto size() const noexcept -> size_type {
        return m_data.size();
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto stride(rank_type r) const noexcept -> index_type {
        return m_map.stride(r);
    }

    [[using gnu: pure, always_inline]]
    constexpr auto is_unique() const noexcept -> bool {
        return m_map.is_unique();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto is_exhaustive() const noexcept -> bool {
        return m_map.is_exhaustive();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto is_strided() const noexcept -> bool {
        return m_map.is_strided();
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto is_always_unique() noexcept -> bool {
        return mapping_type::is_always_unique();
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto is_always_exhaustive() noexcept -> bool {
        return mapping_type::is_always_exhaustive();
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto is_always_strided() noexcept -> bool {
        return mapping_type::is_always_strided();
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
        m_data.fill(value);
        return;
    }

    // The mapping is fully determined by the static extents.
    static constexpr auto serialize(auto& archive, auto& self) -> zpp::bits::errc {
        return archive(self.m_data);
    }

  private:
    friend zpp::bits::access;

    [[using gnu: pure, always_inline]]
    constexpr auto strided_mapping() const noexcept
        -> std::layout_stride::mapping<std::dextents<index_type, 2>> {
        return {
            std::dextents<index_type, 2>{nrows(), ncols()},
            std::array<index_type, 2>{stride(0), stride(1)}
        };
    }

    storage_type m_data;
    mapping_type m_map;
};

template <ScalarArithmetic Scalar, std::integral Index, DenseLayout Layout, Allocatory Alloc>
class Matrix<Scalar, Index, std::dynamic_extent, std::dynamic_extent, Layout, Alloc> final
    : public MatrixBase {
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
    constexpr Matrix(const Matrix& other) = default;
    constexpr auto operator=(const Matrix& other) -> Matrix& = default;
    constexpr Matrix(Matrix&& other) noexcept = default;
    constexpr auto operator=(Matrix&& other) noexcept -> Matrix& = default;
    constexpr ~Matrix() noexcept = default;

    [[using gnu: pure, always_inline]]
    constexpr auto get_allocator() const noexcept -> allocator_type {
        return m_data.get_allocator();
    }

    [[using gnu: always_inline]]
    constexpr explicit Matrix(const allocator_type& alloc) noexcept
        : m_data(alloc), m_map{} {}

    [[using gnu: always_inline]]
    constexpr explicit Matrix(index_type rows, index_type cols, const allocator_type& alloc = {})
        : m_data(static_cast<size_type>(rows) * static_cast<size_type>(cols), alloc),
          m_map{extents_type{rows, cols}} {
        m_data.shrink_to_fit();
    }

    [[using gnu: always_inline]]
    constexpr explicit Matrix(
        index_type rows, index_type cols, const_reference value, const allocator_type& alloc = {}
    )
        : m_data(static_cast<size_type>(rows) * static_cast<size_type>(cols), value, alloc),
          m_map{extents_type{rows, cols}} {
        m_data.shrink_to_fit();
    }

    template <typename T>
        requires std::same_as<std::remove_cv_t<typename T::value_type>, value_type> &&
                     (T::rank() == 2)
    [[using gnu: always_inline]]
    constexpr explicit Matrix(const T& matrix_like, const allocator_type& alloc = {})
        : m_data(
              static_cast<size_type>(matrix_like.extent(0)) *
                  static_cast<size_type>(matrix_like.extent(1)),
              alloc
          ),
          m_map{extents_type{matrix_like.extents()}} {
        using other_index_type = typename T::index_type;

        m_data.shrink_to_fit();
        for (index_type i{0}; i < nrows(); ++i) {
            for (index_type j{0}; j < ncols(); ++j) {
                operator[](i, j) =
                    matrix_like[static_cast<other_index_type>(i), static_cast<other_index_type>(j)];
            }
        }
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::mdspan<value_type, extents_type, layout_type>() noexcept {
        return {data(), m_map};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::mdspan<const value_type, extents_type, layout_type>() const noexcept {
        return {data(), m_map};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::mdspan<
        value_type, std::dextents<index_type, 2>, std::layout_stride>() noexcept {
        return {data(), strided_mapping()};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::mdspan<
        const value_type, std::dextents<index_type, 2>, std::layout_stride>() const noexcept {
        return {data(), strided_mapping()};
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
    constexpr auto extents() const noexcept -> const extents_type& {
        return m_map.extents();
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto mapping() const noexcept -> const mapping_type& {
        return m_map;
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto extent(rank_type r) const noexcept -> index_type {
        return m_map.extents().extent(r);
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto nrows() const noexcept -> index_type {
        return m_map.extents().extent(0);
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto ncols() const noexcept -> index_type {
        return m_map.extents().extent(1);
    }

    [[using gnu: pure, always_inline]]
    constexpr auto size() const noexcept -> size_type {
        return m_data.size();
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto stride(rank_type r) const noexcept -> index_type {
        return m_map.stride(r);
    }

    [[using gnu: pure, always_inline]]
    constexpr auto is_unique() const noexcept -> bool {
        return m_map.is_unique();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto is_exhaustive() const noexcept -> bool {
        return m_map.is_exhaustive();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto is_strided() const noexcept -> bool {
        return m_map.is_strided();
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto is_always_unique() noexcept -> bool {
        return mapping_type::is_always_unique();
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto is_always_exhaustive() noexcept -> bool {
        return mapping_type::is_always_exhaustive();
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto is_always_strided() noexcept -> bool {
        return mapping_type::is_always_strided();
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
    constexpr auto resize(index_type rows, index_type cols) -> void {
        m_data.resize(static_cast<size_type>(rows) * static_cast<size_type>(cols));
        m_data.shrink_to_fit();
        m_map = mapping_type{extents_type{rows, cols}};
        return;
    }

    [[using gnu: always_inline]]
    constexpr auto assign(index_type rows, index_type cols, const_reference value) -> void {
        m_data.assign(static_cast<size_type>(rows) * static_cast<size_type>(cols), value);
        m_data.shrink_to_fit();
        m_map = mapping_type{extents_type{rows, cols}};
        return;
    }

    static constexpr auto serialize(auto& archive, auto& self) -> zpp::bits::errc {
        using archive_type = std::remove_cvref_t<decltype(archive)>;

        if constexpr (archive_type::kind() == zpp::bits::kind::out) {
            const index_type rows{self.nrows()}, cols{self.ncols()};
            return archive(self.m_data, rows, cols);
        } else {
            index_type rows{0}, cols{0};
            if (auto result = archive(self.m_data, rows, cols); failure(result)) {
                return result;
            }
            self.m_map = mapping_type{extents_type{rows, cols}};
            return {};
        }
    }

  private:
    friend zpp::bits::access;

    [[using gnu: pure, always_inline]]
    constexpr auto strided_mapping() const noexcept
        -> std::layout_stride::mapping<std::dextents<index_type, 2>> {
        return {
            std::dextents<index_type, 2>{nrows(), ncols()},
            std::array<index_type, 2>{stride(0), stride(1)}
        };
    }

    storage_type m_data;
    mapping_type m_map;
};

} // namespace basjoo::math
