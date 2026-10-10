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
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include "zpp_bits.h"

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_trait.hpp"
#include "basjoo/math/dense/matrix_base.hpp"

namespace basjoo::math {

// A single class template covering both storage shapes, mirroring std::mdspan:
// the extents and layout live in the type; the mapping keeps the runtime
// dimensions (zero storage when they are compile-time); element access goes
// through the accessor policy. Fully-static extents store the data inline
// (std::array, the former Matrix); fully-dynamic extents store it on the heap
// through Alloc (the former MatrixX). Partially-dynamic extents are rejected
// statically — use a fully-static or fully-dynamic extents type.
template <
    ScalarArithmetic Scalar, DenseExtents Extents, DenseLayout Layout, DenseAccessor AccessorPolicy,
    Allocatory Alloc>
class alignas(32) Matrix final : public MatrixBase {
  public:
    using element_type = typename DenseTrait<Matrix>::element_type;
    using value_type = typename DenseTrait<Matrix>::value_type;
    using reference = typename DenseTrait<Matrix>::reference;
    using const_reference = typename DenseTrait<Matrix>::const_reference;
    using pointer = typename DenseTrait<Matrix>::pointer;
    using const_pointer = typename DenseTrait<Matrix>::const_pointer;
    using size_type = typename DenseTrait<Matrix>::size_type;
    using difference_type = typename DenseTrait<Matrix>::difference_type;
    using index_type = typename DenseTrait<Matrix>::index_type;
    using rank_type = typename DenseTrait<Matrix>::rank_type;
    using data_handle_type = typename DenseTrait<Matrix>::data_handle_type;
    using allocator_type = typename DenseTrait<Matrix>::allocator_type;
    using extents_type = typename DenseTrait<Matrix>::extents_type;
    using layout_type = typename DenseTrait<Matrix>::layout_type;
    using accessor_type = typename DenseTrait<Matrix>::accessor_type;
    using mapping_type = typename DenseTrait<Matrix>::mapping_type;
    using storage_type = typename DenseTrait<Matrix>::storage_type;

    static_assert(Extents::rank() == 2, "Matrix requires a rank-2 extents type");
    static_assert(
        Extents::rank_dynamic() != 1,
        "partially-dynamic extents are not supported; use a fully-static or "
        "fully-dynamic extents type"
    );

    [[using gnu: always_inline]]
    Matrix() noexcept = default;
    Matrix(const Matrix& other) = default;
    auto operator=(const Matrix& other) -> Matrix& = default;
    Matrix(Matrix&& other) noexcept = default;
    auto operator=(Matrix&& other) noexcept -> Matrix& = default;
    ~Matrix() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type
        requires(Extents::rank_dynamic() == 0)
    {
        return allocator_type{};
    }

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type
        requires(Extents::rank_dynamic() == 2)
    {
        return m_data.get_allocator();
    }

    [[using gnu: always_inline]]
    explicit Matrix([[maybe_unused]] const allocator_type& alloc) noexcept
        requires(Extents::rank_dynamic() == 0)
    {}

    [[using gnu: always_inline]]
    explicit Matrix(const allocator_type& alloc) noexcept
        requires(Extents::rank_dynamic() == 2)
        : m_data(alloc) {}

    // Fully-dynamic construction (the former MatrixX surface).
    [[using gnu: always_inline]]
    explicit Matrix(size_type rows, size_type cols, const allocator_type& alloc = {})
        requires(Extents::rank_dynamic() == 2)
        : m_data(rows * cols, alloc), m_map{extents_type{rows, cols}} {
        m_data.shrink_to_fit();
    }

    [[using gnu: always_inline]]
    explicit Matrix(
        size_type rows, size_type cols, const_reference value, const allocator_type& alloc = {}
    )
        requires(Extents::rank_dynamic() == 2)
        : m_data(rows * cols, value, alloc), m_map{extents_type{rows, cols}} {
        m_data.shrink_to_fit();
    }

    // Fully-static construction (the former Matrix surface).
    [[using gnu: always_inline]]
    explicit Matrix(
        [[maybe_unused]] size_type rows, [[maybe_unused]] size_type cols,
        [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        requires(Extents::rank_dynamic() == 0)
    {
#if BASJOO_CHECK_PARAMS == 1
        if (rows != nrows() || cols != ncols()) [[unlikely]] {
            throw std::invalid_argument("Matrix constructor: mismatch matrix sizes detected.");
        }
#endif
    }

    [[using gnu: always_inline]]
    explicit Matrix(
        [[maybe_unused]] size_type rows, [[maybe_unused]] size_type cols, const_reference value,
        [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        requires(Extents::rank_dynamic() == 0)
    {
#if BASJOO_CHECK_PARAMS == 1
        if (rows != nrows() || cols != ncols()) [[unlikely]] {
            throw std::invalid_argument("Matrix constructor: mismatch matrix sizes detected.");
        }
#endif
        m_data.fill(value);
    }

    template <typename U>
    [[using gnu: always_inline]]
    explicit Matrix(
        std::mdspan<U, std::dextents<std::size_t, 2>, std::layout_stride> matrix_mdspan,
        [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        requires(Extents::rank_dynamic() == 0) && std::is_same_v<std::remove_const_t<U>, value_type>
    {
#if BASJOO_CHECK_PARAMS == 1
        if (matrix_mdspan.extent(0) != nrows() || matrix_mdspan.extent(1) != ncols()) [[unlikely]] {
            throw std::invalid_argument("Matrix constructor: mismatch matrix sizes detected.");
        }
#endif
        for (size_type i{0}; i < nrows(); ++i) {
            for (size_type j{0}; j < ncols(); ++j) {
                operator[](i, j) = matrix_mdspan[i, j];
            }
        }
        return;
    }

    template <typename U>
    [[using gnu: always_inline]]
    explicit Matrix(
        std::mdspan<U, std::dextents<std::size_t, 2>, std::layout_stride> matrix_mdspan,
        const allocator_type& alloc = {}
    )
        requires(Extents::rank_dynamic() == 2) && std::is_same_v<std::remove_const_t<U>, value_type>
        : m_data(matrix_mdspan.extent(0) * matrix_mdspan.extent(1), alloc),
          m_map{extents_type{matrix_mdspan.extent(0), matrix_mdspan.extent(1)}} {
        m_data.shrink_to_fit();
        for (size_type i{0}; i < nrows(); ++i) {
            for (size_type j{0}; j < ncols(); ++j) {
                operator[](i, j) = matrix_mdspan[i, j];
            }
        }
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::mdspan<
        value_type, std::dextents<std::size_t, 2>, std::layout_stride,
        std::default_accessor<value_type>>() noexcept {
        return std::mdspan<
            value_type, std::dextents<std::size_t, 2>, std::layout_stride,
            std::default_accessor<value_type>>{
            data(), std::layout_stride::mapping<std::dextents<std::size_t, 2>>{
                        std::dextents<std::size_t, 2>{nrows(), ncols()},
                        std::array<std::size_t, 2>{stride(0), stride(1)}
                    }
        };
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::mdspan<
        const value_type, std::dextents<std::size_t, 2>, std::layout_stride,
        std::default_accessor<const value_type>>() const noexcept {
        return std::mdspan<
            const value_type, std::dextents<std::size_t, 2>, std::layout_stride,
            std::default_accessor<const value_type>>{
            data(), std::layout_stride::mapping<std::dextents<std::size_t, 2>>{
                        std::dextents<std::size_t, 2>{nrows(), ncols()},
                        std::array<std::size_t, 2>{stride(0), stride(1)}
                    }
        };
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto rank() noexcept -> std::size_t {
        return 2;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto rank_dynamic() noexcept -> std::size_t {
        return extents_type::rank_dynamic();
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto static_extent(std::size_t r) noexcept -> std::size_t {
        return extents_type::static_extent(r);
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto extents() const noexcept -> extents_type {
        return m_map.extents();
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto mapping() const noexcept -> mapping_type {
        return m_map;
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto extent(std::size_t r) const noexcept -> size_type {
        return m_map.extents().extent(r);
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto nrows() const noexcept -> size_type {
        return m_map.extents().extent(0);
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto ncols() const noexcept -> size_type {
        return m_map.extents().extent(1);
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto size() const noexcept -> size_type {
        return nrows() * ncols();
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto stride(std::size_t r) const noexcept -> size_type {
        return m_map.stride(r);
    }

    // std::mdspan-shaped queries. The owning storage is always unique and
    // exhaustive, and layout_left/layout_right are always expressible with
    // constant strides.
    [[using gnu: pure, always_inline]]
    constexpr auto data_handle() const noexcept -> const_pointer {
        return data();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto is_exhaustive() const noexcept -> bool {
        return m_map.is_exhaustive();
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

    [[using gnu: always_inline]]
    auto resize(size_type rows, size_type cols) -> void
        requires(Extents::rank_dynamic() == 2)
    {
        m_data.resize(rows * cols);
        m_data.shrink_to_fit();
        m_map = mapping_type{extents_type{rows, cols}};
        return;
    }

    [[using gnu: always_inline]]
    static constexpr auto resize([[maybe_unused]] size_type rows, [[maybe_unused]] size_type cols)
        -> void
        requires(Extents::rank_dynamic() == 0)
    {
#if BASJOO_CHECK_PARAMS == 1
        if (rows != nrows() || cols != ncols()) [[unlikely]] {
            throw std::invalid_argument("Matrix reshape: mismatch matrix sizes detected.");
        }
#endif
        return;
    }

    [[using gnu: always_inline]]
    auto assign(size_type rows, size_type cols, const_reference value) -> void
        requires(Extents::rank_dynamic() == 2)
    {
        m_data.assign(rows * cols, value);
        m_data.shrink_to_fit();
        m_map = mapping_type{extents_type{rows, cols}};
        return;
    }

    [[using gnu: always_inline]]
    constexpr auto assign(
        [[maybe_unused]] size_type rows, [[maybe_unused]] size_type cols,
        [[maybe_unused]] const_reference value
    ) -> void
        requires(Extents::rank_dynamic() == 0)
    {
#if BASJOO_CHECK_PARAMS == 1
        if (rows != nrows() || cols != ncols()) [[unlikely]] {
            throw std::invalid_argument("Matrix assign: mismatch matrix sizes detected.");
        }
#endif
        m_data.fill(value);
        return;
    }

    // zpp hook — public: the library's requires-probe for
    // `Type::serialize(archive, item)` cannot reach a private member even
    // with a friend declaration. Wire format follows the extents:
    // fully-static serializes the flat array, fully-dynamic serializes data +
    // both extents and rebuilds the mapping — matching the layouts of the
    // Matrix/MatrixX classes this template replaces. The mapping itself
    // carries no state for static extents, and for dynamic extents it is
    // reconstructed from the two serialized dimensions, so it is never
    // archived directly.
    static constexpr auto serialize(auto& archive, const Matrix& self) -> zpp::bits::errc {
        if constexpr (extents_type::rank_dynamic() == 0) {
            return archive(self.m_data);
        } else {
            return archive(self.m_data, self.nrows(), self.ncols());
        }
    }
    static constexpr auto serialize(auto& archive, Matrix& self) -> zpp::bits::errc {
        if constexpr (extents_type::rank_dynamic() == 0) {
            return archive(self.m_data);
        } else {
            std::size_t rows{self.nrows()};
            std::size_t cols{self.ncols()};
            if (auto result = archive(self.m_data, rows, cols); failure(result)) {
                return result;
            }
            self.m_map = mapping_type{extents_type{rows, cols}};
            return zpp::bits::errc{};
        }
    }

  private:
    friend zpp::bits::access;

    // m_data comes first: for the static shape the inline array's alignment
    // can only come from the class-level alignas(32) through member order.
    storage_type m_data;
    mapping_type m_map{};
    [[no_unique_address]] accessor_type m_accessor{};
};

} // namespace basjoo::math
