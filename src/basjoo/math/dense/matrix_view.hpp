/**
 * @file matrix_view.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-04-01
 *
 * @copyright Copyright (c) 2025 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <cmath>
#include <iomanip>
#include <ostream>
#include <stdexcept>
#include <type_traits>

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_traits.hpp"
#include "basjoo/math/dense/detail/dense_norm_trait.hpp"
#include "basjoo/math/type_traits.hpp"

namespace basjoo::math {

template <ScalarArithmetic Scalar, MatrixOrder Order>
class MatrixView final {
  public:
    using value_type = typename DenseTraits<MatrixView>::value_type;
    using reference = typename DenseTraits<MatrixView>::reference;
    using const_reference = typename DenseTraits<MatrixView>::const_reference;
    using pointer = typename DenseTraits<MatrixView>::pointer;
    using const_pointer = typename DenseTraits<MatrixView>::const_pointer;
    using size_type = typename DenseTraits<MatrixView>::size_type;
    using difference_type = typename DenseTraits<MatrixView>::difference_type;

    constexpr MatrixView() noexcept = delete;
    constexpr MatrixView(const MatrixView& other) noexcept = default;
    constexpr auto operator=(const MatrixView& other) noexcept -> MatrixView& = default;
    constexpr MatrixView(MatrixView&& other) noexcept = default;
    constexpr auto operator=(MatrixView&&) noexcept -> MatrixView& = default;
    constexpr ~MatrixView() noexcept = default;

    [[using gnu: always_inline]]
    constexpr explicit MatrixView(pointer data, size_type nrows, size_type ncols) noexcept
        : m_data{data}, m_nrows{nrows}, m_ncols{ncols},
          m_stride{order() == MatrixOrder::COL_MAJOR ? nrows : ncols} {}

    [[using gnu: always_inline]]
    constexpr explicit MatrixView(
        pointer data, size_type rows, size_type cols, size_type stride
    ) noexcept
        : m_data{data}, m_nrows{rows}, m_ncols{cols}, m_stride{stride} {}

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto nrows() const noexcept -> size_type {
        return m_nrows;
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto ncols() const noexcept -> size_type {
        return m_ncols;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto order() noexcept -> MatrixOrder {
        return Order;
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto size() const noexcept -> size_type {
        return m_nrows * m_ncols;
    }

    [[using gnu: const, always_inline, leaf]]
    constexpr auto stride() const noexcept -> size_type {
        return m_stride;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() noexcept -> pointer {
        return m_data;
    }
    [[using gnu: pure, always_inline]]
    constexpr auto data() const noexcept -> const_pointer {
        return m_data;
    }

    [[using gnu: always_inline, hot]]
    constexpr auto fill(const_reference value) noexcept -> void {
        for (size_type i{0}; i < m_nrows; ++i) {
            for (size_type j{0}; j < m_ncols; ++j) {
                operator[](i, j) = value;
            }
        }
        return;
    }

    [[using gnu: const, always_inline, leaf]]
    constexpr auto empty() noexcept -> bool {
        return m_nrows * m_ncols == 0;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator[](size_type row, size_type col) noexcept -> reference {
        return m_data[offset(row, col)];
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator[](size_type row, size_type col) const noexcept -> const_reference {
        return m_data[offset(row, col)];
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator[](size_type i) noexcept -> reference {
        return m_data[offset(i)];
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator[](size_type i) const noexcept -> const_reference {
        return m_data[offset(i)];
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto coeff(size_type row, size_type col) const noexcept(!BASJOO_CHECK_PARAMS)
        -> value_type {
#if BASJOO_CHECK_PARAMS == 1
        if (row >= m_nrows || col >= m_ncols) {
            throw std::out_of_range("Matrix index out of range");
        }
#endif
        return operator[](row, col);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto updateCoeff(size_type row, size_type col, const_reference value) noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (row >= m_nrows || col >= m_ncols) {
            throw std::out_of_range("Matrix index out of range");
        }
#endif
        operator[](row, col) = value;
        return;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto coeff(size_type i) const noexcept(!BASJOO_CHECK_PARAMS) -> value_type {
#if BASJOO_CHECK_PARAMS == 1
        if (i >= size()) {
            throw std::out_of_range("Matrix index out of range.");
        }
#endif
        return operator[](i);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto updateCoeff(size_type i, const_reference value) const noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (i >= size()) {
            throw std::out_of_range("Matrix index out of range.");
        }
#endif
        operator[](i) = value;
        return;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto view(
        std::pair<size_type, size_type> row_range, std::pair<size_type, size_type> col_range
    ) noexcept -> MatrixView<value_type, Order> {
        row_range.second = std::min(row_range.second, m_nrows);
        col_range.second = std::min(col_range.second, m_ncols);
        return MatrixView<value_type, Order>(
            data() + offset(row_range.first, col_range.first), row_range.second - row_range.first,
            col_range.second - col_range.first, stride()
        );
    }

    [[using gnu: pure, always_inline]]
    constexpr auto view(
        std::pair<size_type, size_type> row_range, std::pair<size_type, size_type> col_range
    ) const noexcept -> MatrixView<const value_type, Order> {
        row_range.second = std::min(row_range.second, m_nrows);
        col_range.second = std::min(col_range.second, m_ncols);
        return MatrixView<const value_type, Order>(
            data() + offset(row_range.first, col_range.first), row_range.second - row_range.first,
            col_range.second - col_range.first, stride()

        );
    }

    [[using gnu: pure, always_inline]]
    constexpr auto view(std::pair<size_type, size_type> range) noexcept(!BASJOO_CHECK_PARAMS)
        -> MatrixView<value_type, Order> {
#if BASJOO_CHECK_PARAMS == 1
        if (m_nrows != 1 && m_ncols != 1) {
            throw std::invalid_argument("vector does not have setIdentity method.");
        }
#endif
        return view(
            m_ncols == 1 ? range : std::pair<size_type, size_type>{0, 1},
            m_ncols == 1 ? std::pair<size_type, size_type>{0, 1} : range
        );
    }

    [[using gnu: pure, always_inline]]
    constexpr auto view(std::pair<size_type, size_type> range) const noexcept(!BASJOO_CHECK_PARAMS)
        -> MatrixView<const value_type, Order> {
#if BASJOO_CHECK_PARAMS == 1
        if (m_nrows != 1 && m_ncols != 1) {
            throw std::invalid_argument("vector does not have setIdentity method.");
        }
#endif
        return view(
            m_ncols == 1 ? range : std::pair<size_type, size_type>{0, 1},
            m_ncols == 1 ? std::pair<size_type, size_type>{0, 1} : range
        );
    }

    [[using gnu: pure, always_inline]]
    constexpr auto view() noexcept -> MatrixView<value_type, Order> {
        return view(
            {0, std::numeric_limits<value_type>::max()}, {0, std::numeric_limits<value_type>::max()}
        );
    }

    [[using gnu: pure, always_inline]]
    constexpr auto view() const noexcept -> MatrixView<const value_type, Order> {
        return view(
            {0, std::numeric_limits<value_type>::max()}, {0, std::numeric_limits<value_type>::max()}
        );
    }

    [[using gnu: always_inline]]
    constexpr auto setIdentity() noexcept -> void {
        constexpr size_type n{std::min(m_nrows, m_ncols)};
        fill(0.0);
        for (size_type i{0}; i < n; ++i) {
            operator[](i, i) = 1.0;
        }
        return;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator==(const MatrixView& other) const noexcept -> bool {
        if (m_nrows != other.m_nrows || m_ncols != other.m_ncols) [[unlikely]] {
            return false;
        }
        for (size_type i{0}; i < m_nrows; ++i) {
            for (size_type j{0}; j < m_ncols; ++j) {
                if (operator[](i, j) != other[i, j]) {
                    return false;
                }
            }
        }
        return true;
    }

    [[using gnu: pure, always_inline, hot]]
    auto identicalTo(MatrixView<value_type, Order> obj, double tol = 1E-8) const noexcept -> bool {
        if (m_nrows != obj.nrows() || m_ncols != obj.ncols()) {
            return false;
        }
        const size_type n = size();
        detail::DenseNormTraitT<value_type> test(0.0);
        for (size_type i{0}; i < n; ++i) {
            test = std::max(
                test, std::abs(operator[](i) - obj[i]) /
                          std::max(std::abs(operator[](i)), static_cast<decltype(test)>(1.0))
            );
        }
        return test < tol;
    }

  private:
    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto offset(size_type row, size_type col) const noexcept -> size_type {
        if constexpr (order() == ::basjoo::math::MatrixOrder::COL_MAJOR) {
            return (col * m_stride) + row;
        }
        if constexpr (order() == ::basjoo::math::MatrixOrder::ROW_MAJOR) {
            return (row * m_stride) + col;
        }
    }

    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto offset(size_type i) const noexcept -> size_type {
        if constexpr (order() == ::basjoo::math::MatrixOrder::COL_MAJOR) {
            return i / m_nrows * m_stride + i % m_nrows;
        }
        if constexpr (order() == ::basjoo::math::MatrixOrder::ROW_MAJOR) {
            return i / m_ncols * m_stride + i % m_ncols;
        }
    }

    pointer m_data;
    std::size_t m_nrows, m_ncols, m_stride;
};

template <typename Char, ScalarArithmetic Scalar, MatrixOrder Order>
inline auto operator<<(std::basic_ostream<Char>& os, MatrixView<Scalar, Order> matrix_view) noexcept
    -> std::basic_ostream<Char>& {
    using value_type = std::remove_const_t<Scalar>;
    using size_type = typename MatrixView<Scalar, Order>::size_type;
    constexpr size_type kWidth{isComplexArithmeticV<value_type> ? 32 : 16};
    const size_type nrows{matrix_view.nrows()}, ncols{matrix_view.ncols()};
    os << std::fixed;
    for (size_type i{0}; i < nrows; ++i) {
        for (size_type j{0}; j < ncols; ++j) {
            os << std::setw(kWidth) << matrix_view[i, j];
        }
        os << '\n';
    }
    return os;
}

} // namespace basjoo::math
