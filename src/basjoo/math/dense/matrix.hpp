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

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <iomanip>
#include <numeric>
#include <ostream>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "zpp_bits.h"

#ifdef BASJOO_USE_BLAS_LAPACK
#include "cblas.h"
#include "lapacke.h"
#endif

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_traits.hpp"
#include "basjoo/math/dense/detail/dense_norm_trait.hpp"
#include "basjoo/math/dense/matrix_view.hpp"
#include "basjoo/math/type_traits.hpp"

namespace basjoo::math {

template <ScalarArithmetic Scalar, std::size_t NRows, std::size_t NCols, MatrixOrder Order>
class alignas(32) Matrix final {
  public:
    using value_type = typename DenseTraits<Matrix>::value_type;
    using reference = typename DenseTraits<Matrix>::reference;
    using const_reference = typename DenseTraits<Matrix>::const_reference;
    using pointer = typename DenseTraits<Matrix>::pointer;
    using const_pointer = typename DenseTraits<Matrix>::const_pointer;
    using size_type = typename DenseTraits<Matrix>::size_type;
    using difference_type = typename DenseTraits<Matrix>::difference_type;
    using allocator_type = typename DenseTraits<Matrix>::allocator_type;

    constexpr Matrix(const Matrix& other) noexcept = default;
    constexpr auto operator=(const Matrix& other) noexcept -> Matrix& = default;
    constexpr Matrix(Matrix&& other) noexcept = default;
    constexpr auto operator=(Matrix&& other) noexcept -> Matrix& = default;
    constexpr ~Matrix() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return allocator_type{};
    }

    [[using gnu: always_inline]]
    explicit Matrix([[maybe_unused]] const allocator_type& alloc = {}) noexcept {}

    [[using gnu: always_inline]]
    explicit Matrix(
        [[maybe_unused]] size_type nrows, [[maybe_unused]] size_type ncols,
        [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS) {
#if BASJOO_CHECK_PARAMS == 1
        if (nrows != nrows() || ncols != ncols()) [[unlikely]] {
            throw std::invalid_argument("Matrix constructor: mismatch matrix sizes detected.");
        }
#endif
    }

    [[using gnu: always_inline]]
    constexpr explicit Matrix(
        [[maybe_unused]] size_type nrows, [[maybe_unused]] size_type ncols, const_reference value,
        [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS) {
#if BASJOO_CHECK_PARAMS == 1
        if (nrows != nrows() || ncols != ncols()) [[unlikely]] {
            throw std::invalid_argument("Matrix constructor: mismatch matrix sizes detected.");
        }
#endif
        m_data.fill(value);
    }

    [[using gnu: always_inline]]
    constexpr explicit Matrix(
        MatrixView<value_type, Order> matrix_view, [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS) {
#if BASJOO_CHECK_PARAMS == 1
        if (matrix_view.nrows() != nrows() || matrix_view.ncols() != ncols()) [[unlikely]] {
            throw std::invalid_argument("Matrix constructor: mismatch matrix sizes detected.");
        }
#endif
        for (size_type i{0}; i < nrows(); ++i) {
            for (size_type j{0}; j < ncols(); ++j) {
                operator[](i, j) = matrix_view[i, j];
            }
        }
        return;
    }

    [[using gnu: always_inline]]
    constexpr explicit Matrix(
        MatrixView<const value_type, Order> matrix_view,
        [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS) {
#if BASJOO_CHECK_PARAMS == 1
        if (matrix_view.nrows() != nrows() || matrix_view.ncols() != ncols()) [[unlikely]] {
            throw std::invalid_argument("Matrix constructor: mismatch matrix sizes detected.");
        }
#endif
        for (size_type i{0}; i < nrows(); ++i) {
            for (size_type j{0}; j < ncols(); ++j) {
                operator[](i, j) = matrix_view[i, j];
            }
        }
        return;
    }

    [[using gnu: pure, always_inline]]
    constexpr operator MatrixView<value_type, Order>() noexcept {
        return MatrixView<value_type, Order>(m_data.data(), nrows(), ncols());
    }

    [[using gnu: pure, always_inline]]
    constexpr operator MatrixView<const value_type, Order>() const noexcept {
        return MatrixView<const value_type, Order>(m_data.data(), nrows(), ncols());
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto nrows() noexcept -> size_type {
        return NRows;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto ncols() noexcept -> size_type {
        return NCols;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto order() noexcept -> MatrixOrder {
        return Order;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto size() noexcept -> size_type {
        return NRows * NCols;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto stride() noexcept -> size_type {
        if constexpr (order() == MatrixOrder::COL_MAJOR) {
            return nrows();
        }
        if constexpr (order() == MatrixOrder::ROW_MAJOR) {
            return ncols();
        }
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
    static constexpr auto resize([[maybe_unused]] size_type nrows, [[maybe_unused]] size_type ncols)
        -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (nrows != nrows() || ncols != ncols()) [[unlikely]] {
            throw std::invalid_argument("Matrix reshape: mismatch matrix sizes detected.");
        }
#endif
        return;
    }

    [[using gnu: always_inline]]
    static constexpr auto resize([[maybe_unused]] size_type n) -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (n != size()) [[unlikely]] {
            throw std::invalid_argument("Matrix resize: mismatch matrix sizes detected.");
        }
#endif
        return;
    }

    [[using gnu: always_inline]]
    constexpr auto assign(
        [[maybe_unused]] size_type nrows, [[maybe_unused]] size_type ncols, const_reference value
    ) -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (nrows != nrows() || ncols != ncols()) [[unlikely]] {
            throw std::invalid_argument("Matrix assign: mismatch matrix sizes detected.");
        }
#endif
        std::ranges::fill_n(data(), size(), value);
        return;
    }

    [[using gnu: always_inline]]
    constexpr auto assign([[maybe_unused]] size_type n, const_reference value) -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (n != size()) [[unlikely]] {
            throw std::invalid_argument("Matrix assign: mismatch matrix sizes detected.");
        }
#endif
        std::ranges::fill_n(data(), size(), value);
        return;
    }

    [[using gnu: always_inline, hot]]
    auto fill(const_reference value) noexcept -> void {
        std::ranges::fill_n(data(), size(), value);
        return;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto empty() noexcept -> bool {
        return size() == 0;
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
        if (row >= nrows() || col >= ncols()) [[unlikely]] {
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
        if (row >= nrows() || col >= ncols()) [[unlikely]] {
            throw std::out_of_range("Matrix index out of range");
        }
#endif
        operator[](row, col) = value;
        return;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto coeff(size_type i) const noexcept(!BASJOO_CHECK_PARAMS) -> value_type {
#if BASJOO_CHECK_PARAMS == 1
        if (i >= size()) [[unlikely]] {
            throw std::out_of_range("Matrix index out of range.");
        }
#endif
        return operator[](i);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto updateCoeff(size_type i, const_reference value) noexcept(!BASJOO_CHECK_PARAMS)
        -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (i >= size()) [[unlikely]] {
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
        row_range.second = std::min(row_range.second, nrows());
        col_range.second = std::min(col_range.second, ncols());
        return MatrixView<value_type, Order>(
            data() + offset(row_range.first, col_range.first), row_range.second - row_range.first,
            col_range.second - col_range.first, stride()

        );
    }

    [[using gnu: pure, always_inline]]
    constexpr auto view(
        std::pair<size_type, size_type> row_range, std::pair<size_type, size_type> col_range
    ) const noexcept -> MatrixView<const value_type, Order> {
        row_range.second = std::min(row_range.second, nrows());
        col_range.second = std::min(col_range.second, ncols());
        return MatrixView<const value_type, Order>(
            data() + offset(row_range.first, col_range.first), row_range.second - row_range.first,
            col_range.second - col_range.first, stride()

        );
    }

    [[using gnu: pure, always_inline]]
    constexpr auto view(std::pair<size_type, size_type> range) noexcept(!BASJOO_CHECK_PARAMS)
        -> MatrixView<value_type, Order> {
#if BASJOO_CHECK_PARAMS == 1
        if (nrows() != 1 && ncols() != 1) [[unlikely]] {
            throw std::invalid_argument("vector does not have setIdentity method.");
        }
#endif
        return view(
            ncols() == 1 ? range : std::pair<size_type, size_type>{0, 1},
            ncols() == 1 ? std::pair<size_type, size_type>{0, 1} : range
        );
    }

    [[using gnu: pure, always_inline]]
    constexpr auto view(std::pair<size_type, size_type> range) const noexcept(!BASJOO_CHECK_PARAMS)
        -> MatrixView<const value_type, Order> {
#if BASJOO_CHECK_PARAMS == 1
        if (nrows() != 1 && ncols() != 1) [[unlikely]] {
            throw std::invalid_argument("vector does not have setIdentity method.");
        }
#endif
        return view(
            ncols() == 1 ? range : std::pair<size_type, size_type>{0, 1},
            ncols() == 1 ? std::pair<size_type, size_type>{0, 1} : range
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
        constexpr size_type n{std::min(nrows(), ncols())};
        fill(0.0);
        for (size_type i{0}; i < n; ++i) {
            operator[](i, i) = 1.0;
        }
        return;
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator+=(const Matrix& obj) noexcept -> Matrix& {
#ifdef BASJOO_USE_BLAS_LAPACK
        [[maybe_unused]] constexpr value_type alpha(1.0);
        if constexpr (std::is_same_v<value_type, float>) {
            cblas_saxpy(size(), alpha, obj.data(), 1, data(), 1);
        } else if constexpr (std::is_same_v<value_type, double>) {
            cblas_daxpy(size(), alpha, obj.data(), 1, data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            cblas_caxpy(size(), &alpha, obj.data(), 1, data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            cblas_zaxpy(size(), &alpha, obj.data(), 1, data(), 1);
        } else {
            std::transform(data(), data() + size(), obj.data(), data(), std::plus<value_type>());
        }
#else
        std::transform(data(), data() + size(), obj.data(), data(), std::plus<value_type>());
#endif
        return *this;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator+(const Matrix& obj) const& noexcept -> Matrix {
        Matrix result{*this};
        result += obj;
        return result;
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator+(Matrix&& obj) const& noexcept -> Matrix&& {
        obj += *this;
        return std::move(obj);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator+(const Matrix& obj) && noexcept -> Matrix&& {
        operator+=(obj);
        return std::move(*this);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator+(Matrix&& obj) && noexcept -> Matrix&& {
        operator+=(obj);
        return std::move(*this);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator-=(const Matrix& obj) noexcept -> Matrix& {
#ifdef BASJOO_USE_BLAS_LAPACK
        [[maybe_unused]] constexpr value_type alpha(-1.0);
        if constexpr (std::is_same_v<value_type, float>) {
            cblas_saxpy(size(), alpha, obj.data(), 1, data(), 1);
        } else if constexpr (std::is_same_v<value_type, double>) {
            cblas_daxpy(size(), alpha, obj.data(), 1, data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            cblas_caxpy(size(), &alpha, obj.data(), 1, data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            cblas_zaxpy(size(), &alpha, obj.data(), 1, data(), 1);
        } else {
            std::transform(data(), data() + size(), obj.data(), data(), std::minus<value_type>());
        }
#else
        std::transform(data(), data() + size(), obj.data(), data(), std::minus<value_type>());
#endif
        return *this;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator-(const Matrix& obj) const& noexcept -> Matrix {
        Matrix result{*this};
        result -= obj;
        return result;
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator-(Matrix&& obj) const& noexcept -> Matrix&& {
        obj *= -1.0;
        obj += *this;
        return std::move(obj);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator-(const Matrix& obj) && noexcept -> Matrix&& {
        operator-=(obj);
        return std::move(*this);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator-(Matrix&& obj) && noexcept -> Matrix&& {
        operator-=(obj);
        return std::move(*this);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator*=(const ScalarArithmetic auto& fac) noexcept -> Matrix& {
        const auto alpha{static_cast<value_type>(fac)};
#ifdef BASJOO_USE_BLAS_LAPACK
        if constexpr (std::is_same_v<value_type, float>) {
            cblas_sscal(size(), alpha, data(), 1);
        } else if constexpr (std::is_same_v<value_type, double>) {
            cblas_dscal(size(), alpha, data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            cblas_cscal(size(), &alpha, data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            cblas_zscal(size(), &alpha, data(), 1);
        } else {
            std::transform(
                data(), data() + size(), data(),
                [alpha](const value_type& x) constexpr noexcept -> value_type { return x * alpha; }
            );
        }
#else
        std::transform(
            data(), data() + size(), data(),
            [alpha](const value_type& x) constexpr noexcept -> value_type { return x * alpha; }
        );
#endif
        return *this;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator*(const ScalarArithmetic auto& fac) const& noexcept -> Matrix {
        Matrix result{*this};
        result *= fac;
        return result;
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator*(const ScalarArithmetic auto& fac) && noexcept -> Matrix&& {
        operator*=(fac);
        return std::move(*this);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator/=(const ScalarArithmetic auto& den) noexcept -> Matrix& {
        if constexpr (std::is_integral_v<value_type>) {
            const auto alpha{static_cast<value_type>(den)};
            std::transform(
                data(), data() + size(), data(),
                [alpha](const value_type& x) constexpr noexcept -> value_type { return x / alpha; }
            );
        } else {
            const auto alpha{static_cast<value_type>(1.0 / den)};
            operator*=(alpha);
        }
        return *this;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator/(const ScalarArithmetic auto& den) const& noexcept -> Matrix {
        Matrix result{*this};
        result /= den;
        return result;
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator/(const ScalarArithmetic auto& den) && noexcept -> Matrix&& {
        operator/=(den);
        return std::move(*this);
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator-() const& noexcept -> Matrix {
        Matrix result{*this};
        result *= -1.0;
        return result;
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator-() && noexcept -> Matrix&& {
        operator*=(-1.0);
        return std::move(*this);
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator==(const Matrix& other) const noexcept -> bool {
        constexpr size_type n{size()};
        for (size_type i{0}; i < n; ++i) {
            if (m_data[i] != other.m_data[i]) {
                return false;
            }
        }
        return true;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclidean() const noexcept -> detail::DenseNormTraitT<value_type> {
        detail::DenseNormTraitT<value_type> result(0.0);
        if constexpr (nrows() == 1 && ncols() == 1) {
            result = std::abs(m_data[0]);
        }
#ifdef BASJOO_USE_BLAS_LAPACK
        if constexpr (nrows() == 1 || ncols() == 1) {
            if constexpr (std::is_same_v<value_type, float>) {
                result = cblas_snrm2(size(), data(), 1);
            } else if constexpr (std::is_same_v<value_type, double>) {
                result = cblas_dnrm2(size(), data(), 1);
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                result = cblas_scnrm2(size(), data(), 1);
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                result = cblas_dznrm2(size(), data(), 1);
            } else {
                result = std::transform_reduce(
                    data(), data() + size(), 0.0, std::plus<detail::DenseNormTraitT<value_type>>{},
                    [](const value_type& a) constexpr noexcept
                        -> detail::DenseNormTraitT<value_type> { return std::norm(a); }
                );
                result = std::sqrt(result);
            }
        }
#else
        if constexpr (nrows() == 1 || ncols() == 1) {
            result = std::transform_reduce(
                data(), data() + size(), 0.0, std::plus<detail::DenseNormTraitT<value_type>>{},
                [](const value_type& a) constexpr noexcept -> detail::DenseNormTraitT<value_type> {
                    return std::norm(a);
                }
            );
            result = std::sqrt(result);
        }
#endif
        return result;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclideanSqr() const noexcept -> detail::DenseNormTraitT<value_type> {
        detail::DenseNormTraitT<value_type> result(0.0);
        if constexpr (nrows() == 1 && ncols() == 1) {
            result = std::norm(m_data[0]);
        }
#ifdef BASJOO_USE_BLAS_LAPACK
        if constexpr (nrows() == 1 || ncols() == 1) {
            if constexpr (std::is_same_v<value_type, float>) {
                result = cblas_sdot(size(), data(), 1, data(), 1);
            } else if constexpr (std::is_same_v<value_type, double>) {
                result = cblas_ddot(size(), data(), 1, data(), 1);
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                result = cblas_cdotc(size(), data(), 1, data(), 1);
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                result = cblas_zdotc(size(), data(), 1, data(), 1);
            } else {
                result = std::transform_reduce(
                    data(), data() + size(), 0.0, std::plus<detail::DenseNormTraitT<value_type>>{},
                    [](const value_type& a) constexpr noexcept
                        -> detail::DenseNormTraitT<value_type> { return std::norm(a); }
                );
            }
        }
#else
        if constexpr (nrows() == 1 || ncols() == 1) {
            result = std::transform_reduce(
                data(), data() + size(), 0.0, std::plus<detail::DenseNormTraitT<value_type>>{},
                [](const value_type& a) constexpr noexcept -> detail::DenseNormTraitT<value_type> {
                    return std::norm(a);
                }
            );
        }
#endif
        return result;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto identicalTo(
        MatrixView<const value_type, Order> obj, double tol = 1E-8
    ) const noexcept -> bool {
        if (nrows() != obj.nrows() || ncols() != obj.ncols()) {
            return false;
        }
        constexpr size_type n = size();
        detail::DenseNormTraitT<value_type> test(0.0);
        for (size_type i{0}; i < n; ++i) {
            test = std::max(
                test, std::abs(operator[](i) - obj[i]) /
                          std::max(std::abs(operator[](i)), static_cast<decltype(test)>(1.0))
            );
        }
        return test < tol;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto transposed() const noexcept -> Matrix<value_type, NCols, NRows, Order> {
        Matrix<value_type, NCols, NRows, Order> result;
        for (size_type i{0}; i < nrows(); ++i) {
            for (size_type j{0}; j < ncols(); ++j) {
                result[j, i] = operator[](i, j);
            }
        }
        return result;
    }

    [[using gnu: always_inline]]
    constexpr auto selfConjugated() noexcept -> Matrix& {
#ifdef BASJOO_USE_BLAS_LAPACK
        if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
            if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                LAPACKE_clacgv_work(size(), reinterpret_cast<lapack_complex_float*>(data()), 1);
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                LAPACKE_zlacgv_work(size(), reinterpret_cast<lapack_complex_double*>(data()), 1);
            } else {
                std::transform(
                    data(), data() + size(), data(),
                    [](const value_type& a) constexpr noexcept -> value_type {
                        return std::conj(a);
                    }
                );
            }
        }
#else
        if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
            std::transform(
                data(), data() + size(), data(),
                [](const value_type& a) constexpr noexcept -> value_type { return std::conj(a); }
            );
        }
#endif
        return *this;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto conjugated() const& noexcept -> Matrix {
        Matrix result{*this};
        if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
            result.selfConjugated();
        }
        return result;
    }

    [[using gnu: always_inline]]
    constexpr auto conjugated() && noexcept -> Matrix&& {
        if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
            selfConjugated();
        }
        return std::move(*this);
    }

    [[using gnu: pure, always_inline]]
    constexpr auto adjoint() const noexcept -> Matrix<value_type, NCols, NRows, Order> {
        return transposed().selfConjugated();
    }

    template <size_type OuterSize>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(const Matrix<value_type, NCols, OuterSize, Order>& obj) const noexcept
        -> Matrix<value_type, NRows, OuterSize, Order> {
        Matrix<value_type, NRows, OuterSize, Order> result(nrows(), OuterSize, 0.0);
#ifdef BASJOO_USE_BLAS_LAPACK
        [[maybe_unused]] constexpr CBLAS_ORDER cblas_order =
            (order() == ::basjoo::math::MatrixOrder::COL_MAJOR ? CblasColMajor : CblasRowMajor);
        [[maybe_unused]] constexpr blasint lda =
            (order() == ::basjoo::math::MatrixOrder::COL_MAJOR ? nrows() : ncols());
        [[maybe_unused]] constexpr blasint ldb =
            (order() == ::basjoo::math::MatrixOrder::COL_MAJOR ? ncols() : OuterSize);
        [[maybe_unused]] constexpr blasint ldc =
            (order() == ::basjoo::math::MatrixOrder::COL_MAJOR ? nrows() : OuterSize);
        [[maybe_unused]] constexpr value_type alpha(1.0), beta(0.0);
        if constexpr (std::is_same_v<value_type, float>) {
            cblas_sgemm(
                cblas_order, CblasNoTrans, CblasNoTrans, nrows(), OuterSize, ncols(), alpha, data(),
                lda, obj.data(), ldb, beta, result.data(), ldc
            );
        } else if constexpr (std::is_same_v<value_type, double>) {
            cblas_dgemm(
                cblas_order, CblasNoTrans, CblasNoTrans, nrows(), OuterSize, ncols(), alpha, data(),
                lda, obj.data(), ldb, beta, result.data(), ldc
            );
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            cblas_cgemm(
                cblas_order, CblasNoTrans, CblasNoTrans, nrows(), OuterSize, ncols(), &alpha,
                data(), lda, obj.data(), ldb, &beta, result.data(), ldc
            );
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            cblas_zgemm(
                cblas_order, CblasNoTrans, CblasNoTrans, nrows(), OuterSize, ncols(), &alpha,
                data(), lda, obj.data(), ldb, &beta, result.data(), ldc
            );
        } else {
            for (size_type i{0}; i < nrows(); ++i) {
                for (size_type j{0}; j < OuterSize; ++j) {
                    for (size_type k{0}; k < ncols(); ++k) {
                        result[i, j] += operator[](i, k) * obj[k, j];
                    }
                }
            }
        }
#else
        for (size_type i{0}; i < nrows(); ++i) {
            for (size_type j{0}; j < OuterSize; ++j) {
                for (size_type k{0}; k < ncols(); ++k) {
                    result[i, j] += operator[](i, k) * obj[k, j];
                }
            }
        }
#endif
        return result;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(const Vector<value_type, NCols>& obj) const noexcept
        -> Vector<value_type, NRows> {
        Vector<value_type, NRows> result(nrows(), 0.0);
#ifdef BASJOO_USE_BLAS_LAPACK
        [[maybe_unused]] constexpr CBLAS_ORDER cblas_order =
            (order() == ::basjoo::math::MatrixOrder::COL_MAJOR ? CblasColMajor : CblasRowMajor);
        [[maybe_unused]] constexpr blasint lda =
            (order() == ::basjoo::math::MatrixOrder::COL_MAJOR ? nrows() : ncols());
        [[maybe_unused]] constexpr value_type alpha(1.0), beta(0.0);
        if constexpr (std::is_same_v<value_type, float>) {
            cblas_sgemv(
                cblas_order, CblasNoTrans, nrows(), ncols(), alpha, data(), lda, obj.data(), 1,
                beta, result.data(), 1
            );
        } else if constexpr (std::is_same_v<value_type, double>) {
            cblas_dgemv(
                cblas_order, CblasNoTrans, nrows(), ncols(), alpha, data(), lda, obj.data(), 1,
                beta, result.data(), 1
            );
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            cblas_cgemv(
                cblas_order, CblasNoTrans, nrows(), ncols(), &alpha, data(), lda, obj.data(), 1,
                &beta, result.data(), 1
            );
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            cblas_zgemv(
                cblas_order, CblasNoTrans, nrows(), ncols(), &alpha, data(), lda, obj.data(), 1,
                &beta, result.data(), 1
            );
        } else {
            for (size_type i{0}; i < nrows(); ++i) {
                for (size_type j{0}; j < ncols(); ++j) {
                    result[i] += operator[](i, j) * obj[j];
                }
            }
        }
#else
        for (size_type i{0}; i < nrows(); ++i) {
            for (size_type j{0}; j < ncols(); ++j) {
                result[i] += operator[](i, j) * obj[j];
            }
        }
#endif
        return result;
    }

  private:
    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto offset(size_type row, size_type col) const noexcept -> size_type {
        if constexpr (order() == ::basjoo::math::MatrixOrder::COL_MAJOR) {
            return (col * nrows()) + row;
        }
        if constexpr (order() == ::basjoo::math::MatrixOrder::ROW_MAJOR) {
            return (row * ncols()) + col;
        }
    }

    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto offset(size_type i) const noexcept -> size_type {
        if constexpr (order() == ::basjoo::math::MatrixOrder::COL_MAJOR) {
            return i / nrows() * nrows() + i % nrows();
        }
        if constexpr (order() == ::basjoo::math::MatrixOrder::ROW_MAJOR) {
            return i / ncols() * ncols() + i % ncols();
        }
    }

    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    std::array<value_type, NRows * NCols> m_data;
};

template <ScalarArithmetic Scalar, std::size_t NRows, std::size_t NCols, MatrixOrder Order>
[[using gnu: pure, always_inline, hot]]
inline constexpr auto operator*(
    const ScalarArithmetic auto& fac, const Matrix<Scalar, NRows, NCols, Order>& obj
) noexcept -> Matrix<Scalar, NRows, NCols, Order> {
    return obj * fac;
}

template <ScalarArithmetic Scalar, std::size_t NRows, std::size_t NCols, MatrixOrder Order>
[[using gnu: always_inline, hot]]
inline constexpr auto operator*(
    const ScalarArithmetic auto& fac, Matrix<Scalar, NRows, NCols, Order>&& obj
) noexcept -> Matrix<Scalar, NRows, NCols, Order>&& {
    obj *= fac;
    return std::move(obj);
}

template <
    typename Char, ScalarArithmetic Scalar, std::size_t NRows, std::size_t NCols, MatrixOrder Order>
inline auto operator<<(
    std::basic_ostream<Char>& os, const Matrix<Scalar, NRows, NCols, Order>& matrix
) noexcept -> std::basic_ostream<Char>& {
    using value_type = std::remove_const_t<Scalar>;
    using size_type = typename Matrix<Scalar, NRows, NCols, Order>::size_type;
    constexpr size_type kWidth{isComplexArithmeticV<value_type> ? 32 : 16};
    const size_type nrows{matrix.nrows()}, ncols{matrix.ncols()};
    os << std::fixed;
    for (size_type i{0}; i < nrows; ++i) {
        for (size_type j{0}; j < ncols; ++j) {
            os << std::setw(kWidth) << matrix[i, j];
        }
        os << '\n';
    }
    return os;
}

} // namespace basjoo::math

// NOLINTBEGIN(cert-dcl58-cpp)

namespace std {

template <
    basjoo::math::ScalarArithmetic Scalar, std::size_t NRows, std::size_t NCols,
    basjoo::math::MatrixOrder Order>
[[using gnu: pure, always_inline]]
inline constexpr auto abs(const basjoo::math::Matrix<Scalar, NRows, NCols, Order>& matrix) noexcept
    -> basjoo::math::detail::DenseNormTrait<basjoo::math::Matrix<Scalar, NRows, NCols, Order>> {
    basjoo::math::detail::DenseNormTraitT<basjoo::math::Matrix<Scalar, NRows, NCols, Order>> result(
        matrix
    );
    if constexpr (basjoo::math::isComplexArithmeticV<Scalar>) {
        std::transform(
            result.data(), result.data() + result.size(), result.data(),
            [](const Scalar& x) constexpr noexcept
                -> basjoo::math::detail::DenseNormTraitT<Scalar> { return std::abs(x); }
        );
    }
    return result;
}

template <
    basjoo::math::ScalarArithmetic Scalar, std::size_t NRows, std::size_t NCols,
    basjoo::math::MatrixOrder Order>
[[using gnu: pure, always_inline]]
inline constexpr auto conj(const basjoo::math::Matrix<Scalar, NRows, NCols, Order>& matrix) noexcept
    -> basjoo::math::Matrix<Scalar, NRows, NCols, Order> {
    return matrix.conjugated();
}

template <
    basjoo::math::ScalarArithmetic Scalar, std::size_t NRows, std::size_t NCols,
    basjoo::math::MatrixOrder Order>
[[using gnu: always_inline]]
inline constexpr auto conj(basjoo::math::Matrix<Scalar, NRows, NCols, Order>&& matrix) noexcept
    -> basjoo::math::Matrix<Scalar, NRows, NCols, Order>&& {
    matrix.selfConjugated();
    return std::move(matrix);
}

} // namespace std

// NOLINTEND(cert-dcl58-cpp)
