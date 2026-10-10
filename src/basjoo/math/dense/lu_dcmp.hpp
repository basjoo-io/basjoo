/**
 * @file lu_dcmp.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-01-14
 *
 * @copyright Copyright (c) 2025 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <complex>
#include <stdexcept>
#include <type_traits>

#ifdef BASJOO_USE_BLAS_LAPACK
#include "lapacke.h"
#endif

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/detail/dense_partial_pivot_trait.hpp"

namespace basjoo::math {

template <MatArithmetic T>
class LuDcmp final {
  public:
    using matrix_type = std::remove_cvref_t<T>;
    using value_type = typename matrix_type::value_type;
    using size_type = typename matrix_type::size_type;
    using allocator_type = typename matrix_type::allocator_type;

    LuDcmp() noexcept = delete;
    LuDcmp(const LuDcmp& other) noexcept = delete;
    auto operator=(const LuDcmp& other) noexcept -> LuDcmp& = delete;
    LuDcmp(LuDcmp&& other) noexcept = delete;
    auto operator=(LuDcmp&& other) noexcept -> LuDcmp& = delete;
    ~LuDcmp() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return m_lu.get_allocator();
    }

    explicit LuDcmp(matrix_type matrix)
        : m_lu{std::move(matrix)}, m_ipiv(matrix.nrows(), matrix.get_allocator()) {
#if BASJOO_CHECK_PARAMS == 1
        if (m_lu.nrows() != m_lu.ncols()) {
            throw std::invalid_argument("matrix must be a square matrix");
        }
#endif
#ifdef BASJOO_USE_BLAS_LAPACK
        const lapack_int n = m_lu.nrows();
        constexpr lapack_int matrix_layout{
            std::is_same_v<typename matrix_type::layout_type, std::layout_left> ? LAPACK_COL_MAJOR
                                                                                : LAPACK_ROW_MAJOR
        };
        [[maybe_unused]] lapack_int info;
        if constexpr (std::is_same_v<value_type, float>) {
            info = LAPACKE_sgetrf_work(matrix_layout, n, n, m_lu.data(), n, m_ipiv.data());
        }
        if constexpr (std::is_same_v<value_type, double>) {
            info = LAPACKE_dgetrf_work(matrix_layout, n, n, m_lu.data(), n, m_ipiv.data());
        }
        if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            info = LAPACKE_cgetrf_work(
                matrix_layout, n, n, reinterpret_cast<lapack_complex_float*>(m_lu.data()), n,
                m_ipiv.data()
            );
        }
        if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            info = LAPACKE_zgetrf_work(
                matrix_layout, n, n, reinterpret_cast<lapack_complex_double*>(m_lu.data()), n,
                m_ipiv.data()
            );
        }
        if (info != 0) {
            throw std::runtime_error("LAPACKE_xgetrf failed: this matrix is singular.");
        }
#else
        const size_type n{m_lu.ncols()};
        for (size_type i{0}; i < n; ++i) {
            for (size_type j{i}; j < n; ++j) {
                for (size_type k{0}; k < i; ++k) {
                    m_lu[i, j] -= m_lu[i, k] * m_lu[k, j];
                }
            }
            for (size_type j{i + 1}; j < n; ++j) {
                for (size_type k{0}; k < i; ++k) {
                    m_lu[j, i] -= m_lu[j, k] * m_lu[k, i];
                }
                m_lu[j, i] /= m_lu[i, i];
            }
        }
#endif
    }

    [[nodiscard]]
    auto solve() const -> matrix_type {
#ifdef BASJOO_USE_BLAS_LAPACK
        const lapack_int n = m_lu.nrows();
        constexpr lapack_int matrix_layout{
            std::is_same_v<typename matrix_type::layout_type, std::layout_left> ? LAPACK_COL_MAJOR
                                                                                : LAPACK_ROW_MAJOR
        };
        matrix_type b(m_lu), work(n, n);
        [[maybe_unused]] lapack_int info;
        if constexpr (std::is_same_v<value_type, float>) {
            info = LAPACKE_sgetri_work(
                matrix_layout, n, b.data(), n, m_ipiv.data(), work.data(), n * n
            );
        }
        if constexpr (std::is_same_v<value_type, double>) {
            info = LAPACKE_dgetri_work(
                matrix_layout, n, b.data(), n, m_ipiv.data(), work.data(), n * n
            );
        }
        if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            info = LAPACKE_cgetri_work(
                matrix_layout, n, reinterpret_cast<lapack_complex_float*>(b.data()), n,
                m_ipiv.data(), reinterpret_cast<lapack_complex_float*>(work.data()), n * n
            );
        }
        if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            info = LAPACKE_zgetri_work(
                matrix_layout, n, reinterpret_cast<lapack_complex_double*>(b.data()), n,
                m_ipiv.data(), reinterpret_cast<lapack_complex_double*>(work.data()), n * n
            );
        }
        if (info != 0) {
            throw std::runtime_error("LuDcmp: LAPACKE_xgetri_work failed");
        }
        return b;
#else
        const size_type n{m_lu.ncols()};
        matrix_type b(n, n, 0.0);
        for (size_type i{0}; i < n; ++i) {
            b[i, i] = 1.0;
            for (size_type j{0}; j < i; ++j) {
                for (size_type k{j}; k < i; ++k) {
                    b[i, j] -= m_lu[i, k] * b[k, j];
                }
            }
        }
        for (int i = n - 1; i > -1; --i) {
            for (int j = n - 1; j > -1; --j) {
                for (size_type k = i + 1; k < n; ++k) {
                    b[i, j] -= m_lu[i, k] * b[k, j];
                }
                b[i, j] /= m_lu[i, i];
            }
        }
        return b;
#endif
    }

    [[nodiscard]]
    auto solve(VecArithmetic auto b) const -> decltype(b) {
#if BASJOO_CHECK_PARAMS == 1
        if (m_lu.ncols() != b.size()) {
            throw std::invalid_argument("vector size mismatched!");
        }
#endif
#ifdef BASJOO_USE_BLAS_LAPACK
        lapack_int n = b.size();
        constexpr lapack_int matrix_layout{
            std::is_same_v<typename matrix_type::layout_type, std::layout_left> ? LAPACK_COL_MAJOR
                                                                                : LAPACK_ROW_MAJOR
        };
        const lapack_int ldb{
            std::is_same_v<typename matrix_type::layout_type, std::layout_left> ? n : 1
        };
        [[maybe_unused]] lapack_int info;
        if constexpr (std::is_same_v<value_type, float>) {
            info = LAPACKE_sgetrs_work(
                matrix_layout, 'N', n, 1, m_lu.data(), n, m_ipiv.data(), b.data(), ldb
            );
        }
        if constexpr (std::is_same_v<value_type, double>) {
            info = LAPACKE_dgetrs_work(
                matrix_layout, 'N', n, 1, m_lu.data(), n, m_ipiv.data(), b.data(), ldb
            );
        }
        if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            info = LAPACKE_cgetrs_work(
                matrix_layout, 'N', n, 1,
                reinterpret_cast<const lapack_complex_float*>(m_lu.data()), n, m_ipiv.data(),
                reinterpret_cast<lapack_complex_float*>(b.data()), ldb
            );
        }
        if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            info = LAPACKE_zgetrs_work(
                matrix_layout, 'N', n, 1,
                reinterpret_cast<const lapack_complex_double*>(m_lu.data()), n, m_ipiv.data(),
                reinterpret_cast<lapack_complex_double*>(b.data()), ldb
            );
        }
        if (info != 0) {
            throw std::runtime_error("LuDcmp: LAPACKE_xgetrs_work failed");
        }
        return b;
#else
        const size_type n{b.size()};
        for (size_type i{0}; i < n; ++i) {
            for (size_type k{0}; k < i; ++k) {
                b[i] -= m_lu[i, k] * b[k];
            }
        }
        for (int i = n - 1; i > -1; --i) {
            for (size_type k = i + 1; k < n; ++k) {
                b[i] -= m_lu[i, k] * b[k];
            }
            b[i] /= m_lu[i, i];
        }
        return b;
#endif
    }

    [[nodiscard]]
    auto solve(MatArithmetic auto b) const -> decltype(b) {
#if BASJOO_CHECK_PARAMS == 1
        if (m_lu.ncols() != b.nrows()) {
            throw std::invalid_argument("matrix size mismatched!");
        }
#endif
#ifdef BASJOO_USE_BLAS_LAPACK
        lapack_int nrows = b.nrows();
        lapack_int ncols = b.ncols();
        constexpr lapack_int matrix_layout{
            std::is_same_v<typename matrix_type::layout_type, std::layout_left> ? LAPACK_COL_MAJOR
                                                                                : LAPACK_ROW_MAJOR
        };
        const lapack_int ldb{
            std::is_same_v<typename matrix_type::layout_type, std::layout_left> ? nrows : ncols
        };
        [[maybe_unused]] lapack_int info;
        if constexpr (std::is_same_v<value_type, float>) {
            info = LAPACKE_sgetrs_work(
                matrix_layout, 'N', nrows, ncols, m_lu.data(), nrows, m_ipiv.data(), b.data(), ldb
            );
        }
        if constexpr (std::is_same_v<value_type, double>) {
            info = LAPACKE_dgetrs_work(
                matrix_layout, 'N', nrows, ncols, m_lu.data(), nrows, m_ipiv.data(), b.data(), ldb
            );
        }
        if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            info = LAPACKE_cgetrs_work(
                matrix_layout, 'N', nrows, ncols,
                reinterpret_cast<const lapack_complex_float*>(m_lu.data()), nrows, m_ipiv.data(),
                reinterpret_cast<lapack_complex_float*>(b.data()), ldb
            );
        }
        if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            info = LAPACKE_zgetrs_work(
                matrix_layout, 'N', nrows, ncols,
                reinterpret_cast<const lapack_complex_double*>(m_lu.data()), nrows, m_ipiv.data(),
                reinterpret_cast<lapack_complex_double*>(b.data()), ldb
            );
        }
        if (info != 0) {
            throw std::runtime_error("LuDcmp: LAPACKE_xgetrs_work failed");
        }
        return b;
#else
        const size_type nrows{b.nrows()}, ncols{b.ncols()};
        for (size_type i{0}; i < nrows; ++i) {
            for (size_type j{0}; j < ncols; ++j) {
                for (size_type k{0}; k < i; ++k) {
                    b[i, j] -= m_lu[i, k] * b[k, j];
                }
            }
        }
        for (int i = nrows - 1; i > -1; --i) {
            for (int j = ncols - 1; j > -1; --j) {
                for (size_type k = i + 1; k < nrows; ++k) {
                    b[i, j] -= m_lu[i, k] * b[k, j];
                }
                b[i, j] /= m_lu[i, i];
            }
        }
        return b;
#endif
    }

  private:
    matrix_type m_lu;
    detail::DensePartialPivotTraitT<matrix_type> m_ipiv;
};

} // namespace basjoo::math
