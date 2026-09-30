/**
 * @file qr_dcmp.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-01-21
 *
 * @copyright Copyright (c) 2025 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <cmath>
#include <complex>
#include <stdexcept>
#include <type_traits>

#ifdef BASJOO_USE_BLAS_LAPACK
#include "lapacke.h"
#endif

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/detail/dense_degenerate_trait.hpp"

namespace basjoo::math {

template <MatArithmetic T>
class QrDcmp final {
  public:
    using matrix_type = std::remove_cvref_t<T>;
    using value_type = typename matrix_type::value_type;
    using size_type = typename matrix_type::size_type;
    using allocator_type = typename matrix_type::allocator_type;

    QrDcmp() noexcept = delete;
    QrDcmp(const QrDcmp& other) noexcept = delete;
    auto operator=(const QrDcmp& other) noexcept -> QrDcmp& = delete;
    QrDcmp(QrDcmp&& other) noexcept = delete;
    auto operator=(QrDcmp&& other) noexcept -> QrDcmp& = delete;
    ~QrDcmp() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return m_qr.get_allocator();
    }

    explicit QrDcmp(matrix_type matrix)
        : m_qr{std::move(matrix)}, m_tau(matrix.nrows(), matrix.get_allocator()) {
#if BASJOO_CHECK_PARAMS == 1
        if (m_qr.nrows() != m_qr.ncols()) {
            throw std::invalid_argument("QrDcmp: matrix must be squared");
        }
#endif
#ifdef BASJOO_USE_BLAS_LAPACK
        const lapack_int n = m_qr.nrows();
        constexpr lapack_int order{
            matrix_type::kOrder == MatrixOrder::COL_MAJOR ? LAPACK_COL_MAJOR : LAPACK_ROW_MAJOR
        };
        detail::DenseDegenerateTraitT<matrix_type> work(n);
        [[maybe_unused]] lapack_int info;
        if constexpr (std::is_same_v<value_type, float>) {
            info = LAPACKE_sgeqrf_work(order, n, n, m_qr.data(), n, m_tau.data(), work.data(), n);
        }
        if constexpr (std::is_same_v<value_type, double>) {
            info = LAPACKE_dgeqrf_work(order, n, n, m_qr.data(), n, m_tau.data(), work.data(), n);
        }
        if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            info = LAPACKE_cgeqrf_work(
                order, n, n, reinterpret_cast<lapack_complex_float*>(m_qr.data()), n,
                reinterpret_cast<lapack_complex_float*>(m_tau.data()),
                reinterpret_cast<lapack_complex_float*>(work.data()), n
            );
        }
        if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            info = LAPACKE_zgeqrf_work(
                order, n, n, reinterpret_cast<lapack_complex_double*>(m_qr.data()), n,
                reinterpret_cast<lapack_complex_double*>(m_tau.data()),
                reinterpret_cast<lapack_complex_double*>(work.data()), n
            );
        }
        if (info != 0) {
            throw std::runtime_error("QrDcmp: LAPACKE_xgeqrf_work failed");
        }
#else
        const size_type n{matrix.nrows()};
        for (size_type k{0}; k < n - 1; ++k) {
            value_type sum, sigma;
            sum = 0.0;
            for (size_type i{k}; i < n; ++i) {
                sum += m_qr[i, k] * m_qr[i, k];
            }
            sigma = (std::real(m_qr[k, k]) > 0.0) ? std::sqrt(sum) : -std::sqrt(sum);
            m_qr[k, k] += sigma;
            m_tau[k] = static_cast<value_type>(1.0) / (m_qr[k, k] * sigma);
            for (size_type j{k + 1}; j < n; ++j) {
                sum = 0.0;
                for (size_type i{k}; i < n; ++i) {
                    sum += m_qr[i, k] * m_qr[i, j];
                }
                const value_type temp = m_tau[k] * sum;
                for (size_type i{k}; i < n; ++i) {
                    m_qr[i, j] -= m_qr[i, k] * temp;
                }
            }
        }
        m_tau[n - 1] = static_cast<value_type>(-1.0) / (m_qr[n - 1, n - 1] * m_qr[n - 1, n - 1]);
#endif
    }

    [[nodiscard]]
    auto solve() const -> matrix_type {
#ifdef BASJOO_USE_BLAS_LAPACK
        const lapack_int n = m_qr.nrows();
        matrix_type b(n, n, 0.0);
        for (lapack_int i{0}; i < n; ++i) {
            b[i, i] = 1.0;
        }
        constexpr lapack_int order{
            matrix_type::kOrder == MatrixOrder::COL_MAJOR ? LAPACK_COL_MAJOR : LAPACK_ROW_MAJOR
        };
        detail::DenseDegenerateTraitT<matrix_type> work(n);
        [[maybe_unused]] lapack_int info;
        if constexpr (std::is_same_v<value_type, float>) {
            info = LAPACKE_sormqr_work(
                order, 'L', 'T', n, n, n, m_qr.data(), n, m_tau.data(), b.data(), n, work.data(), n
            );
        }
        if constexpr (std::is_same_v<value_type, double>) {
            info = LAPACKE_dormqr_work(
                order, 'L', 'T', n, n, n, m_qr.data(), n, m_tau.data(), b.data(), n, work.data(), n
            );
        }
        if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            info = LAPACKE_cunmqr_work(
                order, 'L', 'C', n, n, n,
                reinterpret_cast<const lapack_complex_float*>(m_qr.data()), n,
                reinterpret_cast<const lapack_complex_float*>(m_tau.data()),
                reinterpret_cast<lapack_complex_float*>(b.data()), n,
                reinterpret_cast<lapack_complex_float*>(work.data()), n
            );
        }
        if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            info = LAPACKE_zunmqr_work(
                order, 'L', 'C', n, n, n,
                reinterpret_cast<const lapack_complex_double*>(m_qr.data()), n,
                reinterpret_cast<const lapack_complex_double*>(m_tau.data()),
                reinterpret_cast<lapack_complex_double*>(b.data()), n,
                reinterpret_cast<lapack_complex_double*>(work.data()), n
            );
        }
        if (info != 0) {
            throw std::runtime_error("QrDcmp: LAPACKE_xormqr_work failed");
        }
        if constexpr (std::is_same_v<value_type, float>) {
            info = LAPACKE_strtrs_work(order, 'U', 'N', 'N', n, n, m_qr.data(), n, b.data(), n);
        }
        if constexpr (std::is_same_v<value_type, double>) {
            info = LAPACKE_dtrtrs_work(order, 'U', 'N', 'N', n, n, m_qr.data(), n, b.data(), n);
        }
        if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            info = LAPACKE_ctrtrs_work(
                order, 'U', 'N', 'N', n, n,
                reinterpret_cast<const lapack_complex_float*>(m_qr.data()), n,
                reinterpret_cast<lapack_complex_float*>(b.data()), n
            );
        }
        if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            info = LAPACKE_ztrtrs_work(
                order, 'U', 'N', 'N', n, n,
                reinterpret_cast<const lapack_complex_double*>(m_qr.data()), n,
                reinterpret_cast<lapack_complex_double*>(b.data()), n
            );
        }
        if (info != 0) {
            throw std::runtime_error("QrDcmp: LAPACKE_xtrtrs_work failed");
        }
        return b;
#else
        const size_type n = m_qr.nrows();
        matrix_type b(n, n, 0.0);
        for (size_type i{0}; i < n; ++i) {
            b[i, i] = 1.0;
        }
        for (size_type k{0}; k < n - 1; ++k) {
            for (size_type j{0}; j < n; ++j) {
                value_type sum(0.0);
                for (size_type i{k}; i < n; ++i) {
                    sum += m_qr[i, k] * b[i, j];
                }
                sum *= m_tau[k];
                for (size_type i{k}; i < n; ++i) {
                    b[i, j] -= m_qr[i, k] * sum;
                }
            }
        }
        for (size_type j{0}; j < n; ++j) {
            for (int i = n - 1; i >= 0; --i) {
                value_type sum(b[i, j]);
                for (size_type k = i + 1; k < n; ++k) {
                    sum -= m_qr[i, k] * b[k, j];
                }
                b[i, j] = -m_qr[i, i] * m_tau[i] * sum;
            }
        }
        return b;
#endif
    }

    [[nodiscard]]
    auto solve(VecArithmetic auto b) const -> decltype(b) {
#if BASJOO_CHECK_PARAMS == 1
        if (m_qr.ncols() != b.size()) {
            throw std::invalid_argument("vector size mismatched!");
        }
#endif
#ifdef BASJOO_USE_BLAS_LAPACK
        const lapack_int n = b.size();
        constexpr lapack_int order{
            matrix_type::kOrder == MatrixOrder::COL_MAJOR ? LAPACK_COL_MAJOR : LAPACK_ROW_MAJOR
        };
        const lapack_int ldb{matrix_type::kOrder == MatrixOrder::COL_MAJOR ? n : 1};
        detail::DenseDegenerateTraitT<matrix_type> work(n);
        [[maybe_unused]] lapack_int info;
        if constexpr (std::is_same_v<value_type, float>) {
            info = LAPACKE_sormqr_work(
                order, 'L', 'T', n, 1, n, m_qr.data(), n, m_tau.data(), b.data(), ldb, work.data(),
                n
            );
        }
        if constexpr (std::is_same_v<value_type, double>) {
            info = LAPACKE_dormqr_work(
                order, 'L', 'T', n, 1, n, m_qr.data(), n, m_tau.data(), b.data(), ldb, work.data(),
                n
            );
        }
        if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            info = LAPACKE_cunmqr_work(
                order, 'L', 'C', n, 1, n,
                reinterpret_cast<const lapack_complex_float*>(m_qr.data()), n,
                reinterpret_cast<const lapack_complex_float*>(m_tau.data()),
                reinterpret_cast<lapack_complex_float*>(b.data()), ldb,
                reinterpret_cast<lapack_complex_float*>(work.data()), n
            );
        }
        if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            info = LAPACKE_zunmqr_work(
                order, 'L', 'C', n, 1, n,
                reinterpret_cast<const lapack_complex_double*>(m_qr.data()), n,
                reinterpret_cast<const lapack_complex_double*>(m_tau.data()),
                reinterpret_cast<lapack_complex_double*>(b.data()), ldb,
                reinterpret_cast<lapack_complex_double*>(work.data()), n
            );
        }
        if (info != 0) {
            throw std::runtime_error("QrDcmp: LAPACKE_xormqr_work failed");
        }
        if constexpr (std::is_same_v<value_type, float>) {
            info = LAPACKE_strtrs_work(order, 'U', 'N', 'N', n, 1, m_qr.data(), n, b.data(), ldb);
        }
        if constexpr (std::is_same_v<value_type, double>) {
            info = LAPACKE_dtrtrs_work(order, 'U', 'N', 'N', n, 1, m_qr.data(), n, b.data(), ldb);
        }
        if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            info = LAPACKE_ctrtrs_work(
                order, 'U', 'N', 'N', n, 1,
                reinterpret_cast<const lapack_complex_float*>(m_qr.data()), n,
                reinterpret_cast<lapack_complex_float*>(b.data()), ldb
            );
        }
        if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            info = LAPACKE_ztrtrs_work(
                order, 'U', 'N', 'N', n, 1,
                reinterpret_cast<const lapack_complex_double*>(m_qr.data()), n,
                reinterpret_cast<lapack_complex_double*>(b.data()), ldb
            );
        }
        if (info != 0) {
            throw std::runtime_error("QrDcmp: LAPACKE_xtrtrs_work failed");
        }
        return b;
#else
        const size_type n{b.size()};
        for (size_type k{0}; k < n - 1; ++k) {
            value_type sum(0.0);
            for (size_type i{k}; i < n; ++i) {
                sum += m_qr[i, k] * b[i];
            }
            sum *= m_tau[k];
            for (size_type i{k}; i < n; ++i) {
                b[i] -= m_qr[i, k] * sum;
            }
        }
        for (int i = n - 1; i >= 0; --i) {
            value_type sum(b[i]);
            for (size_type k = i + 1; k < n; ++k) {
                sum -= m_qr[i, k] * b[k];
            }
            b[i] = -m_qr[i, i] * m_tau[i] * sum;
        }
        return b;
#endif
    }

    [[nodiscard]]
    auto solve(MatArithmetic auto b) const -> decltype(b) {
#if BASJOO_CHECK_PARAMS == 1
        if (m_qr.ncols() != b.nrows()) {
            throw std::invalid_argument("matrix size mismatched!");
        }
#endif
#ifdef BASJOO_USE_BLAS_LAPACK
        const lapack_int nrows = b.nrows();
        const lapack_int ncols = b.ncols();
        constexpr lapack_int order{
            matrix_type::kOrder == MatrixOrder::COL_MAJOR ? LAPACK_COL_MAJOR : LAPACK_ROW_MAJOR
        };
        const lapack_int ldb{matrix_type::kOrder == MatrixOrder::COL_MAJOR ? nrows : ncols};
        detail::DenseDegenerateTraitT<matrix_type> work(nrows);
        [[maybe_unused]] lapack_int info;
        if constexpr (std::is_same_v<value_type, float>) {
            info = LAPACKE_sormqr_work(
                order, 'L', 'T', nrows, ncols, nrows, m_qr.data(), nrows, m_tau.data(), b.data(),
                ldb, work.data(), nrows
            );
        }
        if constexpr (std::is_same_v<value_type, double>) {
            info = LAPACKE_dormqr_work(
                order, 'L', 'T', nrows, ncols, nrows, m_qr.data(), nrows, m_tau.data(), b.data(),
                ldb, work.data(), nrows
            );
        }
        if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            info = LAPACKE_cunmqr_work(
                order, 'L', 'C', nrows, ncols, nrows,
                reinterpret_cast<const lapack_complex_float*>(m_qr.data()), nrows,
                reinterpret_cast<const lapack_complex_float*>(m_tau.data()),
                reinterpret_cast<lapack_complex_float*>(b.data()), ldb,
                reinterpret_cast<lapack_complex_float*>(work.data()), nrows
            );
        }
        if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            info = LAPACKE_zunmqr_work(
                order, 'L', 'C', nrows, ncols, nrows,
                reinterpret_cast<const lapack_complex_double*>(m_qr.data()), nrows,
                reinterpret_cast<const lapack_complex_double*>(m_tau.data()),
                reinterpret_cast<lapack_complex_double*>(b.data()), ldb,
                reinterpret_cast<lapack_complex_double*>(work.data()), nrows
            );
        }
        if (info != 0) {
            throw std::runtime_error("QrDcmp: LAPACKE_xormqr_work failed");
        }
        if constexpr (std::is_same_v<value_type, float>) {
            info = LAPACKE_strtrs_work(
                order, 'U', 'N', 'N', nrows, ncols, m_qr.data(), nrows, b.data(), ldb
            );
        }
        if constexpr (std::is_same_v<value_type, double>) {
            info = LAPACKE_dtrtrs_work(
                order, 'U', 'N', 'N', nrows, ncols, m_qr.data(), nrows, b.data(), ldb
            );
        }
        if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            info = LAPACKE_ctrtrs_work(
                order, 'U', 'N', 'N', nrows, ncols,
                reinterpret_cast<const lapack_complex_float*>(m_qr.data()), nrows,
                reinterpret_cast<lapack_complex_float*>(b.data()), ldb
            );
        }
        if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            info = LAPACKE_ztrtrs_work(
                order, 'U', 'N', 'N', nrows, ncols,
                reinterpret_cast<const lapack_complex_double*>(m_qr.data()), nrows,
                reinterpret_cast<lapack_complex_double*>(b.data()), ldb
            );
        }
        if (info != 0) {
            throw std::runtime_error("QrDcmp: LAPACKE_xtrtrs_work failed");
        }
        return b;
#else
        const size_type nrows{b.nrows()}, ncols{b.ncols()};
        for (size_type k{0}; k < nrows - 1; ++k) {
            for (size_type j{0}; j < ncols; ++j) {
                value_type sum(0.0);
                for (size_type i{k}; i < nrows; ++i) {
                    sum += m_qr[i, k] * b[i, j];
                }
                sum *= m_tau[k];
                for (size_type i{k}; i < nrows; ++i) {
                    b[i, j] -= m_qr[i, k] * sum;
                }
            }
        }
        for (size_type j{0}; j < ncols; ++j) {
            for (int i = nrows - 1; i >= 0; --i) {
                value_type sum(b[i, j]);
                for (size_type k = i + 1; k < nrows; ++k) {
                    sum -= m_qr[i, k] * b[k, j];
                }
                b[i, j] = -m_qr[i, i] * m_tau[i] * sum;
            }
        }
        return b;
#endif
    }

  private:
    matrix_type m_qr;
    detail::DenseDegenerateTraitT<matrix_type> m_tau;
};

} // namespace basjoo::math
