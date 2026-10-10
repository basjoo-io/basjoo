/**
 * @file matrix_base.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2026-10-08
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <iomanip>
#include <mdspan>
#include <numeric>
#include <ostream>
#include <stdexcept>
#include <type_traits>
#include <utility>

#ifdef BASJOO_USE_BLAS_LAPACK
#include "cblas.h"
#include "lapacke.h"
#endif

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_arithmetic_base.hpp"
#include "basjoo/math/dense/dense_trait.hpp"
#include "basjoo/math/dense/detail/dense_dot_trait.hpp"
#include "basjoo/math/dense/detail/dense_norm_trait.hpp"
#include "basjoo/math/dense/detail/dense_transposed_trait.hpp"
#include "basjoo/math/type_traits.hpp"

namespace basjoo::math {

class MatrixBase : public DenseArithmeticBase {
  public:
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator[](this auto&& self, std::size_t row, std::size_t col) noexcept
        -> decltype(auto) {
        if constexpr (
            std::is_same_v<
                typename DenseTrait<std::remove_cvref_t<decltype(self)>>::layout_type,
                std::layout_left>
        ) {
            return self.data()[(col * self.nrows()) + row];
        } else {
            return self.data()[(row * self.ncols()) + col];
        }
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator[](this auto&& self, std::size_t i) noexcept -> decltype(auto) {
        return self.data()[i];
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto coeff(this const auto& self, std::size_t row, std::size_t col) noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type {
#if BASJOO_CHECK_PARAMS == 1
        if (row >= self.nrows() || col >= self.ncols()) [[unlikely]] {
            throw std::out_of_range("Matrix index out of range");
        }
#endif
        return self[row, col];
    }

    [[using gnu: always_inline, hot]]
    constexpr auto updateCoeff(
        this auto&& self, std::size_t row, std::size_t col,
        const typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type& value
    ) noexcept(!BASJOO_CHECK_PARAMS) -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (row >= self.nrows() || col >= self.ncols()) [[unlikely]] {
            throw std::out_of_range("Matrix index out of range");
        }
#endif
        self[row, col] = value;
        return;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto coeff(this const auto& self, std::size_t i) noexcept(!BASJOO_CHECK_PARAMS) ->
        typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type {
#if BASJOO_CHECK_PARAMS == 1
        if (i >= self.size()) [[unlikely]] {
            throw std::out_of_range("Matrix index out of range.");
        }
#endif
        return self[i];
    }

    [[using gnu: always_inline, hot]]
    constexpr auto updateCoeff(
        this auto&& self, std::size_t i,
        const typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type& value
    ) noexcept(!BASJOO_CHECK_PARAMS) -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (i >= self.size()) [[unlikely]] {
            throw std::out_of_range("Matrix index out of range.");
        }
#endif
        self[i] = value;
        return;
    }

    [[using gnu: always_inline, hot]]
    constexpr auto fill(
        this auto&& self,
        const typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type& value
    ) noexcept -> void {
        std::ranges::fill_n(self.data(), self.size(), value);
        return;
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto empty(this const auto& self) noexcept -> bool {
        return self.size() == 0;
    }

    [[using gnu: always_inline]]
    constexpr auto setIdentity(this auto&& self) noexcept -> void {
        const std::size_t n{std::min(self.nrows(), self.ncols())};
        self.fill(0.0);
        for (std::size_t i{0}; i < n; ++i) {
            self[i, i] = 1.0;
        }
        return;
    }

    /// Strided sub-block view over the half-open row/column ranges. With
    /// BASJOO_CHECK_PARAMS enabled, inverted or out-of-bounds ranges throw
    /// std::out_of_range instead of being silently clamped.
    [[using gnu: pure, always_inline]]
    constexpr auto submdspan(
        this auto&& self, std::pair<std::size_t, std::size_t> row_range,
        std::pair<std::size_t, std::size_t> col_range
    ) noexcept(!BASJOO_CHECK_PARAMS) -> decltype(auto) {
        using derived_type = std::remove_cvref_t<decltype(self)>;
        using element_type = std::conditional_t<
            std::is_const_v<std::remove_reference_t<decltype(self)>>,
            const typename DenseTrait<derived_type>::value_type,
            typename DenseTrait<derived_type>::value_type>;
#if BASJOO_CHECK_PARAMS == 1
        if (row_range.first > row_range.second || row_range.second > self.nrows() ||
            col_range.first > col_range.second || col_range.second > self.ncols()) [[unlikely]] {
            throw std::out_of_range("Matrix::submdspan: invalid range.");
        }
#endif
        using layout_type = typename DenseTrait<derived_type>::layout_type;
        const bool is_col_major{std::is_same_v<layout_type, std::layout_left>};
        const std::size_t row_count{row_range.second - row_range.first};
        const std::size_t col_count{col_range.second - col_range.first};
        const std::size_t stride_i{
            std::is_same_v<layout_type, std::layout_left> ? 1 : self.stride(0)
        };
        const std::size_t stride_j{
            std::is_same_v<layout_type, std::layout_left> ? self.stride(1) : 1
        };
        const std::size_t base_offset{
            is_col_major ? col_range.first * self.nrows() + row_range.first
                         : row_range.first * self.ncols() + col_range.first
        };
        return std::mdspan<element_type, std::dextents<std::size_t, 2>, std::layout_stride>{
            self.data() + base_offset, std::layout_stride::mapping<std::dextents<std::size_t, 2>>{
                                           std::dextents<std::size_t, 2>{row_count, col_count},
                                           std::array<std::size_t, 2>{stride_i, stride_j}
                                       }
        };
    }

    /// Vector-style slice: the range is applied along the non-unit dimension
    /// of a 1xN or Nx1 matrix. With BASJOO_CHECK_PARAMS enabled, a
    /// non-vector-like matrix throws std::invalid_argument.
    [[using gnu: pure, always_inline]]
    constexpr auto submdspan(this auto&& self, std::pair<std::size_t, std::size_t> range) noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> decltype(auto) {
#if BASJOO_CHECK_PARAMS == 1
        if (self.nrows() != 1 && self.ncols() != 1) [[unlikely]] {
            throw std::invalid_argument(
                "Matrix::submdspan: vector-style slice only applies on vector-like matrices."
            );
        }
        if (range.first > range.second ||
            range.second > (self.ncols() == 1 ? self.nrows() : self.ncols())) [[unlikely]] {
            throw std::out_of_range("Matrix::submdspan: invalid range.");
        }
#endif
        if (self.ncols() == 1) {
            return self.submdspan(range, std::pair<std::size_t, std::size_t>{0, 1});
        }
        return self.submdspan(std::pair<std::size_t, std::size_t>{0, 1}, range);
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclidean(this const auto& self) noexcept -> detail::DenseNormTraitT<
        typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type> {
        using value_type = typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type;
        detail::DenseNormTraitT<value_type> result(0.0);
        if (self.nrows() == 1 || self.ncols() == 1) {
#ifdef BASJOO_USE_BLAS_LAPACK
            if constexpr (std::is_same_v<value_type, float>) {
                result = cblas_snrm2(self.size(), self.data(), 1);
            } else if constexpr (std::is_same_v<value_type, double>) {
                result = cblas_dnrm2(self.size(), self.data(), 1);
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                result = cblas_scnrm2(self.size(), self.data(), 1);
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                result = cblas_dznrm2(self.size(), self.data(), 1);
            } else {
                result = std::transform_reduce(
                    self.data(), self.data() + self.size(), 0.0,
                    std::plus<detail::DenseNormTraitT<value_type>>{},
                    [](const value_type& a) constexpr noexcept
                        -> detail::DenseNormTraitT<value_type> { return std::norm(a); }
                );
                result = std::sqrt(result);
            }
#else
            result = std::transform_reduce(
                self.data(), self.data() + self.size(), 0.0,
                std::plus<detail::DenseNormTraitT<value_type>>{},
                [](const value_type& a) constexpr noexcept -> detail::DenseNormTraitT<value_type> {
                    return std::norm(a);
                }
            );
            result = std::sqrt(result);
#endif
        }
        return result;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclideanSqr(this const auto& self) noexcept -> detail::DenseNormTraitT<
        typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type> {
        using value_type = typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type;
        detail::DenseNormTraitT<value_type> result(0.0);
        if (self.nrows() == 1 || self.ncols() == 1) {
#ifdef BASJOO_USE_BLAS_LAPACK
            if constexpr (std::is_same_v<value_type, float>) {
                result = cblas_sdot(self.size(), self.data(), 1, self.data(), 1);
            } else if constexpr (std::is_same_v<value_type, double>) {
                result = cblas_ddot(self.size(), self.data(), 1, self.data(), 1);
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                result = cblas_cdotc(self.size(), self.data(), 1, self.data(), 1);
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                result = cblas_zdotc(self.size(), self.data(), 1, self.data(), 1);
            } else {
                result = std::transform_reduce(
                    self.data(), self.data() + self.size(), 0.0,
                    std::plus<detail::DenseNormTraitT<value_type>>{},
                    [](const value_type& a) constexpr noexcept
                        -> detail::DenseNormTraitT<value_type> { return std::norm(a); }
                );
            }
#else
            result = std::transform_reduce(
                self.data(), self.data() + self.size(), 0.0,
                std::plus<detail::DenseNormTraitT<value_type>>{},
                [](const value_type& a) constexpr noexcept -> detail::DenseNormTraitT<value_type> {
                    return std::norm(a);
                }
            );
#endif
        }
        return result;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto identicalTo(
        this const auto& self,
        std::mdspan<
            const typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type,
            std::dextents<std::size_t, 2>, std::layout_stride> obj,
        double tol = 1E-8
    ) noexcept -> bool {
        if (self.nrows() != obj.extent(0) || self.ncols() != obj.extent(1)) {
            return false;
        }
        detail::DenseNormTraitT<
            typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type>
            test(0.0);
        for (std::size_t i{0}; i < self.nrows(); ++i) {
            for (std::size_t j{0}; j < self.ncols(); ++j) {
                test = std::max(
                    test, std::abs(self[i, j] - obj[i, j]) /
                              std::max(std::abs(self[i, j]), static_cast<decltype(test)>(1.0))
                );
            }
        }
        return test < tol;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto transposed(this const auto& self) noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> detail::DenseTransposedTraitT<std::remove_cvref_t<decltype(self)>> {
        const std::size_t rows{self.nrows()};
        const std::size_t cols{self.ncols()};
        detail::DenseTransposedTraitT<std::remove_cvref_t<decltype(self)>> result(cols, rows);
        for (std::size_t i{0}; i < rows; ++i) {
            for (std::size_t j{0}; j < cols; ++j) {
                result[j, i] = self[i, j];
            }
        }
        return result;
    }

    [[using gnu: always_inline]]
    constexpr auto selfConjugated(this auto&& self) noexcept
        -> std::remove_cvref_t<decltype(self)>& {
#ifdef BASJOO_USE_BLAS_LAPACK
        using value_type = typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type;
        if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
            if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                LAPACKE_clacgv_work(
                    self.size(), reinterpret_cast<lapack_complex_float*>(self.data()), 1
                );
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                LAPACKE_zlacgv_work(
                    self.size(), reinterpret_cast<lapack_complex_double*>(self.data()), 1
                );
            } else {
                std::transform(
                    self.data(), self.data() + self.size(), self.data(),
                    [](const value_type& a) constexpr noexcept -> value_type {
                        return std::conj(a);
                    }
                );
            }
        }
#else
        if constexpr (
            ::basjoo::math::isComplexArithmeticV<
                typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type>
        ) {
            std::transform(
                self.data(), self.data() + self.size(), self.data(),
                [](const auto& a) constexpr noexcept -> decltype(std::conj(a)) {
                    return std::conj(a);
                }
            );
        }
#endif
        return static_cast<std::remove_cvref_t<decltype(self)>&>(self);
    }

    [[using gnu: pure, always_inline]]
    constexpr auto conjugated(this auto&& self) noexcept(
        !BASJOO_CHECK_PARAMS && (
         std::is_rvalue_reference_v<decltype(self)>)
    ) -> decltype(auto) {
        if constexpr (std::is_rvalue_reference_v<decltype(self)>) {
            if constexpr (
                ::basjoo::math::isComplexArithmeticV<
                    typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type>
            ) {
                self.selfConjugated();
            }
            return std::forward<decltype(self)>(self);
        } else {
            std::remove_cvref_t<decltype(self)> result{self};
            if constexpr (
                ::basjoo::math::isComplexArithmeticV<
                    typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type>
            ) {
                result.selfConjugated();
            }
            return std::remove_cvref_t<decltype(self)>{std::move(result)};
        }
    }

    [[using gnu: pure, always_inline]]
    constexpr auto adjoint(this const auto& self) noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> detail::DenseTransposedTraitT<std::remove_cvref_t<decltype(self)>> {
        return self.transposed().selfConjugated();
    }

    template <typename Self, typename Obj>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(this const Self& self, const Obj& obj) noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> detail::DenseDotTraitT<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> requires DenseDotable<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> {
        if constexpr (requires(const Obj& o) {
                          o.nrows();
                          o.ncols();
                      }) {
#if BASJOO_CHECK_PARAMS == 1
            if (self.ncols() != obj.nrows()) [[unlikely]] {
                throw std::invalid_argument("Matrix::dot: incompatible dimensions");
            }
#endif
            const std::size_t outer_size = obj.ncols();
            detail::DenseDotTraitT<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> result(
                self.nrows(), outer_size, 0.0
            );
#ifdef BASJOO_USE_BLAS_LAPACK
            using value_type = typename DenseTrait<std::remove_cvref_t<Self>>::value_type;
            [[maybe_unused]] const CBLAS_ORDER cblas_order =
                (std::is_same_v<
                     typename DenseTrait<std::remove_cvref_t<Self>>::layout_type, std::layout_left>
                     ? CblasColMajor
                     : CblasRowMajor);
            [[maybe_unused]] const blasint lda =
                (std::is_same_v<
                     typename DenseTrait<std::remove_cvref_t<Self>>::layout_type, std::layout_left>
                     ? self.nrows()
                     : self.ncols());
            [[maybe_unused]] const blasint ldb =
                (std::is_same_v<
                     typename DenseTrait<std::remove_cvref_t<Obj>>::layout_type, std::layout_left>
                     ? obj.nrows()
                     : obj.ncols());
            [[maybe_unused]] const blasint ldc = static_cast<blasint>(
                std::is_same_v<
                    typename DenseTrait<detail::DenseDotTraitT<
                        std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>>>::layout_type,
                    std::layout_left>
                    ? result.nrows()
                    : result.ncols()
            );
            [[maybe_unused]] constexpr value_type alpha(1.0), beta(0.0);
            if constexpr (std::is_same_v<value_type, float>) {
                cblas_sgemm(
                    cblas_order, CblasNoTrans, CblasNoTrans, static_cast<blasint>(self.nrows()),
                    static_cast<blasint>(outer_size), static_cast<blasint>(self.ncols()), alpha,
                    self.data(), lda, obj.data(), ldb, beta, result.data(), ldc
                );
            } else if constexpr (std::is_same_v<value_type, double>) {
                cblas_dgemm(
                    cblas_order, CblasNoTrans, CblasNoTrans, static_cast<blasint>(self.nrows()),
                    static_cast<blasint>(outer_size), static_cast<blasint>(self.ncols()), alpha,
                    self.data(), lda, obj.data(), ldb, beta, result.data(), ldc
                );
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                cblas_cgemm(
                    cblas_order, CblasNoTrans, CblasNoTrans, static_cast<blasint>(self.nrows()),
                    static_cast<blasint>(outer_size), static_cast<blasint>(self.ncols()), &alpha,
                    self.data(), lda, obj.data(), ldb, &beta, result.data(), ldc
                );
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                cblas_zgemm(
                    cblas_order, CblasNoTrans, CblasNoTrans, static_cast<blasint>(self.nrows()),
                    static_cast<blasint>(outer_size), static_cast<blasint>(self.ncols()), &alpha,
                    self.data(), lda, obj.data(), ldb, &beta, result.data(), ldc
                );
            } else {
                for (std::size_t i{0}; i < self.nrows(); ++i) {
                    for (std::size_t j{0}; j < outer_size; ++j) {
                        for (std::size_t k{0}; k < self.ncols(); ++k) {
                            result[i, j] += self[i, k] * obj[k, j];
                        }
                    }
                }
            }
#else
            for (std::size_t i{0}; i < self.nrows(); ++i) {
                for (std::size_t j{0}; j < outer_size; ++j) {
                    for (std::size_t k{0}; k < self.ncols(); ++k) {
                        result[i, j] += self[i, k] * obj[k, j];
                    }
                }
            }
#endif
            return result;
        } else {
#if BASJOO_CHECK_PARAMS == 1
            if (self.ncols() != obj.size()) [[unlikely]] {
                throw std::invalid_argument("Matrix::dot: incompatible dimensions");
            }
#endif
            detail::DenseDotTraitT<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> result(
                self.nrows(), 0.0
            );
#ifdef BASJOO_USE_BLAS_LAPACK
            using value_type = typename DenseTrait<std::remove_cvref_t<Self>>::value_type;
            [[maybe_unused]] const CBLAS_ORDER cblas_order =
                (std::is_same_v<
                     typename DenseTrait<std::remove_cvref_t<Self>>::layout_type, std::layout_left>
                     ? CblasColMajor
                     : CblasRowMajor);
            [[maybe_unused]] blasint lda =
                (std::is_same_v<
                     typename DenseTrait<std::remove_cvref_t<Self>>::layout_type, std::layout_left>
                     ? self.nrows()
                     : self.ncols());
            [[maybe_unused]] constexpr value_type alpha(1.0), beta(0.0);
            if constexpr (std::is_same_v<value_type, float>) {
                cblas_sgemv(
                    cblas_order, CblasNoTrans, self.nrows(), self.ncols(), alpha, self.data(), lda,
                    obj.data(), 1, beta, result.data(), 1
                );
            } else if constexpr (std::is_same_v<value_type, double>) {
                cblas_dgemv(
                    cblas_order, CblasNoTrans, self.nrows(), self.ncols(), alpha, self.data(), lda,
                    obj.data(), 1, beta, result.data(), 1
                );
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                cblas_cgemv(
                    cblas_order, CblasNoTrans, self.nrows(), self.ncols(), &alpha, self.data(), lda,
                    obj.data(), 1, &beta, result.data(), 1
                );
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                cblas_zgemv(
                    cblas_order, CblasNoTrans, self.nrows(), self.ncols(), &alpha, self.data(), lda,
                    obj.data(), 1, &beta, result.data(), 1
                );
            } else {
                for (std::size_t i{0}; i < self.nrows(); ++i) {
                    for (std::size_t j{0}; j < self.ncols(); ++j) {
                        result[i] += self[i, j] * obj[j];
                    }
                }
            }
#else
            for (std::size_t i{0}; i < self.nrows(); ++i) {
                for (std::size_t j{0}; j < self.ncols(); ++j) {
                    result[i] += self[i, j] * obj[j];
                }
            }
#endif
            return result;
        }
    }

  protected:
    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto offset(this const auto& self, std::size_t row, std::size_t col) noexcept
        -> std::size_t {
        if constexpr (
            std::is_same_v<
                typename DenseTrait<std::remove_cvref_t<decltype(self)>>::layout_type,
                std::layout_left>
        ) {
            return (col * self.nrows()) + row;
        } else {
            return (row * self.ncols()) + col;
        }
    }

    constexpr MatrixBase() noexcept = default;
    constexpr MatrixBase(const MatrixBase&) noexcept = default;
    constexpr MatrixBase(MatrixBase&&) noexcept = default;
    constexpr auto operator=(const MatrixBase&) noexcept -> MatrixBase& = default;
    constexpr auto operator=(MatrixBase&&) noexcept -> MatrixBase& = default;
    constexpr ~MatrixBase() noexcept = default;
};

template <typename T>
[[using gnu: pure, always_inline]]
inline constexpr auto abs(const T& matrix) noexcept
    -> detail::DenseNormTraitT<std::remove_cvref_t<T>> requires std::derived_from<std::remove_cvref_t<T>, MatrixBase> {
    using value_type = typename DenseTrait<std::remove_cvref_t<T>>::value_type;
    detail::DenseNormTraitT<std::remove_cvref_t<T>> result(matrix);
    if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
        std::transform(
            result.data(), result.data() + result.size(), result.data(),
            [](const value_type& x) constexpr noexcept -> detail::DenseNormTraitT<value_type> {
                return std::abs(x);
            }
        );
    }
    return result;
}

template <typename Char, typename T>
inline auto operator<<(std::basic_ostream<Char>& os, const T& matrix) noexcept
    -> std::basic_ostream<Char>& requires std::derived_from<std::remove_cvref_t<T>, MatrixBase> {
    using value_type = typename DenseTrait<std::remove_cvref_t<T>>::value_type;
    constexpr std::size_t kWidth{isComplexArithmeticV<value_type> ? 32 : 16};
    const std::size_t nrows{matrix.nrows()}, ncols{matrix.ncols()};
    os << std::fixed;
    for (std::size_t i{0}; i < nrows; ++i) {
        for (std::size_t j{0}; j < ncols; ++j) {
            os << std::setw(kWidth) << matrix[i, j];
        }
        os << '\n';
    }
    return os;
}

template <typename T>
[[using gnu: pure, always_inline]]
inline constexpr auto conj(T&& matrix) noexcept(!BASJOO_CHECK_PARAMS) -> decltype(auto) requires std::derived_from<std::remove_cvref_t<T>, MatrixBase> {
    if constexpr (std::is_rvalue_reference_v<T&&>) {
        matrix.selfConjugated();
        return std::forward<T>(matrix);
    } else {
        return matrix.conjugated();
    }
}

} // namespace basjoo::math

namespace std {

template <
    basjoo::math::ScalarArithmetic Scalar, basjoo::math::DenseExtents Extents,
    basjoo::math::DenseLayout Layout, basjoo::math::Allocatory Alloc>
[[using gnu: pure, always_inline]]
inline constexpr auto abs(
    const basjoo::math::Matrix<Scalar, Extents, Layout, Alloc>& matrix
) noexcept
    -> basjoo::math::detail::DenseNormTraitT<basjoo::math::Matrix<Scalar, Extents, Layout, Alloc>> {
    return basjoo::math::abs(matrix);
}

template <
    basjoo::math::ScalarArithmetic Scalar, basjoo::math::DenseExtents Extents,
    basjoo::math::DenseLayout Layout, basjoo::math::Allocatory Alloc>
[[using gnu: pure, always_inline]]
inline constexpr auto conj(
    const basjoo::math::Matrix<Scalar, Extents, Layout, Alloc>& matrix
) noexcept -> basjoo::math::Matrix<Scalar, Extents, Layout, Alloc> {
    return basjoo::math::conj(matrix);
}

template <
    basjoo::math::ScalarArithmetic Scalar, basjoo::math::DenseExtents Extents,
    basjoo::math::DenseLayout Layout, basjoo::math::Allocatory Alloc>
[[using gnu: always_inline]]
inline constexpr auto conj(basjoo::math::Matrix<Scalar, Extents, Layout, Alloc>&& matrix) noexcept
    -> basjoo::math::Matrix<Scalar, Extents, Layout, Alloc>&& {
    return basjoo::math::conj(std::move(matrix));
}

} // namespace std

// NOLINTBEGIN(cert-dcl58-cpp)
