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
#include <array>
#include <cmath>
#include <complex>
#include <concepts>
#include <functional>
#include <mdspan>
#include <numeric>
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
    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator[](
        this Self&& self, typename std::remove_cvref_t<Self>::index_type row,
        typename std::remove_cvref_t<Self>::index_type col
    ) noexcept -> decltype(auto) {
        return self.data()[self.mapping()(row, col)];
    }

    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator[](
        this Self&& self, typename std::remove_cvref_t<Self>::index_type i
    ) noexcept -> decltype(auto) {
        return self.data()[i];
    }

    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto coeff(
        this const Self& self, typename std::remove_cvref_t<Self>::index_type row,
        typename std::remove_cvref_t<Self>::index_type col
    ) noexcept(!BASJOO_CHECK_PARAMS) -> typename std::remove_cvref_t<Self>::value_type {
#if BASJOO_CHECK_PARAMS == 1
        if (row >= self.nrows() || col >= self.ncols()) [[unlikely]] {
            throw std::out_of_range("Matrix index out of range");
        }
#endif
        return self[row, col];
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto updateCoeff(
        this Self&& self, typename std::remove_cvref_t<Self>::index_type row,
        typename std::remove_cvref_t<Self>::index_type col,
        typename std::remove_cvref_t<Self>::const_reference value
    ) noexcept(!BASJOO_CHECK_PARAMS) -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (row >= self.nrows() || col >= self.ncols()) [[unlikely]] {
            throw std::out_of_range("Matrix index out of range");
        }
#endif
        self[row, col] = value;
        return;
    }

    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto coeff(
        this const Self& self, typename std::remove_cvref_t<Self>::index_type i
    ) noexcept(!BASJOO_CHECK_PARAMS) -> typename std::remove_cvref_t<Self>::value_type {
#if BASJOO_CHECK_PARAMS == 1
        if (static_cast<typename std::remove_cvref_t<Self>::size_type>(i) >= self.size())
            [[unlikely]] {
            throw std::out_of_range("Matrix index out of range.");
        }
#endif
        return self[i];
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto updateCoeff(
        this Self&& self, typename std::remove_cvref_t<Self>::index_type i,
        typename std::remove_cvref_t<Self>::const_reference value
    ) noexcept(!BASJOO_CHECK_PARAMS) -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (static_cast<typename std::remove_cvref_t<Self>::size_type>(i) >= self.size())
            [[unlikely]] {
            throw std::out_of_range("Matrix index out of range.");
        }
#endif
        self[i] = value;
        return;
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto fill(
        this Self&& self, typename std::remove_cvref_t<Self>::const_reference value
    ) noexcept -> void {
        std::ranges::fill_n(self.data(), self.size(), value);
        return;
    }

    template <typename Self>
    [[using gnu: pure, always_inline, leaf]]
    constexpr auto empty(this const Self& self) noexcept -> bool {
        return self.size() == 0;
    }

    template <typename Self>
    [[using gnu: always_inline]]
    constexpr auto setIdentity(this Self&& self) noexcept -> void {
        using index_type = typename std::remove_cvref_t<Self>::index_type;
        using value_type = typename std::remove_cvref_t<Self>::value_type;

        const index_type n{std::min(self.nrows(), self.ncols())};
        self.fill(value_type(0.0));
        for (index_type i{0}; i < n; ++i) {
            self[i, i] = value_type(1.0);
        }
        return;
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto submdspan(
        this Self&& self,
        std::pair<
            typename std::remove_cvref_t<Self>::index_type,
            typename std::remove_cvref_t<Self>::index_type>
            row_range,
        std::pair<
            typename std::remove_cvref_t<Self>::index_type,
            typename std::remove_cvref_t<Self>::index_type>
            col_range
    ) noexcept(!BASJOO_CHECK_PARAMS) {
#if BASJOO_CHECK_PARAMS == 1
        if (row_range.first > row_range.second || row_range.second > self.nrows() ||
            col_range.first > col_range.second || col_range.second > self.ncols()) [[unlikely]] {
            throw std::out_of_range("Matrix::submdspan: invalid range.");
        }
#endif
        using element_type = std::remove_pointer_t<decltype(self.data())>;
        using index_type = typename std::remove_cvref_t<Self>::index_type;

        return std::mdspan<element_type, std::dextents<index_type, 2>, std::layout_stride>{
            self.data() + self.mapping()(row_range.first, col_range.first),
            std::layout_stride::mapping<std::dextents<index_type, 2>>{
                std::dextents<index_type, 2>{
                    row_range.second - row_range.first, col_range.second - col_range.first
                },
                std::array<index_type, 2>{self.stride(0), self.stride(1)}
            }
        };
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto submdspan(
        this Self&& self, std::pair<
                              typename std::remove_cvref_t<Self>::index_type,
                              typename std::remove_cvref_t<Self>::index_type>
                              range
    ) noexcept(!BASJOO_CHECK_PARAMS) {
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
        using index_type = typename std::remove_cvref_t<Self>::index_type;

        return self.ncols() == 1 ? self.submdspan(range, std::pair<index_type, index_type>{0, 1})
                                 : self.submdspan(std::pair<index_type, index_type>{0, 1}, range);
    }

    // Frobenius norm; equals the Euclidean norm for row/column-vector shaped matrices.
    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclidean(this const Self& self) noexcept
        -> detail::DenseNormTraitT<typename std::remove_cvref_t<Self>::value_type> {
        using value_type = typename std::remove_cvref_t<Self>::value_type;

        detail::DenseNormTraitT<value_type> result(0.0);
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
                [](const value_type& a) constexpr noexcept -> detail::DenseNormTraitT<value_type> {
                    return std::norm(a);
                }
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
        return result;
    }

    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclideanSqr(this const Self& self) noexcept
        -> detail::DenseNormTraitT<typename std::remove_cvref_t<Self>::value_type> {
        using value_type = typename std::remove_cvref_t<Self>::value_type;

        detail::DenseNormTraitT<value_type> result(0.0);
#ifdef BASJOO_USE_BLAS_LAPACK
        if constexpr (std::is_same_v<value_type, float>) {
            result = cblas_sdot(self.size(), self.data(), 1, self.data(), 1);
        } else if constexpr (std::is_same_v<value_type, double>) {
            result = cblas_ddot(self.size(), self.data(), 1, self.data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            std::complex<float> dotc;
            cblas_cdotc_sub(self.size(), self.data(), 1, self.data(), 1, &dotc);
            result = dotc.real();
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            std::complex<double> dotc;
            cblas_zdotc_sub(self.size(), self.data(), 1, self.data(), 1, &dotc);
            result = dotc.real();
        } else {
            result = std::transform_reduce(
                self.data(), self.data() + self.size(), 0.0,
                std::plus<detail::DenseNormTraitT<value_type>>{},
                [](const value_type& a) constexpr noexcept -> detail::DenseNormTraitT<value_type> {
                    return std::norm(a);
                }
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
        return result;
    }

    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto identicalTo(
        this const Self& self, const std::same_as<Self> auto& obj,
        detail::DenseNormTraitT<typename Self::value_type> tol = 1E-8
    ) noexcept -> bool {
        using size_type = typename Self::size_type;

        if (self.nrows() != obj.nrows() || self.ncols() != obj.ncols()) {
            return false;
        }
        // Same type, hence same layout: compare the storage elementwise.
        detail::DenseNormTraitT<typename Self::value_type> test(0.0);
        for (size_type k{0}; k < self.size(); ++k) {
            test = std::max(
                test, std::abs(self.data()[k] - obj.data()[k]) /
                          std::max(std::abs(self.data()[k]), static_cast<decltype(test)>(1.0))
            );
        }
        return test < tol;
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto transposed(this const Self& self) noexcept(
        std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>
    ) -> detail::DenseTransposedTraitT<std::remove_cvref_t<Self>> {
        using index_type = typename std::remove_cvref_t<Self>::index_type;

        const index_type rows{self.nrows()}, cols{self.ncols()};
        detail::DenseTransposedTraitT<std::remove_cvref_t<Self>> result(
            cols, rows, self.get_allocator()
        );
        for (index_type i{0}; i < rows; ++i) {
            for (index_type j{0}; j < cols; ++j) {
                result[j, i] = self[i, j];
            }
        }
        return result;
    }

    template <typename Self>
        requires(!std::is_const_v<std::remove_reference_t<Self>>)
    [[using gnu: always_inline]]
    constexpr auto selfConjugated(this Self&& self) noexcept -> std::remove_cvref_t<Self>& {
        using value_type = typename std::remove_cvref_t<Self>::value_type;

#ifdef BASJOO_USE_BLAS_LAPACK
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
        if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
            std::transform(
                self.data(), self.data() + self.size(), self.data(),
                [](const value_type& a) constexpr noexcept -> value_type { return std::conj(a); }
            );
        }
#endif
        return static_cast<std::remove_cvref_t<Self>&>(self);
    }

    template <typename Self>
    [[using gnu: always_inline]]
    constexpr auto conjugated(this Self&& self) noexcept(
        std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>> || Reusable<Self>
    ) -> decltype(auto) {
        using value_type = typename std::remove_cvref_t<Self>::value_type;

        if constexpr (Reusable<Self>) {
            if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
                self.selfConjugated();
            }
            return static_cast<Self&&>(self);
        } else {
            std::remove_cvref_t<Self> result{self};
            if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
                result.selfConjugated();
            }
            return result;
        }
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto adjoint(this const Self& self) noexcept(
        std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>
    ) -> detail::DenseTransposedTraitT<std::remove_cvref_t<Self>> {
        auto result{self.transposed()};
        result.selfConjugated();
        return result;
    }

    template <typename Self, typename Obj>
        requires DenseDotable<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(this const Self& self, const Obj& obj) noexcept(
        !BASJOO_CHECK_PARAMS || std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>
    ) -> detail::DenseDotTraitT<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> {
        using layout_type [[maybe_unused]] = typename std::remove_cvref_t<Self>::layout_type;
        using value_type = typename std::remove_cvref_t<Self>::value_type;
        using index_type = typename std::remove_cvref_t<Self>::index_type;

        if constexpr (std::derived_from<std::remove_cvref_t<Obj>, MatrixBase>) {
#if BASJOO_CHECK_PARAMS == 1
            if constexpr (!std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>) {
                if (self.ncols() != obj.nrows()) [[unlikely]] {
                    throw std::invalid_argument("Matrix::dot: incompatible dimensions");
                }
            }
#endif
            const index_type outer_size = obj.ncols();
            detail::DenseDotTraitT<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> result(
                self.nrows(), outer_size, value_type(0.0)
            );
#ifdef BASJOO_USE_BLAS_LAPACK
            [[maybe_unused]] constexpr CBLAS_ORDER cblas_layout{
                std::is_same_v<layout_type, std::layout_left> ? CblasColMajor : CblasRowMajor
            };
            // One stride is 1, so the product is the leading dimension whichever layout is in use.
            [[maybe_unused]] const auto lda{static_cast<blasint>(self.stride(0) * self.stride(1))};
            [[maybe_unused]] const auto ldb{static_cast<blasint>(obj.stride(0) * obj.stride(1))};
            [[maybe_unused]] const auto ldc{
                static_cast<blasint>(result.stride(0) * result.stride(1))
            };
            [[maybe_unused]] constexpr value_type alpha(1.0), beta(0.0);
            if constexpr (std::is_same_v<value_type, float>) {
                cblas_sgemm(
                    cblas_layout, CblasNoTrans, CblasNoTrans, static_cast<blasint>(self.nrows()),
                    static_cast<blasint>(outer_size), static_cast<blasint>(self.ncols()), alpha,
                    self.data(), lda, obj.data(), ldb, beta, result.data(), ldc
                );
            } else if constexpr (std::is_same_v<value_type, double>) {
                cblas_dgemm(
                    cblas_layout, CblasNoTrans, CblasNoTrans, static_cast<blasint>(self.nrows()),
                    static_cast<blasint>(outer_size), static_cast<blasint>(self.ncols()), alpha,
                    self.data(), lda, obj.data(), ldb, beta, result.data(), ldc
                );
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                cblas_cgemm(
                    cblas_layout, CblasNoTrans, CblasNoTrans, static_cast<blasint>(self.nrows()),
                    static_cast<blasint>(outer_size), static_cast<blasint>(self.ncols()), &alpha,
                    self.data(), lda, obj.data(), ldb, &beta, result.data(), ldc
                );
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                cblas_zgemm(
                    cblas_layout, CblasNoTrans, CblasNoTrans, static_cast<blasint>(self.nrows()),
                    static_cast<blasint>(outer_size), static_cast<blasint>(self.ncols()), &alpha,
                    self.data(), lda, obj.data(), ldb, &beta, result.data(), ldc
                );
            } else {
                for (index_type i{0}; i < self.nrows(); ++i) {
                    for (index_type j{0}; j < outer_size; ++j) {
                        for (index_type k{0}; k < self.ncols(); ++k) {
                            result[i, j] += self[i, k] * obj[k, j];
                        }
                    }
                }
            }
#else
            for (index_type i{0}; i < self.nrows(); ++i) {
                for (index_type j{0}; j < outer_size; ++j) {
                    for (index_type k{0}; k < self.ncols(); ++k) {
                        result[i, j] += self[i, k] * obj[k, j];
                    }
                }
            }
#endif
            return result;
        } else {
#if BASJOO_CHECK_PARAMS == 1
            if constexpr (!std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>) {
                if (static_cast<typename std::remove_cvref_t<Self>::size_type>(self.ncols()) !=
                    obj.size()) [[unlikely]] {
                    throw std::invalid_argument("Matrix::dot: incompatible dimensions");
                }
            }
#endif
            auto result{detail::makeFilledVector<
                detail::DenseDotTraitT<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>>>(
                static_cast<std::size_t>(self.nrows()), value_type(0.0)
            )};
#ifdef BASJOO_USE_BLAS_LAPACK
            [[maybe_unused]] constexpr CBLAS_ORDER cblas_layout{
                std::is_same_v<layout_type, std::layout_left> ? CblasColMajor : CblasRowMajor
            };
            // One stride is 1, so the product is the leading dimension whichever layout is in use.
            [[maybe_unused]] const auto lda{static_cast<blasint>(self.stride(0) * self.stride(1))};
            [[maybe_unused]] constexpr value_type alpha(1.0), beta(0.0);
            if constexpr (std::is_same_v<value_type, float>) {
                cblas_sgemv(
                    cblas_layout, CblasNoTrans, static_cast<blasint>(self.nrows()),
                    static_cast<blasint>(self.ncols()), alpha, self.data(), lda, obj.data(), 1,
                    beta, result.data(), 1
                );
            } else if constexpr (std::is_same_v<value_type, double>) {
                cblas_dgemv(
                    cblas_layout, CblasNoTrans, static_cast<blasint>(self.nrows()),
                    static_cast<blasint>(self.ncols()), alpha, self.data(), lda, obj.data(), 1,
                    beta, result.data(), 1
                );
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                cblas_cgemv(
                    cblas_layout, CblasNoTrans, static_cast<blasint>(self.nrows()),
                    static_cast<blasint>(self.ncols()), &alpha, self.data(), lda, obj.data(), 1,
                    &beta, result.data(), 1
                );
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                cblas_zgemv(
                    cblas_layout, CblasNoTrans, static_cast<blasint>(self.nrows()),
                    static_cast<blasint>(self.ncols()), &alpha, self.data(), lda, obj.data(), 1,
                    &beta, result.data(), 1
                );
            } else {
                for (index_type i{0}; i < self.nrows(); ++i) {
                    for (index_type j{0}; j < self.ncols(); ++j) {
                        result[static_cast<std::size_t>(i)] +=
                            self[i, j] * obj[static_cast<std::size_t>(j)];
                    }
                }
            }
#else
            for (index_type i{0}; i < self.nrows(); ++i) {
                for (index_type j{0}; j < self.ncols(); ++j) {
                    result[static_cast<std::size_t>(i)] +=
                        self[i, j] * obj[static_cast<std::size_t>(j)];
                }
            }
#endif
            return result;
        }
    }

  protected:
    constexpr MatrixBase() noexcept = default;
    constexpr MatrixBase(const MatrixBase&) noexcept = default;
    constexpr MatrixBase(MatrixBase&&) noexcept = default;
    constexpr auto operator=(const MatrixBase&) noexcept -> MatrixBase& = default;
    constexpr auto operator=(MatrixBase&&) noexcept -> MatrixBase& = default;
    constexpr ~MatrixBase() noexcept = default;
};

// Like std::abs on std::complex (and Vec2/Vec3), the magnitude: the Frobenius norm.
template <typename T>
    requires std::derived_from<T, MatrixBase>
[[using gnu: pure, always_inline]]
inline constexpr auto abs(const T& matrix) noexcept
    -> detail::DenseNormTraitT<typename T::value_type> {
    return matrix.euclidean();
}

// Like std::norm on std::complex (and Vec2/Vec3), the squared magnitude.
template <typename T>
    requires std::derived_from<T, MatrixBase>
[[using gnu: pure, always_inline]]
inline constexpr auto norm(const T& matrix) noexcept
    -> detail::DenseNormTraitT<typename T::value_type> {
    return matrix.euclideanSqr();
}

template <typename T>
    requires std::derived_from<std::remove_cvref_t<T>, MatrixBase>
[[using gnu: always_inline]]
inline constexpr auto conj(T&& matrix) noexcept(
    std::is_nothrow_copy_constructible_v<std::remove_cvref_t<T>> || Reusable<T>
) -> decltype(auto) {
    return std::forward<T>(matrix).conjugated();
}

} // namespace basjoo::math

// Expose the overloads through std:: as well (e.g. `std::abs(lhs - rhs)` in generic code). A
// using-declaration names the same entity, so unqualified calls that also find it through ADL
// on std-namespaced template arguments stay unambiguous.
// NOLINTBEGIN(cert-dcl58-cpp)
namespace std {

using ::basjoo::math::abs;
using ::basjoo::math::conj;
using ::basjoo::math::norm;

} // namespace std
// NOLINTEND(cert-dcl58-cpp)
