/**
 * @file vector_base.hpp
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
#include <iterator>
#include <numeric>
#include <ostream>
#include <span>
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
#include "basjoo/math/type_traits.hpp"

namespace basjoo::math {

class VectorBase : public DenseArithmeticBase {
  public:
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator[](this auto&& self, std::size_t i) noexcept -> decltype(auto) {
        return self.data()[i];
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto coeff(this const auto& self, std::size_t i) noexcept(!BASJOO_CHECK_PARAMS) ->
        typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type {
#if BASJOO_CHECK_PARAMS == 1
        if (i >= self.size()) [[unlikely]] {
            throw std::out_of_range("Vector index out of range");
        }
#endif
        return self.data()[i];
    }

    [[using gnu: always_inline, hot]]
    constexpr auto updateCoeff(
        this auto&& self, std::size_t i,
        const typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type& value
    ) noexcept(!BASJOO_CHECK_PARAMS) -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (i >= self.size()) [[unlikely]] {
            throw std::out_of_range("Vector index out of range");
        }
#endif
        self.data()[i] = value;
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

    [[using gnu: const, always_inline, leaf]]
    constexpr auto empty(this const auto& self) noexcept -> bool {
        return self.size() == 0;
    }

    // std::span-shaped element access — the underlying storage is contiguous,
    // so plain pointers serve as contiguous iterators. Calling front/back on an
    // empty vector is undefined behavior, matching std::span.
    [[using gnu: pure, always_inline, hot]]
    constexpr auto front(this auto&& self) noexcept -> decltype(auto) {
        return self.data()[0];
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto back(this auto&& self) noexcept -> decltype(auto) {
        return self.data()[self.size() - 1];
    }

    [[using gnu: pure, always_inline]]
    constexpr auto size_bytes(this const auto& self) noexcept -> std::size_t {
        return self.size() * sizeof(typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type);
    }

    [[using gnu: pure, always_inline]]
    constexpr auto begin(this auto&& self) noexcept -> decltype(auto) {
        return self.data();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto end(this auto&& self) noexcept -> decltype(auto) {
        return self.data() + self.size();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto cbegin(this const auto& self) noexcept -> decltype(auto) {
        return self.data();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto cend(this const auto& self) noexcept -> decltype(auto) {
        return self.data() + self.size();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto rbegin(this auto&& self) noexcept -> std::reverse_iterator<
        std::remove_cvref_t<decltype(self.data())>> {
        return std::reverse_iterator<std::remove_cvref_t<decltype(self.data())>>{self.end()};
    }

    [[using gnu: pure, always_inline]]
    constexpr auto rend(this auto&& self) noexcept -> std::reverse_iterator<
        std::remove_cvref_t<decltype(self.data())>> {
        return std::reverse_iterator<std::remove_cvref_t<decltype(self.data())>>{self.begin()};
    }

    [[using gnu: pure, always_inline]]
    constexpr auto crbegin(this const auto& self) noexcept ->
        typename DenseTrait<std::remove_cvref_t<decltype(self)>>::const_reverse_iterator {
        return self.cend();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto crend(this const auto& self) noexcept ->
        typename DenseTrait<std::remove_cvref_t<decltype(self)>>::const_reverse_iterator {
        return self.cbegin();
    }

    /// Contiguous sub-span over the half-open range [range.first, range.second).
    /// With BASJOO_CHECK_PARAMS enabled, an inverted or out-of-bounds range
    /// throws std::out_of_range instead of being silently clamped.
    [[using gnu: pure, always_inline]]
    constexpr auto subspan(this auto&& self, std::pair<std::size_t, std::size_t> range) noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> decltype(auto) {
        using element_type = std::conditional_t<
            std::is_const_v<std::remove_reference_t<decltype(self)>>,
            const typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type,
            typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type>;
#if BASJOO_CHECK_PARAMS == 1
        if (range.first > range.second || range.second > self.size()) [[unlikely]] {
            throw std::out_of_range("Vector::subspan: invalid range.");
        }
#endif
        return std::span<element_type>{self.data() + range.first, range.second - range.first};
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclidean(this const auto& self) noexcept -> detail::DenseNormTraitT<
        typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type> {
        using value_type = typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type;
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

    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclideanSqr(this const auto& self) noexcept -> detail::DenseNormTraitT<
        typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type> {
        using value_type = typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type;
        detail::DenseNormTraitT<value_type> result(0.0);
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

    template <typename Self, typename Obj>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclideanTo(this const Self& self, const Obj& obj) noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> detail::DenseNormTraitT<typename DenseTrait<std::remove_cvref_t<Self>>::value_type> requires std::same_as<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> {
#if BASJOO_CHECK_PARAMS == 1
        if (self.size() != obj.size()) [[unlikely]] {
            throw std::invalid_argument("Vectors must have the same size.");
        }
#endif
        return (self - obj).euclidean();
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto normalized(this const auto& self) noexcept
        -> std::remove_cvref_t<decltype(self)> {
        return self / self.euclidean();
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto identicalTo(
        this const auto& self,
        std::span<const typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type> obj,
        double tol = 1E-8
    ) noexcept -> bool {
        if (self.size() != obj.size()) {
            return false;
        }
        detail::DenseNormTraitT<
            typename DenseTrait<std::remove_cvref_t<decltype(self)>>::value_type>
            test(0.0);
        for (std::size_t i{0}; i < self.size(); ++i) {
            test = std::max(
                test, std::abs(self[i] - obj[i]) /
                          std::max(std::abs(self[i]), static_cast<decltype(test)>(1.0))
            );
        }
        return test < tol;
    }

    template <typename Self, typename Obj>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto orthogonalTo(
        this const Self& self, const Obj& obj,
        typename DenseTrait<std::remove_cvref_t<Self>>::value_type tol = 1E-8
    ) noexcept -> bool requires std::same_as<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> {
#if BASJOO_CHECK_PARAMS == 1
        if (self.size() != obj.size()) [[unlikely]] {
            throw std::invalid_argument("Vectors must have the same size.");
        }
#endif
        return std::abs(self.dot(obj)) < tol * tol;
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

    template <typename Self, typename Obj>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(this const Self& self, const Obj& obj) noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> typename DenseTrait<std::remove_cvref_t<Self>>::value_type requires std::same_as<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> {
        using value_type = typename DenseTrait<std::remove_cvref_t<Self>>::value_type;
#if BASJOO_CHECK_PARAMS == 1
        if (self.size() != obj.size()) [[unlikely]] {
            throw std::invalid_argument("Vector::dot: vector and vector dimensions do not match");
        }
#endif
        value_type result(0.0);
#ifdef BASJOO_USE_BLAS_LAPACK
        if constexpr (std::is_same_v<value_type, float>) {
            result = cblas_sdot(self.size(), self.data(), 1, obj.data(), 1);
        } else if constexpr (std::is_same_v<value_type, double>) {
            result = cblas_ddot(self.size(), self.data(), 1, obj.data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            result = cblas_cdotu(self.size(), self.data(), 1, obj.data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            result = cblas_zdotu(self.size(), self.data(), 1, obj.data(), 1);
        } else {
            result =
                std::transform_reduce(self.data(), self.data() + self.size(), obj.data(), result);
        }
#else
        result = std::transform_reduce(self.data(), self.data() + self.size(), obj.data(), result);
#endif
        return result;
    }

    template <typename Self, typename Obj>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(this const Self& self, const Obj& obj) noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> detail::DenseDotTraitT<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> requires DenseDotable<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> {
#if BASJOO_CHECK_PARAMS == 1
        if (self.size() != obj.nrows()) [[unlikely]] {
            throw std::invalid_argument("Vector::dot: Matrix and vector dimensions do not match");
        }
#endif
        const std::size_t outer_size = obj.ncols();
        detail::DenseDotTraitT<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> result(
            outer_size, 0.0
        );
#ifdef BASJOO_USE_BLAS_LAPACK
        using value_type = typename DenseTrait<std::remove_cvref_t<Self>>::value_type;
        [[maybe_unused]] const CBLAS_ORDER cblas_order{
            std::is_same_v<
                typename DenseTrait<std::remove_cvref_t<Obj>>::layout_type, std::layout_left>
                ? CblasColMajor
                : CblasRowMajor
        };
        [[maybe_unused]] const blasint lda = static_cast<blasint>(
            std::is_same_v<
                typename DenseTrait<std::remove_cvref_t<Obj>>::layout_type, std::layout_left>
                ? outer_size
                : self.size()
        );
        [[maybe_unused]] constexpr value_type alpha(1.0), beta(0.0);
        if constexpr (std::is_same_v<value_type, float>) {
            cblas_sgemv(
                cblas_order, CblasNoTrans, static_cast<blasint>(outer_size),
                static_cast<blasint>(self.size()), alpha, obj.data(), lda, self.data(), 1, beta,
                result.data(), 1
            );
        } else if constexpr (std::is_same_v<value_type, double>) {
            cblas_dgemv(
                cblas_order, CblasNoTrans, static_cast<blasint>(outer_size),
                static_cast<blasint>(self.size()), alpha, obj.data(), lda, self.data(), 1, beta,
                result.data(), 1
            );
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            cblas_cgemv(
                cblas_order, CblasNoTrans, static_cast<blasint>(outer_size),
                static_cast<blasint>(self.size()), &alpha, obj.data(), lda, self.data(), 1, &beta,
                result.data(), 1
            );
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            cblas_zgemv(
                cblas_order, CblasNoTrans, static_cast<blasint>(outer_size),
                static_cast<blasint>(self.size()), &alpha, obj.data(), lda, self.data(), 1, &beta,
                result.data(), 1
            );
        } else {
            const std::size_t n = self.size();
            for (std::size_t j{0}; j < outer_size; ++j) {
                for (std::size_t i{0}; i < n; ++i) {
                    result[j] += self[i] * obj[i, j];
                }
            }
        }
#else
        const std::size_t n = self.size();
        for (std::size_t j{0}; j < outer_size; ++j) {
            for (std::size_t i{0}; i < n; ++i) {
                result[j] += self[i] * obj[i, j];
            }
        }
#endif
        return result;
    }

  protected:
    constexpr VectorBase() noexcept = default;
    constexpr VectorBase(const VectorBase&) noexcept = default;
    constexpr VectorBase(VectorBase&&) noexcept = default;
    constexpr auto operator=(const VectorBase&) noexcept -> VectorBase& = default;
    constexpr auto operator=(VectorBase&&) noexcept -> VectorBase& = default;
    constexpr ~VectorBase() noexcept = default;
};

template <typename T>
[[using gnu: pure, always_inline]]
inline constexpr auto abs(const T& vector) noexcept
    -> detail::DenseNormTraitT<std::remove_cvref_t<T>> requires std::derived_from<std::remove_cvref_t<T>, VectorBase> {
    using value_type = typename DenseTrait<std::remove_cvref_t<T>>::value_type;
    detail::DenseNormTraitT<std::remove_cvref_t<T>> result(vector);
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
inline auto operator<<(std::basic_ostream<Char>& os, const T& vector) noexcept
    -> std::basic_ostream<Char>& requires std::derived_from<std::remove_cvref_t<T>, VectorBase> {
    using value_type = typename DenseTrait<std::remove_cvref_t<T>>::value_type;
    constexpr std::size_t kWidth{isComplexArithmeticV<value_type> ? 32 : 16};
    const std::size_t size{vector.size()};
    os << std::fixed;
    for (std::size_t i{0}; i < size; ++i) {
        os << std::setw(kWidth) << vector[i] << '\n';
    }
    return os;
}

template <typename T>
[[using gnu: pure, always_inline]]
inline constexpr auto conj(T&& vector) noexcept(!BASJOO_CHECK_PARAMS) -> decltype(auto) requires std::derived_from<std::remove_cvref_t<T>, VectorBase> {
    if constexpr (std::is_rvalue_reference_v<T&&>) {
        vector.selfConjugated();
        return std::forward<T>(vector);
    } else {
        return vector.conjugated();
    }
}

} // namespace basjoo::math

namespace std {

template <basjoo::math::ScalarArithmetic Scalar, std::size_t Extent, basjoo::math::Allocatory Alloc>
[[using gnu: pure, always_inline]]
inline constexpr auto abs(const basjoo::math::Vector<Scalar, Extent, Alloc>& vector) noexcept
    -> basjoo::math::detail::DenseNormTraitT<basjoo::math::Vector<Scalar, Extent, Alloc>> {
    return basjoo::math::abs(vector);
}

template <basjoo::math::ScalarArithmetic Scalar, std::size_t Extent, basjoo::math::Allocatory Alloc>
[[using gnu: pure, always_inline]]
inline constexpr auto conj(const basjoo::math::Vector<Scalar, Extent, Alloc>& vector) noexcept
    -> basjoo::math::Vector<Scalar, Extent, Alloc> {
    return basjoo::math::conj(vector);
}

template <basjoo::math::ScalarArithmetic Scalar, std::size_t Extent, basjoo::math::Allocatory Alloc>
[[using gnu: always_inline]]
inline constexpr auto conj(basjoo::math::Vector<Scalar, Extent, Alloc>&& vector) noexcept
    -> basjoo::math::Vector<Scalar, Extent, Alloc>&& {
    return basjoo::math::conj(std::move(vector));
}

} // namespace std

// NOLINTBEGIN(cert-dcl58-cpp)
