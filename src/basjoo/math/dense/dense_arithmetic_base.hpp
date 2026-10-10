/**
 * @file dense_arithmetic_base.hpp
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
#include <complex>
#include <concepts>
#include <cstddef>
#include <mdspan>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <utility>

#ifdef BASJOO_USE_BLAS_LAPACK
#include "cblas.h"
#endif

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_trait.hpp"
#include "basjoo/math/dense/detail/dense_norm_trait.hpp"

namespace basjoo::math {

class DenseArithmeticBase {
  public:
    template <typename Self, typename Obj>
    [[using gnu: always_inline, hot]]
    constexpr auto operator+=(this Self&& self, Obj&& obj) noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> std::remove_cvref_t<Self>& requires std::same_as<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> {
        using value_type = typename DenseTrait<std::remove_cvref_t<Self>>::value_type;
#if BASJOO_CHECK_PARAMS == 1
        if constexpr (requires(const std::remove_cvref_t<Self>& s) { s.nrows(); }) {
            if (self.nrows() != obj.nrows() || self.ncols() != obj.ncols()) [[unlikely]] {
                throw std::invalid_argument("Matrix dimensions do not match.");
            }
        } else {
            if (self.size() != obj.size()) [[unlikely]] {
                throw std::invalid_argument("Vector::+=: sizes do not match.");
            }
        }
#endif
#ifdef BASJOO_USE_BLAS_LAPACK
        [[maybe_unused]] constexpr value_type alpha(1.0);
        if constexpr (std::is_same_v<value_type, float>) {
            cblas_saxpy(self.size(), alpha, obj.data(), 1, self.data(), 1);
        } else if constexpr (std::is_same_v<value_type, double>) {
            cblas_daxpy(self.size(), alpha, obj.data(), 1, self.data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            cblas_caxpy(self.size(), &alpha, obj.data(), 1, self.data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            cblas_zaxpy(self.size(), &alpha, obj.data(), 1, self.data(), 1);
        } else {
            std::transform(
                self.data(), self.data() + self.size(), obj.data(), self.data(),
                std::plus<value_type>()
            );
        }
#else
        std::transform(
            self.data(), self.data() + self.size(), obj.data(), self.data(), std::plus<value_type>()
        );
#endif
        return static_cast<std::remove_cvref_t<Self>&>(self);
    }

    template <typename Self, typename Obj>
    [[using gnu: always_inline, hot]]
    constexpr auto operator-=(this Self&& self, Obj&& obj) noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> std::remove_cvref_t<Self>& requires std::same_as<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> {
        using value_type = typename DenseTrait<std::remove_cvref_t<Self>>::value_type;
#if BASJOO_CHECK_PARAMS == 1
        if constexpr (requires(const std::remove_cvref_t<Self>& s) { s.nrows(); }) {
            if (self.nrows() != obj.nrows() || self.ncols() != obj.ncols()) [[unlikely]] {
                throw std::invalid_argument("Matrix dimensions do not match.");
            }
        } else {
            if (self.size() != obj.size()) [[unlikely]] {
                throw std::invalid_argument("Vector::-=: sizes do not match.");
            }
        }
#endif
#ifdef BASJOO_USE_BLAS_LAPACK
        [[maybe_unused]] constexpr value_type alpha(-1.0);
        if constexpr (std::is_same_v<value_type, float>) {
            cblas_saxpy(self.size(), alpha, obj.data(), 1, self.data(), 1);
        } else if constexpr (std::is_same_v<value_type, double>) {
            cblas_daxpy(self.size(), alpha, obj.data(), 1, self.data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            cblas_caxpy(self.size(), &alpha, obj.data(), 1, self.data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            cblas_zaxpy(self.size(), &alpha, obj.data(), 1, self.data(), 1);
        } else {
            std::transform(
                self.data(), self.data() + self.size(), obj.data(), self.data(),
                std::minus<value_type>()
            );
        }
#else
        std::transform(
            self.data(), self.data() + self.size(), obj.data(), self.data(),
            std::minus<value_type>()
        );
#endif
        return static_cast<std::remove_cvref_t<Self>&>(self);
    }

    template <typename Self, typename Obj>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator+(this Self&& self, Obj&& obj) noexcept(
        !BASJOO_CHECK_PARAMS && (
         std::is_rvalue_reference_v<Self&&> || std::is_rvalue_reference_v<Obj&&>)
    ) -> decltype(auto) requires std::same_as<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> {
        if constexpr (std::is_rvalue_reference_v<Self&&>) {
            self.operator+=(obj);
            return std::forward<Self>(self);
        } else if constexpr (std::is_rvalue_reference_v<Obj&&>) {
            obj.operator+=(self);
            return std::forward<Obj>(obj);
        } else {
            std::remove_cvref_t<Self> result{self};
            result.operator+=(obj);
            return std::remove_cvref_t<Self>{std::move(result)};
        }
    }

    template <typename Self, typename Obj>
    [[using gnu: always_inline, hot]]
    constexpr auto operator-(this Self&& self, Obj&& obj) noexcept(
        !BASJOO_CHECK_PARAMS && (
         std::is_rvalue_reference_v<Self&&> || std::is_rvalue_reference_v<Obj&&>)
    ) -> decltype(auto) requires std::same_as<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> {
        if constexpr (std::is_rvalue_reference_v<Self&&>) {
            self.operator-=(obj);
            return std::forward<Self>(self);
        } else if constexpr (std::is_rvalue_reference_v<Obj&&>) {
            obj.operator*=(-1.0);
            obj.operator+=(self);
            return std::forward<Obj>(obj);
        } else {
            std::remove_cvref_t<Self> result{self};
            result.operator-=(obj);
            return std::remove_cvref_t<Self>{std::move(result)};
        }
    }

    template <typename Self, typename Obj>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator==(this const Self& self, const Obj& obj) noexcept -> bool requires std::same_as<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> {
        if constexpr (requires(const std::remove_cvref_t<Self>& s) { s.nrows(); }) {
            if (self.nrows() != obj.nrows() || self.ncols() != obj.ncols()) [[unlikely]] {
                return false;
            }
        } else {
            if (self.size() != obj.size()) [[unlikely]] {
                return false;
            }
        }
        const std::size_t n{self.size()};
        for (std::size_t i{0}; i < n; ++i) {
            if (self.data()[i] != obj.data()[i]) {
                return false;
            }
        }
        return true;
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto operator*=(this Self&& self, const ScalarArithmetic auto& fac) noexcept
        -> std::remove_cvref_t<Self>& {
        using value_type = typename DenseTrait<std::remove_cvref_t<Self>>::value_type;
        const auto alpha{static_cast<value_type>(fac)};
#ifdef BASJOO_USE_BLAS_LAPACK
        if constexpr (std::is_same_v<value_type, float>) {
            cblas_sscal(self.size(), alpha, self.data(), 1);
        } else if constexpr (std::is_same_v<value_type, double>) {
            cblas_dscal(self.size(), alpha, self.data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            cblas_cscal(self.size(), &alpha, self.data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            cblas_zscal(self.size(), &alpha, self.data(), 1);
        } else {
            std::transform(
                self.data(), self.data() + self.size(), self.data(),
                [alpha](const value_type& x) constexpr noexcept -> value_type { return x * alpha; }
            );
        }
#else
        std::transform(
            self.data(), self.data() + self.size(), self.data(),
            [alpha](const value_type& x) constexpr noexcept -> value_type { return x * alpha; }
        );
#endif
        return static_cast<std::remove_cvref_t<Self>&>(self);
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto operator/=(this Self&& self, const ScalarArithmetic auto& den) noexcept
        -> std::remove_cvref_t<Self>& {
        using value_type = typename DenseTrait<std::remove_cvref_t<Self>>::value_type;
        if constexpr (std::is_integral_v<value_type>) {
            const auto alpha{static_cast<value_type>(den)};
            std::transform(
                self.data(), self.data() + self.size(), self.data(),
                [alpha](const value_type& x) constexpr noexcept -> value_type { return x / alpha; }
            );
        } else if constexpr (requires { static_cast<value_type>(1.0 / den); }) {
            const auto alpha{static_cast<value_type>(1.0 / den)};
            self.operator*=(alpha);
        } else {
            // std::operator/(double, std::complex<T>) only exists for T == double; take the
            // reciprocal in the den's own field for the remaining complex scalars.
            const auto alpha{static_cast<value_type>(
                typename std::remove_cvref_t<decltype(den)>::value_type(1.0) / den
            )};
            self.operator*=(alpha);
        }
        return static_cast<std::remove_cvref_t<Self>&>(self);
    }

    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator*(this Self&& self, const ScalarArithmetic auto& fac) noexcept(!BASJOO_CHECK_PARAMS) -> decltype(auto) {
        if constexpr (std::is_rvalue_reference_v<Self&&>) {
            self.operator*=(fac);
            return std::forward<Self>(self);
        } else {
            std::remove_cvref_t<Self> result{self};
            result.operator*=(fac);
            return std::remove_cvref_t<Self>{std::move(result)};
        }
    }

    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator/(this Self&& self, const ScalarArithmetic auto& den) noexcept(!BASJOO_CHECK_PARAMS) -> decltype(auto) {
        if constexpr (std::is_rvalue_reference_v<Self&&>) {
            self.operator/=(den);
            return std::forward<Self>(self);
        } else {
            std::remove_cvref_t<Self> result{self};
            result.operator/=(den);
            return std::remove_cvref_t<Self>{std::move(result)};
        }
    }

    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator-(this Self&& self) noexcept(!BASJOO_CHECK_PARAMS) -> decltype(auto) {
        if constexpr (std::is_rvalue_reference_v<Self&&>) {
            self.operator*=(-1.0);
            return std::forward<Self>(self);
        } else {
            std::remove_cvref_t<Self> result{self};
            result.operator*=(-1.0);
            return std::remove_cvref_t<Self>{std::move(result)};
        }
    }

  protected:
    constexpr DenseArithmeticBase() noexcept = default;
    constexpr DenseArithmeticBase(const DenseArithmeticBase&) noexcept = default;
    constexpr DenseArithmeticBase(DenseArithmeticBase&&) noexcept = default;
    constexpr auto operator=(const DenseArithmeticBase&) noexcept -> DenseArithmeticBase& = default;
    constexpr auto operator=(DenseArithmeticBase&&) noexcept -> DenseArithmeticBase& = default;
    constexpr ~DenseArithmeticBase() noexcept = default;
};

template <typename T>
[[using gnu: always_inline, hot]]
inline constexpr auto operator*(const ScalarArithmetic auto& fac, T&& obj) noexcept(!BASJOO_CHECK_PARAMS) -> decltype(auto) requires std::derived_from<std::remove_cvref_t<T>, DenseArithmeticBase> {
    if constexpr (std::is_rvalue_reference_v<T&&>) {
        obj.operator*=(fac);
        return std::forward<T>(obj);
    } else {
        return obj.operator*(fac);
    }
}

} // namespace basjoo::math
