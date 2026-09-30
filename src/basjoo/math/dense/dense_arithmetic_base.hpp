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
#include <functional>
#include <stdexcept>
#include <type_traits>
#include <utility>

#ifdef BASJOO_USE_BLAS_LAPACK
#include "cblas.h"
#endif

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_trait.hpp"

namespace basjoo::math {

// Binary/unary operators on rvalue operands mutate and return the operand itself as `T&&`
// (zero copy); lvalue-only operands yield a new `T` by value. Binding the result of an
// expression whose reused operand is a temporary to a reference outlives that temporary.
class DenseArithmeticBase {
  public:
    template <typename Self>
        requires(!std::is_const_v<std::remove_reference_t<Self>>)
    [[using gnu: always_inline, hot]]
    constexpr auto
    operator+=(this Self&& self, const std::same_as<std::remove_cvref_t<Self>> auto& obj) noexcept(
        !BASJOO_CHECK_PARAMS || std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>
    ) -> std::remove_cvref_t<Self>& {
        using value_type = typename std::remove_cvref_t<Self>::value_type;

#if BASJOO_CHECK_PARAMS == 1
        if constexpr (!std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>) {
            using rank_type = typename std::remove_cvref_t<Self>::rank_type;
            for (rank_type r{0}; r < std::remove_cvref_t<Self>::rank(); ++r) {
                if (self.extent(r) != obj.extent(r)) [[unlikely]] {
                    throw std::invalid_argument(
                        "DenseArithmeticBase::+=: dimensions do not match!"
                    );
                }
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

    template <typename Self>
        requires(!std::is_const_v<std::remove_reference_t<Self>>)
    [[using gnu: always_inline, hot]]
    constexpr auto
    operator-=(this Self&& self, const std::same_as<std::remove_cvref_t<Self>> auto& obj) noexcept(
        !BASJOO_CHECK_PARAMS || std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>
    ) -> std::remove_cvref_t<Self>& {
        using value_type = typename std::remove_cvref_t<Self>::value_type;

#if BASJOO_CHECK_PARAMS == 1
        if constexpr (!std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>) {
            using rank_type = typename std::remove_cvref_t<Self>::rank_type;
            for (rank_type r{0}; r < std::remove_cvref_t<Self>::rank(); ++r) {
                if (self.extent(r) != obj.extent(r)) [[unlikely]] {
                    throw std::invalid_argument(
                        "DenseArithmeticBase::-=: dimensions do not match!"
                    );
                }
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
        requires std::same_as<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>>
    [[using gnu: always_inline, hot]]
    constexpr auto operator+(this Self&& self, Obj&& obj) noexcept(
        !BASJOO_CHECK_PARAMS || std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>
    ) -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self += obj;
            return static_cast<Self&&>(self);
        } else if constexpr (Reusable<Obj>) {
            obj += self;
            return static_cast<Obj&&>(obj);
        } else {
            std::remove_cvref_t<Self> result{self};
            result += obj;
            return result;
        }
    }

    template <typename Self, typename Obj>
        requires std::same_as<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>>
    [[using gnu: always_inline, hot]]
    constexpr auto operator-(this Self&& self, Obj&& obj) noexcept(
        !BASJOO_CHECK_PARAMS || std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>
    ) -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self -= obj;
            return static_cast<Self&&>(self);
        } else if constexpr (Reusable<Obj>) {
            using value_type = typename std::remove_cvref_t<Obj>::value_type;

#if BASJOO_CHECK_PARAMS == 1
            if constexpr (!std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Obj>>) {
                using rank_type = typename std::remove_cvref_t<Obj>::rank_type;
                for (rank_type r{0}; r < std::remove_cvref_t<Obj>::rank(); ++r) {
                    if (self.extent(r) != obj.extent(r)) [[unlikely]] {
                        throw std::invalid_argument(
                            "DenseArithmeticBase::-: dimensions do not match!"
                        );
                    }
                }
            }
#endif
            std::transform(
                self.data(), self.data() + self.size(), obj.data(), obj.data(),
                std::minus<value_type>()
            );
            return static_cast<Obj&&>(obj);
        } else {
            std::remove_cvref_t<Self> result{self};
            result -= obj;
            return result;
        }
    }

    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator==(this const Self& self, const std::same_as<Self> auto& obj) noexcept
        -> bool {
        using rank_type = typename Self::rank_type;
        using size_type = typename Self::size_type;

        for (rank_type r{0}; r < Self::rank(); ++r) {
            if (self.extent(r) != obj.extent(r)) [[unlikely]] {
                return false;
            }
        }
        const size_type n{self.size()};
        for (size_type i{0}; i < n; ++i) {
            if (self.data()[i] != obj.data()[i]) {
                return false;
            }
        }
        return true;
    }

    template <typename Self>
        requires(!std::is_const_v<std::remove_reference_t<Self>>)
    [[using gnu: always_inline, hot]]
    constexpr auto operator*=(this Self&& self, const ScalarArithmetic auto& fac) noexcept
        -> std::remove_cvref_t<Self>& {
        using value_type = typename std::remove_cvref_t<Self>::value_type;
        static_assert(
            ComplexArithmetic<value_type> || Arithmetic<decltype(fac)>,
            "DenseArithmeticBase::*=: a real valued object only scales by a real scalar!"
        );

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
        requires(!std::is_const_v<std::remove_reference_t<Self>>)
    [[using gnu: always_inline, hot]]
    constexpr auto operator/=(this Self&& self, const ScalarArithmetic auto& den) noexcept
        -> std::remove_cvref_t<Self>& {
        using value_type = typename std::remove_cvref_t<Self>::value_type;
        static_assert(
            ComplexArithmetic<value_type> || Arithmetic<decltype(den)>,
            "DenseArithmeticBase::/=: a real valued object only scales by a real scalar!"
        );

        if constexpr (std::is_integral_v<value_type>) {
            const auto alpha{static_cast<value_type>(den)};
            std::transform(
                self.data(), self.data() + self.size(), self.data(),
                [alpha](const value_type& x) constexpr noexcept -> value_type { return x / alpha; }
            );
        } else if constexpr (requires { static_cast<value_type>(1.0 / den); }) {
            const auto alpha{static_cast<value_type>(1.0 / den)};
            self *= alpha;
        } else {
            const auto alpha{static_cast<value_type>(
                typename std::remove_cvref_t<decltype(den)>::value_type(1.0) / den
            )};
            self *= alpha;
        }
        return static_cast<std::remove_cvref_t<Self>&>(self);
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto operator*(this Self&& self, const ScalarArithmetic auto& fac) noexcept(
        std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>> || Reusable<Self>
    ) -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self *= fac;
            return static_cast<Self&&>(self);
        } else {
            std::remove_cvref_t<Self> result{self};
            result *= fac;
            return result;
        }
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto operator/(this Self&& self, const ScalarArithmetic auto& den) noexcept(
        std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>> || Reusable<Self>
    ) -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self /= den;
            return static_cast<Self&&>(self);
        } else {
            std::remove_cvref_t<Self> result{self};
            result /= den;
            return result;
        }
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto operator-(this Self&& self) noexcept(
        std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>> || Reusable<Self>
    ) -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self.selfNegate();
            return static_cast<Self&&>(self);
        } else {
            std::remove_cvref_t<Self> result{self};
            result.selfNegate();
            return result;
        }
    }

  protected:
    constexpr DenseArithmeticBase() noexcept = default;
    constexpr DenseArithmeticBase(const DenseArithmeticBase&) noexcept = default;
    constexpr DenseArithmeticBase(DenseArithmeticBase&&) noexcept = default;
    constexpr auto operator=(const DenseArithmeticBase&) noexcept -> DenseArithmeticBase& = default;
    constexpr auto operator=(DenseArithmeticBase&&) noexcept -> DenseArithmeticBase& = default;
    constexpr ~DenseArithmeticBase() noexcept = default;

  private:
    template <typename Self>
        requires(!std::is_const_v<Self>)
    [[using gnu: always_inline, hot]]
    constexpr auto selfNegate(this Self& self) noexcept -> void {
        using value_type = typename Self::value_type;

#ifdef BASJOO_USE_BLAS_LAPACK
        [[maybe_unused]] constexpr value_type alpha(-1.0);
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
                self.data(), self.data() + self.size(), self.data(), std::negate<value_type>()
            );
        }
#else
        std::transform(
            self.data(), self.data() + self.size(), self.data(), std::negate<value_type>()
        );
#endif
    }
};

namespace detail {

// A dense vector of `n` copies of `value`. The Vector<T, Index, 2/3> specializations do
// not derive from DenseArithmeticBase; their size is implied and they take a broadcast scalar.
template <typename V>
[[using gnu: always_inline]]
constexpr auto makeFilledVector(std::size_t n, const typename V::value_type& value) -> V {
    if constexpr (std::derived_from<V, DenseArithmeticBase>) {
        return V(n, value);
    } else {
        return V(value);
    }
}

} // namespace detail

template <typename T>
    requires std::derived_from<std::remove_cvref_t<T>, DenseArithmeticBase>
[[using gnu: always_inline, hot]]
inline constexpr auto operator*(const ScalarArithmetic auto& fac, T&& obj) noexcept(
    std::is_nothrow_copy_constructible_v<std::remove_cvref_t<T>> || Reusable<T>
) -> decltype(auto) {
    if constexpr (Reusable<T>) {
        obj *= fac;
        return static_cast<T&&>(obj);
    } else {
        return obj * fac;
    }
}

} // namespace basjoo::math
