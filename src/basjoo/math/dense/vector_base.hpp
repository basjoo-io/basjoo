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
#include <span>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "cblas.h"

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_arithmetic_base.hpp"
#include "basjoo/math/dense/dense_lanes.hpp"
#include "basjoo/math/dense/dense_trait.hpp"
#include "basjoo/math/dense/detail/dense_dot_trait.hpp"
#include "basjoo/math/dense/detail/dense_norm_trait.hpp"
#include "basjoo/math/type_traits.hpp"

namespace basjoo::math {

// When Blas is true, euclidean, euclideanSqr, dot and orthogonalTo call BLAS at run time for
// float, double and their complex types; otherwise, and in constant evaluation, they run plain
// loops. Vector derives VectorBase<false>: its two class heads are where BLAS is switched on. The
// members work on the extent(0) elements, which a padded type stores in more than extent(0) slots.
template <bool Blas = false>
class VectorBase : public DenseArithmeticBase {
  public:
    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator[](
        this Self&& self, std::convertible_to<typename std::remove_cvref_t<Self>::index_type> auto i
    ) noexcept -> decltype(auto) {
        return self.data()[static_cast<typename std::remove_cvref_t<Self>::index_type>(i)];
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto coeff(
        this const Self& self,
        std::convertible_to<typename std::remove_cvref_t<Self>::index_type> auto i
    ) noexcept(!BASJOO_CHECK_PARAMS) -> typename std::remove_cvref_t<Self>::value_type {
        const auto k{static_cast<typename std::remove_cvref_t<Self>::index_type>(i)};
#if BASJOO_CHECK_PARAMS == 1
        if (k >= self.size()) [[unlikely]] {
            throw std::out_of_range("Vector index out of range");
        }
#endif
        return self.data()[k];
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto updateCoeff(
        this Self&& self,
        std::convertible_to<typename std::remove_cvref_t<Self>::index_type> auto i,
        typename std::remove_cvref_t<Self>::const_reference value
    ) noexcept(!BASJOO_CHECK_PARAMS) -> void {
        const auto k{static_cast<typename std::remove_cvref_t<Self>::index_type>(i)};
#if BASJOO_CHECK_PARAMS == 1
        if (k >= self.extent(0)) [[unlikely]] {
            throw std::out_of_range("Vector index out of range");
        }
#endif
        self.data()[k] = value;
        return;
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto fill(
        this Self&& self, typename std::remove_cvref_t<Self>::const_reference value
    ) noexcept -> void {
        using size_type = typename std::remove_cvref_t<Self>::size_type;

        const auto n{static_cast<size_type>(self.extent(0))};
        for (size_type k{0}; k < n; ++k) {
            self.data()[k] = value;
        }
        return;
    }

    template <typename Self>
    [[using gnu: pure, always_inline, leaf]]
    constexpr auto empty(this const Self& self) noexcept -> bool {
        return self.size() == 0;
    }

    // A static extent is checked at compile time, where a dynamic one has dynamic_extent and is
    // checked at run time instead; a fixed-size vector has no throw left, so its call is noexcept.
    template <std::size_t Count, typename Self>
    [[using gnu: always_inline]]
    constexpr auto first(this Self&& self) noexcept(
        !BASJOO_CHECK_PARAMS || std::remove_cvref_t<Self>::rank_dynamic() == 0
    ) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;
        static_assert(
            Count <= std::remove_cvref_t<Self>::static_extent(0),
            "Vector::first: count out of range."
        );

#if BASJOO_CHECK_PARAMS == 1
        if constexpr (std::remove_cvref_t<Self>::rank_dynamic() != 0) {
            if (Count > static_cast<std::size_t>(self.extent(0))) [[unlikely]] {
                throw std::out_of_range("Vector::first: count out of range.");
            }
        }
#endif
        return std::span<element_type, Count>{self.data(), Count};
    }

    template <typename Self>
    [[using gnu: always_inline]]
    constexpr auto first(
        this Self&& self, typename std::remove_cvref_t<Self>::size_type count
    ) noexcept(!BASJOO_CHECK_PARAMS) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;

#if BASJOO_CHECK_PARAMS == 1
        if (count > static_cast<decltype(count)>(self.extent(0))) [[unlikely]] {
            throw std::out_of_range("Vector::first: count out of range.");
        }
#endif
        return std::span<element_type>{self.data(), count};
    }

    template <std::size_t Count, typename Self>
    [[using gnu: always_inline]]
    constexpr auto last(this Self&& self) noexcept(
        !BASJOO_CHECK_PARAMS || std::remove_cvref_t<Self>::rank_dynamic() == 0
    ) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;
        static_assert(
            Count <= std::remove_cvref_t<Self>::static_extent(0),
            "Vector::last: count out of range."
        );

#if BASJOO_CHECK_PARAMS == 1
        if constexpr (std::remove_cvref_t<Self>::rank_dynamic() != 0) {
            if (Count > static_cast<std::size_t>(self.extent(0))) [[unlikely]] {
                throw std::out_of_range("Vector::last: count out of range.");
            }
        }
#endif
        return std::span<element_type, Count>{
            self.data() + (static_cast<std::size_t>(self.extent(0)) - Count), Count
        };
    }

    template <typename Self>
    [[using gnu: always_inline]]
    constexpr auto last(
        this Self&& self, typename std::remove_cvref_t<Self>::size_type count
    ) noexcept(!BASJOO_CHECK_PARAMS) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;

        const auto n{static_cast<decltype(count)>(self.extent(0))};
#if BASJOO_CHECK_PARAMS == 1
        if (count > n) [[unlikely]] {
            throw std::out_of_range("Vector::last: count out of range.");
        }
#endif
        return std::span<element_type>{self.data() + (n - count), count};
    }

    template <std::size_t Offset, std::size_t Count = std::dynamic_extent, typename Self>
    [[using gnu: always_inline]]
    constexpr auto subspan(this Self&& self) noexcept(
        !BASJOO_CHECK_PARAMS || std::remove_cvref_t<Self>::rank_dynamic() == 0
    ) {
        using self_type = std::remove_cvref_t<Self>;
        using element_type = std::remove_pointer_t<decltype(self.data())>;
        static_assert(
            Offset <= self_type::static_extent(0) &&
                (Count == std::dynamic_extent || Count <= self_type::static_extent(0) - Offset),
            "Vector::subspan: invalid range."
        );
        constexpr std::size_t kExtent{
            Count != std::dynamic_extent ? Count
                                         : (self_type::static_extent(0) != std::dynamic_extent
                                                ? self_type::static_extent(0) - Offset
                                                : std::dynamic_extent)
        };
        const auto n{static_cast<std::size_t>(self.extent(0))};
#if BASJOO_CHECK_PARAMS == 1
        if constexpr (self_type::rank_dynamic() != 0) {
            if (Offset > n || (Count != std::dynamic_extent && Count > n - Offset)) [[unlikely]] {
                throw std::out_of_range("Vector::subspan: invalid range.");
            }
        }
#endif
        return std::span<element_type, kExtent>{
            self.data() + Offset, Count != std::dynamic_extent ? Count : n - Offset
        };
    }

    template <typename Self>
    [[using gnu: always_inline]]
    constexpr auto subspan(
        this Self&& self, typename std::remove_cvref_t<Self>::size_type offset
    ) noexcept(!BASJOO_CHECK_PARAMS) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;

        const auto n{static_cast<decltype(offset)>(self.extent(0))};
#if BASJOO_CHECK_PARAMS == 1
        if (offset > n) [[unlikely]] {
            throw std::out_of_range("Vector::subspan: invalid range.");
        }
#endif
        return std::span<element_type>{self.data() + offset, n - offset};
    }

    template <typename Self>
    [[using gnu: always_inline]]
    constexpr auto subspan(
        this Self&& self, typename std::remove_cvref_t<Self>::size_type offset,
        typename std::remove_cvref_t<Self>::size_type count
    ) noexcept(!BASJOO_CHECK_PARAMS) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;

#if BASJOO_CHECK_PARAMS == 1
        const auto n{static_cast<decltype(offset)>(self.extent(0))};
        if (offset > n || count > n - offset) [[unlikely]] {
            throw std::out_of_range("Vector::subspan: invalid range.");
        }
#endif
        return std::span<element_type>{self.data() + offset, count};
    }

    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclidean(this const Self& self) noexcept
        -> detail::DenseNormTraitT<typename std::remove_cvref_t<Self>::value_type> {
        using value_type = typename std::remove_cvref_t<Self>::value_type;
        using norm_type = detail::DenseNormTraitT<value_type>;

        if constexpr (Blas) {
            if !consteval {
                // nrm2 scales its sum against overflow and underflow.
                if constexpr (std::is_same_v<value_type, float>) {
                    return cblas_snrm2(self.extent(0), self.data(), 1);
                } else if constexpr (std::is_same_v<value_type, double>) {
                    return cblas_dnrm2(self.extent(0), self.data(), 1);
                } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                    return cblas_scnrm2(self.extent(0), self.data(), 1);
                } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                    return cblas_dznrm2(self.extent(0), self.data(), 1);
                }
            }
        }
        return static_cast<norm_type>(std::sqrt(self.euclideanSqr()));
    }

    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclideanSqr(this const Self& self) noexcept
        -> detail::DenseNormTraitT<typename std::remove_cvref_t<Self>::value_type> {
        using value_type = typename std::remove_cvref_t<Self>::value_type;
        using size_type = typename std::remove_cvref_t<Self>::size_type;
        using norm_type = detail::DenseNormTraitT<value_type>;

        if constexpr (Blas) {
            if !consteval {
                // A complex element is two reals, whose squares sum to its squared magnitude.
                if constexpr (std::is_same_v<value_type, float>) {
                    return cblas_sdot(self.extent(0), self.data(), 1, self.data(), 1);
                } else if constexpr (std::is_same_v<value_type, double>) {
                    return cblas_ddot(self.extent(0), self.data(), 1, self.data(), 1);
                } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                    const auto* const lanes{reinterpret_cast<const float*>(self.data())};
                    return cblas_sdot(2 * self.extent(0), lanes, 1, lanes, 1);
                } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                    const auto* const lanes{reinterpret_cast<const double*>(self.data())};
                    return cblas_ddot(2 * self.extent(0), lanes, 1, lanes, 1);
                }
            }
        }
        const value_type* const p{self.data()};
        const auto n{static_cast<size_type>(self.extent(0))};
        norm_type sum(0.0);
        for (size_type k{0}; k < n; ++k) {
            if constexpr (ComplexArithmetic<value_type>) {
                sum += p[k].real() * p[k].real() + p[k].imag() * p[k].imag();
            } else {
                sum = static_cast<norm_type>(sum + p[k] * p[k]);
            }
        }
        return sum;
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto euclideanTo(this const Self& self, const std::same_as<Self> auto& obj) noexcept(
        !BASJOO_CHECK_PARAMS || Self::rank_dynamic() == 0
    ) -> detail::DenseNormTraitT<typename Self::value_type> {
#if BASJOO_CHECK_PARAMS == 1
        // Two fixed-size vectors of one type have one extent: only a dynamic one is checked, so
        // that the call stays noexcept for fixed sizes.
        if constexpr (Self::rank_dynamic() != 0) {
            if (self.extent(0) != obj.extent(0)) [[unlikely]] {
                throw std::invalid_argument("Vectors must have the same size.");
            }
        }
#endif
        return (self - obj).euclidean();
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto normalized(this Self&& self) noexcept(
        std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>> || Reusable<Self>
    ) -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self /= self.euclidean();
            return static_cast<Self&&>(self);
        } else {
            return self / self.euclidean();
        }
    }

    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto identicalTo(
        this const Self& self, const std::same_as<Self> auto& obj,
        detail::DenseNormTraitT<typename Self::value_type> tol = 1E-8
    ) noexcept -> bool {
        using size_type = typename Self::size_type;

        if (self.extent(0) != obj.extent(0)) {
            return false;
        }
        detail::DenseNormTraitT<typename Self::value_type> test(0.0);
        const auto n{static_cast<size_type>(self.extent(0))};
        for (size_type i{0}; i < n; ++i) {
            test = std::max(
                test, std::abs(self.data()[i] - obj.data()[i]) /
                          std::max(std::abs(self.data()[i]), static_cast<decltype(test)>(1.0))
            );
        }
        return test < tol;
    }

    // |<self, obj>| within tol^2, the inner product conjugating self. The bound is inclusive, so
    // that an integer tol, 1E-8 truncated to 0 by default, asks for an exactly zero product.
    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto orthogonalTo(
        this const Self& self, const std::same_as<Self> auto& obj,
        detail::DenseNormTraitT<typename Self::value_type> tol =
            detail::DenseNormTraitT<typename Self::value_type>(1E-8)
    ) noexcept -> bool {
        using value_type = typename Self::value_type;
        using size_type = typename Self::size_type;

        if (self.extent(0) != obj.extent(0)) [[unlikely]] {
            return false;
        }
        if constexpr (Blas) {
            if !consteval {
                if constexpr (std::is_same_v<value_type, float>) {
                    return std::abs(cblas_sdot(self.extent(0), self.data(), 1, obj.data(), 1)) <=
                           tol * tol;
                } else if constexpr (std::is_same_v<value_type, double>) {
                    return std::abs(cblas_ddot(self.extent(0), self.data(), 1, obj.data(), 1)) <=
                           tol * tol;
                } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                    value_type inner;
                    cblas_cdotc_sub(self.extent(0), self.data(), 1, obj.data(), 1, &inner);
                    return std::abs(inner) <= tol * tol;
                } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                    value_type inner;
                    cblas_zdotc_sub(self.extent(0), self.data(), 1, obj.data(), 1, &inner);
                    return std::abs(inner) <= tol * tol;
                }
            }
        }
        const auto n{static_cast<size_type>(self.extent(0))};
        value_type inner(0.0);
        for (size_type k{0}; k < n; ++k) {
            if constexpr (ComplexArithmetic<value_type>) {
                inner += std::conj(self[k]) * obj[k];
            } else {
                inner = static_cast<value_type>(inner + self[k] * obj[k]);
            }
        }
        if constexpr (std::is_unsigned_v<value_type>) {
            return inner <= tol * tol;
        } else {
            return std::abs(inner) <= tol * tol;
        }
    }

    template <typename Self>
        requires(!std::is_const_v<std::remove_reference_t<Self>>)
    [[using gnu: always_inline]]
    constexpr auto selfConjugated(this Self&& self) noexcept -> std::remove_cvref_t<Self>& {
        using value_type = typename std::remove_cvref_t<Self>::value_type;

        if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
            DenseLanes<value_type>::conjugate(self.size(), self.data());
        }
        return static_cast<std::remove_cvref_t<Self>&>(self);
    }

    template <typename Self>
    [[using gnu: always_inline]]
    constexpr auto conjugated(this Self&& self) noexcept(
        std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>> || Reusable<Self>
    ) -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self.selfConjugated();
            return static_cast<Self&&>(self);
        } else {
            std::remove_cvref_t<Self> result{self};
            result.selfConjugated();
            return result;
        }
    }

    template <typename Self, typename Obj>
        requires DenseDotable<Self, Obj>
    [[using gnu: always_inline, hot]]
    constexpr auto dot(this const Self& self, const Obj& obj) noexcept(
        !BASJOO_CHECK_PARAMS || std::is_nothrow_copy_constructible_v<Self>
    ) -> detail::DenseDotTraitT<Self, Obj> {
        using value_type = typename Self::value_type;
        using size_type = typename Self::size_type;

        const auto n{static_cast<size_type>(self.extent(0))};
        if constexpr (Obj::rank() == 1) {
#if BASJOO_CHECK_PARAMS == 1
            if constexpr (!std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>) {
                if (self.extent(0) != obj.extent(0)) [[unlikely]] {
                    throw std::invalid_argument(
                        "Vector::dot: vector and vector dimensions do not match"
                    );
                }
            }
#endif
            value_type result(0.0);
            if constexpr (Blas) {
                if !consteval {
                    if constexpr (std::is_same_v<value_type, float>) {
                        return cblas_sdot(self.extent(0), self.data(), 1, obj.data(), 1);
                    } else if constexpr (std::is_same_v<value_type, double>) {
                        return cblas_ddot(self.extent(0), self.data(), 1, obj.data(), 1);
                    } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                        cblas_cdotu_sub(self.extent(0), self.data(), 1, obj.data(), 1, &result);
                        return result;
                    } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                        cblas_zdotu_sub(self.extent(0), self.data(), 1, obj.data(), 1, &result);
                        return result;
                    }
                }
            }
            for (size_type k{0}; k < n; ++k) {
                result = static_cast<value_type>(result + self[k] * obj[k]);
            }
            return result;
        } else {
            using obj_index_type = typename Obj::index_type;
#if BASJOO_CHECK_PARAMS == 1
            if constexpr (!std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>) {
                if (n != static_cast<size_type>(obj.nrows())) [[unlikely]] {
                    throw std::invalid_argument(
                        "Vector::dot: Matrix and vector dimensions do not match"
                    );
                }
            }
#endif
            const size_type outer_size{static_cast<size_type>(obj.ncols())};
            using result_type = detail::DenseDotTraitT<Self, Obj>;
            result_type result(outer_size, value_type(0.0), self.get_allocator());
            if constexpr (Blas) {
                if !consteval {
                    // x^T A == A^T x
                    [[maybe_unused]] constexpr CBLAS_ORDER cblas_layout{
                        std::is_same_v<typename Obj::layout_type, std::layout_left> ? CblasColMajor
                                                                                    : CblasRowMajor
                    };
                    [[maybe_unused]] const auto m{static_cast<blasint>(obj.nrows())};
                    [[maybe_unused]] const auto cols{static_cast<blasint>(obj.ncols())};
                    // One stride is 1, so the product is the leading dimension whichever layout is
                    // in use.
                    [[maybe_unused]] const auto lda{
                        static_cast<blasint>(obj.stride(0) * obj.stride(1))
                    };
                    [[maybe_unused]] constexpr value_type alpha(1.0), beta(0.0);
                    if constexpr (std::is_same_v<value_type, float>) {
                        cblas_sgemv(
                            cblas_layout, CblasTrans, m, cols, alpha, obj.data(), lda, self.data(),
                            1, beta, result.data(), 1
                        );
                        return result;
                    } else if constexpr (std::is_same_v<value_type, double>) {
                        cblas_dgemv(
                            cblas_layout, CblasTrans, m, cols, alpha, obj.data(), lda, self.data(),
                            1, beta, result.data(), 1
                        );
                        return result;
                    } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                        cblas_cgemv(
                            cblas_layout, CblasTrans, m, cols, &alpha, obj.data(), lda, self.data(),
                            1, &beta, result.data(), 1
                        );
                        return result;
                    } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                        cblas_zgemv(
                            cblas_layout, CblasTrans, m, cols, &alpha, obj.data(), lda, self.data(),
                            1, &beta, result.data(), 1
                        );
                        return result;
                    }
                }
            }
            if constexpr (std::is_same_v<typename Obj::layout_type, std::layout_left>) {
                // Each element is the product with a column of obj.
                for (size_type j{0}; j < outer_size; ++j) {
                    for (size_type i{0}; i < n; ++i) {
                        result[j] +=
                            self[i] *
                            obj[static_cast<obj_index_type>(i), static_cast<obj_index_type>(j)];
                    }
                }
            } else {
                // A combination of the rows of obj, each element still summing its terms in i
                // order.
                for (size_type i{0}; i < n; ++i) {
                    const value_type x{self[i]};
                    for (size_type j{0}; j < outer_size; ++j) {
                        result[j] +=
                            x * obj[static_cast<obj_index_type>(i), static_cast<obj_index_type>(j)];
                    }
                }
            }
            return result;
        }
    }

  protected:
    constexpr VectorBase() noexcept = default;
    constexpr VectorBase(const VectorBase&) noexcept = default;
    constexpr VectorBase(VectorBase&&) noexcept = default;
    constexpr auto operator=(const VectorBase&) noexcept -> VectorBase& = default;
    constexpr auto operator=(VectorBase&&) noexcept -> VectorBase& = default;
    constexpr ~VectorBase() noexcept = default;
};

// Like std::abs on std::complex (and Vec2/Vec3), the magnitude: the Euclidean norm.
template <typename T>
    requires std::derived_from<T, VectorBase<false>> || std::derived_from<T, VectorBase<true>>
[[using gnu: pure, always_inline]]
inline constexpr auto abs(const T& vector) noexcept
    -> detail::DenseNormTraitT<typename T::value_type> {
    return vector.euclidean();
}

// Like std::norm on std::complex (and Vec2/Vec3), the squared magnitude.
template <typename T>
    requires std::derived_from<T, VectorBase<false>> || std::derived_from<T, VectorBase<true>>
[[using gnu: pure, always_inline]]
inline constexpr auto norm(const T& vector) noexcept
    -> detail::DenseNormTraitT<typename T::value_type> {
    return vector.euclideanSqr();
}

template <typename T>
    requires std::derived_from<std::remove_cvref_t<T>, VectorBase<false>> ||
             std::derived_from<std::remove_cvref_t<T>, VectorBase<true>>
[[using gnu: always_inline]]
inline constexpr auto conj(T&& vector) noexcept(
    std::is_nothrow_copy_constructible_v<std::remove_cvref_t<T>> || Reusable<T>
) -> decltype(auto) {
    return std::forward<T>(vector).conjugated();
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
