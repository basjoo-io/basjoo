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
#include <functional>
#include <numeric>
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
        this const Self& self, typename std::remove_cvref_t<Self>::index_type i
    ) noexcept(!BASJOO_CHECK_PARAMS) -> typename std::remove_cvref_t<Self>::value_type {
#if BASJOO_CHECK_PARAMS == 1
        if (i >= self.size()) [[unlikely]] {
            throw std::out_of_range("Vector index out of range");
        }
#endif
        return self.data()[i];
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto updateCoeff(
        this Self&& self, typename std::remove_cvref_t<Self>::index_type i,
        typename std::remove_cvref_t<Self>::const_reference value
    ) noexcept(!BASJOO_CHECK_PARAMS) -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (i >= self.size()) [[unlikely]] {
            throw std::out_of_range("Vector index out of range");
        }
#endif
        self.data()[i] = value;
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

    template <std::size_t Count, typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto first(this Self&& self) noexcept(!BASJOO_CHECK_PARAMS) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;

#if BASJOO_CHECK_PARAMS == 1
        if (Count > self.size()) [[unlikely]] {
            throw std::out_of_range("Vector::first: count out of range.");
        }
#endif
        return std::span<element_type, Count>{self.data(), Count};
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto first(
        this Self&& self, typename std::remove_cvref_t<Self>::size_type count
    ) noexcept(!BASJOO_CHECK_PARAMS) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;

#if BASJOO_CHECK_PARAMS == 1
        if (count > self.size()) [[unlikely]] {
            throw std::out_of_range("Vector::first: count out of range.");
        }
#endif
        return std::span<element_type>{self.data(), count};
    }

    template <std::size_t Count, typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto last(this Self&& self) noexcept(!BASJOO_CHECK_PARAMS) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;

#if BASJOO_CHECK_PARAMS == 1
        if (Count > self.size()) [[unlikely]] {
            throw std::out_of_range("Vector::last: count out of range.");
        }
#endif
        return std::span<element_type, Count>{self.data() + (self.size() - Count), Count};
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto last(
        this Self&& self, typename std::remove_cvref_t<Self>::size_type count
    ) noexcept(!BASJOO_CHECK_PARAMS) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;

#if BASJOO_CHECK_PARAMS == 1
        if (count > self.size()) [[unlikely]] {
            throw std::out_of_range("Vector::last: count out of range.");
        }
#endif
        return std::span<element_type>{self.data() + (self.size() - count), count};
    }

    template <std::size_t Offset, std::size_t Count = std::dynamic_extent, typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto subspan(this Self&& self) noexcept(!BASJOO_CHECK_PARAMS) {
        using self_type = std::remove_cvref_t<Self>;
        using element_type = std::remove_pointer_t<decltype(self.data())>;
        constexpr std::size_t kExtent{
            Count != std::dynamic_extent ? Count
                                         : (self_type::static_extent(0) != std::dynamic_extent
                                                ? self_type::static_extent(0) - Offset
                                                : std::dynamic_extent)
        };
#if BASJOO_CHECK_PARAMS == 1
        if (Offset > self.size() || (Count != std::dynamic_extent && Count > self.size() - Offset))
            [[unlikely]] {
            throw std::out_of_range("Vector::subspan: invalid range.");
        }
#endif
        return std::span<element_type, kExtent>{
            self.data() + Offset, Count != std::dynamic_extent ? Count : self.size() - Offset
        };
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto subspan(
        this Self&& self, typename std::remove_cvref_t<Self>::size_type offset
    ) noexcept(!BASJOO_CHECK_PARAMS) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;

#if BASJOO_CHECK_PARAMS == 1
        if (offset > self.size()) [[unlikely]] {
            throw std::out_of_range("Vector::subspan: invalid range.");
        }
#endif
        return std::span<element_type>{self.data() + offset, self.size() - offset};
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto subspan(
        this Self&& self, typename std::remove_cvref_t<Self>::size_type offset,
        typename std::remove_cvref_t<Self>::size_type count
    ) noexcept(!BASJOO_CHECK_PARAMS) {
        using element_type = std::remove_pointer_t<decltype(self.data())>;

#if BASJOO_CHECK_PARAMS == 1
        if (offset > self.size() || count > self.size() - offset) [[unlikely]] {
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
    constexpr auto euclideanTo(this const Self& self, const std::same_as<Self> auto& obj) noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> detail::DenseNormTraitT<typename Self::value_type> {
#if BASJOO_CHECK_PARAMS == 1
        if (self.size() != obj.size()) [[unlikely]] {
            throw std::invalid_argument("Vectors must have the same size.");
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

        if (self.size() != obj.size()) {
            return false;
        }
        detail::DenseNormTraitT<typename Self::value_type> test(0.0);
        for (size_type i{0}; i < self.size(); ++i) {
            test = std::max(
                test, std::abs(self.data()[i] - obj.data()[i]) /
                          std::max(std::abs(self.data()[i]), static_cast<decltype(test)>(1.0))
            );
        }
        return test < tol;
    }

    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto orthogonalTo(
        this const Self& self, const std::same_as<Self> auto& obj,
        detail::DenseNormTraitT<typename Self::value_type> tol = 1E-8
    ) noexcept -> bool {
        if (self.size() != obj.size()) [[unlikely]] {
            return false;
        }
        return std::abs(self.dot(obj)) < tol * tol;
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
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(this const Self& self, const Obj& obj) noexcept(
        !BASJOO_CHECK_PARAMS || std::is_nothrow_copy_constructible_v<Self>
    ) -> detail::DenseDotTraitT<Self, Obj> {
        using value_type = typename Self::value_type;
        using size_type = typename Self::size_type;

        if constexpr (Obj::rank() == 1) {
#if BASJOO_CHECK_PARAMS == 1
            if constexpr (!std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>) {
                if (self.size() != obj.size()) [[unlikely]] {
                    throw std::invalid_argument(
                        "Vector::dot: vector and vector dimensions do not match"
                    );
                }
            }
#endif
            value_type result(0.0);
#ifdef BASJOO_USE_BLAS_LAPACK
            if constexpr (std::is_same_v<value_type, float>) {
                result = cblas_sdot(self.size(), self.data(), 1, obj.data(), 1);
            } else if constexpr (std::is_same_v<value_type, double>) {
                result = cblas_ddot(self.size(), self.data(), 1, obj.data(), 1);
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                cblas_cdotu_sub(self.size(), self.data(), 1, obj.data(), 1, &result);
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                cblas_zdotu_sub(self.size(), self.data(), 1, obj.data(), 1, &result);
            } else {
                result = std::transform_reduce(
                    self.data(), self.data() + self.size(), obj.data(), result
                );
            }
#else
            result =
                std::transform_reduce(self.data(), self.data() + self.size(), obj.data(), result);
#endif
            return result;
        } else {
            using obj_index_type = typename Obj::index_type;
#if BASJOO_CHECK_PARAMS == 1
            if constexpr (!std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>) {
                if (self.size() != static_cast<size_type>(obj.nrows())) [[unlikely]] {
                    throw std::invalid_argument(
                        "Vector::dot: Matrix and vector dimensions do not match"
                    );
                }
            }
#endif
            const size_type outer_size{static_cast<size_type>(obj.ncols())};
            auto result{detail::makeFilledVector<detail::DenseDotTraitT<Self, Obj>>(
                outer_size, value_type(0.0)
            )};
#ifdef BASJOO_USE_BLAS_LAPACK
            // x^T A == A^T x
            [[maybe_unused]] constexpr CBLAS_ORDER cblas_layout{
                std::is_same_v<typename Obj::layout_type, std::layout_left> ? CblasColMajor
                                                                            : CblasRowMajor
            };
            [[maybe_unused]] const auto m{static_cast<blasint>(obj.nrows())};
            [[maybe_unused]] const auto n{static_cast<blasint>(obj.ncols())};
            // One stride is 1, so the product is the leading dimension whichever layout is in use.
            [[maybe_unused]] const auto lda{static_cast<blasint>(obj.stride(0) * obj.stride(1))};
            [[maybe_unused]] constexpr value_type alpha(1.0), beta(0.0);
            if constexpr (std::is_same_v<value_type, float>) {
                cblas_sgemv(
                    cblas_layout, CblasTrans, m, n, alpha, obj.data(), lda, self.data(), 1, beta,
                    result.data(), 1
                );
            } else if constexpr (std::is_same_v<value_type, double>) {
                cblas_dgemv(
                    cblas_layout, CblasTrans, m, n, alpha, obj.data(), lda, self.data(), 1, beta,
                    result.data(), 1
                );
            } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                cblas_cgemv(
                    cblas_layout, CblasTrans, m, n, &alpha, obj.data(), lda, self.data(), 1, &beta,
                    result.data(), 1
                );
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                cblas_zgemv(
                    cblas_layout, CblasTrans, m, n, &alpha, obj.data(), lda, self.data(), 1, &beta,
                    result.data(), 1
                );
            } else {
                for (size_type j{0}; j < outer_size; ++j) {
                    for (size_type i{0}; i < self.size(); ++i) {
                        result[j] +=
                            self[i] *
                            obj[static_cast<obj_index_type>(i), static_cast<obj_index_type>(j)];
                    }
                }
            }
#else
            for (size_type j{0}; j < outer_size; ++j) {
                for (size_type i{0}; i < self.size(); ++i) {
                    result[j] +=
                        self[i] *
                        obj[static_cast<obj_index_type>(i), static_cast<obj_index_type>(j)];
                }
            }
#endif
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
    requires std::derived_from<T, VectorBase>
[[using gnu: pure, always_inline]]
inline constexpr auto abs(const T& vector) noexcept
    -> detail::DenseNormTraitT<typename T::value_type> {
    return vector.euclidean();
}

// Like std::norm on std::complex (and Vec2/Vec3), the squared magnitude.
template <typename T>
    requires std::derived_from<T, VectorBase>
[[using gnu: pure, always_inline]]
inline constexpr auto norm(const T& vector) noexcept
    -> detail::DenseNormTraitT<typename T::value_type> {
    return vector.euclideanSqr();
}

template <typename T>
    requires std::derived_from<std::remove_cvref_t<T>, VectorBase>
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
