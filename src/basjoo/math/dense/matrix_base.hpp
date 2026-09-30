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
#include <mdspan>
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
#include "basjoo/math/dense/detail/dense_transposed_trait.hpp"
#include "basjoo/math/type_traits.hpp"

namespace basjoo::math {

// When Blas is true, euclidean, euclideanSqr and dot call BLAS at run time for float, double and
// their complex types; otherwise, and in constant evaluation, they run plain loops. Matrix derives
// MatrixBase<false>: its two class heads are where BLAS is switched on.
template <bool Blas = false>
class MatrixBase : public DenseArithmeticBase {
  public:
    // The offset from the leading dimension, not the mapping's operator(), which GCC leaves out of
    // line in large translation units; a padded storage would also build its strided mapping on
    // every access. The unit stride stays implicit: multiplying by it makes GCC emit slower loops
    // for a dynamic product. Neither standard library checks the indices in its mapping outside its
    // debug or hardened mode.
    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator[](
        this Self&& self,
        std::convertible_to<typename std::remove_cvref_t<Self>::index_type> auto row,
        std::convertible_to<typename std::remove_cvref_t<Self>::index_type> auto col
    ) noexcept -> decltype(auto) {
        using self_type = std::remove_cvref_t<Self>;
        using index_type = typename self_type::index_type;
        constexpr bool kLeft{std::is_same_v<typename self_type::layout_type, std::layout_left>};

        const auto i{static_cast<index_type>(row)};
        const auto j{static_cast<index_type>(col)};
        // The stride of the dimension that is not contiguous, padding included.
        const index_type lead{kLeft ? self.stride(1) : self.stride(0)};
        return self.data()[kLeft ? i + j * lead : i * lead + j];
    }

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
        std::convertible_to<typename std::remove_cvref_t<Self>::index_type> auto row,
        std::convertible_to<typename std::remove_cvref_t<Self>::index_type> auto col
    ) noexcept(!BASJOO_CHECK_PARAMS) -> typename std::remove_cvref_t<Self>::value_type {
        using index_type = typename std::remove_cvref_t<Self>::index_type;

        const auto i{static_cast<index_type>(row)};
        const auto j{static_cast<index_type>(col)};
#if BASJOO_CHECK_PARAMS == 1
        if (i >= self.nrows() || j >= self.ncols()) [[unlikely]] {
            throw std::out_of_range("Matrix index out of range");
        }
#endif
        return self[i, j];
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto updateCoeff(
        this Self&& self,
        std::convertible_to<typename std::remove_cvref_t<Self>::index_type> auto row,
        std::convertible_to<typename std::remove_cvref_t<Self>::index_type> auto col,
        typename std::remove_cvref_t<Self>::const_reference value
    ) noexcept(!BASJOO_CHECK_PARAMS) -> void {
        using index_type = typename std::remove_cvref_t<Self>::index_type;

        const auto i{static_cast<index_type>(row)};
        const auto j{static_cast<index_type>(col)};
#if BASJOO_CHECK_PARAMS == 1
        if (i >= self.nrows() || j >= self.ncols()) [[unlikely]] {
            throw std::out_of_range("Matrix index out of range");
        }
#endif
        self[i, j] = value;
        return;
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto coeff(
        this const Self& self,
        std::convertible_to<typename std::remove_cvref_t<Self>::index_type> auto i
    ) noexcept(!BASJOO_CHECK_PARAMS) -> typename std::remove_cvref_t<Self>::value_type {
        const auto k{static_cast<typename std::remove_cvref_t<Self>::index_type>(i)};
#if BASJOO_CHECK_PARAMS == 1
        if (static_cast<typename std::remove_cvref_t<Self>::size_type>(k) >= self.size())
            [[unlikely]] {
            throw std::out_of_range("Matrix index out of range.");
        }
#endif
        return self[k];
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
        using self_type = std::remove_cvref_t<Self>;
        using size_type = typename self_type::size_type;

        if (static_cast<size_type>(k) >= self.size()) [[unlikely]] {
            throw std::out_of_range("Matrix index out of range.");
        }
        // A padded storage has one unit stride: in each stored vector, the offsets past the extent
        // of that stride are padding.
        if constexpr (!self_type::is_always_exhaustive()) {
            const auto lead{static_cast<size_type>(self.stride(0) * self.stride(1))};
            const auto extent{
                static_cast<size_type>(self.stride(0) == 1 ? self.nrows() : self.ncols())
            };
            if (static_cast<size_type>(k) % lead >= extent) [[unlikely]] {
                throw std::out_of_range("Matrix::updateCoeff: the offset is padding.");
            }
        }
#endif
        self[k] = value;
        return;
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto fill(
        this Self&& self, typename std::remove_cvref_t<Self>::const_reference value
    ) noexcept -> void {
        using self_type = std::remove_cvref_t<Self>;
        using size_type = typename self_type::size_type;
        using index_type = typename self_type::index_type;

        if constexpr (self_type::is_always_exhaustive()) {
            const size_type n{self.size()};
            for (size_type k{0}; k < n; ++k) {
                self.data()[k] = value;
            }
        } else {
            // Element by element, so that the padding of a strided storage keeps its zeros.
            for (index_type col{0}; col < self.ncols(); ++col) {
                for (index_type row{0}; row < self.nrows(); ++row) {
                    self[row, col] = value;
                }
            }
        }
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
    [[using gnu: always_inline]]
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
    [[using gnu: always_inline]]
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
        using norm_type = detail::DenseNormTraitT<value_type>;

        if constexpr (Blas) {
            if !consteval {
                // nrm2 scales its sum against overflow and underflow.
                if constexpr (std::is_same_v<value_type, float>) {
                    return cblas_snrm2(self.size(), self.data(), 1);
                } else if constexpr (std::is_same_v<value_type, double>) {
                    return cblas_dnrm2(self.size(), self.data(), 1);
                } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                    return cblas_scnrm2(self.size(), self.data(), 1);
                } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                    return cblas_dznrm2(self.size(), self.data(), 1);
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
                    return cblas_sdot(self.size(), self.data(), 1, self.data(), 1);
                } else if constexpr (std::is_same_v<value_type, double>) {
                    return cblas_ddot(self.size(), self.data(), 1, self.data(), 1);
                } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                    const auto* const lanes{reinterpret_cast<const float*>(self.data())};
                    return cblas_sdot(2 * self.size(), lanes, 1, lanes, 1);
                } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                    const auto* const lanes{reinterpret_cast<const double*>(self.data())};
                    return cblas_ddot(2 * self.size(), lanes, 1, lanes, 1);
                }
            }
        }
        const value_type* const p{self.data()};
        const size_type n{self.size()};
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

    // The result is written in its storage order, so that the side copied with a stride is the one
    // read. A large dynamic matrix goes in 16 x 16 tiles, which keep that side in cache. Under
    // 128 x 128 elements, or with a side shorter than a tile, the plain loop is as fast.
    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto transposed(this const Self& self) noexcept(
        std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>
    ) -> detail::DenseTransposedTraitT<std::remove_cvref_t<Self>> {
        using value_type = typename Self::value_type;
        using size_type = typename Self::size_type;
        constexpr bool kLeft{std::is_same_v<typename Self::layout_type, std::layout_left>};
        constexpr size_type kTile{16};

        detail::DenseTransposedTraitT<std::remove_cvref_t<Self>> result(
            self.ncols(), self.nrows(), self.get_allocator()
        );
        // (outer, inner) is (i, j) for layout_left and (j, i) for layout_right: element (i, j) of
        // self is src[inner * outer_size + outer] and goes to dst[outer * inner_size + inner]. Held
        // here, the pointers stay in registers; read through self and result, they would be
        // reloaded after every complex store.
        const auto rows{static_cast<size_type>(self.nrows())};
        const auto cols{static_cast<size_type>(self.ncols())};
        const size_type outer_size{kLeft ? rows : cols}, inner_size{kLeft ? cols : rows};
        const value_type* const src{self.data()};
        value_type* const dst{result.data()};
        if (Self::extents_type::rank_dynamic() == 0 || rows < kTile || cols < kTile ||
            rows * cols < 128 * 128) {
            for (size_type outer{0}; outer < outer_size; ++outer) {
                for (size_type inner{0}; inner < inner_size; ++inner) {
                    dst[outer * inner_size + inner] = src[inner * outer_size + outer];
                }
            }
            return result;
        }
        for (size_type outer_begin{0}; outer_begin < outer_size; outer_begin += kTile) {
            const size_type outer_end{std::min(outer_begin + kTile, outer_size)};
            for (size_type inner_begin{0}; inner_begin < inner_size; inner_begin += kTile) {
                const size_type inner_end{std::min(inner_begin + kTile, inner_size)};
                for (size_type outer{outer_begin}; outer < outer_end; ++outer) {
                    for (size_type inner{inner_begin}; inner < inner_end; ++inner) {
                        dst[outer * inner_size + inner] = src[inner * outer_size + outer];
                    }
                }
            }
        }
        return result;
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

    // The conjugate transpose, in one pass over the elements: the loops of transposed, conjugating
    // each element they copy.
    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto adjoint(this const Self& self) noexcept(
        std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>
    ) -> detail::DenseTransposedTraitT<std::remove_cvref_t<Self>> {
        using value_type = typename Self::value_type;

        if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
            using size_type = typename Self::size_type;
            constexpr bool kLeft{std::is_same_v<typename Self::layout_type, std::layout_left>};
            constexpr size_type kTile{16};

            detail::DenseTransposedTraitT<std::remove_cvref_t<Self>> result(
                self.ncols(), self.nrows(), self.get_allocator()
            );
            const auto rows{static_cast<size_type>(self.nrows())};
            const auto cols{static_cast<size_type>(self.ncols())};
            const size_type outer_size{kLeft ? rows : cols}, inner_size{kLeft ? cols : rows};
            const value_type* const src{self.data()};
            value_type* const dst{result.data()};
            if (Self::extents_type::rank_dynamic() == 0 || rows < kTile || cols < kTile ||
                rows * cols < 128 * 128) {
                for (size_type outer{0}; outer < outer_size; ++outer) {
                    for (size_type inner{0}; inner < inner_size; ++inner) {
                        dst[outer * inner_size + inner] =
                            std::conj(src[inner * outer_size + outer]);
                    }
                }
                return result;
            }
            for (size_type outer_begin{0}; outer_begin < outer_size; outer_begin += kTile) {
                const size_type outer_end{std::min(outer_begin + kTile, outer_size)};
                for (size_type inner_begin{0}; inner_begin < inner_size; inner_begin += kTile) {
                    const size_type inner_end{std::min(inner_begin + kTile, inner_size)};
                    for (size_type outer{outer_begin}; outer < outer_end; ++outer) {
                        for (size_type inner{inner_begin}; inner < inner_end; ++inner) {
                            dst[outer * inner_size + inner] =
                                std::conj(src[inner * outer_size + outer]);
                        }
                    }
                }
            }
            return result;
        } else {
            return self.transposed();
        }
    }

    // NOLINTBEGIN(readability-function-cognitive-complexity)
    template <typename Self, typename Obj>
        requires DenseDotable<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>>
    [[using gnu: always_inline, hot]]
    constexpr auto dot(this const Self& self, const Obj& obj) noexcept(
        !BASJOO_CHECK_PARAMS || std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>
    ) -> detail::DenseDotTraitT<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> {
        using layout_type = typename std::remove_cvref_t<Self>::layout_type;
        using value_type = typename std::remove_cvref_t<Self>::value_type;
        using index_type = typename std::remove_cvref_t<Self>::index_type;
        constexpr bool kLeft{std::is_same_v<layout_type, std::layout_left>};

        // Dispatch on the rank, whichever of MatrixBase<true> and MatrixBase<false> the operand
        // derives.
        if constexpr (std::remove_cvref_t<Obj>::rank() == 2) {
#if BASJOO_CHECK_PARAMS == 1
            if constexpr (!std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>) {
                if (self.ncols() != obj.nrows()) [[unlikely]] {
                    throw std::invalid_argument("Matrix::dot: incompatible dimensions");
                }
            }
#endif
            const index_type outer_size = obj.ncols();
            detail::DenseDotTraitT<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>> result(
                self.nrows(), outer_size, value_type(0.0), self.get_allocator()
            );
            if constexpr (Blas) {
                if !consteval {
                    [[maybe_unused]] constexpr CBLAS_ORDER cblas_layout{
                        kLeft ? CblasColMajor : CblasRowMajor
                    };
                    // One stride is 1, so the product is the leading dimension whichever layout is
                    // in use.
                    [[maybe_unused]] const auto lda{
                        static_cast<blasint>(self.stride(0) * self.stride(1))
                    };
                    [[maybe_unused]] const auto ldb{
                        static_cast<blasint>(obj.stride(0) * obj.stride(1))
                    };
                    [[maybe_unused]] const auto ldc{
                        static_cast<blasint>(result.stride(0) * result.stride(1))
                    };
                    [[maybe_unused]] constexpr value_type alpha(1.0), beta(0.0);
                    if constexpr (std::is_same_v<value_type, float>) {
                        cblas_sgemm(
                            cblas_layout, CblasNoTrans, CblasNoTrans,
                            static_cast<blasint>(self.nrows()), static_cast<blasint>(outer_size),
                            static_cast<blasint>(self.ncols()), alpha, self.data(), lda, obj.data(),
                            ldb, beta, result.data(), ldc
                        );
                        return result;
                    } else if constexpr (std::is_same_v<value_type, double>) {
                        cblas_dgemm(
                            cblas_layout, CblasNoTrans, CblasNoTrans,
                            static_cast<blasint>(self.nrows()), static_cast<blasint>(outer_size),
                            static_cast<blasint>(self.ncols()), alpha, self.data(), lda, obj.data(),
                            ldb, beta, result.data(), ldc
                        );
                        return result;
                    } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                        cblas_cgemm(
                            cblas_layout, CblasNoTrans, CblasNoTrans,
                            static_cast<blasint>(self.nrows()), static_cast<blasint>(outer_size),
                            static_cast<blasint>(self.ncols()), &alpha, self.data(), lda,
                            obj.data(), ldb, &beta, result.data(), ldc
                        );
                        return result;
                    } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                        cblas_zgemm(
                            cblas_layout, CblasNoTrans, CblasNoTrans,
                            static_cast<blasint>(self.nrows()), static_cast<blasint>(outer_size),
                            static_cast<blasint>(self.ncols()), &alpha, self.data(), lda,
                            obj.data(), ldb, &beta, result.data(), ldc
                        );
                        return result;
                    }
                }
            }
            // In storage order, the innermost loop running over contiguous elements; each element
            // of the product still sums its terms in k order.
            if constexpr (kLeft) {
                for (index_type j{0}; j < outer_size; ++j) {
                    for (index_type k{0}; k < self.ncols(); ++k) {
                        const value_type b{obj[k, j]};
                        for (index_type i{0}; i < self.nrows(); ++i) {
                            result[i, j] += self[i, k] * b;
                        }
                    }
                }
            } else {
                for (index_type i{0}; i < self.nrows(); ++i) {
                    for (index_type k{0}; k < self.ncols(); ++k) {
                        const value_type a{self[i, k]};
                        for (index_type j{0}; j < outer_size; ++j) {
                            result[i, j] += a * obj[k, j];
                        }
                    }
                }
            }
            return result;
        } else {
#if BASJOO_CHECK_PARAMS == 1
            if constexpr (!std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>) {
                if (std::cmp_not_equal(self.ncols(), obj.extent(0))) [[unlikely]] {
                    throw std::invalid_argument("Matrix::dot: incompatible dimensions");
                }
            }
#endif
            using result_type =
                detail::DenseDotTraitT<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>>;
            result_type result(
                static_cast<std::size_t>(self.nrows()), value_type(0.0), self.get_allocator()
            );
            if constexpr (Blas) {
                if !consteval {
                    [[maybe_unused]] constexpr CBLAS_ORDER cblas_layout{
                        kLeft ? CblasColMajor : CblasRowMajor
                    };
                    // One stride is 1, so the product is the leading dimension whichever layout is
                    // in use.
                    [[maybe_unused]] const auto lda{
                        static_cast<blasint>(self.stride(0) * self.stride(1))
                    };
                    [[maybe_unused]] constexpr value_type alpha(1.0), beta(0.0);
                    if constexpr (std::is_same_v<value_type, float>) {
                        cblas_sgemv(
                            cblas_layout, CblasNoTrans, static_cast<blasint>(self.nrows()),
                            static_cast<blasint>(self.ncols()), alpha, self.data(), lda, obj.data(),
                            1, beta, result.data(), 1
                        );
                        return result;
                    } else if constexpr (std::is_same_v<value_type, double>) {
                        cblas_dgemv(
                            cblas_layout, CblasNoTrans, static_cast<blasint>(self.nrows()),
                            static_cast<blasint>(self.ncols()), alpha, self.data(), lda, obj.data(),
                            1, beta, result.data(), 1
                        );
                        return result;
                    } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                        cblas_cgemv(
                            cblas_layout, CblasNoTrans, static_cast<blasint>(self.nrows()),
                            static_cast<blasint>(self.ncols()), &alpha, self.data(), lda,
                            obj.data(), 1, &beta, result.data(), 1
                        );
                        return result;
                    } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                        cblas_zgemv(
                            cblas_layout, CblasNoTrans, static_cast<blasint>(self.nrows()),
                            static_cast<blasint>(self.ncols()), &alpha, self.data(), lda,
                            obj.data(), 1, &beta, result.data(), 1
                        );
                        return result;
                    }
                }
            }
            if constexpr (kLeft) {
                // A combination of the columns, each element still summing its terms in j order.
                for (index_type j{0}; j < self.ncols(); ++j) {
                    const value_type x{obj[static_cast<std::size_t>(j)]};
                    for (index_type i{0}; i < self.nrows(); ++i) {
                        result[static_cast<std::size_t>(i)] += self[i, j] * x;
                    }
                }
            } else {
                for (index_type i{0}; i < self.nrows(); ++i) {
                    for (index_type j{0}; j < self.ncols(); ++j) {
                        result[static_cast<std::size_t>(i)] +=
                            self[i, j] * obj[static_cast<std::size_t>(j)];
                    }
                }
            }
            return result;
        }
    }
    // NOLINTEND(readability-function-cognitive-complexity)

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
    requires std::derived_from<T, MatrixBase<false>> || std::derived_from<T, MatrixBase<true>>
[[using gnu: pure, always_inline]]
inline constexpr auto abs(const T& matrix) noexcept
    -> detail::DenseNormTraitT<typename T::value_type> {
    return matrix.euclidean();
}

// Like std::norm on std::complex (and Vec2/Vec3), the squared magnitude.
template <typename T>
    requires std::derived_from<T, MatrixBase<false>> || std::derived_from<T, MatrixBase<true>>
[[using gnu: pure, always_inline]]
inline constexpr auto norm(const T& matrix) noexcept
    -> detail::DenseNormTraitT<typename T::value_type> {
    return matrix.euclideanSqr();
}

template <typename T>
    requires std::derived_from<std::remove_cvref_t<T>, MatrixBase<false>> ||
             std::derived_from<std::remove_cvref_t<T>, MatrixBase<true>>
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
