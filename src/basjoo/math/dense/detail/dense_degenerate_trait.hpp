/**
 * @file dense_degenerate_trait.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>
#include <cstddef>
#include <mdspan>

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_trait.hpp"

namespace basjoo::math::detail {

template <typename T, DenseLayout Layout = std::layout_left>
struct DenseDegenerateTrait;

template <typename T, DenseLayout Layout = std::layout_left>
using DenseDegenerateTraitT = typename DenseDegenerateTrait<T, Layout>::type;

template <ScalarArithmetic Scalar, DenseLayout Layout>
struct DenseDegenerateTrait<Scalar, Layout> final {
    using type = Scalar;
};

template <
    ScalarArithmetic Scalar, std::integral Index, std::size_t R, std::size_t C,
    DenseLayout MatrixLayout, Allocatory Alloc>
struct DenseDegenerateTrait<Matrix<Scalar, Index, R, C, MatrixLayout, Alloc>, std::layout_left>
    final {
    using type = Vector<Scalar, Index, R, Alloc>;
};

template <
    ScalarArithmetic Scalar, std::integral Index, std::size_t R, std::size_t C,
    DenseLayout MatrixLayout, Allocatory Alloc>
struct DenseDegenerateTrait<Matrix<Scalar, Index, R, C, MatrixLayout, Alloc>, std::layout_right>
    final {
    using type = Vector<Scalar, Index, C, Alloc>;
};

// Any vector, not only a Vector, degenerates to its scalar, so that a struct of named fields can
// serve as the value type of a piecewise function too.
template <VecArithmetic T, DenseLayout Layout>
struct DenseDegenerateTrait<T, Layout> final {
    using type = typename T::value_type;
};

} // namespace basjoo::math::detail
