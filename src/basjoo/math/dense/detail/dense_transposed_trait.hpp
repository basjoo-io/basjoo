/**
 * @file dense_transposed_trait.hpp
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

#include <concepts>
#include <cstddef>

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_trait.hpp"

namespace basjoo::math::detail {

template <typename T>
struct DenseTransposedTrait;

template <
    ScalarArithmetic Scalar, std::integral Index, std::size_t R, std::size_t C, DenseLayout Layout,
    Allocatory Alloc>
struct DenseTransposedTrait<Matrix<Scalar, Index, R, C, Layout, Alloc>> final {
    using type = Matrix<Scalar, Index, C, R, Layout, Alloc>;
};

template <typename T>
using DenseTransposedTraitT = typename DenseTransposedTrait<T>::type;

} // namespace basjoo::math::detail
