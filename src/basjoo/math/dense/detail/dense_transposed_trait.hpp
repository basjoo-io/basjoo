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

#include <cstddef>
#include <mdspan>

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_trait.hpp"

namespace basjoo::math::detail {

// Primary template left undefined so that non-matrix types fail at compile
// time when the trait is looked up.
template <typename T>
struct DenseTransposedTrait;

template <
    typename Scalar, std::size_t R, std::size_t C, DenseLayout Layout, DenseAccessor AccessorPolicy,
    Allocatory Alloc>
struct DenseTransposedTrait<
    Matrix<Scalar, std::extents<std::size_t, R, C>, Layout, AccessorPolicy, Alloc>>
    final {
    using type = Matrix<Scalar, std::extents<std::size_t, C, R>, Layout, AccessorPolicy, Alloc>;
};

template <typename Scalar, DenseLayout Layout, DenseAccessor AccessorPolicy, Allocatory Alloc>
struct DenseTransposedTrait<
    Matrix<Scalar, std::dextents<std::size_t, 2>, Layout, AccessorPolicy, Alloc>>
    final {
    using type = Matrix<Scalar, std::dextents<std::size_t, 2>, Layout, AccessorPolicy, Alloc>;
};

template <typename T>
using DenseTransposedTraitT = typename DenseTransposedTrait<T>::type;

} // namespace basjoo::math::detail
