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

#include <cstddef>
#include <mdspan>

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_trait.hpp"

namespace basjoo::math::detail {

// Primary template left undefined so that unsupported types fail at the
// point of use instead of triggering a hard error at instantiation.
template <typename T, DenseLayout Layout = std::layout_left>
struct DenseDegenerateTrait;

template <typename T, DenseLayout Layout = std::layout_left>
using DenseDegenerateTraitT = typename DenseDegenerateTrait<T, Layout>::type;

template <ScalarArithmetic Scalar>
struct DenseDegenerateTrait<Scalar, std::layout_left> final {
    using type = Scalar;
};

template <ScalarArithmetic Scalar>
struct DenseDegenerateTrait<Scalar, std::layout_right> final {
    using type = Scalar;
};

template <
    typename Scalar, std::size_t R, std::size_t C, DenseLayout Layout, DenseAccessor AccessorPolicy,
    Allocatory Alloc>
struct DenseDegenerateTrait<
    Matrix<Scalar, std::extents<std::size_t, R, C>, Layout, AccessorPolicy, Alloc>,
    std::layout_left>
    final {
    using type = Vector<Scalar, R>;
};

template <
    typename Scalar, std::size_t R, std::size_t C, DenseLayout Layout, DenseAccessor AccessorPolicy,
    Allocatory Alloc>
struct DenseDegenerateTrait<
    Matrix<Scalar, std::extents<std::size_t, R, C>, Layout, AccessorPolicy, Alloc>,
    std::layout_right>
    final {
    using type = Vector<Scalar, C>;
};

template <typename Scalar, DenseLayout Layout, DenseAccessor AccessorPolicy, Allocatory Alloc>
struct DenseDegenerateTrait<
    Matrix<Scalar, std::dextents<std::size_t, 2>, Layout, AccessorPolicy, Alloc>, std::layout_left>
    final {
    using type = Vector<Scalar, std::dynamic_extent, Alloc>;
};

template <typename Scalar, DenseLayout Layout, DenseAccessor AccessorPolicy, Allocatory Alloc>
struct DenseDegenerateTrait<
    Matrix<Scalar, std::dextents<std::size_t, 2>, Layout, AccessorPolicy, Alloc>, std::layout_right>
    final {
    using type = Vector<Scalar, std::dynamic_extent, Alloc>;
};

template <ScalarArithmetic Scalar, std::size_t Extent, Allocatory Alloc>
struct DenseDegenerateTrait<Vector<Scalar, Extent, Alloc>, std::layout_left> final {
    using type = Scalar;
};

template <ScalarArithmetic Scalar, std::size_t Extent, Allocatory Alloc>
struct DenseDegenerateTrait<Vector<Scalar, Extent, Alloc>, std::layout_right> final {
    using type = Scalar;
};

} // namespace basjoo::math::detail
