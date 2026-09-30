/**
 * @file dense_partial_pivot_trait.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-01-19
 *
 * @copyright Copyright (c) 2025 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <mdspan>
#include <memory>

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_trait.hpp"

namespace basjoo::math::detail {

template <typename T, DenseLayout Layout = std::layout_left>
struct DensePartialPivotTrait;

template <typename T, DenseLayout Layout = std::layout_left>
using DensePartialPivotTraitT = typename DensePartialPivotTrait<T, Layout>::type;

template <
    ScalarArithmetic Scalar, std::integral Index, std::size_t R, std::size_t C,
    DenseLayout MatrixLayout, Allocatory Alloc>
struct DensePartialPivotTrait<Matrix<Scalar, Index, R, C, MatrixLayout, Alloc>, std::layout_left>
    final {
    using type = Vector<
        std::int32_t, Index, R,
        typename std::allocator_traits<Alloc>::template rebind_alloc<std::int32_t>>;
};

template <
    ScalarArithmetic Scalar, std::integral Index, std::size_t R, std::size_t C,
    DenseLayout MatrixLayout, Allocatory Alloc>
struct DensePartialPivotTrait<Matrix<Scalar, Index, R, C, MatrixLayout, Alloc>, std::layout_right>
    final {
    using type = Vector<
        std::int32_t, Index, C,
        typename std::allocator_traits<Alloc>::template rebind_alloc<std::int32_t>>;
};

} // namespace basjoo::math::detail
