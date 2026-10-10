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

#include <cstddef>
#include <cstdint>
#include <mdspan>

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_trait.hpp"

namespace basjoo::math::detail {

// Primary template left undefined so that unsupported types fail at the
// point of use instead of triggering a hard error at instantiation.
template <typename T, DenseLayout Layout = std::layout_left>
struct DensePartialPivotTrait;

template <typename T, DenseLayout Layout = std::layout_left>
using DensePartialPivotTraitT = typename DensePartialPivotTrait<T, Layout>::type;

template <
    typename Scalar, std::size_t R, std::size_t C, DenseLayout Layout, DenseAccessor AccessorPolicy,
    Allocatory Alloc>
struct DensePartialPivotTrait<
    Matrix<Scalar, std::extents<std::size_t, R, C>, Layout, AccessorPolicy, Alloc>,
    std::layout_left>
    final {
    using type = Vector<std::int32_t, R>;
};

template <
    typename Scalar, std::size_t R, std::size_t C, DenseLayout Layout, DenseAccessor AccessorPolicy,
    Allocatory Alloc>
struct DensePartialPivotTrait<
    Matrix<Scalar, std::extents<std::size_t, R, C>, Layout, AccessorPolicy, Alloc>,
    std::layout_right>
    final {
    using type = Vector<std::int32_t, C>;
};

template <
    ScalarArithmetic Scalar, ::basjoo::math::DenseLayout Layout,
    ::basjoo::math::DenseAccessor AccessorPolicy, Allocatory Alloc>
struct DensePartialPivotTrait<
    Matrix<Scalar, std::dextents<std::size_t, 2>, Layout, AccessorPolicy, Alloc>, std::layout_left>
    final {
    using type = Vector<
        std::int32_t, std::dynamic_extent,
        typename std::allocator_traits<Alloc>::template rebind_alloc<std::int32_t>>;
};

template <
    ScalarArithmetic Scalar, ::basjoo::math::DenseLayout Layout,
    ::basjoo::math::DenseAccessor AccessorPolicy, Allocatory Alloc>
struct DensePartialPivotTrait<
    Matrix<Scalar, std::dextents<std::size_t, 2>, Layout, AccessorPolicy, Alloc>, std::layout_right>
    final {
    using type = Vector<
        std::int32_t, std::dynamic_extent,
        typename std::allocator_traits<Alloc>::template rebind_alloc<std::int32_t>>;
};

} // namespace basjoo::math::detail
