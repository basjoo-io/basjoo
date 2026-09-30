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

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_traits.hpp"

namespace basjoo::math::detail {

template <typename T, ::basjoo::math::MatrixOrder Order = ::basjoo::math::MatrixOrder::COL_MAJOR>
struct DensePartialPivotTrait final {
    static_assert(false, "MatrixDegenerationTrait is not implemented for this type.");
};

template <typename T, ::basjoo::math::MatrixOrder Order = ::basjoo::math::MatrixOrder::COL_MAJOR>
using DensePartialPivotTraitT = typename DensePartialPivotTrait<T, Order>::type;

template <
    ScalarArithmetic Scalar, std::size_t NRows, std::size_t NCols,
    ::basjoo::math::MatrixOrder Order>
struct DensePartialPivotTrait<
    Matrix<Scalar, NRows, NCols, Order>, ::basjoo::math::MatrixOrder::COL_MAJOR>
    final {
    using type = Vector<std::int32_t, NRows>;
};

template <
    ScalarArithmetic Scalar, std::size_t NRows, std::size_t NCols,
    ::basjoo::math::MatrixOrder Order>
struct DensePartialPivotTrait<
    Matrix<Scalar, NRows, NCols, Order>, ::basjoo::math::MatrixOrder::ROW_MAJOR>
    final {
    using type = Vector<std::int32_t, NCols>;
};

template <ScalarArithmetic Scalar, ::basjoo::math::MatrixOrder Order, Allocatory Alloc>
struct DensePartialPivotTrait<MatrixX<Scalar, Order, Alloc>, ::basjoo::math::MatrixOrder::COL_MAJOR>
    final {
    using type = VectorX<
        std::int32_t, typename std::allocator_traits<Alloc>::template rebind_alloc<std::int32_t>>;
};

template <ScalarArithmetic Scalar, ::basjoo::math::MatrixOrder Order, Allocatory Alloc>
struct DensePartialPivotTrait<MatrixX<Scalar, Order, Alloc>, ::basjoo::math::MatrixOrder::ROW_MAJOR>
    final {
    using type = VectorX<
        std::int32_t, typename std::allocator_traits<Alloc>::template rebind_alloc<std::int32_t>>;
};

} // namespace basjoo::math::detail
