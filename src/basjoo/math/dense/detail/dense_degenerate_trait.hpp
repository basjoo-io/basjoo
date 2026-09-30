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

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_traits.hpp"

namespace basjoo::math::detail {

template <typename T, ::basjoo::math::MatrixOrder Order = ::basjoo::math::MatrixOrder::COL_MAJOR>
struct DenseDegenerateTrait final {
    static_assert(false, "MatrixDegenerationTrait is not implemented for this type.");
};

template <typename T, ::basjoo::math::MatrixOrder Order = ::basjoo::math::MatrixOrder::COL_MAJOR>
using DenseDegenerateTraitT = typename DenseDegenerateTrait<T, Order>::type;

template <ScalarArithmetic Scalar>
struct DenseDegenerateTrait<Scalar, ::basjoo::math::MatrixOrder::COL_MAJOR> final {
    using type = Scalar;
};

template <ScalarArithmetic Scalar>
struct DenseDegenerateTrait<Scalar, ::basjoo::math::MatrixOrder::ROW_MAJOR> final {
    using type = Scalar;
};

template <
    ScalarArithmetic Scalar, std::size_t NRows, std::size_t NCols,
    ::basjoo::math::MatrixOrder Order>
struct DenseDegenerateTrait<
    Matrix<Scalar, NRows, NCols, Order>, ::basjoo::math::MatrixOrder::COL_MAJOR>
    final {
    using type = Vector<Scalar, NRows>;
};

template <
    ScalarArithmetic Scalar, std::size_t NRows, std::size_t NCols,
    ::basjoo::math::MatrixOrder Order>
struct DenseDegenerateTrait<
    Matrix<Scalar, NRows, NCols, Order>, ::basjoo::math::MatrixOrder::ROW_MAJOR>
    final {
    using type = Vector<Scalar, NCols>;
};

template <ScalarArithmetic Scalar, ::basjoo::math::MatrixOrder Order, Allocatory Alloc>
struct DenseDegenerateTrait<MatrixX<Scalar, Order, Alloc>, ::basjoo::math::MatrixOrder::COL_MAJOR>
    final {
    using type = VectorX<Scalar, Alloc>;
};

template <ScalarArithmetic Scalar, ::basjoo::math::MatrixOrder Order, Allocatory Alloc>
struct DenseDegenerateTrait<MatrixX<Scalar, Order, Alloc>, ::basjoo::math::MatrixOrder::ROW_MAJOR>
    final {
    using type = VectorX<Scalar, Alloc>;
};

template <ScalarArithmetic Scalar, std::size_t N>
struct DenseDegenerateTrait<Vector<Scalar, N>, ::basjoo::math::MatrixOrder::COL_MAJOR> final {
    using type = Scalar;
};

template <ScalarArithmetic Scalar, std::size_t N>
struct DenseDegenerateTrait<Vector<Scalar, N>, ::basjoo::math::MatrixOrder::ROW_MAJOR> final {
    using type = Scalar;
};

template <ScalarArithmetic Scalar, Allocatory Alloc>
struct DenseDegenerateTrait<VectorX<Scalar, Alloc>, ::basjoo::math::MatrixOrder::COL_MAJOR> final {
    using type = Scalar;
};

template <ScalarArithmetic Scalar, Allocatory Alloc>
struct DenseDegenerateTrait<VectorX<Scalar, Alloc>, ::basjoo::math::MatrixOrder::ROW_MAJOR> final {
    using type = Scalar;
};

} // namespace basjoo::math::detail
