/**
 * @file dense_generate_trait.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-09-07
 *
 * @copyright Copyright (c) 2025 basjoo development team.
 *            All rights reserved.
 *
 */

#pragma once

#include "basjoo/math/dense/dense_traits.hpp"

namespace basjoo::math::detail {

template <typename T, ::basjoo::math::MatrixOrder Order = ::basjoo::math::MatrixOrder::COL_MAJOR>
struct DenseGenerateTrait final {
    static_assert(false, "MatrixDegenerationTrait is not implemented for this type.");
};

template <typename T, ::basjoo::math::MatrixOrder Order = ::basjoo::math::MatrixOrder::COL_MAJOR>
using DenseGenerateTraitT = typename DenseGenerateTrait<T, Order>::type;

template <ScalarArithmetic Scalar, std::size_t N>
struct DenseGenerateTrait<::basjoo::math::Vector<Scalar, N>, ::basjoo::math::MatrixOrder::COL_MAJOR>
    final {
    using type = ::basjoo::math::Matrix<Scalar, N, N, ::basjoo::math::MatrixOrder::COL_MAJOR>;
};

template <ScalarArithmetic Scalar, std::size_t N>
struct DenseGenerateTrait<::basjoo::math::Vector<Scalar, N>, ::basjoo::math::MatrixOrder::ROW_MAJOR>
    final {
    using type = ::basjoo::math::Matrix<Scalar, N, N, ::basjoo::math::MatrixOrder::ROW_MAJOR>;
};

template <ScalarArithmetic Scalar, Allocatory Alloc>
struct DenseGenerateTrait<
    ::basjoo::math::VectorX<Scalar, Alloc>, ::basjoo::math::MatrixOrder::COL_MAJOR>
    final {
    using type = ::basjoo::math::MatrixX<Scalar, ::basjoo::math::MatrixOrder::COL_MAJOR, Alloc>;
};

template <ScalarArithmetic Scalar, Allocatory Alloc>
struct DenseGenerateTrait<
    ::basjoo::math::VectorX<Scalar, Alloc>, ::basjoo::math::MatrixOrder::ROW_MAJOR>
    final {
    using type = ::basjoo::math::MatrixX<Scalar, ::basjoo::math::MatrixOrder::ROW_MAJOR, Alloc>;
};

} // namespace basjoo::math::detail
