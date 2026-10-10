/**
 * @file dense_norm_trait.hpp
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

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_trait.hpp"

namespace basjoo::math::detail {

// Primary template left undefined so that unsupported types fail at the
// point of use instead of triggering a hard error at instantiation.
template <typename T>
struct DenseNormTrait;

template <typename T>
using DenseNormTraitT = typename DenseNormTrait<T>::type;

template <std::integral Scalar>
struct DenseNormTrait<Scalar> final {
    using type = Scalar;
};

template <std::floating_point Scalar>
struct DenseNormTrait<Scalar> final {
    using type = Scalar;
};

template <ComplexArithmetic Scalar>
struct DenseNormTrait<Scalar> final {
    using type = typename Scalar::value_type;
};

template <std::floating_point Scalar, std::size_t Extent, Allocatory Alloc>
struct DenseNormTrait<Vector<Scalar, Extent, Alloc>> final {
    using type = Vector<Scalar, Extent, Alloc>;
};

template <ComplexArithmetic Scalar, std::size_t Extent, Allocatory Alloc>
struct DenseNormTrait<Vector<Scalar, Extent, Alloc>> final {
    using type = Vector<typename Scalar::value_type, Extent, Alloc>;
};

template <
    std::floating_point Scalar, DenseExtents Extents, DenseLayout Layout,
    DenseAccessor AccessorPolicy, Allocatory Alloc>
struct DenseNormTrait<Matrix<Scalar, Extents, Layout, AccessorPolicy, Alloc>> final {
    using type = Matrix<Scalar, Extents, Layout, AccessorPolicy, Alloc>;
};

template <
    ComplexArithmetic Scalar, DenseExtents Extents, DenseLayout Layout,
    DenseAccessor AccessorPolicy, Allocatory Alloc>
struct DenseNormTrait<Matrix<Scalar, Extents, Layout, AccessorPolicy, Alloc>> final {
    using type = Matrix<typename Scalar::value_type, Extents, Layout, AccessorPolicy, Alloc>;
};

} // namespace basjoo::math::detail
