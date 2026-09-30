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
#include <memory>

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_trait.hpp"

namespace basjoo::math::detail {

template <typename T>
struct DenseNormTrait;

template <typename T>
using DenseNormTraitT = typename DenseNormTrait<T>::type;

template <Arithmetic Scalar>
struct DenseNormTrait<Scalar> final {
    using type = Scalar;
};

template <ComplexArithmetic Scalar>
struct DenseNormTrait<Scalar> final {
    using type = typename Scalar::value_type;
};

} // namespace basjoo::math::detail
