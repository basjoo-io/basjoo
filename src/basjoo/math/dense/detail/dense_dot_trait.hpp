/**
 * @file dense_dot_trait.hpp
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

template <typename Lhs, typename Rhs>
struct DenseDotTrait;

template <ScalarArithmetic Scalar, std::integral Index, std::size_t N, Allocatory Alloc>
struct DenseDotTrait<Vector<Scalar, Index, N, Alloc>, Vector<Scalar, Index, N, Alloc>> final {
    using type = Scalar;
};

template <
    ScalarArithmetic Scalar, std::integral Index, std::size_t N, std::size_t M, DenseLayout Layout,
    Allocatory Alloc>
struct DenseDotTrait<Vector<Scalar, Index, N, Alloc>, Matrix<Scalar, Index, N, M, Layout, Alloc>>
    final {
    using type = Vector<Scalar, Index, M, Alloc>;
};

template <
    ScalarArithmetic Scalar, std::integral Index, std::size_t R, std::size_t K, std::size_t M,
    DenseLayout Layout, Allocatory Alloc>
struct DenseDotTrait<
    Matrix<Scalar, Index, R, K, Layout, Alloc>, Matrix<Scalar, Index, K, M, Layout, Alloc>>
    final {
    using type = Matrix<Scalar, Index, R, M, Layout, Alloc>;
};

template <
    ScalarArithmetic Scalar, std::integral Index, std::size_t R, std::size_t N, DenseLayout Layout,
    Allocatory Alloc>
struct DenseDotTrait<Matrix<Scalar, Index, R, N, Layout, Alloc>, Vector<Scalar, Index, N, Alloc>>
    final {
    using type = Vector<Scalar, Index, R, Alloc>;
};

template <typename Lhs, typename Rhs>
using DenseDotTraitT = typename DenseDotTrait<Lhs, Rhs>::type;

} // namespace basjoo::math::detail

namespace basjoo::math {

template <typename Lhs, typename Rhs>
concept DenseDotable = requires { typename detail::DenseDotTraitT<Lhs, Rhs>; };

} // namespace basjoo::math
