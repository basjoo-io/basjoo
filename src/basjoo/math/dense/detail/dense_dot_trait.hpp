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

#include <cstddef>
#include <mdspan>

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_trait.hpp"

namespace basjoo::math::detail {

// Primary template left undefined: unsupported (Self, Obj) pairs fail to
// satisfy DenseDotable at compile time instead of producing a hard error.
template <typename Self, typename Obj>
struct DenseDotTrait;

// Vector<Scalar, N> · Matrix<Scalar, extents<N, M>> -> Vector<Scalar, M>
template <
    typename Scalar, std::size_t N, std::size_t M, DenseLayout ObjLayout, DenseAccessor ObjAccessor,
    Allocatory VecAlloc, Allocatory ObjAlloc>
struct DenseDotTrait<
    Vector<Scalar, N, VecAlloc>,
    Matrix<Scalar, std::extents<std::size_t, N, M>, ObjLayout, ObjAccessor, ObjAlloc>>
    final {
    using type = Vector<Scalar, M>;
};

// Vector<Scalar, dyn, Alloc> · Matrix<Scalar, dextents, Layout, Alloc> -> Vector<Scalar, dyn,
// Alloc>
template <
    typename Scalar, DenseLayout ObjLayout, DenseAccessor ObjAccessor, Allocatory VecAlloc,
    Allocatory ObjAlloc>
struct DenseDotTrait<
    Vector<Scalar, std::dynamic_extent, VecAlloc>,
    Matrix<Scalar, std::dextents<std::size_t, 2>, ObjLayout, ObjAccessor, ObjAlloc>>
    final {
    using type = Vector<Scalar, std::dynamic_extent, VecAlloc>;
};

// Matrix<Scalar, extents<R, K>> · Matrix<Scalar, extents<K, M>> -> Matrix<Scalar, extents<R, M>,
// LhsLayout>
template <
    typename Scalar, std::size_t R, std::size_t K, std::size_t M, DenseLayout LhsLayout,
    DenseLayout ObjLayout, DenseAccessor LhsAccessor, DenseAccessor ObjAccessor,
    Allocatory LhsAlloc, Allocatory ObjAlloc>
struct DenseDotTrait<
    Matrix<Scalar, std::extents<std::size_t, R, K>, LhsLayout, LhsAccessor, LhsAlloc>,
    Matrix<Scalar, std::extents<std::size_t, K, M>, ObjLayout, ObjAccessor, ObjAlloc>>
    final {
    using type = Matrix<Scalar, std::extents<std::size_t, R, M>, LhsLayout, LhsAccessor, LhsAlloc>;
};

// Matrix<Scalar, dextents, Layout, Alloc> · Matrix<Scalar, dextents, Layout, Alloc> -> same
template <
    typename Scalar, DenseLayout LhsLayout, DenseLayout ObjLayout, DenseAccessor LhsAccessor,
    DenseAccessor ObjAccessor, Allocatory LhsAlloc, Allocatory ObjAlloc>
struct DenseDotTrait<
    Matrix<Scalar, std::dextents<std::size_t, 2>, LhsLayout, LhsAccessor, LhsAlloc>,
    Matrix<Scalar, std::dextents<std::size_t, 2>, ObjLayout, ObjAccessor, ObjAlloc>>
    final {
    using type = Matrix<Scalar, std::dextents<std::size_t, 2>, LhsLayout, LhsAccessor, LhsAlloc>;
};

// Matrix<Scalar, extents<R, N>> · Vector<Scalar, N> -> Vector<Scalar, R>
template <
    typename Scalar, std::size_t R, std::size_t N, DenseLayout LhsLayout, DenseAccessor LhsAccessor,
    Allocatory LhsAlloc, Allocatory VecAlloc>
struct DenseDotTrait<
    Matrix<Scalar, std::extents<std::size_t, R, N>, LhsLayout, LhsAccessor, LhsAlloc>,
    Vector<Scalar, N, VecAlloc>>
    final {
    using type = Vector<Scalar, R>;
};

// Matrix<Scalar, dextents, Layout, Alloc> · Vector<Scalar, dyn, Alloc> -> Vector<Scalar, dyn,
// Alloc>
template <
    typename Scalar, DenseLayout LhsLayout, DenseAccessor LhsAccessor, Allocatory LhsAlloc,
    Allocatory VecAlloc>
struct DenseDotTrait<
    Matrix<Scalar, std::dextents<std::size_t, 2>, LhsLayout, LhsAccessor, LhsAlloc>,
    Vector<Scalar, std::dynamic_extent, VecAlloc>>
    final {
    using type = Vector<Scalar, std::dynamic_extent, VecAlloc>;
};

template <typename Self, typename Obj>
using DenseDotTraitT = typename DenseDotTrait<Self, Obj>::type;

} // namespace basjoo::math::detail

namespace basjoo::math {

template <typename Self, typename Obj>
concept DenseDotable = requires { typename detail::DenseDotTraitT<Self, Obj>; };

} // namespace basjoo::math
