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

#include "basjoo/math/dense/dense_trait.hpp"

namespace basjoo::math::detail {

// Primary template left undefined so that unsupported types fail at the
// point of use instead of triggering a hard error at instantiation.
template <typename T, DenseLayout Layout = std::layout_left>
struct DenseGenerateTrait;

template <typename T, DenseLayout Layout = std::layout_left>
using DenseGenerateTraitT = typename DenseGenerateTrait<T, Layout>::type;

template <ScalarArithmetic Scalar, std::size_t Extent, Allocatory Alloc>
struct DenseGenerateTrait<::basjoo::math::Vector<Scalar, Extent, Alloc>, std::layout_left> final {
    using type =
        ::basjoo::math::Matrix<Scalar, std::extents<std::size_t, Extent, Extent>, std::layout_left>;
};

template <ScalarArithmetic Scalar, std::size_t Extent, Allocatory Alloc>
struct DenseGenerateTrait<::basjoo::math::Vector<Scalar, Extent, Alloc>, std::layout_right> final {
    using type = ::basjoo::math::Matrix<
        Scalar, std::extents<std::size_t, Extent, Extent>, std::layout_right>;
};

template <ScalarArithmetic Scalar, Allocatory Alloc>
struct DenseGenerateTrait<
    ::basjoo::math::Vector<Scalar, std::dynamic_extent, Alloc>, std::layout_left>
    final {
    using type = ::basjoo::math::Matrix<
        Scalar, std::dextents<std::size_t, 2>, std::layout_left, std::default_accessor<Scalar>,
        Alloc>;
};

template <ScalarArithmetic Scalar, Allocatory Alloc>
struct DenseGenerateTrait<
    ::basjoo::math::Vector<Scalar, std::dynamic_extent, Alloc>, std::layout_right>
    final {
    using type = ::basjoo::math::Matrix<
        Scalar, std::dextents<std::size_t, 2>, std::layout_right, std::default_accessor<Scalar>,
        Alloc>;
};

} // namespace basjoo::math::detail
