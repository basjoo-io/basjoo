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

#include <concepts>
#include <cstddef>
#include <mdspan>

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_trait.hpp"

namespace basjoo::math::detail {

template <typename T, DenseLayout Layout = std::layout_left>
struct DenseGenerateTrait;

template <typename T, DenseLayout Layout = std::layout_left>
using DenseGenerateTraitT = typename DenseGenerateTrait<T, Layout>::type;

template <
    ScalarArithmetic Scalar, std::integral Index, std::size_t N, Allocatory Alloc,
    DenseLayout Layout>
struct DenseGenerateTrait<::basjoo::math::Vector<Scalar, Index, N, Alloc>, Layout> final {
    using type = ::basjoo::math::Matrix<Scalar, Index, N, N, Layout, Alloc>;
};

} // namespace basjoo::math::detail
