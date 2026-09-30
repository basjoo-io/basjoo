/**
 * @file dense_trait.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-01-08
 *
 * @copyright Copyright (c) 2025 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <array>
#include <cstddef>
#include <mdspan>
#include <memory_resource>
#include <span>
#include <type_traits>
#include <vector>

#include "basjoo/common/aligned_allocator.hpp"
#include "basjoo/math/concepts.hpp"

namespace basjoo::math {

template <typename T>
concept DenseLayout = std::same_as<T, std::layout_left> || std::same_as<T, std::layout_right>;

template <
    ScalarArithmetic Scalar, std::integral Index = std::size_t, std::size_t N = std::dynamic_extent,
    Allocatory Alloc = ::basjoo::common::AlignedAllocator<Scalar, 32>>
class Vector;

// A matrix is either fully static or fully dynamic in shape.
template <
    ScalarArithmetic Scalar, std::integral Index = std::size_t,
    std::size_t NRows = std::dynamic_extent, std::size_t NCols = std::dynamic_extent,
    DenseLayout Layout = std::layout_left,
    Allocatory Alloc = ::basjoo::common::AlignedAllocator<Scalar, 32>>
    requires((NRows == std::dynamic_extent) == (NCols == std::dynamic_extent))
class Matrix;

namespace pmr {

template <ScalarArithmetic Scalar, std::integral Index = std::size_t>
using Vector = ::basjoo::math::Vector<
    Scalar, Index, std::dynamic_extent, std::pmr::polymorphic_allocator<Scalar>>;

template <
    ScalarArithmetic Scalar, std::integral Index = std::size_t,
    DenseLayout Layout = std::layout_left>
using Matrix = ::basjoo::math::Matrix<
    Scalar, Index, std::dynamic_extent, std::dynamic_extent, Layout,
    std::pmr::polymorphic_allocator<Scalar>>;

} // namespace pmr

template <typename T>
struct DenseTrait;

template <ScalarArithmetic Scalar, std::integral Index, std::size_t N, Allocatory Alloc>
struct DenseTrait<Vector<Scalar, Index, N, Alloc>> final {
    using element_type = Scalar;
    using value_type = std::remove_cv_t<Scalar>;
    using pointer = Scalar*;
    using const_pointer = const Scalar*;
    using reference = Scalar&;
    using const_reference = const Scalar&;
    using extents_type = std::extents<Index, N>;
    using layout_type = std::layout_left;
    using allocator_type = Alloc;
    using size_type = typename extents_type::size_type;
    using difference_type = std::ptrdiff_t;
    using index_type = typename extents_type::index_type;
    using rank_type = typename extents_type::rank_type;
    using mapping_type = typename layout_type::template mapping<extents_type>;
    using storage_type = std::conditional_t<
        N == std::dynamic_extent, std::vector<Scalar, Alloc>, std::array<Scalar, N>>;
};

template <
    ScalarArithmetic Scalar, std::integral Index, std::size_t NRows, std::size_t NCols,
    DenseLayout Layout, Allocatory Alloc>
struct DenseTrait<Matrix<Scalar, Index, NRows, NCols, Layout, Alloc>> final {
    using element_type = Scalar;
    using value_type = std::remove_cv_t<Scalar>;
    using pointer = Scalar*;
    using const_pointer = const Scalar*;
    using reference = Scalar&;
    using const_reference = const Scalar&;
    using extents_type = std::extents<Index, NRows, NCols>;
    using layout_type = Layout;
    using allocator_type = Alloc;
    using size_type = typename extents_type::size_type;
    using difference_type = std::ptrdiff_t;
    using index_type = typename extents_type::index_type;
    using rank_type = typename extents_type::rank_type;
    using mapping_type = typename Layout::template mapping<extents_type>;
    using storage_type = std::conditional_t<
        extents_type::rank_dynamic() == 0,
        std::array<Scalar, extents_type::static_extent(0) * extents_type::static_extent(1)>,
        std::vector<Scalar, Alloc>>;
};

} // namespace basjoo::math
